/**
 * Replays the same recorded Rapier poses through both adapters, optionally against
 * a preserved pre-change src/scene + src/core tree. Also exercises the real UI.
 * node --import tsx scripts/rendering-browser-check.mjs [output] [baseline root]
 * SANDBOX_LIVE_SECONDS=60 runs four live trials. CPU times are submission timings,
 * never GPU timings or evidence that physics advances in real time.
 */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';
import { mkdir, readFile, writeFile, readdir } from 'node:fs/promises';
import { dirname, resolve, join, relative } from 'node:path';
import { cpus, platform, release } from 'node:os';
import { createServer } from 'vite';
import { createEmbodiedCharacter } from '../src/character/EmbodiedCharacter.ts';

const output = resolve(process.argv[2] ?? 'evidence/sandbox-rendering');
const baseline = process.argv[3] ? '/' + relative(process.cwd(), resolve(process.argv[3])).replaceAll('\\', '/') : null;
const runtimeRequire = createRequire(join(dirname(process.env.CODEX_MCP_NODE_PATH ?? process.execPath), 'sandbox-qa.cjs'));
const { chromium } = runtimeRequire('playwright');
const stages = (process.env.SANDBOX_STAGES ?? 'replay,ui,live').split(',');
const fingerprint = async () => Object.fromEntries(await Promise.all([
  ...(await readdir('src/scene')).filter(name => name.endsWith('.ts')).map(name => 'src/scene/' + name),
  'src/demo/DemoRuntime.ts', 'src/core/FixedStepLoop.ts', 'src/core/humanoid.ts', 'src/core/geometry.ts',
].map(async path => [path, createHash('sha256').update(await readFile(path)).digest('hex')])));
const report = { date: new Date().toISOString(), host: { platform: platform(), release: release(), cpu: cpus()[0]?.model },
  baseline, referenceLaptopVerified: false, qualification: 'Local headless browser. Presentation CPU includes interpolation and render submission, not GPU execution. Live physics is measured separately.',
  poses: {}, replay: [], live: [], interactions: [], errors: [] };
await mkdir(output, { recursive: true });
report.sourceBefore = await fingerprint();
const save = () => writeFile(join(output, 'report.json'), JSON.stringify(report, null, 2));
const html = '<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Sandbox verification</title></head><body style="margin:0"><div id="root"></div><script type="module" src="/__sandbox-ui.tsx"></script></body></html>';
const ui = `import React from 'react'; import {createRoot} from 'react-dom/client'; import {EmbodiedDemo} from '/src/demo/EmbodiedDemo.tsx'; import '/app/globals.css'; createRoot(document.getElementById('root')).render(<EmbodiedDemo/>);`;
const server = await createServer({ configFile: false, root: process.cwd(), cacheDir: '.sites-runtime/render-qa-vite',
  resolve: { alias: { '@': process.cwd() } }, server: { host: '127.0.0.1', port: 0 },
  plugins: [{ name: 'sandbox-verification', configureServer(vite) {
    vite.middlewares.use(async (req, res, next) => {
      const path = req.url.split('?')[0];
      if (path === '/') { res.setHeader('Content-Type', 'text/html'); res.end(await vite.transformIndexHtml(req.url, html)); }
      else if (path === '/__sandbox-replay') { res.setHeader('Content-Type', 'text/html'); res.end('<!doctype html><html><body style="margin:0"><div id="host" style="position:absolute;inset:0"></div></body></html>'); }
      else next();
    });
  }, resolveId(id) { if (id === '/__sandbox-ui.tsx') return '\0sandbox-ui.tsx'; },
  load(id) { if (id === '\0sandbox-ui.tsx') return ui; },
  transform(code, id) { if (id === '\0sandbox-ui.tsx') return code.replace('<EmbodiedDemo/>', 'React.createElement(EmbodiedDemo)'); }
  }] });
let browser;
try {
  await server.listen();
  const url = `http://127.0.0.1:${server.httpServer.address().port}`;
  browser = await chromium.launch({ channel: 'msedge', headless: true, args: ['--disable-background-timer-throttling', '--disable-renderer-backgrounding'] });
  report.browser = await browser.version();
  const browserSession = await browser.newBrowserCDPSession();
  report.system = await browserSession.send('SystemInfo.getInfo');
  const recordings = {};
  if (stages.includes('replay')) for (const scene of ['protocol', 'playground']) {
    if (process.env.SANDBOX_POSES) {
      const data = await readFile(join(process.env.SANDBOX_POSES, `${scene}-poses.json`), 'utf8');
      recordings[scene] = JSON.parse(data);
      report.poses[scene] = { frames: recordings[scene].length, sha256: createHash('sha256').update(data).digest('hex') };
      await writeFile(join(output, `${scene}-poses.json`), data);
      continue;
    }
    const character = await createEmbodiedCharacter('webgl', scene === 'protocol' ? { room: true } : { playground: { station: 'wobble', difficulty: 'gentle' } });
    const poses = [character.getSnapshot('webgl')], physics = [];
    if (scene === 'protocol') character.requestStrike();
    for (let i = 0; i < 90; i++) {
      const start = performance.now(); character.fixedUpdate(1 / 60, null); physics.push(performance.now() - start);
      poses.push(character.getSnapshot('webgl'));
    }
    character.dispose(); recordings[scene] = poses;
    const data = JSON.stringify(poses);
    report.poses[scene] = { frames: poses.length, sha256: createHash('sha256').update(data).digest('hex'),
      phases: [...new Set(poses.map(pose => pose.striker?.phase).filter(Boolean))],
      physicsMeanMs: physics.reduce((a, b) => a + b) / physics.length, physicsMaxMs: Math.max(...physics) };
    await writeFile(join(output, `${scene}-poses.json`), data);
  }
  console.log('Recorded identical poses for before/after replay.'); await save();
  if (stages.includes('replay')) for (const [width, height] of [[1366,768], [1920,1080]]) for (const scene of ['protocol','playground']) {
    for (const renderer of ['webgl','canvas2d']) for (const source of baseline ? ['before','after'] : ['after']) {
      const page = await browser.newPage({ viewport: { width, height }, deviceScaleFactor: 1 });
      page.on('pageerror', error => report.errors.push(error.message));
      await page.goto(`${url}/__sandbox-replay`);
      const entry = await page.evaluate(async ({ poses, renderer, sourceRoot, scene, width, height }) => {
        const { createPoseView, createSharedCamera } = await import(sourceRoot + '/scene/index.ts');
        const camera = createSharedCamera();
        camera.setView(scene === 'protocol' ? { x:1,y:3.7,z:7.8 } : { x:10.5,y:12.5,z:17 }, scene === 'protocol' ? {x:0,y:1.4,z:-1} : {x:0,y:.2,z:-.4});
        const view = createPoseView(renderer, { camera });
        view.mount(document.getElementById('host')); view.resize(width, height, devicePixelRatio);
        // Baseline's normal counter resets after shadows; disable that for honest comparison.
        if (view.renderer) view.renderer.info.autoReset = false;
        const freeze = value => { if (value && typeof value === 'object') { Object.freeze(value); Object.values(value).forEach(freeze); } };
        freeze(poses);
        const cpu = [], calls = [], memory = [];
        for (let i = 0; i < 240; i++) {
          await new Promise(requestAnimationFrame);
          const pose = poses[i % poses.length], previous = poses[Math.max(0, i % poses.length - 1)];
          if (view.renderer) view.renderer.info.reset();
          const start = performance.now(); view.setSnapshot(previous, pose, 0.6); view.render();
          const duration = performance.now() - start;
          if (i >= 60) { cpu.push(duration); calls.push(view.getMetrics?.().drawCalls ?? 0); }
        }
        const metrics = view.getMetrics?.() ?? {};
        const sorted = [...cpu].sort((a,b)=>a-b);
        window.__replay = { view, camera, poses };
        return { cpuSamples: cpu.length, cpuP50Ms: sorted[Math.floor(sorted.length*.5)], cpuP95Ms: sorted[Math.floor(sorted.length*.95)],
          cpuMaxMs: Math.max(...cpu), maxDrawCalls: Math.max(...calls), metrics, memory };
      }, { poses: recordings[scene], renderer, scene, sourceRoot: source === 'before' ? baseline : '/src', width, height });
      await page.screenshot({ path: join(output, `${source}-${scene}-${renderer}-${width}.png`) });
      report.replay.push({ source, scene, renderer, width, height, ...entry });
      console.log(JSON.stringify({ stage:'replay',source,scene,renderer,width,p95Ms:entry.cpuP95Ms,calls:entry.maxDrawCalls }));
      const cleanup = await page.evaluate(() => {
        const { view, poses } = window.__replay;
        const qualityChecks = [];
        if (view.setQuality) for (const quality of ['low', 'high', 'auto']) {
          view.setQuality(quality); view.setSnapshot(poses[0], poses[0], 1); view.render();
          qualityChecks.push({ quality, ...view.getMetrics(), shadowsEnabled: view.renderer?.shadowMap.enabled ?? null });
        }
        const resources = new Set();
        if (view.scene) view.scene.traverse(object => {
          if (object.geometry) resources.add(object.geometry);
          for (const material of Array.isArray(object.material) ? object.material : object.material ? [object.material] : []) {
            resources.add(material);
            for (const value of Object.values(material)) if (value?.isTexture) resources.add(value);
          }
        });
        for (const material of view.bodyMaterials?.values() ?? []) resources.add(material);
        if (view.selectedMaterial) resources.add(view.selectedMaterial);
        const releases = [];
        for (const resource of resources) { const count = { type: resource.type, count: 0 }; releases.push(count); resource.addEventListener('dispose', () => count.count++); }
        view.dispose(); view.dispose();
        return { qualityChecks, resources: releases.length, invalidReleases: releases.filter(item => item.count !== 1) };
      });
      if (source === 'after') {
        assert.deepEqual(cleanup.invalidReleases, []);
        for (const check of cleanup.qualityChecks) {
          assert.ok(check.effectivePixelRatio <= 1);
          if (renderer === 'webgl') {
            assert.equal(check.shadowsEnabled, true);
            assert.equal(check.shadowResolution, check.quality === 'low' ? 512 : 1024);
          }
        }
      }
      report.replay.at(-1).cleanup = cleanup;
      await page.close(); await save();
    }
  }
  // Real React UI, production runtime, and pointer handlers. Never change physics assertions.
  if (stages.includes('ui')) for (const scene of ['protocol','playground']) {
    const page = await browser.newPage({ viewport: { width: 1366, height: 768 } });
    page.on('pageerror', error => report.errors.push(error.message));
    await page.goto(url + (scene === 'playground' ? '/?mode=playground' : '/'));
    await page.waitForFunction(() => window.__EMBODIED_DEMO__?.ready(), null, { timeout: 90000 });
    await page.getByTestId('pause-toggle').click();
    for (const renderer of ['webgl','canvas2d']) {
      const picker = page.getByTestId('renderer-picker');
      if (scene === 'protocol') await picker.selectOption(renderer);
      else { await picker.click(); await page.getByRole('option', { name: renderer === 'webgl' ? 'WebGL2' : 'Canvas 2D', exact: true }).click(); }
      for (const quality of ['auto','low','high']) {
        await page.getByTestId('quality-picker').selectOption(quality);
        await page.waitForTimeout(100);
        const metrics = await page.evaluate(() => ({ quality: window.__EMBODIED_DEMO__.quality(), metrics: window.__EMBODIED_DEMO__.presentation() }));
        assert.equal(metrics.quality, quality);
        if (renderer === 'webgl') assert.equal(metrics.metrics.shadowResolution, quality === 'low' ? 512 : 1024);
      }
      const before = await page.evaluate(() => window.__EMBODIED_DEMO__.camera());
      const host = await page.getByTestId('simulation-view').boundingBox();
      await page.mouse.move(host.x + host.width * .75, host.y + host.height * .4);
      await page.mouse.down(); await page.mouse.move(host.x + host.width * .75 + 65, host.y + host.height * .4 + 25, { steps: 6 }); await page.mouse.up();
      await page.mouse.down({button:'right'}); await page.mouse.move(host.x + host.width * .75 + 90, host.y + host.height * .4 + 40, {steps:4}); await page.mouse.up({button:'right'});
      await page.mouse.wheel(0, -140);
      await page.waitForTimeout(100);
      assert.notDeepEqual(await page.evaluate(() => window.__EMBODIED_DEMO__.camera()), before);
      await page.getByTestId('reset-button').click();
      assert.equal(await page.getByTestId('pause-toggle').getAttribute('aria-pressed'), 'true');
      await page.screenshot({ path: join(output, `ui-${scene}-${renderer}-desktop.png`) });
      await page.setViewportSize({ width: 390, height: 844 });
      await page.waitForTimeout(150);
      if (scene === 'playground') {
        await page.getByRole('button', { name: 'Stations' }).click();
        await page.screenshot({ path: join(output, `ui-${scene}-${renderer}-tray.png`) });
        await page.getByTestId('station-wobble').click();
        assert.equal(await page.getByRole('button', { name: 'Stations' }).getAttribute('aria-expanded'), 'false');
      }
      await page.screenshot({ path: join(output, `ui-${scene}-${renderer}-narrow.png`) });
      assert.ok(await page.getByTestId('pause-toggle').isVisible());
      assert.ok(await page.getByTestId('quality-picker').isVisible());
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      await page.setViewportSize({ width: 1366, height: 768 });
      if (scene === 'playground') {
        await page.getByTestId('station-flat').click();
        await page.getByRole('button', { name: 'Focus body' }).click();
      }
      await page.getByTestId('pause-toggle').click();
      if (scene === 'protocol') {
        await page.getByTestId('strike-button').click();
        await page.waitForFunction(() => Number(document.querySelector('[data-testid="strike-count"]').textContent) > 0);
      } else {
        const point = await page.evaluate(() => window.__EMBODIED_DEMO__.regionPoint('torso'));
        assert.ok(point?.visible, 'The focused torso is visible for dragging');
        await page.mouse.move(point.x, point.y); await page.mouse.down();
        await page.waitForFunction(() => window.__EMBODIED_DEMO__.diagnostics().activeGrab);
        await page.mouse.move(point.x + 30, point.y - 10, {steps:5}); await page.mouse.up();
        await page.waitForFunction(() => !window.__EMBODIED_DEMO__.diagnostics().activeGrab);
      }
      await page.waitForTimeout(500);
      await page.getByTestId('pause-toggle').click();
      report.interactions.push({ scene, renderer, passed: true }); await save();
    }
    await page.close();
  }
  const seconds = Number(process.env.SANDBOX_LIVE_SECONDS ?? 60);
  if (stages.includes('live') && seconds > 0) for (const scene of ['protocol','playground']) for (const renderer of ['webgl','canvas2d']) {
    const page = await browser.newPage({ viewport: {width:1366,height:768} });
    await page.goto(`${url}/__sandbox-replay`);
    await page.evaluate(async ({ scene, renderer, seconds }) => {
      const { DemoRuntime } = await import('/src/demo/DemoRuntime.ts');
      const { createEmbodiedCharacter } = await import('/src/character/index.ts');
      const character = await createEmbodiedCharacter(renderer, scene === 'protocol' ? {room:true} : {playground:{station:'wobble',difficulty:'gentle'}});
      const runtime = new DemoRuntime({host:document.getElementById('host'),character,renderer,
        capabilities:{webgl2:true,canvas2d:true,browser:navigator.userAgent,devicePixelRatio,viewport:{width:innerWidth,height:innerHeight}}});
      const cpu=[],intervals=[],physics=[],calls=[]; let last=null;
      const start = performance.now();
      let strikeRequested = false;
      const original = runtime.loop.render;
      runtime.loop.render = (alpha, now) => {
        original(alpha, now);
        if (scene === 'protocol' && !strikeRequested && now - start > 5000) {
          strikeRequested = runtime.requestStrike();
        }
        if (now - start >= 1000) {
          if(last!==null)intervals.push(now-last);
          cpu.push(runtime.renderMetrics.presentationCpuMs); physics.push(runtime.timing.lastStepMs); calls.push(runtime.renderMetrics.drawCalls);
          last=now;
        }
        if (now - start >= (seconds+1)*1000) {
          const summary = values => { const sorted=[...values].sort((a,b)=>a-b); return { samples:values.length,mean:values.reduce((a,b)=>a+b,0)/values.length,p95:sorted[Math.floor(sorted.length*.95)],max:Math.max(...values) }; };
          window.__liveResult={wallSeconds:(now-start-1000)/1000,strikeRequested,intervals:summary(intervals),cpu:summary(cpu),physics:summary(physics),maxDrawCalls:Math.max(...calls),timing:runtime.timing,quality:runtime.quality,metrics:runtime.renderMetrics};
          runtime.dispose();
        }
      };
    }, { scene, renderer, seconds });
    await page.waitForFunction(() => window.__liveResult, null, { timeout:(seconds+40)*1000 });
    const result=await page.evaluate(()=>window.__liveResult);
    report.live.push({ scene,renderer,...result, averageFps:1000/result.intervals.mean });
    console.log(JSON.stringify({stage:'live',scene,renderer,fps:1000/result.intervals.mean,intervalP95:result.intervals.p95,physicsMean:result.timing.meanStepMs}));
    await page.close(); await save();
  }
} catch (error) { report.errors.push(error.stack ?? String(error)); console.error(error); process.exitCode=1; }
finally {
  await browser?.close(); await server.close();
  report.sourceAfter = await fingerprint();
  report.sourceUnchanged = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
  await save();
}
if(report.errors.length) process.exitCode=1;
console.log(`Report: ${join(output,'report.json')}`);
