import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, join, relative, resolve } from 'node:path';
import { assertSmokeAdvanceFits, assertSmokeModelContract, readSmokeClock } from './rust-smoke-contract.mjs';

// A focused browser regression, not a recovery/performance qualification.
// Only actual Playwright mouse input starts/moves/ends grabs. Synthetic events
// below represent browser interruptions; no worker/physics state is injected.
const artifact = resolve(process.argv[2] ?? 'rust/dist/app');
const output = resolve(process.argv[3] ?? `outputs/rust-grab-${new Date().toISOString().replaceAll(/[:.]/g, '-')}`);
assert.ok(!existsSync(join(output, 'browser-result.json')), 'Use a fresh output directory');
assert.ok(existsSync(join(artifact, 'index.html')), 'Build the Rust browser app before running this test');
mkdirSync(output, { recursive: true });
// Optional system ffmpeg support for native Playwright recording in minimal CI.
// Registry is imported only after selecting the local executable directory.
if (process.env.RUST_BROWSER_FFMPEG) {
  assert.equal(process.platform, 'linux', 'RUST_BROWSER_FFMPEG currently supports Linux');
  const require = createRequire(import.meta.url);
  const core = dirname(require.resolve('playwright-core/package.json'));
  const descriptor = JSON.parse(readFileSync(join(core, 'browsers.json'), 'utf8')).browsers.find(x => x.name === 'ffmpeg');
  const tools = join(output, 'playwright-tools');
  const ffmpeg = join(tools, `ffmpeg-${descriptor.revision}`, 'ffmpeg-linux');
  mkdirSync(dirname(ffmpeg), { recursive: true });
  if (!existsSync(ffmpeg)) symlinkSync(resolve(process.env.RUST_BROWSER_FFMPEG), ffmpeg);
  process.env.PLAYWRIGHT_BROWSERS_PATH = tools;
}
const { chromium, browserLaunchOptions } = await import('./rust-browser-test.mjs');
const hash = value => createHash('sha256').update(value).digest('hex');
function assetHashes(directory) {
  return Object.fromEntries(readdirSync(directory, { withFileTypes: true }).flatMap(entry => {
    if (entry.name.startsWith('.')) return [];
    const file = join(directory, entry.name);
    assert.ok(!entry.isSymbolicLink(), 'Artifact symlinks are not admitted');
    return entry.isDirectory() ? Object.entries(assetHashes(file)) : [[relative(artifact, file).replaceAll('\\', '/'), hash(readFileSync(file))]];
  }));
}
const report = {
  started: new Date().toISOString(), passed: false,
  scope: 'Real WebGL2 Rust/WASM browser, paired physical hand-drag and interruption regression',
  physicalQualification: false,
  limits: { maximumPhysicalSecondsPerScenario: 5, requestedTicksPerAdvance: 4, pullWindowSeconds: 0.1 },
  recording: { mode: 'Native Playwright wall-time recording with visible manual simulation-clock label', retimed: false,
    note: 'Manual advance integrates 4 physical ticks (0.100 s) on demand. Wall footage is not a real-time physics/performance demonstration.' },
  artifact, artifactSha256: assetHashes(artifact), browserErrors: [], httpErrors: [], scenarios: [], checks: [],
  relevantSourceSha256: Object.fromEntries(['rust/crates/sim/src/grab.rs', 'rust/crates/sim/src/lib.rs',
    'rust/crates/browser/driver.js', 'scripts/verify-rust-grab.mjs'].map(file => [file, hash(readFileSync(file))])),
};
const save = () => writeFileSync(join(output, 'browser-result.json'), JSON.stringify(report, null, 2) + '\n');
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0'], { stdio: ['ignore', 'pipe', 'inherit'] });
let browser, context, page, video, realtimeVideo;
const state = () => page.evaluate(() => window.__lhRust.diagnostics);
const idle = () => page.waitForFunction(() => window.__lhRust && !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
const physical = d => ({ tick: d.tick, time: d.time, motion: d.motion, segments: d.snapshot.segments });
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
function difference(a, b) {
  assert.equal(a.tick, b.tick); assert.equal(a.time, b.time);
  return a.segments.map((part, i) => {
    const other = b.segments[i]; assert.equal(part.id, other.id);
    return { id: part.id, positionM: distance(part.position, other.position),
      linearVelocityMS: distance(part.linearVelocity, other.linearVelocity),
      angularVelocityRadS: distance(part.angularVelocity, other.angularVelocity),
      rotationComponentMax: Math.max(...['x', 'y', 'z', 'w'].map(axis => Math.abs(part.rotation[axis] - other.rotation[axis]))) };
  });
}
function assertSamePhysics(a, b, name) {
  const differences = difference(a, b);
  assert.deepEqual(a.segments, b.segments, name + ': all positions, rotations and velocities must match exactly');
  return { name, allPoseAndVelocityComponentsExactlyEqual: true, maximumPositionDifferenceM: Math.max(...differences.map(d => d.positionM)) };
}
async function label(name) { await page.evaluate(value => { window.__grabEvidence.scene = value; }, name); }
async function screenshot(name) {
  await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  const file = `${name}.png`; await page.screenshot({ path: join(output, file), fullPage: true }); return file;
}
async function advance(ticks = 4) {
  assertSmokeAdvanceFits(await state(), ticks);
  const reply = await page.evaluate(t => window.__lhRust.advance(t), ticks); await idle();
  assert.equal(reply.failure, null, 'Native integration integrity');
  assert.equal(reply.completed_ticks, ticks, 'Requested complete physical ticks');
  const d = await state(); readSmokeClock(d);
  assert.equal(d.error, ''); assert.equal(d.failure, null); assert.equal(d.workerInstances, 1);
  return physical(d);
}
async function reset(name) {
  await page.mouse.up(); await idle();
  const generation = (await state()).generation;
  await page.locator('#reset').click();
  await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, generation); await idle();
  await page.waitForFunction(() => window.__lhRust.diagnostics.graphics.tick === 0);
  await label(name);
  const d = await state(); assert.equal(d.tick, 0); assert.equal(d.time, 0); assert.equal(d.activePointer, null);
  assertSmokeModelContract(d); readSmokeClock(d);
  return physical(d);
}
async function grab(dx = 75) {
  const d = await state();
  const hand = d.snapshot.segments.findIndex(p => p.id === 'right_hand');
  assert.ok(hand >= 0); assert.ok(['Upright', 'Reacting'].includes(d.motion));
  const point = await page.evaluate(i => window.__lhRust.projectSegment(i), hand);
  await page.mouse.move(point.x, point.y); await page.mouse.down(); await idle();
  const picked = await state(); assert.equal(picked.activePointer, 'grab');
  assert.equal(picked.lastPick.segmentId, 'right_hand', 'Trusted pointer must pick an actual right-hand triangle');
  await page.mouse.move(point.x + dx, point.y, { steps: 6 }); await idle();
  return { hand, point, dxPixels: dx, pick: picked.lastPick };
}
async function staleMoveAndAdvance(origin) {
  await page.mouse.move(origin.x + 135, origin.y - 30, { steps: 4 }); await idle();
  assert.equal((await state()).activePointer, null, 'Stale pointer movement must not resume a grab');
  return advance();
}
try {
  const origin = await new Promise((resolve, reject) => {
    let text = ''; const timer = setTimeout(() => reject(new Error('Preview startup timed out')), 10000);
    server.stdout.on('data', data => { text += data; const url = text.match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0]; if (url) { clearTimeout(timer); resolve(url); } });
    server.once('error', reject); server.once('exit', code => { clearTimeout(timer); reject(new Error(`Preview exited ${code}`)); });
  });
  browser = await chromium.launch(browserLaunchOptions(true)); report.browserVersion = browser.version();
  context = await browser.newContext({ viewport: { width: 1280, height: 900 }, recordVideo: { dir: output, size: { width: 1280, height: 900 } } });
  await context.addInitScript(() => {
    let api;
    Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api,
      set(value) { api = value; api.setManual(true); } });
    window.__grabEvidence = { scene: 'Initialize', clockMode: 'MANUAL SIMULATION CLOCK', wallStart: performance.now(), pointerId: null, commands: [], replies: [] };
    // Read-only transport tracing. Original messages and transfer lists pass unchanged.
    const NativeWorker = window.Worker;
    window.Worker = class extends NativeWorker {
      constructor(...args) { super(...args); this.addEventListener('message', event => {
        const reply = event.data?.reply;
        if (reply) window.__grabEvidence.replies.push({ tick: reply.snapshot?.stamp?.tick, acknowledgements: reply.acknowledgements, failure: reply.failure });
      }); }
      postMessage(message, ...args) {
        if (message.commands) window.__grabEvidence.commands.push(structuredClone(message.commands));
        return super.postMessage(message, ...args);
      }
    };
    document.addEventListener('pointerdown', e => { window.__grabEvidence.pointerId = e.pointerId; }, true);
    document.addEventListener('DOMContentLoaded', () => {
      const banner = document.createElement('div'); banner.id = 'grab-evidence-clock';
      Object.assign(banner.style, { position: 'fixed', bottom: '8px', left: '8px', zIndex: '2147483647', pointerEvents: 'none', color: '#fff', background: '#091424ed', border: '1px solid #7185a3', padding: '9px 12px', font: '13px monospace', whiteSpace: 'pre' });
      document.body.append(banner);
      const draw = () => { const d = api?.diagnostics; const e = window.__grabEvidence;
        banner.textContent = `${e.clockMode} | ${e.scene}\nsim ${(d?.time ?? 0).toFixed(3)} s | tick ${d?.tick ?? 0} | wall ${((performance.now() - e.wallStart) / 1000).toFixed(2)} s\n${e.clockMode === 'MANUAL SIMULATION CLOCK' ? '4 ticks = 0.100 simulated s per advance | footage is wall time, not real-time physics' : 'Automatic scheduler | native wall-time footage | actual simulation time shown above'}`;
        requestAnimationFrame(draw); }; draw();
    });
  });
  page = await context.newPage(); video = page.video();
  page.on('pageerror', error => report.browserErrors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) report.httpErrors.push({ status: response.status(), url: response.url() }); });
  await page.goto(`${origin}?renderer=webgl2`);
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3, null, { timeout: 60000 });
  const initial = await state(); assert.equal(initial.backend, 'WebGL2'); assert.equal(initial.tick, 0);
  assertSmokeModelContract(initial); readSmokeClock(initial); report.initial = physical(initial);
  await page.locator('#region').selectOption('right_hand'); await idle();
  const fetchedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const r = await fetch(new URL(name, document.baseURI)); if (!r.ok) throw new Error('Artifact fetch failed: ' + name);
    const h = await crypto.subtle.digest('SHA-256', await r.arrayBuffer());
    return [name, [...new Uint8Array(h)].map(x => x.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(report.artifactSha256));
  assert.deepEqual(fetchedAssets, report.artifactSha256); report.checks.push('Every served asset matches the captured built artifact hash');

  await label('No-grab baseline');
  const baseline = { name: 'No grab', initial: physical(initial), samples: [] }; report.scenarios.push(baseline);
  for (let n = 0; n < 3; n++) baseline.samples.push(await advance());
  baseline.screenshot = await screenshot('baseline-0.300s'); save();

  const release = { name: 'Actual hand pull, mouse release, natural continuation', initial: await reset('Actual hand drag, then release'), samples: [] }; report.scenarios.push(release);
  report.checks.push(assertSamePhysics(release.initial, baseline.initial, 'Identical trial initialization'));
  release.input = await grab(); release.samples.push(await advance());
  release.pullVsNoGrab = difference(release.samples[0], baseline.samples[0]);
  const handDelta = release.pullVsNoGrab.find(p => p.id === 'right_hand').positionM;
  const torsoDelta = release.pullVsNoGrab.find(p => p.id === 'torso').positionM;
  assert.ok(handDelta > 0.0001, `Actual hand drag must move physical hand versus no-grab baseline: ${handDelta} m`);
  assert.ok(torsoDelta > 0.000001, `Actual hand drag must propagate to ungrabbed torso: ${torsoDelta} m`);
  release.screenshot = await screenshot('actual-drag-0.100s');
  console.log(JSON.stringify({ finding: 'actual-browser-physical-coupling', simulatedSeconds: release.samples[0].time, handDifferenceM: handDelta, ungrabbedTorsoDifferenceM: torsoDelta }));
  await page.mouse.up(); await idle(); assert.equal((await state()).activePointer, null);
  release.samples.push(await staleMoveAndAdvance(release.input.point)); release.samples.push(await advance());
  release.postReleaseScreenshot = await screenshot('released-0.300s'); save();

  const held = { name: 'Continued held-pull control', initial: await reset('Held-pull control'), samples: [] }; report.scenarios.push(held);
  held.input = await grab(); held.samples.push(await advance());
  report.checks.push(assertSamePhysics(held.samples[0], release.samples[0], 'Identical physical state before release'));
  held.samples.push(await advance());
  held.vsReleased = difference(held.samples[1], release.samples[1]);
  assert.ok(held.vsReleased.find(p => p.id === 'right_hand').positionM > 0.00001, 'Release must physically differ from a continuing held pull');
  await page.mouse.up(); await idle(); save();

  for (const interruption of ['pointercancel', 'blur']) {
    const trial = { name: interruption, initial: await reset(`${interruption}: cancel and stale pointer`), samples: [] }; report.scenarios.push(trial);
    trial.input = await grab(); trial.samples.push(await advance());
    report.checks.push(assertSamePhysics(trial.samples[0], release.samples[0], `${interruption}: identical pre-cancellation physical state`));
    if (interruption === 'pointercancel') await page.evaluate(() => {
      document.querySelector('#viewport canvas').dispatchEvent(new PointerEvent('pointercancel', { bubbles: true, pointerId: window.__grabEvidence.pointerId, pointerType: 'mouse', buttons: 1 }));
    });
    else await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    await idle(); assert.equal((await state()).activePointer, null);
    trial.samples.push(await staleMoveAndAdvance(trial.input.point));
    trial.releaseEquivalence = assertSamePhysics(trial.samples[1], release.samples[1], `${interruption}: physical continuation equals real mouse release despite stale held-button motion`);
    report.checks.push(trial.releaseEquivalence);
    trial.screenshot = await screenshot(`${interruption}-0.200s`);
    await page.mouse.up(); await idle();
    if (interruption === 'pointercancel') {
      await label('Fresh grab after pointercancel');
      trial.freshInput = await grab(-75); trial.samples.push(await advance());
      trial.freshGrabVsNoNewGrab = difference(trial.samples[2], release.samples[2]);
      assert.ok(trial.freshGrabVsNoNewGrab.find(p => p.id === 'right_hand').positionM > 0.0001, 'A fresh real pointer press must restore physical hand coupling');
      assert.ok(trial.freshGrabVsNoNewGrab.find(p => p.id === 'torso').positionM > 0.000001, 'Fresh hand pull must propagate to torso');
      trial.freshScreenshot = await screenshot('fresh-grab-0.300s');
      await page.mouse.up(); await idle();
    }
    save();
  }

  const trial = { name: 'Reset during active physical drag', initial: await reset('Reset during hand drag'), samples: [] }; report.scenarios.push(trial);
  trial.input = await grab(); trial.samples.push(await advance());
  const generation = (await state()).generation;
  // Keyboard reset occurs with the real mouse button still held.
  await page.keyboard.press('r');
  await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, generation); await idle();
  await page.waitForFunction(() => window.__lhRust.diagnostics.graphics.tick === 0);
  assert.equal((await state()).activePointer, null); trial.afterReset = physical(await state());
  report.checks.push(assertSamePhysics(trial.afterReset, baseline.initial, 'Reset restores clean physical initialization'));
  trial.samples.push(await staleMoveAndAdvance(trial.input.point));
  report.checks.push(assertSamePhysics(trial.samples[1], baseline.samples[0], 'Reset leaves no phantom physical force after stale held-button movement'));
  trial.screenshot = await screenshot('reset-clean-0.100s'); await page.mouse.up(); await idle();
  report.transport = await page.evaluate(() => ({ commands: window.__grabEvidence.commands, replies: window.__grabEvidence.replies }));
  const rejected = report.transport.replies.flatMap(r => r.acknowledgements ?? []).filter(a => a.rejected != null);
  assert.deepEqual(rejected, [], 'All actual pointer/lifecycle commands must be accepted');
  assert.deepEqual(report.browserErrors, []); assert.deepEqual(report.httpErrors, []);
  report.final = { backend: (await state()).backend, workerInstances: (await state()).workerInstances, activePointer: (await state()).activePointer };
  assert.equal(report.final.activePointer, null);
  report.checks.push('Cancellation matches an actual released physical trajectory; continuing held-pull control diverges. This tests physical behavior, not pointer metadata alone.');

  // Separate short wall-time scene. Unlike the deterministic regression above,
  // this lets the ordinary automatic scheduler run, and reports its actual pace.
  await page.close();
  page = await context.newPage(); realtimeVideo = page.video();
  page.on('pageerror', error => report.browserErrors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) report.httpErrors.push({ status: response.status(), url: response.url() }); });
  await page.goto(`${origin}?renderer=webgl2`);
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3, null, { timeout: 60000 });
  assert.equal((await state()).tick, 0);
  await page.locator('#region').selectOption('right_hand'); await idle();
  const realtimeInput = await grab();
  await page.evaluate(() => {
    window.__grabEvidence.clockMode = 'AUTOMATIC SCHEDULER'; window.__grabEvidence.scene = 'Actual hand drag, then release';
    window.__grabEvidence.wallStart = performance.now(); window.__lhRust.setManual(false);
  });
  await page.waitForTimeout(1100);
  await page.mouse.up();
  await page.waitForFunction(() => window.__lhRust.diagnostics.activePointer === null);
  const realtimeReleased = physical(await state()); assert.equal((await state()).activePointer, null);
  const remaining = await page.evaluate(() => Math.max(0, 2000 - (performance.now() - window.__grabEvidence.wallStart)));
  await page.waitForTimeout(remaining);
  await page.evaluate(() => window.__lhRust.setManual(true)); await idle();
  const realtimeFinal = await state(); readSmokeClock(realtimeFinal);
  assert.ok(realtimeFinal.tick > 0, 'Automatic scheduler must actually integrate physics');
  assert.equal(realtimeFinal.failure, null); assert.equal(realtimeFinal.error, '');
  assert.equal(realtimeFinal.activePointer, null); assert.equal(realtimeFinal.workerInstances, 1);
  await label('Automatic 2 s observation ended; frozen for screenshot');
  report.realtimeScene = { input: realtimeInput, release: realtimeReleased, final: physical(realtimeFinal),
    observedWallSeconds: await page.evaluate(() => (performance.now() - window.__grabEvidence.wallStart) / 1000),
    screenshot: await screenshot('automatic-scheduler-after-2s-wall'),
    note: 'Separate native wall-time footage using the automatic scheduler; actual integrated time is reported, with no speed/performance admission.' };
  assert.deepEqual(report.browserErrors, []); assert.deepEqual(report.httpErrors, []);
  report.passed = true;
} catch (error) {
  report.error = error.stack ?? String(error);
  if (page && !page.isClosed()) {
    try { report.failureState = await state(); await screenshot('failure'); } catch {}
  }
} finally {
  report.finished = new Date().toISOString();
  try {
    await context?.close();
    if (video) { const file = join(output, 'grab-manual-clock.webm'); await video.saveAs(file); report.recording.path = relative(output, file); }
    if (realtimeVideo) { const file = join(output, 'grab-automatic-scheduler.webm'); await realtimeVideo.saveAs(file); report.realtimeRecording = { path: relative(output, file), retimed: false, mode: 'Native Playwright wall time; automatic scheduler' }; }
  }
  catch (error) { report.recording.error = String(error); report.passed = false; }
  await browser?.close(); server.kill(); save();
}
console.log(JSON.stringify({ passed: report.passed, output, error: report.error, recording: report.recording, scenarios: report.scenarios.map(s => ({ name: s.name, maximumSimulationSeconds: Math.max(s.initial?.time ?? 0, ...s.samples.map(p => p.time)) })) }, null, 2));
process.exitCode = report.passed ? 0 : 1;
