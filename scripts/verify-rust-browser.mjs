import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3] ?? 'http://127.0.0.1:5185/';
const artifact = resolve(process.argv[4] ?? 'rust/dist/app');
const hash = value => createHash('sha256').update(value).digest('hex');
function files(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap(entry => {
    if (['target', 'dist', '.git'].includes(entry.name)) return [];
    assert.ok(!entry.isSymbolicLink(), 'Refuse source/artifact symlink: ' + join(directory, entry.name));
    const file = join(directory, entry.name);
    return entry.isDirectory() ? files(file) : [file];
  });
}
const sourceFiles = [...files('rust/crates'), ...files('rust/vendor'),
  ...files('rust/browser'), 'rust/Cargo.toml', 'rust/Cargo.lock', 'rust/Trunk.toml', 'rust/rust-toolchain.toml',
  'scripts/build-rust-browser.mjs', 'scripts/build-rust-worker.mjs',
  'scripts/serve-rust-preview.mjs', 'scripts/verify-rust-browser.mjs'].map(file => file.replaceAll('\\', '/')).sort();
const sourceBefore = Object.fromEntries(sourceFiles.map(file => [file, hash(readFileSync(file))]));
const localAssets = Object.fromEntries(files(artifact).map(file => [relative(artifact, file).replaceAll('\\', '/'), hash(readFileSync(file))]));
mkdirSync(output, { recursive: true });
const report = { scope: 'Built Rust UI, actual WebGPU/WebGL2, physical pointer and lifecycle smoke. No full physics, hardware/touch or performance admission.',
  started: new Date().toISOString(), sourceBefore, localAssets, cases: [], passed: false };
const browser = await chromium.launch(browserLaunchOptions(true));
report.browser = browser.version();
let currentPage;
const state = page => page.evaluate(() => window.__lhRust.diagnostics);
const idle = page => page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
const physical = value => ({ tick: value.tick, time: value.time, generation: value.generation, segments: value.snapshot.segments });
async function advance(page, ticks) {
  const reply = await page.evaluate(t => window.__lhRust.advance(t), ticks);
  await idle(page);
  assert.equal(reply.failure, null, 'Native integrity must remain valid');
  return reply;
}
try {
  for (const backend of ['webgl2', 'webgpu']) {
    const trial = { backend, checks: [], errors: [], httpErrors: [], passed: false };
    report.cases.push(trial);
    const context = await browser.newContext({ viewport: { width: 1280, height: 900 }, hasTouch: true });
    const page = await context.newPage();
    currentPage = page;
    page.on('pageerror', error => trial.errors.push(String(error)));
    page.on('console', message => { if (message.type() === 'error') trial.errors.push(message.text()); });
    page.on('response', response => { if (response.status() >= 400) trial.httpErrors.push({ url: response.url(), status: response.status() }); });
    const url = new URL(origin); url.searchParams.set('renderer', backend);
    await page.goto(url.href);
    await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3, {}, { timeout: 45000 });
    await page.evaluate(() => window.__lhRust.setManual(true)); await idle(page);
    let value = await state(page);
    assert.equal(value.backend, backend === 'webgl2' ? 'WebGL2' : 'WebGPU');
    assert.equal(value.graphics.renderedSegments, 25);
    assert.match(value.graphics.renderFormat, /Srgb$/);
    assert.equal(await page.locator('#renderer').inputValue(), backend);
    trial.checks.push('Forced backend renders all 25 segments through an sRGB attachment and is reflected in the UI');
    trial.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
      const response = await fetch(new URL(name, document.baseURI));
      if (!response.ok) throw new Error('Artifact fetch failed: ' + name);
      const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
      return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
    }))), Object.keys(localAssets));
    assert.deepEqual(trial.loadedAssets, localAssets);
    trial.checks.push('Every served artifact equals its captured local hash');
    await page.screenshot({ path: join(output, backend + '-desktop.png'), fullPage: true });
    for (const mode of ['protocol', 'playground']) {
      value = await state(page);
      if (value.mode !== mode) {
        const generation = value.generation;
        await page.getByRole('button', { name: mode === 'protocol' ? 'Impact protocol' : 'Balance playground', exact: true }).click();
        await page.waitForFunction(({ mode, generation }) => window.__lhRust.diagnostics.mode === mode && window.__lhRust.diagnostics.generation === generation + 1, { mode, generation });
        await idle(page);
      }
      await page.locator('#pause').click();
      await page.waitForFunction(() => window.__lhRust.diagnostics.confirmedPaused); await idle(page);
      const paused = physical(await state(page));
      const stopped = await advance(page, 4);
      assert.equal(stopped.completed_ticks, 0);
      assert.deepEqual(physical(await state(page)), paused);
      const frames = (await state(page)).graphics.frames;
      await page.locator('#quality').selectOption('low');
      await page.locator('#camera-focus').click();
      await page.waitForFunction(previous => window.__lhRust.diagnostics.graphics.frames > previous, frames);
      assert.deepEqual(physical(await state(page)), paused);
      trial.checks.push(mode + ': pause freezes actual positions, velocities and time while the view redraws');
      await page.locator('#pause').click();
      await page.waitForFunction(() => !window.__lhRust.diagnostics.confirmedPaused); await idle(page);
      const before = await state(page);
      const resumed = await advance(page, 4);
      assert.equal(resumed.completed_ticks, 4);
      value = await state(page);
      assert.equal(value.tick, before.tick + 4);
      assert.ok(Math.abs(value.time - before.time - 4 / 60) < 1e-12);
      trial.checks.push(mode + ': resume advances exactly four fixed ticks');
      const generation = value.generation;
      await page.locator('#reset').click();
      await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, generation); await idle(page);
      value = await state(page);
      assert.equal(value.tick, 0); assert.equal(value.time, 0); assert.equal(value.workerInstances, 1);
      trial.checks.push(mode + ': reset isolates a new generation in the same worker');
    }
    await page.locator('#camera-reset').click();
    const fixedFrames = (await state(page)).graphics.frames;
    await page.waitForFunction(frames => window.__lhRust.diagnostics.graphics.frames > frames, fixedFrames);
    await page.screenshot({ path: join(output, backend + '-fixed-pose.png'), fullPage: true });
    const grounded = await state(page);
    await page.locator('#floor').uncheck();
    await page.waitForFunction(() => !window.__lhRust.diagnostics.floorEnabled); await idle(page);
    value = await state(page);
    assert.equal(value.graphics.floorEnabled, false); assert.equal(value.snapshot.contacts.length, 0);
    await advance(page, 4);
    assert.ok((await state(page)).snapshot.segments[0].position.y < grounded.snapshot.segments[0].position.y);
    await advance(page, 4); await advance(page, 4);
    value = await state(page);
    assert.equal(value.snapshot.motion, 'Falling'); assert.equal(value.snapshot.counters.falls, 1);
    assert.equal(await page.locator('#motion').textContent(), 'Falling');
    const fallingHead = await page.evaluate(() => window.__lhRust.projectSegment(4));
    await page.mouse.move(fallingHead.x, fallingHead.y); await page.mouse.down(); await idle(page);
    assert.equal((await state(page)).activePointer, 'orbit');
    await page.mouse.up(); await idle(page);
    trial.checks.push('An unsupported body reports one measured fall, updates the UI and refuses a new surface grab');
    await page.locator('#floor').check();
    await page.waitForFunction(() => window.__lhRust.diagnostics.floorEnabled); await idle(page);
    trial.checks.push('Ground control removes native contact and rendered ground; the body then falls under gravity');
    const generation = (await state(page)).generation;
    await page.locator('#heading').selectOption('1.0471975512');
    await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, generation); await idle(page);
    value = await state(page);
    assert.ok(Math.abs(value.snapshot.segments[0].rotation.y - 0.5) < 1e-5);
    trial.checks.push('Heading control rotates the initialized physical body by 60 degrees');
    await page.locator('#heading').selectOption('0');
    await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, value.generation); await idle(page);
    await page.locator('#camera-reset').click();
    await page.locator('#region').selectOption('head');
    await page.waitForFunction(() => window.__lhRust.diagnostics.selected === 'head');
    // Same native initial state and clock, first without input and then with a
    // trusted pointer drag. This distinguishes applied force from mere picking.
    for (let i = 0; i < 3; i++) await advance(page, 4);
    const noGrab = await state(page);
    await page.locator('#reset').click();
    await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, noGrab.generation); await idle(page);
    await page.waitForFunction(() => window.__lhRust.diagnostics.graphics.tick === 0);
    const point = await page.evaluate(() => window.__lhRust.projectSegment(4));
    await page.mouse.move(point.x, point.y); await page.mouse.down(); await idle(page);
    value = await state(page);
    assert.equal(value.lastPick.segmentId, 'head');
    assert.equal(value.activePointer, 'grab');
    await page.mouse.move(point.x + 70, point.y, { steps: 6 }); await idle(page);
    for (let i = 0; i < 3; i++) await advance(page, 4);
    const grabbed = await state(page);
    const a = noGrab.snapshot.segments[4].position, b = grabbed.snapshot.segments[4].position;
    const displacement = Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    assert.ok(displacement > 0.001, 'The real drag must change physical motion versus the same undisturbed clock');
    await page.mouse.up(); await idle(page);
    assert.equal((await state(page)).activePointer, null);
    trial.drag = { localAnchor: value.lastPick.localAnchor, triangle: value.lastPick.triangle, displacement };
    trial.checks.push('Trusted pointer picks a canonical head triangle and physically changes motion; release cancels the pointer');
    const secondPick = await page.evaluate(() => window.__lhRust.projectSegment(4));
    await page.mouse.move(secondPick.x, secondPick.y); await page.mouse.down(); await idle(page);
    assert.equal((await state(page)).activePointer, 'grab');
    await page.keyboard.press('ArrowRight'); await idle(page);
    assert.equal((await state(page)).activePointer, null);
    await page.mouse.up();
    trial.checks.push('Keyboard camera movement cancels an active physical grab');
    await page.keyboard.press('Space');
    await page.waitForFunction(() => window.__lhRust.diagnostics.confirmedPaused); await idle(page);
    const suspended = physical(await state(page));
    await page.keyboard.press('ArrowRight');
    assert.deepEqual(physical(await state(page)), suspended);
    trial.checks.push('Canvas keyboard shortcuts pause physics and orbit without integrating');
    const other = backend === 'webgl2' ? 'webgpu' : 'webgl2';
    const changes = (await state(page)).rendererChanges;
    await page.locator('#renderer').selectOption(other);
    await page.waitForFunction(({ other, changes }) => window.__lhRust.diagnostics.ready && window.__lhRust.diagnostics.rendererChanges === changes + 1 && window.__lhRust.diagnostics.backend.toLowerCase() === other, { other, changes }, { timeout: 45000 });
    assert.deepEqual(physical(await state(page)), suspended);
    assert.equal((await state(page)).workerInstances, 1);
    trial.checks.push('Changing graphics backend preserves the paused physics and the single worker');
    // Exercise actual browser context loss, rather than dispatching a synthetic
    // notification. WebGPU device loss remains a separate qualification item.
    if (other === 'webgl2') {
      const changes = (await state(page)).rendererChanges;
      assert.equal(await page.evaluate(() => {
        const extension = document.getElementById('simulation').getContext('webgl2').getExtension('WEBGL_lose_context');
        if (!extension) return false;
        extension.loseContext(); return true;
      }), true);
      await page.waitForFunction(() => !window.__lhRust.diagnostics.ready && window.__lhRust.diagnostics.error.includes('lost'));
      assert.deepEqual(physical(await state(page)), suspended);
      await page.locator('#graphics-retry').click();
      await page.waitForFunction(changes => window.__lhRust.diagnostics.ready && window.__lhRust.diagnostics.rendererChanges === changes + 1, changes, { timeout: 45000 });
      assert.deepEqual(physical(await state(page)), suspended);
      assert.equal((await state(page)).workerInstances, 1);
      trial.checks.push('Actual WebGL context loss and retry preserve physical state in the same worker');
    }
    await assert.rejects(page.evaluate(() => window.__lhRust.advance(5)), /0–4 ticks/);
    trial.checks.push('Oversized manual advances are rejected');
    await page.setViewportSize({ width: 390, height: 844 });
    await page.waitForFunction(() => window.__lhRust.diagnostics.graphics.width < 500);
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
    await page.screenshot({ path: join(output, backend + '-mobile-layout.png'), fullPage: true });
    trial.checks.push('Narrow layout resizes the actual renderer without horizontal overflow');
    await page.locator('#pause').click();
    await page.waitForFunction(() => !window.__lhRust.diagnostics.confirmedPaused); await idle(page);
    value = await state(page);
    await page.locator('#reset').click();
    await page.waitForFunction(g => window.__lhRust.diagnostics.generation === g + 1, value.generation); await idle(page);
    await page.locator('#simulation').scrollIntoViewIfNeeded();
    const touchFrames = (await state(page)).graphics.frames;
    await page.waitForFunction(frames => window.__lhRust.diagnostics.graphics.frames > frames, touchFrames);
    // Chromium dispatch generates trusted browser touch/pointer events. This
    // checks the adapter but does not qualify a physical touchscreen device.
    const cdp = await context.newCDPSession(page);
    const touchHead = await page.evaluate(() => window.__lhRust.projectSegment(4));
    const first = { x: touchHead.x, y: touchHead.y, id: 1 };
    const second = { x: touchHead.x + 45, y: touchHead.y + 10, id: 2 };
    const camera = (await state(page)).graphics.camera;
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [first] }); await idle(page);
    assert.equal((await state(page)).activePointer, 'grab');
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [first, second] }); await idle(page);
    assert.equal((await state(page)).activePointer, null);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ ...first, x: first.x - 10 }, { ...second, x: second.x + 30, y: second.y + 10 }] });
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] }); await idle(page);
    const cameraAfter = (await state(page)).graphics.camera;
    assert.ok(cameraAfter.distance < camera.distance && cameraAfter.yaw !== camera.yaw);
    for (let i = 0; i < 3; i++) await advance(page, 4);
    assert.deepEqual((await state(page)).snapshot.segments, noGrab.snapshot.segments);
    trial.checks.push('Trusted emulated touch picks a surface; two-finger pinch/orbit cancels the grab and leaves undisturbed native motion');
    trial.final = await state(page);
    await page.evaluate(() => window.__lhRust.dispose());
    assert.equal((await state(page)).disposed, true);
    assert.equal((await state(page)).graphics, null);
    trial.checks.push('Explicit teardown disposes rendering and the bridge');
    assert.deepEqual(trial.errors, []); assert.deepEqual(trial.httpErrors, []);
    trial.passed = true;
    await context.close();
  }
  report.passed = true;
} catch (error) {
  report.failure = String(error);
  if (currentPage) {
    try {
      report.failureState = await state(currentPage);
      await currentPage.screenshot({ path: join(output, 'failure.png'), fullPage: true });
    } catch (captureError) { report.captureFailure = String(captureError); }
  }
}
finally {
  report.sourceUnchanged = sourceFiles.every(file => hash(readFileSync(file)) === sourceBefore[file]);
  report.artifactsUnchanged = Object.entries(localAssets).every(([file, digest]) => hash(readFileSync(join(artifact, file))) === digest);
  report.passed &&= report.sourceUnchanged && report.artifactsUnchanged;
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, failure: report.failure, browser: report.browser,
  sourceUnchanged: report.sourceUnchanged, artifactsUnchanged: report.artifactsUnchanged,
  cases: report.cases.map(trial => ({ backend: trial.backend, passed: trial.passed, checks: trial.checks.length, errors: trial.errors })) }));
if (!report.passed) process.exitCode = 1;
