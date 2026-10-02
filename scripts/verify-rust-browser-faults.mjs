// Fault injection surrounds the real worker adapter; the shipped WASM is intact.
import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3] ?? 'http://127.0.0.1:5185/';
const hash = file => createHash('sha256').update(readFileSync(file)).digest('hex');
const sources = ['rust/crates/browser/driver.js', 'rust/crates/browser/src/main.rs', 'rust/browser/sim-worker.js',
  'rust/crates/sim/src/runtime.rs', 'rust/crates/worker/src/lib.rs', 'scripts/verify-rust-browser-faults.mjs'];
const sourceBefore = Object.fromEntries(sources.map(file => [file, hash(file)]));
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), sourceBefore, cases: [], passed: false,
  scope: 'Real browser/bootstrap/lifecycle with an appended worker crash/hang harness. No performance or full physics admission.' };
const browser = await chromium.launch(browserLaunchOptions(true));
report.browser = browser.version();
const diagnostics = page => page.evaluate(() => window.__lhRust.diagnostics);
const idle = page => page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
let currentPage;
try {
  for (const backend of ['webgl2', 'webgpu']) {
    const trial = { backend, checks: [], errors: [] }; report.cases.push(trial);
    const context = await browser.newContext({ viewport: { width: 1280, height: 900 } });
    const page = await context.newPage(); currentPage = page;
    page.on('pageerror', error => trial.errors.push(String(error)));
    page.on('console', message => { if (message.type() === 'error') trial.errors.push(message.text()); });
    await page.addInitScript(() => {
      const OriginalWorker = window.Worker;
      window.__qaWorkers = [];
      window.Worker = class extends OriginalWorker {
        constructor(url, options) { super(url, options); window.__qaWorkers.push(this); }
      };
    });
    await page.route('**/sim-worker.js', async route => {
      if (route.request().resourceType() !== 'script') { await route.continue(); return; }
      const response = await route.fetch(); const original = await response.text();
      trial.workerAdapterOriginalSha256 = createHash('sha256').update(original).digest('hex');
      const harness = `\nconst qaHandler = self.onmessage; let qaFault = null;
self.onmessage = event => {
  if (event.data.type === '__qa_armCrash') { qaFault = 'crash'; return; }
  if (event.data.type === '__qa_armHang') { qaFault = 'hang'; return; }
  if (event.data.type === 'advance' && qaFault) {
    if (qaFault === 'crash') throw new Error('Intentional QA worker crash');
    return;
  }
  qaHandler(event);
};\n`;
      await route.fulfill({ response, body: original + harness });
    });
    let releaseWasm;
    const gate = new Promise(resolve => { releaseWasm = resolve; });
    await page.route('**/lh_worker_bg.wasm', async route => { await gate; await route.continue(); });
    const url = new URL(origin); url.searchParams.set('renderer', backend);
    await page.goto(url.href, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(() => window.__lhRust && window.__lhRust.diagnostics.graphics?.backend, {}, { timeout: 45000 });
    await page.evaluate(() => window.__lhRust.setManual(true));
    for (const selector of ['#pause', '#reset', '#floor', '#heading', '.mode-tabs button']) {
      for (const control of await page.locator(selector).all()) assert.equal(await control.isDisabled(), true);
    }
    await page.locator('h1').click(); await page.keyboard.press('Space'); await page.keyboard.press('g'); await page.keyboard.press('r');
    await page.evaluate(() => {
      window.__lhRust.action({ type: 'floor', value: false });
      window.__lhRust.action({ type: 'heading', value: 1.0471975512 });
      window.__lhRust.action({ type: 'mode', value: 'playground' });
    });
    let state = await diagnostics(page);
    assert.equal(state.ready, false); assert.equal(state.paused, false); assert.equal(state.heading, 0);
    assert.equal(state.mode, 'protocol'); assert.equal(state.desiredFloorEnabled, true);
    assert.equal(await page.locator('#floor').isChecked(), true); assert.equal(await page.locator('#heading').inputValue(), '0');
    trial.checks.push('Held worker bootstrap disables physical controls and ignores premature keyboard/debug actions');
    releaseWasm(); await page.unroute('**/lh_worker_bg.wasm');
    await page.waitForFunction(() => window.__lhRust.diagnostics.ready, {}, { timeout: 45000 }); await idle(page);
    state = await diagnostics(page); assert.equal(state.tick, 0); assert.equal(state.paused, false); assert.equal(state.floorEnabled, true);
    const head = await page.evaluate(() => window.__lhRust.projectSegment(4));
    await page.mouse.move(head.x, head.y); await page.mouse.down(); await idle(page);
    assert.equal((await diagnostics(page)).activePointer, 'grab');
    const beforeCollapse = await diagnostics(page);
    await page.evaluate(async () => {
      document.getElementById('simulation').style.display = 'none';
      for (let i = 0; i < 3; i++) await new Promise(requestAnimationFrame);
    });
    state = await diagnostics(page);
    assert.equal(state.error, ''); assert.equal(state.ready, true); assert.equal(state.activePointer, null);
    assert.deepEqual(state.snapshot.segments, beforeCollapse.snapshot.segments);
    const collapsedFrames = state.graphics.frames;
    await page.evaluate(() => { document.getElementById('simulation').style.display = ''; });
    await page.waitForFunction(frames => window.__lhRust.diagnostics.graphics.frames > frames, collapsedFrames);
    await page.mouse.move(head.x + 8, head.y); await page.mouse.up(); await idle(page);
    assert.equal((await diagnostics(page)).activePointer, null);
    assert.equal((await diagnostics(page)).workerInstances, beforeCollapse.workerInstances);
    trial.checks.push('A zero-sized canvas cancels the active grab, suspends drawing and restores its view without a physical reset or old-press resumption');
    await page.getByRole('button', { name: 'Balance playground', exact: true }).click(); await idle(page);
    await page.locator('#heading').selectOption('1.0471975512'); await idle(page);
    await page.locator('#floor').uncheck(); await idle(page);
    const initial = await diagnostics(page);
    for (const fault of ['Crash', 'Hang']) {
      const before = await diagnostics(page);
      await page.evaluate(fault => {
        window.__qaWorkers.at(-1).postMessage({ type: '__qa_arm' + fault });
        window.__qaAdvanceResult = window.__lhRust.advance(4).then(() => 'resolved', error => error.message);
      }, fault);
      await page.waitForFunction(() => window.__lhRust.diagnostics.workerFault, {}, { timeout: 15000 });
      const failure = await page.evaluate(() => window.__qaAdvanceResult);
      assert.match(failure, fault === 'Crash' ? /Intentional QA worker crash/ : /timed out/);
      state = await diagnostics(page);
      assert.equal(state.outstanding, false); assert.equal(state.queueDepth, 0); assert.equal(state.manualRequests, 0);
      assert.equal(state.ready, false); assert.equal(state.canReset, true); assert.equal(state.activePointer, null);
      assert.deepEqual(state.snapshot.segments, before.snapshot.segments);
      assert.equal(await page.locator('#pause').isDisabled(), true); assert.equal(await page.locator('#reset').isDisabled(), false);
      assert.match(await page.evaluate(() => window.__lhRust.advance(1).catch(error => error.message)), /unavailable/);
      await page.locator('#reset').click();
      await page.waitForFunction(() => window.__lhRust.diagnostics.ready, {}, { timeout: 45000 }); await idle(page);
      state = await diagnostics(page);
      assert.equal(state.workerInstances, before.workerInstances + 1); assert.equal(state.generation, before.generation + 1);
      assert.equal(state.tick, 0); assert.equal(state.workerFault, false); assert.equal(state.mode, 'playground');
      assert.equal(state.floorEnabled, false); assert.equal(state.desiredFloorEnabled, false); assert.equal(state.heading, initial.heading);
      assert.deepEqual(state.snapshot.segments, initial.snapshot.segments);
      assert.deepEqual(state.graphics.camera, before.graphics.camera);
      await page.evaluate(() => window.__qaWorkers.at(-2).dispatchEvent(new MessageEvent('message', { data: { type: 'snapshot', requestId: -1 } })));
      assert.equal((await diagnostics(page)).workerFault, false);
      const reply = await page.evaluate(() => window.__lhRust.advance(4));
      assert.equal(reply.completed_ticks, 4); assert.equal(reply.failure, null); await idle(page);
      await page.locator('#reset').click(); await idle(page);
      trial.checks.push(`Actual worker ${fault.toLowerCase()} rejects the pending advance; Reset starts a clean generation and ignores late old-worker packets`);
    }
    if ((await diagnostics(page)).backend.toLowerCase() !== 'webgl2') {
      await page.locator('#renderer').selectOption('webgl2');
      await page.waitForFunction(() => window.__lhRust.diagnostics.ready && window.__lhRust.diagnostics.backend.toLowerCase() === 'webgl2', {}, { timeout: 45000 });
    }
    assert.equal(await page.evaluate(() => {
      const extension = document.getElementById('simulation').getContext('webgl2').getExtension('WEBGL_lose_context');
      if (!extension) return false;
      extension.loseContext(); return true;
    }), true);
    await page.waitForFunction(() => !window.__lhRust.diagnostics.ready && window.__lhRust.diagnostics.error.includes('lost'));
    const lost = await diagnostics(page);
    await page.locator('#reset').click(); await idle(page);
    state = await diagnostics(page);
    assert.equal(state.generation, lost.generation + 1); assert.equal(state.ready, false);
    assert.match(state.error, /Graphics connection lost/); assert.equal(await page.locator('#graphics-retry').isVisible(), true);
    const resetWhileLost = state;
    await page.locator('#graphics-retry').click();
    await page.waitForFunction(() => window.__lhRust.diagnostics.ready, {}, { timeout: 45000 });
    state = await diagnostics(page);
    assert.deepEqual(state.snapshot.segments, resetWhileLost.snapshot.segments);
    assert.equal(state.workerInstances, resetWhileLost.workerInstances); assert.equal(state.error, '');
    trial.checks.push('Reset during actual graphics loss retains the visible retry action; retry preserves the reset physics');
    await page.evaluate(() => {
      window.__qaWorkers.at(-1).postMessage({ type: '__qa_armHang' });
      window.__qaAdvanceResult = window.__lhRust.advance(4).catch(error => error.message);
      window.__lhRust.dispose();
    });
    assert.equal(await page.evaluate(() => window.__qaAdvanceResult), 'Disposed');
    state = await diagnostics(page); assert.equal(state.disposed, true); assert.equal(state.outstanding, false);
    trial.checks.push('Teardown rejects a pending advance and clears its watchdog');
    assert.deepEqual(trial.errors, []); trial.passed = true;
    await context.close();
  }
  report.passed = true;
} catch (error) {
  report.failure = String(error);
  if (currentPage) {
    try { report.failureState = await diagnostics(currentPage); await currentPage.screenshot({ path: join(output, 'failure.png'), fullPage: true }); }
    catch (captureError) { report.captureFailure = String(captureError); }
  }
} finally {
  report.sourceUnchanged = sources.every(file => hash(file) === sourceBefore[file]);
  report.passed &&= report.sourceUnchanged;
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, failure: report.failure, sourceUnchanged: report.sourceUnchanged,
  cases: report.cases.map(trial => ({ backend: trial.backend, passed: trial.passed, checks: trial.checks.length, errors: trial.errors })) }));
if (!report.passed) process.exitCode = 1;
