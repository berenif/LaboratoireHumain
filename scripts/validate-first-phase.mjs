// One metered ordinary-browser history; no physics is run while encoding its frames.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';

const caseName = process.argv[2];
assert.ok(['primary', 'repeat'].includes(caseName));
const root = 'outputs/browser-first-phase-2026-10-16';
const ledger = JSON.parse(readFileSync('docs/evidence/browser-first-phase-2026-10-16.json'));
assert.equal(ledger.status, 'reserved');
for (const [path, hash] of Object.entries(ledger.inputs)) {
  assert.equal(createHash('sha256').update(readFileSync(path)).digest('hex'), hash, `Frozen input changed: ${path}`);
}
if (caseName === 'repeat') assert.equal(JSON.parse(readFileSync(`${root}/primary/result.json`)).passed, true);
const output = `${root}/${caseName}`;
mkdirSync(output, { recursive: true });
openSync(`${output}/started-once`, 'wx');
mkdirSync(`${output}/frames`);
const result = { caseName, startedUtc: new Date().toISOString(), qualification: false, maximumTicks: 840, maximumWallSeconds: 180, phases: [], armStages: [], frames: 0 };
const started = performance.now();
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', 'rust/dist/app', '0'], { stdio: ['ignore', 'pipe', 'inherit'] });
let browser, page;
const watchdog = setTimeout(() => { browser?.close(); server.kill(); }, 180000);
try {
  const origin = await new Promise((resolve, reject) => {
    server.stdout.on('data', data => { const url = String(data).match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0]; if (url) resolve(url); });
    server.once('error', reject);
    server.once('exit', code => reject(new Error(`Preview exited ${code}`)));
  });
  browser = await chromium.launch(browserLaunchOptions(true));
  page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  result.browser = browser.version();
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  await page.addInitScript(() => {
    let api;
    Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api, set(value) { api = value; api.setManual(true); } });
  });
  await page.goto(`${origin}?renderer=webgl2`);
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready, null, { timeout: 45000 });
  assert.equal(await page.evaluate(() => window.__lhQualification), undefined, 'Use ordinary initialization');
  const state = () => page.evaluate(() => window.__lhRust.diagnostics);
  let value = await state();
  assert.equal(value.tick, 0, 'No unmetered automatic integration');
  const record = async () => {
    value = await state();
    const recovery = value.snapshot.diagnostics.recovery;
    const first = recovery?.first_phase;
    result.final = { tick: value.tick, time: value.time, motion: value.motion, counters: value.snapshot.counters, recovery, failure: value.failure, workerInstances: value.workerInstances };
    result.nativeSteps = value.tick * 4;
    appendFileSync(`${output}/trace.ndjson`, JSON.stringify(value.snapshot) + '\n');
    // Save progress before assertions so a failed criterion cannot erase consumed work.
    writeFileSync(`${output}/result.json`, JSON.stringify(result, null, 2) + '\n');
    if (recovery) {
      if (!result.firstRecovery) {
        result.firstRecovery = { time: value.time, tick: value.tick };
        await page.locator('#camera-focus').click();
      }
      assert.ok(first, 'Missing first-phase diagnostic');
      assert.ok(first.blocked_reason || (first.complete && first.eligible), 'Missing progress/failure explanation');
      assert.equal(recovery.phase, 'Roll', 'Later recovery phase must remain disabled');
      assert.notEqual(recovery.arm_transfer, 'right:Roll', 'Arm transfer must stop before roll continuation');
      if (!result.phases.includes(recovery.phase)) result.phases.push(recovery.phase);
      if (result.armStages.at(-1)?.stage !== recovery.arm_transfer) result.armStages.push({ stage: recovery.arm_transfer, time: value.time });
      if (first.complete && !result.firstCompletion) result.firstCompletion = { time: value.time, observation: first };
    }
    await page.screenshot({ path: `${output}/frames/${String(result.frames++).padStart(4, '0')}.jpg`, type: 'jpeg', quality: 65 });
    writeFileSync(`${output}/result.json`, JSON.stringify(result, null, 2) + '\n');
  };
  const advance = async () => {
    assert.ok(performance.now() - started < 175000, 'Wall deadline');
    const reply = await page.evaluate(() => window.__lhRust.advance(4));
    await record();
    assert.equal(reply.failure, null);
    assert.equal(reply.completed_ticks, 4);
  };
  for (let tick = 0; tick < 120; tick += 4) await advance();
  await page.locator('#strike').click();
  await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
  assert.equal((await state()).strikerPhase, 'positioning');
  while (value.tick < result.maximumTicks && !value.failure) {
    await advance();
    const first = value.snapshot.diagnostics.recovery?.first_phase;
    if (first?.complete && first.hold_valid_s >= 1 && value.time - result.firstCompletion.time >= 1) break;
  }
  result.errors = errors;
  assert.deepEqual(errors, []);
  assert.equal(value.workerInstances, 1);
  assert.equal(value.strikes, 1);
  assert.equal(value.snapshot.counters.recoveries, 0, 'First phase is not full get-up');
  assert.deepEqual(result.armStages.map(row => row.stage), ['right:BackSupport', 'right:Fold', 'right:Complete']);
  const first = value.snapshot.diagnostics.recovery?.first_phase;
  assert.ok(first?.complete, 'First phase did not complete within the frozen history');
  assert.ok(first.eligible && first.hold_valid_s >= 1, 'No continuous one-second supported arm clearance after completion');
  assert.ok(first.qualified_s >= 0.55, 'Completion requires consecutive evidence');
  assert.equal(first.blocked_reason, null);
  result.passed = true;
} catch (error) {
  result.error = String(error);
  result.passed = false;
  process.exitCode = 1;
} finally {
  result.wallSeconds = (performance.now() - started) / 1000;
  result.finishedUtc = new Date().toISOString();
  writeFileSync(`${output}/result.json`, JSON.stringify(result, null, 2) + '\n');
  clearTimeout(watchdog);
  await browser?.close();
  server.kill();
  console.log(JSON.stringify(result));
}
