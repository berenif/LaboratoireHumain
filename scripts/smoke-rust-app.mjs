import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { assertSmokeAdvanceFits, assertSmokeModelContract, maximumSmokeTicks, readSmokeClock } from './rust-smoke-contract.mjs';

// One browser, one bounded physical trajectory. UI initialization/reset checks
// run at tick zero; no qualification, recovery campaign or hidden warm-up.
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', process.argv[2] ?? 'rust/dist/app', '0', process.argv[3] ?? '/'], { stdio: ['ignore', 'pipe', 'inherit'] });
let browser;
try {
  const origin = await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error('Preview startup timed out')), 10000);
    server.stdout.on('data', data => {
      const url = String(data).match(/http:\/\/127\.0\.0\.1:\d+\/\S*/)?.[0];
      if (url) { clearTimeout(timeout); resolve(url); }
    });
    server.once('error', reject);
    server.once('exit', code => { clearTimeout(timeout); reject(new Error(`Preview exited ${code}`)); });
  });
  browser = await chromium.launch(browserLaunchOptions(true));
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`); });
  await page.addInitScript(() => {
    let api;
    Object.defineProperty(window, '__lhRust', {
      configurable: true, get: () => api,
      set(value) { api = value; api.setManual(true); },
    });
  });
  await page.goto(`${origin}?renderer=webgl2`);
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3, null, { timeout: 60000 });
  const state = () => page.evaluate(() => window.__lhRust.diagnostics);
  const idle = () => page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
  const initial = await state();
  assert.equal(initial.tick, 0);
  assert.equal(initial.backend, 'WebGL2');
  assertSmokeModelContract(initial);
  assert.match(await page.locator('.preview-tag').innerText(), /Experimental/);
  for (const id of ['pause', 'reset', 'floor', 'heading', 'region', 'strike', 'renderer', 'quality', 'camera-focus', 'camera-reset']) {
    assert.equal(await page.locator(`#${id}`).count(), 1, id);
  }
  await page.getByRole('button', { name: 'Balance playground', exact: true }).click(); await idle();
  assert.equal((await state()).mode, 'playground');
  assert.equal(await page.locator('#station option').count(), 7);
  await page.locator('#station').selectOption('slope'); await idle();
  assert.equal((await state()).station, 'slope');
  await page.getByRole('button', { name: 'Impact protocol', exact: true }).click(); await idle();
  await page.locator('#reset').click(); await idle();
  assert.equal((await state()).tick, 0);
  await page.locator('#quality').selectOption('low');
  await page.waitForFunction(() => window.__lhRust.diagnostics.quality === 'low');
  await page.locator('#region').selectOption('torso');
  await page.locator('#camera-focus').click();
  await page.locator('#camera-reset').click();
  await page.locator('#pause').click(); await idle();
  assert.equal((await state()).confirmedPaused, true);
  assert.equal(await page.locator('#strike').isDisabled(), true);
  assert.equal((await page.evaluate(() => window.__lhRust.advance(4))).completed_ticks, 0);
  await page.locator('#pause').click(); await idle();
  const advance = async () => {
    assertSmokeAdvanceFits(await state(), 4);
    const reply = await page.evaluate(() => window.__lhRust.advance(4));
    assert.equal(reply.failure, null);
    assert.equal(reply.completed_ticks, 4);
  };
  for (let tick = 0; tick < 120; tick += 4) await advance();
  assert.equal((await state()).canStrike, true);
  await page.locator('#strike').click(); await idle();
  assert.equal((await state()).strikerPhase, 'positioning');
  for (let tick = 120; tick < maximumSmokeTicks && (await state()).strikes === 0; tick += 4) await advance();
  const final = await state();
  assert.equal(final.strikes, 1, 'Striker must make physical contact');
  assert.equal(final.workerInstances, 1);
  assert.ok(final.tick <= maximumSmokeTicks && final.time <= 5.001);
  assert.equal(final.failure, null);
  assert.deepEqual(errors, []);
  console.log(JSON.stringify({ passed: true, runtime: 'Rust/WASM', backend: final.backend, ticks: final.tick, simulatedSeconds: final.time, nativeSubsteps: readSmokeClock(final).nativeSubsteps, strikes: final.strikes, segments: final.snapshot.segments.length, physicalQualification: false }));
} finally {
  await browser?.close();
  server.kill();
}
