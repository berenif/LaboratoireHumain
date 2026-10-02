import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';

const output = resolve(process.argv[2] ?? 'evidence/rust-striker-browser');
const origin = process.argv[3] ?? 'http://127.0.0.1:5187/';
const artifact = resolve(process.argv[4] ?? 'rust/dist/striker');
assert.ok(!existsSync(output), 'Use a fresh evidence directory');
const hash = value => createHash('sha256').update(value).digest('hex');
function files(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap(entry => {
    if (['target', 'dist', '.git'].includes(entry.name)) return [];
    assert.ok(!entry.isSymbolicLink(), 'Refuse symlink: ' + join(directory, entry.name));
    const path = join(directory, entry.name);
    return entry.isDirectory() ? files(path) : [path];
  });
}
const sources = [...files('rust/crates'), ...files('rust/vendor'), ...files('rust/browser'),
  'rust/Cargo.toml', 'rust/Cargo.lock', 'scripts/verify-rust-striker.mjs'].sort();
const sourceHashes = () => Object.fromEntries(sources.map(path => [path.replaceAll('\\', '/'), hash(readFileSync(path))]));
const assets = Object.fromEntries(files(artifact).map(path => [relative(artifact, path).replaceAll('\\', '/'), hash(readFileSync(path))]));
const report = { scope: 'Actual WASM contact strikes, body descent and apparatus lifecycle on WebGPU/WebGL2 at three headings; no recovery, five-cycle or performance admission.',
  started: new Date().toISOString(), sourceBefore: sourceHashes(), assets, cases: [], backends: [], passed: false, releaseAccepted: false };
mkdirSync(output);
const state = page => page.evaluate(() => window.__lhRust.diagnostics);
const idle = page => page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
async function action(page, type, value) {
  await page.evaluate(({ type, value }) => window.__lhRust.action({ type, value }), { type, value });
  await idle(page);
}
async function advance(page, ticks) {
  const reply = await page.evaluate(t => window.__lhRust.advance(t), ticks);
  await idle(page);
  assert.equal(reply.failure, null, 'Physical integrity: ' + JSON.stringify(reply.failure));
  return reply;
}
const browser = await chromium.launch(browserLaunchOptions(true));
report.browser = browser.version();
let currentPage;
try {
  for (const backend of ['webgl2', 'webgpu']) {
    const context = await browser.newContext({ viewport: { width: 1280, height: 900 }, hasTouch: true });
    const page = await context.newPage(); currentPage = page;
    const details = { backend, errors: [], checks: [] }; report.backends.push(details);
    page.on('pageerror', error => details.errors.push(String(error)));
    page.on('console', message => { if (message.type() === 'error') details.errors.push(message.text()); });
    const url = new URL(origin); url.searchParams.set('renderer', backend);
    await page.goto(url.href);
    await page.waitForFunction(() => window.__lhRust?.diagnostics.ready, {}, { timeout: 45000 });
    await page.evaluate(() => window.__lhRust.setManual(true)); await idle(page);
    details.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
      const response = await fetch(new URL(name, document.baseURI));
      if (!response.ok) throw new Error('Cannot load ' + name);
      const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
      return [name, [...new Uint8Array(digest)].map(v => v.toString(16).padStart(2, '0')).join('')];
    }))), Object.keys(assets));
    assert.deepEqual(details.loadedAssets, assets);
    assert.equal((await state(page)).backend, backend === 'webgl2' ? 'WebGL2' : 'WebGPU');
    const headingValues = await page.locator('#heading option').evaluateAll(options => options.map(option => option.value));
    for (const [index, heading] of [0, Math.PI / 3, -Math.PI / 4].entries()) {
      const trial = { backend, heading, passed: false, trace: [] }; report.cases.push(trial);
      assert.ok(Math.abs(Number(headingValues[index]) - heading) < 1e-9);
      await page.locator('#heading').selectOption(headingValues[index]); await idle(page);
      assert.equal(await page.locator('#heading').inputValue(), headingValues[index]);
      if ((await state(page)).confirmedPaused) await action(page, 'pause');
      for (let tick = 0; tick < 120; tick += 4) await advance(page, 4);
      const ready = await state(page);
      assert.equal(ready.mode, 'protocol');
      assert.equal(ready.graphics.renderedStrikerPieces, 4);
      assert.equal(ready.snapshot.striker.available, true);
      trial.initial = ready.snapshot;
      await page.locator('#pause').click(); await idle(page);
      assert.equal(await page.locator('#strike').isDisabled(), true);
      await page.locator('body').click({ position: { x: 10, y: 10 } }); await page.keyboard.press('p'); await idle(page);
      assert.equal((await state(page)).snapshot.striker.phase, 'idle');
      await action(page, 'pause');
      if (index === 1) { await page.locator('body').click({ position: { x: 10, y: 10 } }); await page.keyboard.press('p'); await idle(page); }
      else { await page.locator('#strike').click(); await idle(page); }
      const positioning = await state(page);
      assert.equal(positioning.snapshot.striker.phase, 'positioning');
      assert.equal(positioning.strikes, 0, 'A request is not a collision');
      assert.equal(await page.locator('#strike').isDisabled(), true);
      await action(page, 'strike'); // Busy attempts are dropped, never deferred.
      await action(page, 'pause');
      const paused = await state(page);
      assert.equal((await advance(page, 4)).completed_ticks, 0);
      assert.deepEqual((await state(page)).snapshot.striker, paused.snapshot.striker);
      assert.deepEqual((await state(page)).snapshot.segments, paused.snapshot.segments);
      await action(page, 'pause');
      let minimumPelvisY = Infinity;
      let photographed = false;
      for (let tick = 0; tick < 180; tick += 4) {
        const reply = await advance(page, 4); assert.equal(reply.completed_ticks, 4);
        const value = await state(page); trial.trace.push(value.snapshot);
        minimumPelvisY = Math.min(minimumPelvisY, value.snapshot.segments[0].position.y);
        assert.equal(value.snapshot.segments.length, 25);
        if (index === 0 && !photographed && value.strikes === 1) {
          photographed = true;
          await page.screenshot({ path: join(output, backend + '-impact.png'), fullPage: true });
        }
      }
      const final = await state(page);
      trial.final = final.snapshot; trial.minimumPelvisY = minimumPelvisY;
      assert.equal(final.strikes, 1);
      assert.ok(final.snapshot.striker.lastImpactImpulseNs > 0.25);
      assert.ok(minimumPelvisY < 0.56);
      assert.equal(final.snapshot.striker.available, true);
      assert.equal(final.workerInstances, 1);
      assert.equal(final.snapshot.counters.falls, 1);
      assert.deepEqual(final.snapshot.segments.map(p => p.id), ready.snapshot.segments.map(p => p.id));
      trial.passed = true;
      writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2));
    }
    details.checks.push('Button/keyboard each produce one measured contact; paused and busy input never queues; apparatus and body freeze together');
    // A new attempt on the same fallen body exercises retargeting and ownership,
    // without pretending that an automatic recovery has occurred.
    const previous = await state(page);
    await page.locator('#strike').click(); await idle(page);
    assert.equal((await state(page)).snapshot.striker.phase, 'positioning');
    for (let tick = 0; tick < 120; tick += 4) await advance(page, 4);
    const repeated = await state(page);
    details.repeatedAttempt = repeated.snapshot;
    assert.equal(repeated.strikes, 2);
    assert.equal(repeated.snapshot.striker.available, true);
    assert.equal(repeated.generation, previous.generation);
    details.checks.push('A second attempt reuses the same physical trial and records a second contact');
    await action(page, 'reset'); await action(page, 'pause');
    await page.setViewportSize({ width: 390, height: 844 });
    await page.screenshot({ path: join(output, backend + '-mobile.png'), fullPage: true });
    await action(page, 'pause'); await page.locator('#strike').tap(); await idle(page);
    assert.equal((await state(page)).snapshot.striker.phase, 'positioning');
    details.checks.push('Mobile touch control starts a fresh strike');
    await action(page, 'mode', 'playground');
    assert.equal((await state(page)).graphics.renderedStrikerPieces, 0);
    assert.equal(await page.locator('#strike').count(), 0);
    await context.close();
  }
  report.sourceAfter = sourceHashes(); assert.deepEqual(report.sourceAfter, report.sourceBefore);
  report.passed = report.cases.length === 6 && report.cases.every(c => c.passed) && report.backends.every(b => !b.errors.length);
} catch (error) {
  report.error = String(error.stack ?? error);
  if (currentPage && !currentPage.isClosed()) {
    report.failureState = await state(currentPage);
    await currentPage.screenshot({ path: join(output, 'failure.png'), fullPage: true });
  }
} finally {
  await browser.close(); report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2));
  console.log(JSON.stringify({ cases: report.cases.length, passed: report.passed, error: report.error, releaseAccepted: false }));
  if (!report.passed) process.exitCode = 1;
}
