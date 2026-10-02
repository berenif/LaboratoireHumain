import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';

const output = resolve(process.argv[2] ?? 'evidence/rust-terrain-browser');
const origin = process.argv[3] ?? 'http://127.0.0.1:5186/';
const artifact = resolve(process.argv[4] ?? 'rust/dist/terrain');
assert.ok(!existsSync(output), 'Use a fresh evidence directory');
const hash = value => createHash('sha256').update(value).digest('hex');
function files(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap(entry => {
    if (['target', 'dist', '.git'].includes(entry.name)) return [];
    assert.ok(!entry.isSymbolicLink(), 'Refuse symlink: ' + join(directory, entry.name));
    const file = join(directory, entry.name);
    return entry.isDirectory() ? files(file) : [file];
  });
}
const sources = [...files('rust/crates'), ...files('rust/vendor'), ...files('rust/browser'),
  'rust/Cargo.toml', 'rust/Cargo.lock', 'scripts/verify-rust-terrain.mjs'].sort();
const sourceHashes = () => Object.fromEntries(sources.map(file => [file.replaceAll('\\', '/'), hash(readFileSync(file))]));
const assets = Object.fromEntries(files(artifact).map(file => [relative(artifact, file).replaceAll('\\', '/'), hash(readFileSync(file))]));
const report = { scope: 'Actual WASM scene commands, 21 terrain setups per backend, two seconds of integrated anatomy, moving deck pause/resume. Falls are recorded; this is not balance, stepping, recovery or performance admission.',
  started: new Date().toISOString(), sourceBefore: sourceHashes(), assets, cases: [], backends: [], passed: false, releaseAccepted: false };
mkdirSync(output);
const state = page => page.evaluate(() => window.__lhRust.diagnostics);
const idle = page => page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding && !window.__lhRust.diagnostics.queueDepth);
async function action(page, type, value) {
  await page.evaluate(({ type, value }) => window.__lhRust.action({ type, value }), { type, value });
  await idle(page);
}
async function select(page, id, value) {
  const before = await state(page);
  if (before[id] === value) return;
  await page.locator('#' + id).selectOption(value);
  await page.waitForFunction(({ generation, id, value }) => {
    const s = window.__lhRust.diagnostics;
    return s.generation === generation + 1 && s[id] === value;
  }, { generation: before.generation, id, value });
  await idle(page);
}
async function advance(page, ticks) {
  const reply = await page.evaluate(ticks => window.__lhRust.advance(ticks), ticks);
  await idle(page);
  assert.equal(reply.failure, null, 'Physical integrity: ' + reply.failure);
  return reply;
}
const browser = await chromium.launch(browserLaunchOptions(true));
report.browser = browser.version();
try {
  for (const backend of ['webgl2', 'webgpu']) {
    const context = await browser.newContext({ viewport: { width: 1280, height: 900 } });
    const page = await context.newPage();
    const details = { backend, errors: [], checks: [] };
    report.backends.push(details);
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
    await page.getByRole('button', { name: 'Balance playground', exact: true }).click(); await idle(page);
    for (const difficulty of ['gentle', 'challenging', 'extreme']) {
      for (const station of ['flat', 'slope', 'rubble', 'beam', 'stones', 'wobble', 'hurdles']) {
        const trial = { backend, difficulty, station, passed: false };
        report.cases.push(trial);
        try {
          await action(page, 'reset');
          await select(page, 'difficulty', difficulty);
          await select(page, 'station', station);
          let initial = await state(page);
          if (initial.confirmedPaused) { await action(page, 'pause'); initial = await state(page); }
          assert.equal(initial.tick, 0);
          assert.deepEqual(initial.snapshot.environment.settings, { station, difficulty });
          assert.equal(initial.snapshot.environment.pieces.length, 26);
          assert.equal(initial.graphics.renderedEnvironmentPieces, 26);
          assert.equal(initial.snapshot.segments.length, 25);
          trial.initial = initial.snapshot;
          for (let ticks = 0; ticks < 120; ticks += 4) {
            const reply = await advance(page, 4);
            assert.equal(reply.completed_ticks, 4);
          }
          const final = await state(page);
          trial.final = final.snapshot;
          assert.equal(final.tick, 120);
          assert.equal(final.workerInstances, 1);
          assert.deepEqual(final.snapshot.segments.map(p => p.id), initial.snapshot.segments.map(p => p.id));
          if (difficulty === 'challenging' && ['slope', 'stones', 'wobble'].includes(station)) {
            await page.locator('#camera-reset').click();
            const frames = final.graphics.frames;
            await page.waitForFunction(f => window.__lhRust.diagnostics.graphics.frames > f + 2, frames);
            await page.screenshot({ path: join(output, `${backend}-${station}.png`), fullPage: true });
          }
          trial.passed = true;
        } catch (error) {
          trial.error = String(error.stack ?? error);
          trial.final = (await state(page)).snapshot;
        }
        writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2));
      }
    }
    await action(page, 'reset'); await select(page, 'station', 'wobble'); await select(page, 'difficulty', 'gentle');
    if ((await state(page)).confirmedPaused) await action(page, 'pause');
    for (let ticks = 0; ticks < 180; ticks += 4) await advance(page, Math.min(4, 180 - ticks));
    const before = await state(page);
    const moving = before.snapshot.environment.pieces.find(p => Math.hypot(p.angularVelocity.x, p.angularVelocity.y, p.angularVelocity.z) > 0.001);
    assert.ok(moving, 'An integrated deck must actually move after ramp-up');
    await action(page, 'pause');
    const paused = await state(page);
    assert.equal((await advance(page, 4)).completed_ticks, 0);
    const stopped = await state(page);
    assert.deepEqual(stopped.snapshot.segments, paused.snapshot.segments);
    assert.deepEqual(stopped.snapshot.environment, paused.snapshot.environment);
    assert.equal(stopped.tick, paused.tick);
    await action(page, 'pause'); await advance(page, 4);
    assert.notDeepEqual((await state(page)).snapshot.environment.pieces, paused.snapshot.environment.pieces);
    details.checks.push('Moving deck is integrated, freezes with anatomy on pause, and resumes');
    await action(page, 'pause'); await select(page, 'station', 'beam');
    assert.equal((await state(page)).confirmedPaused, true);
    assert.equal((await state(page)).tick, 0);
    details.checks.push('Station changes preserve pause and create a fresh generation at tick zero');
    await page.getByRole('button', { name: 'Impact protocol', exact: true }).click(); await idle(page);
    const protocol = await state(page);
    assert.equal(protocol.snapshot.environment, undefined);
    assert.equal(protocol.graphics.renderedEnvironmentPieces, 0);
    details.checks.push('Returning to protocol removes course geometry');
    await context.close();
  }
  report.sourceAfter = sourceHashes();
  assert.deepEqual(report.sourceAfter, report.sourceBefore, 'Source changed during qualification');
  report.passed = report.cases.length === 42 && report.cases.every(c => c.passed) && report.backends.every(b => !b.errors.length);
} catch (error) {
  report.error = String(error.stack ?? error);
} finally {
  await browser.close();
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2));
  console.log(JSON.stringify({ cases: report.cases.length, failures: report.cases.filter(c => !c.passed).map(c => ({ backend: c.backend, station: c.station, difficulty: c.difficulty, error: c.error })), error: report.error, passed: report.passed, releaseAccepted: false }));
  if (!report.passed) process.exitCode = 1;
}
