// Actual-worker linear contact diagnostics. Failures remain failures.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { join, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3] ?? 'http://127.0.0.1:5200/';
const artifact = resolve(process.argv[4] ?? 'rust/dist/worker-tangent-reporting');
const excitation = process.argv[5] ?? 'stress';
assert.ok(['stress', 'bounded'].includes(excitation), 'Unknown fixture excitation');
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const paths = [];
function files(dir) {
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    if (['target', 'dist', '.git'].includes(entry.name)) continue;
    const path = join(dir, entry.name);
    assert.ok(!entry.isSymbolicLink(), 'Refuse source symlink');
    if (entry.isDirectory()) files(path); else paths.push(path.replaceAll('\\', '/'));
  }
}
files('rust'); paths.push('scripts/verify-rust-tangent-calibration.mjs', 'scripts/build-rust-worker.mjs');
const fingerprint = () => Object.fromEntries(paths.sort().map(path => [path, hash(readFileSync(path))]));
const assets = Object.fromEntries(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(name => [name, hash(readFileSync(join(artifact, name)))]));
mkdirSync(output, { recursive: true });
const report = { scope: 'Actual isolated-box linear tangential/normal impulse and friction diagnostics; no character gate, angular friction, performance or release admission.',
  excitation, started: new Date().toISOString(), sourceBefore: fingerprint(), assets, cases: [], passed: false, releaseAccepted: false };
const browser = await chromium.launch(browserLaunchOptions()); report.browser = browser.version();
try {
  const page = await browser.newPage(); await page.goto(origin);
  report.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const response = await fetch(new URL(name, location.href));
    if (!response.ok) throw new Error('Asset fetch failed');
    const bytes = await response.arrayBuffer(), digest = await crypto.subtle.digest('SHA-256', bytes);
    return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(assets));
  assert.deepEqual(report.loadedAssets, assets);
  const result = await page.evaluate(async excitation => {
    const worker = new Worker(new URL('sim-worker.js', location.href), { type: 'module' });
    let id = 0, pending, boot;
    const ready = new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error('Worker boot timeout')), 15000);
      boot = () => { clearTimeout(timer); resolve(); };
    });
    worker.onmessage = ({ data }) => { if (data.type === 'ready') boot(); else pending?.(data); };
    const send = message => new Promise((resolve, reject) => {
      const requestId = ++id, timer = setTimeout(() => reject(new Error('Calibration request timeout')), 15000);
      pending = data => { clearTimeout(timer); pending = null; if (data.requestId !== requestId) reject(new Error('Request mismatch')); else resolve(data); };
      worker.postMessage({ ...message, requestId });
    });
    const cases = [];
    try {
      await ready;
      const invalid = await send({ type: 'calibrateTangent', model: 'simplified', heading: 0, warmstart: 0.5, scenario: 'static', friction: 5 });
      if (invalid.type !== 'error') throw new Error('Invalid fixture accepted');
      for (const model of ['simplified', 'coulomb']) {
        for (const friction of [0.5, 4]) {
          for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
            for (const warmstart of [0, 0.5, 1]) {
              for (const scenario of ['static', 'sliding', 'changing', 'unloading', 'contact-loss', 'frictionless']) {
                const response = await send({ type: 'calibrateTangent', model, friction, heading, warmstart, scenario, excitation });
                if (response.type !== 'calibration') throw new Error('Calibration rejected: ' + response.message);
                const r = response.report;
                if (r.model !== model || r.requestedFriction !== friction || Math.fround(r.heading) !== Math.fround(heading)
                  || r.warmstart !== warmstart || r.scenario !== scenario || r.excitation !== excitation || r.settlingSubsteps !== 480 || r.observedSubsteps !== 480) throw new Error('Fixture identity/clock differs');
                for (const field of ['maxTangentMomentumErrorNs', 'maxNormalMomentumErrorNs', 'maxFrictionBoundExcessNs', 'maxHorizontalSpeedMps', 'meanTangentForceN']) {
                  if (typeof r[field] !== 'number' || !Number.isFinite(r[field]) || r[field] < 0) throw new Error('Invalid observation: ' + field);
                }
                const behavior = scenario === 'static' || scenario === 'changing'
                  ? Math.fround(r.maxHorizontalSpeedMps) <= Math.fround(0.001) && Math.fround(Math.abs(Math.fround(r.meanTangentForceN) - 20)) <= Math.fround(0.2)
                  : scenario === 'sliding'
                    ? r.maxHorizontalSpeedMps > 1 && Math.abs(r.meanTangentForceN - friction * 10 * 9.81) <= 0.5
                    : r.zeroSignalWhenContactAbsent;
                const passed = Math.fround(r.maxTangentMomentumErrorNs) <= Math.fround(0.002) && Math.fround(r.maxNormalMomentumErrorNs) <= Math.fround(0.002)
                  && Math.fround(r.maxFrictionBoundExcessNs) <= Math.fround(0.002) && r.zeroSignalWhenContactAbsent && behavior
                  && (excitation !== 'bounded' || Math.fround(r.maxHorizontalSpeedMps) <= Math.fround(4.1));
                if (r.passed !== passed || r.releaseAccepted !== false) throw new Error('Rust pass flag disagrees with observed limits');
                cases.push(r);
              }
            }
          }
        }
      }
      const initial = await send({ type: 'initialize', mode: 'protocol', heading: 0, profiling: false });
      if (initial.type !== 'snapshot' || initial.reply.snapshot.stamp.tick !== 0) throw new Error('Calibration contaminated runtime initialization');
      const firstBits = [...new Uint32Array(initial.poseBuffer)];
      worker.postMessage({ type: 'recycle', bufferId: initial.bufferId, poseBuffer: initial.poseBuffer }, [initial.poseBuffer]);
      const refused = await send({ type: 'calibrateTangent', model: 'simplified', heading: 0, warmstart: 0.5, scenario: 'static', friction: 4 });
      if (refused.type !== 'error' || refused.message !== 'Error: Calibration requires an uninitialized worker') throw new Error('Live calibration not rejected');
      const after = await send({ type: 'advance', ticks: 0 });
      if (after.type !== 'snapshot' || after.reply.snapshot.stamp.tick !== 0 || [...new Uint32Array(after.poseBuffer)].some((value, i) => value !== firstBits[i])) throw new Error('Rejected calibration changed live physics');
      worker.postMessage({ type: 'recycle', bufferId: after.bufferId, poseBuffer: after.poseBuffer }, [after.poseBuffer]);
      const closed = await send({ type: 'shutdown' });
      if (closed.type !== 'closed') throw new Error('Shutdown failed');
      return { cases, invalidFixtureRejected: true, calibrationIsolatedFromLiveRuntime: true };
    } finally { worker.terminate(); }
  }, excitation);
  Object.assign(report, result);
  report.sourceUnchanged = JSON.stringify(fingerprint()) === JSON.stringify(report.sourceBefore);
  report.artifactsUnchanged = Object.entries(assets).every(([name, digest]) => hash(readFileSync(join(artifact, name))) === digest);
  report.passed = report.cases.length === 216 && report.cases.every(row => row.passed) && report.sourceUnchanged && report.artifactsUnchanged;
} catch (error) { report.failure = String(error); }
finally {
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, cases: report.cases.length, failed: report.cases.filter(row => !row.passed).length,
  maxTangentMomentumErrorNs: Math.max(0, ...report.cases.map(row => row.maxTangentMomentumErrorNs)), failure: report.failure,
  sourceUnchanged: report.sourceUnchanged, artifactsUnchanged: report.artifactsUnchanged }));
if (!report.passed) process.exitCode = 1;
