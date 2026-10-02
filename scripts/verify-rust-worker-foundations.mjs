// Actual cold-worker A/B fixtures, with the candidate profile passed to every rig.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3], artifact = resolve(process.argv[4] ?? '');
assert.ok(origin && process.argv[4] && process.argv[5], 'Supply origin, worker artifact and frozen profile');
const profile = JSON.parse(readFileSync(process.argv[5], 'utf8'));
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function files(dir) {
  return readdirSync(dir, { withFileTypes: true }).flatMap(entry => {
    if (['target', 'dist', '.git'].includes(entry.name)) return [];
    assert.ok(!entry.isSymbolicLink(), 'Refuse source symlink');
    const path = join(dir, entry.name);
    return entry.isDirectory() ? files(path) : [path.replaceAll('\\', '/')];
  });
}
const fingerprint = () => Object.fromEntries([...files('rust'), 'scripts/verify-rust-worker-foundations.mjs',
  'scripts/build-rust-worker.mjs'].sort().map(path => [path, hash(readFileSync(path))]));
const assets = Object.fromEntries(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(name => [name, hash(readFileSync(join(artifact, name)))]));
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), scope: 'Actual WASM primitive/canonical contact, isolated joints, individual feet and loaded-chain foundations. No C/D, angular tangent, terrain, input, timing or release admission.',
  profile, sourceBefore: fingerprint(), assets, stages: [], passed: false, releaseAccepted: false };
const browser = await chromium.launch(browserLaunchOptions());
report.browser = browser.version();
try {
  const page = await browser.newPage(); await page.goto(origin);
  report.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const response = await fetch(new URL(name, location.href));
    if (!response.ok) throw new Error('Asset fetch failed');
    const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
    return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(assets));
  assert.deepEqual(report.loadedAssets, assets, 'Fetched artifacts differ');
  const result = await page.evaluate(async profile => {
    const worker = new Worker(new URL('sim-worker.js', location.href), { type: 'module' });
    let pending, requestId = 0;
    const ready = new Promise((resolve, reject) => {
      worker.onmessage = event => event.data.type === 'ready' ? resolve(event.data) : pending?.(event.data);
      worker.onerror = event => reject(new Error(event.message));
    });
    const send = message => new Promise((resolve, reject) => {
      const id = ++requestId, timeout = setTimeout(() => reject(new Error('Foundation request timed out')), 60000);
      pending = data => { clearTimeout(timeout); pending = null; data.requestId === id ? resolve(data) : reject(new Error('Request identity differs')); };
      worker.postMessage({ ...message, requestId: id });
    });
    const stages = [];
    function sameProfile(actual, expected) {
      if (typeof expected === 'number') return typeof actual === 'number' && Math.fround(actual) === Math.fround(expected);
      if (expected && typeof expected === 'object') return Object.entries(expected).every(([key, value]) => sameProfile(actual?.[key], value));
      return actual === expected;
    }
    try {
      await ready;
      const invalid = await send({ type: 'calibrateFoundation', kind: 'feet', profile: { ...profile, contact_fricton_model: 'simplified' } });
      if (invalid.type !== 'error') throw new Error('Profile typo was accepted');
      for (const [kind, count] of [['primitive', 12], ['canonical', 144], ['joints', 216], ['feet', 12], ['chains', 3]]) {
        const response = await send({ type: 'calibrateFoundation', kind, profile });
        if (response.type !== 'foundation') throw new Error('Foundation rejected: ' + response.message);
        const r = response.report;
        if (r.kind !== kind || !sameProfile(r.profile, profile) || r.cases.length !== count || r.releaseAccepted !== false) throw new Error('Fixture/profile identity differs');
        if (r.cases.some(row => typeof row.passed !== 'boolean')) throw new Error('Missing physical pass flag');
        stages.push({ kind, expectedCases: count, frozenProfileApplied: true, cases: r.cases, passed: r.cases.every(row => row.passed) });
      }
      const initial = await send({ type: 'initialize', mode: 'protocol', heading: 0, profiling: false, profile });
      if (initial.type !== 'snapshot' || initial.reply.snapshot.stamp.tick !== 0) throw new Error('Cold fixtures contaminated initialization');
      const bits = [...new Uint32Array(initial.poseBuffer)];
      worker.postMessage({ type: 'recycle', bufferId: initial.bufferId, poseBuffer: initial.poseBuffer }, [initial.poseBuffer]);
      const refused = await send({ type: 'calibrateFoundation', kind: 'feet', profile });
      if (refused.type !== 'error' || refused.message !== 'Error: Calibration requires an uninitialized worker') throw new Error('Live calibration accepted');
      const after = await send({ type: 'advance', ticks: 0 });
      if (after.type !== 'snapshot' || after.reply.snapshot.stamp.tick !== 0 || [...new Uint32Array(after.poseBuffer)].some((v, i) => v !== bits[i])) throw new Error('Rejected fixture changed physics');
      worker.postMessage({ type: 'recycle', bufferId: after.bufferId, poseBuffer: after.poseBuffer }, [after.poseBuffer]);
      if ((await send({ type: 'shutdown' })).type !== 'closed') throw new Error('Shutdown failed');
      return { stages, profileTypoRejected: true, fixturesIsolatedFromLiveRuntime: true };
    } finally { worker.terminate(); }
  }, profile);
  Object.assign(report, result);
  report.sourceUnchanged = JSON.stringify(fingerprint()) === JSON.stringify(report.sourceBefore);
  report.artifactsUnchanged = Object.entries(assets).every(([name, digest]) => hash(readFileSync(join(artifact, name))) === digest);
  report.passed = report.stages.length === 5 && report.stages.every(stage => stage.passed) && report.sourceUnchanged && report.artifactsUnchanged;
} catch (error) { report.failure = String(error); }
finally {
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, stages: report.stages.map(r => ({ kind: r.kind, cases: r.cases.length, failed: r.cases.filter(x => !x.passed).length })),
  failure: report.failure, sourceUnchanged: report.sourceUnchanged, artifactsUnchanged: report.artifactsUnchanged }));
if (!report.passed) process.exitCode = 1;
