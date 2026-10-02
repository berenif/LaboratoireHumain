import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function firstDifference(actual, expected, path = '$') {
  if (Object.is(actual, expected)) return null;
  if (!actual || !expected || typeof actual !== 'object' || typeof expected !== 'object') {
    return { path, actual, expected };
  }
  const actualKeys = Object.keys(actual), expectedKeys = Object.keys(expected);
  if (actualKeys.length !== expectedKeys.length || actualKeys.some(key => !Object.hasOwn(expected, key))) {
    return { path, actualKeys, expectedKeys };
  }
  for (const key of actualKeys) {
    const difference = firstDifference(actual[key], expected[key], `${path}.${key}`);
    if (difference) return difference;
  }
  return null;
}
function assertExact(actual, expected, label) {
  const difference = firstDifference(actual, expected);
  if (difference) throw new Error(`${label}: ${JSON.stringify(difference)}`);
}
const artifact = resolve(process.argv[4] ?? 'rust/dist/worker');
const localAssets = Object.fromEntries(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(name => [name, hash(readFileSync(join(artifact, name)))]));
const baselineOrigin = process.argv[5];
const baselineArtifact = resolve(process.argv[6] ?? 'rust/dist/worker');
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), localAssets, passed: false,
  scope: 'Paired actual-worker profiling/plain physics replies and packed poses. No full physics or performance admission.' };
const browser = await chromium.launch(browserLaunchOptions()); report.browser = browser.version();
try {
  const page = await browser.newPage();
  await page.goto(process.argv[3] ?? 'http://127.0.0.1:5184/');
  report.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const response = await fetch(new URL(name, location.href));
    if (!response.ok) throw new Error('Asset fetch failed');
    const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
    return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(localAssets));
  assert.deepEqual(report.loadedAssets, localAssets);
  let baselinePage = null;
  if (baselineOrigin) {
    baselinePage = await browser.newPage();
    await baselinePage.goto(baselineOrigin);
    report.baselineAssets = Object.fromEntries(Object.keys(localAssets).map(name => [name, hash(readFileSync(join(baselineArtifact, name)))]));
    const fetched = await baselinePage.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
      const response = await fetch(new URL(name, location.href));
      if (!response.ok) throw new Error('Baseline asset fetch failed');
      const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
      return [name, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
    }))), Object.keys(localAssets));
    assert.deepEqual(fetched, report.baselineAssets);
  }
  report.cases = [];
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const cases = [];
    const trials = [{ page, profiling: false }, { page, profiling: true }];
    if (baselinePage) trials.push({ page: baselinePage, profiling: false });
    for (const { page: trialPage, profiling } of trials) {
      cases.push(await trialPage.evaluate(async ({ heading, profiling }) => {
        const worker = new Worker(new URL('sim-worker.js', location.href), { type: 'module' });
        let id = 0, ready, pending = null;
        const boot = new Promise((resolve, reject) => {
          const timer = setTimeout(() => reject(new Error('Worker boot timeout')), 10000);
          ready = message => { clearTimeout(timer); resolve(message); };
        });
        worker.onmessage = ({ data }) => {
          if (data.type === 'ready') ready(data);
          else { const resolve = pending; pending = null; resolve?.(data); }
        };
        const replies = [], packs = [];
        const send = async message => {
          const result = await new Promise((resolve, reject) => {
            const timer = setTimeout(() => reject(new Error('Worker request timeout')), 10000);
            pending = data => { clearTimeout(timer); resolve(data); };
            worker.postMessage({ ...message, requestId: ++id });
          });
          if (result.type !== 'snapshot' || result.reply.failure) throw new Error('Unexpected worker result: ' + JSON.stringify(result));
          if (message.type === 'advance' && result.reply.completed_ticks) {
            if (profiling && (!result.timing?.clockValid || result.timing.ticks.length !== result.reply.completed_ticks)) throw new Error('Invalid profiling record');
            if (!profiling && result.timing !== null) throw new Error('Plain worker unexpectedly profiled');
          }
          packs.push([...new Uint32Array(result.poseBuffer)]);
          worker.postMessage({ type: 'recycle', bufferId: result.bufferId, poseBuffer: result.poseBuffer }, [result.poseBuffer]);
          result.reply.snapshot.diagnostics.controller_ms = null;
          result.reply.snapshot.diagnostics.update_ms = null;
          replies.push(result.reply); return result.reply;
        };
        try {
          await boot;
          let reply = await send({ type: 'initialize', mode: 'playground', heading, profiling });
          let sequence = 0;
          const commands = actions => send({ type: 'commands', commands: actions.map(action => ({
            stamp: { schema: 4, generation: reply.snapshot.stamp.generation, tick: reply.snapshot.stamp.tick, sequence: ++sequence }, action,
          })) });
          for (let i = 0; i < 10; i++) reply = await send({ type: 'advance', ticks: 4 });
          const hand = reply.snapshot.segments.findIndex(pose => pose.id === 'rightHand');
          const target = { ...reply.snapshot.segments[hand].position, x: reply.snapshot.segments[hand].position.x + 0.04 };
          reply = await commands([{ GrabBegin: { press: 41, segment: hand, local_anchor: { x: 0, y: 0, z: 0 }, target } }]);
          for (let i = 0; i < 5; i++) reply = await send({ type: 'advance', ticks: 4 });
          reply = await commands(['Pause']); reply = await send({ type: 'advance', ticks: 4 });
          reply = await commands(['Resume']);
          for (let i = 0; i < 5; i++) reply = await send({ type: 'advance', ticks: 4 });
          reply = await commands(['Reset']);
          return { replies, packs, completedTicks: 80, final: reply.snapshot.stamp };
        } finally { worker.terminate(); }
      }, { heading, profiling }));
    }
    assertExact(cases[1], cases[0], 'Profiling changes physical output');
    if (baselinePage) assertExact(cases[0], cases[2], 'Candidate differs from baseline');
    report.cases.push({ heading, replies: cases[0].replies.length, completedTicks: cases[0].completedTicks,
      exactPhysicsReplies: true, exactPackedPoseBits: true, exactBaselinePhysics: baselinePage ? true : null });
  }
  report.artifactsUnchanged = Object.entries(localAssets).every(([name, digest]) => hash(readFileSync(join(artifact, name))) === digest);
  if (baselinePage) report.artifactsUnchanged &&= Object.entries(report.baselineAssets).every(([name, digest]) => hash(readFileSync(join(baselineArtifact, name))) === digest);
  report.passed = report.artifactsUnchanged;
} catch (error) { report.failure = String(error); }
finally {
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify(report));
if (!report.passed) process.exitCode = 1;
