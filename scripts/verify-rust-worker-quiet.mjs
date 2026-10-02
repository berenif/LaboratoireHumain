// Full fixed-clock worker standing observation. This does not admit other
// physics gates, rendering, public environments, timing, or a release.
import assert from 'node:assert/strict';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence directory');
const origin = process.argv[3] ?? 'http://127.0.0.1:5191/';
const artifact = resolve(process.argv[4] ?? 'rust/dist/worker-quiet');
// '-' reserves the profile slot while testing the artifact's ordinary default.
const profile = process.argv[5] && process.argv[5] !== '-' ? JSON.parse(readFileSync(resolve(process.argv[5]))) : undefined;
const fixtureArgs = process.argv.slice(6).filter(arg => arg !== '--drift');
const modes = fixtureArgs[0] ? [fixtureArgs[0]] : ['protocol', 'playground'];
const headings = fixtureArgs[1] ? [Number(fixtureArgs[1])] : [0, Math.PI / 3, -Math.PI / 4];
const qualification = fixtureArgs[2] === 'drift' || process.argv.includes('--drift') ? 'quiet-drift' : 'quiet';
assert.ok(modes.every(mode => ['protocol', 'playground'].includes(mode)) && headings.every(Number.isFinite), 'Invalid fixture selection');
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const sourceFiles = [];
function sources(dir) {
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const file = join(dir, entry.name);
    if (entry.isDirectory() && !['target', 'dist'].includes(entry.name)) sources(file);
    else if (entry.isFile() && (/\.(rs|js|toml|lock|json)$/.test(file))) sourceFiles.push(file);
  }
}
sources('rust');
sourceFiles.push('scripts/verify-rust-worker-quiet.mjs', 'scripts/build-rust-worker.mjs');
const fingerprint = () => Object.fromEntries(sourceFiles.sort().map(file => [file.replaceAll('\\', '/'), hash(readFileSync(file))]));
const localAssets = Object.fromEntries(['sim-worker.js', 'lh_worker.js', 'lh_worker_bg.wasm'].map(file => [file, hash(readFileSync(join(artifact, file)))]));
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), scope: 'Actual dedicated-worker 2 s settle + 30 s quiet standing at 240 Hz for the explicitly selected mode tags and headings. Flat floor only; no other gate, renderer, environment, performance or release admission.',
  sourceBefore: fingerprint(), localAssets, profile: profile ?? null, selectedModes: modes, selectedHeadings: headings, qualification,
  cases: [], passed: false, releaseAccepted: false };
const browser = await chromium.launch(browserLaunchOptions());
report.browser = browser.version();
try {
  const page = await browser.newPage();
  await page.goto(origin);
  report.loadedAssets = await page.evaluate(async files => Object.fromEntries(await Promise.all(files.map(async file => {
    const response = await fetch(new URL(file, location.href));
    if (!response.ok) throw new Error('Asset fetch failed');
    const digest = await crypto.subtle.digest('SHA-256', await response.arrayBuffer());
    return [file, [...new Uint8Array(digest)].map(value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(localAssets));
  assert.deepEqual(report.loadedAssets, localAssets);
  for (const mode of modes) {
    for (const heading of headings) {
      const trial = await page.evaluate(async ({ mode, heading, profile, qualification }) => {
        const worker = new Worker(new URL('sim-worker.js', location.href), { type: 'module' });
        let request = 0, pending = null, ready;
        const boot = new Promise((resolve, reject) => {
          const timer = setTimeout(() => reject(new Error('Worker boot timeout')), 15000);
          ready = data => { clearTimeout(timer); resolve(data); };
        });
        worker.onmessage = ({ data }) => {
          if (data.type === 'ready') ready(data);
          else pending?.(data);
        };
        const send = async message => {
          const id = ++request;
          const result = await new Promise((resolve, reject) => {
            const timer = setTimeout(() => reject(new Error('Worker request timeout')), 15000);
            pending = data => { clearTimeout(timer); pending = null; resolve(data); };
            worker.postMessage({ ...message, requestId: id });
          });
          if (result.requestId !== id) throw new Error('Request identity mismatch');
          return result;
        };
        const poseBits = result => {
          if (!(result.poseBuffer instanceof ArrayBuffer) || result.poseBuffer.byteLength !== 1300) throw new Error('Invalid pose buffer');
          const expected = [];
          for (const segment of result.reply.snapshot.segments) {
            for (const key of ['position', 'rotation', 'linearVelocity', 'angularVelocity']) {
              for (const axis of key === 'rotation' ? ['x', 'y', 'z', 'w'] : ['x', 'y', 'z']) {
                const value = segment[key][axis];
                if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error('Non-finite physical observation');
                expected.push(value);
              }
            }
          }
          const bits = [...new Uint32Array(result.poseBuffer)], reference = new Uint32Array(new Float32Array(expected).buffer);
          if (bits.length !== reference.length || bits.some((value, i) => value !== reference[i])) throw new Error('Packed pose differs from physical reply');
          worker.postMessage({ type: 'recycle', bufferId: result.bufferId, poseBuffer: result.poseBuffer }, [result.poseBuffer]);
          return bits;
        };
        const result = { mode, heading, allPackedPosesExact: true, advances: 0, completedTicks: 0 };
        try {
          await boot;
          // Failed qualification must leave initialization atomic.
          const invalid = await send({ type: 'initialize', mode, heading, profile, floorEnabled: false, qualification, profiling: false });
          if (invalid.type !== 'error') throw new Error('Floorless qualification accepted');
          const initial = await send({ type: 'initialize', mode, heading, profile, qualification, profiling: false });
          if (initial.type !== 'snapshot' || !initial.quietProgress || initial.quietProgress.completedTicks !== 0) throw new Error('Invalid qualification initialization');
          poseBits(initial);
          result.floorlessInitializationRejectedAtomically = true;
          let latest = initial, latestBits;
          for (let i = 0; i < 480; i++) {
            latest = await send({ type: 'advance', ticks: 4 });
            if (latest.type !== 'snapshot' || !latest.quietProgress) throw new Error('Missing quiet observation');
            const reply = latest.reply;
            if (reply.completed_ticks < 0 || reply.completed_ticks > 4 || reply.snapshot.stamp.tick !== result.completedTicks + reply.completed_ticks) throw new Error('Fixed-clock boundary mismatch');
            result.completedTicks += reply.completed_ticks; result.advances++;
            latestBits = poseBits(latest);
            if (latest.quietProgress.done) break;
          }
          const measurement = await send({ type: 'quietReport' });
          if (measurement.type !== 'qualification' || !measurement.report) throw new Error('Missing complete measurement');
          result.measurement = measurement.report;
          if (profile) {
            const sameTypedValue = (actual, expected) => {
              if (typeof expected === 'number') return typeof actual === 'number' && Math.fround(actual) === Math.fround(expected);
              if (expected && typeof expected === 'object') return Object.entries(expected).every(([key, value]) => Object.hasOwn(actual, key) && sameTypedValue(actual[key], value));
              return actual === expected;
            };
            if (!sameTypedValue(result.measurement.profile, profile)) throw new Error('Actual profile differs from frozen input');
            result.frozenProfileApplied = true;
          }
          if (qualification === 'quiet-drift') {
            const model = profile?.contact_friction_model === 'coulomb' ? 'Coulomb' : 'Simplified';
            if (result.measurement.driftTrace?.samples[0]?.tangentReporting?.solverModel !== model) throw new Error('Actual friction solver differs from requested model');
            result.actualFrictionSolverChecked = model;
          }
          const frozen = await send({ type: 'advance', ticks: 4 });
          const frozenBits = poseBits(frozen);
          if (frozen.reply.completed_ticks !== 0 || frozenBits.some((value, i) => value !== latestBits[i])) throw new Error('Terminal trial continues advancing');
          result.terminalTrialFrozen = true;
          const command = await send({ type: 'commands', commands: [{ stamp: { schema: 4, generation: latest.reply.snapshot.stamp.generation, tick: latest.reply.snapshot.stamp.tick, sequence: 1 }, action: 'Reset' }] });
          if (command.type !== 'error' || command.message !== 'Unsupported') throw new Error('Valid trial-contaminating command did not receive Unsupported');
          result.contaminatingCommandRejected = true;
          result.contaminatingCommandReject = command.message;
          const after = await send({ type: 'quietReport' });
          if (JSON.stringify(after.report) !== JSON.stringify(result.measurement)) throw new Error('Rejected command changed measurement');
          const closed = await send({ type: 'shutdown' });
          if (closed.type !== 'closed') throw new Error('Worker did not shut down');
          const progress = result.measurement.progress;
          result.passed = progress.passed === true && progress.completedTicks === 1920 && progress.settlingSubsteps === 480 && progress.quietSubsteps === 7200 && progress.firstFailure === null;
          return result;
        } finally { worker.terminate(); }
      }, { mode, heading, profile, qualification });
      report.cases.push(trial);
      writeFileSync(join(output, `${mode}-${heading === 0 ? '0' : heading > 0 ? 'plus60' : 'minus45'}.json`), JSON.stringify(trial, null, 2) + '\n');
      console.log(JSON.stringify({ mode, heading, passed: trial.passed, completedTicks: trial.completedTicks, quietSubsteps: trial.measurement.progress.quietSubsteps, firstFailure: trial.measurement.progress.firstFailure }));
    }
  }
  report.sourceUnchanged = JSON.stringify(fingerprint()) === JSON.stringify(report.sourceBefore);
  report.artifactsUnchanged = Object.entries(localAssets).every(([file, digest]) => hash(readFileSync(join(artifact, file))) === digest);
  report.passed = report.cases.length === modes.length * headings.length && report.cases.every(trial => trial.passed) && report.sourceUnchanged && report.artifactsUnchanged;
} catch (error) { report.failure = String(error); }
finally {
  report.finished = new Date().toISOString();
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, cases: report.cases.length, failure: report.failure, sourceUnchanged: report.sourceUnchanged, artifactsUnchanged: report.artifactsUnchanged }));
if (!report.passed) process.exitCode = 1;
