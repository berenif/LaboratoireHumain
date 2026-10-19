// Explicit fixed-fixture TEST footage. Run only after source/native qualification.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, join, relative, resolve } from 'node:path';

const artifact = resolve(process.argv[2]);
const output = resolve(process.argv[3]);
const kind = process.argv[4];
const mode = process.argv[5] ?? 'thirty-second-standing-observation';
assert.ok(['thirty-second-standing-observation', 'bounded-back-pass-plus-five-seconds'].includes(mode), 'Explicit reviewed observation mode required');
assert.ok(mode !== 'bounded-back-pass-plus-five-seconds' || kind === 'back', 'Bounded mode is authorized only for back');
assert.ok(process.argv.length <= 6, 'Unexpected arguments');
const boundedObservation = mode === 'bounded-back-pass-plus-five-seconds';
const postPassSeconds = boundedObservation ? 5 : 30;
assert.ok(['back', 'belly', 'side', 'side-supported'].includes(kind), 'Explicit fixed fixture required');
assert.ok(!existsSync(join(output, 'result.json')), 'Fresh output required');
mkdirSync(output, { recursive: true });
const recordVideo = process.env.RUST_BROWSER_NO_VIDEO !== '1';
if (recordVideo && process.env.RUST_BROWSER_FFMPEG) {
  const require = createRequire(import.meta.url);
  const core = dirname(require.resolve('playwright-core/package.json'));
  const descriptor = JSON.parse(readFileSync(join(core, 'browsers.json'))).browsers.find(x => x.name === 'ffmpeg');
  const tools = join(output, 'playwright-tools');
  const executable = join(tools, `ffmpeg-${descriptor.revision}`, 'ffmpeg-linux');
  mkdirSync(dirname(executable), { recursive: true });
  symlinkSync(resolve(process.env.RUST_BROWSER_FFMPEG), executable);
  process.env.PLAYWRIGHT_BROWSERS_PATH = tools;
}
const { chromium, browserLaunchOptions } = await import('../../../scripts/rust-browser-test.mjs');
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function hashes(root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path)) : [[relative(artifact, path), hash(readFileSync(path))]];
  }));
}
const report = {
  at: new Date().toISOString(), passed: false, thirtySecondStandingObserved: false, qualificationClaim: false, recordVideo, kind, mode, artifact,
  knownNativeLimitation: 'This frozen experimental feature back case loses its original fixed10mm foot-origin drift gate at21.85875s of continuation. A bounded pass+5s clip cannot clear that failure or establish30s standing.',
  artifactSha256: hashes(artifact), errors: [], progress: [],
  scope: 'Isolated fixed-fixture learned-recovery TEST with source-fidelity default20+5 numerical profile,800Hz physics/40Hz control. Existing WASM fixture factory/trigger, original driver/renderer/one-tick scheduler. Native wall-time footage; no retiming, physical commands, reset, support injection or3+1 profile override. Browser snapshots are40Hz; native every800Hz qualification remains separate.',
  performanceContext: process.env.FIXTURE_PERFORMANCE_CONTEXT ?? 'Unspecified; do not infer clean timing',
  maximumRecoveryPhysicalSeconds: 25, requiredPostPassPhysicalSeconds: postPassSeconds,
};
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0'], { stdio: ['ignore', 'pipe', 'inherit'] });
let browser, context, page, video;
try {
  const origin = await new Promise((resolve, reject) => {
    let text = '';
    const timer = setTimeout(() => reject(new Error('Preview timeout')), 10000);
    server.on('error', reject);
    server.stdout.on('data', data => {
      text += data;
      const url = text.match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];
      if (url) { clearTimeout(timer); resolve(url); }
    });
  });
  browser = await chromium.launch(browserLaunchOptions(true));
  report.browserVersion = browser.version();
  context = await browser.newContext({ viewport: { width: 1280, height: 900 },
    ...(recordVideo ? { recordVideo: { dir: output, size: { width: 1280, height: 900 } } } : {}) });
  await context.addInitScript(({ kind, postPassSeconds, boundedObservation }) => {
    window.__lhGetUpTrial = true;
    let api;
    Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api,
      set(value) { api = value; api.setManual(true); } });
    const evidence = window.__fixtureEvidence = {
      kind, workerInstances: 0, workerIdentities: [], samples: [], commands: [], physicalInputs: [],
      initializations: [], trigger: null, profile: null, firstPassTimeS: null,
      startedAtMs: null, endedAtMs: null, initialFrames: null, endedFrames: null,
      stopReason: null, error: null, postPassDiagnostics: [], standingViolations: [], postPassContinuity: [], distinctPostPassSamples: 0, duplicatePostPassPackets: 0,
    };
    const stop = reason => {
      if (evidence.endedAtMs !== null || evidence.startedAtMs === null) return;
      evidence.stopReason = reason;
      evidence.endedAtMs = performance.now();
      evidence.endedFrames = api?.diagnostics.graphics?.frames ?? null;
      api?.setManual(true);
    };
    const Native = window.Worker;
    window.Worker = class extends Native {
      constructor(...args) {
        super(...args);
        evidence.workerInstances++;
        this.addEventListener('message', event => {
          try {
            const message = event.data;
            if (message.type === 'ready') evidence.workerIdentities.push(message.workerIdentity);
            if (message.type === 'error') { evidence.error = message.message; stop('worker-error'); return; }
            if (!message.reply) return;
            const fixture = message.fixtureTest;
            if (!fixture?.isolatedTestOnly || fixture.kind !== kind) throw new Error('Explicit fixture adapter metadata missing');
            if (fixture.profile.solver_iterations !== 20 || fixture.profile.additional_body_iterations !== 5
              || fixture.profile.getup_policy_enabled !== true || fixture.profile.dt_s !== 1 / 800
              || fixture.profile.tick_substeps !== 20) throw new Error('Wrong source-fidelity fixture profile');
            const profile = JSON.stringify(fixture.profile);
            if (evidence.profile !== null && evidence.profile !== profile) throw new Error('Fixture profile changed');
            evidence.profile = profile;
            if (fixture.triggerEvidence) {
              if (evidence.trigger !== null) throw new Error('Fixture triggered more than once');
              evidence.trigger = structuredClone(fixture.triggerEvidence);
            }
            const snapshot = message.reply.snapshot;
            if (!snapshot.diagnostics.getup?.learned_policy_loaded) throw new Error('Learned prototype must be explicit in fixture TEST');
            if (snapshot.stamp.generation !== 1) throw new Error('Fixture generation changed');
            evidence.samples.push(structuredClone(snapshot));
            if (fixture.postPassStanding) {
              const stable = snapshot.diagnostics.getup.policy.stable_standing_s;
              const previous = evidence.postPassContinuity.at(-1);
              const physicalDelta = previous ? snapshot.simulation_time_s - previous.time : null;
              const stableDelta = previous ? stable - previous.stable : null;
              const native = fixture.postPassStanding.nativeSubsteps;
              const nativeDelta = previous ? native - previous.native : null;
              const duplicate = previous && nativeDelta === 0;
              const clockSequence = !previous || (duplicate
                ? physicalDelta === 0 && stableDelta === 0 && message.reply.completed_ticks === 0
                : nativeDelta === 20 && Math.abs(physicalDelta - .025) <= 1e-9 && message.reply.completed_ticks === 1);
              if (duplicate) evidence.duplicatePostPassPackets++;
              else evidence.distinctPostPassSamples++;
              const continuous = snapshot.motion === 'Upright' && stable >= 1 && clockSequence
                && (!previous || Math.abs(stableDelta - physicalDelta) <= 1e-9);
              evidence.postPassContinuity.push({ native, time: snapshot.simulation_time_s, stable, motion: snapshot.motion,
                physicalDelta, stableDelta, nativeDelta, duplicate: Boolean(duplicate), clockSequence, continuous });
              if (!continuous) {
                evidence.standingViolations.push({ time: snapshot.simulation_time_s, check: 'original-native-stable-interval-continuity',
                  stable, motion: snapshot.motion, physicalDelta, stableDelta, nativeDelta, clockSequence });
                stop('sampled-standing-continuity-failure');
              }
              evidence.postPassDiagnostics.push(structuredClone(fixture.postPassStanding));
              const failed = Object.entries(fixture.postPassStanding.predicates).filter(([, passed]) => passed !== true);
              if (failed.length) {
                evidence.standingViolations.push({ time: snapshot.simulation_time_s, failed });
                stop('sampled-standing-predicate-failure');
              }
            }
            if (evidence.startedAtMs === null) return;
            if (message.reply.failure) { stop('physical-or-resource-halt'); return; }
            const policy = snapshot.diagnostics.getup.policy;
            if (policy?.prototype_standing_passed && evidence.firstPassTimeS === null) {
              evidence.firstPassTimeS = snapshot.simulation_time_s;
            }
            if (evidence.firstPassTimeS !== null
              && snapshot.simulation_time_s >= evidence.firstPassTimeS + postPassSeconds) stop(boundedObservation
                ? 'bounded-five-physical-seconds-after-first-pass' : 'thirty-physical-seconds-after-first-pass');
            if (fixture.triggerNative !== null && evidence.firstPassTimeS === null
              && snapshot.simulation_time_s >= fixture.triggerNative / 800 + 25) stop('unchanged-recovery-deadline');
          } catch (error) { evidence.error = String(error); stop('instrumentation-error'); }
        });
      }
      postMessage(message, ...args) {
        if (message.type === 'initialize') {
          message = { ...message, testFixedGetUpFixture: kind };
          evidence.initializations.push(structuredClone(message));
        }
        if (message.commands?.length) evidence.commands.push(structuredClone(message.commands));
        return super.postMessage(message, ...args);
      }
    };
    document.addEventListener('pointerdown', event => {
      if (evidence.startedAtMs !== null && evidence.endedAtMs === null
        && event.target?.closest?.('#viewport')) evidence.physicalInputs.push({ type: event.type, atMs: event.timeStamp });
    }, true);
    document.addEventListener('DOMContentLoaded', () => {
      const label = document.createElement('div');
      Object.assign(label.style, { position: 'fixed', bottom: '8px', left: '8px', zIndex: '2147483647',
        pointerEvents: 'none', background: '#091424ed', color: '#fff', padding: '8px 12px', font: '12px monospace', whiteSpace: 'pre' });
      document.body.append(label);
      const draw = () => {
        const d = api?.diagnostics, policy = d?.snapshot?.diagnostics.getup?.policy;
        const wall = evidence.startedAtMs === null ? 0 : ((evidence.endedAtMs ?? performance.now()) - evidence.startedAtMs) / 1000;
        label.textContent = `ISOLATED FIXED-FIXTURE TEST | ${kind} | default20+5 |800Hz/40Hz\n${boundedObservation ? 'BOUNDED PASS+5s OBSERVATION | NOT30s QUALIFIED | known later10mm foot-drift failure' : '30s post-pass sampled observation; native qualification separate'}\nLearned prototype explicitly enabled | native wall time, NOT RETIMED\nsim ${(d?.time ?? 0).toFixed(3)}s | wall ${wall.toFixed(2)}s | ${policy?.phase ?? 'physical preparation'}\ntrigger native ${evidence.trigger?.native ?? 'pending'} | stable ${(policy?.stable_standing_s ?? 0).toFixed(3)}s | first native pass ${Boolean(policy?.prototype_standing_passed)}\n${evidence.stopReason ?? 'ordinary automatic scheduler; no physical input'}`;
        requestAnimationFrame(draw);
      };
      draw();
    });
  }, { kind, postPassSeconds, boundedObservation });
  page = await context.newPage();
  video = page.video();
  page.on('pageerror', error => report.errors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) report.errors.push(`${response.status()} ${response.url()}`); });
  await page.goto(origin + '?renderer=webgl2');
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3,
    null, { timeout: 60000 });
  report.initial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(report.initial.tick, 0);
  assert.equal(report.initial.time, 0);
  await page.locator('#camera-focus').click();
  await page.screenshot({ path: join(output, 'initial-fixed-fixture.png'), fullPage: true });
  await page.evaluate(() => {
    const e = window.__fixtureEvidence;
    window.__lhRust.drainTelemetry();
    e.startedAtMs = performance.now();
    e.initialFrames = window.__lhRust.diagnostics.graphics.frames;
    window.__lhRust.setManual(false);
  });
  const deadline = Date.now() + 20 * 60 * 1000;
  report.telemetry = { packets: [], frames: [], inputs: [], bounds: null };
  let nextProgressS = 0;
  while (true) {
    await page.waitForTimeout(1000);
    const update = await page.evaluate(() => {
      const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
      return { time: d.time, tick: d.tick, motion: d.motion, policy: d.snapshot?.diagnostics.getup?.policy,
        error: e.error || d.error, stopReason: e.stopReason, ended: e.endedAtMs !== null,
        wallSeconds: ((e.endedAtMs ?? performance.now()) - e.startedAtMs) / 1000,
        telemetry: window.__lhRust.drainTelemetry() };
    });
    const telemetry = update.telemetry;
    report.telemetry.packets.push(...telemetry.packets);
    report.telemetry.frames.push(...telemetry.frames);
    report.telemetry.inputs.push(...telemetry.inputs);
    report.telemetry.bounds = telemetry.bounds;
    for (const name of ['droppedPackets', 'droppedFrames', 'droppedInputs', 'pendingVisualInputs']) report.telemetry[name] = telemetry[name];
    delete update.telemetry;
    if (update.time >= nextProgressS || update.ended || update.error) {
      report.progress.push(update);
      console.log(JSON.stringify({ finding: 'fixed-fixture-test-progress', ...update }));
      nextProgressS = update.time + 1;
    }
    if (update.ended || update.error) break;
    assert.ok(Date.now() < deadline, 'Twenty-minute native-wall-time watchdog exceeded');
  }
  await page.evaluate(() => window.__lhRust.setManual(true));
  await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding);
  report.final = await page.evaluate(() => window.__lhRust.diagnostics);
  report.evidence = await page.evaluate(() => window.__fixtureEvidence);
  report.fullSourceProfile = JSON.parse(report.evidence.profile);
  const e = report.evidence;
  assert.equal(e.workerInstances, 1);
  assert.equal(e.workerIdentities.length, 1);
  assert.equal(e.initializations.length, 1);
  assert.equal(e.initializations[0].profile, undefined, 'No3+1 profile override');
  assert.deepEqual(e.commands, []);
  assert.deepEqual(e.physicalInputs, []);
  assert.equal(e.error, null);
  assert.deepEqual(e.standingViolations, []);
  assert.ok(e.distinctPostPassSamples >= postPassSeconds * 40 + 1, 'Required distinct consecutive20-native-step post-pass standing diagnostics');
  assert.ok(e.trigger?.physicalPoseVelocityBitsUnchanged);
  assert.deepEqual(e.trigger.beforePoseWords, e.trigger.afterPoseWords);
  assert.equal(e.trigger.beforePoseWords.length, 325);
  assert.equal(e.stopReason, boundedObservation ? 'bounded-five-physical-seconds-after-first-pass' : 'thirty-physical-seconds-after-first-pass');
  assert.equal(report.final.failure, null);
  assert.equal(report.final.error, '');
  assert.equal(report.final.motion, 'Upright');
  assert.ok(report.final.time >= e.firstPassTimeS + postPassSeconds);
  assert.deepEqual(report.errors, []);
  report.observation = {
    simulationSeconds: report.final.time,
    wallSeconds: (e.endedAtMs - e.startedAtMs) / 1000,
    integratedSimulationToWallRatio: report.final.time / ((e.endedAtMs - e.startedAtMs) / 1000),
    presentedFrames: e.endedFrames - e.initialFrames,
    note: 'Recorded encoded/concurrency context; not realtime/mobile or native800Hz standing qualification.',
  };
  await page.screenshot({ path: join(output, boundedObservation ? 'after-bounded-five-physical-seconds-standing.png' : 'after-thirty-physical-seconds-standing.png'), fullPage: true });
  report.thirtySecondStandingObserved = !boundedObservation;
  report.passed = true;
} catch (error) {
  report.error = error.stack ?? String(error);
  if (page && !page.isClosed()) {
    try {
      report.failureState = await page.evaluate(() => window.__lhRust?.diagnostics);
      report.evidence = await page.evaluate(() => window.__fixtureEvidence);
      await page.screenshot({ path: join(output, 'failure.png'), fullPage: true });
    } catch {}
  }
} finally {
  report.finished = new Date().toISOString();
  try {
    await context?.close();
    if (video) {
      await video.saveAs(join(output, 'fixed-fixture-native-wall-time.webm'));
      report.recording = { path: 'fixed-fixture-native-wall-time.webm', retimed: false };
    }
  } catch (error) { report.recordingError = String(error); report.passed = false; }
  await browser?.close();
  server.kill();
  writeFileSync(join(output, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({ passed: report.passed, error: report.error, finalTime: report.final?.time,
    stopReason: report.evidence?.stopReason, observation: report.observation, output }, null, 2));
  process.exitCode = report.passed ? 0 : 1;
}
