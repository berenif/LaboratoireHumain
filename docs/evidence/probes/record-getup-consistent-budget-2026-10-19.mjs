// Explicit consistent-solver-budget fixed-fixture TEST footage. Run only after source/native qualification.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';
import { dirname, join, relative, resolve } from 'node:path';

const artifact = resolve(process.argv[2]);
const output = resolve(process.argv[3]);
const kind = process.argv[4];
const mode = process.argv[5] ?? 'thirty-second-standing-observation';
assert.ok(['thirty-second-standing-observation', 'bounded-back-pass-plus-five-seconds'].includes(mode), 'Explicit reviewed observation mode required');
assert.ok(mode !== 'bounded-back-pass-plus-five-seconds' || kind === 'back', 'Bounded mode is authorized only for back');
assert.ok(process.argv.length <= 6, 'Unexpected arguments');
assert.equal(mode, 'thirty-second-standing-observation', 'Combined feature run retains the full30physical-second observation');
assert.ok(['back','belly','side','side-supported'].includes(kind), 'Approved fresh four-pose consistent-budget cases only');
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
const { chromium, browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
function hashes(root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path)) : [[relative(artifact, path), hash(readFileSync(path))]];
  }));
}
const report = {
  at: new Date().toISOString(), passed: false, thirtySecondStandingObserved: false, qualificationClaim: false, recordVideo, kind, mode, artifact, selectedGetUpSolverBudget: 'consistent-recovery-phase-v1',
  experimentalDisposition: 'Explicit consistent-recovery-phase numerical budget candidate: native four-pose first-standing preflight passes; final native holds and actual-WASM four-pose gates must pass independently. No global recovery or release admission from this run alone.',
  experimentalFeatures: ['lh-worker/root-translation-carry-experiment', 'lh-sim/getup-terminal-balance-experiment', 'lh-sim/standing-dynamics-experiment'],
  artifactSha256: hashes(artifact), errors: [], progress: [],
  scope: 'Isolated fixed-fixture learned-recovery TEST with complete explicit3+1 consistent-recovery-phase numerical profile,800Hz physics/40Hz control. Existing WASM fixture factory/trigger, original driver/renderer/one-tick scheduler. Native wall-time footage; no retiming, physical commands, reset, support injection or midworld profile changes. Browser snapshots are40Hz; native every800Hz qualification remains separate.',
  performanceContext: process.env.FIXTURE_PERFORMANCE_CONTEXT ?? 'Physical-only carry trial with first-pass checkpoint hold; no performance qualification',
  diagnosticReplay: false, checkpointCaptures: {},
  maximumRecoveryPhysicalSeconds: 25, requiredPostPassPhysicalSeconds: postPassSeconds,
};
const adapterManifestPath = artifact + '-fixture-adapter-manifest.json';
const adapterManifest = JSON.parse(readFileSync(adapterManifestPath));
assert.deepEqual(report.artifactSha256, adapterManifest.artifactSha256, 'Every frozen combined app byte must match its adapter manifest');
assert.deepEqual([...adapterManifest.sourceBuildFeatures].sort(), [...report.experimentalFeatures].sort(), 'Exactly the approved three opt-in features required');
const approvedProfile = adapterManifest.explicitProfile.value;
assert.equal(adapterManifest.explicitProfile.sha256, 'da054ad1556556b20e9469c042e6e5fd4088f3630092a9d4a7b992de43f2199c');
report.explicitProfile = structuredClone(approvedProfile);
report.adapterManifest = { path: adapterManifestPath, sha256: hash(readFileSync(adapterManifestPath)) };
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0'], { stdio: ['ignore', 'pipe', 'inherit'] });
let browser, context, page, video;
async function captureReadOnlyCheckpoint(label) {
  await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding);
  const captured = await page.evaluate(async () => {
    const api = window.__lhRust;
    const before = structuredClone(api.diagnostics);
    const controller = await api.getupReport();
    const checkpoint = await api.getupCheckpoint();
    const after = api.diagnostics;
    const words = snapshot => snapshot.segments.flatMap(p => ['position','rotation','linearVelocity','angularVelocity']
      .flatMap(k => (k === 'rotation' ? ['x','y','z','w'] : ['x','y','z']).map(a => p[k][a])));
    const beforeWords = Array.from(new Uint32Array(new Float32Array(words(before.snapshot)).buffer));
    const afterWords = Array.from(new Uint32Array(new Float32Array(words(after.snapshot)).buffer));
    if (before.time !== after.time || before.tick !== after.tick
      || before.snapshot.stamp.generation !== after.snapshot.stamp.generation || beforeWords.length !== 325
      || JSON.stringify(beforeWords) !== JSON.stringify(afterWords)) throw new Error('Read-only capture changed physical state');
    const bytes = checkpoint.checkpoint;
    let binary = '';
    for (let i = 0; i < bytes.length; i += 0x8000) binary += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
    return { controller, checkpointBase64: btoa(binary), verification: checkpoint.verification,
      time: before.time, tick: before.tick, cachedPoseVelocityWordsUnchanged: true };
  });
  const bytes = Buffer.from(captured.checkpointBase64, 'base64');
  const checkpointPath = join(output, `${label}-checkpoint.bin`);
  const controllerPath = join(output, `${label}-controller.json`);
  writeFileSync(checkpointPath, bytes);
  writeFileSync(controllerPath, JSON.stringify(captured.controller, null, 2) + '\n');
  report.checkpointCaptures[label] = { checkpointPath, checkpointSha256: hash(bytes), bytes: bytes.length,
    controllerPath, controllerSha256: hash(readFileSync(controllerPath)), verification: captured.verification,
    time: captured.time, tick: captured.tick, cachedPoseVelocityWordsUnchanged: captured.cachedPoseVelocityWordsUnchanged };
  assert.equal(captured.verification.passed, true);
  const envelope = parseGetUpCheckpoint(bytes);
  assert.equal(envelope.schema, 2, 'Active carry checkpoint must preserve schema2 history');
  assert.equal(envelope.wrapper.format, 'f32-dq-f64-remainder-v1');
  assert.equal(captured.verification.rootIntegratorStateEqual, true);
  report.checkpointCaptures[label].schema = envelope.schema;
  report.checkpointCaptures[label].rawAuxiliarySha256 = hash(envelope.auxiliaryBytes);
  assert.equal(captured.verification.physicsIntegrations, 0);
  assert.equal(captured.verification.nativeSubsteps, Math.round(captured.time * 800));
  report.checkpointCaptures[label].readOnlyGuarantee = 'Source methods borrow Runtime immutably; native checkpoint verification reports0integrations and matching current native clock. Cached325pose words are supplementary only.';
  return captured.controller;
}
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
  await context.addInitScript(({ kind, postPassSeconds, boundedObservation, approvedProfile }) => {
    window.__lhGetUpTrial = true;
    let api;
    Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api,
      set(value) { api = value; api.setManual(true); } });
    const evidence = window.__fixtureEvidence = {
      kind, workerInstances: 0, workerIdentities: [], samples: [], commands: [], physicalInputs: [],
      initializations: [], trigger: null, profile: null, firstPassTimeS: null,
      startedAtMs: null, endedAtMs: null, initialFrames: null, endedFrames: null,
      stopReason: null, error: null, activation: null, historicalDiagnostics: [], solverBudgetSamples: [], lastReceivedPackedPosePacket: null, passCheckpointPending: false, passCheckpointSaved: false, postPassDiagnostics: [], standingViolations: [], postPassContinuity: [], distinctPostPassSamples: 0, duplicatePostPassPackets: 0,
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
            if (fixture.profile.solver_iterations !== 3 || fixture.profile.additional_body_iterations !== 1
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
            if (message.poseBuffer instanceof ArrayBuffer) {
              evidence.lastReceivedPackedPosePacket = {
                time: snapshot.simulation_time_s, native: Math.round(snapshot.simulation_time_s * 800),
                wordsUint32: Array.from(new Uint32Array(message.poseBuffer)),
                source: 'Copied received transfer-buffer words before driver recycles/detaches it; no physical write',
              };
            }
            if (!snapshot.diagnostics.getup?.learned_policy_loaded) throw new Error('Learned prototype must be explicit in fixture TEST');
            if (snapshot.stamp.generation !== 1) throw new Error('Fixture generation changed');
            if (fixture.carryActivationEvidence) {
              if (evidence.activation !== null) throw new Error('Active-world carry activation repeated');
              evidence.activation = structuredClone(fixture.carryActivationEvidence);
              if (evidence.activation.activationCalls !== 1
                || evidence.activation.afterReport.nativeSubsteps !== 0
                || evidence.activation.afterReport.rootTranslationCompensation?.algorithm !== 'f32-dq-f64-remainder-v1') throw new Error('Explicit native0 carry activation proof missing');
            }
            const solver = fixture.fixtureSolverBudget;
            if (!solver || ![20,32].includes(solver.currentInternalPgsIterations)
              || ![20,32].includes(solver.selectedNextInternalPgsIterations)
              || typeof solver.sourceRecoveryActive !== 'boolean') throw new Error('Actual solver-budget metadata missing');
            if (solver.sourceRecoveryActive && snapshot.motion === 'Recovering'
              && solver.selectedNextInternalPgsIterations !== 32) throw new Error('Source recovery did not select explicit32internal PGS');
            if (snapshot.diagnostics.getup.policy?.prototype_standing_passed
              && (solver.sourceRecoveryActive || solver.currentInternalPgsIterations !== 20
                || solver.selectedNextInternalPgsIterations !== 20)) throw new Error('Terminal standing must use declared20internal PGS');
            evidence.solverBudgetSamples.push({native:Math.round(snapshot.simulation_time_s*800),
              time:snapshot.simulation_time_s,motion:snapshot.motion,...structuredClone(solver)});
            evidence.samples.push(structuredClone(snapshot));
            if (fixture.originalAnchorStanding) {
              evidence.historicalDiagnostics.push(structuredClone(fixture.originalAnchorStanding));
              const failed = Object.entries(fixture.originalAnchorStanding.predicates).filter(([, value]) => value !== true);
              if (failed.length) {
                evidence.standingViolations.push({ time: snapshot.simulation_time_s,
                  check: 'exact-original-controller-stable-feet', failed,
                  historicalFootDriftM: fixture.originalAnchorStanding.footDriftM });
                stop('original-historical-anchor-predicate-failure');
              }
              if (evidence.firstPassTimeS === null && evidence.startedAtMs !== null) {
                evidence.passCheckpointPending = true;
                api.setManual(true); // Request scheduling only; preserve this exact first-pass world.
              }
            }
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
          message = { ...message, testFixedGetUpFixture: kind, testFixedGetUpProfile: approvedProfile };
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
        label.textContent = `ISOLATED FIXED-FIXTURE TEST | ${kind} | explicit3+1 |800Hz/40Hz\nEXPERIMENTAL: carry + standing-dynamics + terminal-balance | sourcePGS32 then terminalPGS20\n${boundedObservation ? 'BOUNDED PASS+5s OBSERVATION | NOT30s QUALIFIED | known later10mm foot-drift failure' : '30s post-pass sampled observation; native qualification separate'}\nLearned prototype explicitly enabled | native wall time, NOT RETIMED\nsim ${(d?.time ?? 0).toFixed(3)}s | wall ${wall.toFixed(2)}s | ${policy?.phase ?? 'physical preparation'}\ntrigger native ${evidence.trigger?.native ?? 'pending'} | stable ${(policy?.stable_standing_s ?? 0).toFixed(3)}s | first native pass ${Boolean(policy?.prototype_standing_passed)}\n${evidence.stopReason ?? 'ordinary automatic scheduler; no physical input'}`;
        requestAnimationFrame(draw);
      };
      draw();
    });
  }, { kind, postPassSeconds, boundedObservation, approvedProfile });
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
  const activation = await page.evaluate(() => {
    const e = window.__fixtureEvidence, a = e.activation;
    if (!a) throw new Error('Activation evidence not retained at initial packet');
    const encode = bytes => {
      let text = '';
      for (let i=0;i<bytes.length;i+=0x8000) text += String.fromCharCode(...bytes.subarray(i,i+0x8000));
      return btoa(text);
    };
    const beforeBase64 = encode(a.beforeCheckpoint), afterBase64 = encode(a.afterCheckpoint);
    delete a.beforeCheckpoint; delete a.afterCheckpoint;
    return { metadata: structuredClone(a), beforeBase64, afterBase64 };
  });
  const beforeActivation = Buffer.from(activation.beforeBase64,'base64'), afterActivation = Buffer.from(activation.afterBase64,'base64');
  report.activationProof = { ...activation.metadata, independentlyDecodedPurity: proveActivationPurity(beforeActivation,afterActivation) };
  assert.deepEqual(report.activationProof.poseVelocityF32WordsBefore,report.activationProof.poseVelocityF32WordsAfter);
  assert.deepEqual(report.activationProof.afterReport.profile,approvedProfile);
  for (const [label,bytes] of [['before',beforeActivation],['after',afterActivation]]) {
    const path = join(output,`activation-${label}-checkpoint.bin`);
    writeFileSync(path,bytes);
    report.activationProof[`${label}Checkpoint`] = { path,sha256:hash(bytes),bytes:bytes.length };
  }
  writeFileSync(join(output,'activation-proof.json'),JSON.stringify(report.activationProof,null,2)+'\n');
  await page.locator('#camera-focus').click();
  const viewport = await page.locator('#viewport').boundingBox();
  assert.ok(viewport, 'Actual viewport required for ordinary camera framing');
  await page.mouse.move(viewport.x + viewport.width / 2, viewport.y + viewport.height / 2);
  await page.mouse.wheel(0, 400);
  await page.waitForTimeout(150);
  report.framing = { ordinaryUiOnly: true, cameraFocusBody: true, preStartWheelDeltaY: 400, note: 'Zoom out before any physics to keep lying and standing full body in view; no physical command.' };
  report.framedInitial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(report.framedInitial.tick, 0);
  assert.equal(report.framedInitial.time, 0);
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
        passCheckpointPending: e.passCheckpointPending,
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
    if (update.passCheckpointPending) {
      const controller = await captureReadOnlyCheckpoint('first-pass');
      const historical = await page.evaluate(() => window.__fixtureEvidence.historicalDiagnostics[0]);
      assert.deepEqual(controller.stable_feet, historical.originalStableFeet,
        'Native controller report and retained original anchors must match exactly');
      await page.evaluate(() => {
        const e = window.__fixtureEvidence;
        e.passCheckpointPending = false;
        e.passCheckpointSaved = true;
        if (e.endedAtMs === null) window.__lhRust.setManual(false);
      });
    }
    if (update.ended || update.error) break;
    assert.ok(Date.now() < deadline, 'Twenty-minute native-wall-time watchdog exceeded');
  }
  await page.evaluate(() => window.__lhRust.setManual(true));
  await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding);
  const failedWorker = await page.evaluate(() => {
    const d = window.__lhRust.diagnostics, e = window.__fixtureEvidence;
    if (!d.workerFault) return null;
    e.stopReason ??= 'renderer-or-driver-terminated-worker';
    e.endedAtMs ??= performance.now();
    e.endedFrames ??= d.graphics?.frames ?? null;
    return { workerFault: true, error: d.error, time: d.time,
      reason: 'Original driver already terminated the worker. No stop checkpoint can be exported; no resurrection or continuation attempted.' };
  });
  if (failedWorker) report.stopCheckpointUnavailable = failedWorker;
  else await captureReadOnlyCheckpoint('stop');
  report.diagnosticCaptureComplete = Boolean(report.checkpointCaptures['first-pass'] && report.checkpointCaptures.stop);
  report.final = await page.evaluate(() => window.__lhRust.diagnostics);
  report.evidence = await page.evaluate(() => window.__fixtureEvidence);
  report.fullSourceProfile = JSON.parse(report.evidence.profile);
  const e = report.evidence;
  assert.equal(e.workerInstances, 1);
  assert.equal(e.workerIdentities.length, 1);
  assert.equal(e.initializations.length, 1);
  assert.equal(e.initializations[0].profile, undefined, 'Ordinary driver profile path unchanged');
  assert.deepEqual(e.initializations[0].testFixedGetUpProfile, approvedProfile, 'Exact complete explicit fixture profile retained');
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
