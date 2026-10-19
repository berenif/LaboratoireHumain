// SOURCE-ONLY until independently cleared. Isolated belly fixture, one ordinary
// keyboard Strike, original driver/renderer/transport and one-tick scheduler.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, join, relative, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';

assert.ok(process.argv[2] && process.argv[3], 'Usage: recorder ARTIFACT FRESH_OUTPUT [belly]');
assert.ok(process.argv.length <= 5, 'Unexpected arguments');
const artifact = resolve(process.argv[2]), output = resolve(process.argv[3]);
const kind = process.argv[4] ?? 'belly';
assert.equal(kind, 'belly', 'This bounded repeated-cycle observation is approved only for a fresh belly fixture');
assert.ok(!existsSync(join(output, 'result.json')), 'Fresh output required');
mkdirSync(output, { recursive: true });
const recordVideo = process.env.RUST_BROWSER_NO_VIDEO !== '1';
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const sameJson = (a, b) => {
  const canonical = x => Array.isArray(x) ? x.map(canonical) : x && typeof x === 'object'
    ? Object.fromEntries(Object.keys(x).sort().map(key => [key, canonical(x[key])])) : x;
  return JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
};
function hashes(root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path)) : [[relative(artifact, path), hash(readFileSync(path))]];
  }));
}
const report = {
  at: new Date().toISOString(), passed: false, observationComplete: false, qualificationClaim: false,
  repeatedRecoveryDemonstrated: false, thirtySecondContinuousStandingClaim: false,
  kind, mode: 'initial-native-pass-plus-three-seconds-one-ordinary-strike-bounded-response', artifact, recordVideo,
  selectedGetUpSolverBudget: 'consistent-recovery-phase-v1', rootRotationNormalization: 'unit-free-root-v1',
  repeatedRecoveryPolicy: 'completed-cycle-confirmed-fall-v1',
  experimentalFeatures: ['lh-worker/root-translation-carry-experiment', 'lh-sim/getup-terminal-balance-experiment',
    'lh-sim/standing-dynamics-experiment', 'lh-sim/unit-root-rotation-experiment', 'lh-sim/getup-repeat-cycle-experiment'],
  scope: 'Isolated fresh belly TEST: unchanged native0 carry activation once, original80-native preparation polling, '
    + '800Hz physics/40Hz publications, first native1s standing latch plus3physical s continuous original-anchor hold. '
    + 'Then exactly one actual keyboard KeyP through the unchanged driver and genuine striker apparatus. '
    + 'No reset, support injection, direct physical command, retiming, profile change, or performance/release claim.',
  observationRules: { recoveryDeadlineSeconds: 25, originalAnchorHoldSeconds: 3, contactedResistedObservationSeconds: 30,
    uncontactedApparatusDeadlineSeconds: 5, nativeHz: 800, publicationHz: 40,
    contactedClock: 'First positive-impact publication upper bound; observing30s from it conservatively covers at least30physical s after actual first contact.',
    failedRecoveryDeadlineNeverExtended: true },
  performanceContext: process.env.FIXTURE_PERFORMANCE_CONTEXT ?? 'Physical-only repeated-cycle TEST; no performance qualification',
  errors: [], progress: [], diagnosticReplay: false, checkpointCaptures: {},
};
let server, browser, context, page, video;

// Scheduling is already paused by the observer before the original driver pump.
// A dead/disposed worker is never asked to export or resurrected for evidence.
async function captureReadOnlyCheckpoint(label) {
  await page.waitForFunction(() => {
    const d = window.__lhRust?.diagnostics;
    return d && (!d.outstanding || d.workerFault || d.disposed);
  }, null, { timeout: 100000 });
  const captured = await page.evaluate(async () => {
    const api = window.__lhRust, e = window.__fixtureEvidence;
    const before = structuredClone(api.diagnostics);
    if (before.workerFault || before.disposed || e.error) return { unavailable: true,
      workerFault: before.workerFault, disposed: before.disposed, error: e.error || before.error, time: before.time };
    const controller = await api.getupReport();
    if (api.diagnostics.workerFault || api.diagnostics.disposed || e.error) return { unavailable: true,
      workerFault: api.diagnostics.workerFault, disposed: api.diagnostics.disposed, error: e.error || api.diagnostics.error };
    const checkpoint = await api.getupCheckpoint();
    const after = api.diagnostics;
    const words = snapshot => snapshot.segments.flatMap(p => ['position', 'rotation', 'linearVelocity', 'angularVelocity']
      .flatMap(key => (key === 'rotation' ? ['x', 'y', 'z', 'w'] : ['x', 'y', 'z']).map(axis => p[key][axis])));
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
  if (captured.unavailable) {
    report.checkpointCaptures[label] = { unavailable: true, ...captured,
      reason: 'Original worker unavailable or observer faulted; no export, resurrection, or pending capture retry.' };
    return null;
  }
  const bytes = Buffer.from(captured.checkpointBase64, 'base64');
  const checkpointPath = join(output, `${label}-checkpoint.bin`), controllerPath = join(output, `${label}-controller.json`);
  writeFileSync(checkpointPath, bytes);
  writeFileSync(controllerPath, JSON.stringify(captured.controller, null, 2) + '\n');
  const envelope = parseGetUpCheckpoint(bytes);
  report.checkpointCaptures[label] = { checkpointPath, checkpointSha256: hash(bytes), bytes: bytes.length,
    controllerPath, controllerSha256: hash(readFileSync(controllerPath)), verification: captured.verification,
    time: captured.time, tick: captured.tick, schema: envelope.schema, rawAuxiliarySha256: hash(envelope.auxiliaryBytes),
    cachedPoseVelocityWordsUnchanged: true,
    readOnlyGuarantee: 'Immutable native export/verification; zero physics integrations, unchanged native clock and root-integrator state. Cached325pose words are supplementary.' };
  assert.equal(envelope.schema, 2, 'Active carry checkpoint must preserve schema2 history');
  assert.equal(envelope.wrapper.format, 'f32-dq-f64-remainder-v1');
  assert.equal(captured.verification.passed, true);
  assert.equal(captured.verification.physicsIntegrations, 0);
  assert.equal(captured.verification.rootIntegratorStateEqual, true);
  assert.equal(captured.verification.nativeSubsteps, Math.round(captured.time * 800));
  return captured.controller;
}

try {
  if (recordVideo && process.env.RUST_BROWSER_FFMPEG) {
    const require = createRequire(import.meta.url), core = dirname(require.resolve('playwright-core/package.json'));
    const descriptor = JSON.parse(readFileSync(join(core, 'browsers.json'))).browsers.find(x => x.name === 'ffmpeg');
    const tools = join(output, 'playwright-tools'), executable = join(tools, `ffmpeg-${descriptor.revision}`, 'ffmpeg-linux');
    mkdirSync(dirname(executable), { recursive: true });
    symlinkSync(resolve(process.env.RUST_BROWSER_FFMPEG), executable);
    process.env.PLAYWRIGHT_BROWSERS_PATH = tools;
  }
  report.artifactSha256 = hashes(artifact);
  const adapterManifestPath = artifact + '-fixture-adapter-manifest.json';
  const adapterManifest = JSON.parse(readFileSync(adapterManifestPath));
  assert.deepEqual(report.artifactSha256, adapterManifest.artifactSha256, 'Every frozen app byte must match its adapter manifest');
  assert.deepEqual([...adapterManifest.sourceBuildFeatures].sort(), [...report.experimentalFeatures].sort(), 'Exactly the five approved opt-in features required');
  const approvedProfile = adapterManifest.explicitProfile.value;
  const approvedProfilePath = resolve('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json');
  const approvedProfileBytes = readFileSync(approvedProfilePath);
  const approvedProfileSha256 = 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6';
  assert.equal(adapterManifest.explicitProfile.sha256, approvedProfileSha256);
  assert.equal(hash(approvedProfileBytes), approvedProfileSha256, 'Approved complete profile document bytes required');
  assert.deepEqual(approvedProfile, JSON.parse(approvedProfileBytes), 'Manifest profile value must equal the complete approved document');
  assert.equal(approvedProfile.id, 'getup-800hz-3plus1-repeat-cycle-v1');
  assert.equal(approvedProfile.getup_repeated_recovery, 'completed-cycle-confirmed-fall-v1');
  assert.equal(approvedProfile.dt_s, 1 / 800);
  assert.equal(approvedProfile.tick_substeps, 20);
  assert.equal(approvedProfile.solver_iterations, 3);
  assert.equal(approvedProfile.additional_body_iterations, 1);
  report.explicitProfile = structuredClone(approvedProfile);
  report.approvedProfileDocument = { path: approvedProfilePath, sha256: approvedProfileSha256 };
  report.adapterManifest = { path: adapterManifestPath, sha256: hash(readFileSync(adapterManifestPath)) };
  const driverPaths = Object.keys(report.artifactSha256).filter(path => path.endsWith('/driver.js'));
  assert.equal(driverPaths.length, 1, 'Exactly one original browser driver required');
  const originalDriverSha256 = hash(readFileSync(resolve('rust/crates/browser/driver.js')));
  assert.equal(report.artifactSha256[driverPaths[0]], originalDriverSha256, 'Copied artifact must retain the unchanged original driver');
  report.originalDriver = { artifactPath: driverPaths[0], sha256: originalDriverSha256, byteEqual: true };
  const { chromium, browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
  server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0'], { stdio: ['ignore', 'pipe', 'inherit'] });
  const origin = await new Promise((resolveOrigin, reject) => {
    let text = '';
    const timer = setTimeout(() => reject(new Error('Preview timeout')), 10000);
    server.on('error', reject);
    server.stdout.on('data', data => {
      text += data;
      const url = text.match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];
      if (url) { clearTimeout(timer); resolveOrigin(url); }
    });
  });
  browser = await chromium.launch(browserLaunchOptions(true));
  report.browserVersion = browser.version();
  context = await browser.newContext({ viewport: { width: 1280, height: 900 },
    ...(recordVideo ? { recordVideo: { dir: output, size: { width: 1280, height: 900 } } } : {}) });
  await context.addInitScript(({ kind, approvedProfile }) => {
    window.__lhGetUpTrial = true;
    let api;
    Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api,
      set(value) { api = value; api.setManual(true); } });
    const canonical = x => Array.isArray(x) ? x.map(canonical) : x && typeof x === 'object'
      ? Object.fromEntries(Object.keys(x).sort().map(key => [key, canonical(x[key])])) : x;
    const equal = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
    const nativeAt = time => Math.round(time * 800);
    const e = window.__fixtureEvidence = {
      kind, workerInstances: 0, workerIdentities: [], initializations: [], samples: [], publications: [],
      commands: [], strikeCommands: [], acknowledgements: [], physicalInputs: [], keyboardInputs: [],
      trigger: null, profile: null, typedProfilePublications: [], activation: null,
      startedAtMs: null, endedAtMs: null, initialFrames: null, endedFrames: null, stopReason: null, error: null,
      phase: 'initial-recovery', initialSourceStartedAtS: null, initialImpactNative: null, initialFirstPass: null, initialHold: [],
      beforeStrike: null, strikeInputRequested: false, strikeInputBoundary: null, firstImpact: null,
      confirmedFall: null, firstNewSource: null, sourceChanges: [], secondFirstPass: null, finalHold: [],
      motionTransitions: [], counterTransitions: [], epochPublications: [], recoveryImpactClocks: [], solverBudgetSamples: [], rootRotationPolicySamples: [],
      standingDiagnostics: [], standingViolations: [], lastReceivedPackedPosePacket: null,
      pendingCaptures: [], savedCaptures: [], skippedCaptures: [], checkpointAnchors: {},
      lastDistinctPublication: null, distinctPublications: 0, duplicatePublications: 0,
      stop(reason, error = null) {
        if (error !== null) this.error ??= String(error);
        if (this.endedAtMs === null) {
          this.stopReason = reason; this.endedAtMs = performance.now(); this.endedFrames = api?.diagnostics.graphics?.frames ?? null;
        }
        if (this.error) this.skippedCaptures.push(...this.pendingCaptures.splice(0));
        api?.setManual(true);
      },
    };
    const requestCapture = label => {
      if (e.error || e.endedAtMs !== null || e.savedCaptures.includes(label) || e.pendingCaptures.includes(label)) return;
      e.pendingCaptures.push(label);
      api.setManual(true); // Runs before the unchanged driver's onmessage/pump.
    };
    const failHold = (label, snapshot, failed, detail) => {
      e.standingViolations.push({ label, native: nativeAt(snapshot.simulation_time_s), time: snapshot.simulation_time_s, failed, detail });
      e.stop(label + '-original-anchor-hold-failure');
    };
    const checkHold = (label, history, snapshot, fixture, completedTicks, epoch) => {
      const policy = snapshot.diagnostics.getup.policy, native = nativeAt(snapshot.simulation_time_s);
      const original = fixture.originalAnchorStanding, standing = fixture.postPassStanding;
      if (!original || !standing) { failHold(label, snapshot, ['original-anchor-diagnostic-missing']); return; }
      const failed = [
        ...Object.entries(original.predicates).filter(([, passed]) => passed !== true).map(([name]) => 'original.' + name),
        ...Object.entries(standing.predicates).filter(([, passed]) => passed !== true).map(([name]) => 'current-anchor.' + name),
      ];
      const previous = history.at(-1), duplicate = previous?.native === native;
      const nativeDelta = previous ? native - previous.native : null;
      const stableDelta = previous ? policy.stable_standing_s - previous.stableS : null;
      const timeDelta = previous ? snapshot.simulation_time_s - previous.time : null;
      const clockContinuous = !previous || (duplicate ? completedTicks === 0 && timeDelta === 0 && stableDelta === 0
        : completedTicks === 1 && nativeDelta === 20 && Math.abs(timeDelta - .025) <= 1e-9 && Math.abs(stableDelta - .025) <= 1e-9);
      const expectedAnchors = e.checkpointAnchors[label] ?? original.originalStableFeet;
      if (!equal(original.originalStableFeet, expectedAnchors)) failed.push('controller-original-anchors-changed');
      if (snapshot.motion !== 'Upright' || !policy.prototype_standing_passed || policy.stable_standing_s < 1) failed.push('native1s-standing-latch');
      if (policy.started_at_s !== epoch) failed.push('controller-epoch-changed');
      if (!clockContinuous) failed.push('native800Hz-stable-counter-continuity');
      if (original.physicsIntegrations !== 0 || standing.physicsIntegrations !== 0
        || original.nativeSubsteps !== native || standing.nativeSubsteps !== native) failed.push('read-only-diagnostic-clock');
      const value = { native, time: snapshot.simulation_time_s, stableS: policy.stable_standing_s,
        startedAtS: policy.started_at_s, motion: snapshot.motion, duplicate: Boolean(duplicate), nativeDelta, stableDelta, timeDelta,
        clockContinuous, originalAnchorStanding: structuredClone(original), postPassStanding: structuredClone(standing), passed: failed.length === 0 };
      history.push(value);
      if (failed.length) failHold(label, snapshot, failed, value);
    };
    const Native = window.Worker;
    window.Worker = class extends Native {
      constructor(...args) {
        super(...args); e.workerInstances++;
        this.addEventListener('message', event => {
          try {
            const message = event.data;
            if (message.type === 'ready') e.workerIdentities.push(message.workerIdentity);
            if (message.type === 'error') { e.stop('worker-error', message.message); return; }
            if (!message.reply) return;
            const fixture = message.fixtureTest, snapshot = message.reply.snapshot;
            if (!fixture?.isolatedTestOnly || fixture.kind !== kind) throw new Error('Explicit fresh belly fixture metadata missing');
            if (fixture.preparationPollingNative !== 80) throw new Error('Original belly80-native preparation polling changed');
            if (!equal(fixture.profile, approvedProfile)
              || fixture.profile.getup_repeated_recovery !== 'completed-cycle-confirmed-fall-v1') throw new Error('Actual complete typed profile differs from the approved repeated-cycle profile');
            e.profile = JSON.stringify(fixture.profile);
            const time = snapshot.simulation_time_s, native = nativeAt(time), policy = snapshot.diagnostics.getup?.policy;
            if (!snapshot.diagnostics.getup?.learned_policy_loaded || !policy) throw new Error('Explicit learned prototype policy missing');
            if (snapshot.stamp.generation !== 1) throw new Error('Fixture generation changed');
            const previous = e.lastDistinctPublication;
            const duplicate = previous?.native === native;
            if (previous && !(duplicate ? message.reply.completed_ticks === 0
              : native - previous.native === 20 && message.reply.completed_ticks === 1)) throw new Error('Original one-tick20-native publication sequence changed');
            if (duplicate) e.duplicatePublications++; else e.distinctPublications++;
            e.typedProfilePublications.push({ native, time, profile: structuredClone(fixture.profile) });
            if (fixture.triggerEvidence) {
              if (e.trigger !== null) throw new Error('Original fixture trigger repeated');
              e.trigger = structuredClone(fixture.triggerEvidence);
            }
            if (fixture.carryActivationEvidence) {
              if (e.activation !== null) throw new Error('Native0 carry activation repeated');
              e.activation = structuredClone(fixture.carryActivationEvidence);
              if (e.activation.activationCalls !== 1 || e.activation.afterReport.nativeSubsteps !== 0
                || e.activation.afterReport.rootTranslationCompensation?.algorithm !== 'f32-dq-f64-remainder-v1') throw new Error('Native0 activation proof missing');
            }
            const root = fixture.fixtureRootRotationPolicy, solver = fixture.fixtureSolverBudget;
            if (!root || root.algorithm !== 'unit-free-root-v1' || root.enabled !== true || root.validationError !== null
              || !equal(root.root, [0, 0]) || !Number.isFinite(root.rootNormSquared) || Math.abs(root.rootNormSquared - 1) > .001) throw new Error('Actual unit-root policy/domain metadata missing');
            if (!solver || ![20, 32].includes(solver.currentInternalPgsIterations)
              || ![20, 32].includes(solver.selectedNextInternalPgsIterations) || typeof solver.sourceRecoveryActive !== 'boolean') throw new Error('Actual solver-budget metadata missing');
            if (solver.sourceRecoveryActive && snapshot.motion === 'Recovering' && solver.selectedNextInternalPgsIterations !== 32) throw new Error('Source recovery did not select32internal PGS');
            // A terminal latch can remain set while an ordinary strike perturbs
            // the body. Enforce terminal budget only when physically Upright.
            if (policy.prototype_standing_passed && snapshot.motion === 'Upright' && policy.stable_standing_s >= 1
              && (solver.sourceRecoveryActive || solver.currentInternalPgsIterations !== 20 || solver.selectedNextInternalPgsIterations !== 20)) throw new Error('Upright terminal standing must use20internal PGS');
            const epoch = fixture.fixtureRecoveryEpoch, current = fixture.currentStandingDiagnostic;
            if (!epoch || epoch.startedAtS !== policy.started_at_s || !Array.isArray(epoch.observedControllerStartChanges)
              || !Number.isInteger(epoch.ordinaryStrikeCommandsSeen)) throw new Error('Read-only source epoch/ordinary-Strike metadata missing');
            if (!current || current.physicsIntegrations !== 0 || current.nativeSubsteps !== native
              || !current.predicates || !Object.keys(current.predicates).length) throw new Error('Read-only current physical standing diagnostic missing');
            if (epoch.ordinaryStrikeCommandsSeen < 0 || epoch.ordinaryStrikeCommandsSeen > 1) throw new Error('Adapter observed more than one ordinary Strike');
            e.epochPublications.push({ native, time, ...structuredClone(epoch) });
            e.solverBudgetSamples.push({ native, time, motion: snapshot.motion, ...structuredClone(solver) });
            e.rootRotationPolicySamples.push({ native, time, ...structuredClone(root) });
            e.standingDiagnostics.push({ native, time, motion: snapshot.motion, current: structuredClone(current),
              original: structuredClone(fixture.originalAnchorStanding ?? null), postPass: structuredClone(fixture.postPassStanding ?? null) });
            if (message.poseBuffer instanceof ArrayBuffer) e.lastReceivedPackedPosePacket = { native, time,
              wordsUint32: Array.from(new Uint32Array(message.poseBuffer)), source: 'Received transfer buffer copied before unchanged driver recycles/detaches it; no physical write' };
            e.samples.push(structuredClone(snapshot));
            e.publications.push({ native, time, completedTicks: message.reply.completed_ticks, requestId: message.requestId,
              fixtureRecoveryEpoch: structuredClone(epoch), striker: structuredClone(snapshot.striker), counters: structuredClone(snapshot.counters) });
            for (const acknowledgement of message.reply.acknowledgements ?? []) e.acknowledgements.push(structuredClone(acknowledgement));
            if (e.motionTransitions.at(-1)?.motion !== snapshot.motion) e.motionTransitions.push({ native, time, motion: snapshot.motion, startedAtS: policy.started_at_s });
            if (!equal(e.counterTransitions.at(-1)?.counters, snapshot.counters)) e.counterTransitions.push({ native, time, counters: structuredClone(snapshot.counters) });
            if (policy.started_at_s !== null && e.initialSourceStartedAtS === null) e.initialSourceStartedAtS = policy.started_at_s;
            const impactNative = Number.isFinite(policy.since_impact_s) ? native - nativeAt(policy.since_impact_s) : null;
            if (impactNative !== null && (!Number.isSafeInteger(impactNative) || impactNative < 0 || impactNative > native))
              throw new Error('Invalid native impact clock from policy.since_impact_s');
            if (e.phase === 'initial-recovery' && policy.started_at_s !== null) {
              if (impactNative === null) throw new Error('Initial source since_impact_s clock missing');
              e.initialImpactNative ??= impactNative;
              if (impactNative !== e.initialImpactNative) throw new Error('Initial source impact clock changed');
            }
            if (e.firstNewSource && policy.started_at_s === e.firstNewSource.startedAtS
              && impactNative !== e.firstNewSource.impactNative) throw new Error('New source since_impact_s changed the fixed impact clock');
            e.recoveryImpactClocks.push({ native, time, startedAtS: policy.started_at_s, sinceImpactS: policy.since_impact_s, impactNative });
            if (!duplicate) e.lastDistinctPublication = { native, time, motion: snapshot.motion };
            if (e.startedAtMs === null || e.endedAtMs !== null) return;
            if (message.reply.failure) { e.stop('physical-or-resource-halt'); return; }
            if (e.phase === 'initial-recovery' && policy.prototype_standing_passed && policy.stable_standing_s >= 1 && snapshot.motion === 'Upright') {
              if (e.initialImpactNative === null || native > e.initialImpactNative + 25 * 800) { e.stop('initial-impact25s-deadline'); return; }
              e.initialFirstPass = { native, time, startedAtS: policy.started_at_s, counters: structuredClone(snapshot.counters) };
              e.checkpointAnchors.initial = structuredClone(fixture.originalAnchorStanding?.originalStableFeet);
              e.phase = 'initial-hold'; requestCapture('initial-first-pass');
            }
            if (e.phase === 'initial-recovery') {
              const impact = e.initialImpactNative ?? fixture.triggerNative;
              if (impact !== null && native >= impact + 25 * 800) e.stop('initial-impact25s-deadline');
              return;
            }
            if (e.phase === 'initial-hold') {
              checkHold('initial', e.initialHold, snapshot, fixture, message.reply.completed_ticks, e.initialFirstPass.startedAtS);
              if (e.endedAtMs !== null) return;
              if (native >= e.initialFirstPass.native + 3 * 800) {
                e.beforeStrike = { native, time, tick: snapshot.stamp.tick, sourceStartedAtS: policy.started_at_s,
                  counters: structuredClone(snapshot.counters), striker: structuredClone(snapshot.striker),
                  originalStableFeet: structuredClone(fixture.originalAnchorStanding.originalStableFeet) };
                e.phase = 'before-strike'; requestCapture('before-strike');
              }
              return;
            }
            if (!e.strikeInputRequested) return;
            const rawStriker = snapshot.striker;
            const impactId = rawStriker?.impactId ?? rawStriker?.impact_id;
            const impulse = rawStriker?.lastImpactImpulseNs ?? rawStriker?.last_impact_impulse_ns ?? rawStriker?.last_impulse_ns;
            const baselineImpact = e.beforeStrike.striker.impactId ?? e.beforeStrike.striker.impact_id;
            if (e.firstImpact === null && impactId > baselineImpact && impulse > 0) {
              if (impactId !== baselineImpact + 1 || snapshot.counters.strikes !== e.beforeStrike.counters.strikes + 1)
                throw new Error('One ordinary Strike must have exactly one corroborated solver impact');
              e.firstImpact = { firstObservedNative: native, firstObservedTimeS: time,
                actualImpactNativeInterval: { exclusiveLower: previous?.native ?? e.beforeStrike.native, inclusiveUpper: native },
                impact_id: impactId, last_impulse_ns: impulse, striker: structuredClone(rawStriker), counters: structuredClone(snapshot.counters),
                observationDeadlineNative: native + 30 * 800 };
            }
            const motionConfirmsFall = ['Falling', 'Fallen'].includes(snapshot.motion);
            const counterConfirmsFall = snapshot.counters.falls > e.beforeStrike.counters.falls;
            if (e.confirmedFall === null && (motionConfirmsFall || counterConfirmsFall)) e.confirmedFall = {
              native, time, motion: snapshot.motion, motionConfirmsFall, counterConfirmsFall, counters: structuredClone(snapshot.counters),
            };
            if (policy.started_at_s !== null && policy.started_at_s !== e.beforeStrike.sourceStartedAtS
              && e.sourceChanges.at(-1)?.startedAtS !== policy.started_at_s) {
              const source = { observedNative: native, observedTimeS: time, startedAtS: policy.started_at_s,
                sourceNative: nativeAt(policy.started_at_s), impactNative, recoveryDeadlineNative: impactNative + 25 * 800,
                metadata: structuredClone(epoch), counters: structuredClone(snapshot.counters), motion: snapshot.motion };
              if (impactNative === null) throw new Error('New controller source has no since_impact_s native impact clock');
              e.sourceChanges.push(source);
              if (e.firstNewSource === null) { e.firstNewSource = source; requestCapture('later-source-start'); }
              else if (e.secondFirstPass !== null) { e.stop('source-changed-during-earned-final-hold'); return; }
            }
            if (e.firstNewSource !== null && e.secondFirstPass === null && policy.prototype_standing_passed
              && policy.stable_standing_s >= 1 && snapshot.motion === 'Upright'
              && policy.started_at_s === e.firstNewSource.startedAtS) {
              if (native > e.firstNewSource.recoveryDeadlineNative) { e.stop('new-source25s-deadline'); return; }
              e.secondFirstPass = { native, time, startedAtS: policy.started_at_s, counters: structuredClone(snapshot.counters) };
              e.checkpointAnchors.final = structuredClone(fixture.originalAnchorStanding?.originalStableFeet);
              e.phase = 'final-hold'; requestCapture('second-first-pass');
            }
            if (e.phase === 'final-hold') {
              checkHold('final', e.finalHold, snapshot, fixture, message.reply.completed_ticks, e.secondFirstPass.startedAtS);
              if (e.endedAtMs !== null) return;
              if (native >= e.secondFirstPass.native + 3 * 800) e.stop('new-source-native-pass-plus3s-original-anchor-hold');
              return;
            }
            // Fix the first new source's actual impact deadline permanently.
            // Source start can follow contact; it must not extend the25s clock.
            if (e.firstNewSource !== null && native >= e.firstNewSource.recoveryDeadlineNative) { e.stop('new-source25s-deadline'); return; }
            if (e.confirmedFall !== null && e.firstNewSource === null
              && native >= (e.firstImpact?.actualImpactNativeInterval.exclusiveLower ?? e.beforeStrike.native) + 25 * 800) {
              e.stop('confirmed-fall-without-new-source-contact25s-bound'); return;
            }
            if (e.firstImpact !== null && e.confirmedFall === null && e.firstNewSource === null
              && native >= e.firstImpact.observationDeadlineNative) { e.stop('contacted-resisted30s-after-first-impact-publication'); return; }
            if (e.firstImpact === null) {
              const phase = rawStriker?.phase?.toLowerCase?.();
              const commandBoundary = e.strikeCommands[0]?.command.stamp.tick * 20;
              const progressed = Number.isFinite(commandBoundary) && native > commandBoundary;
              if (progressed && phase === 'idle' && rawStriker.available) e.stop('uncontacted-strike-apparatus-idle');
              else if (native >= e.beforeStrike.native + 5 * 800) e.stop('uncontacted-strike5s-bound');
            }
          } catch (error) { e.stop('instrumentation-error', error); }
        });
      }
      postMessage(message, ...args) {
        if (message.type === 'initialize') {
          message = { ...message, testFixedGetUpFixture: kind, testFixedGetUpProfile: approvedProfile };
          e.initializations.push(structuredClone(message));
        }
        if (message.type === 'advance' && message.ticks !== 1) e.stop('instrumentation-error', 'Automatic scheduler must advance exactly one ordinary tick');
        if (message.commands?.length) {
          e.commands.push(structuredClone(message.commands));
          for (const command of message.commands) {
            if (command.action !== 'Strike' || !e.strikeInputRequested || e.strikeCommands.length !== 0)
              e.stop('instrumentation-error', 'Only the one approved genuine ordinary Strike is allowed');
            else e.strikeCommands.push({ command: structuredClone(command), requestId: message.requestId, browserAtMs: performance.now() });
          }
        }
        return super.postMessage(message, ...args);
      }
    };
    document.addEventListener('pointerdown', event => {
      if (e.startedAtMs !== null && e.endedAtMs === null && event.target?.closest?.('#viewport'))
        e.physicalInputs.push({ type: event.type, atMs: event.timeStamp });
    }, true);
    for (const type of ['keydown', 'keyup']) window.addEventListener(type, event => {
      if (e.startedAtMs === null || e.endedAtMs !== null) return;
      e.keyboardInputs.push({ type, code: event.code, key: event.key, repeat: event.repeat, isTrusted: event.isTrusted,
        atMs: event.timeStamp, native: nativeAt(api.diagnostics.time), target: event.target?.id ?? event.target?.tagName });
      if (event.code !== 'KeyP' || event.repeat || !e.strikeInputRequested)
        e.stop('instrumentation-error', 'Unexpected physical keyboard input');
    }, true);
    document.addEventListener('DOMContentLoaded', () => {
      const label = document.createElement('div');
      Object.assign(label.style, { position: 'fixed', bottom: '8px', left: '8px', zIndex: '2147483647', pointerEvents: 'none',
        background: '#091424ed', color: '#fff', padding: '8px 12px', font: '12px monospace', whiteSpace: 'pre' });
      document.body.append(label);
      const draw = () => {
        const d = api?.diagnostics, p = d?.snapshot?.diagnostics.getup?.policy;
        label.textContent = `ISOLATED BELLY REPEATED-CYCLE TEST |800Hz physics /40Hz publication\ncarry + terminal-balance + standing-dynamics + unit-root + confirmed-fall repeated recovery\nNative1s latch +3physical s original-anchor hold; ONE ordinary keyboard Strike\nNative wall time; NO RETIMING / performance / global qualification claim\nsim ${(d?.time ?? 0).toFixed(3)}s | ${e.phase} | motion ${d?.motion ?? 'starting'} | source ${p?.started_at_s ?? 'pending'}\nstable ${(p?.stable_standing_s ?? 0).toFixed(3)}s | commands ${e.strikeCommands.length} | contact ${e.firstImpact?.impact_id ?? 'pending'}\n${e.stopReason ?? 'original automatic one-tick scheduler'}`;
        requestAnimationFrame(draw);
      };
      draw();
    });
  }, { kind, approvedProfile });
  page = await context.newPage(); video = page.video();
  page.on('pageerror', error => report.errors.push(String(error)));
  page.on('response', response => { if (response.status() >= 400) report.errors.push(`${response.status()} ${response.url()}`); });
  await page.goto(origin + '?renderer=webgl2');
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3,
    null, { timeout: 60000 });
  report.initial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(report.initial.tick, 0); assert.equal(report.initial.time, 0);
  assert.equal(report.initial.backend, 'WebGL2');
  const activation = await page.evaluate(() => {
    const e = window.__fixtureEvidence, a = e.activation;
    if (e.error) throw new Error(e.error);
    if (!a) throw new Error('Activation evidence not retained at initial packet');
    const encode = bytes => {
      let text = ''; for (let i = 0; i < bytes.length; i += 0x8000) text += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
      return btoa(text);
    };
    const beforeBase64 = encode(a.beforeCheckpoint), afterBase64 = encode(a.afterCheckpoint);
    delete a.beforeCheckpoint; delete a.afterCheckpoint;
    return { metadata: structuredClone(a), beforeBase64, afterBase64 };
  });
  const beforeActivation = Buffer.from(activation.beforeBase64, 'base64'), afterActivation = Buffer.from(activation.afterBase64, 'base64');
  report.activationProof = { ...activation.metadata, independentlyDecodedPurity: proveActivationPurity(beforeActivation, afterActivation) };
  assert.deepEqual(report.activationProof.poseVelocityF32WordsBefore, report.activationProof.poseVelocityF32WordsAfter);
  assert.deepEqual(report.activationProof.afterReport.profile, approvedProfile);
  assert.deepEqual(report.activationProof.beforeReport.rootRotationNormalization, report.activationProof.afterReport.rootRotationNormalization);
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.algorithm, 'unit-free-root-v1');
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.enabled, true);
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.validationError, null);
  assert.equal(report.activationProof.activationCalls, 1);
  for (const verification of [report.activationProof.beforeVerification, report.activationProof.afterVerification]) {
    assert.equal(verification.passed, true); assert.equal(verification.physicsIntegrations, 0);
    assert.equal(verification.nativeSubsteps, 0); assert.equal(verification.forecastSubsteps, 0);
    assert.equal(verification.rootIntegratorStateEqual, true);
  }
  for (const [label, bytes] of [['before', beforeActivation], ['after', afterActivation]]) {
    const path = join(output, `activation-${label}-checkpoint.bin`); writeFileSync(path, bytes);
    report.activationProof[`${label}Checkpoint`] = { path, sha256: hash(bytes), bytes: bytes.length };
  }
  writeFileSync(join(output, 'activation-proof.json'), JSON.stringify(report.activationProof, null, 2) + '\n');
  await page.locator('#camera-focus').click();
  const viewport = await page.locator('#viewport').boundingBox();
  assert.ok(viewport, 'Actual viewport required');
  await page.mouse.move(viewport.x + viewport.width / 2, viewport.y + viewport.height / 2);
  await page.mouse.wheel(0, 400); await page.waitForTimeout(150);
  // The unchanged driver's key handler deliberately ignores focused buttons.
  await page.locator('#simulation').focus();
  report.framing = { ordinaryUiOnly: true, cameraFocusBody: true, preStartWheelDeltaY: 400, keyboardTarget: 'simulation canvas' };
  report.framedInitial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(report.framedInitial.tick, 0); assert.equal(report.framedInitial.time, 0);
  await page.screenshot({ path: join(output, 'initial-fixed-belly-fixture.png'), fullPage: true });
  await page.evaluate(() => {
    const e = window.__fixtureEvidence;
    if (e.error || e.endedAtMs !== null) throw new Error(e.error || e.stopReason);
    window.__lhRust.drainTelemetry(); e.startedAtMs = performance.now();
    e.initialFrames = window.__lhRust.diagnostics.graphics.frames; window.__lhRust.setManual(false);
  });
  report.telemetry = { packets: [], frames: [], inputs: [], bounds: null };
  const watchdog = Date.now() + 30 * 60 * 1000;
  let nextProgressS = 0;
  while (true) {
    await page.waitForTimeout(1000);
    const update = await page.evaluate(() => {
      const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
      if (d.workerFault || d.disposed || d.halted || d.failure || d.error) e.stop(d.workerFault || d.disposed
        ? 'renderer-or-driver-terminated-worker' : 'physical-resource-or-renderer-halt', d.error || null);
      return { time: d.time, tick: d.tick, motion: d.motion, policy: d.snapshot?.diagnostics.getup?.policy, phase: e.phase,
        error: e.error || d.error, stopReason: e.stopReason, ended: e.endedAtMs !== null,
        pendingCaptures: [...e.pendingCaptures], strikeCommands: e.strikeCommands.length,
        wallSeconds: ((e.endedAtMs ?? performance.now()) - e.startedAtMs) / 1000, telemetry: window.__lhRust.drainTelemetry() };
    });
    const telemetry = update.telemetry;
    for (const name of ['packets', 'frames', 'inputs']) report.telemetry[name].push(...telemetry[name]);
    for (const name of ['bounds', 'droppedPackets', 'droppedFrames', 'droppedInputs', 'pendingVisualInputs']) report.telemetry[name] = telemetry[name];
    delete update.telemetry;
    if (update.time >= nextProgressS || update.ended || update.error || update.pendingCaptures.length) {
      report.progress.push(update); console.log(JSON.stringify({ finding: 'bounded-repeat-cycle-test-progress', ...update })); nextProgressS = update.time + 1;
    }
    if (update.ended || update.error) break; // Do not run queued exports on a failed/terminated world.
    if (report.errors.length) {
      await page.evaluate(message => window.__fixtureEvidence.stop('page-or-resource-error', message), report.errors.join('\n')); break;
    }
    for (const label of update.pendingCaptures) {
      const controller = await captureReadOnlyCheckpoint(label);
      if (!controller) throw new Error(`Read-only ${label} capture unavailable`);
      const expected = await page.evaluate(label => {
        const e = window.__fixtureEvidence;
        return label === 'initial-first-pass' ? e.checkpointAnchors.initial
          : label === 'second-first-pass' ? e.checkpointAnchors.final : label === 'before-strike' ? e.beforeStrike.originalStableFeet : null;
      }, label);
      if (expected !== null) assert.deepEqual(controller.stable_feet, expected, 'Read-only controller must retain its exact original stable_feet');
      if (label === 'later-source-start' || label === 'second-first-pass') {
        const source = await page.evaluate(() => window.__fixtureEvidence.firstNewSource);
        assert.ok(source, 'Later source checkpoint requires an observed new source');
        assert.equal(controller.impact_substep, source.impactNative, 'Native controller impact_substep must corroborate since_impact_s, without moving the deadline');
        assert.equal(controller.status.started_at_s, source.startedAtS, 'Checkpoint must belong to the new source epoch');
        report.checkpointCaptures[label].corroboratedImpactNative = controller.impact_substep;
      }
      await page.evaluate(label => {
        const e = window.__fixtureEvidence;
        e.pendingCaptures = e.pendingCaptures.filter(value => value !== label); e.savedCaptures.push(label);
      }, label);
      if (label === 'before-strike') {
        await page.screenshot({ path: join(output, 'before-one-ordinary-strike.png'), fullPage: true });
        await page.locator('#simulation').focus();
        report.keyboardBoundary = await page.evaluate(() => {
          const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
          if (e.error || e.endedAtMs !== null || d.workerFault || d.disposed || d.outstanding || !d.canStrike)
            throw new Error('Genuine ordinary Strike unavailable at the frozen first-pass+3s boundary');
          if (d.time !== e.beforeStrike.time || d.tick !== e.beforeStrike.tick || e.strikeInputRequested) throw new Error('Before-strike boundary advanced or input repeated');
          e.strikeInputRequested = true; e.phase = 'post-strike';
          return e.strikeInputBoundary = { native: Math.round(d.time * 800), time: d.time, tick: d.tick,
            browserAtMs: performance.now(), method: "page.keyboard.press('KeyP')", directApiAction: false, directWorkerCommand: false };
        });
        // This is the sole physical input. Do not replace it with api.action,
        // qualificationCommand, Worker.postMessage, reset, or a synthetic event.
        await page.keyboard.press('KeyP');
        await page.waitForFunction(() => {
          const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
          return e.error || d.workerFault || d.disposed || (e.strikeCommands.length === 1 && !d.outstanding);
        }, null, { timeout: 100000 });
        const boundary = await page.evaluate(() => ({ evidence: {
          error: window.__fixtureEvidence.error, commands: window.__fixtureEvidence.strikeCommands,
          acknowledgements: window.__fixtureEvidence.acknowledgements, keyboardInputs: window.__fixtureEvidence.keyboardInputs,
        }, diagnostics: window.__lhRust.diagnostics }));
        report.ordinaryStrikeBoundary = boundary;
        assert.equal(boundary.evidence.error, null);
        assert.equal(boundary.diagnostics.workerFault, false); assert.equal(boundary.diagnostics.disposed, false);
        assert.equal(boundary.evidence.commands.length, 1);
        const command = boundary.evidence.commands[0].command;
        assert.equal(command.action, 'Strike'); assert.equal(command.stamp.schema, 4);
        assert.equal(command.stamp.generation, 1); assert.equal(command.stamp.tick, report.keyboardBoundary.tick);
        const acknowledgement = boundary.evidence.acknowledgements.find(value => value.stamp.sequence === command.stamp.sequence);
        assert.ok(acknowledgement, 'Real ordinary Strike acknowledgement required');
        assert.equal(acknowledgement.rejected, null);
      }
    }
    await page.evaluate(() => {
      const e = window.__fixtureEvidence;
      if (!e.error && e.endedAtMs === null && e.pendingCaptures.length === 0) window.__lhRust.setManual(false);
    });
    if (Date.now() >= watchdog) {
      await page.evaluate(() => window.__fixtureEvidence.stop('native-wall-time-watchdog', 'Thirty-minute wall watchdog exceeded')); break;
    }
  }
  await page.evaluate(() => window.__lhRust.setManual(true));
  const finalAvailability = await page.evaluate(() => {
    const d = window.__lhRust.diagnostics, e = window.__fixtureEvidence;
    return { workerFault: d.workerFault, disposed: d.disposed, observerError: e.error, time: d.time, error: d.error };
  });
  if (finalAvailability.workerFault || finalAvailability.disposed || finalAvailability.observerError)
    report.stopCheckpointUnavailable = { ...finalAvailability, reason: 'Known fault: skip getupReport/getupCheckpoint and pending captures; retain raw result.' };
  else await captureReadOnlyCheckpoint('stop');
  report.final = await page.evaluate(() => window.__lhRust.diagnostics);
  report.evidence = await page.evaluate(() => {
    const { stop, ...data } = window.__fixtureEvidence; return data;
  });
  const e = report.evidence, finalSnapshot = report.final.snapshot;
  report.fullSourceProfile = JSON.parse(e.profile);
  assert.equal(e.workerInstances, 1); assert.equal(e.workerIdentities.length, 1); assert.equal(e.initializations.length, 1);
  assert.equal(e.initializations[0].profile, undefined, 'Ordinary driver profile path unchanged');
  assert.deepEqual(e.initializations[0].testFixedGetUpProfile, approvedProfile);
  assert.ok(e.typedProfilePublications.length === e.samples.length && e.typedProfilePublications.every(value => sameJson(value.profile, approvedProfile)), 'Exact actual typed profile required on every publication');
  assert.deepEqual(e.physicalInputs, []);
  assert.ok(e.trigger?.physicalPoseVelocityBitsUnchanged); assert.deepEqual(e.trigger.beforePoseWords, e.trigger.afterPoseWords);
  assert.equal(e.trigger.beforePoseWords.length, 325);
  if (e.strikeInputRequested) {
    assert.equal(e.commands.flat().length, 1); assert.equal(e.strikeCommands.length, 1);
    const keydown = e.keyboardInputs.filter(value => value.type === 'keydown');
    assert.equal(keydown.length, 1); assert.equal(keydown[0].code, 'KeyP'); assert.equal(keydown[0].isTrusted, true);
    assert.equal(e.epochPublications.at(-1).ordinaryStrikeCommandsSeen, 1);
  }
  const finalStandingObservation = e.standingDiagnostics.at(-1);
  const current = finalStandingObservation?.current, original = finalStandingObservation?.original;
  // The null-anchor query intentionally reports a null supplied-anchor drift
  // predicate. Only actual controller stable_feet can establish final standing.
  const failedFinalPredicates = original
    ? Object.entries(original.predicates).filter(([, value]) => value !== true).map(([key]) => key)
    : ['native-original-anchor-diagnostic-unavailable'];
  const finalPolicy = finalSnapshot?.diagnostics.getup?.policy;
  report.finalPhysicalStanding = { evaluatedAtNative: Math.round(report.final.time * 800), motion: report.final.motion,
    nativeStableStandingS: finalPolicy?.stable_standing_s, nativeStandingLatch: finalPolicy?.prototype_standing_passed,
    predicateSource: original ? 'actual-controller-original-stable_feet' : 'unavailable-without-earned-native-stability',
    originalAnchorPredicates: original?.predicates, originalStableFeet: original?.originalStableFeet,
    currentRawPredicates: current?.predicates, failedPredicates: failedFinalPredicates,
    passed: failedFinalPredicates.length === 0 && report.final.motion === 'Upright'
      && finalPolicy?.prototype_standing_passed === true && finalPolicy?.stable_standing_s >= 1,
    continuousStandingAfterPerturbationClaim: false,
    note: 'Current physical standing is evaluated separately. The30s resisted-contact observation is not a30s continuous standing qualification.' };
  report.initialHoldCompleted = Boolean(e.initialFirstPass && e.beforeStrike && e.beforeStrike.native - e.initialFirstPass.native >= 2400
    && e.initialHold.filter(value => !value.duplicate).length >= 121 && e.initialHold.every(value => value.passed));
  report.repeatedRecoveryDemonstrated = Boolean(e.firstImpact && e.confirmedFall && e.firstNewSource && e.secondFirstPass
    && e.secondFirstPass.native <= e.firstNewSource.recoveryDeadlineNative
    && finalSnapshot.counters.recoveries === e.beforeStrike.counters.recoveries + 1 && e.sourceChanges.length === 1
    && e.finalHold.filter(value => !value.duplicate).length >= 121 && e.finalHold.every(value => value.passed)
    && e.stopReason === 'new-source-native-pass-plus3s-original-anchor-hold');
  if (report.repeatedRecoveryDemonstrated) report.outcome = 'contacted-confirmed-fall-new-source-recovery-plus3s-original-anchor-hold';
  else if (e.stopReason === 'contacted-resisted30s-after-first-impact-publication' && e.firstImpact && !e.confirmedFall && !e.firstNewSource)
    report.outcome = 'contacted-resisted-no-confirmed-fall-or-new-source';
  else if (e.stopReason?.startsWith('uncontacted-strike')) report.outcome = 'uncontacted-strike-observation-only';
  else report.outcome = 'bounded-failure-or-incomplete-recovery';
  report.observationComplete = report.repeatedRecoveryDemonstrated
    || report.outcome === 'contacted-resisted-no-confirmed-fall-or-new-source'
    || report.outcome === 'uncontacted-strike-observation-only';
  report.diagnosticCaptureComplete = Boolean(report.checkpointCaptures['initial-first-pass']?.verification?.passed
    && report.checkpointCaptures['before-strike']?.verification?.passed && report.checkpointCaptures.stop?.verification?.passed
    && (!report.repeatedRecoveryDemonstrated || report.checkpointCaptures['second-first-pass']?.verification?.passed));
  report.passed = report.initialHoldCompleted && report.diagnosticCaptureComplete && e.error === null && report.errors.length === 0
    && !report.final.failure && !report.final.error && !report.final.workerFault && !report.final.disposed
    && e.standingViolations.length === 0 && report.finalPhysicalStanding.passed
    && (report.repeatedRecoveryDemonstrated || report.outcome === 'contacted-resisted-no-confirmed-fall-or-new-source');
  report.observation = { simulationSeconds: report.final.time, wallSeconds: (e.endedAtMs - e.startedAtMs) / 1000,
    presentedFrames: e.endedFrames - e.initialFrames,
    contactedObservationSecondsAfterFirstImpactPublication: e.firstImpact ? report.final.time - e.firstImpact.firstObservedTimeS : null,
    note: 'Native wall-time recording; no retiming, realtime/mobile performance claim, or native800Hz global qualification.' };
  await page.screenshot({ path: join(output, 'bounded-repeat-cycle-stop.png'), fullPage: true });
} catch (error) {
  report.error = error.stack ?? String(error);
  if (page && !page.isClosed()) {
    try {
      await page.evaluate(message => window.__fixtureEvidence?.stop('recorder-error', message), String(error));
      report.failureState = await page.evaluate(() => window.__lhRust?.diagnostics);
      report.evidence = await page.evaluate(() => { const { stop, ...data } = window.__fixtureEvidence ?? {}; return data; });
      await page.screenshot({ path: join(output, 'failure.png'), fullPage: true });
    } catch (captureError) { report.rawFailureCaptureError = String(captureError); }
  }
} finally {
  report.finished = new Date().toISOString();
  // Persist raw evidence before encoder/context cleanup, including failed runs.
  writeFileSync(join(output, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  try {
    await context?.close();
    if (video) {
      await video.saveAs(join(output, 'repeat-cycle-native-wall-time.webm'));
      report.recording = { path: 'repeat-cycle-native-wall-time.webm', retimed: false };
    }
  } catch (error) { report.recordingError = String(error); report.passed = false; }
  try { await browser?.close(); } catch (error) { report.browserCloseError = String(error); }
  server?.kill();
  writeFileSync(join(output, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({ passed: report.passed, observationComplete: report.observationComplete,
    outcome: report.outcome, repeatedRecoveryDemonstrated: report.repeatedRecoveryDemonstrated,
    error: report.error, finalTime: report.final?.time ?? report.failureState?.time,
    stopReason: report.evidence?.stopReason, output }, null, 2));
  process.exitCode = report.passed ? 0 : 1;
}
