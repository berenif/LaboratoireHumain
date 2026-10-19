// SOURCE-ONLY preparation. Execute only after independent source/native approval.
// One fresh fixture, unchanged driver/renderer/automatic one-tick integration.
// Read-only publication/checkpoint observers never force the optional native replan.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, symlinkSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { pathToFileURL } from 'node:url';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';

assert.ok(process.argv[2] && process.argv[3] && process.argv[4], 'Usage: recorder ARTIFACT FRESH_OUTPUT back|belly|side|side-supported [thirty-second-standing-observation]');
assert.ok(process.argv.length <= 6, 'Unexpected arguments');
const artifact = resolve(process.argv[2]), output = resolve(process.argv[3]), kind = process.argv[4];
const mode = process.argv[5] ?? 'thirty-second-standing-observation';
assert.equal(mode, 'thirty-second-standing-observation', 'Unchanged native1s pass plus30physical-second hold required');
assert.ok(['back', 'belly', 'side', 'side-supported'].includes(kind), 'Exactly one explicit fresh fixture required');
assert.ok(output !== artifact && !output.startsWith(artifact + sep), 'Evidence must not modify the frozen app');
assert.ok(!existsSync(output) || readdirSync(output).length === 0, 'Fresh empty output required');
mkdirSync(output, { recursive: true });
const recordVideo = process.env.RUST_BROWSER_NO_VIDEO !== '1';
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
  ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
const sameJson = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
function hashes(root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path)) : [[relative(artifact, path), hash(readFileSync(path))]];
  }));
}
const buildFeatures = ['lh-worker/root-translation-carry-experiment', 'lh-sim/standing-dynamics-experiment',
  'lh-sim/unit-root-rotation-experiment', 'lh-sim/getup-terminal-replan-experiment'];
const effectiveSimFeatures = ['root-translation-carry-experiment', 'getup-terminal-balance-experiment',
  'standing-dynamics-experiment', 'unit-root-rotation-experiment', 'getup-repeat-cycle-experiment', 'getup-terminal-replan-experiment'];
const profilePath = resolve('docs/evidence/profiles/getup-800hz-3plus1-one-terminal-replan-v1.json');
const profileSha256 = '9193400c547b0d3457e4cd8f8e609932c245b0b2a26fe037410451dba4e350e3';
const report = {
  at: new Date().toISOString(), passed: false, thirtySecondStandingObserved: false, qualificationClaim: false,
  kind, mode, artifact, recordVideo, selectedGetUpSolverBudget: 'consistent-recovery-phase-v1',
  rootRotationNormalization: 'unit-free-root-v1', repeatedRecoveryPolicy: 'completed-cycle-one-terminal-reference-replan-v1',
  experimentalFeatures: buildFeatures, requiredEffectiveSimFeatures: effectiveSimFeatures,
  experimentalDisposition: 'Bounded experimental one-terminal-reference replan, at most once per physical trial. No performance, global recovery, or release admission.',
  scope: 'One isolated fresh fixed-fixture TEST, complete explicit3+1 profile,800Hz physics/40Hz publications. '
    + 'Native0 carry activation once; original back/belly80-native or side20-native preparation polling and trigger. '
    + 'Original driver/renderer/automatic one-tick scheduler. No physical commands, inputs, resets, injected support, retiming, or profile changes. '
    + 'Original impact25s deadline, native1s standing latch, then30physical seconds continuous original-controller-anchor/counter observation.',
  observationRules: { nativeHz: 800, publicationHz: 40, maximumRecoveryPhysicalSeconds: 25, requiredPostPassPhysicalSeconds: 30,
    nativeReplanGate: 'Production feature alone requires240consecutive real settled native samples, max body linear0.65m/s/angular1.8rad/s, nonfoot load>=3N, low pelvis or tilted torso.',
    browserGateClaim: false, originalImpactDeadlineNeverReset: true,
    replanCheckpointBoundary: 'First publication with used=true; actual native attempt may precede this publication by0..19substeps. No immediate pre/post-replan physical-bit claim.' },
  performanceContext: process.env.FIXTURE_PERFORMANCE_CONTEXT ?? 'Physical-only bounded replan TEST; no performance qualification',
  errors: [], progress: [], diagnosticReplay: false, checkpointCaptures: {},
};
let server, browser, context, page, video;

// The observer has already paused scheduling before the unchanged driver pump.
// Known worker/disposal/observer faults bypass all exports and pending captures.
async function captureReadOnlyCheckpoint(label) {
  await page.waitForFunction(() => {
    const d = window.__lhRust?.diagnostics, e = window.__fixtureEvidence;
    return d && (!d.outstanding || d.workerFault || d.disposed || e?.error);
  }, null, { timeout: 100000 });
  const captured = await page.evaluate(async () => {
    const api = window.__lhRust, e = window.__fixtureEvidence;
    const unavailable = () => {
      const d = api.diagnostics;
      return d.workerFault || d.disposed || e.error ? { unavailable: true, workerFault: Boolean(d.workerFault),
        disposed: Boolean(d.disposed), observerError: e.error, error: e.error || d.error, time: d.time } : null;
    };
    const initialFault = unavailable();
    if (initialFault) return initialFault;
    const before = structuredClone(api.diagnostics);
    // Also settle the recorder's wait if the original driver terminates a worker
    // while a read-only request is in flight. Do not resurrect or retry exports.
    const guarded = async request => {
      const fault = unavailable(); if (fault) return fault;
      let interval, timeout;
      const failure = new Promise((resolveFault, reject) => {
        interval = setInterval(() => { const fault = unavailable(); if (fault) resolveFault(fault); }, 50);
        // This timer only bounds an evidence export, never a physical deadline.
        timeout = setTimeout(() => reject(new Error('Read-only export wall watchdog exceeded')), 100000);
      });
      try { return await Promise.race([request(), failure]); }
      finally { clearInterval(interval); clearTimeout(timeout); }
    };
    const controller = await guarded(() => api.getupReport());
    if (controller?.unavailable) return controller;
    const betweenFault = unavailable(); if (betweenFault) return betweenFault;
    const checkpoint = await guarded(() => api.getupCheckpoint());
    if (checkpoint?.unavailable) return checkpoint;
    const afterFault = unavailable(); if (afterFault) return afterFault;
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
      expectedReplanState: structuredClone(e.terminalReferenceReplanSamples.at(-1)?.state),
      expectedOriginalImpactNative: e.originalImpactNative,
      time: before.time, tick: before.tick, cachedPoseVelocityWordsUnchanged: true };
  });
  if (captured.unavailable) {
    report.checkpointCaptures[label] = { ...captured,
      reason: 'Known original-worker/disposal/observer fault: no export, resurrection, or pending capture retry.' };
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
    readOnlyGuarantee: 'Immutable native export/verification reports0integrations, unchanged native clock/root-integrator state. Cached325pose words are supplementary only.' };
  assert.equal(envelope.schema, 2, 'Active carry checkpoint must preserve schema2 history');
  assert.equal(envelope.wrapper.format, 'f32-dq-f64-remainder-v1');
  assert.equal(envelope.wrapper.roots.length, 1, 'Active carry root must persist in every checkpoint');
  assert.deepEqual(envelope.wrapper.roots[0].root, [0, 0]);
  report.checkpointCaptures[label].activeCarryRoot = [0, 0];
  assert.deepEqual(captured.controller.terminal_reference_replan, captured.expectedReplanState, 'Read-only controller export must match actual publication replan state');
  assert.equal(captured.controller.impact_substep, captured.expectedOriginalImpactNative, 'Read-only controller export must preserve original physical impact epoch');
  assert.equal(captured.verification.passed, true);
  assert.equal(captured.verification.physicsIntegrations, 0);
  assert.equal(captured.verification.rootIntegratorStateEqual, true);
  assert.equal(captured.verification.nativeSubsteps, Math.round(captured.time * 800));
  return captured.controller;
}

try {
  report.artifactSha256 = hashes(artifact);
  const adapterManifestPath = artifact + '-fixture-adapter-manifest.json';
  const adapterManifest = JSON.parse(readFileSync(adapterManifestPath));
  assert.deepEqual(report.artifactSha256, adapterManifest.artifactSha256, 'Every frozen app byte must match the adapter manifest');
  assert.deepEqual([...adapterManifest.sourceBuildFeatures].sort(), [...buildFeatures].sort(), 'Exactly the four approved build flags required');
  assert.equal(adapterManifest.adapterSha256, report.artifactSha256['sim-worker.js']);
  assert.equal(adapterManifest.isolatedTestOnly, true);
  const approvedProfileBytes = readFileSync(profilePath), approvedProfile = JSON.parse(approvedProfileBytes);
  assert.equal(hash(approvedProfileBytes), profileSha256, 'Exact approved complete profile document required');
  assert.equal(adapterManifest.explicitProfile.sha256, profileSha256);
  assert.deepEqual(adapterManifest.explicitProfile.value, approvedProfile);
  assert.equal(approvedProfile.id, 'getup-800hz-3plus1-one-terminal-replan-v1');
  assert.equal(approvedProfile.getup_repeated_recovery, 'completed-cycle-one-terminal-reference-replan-v1');
  assert.equal(approvedProfile.getup_solver_budget, 'consistent-recovery-phase-v1');
  assert.equal(approvedProfile.root_rotation_normalization, 'unit-free-root-v1');
  assert.equal(approvedProfile.dt_s, 1 / 800); assert.equal(approvedProfile.tick_substeps, 20);
  assert.equal(approvedProfile.solver_iterations, 3); assert.equal(approvedProfile.additional_body_iterations, 1);
  assert.equal(approvedProfile.getup_policy_enabled, true);
  const repeatBaselinePath = resolve('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json');
  const repeatBaselineBytes = readFileSync(repeatBaselinePath), repeatBaseline = JSON.parse(repeatBaselineBytes);
  assert.equal(hash(repeatBaselineBytes), 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
  assert.deepEqual({ ...approvedProfile, id: repeatBaseline.id, getup_repeated_recovery: repeatBaseline.getup_repeated_recovery }, repeatBaseline,
    'Only id and typed repeated-recovery enum may differ from repeat baseline');
  report.explicitProfile = structuredClone(approvedProfile);
  report.approvedProfileDocument = { path: profilePath, sha256: profileSha256 };
  report.onlyTypedDifferencesFromRepeatBaseline = ['id', 'getup_repeated_recovery'];
  report.adapterManifest = { path: adapterManifestPath, sha256: hash(readFileSync(adapterManifestPath)) };
  const buildManifestPath = resolve(adapterManifest.approvedFrozenManifest.path), buildManifestBytes = readFileSync(buildManifestPath);
  assert.equal(hash(buildManifestBytes), adapterManifest.approvedFrozenManifest.sha256, 'Referenced frozen build manifest must match');
  const buildManifest = JSON.parse(buildManifestBytes);
  assert.deepEqual([...buildManifest.extraFeatures].sort(), [...buildFeatures].sort());
  assert.equal(buildManifest.target, 'wasm32-unknown-unknown');
  assert.deepEqual(buildManifest.rustFlags, [], 'No target/numerical flags permitted');
  assert.equal(buildManifest.productionDefaultsChanged, false);
  assert.deepEqual(Object.keys(buildManifest.artifactSha256).sort(), Object.keys(report.artifactSha256).sort());
  for (const [path, sha256] of Object.entries(buildManifest.artifactSha256)) {
    if (path !== 'sim-worker.js') assert.equal(report.artifactSha256[path], sha256, `Only the fixture adapter may differ: ${path}`);
  }
  assert.equal(adapterManifest.originalWorkerTransportSha256, buildManifest.artifactSha256['sim-worker.js']);
  report.frozenBuildManifest = { path: buildManifestPath, sha256: hash(buildManifestBytes) };
  const sourceManifestPath = resolve(buildManifest.sourceManifest.path), sourceManifestBytes = readFileSync(sourceManifestPath);
  assert.equal(hash(sourceManifestBytes), buildManifest.sourceManifest.sha256, 'Referenced source freeze must match');
  const sourceManifest = JSON.parse(sourceManifestBytes), frozenSourceRoot = join(dirname(sourceManifestPath), 'source');
  assert.deepEqual([...sourceManifest.extraFeatures].sort(), [...buildFeatures].sort());
  assert.equal(sourceManifest.baseReadyAndBumplessEnabled, false, 'Base-ready and bumpless must stay OFF');
  assert.equal(sourceManifest.profileSha256, profileSha256);
  for (const [path, sha256] of Object.entries(sourceManifest.sourceFiles)) {
    assert.equal(hash(readFileSync(join(frozenSourceRoot, path))), sha256, `Frozen source byte mismatch: ${path}`);
  }
  const fingerprint = buildManifest.simFingerprints.find(entry => sameJson([...JSON.parse(entry.features)].sort(), [...effectiveSimFeatures].sort()));
  assert.ok(fingerprint, 'Exactly the six effective sim features, including transitive repeat/balance, must be attested');
  assert.deepEqual(fingerprint.rustflags, []);
  const fingerprintBytes = readFileSync(resolve(fingerprint.path));
  assert.equal(hash(fingerprintBytes), fingerprint.sha256);
  const actualFingerprint = JSON.parse(fingerprintBytes);
  assert.deepEqual([...JSON.parse(actualFingerprint.features)].sort(), [...effectiveSimFeatures].sort());
  assert.deepEqual(actualFingerprint.rustflags, []);
  report.frozenSourceManifest = { path: sourceManifestPath, sha256: hash(sourceManifestBytes),
    sourceFilesVerified: Object.keys(sourceManifest.sourceFiles).length, baseReadyAndBumplessEnabled: false };
  report.effectiveSimFingerprint = structuredClone(fingerprint);
  const driverPaths = Object.keys(report.artifactSha256).filter(path => path.endsWith('/driver.js'));
  assert.equal(driverPaths.length, 1, 'Exactly one original browser driver required');
  const originalDriverSha256 = hash(readFileSync(resolve('rust/crates/browser/driver.js')));
  assert.equal(report.artifactSha256[driverPaths[0]], originalDriverSha256, 'Unchanged original browser driver required');
  report.originalDriver = { artifactPath: driverPaths[0], sha256: originalDriverSha256, byteEqual: true };
  if (recordVideo && process.env.RUST_BROWSER_FFMPEG) {
    const require = createRequire(import.meta.url), core = dirname(require.resolve('playwright-core/package.json'));
    const descriptor = JSON.parse(readFileSync(join(core, 'browsers.json'))).browsers.find(x => x.name === 'ffmpeg');
    const tools = join(output, 'playwright-tools'), executable = join(tools, `ffmpeg-${descriptor.revision}`, 'ffmpeg-linux');
    mkdirSync(dirname(executable), { recursive: true });
    symlinkSync(resolve(process.env.RUST_BROWSER_FFMPEG), executable);
    process.env.PLAYWRIGHT_BROWSERS_PATH = tools;
  }
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
    const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
      ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
    const equal = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
    const nativeAt = time => Math.round(time * 800);
    const e = window.__fixtureEvidence = {
      kind, workerInstances: 0, workerIdentities: [], initializations: [], samples: [], publications: [],
      commands: [], physicalInputs: [], resetRequests: [], trigger: null, originalTriggerNative: null,
      profile: null, typedProfilePublications: [], activation: null, startedAtMs: null, endedAtMs: null,
      initialFrames: null, endedFrames: null, stopReason: null, error: null, rawWorkerFaults: [], rawObserverFault: null,
      solverBudgetSamples: [], rootRotationPolicySamples: [], terminalReferenceReplanSamples: [],
      replanObserved: null, replanUsedTransitions: 0, replanConsumedState: null, previousReplanUsed: null,
      policyPublications: [], recoveryImpactClocks: [], originalImpactNative: null, controllerStartChanges: [],
      lastControllerStartedAtS: undefined, lastReceivedPackedPosePacket: null,
      firstPassTimeS: null, firstPass: null, checkpointAnchors: null, historicalDiagnostics: [],
      standingDiagnostics: [], postPassDiagnostics: [], standingViolations: [], postPassContinuity: [],
      distinctPostPassSamples: 0, duplicatePostPassPackets: 0,
      pendingCaptures: [], savedCaptures: [], skippedCaptures: [], lastDistinctPublication: null,
      distinctPublications: 0, duplicatePublications: 0,
      stop(reason, error = null) {
        if (error !== null) this.error ??= String(error);
        if (this.endedAtMs === null) {
          this.stopReason = reason; this.endedAtMs = performance.now(); this.endedFrames = api?.diagnostics.graphics?.frames ?? null;
        }
        if (this.error || api?.diagnostics.workerFault || api?.diagnostics.disposed) this.skippedCaptures.push(...this.pendingCaptures.splice(0));
        api?.setManual(true);
      },
    };
    const requestCapture = (label, native) => {
      if (e.error || e.savedCaptures.includes(label) || e.pendingCaptures.some(value => value.label === label)) return;
      e.pendingCaptures.push({ label, native });
      api.setManual(true); // Observer runs before original driver's onmessage/pump.
    };
    const validateReplan = (value, native) => {
      if (!value || !equal(Object.keys(value).sort(), ['maximumReplansPerPhysicalTrial', 'state'].sort())
        || value.maximumReplansPerPhysicalTrial !== 1 || !value.state) throw new Error('Actual typed terminal replan metadata missing');
      const state = value.state;
      if (!equal(Object.keys(state).sort(), ['format', 'used', 'settled_native_samples', 'attempted_native_substep', 'prior_forecast_substeps'].sort())
        || state.format !== 'one-terminal-reference-fall-replan-v1' || typeof state.used !== 'boolean'
        || !Number.isSafeInteger(state.settled_native_samples) || state.settled_native_samples < 0
        || !Number.isSafeInteger(state.prior_forecast_substeps) || state.prior_forecast_substeps < 0) throw new Error('Malformed actual terminal replan state');
      if (state.used ? state.settled_native_samples !== 0 || !Number.isSafeInteger(state.attempted_native_substep)
        || state.attempted_native_substep < 0 || state.attempted_native_substep > native
        || state.prior_forecast_substeps <= 0 || state.prior_forecast_substeps > 250 * 20
        : state.settled_native_samples >= 240 || state.attempted_native_substep !== null || state.prior_forecast_substeps !== 0) {
        throw new Error('Actual terminal replan state violates the one-budget native contract');
      }
      return state;
    };
    const Native = window.Worker;
    window.Worker = class extends Native {
      constructor(...args) {
        super(...args); e.workerInstances++;
        this.addEventListener('message', event => {
          const message = event.data;
          try {
            if (message.type === 'ready') e.workerIdentities.push(message.workerIdentity);
            if (message.type === 'error') { e.rawWorkerFaults.push(structuredClone(message)); e.stop('worker-error', message.message); return; }
            if (!message.reply) return;
            const fixture = message.fixtureTest, snapshot = message.reply.snapshot;
            if (!fixture?.isolatedTestOnly || fixture.kind !== kind) throw new Error('Explicit single fresh fixture metadata missing');
            if (fixture.preparationPollingNative !== (kind.startsWith('side') ? 20 : 80)) throw new Error('Original fixture preparation polling changed');
            if (!equal(fixture.profile, approvedProfile)) throw new Error('Actual entire typed profile differs from approved document');
            e.profile = JSON.stringify(fixture.profile);
            const time = snapshot.simulation_time_s, native = nativeAt(time), policy = snapshot.diagnostics.getup?.policy;
            if (!Number.isSafeInteger(native) || time !== native / 800 || native < 0) throw new Error('Invalid native physical clock');
            if (!snapshot.diagnostics.getup?.learned_policy_loaded || !policy) throw new Error('Explicit learned prototype missing');
            if (snapshot.stamp.generation !== 1) throw new Error('Fixture generation changed');
            const previous = e.lastDistinctPublication, duplicate = previous?.native === native;
            if (previous && !(duplicate ? message.reply.completed_ticks === 0
              : native - previous.native === 20 && message.reply.completed_ticks === 1)) throw new Error('Original automatic one-tick publication sequence changed');
            if (duplicate) e.duplicatePublications++; else e.distinctPublications++;
            e.typedProfilePublications.push({ native, time, profile: structuredClone(fixture.profile) });
            if (fixture.triggerEvidence) {
              if (e.trigger !== null) throw new Error('Original fixture triggered more than once');
              e.trigger = structuredClone(fixture.triggerEvidence);
              if (!Number.isSafeInteger(e.trigger.native) || e.trigger.native <= 0
                || e.trigger.native % fixture.preparationPollingNative !== 0) throw new Error('Original trigger polling epoch invalid');
            }
            if (fixture.triggerNative !== null) {
              if (!Number.isSafeInteger(fixture.triggerNative) || fixture.triggerNative < 0 || fixture.triggerNative > native) throw new Error('Invalid fixture trigger epoch');
              e.originalTriggerNative ??= fixture.triggerNative;
              e.originalImpactNative ??= fixture.triggerNative;
              if (fixture.triggerNative !== e.originalTriggerNative || fixture.triggerNative !== e.originalImpactNative
                || e.trigger?.native !== fixture.triggerNative) throw new Error('Original physical trigger/impact epoch changed across controller replan');
            } else if (e.originalTriggerNative !== null) throw new Error('Original physical trigger epoch erased');
            if (fixture.carryActivationEvidence) {
              if (e.activation !== null) throw new Error('Native0 carry activation repeated');
              e.activation = structuredClone(fixture.carryActivationEvidence);
              if (e.activation.activationCalls !== 1 || e.activation.beforeReport.nativeSubsteps !== 0
                || e.activation.afterReport.nativeSubsteps !== 0
                || e.activation.afterReport.rootTranslationCompensation?.algorithm !== 'f32-dq-f64-remainder-v1') throw new Error('Native0 carry activation proof missing');
              const beforeReplan = e.activation.beforeReport.terminalReferenceReplan;
              const afterReplan = e.activation.afterReport.terminalReferenceReplan;
              const state = validateReplan(beforeReplan, 0);
              validateReplan(afterReplan, 0);
              if (!equal(beforeReplan, afterReplan) || state.used || state.settled_native_samples !== 0) throw new Error('Native0 carry activation altered terminal replan budget');
            }
            const root = fixture.fixtureRootRotationPolicy, solver = fixture.fixtureSolverBudget;
            if (!root || root.algorithm !== 'unit-free-root-v1' || root.enabled !== true || root.validationError !== null
              || !equal(root.root, [0, 0]) || !Number.isFinite(root.rootNormSquared) || Math.abs(root.rootNormSquared - 1) > .001) throw new Error('Actual unit-root policy/domain metadata missing');
            if (!solver || ![20, 32].includes(solver.currentInternalPgsIterations)
              || ![20, 32].includes(solver.selectedNextInternalPgsIterations) || typeof solver.sourceRecoveryActive !== 'boolean') throw new Error('Actual solver-budget metadata missing');
            if (solver.sourceRecoveryActive && snapshot.motion === 'Recovering' && solver.selectedNextInternalPgsIterations !== 32) throw new Error('Source recovery did not select32internal PGS');
            if (policy.prototype_standing_passed && snapshot.motion === 'Upright' && policy.stable_standing_s >= 1
              && (solver.sourceRecoveryActive || solver.currentInternalPgsIterations !== 20 || solver.selectedNextInternalPgsIterations !== 20)) throw new Error('Terminal standing must use20internal PGS');
            const state = validateReplan(fixture.fixtureTerminalReferenceReplan, native);
            e.terminalReferenceReplanSamples.push({ native, time, ...structuredClone(fixture.fixtureTerminalReferenceReplan) });
            if (e.previousReplanUsed === null && state.used) throw new Error('Fresh fixture started with consumed replan budget');
            if (e.previousReplanUsed === true && !state.used) throw new Error('One-per-physical-trial replan budget replenished across controller source changes');
            if (state.used) {
              if (e.replanConsumedState && !equal(e.replanConsumedState, state)) throw new Error('Consumed attempted timestamp/prior forecast state changed');
              if (!e.replanConsumedState) {
                if (e.previousReplanUsed !== false || native - state.attempted_native_substep > 19) throw new Error('First native replan publication boundary invalid');
                e.replanUsedTransitions++;
                if (e.replanUsedTransitions !== 1) throw new Error('More than one physical-trial replan transition');
                e.replanConsumedState = structuredClone(state);
                e.replanObserved = { native, time, publicationDelayNativeSubsteps: native - state.attempted_native_substep,
                  actualAttemptNative: state.attempted_native_substep, priorForecastSubsteps: state.prior_forecast_substeps,
                  originalTriggerNative: e.originalTriggerNative, originalImpactNative: e.originalImpactNative,
                  policy: structuredClone(policy), solverBudget: structuredClone(solver),
                  physicalBitClaim: false, note: 'First observed publication, not immediate native pre/post attempt state.' };
                requestCapture('replan-observed', native);
              }
            }
            e.previousReplanUsed = state.used;
            const current = fixture.currentStandingDiagnostic;
            if (!current || current.physicsIntegrations !== 0 || current.nativeSubsteps !== native
              || !current.predicates || !Object.keys(current.predicates).length) throw new Error('Read-only current standing/contact/twist diagnostic missing');
            if (current.footDriftM !== null || current.predicates.footDriftFromSuppliedAnchors !== null) throw new Error('Null-anchor diagnostic must keep foot drift explicitly unavailable');
            e.standingDiagnostics.push({ native, time, motion: snapshot.motion, current: structuredClone(current),
              suppliedAnchorDriftAvailable: false, original: structuredClone(fixture.originalAnchorStanding ?? null),
              postPass: structuredClone(fixture.postPassStanding ?? null) });
            e.solverBudgetSamples.push({ native, time, motion: snapshot.motion, ...structuredClone(solver) });
            e.rootRotationPolicySamples.push({ native, time, ...structuredClone(root) });
            e.policyPublications.push({ native, time, policy: structuredClone(policy) });
            if (e.lastControllerStartedAtS !== policy.started_at_s) {
              e.controllerStartChanges.push({ native, time, previousStartedAtS: e.lastControllerStartedAtS ?? null,
                startedAtS: policy.started_at_s, used: state.used, originalTriggerNative: e.originalTriggerNative,
                note: 'Controller forecast start may reset/null during replan; never a new physical trial or deadline.' });
              e.lastControllerStartedAtS = policy.started_at_s;
            }
            const impactNative = Number.isFinite(policy.since_impact_s) ? native - nativeAt(policy.since_impact_s) : null;
            if (impactNative !== null) {
              if (!Number.isSafeInteger(impactNative) || impactNative < 0 || impactNative > native) throw new Error('Invalid native policy impact clock');
              if (e.originalTriggerNative === null || impactNative !== e.originalTriggerNative || impactNative !== e.originalImpactNative) throw new Error('Policy original impact clock changed during controller replan');
            }
            e.recoveryImpactClocks.push({ native, time, startedAtS: policy.started_at_s,
              sinceImpactS: policy.since_impact_s, inferredOriginalImpactNative: impactNative,
              originalTriggerNative: e.originalTriggerNative, originalImpactNative: e.originalImpactNative });
            if (message.poseBuffer instanceof ArrayBuffer) e.lastReceivedPackedPosePacket = { native, time,
              wordsUint32: Array.from(new Uint32Array(message.poseBuffer)), source: 'Received transfer buffer copied before original driver recycles/detaches it; no physical write' };
            e.samples.push(structuredClone(snapshot));
            e.publications.push({ native, time, completedTicks: message.reply.completed_ticks, requestId: message.requestId,
              triggerNative: fixture.triggerNative, counters: structuredClone(snapshot.counters), replanUsed: state.used });
            if (!duplicate) e.lastDistinctPublication = { native, time };
            if (e.startedAtMs === null || e.endedAtMs !== null) return;
            if (message.reply.failure) { e.stop('physical-or-resource-halt'); return; }
            const earnedPass = policy.prototype_standing_passed === true && policy.stable_standing_s >= 1 && snapshot.motion === 'Upright';
            if (e.firstPass === null && earnedPass) {
              if (e.originalImpactNative === null || native > e.originalImpactNative + 25 * 800) { e.stop('unchanged-recovery-deadline'); return; }
              if (!fixture.originalAnchorStanding?.originalStableFeet) throw new Error('Earned pass lacks actual original controller stable_feet');
              e.firstPassTimeS = time;
              e.firstPass = { native, time, stableS: policy.stable_standing_s, startedAtS: policy.started_at_s,
                counters: structuredClone(snapshot.counters), originalImpactNative: e.originalImpactNative };
              e.checkpointAnchors = structuredClone(fixture.originalAnchorStanding.originalStableFeet);
              requestCapture('first-pass', native); // Queue after replan-observed if both share this publication.
            }
            if (e.firstPass !== null) {
              const original = fixture.originalAnchorStanding, standing = fixture.postPassStanding;
              const failed = [];
              if (!original || !standing) failed.push('earned-original-anchor-diagnostic-missing');
              else {
                e.historicalDiagnostics.push(structuredClone(original)); e.postPassDiagnostics.push(structuredClone(standing));
                for (const [label, value] of [['original', original], ['postPass', standing]]) {
                  failed.push(...Object.entries(value.predicates).filter(([, passed]) => passed !== true).map(([name]) => label + '.' + name));
                  if (value.physicsIntegrations !== 0 || value.nativeSubsteps !== native) failed.push(label + '.read-only-native-clock');
                }
                if (!equal(original.originalStableFeet, e.checkpointAnchors)) failed.push('original-controller-stable_feet-changed');
              }
              const previous = e.postPassContinuity.at(-1), holdDuplicate = previous?.native === native;
              const nativeDelta = previous ? native - previous.native : null;
              const physicalDelta = previous ? time - previous.time : null;
              const stableDelta = previous ? policy.stable_standing_s - previous.stable : null;
              const clockSequence = !previous || (holdDuplicate ? nativeDelta === 0 && physicalDelta === 0 && stableDelta === 0 && message.reply.completed_ticks === 0
                : nativeDelta === 20 && Math.abs(physicalDelta - .025) <= 1e-9 && Math.abs(stableDelta - .025) <= 1e-9 && message.reply.completed_ticks === 1);
              if (!earnedPass) failed.push('native1s-standing-latch-or-physical-Upright');
              if (!clockSequence) failed.push('native-stable-counter-continuity');
              for (const name of ['strikes', 'falls', 'recoveries', 'steps']) {
                if (snapshot.counters[name] !== e.firstPass.counters[name]) failed.push('original-counter.' + name + '-changed');
              }
              const expectedUprightTicks = e.firstPass.counters.upright_ticks + (native - e.firstPass.native) / 20;
              if (!Number.isSafeInteger(expectedUprightTicks) || snapshot.counters.upright_ticks !== expectedUprightTicks) failed.push('ordinary-upright-tick-counter-continuity');
              if (policy.started_at_s !== e.firstPass.startedAtS) failed.push('postpass-controller-source-changed');
              if (holdDuplicate) e.duplicatePostPassPackets++; else e.distinctPostPassSamples++;
              e.postPassContinuity.push({ native, time, stable: policy.stable_standing_s, motion: snapshot.motion,
                nativeDelta, physicalDelta, stableDelta, duplicate: Boolean(holdDuplicate), clockSequence,
                counters: structuredClone(snapshot.counters), expectedUprightTicks, continuous: failed.length === 0 });
              if (failed.length) { e.standingViolations.push({ native, time, failed }); e.stop('original-anchor-counter-standing-hold-failure'); return; }
              if (native >= e.firstPass.native + 30 * 800) e.stop('thirty-physical-seconds-after-first-pass');
            } else if (e.originalImpactNative !== null && native >= e.originalImpactNative + 25 * 800) e.stop('unchanged-recovery-deadline');
          } catch (error) {
            e.rawObserverFault = { error: String(error), messageType: message?.type ?? null,
              requestId: message?.requestId ?? null, fixtureTest: structuredClone(message?.fixtureTest ?? null),
              reply: structuredClone(message?.reply ?? null) };
            e.stop('instrumentation-error', String(error));
          }
        });
      }
      postMessage(message, ...args) {
        if (message.type === 'initialize') {
          message = { ...message, testFixedGetUpFixture: kind, testFixedGetUpProfile: approvedProfile };
          e.initializations.push(structuredClone(message));
        }
        if (message.commands?.length) {
          e.commands.push(structuredClone(message.commands)); e.stop('unexpected-physical-command', 'Physical commands are forbidden in this fixture observation');
          throw new Error('Physical commands forbidden');
        }
        if (message.type === 'reset') {
          e.resetRequests.push(structuredClone(message)); e.stop('unexpected-reset', 'Reset is forbidden in this fixture observation');
          throw new Error('Reset forbidden');
        }
        return super.postMessage(message, ...args);
      }
    };
    for (const type of ['pointerdown', 'keydown', 'keyup']) document.addEventListener(type, event => {
      if (e.startedAtMs !== null && e.endedAtMs === null && (type !== 'pointerdown' || event.target?.closest?.('#viewport'))) {
        e.physicalInputs.push({ type, code: event.code ?? null, atMs: event.timeStamp, isTrusted: event.isTrusted });
        e.stop('unexpected-physical-input', 'Physical inputs forbidden during autonomous observation');
      }
    }, true);
    document.addEventListener('DOMContentLoaded', () => {
      const label = document.createElement('div');
      Object.assign(label.style, { position: 'fixed', bottom: '8px', left: '8px', zIndex: '2147483647',
        pointerEvents: 'none', background: '#091424ed', color: '#fff', padding: '8px 12px', font: '12px monospace', whiteSpace: 'pre' });
      document.body.append(label);
      const draw = () => {
        const d = api?.diagnostics, policy = d?.snapshot?.diagnostics.getup?.policy;
        const wall = e.startedAtMs === null ? 0 : ((e.endedAtMs ?? performance.now()) - e.startedAtMs) / 1000;
        label.textContent = `ISOLATED FIXED-FIXTURE TEST | ${kind} | explicit3+1 |800Hz/40Hz\nBOUNDED EXPERIMENTAL ONE-TERMINAL-REFERENCE REPLAN | maximum1 per physical trial\ncarry + standing dynamics + unit-root + repeated-cycle/terminal balance | base-ready/bumpless OFF\nOriginal impact25s deadline + native1s pass +30s original-anchor/counter hold\n40Hz readback cannot prove240consecutive800Hz predicates | NO PERFORMANCE/GLOBAL ADMISSION\nsim ${(d?.time ?? 0).toFixed(3)}s | wall ${wall.toFixed(2)}s | ${policy?.phase ?? 'physical preparation'}\ntrigger native ${e.originalTriggerNative ?? 'pending'} | used ${e.previousReplanUsed ?? 'pending'} | stable ${(policy?.stable_standing_s ?? 0).toFixed(3)}s\n${e.stopReason ?? 'original automatic one-tick scheduler; no physical input'}`;
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
  assert.equal(report.initial.tick, 0); assert.equal(report.initial.time, 0); assert.equal(report.initial.backend, 'WebGL2');
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
  assert.equal(report.activationProof.activationCalls, 1);
  assert.deepEqual(report.activationProof.poseVelocityF32WordsBefore, report.activationProof.poseVelocityF32WordsAfter);
  assert.equal(report.activationProof.poseVelocityF32WordsBefore.length, 325);
  assert.deepEqual(report.activationProof.beforeReport.profile, approvedProfile);
  assert.deepEqual(report.activationProof.afterReport.profile, approvedProfile);
  assert.deepEqual(report.activationProof.beforeReport.rootRotationNormalization, report.activationProof.afterReport.rootRotationNormalization);
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.algorithm, 'unit-free-root-v1');
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.enabled, true);
  assert.equal(report.activationProof.afterReport.rootRotationNormalization.validationError, null);
  assert.deepEqual(report.activationProof.beforeReport.terminalReferenceReplan, report.activationProof.afterReport.terminalReferenceReplan);
  assert.deepEqual(report.activationProof.beforeReport.terminalReferenceReplan, { maximumReplansPerPhysicalTrial: 1,
    state: { format: 'one-terminal-reference-fall-replan-v1', used: false, settled_native_samples: 0, attempted_native_substep: null, prior_forecast_substeps: 0 } });
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
  const viewport = await page.locator('#viewport').boundingBox(); assert.ok(viewport, 'Actual viewport required');
  await page.mouse.move(viewport.x + viewport.width / 2, viewport.y + viewport.height / 2);
  await page.mouse.wheel(0, 400); await page.waitForTimeout(150);
  report.framing = { ordinaryUiOnly: true, cameraFocusBody: true, preStartWheelDeltaY: 400, note: 'Camera-only framing before any physical integration.' };
  report.framedInitial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(report.framedInitial.tick, 0); assert.equal(report.framedInitial.time, 0);
  await page.screenshot({ path: join(output, 'initial-fixed-fixture.png'), fullPage: true });
  await page.evaluate(() => {
    const e = window.__fixtureEvidence;
    if (e.error || e.endedAtMs !== null) throw new Error(e.error || e.stopReason);
    window.__lhRust.drainTelemetry(); e.startedAtMs = performance.now();
    e.initialFrames = window.__lhRust.diagnostics.graphics.frames; window.__lhRust.setManual(false);
  });
  report.telemetry = { packets: [], frames: [], inputs: [], bounds: null };
  const watchdog = Date.now() + 20 * 60 * 1000;
  let nextProgressS = 0;
  while (true) {
    await page.waitForTimeout(1000);
    const update = await page.evaluate(() => {
      const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
      if (d.workerFault || d.disposed || d.halted || d.failure || d.error) e.stop(d.workerFault || d.disposed
        ? 'renderer-or-driver-terminated-worker' : 'physical-resource-or-renderer-halt', d.error || null);
      return { time: d.time, tick: d.tick, motion: d.motion, policy: d.snapshot?.diagnostics.getup?.policy,
        error: e.error || d.error, workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed),
        stopReason: e.stopReason, ended: e.endedAtMs !== null, pendingCaptures: [...e.pendingCaptures],
        replanObserved: e.replanObserved, wallSeconds: ((e.endedAtMs ?? performance.now()) - e.startedAtMs) / 1000,
        telemetry: window.__lhRust.drainTelemetry() };
    });
    for (const name of ['packets', 'frames', 'inputs']) report.telemetry[name].push(...update.telemetry[name]);
    for (const name of ['bounds', 'droppedPackets', 'droppedFrames', 'droppedInputs', 'pendingVisualInputs']) report.telemetry[name] = update.telemetry[name];
    delete update.telemetry;
    if (update.time >= nextProgressS || update.ended || update.error || update.pendingCaptures.length) {
      report.progress.push(update); console.log(JSON.stringify({ finding: 'one-terminal-replan-fixed-fixture-progress', ...update })); nextProgressS = update.time + 1;
    }
    if (report.errors.length) await page.evaluate(message => window.__fixtureEvidence.stop('page-or-resource-error', message), report.errors.join('\n'));
    if (update.error || update.workerFault || update.disposed || report.errors.length) break; // Never export queued captures from a known failed observer/worker.
    // A replan and first pass may be observed on the same publication. Complete
    // all read-only captures sequentially at that exact native clock, then resume.
    for (const pending of update.pendingCaptures) {
      const controller = await captureReadOnlyCheckpoint(pending.label);
      if (!controller) {
        await page.evaluate(() => window.__fixtureEvidence.stop('checkpoint-unavailable', 'Original worker/observer unavailable for queued read-only capture'));
        break;
      }
      assert.equal(report.checkpointCaptures[pending.label].verification.nativeSubsteps, pending.native, 'Paused publication boundary must not integrate between queued captures');
      const expected = await page.evaluate(() => ({ state: window.__fixtureEvidence.replanConsumedState,
        anchors: window.__fixtureEvidence.checkpointAnchors, impact: window.__fixtureEvidence.originalImpactNative }));
      if (expected.state) assert.deepEqual(controller.terminal_reference_replan, expected.state, 'Actual controller must retain one-per-trial consumed budget');
      assert.equal(controller.impact_substep, expected.impact, 'Controller export must retain the original physical trigger/impact epoch');
      if (pending.label === 'first-pass') assert.deepEqual(controller.stable_feet, expected.anchors, 'Read-only controller must retain exact original stable_feet');
      await page.evaluate(label => {
        const e = window.__fixtureEvidence;
        if (e.pendingCaptures[0]?.label !== label) throw new Error('Read-only capture queue order changed');
        e.pendingCaptures.shift(); e.savedCaptures.push(label);
      }, pending.label);
    }
    const afterCapture = await page.evaluate(() => {
      const e = window.__fixtureEvidence;
      if (!e.error && e.endedAtMs === null && e.pendingCaptures.length === 0) window.__lhRust.setManual(false);
      return { ended: e.endedAtMs !== null, error: e.error };
    });
    if (afterCapture.ended || afterCapture.error) break;
    if (Date.now() >= watchdog) {
      await page.evaluate(() => window.__fixtureEvidence.stop('native-wall-time-watchdog', 'Twenty-minute physical-observation wall watchdog exceeded')); break;
    }
  }
  await page.evaluate(() => window.__lhRust.setManual(true));
  const finalAvailability = await page.evaluate(() => {
    const d = window.__lhRust.diagnostics, e = window.__fixtureEvidence;
    return { workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed), observerError: e.error, time: d.time, error: d.error };
  });
  if (finalAvailability.workerFault || finalAvailability.disposed || finalAvailability.observerError)
    report.stopCheckpointUnavailable = { ...finalAvailability, reason: 'Known fault: skip getupReport/getupCheckpoint and failed pending captures; retain raw result.' };
  else await captureReadOnlyCheckpoint('stop');
  report.final = await page.evaluate(() => window.__lhRust.diagnostics);
  report.evidence = await page.evaluate(() => { const { stop, ...data } = window.__fixtureEvidence; return data; });
  const e = report.evidence, policy = report.final.snapshot?.diagnostics.getup?.policy;
  report.fullSourceProfile = e.profile === null ? null : JSON.parse(e.profile);
  const lastStanding = e.standingDiagnostics.at(-1), original = lastStanding?.original;
  const failedFinalPredicates = original ? Object.entries(original.predicates).filter(([, value]) => value !== true).map(([key]) => key)
    : ['actual-original-controller-anchor-diagnostic-unavailable'];
  report.finalPhysicalStanding = { evaluatedAtNative: Math.round(report.final.time * 800), motion: report.final.motion,
    nativeStableStandingS: policy?.stable_standing_s, nativeStandingLatch: policy?.prototype_standing_passed,
    predicateSource: original ? 'actual-controller-original-stable_feet' : 'unavailable-without-earned-native-stability',
    originalAnchorPredicates: original?.predicates ?? null, originalStableFeet: original?.originalStableFeet ?? null,
    currentNullAnchorRawPredicates: lastStanding?.current.predicates ?? null, suppliedAnchorDriftAvailable: false,
    failedPredicates: failedFinalPredicates,
    passed: failedFinalPredicates.length === 0 && report.final.motion === 'Upright'
      && policy?.prototype_standing_passed === true && policy?.stable_standing_s >= 1 };
  report.replanObservation = { observed: Boolean(e.replanObserved), transitionCount: e.replanUsedTransitions,
    firstObserved: e.replanObserved, forced: false, browserProvesNative240SampleGate: false,
    replanCheckpointSaved: Boolean(report.checkpointCaptures['replan-observed']?.verification?.passed),
    note: 'No replan is required for a truthful outcome. If the native settled gate never fires, the unchanged original25s failure remains final.' };
  report.recoveryDeadline = { originalTriggerNative: e.originalTriggerNative, originalImpactNative: e.originalImpactNative,
    deadlineNative: e.originalImpactNative === null ? null : e.originalImpactNative + 25 * 800,
    firstPassNative: e.firstPass?.native ?? null, preservedAcrossControllerStartChanges: true };
  report.diagnosticCaptureComplete = Boolean(report.checkpointCaptures.stop?.verification?.passed
    && (!e.firstPass || report.checkpointCaptures['first-pass']?.verification?.passed)
    && (!e.replanObserved || report.checkpointCaptures['replan-observed']?.verification?.passed));
  report.thirtySecondStandingObserved = Boolean(e.firstPass && Math.round(report.final.time * 800) >= e.firstPass.native + 30 * 800
    && e.distinctPostPassSamples >= 1201 && e.postPassContinuity.every(value => value.continuous)
    && e.standingViolations.length === 0 && e.stopReason === 'thirty-physical-seconds-after-first-pass');
  report.outcome = report.thirtySecondStandingObserved ? 'native-pass-plus30s-original-anchor-counter-hold'
    : e.stopReason === 'unchanged-recovery-deadline' ? 'original25s-recovery-deadline-failure' : 'physical-observer-or-resource-failure';
  report.observation = { simulationSeconds: report.final.time, wallSeconds: (e.endedAtMs - e.startedAtMs) / 1000,
    presentedFrames: e.endedFrames - e.initialFrames,
    note: 'Native wall-time evidence only; no retiming, realtime/mobile performance claim, or native800Hz global qualification.' };
  assert.equal(e.workerInstances, 1); assert.equal(e.workerIdentities.length, 1); assert.equal(e.initializations.length, 1);
  assert.equal(e.initializations[0].profile, undefined, 'Ordinary driver profile path unchanged');
  assert.deepEqual(e.initializations[0].testFixedGetUpProfile, approvedProfile);
  assert.deepEqual(report.fullSourceProfile, approvedProfile);
  assert.equal(e.typedProfilePublications.length, e.samples.length);
  assert.ok(e.typedProfilePublications.every(value => sameJson(value.profile, approvedProfile)));
  assert.deepEqual(e.commands, []); assert.deepEqual(e.physicalInputs, []); assert.deepEqual(e.resetRequests, []);
  assert.ok(e.trigger?.physicalPoseVelocityBitsUnchanged); assert.deepEqual(e.trigger.beforePoseWords, e.trigger.afterPoseWords);
  assert.equal(e.trigger.beforePoseWords.length, 325);
  assert.ok(e.replanUsedTransitions <= 1); assert.deepEqual(e.pendingCaptures, []);
  assert.equal(e.error, null); assert.deepEqual(report.errors, []);
  assert.equal(report.final.failure, null); assert.equal(report.final.error, '');
  assert.equal(Boolean(report.final.workerFault), false); assert.equal(Boolean(report.final.disposed), false);
  assert.ok(report.diagnosticCaptureComplete, 'All applicable schema2 read-only checkpoints required');
  assert.ok(report.thirtySecondStandingObserved, 'Original native1s pass plus30s continuous original-anchor/counter hold required');
  assert.ok(report.finalPhysicalStanding.passed, 'Final physical standing must use earned actual original anchors');
  await page.screenshot({ path: join(output, 'after-thirty-physical-seconds-standing.png'), fullPage: true });
  report.passed = true;
} catch (error) {
  report.error = error.stack ?? String(error);
  if (page && !page.isClosed()) {
    try {
      await page.evaluate(message => {
        const e = window.__fixtureEvidence;
        if (!e) return;
        e.recorderError = message; // Keep the original observer/native fault and stop reason intact.
        e.stop('recorder-error');
        e.skippedCaptures.push(...e.pendingCaptures.splice(0));
      }, String(error));
      report.failureState = await page.evaluate(() => window.__lhRust?.diagnostics);
      report.evidence = await page.evaluate(() => { const { stop, ...data } = window.__fixtureEvidence ?? {}; return data; });
      await page.screenshot({ path: join(output, 'failure.png'), fullPage: true });
    } catch (captureError) { report.rawFailureCaptureError = String(captureError); }
  }
} finally {
  report.finished = new Date().toISOString();
  // Preserve raw failed-run evidence before browser/encoder cleanup.
  writeFileSync(join(output, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  try {
    await context?.close();
    if (video) {
      await video.saveAs(join(output, 'fixed-fixture-native-wall-time.webm'));
      report.recording = { path: 'fixed-fixture-native-wall-time.webm', retimed: false };
    }
  } catch (error) { report.recordingError = String(error); report.passed = false; }
  try { await browser?.close(); } catch (error) { report.browserCloseError = String(error); }
  server?.kill();
  writeFileSync(join(output, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({ passed: report.passed, outcome: report.outcome, error: report.error,
    finalTime: report.final?.time ?? report.failureState?.time, stopReason: report.evidence?.stopReason,
    replanObserved: report.replanObservation?.observed, thirtySecondStandingObserved: report.thirtySecondStandingObserved, output }, null, 2));
  process.exitCode = report.passed ? 0 : 1;
}
