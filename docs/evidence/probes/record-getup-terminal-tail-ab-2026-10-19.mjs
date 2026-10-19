// SOURCE-ONLY preparation. No browser, dynamics, build or test has been run.
// Fresh BACK A/B worlds; original renderer/driver/transport/one-tick scheduling.
// Candidate replaces ONLY the appended standing_target tuple with the final
// interpolated 25-vector. Both arms use the identical complete baseline profile.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { pathToFileURL } from 'node:url';

// INTERFACE CONSTANTS. Fail closed until adapter/native owners confirm every
// name and native0/original-anchor row guarantee. No permissive fallback aliases.
const CONTRACT = Object.freeze({
  confirmed: true,
  fixtureTraceField: 'endpointNativeTrace',
  fixtureIdentityField: 'endpointBuildIdentity',
  fixtureTraceErrorField: 'endpointTraceError',
  nativeTraceFormat: 'read-only-post-native-observer-v1',
  experiment: 'terminal-tail-ab-v1',
  nativeHz: 800, publicationSubsteps: 20,
  baselineVariant: 'source-canonical', candidateVariant: 'last-interpolated-duplicate',
  commonBuildFeatures: [
    'lh-worker/root-translation-carry-experiment',
    'lh-sim/getup-terminal-balance-experiment',
    'lh-sim/standing-dynamics-experiment',
    'lh-sim/unit-root-rotation-experiment',
    'lh-sim/getup-repeat-cycle-experiment',
    'lh-worker/getup-terminal-tail-observer-experiment',
  ],
  candidateBuildFeature: 'lh-sim/getup-terminal-tail-duplicate-experiment',
  // Row contract: nativeSubsteps, poseWords[325], physicalObservationWords[67],
  // current(actual original stable_feet where earned), getup.policy, motion,
  // counters, originalStableFeet, commandInputNativeSubstep,
  // requestedMotorEffortNm, directPelvisForceN, directPelvisTorqueNm.
  // Native0 is a separate read-only receipt; full post-native rows start at1.
  triagePredicates: ['comInSupport', 'captureInSupport'],
  triageMinimumFootLoadN: 3, triageMinimumFootWeightFraction: .8,
  triageTorsoUp: .97, triageConsecutiveNative: 80, triageAfterHoldingNative: 400,
  recoveryDeadlineNative: 25 * 800, postPassNative: 30 * 800,
  exportWallWatchdogMs: 30_000, startupWallWatchdogMs: 60_000,
  noProgressWallWatchdogMs: 60_000, totalWallWatchdogMs: 20 * 60_000,
});
const profilePath = resolve('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json');
const profileSha256 = 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6';
const hash = value => createHash('sha256').update(value).digest('hex');
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
  ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
const equal = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
const json = (path, value) => writeFileSync(path, JSON.stringify(value, null, 2) + '\n');
const withWallWatchdog = async (promise, ms, label) => {
  let timer;
  try { return await Promise.race([promise, new Promise((_, reject) => {
    timer = setTimeout(() => reject(new Error(label + ' wall watchdog exceeded')), ms);
  })]); } finally { clearTimeout(timer); }
};
function hashes(root, base = root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path, base)) : [[relative(base, path), hash(readFileSync(path))]];
  }));
}
function verifyArtifact(artifact, arm, approvedProfile) {
  const adapterPath = artifact + '-fixture-adapter-manifest.json';
  const adapterBytes = readFileSync(adapterPath), adapter = JSON.parse(adapterBytes);
  const artifactSha256 = hashes(artifact);
  assert.deepEqual(artifactSha256, adapter.artifactSha256, 'All immutable fixture app bytes must match adapter manifest');
  assert.equal(adapter.isolatedTestOnly, true);
  assert.equal(adapter.adapterSha256, artifactSha256['sim-worker.js']);
  assert.equal(adapter.explicitProfile.sha256, profileSha256);
  assert.deepEqual(adapter.explicitProfile.value, approvedProfile, 'Complete identical typed baseline profile required in both arms');
  const expectedFeatures = [...CONTRACT.commonBuildFeatures,
    ...(arm === 'candidate' ? [CONTRACT.candidateBuildFeature] : [])];
  assert.deepEqual([...adapter.sourceBuildFeatures].sort(), expectedFeatures.sort(), 'Only reviewed observer plus optional single tuple candidate build features');
  const frozenPath = resolve(adapter.approvedFrozenManifest.path), frozenBytes = readFileSync(frozenPath);
  assert.equal(hash(frozenBytes), adapter.approvedFrozenManifest.sha256);
  const frozen = JSON.parse(frozenBytes);
  assert.deepEqual([...frozen.extraFeatures].sort(), expectedFeatures.sort());
  assert.equal(frozen.target, 'wasm32-unknown-unknown');
  assert.deepEqual(frozen.rustFlags, []); assert.equal(frozen.productionDefaultsChanged, false);
  assert.deepEqual(Object.keys(frozen.artifactSha256).sort(), Object.keys(artifactSha256).sort());
  for (const [path, sha] of Object.entries(frozen.artifactSha256)) {
    if (path !== 'sim-worker.js') assert.equal(artifactSha256[path], sha, 'Only isolated worker adapter may differ from frozen build: ' + path);
  }
  assert.equal(adapter.originalWorkerTransportSha256, frozen.artifactSha256['sim-worker.js']);
  const effectiveSimFeatures = expectedFeatures.map(value => value.slice(value.indexOf('/') + 1)).sort();
  const fingerprint = frozen.simFingerprints.find(value => equal([...JSON.parse(value.features)].sort(), effectiveSimFeatures));
  assert.ok(fingerprint, 'Exact effective sim features must exclude base-ready/bumpless/replan or other policies');
  assert.deepEqual(fingerprint.rustflags, []);
  const fingerprintBytes = readFileSync(resolve(fingerprint.path));
  assert.equal(hash(fingerprintBytes), fingerprint.sha256, 'Frozen effective sim fingerprint bytes required');
  const actualFingerprint = JSON.parse(fingerprintBytes);
  assert.deepEqual([...JSON.parse(actualFingerprint.features)].sort(), effectiveSimFeatures);
  assert.deepEqual(actualFingerprint.rustflags, []);

  const sourcePath = resolve(frozen.sourceManifest.path), sourceBytes = readFileSync(sourcePath);
  assert.equal(hash(sourceBytes), frozen.sourceManifest.sha256);
  const source = JSON.parse(sourceBytes), sourceRoot = join(dirname(sourcePath), 'source');
  assert.equal(source.baseReadyAndBumplessEnabled, false);
  assert.equal(source.profileSha256, profileSha256);
  for (const [path, sha] of Object.entries(source.sourceFiles)) assert.equal(hash(readFileSync(join(sourceRoot, path))), sha);
  const drivers = Object.keys(artifactSha256).filter(path => path.endsWith('/driver.js'));
  assert.equal(drivers.length, 1);
  assert.equal(artifactSha256[drivers[0]], hash(readFileSync(resolve('rust/crates/browser/driver.js'))));
  return { artifactSha256, expectedFeatures, adapterManifest: { path: adapterPath, sha256: hash(adapterBytes) },
    frozenBuildManifest: { path: frozenPath, sha256: hash(frozenBytes) },
    frozenSourceManifest: { path: sourcePath, sha256: hash(sourceBytes), verifiedFiles: Object.keys(source.sourceFiles).length },
    originalDriver: { path: drivers[0], sha256: artifactSha256[drivers[0]], byteEqual: true },
    frozenSourceFiles: source.sourceFiles, effectiveSimFeatures, effectiveSimFingerprint: fingerprint };
}

assert.equal(process.argv.length, 5, 'Usage: recorder BASELINE_ARTIFACT CANDIDATE_ARTIFACT FRESH_OUTPUT');
assert.equal(CONTRACT.confirmed, true, 'SOURCE-ONLY DRAFT: native observer/adapter interface has not been confirmed');
const artifacts = { baseline: resolve(process.argv[2]), candidate: resolve(process.argv[3]) };
const output = resolve(process.argv[4]);
assert.notEqual(artifacts.baseline, artifacts.candidate, 'Separate immutable builds required');
for (const artifact of Object.values(artifacts)) {
  assert.ok(output !== artifact && !output.startsWith(artifact + sep) && !artifact.startsWith(output + sep), 'Evidence and immutable artifacts must not overlap');
}
assert.ok(!existsSync(output) || readdirSync(output).length === 0, 'Fresh empty output required');
mkdirSync(output, { recursive: true });
const profileBytes = readFileSync(profilePath), approvedProfile = JSON.parse(profileBytes);
assert.equal(hash(profileBytes), profileSha256);
assert.equal(approvedProfile.dt_s, 1 / 800); assert.equal(approvedProfile.tick_substeps, 20);
assert.equal(approvedProfile.solver_iterations, 3); assert.equal(approvedProfile.additional_body_iterations, 1);
assert.equal(approvedProfile.getup_policy_enabled, true);
assert.equal(approvedProfile.getup_solver_budget, 'consistent-recovery-phase-v1');
assert.equal(approvedProfile.root_rotation_normalization, 'unit-free-root-v1');
assert.equal(approvedProfile.getup_repeated_recovery, 'completed-cycle-confirmed-fall-v1', 'Both arms keep the complete repeat-cycle baseline; no current-trial retry or epoch renewal');
const { parseGetUpCheckpoint, proveActivationPurity } = await import(pathToFileURL(resolve('docs/evidence/probes/getup-checkpoint-envelope-2026-10-19.mjs')).href);
const { chromium, browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
const report = { at: new Date().toISOString(), passed: false, qualificationClaim: false, performanceClaim: false,
  mode: 'fresh-back-terminal-tail-ab', profile: approvedProfile, profilePath, profileSha256, contract: CONTRACT,
  noEncoder: true, noRetries: true, arms: {}, comparison: null,
  scope: 'Same complete repeat-cycle baseline profile with no current-trial retry, carry once native0, original80-native fixture preparation polling, original renderer/driver/transport/automatic one-tick scheduler. '
    + 'Every800Hz current physical/load/support/capture/motion/control/material-slip row through actual first Holding+0.5s. '
    + '80consecutive supported-foot/80%BW/nonfoot<3N/torsoUp>=.97/COM-support/capture-support samples by that bound, or immediate NEGATIVE. '
    + 'Only a passing triage continues in the SAME WORLD to original25s deadline/native1s latch and30physical seconds unchanged controller anchors/counters. No physical commands, input, reset, support injection, retiming or epoch renewal.' };

async function observeArm(arm) {
  const artifact = artifacts[arm], armOutput = join(output, arm);
  mkdirSync(armOutput);
  const r = { arm, artifact, at: new Date().toISOString(), passed: false, outcome: 'NOT_STARTED',
    errors: [], progress: [], nativeTrace: { path: 'native-trace.ndjson', rows: 0, native0Included: false },
    controllerCaptures: {}, noEncoder: true, checkpointResumeSupported: arm === 'baseline',
    expectedMutation: arm === 'baseline' ? 'none' : 'appended standing_target tuple replaced by duplicate final interpolated25-vector only' };
  report.arms[arm] = r;
  let server, browser, context, page;
  const evalPage = (...args) => withWallWatchdog(page.evaluate(...args), CONTRACT.exportWallWatchdogMs, 'browser evidence operation');
  const tracePath = join(armOutput, r.nativeTrace.path);
  writeFileSync(tracePath, '');
  const prefixHasher = createHash('sha256');
  let nativePrefixRows = 0, nativeFirstEndpointLookahead = null, prefixDigest = null;
  const drain = update => {
    const rows = update.nativeTrace ?? [];
    if (rows.length) {
      appendFileSync(tracePath, rows.map(row => JSON.stringify(row)).join('\n') + '\n');
      r.nativeTrace.rows += rows.length;
      if (rows.some(row => row.nativeSubsteps === 0)) r.nativeTrace.native0Included = true;
      for (const row of rows) {
        // Build identity is deliberately absent from the physical/policy row.
        // Starting at the first endpoint-consuming decision, equality is no
        // longer an expectation. Native post-step n corresponds to control n-1.
        const p = row.getup?.policy;
        if (nativeFirstEndpointLookahead === null && Number.isFinite(p?.started_at_s) && p.reference_frames > 0)
          nativeFirstEndpointLookahead = Math.round(p.started_at_s * 800) + Math.max(0, p.reference_frames - 5) * 20 + 1;
        if (nativeFirstEndpointLookahead === null || row.nativeSubsteps < nativeFirstEndpointLookahead) {
          prefixHasher.update(JSON.stringify(canonical(row)) + '\n'); nativePrefixRows++;
        }
      }
    }
    delete update.nativeTrace;
  };
  async function captureController(label) {
    await page.waitForFunction(() => {
      const d = window.__lhRust?.diagnostics, e = window.__fixtureEvidence;
      return d && (!d.outstanding || d.workerFault || d.disposed || e?.error);
    }, null, { timeout: CONTRACT.exportWallWatchdogMs });
    const value = await withWallWatchdog(evalPage(async ({ watchdogMs }) => {
      const api = window.__lhRust, e = window.__fixtureEvidence;
      const fault = () => Boolean(e.error || api?.diagnostics.workerFault || api?.diagnostics.disposed);
      if (fault()) return { unavailable: true, reason: 'Known native/worker/observer fault; do not export or retry.' };
      const before = { time: api.diagnostics.time, tick: api.diagnostics.tick };
      let timer, interval;
      const failure = new Promise((resolveFault, reject) => {
        interval = setInterval(() => { if (fault()) resolveFault({ unavailable: true, reason: 'Original worker/observer fault during export' }); }, 25);
        timer = setTimeout(() => reject(new Error('getupReport read-only export watchdog')), watchdogMs);
      });
      try {
        const controller = await Promise.race([api.getupReport(), failure]);
        if (controller.unavailable) return controller;
        if (fault()) return { unavailable: true, reason: 'Original worker/observer failed after export' };
        if (api.diagnostics.time !== before.time || api.diagnostics.tick !== before.tick) throw new Error('Read-only controller export advanced physical clock');
        return { controller, native: Math.round(before.time * 800), integrations: 0 };
      } finally { clearTimeout(timer); clearInterval(interval); }
    }, { watchdogMs: CONTRACT.exportWallWatchdogMs }), CONTRACT.exportWallWatchdogMs + 1000, 'read-only controller capture');
    if (value.unavailable) { r.controllerCaptures[label] = value; return null; }
    const path = join(armOutput, label + '-controller.json'); json(path, value.controller);
    r.controllerCaptures[label] = { path: relative(armOutput, path), sha256: hash(readFileSync(path)), native: value.native, physicsIntegrations: 0 };
    return value.controller;
  }
  try {
    Object.assign(r, verifyArtifact(artifact, arm, approvedProfile));
    server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0'], { stdio: ['ignore', 'pipe', 'pipe'] });
    const origin = await withWallWatchdog(new Promise((resolveOrigin, reject) => {
      let text = '';
      server.on('error', reject); server.on('exit', code => reject(new Error('Preview exited before ready: ' + code)));
      server.stderr.on('data', data => { r.previewStderr = (r.previewStderr ?? '') + String(data); });
      server.stdout.on('data', data => { text += data; const match = text.match(/http:\/\/127\.0\.0\.1:\d+\//); if (match) resolveOrigin(match[0]); });
    }), 10_000, 'preview startup');
    browser = await chromium.launch(browserLaunchOptions(true));
    r.browserVersion = browser.version();
    context = await browser.newContext({ viewport: { width: 1280, height: 900 } }); // No video/encoder on first diagnostic.
    await context.addInitScript(({ arm, approvedProfile, C }) => {
      window.__lhGetUpTrial = true;
      let api;
      Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api,
        set(value) { api = value; api.setManual(true); } });
      const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === 'object'
        ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
      const equal = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
      const e = window.__fixtureEvidence = { arm, workerInstances: 0, workerIdentities: [], initializations: [],
        commands: [], physicalInputs: [], resetRequests: [], trigger: null, activation: null, profile: null,
        buildIdentity: null, nativeTrace: [], traceRows: 0, lastNative: null, lastPublicationNative: null,
        originalTriggerNative: null, originalImpactNative: null, controllerStartedAtS: null, referenceFrames: null,
        native0State: null, firstHoldingNative: null, firstHoldingCommandInputNative: null, expectedFirstHoldingNative: null, triageDeadlineNative: null, triageAwaitingCapture: false,
        triageConsecutive: 0, triageMaximumConsecutive: 0, triageFirst80Native: null, triage: null,
        extraUncreditedRows: [], firstPass: null, originalAnchors: null, postPassNativeRows: 0,
        standingViolations: [], materialSlip: {
          left_foot: { observedLoadedRows: 0, unavailableLoadedRows: 0, sampledImpulseWeightedTangentTravelM: 0, maximumImpulseWeightedTangentSpeedMps: 0 },
          right_foot: { observedLoadedRows: 0, unavailableLoadedRows: 0, sampledImpulseWeightedTangentTravelM: 0, maximumImpulseWeightedTangentSpeedMps: 0 },
          interpretation: 'Native800Hz sum of current impulse-weighted loaded-contact tangent speed times1/800; material contact slip, not foot-origin displacement. Missing speeds remain unavailable.',
        }, publications: [], stopReason: null, error: null, recorderError: null,
        rawWorkerFaults: [], rawObserverFault: null, pendingControllerCaptures: [], capturedControllerLabels: [],
        startedAtMs: null, endedAtMs: null, initialFrames: null, endedFrames: null,
        stop(reason, error = null) {
          if (error !== null) this.error ??= String(error);
          if (this.endedAtMs === null) { this.stopReason = reason; this.endedAtMs = performance.now(); this.endedFrames = api?.diagnostics.graphics?.frames ?? null; }
          api?.setManual(true);
        } };
      const scheduleCapture = label => {
        if (!e.capturedControllerLabels.includes(label) && !e.pendingControllerCaptures.includes(label)) e.pendingControllerCaptures.push(label);
        api.setManual(true);
      };
      const allPredicates = value => value && Object.keys(value).length > 0 && Object.values(value).every(v => v === true);
      function nativeRow(row) {
        const n = row.nativeSubsteps, d = row.current, p = row.getup?.policy;
        if (!Number.isSafeInteger(n) || n <= 0 || n !== e.lastNative + 1 || row.commandInputNativeSubstep !== n - 1 || row.physicalSeconds !== n / 800) throw new Error('Native trace must contain exact continuous post-native samples1..N and command-input n-1, with no interpolation/gap/duplicate');
        if (!Array.isArray(row.poseWords) || row.poseWords.length !== 325 || !Array.isArray(row.physicalObservationWords) || row.physicalObservationWords.length !== 67
          || ![...row.poseWords, ...row.physicalObservationWords].every(v => Number.isInteger(v) && v >= 0 && v <= 0xffffffff)) throw new Error('Exact native physical words missing');
        if (!d || d.physicsIntegrations !== 0 || d.nativeSubsteps !== n || !d.predicates
          || !Array.isArray(d.motion) || d.motion.length !== 25 || !d.contactMotion?.feet || !Array.isArray(d.loadsN) || d.loadsN.length !== 25
          || !Array.isArray(d.solverSupportPoints) || !d.com || !d.capture || !Number.isFinite(d.torsoUp)) throw new Error('Current native load/support/capture/motion/material-contact trace missing');
        if (row.originalStableFeet === null && (d.footDriftM !== null || d.predicates.footDriftFromSuppliedAnchors !== null)) throw new Error('Unanchored current diagnostic must keep drift explicitly unavailable');
        if (row.originalStableFeet !== null && (!Array.isArray(row.originalStableFeet) || row.originalStableFeet.length !== 2 || !Array.isArray(d.footDriftM))) throw new Error('Original native controller anchors missing');
        if (!p || !row.counters || typeof row.motion !== 'string' || !('requestedMotorEffortNm' in row)
          || !('directPelvisForceN' in row) || !('directPelvisTorqueNm' in row)
          || row.getup.motor_controls?.length !== 21 || !row.getup.motor_controls.every(value => Number.isFinite(value) && Math.abs(value) <= 1)) throw new Error('Native policy/motion/counter/21requested-control evidence missing');
        if (!Array.isArray(d.footIds) || d.footIds.length !== 2 || new Set(d.footIds).size !== 2
          || !d.footIds.every(i => Number.isSafeInteger(i) && i >= 0 && i < 25)
          || !d.loadsN.every(Number.isFinite) || !Number.isFinite(row.weightN) || row.weightN <= 0) throw new Error('Current exact foot indices/load/bodyweight evidence missing');
        if (!Number.isFinite(row.requestedMotorEffortNm) || row.requestedMotorEffortNm < 0 || row.requestedMotorEffortNm > 920.001
          || row.directPelvisForceN !== 0 || row.directPelvisTorqueNm !== 0) throw new Error('Motor effort cap or no-direct-pelvis-command contract violated');
        const contact = d.contactMotion, vectorFinite = value => value && ['x', 'y', 'z'].every(axis => Number.isFinite(value[axis]));
        if (contact.physicsIntegrations !== 0 || contact.velocitySource !== 'current-generalized-motion'
          || contact.feet.length !== 2 || !equal(contact.feet.map(foot => foot.body).sort(), ['left_foot', 'right_foot'])) throw new Error('Current exact two-foot material contact diagnostic missing');
        for (const foot of contact.feet) {
          if (!Number.isFinite(foot.loadN) || !vectorFinite(foot.origin) || !vectorFinite(foot.comVelocityMps)
            || !vectorFinite(foot.angularVelocityRadps) || !Array.isArray(foot.contacts)
            || !(foot.impulseWeightedTangentSpeedMps === null || Number.isFinite(foot.impulseWeightedTangentSpeedMps) && foot.impulseWeightedTangentSpeedMps >= 0)
            || !foot.contacts.every(value => Number.isFinite(value.normalImpulseNs) && value.normalImpulseNs >= 0
              && Number.isFinite(value.tangentSpeedMps) && value.tangentSpeedMps >= 0 && vectorFinite(value.velocityMps)
              && vectorFinite(value.tangentVelocityMps) && vectorFinite(value.point) && vectorFinite(value.normal))) throw new Error('Finite current material-point contact/slip contract missing');
          if (foot.loadN >= C.triageMinimumFootLoadN) {
            const slip = e.materialSlip[foot.body], speed = foot.impulseWeightedTangentSpeedMps;
            if (speed === null) slip.unavailableLoadedRows++;
            else { slip.observedLoadedRows++; slip.sampledImpulseWeightedTangentTravelM += speed / 800;
              slip.maximumImpulseWeightedTangentSpeedMps = Math.max(slip.maximumImpulseWeightedTangentSpeedMps, speed); }
          }
        }
        if (Number.isFinite(p.since_impact_s)) {
          const impact = n - Math.round(p.since_impact_s * 800);
          if (e.originalImpactNative === null || impact !== e.originalImpactNative) throw new Error('Policy renewed original physical impact deadline');
        }
        if (e.controllerStartedAtS === null && Number.isFinite(p.started_at_s)) e.controllerStartedAtS = p.started_at_s;
        if (e.controllerStartedAtS !== null && p.started_at_s !== e.controllerStartedAtS) throw new Error('Controller start epoch changed in no-retry world');
        if (Number.isFinite(p.started_at_s) && p.reference_frames > 0) {
          if (e.referenceFrames !== null && e.referenceFrames !== p.reference_frames) throw new Error('Reference count changed');
          e.referenceFrames = p.reference_frames;
          e.expectedFirstHoldingNative ??= Math.round(p.started_at_s * 800) + p.reference_frames * 20;
        }
        if (p.phase === 'slow-holding' && e.firstHoldingNative === null) {
          e.firstHoldingNative = n; e.firstHoldingCommandInputNative = row.commandInputNativeSubstep;
          e.triageDeadlineNative = row.commandInputNativeSubstep + C.triageAfterHoldingNative;
        }
        // The progress gate deliberately has no extra persistence/head/full-
        // motion/anchor-drift requirement. Those remain original qualification.
        const computedProgressNumerics = {
          summedFootLoadN: d.footIds.reduce((sum, i) => sum + d.loadsN[i], 0),
          eightyPercentBodyWeightN: C.triageMinimumFootWeightFraction * row.weightN,
          jsFootLoadFractionDiagnosticOnly: d.footIds.reduce((sum, i) => sum + d.loadsN[i], 0) >= C.triageMinimumFootWeightFraction * row.weightN,
          jsOnlyFeetDiagnosticOnly: d.loadsN.every((load, i) => d.footIds.includes(i) || load < 3),
          jsTorsoUpDiagnosticOnly: d.torsoUp >= C.triageTorsoUp,
        };
        const progressPredicates = {
          eachFootCurrentLoadAtLeast3N: d.footIds.every(i => d.loadsN[i] >= C.triageMinimumFootLoadN),
          currentFootLoadAtLeast80PercentBodyWeight: d.predicates.footLoadFraction === true,
          everyCurrentNonfootLoadBelow3N: d.predicates.onlyFeet === true,
          torsoUpAtLeast097: d.predicates.torso === true,
          currentComInSupport: d.predicates.comInSupport === true,
          currentCaptureInSupport: d.predicates.captureInSupport === true,
        };
        const progress = Object.values(progressPredicates).every(value => value === true);
        const deadline = e.triageDeadlineNative ?? (e.expectedFirstHoldingNative === null ? null : e.expectedFirstHoldingNative + C.triageAfterHoldingNative);
        if (e.triage === null && (deadline === null || n <= deadline)) {
          e.triageConsecutive = progress ? e.triageConsecutive + 1 : 0;
          e.triageMaximumConsecutive = Math.max(e.triageMaximumConsecutive, e.triageConsecutive);
          if (e.triageConsecutive >= C.triageConsecutiveNative && e.triageFirst80Native === null) e.triageFirst80Native = n;
        }
        if (e.triage === null && deadline !== null && n >= deadline) {
          const passed = e.firstHoldingNative !== null && e.triageConsecutive >= C.triageConsecutiveNative;
          e.triage = { passed, deadlineNative: deadline, firstHoldingNative: e.firstHoldingNative, firstHoldingCommandInputNative: e.firstHoldingCommandInputNative,
            first80Native: e.triageFirst80Native, currentConsecutiveNative: e.triageConsecutive, maximumConsecutiveNative: e.triageMaximumConsecutive,
            acceptance: 'current consecutive streak>=80 at exact cutoff; historical maximum cannot pass',
            evaluatedThroughNative: deadline, progressPredicates: structuredClone(progressPredicates),
            computedProgressNumerics: structuredClone(computedProgressNumerics),
            currentPredicates: structuredClone(d.predicates), torsoUp: d.torsoUp,
            reason: passed ? 'current80consecutive-native-progress-at-first-Holding+0.5s' : e.firstHoldingNative === null ? 'actual-first-Holding-missing' : 'bounded-native-progress-not-earned' };
          e.triageAwaitingCapture = true; scheduleCapture('triage');
          if (!passed) e.stop('NEGATIVE-bounded-triage');
        }
        if (e.triageAwaitingCapture && e.triage && n > e.triage.deadlineNative) e.extraUncreditedRows.push(n);
        // All ten exact standing predicates, including actual original native
        // stable_feet, must be present in the native observer after an earned
        // stability interval. Null-anchor current predicates remain raw context.
        const earnedPass = p.prototype_standing_passed === true && p.stable_standing_s >= 1 && row.motion === 'Upright';
        if (e.firstPass === null && earnedPass) {
          if (e.originalImpactNative === null || n > e.originalImpactNative + C.recoveryDeadlineNative) throw new Error('Native pass outside unchanged original25s deadline');
          if (!row.originalStableFeet || !allPredicates(d.predicates)) throw new Error('Native pass lacks actual original-anchor native predicates');
          e.originalAnchors = structuredClone(row.originalStableFeet);
          e.firstPass = { native: n, startedAtS: p.started_at_s, stableS: p.stable_standing_s, counters: structuredClone(row.counters) };
          scheduleCapture('first-pass');
        }
        if (e.firstPass !== null && n <= e.firstPass.native + C.postPassNative) {
          const failed = [];
          if (!allPredicates(d.predicates)) failed.push('native-original-anchor-standing-predicates');
          if (!equal(row.originalStableFeet, e.originalAnchors)) failed.push('original-controller-stable_feet-changed');
          if (!earnedPass || Math.abs(p.stable_standing_s - e.firstPass.stableS - (n - e.firstPass.native) / 800) > 1e-9) failed.push('native-standing-latch-counter-continuity');
          if (p.started_at_s !== e.firstPass.startedAtS) failed.push('original-controller-epoch-changed');
          for (const name of ['strikes', 'falls', 'recoveries', 'steps']) if (row.counters[name] !== e.firstPass.counters[name]) failed.push('original-counter.' + name);
          e.postPassNativeRows++;
          if (failed.length) { e.standingViolations.push({ native: n, failed }); e.stop('NEGATIVE-original-anchor-counter-hold'); }
          if (n >= e.firstPass.native + C.postPassNative && e.endedAtMs === null) e.stop('thirty-physical-seconds-original-anchor-counter-hold');
        } else if (e.firstPass === null && e.triage?.passed && e.originalImpactNative !== null && n >= e.originalImpactNative + C.recoveryDeadlineNative && e.endedAtMs === null) e.stop('NEGATIVE-original25s-recovery-deadline');
        e.nativeTrace.push(structuredClone(row)); e.traceRows++; e.lastNative = n;
      }
      const Native = window.Worker;
      window.Worker = class extends Native {
        constructor(...args) {
          super(...args); e.workerInstances++;
          this.addEventListener('message', event => {
            const m = event.data;
            try {
              if (m.type === 'ready') e.workerIdentities.push(structuredClone(m.workerIdentity));
              if (m.type === 'error') { e.rawWorkerFaults.push(structuredClone(m)); e.stop('worker-error', m.message); return; }
              if (!m.reply) return;
              const f = m.fixtureTest, snapshot = m.reply.snapshot, n = Math.round(snapshot.simulation_time_s * 800);
              if (!f?.isolatedTestOnly || f.kind !== 'back' || f.preparationPollingNative !== 80 || !equal(f.profile, approvedProfile)) throw new Error('Original BACK80-native fixture/complete baseline profile missing');
              if (snapshot.simulation_time_s !== n / 800 || snapshot.stamp.generation !== 1 || !snapshot.diagnostics.getup?.learned_policy_loaded) throw new Error('Native physical clock/generation/explicit learned policy changed');
              e.profile = structuredClone(f.profile);
              const identity = f[C.fixtureIdentityField];
              const expectedIdentity = { experiment: C.experiment, variant: arm === 'baseline' ? C.baselineVariant : C.candidateVariant,
                checkpointResumeSupported: arm === 'baseline', nativeTrace: C.nativeTraceFormat, maxTicksPerCall: 1,
                poseWordLayout: 'per body: positionXYZ,quaternionXYZW,cachedLinearVelocityXYZ,cachedAngularVelocityXYZ',
                currentMotionSource: 'fresh physicalObservationWords and current.motion; pose velocity words are cache receipts' };
              if (!equal(identity, expectedIdentity)) throw new Error('Actual WASM build identity differs from required arm');
              if (e.buildIdentity && !equal(identity, e.buildIdentity)) throw new Error('Build identity changed'); e.buildIdentity = structuredClone(identity);
              if (f[C.fixtureTraceErrorField] !== null) throw new Error('Native observer error: ' + String(f[C.fixtureTraceErrorField]));
              if (f.triggerEvidence) {
                if (e.trigger !== null) throw new Error('Original trigger repeated'); e.trigger = structuredClone(f.triggerEvidence);
                if (!Number.isSafeInteger(e.trigger.native) || e.trigger.native <= 0 || e.trigger.native % 80 !== 0) throw new Error('Original80-native fixture trigger epoch invalid');
              }
              if (f.triggerNative !== null) {
                e.originalTriggerNative ??= f.triggerNative; e.originalImpactNative ??= f.triggerNative;
                if (e.originalTriggerNative !== f.triggerNative || e.originalImpactNative !== f.triggerNative || e.trigger?.native !== f.triggerNative) throw new Error('Original physical trigger/impact epoch changed');
              } else if (e.originalTriggerNative !== null) throw new Error('Original physical epoch erased');
              if (f.carryActivationEvidence) { if (e.activation !== null) throw new Error('Native0 carry activation repeated'); e.activation = structuredClone(f.carryActivationEvidence); }
              const rows = f[C.fixtureTraceField];
              if (!Array.isArray(rows) || rows.length > 20) throw new Error('Bounded native trace packet missing');
              const duplicate = e.lastPublicationNative === n;
              if (e.lastPublicationNative === null ? n !== 0 || m.reply.completed_ticks !== 0 : duplicate ? m.reply.completed_ticks !== 0 : n - e.lastPublicationNative !== 20 || m.reply.completed_ticks !== 1) throw new Error('Original automatic one-tick scheduler changed');
              if (e.lastNative === null) {
                const state = f.endpointNative0State;
                if (n !== 0 || !state || state.nativeSubsteps !== 0 || state.physicsIntegrations !== 0
                  || state.physicalObservationError !== null || state.integratorReadError !== null
                  || state.poseWords?.length !== 325 || state.physicalObservationWords?.length !== 67
                  || typeof state.rootTranslationIntegratorStateJson !== 'string'
                  || state.current?.nativeSubsteps !== 0 || state.current.physicsIntegrations !== 0
                  || state.current.motion?.length !== 25 || state.current.loadsN?.length !== 25
                  || !state.current.contactMotion?.feet) throw new Error('Explicit read-only native0 physical/generalized/integrator receipt missing');
                e.native0State = structuredClone(state); e.lastNative = 0;
                e.nativeTrace.push({ nativeSubsteps: 0, receiptKind: 'native0-read-only-state', ...structuredClone(state) }); e.traceRows++;
              }
              const expectedRows = n - e.lastNative;
              if (rows.length !== expectedRows) throw new Error('Native trace row count does not exactly cover original advancement');
              for (const row of rows) nativeRow(row);
              if (e.lastNative !== n) throw new Error('Native observer/publication clocks disagree');
              e.publications.push({ native: n, completedTicks: m.reply.completed_ticks, counters: structuredClone(snapshot.counters), motion: snapshot.motion });
              e.lastPublicationNative = n;
              if (e.firstPass) {
                const firstPublication = e.publications.find(value => value.native >= e.firstPass.native && value.motion === 'Upright');
                if (firstPublication) {
                  const expectedTicks = firstPublication.counters.upright_ticks + (n - firstPublication.native) / 20;
                  if (snapshot.counters.upright_ticks !== expectedTicks) e.stop('NEGATIVE-ordinary-upright-tick-counter');
                }
              }
              if (m.reply.failure) e.stop('physical-or-resource-halt');
            } catch (error) {
              e.rawObserverFault = { error: String(error), type: m?.type ?? null, requestId: m?.requestId ?? null,
                fixture: structuredClone(m?.fixtureTest ?? null), reply: structuredClone(m?.reply ?? null) };
              e.stop('instrumentation-error', String(error));
            }
          });
        }
        postMessage(message, ...args) {
          if (message.type === 'initialize') {
            message = { ...message, testFixedGetUpFixture: 'back', testFixedGetUpProfile: approvedProfile };
            e.initializations.push(structuredClone(message));
          }
          if (message.commands?.length) { e.commands.push(structuredClone(message.commands)); e.stop('unexpected-physical-command', 'Physical commands forbidden'); throw new Error('Physical commands forbidden'); }
          if (message.type === 'reset') { e.resetRequests.push(structuredClone(message)); e.stop('unexpected-reset', 'Reset forbidden'); throw new Error('Reset forbidden'); }
          return super.postMessage(message, ...args);
        }
      };
      for (const type of ['pointerdown', 'keydown', 'keyup']) document.addEventListener(type, event => {
        if (e.startedAtMs !== null && e.endedAtMs === null && (type !== 'pointerdown' || event.target?.closest?.('#viewport'))) {
          e.physicalInputs.push({ type, code: event.code ?? null, isTrusted: event.isTrusted }); e.stop('unexpected-physical-input', 'Physical input forbidden');
        }
      }, true);
    }, { arm, approvedProfile, C: CONTRACT });
    page = await context.newPage();
    page.on('pageerror', error => r.errors.push(String(error)));
    page.on('response', response => { if (response.status() >= 400) r.errors.push(`${response.status()} ${response.url()}`); });
    await page.goto(origin + '?renderer=webgl2', { timeout: CONTRACT.startupWallWatchdogMs });
    await page.waitForFunction(() => window.__fixtureEvidence?.error || window.__lhRust?.diagnostics.ready && window.__lhRust.diagnostics.graphics.frames >= 3,
      null, { timeout: CONTRACT.startupWallWatchdogMs });
    r.initial = await evalPage(() => window.__lhRust?.diagnostics);
    assert.equal(r.initial.tick, 0); assert.equal(r.initial.time, 0); assert.equal(r.initial.backend, 'WebGL2');
    const activation = await evalPage(arm => {
      const e = window.__fixtureEvidence, a = e.activation;
      if (e.error) throw new Error(e.error); if (!a) throw new Error('Native0 carry activation evidence missing');
      if (arm === 'candidate') return { metadata: structuredClone(a) };
      const encode = bytes => { let text = ''; for (let i = 0; i < bytes.length; i += 0x8000) text += String.fromCharCode(...bytes.subarray(i, i + 0x8000)); return btoa(text); };
      const beforeBase64 = encode(a.beforeCheckpoint), afterBase64 = encode(a.afterCheckpoint);
      delete a.beforeCheckpoint; delete a.afterCheckpoint;
      return { metadata: structuredClone(a), beforeBase64, afterBase64 };
    }, arm);
    r.activationProof = activation.metadata;
    assert.equal(r.activationProof.activationCalls, 1);
    assert.equal(r.activationProof.beforeReport.nativeSubsteps, 0); assert.equal(r.activationProof.afterReport.nativeSubsteps, 0);
    assert.deepEqual(r.activationProof.beforeReport.profile, approvedProfile); assert.deepEqual(r.activationProof.afterReport.profile, approvedProfile);
    assert.equal(r.activationProof.afterReport.rootTranslationCompensation.algorithm, 'f32-dq-f64-remainder-v1');
    assert.deepEqual(r.activationProof.poseVelocityF32WordsBefore, r.activationProof.poseVelocityF32WordsAfter);
    assert.equal(r.activationProof.poseVelocityF32WordsBefore.length, 325);
    assert.deepEqual(r.activationProof.beforeReport.rootRotationNormalization, r.activationProof.afterReport.rootRotationNormalization);
    if (arm === 'baseline') {
      const before = Buffer.from(activation.beforeBase64, 'base64'), after = Buffer.from(activation.afterBase64, 'base64');
      r.activationProof.independentlyDecodedPurity = proveActivationPurity(before, after);
      for (const [label, bytes] of [['before', before], ['after', after]]) {
        writeFileSync(join(armOutput, `activation-${label}-checkpoint.bin`), bytes);
        r.activationProof[label + 'Checkpoint'] = { bytes: bytes.length, sha256: hash(bytes), schema: parseGetUpCheckpoint(bytes).schema };
      }
      for (const value of [r.activationProof.beforeVerification, r.activationProof.afterVerification]) {
        assert.equal(value.passed, true); assert.equal(value.physicsIntegrations, 0);
        assert.equal(value.nativeSubsteps, 0); assert.equal(value.forecastSubsteps, 0); assert.equal(value.rootIntegratorStateEqual, true);
      }
    } else {
      // The candidate refuses checkpoint export/restore. Native0 read-only
      // receipts prove only the stated live physical/generalized purity. They
      // cannot establish binary-world or embedded-Runtime string equality.
      const before = r.activationProof.beforeNative0State, after = r.activationProof.afterNative0State;
      assert.ok(before && after, 'Candidate native0 receipt names await adapter confirmation');
      for (const value of [before, after]) {
        assert.equal(value.nativeSubsteps, 0); assert.equal(value.physicsIntegrations, 0);
        assert.equal(value.physicalObservationError, null); assert.equal(value.integratorReadError, null);
        assert.equal(value.poseWords.length, 325); assert.equal(value.physicalObservationWords.length, 67);
      }
      assert.deepEqual(before.poseWords, after.poseWords); assert.deepEqual(before.physicalObservationWords, after.physicalObservationWords);
      assert.deepEqual(r.activationProof.beforeCurrent, r.activationProof.afterCurrent);
      assert.deepEqual(r.activationProof.beforeReport.jointQ, r.activationProof.afterReport.jointQ);
      assert.deepEqual(r.activationProof.beforeReport.jointDq, r.activationProof.afterReport.jointDq);
      assert.deepEqual(r.activationProof.beforeReport.solverBudget, r.activationProof.afterReport.solverBudget);
      assert.equal(r.activationProof.binaryCheckpointPurityClaim, false);
      assert.equal(r.activationProof.beforeCheckpoint, null); assert.equal(r.activationProof.afterCheckpoint, null);
      const beforeCarry = JSON.parse(before.rootTranslationIntegratorStateJson), afterCarry = JSON.parse(after.rootTranslationIntegratorStateJson);
      assert.equal(beforeCarry.length, 1); assert.equal(afterCarry.length, 1);
      assert.equal(beforeCarry[0].state, null); assert.deepEqual(beforeCarry[0].root, [0, 0]);
      assert.deepEqual(afterCarry[0].root, [0, 0]); assert.deepEqual(afterCarry[0].state.remainderbits, [0, 0, 0]);
      assert.equal(afterCarry[0].state.qbits.length, 3);
      assert.equal(r.activationProof.checkpointExportRefused, true, 'Save explicit native candidate checkpoint-export refusal');
      assert.equal(r.activationProof.checkpointRestoreRefused, true, 'Save explicit native candidate checkpoint-restore refusal');
      assert.ok(Array.isArray(r.activationProof.checkpointExportRefusal)
        && r.activationProof.checkpointExportRefusal.length === 2
        && r.activationProof.checkpointExportRefusal.every(value => typeof value === 'string' && value.includes('does not support checkpoint export')));
      assert.equal(typeof r.activationProof.checkpointRestoreRefusal, 'string');
      assert.ok(r.activationProof.checkpointRestoreRefusal.includes('does not support checkpoint resume'));
      r.activationProof.binaryWorldPurityClaim = false;
    }
    r.checkpointDisposition = { resumeSupported: arm === 'baseline', candidateCheckpointExportRestoreVerifyAttemptedByRecorder: false,
      candidateNativeRefusals: arm === 'candidate' ? { export: r.activationProof.checkpointExportRefusal, restore: r.activationProof.checkpointRestoreRefusal } : null,
      limitation: arm === 'candidate' ? 'Checkpoint export and restore deliberately refused; native0 physical/generalized receipts only, no binary-world purity claim.' : null };
    json(join(armOutput, 'activation-proof.json'), r.activationProof);
    await page.screenshot({ path: join(armOutput, 'native0-initial-back.png'), fullPage: true });
    await evalPage(() => {
      const e = window.__fixtureEvidence; if (e.error || e.endedAtMs !== null) throw new Error(e.error || e.stopReason);
      window.__lhRust.drainTelemetry(); e.startedAtMs = performance.now();
      e.initialFrames = window.__lhRust.diagnostics.graphics.frames; window.__lhRust.setManual(false);
    });
    let lastNative = 0, lastProgressAt = Date.now(), nextProgressNative = 0;
    const startedAt = Date.now();
    while (true) {
      await page.waitForTimeout(250);
      const update = await withWallWatchdog(evalPage(() => {
        const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
        if (d.workerFault || d.disposed || d.halted || d.failure || d.error) e.stop(d.workerFault || d.disposed ? 'original-worker-terminated' : 'physical-resource-renderer-halt', d.error || null);
        const rows = e.nativeTrace.splice(0);
        return { native: Math.round(d.time * 800), time: d.time, motion: d.motion, policy: d.snapshot?.diagnostics.getup?.policy,
          workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed), error: e.error || d.error,
          stopReason: e.stopReason, ended: e.endedAtMs !== null, pending: [...e.pendingControllerCaptures],
          triage: e.triage, traceRows: e.traceRows, nativeTrace: rows };
      }), CONTRACT.exportWallWatchdogMs, 'read-only status/drain');
      drain(update);
      if (update.native !== lastNative) { lastProgressAt = Date.now(); lastNative = update.native; }
      if (update.native >= nextProgressNative || update.ended || update.error || update.pending.length) {
        r.progress.push(update); console.log(JSON.stringify({ finding: 'terminal-tail-ab', arm, ...update })); nextProgressNative = update.native + 800;
      }
      if (r.errors.length) await evalPage(message => window.__fixtureEvidence.stop('page-or-resource-error', message), r.errors.join('\n'));
      if (update.error || update.workerFault || update.disposed || r.errors.length) break;
      for (const label of update.pending) {
        const controller = await captureController(label);
        if (!controller) break;
        await evalPage(label => {
          const e = window.__fixtureEvidence;
          if (e.pendingControllerCaptures[0] !== label) throw new Error('Controller capture queue order changed');
          e.pendingControllerCaptures.shift(); e.capturedControllerLabels.push(label);
          if (label === 'triage') e.triageAwaitingCapture = false;
        }, label);
      }
      const afterCapture = await evalPage(() => {
        const e = window.__fixtureEvidence;
        if (!e.error && e.endedAtMs === null && !e.pendingControllerCaptures.length) window.__lhRust.setManual(false);
        return { ended: e.endedAtMs !== null, error: e.error };
      });
      if (afterCapture.ended || afterCapture.error) break;
      if (Date.now() - lastProgressAt >= CONTRACT.noProgressWallWatchdogMs || Date.now() - startedAt >= CONTRACT.totalWallWatchdogMs) {
        await evalPage(() => window.__fixtureEvidence.stop('native-progress-wall-watchdog', 'Bounded evidence watchdog; no physical epoch extension')); break;
      }
    }
    await evalPage(() => window.__lhRust.setManual(true));
    const availability = await evalPage(() => {
      const d = window.__lhRust.diagnostics, e = window.__fixtureEvidence;
      return { workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed), observerError: e.error };
    });
    if (!availability.workerFault && !availability.disposed && !availability.observerError) await captureController('stop');
    else r.stopExportUnavailable = { ...availability, reason: 'Preserve original fault; skip all controller/checkpoint exports and never resurrect worker' };
    r.final = await evalPage(() => window.__lhRust.diagnostics);
    r.evidence = await evalPage(() => { const { stop, nativeTrace, ...rest } = window.__fixtureEvidence; return { ...rest, remainingTrace: nativeTrace }; });
    drain({ nativeTrace: r.evidence.remainingTrace }); delete r.evidence.remainingTrace;
    const e = r.evidence;
    prefixDigest = prefixHasher.digest('hex');
    r.nativeTrace.prefixBeforeEndpointLookahead = { rows: nativePrefixRows, sha256: prefixDigest,
      firstEndpointLookaheadCommandInputNative: nativeFirstEndpointLookahead === null ? null : nativeFirstEndpointLookahead - 1,
      firstAffectedNativeRow: nativeFirstEndpointLookahead };
    r.nativeTrace.sha256 = hash(readFileSync(tracePath));
    r.triage = e.triage;
    r.thirtySecondStandingObserved = Boolean(e.triage?.passed && e.firstPass && e.postPassNativeRows >= CONTRACT.postPassNative + 1
      && e.lastNative >= e.firstPass.native + CONTRACT.postPassNative && !e.standingViolations.length
      && e.stopReason === 'thirty-physical-seconds-original-anchor-counter-hold');
    r.outcome = e.triage?.passed === false ? 'NEGATIVE-bounded-triage'
      : r.thirtySecondStandingObserved ? 'native1s-pass-plus30s-original-anchor-counter-hold'
      : e.stopReason?.startsWith('NEGATIVE-') ? e.stopReason : 'physical-observer-or-resource-failure';
    assert.equal(e.workerInstances, 1); assert.equal(e.workerIdentities.length, 1); assert.equal(e.initializations.length, 1);
    assert.deepEqual(e.commands, []); assert.deepEqual(e.physicalInputs, []); assert.deepEqual(e.resetRequests, []);
    assert.deepEqual(e.profile, approvedProfile); assert.equal(e.initializations[0].profile, undefined);
    assert.equal(e.trigger?.physicalPoseVelocityBitsUnchanged, true); assert.deepEqual(e.trigger.beforePoseWords, e.trigger.afterPoseWords);
    assert.equal(r.nativeTrace.native0Included, true); assert.equal(r.nativeTrace.rows, e.lastNative + 1);
    assert.equal(e.error, null); assert.deepEqual(r.errors, []);
    assert.ok(e.extraUncreditedRows.length <= 19);
    assert.equal(Boolean(r.final.workerFault), false); assert.equal(Boolean(r.final.disposed), false);
    assert.equal(r.final.error, ''); assert.equal(r.final.failure, null);
    r.observationValid = true;
    r.passed = r.thirtySecondStandingObserved;
    await page.screenshot({ path: join(armOutput, r.passed ? 'after30physicalseconds.png' : 'NEGATIVE.png'), fullPage: true });
  } catch (error) {
    r.error = error.stack ?? String(error);
    if (page && !page.isClosed()) {
      try {
        await withWallWatchdog(evalPage(message => {
          const e = window.__fixtureEvidence;
          if (e) { e.recorderError = message; e.stop('recorder-error'); }
        }, String(error)), 5000, 'failure stop');
        r.failureState = await withWallWatchdog(evalPage(() => window.__lhRust?.diagnostics), 5000, 'failure state');
        r.evidence = await withWallWatchdog(evalPage(() => {
          const { stop, nativeTrace, ...rest } = window.__fixtureEvidence ?? {}; return { ...rest, remainingTrace: nativeTrace ?? [] };
        }), 5000, 'raw failure evidence');
        drain({ nativeTrace: r.evidence.remainingTrace }); delete r.evidence.remainingTrace;
        await withWallWatchdog(page.screenshot({ path: join(armOutput, 'failure.png'), fullPage: true }), 5000, 'failure screenshot');
      } catch (captureError) { r.rawFailureCaptureError = String(captureError); }
    }
    if (!r.outcome || r.outcome === 'NOT_STARTED') r.outcome = 'physical-observer-or-resource-failure';
  } finally {
    r.finished = new Date().toISOString();
    if (prefixDigest === null) {
      prefixDigest = prefixHasher.digest('hex');
      r.nativeTrace.prefixBeforeEndpointLookahead = { rows: nativePrefixRows, sha256: prefixDigest,
        firstEndpointLookaheadCommandInputNative: nativeFirstEndpointLookahead === null ? null : nativeFirstEndpointLookahead - 1,
        firstAffectedNativeRow: nativeFirstEndpointLookahead };
    }
    r.nativeTrace.sha256 = hash(readFileSync(tracePath));
    if (r.evidence?.triage?.passed === false) r.outcome = 'NEGATIVE-bounded-triage';
    // Save raw result before any browser cleanup. No failed worker export awaits.
    json(join(armOutput, 'result.json'), r); json(join(output, 'result.json'), report);
    try { await withWallWatchdog(context?.close() ?? Promise.resolve(), 5000, 'context close'); } catch (error) { r.cleanupError = String(error); }
    try { await withWallWatchdog(browser?.close() ?? Promise.resolve(), 5000, 'browser close'); } catch (error) { r.cleanupError ??= String(error); }
    server?.kill();
    json(join(armOutput, 'result.json'), r);
  }
  return r;
}

try {
  await observeArm('baseline'); await observeArm('candidate');
  const a = report.arms.baseline, b = report.arms.candidate;
  const controller = (r, label) => r.controllerCaptures[label]?.path
    ? JSON.parse(readFileSync(join(output, r.arm, r.controllerCaptures[label].path))) : null;
  const ac = controller(a, 'triage'), bc = controller(b, 'triage');
  const prefix = ac && bc ? { teacherFramesEqual: equal(ac.teacher_frames, bc.teacher_frames),
    teacherInitialEqual: equal(ac.teacher_initial, bc.teacher_initial), teacherNativeDtEqual: ac.teacher_native_dt_s === bc.teacher_native_dt_s,
    forecastSubstepsEqual: ac.status.forecast_substeps === bc.status.forecast_substeps,
    referenceCountsEqual: ac.reference.length === bc.reference.length && ac.status.reference_frames === bc.status.reference_frames,
    completeInterpolatedPrefixEqual: equal(ac.reference.slice(0, -1), bc.reference.slice(0, -1)),
    duplicateFinalTupleExact: equal(bc.reference.at(-1), bc.reference.at(-2)),
    onlyAppendedTupleDiffers: equal(ac.reference.slice(0, -1), bc.reference.slice(0, -1)) && !equal(ac.reference.at(-1), bc.reference.at(-1)),
    teacherPowerAndKpKdClaim: 'Build/source attestation plus identical teacher/prefix; no profile/gain/timing change permitted',
    baselineReferenceFrames: ac.status.reference_frames, candidateReferenceFrames: bc.status.reference_frames,
    baselineOriginalImpactNative: a.evidence?.originalImpactNative, candidateOriginalImpactNative: b.evidence?.originalImpactNative } : null;
  report.comparison = { available: Boolean(prefix), controllerPrefix: prefix,
    nativePrefixBeforeEndpointLookaheadEqual: Boolean(a.nativeTrace.prefixBeforeEndpointLookahead && b.nativeTrace.prefixBeforeEndpointLookahead
      && equal(a.nativeTrace.prefixBeforeEndpointLookahead, b.nativeTrace.prefixBeforeEndpointLookahead)),
    originalDriverEqual: equal(a.originalDriver, b.originalDriver), frozenSourceFilesEqual: equal(a.frozenSourceFiles, b.frozenSourceFiles), completeProfileEqual: equal(a.evidence?.profile, b.evidence?.profile),
    observationOnly: true, candidateCausalOrPerformanceQualification: false };
  report.recordingComplete = Boolean(a.observationValid && b.observationValid && prefix && prefix.teacherFramesEqual && prefix.teacherInitialEqual
    && prefix.teacherNativeDtEqual && prefix.forecastSubstepsEqual && prefix.referenceCountsEqual
    && prefix.completeInterpolatedPrefixEqual && prefix.duplicateFinalTupleExact && prefix.onlyAppendedTupleDiffers
    && report.comparison.nativePrefixBeforeEndpointLookaheadEqual && report.comparison.originalDriverEqual && report.comparison.frozenSourceFilesEqual && report.comparison.completeProfileEqual);
  report.candidateStandingObserved = b.passed === true;
  report.passed = report.recordingComplete && report.candidateStandingObserved;
} catch (error) { report.error = error.stack ?? String(error); }
finally {
  report.finished = new Date().toISOString(); json(join(output, 'result.json'), report);
  console.log(JSON.stringify({ passed: report.passed, qualificationClaim: false, performanceClaim: false,
    baseline: report.arms.baseline?.outcome, candidate: report.arms.candidate?.outcome, comparison: report.comparison, output }, null, 2));
  // All evidence writes above are synchronous; never leave the CLI hanging on
  // an already-failed browser transport or cleanup promise.
  process.exit(report.passed ? 0 : 1);
}
