// SOURCE-ONLY, unconfirmed preparation. No browser, build, dynamics or actor call has been run.
// Bounded causal diagnostic only: fresh BACK worlds, same complete baseline profile,
// original renderer/driver/transport/automatic one-tick scheduler. Candidate zeros
// ONLY the applied right_ankle_y motor torque for completed natives2181..2200.
// Both arms stop exactly at2380 (2.975s), irrespective of diagnostic sign.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { pathToFileURL } from 'node:url';

// INTERFACE CONSTANTS. Fail closed until adapter/native owners confirm every
// name and native0/original-anchor row guarantee. No permissive fallback aliases.
export const CONTRACT = Object.freeze({
  confirmed: true,
  reviewRequired: 'Native/adapter exact identity, applied-torque and independent sole separation contract awaiting owner review.',
  fixtureTraceField: 'ablationNativeTrace', fixtureIdentityField: 'ablationBuildIdentity', fixtureTraceErrorField: 'ablationTraceError', fixtureNative0Field: 'ablationNative0State',
  fixtureReferenceField: 'ablationReferenceEvidence',
  nativeTraceFormat: 'read-only-post-native-observer-v1', experiment: 'right-ankle-y-braking-ablation-v1',
  baselineVariant: 'source-normal-motors', candidateVariant: 'zero-applied-right-ankle-y-2181-2200',
  commonBuildFeatures: [
    'lh-worker/root-translation-carry-experiment',
    'lh-sim/standing-dynamics-experiment', 'lh-sim/unit-root-rotation-experiment',
    'lh-sim/getup-repeat-cycle-experiment', 'lh-worker/getup-terminal-tail-observer-experiment',
  ],
  candidateBuildFeature: 'lh-sim/getup-right-ankle-braking-ablation-experiment',
  baselineEffectiveSimFeatures: ['root-translation-carry-experiment', 'getup-terminal-tail-observer-experiment',
    'standing-dynamics-experiment', 'unit-root-rotation-experiment', 'getup-repeat-cycle-experiment', 'getup-terminal-balance-experiment'],
  sourceFreezePath: 'outputs/getup-ankle-braking-ablation-20261019/source-freeze.json',
  sourceFreezeSha256: '4b49156d9c339e3df96b9b0e74aebabac326386d3118ba3030fa7bafc7fb07da',
  candidateCheckpointRefusal: 'Checkpoint export/resume is disabled for the right-ankle braking ablation build',
  nativeHz: 800, publicationSubsteps: 20, preparationPollingNative: 80,
  firstAffectedNative: 2181, lastAblatedNative: 2200, restoredNative: 2201,
  stopNative: 2380, stopPhysicalSeconds: 2.975,
  actuatorIndex: 8, qposIndex: 7, actuatorId: 'right_ankle_y', gear: 20,
  minimumLoadedN: 3, unloadingConsecutiveNative: 20,
  // Exact native row keys. Source review is still pending; confirmed stays false.
  ablationRowField: 'motorApplication', intendedTorqueField: 'intendedTorqueNm', appliedTorqueField: 'appliedTorqueNm',
  overrideAppliedField: 'overrideApplied',
  separationField: 'footFloorClearanceM',
  separationSource: 'minimum transformed source primitive endpoint Y minus radius minus actual floor collider AABB maximum Y',
  separationThresholdM: 0,
  exportWallWatchdogMs: 30_000, startupWallWatchdogMs: 60_000,
  noProgressWallWatchdogMs: 60_000, totalWallWatchdogMs: 5 * 60_000,
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
  assert.deepEqual([...adapter.sourceBuildFeatures].sort(), expectedFeatures.sort(), 'Only reviewed read-only observer plus candidate actuator8 ablation feature');
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
  // Cargo CLI omits terminal-balance because repeat-cycle enables it implicitly.
  // Attest the exact effective lh-sim feature fingerprint separately.
  const effectiveSimFeatures = [...CONTRACT.baselineEffectiveSimFeatures,
    ...(arm === 'candidate' ? [CONTRACT.candidateBuildFeature.split('/')[1]] : [])].sort();
  const fingerprint = frozen.simFingerprints.find(value => equal([...JSON.parse(value.features)].sort(), effectiveSimFeatures));
  assert.ok(fingerprint, 'Exact effective sim features exclude all policy/actor/reference edits and unreviewed features');
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
  assert.equal(source.overrideSourceFreeze, CONTRACT.sourceFreezePath);
  assert.equal(source.overrideSourceFreezeSha256, CONTRACT.sourceFreezeSha256);
  const freezeBytes = readFileSync(resolve(CONTRACT.sourceFreezePath));
  assert.equal(hash(freezeBytes), CONTRACT.sourceFreezeSha256, 'Exact native owner source freeze required');
  const freeze = JSON.parse(freezeBytes);
  assert.deepEqual([...freeze.featuresBaseline].sort(), [...CONTRACT.commonBuildFeatures].sort());
  assert.equal(freeze.additionalCandidateFeature, CONTRACT.candidateBuildFeature);
  assert.equal(source.sourceBase, freeze.base);
  for (const entry of freeze.files) assert.equal(source.sourceFiles[entry.path], entry.sha256, 'Frozen copied source differs from reviewed override: ' + entry.path);
  assert.equal(source.profileSha256, profileSha256);
  for (const [path, sha] of Object.entries(source.sourceFiles)) assert.equal(hash(readFileSync(join(sourceRoot, path))), sha);
  const drivers = Object.keys(artifactSha256).filter(path => path.endsWith('/driver.js'));
  assert.equal(drivers.length, 1);
  assert.equal(artifactSha256[drivers[0]], hash(readFileSync(resolve('rust/crates/browser/driver.js'))));
  return { artifactSha256, expectedFeatures, adapterManifest: { path: adapterPath, sha256: hash(adapterBytes) },
    frozenBuildManifest: { path: frozenPath, sha256: hash(frozenBytes) },
    frozenSourceManifest: { path: sourcePath, sha256: hash(sourceBytes), verifiedFiles: Object.keys(source.sourceFiles).length },
    originalDriver: { path: drivers[0], sha256: artifactSha256[drivers[0]], byteEqual: true },
    frozenSourceFiles: source.sourceFiles, sourceFreezeSha256: CONTRACT.sourceFreezeSha256, effectiveSimFeatures, effectiveSimFingerprint: fingerprint };
}


const field = (value, path) => String(path).split('.').reduce((v, k) => v?.[k], value);
const finiteVector = value => value && ['x', 'y', 'z'].every(k => Number.isFinite(value[k]));
const words = (value, count) => Array.isArray(value) && value.length === count
  && value.every(n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff);
const finiteArray = (value, count) => Array.isArray(value) && value.length === count && value.every(Number.isFinite);
const f32FromWord = word => { const bytes = new ArrayBuffer(4), view = new DataView(bytes); view.setUint32(0, word, true); return view.getFloat32(0, true); };
const signedExtremum = () => ({ minimum: null, minimumNative: null, maximum: null, maximumNative: null });
const extend = (value, x, n) => {
  assert.ok(Number.isFinite(x), 'Nonfinite observable');
  if (value.minimum === null || x < value.minimum) { value.minimum = x; value.minimumNative = n; }
  if (value.maximum === null || x > value.maximum) { value.maximum = x; value.maximumNative = n; }
};

// Pure source-only checks. This function does not create a world or call actors.
// Per-row readbacks are authoritative; aggregate override counts alone cannot pass.
export function analyzeTrace(rows, arm, C = CONTRACT) {
  assert.ok(['baseline', 'candidate'].includes(arm));
  assert.equal(rows.length, C.stopNative + 1, 'Full native0 plus1..2380 trace required');
  const result = { valid: false, diagnosticOnly: true, finalNative: null, rows: rows.length,
    overrideNatives: [], firstLeftLoadBelow3N: null, firstLeftLoadBelow3NAfterIntervention: null,
    firstUnloaded20Native: null, firstUnloaded20NativeAfterIntervention: null, firstCompletedLeftLoadBelow3N: null,
    streakCounting: 'Consecutive completed native samples1..2380; read-only native0 is recorded separately and adds no duration.',
    currentUnloadedStreak: 0, maximumUnloadedStreak: 0, unloadingIntervals: [],
    soleFloorSeparation: { firstPositiveNative: null, positiveRows: 0, nonpositiveRows: 0, extremaM: signedExtremum(), thresholdM: C.separationThresholdM },
    unloadedSeparation: { firstSeparatedNative: null, rows: 0, separatedRows: 0, unseparatedRows: 0, minimumM: null, maximumM: null,
      thresholdM: C.separationThresholdM, source: C.separationSource,
      inferenceFromAbsentPositiveImpulseContacts: false },
    extrema: { leftLoadN: signedExtremum(), rightLoadN: signedExtremum(), nonfootLoadN: signedExtremum(),
      torsoUp: signedExtremum(), headM: signedExtremum(), totalFootLoadFraction: signedExtremum(),
      maximumBodyLinearMps: signedExtremum(), maximumBodyAngularRadps: signedExtremum(), requestedMotorEffortNm: signedExtremum(),
      intendedTorqueNm: Array.from({ length: 21 }, signedExtremum), appliedTorqueNm: Array.from({ length: 21 }, signedExtremum),
      integrityLimits: Object.fromEntries(['maxAnchorSeparationM', 'maxFloorPenetrationM', 'maxSelfPenetrationM', 'maxLimitErrorRad'].map(k => [k, signedExtremum()])),
      jointAnglesRad: Array.from({ length: 21 }, signedExtremum), jointVelocitiesRadps: Array.from({ length: 21 }, signedExtremum) },
    jointExtremaSource: 'Fresh post-native physicalObservationWords: qpos[0..21), dq[46..67), f32 bits decoded exactly. Source frozen getup.rs physical_observation.',
    controllerJointReceiptInterpretation: 'Raw getup.joint_angles_rad / joint_velocities_radps belong to command-input n-1, before integration; they are archived but not used for post-native joint extrema.',
    limits: { normalizedMotorControlAbsMaximum: 0, requestedMotorEffortNmMaximumAllowed: 920.001,
      integrityLimitsInterpretation: 'Existing cumulative maxima through each native, not instantaneous or interval-local values. Never subtract maxima to derive interval values.',
      directPelvisForceAndTorqueAlwaysZero: true, jointPositionRangeChecks: 'Raw per-joint extrema retained; no invented hard-limit threshold' },
    windows: {}, materialSlip: Object.fromEntries(['left_foot', 'right_foot'].map(body => [body, {
      loadedRows: 0, unavailableLoadedRows: 0, observedLoadedRows: 0, sampledImpulseWeightedTangentTravelM: 0, maximumImpulseWeightedTangentSpeedMps: 0 }])),
    materialSlipInterpretation: 'Current loaded material-contact tangent speed times1/800, with unavailable values explicit; not foot-origin travel.' };
  for (const name of ['full', 'preIntervention', 'ablated', 'restored']) result.windows[name] = {
    nativeRange: name === 'full' ? [1, C.stopNative] : name === 'preIntervention' ? [1, C.firstAffectedNative - 1]
      : name === 'ablated' ? [C.firstAffectedNative, C.lastAblatedNative] : [C.restoredNative, C.stopNative],
    rows: 0, leftUnloadedRows: 0, bothFeetLoadedRows: 0, comSupportedRows: 0, captureSupportedRows: 0,
    onlyFeetRows: 0, linearMotionPassedRows: 0, angularMotionPassedRows: 0, motionCounts: {},
    firstRow: null, lastRow: null,
    extrema: { leftLoadN: signedExtremum(), rightLoadN: signedExtremum(), leftClearanceM: signedExtremum(), rightClearanceM: signedExtremum(),
      torsoUp: signedExtremum(), headM: signedExtremum(), maximumBodyLinearMps: signedExtremum(), maximumBodyAngularRadps: signedExtremum(),
      intendedTorqueNm: Array.from({ length: 21 }, signedExtremum), appliedTorqueNm: Array.from({ length: 21 }, signedExtremum),
      jointAnglesRad: Array.from({ length: 21 }, signedExtremum), jointVelocitiesRadps: Array.from({ length: 21 }, signedExtremum) },
    materialSlip: Object.fromEntries(['left_foot', 'right_foot'].map(body => [body, { loadedRows: 0, unavailableLoadedRows: 0, observedLoadedRows: 0,
      sampledImpulseWeightedTangentTravelM: 0, maximumImpulseWeightedTangentSpeedMps: 0 }])),
    cumulativeIntegrityLimitReadbacks: { interpretation: 'Cumulative maxima through n. These are readbacks within this window, not interval-local extrema; no subtraction.', first: null, last: null },
  };
  const zero = rows[0];
  assert.equal(zero.nativeSubsteps, 0); assert.equal(zero.physicsIntegrations, 0);
  assert.equal(zero.physicalObservationError, null); assert.equal(zero.integratorReadError, null);
  assert.ok(words(zero.poseWords, 325) && words(zero.physicalObservationWords, 67));
  assert.equal(typeof zero.rootTranslationIntegratorStateJson, 'string');
  assert.equal(zero.current?.nativeSubsteps, 0); assert.equal(zero.current?.physicsIntegrations, 0);
  assert.equal(zero.snapshot?.simulation_time_s, 0); assert.equal(zero.snapshot?.stamp?.generation, 1);
  assert.ok(zero.snapshot.diagnostics?.getup && zero.snapshot.counters && typeof zero.snapshot.motion === 'string', 'Full native0 policy/control/counter/motion snapshot required');
  assert.ok(finiteArray(zero.current.loadsN, 25) && Array.isArray(zero.current.footIds) && zero.current.footIds.length === 2);
  result.native0LeftLoadN = zero.current.loadsN[zero.current.footIds[0]];
  result.native0LeftLoadBelow3N = result.native0LeftLoadN < C.minimumLoadedN;
  if (result.native0LeftLoadBelow3N) result.firstLeftLoadBelow3N = 0;
  let streak = 0, postInterventionStreak = 0, interval = null, expectedCount = 0, startEpoch = null, impactEpoch = null;
  for (let n = 1; n <= C.stopNative; n++) {
    const row = rows[n], d = row.current, p = row.getup?.policy;
    assert.equal(row.nativeSubsteps, n, 'Trace gap/duplicate/out-of-order at' + n);
    assert.equal(row.commandInputNativeSubstep, n - 1); assert.equal(row.physicalSeconds, n / C.nativeHz);
    assert.ok(words(row.poseWords, 325) && words(row.physicalObservationWords, 67), 'Missing exact physical words at' + n);
    const currentQ = row.physicalObservationWords.slice(0, 21).map(f32FromWord), currentDq = row.physicalObservationWords.slice(46, 67).map(f32FromWord);
    assert.ok(currentQ.every(Number.isFinite) && currentDq.every(Number.isFinite), 'Nonfinite fresh post-native joint state');
    assert.equal(d?.physicsIntegrations, 0); assert.equal(d?.nativeSubsteps, n);
    assert.ok(d.predicates && finiteArray(d.loadsN, 25) && finiteArray(d.supportPersistence, 25)
      && Array.isArray(d.motion) && d.motion.length === 25 && Array.isArray(d.solverSupportPoints)
      && finiteVector(d.com) && finiteVector(d.comVelocity) && finiteVector(d.capture));
    assert.ok(Array.isArray(d.footIds) && d.footIds.length === 2 && new Set(d.footIds).size === 2);
    assert.ok(d.footIds.every(i => Number.isSafeInteger(i) && i >= 0 && i < 25));
    assert.ok(Number.isFinite(row.weightN) && row.weightN > 0 && p && row.counters && typeof row.motion === 'string');
    assert.ok(finiteArray(row.getup.motor_controls, 21) && row.getup.motor_controls.every(v => Math.abs(v) <= 1));
    assert.ok(finiteArray(row.getup.joint_angles_rad, 21) && finiteArray(row.getup.joint_velocities_radps, 21));
    assert.equal(row.getup.learned_policy_loaded, true);
    assert.ok(Number.isFinite(row.requestedMotorEffortNm) && row.requestedMotorEffortNm >= 0 && row.requestedMotorEffortNm <= 920.001);
    assert.equal(row.directPelvisForceN, 0); assert.equal(row.directPelvisTorqueNm, 0);
    assert.equal(typeof row.rootTranslationIntegratorStateJson, 'string');
    if (row.originalStableFeet === null) {
      assert.equal(d.footDriftM, null); assert.equal(d.predicates.footDriftFromSuppliedAnchors, null);
    } else assert.ok(Array.isArray(row.originalStableFeet) && row.originalStableFeet.length === 2 && finiteArray(d.footDriftM, 2));
    if (Number.isFinite(p.started_at_s)) { startEpoch ??= p.started_at_s; assert.equal(p.started_at_s, startEpoch, 'No current-trial retry or epoch renewal'); }
    if (Number.isFinite(p.since_impact_s)) {
      const inferredImpact = n - Math.round(p.since_impact_s * C.nativeHz);
      impactEpoch ??= inferredImpact; assert.equal(inferredImpact, impactEpoch, 'Original physical impact epoch renewed');
    }
    const a = field(row, C.ablationRowField), intended = field(a, C.intendedTorqueField), applied = field(a, C.appliedTorqueField);
    assert.ok(a && finiteArray(intended, 21) && finiteArray(applied, 21)
      && finiteArray(a.intendedControls, 21) && finiteArray(a.appliedControls, 21)
      && a.intendedControls.every(v => Math.abs(v) <= 1) && a.appliedControls.every(v => Math.abs(v) <= 1), 'Separate native intended/applied actuator torque readbacks required at' + n);
    const active = arm === 'candidate' && n >= C.firstAffectedNative && n <= C.lastAblatedNative;
    const actualOverride = field(a, C.overrideAppliedField);
    assert.equal(actualOverride, active, 'Per-native override receipt differs at' + n);
    if (actualOverride) { result.overrideNatives.push(n); expectedCount++; }
    const intendedEffort = intended.reduce((sum, tau) => sum + Math.abs(tau), 0);
    assert.ok(Math.abs(row.requestedMotorEffortNm - intendedEffort) <= 1e-3, 'Requested effort must retain original intended-torque sum, including during ablation');
    assert.deepEqual(a.intendedControls, row.getup.motor_controls, 'Readback must retain original intended controller output');
    assert.equal(intended[C.actuatorIndex], Math.fround(a.intendedControls[C.actuatorIndex] * C.gear), 'right_ankle_y intended gear20 torque mismatch');
    assert.equal(a.commandInputNativeSubstep, n - 1, 'Motor readback must describe command consumed by completed native n');
    assert.deepEqual(a.overriddenActuatorIndices, active ? [C.actuatorIndex] : [], 'Native override actuator receipt differs at' + n);
    for (let i = 0; i < 21; i++) {
      assert.equal(a.appliedControls[i], active && i === C.actuatorIndex ? 0 : a.intendedControls[i], 'Actual applied control mismatch at native' + n + ', actuator' + i);
      assert.equal(applied[i], active && i === C.actuatorIndex ? 0 : intended[i], 'Actual applied motor torque mismatch at native' + n + ', actuator' + i);
      extend(result.extrema.intendedTorqueNm[i], intended[i], n); extend(result.extrema.appliedTorqueNm[i], applied[i], n);
      extend(result.extrema.jointAnglesRad[i], currentQ[i], n);
      extend(result.extrema.jointVelocitiesRadps[i], currentDq[i], n);
      result.limits.normalizedMotorControlAbsMaximum = Math.max(result.limits.normalizedMotorControlAbsMaximum, Math.abs(row.getup.motor_controls[i]));
    }
    const contact = d.contactMotion;
    assert.equal(contact?.physicsIntegrations, 0); assert.equal(contact.velocitySource, 'current-generalized-motion');
    assert.ok(Array.isArray(contact.feet) && contact.feet.length === 2);
    assert.deepEqual(contact.feet.map(f => f.body).sort(), ['left_foot', 'right_foot']);
    for (const foot of contact.feet) {
      assert.ok(Number.isFinite(foot.loadN) && finiteVector(foot.origin) && finiteVector(foot.comVelocityMps) && finiteVector(foot.angularVelocityRadps)
        && Array.isArray(foot.contacts) && (foot.impulseWeightedTangentSpeedMps === null || Number.isFinite(foot.impulseWeightedTangentSpeedMps) && foot.impulseWeightedTangentSpeedMps >= 0));
      assert.ok(foot.contacts.every(c => Number.isFinite(c.normalImpulseNs) && c.normalImpulseNs >= 0 && Number.isFinite(c.tangentSpeedMps) && c.tangentSpeedMps >= 0
        && finiteVector(c.point) && finiteVector(c.normal) && finiteVector(c.velocityMps) && finiteVector(c.tangentVelocityMps)));
      if (foot.loadN >= C.minimumLoadedN) {
        const slip = result.materialSlip[foot.body]; slip.loadedRows++;
        if (foot.impulseWeightedTangentSpeedMps === null) slip.unavailableLoadedRows++;
        else { slip.observedLoadedRows++; slip.sampledImpulseWeightedTangentTravelM += foot.impulseWeightedTangentSpeedMps / C.nativeHz;
          slip.maximumImpulseWeightedTangentSpeedMps = Math.max(slip.maximumImpulseWeightedTangentSpeedMps, foot.impulseWeightedTangentSpeedMps); }
      }
    }
    const left = contact.feet.find(f => f.body === 'left_foot'), right = contact.feet.find(f => f.body === 'right_foot');
    assert.equal(left.loadN, d.loadsN[d.footIds[0]], 'footIds[0] must be native left_foot');
    assert.equal(right.loadN, d.loadsN[d.footIds[1]], 'footIds[1] must be native right_foot');
    const unloaded = left.loadN < C.minimumLoadedN;
    const clearance = field(row, C.separationField);
    assert.ok(finiteArray(clearance, 2), 'Independent geometric left/right foot–floor clearance unavailable at' + n);
    const separation = clearance[0];
    for (const [key, extrema] of Object.entries(result.extrema.integrityLimits)) {
      assert.ok(Number.isFinite(row.integrityLimits?.[key]) && row.integrityLimits[key] >= 0, 'Current native integrity-limit observable missing: ' + key);
      extend(extrema, row.integrityLimits[key], n);
    }
    const allSeparation = result.soleFloorSeparation; extend(allSeparation.extremaM, separation, n);
    if (separation > C.separationThresholdM) { allSeparation.positiveRows++; allSeparation.firstPositiveNative ??= n; } else allSeparation.nonpositiveRows++;
    if (n >= C.firstAffectedNative) postInterventionStreak = unloaded ? postInterventionStreak + 1 : 0;
    if (postInterventionStreak >= C.unloadingConsecutiveNative) result.firstUnloaded20NativeAfterIntervention ??= n;
    if (unloaded) {
      result.firstLeftLoadBelow3N ??= n; result.firstCompletedLeftLoadBelow3N ??= n;
      if (n >= C.firstAffectedNative) result.firstLeftLoadBelow3NAfterIntervention ??= n;
      streak++; interval ??= { startNative: n, endNative: n, rows: 0 }; interval.endNative = n; interval.rows++;
      result.maximumUnloadedStreak = Math.max(result.maximumUnloadedStreak, streak);
      if (streak >= C.unloadingConsecutiveNative) {
        result.firstUnloaded20Native ??= n;
      }
      const s = result.unloadedSeparation; s.rows++;
      s.minimumM = s.minimumM === null ? separation : Math.min(s.minimumM, separation);
      s.maximumM = s.maximumM === null ? separation : Math.max(s.maximumM, separation);
      if (separation > C.separationThresholdM) { s.separatedRows++; s.firstSeparatedNative ??= n; } else s.unseparatedRows++;
    } else { if (interval) result.unloadingIntervals.push(interval); interval = null; streak = 0; }
    for (const name of ['full', n < C.firstAffectedNative ? 'preIntervention' : n <= C.lastAblatedNative ? 'ablated' : 'restored']) {
      const w = result.windows[name]; w.rows++; w.leftUnloadedRows += Number(unloaded);
      w.bothFeetLoadedRows += Number(left.loadN >= C.minimumLoadedN && right.loadN >= C.minimumLoadedN);
      for (const [out, predicate] of Object.entries({ comSupportedRows: 'comInSupport', captureSupportedRows: 'captureInSupport', onlyFeetRows: 'onlyFeet',
        linearMotionPassedRows: 'linearMotion', angularMotionPassedRows: 'angularMotion' })) w[out] += Number(d.predicates[predicate] === true);
      w.motionCounts[row.motion] = (w.motionCounts[row.motion] ?? 0) + 1;
      const snapshot = { native: n, leftLoadN: left.loadN, rightLoadN: right.loadN, leftSoleSeparationM: separation, torsoUp: d.torsoUp,
        com: d.com, capture: d.capture, support: d.solverSupportPoints, predicates: d.predicates, motion: row.motion };
      w.firstRow ??= snapshot; w.lastRow = snapshot;
      for (const [key, value] of Object.entries({ leftLoadN: left.loadN, rightLoadN: right.loadN, leftClearanceM: clearance[0], rightClearanceM: clearance[1],
        torsoUp: d.torsoUp, headM: d.headM, maximumBodyLinearMps: Math.max(...d.motion.map(m => m.linearMps)),
        maximumBodyAngularRadps: Math.max(...d.motion.map(m => m.angularRadps)) })) extend(w.extrema[key], value, n);
      for (let i = 0; i < 21; i++) {
        extend(w.extrema.intendedTorqueNm[i], intended[i], n); extend(w.extrema.appliedTorqueNm[i], applied[i], n);
        extend(w.extrema.jointAnglesRad[i], currentQ[i], n); extend(w.extrema.jointVelocitiesRadps[i], currentDq[i], n);
      }
      w.cumulativeIntegrityLimitReadbacks.first ??= { native: n, values: row.integrityLimits };
      w.cumulativeIntegrityLimitReadbacks.last = { native: n, values: row.integrityLimits };
      for (const foot of contact.feet) if (foot.loadN >= C.minimumLoadedN) {
        const slip = w.materialSlip[foot.body]; slip.loadedRows++;
        if (foot.impulseWeightedTangentSpeedMps === null) slip.unavailableLoadedRows++;
        else { slip.observedLoadedRows++; slip.sampledImpulseWeightedTangentTravelM += foot.impulseWeightedTangentSpeedMps / C.nativeHz;
          slip.maximumImpulseWeightedTangentSpeedMps = Math.max(slip.maximumImpulseWeightedTangentSpeedMps, foot.impulseWeightedTangentSpeedMps); }
      }
    }
    const nonfootMaximum = Math.max(...d.loadsN.filter((_, i) => !d.footIds.includes(i)));
    for (const [name, value] of Object.entries({ leftLoadN: left.loadN, rightLoadN: right.loadN, nonfootLoadN: nonfootMaximum,
      torsoUp: d.torsoUp, headM: d.headM, totalFootLoadFraction: (left.loadN + right.loadN) / row.weightN,
      maximumBodyLinearMps: Math.max(...d.motion.map(m => m.linearMps)), maximumBodyAngularRadps: Math.max(...d.motion.map(m => m.angularRadps)),
      requestedMotorEffortNm: row.requestedMotorEffortNm })) extend(result.extrema[name], value, n);
  }
  if (interval) result.unloadingIntervals.push(interval);
  result.currentUnloadedStreak = streak; result.currentUnloadedStreakSinceInterventionBoundary = postInterventionStreak;
  result.currentAtLeast20Unloaded = streak >= C.unloadingConsecutiveNative;
  result.overrideCount = expectedCount;
  assert.deepEqual(result.overrideNatives, arm === 'baseline' ? [] : Array.from({ length: 20 }, (_, i) => C.firstAffectedNative + i));
  result.finalNative = rows.at(-1).nativeSubsteps; result.valid = true;
  return result;
}

export function comparePair(aRows, bRows, aController, bController, C = CONTRACT) {
  assert.equal(aRows.length, C.stopNative + 1); assert.equal(bRows.length, C.stopNative + 1);
  // No reduced-coordinate comparison: every actual native0/physical/policy/
  // control/readback field is compared. Build identity lives outside these rows.
  const aPrefix = aRows.slice(0, C.firstAffectedNative), bPrefix = bRows.slice(0, C.firstAffectedNative);
  const firstDifference = aRows.findIndex((row, i) => !equal(row, bRows[i]));
  const teacherKeys = ['teacher_frames', 'teacher_initial', 'teacher_native_dt_s', 'teacher_failure', 'reference'];
  const teacherReference = Object.fromEntries(teacherKeys.map(key => {
    assert.ok(Object.hasOwn(aController, key) && Object.hasOwn(bController, key), 'Missing complete archive: ' + key);
    return [key, { equal: equal(aController[key], bController[key]), baselineSha256: hash(JSON.stringify(canonical(aController[key]))),
      candidateSha256: hash(JSON.stringify(canonical(bController[key]))) }];
  }));
  const result = { fullObservablePrefixEqual: equal(aPrefix, bPrefix), prefixNativeRange: [0, C.firstAffectedNative - 1], prefixRows: C.firstAffectedNative,
    baselinePrefixSha256: hash(JSON.stringify(canonical(aPrefix))), candidatePrefixSha256: hash(JSON.stringify(canonical(bPrefix))),
    firstObservableDifferenceNative: firstDifference < 0 ? null : aRows[firstDifference].nativeSubsteps,
    firstDifferenceAllowedAtNative: C.firstAffectedNative,
    prefixComparisonExclusions: [],
    teacherReference, teacherReferenceEqual: Object.values(teacherReference).every(v => v.equal),
    controllerArchiveNative: C.stopNative, studentDecisionEqualityAfterInterventionRequired: false,
    intendedTrajectoryEqualityAfterInterventionRequired: false, boundedCausalDiagnosticOnly: true,
    policyAdmissionClaim: false, recoveryClaim: false, standingClaim: false, performanceClaim: false };
  assert.equal(result.fullObservablePrefixEqual, true, 'Actual full observable prefix0..2180 differs');
  assert.ok(firstDifference < 0 || aRows[firstDifference].nativeSubsteps >= C.firstAffectedNative);
  assert.equal(result.teacherReferenceEqual, true, 'Unchanged complete teacher/reference archives required');
  for (const key of ['started_substep', 'impact_substep', 'teacher_native_dt_s']) assert.equal(aController[key], bController[key]);
  for (const key of ['forecast_substeps', 'teacher_decisions', 'reference_frames']) assert.equal(aController.status[key], bController.status[key]);
  assert.equal(aController.reference.length, aController.status.reference_frames);
  assert.equal(bController.reference.length, bController.status.reference_frames);
  return result;
}

export function expectedBuildIdentity(arm, C = CONTRACT) {
  const candidate = arm === 'candidate';
  return { experiment: C.experiment, variant: candidate ? C.candidateVariant : C.baselineVariant,
    checkpointResumeSupported: !candidate, checkpointExportSupported: !candidate,
    candidateFeature: 'getup-right-ankle-braking-ablation-experiment', candidateFeatureEnabled: candidate,
    observerFeature: 'getup-terminal-tail-observer-experiment',
    intervention: { qposIndex: 7, actuatorIndex: 8, joint: 'right_ankle_y', firstPostNative: 2181, lastPostNative: 2200,
      normalAgainPostNative: 2201, teacherExcludedBy: 'fork clears getup_policy_enabled' },
    footFloorClearance: { order: ['left_foot', 'right_foot'], units: 'metres', source: C.separationSource,
      separationCorroboration: 'strictly positive; independent of the existing 3 N unload criterion' },
    nativeTrace: C.nativeTraceFormat, maxTicksPerCall: 1,
    poseWordLayout: 'per body: positionXYZ,quaternionXYZW,cachedLinearVelocityXYZ,cachedAngularVelocityXYZ',
    currentMotionSource: 'fresh physicalObservationWords and current.motion; pose velocity words are cache receipts' };
}

export async function main(argv = process.argv) {
  assert.equal(argv.length, 5, 'Usage: recorder BASELINE_ARTIFACT CANDIDATE_ARTIFACT FRESH_OUTPUT');
  assert.equal(CONTRACT.confirmed, true, 'SOURCE-ONLY DRAFT: exact native observer/adapter contract and execution have not been approved');
  for (const key of ['fixtureTraceField', 'fixtureIdentityField', 'fixtureTraceErrorField', 'fixtureNative0Field', 'fixtureReferenceField', 'experiment', 'nativeTraceFormat',
    'baselineVariant', 'candidateVariant', 'candidateBuildFeature', 'ablationRowField', 'intendedTorqueField', 'appliedTorqueField',
    'overrideAppliedField', 'separationField', 'separationSource']) assert.equal(typeof CONTRACT[key], 'string', 'Unconfirmed contract field: ' + key);
  assert.ok(Number.isFinite(CONTRACT.separationThresholdM) && Array.isArray(CONTRACT.commonBuildFeatures));
  const artifacts = { baseline: resolve(argv[2]), candidate: resolve(argv[3]) }, output = resolve(argv[4]);
  assert.notEqual(artifacts.baseline, artifacts.candidate, 'Separate immutable actual-WASM builds required');
  for (const artifact of Object.values(artifacts)) assert.ok(output !== artifact && !output.startsWith(artifact + sep) && !artifact.startsWith(output + sep), 'Artifacts and fresh evidence must not overlap');
  assert.ok(!existsSync(output) || readdirSync(output).length === 0, 'Fresh empty output required');
  mkdirSync(output, { recursive: true });
  const profileBytes = readFileSync(profilePath), approvedProfile = JSON.parse(profileBytes);
  assert.equal(hash(profileBytes), profileSha256);
  assert.equal(approvedProfile.dt_s, 1 / 800); assert.equal(approvedProfile.tick_substeps, 20);
  assert.equal(approvedProfile.solver_iterations, 3); assert.equal(approvedProfile.additional_body_iterations, 1);
  assert.equal(approvedProfile.getup_policy_enabled, true);
  assert.equal(approvedProfile.getup_solver_budget, 'consistent-recovery-phase-v1');
  assert.equal(approvedProfile.root_rotation_normalization, 'unit-free-root-v1');
  assert.equal(approvedProfile.getup_repeated_recovery, 'completed-cycle-confirmed-fall-v1');
  const { parseGetUpCheckpoint, proveActivationPurity } = await import(pathToFileURL(resolve('docs/evidence/probes/getup-checkpoint-envelope-2026-10-19.mjs')).href);
  const { chromium, browserLaunchOptions } = await import(pathToFileURL(resolve('scripts/rust-browser-test.mjs')).href);
  const report = { at: new Date().toISOString(), passed: false, recordingComplete: false,
    mode: 'fresh-back-right-ankle-applied-torque-ablation', contract: CONTRACT, profile: approvedProfile, profilePath, profileSha256,
    scope: 'Bounded causal diagnostic only. Fresh BACK arms, original80-native fixture polling, carry once at native0, renderer/driver/transport/automatic one-tick schedule. '
      + 'Identical complete baseline profile/policy/actors/teacher/reference. Candidate only zeros applied actuator8/right_ankle_y torque on completed natives2181..2200. '
      + 'Exact readback on every native; normal candidate application resumes2201. Both arms stop2380/2.975s regardless of sign. No continuation, sweep, reset, retry or epoch renewal.',
    qualificationClaim: false, policyAdmissionClaim: false, recoveryClaim: false, standingClaim: false, performanceClaim: false,
    noEncoder: true, noRetries: true, arms: {}, comparison: null };

  async function observeArm(arm) {
    const artifact = artifacts[arm], armOutput = join(output, arm); mkdirSync(armOutput);
    const r = report.arms[arm] = { arm, artifact, at: new Date().toISOString(), observationValid: false,
      outcome: 'NOT_STARTED', errors: [], progress: [], controllerCaptures: {}, nativeTrace: { path: 'native-trace.ndjson', rows: 0, native0Included: false },
      expectedMutation: arm === 'baseline' ? 'none' : 'applied actuator8 torque zero only on completed natives2181..2200',
      noEncoder: true, checkpointResumeSupported: arm === 'baseline' };
    let server, browser, context, page;
    const tracePath = join(armOutput, r.nativeTrace.path); writeFileSync(tracePath, '');
    const evalPage = (...args) => withWallWatchdog(page.evaluate(...args), CONTRACT.exportWallWatchdogMs, 'browser evidence operation');
    const drain = rows => {
      if (!rows?.length) return;
      appendFileSync(tracePath, rows.map(row => JSON.stringify(row)).join('\n') + '\n');
      r.nativeTrace.rows += rows.length;
      if (rows.some(row => row.nativeSubsteps === 0)) r.nativeTrace.native0Included = true;
    };
    async function captureStopController() {
      // A single label and exact paused STOP state avoid the former queued
      // triage-label bug. No intermediate controller export is credited.
      await page.waitForFunction(() => {
        const d = window.__lhRust?.diagnostics, e = window.__fixtureEvidence;
        return d && (!d.outstanding || d.workerFault || d.disposed || e?.error);
      }, null, { timeout: CONTRACT.exportWallWatchdogMs });
      const value = await evalPage(async ({ C }) => {
        const api = window.__lhRust, e = window.__fixtureEvidence;
        const bad = () => Boolean(e.error || api.diagnostics.workerFault || api.diagnostics.disposed);
        if (bad()) return { unavailable: true, reason: 'Original worker/native/observer fault; preserve it without exports or retries.' };
        if (e.lastNative !== C.stopNative || Math.round(api.diagnostics.time * C.nativeHz) !== C.stopNative
          || e.stopReason !== 'bounded-native2380-stop') throw new Error('Controller archive requires exact native2380 STOP state');
        const before = { tick: api.diagnostics.tick, time: api.diagnostics.time, native: e.lastNative };
        let timer, interval;
        try {
          const failure = new Promise((resolveFault, reject) => {
            interval = setInterval(() => { if (bad()) resolveFault({ unavailable: true, reason: 'Original worker/observer fault during STOP export' }); }, 25);
            timer = setTimeout(() => reject(new Error('Read-only STOP controller export wall watchdog exceeded')), C.exportWallWatchdogMs - 1000);
          });
          const controller = await Promise.race([api.getupReport(), failure]);
          if (controller.unavailable) return controller;
          if (bad()) return { unavailable: true, reason: 'Original worker/observer fault after STOP export' };
          if (before.tick !== api.diagnostics.tick || before.time !== api.diagnostics.time || before.native !== e.lastNative) throw new Error('Read-only STOP controller export advanced physical state');
          return { controller, native: before.native, physicalSeconds: before.time, physicsIntegrations: 0 };
        } finally { clearTimeout(timer); clearInterval(interval); }
      }, { C: CONTRACT });
      if (value.unavailable) { r.controllerCaptures.stop = value; return null; }
      const path = join(armOutput, 'stop-controller.json'); json(path, value.controller);
      r.controllerCaptures.stop = { path: relative(armOutput, path), sha256: hash(readFileSync(path)), native: value.native,
        physicalSeconds: value.physicalSeconds, physicsIntegrations: 0, labelMatchesPhysicalState: true };
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
      browser = await chromium.launch(browserLaunchOptions(true)); r.browserVersion = browser.version();
      context = await browser.newContext({ viewport: { width: 1280, height: 900 } });
      await context.addInitScript(({ arm, approvedProfile, C, expectedIdentity }) => {
        window.__lhGetUpTrial = true;
        let api;
        Object.defineProperty(window, '__lhRust', { configurable: true, get: () => api, set(value) { api = value; api.setManual(true); } });
        const canonical = v => Array.isArray(v) ? v.map(canonical) : v && typeof v === 'object'
          ? Object.fromEntries(Object.keys(v).sort().map(k => [k, canonical(v[k])])) : v;
        const equal = (a, b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
        const e = window.__fixtureEvidence = { arm, workerInstances: 0, workerIdentities: [], initializations: [], commands: [], physicalInputs: [], resetRequests: [],
          trigger: null, activation: null, profile: null, buildIdentity: null, native0State: null, referenceEvidence: null, nativeTrace: [], traceRows: 0,
          lastNative: null, lastPublicationNative: null, originalTriggerNative: null, publications: [], traceGaps: [],
          stopReason: null, error: null, recorderError: null, rawWorkerFaults: [], rawObserverFault: null,
          startedAtMs: null, endedAtMs: null, initialFrames: null, endedFrames: null,
          stop(reason, error = null) {
            if (error !== null) this.error ??= String(error);
            if (this.endedAtMs === null) { this.stopReason = reason; this.endedAtMs = performance.now(); this.endedFrames = api?.diagnostics.graphics?.frames ?? null; }
            api?.setManual(true);
          } };
        const Native = window.Worker;
        window.Worker = class extends Native {
          constructor(...args) {
            super(...args); e.workerInstances++;
            this.addEventListener('message', event => {
              const m = event.data;
              if (m.type === 'ready') e.workerIdentities.push(structuredClone(m.workerIdentity));
              if (m.type === 'error') { e.rawWorkerFaults.push(structuredClone(m)); e.stop('worker-error', m.message); return; }
              if (!m.reply) return;
              const f = m.fixtureTest, snapshot = m.reply.snapshot, n = Math.round(snapshot.simulation_time_s * C.nativeHz);
              // Preserve raw rows BEFORE validation, including every row in a
              // faulty packet. The gap/fault is evidence, never interpolated.
              const rows = f?.[C.fixtureTraceField];
              const state = f?.[C.fixtureNative0Field];
              if (e.lastNative === null && state) { e.nativeTrace.push({ receiptKind: 'native0-read-only-state', ...structuredClone(state), snapshot: structuredClone(snapshot) }); e.traceRows++; }
              if (Array.isArray(rows)) { e.nativeTrace.push(...structuredClone(rows)); e.traceRows += rows.length; }
              try {
                if (!f?.isolatedTestOnly || f.kind !== 'back' || f.preparationPollingNative !== C.preparationPollingNative || !equal(f.profile, approvedProfile)) throw new Error('Original BACK80-native fixture/full baseline profile missing');
                if (snapshot.simulation_time_s !== n / C.nativeHz || snapshot.stamp.generation !== 1 || !snapshot.diagnostics.getup?.learned_policy_loaded) throw new Error('Native physical clock/generation/learned actor changed');
                e.profile = structuredClone(f.profile);
                const identity = f[C.fixtureIdentityField];
                if (!equal(identity, expectedIdentity)) throw new Error('Wrong exact actual-WASM arm identity/provenance');
                if (e.buildIdentity && !equal(identity, e.buildIdentity)) throw new Error('Actual-WASM build identity changed');
                e.buildIdentity = structuredClone(identity);
                if (f[C.fixtureReferenceField]) {
                  if (e.referenceEvidence !== null) throw new Error('Initial teacher/reference archive repeated');
                  e.referenceEvidence = structuredClone(f[C.fixtureReferenceField]);
                }
                if (f[C.fixtureTraceErrorField] !== null) throw new Error('Native observer error: ' + String(f[C.fixtureTraceErrorField]));
                if (f.triggerEvidence) { if (e.trigger !== null) throw new Error('Original physical trigger repeated'); e.trigger = structuredClone(f.triggerEvidence);
                  if (!Number.isSafeInteger(e.trigger.native) || e.trigger.native <= 0 || e.trigger.native % C.preparationPollingNative !== 0) throw new Error('Original80-native trigger invalid'); }
                if (f.triggerNative !== null) { e.originalTriggerNative ??= f.triggerNative;
                  if (e.originalTriggerNative !== f.triggerNative || e.trigger?.native !== f.triggerNative) throw new Error('Original trigger/impact epoch changed');
                } else if (e.originalTriggerNative !== null) throw new Error('Original trigger/impact epoch erased');
                if (f.carryActivationEvidence) { if (e.activation !== null) throw new Error('Native0 carry activation repeated'); e.activation = structuredClone(f.carryActivationEvidence); }
                if (!Array.isArray(rows) || rows.length > C.publicationSubsteps) throw new Error('Bounded native packet missing');
                const duplicate = e.lastPublicationNative === n;
                if (e.lastPublicationNative === null ? n !== 0 || m.reply.completed_ticks !== 0
                  : duplicate ? m.reply.completed_ticks !== 0 : n - e.lastPublicationNative !== C.publicationSubsteps || m.reply.completed_ticks !== 1) throw new Error('Original automatic one-tick scheduler changed');
                if (e.lastNative === null) {
                  if (n !== 0 || !state || state.nativeSubsteps !== 0 || state.physicsIntegrations !== 0
                    || state.physicalObservationError !== null || state.integratorReadError !== null || state.poseWords?.length !== 325 || state.physicalObservationWords?.length !== 67
                    || state.current?.nativeSubsteps !== 0 || state.current.physicsIntegrations !== 0) throw new Error('Explicit read-only native0 state missing');
                  e.native0State = structuredClone(state); e.lastNative = 0;
                }
                if (rows.length !== n - e.lastNative) { e.traceGaps.push({ expectedFromNative: e.lastNative + 1, publicationNative: n, receivedNatives: rows.map(r => r.nativeSubsteps) }); throw new Error('Native packet row-count gap'); }
                for (const row of rows) {
                  if (row.nativeSubsteps !== e.lastNative + 1 || row.commandInputNativeSubstep !== row.nativeSubsteps - 1 || row.physicalSeconds !== row.nativeSubsteps / C.nativeHz) {
                    e.traceGaps.push({ expectedNative: e.lastNative + 1, row: structuredClone(row) }); throw new Error('Native gap/duplicate/order/clock mismatch'); }
                  e.lastNative = row.nativeSubsteps;
                  if (row.nativeSubsteps > C.stopNative) throw new Error('Original automatic scheduler overshot exact native2380 stop');
                  if (row.nativeSubsteps === C.stopNative) e.stop('bounded-native2380-stop');
                }
                if (e.lastNative !== n) throw new Error('Native observer/publication clocks disagree');
                e.publications.push({ native: n, completedTicks: m.reply.completed_ticks, counters: structuredClone(snapshot.counters), motion: snapshot.motion });
                e.lastPublicationNative = n;
                if (m.reply.failure) e.stop('physical-or-resource-halt');
              } catch (error) {
                e.rawObserverFault = { error: String(error), type: m?.type ?? null, requestId: m?.requestId ?? null,
                  fixture: structuredClone(m?.fixtureTest ?? null), reply: structuredClone(m?.reply ?? null) };
                e.stop('instrumentation-error', String(error));
              }
            });
          }
          postMessage(message, ...args) {
            if (message.type === 'initialize') { message = { ...message, testFixedGetUpFixture: 'back', testFixedGetUpProfile: approvedProfile }; e.initializations.push(structuredClone(message)); }
            if (message.commands?.length) { e.commands.push(structuredClone(message.commands)); e.stop('unexpected-physical-command', 'Physical commands forbidden'); throw new Error('Physical commands forbidden'); }
            if (message.type === 'reset') { e.resetRequests.push(structuredClone(message)); e.stop('unexpected-reset', 'Reset forbidden'); throw new Error('Reset forbidden'); }
            return super.postMessage(message, ...args);
          }
        };
        for (const type of ['pointerdown', 'keydown', 'keyup']) document.addEventListener(type, event => {
          if (e.startedAtMs !== null && e.endedAtMs === null && (type !== 'pointerdown' || event.target?.closest?.('#viewport'))) {
            e.physicalInputs.push({ type, code: event.code ?? null, isTrusted: event.isTrusted }); e.stop('unexpected-physical-input', 'Physical input forbidden'); }
        }, true);
      }, { arm, approvedProfile, C: CONTRACT, expectedIdentity: expectedBuildIdentity(arm) });
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
        && r.activationProof.checkpointExportRefusal.every(value => typeof value === 'string' && value.includes(CONTRACT.candidateCheckpointRefusal)));
      assert.equal(typeof r.activationProof.checkpointRestoreRefusal, 'string');
      assert.ok(r.activationProof.checkpointRestoreRefusal.includes(CONTRACT.candidateCheckpointRefusal));
      r.activationProof.binaryWorldPurityClaim = false;
    }
    r.checkpointDisposition = { resumeSupported: arm === 'baseline', candidateCheckpointExportRestoreVerifyAttemptedByRecorder: false,
      candidateNativeRefusals: arm === 'candidate' ? { export: r.activationProof.checkpointExportRefusal, restore: r.activationProof.checkpointRestoreRefusal } : null,
      limitation: arm === 'candidate' ? 'Checkpoint export and restore deliberately refused; native0 physical/generalized receipts only, no binary-world purity claim.' : null };
    json(join(armOutput, 'activation-proof.json'), r.activationProof);
      await page.screenshot({ path: join(armOutput, 'native0-initial-back.png'), fullPage: true });
      await evalPage(() => {
        const e = window.__fixtureEvidence; if (e.error || e.endedAtMs !== null) throw new Error(e.error || e.stopReason);
        window.__lhRust.drainTelemetry(); e.startedAtMs = performance.now(); e.initialFrames = window.__lhRust.diagnostics.graphics.frames;
        window.__lhRust.setManual(false);
      });
      let lastNative = 0, lastProgressAt = Date.now(), nextProgressNative = 0;
      const startedAt = Date.now();
      while (true) {
        await page.waitForTimeout(250);
        const update = await evalPage(() => {
          const e = window.__fixtureEvidence, d = window.__lhRust.diagnostics;
          if (d.workerFault || d.disposed || d.halted || d.failure || d.error) e.stop(d.workerFault || d.disposed ? 'original-worker-terminated' : 'physical-resource-renderer-halt', d.error || null);
          return { native: e.lastNative, publicationNative: Math.round(d.time * 800), physicalSeconds: d.time,
            workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed), error: e.error || d.error,
            stopReason: e.stopReason, ended: e.endedAtMs !== null, nativeTrace: e.nativeTrace.splice(0) };
        });
        drain(update.nativeTrace); delete update.nativeTrace;
        if (update.native !== lastNative) { lastProgressAt = Date.now(); lastNative = update.native; }
        if (update.native >= nextProgressNative || update.ended || update.error) {
          r.progress.push(update); console.log(JSON.stringify({ finding: 'right-ankle-bounded-ablation', arm, ...update })); nextProgressNative = update.native + 800;
        }
        if (r.errors.length) await evalPage(message => window.__fixtureEvidence.stop('page-or-resource-error', message), r.errors.join('\n'));
        if (update.ended || update.error || update.workerFault || update.disposed || r.errors.length) break;
        if (Date.now() - lastProgressAt >= CONTRACT.noProgressWallWatchdogMs || Date.now() - startedAt >= CONTRACT.totalWallWatchdogMs) {
          await evalPage(() => window.__fixtureEvidence.stop('native-progress-wall-watchdog', 'Bounded wall watchdog; no physical continuation or retry')); break;
        }
      }
      await evalPage(() => window.__lhRust.setManual(true));
      const availability = await evalPage(() => {
        const d = window.__lhRust.diagnostics, e = window.__fixtureEvidence;
        return { workerFault: Boolean(d.workerFault), disposed: Boolean(d.disposed), observerError: e.error, native: e.lastNative, stopReason: e.stopReason };
      });
      if (!availability.workerFault && !availability.disposed && !availability.observerError && availability.native === CONTRACT.stopNative && availability.stopReason === 'bounded-native2380-stop') await captureStopController();
      else r.stopExportUnavailable = { ...availability, reason: 'Preserve original fault/incomplete run; never export a mislabeled STOP state or resurrect a worker.' };
      r.final = await evalPage(() => window.__lhRust.diagnostics);
      r.evidence = await evalPage(() => { const { stop, nativeTrace, ...rest } = window.__fixtureEvidence; return { ...rest, remainingTrace: nativeTrace }; });
      drain(r.evidence.remainingTrace); delete r.evidence.remainingTrace;
      const e = r.evidence;
      assert.equal(e.workerInstances, 1); assert.equal(e.workerIdentities.length, 1); assert.equal(e.initializations.length, 1);
      assert.deepEqual(e.commands, []); assert.deepEqual(e.physicalInputs, []); assert.deepEqual(e.resetRequests, []);
      assert.deepEqual(e.profile, approvedProfile); assert.equal(e.initializations[0].profile, undefined);
      assert.equal(e.trigger?.physicalPoseVelocityBitsUnchanged, true); assert.deepEqual(e.trigger.beforePoseWords, e.trigger.afterPoseWords);
      assert.equal(r.nativeTrace.native0Included, true); assert.equal(r.nativeTrace.rows, CONTRACT.stopNative + 1);
      assert.equal(e.lastNative, CONTRACT.stopNative); assert.equal(e.lastPublicationNative, CONTRACT.stopNative);
      assert.equal(e.stopReason, 'bounded-native2380-stop'); assert.equal(r.final.time, CONTRACT.stopPhysicalSeconds);
      assert.equal(e.error, null); assert.deepEqual(e.traceGaps, []); assert.deepEqual(e.rawWorkerFaults, []); assert.equal(e.rawObserverFault, null); assert.deepEqual(r.errors, []);
      assert.equal(Boolean(r.final.workerFault), false); assert.equal(Boolean(r.final.disposed), false);
      assert.equal(r.final.error, ''); assert.equal(r.final.failure, null);
      assert.equal(r.controllerCaptures.stop?.native, CONTRACT.stopNative);
      const stopController = JSON.parse(readFileSync(join(armOutput, r.controllerCaptures.stop.path)));
      assert.ok(e.referenceEvidence, 'Initial complete teacher/reference archive missing');
      for (const key of ['teacher_frames', 'teacher_initial', 'teacher_native_dt_s', 'teacher_failure', 'reference'])
        assert.ok(Object.hasOwn(e.referenceEvidence, key) && equal(e.referenceEvidence[key], stopController[key]), 'Teacher/reference archive changed between first availability and exact STOP: ' + key);
      const initialArchivePath = join(armOutput, 'initial-teacher-reference-controller.json'); json(initialArchivePath, e.referenceEvidence);
      r.initialTeacherReferenceArchive = { path: relative(armOutput, initialArchivePath), sha256: hash(readFileSync(initialArchivePath)), unchangedAtStop: true };
      const rows = readFileSync(tracePath, 'utf8').trim().split('\n').map(line => JSON.parse(line));
      r.analysis = analyzeTrace(rows, arm);
      r.observationValid = true; r.outcome = 'completed-bounded-causal-diagnostic';
      await page.screenshot({ path: join(armOutput, 'native2380-bounded-stop.png'), fullPage: true });
    } catch (error) {
      r.error = error.stack ?? String(error);
      if (page && !page.isClosed()) {
        try {
          await withWallWatchdog(evalPage(message => { const e = window.__fixtureEvidence; if (e) { e.recorderError = message; e.stop('recorder-error'); } }, String(error)), 5000, 'failure stop');
          r.failureState = await withWallWatchdog(evalPage(() => window.__lhRust?.diagnostics), 5000, 'failure state');
          r.evidence = await withWallWatchdog(evalPage(() => { const { stop, nativeTrace, ...rest } = window.__fixtureEvidence ?? {}; return { ...rest, remainingTrace: nativeTrace ?? [] }; }), 5000, 'raw failure evidence');
          drain(r.evidence.remainingTrace); delete r.evidence.remainingTrace;
          await withWallWatchdog(page.screenshot({ path: join(armOutput, 'failure.png'), fullPage: true }), 5000, 'failure screenshot');
        } catch (captureError) { r.rawFailureCaptureError = String(captureError); }
      }
      r.outcome = 'physical-observer-resource-or-contract-failure';
    } finally {
      r.finished = new Date().toISOString(); r.nativeTrace.sha256 = hash(readFileSync(tracePath));
      // Save original errors/raw trace before cleanup. No failed transport awaits
      // an additional controller/checkpoint request or a replacement world.
      json(join(armOutput, 'result.json'), r); json(join(output, 'result.json'), report);
      try { await withWallWatchdog(context?.close() ?? Promise.resolve(), 5000, 'context close'); } catch (error) { r.cleanupError = String(error); }
      try { await withWallWatchdog(browser?.close() ?? Promise.resolve(), 5000, 'browser close'); } catch (error) { r.cleanupError ??= String(error); }
      server?.kill(); json(join(armOutput, 'result.json'), r);
    }
    return r;
  }
  try {
    await observeArm('baseline'); await observeArm('candidate');
    const a = report.arms.baseline, b = report.arms.candidate;
    if (a.observationValid && b.observationValid) {
      const trace = r => readFileSync(join(output, r.arm, r.nativeTrace.path), 'utf8').trim().split('\n').map(line => JSON.parse(line));
      const controller = r => JSON.parse(readFileSync(join(output, r.arm, r.controllerCaptures.stop.path)));
      report.comparison = comparePair(trace(a), trace(b), controller(a), controller(b));
      report.comparison.originalDriverEqual = equal(a.originalDriver, b.originalDriver);
      report.comparison.frozenSourceFilesEqual = equal(a.frozenSourceFiles, b.frozenSourceFiles);
      report.comparison.completeProfileEqual = equal(a.evidence.profile, b.evidence.profile);
      assert.equal(report.comparison.originalDriverEqual, true); assert.equal(report.comparison.frozenSourceFilesEqual, true); assert.equal(report.comparison.completeProfileEqual, true);
      report.recordingComplete = true;
      // "passed" means recorder/causal isolation checks only. Outcome signs do
      // not grant policy admission, recovery, standing or performance credit.
      report.passed = true;
    }
  } catch (error) { report.error = error.stack ?? String(error); }
  finally {
    report.finished = new Date().toISOString(); json(join(output, 'result.json'), report);
    console.log(JSON.stringify({ passed: report.passed, recordingComplete: report.recordingComplete, boundedCausalDiagnosticOnly: true,
      policyAdmissionClaim: false, recoveryClaim: false, standingClaim: false, performanceClaim: false,
      baseline: report.arms.baseline?.outcome, candidate: report.arms.candidate?.outcome, comparison: report.comparison, output }, null, 2));
  }
  return report;
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  main().then(report => process.exit(report.passed ? 0 : 1)).catch(error => { console.error(error.stack ?? String(error)); process.exit(1); });
}
