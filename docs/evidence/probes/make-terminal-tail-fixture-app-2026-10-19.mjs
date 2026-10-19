import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { createHash } from 'node:crypto';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';
const input = resolve(process.argv[2]), output = resolve(process.argv[3]), manifestPath = resolve(process.argv[4]);
const profilePath = resolve(process.argv[5]);
const arm = process.argv[6];
assert.ok(['baseline','candidate'].includes(arm), 'Explicit endpoint A/B arm required');
const candidate = arm === 'candidate';
const hash = b => createHash('sha256').update(b).digest('hex');
assert.equal(hash(readFileSync(profilePath)), 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
const profile = JSON.parse(readFileSync(profilePath));
const features = ['lh-worker/root-translation-carry-experiment', 'lh-sim/getup-terminal-balance-experiment', 'lh-sim/standing-dynamics-experiment', 'lh-sim/unit-root-rotation-experiment', 'lh-sim/getup-repeat-cycle-experiment', 'lh-worker/getup-terminal-tail-observer-experiment', ...(candidate ? ['lh-sim/getup-terminal-tail-duplicate-experiment'] : [])];
assert.deepEqual([...JSON.parse(readFileSync(manifestPath)).extraFeatures].sort(), [...features].sort());
const built = spawnSync(process.execPath, ['docs/evidence/probes/make-fixed-getup-fixture-app-2026-10-19.mjs', input, output, manifestPath], { encoding: 'utf8' });
assert.equal(built.status, 0, built.stderr || built.stdout);
let source = readFileSync(join(output, 'sim-worker.js'), 'utf8');
assert.equal(hash(source), 'da24141dabdd72cba4e57c84b6d40cdd2e876f8ac1f9555c369db8ed33947df2');
function replaceOnce(a,b) { assert.equal(source.split(a).length, 2, a); source = source.replace(a,b); }
replaceOnce('let postPassAnchors = null;', `let postPassAnchors = null;
let carryActivationEvidence = null;
let originalStandingAnchors = null;
let endpointNativeTrace = [];
let endpointTraceError = null;
let endpointNative0State = null;
let endpointBuildIdentity = null;
let endpointReferenceEvidence = null;
let endpointReferenceSent = false;
const approvedFixtureProfile = ${JSON.stringify(profile)};
const canonical = value => value && typeof value === 'object'
  ? Array.isArray(value) ? value.map(canonical) : Object.fromEntries(Object.keys(value).sort().map(k => [k, canonical(value[k])])) : value;
const sameJson = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
${parseGetUpCheckpoint.toString()}
${proveActivationPurity.toString()}
const endpointCandidate = ${candidate};
function verifyEndpointCheckpoint(bytes) {
  try {
    const value = JSON.parse(runtime.verify_getup_checkpoint(bytes));
    if (endpointCandidate) throw new Error('Candidate unexpectedly supports checkpoint resume');
    return value;
  } catch (error) {
    if (!endpointCandidate || !String(error).includes('Endpoint-only A/B candidate does not support checkpoint resume')) throw error;
    const actual = JSON.parse(runtime.getup_fixture_report());
    return { passed: false, expectedCandidateResumeRefusal: true, error: String(error),
      physicsIntegrations: 0, nativeSubsteps: actual.nativeSubsteps,
      forecastSubsteps: lastFixtureSnapshot?.diagnostics.getup?.policy?.forecast_substeps ?? 0 };
  }
}`);
replaceOnce('      fixedFixtureProfile = JSON.parse(getup_trial_profile());', `      const sourceBaseline = JSON.parse(getup_trial_profile());
      sourceBaseline.id = approvedFixtureProfile.id;
      sourceBaseline.solver_iterations = 3;
      sourceBaseline.additional_body_iterations = 1;
      sourceBaseline.getup_solver_budget = 'consistent-recovery-phase-v1';
      sourceBaseline.root_rotation_normalization = 'unit-free-root-v1';
      sourceBaseline.getup_repeated_recovery = 'completed-cycle-confirmed-fall-v1';
      if (!sameJson(sourceBaseline, approvedFixtureProfile)
        || !sameJson(message.testFixedGetUpProfile, approvedFixtureProfile)) throw new Error('Exact approved complete3+1 phase-budget profile required; only declared profile differences allowed');
      fixedFixtureProfile = message.testFixedGetUpProfile;`);
replaceOnce('fixedFixtureProfile.solver_iterations !== 20 || fixedFixtureProfile.additional_body_iterations !== 5',
  'fixedFixtureProfile.solver_iterations !== 3 || fixedFixtureProfile.additional_body_iterations !== 1');
replaceOnce('      const next = WorkerRuntime.getup_fixture(fixedFixture);',
  '      const next = WorkerRuntime.getup_fixture_with_profile(fixedFixture, JSON.stringify(fixedFixtureProfile));');
replaceOnce('      runtime = next;', `      runtime = next;
      endpointBuildIdentity = JSON.parse(runtime.terminal_tail_build_identity());
      if (endpointBuildIdentity.experiment !== 'terminal-tail-ab-v1'
        || endpointBuildIdentity.variant !== (endpointCandidate ? 'last-interpolated-duplicate' : 'source-canonical')
        || endpointBuildIdentity.checkpointResumeSupported !== !endpointCandidate
        || endpointBuildIdentity.nativeTrace !== 'read-only-post-native-observer-v1'
        || endpointBuildIdentity.maxTicksPerCall !== 1) throw new Error('Wrong endpoint A/B build identity');
      const readNative0 = () => {
        const observed = JSON.parse(runtime.advance_terminal_tail_observed(0));
        const value = observed.native0State;
        if (observed.traceError !== null || observed.nativeTrace.length !== 0
          || !sameJson(observed.buildIdentity, endpointBuildIdentity)
          || !value || value.nativeSubsteps !== 0 || value.physicsIntegrations !== 0
          || value.physicalObservationError !== null || value.integratorReadError !== null
          || value.poseWords?.length !== 325 || value.physicalObservationWords?.length !== 67
          || typeof value.rootTranslationIntegratorStateJson !== 'string') throw new Error('Invalid read-only native0 receipt');
        observed.native0State.current = JSON.parse(runtime.getup_standing_diagnostic('null'));
        return observed;
      };
      const expectedExportRefusal = () => {
        try { runtime.getup_checkpoint(); }
        catch (error) {
          if (String(error).includes('Endpoint-only A/B candidate does not support checkpoint export')) {
            return { expected: true, operation: 'checkpoint-export', error: String(error),
              evidenceOnly: true, noCheckpointBytes: true };
          }
          throw error;
        }
        throw new Error('Candidate unexpectedly exported an unmarked checkpoint');
      };
      const expectedResumeRefusal = () => {
        try {
          const refused = WorkerRuntime.from_getup_checkpoint(new Uint8Array());
          refused.free();
        } catch (error) {
          if (String(error).includes('Endpoint-only A/B candidate does not support checkpoint resume')) {
            return { expected: true, operation: 'checkpoint-restore', error: String(error),
              inputBytes: 0, noDecodedOrCreatedRuntime: true };
          }
          throw error;
        }
        throw new Error('Candidate unexpectedly supported checkpoint restore');
      };
      const checkpointRestoreRefusal = endpointCandidate ? expectedResumeRefusal() : null;
      const beforeObserved = readNative0();
      const beforeRefresh = beforeObserved.reply, beforeNative0State = beforeObserved.native0State;
      const beforeReport = JSON.parse(runtime.getup_fixture_report());
      const beforeCurrent = JSON.parse(runtime.getup_standing_diagnostic('null'));
      const beforeCheckpoint = endpointCandidate ? null : runtime.getup_checkpoint();
      const beforeVerification = endpointCandidate ? expectedExportRefusal() : verifyEndpointCheckpoint(beforeCheckpoint);
      if (beforeReport.nativeSubsteps !== 0 || beforeReport.rootTranslationCompensation !== undefined
        || !sameJson(beforeReport.profile, approvedFixtureProfile)) throw new Error('Fresh inactive native0 fixture/profile required');
      runtime.set_root_translation_compensation_enabled(true);
      // Baseline checkpoint bytes bracket ONLY activation, before another
      // legitimate zero-tick observation advances observation_sequence.
      const afterCheckpoint = endpointCandidate ? null : runtime.getup_checkpoint();
      const afterVerification = endpointCandidate ? expectedExportRefusal() : verifyEndpointCheckpoint(afterCheckpoint);
      const afterObserved = readNative0();
      const afterRefresh = afterObserved.reply, afterNative0State = afterObserved.native0State;
      const afterReport = JSON.parse(runtime.getup_fixture_report());
      const afterCurrent = JSON.parse(runtime.getup_standing_diagnostic('null'));
      const beforeCarry = JSON.parse(beforeNative0State.rootTranslationIntegratorStateJson);
      const afterCarry = JSON.parse(afterNative0State.rootTranslationIntegratorStateJson);
      if (beforeCarry.length !== 1 || afterCarry.length !== 1 || beforeCarry[0].state !== null
        || !sameJson(beforeCarry[0].root, afterCarry[0].root)
        || afterCarry[0].state?.qbits?.length !== 3 || afterCarry[0].state?.remainderbits?.length !== 3
        || !afterCarry[0].state.qbits.every(n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff)
        || !afterCarry[0].state.remainderbits.every(n => n === 0)) throw new Error('Native0 carry identity/zero-history readback required');
      const livePurity = {
        method: 'read-only-live-native0-observer-receipts', beforeNativeSubsteps: 0, afterNativeSubsteps: 0,
        poseVelocityWordsEqual: sameJson(beforeNative0State.poseWords, afterNative0State.poseWords),
        freshPhysicalObservationWordsEqual: sameJson(beforeNative0State.physicalObservationWords, afterNative0State.physicalObservationWords),
        currentDiagnosticExactlyEqual: sameJson(beforeCurrent, afterCurrent),
        jointQExactlyEqual: sameJson(beforeReport.jointQ, afterReport.jointQ),
        jointDqExactlyEqual: sameJson(beforeReport.jointDq, afterReport.jointDq),
        root: afterCarry[0].root, algorithm: 'f32-dq-f64-remainder-v1', initialRemainderBits: [0,0,0],
        binaryWorldEqualityClaim: false, embeddedRuntimeStringEqualityClaim: false,
      };
      if (!livePurity.poseVelocityWordsEqual || !livePurity.freshPhysicalObservationWordsEqual
        || !livePurity.currentDiagnosticExactlyEqual || !livePurity.jointQExactlyEqual || !livePurity.jointDqExactlyEqual) throw new Error('Carry activation changed live native0 physical state');
      const purity = endpointCandidate ? livePurity : proveActivationPurity(beforeCheckpoint, afterCheckpoint);
      if (!sameJson(beforeReport.solverBudget, afterReport.solverBudget)) throw new Error('Carry activation changed actual solver-budget selection');
      if (!sameJson(beforeReport.rootRotationNormalization, afterReport.rootRotationNormalization)
        || afterReport.rootRotationNormalization?.algorithm !== 'unit-free-root-v1'
        || afterReport.rootRotationNormalization.enabled !== true
        || afterReport.rootRotationNormalization.validationError !== null
        || !sameJson(afterReport.rootRotationNormalization.root, [0,0])) throw new Error('Actual physical unit-root policy missing or changed by carry activation');
      const beforeWords = poseWords(beforeRefresh.snapshot), afterWords = poseWords(afterRefresh.snapshot);
      const verificationOkay = endpointCandidate
        ? beforeVerification.expected === true && afterVerification.expected === true
        : [beforeVerification,afterVerification].every(v => v.passed === true
          && v.physicsIntegrations === 0 && v.nativeSubsteps === 0 && v.forecastSubsteps === 0
          && v.rootIntegratorStateEqual === true);
      if (beforeRefresh.completed_ticks !== 0 || afterRefresh.completed_ticks !== 0
        || beforeRefresh.snapshot.simulation_time_s !== 0 || afterRefresh.snapshot.simulation_time_s !== 0
        || beforeRefresh.snapshot.stamp.generation !== 1 || afterRefresh.snapshot.stamp.generation !== 1
        || !sameJson(beforeWords, afterWords) || beforeWords.length !== 325
        || beforeReport.state.preparing !== true || afterReport.state.preparing !== true
        || beforeRefresh.snapshot.diagnostics.getup.policy.forecast_substeps !== 0
        || afterRefresh.snapshot.diagnostics.getup.policy.forecast_substeps !== 0
        || afterReport.nativeSubsteps !== 0 || !sameJson(afterReport.profile, approvedFixtureProfile)
        || afterReport.rootTranslationCompensation?.algorithm !== purity.algorithm
        || !sameJson(afterReport.rootTranslationCompensation?.activeRoots, [purity.root])
        || !verificationOkay) throw new Error('Activation violated native0 physical/profile/carry purity');
      fixedFixtureProfile = afterReport.profile;
      carryActivationEvidence = { activationCalls: 1, purity, livePurity, beforeReport, afterReport,
        beforeNative0State, afterNative0State, beforeCurrent, afterCurrent,
        beforeVerification, afterVerification, beforeCheckpoint, afterCheckpoint,
        candidateCheckpointRefusals: endpointCandidate ? [beforeVerification, afterVerification, checkpointRestoreRefusal] : [],
        checkpointExportRefused: endpointCandidate && beforeVerification.expected && afterVerification.expected,
        checkpointRestoreRefused: endpointCandidate && checkpointRestoreRefusal.expected,
        checkpointExportRefusal: endpointCandidate ? [beforeVerification.error, afterVerification.error] : null,
        checkpointRestoreRefusal: checkpointRestoreRefusal?.error ?? null,
        binaryCheckpointPurityClaim: !endpointCandidate,
        poseVelocityF32WordsBefore: beforeWords, poseVelocityF32WordsAfter: afterWords,
        nativePhysicsBefore: 0, nativePhysicsAfter: 0, forecastBefore: 0, forecastAfter: 0 };`);
replaceOnce('  let postPassStanding = null;', `  if (!endpointReferenceSent && reply.snapshot.diagnostics.getup?.policy?.reference_frames > 0) {
    endpointReferenceEvidence = JSON.parse(runtime.getup_report());
    endpointReferenceSent = true;
  }
  let postPassStanding = null;
  let originalAnchorStanding = null;
  const budgetReport = JSON.parse(runtime.getup_fixture_report());
  if (budgetReport.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)
    || !sameJson(budgetReport.profile, approvedFixtureProfile)
    || ![20,32].includes(budgetReport.solverBudget?.currentInternalPgsIterations)
    || ![20,32].includes(budgetReport.solverBudget?.selectedNextInternalPgsIterations)) throw new Error('Actual typed solver-budget report mismatch');
  const fixtureSolverBudget = budgetReport.solverBudget;
  const fixtureRootRotationPolicy = budgetReport.rootRotationNormalization;
  if (fixtureRootRotationPolicy?.algorithm !== 'unit-free-root-v1' || fixtureRootRotationPolicy.enabled !== true
    || fixtureRootRotationPolicy.validationError !== null || !sameJson(fixtureRootRotationPolicy.root,[0,0])
    || !Number.isFinite(fixtureRootRotationPolicy.rootNormSquared)
    || Math.abs(fixtureRootRotationPolicy.rootNormSquared-1) > .001) throw new Error('Actual physical unit-root policy/domain mismatch');`);
replaceOnce("    if (postPassAnchors === null) postPassAnchors = ['left_foot', 'right_foot'].map(id => {", `    if (originalStandingAnchors === null) {
      const controller = JSON.parse(runtime.getup_report());
      if (!Array.isArray(controller.stable_feet) || controller.stable_feet.length !== 2
        || !controller.stable_feet.every(p => ['x','y','z'].every(k => Number.isFinite(p[k])))) throw new Error('Exact original standing anchors required');
      originalStandingAnchors = controller.stable_feet;
    }
    const original = JSON.parse(runtime.getup_standing_diagnostic(JSON.stringify(originalStandingAnchors)));
    if (original.physicsIntegrations !== 0 || original.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)) throw new Error('Original-anchor observer changed native clock');
    originalAnchorStanding = { nativeSubsteps: original.nativeSubsteps, physicsIntegrations: original.physicsIntegrations,
      predicates: original.predicates, footDriftM: original.footDriftM, feet: original.feet,
      originalStableFeet: originalStandingAnchors, thresholds: original.thresholds };
    if (postPassAnchors === null) postPassAnchors = ['left_foot', 'right_foot'].map(id => {`);
replaceOnce('triggerNative, triggerEvidence, postPassAnchors, postPassStanding },',
  'triggerNative, triggerEvidence, postPassAnchors, postPassStanding, originalAnchorStanding, fixtureSolverBudget, fixtureRootRotationPolicy, carryActivationEvidence, endpointNativeTrace, endpointBuildIdentity, endpointReferenceEvidence, endpointTraceError, endpointNative0State },');
replaceOnce('  triggerEvidence = null;', '  triggerEvidence = null;\n  carryActivationEvidence = null;\n  endpointNativeTrace = [];\n  endpointReferenceEvidence = null;\n  endpointTraceError = null;\n  endpointNative0State = null;');

// Same immutable native observer path in both arms. Exactly one original advance.
const ordinaryAdvance = `    if (profiling) {
      const json = runtime.advance_profiled(ticks);
      const parseStarted = performance.now();
      const result = JSON.parse(json);
      const parseMs = performance.now() - parseStarted;
      publish(message.requestId, result.reply, started, { ticks: result.timings, clockValid: result.clockValid,
        runtimeMs: result.runtimeMs, runtimeNonTickMs: result.runtimeNonTickMs,
        nativeOutputMs: runtime.native_output_ms(), jsonParseMs: parseMs });
    } else publish(message.requestId, JSON.parse(runtime.advance(ticks)), started, null);`;
replaceOnce(ordinaryAdvance, `    const observed = JSON.parse(runtime.advance_terminal_tail_observed(ticks));
    if (!Array.isArray(observed.nativeTrace)
      || observed.nativeTrace.length > 20 || !sameJson(observed.buildIdentity, endpointBuildIdentity)) {
      throw new Error('Bounded native endpoint observer failed: ' + JSON.stringify(observed.traceError));
    }
    endpointNativeTrace = observed.nativeTrace;
    endpointTraceError = observed.traceError;
    endpointNative0State = observed.native0State;
    if (endpointNative0State) endpointNative0State.current = JSON.parse(runtime.getup_standing_diagnostic('null'));
    publish(message.requestId, observed.reply, started, null);`);
replaceOnce('      const verification = JSON.parse(runtime.verify_getup_checkpoint(checkpoint));',
  '      const verification = verifyEndpointCheckpoint(checkpoint);');

writeFileSync(join(output, 'sim-worker.js'), source);
function hashes(root, base = root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes:true }).flatMap(entry => {
    const p = join(root,entry.name);
    return entry.isDirectory() ? Object.entries(hashes(p,base)) : [[relative(base,p),hash(readFileSync(p))]];
  }));
}
const manifest = JSON.parse(readFileSync(output + '-fixture-adapter-manifest.json'));
manifest.baseFixtureAdapterSha256 = manifest.adapterSha256;
manifest.adapterSha256 = hash(source);
manifest.scope = 'Fresh endpoint-only A/B on identical complete repeat3+1 profile. Copied worker uses the same bounded native observer in both arms and activates carry once at native0. Baseline retains binary checkpoint proof; candidate deliberately refuses export/restore and uses live native0 pose/current generalized/integrator receipts. Original renderer, driver, scheduler, anchors and gates preserved; no physical commands.';
manifest.explicitProfile = { path: profilePath, sha256: hash(readFileSync(profilePath)), value: profile };
manifest.endpointArm = arm;
manifest.endpointVariant = candidate ? 'last-interpolated-duplicate' : 'source-canonical';
manifest.endpointObserver = 'read-only-post-native-observer-v1';
manifest.checkpointResumeSupported = !candidate;
manifest.activeAlgorithm = 'f32-dq-f64-remainder-v1';
manifest.unitRootAlgorithm = 'unit-free-root-v1';
manifest.artifactSha256 = hashes(output);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared endpoint-only A/B fixture '+arm+' at '+output);
