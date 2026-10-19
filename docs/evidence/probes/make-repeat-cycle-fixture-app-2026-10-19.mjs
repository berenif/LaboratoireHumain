import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { createHash } from 'node:crypto';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';
const input = resolve(process.argv[2]), output = resolve(process.argv[3]), manifestPath = resolve(process.argv[4]);
const profilePath = resolve(process.argv[5]);
const hash = b => createHash('sha256').update(b).digest('hex');
assert.equal(hash(readFileSync(profilePath)), 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
const profile = JSON.parse(readFileSync(profilePath));
const features = ['lh-worker/root-translation-carry-experiment', 'lh-sim/getup-terminal-balance-experiment', 'lh-sim/standing-dynamics-experiment', 'lh-sim/unit-root-rotation-experiment', 'lh-sim/getup-repeat-cycle-experiment'];
assert.deepEqual([...JSON.parse(readFileSync(manifestPath)).extraFeatures].sort(), [...features].sort());
const built = spawnSync(process.execPath, ['docs/evidence/probes/make-fixed-getup-fixture-app-2026-10-19.mjs', input, output, manifestPath], { encoding: 'utf8' });
assert.equal(built.status, 0, built.stderr || built.stdout);
let source = readFileSync(join(output, 'sim-worker.js'), 'utf8');
assert.equal(hash(source), 'da24141dabdd72cba4e57c84b6d40cdd2e876f8ac1f9555c369db8ed33947df2');
function replaceOnce(a,b) { assert.equal(source.split(a).length, 2, a); source = source.replace(a,b); }
replaceOnce('let postPassAnchors = null;', `let postPassAnchors = null;
let carryActivationEvidence = null;
let originalStandingAnchors = null;
const approvedFixtureProfile = ${JSON.stringify(profile)};
const canonical = value => value && typeof value === 'object'
  ? Array.isArray(value) ? value.map(canonical) : Object.fromEntries(Object.keys(value).sort().map(k => [k, canonical(value[k])])) : value;
const sameJson = (a,b) => JSON.stringify(canonical(a)) === JSON.stringify(canonical(b));
${parseGetUpCheckpoint.toString()}
${proveActivationPurity.toString()}`);
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
      const beforeRefresh = JSON.parse(runtime.advance(0));
      const beforeReport = JSON.parse(runtime.getup_fixture_report());
      const beforeCheckpoint = runtime.getup_checkpoint();
      const beforeVerification = JSON.parse(runtime.verify_getup_checkpoint(beforeCheckpoint));
      if (beforeReport.nativeSubsteps !== 0 || beforeReport.rootTranslationCompensation !== undefined
        || !sameJson(beforeReport.profile, approvedFixtureProfile)) throw new Error('Fresh inactive native0 fixture/profile required');
      runtime.set_root_translation_compensation_enabled(true);
      const afterCheckpoint = runtime.getup_checkpoint();
      const afterVerification = JSON.parse(runtime.verify_getup_checkpoint(afterCheckpoint));
      const purity = proveActivationPurity(beforeCheckpoint, afterCheckpoint);
      const afterReport = JSON.parse(runtime.getup_fixture_report());
      if (!sameJson(beforeReport.solverBudget, afterReport.solverBudget)) throw new Error('Carry activation changed actual solver-budget selection');
      if (!sameJson(beforeReport.rootRotationNormalization, afterReport.rootRotationNormalization)
        || afterReport.rootRotationNormalization?.algorithm !== 'unit-free-root-v1'
        || afterReport.rootRotationNormalization.enabled !== true
        || afterReport.rootRotationNormalization.validationError !== null
        || !sameJson(afterReport.rootRotationNormalization.root, [0,0])) throw new Error('Actual physical unit-root policy missing or changed by carry activation');
      const afterRefresh = JSON.parse(runtime.advance(0));
      const beforeWords = poseWords(beforeRefresh.snapshot), afterWords = poseWords(afterRefresh.snapshot);
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
        || ![beforeVerification,afterVerification].every(v => v.passed === true
          && v.physicsIntegrations === 0 && v.nativeSubsteps === 0 && v.forecastSubsteps === 0
          && v.rootIntegratorStateEqual === true)) throw new Error('Activation violated native0 physical/profile/carry purity');
      fixedFixtureProfile = afterReport.profile;
      carryActivationEvidence = { activationCalls: 1, purity, beforeReport, afterReport,
        beforeVerification, afterVerification, beforeCheckpoint, afterCheckpoint,
        poseVelocityF32WordsBefore: beforeWords, poseVelocityF32WordsAfter: afterWords,
        nativePhysicsBefore: 0, nativePhysicsAfter: 0, forecastBefore: 0, forecastAfter: 0 };`);
replaceOnce('  let postPassStanding = null;', `  let postPassStanding = null;
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
  'triggerNative, triggerEvidence, postPassAnchors, postPassStanding, originalAnchorStanding, fixtureSolverBudget, fixtureRootRotationPolicy, carryActivationEvidence },');
replaceOnce('  triggerEvidence = null;', '  triggerEvidence = null;\n  carryActivationEvidence = null;');
// Read-only lifecycle observation is isolated to this copied worker. A physical
// perturbation may invalidate prior standing; never reuse old episode anchors.
replaceOnce('let originalStandingAnchors = null;', `let originalStandingAnchors = null;
let observedRecoveryStartS = null;
let previousRecoveryStartS = null;
let observedControllerStartChanges = [];
let ordinaryStrikeCommandsSeen = 0;
let previousNativeStableS = 0;`);
replaceOnce('  let postPassStanding = null;', `  const recoveryStart = reply.snapshot.diagnostics.getup?.policy?.started_at_s ?? null;
  if (recoveryStart !== observedRecoveryStartS) {
    previousRecoveryStartS = observedRecoveryStartS;
    observedControllerStartChanges.push({ from: observedRecoveryStartS, to: recoveryStart,
      nativeSubsteps: Math.round(reply.snapshot.simulation_time_s * 800) });
    observedRecoveryStartS = recoveryStart;
    originalStandingAnchors = null;
    postPassAnchors = null;
  }
  const stableNow = reply.snapshot.diagnostics.getup?.policy?.stable_standing_s ?? 0;
  if (stableNow < previousNativeStableS || reply.snapshot.motion !== 'Upright'
    || !reply.snapshot.diagnostics.getup?.policy?.prototype_standing_passed) {
    originalStandingAnchors = null;
    postPassAnchors = null;
  }
  previousNativeStableS = stableNow;
  const currentStandingDiagnostic = JSON.parse(runtime.getup_standing_diagnostic('null'));
  if (currentStandingDiagnostic.physicsIntegrations !== 0
    || currentStandingDiagnostic.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)) throw new Error('Current observer clock mismatch');
  let postPassStanding = null;`);
replaceOnce('if (reply.snapshot.diagnostics.getup?.policy?.prototype_standing_passed) {',
  `if (reply.snapshot.diagnostics.getup?.policy?.prototype_standing_passed
    && reply.snapshot.diagnostics.getup.policy.stable_standing_s >= 1
    && reply.snapshot.motion === 'Upright') {`);
replaceOnce("        throw new Error('Physical commands are disabled in this isolated autonomous fixed-fixture TEST');", `        const policy = lastFixtureSnapshot?.diagnostics.getup?.policy;
        if (!Array.isArray(message.commands) || message.commands.length !== 1
          || message.commands[0]?.action !== 'Strike' || ordinaryStrikeCommandsSeen !== 0
          || message.commands[0].stamp?.generation !== 1
          || triggerNative === null || lastFixtureSnapshot.motion !== 'Upright'
          || !policy?.prototype_standing_passed || policy.stable_standing_s < 4
          || fixedFixtureProfile?.getup_repeated_recovery !== 'completed-cycle-confirmed-fall-v1') {
          throw new Error('Only one ordinary Strike after initial earned pass plus3physical seconds is permitted');
        }
        ordinaryStrikeCommandsSeen++;`);
replaceOnce("    if (message.type === 'recycle') {", `    if (message.commands?.length && message.type !== 'commands') throw new Error('Commands require the original dedicated driver command request');
    if (message.type === 'recycle') {`);
replaceOnce('fixtureRootRotationPolicy, carryActivationEvidence },', `fixtureRootRotationPolicy, carryActivationEvidence,
      currentStandingDiagnostic,
      fixtureRecoveryEpoch: { startedAtS: observedRecoveryStartS, previousStartedAtS: previousRecoveryStartS,
        observedControllerStartChanges, ordinaryStrikeCommandsSeen } },`);

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
manifest.scope = 'Fresh explicit repeat-cycle lifecycle TEST on complete3+1 phase-budget/unit-root profile. Carry activates once at native0; only one ordinary driver Strike is allowed after initial native standing plus3physical seconds. Epoch-aware read-only diagnostics clear presentation-only anchors when controller source start changes. No reset, injected physical state, force override, or production edit.';
manifest.repeatedRecoveryAlgorithm = 'completed-cycle-confirmed-fall-v1';
manifest.explicitProfile = { path: profilePath, sha256: hash(readFileSync(profilePath)), value: profile };
manifest.activeAlgorithm = 'f32-dq-f64-remainder-v1';
manifest.unitRootAlgorithm = 'unit-free-root-v1';
manifest.artifactSha256 = hashes(output);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared explicit carry+3+1 fixture app '+output);
