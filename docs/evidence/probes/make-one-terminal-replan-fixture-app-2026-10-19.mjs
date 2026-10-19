import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { createHash } from 'node:crypto';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';

function requireTerminalReplanReport(value, native) {
  const keys = o => o && typeof o === 'object' && !Array.isArray(o) ? Object.keys(o).sort().join(',') : '';
  if (keys(value) !== 'maximumReplansPerPhysicalTrial,state' || value.maximumReplansPerPhysicalTrial !== 1
    || keys(value.state) !== 'attempted_native_substep,format,prior_forecast_substeps,settled_native_samples,used') throw new Error('Strict actual one-replan readback required');
  const s = value.state;
  if (s.format !== 'one-terminal-reference-fall-replan-v1' || typeof s.used !== 'boolean'
    || !Number.isSafeInteger(s.settled_native_samples) || s.settled_native_samples < 0 || s.settled_native_samples >= 240
    || !Number.isSafeInteger(s.prior_forecast_substeps) || s.prior_forecast_substeps < 0 || s.prior_forecast_substeps > 5000
    || (s.used ? s.settled_native_samples !== 0 || !Number.isSafeInteger(s.attempted_native_substep)
      || s.attempted_native_substep < 0 || s.attempted_native_substep > native || s.prior_forecast_substeps === 0
      : s.attempted_native_substep !== null || s.prior_forecast_substeps !== 0)) throw new Error('Invalid actual one-replan budget state');
  return value;
}

const input = resolve(process.argv[2]), output = resolve(process.argv[3]), manifestPath = resolve(process.argv[4]);
const profilePath = resolve(process.argv[5]);
const hash = b => createHash('sha256').update(b).digest('hex');
assert.equal(hash(readFileSync(profilePath)), '9193400c547b0d3457e4cd8f8e609932c245b0b2a26fe037410451dba4e350e3');
const profile = JSON.parse(readFileSync(profilePath));
const features = ['lh-worker/root-translation-carry-experiment', 'lh-sim/standing-dynamics-experiment', 'lh-sim/unit-root-rotation-experiment', 'lh-sim/getup-terminal-replan-experiment'];
assert.deepEqual([...JSON.parse(readFileSync(manifestPath)).extraFeatures].sort(), [...features].sort());
const built = spawnSync(process.execPath, ['docs/evidence/probes/make-fixed-getup-fixture-app-2026-10-19.mjs', input, output, manifestPath], { encoding: 'utf8' });
assert.equal(built.status, 0, built.stderr || built.stdout);
let source = readFileSync(join(output, 'sim-worker.js'), 'utf8');
assert.equal(hash(source), 'da24141dabdd72cba4e57c84b6d40cdd2e876f8ac1f9555c369db8ed33947df2');
function replaceOnce(a,b) { assert.equal(source.split(a).length, 2, a); source = source.replace(a,b); }
replaceOnce('let postPassAnchors = null;', `let postPassAnchors = null;
let carryActivationEvidence = null;
let originalStandingAnchors = null;
let lastTerminalReplanState = null;
${requireTerminalReplanReport.toString()}
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
      sourceBaseline.getup_repeated_recovery = 'completed-cycle-one-terminal-reference-replan-v1';
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
      const unusedReplanBefore = requireTerminalReplanReport(beforeReport.terminalReferenceReplan, 0);
      if (unusedReplanBefore.state.used || unusedReplanBefore.state.settled_native_samples !== 0) throw new Error('Native0 replan budget must be unused and unprimed');
      runtime.set_root_translation_compensation_enabled(true);
      const afterCheckpoint = runtime.getup_checkpoint();
      const afterVerification = JSON.parse(runtime.verify_getup_checkpoint(afterCheckpoint));
      const purity = proveActivationPurity(beforeCheckpoint, afterCheckpoint);
      const afterReport = JSON.parse(runtime.getup_fixture_report());
      const unusedReplanAfter = requireTerminalReplanReport(afterReport.terminalReferenceReplan, 0);
      if (!sameJson(unusedReplanBefore, unusedReplanAfter)) throw new Error('Carry activation changed original one-replan budget');
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
replaceOnce('  let postPassStanding = null;', `  const currentStandingDiagnostic = JSON.parse(runtime.getup_standing_diagnostic('null'));
  if (currentStandingDiagnostic.physicsIntegrations !== 0
    || currentStandingDiagnostic.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)) throw new Error('Current read-only diagnostic clock mismatch');
  let postPassStanding = null;
  let originalAnchorStanding = null;
  const budgetReport = JSON.parse(runtime.getup_fixture_report());
  if (budgetReport.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)
    || !sameJson(budgetReport.profile, approvedFixtureProfile)
    || ![20,32].includes(budgetReport.solverBudget?.currentInternalPgsIterations)
    || ![20,32].includes(budgetReport.solverBudget?.selectedNextInternalPgsIterations)) throw new Error('Actual typed solver-budget report mismatch');
  const fixtureSolverBudget = budgetReport.solverBudget;
  const fixtureRootRotationPolicy = budgetReport.rootRotationNormalization;
  const fixtureTerminalReferenceReplan = requireTerminalReplanReport(budgetReport.terminalReferenceReplan,
    Math.round(reply.snapshot.simulation_time_s * 800));
  if (lastTerminalReplanState?.used && (!fixtureTerminalReferenceReplan.state.used
    || fixtureTerminalReferenceReplan.state.attempted_native_substep !== lastTerminalReplanState.attempted_native_substep
    || fixtureTerminalReferenceReplan.state.prior_forecast_substeps !== lastTerminalReplanState.prior_forecast_substeps)) throw new Error('One-per-world replan budget renewed or attempt changed');
  lastTerminalReplanState = structuredClone(fixtureTerminalReferenceReplan.state);
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
  'triggerNative, triggerEvidence, postPassAnchors, postPassStanding, originalAnchorStanding, fixtureSolverBudget, fixtureRootRotationPolicy, fixtureTerminalReferenceReplan, currentStandingDiagnostic, carryActivationEvidence },');
replaceOnce('  triggerEvidence = null;', '  triggerEvidence = null;\n  carryActivationEvidence = null;');
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
manifest.scope = 'Fresh explicit one-terminal-reference-replan numerical-profile TEST. Only copiedworker uses typed fixture initialization, once-only native0 carry activation and immutable live budget readback. No physical commands/reset/replan API call. Binaryworld/Runtime purity and unused replan state are preserved at activation; original25s epoch, anchor and counter gates remain unchanged.';
manifest.oneTerminalReplanAlgorithm = 'one-terminal-reference-fall-replan-v1';
manifest.standingObserver = 'Existing immutable current standing/contact/generalized-twist diagnostic each40Hz publication with null drift before anchors; after an earned pass, exact original controller stable_feet and post-pass anchors are retained. All queries report0integrations; no continuous800Hz row export.';
manifest.explicitProfile = { path: profilePath, sha256: hash(readFileSync(profilePath)), value: profile };
manifest.activeAlgorithm = 'f32-dq-f64-remainder-v1';
manifest.unitRootAlgorithm = 'unit-free-root-v1';
manifest.artifactSha256 = hashes(output);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared explicit carry+3+1 fixture app '+output);
