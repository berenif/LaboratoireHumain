// SOURCE-ONLY draft. Reuses the reviewed fixed maker and baseline native0 carry recipe.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { createHash } from 'node:crypto';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname } from 'node:path';
const { parseGetUpCheckpoint, proveActivationPurity } = await import(pathToFileURL(resolve('docs/evidence/probes/getup-checkpoint-envelope-2026-10-19.mjs')).href);
const contractPath = join(dirname(fileURLToPath(import.meta.url)), 'teacher-feasibility-contract-2026-10-19.json');
const CONTRACT = JSON.parse(readFileSync(contractPath));
assert.equal(CONTRACT.executionApproved, true, 'SOURCE-ONLY draft: execution gate is false');
assert.equal(CONTRACT.schemaPinned, true, 'Final schema/freeze/audit must be pinned before use');
assert.equal(process.argv.length, 6, 'Usage: maker INPUT FRESH_OUTPUT FROZEN_MANIFEST PROFILE');
const input = resolve(process.argv[2]), output = resolve(process.argv[3]), manifestPath = resolve(process.argv[4]);
const profilePath = resolve(process.argv[5]);
const hash = b => createHash('sha256').update(b).digest('hex');
assert.equal(hash(readFileSync(resolve(CONTRACT.checkpointHelper.path))), CONTRACT.checkpointHelper.sha256);
assert.equal(hash(readFileSync(resolve(CONTRACT.nativeSourceFreeze.path))), CONTRACT.nativeSourceFreeze.sha256);
for (const [name,value] of Object.entries(CONTRACT.pendingFreeze)) assert.match(value ?? '', /^[a-f0-9]{64}$/, 'Pending final freeze: '+name);
assert.equal(hash(readFileSync(fileURLToPath(import.meta.url))), CONTRACT.pendingFreeze.adapterSourceSha256);
assert.equal(hash(readFileSync(manifestPath)), CONTRACT.pendingFreeze.approvedAppManifestSha256);
const frozenBuildManifest = JSON.parse(readFileSync(manifestPath));
assert.equal(frozenBuildManifest.sourceManifest.sha256, CONTRACT.pendingFreeze.nativeSourceManifestSha256);
assert.equal(hash(readFileSync(resolve(frozenBuildManifest.sourceManifest.path))), CONTRACT.pendingFreeze.nativeSourceManifestSha256);
const presentationBytes = readFileSync(resolve(CONTRACT.fixedPresentationManifest.path));
assert.equal(hash(presentationBytes), CONTRACT.fixedPresentationManifest.sha256);
const presentation = JSON.parse(presentationBytes);
for (const [path,expected] of Object.entries(presentation.artifactSha256)) {
  if (!CONTRACT.fixedPresentationManifest.allowedChangedAssets.includes(path)) assert.equal(hash(readFileSync(join(input,path))),expected,'Fixed renderer/driver/transport asset changed: '+path);
}
assert.equal(hash(readFileSync(profilePath)), 'aaa415a19767c218a26b05bad277193cccba2a726b52dcd5aac0a5444afc39d6');
const profile = JSON.parse(readFileSync(profilePath));
const features = CONTRACT.features;
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
// This branch is the real-world stop. It runs AFTER the existing trigger plus
// zero-tick refresh and BEFORE the pending ordinary advance(1) can integrate.
replaceOnce('let carryActivationEvidence = null;', `let carryActivationEvidence = null;
let teacherTriggeredReply = null;
let teacherTerminalLatch = false;
let teacherTerminalEvidence = null;
const teacherContract = ${JSON.stringify(CONTRACT)};
const rawError = error => ({ name: error?.name ?? null, message: String(error), stack: error?.stack ?? null });
const equalBytes = (a,b) => a instanceof Uint8Array && b instanceof Uint8Array
  && a.length === b.length && a.every((value,index) => value === b[index]);
function exportTeacherOnceAtTerminal() {
  if (teacherTerminalLatch) throw new Error('Terminal teacher export cannot repeat');
  // Latch before reports, checkpoint capture or export. Every error path stays
  // closed to subsequent advances, commands, resets and replacement runtimes.
  teacherTerminalLatch = true;
  const e = teacherTerminalEvidence = { terminal: true, realNativeSubsteps: triggerNative,
    exportMethod: teacherContract.exportMethod, exportCalls: 0, terminalLatchBeforeCall: true,
    beforeCheckpoint: null, afterCheckpoint: null, rawExportJson: null,
    exportError: null, captureError: null, invariantErrors: [],
    beforeControllerJson: null, afterControllerJson: null,
    beforeFixtureJson: null, afterFixtureJson: null,
    checkpointBytesExactlyEqual: false, controllerReportExactlyEqual: false,
    fixtureReportExactlyEqual: false, noAdvanceInsideCheckpointBracket: true };
  try {
    if (fixedFixture !== 'back' || triggerNative !== teacherContract.terminalNative
      || teacherTriggeredReply?.completed_ticks !== 0
      || teacherTriggeredReply.snapshot.simulation_time_s !== teacherContract.terminalNative / teacherContract.nativeHz) {
      throw new Error('Teacher export requires the original trigger plus advance(0) at natural BACK native480');
    }
    e.beforeControllerJson = runtime.getup_report();
    e.beforeFixtureJson = runtime.getup_fixture_report();
    const before = JSON.parse(e.beforeControllerJson), fixture = JSON.parse(e.beforeFixtureJson);
    if (before.started_substep !== null || before.impact_substep !== 480
      || !Array.isArray(before.teacher_frames) || before.teacher_frames.length !== 0
      || !Array.isArray(before.reference) || before.reference.length !== 0
      || fixture.nativeSubsteps !== 480 || fixture.state.preparing !== false
      || fixture.state.trigger_substep !== 480 || !sameJson(fixture.profile, approvedFixtureProfile)) {
      throw new Error('Real controller must remain empty/unstarted at untouched trigger480');
    }
    // Nothing except the ONE immutable export lies between these two raw
    // checkpoint captures. No advance(0), verification, report or parse call.
    e.beforeCheckpoint = runtime.getup_checkpoint();
    try { e.exportCalls++; e.rawExportJson = runtime.getup_teacher_feasibility(); }
    catch (error) { e.exportError = rawError(error); }
    finally { e.afterCheckpoint = runtime.getup_checkpoint(); }
    e.afterControllerJson = runtime.getup_report();
    e.afterFixtureJson = runtime.getup_fixture_report();
    e.checkpointBytesExactlyEqual = equalBytes(e.beforeCheckpoint, e.afterCheckpoint);
    e.controllerReportExactlyEqual = e.beforeControllerJson === e.afterControllerJson;
    e.fixtureReportExactlyEqual = e.beforeFixtureJson === e.afterFixtureJson;
    if (!e.checkpointBytesExactlyEqual) e.invariantErrors.push('Raw full checkpoints differ across teacher export');
    if (!e.controllerReportExactlyEqual) e.invariantErrors.push('Read-only real controller report changed across teacher export');
    if (!e.fixtureReportExactlyEqual) e.invariantErrors.push('Read-only real fixture report changed across teacher export');
  } catch (error) { e.captureError = rawError(error); }
}`);
replaceOnce("if (!['back', 'belly', 'side', 'side-supported'].includes(message.testFixedGetUpFixture)",
  "if (message.testFixedGetUpFixture !== 'back'");
replaceOnce('  if (!fixture.state.ready) {', `  if (native >= teacherContract.terminalNative && !fixture.state.ready) {
    throw new Error('Natural BACK admission was not ready at original native480; no later preparation is permitted');
  }
  if (fixture.state.ready && native !== teacherContract.terminalNative) {
    throw new Error('Natural BACK admission occurred at an unexpected native boundary');
  }
  if (!fixture.state.ready) {`);
replaceOnce('  lastFixtureSnapshot = refreshed.snapshot;',
  '  lastFixtureSnapshot = refreshed.snapshot;\n  teacherTriggeredReply = refreshed;');
replaceOnce('    prepareFixtureTrigger(message);', `    prepareFixtureTrigger(message);
    if (teacherTriggeredReply !== null) {
      exportTeacherOnceAtTerminal();
      publish(message.requestId, teacherTriggeredReply, started, null);
      return; // Authoritative stop: never reach the ordinary advance(1).
    }`);
replaceOnce("    if (message.type === 'recycle') {", `    if (teacherTerminalLatch && !['recycle','shutdown','getupReport','getupCheckpoint'].includes(message.type)) {
      throw new Error('Terminal teacher latch prohibits all advances, physical commands, resets and initialization, including after export failure');
    }
    if (message.commands?.length) throw new Error('Physical commands are forbidden in teacher-only feasibility');
    if (message.type === 'recycle') {`);
replaceOnce('originalAnchorStanding, fixtureSolverBudget, fixtureRootRotationPolicy, carryActivationEvidence },',
  'originalAnchorStanding, fixtureSolverBudget, fixtureRootRotationPolicy, carryActivationEvidence, teacherTerminalEvidence },');

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
manifest.scope = 'Isolated fresh actual-WASM BACK teacher-only feasibility. Complete approved repeat profile, native0 carry once, original80-native preparation polling. Real integration ends at natural trigger480 after existing trigger+advance(0), before pending advance(1). One inline immutable teacher export, raw whole-checkpoint equality and unchanged controller/fixture reports. Terminal latch prohibits every further advance/command/reset even on errors. No real source controller start, suffix, reference application, handoff, recovery counters or performance claim.';
manifest.explicitProfile = { path: profilePath, sha256: hash(readFileSync(profilePath)), value: profile };
manifest.activeAlgorithm = 'f32-dq-f64-remainder-v1';
manifest.unitRootAlgorithm = 'unit-free-root-v1';
manifest.teacherFeasibilityContract = { path: contractPath, sha256: hash(readFileSync(contractPath)), value: CONTRACT };
manifest.teacherFeasibilitySource = { path: fileURLToPath(import.meta.url), sha256: hash(readFileSync(fileURLToPath(import.meta.url))) };
manifest.artifactSha256 = hashes(output);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared teacher-only trigger480 terminal fixture app '+output);
