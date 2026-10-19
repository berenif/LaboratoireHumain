import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { createHash } from 'node:crypto';
import { parseGetUpCheckpoint, proveActivationPurity } from './getup-checkpoint-envelope-2026-10-19.mjs';
const input = resolve(process.argv[2]), output = resolve(process.argv[3]), manifestPath = resolve(process.argv[4]);
const profilePath = resolve(process.argv[5]);
const hash = b => createHash('sha256').update(b).digest('hex');
assert.equal(hash(readFileSync(profilePath)), 'ea6da63a9996d9528f3dc25041cf476fdf0cfa828f57c57d119b78c8ebd0f3a7');
const profile = JSON.parse(readFileSync(profilePath));
const features = ['lh-worker/root-translation-carry-experiment', 'lh-sim/getup-terminal-balance-experiment', 'lh-sim/standing-dynamics-experiment'];
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
      if (!sameJson(sourceBaseline, approvedFixtureProfile)
        || !sameJson(message.testFixedGetUpProfile, approvedFixtureProfile)) throw new Error('Exact approved complete3+1 profile required; all other source settings unchanged');
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
replaceOnce('  let postPassStanding = null;', '  let postPassStanding = null;\n  let originalAnchorStanding = null;');
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
  'triggerNative, triggerEvidence, postPassAnchors, postPassStanding, originalAnchorStanding, carryActivationEvidence },');
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
manifest.scope = 'Fresh explicit3+1 fixed-fixture TEST. Only copiedworker uses existing profile constructor and activates default-OFF carry once at native0. Binaryworld/embeddedRuntime purity is mandatory before any physical integration; original anchors/counter gates stay unchanged.';
manifest.explicitProfile = { path: profilePath, sha256: hash(readFileSync(profilePath)), value: profile };
manifest.activeAlgorithm = 'f32-dq-f64-remainder-v1';
manifest.artifactSha256 = hashes(output);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify(manifest,null,2)+'\n');
console.log('Prepared explicit carry+3+1 fixture app '+output);
