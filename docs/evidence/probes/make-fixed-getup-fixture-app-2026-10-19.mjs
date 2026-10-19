// Isolated test artifact builder. Never edits the production sim-worker or driver.
import assert from 'node:assert/strict';
import { cpSync, existsSync, lstatSync, readFileSync, readdirSync, realpathSync, writeFileSync } from 'node:fs';
import { resolve, join, relative, dirname, sep } from 'node:path';
import { createHash } from 'node:crypto';

const input = resolve(process.argv[2]);
const output = resolve(process.argv[3]);
const expectedManifestPath = resolve(process.argv[4]);
const expectedManifest = JSON.parse(readFileSync(expectedManifestPath));
assert.ok(expectedManifest.artifactSha256 && typeof expectedManifest.artifactSha256 === 'object', 'Explicit frozen app hash manifest required');
assert.ok(existsSync(join(input, 'index.html')), 'Explicit built app required');
assert.ok(!existsSync(output), 'Fresh output required');
assert.ok(!existsSync(output + '-fixture-adapter-manifest.json'), 'Fresh sibling manifest required');
const root = resolve(process.cwd());
assert.ok(input.startsWith(root + sep) && output.startsWith(root + sep), 'Workspace-child artifacts required');
assert.equal(realpathSync(input), input, 'No symlinked input path');
assert.equal(realpathSync(dirname(output)), dirname(output), 'Existing non-symlink output parent required');
assert.ok(!output.startsWith(input + sep) && !input.startsWith(output + sep), 'Non-overlapping artifact trees required');
function requireRegularTree(path) {
  const stat = lstatSync(path);
  assert.ok(!stat.isSymbolicLink(), 'Artifact symlinks are not allowed');
  if (stat.isDirectory()) for (const entry of readdirSync(path)) requireRegularTree(join(path, entry));
  else assert.ok(stat.isFile(), 'Only regular artifact files are allowed');
}
requireRegularTree(input);
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
assert.deepEqual(hashes(input), expectedManifest.artifactSha256, 'Every input asset must match the approved frozen app manifest');
const original = readFileSync(join(input, 'sim-worker.js'), 'utf8');
assert.equal(hash(original), 'a3a2b18c222c045fc08bf19aff2e96b92f0bc18ad6023bcca660195054267996', 'Reviewed ordinary worker transport required');
let source = original;
function replaceOnce(old, next) {
  assert.equal(source.split(old).length, 2, 'Unique reviewed insertion point required');
  source = source.replace(old, next);
}
replaceOnce('let recoveryQualification = false;', `let recoveryQualification = false;
// Explicit fixed-fixture TEST adapter. No ordinary initialization is allowed here.
let fixedFixture = null;
let fixedFixtureProfile = null;
let lastFixtureSnapshot = null;
let triggerNative = null;
let triggerEvidence = null;
let postPassAnchors = null;
const poseWords = snapshot => {
  if (snapshot.segments.length !== 25 || new Set(snapshot.segments.map(p => p.id)).size !== 25) throw new Error('Expected25unique physical bodies/frames');
  const values = snapshot.segments.flatMap(pose => ['position', 'rotation', 'linearVelocity', 'angularVelocity']
    .flatMap(key => key === 'rotation' ? [pose[key].x, pose[key].y, pose[key].z, pose[key].w]
      : [pose[key].x, pose[key].y, pose[key].z]));
  if (values.length !== 325 || !values.every(Number.isFinite)) throw new Error('Expected325finite physical pose/velocity values');
  return Array.from(new Uint32Array(new Float32Array(values).buffer));
};
function prepareFixtureTrigger(message) {
  if (!fixedFixture || message.type !== 'advance') return;
  if (message.ticks !== 1) throw new Error('Fixed-fixture TEST requires ordinary one-tick advances');
  if (triggerNative !== null) return;
  const native = Math.round(lastFixtureSnapshot.simulation_time_s * 800);
  const pollingNative = fixedFixture.startsWith('side') ? 20 : 80;
  if (native === 0 || native % pollingNative !== 0) return;
  const fixture = JSON.parse(runtime.getup_fixture_report());
  if (fixture.nativeSubsteps !== native || lastFixtureSnapshot.simulation_time_s !== native / 800
    || fixture.state.kind !== fixedFixture || fixture.state.preparing !== true) {
    throw new Error('Fixture native boundary or preparation identity mismatch');
  }
  if (!fixture.state.ready) {
    const maximum = fixedFixture === 'side-supported' ? 3200 : 1600;
    if (native >= maximum) throw new Error('Fixed fixture was not physically admitted within its unchanged native preparation budget');
    return;
  }
  const before = poseWords(lastFixtureSnapshot);
  runtime.trigger_getup_fixture();
  const refreshed = JSON.parse(runtime.advance(0));
  const after = poseWords(refreshed.snapshot);
  const afterFixture = JSON.parse(runtime.getup_fixture_report());
  if (refreshed.completed_ticks !== 0 || refreshed.snapshot.simulation_time_s !== lastFixtureSnapshot.simulation_time_s
    || refreshed.snapshot.stamp.generation !== lastFixtureSnapshot.stamp.generation
    || before.length !== 325 || after.length !== 325
    || afterFixture.nativeSubsteps !== native || afterFixture.state.trigger_substep !== native
    || afterFixture.state.preparing !== false || afterFixture.state.kind !== fixedFixture
    || lastFixtureSnapshot.segments.some((pose, i) => pose.id !== refreshed.snapshot.segments[i].id)
    || before.some((word, i) => word !== after[i])) {
    throw new Error('Trigger changed physical pose/velocity bits or advanced the physical clock');
  }
  triggerNative = native;
  triggerEvidence = { native, beforeNativeSubsteps: fixture.nativeSubsteps, afterNativeSubsteps: afterFixture.nativeSubsteps,
    fixture: fixture.state, afterFixtureState: afterFixture.state, beforePoseWords: before,
    afterPoseWords: after, physicalPoseVelocityBitsUnchanged: true, refreshCompletedTicks: 0 };
  lastFixtureSnapshot = refreshed.snapshot;
}`);
replaceOnce('function publish(requestId, reply, started, timing) {', `function publish(requestId, reply, started, timing) {
  lastFixtureSnapshot = reply.snapshot;
  let postPassStanding = null;
  if (reply.snapshot.diagnostics.getup?.policy?.prototype_standing_passed) {
    if (postPassAnchors === null) postPassAnchors = ['left_foot', 'right_foot'].map(id => {
      const foot = reply.snapshot.segments.find(pose => pose.id === id);
      if (!foot) throw new Error('Missing standing foot');
      return { ...foot.position };
    });
    postPassStanding = JSON.parse(runtime.getup_standing_diagnostic(JSON.stringify(postPassAnchors)));
    if (postPassStanding.physicsIntegrations !== 0
      || postPassStanding.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)) {
      throw new Error('Standing observer advanced or mismatched the physical clock');
    }
  }`);
replaceOnce('    transferBuffersAllocated: buffers.length,', `    transferBuffersAllocated: buffers.length,
    fixtureTest: { isolatedTestOnly: true, kind: fixedFixture, profile: fixedFixtureProfile,
      preparationPollingNative: fixedFixture?.startsWith('side') ? 20 : 80,
      triggerNative, triggerEvidence, postPassAnchors, postPassStanding },`);
replaceOnce("    if (message.type === 'initialize') {", `    if (message.type === 'initialize') {
      if (runtime) throw new Error('Fixed-fixture TEST has already initialized its only worker runtime');
      if (!['back', 'belly', 'side', 'side-supported'].includes(message.testFixedGetUpFixture)
        || message.mode !== 'protocol' || message.heading !== 0 || message.generation !== 1
        || message.floorEnabled !== true || message.getupTrial !== true
        || message.profile !== undefined || message.qualification !== undefined) {
        throw new Error('Isolated fixed-fixture TEST requires an explicit kind and source-fidelity default20+5 prototype initialization');
      }
      fixedFixture = message.testFixedGetUpFixture;
      fixedFixtureProfile = JSON.parse(getup_trial_profile());
      if (fixedFixtureProfile.solver_iterations !== 20 || fixedFixtureProfile.additional_body_iterations !== 5
        || fixedFixtureProfile.getup_policy_enabled !== true || fixedFixtureProfile.dt_s !== 1 / 800
        || fixedFixtureProfile.tick_substeps !== 20) throw new Error('Unexpected fixed-fixture source profile');`);
replaceOnce('      const next = new WorkerRuntime(message.mode, message.heading, String(message.generation ?? 1), message.floorEnabled, profile, playground);',
  '      const next = WorkerRuntime.getup_fixture(fixedFixture);');
replaceOnce("      if (message.type === 'commands') {", `      if (message.type === 'commands') {
        throw new Error('Physical commands are disabled in this isolated autonomous fixed-fixture TEST');
      }
      if (message.type === 'commands') {`);
replaceOnce("    const ticks = message.type === 'advance' ? message.ticks : 0;", `    prepareFixtureTrigger(message);
    const ticks = message.type === 'advance' ? message.ticks : 0;`);
// Trigger evidence is intentionally retained in the first post-trigger snapshot.
// The harness records it; later packets retain only the lightweight native marker.
replaceOnce('    ...(recoveryQualification ? { recoveryProgress: JSON.parse(runtime.recovery_progress()), workerIdentity } : {}) }, [slot.buffer]);',
  '    ...(recoveryQualification ? { recoveryProgress: JSON.parse(runtime.recovery_progress()), workerIdentity } : {}) }, [slot.buffer]);\n  triggerEvidence = null;');
cpSync(input, output, { recursive: true });
writeFileSync(join(output, 'sim-worker.js'), source);
function hashes(root, base = root) {
  return Object.fromEntries(readdirSync(root, { withFileTypes: true }).flatMap(entry => {
    const path = join(root, entry.name);
    return entry.isDirectory() ? Object.entries(hashes(path, base)) : [[relative(base, path), hash(readFileSync(path))]];
  }));
}
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify({
  at: new Date().toISOString(), input, output, isolatedTestOnly: true,
  originalWorkerTransportSha256: hash(original), adapterSha256: hash(source),
  approvedFrozenManifest: { path: expectedManifestPath, sha256: hash(readFileSync(expectedManifestPath)) },
  sourceBuildFeatures: expectedManifest.extraFeatures ?? expectedManifest.features ?? expectedManifest.effectiveFeatures ?? null,
  scope: 'Only copied sim-worker initialization/fixture polling/trigger metadata differs. No production file, renderer, driver, WASM, physical model, source profile, actor or scheduler change.',
  preparation: 'Ordinary one-tick advances; ready is inspected only at the native probe polling boundaries: back/belly80native steps, lateral20. Trigger+advance(0) verifies identical325Float32 pose/velocity words before the next one-tick advance.',
  standingObserver: 'After first native-latched standing pass only: existing read-only getup_standing_diagnostic at40Hz with explicit post-pass foot anchors; zero physical integrations. Full800Hz qualification remains separate.',
  artifactSha256: hashes(output),
}, null, 2) + '\n');
console.log('Prepared isolated fixed-fixture TEST app at ' + output);
