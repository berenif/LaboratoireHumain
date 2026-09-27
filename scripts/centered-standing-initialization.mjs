import assert from 'node:assert/strict';
import { writeFileSync } from 'node:fs';
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { BALANCE_LIMITS } from '../src/character/BalanceController.ts';
import { add, dot, length, scale, sub, worldPoint } from '../src/character/math.ts';
import { composeUprightPose } from '../src/character/pose.ts';
import { measureMassState } from '../src/character/mass-state.ts';
import { recoverySupportHull } from '../src/character/recovery-support.ts';

const soleIds = ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const zero = { x: 0, y: 0, z: 0 };
function centroid(poses) {
  const hull = recoverySupportHull(soleIds.flatMap(id => {
    const pose = poses.get(id);
    return SEGMENT_BY_ID.get(id).geometry.supportPatch.map(point => worldPoint(pose.position, pose.rotation, point));
  }));
  let area2 = 0, x = 0, z = 0;
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i], b = hull[(i + 1) % hull.length], signed = a.x * b.z - b.x * a.z;
    area2 += signed; x += (a.x + b.x) * signed; z += (a.z + b.z) * signed;
  }
  assert.ok(Math.abs(area2) > 1e-6);
  return { x: x / (3 * area2), y: 0, z: z / (3 * area2) };
}

// Called only after tsx registration. The prototype is supplied by a completed
// control construction. Change the planning function before body creation,
// then restore it immediately; never reset or reposition a dynamic assembly.
export async function createCenteredStandingCharacter(createCharacter, prototype, heading, receiptPath) {
  assert.equal(typeof prototype?.initialPose, 'function');
  const original = prototype.initialPose;
  let receipt;
  prototype.initialPose = function () {
    assert.equal(this.fixedSteps, 0);
    assert.equal(this.ragdollBodies.size, 0, 'Initialization must precede every dynamic body');
    assert.ok(!this.playground && !this.room, 'This diagnostic is limited to the flat standing fixture');
    const base = original.call(this), target = centroid(base);
    const forward = { x: Math.sin(heading), y: 0, z: Math.cos(heading) };
    const feet = Object.fromEntries(['leftFoot', 'rightFoot'].map(id => [id, base.get(id).position]));
    const rotations = Object.fromEntries(['leftFoot', 'rightFoot'].map(id => [id, base.get(id).rotation]));
    const evaluate = distance => {
      const rootTranslation = add(this.initialPosition, scale(forward, distance));
      const poses = composeUprightPose({ rootTranslation, heading,
        kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion, reactionOffset: zero,
        simulationTime: 0, activeGrab: null, supportFeet: feet, supportFootRotations: rotations, step: null }).poses;
      const mass = measureMassState(poses.values());
      return { distance, rootTranslation, poses, mass, residual: dot(sub(mass.position, target), forward) };
    };
    let lower = evaluate(0), upper = evaluate(BALANCE_LIMITS.maxStepReachM);
    const samples = [lower, upper].map(({ distance, residual }) => ({ distance, residual }));
    receipt = { target, samples, qualification: 'Kinematic initialization solve; construction is not yet verified.' };
    assert.ok(lower.residual < 0 && upper.residual > 0, 'The declared anatomical search interval must bracket the centroid');
    let solved;
    for (let iteration = 0; iteration < 40; iteration++) {
      solved = evaluate((lower.distance + upper.distance) / 2);
      samples.push({ distance: solved.distance, residual: solved.residual });
      if (Math.abs(solved.residual) < 1e-9) break;
      if (solved.residual < 0) lower = solved; else upper = solved;
    }
    assert.ok(Math.abs(solved.residual) < 1e-8, 'Initial planned COM did not converge');
    const footErrors = Object.fromEntries(soleIds.map(id => [id, length(sub(solved.poses.get(id).position, base.get(id).position))]));
    Object.assign(receipt, { rootShiftM: solved.distance, rootTranslation: solved.rootTranslation,
      pelvis: solved.poses.get('pelvis').position, centerOfMass: solved.mass.position, footErrors });
    assert.ok(Math.max(...Object.values(footErrors)) < 1e-5, `Planned sole anchors moved: ${JSON.stringify(footErrors)}`);
    const finalCentroid = centroid(solved.poses);
    assert.ok(Math.hypot(solved.mass.position.x - finalCentroid.x, solved.mass.position.z - finalCentroid.z) < 1e-5,
      'The final whole-body COM must match the actual planned support centroid');
    receipt = { target, finalCentroid, rootTranslation: solved.rootTranslation, pelvis: solved.poses.get('pelvis').position,
      centerOfMass: solved.mass.position, rootShiftM: solved.distance, footErrors, samples,
      qualification: 'A bounded kinematic initial pose is selected before dynamic bodies exist. No physical warmup, reset, pose write or velocity write is used.' };
    return solved.poses;
  };
  let character;
  try { character = await createCharacter('canvas2d', { heading }); }
  finally {
    prototype.initialPose = original;
    writeFileSync(receiptPath, JSON.stringify(receipt ?? { qualification: 'Initialization aborted before planning receipt' }, null, 2) + '\n', { flag: 'wx' });
  }
  assert.ok(receipt);
  const measured = measureMassState(character.poses.values()).position;
  const actualCentroid = centroid(character.poses);
  const measuredResidualM = Math.hypot(measured.x - actualCentroid.x, measured.z - actualCentroid.z);
  if (measuredResidualM >= 1e-5) { character.dispose(); throw new Error(`Constructed COM differs from planned centroid: ${measuredResidualM}`); }
  return { character, receipt: { ...receipt, measuredCenterOfMass: measured, measuredResidualM } };
}
