import { writeFileSync } from 'node:fs';
import RAPIER from '@dimforge/rapier3d-compat';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { articulatedCoordinateResponse } from '../src/character/articulated-inertia.ts';
import { jointCoordinates, jointCoordinateKinematics } from '../src/character/joint-coordinates.ts';
import { add, scale, worldPoint } from '../src/character/math.ts';

const dt = 1 / 60, zero = { x: 0, y: 0, z: 0 };
const c = await createEmbodiedCharacter('canvas2d');
const initial = c.getSnapshot('canvas2d').segments.find(p => p.id === 'rightHand');
const anchor = { x: .025, y: .015, z: .01 }, start = worldPoint(initial.position, initial.rotation, anchor);
c.fixedUpdate(dt, { kind: 'begin', pointerId: 41, region: 'rightHand', segment: 'rightHand', localAnchor: anchor, worldTarget: start, timestampMs: 0 });
const results = [];
for (let frame = 1; frame <= 85; frame++) {
  c.fixedUpdate(dt, { kind: 'move', pointerId: 41, worldTarget: add(start, { x: 0, y: .03 * frame / 90, z: .70 * frame / 90 }), timestampMs: frame * dt * 1000 });
  if (![40, 80, 84, 85].includes(frame)) continue;
  const snapshot = c.world.takeSnapshot(), supports = c.motorSupportConstraints();
  const predicted = articulatedCoordinateResponse(c.ragdollBodies, supports);
  const ids = ['leftThigh', 'leftShin'];
  const measure = world => ids.map(id => {
    const d = SEGMENT_BY_ID.get(id), p = world.getRigidBody(c.ragdollBodies.get(d.parent).handle), b = world.getRigidBody(c.ragdollBodies.get(id).handle);
    const q = jointCoordinates(p.rotation(), b.rotation(), d.jointProfile);
    return { id, coordinates: q, velocity: jointCoordinateKinematics(p.angvel(), b.angvel(), p.rotation(), q, d.jointProfile).rates };
  });
  const branches = [];
  for (const inputId of ids) for (const impulse of [0, .1, -.1]) {
    const w = RAPIER.World.restoreSnapshot(snapshot);
    const d = SEGMENT_BY_ID.get(inputId), p = w.getRigidBody(c.ragdollBodies.get(d.parent).handle), b = w.getRigidBody(c.ragdollBodies.get(inputId).handle);
    const q = jointCoordinates(p.rotation(), b.rotation(), d.jointProfile);
    const axis = jointCoordinateKinematics(zero, zero, p.rotation(), q, d.jointProfile).torqueAxesWorld.x;
    b.applyTorqueImpulse(scale(axis, impulse), true); p.applyTorqueImpulse(scale(axis, -impulse), true);
    const queue = new RAPIER.EventQueue(true);
    w.step(queue, c.physicsHooks);
    branches.push({ inputId, impulse, after: measure(w) });
    queue.free(); w.free();
  }
  results.push({ frame, before: measure(c.world), supports,
    predicted: ids.flatMap(input => ids.map(output => ({ input, output, response: predicted.get(output, 'x', input, 'x') }))),
    measured: branches.filter(b => b.impulse).map(b => {
      const baseline = branches.find(a => a.inputId === b.inputId && a.impulse === 0);
      return { input: b.inputId, impulse: b.impulse, outputs: b.after.map((after, i) => ({
        id: after.id, response: (after.velocity.x - baseline.after[i].velocity.x) / b.impulse,
        deltaVelocity: after.velocity.x - baseline.after[i].velocity.x,
      })) };
    }), branches });
}
writeFileSync('evidence/collision/articulated-response.json', JSON.stringify({ dt, results }, null, 2));
console.log(JSON.stringify(results.map(({ frame, before, predicted, measured }) => ({ frame, before, predicted, measured }))));
c.dispose();
