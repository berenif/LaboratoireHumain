import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { jointCoordinateKinematics, jointCoordinates } = await import('../src/character/joint-coordinates.ts');
const { articulatedCoordinateResponse } = await import('../src/character/articulated-inertia.ts');
const { scale } = await import('../src/character/math.ts');
const output = process.argv[2];
assert.ok(output && !existsSync(output));
const sourceBefore = fingerprints(), runs = [];
const targets = [['rightThigh', 'x'], ['rightShin', 'x'], ['rightAnkle', 'x'], ['rightFoot', 'z'], ['rightForefoot', 'x']];
const vec = v => ({ x: v.x, y: v.y, z: v.z });
const state = (world, handles) => Object.fromEntries(handles.map(([id, handle]) => {
  const b = world.getRigidBody(handle), q = b.rotation(), frame = b.principalInertiaLocalFrame();
  return [id, { position: vec(b.translation()), rotation: { ...vec(q), w: q.w }, linear: vec(b.linvel()),
    angular: vec(b.angvel()), mass: b.mass(), localCom: vec(b.localCom()), inertia: vec(b.principalInertia()), frame: { ...vec(frame), w: frame.w } }];
}));
try {
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const c = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      c.floorCollider.setEnabled(false); c.world.gravity = { x: 0, y: 0, z: 0 };
      for (const collider of c.ragdollColliders.values()) collider.setCollisionGroups(0);
      for (const body of c.ragdollBodies.values()) body.wakeUp();
      const handles = [...c.ragdollBodies].map(([id, body]) => [id, body.handle]);
      const initial = state(c.world, handles), bytes = c.world.takeSnapshot();
      const definition = SEGMENT_BY_ID.get('rightAnkle'), parent = c.ragdollBodies.get(definition.parent), child = c.ragdollBodies.get(definition.id);
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile);
      const axis = jointCoordinateKinematics(parent.angvel(), child.angvel(), parent.rotation(), coordinates, definition.jointProfile).torqueAxesWorld.x;
      const model = articulatedCoordinateResponse(c.ragdollBodies);
      const predicted = targets.map(([id, coordinate]) => model.get(id, coordinate, 'rightAnkle', 'x'));
      const cases = [];
      for (const impulse of [0, 0.001, -0.001]) {
        const world = RAPIER.World.restoreSnapshot(bytes), queue = new RAPIER.EventQueue(true);
        try {
          assert.deepEqual(state(world, handles), initial);
          if (impulse) {
            world.getRigidBody(child.handle).applyTorqueImpulse(scale(axis, impulse), true);
            world.getRigidBody(parent.handle).applyTorqueImpulse(scale(axis, -impulse), true);
          }
          world.step(queue, c.physicsHooks);
          const rates = targets.map(([id, coordinate]) => {
            const d = SEGMENT_BY_ID.get(id), p = world.getRigidBody(c.ragdollBodies.get(d.parent).handle), b = world.getRigidBody(c.ragdollBodies.get(id).handle);
            const q = jointCoordinates(p.rotation(), b.rotation(), d.jointProfile);
            return jointCoordinateKinematics(p.angvel(), b.angvel(), p.rotation(), q, d.jointProfile).rates[coordinate];
          });
          cases.push({ impulse, rates, bodies: state(world, handles) });
        } finally { queue.free(); world.free(); }
      }
      const comparisons = cases.filter(s => s.impulse).map(s => ({ impulse: s.impulse,
        coordinates: targets.map(([id, coordinate], i) => {
          const uncorrected = s.rates[i] / s.impulse, incremental = (s.rates[i] - cases[0].rates[i]) / s.impulse;
          return { id, coordinate, predicted: predicted[i], baselineRate: cases[0].rates[i], uncorrected, incremental,
            uncorrectedPass: Math.abs(predicted[i] - uncorrected) <= Math.max(0.15, Math.abs(uncorrected) * 0.15),
            incrementalPass: Math.abs(predicted[i] - incremental) <= Math.max(0.15, Math.abs(incremental) * 0.15) };
        }) }));
      runs.push({ heading, snapshotSha256: sha256(bytes), initial, predicted, cases, comparisons });
      console.log(JSON.stringify({ heading, comparisons }));
    } finally { c.dispose(); }
  }
  const sourceAfter = fingerprints(); assert.deepEqual(sourceBefore, sourceAfter);
  writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), sourceBefore, sourceAfter, runs,
    qualification: 'Unchanged original free-ankle fixture plus zero and opposite-sign controls. Diagnostic only; original assertion and tolerance are unedited.' }, null, 2) + '\n', { flag: 'wx' });
} finally { unregister(); }
