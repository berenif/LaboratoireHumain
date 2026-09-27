import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENTS } = await import('../src/core/humanoid.ts');
const { quatMultiply, quatInverse, add, sub, scale, worldPoint } = await import('../src/character/math.ts');
const output = process.argv[2];
if (!output || existsSync(output)) throw new Error('Specify a fresh output JSON.');
const sourceBefore = fingerprints(), dt = 1 / 60, results = [];
const modes = (process.env.RIGID_MODES ?? 'fixed-joints,compound').split(',');
const exportSnapshots = process.env.RIGID_EXPORT_SNAPSHOTS === '1';
const snapshotTick = Number(process.env.RIGID_SNAPSHOT_TICK ?? 0);
assert.ok(snapshotTick === 0 || snapshotTick === 1);
assert.ok(modes.every(mode => ['fixed-joints', 'fixed-multibody', 'compound'].includes(mode)));
const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 };
const ids = ['pelvis', 'leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const state = c => Object.fromEntries(ids.map(id => {
  const body = c.ragdollBodies.get(id);
  return [id, { position: body.translation(), com: body.worldCom(), velocity: body.linvel(), rotation: body.rotation() }];
}));
try {
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) for (const mode of modes) {
    const c = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      const initial = state(c);
      const massBefore = [...c.ragdollBodies.values()].reduce((sum, body) => sum + body.mass(), 0);
      const comBefore = [...c.ragdollBodies.values()].reduce((sum, body) => add(sum, scale(body.worldCom(), body.mass() / massBefore)), zero);
      let readState = () => state(c), massAfter = massBefore, comAfter = comBefore;
      c.nativeMotors.disable(c.jointsByChild);
      for (const joint of c.ragdollJoints.splice(0)) c.world.removeImpulseJoint(joint, false);
      c.jointsByChild.clear();
      if (mode === 'fixed-joints' || mode === 'fixed-multibody') for (const definition of SEGMENTS.filter(d => d.parent && d.jointProfile)) {
        const parent = c.ragdollBodies.get(definition.parent), child = c.ragdollBodies.get(definition.id);
        const relative = quatMultiply(quatInverse(parent.rotation()), child.rotation());
        const data = RAPIER.JointData.fixed(definition.jointProfile.parentFrame.anchor,
          relative, definition.jointProfile.childFrame.anchor, identity);
        const joint = mode === 'fixed-multibody'
          ? c.world.createMultibodyJoint(data, parent, child, true)
          : c.world.createImpulseJoint(data, parent, child, true);
        assert.ok(joint?.isValid(), 'Native fixed joint creation failed');
        joint.setContactsEnabled(false);
        if (mode === 'fixed-joints') {
          c.ragdollJoints.push(joint); c.jointsByChild.set(definition.id, joint);
        }
      }
      if (mode === 'compound') {
        const descriptors = [...c.ragdollColliders.values()].map(collider => {
          const p = collider.translation();
          return new RAPIER.ColliderDesc(collider.shape).setTranslation(p.x, p.y, p.z).setRotation(collider.rotation())
            .setMass(collider.mass()).setFriction(collider.friction()).setFrictionCombineRule(collider.frictionCombineRule())
            .setRestitution(collider.restitution()).setContactSkin(collider.contactSkin()).setCollisionGroups(collider.collisionGroups());
        });
        for (const body of c.ragdollBodies.values()) c.world.removeRigidBody(body);
        c.ragdollBodies.clear(); c.ragdollColliders.clear();
        const compound = c.world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setCanSleep(false)
          .setAngularDamping(.52).setAdditionalSolverIterations(5).setCcdEnabled(true));
        for (const descriptor of descriptors) c.world.createCollider(descriptor, compound);
        compound.recomputeMassPropertiesFromColliders();
        massAfter = compound.mass(); comAfter = compound.worldCom();
        assert.ok(Math.abs(massAfter - massBefore) < 1e-4, 'Compound mass differs');
        assert.ok(Math.hypot(...Object.values(sub(comAfter, comBefore))) < 1e-6, 'Compound COM differs');
        readState = () => Object.fromEntries(ids.map(id => {
          const position = worldPoint(compound.translation(), compound.rotation(), initial[id].position);
          const com = worldPoint(compound.translation(), compound.rotation(), initial[id].com);
          const a = compound.rotation(), b = initial[id].rotation;
          // Preserve the native f32 quaternion components at identity. The
          // production composition helper normalizes its result, which would
          // make equal initial orientations compare unequal in this readback.
          const rotation = { x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
          return [id, { position, com, velocity: compound.velocityAtPoint(com), rotation }];
        }));
      }
      assert.deepEqual(JSON.parse(JSON.stringify(readState())), JSON.parse(JSON.stringify(initial)),
        'Creating the calibration fixture moved a body marker');
      const retainedBodyHandles = [...c.ragdollBodies.values()].map(body => body.handle);
      const jointCounts = { impulse: c.world.impulseJoints.len(), multibody: c.world.multibodyJoints.len() };
      if (mode !== 'compound') {
        assert.equal(retainedBodyHandles.length, 25);
        assert.ok(retainedBodyHandles.every(handle => c.world.getRigidBody(handle)?.isDynamic()));
        massAfter = [...c.ragdollBodies.values()].reduce((sum, body) => sum + body.mass(), 0);
        assert.equal(massAfter, massBefore);
      }
      const saveSnapshot = () => {
        if (!exportSnapshots || mode !== 'fixed-joints') return;
        const rawParts = handle => {
          const bytes = new ArrayBuffer(8), view = new DataView(bytes);
          view.setFloat64(0, handle, true);
          return [view.getUint32(0, true), view.getUint32(4, true)];
        };
        const colliderEntries = [...c.ragdollColliders.values()];
        const excludedColliderPairs = [];
        for (let i = 0; i < colliderEntries.length; i++) for (let j = i + 1; j < colliderEntries.length; j++) {
          if (c.physicsHooks.filterContactPair(colliderEntries[i].handle, colliderEntries[j].handle) === null) {
            excludedColliderPairs.push([rawParts(colliderEntries[i].handle), rawParts(colliderEntries[j].handle)]);
          }
        }
        const snapshot = c.world.takeSnapshot(), stem = `${output}.${heading}`;
        writeFileSync(`${stem}.bin`, snapshot, { flag: 'wx' });
        writeFileSync(`${stem}.metadata.json`, JSON.stringify({ heading, snapshotTick, snapshotSha256: sha256(snapshot),
          bodyHandles: Object.fromEntries([...c.ragdollBodies].map(([id, body]) => [id, rawParts(body.handle)])),
          excludedColliderPairs }, null, 2) + '\n', { flag: 'wx' });
      };
      if (snapshotTick === 0) saveSnapshot();
      let start, end;
      const integratedVelocity = Object.fromEntries(ids.map(id => [id, { ...zero }]));
      const samples = [];
      for (let tick = 1; tick <= 720; tick++) {
        c.world.step(c.eventQueue, c.physicsHooks);
        c.eventQueue.clear();
        if (tick === snapshotTick) saveSnapshot();
        const measured = readState();
        if (tick === 120) start = measured;
        if (tick > 120) for (const id of ids) integratedVelocity[id] = add(integratedVelocity[id], scale(measured[id].velocity, dt));
        if (tick >= 120 && tick % 60 === 0) samples.push({ tick, measured });
        end = measured;
      }
      assert.ok(retainedBodyHandles.every(handle => c.world.getRigidBody(handle)?.isDynamic()));
      results.push({ heading, mode, massBefore, massAfter, comBefore, comAfter, retainedBodyHandles, jointCounts, initial, start, end, samples, integratedVelocity,
        metrics: Object.fromEntries(ids.map(id => {
          const displacement = sub(end[id].position, start[id].position);
          const difference = sub(sub(end[id].com, start[id].com), integratedVelocity[id]);
          return [id, { endpointDriftM: Math.hypot(displacement.x, displacement.z),
            endpointVelocityIntegralErrorM: Math.hypot(difference.x, difference.z) }];
        })) });
    } finally { c.dispose(); }
  }
  const sourceAfter = fingerprints(); assert.deepEqual(sourceAfter, sourceBefore);
  writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version, rapierVersion: RAPIER.version(),
    sourceBefore, sourceAfter, sourceFingerprint: sha256(JSON.stringify(sourceBefore)),
    qualification: 'Calibration only: replace the articulated degrees of freedom with native fixed impulse/multibody joints, or combine the same collider geometry and mass into one dynamic body. Initially identical body markers and COM, same floor and world settings. No standing controller is stepped. Fixed multibody retains the 25 bodies but changes constraint representation; compound construction also removes internal collision pairs and changes inertia representation. Neither establishes humanoid acceptance.', results }, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(results.map(({ heading, mode, metrics }) => ({ heading, mode, metrics }))));
} catch (error) {
  writeFileSync(`${output}.failure.json`, JSON.stringify({ generatedAt: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], sourceBefore, sourceAfter: fingerprints(),
    error: String(error), stack: error.stack, completedResults: results }, null, 2) + '\n', { flag: 'wx' });
  throw error;
} finally { unregister(); }
