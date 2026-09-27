import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const unregister = register();
const { configureHumanoidWorld } = await import('../src/character/physics-settings.ts');
const { add, sub, scale, cross, dot, rotate, quatMultiply, quatFromAxisAngle, worldPoint } = await import('../src/character/math.ts');
const output = process.argv[2];
if (!output || existsSync(output)) throw new Error('Specify a fresh output JSON.');
const sourceBefore = fingerprints(), dt = 1 / 60, results = [];
const zero = { x: 0, y: 0, z: 0 }, identity = { ...zero, w: 1 };
const zAxis = { x: 0, y: 0, z: 1 }, footAnchor = { x: 0, y: .04, z: 0 }, rodAnchor = { x: 0, y: -.4, z: 0 };
await RAPIER.init();
try {
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) for (const lean of [-.1, .1]) {
    for (const mode of ['compound', 'fixed-joint', 'motor-joint']) {
      const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
      configureHumanoidWorld(world);
      try {
        world.createCollider(RAPIER.ColliderDesc.cuboid(10, .1, 10).setTranslation(0, -.1, 0).setFriction(4));
        const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
        const tilt = quatFromAxisAngle(zAxis, lean), rodRotation = quatMultiply(yaw, tilt);
        const footPosition = { x: 0, y: .044, z: 0 };
        const rodLocal = add(footAnchor, rotate(tilt, scale(rodAnchor, -1)));
        const rodPosition = worldPoint(footPosition, yaw, rodLocal);
        const descriptor = (position, rotation) => RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(position.x, position.y, position.z).setRotation(rotation)
          .setCanSleep(false).setAngularDamping(.52).setAdditionalSolverIterations(5);
        const foot = world.createRigidBody(descriptor(footPosition, yaw));
        world.createCollider(RAPIER.ColliderDesc.cuboid(.1, .04, .14).setMass(.8).setFriction(4).setContactSkin(.004), foot);
        let rod, joint;
        if (mode === 'compound') {
          world.createCollider(RAPIER.ColliderDesc.cuboid(.05, .4, .05).setMass(10)
            .setTranslation(rodLocal.x, rodLocal.y, rodLocal.z).setRotation(tilt), foot);
        } else {
          rod = world.createRigidBody(descriptor(rodPosition, rodRotation));
          world.createCollider(RAPIER.ColliderDesc.cuboid(.05, .4, .05).setMass(10), rod);
          const data = mode === 'fixed-joint'
            ? RAPIER.JointData.fixed(footAnchor, tilt, rodAnchor, identity)
            : RAPIER.JointData.revolute(footAnchor, rodAnchor, zAxis);
          joint = world.createImpulseJoint(data, foot, rod, true);
          joint.setContactsEnabled(false);
          if (mode === 'motor-joint') {
            joint.configureMotorModel(RAPIER.MotorModel.ForceBased);
            joint.setMotorMaxForce(160);
          }
        }
        let start, integratedVelocity = { ...zero }, maxHorizontalExcursionM = 0, maxSpeedMps = 0;
        const samples = [];
        for (let tick = 1; tick <= 720; tick++) {
          if (mode === 'motor-joint') {
            const anchor = worldPoint(foot.translation(), foot.rotation(), footAnchor);
            const compensation = cross(sub(rod.worldCom(), anchor), { x: 0, y: rod.mass() * 9.81, z: 0 });
            const feedforward = dot(compensation, rotate(foot.rotation(), zAxis));
            joint.configureMotor(lean, feedforward / 30, 180, 30);
          }
          world.step();
          const position = foot.translation(), com = foot.worldCom(), velocity = foot.linvel();
          if (tick === 120) start = { position, com, velocity };
          if (tick > 120) {
            integratedVelocity = add(integratedVelocity, scale(velocity, dt));
            maxHorizontalExcursionM = Math.max(maxHorizontalExcursionM, Math.hypot(position.x - start.position.x, position.z - start.position.z));
            maxSpeedMps = Math.max(maxSpeedMps, Math.hypot(velocity.x, velocity.y, velocity.z));
          }
          if (tick >= 120 && tick % 60 === 0) samples.push({ tick, position, com, velocity, rotation: foot.rotation() });
        }
        const end = samples.at(-1), displacement = sub(end.com, start.com);
        results.push({ heading, lean, mode, massKg: .8 + 10, start, end, samples, integratedVelocity,
          endpointDriftM: Math.hypot(end.position.x - start.position.x, end.position.z - start.position.z),
          endpointVelocityIntegralErrorM: Math.hypot(displacement.x - integratedVelocity.x, displacement.z - integratedVelocity.z),
          maxHorizontalExcursionM, maxSpeedMps });
      } finally { world.free(); }
    }
  }
  const sourceAfter = fingerprints();
  assert.deepEqual(sourceAfter, sourceBefore, 'Source changed during diagnostic');
  writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version, rapierVersion: RAPIER.version(),
    sourceBefore, sourceAfter, sourceFingerprint: sha256(JSON.stringify(sourceBefore)),
    qualification: 'Independent rigid compound / fixed joint / finite force-based motor comparison, with unchanged humanoid world settings. This is not a humanoid acceptance run or a proof of its cause.', results }, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(results.map(({ heading, lean, mode, endpointDriftM, endpointVelocityIntegralErrorM, maxSpeedMps }) =>
    ({ heading, lean, mode, endpointDriftM, endpointVelocityIntegralErrorM, maxSpeedMps }))));
} finally { unregister(); }
