import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
const unregister = register();
const { configureHumanoidWorld } = await import('../src/character/physics-settings.ts');
const output = process.argv[2];
if (!output || existsSync(output)) throw new Error('Specify a fresh output JSON.');
const source = fingerprints(), dt = 1 / 60;
await RAPIER.init();
const results = [];
try {
  for (const forceN of [0, 5]) {
    const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    configureHumanoidWorld(world);
    try {
      const floor = world.createCollider(RAPIER.ColliderDesc.cuboid(10, .1, 10).setTranslation(0, -.1, 0).setFriction(4));
      // Independent API-calibration fixture, never a humanoid acceptance run.
      const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, .2, 0)
        .enabledRotations(false, false, false).setCanSleep(false));
      const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(.1, .1, .1).setMass(1).setFriction(4), body);
      for (let tick = 0; tick < 120; tick++) world.step();
      const start = { position: body.translation(), velocity: body.linvel() };
      body.addForce({ x: forceN, y: 0, z: 0 }, true);
      let normalImpulse = 0, reportedTangentMagnitudeSum = 0;
      for (let tick = 0; tick < 60; tick++) {
        world.step();
        world.contactPair(collider, floor, manifold => {
          for (let point = 0; point < manifold.numContacts(); point++) {
            normalImpulse += manifold.contactImpulse(point);
            reportedTangentMagnitudeSum += Math.hypot(manifold.contactTangentImpulseX(point), manifold.contactTangentImpulseY(point));
          }
        });
      }
      const end = { position: body.translation(), velocity: body.linvel() };
      results.push({ forceN, start, end, displacementX: end.position.x - start.position.x,
        normalImpulse, reportedTangentMagnitudeSum,
        inferredHorizontalContactImpulse: body.mass() * (end.velocity.x - start.velocity.x) - forceN * 60 * dt });
    } finally { world.free(); }
  }
  assert.deepEqual(fingerprints(), source);
  const result = { generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
    nodeVersion: process.version, rapierVersion: RAPIER.version(), source, sourceFingerprint: sha256(JSON.stringify(source)),
    qualification: 'Isolated 1 kg translating-box API calibration. Contact impulse inferred from horizontal momentum balance, with no damping or other horizontal force.', results };
  writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(results));
} finally { unregister(); }
