import assert from 'node:assert/strict';
import { existsSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import { fingerprints } from './capture-physics-baseline.mjs';
const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { add, sub, scale, length } = await import('../src/character/math.ts');
const { jointCoordinates } = await import('../src/character/joint-coordinates.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const output = process.argv[2]; assert.ok(output && !existsSync(output));
const sourceBefore = fingerprints(), dt = 1 / 60, zero = { x: 0, y: 0, z: 0 };
const c = await createEmbodiedCharacter(), samples = [], ids = ['leftHand', 'leftForearm', 'leftUpperArm'];
try {
  for (let tick = 0; tick < 120; tick++) c.fixedUpdate(dt, null);
  const start = { ...c.ragdollBodies.get('leftHand').translation() }, root = { ...c.ragdollBodies.get('pelvis').translation() };
  const torso = { ...c.ragdollBodies.get('torso').translation() }, target = { x: 0.25, y: torso.y, z: torso.z + 0.01 };
  c.fixedUpdate(dt, { kind: 'begin', pointerId: 301, region: 'leftHand', segment: 'leftHand', localAnchor: zero, worldTarget: start, timestampMs: 0 });
  let touched = false, minimumTorsoDistance = Infinity, firstTorsoContact = null, firstRecovery = null;
  for (let tick = 1; tick <= 240; tick++) {
    const amount = Math.min(tick / 120, 1), position = add(start, scale(sub(target, start), amount));
    c.fixedUpdate(dt, { kind: 'move', pointerId: 301, worldTarget: position, timestampMs: tick * 1000 / 60 });
    if (c.isRecoveryState() && !firstRecovery) firstRecovery = { tick, state: c.state };
    const distances = [];
    for (const id of ids) {
      const collider = c.ragdollColliders.get(id), trunk = c.ragdollColliders.get('torso');
      c.world.contactPair(collider, trunk, m => {
        if (m.numContacts()) { touched = true; firstTorsoContact ??= { tick, state: c.state, segment: id }; }
      });
      const contact = collider.contactCollider(trunk, 1);
      if (contact) minimumTorsoDistance = Math.min(minimumTorsoDistance, contact.distance);
      distances.push({ id, torsoDistance: contact?.distance ?? null });
    }
    if (tick === 1 || tick % 20 === 0) {
      const contacts = [];
      for (const id of ids) for (const [otherId, other] of c.ragdollColliders) {
        if (id === otherId) continue;
        c.world.contactPair(c.ragdollColliders.get(id), other, m => {
          if (m.numContacts()) contacts.push({ id, otherId, count: m.numContacts() });
        });
      }
      samples.push(JSON.parse(JSON.stringify({ tick, state: c.state, step: c.step, distances, contacts,
        bodies: Object.fromEntries([...ids, 'torso', 'pelvis'].map(id => [id, c.ragdollBodies.get(id).translation()])),
        actuationPhase: c.isRecoveryState() ? 'recovery' : 'standing',
        arm: c.isRecoveryState() ? [] : ['leftShoulderGirdle', 'leftUpperArm', 'leftForearm', 'leftForearmTwist', 'leftHand'].map(id => {
          const definition = SEGMENT_BY_ID.get(id), command = c.standingCommands.find(item => item.id === id);
          return { id, targetPose: c.standingTargets.get(id), command, motor: c.lastMotorResults.get(id),
            currentCoordinates: jointCoordinates(c.ragdollBodies.get(definition.parent).rotation(),
              c.ragdollBodies.get(id).rotation(), definition.jointProfile),
            targetCoordinates: command ? jointCoordinates({ x: 0, y: 0, z: 0, w: 1 }, command.targetLocalRotation, definition.jointProfile) : null,
            axes: definition.jointProfile.axes };
        }),
        grab: c.grabControlDiagnostics, requested: position })));
    }
  }
  const sourceAfter = fingerprints(); assert.deepEqual(sourceBefore, sourceAfter);
  const result = { generatedAt: new Date().toISOString(), sourceBefore, sourceAfter, touched, firstTorsoContact, firstRecovery, minimumTorsoDistance,
    rootTravelM: length(sub(c.ragdollBodies.get('pelvis').translation(), root)), samples,
    qualification: 'Original left-hand drag fixture with read-only distance, native contact and grab-controller telemetry. No target, limit, mass, solver or assertion changes.' };
  writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ touched, firstTorsoContact, firstRecovery, minimumTorsoDistance,
    rootTravelM: result.rootTravelM, finalState: samples.at(-1).state }));
} finally { c.dispose(); unregister(); }
