import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';
const unregister = register();
after(unregister);
const { createEmbodiedCharacter } = await import('../src/character/EmbodiedCharacter.ts');
const { SEGMENTS, SEGMENT_BY_ID, TOTAL_MASS_KG } = await import('../src/core/humanoid.ts');
const { massState } = await import('../src/character/BalanceController.ts');
const { recoveryMassState } = await import('../src/character/recovery-support.ts');
const { jointRotationFromCoordinates } = await import('../src/character/joint-coordinates.ts');
const { dot, length, rotate, sub, worldPoint } = await import('../src/character/math.ts');

const FORWARD = { x: 0, y: 0, z: 1 };
const DOWN = { x: 0, y: -1, z: 0 };
const dt = 1 / 60;
const armRoles = new Set(['upper-arm', 'forearm', 'forearm-twist', 'hand']);
const trunkIds = ['pelvis', 'lumbar', 'torso'];

function measuredMass(character) {
  let massKg = 0;
  const position = { x: 0, y: 0, z: 0 }, velocity = { x: 0, y: 0, z: 0 };
  for (const body of character.ragdollBodies.values()) {
    const mass = body.mass(), center = body.worldCom(), speed = body.linvel();
    massKg += mass;
    for (const axis of ['x', 'y', 'z']) {
      position[axis] += mass * center[axis];
      velocity[axis] += mass * speed[axis];
    }
  }
  for (const axis of ['x', 'y', 'z']) { position[axis] /= massKg; velocity[axis] /= massKg; }
  return { position, velocity, massKg };
}

test('anatomical elbow flexion moves the forearm forward while knee flexion moves the shin backward', () => {
  for (const side of ['left', 'right']) {
    for (const [suffix, sign] of [['Forearm', 1], ['Shin', -1]]) {
      const definition = SEGMENT_BY_ID.get(`${side}${suffix}`);
      const rotated = rotate(jointRotationFromCoordinates({ x: Math.PI / 4, y: 0, z: 0 }, definition.jointProfile), DOWN);
      assert.ok(sign * rotated.z > 0.7, `${definition.id}: incorrect flexion direction ${rotated.z}`);
    }
  }
});

test('every nonadjacent arm segment collides with the trunk and starts outside it', async () => {
  const character = await createEmbodiedCharacter('canvas2d');
  try {
    for (const arm of SEGMENTS.filter(def => armRoles.has(def.role))) for (const trunkId of trunkIds) {
      const a = character.ragdollColliders.get(arm.id), b = character.ragdollColliders.get(trunkId);
      assert.notEqual(character.physicsHooks.filterContactPair(a.handle, b.handle), null,
        `${arm.id}/${trunkId}: legitimate self-collision disabled`);
      const contact = a.contactCollider(b, 0);
      assert.ok((contact?.distance ?? 0) >= -0.0015,
        `${arm.id}/${trunkId}: initial penetration ${-(contact?.distance ?? 0)}m`);
    }
  } finally { character.dispose(); }
});

test('neutral elbows and toes agree on forward direction at rotated headings', async () => {
  for (const heading of [0, Math.PI / 2, -Math.PI / 2, Math.PI]) {
    const character = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      const poses = new Map(character.getSnapshot('canvas2d').segments.map(p => [p.id, p]));
      const forward = rotate(poses.get('torso').rotation, FORWARD);
      for (const side of ['left', 'right']) {
        const upper = poses.get(`${side}UpperArm`), hand = poses.get(`${side}Hand`);
        const elbow = worldPoint(upper.position, upper.rotation, SEGMENT_BY_ID.get(`${side}Forearm`).jointAnchorParent);
        assert.ok(dot(sub(hand.position, elbow), forward) > 0.025, `${side}: hand bends behind elbow at heading ${heading}`);
        assert.ok(dot(rotate(upper.rotation, FORWARD), forward) > 0.75, `${side}: upper-arm frame reverses heading`);
        assert.ok(dot(sub(poses.get(`${side}Forefoot`).position, poses.get(`${side}Foot`).position), forward) > 0.10,
          `${side}: toes point away from body heading`);
      }
    } finally { character.dispose(); }
  }
});

test('balance and recovery mass centres match Rapier rather than mesh origins', async () => {
  for (const heading of [0, 1.2, -2.0]) {
    const character = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      const poses = new Map(character.getSnapshot('canvas2d').segments.map(p => [p.id, p]));
      const measured = measuredMass(character);
      assert.ok(Math.abs(measured.massKg - TOTAL_MASS_KG) < 2e-5);
      for (const estimate of [massState(poses), recoveryMassState(poses.values())]) {
        assert.ok(length(sub(estimate.position, measured.position)) < 2e-6,
          `mass-centre error ${length(sub(estimate.position, measured.position))}m`);
        assert.ok(length(sub(estimate.velocity, measured.velocity)) < 2e-6);
      }
    } finally { character.dispose(); }
  }
});

test('mass-centre integration is translation invariant and rejects zero-volume surfaces', async () => {
  const { createConvexGeometry, convexVolumeCentroid } = await import('../src/core/geometry.ts');
  const vertices = [{ x: 0, y: 0, z: 0 }, { x: 2, y: 0, z: 0 }, { x: 0, y: 4, z: 0 }, { x: 0, y: 0, z: 6 }];
  const triangles = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
  for (const offset of [{ x: 0, y: 0, z: 0 }, { x: 10, y: -20, z: 30 }]) {
    const moved = vertices.map(v => ({ x: v.x + offset.x, y: v.y + offset.y, z: v.z + offset.z }));
    for (const faces of [triangles, triangles.map(([a, b, c]) => [a, c, b])]) {
      const centre = convexVolumeCentroid(createConvexGeometry(moved, faces));
      assert.ok(length(sub(centre, { x: .5 + offset.x, y: 1 + offset.y, z: 1.5 + offset.z })) < 1e-12);
    }
  }
  assert.throws(() => convexVolumeCentroid(createConvexGeometry(vertices.map(v => ({ ...v, z: 0 })), triangles)), /volume/i);
});

test('native elbow limits enforce the anatomical frame under torque at rotated headings', async () => {
  const { default: RAPIER } = await import('@dimforge/rapier3d-compat');
  const { createRapierJointLimitAdapter } = await import('../src/character/rapier-joint-adapter.ts');
  const { jointCoordinates } = await import('../src/character/joint-coordinates.ts');
  const { quatFromAxisAngle } = await import('../src/character/math.ts');
  const { flattenGeometryVertices, flattenGeometryIndices } = await import('../src/core/geometry.ts');
  await RAPIER.init();
  const zero = { x: 0, y: 0, z: 0 };
  for (const side of ['left', 'right']) for (const heading of [0, 1.1, -1.7]) {
    const world = new RAPIER.World(zero);
    try {
      world.timestep = 1 / 120;
      world.numSolverIterations = 20;
      const definition = SEGMENT_BY_ID.get(`${side}Forearm`), profile = definition.jointProfile;
      const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
      const localAxis = rotate(profile.parentFrame.rotation, { x: 1, y: 0, z: 0 });
      const axis = rotate(yaw, localAxis), anchor = profile.childFrame.anchor;
      const at = rotate(yaw, { x: -anchor.x, y: -anchor.y, z: -anchor.z });
      const parent = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setRotation(yaw));
      const child = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
        .setRotation(yaw).setTranslation(at.x, at.y, at.z).setCanSleep(false).setAngularDamping(0.2));
      world.createCollider(RAPIER.ColliderDesc.convexMesh(flattenGeometryVertices(definition.geometry), flattenGeometryIndices(definition.geometry))
        .setMass(definition.massKg), child);
      const joint = world.createImpulseJoint(RAPIER.JointData.revoluteWithAxes(zero, anchor, localAxis, localAxis), parent, child, true);
      joint.setLocalFrame1(zero, profile.parentFrame.rotation);
      joint.setLocalFrame2(anchor, profile.childFrame.rotation);
      createRapierJointLimitAdapter(world).constrain(joint, profile);
      for (const sign of [1, -1]) {
        child.resetTorques(true);
        child.addTorque({ x: axis.x * .08 * sign, y: axis.y * .08 * sign, z: axis.z * .08 * sign }, true);
        for (let step = 0; step < 360; step++) world.step();
        const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
        const expected = sign > 0 ? profile.axes[0].maxRadians : profile.axes[0].minRadians;
        assert.ok(Math.abs(coordinates.x - expected) < .005, `${side}/${heading}: native elbow limit ${coordinates.x} vs ${expected}`);
        assert.ok(Math.hypot(coordinates.y, coordinates.z) < .001, `${side}/${heading}: locked axes drift`);
      }
    } finally { world.free(); }
  }
});

test('inward hand drags produce real trunk contact without passing through it at rotated headings', async t => {
  const { quatFromAxisAngle, add } = await import('../src/character/math.ts');
  const metrics = [];
  for (const side of ['left', 'right']) for (const heading of [0, 1.1]) {
    const character = await createEmbodiedCharacter('canvas2d', { heading });
    try {
      const id = `${side}Hand`, sign = side === 'left' ? 1 : -1;
      const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
      const start = character.getSnapshot('canvas2d').segments.find(p => p.id === id).position;
      const originalBodies = new Map(character.ragdollBodies);
      const pairs = SEGMENTS.filter(d => armRoles.has(d.role)).flatMap(arm => trunkIds.map(trunk => [arm.id, trunk]));
      let maxPenetrationM = 0, contactImpulseNs = 0, contactTicks = 0, maxMassCentreErrorM = 0, maxJointGapM = 0;
      character.fixedUpdate(dt, { kind: 'begin', pointerId: 60, region: id, segment: id,
        localAnchor: { x: 0, y: 0, z: 0 }, worldTarget: start, timestampMs: 0 });
      for (let tick = 1; tick <= 180; tick++) {
        const fraction = Math.min(1, tick / 120);
        character.fixedUpdate(dt, tick <= 120 ? { kind: 'move', pointerId: 60,
          worldTarget: add(start, rotate(yaw, { x: sign * .55 * fraction, y: .38 * fraction, z: .02 * fraction })),
          timestampMs: tick * dt * 1000 } : tick === 121 ? { kind: 'end', pointerId: 60, timestampMs: tick * dt * 1000 } : null);
        const snapshot = character.getSnapshot('canvas2d'), diagnostics = snapshot.diagnostics;
        assert.equal(diagnostics.physicsOwnership, 'rapier-dynamic');
        assert.equal(diagnostics.finite, true);
        assert.deepEqual(diagnostics.errors, []);
        maxJointGapM = Math.max(maxJointGapM, diagnostics.maxJointSeparationM);
        assert.ok(diagnostics.maxJointSeparationM <= .08);
        assert.ok(diagnostics.maxFloorPenetrationM <= .08);
        if (diagnostics.balance && diagnostics.grabControl.active) {
          assert.ok(length(sub(diagnostics.balance.externalForce, diagnostics.grabControl.force)) < 1e-9,
            'balance reacted to a force different from the applied grab impulse');
        }
        for (const pose of snapshot.segments) {
          const body = originalBodies.get(pose.id);
          assert.equal(character.ragdollBodies.get(pose.id), body);
          assert.ok(body.isDynamic());
          assert.deepEqual({ ...body.translation() }, pose.position);
          assert.deepEqual({ ...body.rotation() }, pose.rotation);
        }
        const measured = measuredMass(character);
        for (const estimate of [massState(new Map(snapshot.segments.map(p => [p.id, p]))), recoveryMassState(snapshot.segments)]) {
          const error = length(sub(estimate.position, measured.position));
          maxMassCentreErrorM = Math.max(error, maxMassCentreErrorM);
          assert.ok(error < 2e-6);
          assert.ok(length(sub(estimate.velocity, measured.velocity)) < 2e-6);
        }
        let contacted = false;
        for (const [arm, trunk] of pairs) {
          const a = character.ragdollColliders.get(arm), b = character.ragdollColliders.get(trunk);
          assert.notEqual(character.physicsHooks.filterContactPair(a.handle, b.handle), null);
          const penetration = Math.max(0, -(a.contactCollider(b, 0)?.distance ?? 0));
          maxPenetrationM = Math.max(maxPenetrationM, penetration);
          assert.ok(penetration <= .015, `${side}/${heading}/${tick}: ${arm}/${trunk} penetration ${penetration}m`);
          character.world.contactPair(a, b, manifold => {
            for (let point = 0; point < manifold.numContacts(); point++) {
              const impulse = Math.max(0, manifold.contactImpulse(point));
              contactImpulseNs += impulse;
              contacted ||= impulse > 1e-6;
            }
          });
        }
        if (contacted) contactTicks++;
      }
      assert.ok(contactTicks >= 3, `${side}/${heading}: no sustained physical arm/trunk collision was exercised`);
      assert.ok(contactImpulseNs > .05, `${side}/${heading}: no meaningful contact impulse`);
      metrics.push({ side, heading, contactTicks, contactImpulseNs, maxPenetrationM, maxMassCentreErrorM, maxJointGapM });
    } finally { character.dispose(); }
  }
  t.diagnostic(JSON.stringify(metrics));
});


test('shoulder abduction opens away from the trunk on both sides', () => {
  for (const side of ['left', 'right']) {
    const sign = side === 'left' ? -1 : 1;
    const profile = SEGMENT_BY_ID.get(`${side}UpperArm`).jointProfile;
    const angle = sign * Math.PI / 3;
    const axis = profile.axes.find(a => a.coordinate === 'z');
    assert.ok(angle >= axis.minRadians && angle <= axis.maxRadians, `${side}: outward reach is incorrectly limited`);
    const arm = rotate(jointRotationFromCoordinates({ x: 0, y: 0, z: angle }, profile), DOWN);
    assert.ok(sign * arm.x > .85, `${side}: abduction points into the trunk`);
  }
});
