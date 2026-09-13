import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { articulatedCoordinateResponse } = await import("../src/character/articulated-inertia.ts");
const {
  jointCoordinateKinematics,
  jointCoordinates,
  jointFrameAxesWorld,
  jointRotationFromCoordinates,
} = await import("../src/character/joint-coordinates.ts");
const {
  applyCoupledJointMotors,
  applyPassiveJointResistance,
} = await import("../src/character/joint-motors.ts");
const { seedRecoveryFixture } = await import("../scripts/recovery-fixtures.ts");
const {
  add,
  dot,
  quatFromAxisAngle,
  quatMultiply,
  rotate,
  scale,
} = await import("../src/character/math.ts");

const zero = { x: 0, y: 0, z: 0 };
const close = (actual, expected, tolerance = 1e-8) => assert.ok(
  Math.abs(actual - expected) < tolerance,
  `${actual} differs from ${expected}`,
);
const closeVector = (actual, expected, tolerance = 1e-8) => {
  close(actual.x, expected.x, tolerance);
  close(actual.y, expected.y, tolerance);
  close(actual.z, expected.z, tolerance);
};

function fakeBody(rotation, inverseInertia = 1) {
  let impulse = { ...zero };
  return {
    rotation: () => rotation,
    angvel: () => zero,
    effectiveWorldInvInertia: () => ({
      m11: inverseInertia, m12: 0, m13: 0,
      m22: inverseInertia, m23: 0, m33: inverseInertia,
    }),
    isSleeping: () => false,
    applyTorqueImpulse: (value) => { impulse = add(impulse, value); },
    impulse: () => impulse,
  };
}

function runIsolated(id, current, target, heading) {
  const definition = SEGMENT_BY_ID.get(id);
  const profile = definition.jointProfile;
  const headingRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  const parent = fakeBody(headingRotation, 0.7);
  const child = fakeBody(quatMultiply(
    headingRotation,
    jointRotationFromCoordinates(current, profile),
  ), 1.3);
  const bodies = new Map([[definition.parent, parent], [id, child]]);
  const result = applyCoupledJointMotors(bodies, [{
    id,
    targetLocalRotation: jointRotationFromCoordinates(target, profile),
    stiffness: 240,
    damping: 12,
    strengthScale: 1,
  }], 1 / 60).get(id);
  closeVector(child.impulse(), {
    x: result.torqueWorld.x / 60,
    y: result.torqueWorld.y / 60,
    z: result.torqueWorld.z / 60,
  });
  closeVector(parent.impulse(), {
    x: -result.torqueWorld.x / 60,
    y: -result.torqueWorld.y / 60,
    z: -result.torqueWorld.z / 60,
  });
  return result;
}

test("an elbow hinge produces only its permitted equal-and-opposite torque at every heading", () => {
  let baseline;
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const result = runIsolated(
      "leftForearm",
      { x: 0.2, y: 0, z: 0 },
      { x: 0.8, y: 0, z: 0 },
      heading,
    );
    const profile = SEGMENT_BY_ID.get("leftForearm").jointProfile;
    const headingRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const basis = jointFrameAxesWorld(headingRotation, profile);
    assert.ok(dot(result.torqueWorld, basis.x) > 0);
    close(dot(result.torqueWorld, basis.y), 0);
    close(dot(result.torqueWorld, basis.z), 0);
    if (!baseline) baseline = result.torqueWorld;
    else closeVector(result.torqueWorld, rotate(headingRotation, baseline), 1e-6);
  }
});

test("a combined hip maps generalized torque through its non-unit conjugate axis", () => {
  const definition = SEGMENT_BY_ID.get("rightThigh");
  const profile = definition.jointProfile;
  const current = { x: -0.096, y: -0.68, z: 0.342 };
  const target = { ...current, y: -0.579 };
  const generalizedTorque = 240 * (target.y - current.y);
  let baseline;
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const parentRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const childRotation = quatMultiply(
      parentRotation,
      jointRotationFromCoordinates(current, profile),
    );
    const parent = fakeBody(parentRotation, 0);
    const child = fakeBody(childRotation, 0);
    const bodies = new Map([[definition.parent, parent], [definition.id, child]]);
    const result = applyCoupledJointMotors(bodies, [{
      id: definition.id,
      targetLocalRotation: jointRotationFromCoordinates(target, profile),
      stiffness: 240,
      damping: 12,
      strengthScale: 1,
    }], 1 / 60).get(definition.id);
    const conjugate = jointCoordinateKinematics(
      zero, zero, parentRotation, current, profile,
    ).torqueAxesWorld.y;
    closeVector(result.torqueWorld, {
      x: conjugate.x * generalizedTorque,
      y: conjugate.y * generalizedTorque,
      z: conjugate.z * generalizedTorque,
    }, 1e-6);
    assert.ok(Math.abs(dot(conjugate, jointFrameAxesWorld(parentRotation, profile).x)) > 0.1);
    if (!baseline) baseline = result.torqueWorld;
    else closeVector(result.torqueWorld, rotate(parentRotation, baseline), 1e-6);
  }
});

test("a multi-axis hip retains each permitted coordinate and rotates equivariantly", () => {
  const target = { x: 0.18, y: 0.12, z: -0.09 };
  let baseline;
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const result = runIsolated("leftThigh", zero, target, heading);
    const profile = SEGMENT_BY_ID.get("leftThigh").jointProfile;
    const headingRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const basis = jointFrameAxesWorld(headingRotation, profile);
    for (const axis of profile.axes) {
      assert.equal(Math.sign(dot(result.torqueWorld, basis[axis.coordinate])), Math.sign(target[axis.coordinate]));
    }
    if (!baseline) baseline = result.torqueWorld;
    else closeVector(result.torqueWorld, rotate(headingRotation, baseline), 1e-6);
  }
});

test("combined oblique-axis torque stays inside the documented aggregate cap", () => {
  const definition = SEGMENT_BY_ID.get("rightThigh");
  const profile = definition.jointProfile;
  const current = { x: -0.3, y: -0.6, z: 0.55 };
  const target = { x: 1.8, y: 0.7, z: -0.75 };
  const parent = fakeBody({ x: 0, y: 0, z: 0, w: 1 }, 0);
  const child = fakeBody(jointRotationFromCoordinates(current, profile), 0);
  const result = applyCoupledJointMotors(
    new Map([[definition.parent, parent], [definition.id, child]]),
    [{
      id: definition.id,
      targetLocalRotation: jointRotationFromCoordinates(target, profile),
      stiffness: 1e5,
      damping: 0,
      strengthScale: 1,
    }],
    1 / 60,
  ).get(definition.id);
  const aggregateCap = Math.hypot(...profile.axes.map((axis) => axis.maxMotorTorqueNm));
  assert.ok(Math.hypot(
    result.torqueWorld.x, result.torqueWorld.y, result.torqueWorld.z,
  ) <= aggregateCap + 1e-8);
  closeVector(child.impulse(), scale(result.torqueWorld, 1 / 60));
  closeVector(parent.impulse(), scale(result.torqueWorld, -1 / 60));
});

test("passive soft-limit resistance remains active with posture strength disabled", () => {
  const definition = SEGMENT_BY_ID.get("leftForearm");
  const profile = definition.jointProfile;
  const axisProfile = profile.axes[0];
  const zone = (axisProfile.maxRadians - axisProfile.minRadians) * profile.limitSoftZoneFraction;
  const current = { x: axisProfile.maxRadians - zone * 0.25, y: 0, z: 0 };
  const parent = fakeBody({ x: 0, y: 0, z: 0, w: 1 }, 0.7);
  const child = fakeBody(jointRotationFromCoordinates(current, profile), 1.3);
  const bodies = new Map([[definition.parent, parent], [definition.id, child]]);
  const command = {
    id: definition.id,
    targetLocalRotation: jointRotationFromCoordinates(current, profile),
    stiffness: 0,
    damping: 0,
    strengthScale: 0,
  };
  const inactive = applyCoupledJointMotors(bodies, [command], 1 / 60).get(definition.id);
  closeVector(inactive.torqueWorld, zero);
  const resisting = applyCoupledJointMotors(
    bodies, [command], 1 / 60, { passiveResistance: true },
  ).get(definition.id);
  const torqueAxis = jointCoordinateKinematics(
    zero, zero, { x: 0, y: 0, z: 0, w: 1 }, current, profile,
  ).torqueAxesWorld.x;
  assert.ok(dot(resisting.torqueWorld, torqueAxis) < 0, "upper-limit resistance must oppose flexion");
  closeVector(child.impulse(), scale(resisting.torqueWorld, 1 / 60));
  closeVector(parent.impulse(), scale(resisting.torqueWorld, -1 / 60));
});

test("articulated inertia predicts Rapier's post-constraint ankle response", async () => {
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const character = await createEmbodiedCharacter("canvas2d", { heading });
    try {
      character.floorCollider.setEnabled(false);
      character.world.gravity = zero;
      for (const collider of character.ragdollColliders.values()) collider.setCollisionGroups(0);
      for (const body of character.ragdollBodies.values()) body.wakeUp();

      const definition = SEGMENT_BY_ID.get("rightAnkle");
      const parent = character.ragdollBodies.get(definition.parent);
      const child = character.ragdollBodies.get(definition.id);
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile);
      const axis = jointCoordinateKinematics(
        parent.angvel(), child.angvel(), parent.rotation(), coordinates, definition.jointProfile,
      ).torqueAxesWorld.x;
      const predicted = articulatedCoordinateResponse(character.ragdollBodies);

      const impulse = 0.001;
      child.applyTorqueImpulse(scale(axis, impulse), true);
      parent.applyTorqueImpulse(scale(axis, -impulse), true);
      character.world.step(character.eventQueue, character.physicsHooks);

      for (const [id, coordinate] of [
        ["rightThigh", "x"],
        ["rightShin", "x"],
        ["rightAnkle", "x"],
        ["rightFoot", "z"],
        ["rightForefoot", "x"],
      ]) {
        const targetDefinition = SEGMENT_BY_ID.get(id);
        const targetParent = character.ragdollBodies.get(targetDefinition.parent);
        const targetChild = character.ragdollBodies.get(id);
        const targetCoordinates = jointCoordinates(
          targetParent.rotation(), targetChild.rotation(), targetDefinition.jointProfile,
        );
        const actual = jointCoordinateKinematics(
          targetParent.angvel(), targetChild.angvel(), targetParent.rotation(),
          targetCoordinates, targetDefinition.jointProfile,
        ).rates[coordinate] / impulse;
        const expected = predicted.get(id, coordinate, "rightAnkle", "x");
        const tolerance = Math.max(0.15, Math.abs(actual) * 0.15);
        close(expected, actual, tolerance);
      }

      const copyInertia = (matrix) => ({
        m11: matrix.m11, m12: matrix.m12, m13: matrix.m13,
        m22: matrix.m22, m23: matrix.m23, m33: matrix.m33,
      });
      const parentInverse = copyInertia(parent.effectiveWorldInvInertia());
      const childInverse = copyInertia(child.effectiveWorldInvInertia());
      const along = (matrix) => dot(axis, {
        x: matrix.m11 * axis.x + matrix.m12 * axis.y + matrix.m13 * axis.z,
        y: matrix.m12 * axis.x + matrix.m22 * axis.y + matrix.m23 * axis.z,
        z: matrix.m13 * axis.x + matrix.m23 * axis.y + matrix.m33 * axis.z,
      });
      assert.ok(
        (along(parentInverse) + along(childInverse)) / Math.abs(predicted.get(
          "rightAnkle", "x", "rightAnkle", "x",
        )) > 20,
        "the regression fixture must distinguish free-body from constrained articulated inertia",
      );
    } finally {
      character.dispose();
    }
  }
});

test("support-conditioned response predicts swing hip and ankle impulses", async () => {
  for (const sourceId of ["rightThigh", "rightAnkle"]) {
    const driven = await createEmbodiedCharacter("canvas2d");
    const control = await createEmbodiedCharacter("canvas2d");
    try {
      const leftContact = driven.lastContacts.find((contact) => contact.segment === "leftFoot");
      assert.ok(leftContact, "standing fixture must expose a measured left sole contact");
      const supports = [{
        segment: "leftFoot",
        points: [leftContact.point],
        directions: [{ x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }],
      }];
      for (const character of [driven, control]) {
        character.ragdollColliders.get("rightFoot").setCollisionGroups(0);
        character.ragdollColliders.get("rightForefoot").setCollisionGroups(0);
        for (const body of character.ragdollBodies.values()) body.wakeUp();
        // Clear the disabled swing-sole manifolds before measuring response.
        character.world.step(character.eventQueue, character.physicsHooks);
      }

      const definition = SEGMENT_BY_ID.get(sourceId);
      const parent = driven.ragdollBodies.get(definition.parent);
      const child = driven.ragdollBodies.get(sourceId);
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile);
      const axis = jointCoordinateKinematics(
        parent.angvel(), child.angvel(), parent.rotation(), coordinates, definition.jointProfile,
      ).torqueAxesWorld.x;
      const predicted = articulatedCoordinateResponse(driven.ragdollBodies, supports).get(
        sourceId, "x", sourceId, "x",
      );
      const direction = sourceId === "rightAnkle" ? -1 : 1;
      const impulse = 0.001 * direction;
      child.applyTorqueImpulse(scale(axis, impulse), true);
      parent.applyTorqueImpulse(scale(axis, -impulse), true);
      driven.world.step(driven.eventQueue, driven.physicsHooks);
      control.world.step(control.eventQueue, control.physicsHooks);

      const rate = (character) => {
        const targetParent = character.ragdollBodies.get(definition.parent);
        const targetChild = character.ragdollBodies.get(sourceId);
        const targetCoordinates = jointCoordinates(
          targetParent.rotation(), targetChild.rotation(), definition.jointProfile,
        );
        return jointCoordinateKinematics(
          targetParent.angvel(), targetChild.angvel(), targetParent.rotation(),
          targetCoordinates, definition.jointProfile,
        ).rates.x;
      };
      const actual = (rate(driven) - rate(control)) / impulse;
      close(predicted, actual, Math.abs(actual) * 0.1);
    } finally {
      driven.dispose();
      control.dispose();
    }
  }
});

test("simultaneous passive resistance does not amplify a seeded recovery chain", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    seedRecoveryFixture(character, {
      id: "passive-half-kneel-regression",
      pose: "half-kneel",
      side: "left",
      heading: Math.PI / 3,
    });
    // Isolate internal passive mechanics from gravity, ground impulses, and
    // self-contact. Collider mass properties remain installed.
    character.floorCollider.setEnabled(false);
    character.world.gravity = zero;
    for (const collider of character.ragdollColliders.values()) collider.setCollisionGroups(0);

    const maximumAngularSpeed = () => Math.max(...[...character.ragdollBodies.values()].map((body) => {
      const velocity = body.angvel();
      return Math.hypot(velocity.x, velocity.y, velocity.z);
    }));
    let peak = maximumAngularSpeed();
    for (let frame = 0; frame < 120; frame++) {
      applyPassiveJointResistance(character.ragdollBodies, 1 / 60);
      peak = Math.max(peak, maximumAngularSpeed());
      character.world.step(character.eventQueue, character.physicsHooks);
      peak = Math.max(peak, maximumAngularSpeed());
    }
    assert.ok(peak < 2, `passive chain amplified angular speed to ${peak} rad/s`);
    assert.ok(maximumAngularSpeed() < 0.1, "passive chain failed to settle");
  } finally {
    character.dispose();
  }
});
