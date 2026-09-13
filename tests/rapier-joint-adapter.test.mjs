import assert from "node:assert/strict";
import test, { after, before } from "node:test";
import RAPIER from "@dimforge/rapier3d-compat";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);
before(async () => { await RAPIER.init(); });

const {
  EXPECTED_RAPIER_VERSION,
  RapierJointLimitAdapter,
  assertRapierJointLimitCompatibility,
  constrainRapierJoint,
  createRapierJointLimitAdapter,
} = await import("../src/character/rapier-joint-adapter.ts");
const {
  jointCoordinates,
  jointFrameAxesWorld,
  jointRotationFromCoordinates,
} = await import("../src/character/joint-coordinates.ts");
const {
  add,
  quatFromAxisAngle,
  quatMultiply,
  rotate,
  scale,
} = await import("../src/character/math.ts");

const identity = { x: 0, y: 0, z: 0, w: 1 };
const zero = { x: 0, y: 0, z: 0 };
const frame = (rotation = identity) => ({ anchor: zero, rotation });
const axis = (coordinate, minRadians, maxRadians) => ({
  coordinate,
  minRadians,
  maxRadians,
  passiveStiffnessNmPerRad: 20,
  dampingNmsPerRad: 2,
  maxMotorTorqueNm: 80,
});
const profile = (kind, axes, parentFrame = frame(), childFrame = frame()) => ({
  kind,
  parentFrame,
  childFrame,
  axes,
  limitSoftZoneFraction: 0.15,
});

function close(actual, expected, tolerance = 2e-3) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} differs from ${expected}`);
}

function createBodies(world, parentRotation, childRotation) {
  const parent = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setRotation(parentRotation),
  );
  const child = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setRotation(childRotation)
      .setAngularDamping(0)
      .setCanSleep(false)
      .setAdditionalSolverIterations(8),
  );
  world.createCollider(RAPIER.ColliderDesc.ball(0.12).setDensity(500), child);
  child.recomputeMassPropertiesFromColliders();
  return { parent, child };
}

function createWorld() {
  const world = new RAPIER.World(zero);
  world.integrationParameters.dt = 1 / 120;
  world.integrationParameters.numSolverIterations = 12;
  return world;
}

function drive(world, body, torque, steps = 600) {
  // Rapier retains user torques until resetTorques(); add this once so the
  // load is constant instead of growing every step.
  body.addTorque(torque, true);
  for (let step = 0; step < steps; step += 1) {
    world.step();
  }
}

test("adapter pins the installed raw ABI and validates multi-axis readback", () => {
  assert.equal(RAPIER.version(), EXPECTED_RAPIER_VERSION);
  assert.doesNotThrow(assertRapierJointLimitCompatibility);
  const world = createWorld();
  try {
    const { parent, child } = createBodies(world, identity, identity);
    const joint = world.createImpulseJoint(
      RAPIER.JointData.spherical(zero, zero),
      parent,
      child,
      true,
    );
    // In 0.20.0 the high-level factory yields a generic wrapper. This is why
    // all multi-axis limits are deliberately isolated behind the raw adapter.
    assert.equal(joint.type(), RAPIER.JointType.Generic);
    assert.equal(typeof joint.configureMotor, "undefined");

    const limits = profile("multi-axis", [
      axis("x", -0.2, 0.5),
      axis("y", -0.1, 0.4),
      axis("z", -0.3, 0.3),
    ]);
    const adapter = createRapierJointLimitAdapter(world);
    const readback = adapter.constrain(joint, limits);
    assert.deepEqual(readback.map(entry => entry.coordinate), ["x", "y", "z"]);
    for (const expected of limits.axes) {
      const actual = adapter.read(joint, expected.coordinate);
      assert.equal(actual.enabled, true);
      close(actual.minRadians, expected.minRadians, 2e-6);
      close(actual.maxRadians, expected.maxRadians, 2e-6);
    }
  } finally {
    world.free();
  }
});

test("generic multi-axis limits hold combined sustained torque at rotated headings", () => {
  const limits = profile("multi-axis", [
    axis("x", -0.2, 0.5),
    axis("y", -0.1, 0.4),
    axis("z", -0.3, 0.3),
  ]);

  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const world = createWorld();
    try {
      const parentRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
      const childRotation = quatMultiply(
        parentRotation,
        jointRotationFromCoordinates(zero, limits),
      );
      const { parent, child } = createBodies(world, parentRotation, childRotation);
      const joint = world.createImpulseJoint(
        RAPIER.JointData.spherical(zero, zero),
        parent,
        child,
        true,
      );
      joint.setLocalFrame1(limits.parentFrame.anchor, limits.parentFrame.rotation);
      joint.setLocalFrame2(limits.childFrame.anchor, limits.childFrame.rotation);
      constrainRapierJoint(world, joint, limits);
      const basis = jointFrameAxesWorld(parentRotation, limits);
      drive(world, child, add(add(scale(basis.x, 5), scale(basis.y, 4)), scale(basis.z, 3)));
      const measured = jointCoordinates(parent.rotation(), child.rotation(), limits);
      close(measured.x, 0.5, 0.012);
      close(measured.y, 0.4, 0.012);
      close(measured.z, 0.3, 0.012);
    } finally {
      world.free();
    }
  }
});

test("one-way hinges keep their axis and bend direction on mirrored frames", () => {
  const maximum = 145 * Math.PI / 180;
  const cases = [
    { heading: 0, frameRotation: identity },
    { heading: 0.8, frameRotation: identity },
    { heading: -0.7, frameRotation: quatFromAxisAngle({ x: 0, y: 1, z: 0 }, Math.PI) },
  ];

  for (const { heading, frameRotation } of cases) {
    const limits = profile("hinge", [axis("x", 0, maximum)], frame(frameRotation), frame(frameRotation));
    for (const direction of [-1, 1]) {
      const world = createWorld();
      try {
        const parentRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
        const childRotation = quatMultiply(parentRotation, jointRotationFromCoordinates(zero, limits));
        const { parent, child } = createBodies(world, parentRotation, childRotation);
        const hingeAxisParent = rotate(limits.parentFrame.rotation, { x: 1, y: 0, z: 0 });
        const hingeAxisChild = rotate(limits.childFrame.rotation, { x: 1, y: 0, z: 0 });
        const joint = world.createImpulseJoint(
          RAPIER.JointData.revoluteWithAxes(zero, zero, hingeAxisParent, hingeAxisChild),
          parent,
          child,
          true,
        );
        joint.setLocalFrame1(limits.parentFrame.anchor, limits.parentFrame.rotation);
        joint.setLocalFrame2(limits.childFrame.anchor, limits.childFrame.rotation);
        createRapierJointLimitAdapter(world).constrain(joint, limits);
        const basis = jointFrameAxesWorld(parentRotation, limits);
        const torque = add(
          scale(basis.x, 5 * direction),
          add(scale(basis.y, 4), scale(basis.z, -3)),
        );
        drive(world, child, torque);
        const measured = jointCoordinates(parent.rotation(), child.rotation(), limits);
        close(measured.x, direction > 0 ? maximum : 0, 0.012);
        close(measured.y, 0, 2e-3);
        close(measured.z, 0, 2e-3);
      } finally {
        world.free();
      }
    }
  }
});

test("non-identity joint frames map structural limits into the intended body-local axis", () => {
  const frameRotation = quatFromAxisAngle({ x: 0, y: 0, z: 1 }, Math.PI / 2);
  const limits = profile("hinge", [axis("x", -0.25, 0.5)], frame(frameRotation), frame(frameRotation));

  for (const heading of [0, 1.1, -0.9]) {
    const world = createWorld();
    try {
      const parentRotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
      const childRotation = quatMultiply(parentRotation, jointRotationFromCoordinates(zero, limits));
      const { parent, child } = createBodies(world, parentRotation, childRotation);
      const localAxis = rotate(frameRotation, { x: 1, y: 0, z: 0 });
      const joint = world.createImpulseJoint(
        RAPIER.JointData.revoluteWithAxes(zero, zero, localAxis, localAxis),
        parent,
        child,
        true,
      );
      joint.setLocalFrame1(zero, frameRotation);
      joint.setLocalFrame2(zero, frameRotation);
      createRapierJointLimitAdapter(world).constrain(joint, limits);
      drive(world, child, scale(jointFrameAxesWorld(parentRotation, limits).x, 5));
      const measured = jointCoordinates(parent.rotation(), child.rotation(), limits);
      close(measured.x, 0.5, 0.012);
      close(measured.y, 0, 2e-3);
      close(measured.z, 0, 2e-3);
    } finally {
      world.free();
    }
  }
});

test("adapter rejects malformed profiles before mutating the joint", () => {
  const world = createWorld();
  try {
    const { parent, child } = createBodies(world, identity, identity);
    const joint = world.createImpulseJoint(RAPIER.JointData.spherical(zero, zero), parent, child, true);
    const adapter = new RapierJointLimitAdapter(world);
    assert.throws(
      () => adapter.constrain(joint, profile("hinge", [axis("x", 0, 1), axis("y", 0, 1)])),
      /exactly one/,
    );
    assert.throws(
      () => adapter.constrain(joint, profile("multi-axis", [axis("x", 0, 1), axis("x", -1, 1)])),
      /repeats the x coordinate/,
    );
    assert.equal(adapter.read(joint, "x").enabled, false);
    assert.throws(
      () => adapter.constrainAxis(joint, axis("z", 1, -1)),
      /minimum exceeds/,
    );
    assert.throws(() => adapter.read(-1, "x"), /Invalid Rapier impulse-joint handle/);
  } finally {
    world.free();
  }
});
