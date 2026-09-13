import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const {
  clampJointCoordinates,
  jointAngularVelocityCoordinates,
  jointCoordinateKinematics,
  jointCoordinates,
  jointCoordinateTargetError,
  jointFrameAxesWorld,
  jointLimitError,
  jointLimitErrorMagnitude,
  jointRotationFromCoordinates,
  wrapAngle,
} = await import("../src/character/joint-coordinates.ts");
const {
  add,
  dot,
  length,
  quatFromAxisAngle,
  quatInverse,
  quatMultiply,
  rotate,
  scale,
} = await import("../src/character/math.ts");

const identity = { x: 0, y: 0, z: 0, w: 1 };
const zero = { x: 0, y: 0, z: 0 };
const frame = (rotation = identity) => ({ anchor: zero, rotation });
const axis = (coordinate, minRadians = -1, maxRadians = 1) => ({
  coordinate,
  minRadians,
  maxRadians,
  passiveStiffnessNmPerRad: 20,
  dampingNmsPerRad: 2,
  maxMotorTorqueNm: 80,
});
const profile = (axes, extra = {}) => ({
  kind: axes.length === 1 ? "hinge" : "multi-axis",
  parentFrame: frame(),
  childFrame: frame(),
  axes,
  limitSoftZoneFraction: 0.15,
  ...extra,
});

function close(actual, expected, tolerance = 1e-10) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} differs from ${expected}`);
}

function closeVec(actual, expected, tolerance = 1e-10) {
  for (const coordinate of ["x", "y", "z"]) close(actual[coordinate], expected[coordinate], tolerance);
}

test("combined joint coordinates round-trip through reference frames at rotated headings", () => {
  const frames = profile([axis("x"), axis("y"), axis("z")], {
    parentFrame: frame(quatFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.42)),
    childFrame: frame(quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -0.31)),
  });
  const expected = { x: 0.47, y: -0.36, z: 0.28 };
  const bodyRelative = jointRotationFromCoordinates(expected, frames);

  for (const heading of [0, Math.PI / 3, -Math.PI * 0.74]) {
    const parent = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const child = quatMultiply(parent, bodyRelative);
    closeVec(jointCoordinates(parent, child, frames), expected);
    // The quaternion antipode must measure identically.
    closeVec(jointCoordinates(parent, {
      x: -child.x,
      y: -child.y,
      z: -child.z,
      w: -child.w,
    }, frames), expected);
  }
});

test("coordinates absent from a hinge profile are structurally locked", () => {
  const hinge = profile([axis("x", 0, 2.4)]);
  const relative = jointRotationFromCoordinates({ x: 0.73, y: 0.9, z: -0.8 }, hinge);
  closeVec(jointCoordinates(identity, relative, hinge), { x: 0.73, y: 0, z: 0 });
  closeVec(clampJointCoordinates({ x: 3, y: 0.9, z: -0.8 }, hinge), { x: 2.4, y: 0, z: 0 });
  closeVec(jointLimitError({ x: 2.7, y: 0.2, z: -0.1 }, hinge), { x: 0.3, y: 0.2, z: -0.1 });
});

test("asymmetric limits clamp about their center, including across the pi seam", () => {
  const radians = degrees => degrees * Math.PI / 180;
  const seam = profile([axis("x", radians(170), radians(190))]);

  close(clampJointCoordinates({ ...zero, x: radians(-179) }, seam).x, radians(-179));
  close(jointLimitError({ ...zero, x: radians(-179) }, seam).x, 0);
  close(clampJointCoordinates({ ...zero, x: radians(-150) }, seam).x, radians(-170));
  close(jointLimitError({ ...zero, x: radians(-150) }, seam).x, radians(20));
  close(clampJointCoordinates({ ...zero, x: radians(150) }, seam).x, radians(170));
  close(jointLimitError({ ...zero, x: radians(150) }, seam).x, radians(-20));
  close(jointLimitErrorMagnitude({ ...zero, x: radians(150) }, seam), radians(20));
  close(wrapAngle(3 * Math.PI), Math.PI);
  close(wrapAngle(-3 * Math.PI), -Math.PI);
});

test("target errors and angular velocities use the parent joint-frame axes", () => {
  const parentFrameRotation = quatFromAxisAngle({ x: 0, y: 0, z: 1 }, Math.PI / 2);
  const constrained = profile([axis("x"), axis("z")], {
    kind: "multi-axis",
    parentFrame: frame(parentFrameRotation),
  });
  const heading = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0.7);
  const axes = jointFrameAxesWorld(heading, constrained);
  closeVec(axes.x, rotate(quatMultiply(heading, parentFrameRotation), { x: 1, y: 0, z: 0 }));
  closeVec(axes.y, rotate(quatMultiply(heading, parentFrameRotation), { x: 0, y: 1, z: 0 }));
  closeVec(axes.z, rotate(quatMultiply(heading, parentFrameRotation), { x: 0, y: 0, z: 1 }));

  const parentVelocity = { x: 0.1, y: -0.2, z: 0.3 };
  const childVelocity = add(parentVelocity, add(scale(axes.x, 1.25), scale(axes.z, -0.75)));
  closeVec(
    jointAngularVelocityCoordinates(parentVelocity, childVelocity, heading, zero, constrained),
    { x: 1.25, y: 0, z: -0.75 },
  );
  closeVec(
    jointCoordinateTargetError(
      { x: Math.PI - 0.1, y: 0.5, z: -Math.PI + 0.1 },
      { x: -Math.PI + 0.1, y: -0.5, z: Math.PI - 0.1 },
      constrained,
    ),
    { x: 0.2, y: 0, z: -0.2 },
  );
});

test("combined-coordinate rates and conjugate torque axes match quaternion finite differences", () => {
  const combined = profile([axis("x"), axis("y"), axis("z")], {
    parentFrame: frame(quatFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.27)),
    childFrame: frame(quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -0.19)),
  });
  const coordinates = { x: -0.096, y: -0.68, z: 0.342 };
  const coordinateTorque = { x: -3.06, y: 18.18, z: 14.04 };
  const frameVelocity = { x: -0.8, y: 0.5, z: 1.1 };
  const epsilon = 1e-6;

  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const parent = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    const child = quatMultiply(parent, jointRotationFromCoordinates(coordinates, combined));
    const frameAxes = jointFrameAxesWorld(parent, combined);
    const relativeWorldVelocity = add(
      add(scale(frameAxes.x, frameVelocity.x), scale(frameAxes.y, frameVelocity.y)),
      scale(frameAxes.z, frameVelocity.z),
    );
    const parentVelocity = { x: 0.17, y: -0.23, z: 0.31 };
    const childVelocity = add(parentVelocity, relativeWorldVelocity);
    const kinematics = jointCoordinateKinematics(
      parentVelocity, childVelocity, parent, coordinates, combined,
    );

    const speed = length(relativeWorldVelocity);
    const plus = jointCoordinates(parent, quatMultiply(
      quatFromAxisAngle(relativeWorldVelocity, speed * epsilon), child,
    ), combined);
    const minus = jointCoordinates(parent, quatMultiply(
      quatFromAxisAngle(relativeWorldVelocity, -speed * epsilon), child,
    ), combined);
    const finiteDifference = {
      x: wrapAngle(plus.x - minus.x) / (2 * epsilon),
      y: wrapAngle(plus.y - minus.y) / (2 * epsilon),
      z: wrapAngle(plus.z - minus.z) / (2 * epsilon),
    };
    closeVec(kinematics.rates, finiteDifference, 5e-9);
    closeVec(
      jointAngularVelocityCoordinates(
        parentVelocity, childVelocity, parent, coordinates, combined,
      ),
      finiteDifference,
      5e-9,
    );

    const physicalTorque = add(
      add(
        scale(kinematics.torqueAxesWorld.x, coordinateTorque.x),
        scale(kinematics.torqueAxesWorld.y, coordinateTorque.y),
      ),
      scale(kinematics.torqueAxesWorld.z, coordinateTorque.z),
    );
    const coordinatePower = coordinateTorque.x * kinematics.rates.x
      + coordinateTorque.y * kinematics.rates.y
      + coordinateTorque.z * kinematics.rates.z;
    close(dot(physicalTorque, relativeWorldVelocity), coordinatePower, 1e-9);
    // Combined rotation makes the y-coordinate's conjugate torque direction
    // non-orthogonal: at this pose it includes both frame x and z components.
    close(dot(kinematics.torqueAxesWorld.y, frameAxes.x), -0.138378738, 1e-8);
    close(dot(kinematics.torqueAxesWorld.y, frameAxes.y), 1, 1e-10);
    close(dot(kinematics.torqueAxesWorld.y, frameAxes.z), -0.096986557, 1e-8);
  }
});

test("coordinate helpers reject invalid quaternions and profiles", () => {
  const multi = profile([axis("x"), axis("y")]);
  assert.throws(
    () => jointCoordinates(identity, { x: 0, y: 0, z: 0, w: 0 }, multi),
    /non-zero quaternions/,
  );
  assert.throws(
    () => clampJointCoordinates(zero, profile([axis("x"), axis("x")])),
    /repeats the x coordinate/,
  );
  assert.throws(() => wrapAngle(Number.NaN), /must be finite/);
  assert.throws(
    () => jointRotationFromCoordinates({ x: Infinity, y: 0, z: 0 }, profile([axis("x")])),
    /must be finite/,
  );
  // Verify the documented inverse relation explicitly for non-identity frames.
  const framed = profile([axis("x"), axis("y"), axis("z")], {
    parentFrame: frame(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0.4)),
    childFrame: frame(quatInverse(quatFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.2))),
  });
  closeVec(
    jointCoordinates(identity, jointRotationFromCoordinates(zero, framed), framed),
    zero,
  );
});
