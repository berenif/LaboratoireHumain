import type {
  JointAxisProfile,
  JointCoordinate,
  JointProfile,
  Quat,
  Vec3,
} from "../core/types";
import { dot, quatInverse, quatMultiply, rotate, sub } from "./math";

export const JOINT_COORDINATES = ["x", "y", "z"] as const satisfies readonly JointCoordinate[];

const BASIS: Readonly<Record<JointCoordinate, Vec3>> = {
  x: { x: 1, y: 0, z: 0 },
  y: { x: 0, y: 1, z: 0 },
  z: { x: 0, y: 0, z: 1 },
};

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const QUATERNION_EPSILON = 1e-12;

export interface JointCoordinateKinematics {
  /** Time derivatives of the permitted anatomical coordinates. */
  readonly rates: Vec3;
  /**
   * World-space physical torque vectors conjugate to one unit of generalized
   * torque in each coordinate. These vectors are generally neither orthogonal
   * nor unit length once several coordinates are nonzero.
   */
  readonly torqueAxesWorld: Readonly<Record<JointCoordinate, Vec3>>;
}

function assertFinite(value: number, label: string): void {
  if (!Number.isFinite(value)) throw new RangeError(`${label} must be finite.`);
}

function canonicalQuaternion(input: Quat): Quat {
  const magnitude = Math.hypot(input.x, input.y, input.z, input.w);
  if (!Number.isFinite(magnitude) || magnitude < QUATERNION_EPSILON) {
    throw new RangeError("Joint rotations must be finite, non-zero quaternions.");
  }

  let q: Quat = {
    x: input.x / magnitude,
    y: input.y / magnitude,
    z: input.z / magnitude,
    w: input.w / magnitude,
  };
  // q and -q are the same orientation. Choose one hemisphere so every
  // consumer observes the same coordinate, including at the pi seam.
  const seamSign = Math.abs(q.w) <= QUATERNION_EPSILON
    ? (Math.abs(q.x) > QUATERNION_EPSILON
      ? q.x
      : Math.abs(q.y) > QUATERNION_EPSILON
        ? q.y
        : q.z)
    : q.w;
  if (seamSign < 0) q = { x: -q.x, y: -q.y, z: -q.z, w: -q.w };
  return q;
}

function axisByCoordinate(profile: JointProfile): ReadonlyMap<JointCoordinate, JointAxisProfile> {
  const result = new Map<JointCoordinate, JointAxisProfile>();
  for (const axis of profile.axes) {
    if (result.has(axis.coordinate)) {
      throw new RangeError(`Joint profile repeats the ${axis.coordinate} coordinate.`);
    }
    assertFinite(axis.minRadians, `${axis.coordinate} minimum`);
    assertFinite(axis.maxRadians, `${axis.coordinate} maximum`);
    if (axis.minRadians > axis.maxRadians) {
      throw new RangeError(`Joint ${axis.coordinate} minimum exceeds its maximum.`);
    }
    result.set(axis.coordinate, axis);
  }
  return result;
}

/** Wrap an angle onto Rapier's signed angular-coordinate branch. */
export function wrapAngle(radians: number): number {
  assertFinite(radians, "Joint angle");
  const wrapped = Math.atan2(Math.sin(radians), Math.cos(radians));
  return Object.is(wrapped, -0) ? 0 : wrapped;
}

function coordinateQuaternion(coordinates: Vec3): Quat {
  const x = wrapAngle(coordinates.x);
  const y = wrapAngle(coordinates.y);
  const z = wrapAngle(coordinates.z);
  return canonicalQuaternion({
    x: Math.tan(x * 0.5),
    y: Math.tan(y * 0.5),
    z: Math.tan(z * 0.5),
    w: 1,
  });
}

/**
 * Measure Rapier's three angular joint coordinates from body world rotations.
 *
 * The reference frames are part of the definition: at zero coordinates,
 * `parentWorld * parentFrame.rotation` equals
 * `childWorld * childFrame.rotation`. This is intentionally not an Euler
 * decomposition. It matches Rapier's per-basis angular limit coordinate.
 */
export function jointCoordinates(
  parentWorld: Quat,
  childWorld: Quat,
  profile: JointProfile,
): Vec3 {
  const parentFrameWorld = quatMultiply(parentWorld, profile.parentFrame.rotation);
  const childFrameWorld = quatMultiply(childWorld, profile.childFrame.rotation);
  const relative = canonicalQuaternion(
    quatMultiply(quatInverse(parentFrameWorld), childFrameWorld),
  );
  return {
    x: wrapAngle(2 * Math.atan2(relative.x, relative.w)),
    y: wrapAngle(2 * Math.atan2(relative.y, relative.w)),
    z: wrapAngle(2 * Math.atan2(relative.z, relative.w)),
  };
}

/**
 * Convert joint coordinates into the child's body-local rotation relative to
 * its parent. Coordinates absent from the profile are structurally locked at
 * zero. Compose the result as `childWorld = parentWorld * result`.
 */
export function jointRotationFromCoordinates(coordinates: Vec3, profile: JointProfile): Quat {
  const permitted = axisByCoordinate(profile);
  const filtered: Vec3 = {
    x: permitted.has("x") ? coordinates.x : 0,
    y: permitted.has("y") ? coordinates.y : 0,
    z: permitted.has("z") ? coordinates.z : 0,
  };
  const jointRotation = coordinateQuaternion(filtered);
  return canonicalQuaternion(quatMultiply(
    quatMultiply(profile.parentFrame.rotation, jointRotation),
    quatInverse(profile.childFrame.rotation),
  ));
}

function centeredLimit(
  value: number,
  axis: Pick<JointAxisProfile, "coordinate" | "minRadians" | "maxRadians">,
): Readonly<{ clamped: number; error: number }> {
  assertFinite(value, `${axis.coordinate} coordinate`);
  assertFinite(axis.minRadians, `${axis.coordinate} minimum`);
  assertFinite(axis.maxRadians, `${axis.coordinate} maximum`);
  if (axis.minRadians > axis.maxRadians) {
    throw new RangeError(`Joint ${axis.coordinate} minimum exceeds its maximum.`);
  }
  const center = (axis.minRadians + axis.maxRadians) * 0.5;
  const halfWidth = (axis.maxRadians - axis.minRadians) * 0.5;
  const centered = wrapAngle(value - center);
  const clampedCentered = Math.max(-halfWidth, Math.min(halfWidth, centered));
  return {
    clamped: wrapAngle(center + clampedCentered),
    error: centered - clampedCentered,
  };
}

/** Clamp permitted coordinates to their asymmetric ranges and lock all others. */
export function clampJointCoordinates(coordinates: Vec3, profile: JointProfile): Vec3 {
  const axes = axisByCoordinate(profile);
  return {
    x: axes.has("x") ? centeredLimit(coordinates.x, axes.get("x")!).clamped : 0,
    y: axes.has("y") ? centeredLimit(coordinates.y, axes.get("y")!).clamped : 0,
    z: axes.has("z") ? centeredLimit(coordinates.z, axes.get("z")!).clamped : 0,
  };
}

/**
 * Signed radians outside each structural range. A zero component is inside
 * the range; coordinates absent from the profile report their displacement
 * from the structurally locked zero coordinate.
 */
export function jointLimitError(coordinates: Vec3, profile: JointProfile): Vec3 {
  const axes = axisByCoordinate(profile);
  return {
    x: axes.has("x") ? centeredLimit(coordinates.x, axes.get("x")!).error : wrapAngle(coordinates.x),
    y: axes.has("y") ? centeredLimit(coordinates.y, axes.get("y")!).error : wrapAngle(coordinates.y),
    z: axes.has("z") ? centeredLimit(coordinates.z, axes.get("z")!).error : wrapAngle(coordinates.z),
  };
}

export function jointLimitErrorMagnitude(coordinates: Vec3, profile: JointProfile): number {
  const error = jointLimitError(coordinates, profile);
  return Math.hypot(error.x, error.y, error.z);
}

/** Shortest signed target-minus-current error on permitted coordinates. */
export function jointCoordinateTargetError(
  current: Vec3,
  target: Vec3,
  profile: JointProfile,
): Vec3 {
  const axes = axisByCoordinate(profile);
  return {
    x: axes.has("x") ? wrapAngle(target.x - current.x) : 0,
    y: axes.has("y") ? wrapAngle(target.y - current.y) : 0,
    z: axes.has("z") ? wrapAngle(target.z - current.z) : 0,
  };
}

/** World-space basis used by Rapier and by the coupled joint motors. */
export function jointFrameAxesWorld(
  parentWorld: Quat,
  profile: Pick<JointProfile, "parentFrame">,
): Readonly<Record<JointCoordinate, Vec3>> {
  const frameWorld = quatMultiply(parentWorld, profile.parentFrame.rotation);
  return {
    x: rotate(frameWorld, BASIS.x),
    y: rotate(frameWorld, BASIS.y),
    z: rotate(frameWorld, BASIS.z),
  };
}

/**
 * Exact local differential kinematics for the quaternion-component joint
 * coordinates used above.
 *
 * Let r = q.xyz / q.w = tan(c / 2) and let omega be child-minus-parent spatial
 * angular velocity in the parent joint frame. Then
 *
 *   rdot = 1/2 (omega + omega x r + r (omega dot r))
 *   cdot_i = 2 rdot_i / (1 + r_i^2) = A_i omega.
 *
 * Virtual power requires a generalized coordinate torque Q_i to create the
 * physical frame torque A_i^T Q_i. Rotating each row A_i into world space
 * therefore gives both the exact rate projection and its conjugate physical
 * torque axis.
 */
export function jointCoordinateKinematics(
  parentAngularVelocity: Vec3,
  childAngularVelocity: Vec3,
  parentWorld: Quat,
  coordinates: Vec3,
  profile: JointProfile,
): JointCoordinateKinematics {
  const permitted = axisByCoordinate(profile);
  const r = {
    x: Math.tan(wrapAngle(coordinates.x) * 0.5),
    y: Math.tan(wrapAngle(coordinates.y) * 0.5),
    z: Math.tan(wrapAngle(coordinates.z) * 0.5),
  };
  if (![r.x, r.y, r.z].every(Number.isFinite)) {
    throw new RangeError("Joint coordinates are singular at the pi seam.");
  }
  const rows: Readonly<Record<JointCoordinate, Vec3>> = {
    x: {
      x: 1,
      y: (r.z + r.x * r.y) / (1 + r.x * r.x),
      z: (-r.y + r.x * r.z) / (1 + r.x * r.x),
    },
    y: {
      x: (-r.z + r.y * r.x) / (1 + r.y * r.y),
      y: 1,
      z: (r.x + r.y * r.z) / (1 + r.y * r.y),
    },
    z: {
      x: (r.y + r.z * r.x) / (1 + r.z * r.z),
      y: (-r.x + r.z * r.y) / (1 + r.z * r.z),
      z: 1,
    },
  };
  const frameWorld = quatMultiply(parentWorld, profile.parentFrame.rotation);
  const torqueAxesWorld: Readonly<Record<JointCoordinate, Vec3>> = {
    x: rotate(frameWorld, rows.x),
    y: rotate(frameWorld, rows.y),
    z: rotate(frameWorld, rows.z),
  };
  const relative = sub(childAngularVelocity, parentAngularVelocity);
  return {
    rates: {
      x: permitted.has("x") ? dot(relative, torqueAxesWorld.x) : 0,
      y: permitted.has("y") ? dot(relative, torqueAxesWorld.y) : 0,
      z: permitted.has("z") ? dot(relative, torqueAxesWorld.z) : 0,
    },
    torqueAxesWorld,
  };
}

/** Measure exact permitted coordinate rates, including combined-rotation coupling. */
export function jointAngularVelocityCoordinates(
  parentAngularVelocity: Vec3,
  childAngularVelocity: Vec3,
  parentWorld: Quat,
  coordinates: Vec3,
  profile: JointProfile,
): Vec3 {
  return jointCoordinateKinematics(
    parentAngularVelocity,
    childAngularVelocity,
    parentWorld,
    coordinates,
    profile,
  ).rates;
}

export function zeroJointCoordinates(): Vec3 {
  return ZERO;
}
