import type { RigidBody } from "@dimforge/rapier3d-compat";

import { SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type {
  JointAxisProfile,
  JointCoordinate,
  JointProfile,
  Quat,
  SegmentId,
  Vec3,
} from "../core/types";
import {
  clampJointCoordinates,
  jointCoordinateKinematics,
  jointCoordinateTargetError,
  jointCoordinates,
} from "./joint-coordinates";
import { add, clamp, clampLength, dot, scale } from "./math";
import {
  solveReducedCoordinateMotorTorques,
  type RecoveryBodyInverseInertia,
  type ReducedCoordinateMotorIntent,
} from "./recovery-motors";
import {
  articulatedCoordinateResponse,
  type ArticulatedSupportConstraint,
} from "./articulated-inertia";

const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const COORDINATES = ["x", "y", "z"] as const satisfies readonly JointCoordinate[];
type MutableVec3 = { x: number; y: number; z: number };

export interface JointMotorCommand {
  readonly id: SegmentId;
  /** Child body rotation relative to its parent body, including joint reference frames. */
  readonly targetLocalRotation: Quat;
  readonly stiffness: number;
  readonly damping: number;
  readonly strengthScale: number;
  /** Calibrates muscle recruitment after the coupled response solve. */
  readonly effortScale?: number;
  readonly feedforwardWorld?: Vec3;
}

export interface JointMotorResult {
  readonly coordinates: Vec3;
  readonly targetCoordinates: Vec3;
  readonly coordinateError: Vec3;
  readonly torqueWorld: Vec3;
  readonly saturationRatio: number;
}

export interface JointMotorSolveOptions {
  /** Loaded, sticking ground patches that condition the articulated response. */
  readonly supports?: readonly ArticulatedSupportConstraint[];
  /** Add profile damping and soft-limit resistance independently of posture strength. */
  readonly passiveResistance?: boolean;
}

function coordinateValue(vector: Vec3, coordinate: JointCoordinate): number {
  return vector[coordinate];
}

function fromCoordinates(
  values: Vec3,
  basis: Readonly<Record<JointCoordinate, Vec3>>,
): Vec3 {
  return add(add(scale(basis.x, values.x), scale(basis.y, values.y)), scale(basis.z, values.z));
}

/** Least-squares coordinates of a physical torque in the conjugate axis span. */
function worldTorqueCoordinates(
  vector: Vec3,
  axes: readonly { coordinate: JointCoordinate }[],
  torqueAxes: Readonly<Record<JointCoordinate, Vec3>>,
): Vec3 {
  const coordinates = axes.map((axis) => axis.coordinate);
  const size = coordinates.length;
  const matrix = coordinates.map((left) => coordinates.map((right) => (
    dot(torqueAxes[left], torqueAxes[right])
  )));
  const rhs = coordinates.map((coordinate) => dot(torqueAxes[coordinate], vector));
  // Partial-pivot Gaussian elimination is sufficient for this 1-3 dimensional
  // Gram system. Profile axes are independent throughout the supported range.
  for (let column = 0; column < size; column++) {
    let pivot = column;
    for (let row = column + 1; row < size; row++) {
      if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
    }
    if (Math.abs(matrix[pivot][column]) < 1e-12) {
      throw new RangeError("Joint coordinate torque axes must be linearly independent.");
    }
    [matrix[column], matrix[pivot]] = [matrix[pivot], matrix[column]];
    [rhs[column], rhs[pivot]] = [rhs[pivot], rhs[column]];
    for (let row = column + 1; row < size; row++) {
      const ratio = matrix[row][column] / matrix[column][column];
      for (let inner = column; inner < size; inner++) {
        matrix[row][inner] -= ratio * matrix[column][inner];
      }
      rhs[row] -= ratio * rhs[column];
    }
  }
  const values = new Array<number>(size).fill(0);
  for (let row = size - 1; row >= 0; row--) {
    let value = rhs[row];
    for (let column = row + 1; column < size; column++) value -= matrix[row][column] * values[column];
    values[row] = value / matrix[row][row];
  }
  const result: MutableVec3 = { x: 0, y: 0, z: 0 };
  for (let index = 0; index < size; index++) result[coordinates[index]] = values[index];
  return result;
}

function inverseInertias(bodies: ReadonlyMap<SegmentId, RigidBody>): Map<SegmentId, RecoveryBodyInverseInertia> {
  const result = new Map<SegmentId, RecoveryBodyInverseInertia>();
  for (const [id, body] of bodies) {
    const matrix = body.effectiveWorldInvInertia();
    result.set(id, {
      m11: matrix.m11, m12: matrix.m12, m13: matrix.m13,
      m22: matrix.m22, m23: matrix.m23, m33: matrix.m33,
    });
  }
  return result;
}

function completeAssembly(bodies: ReadonlyMap<SegmentId, RigidBody>): boolean {
  return bodies.size === SEGMENTS.length && SEGMENTS.every(({ id }) => bodies.has(id));
}

type PassiveAxisFeedback = Readonly<{
  error: number;
  kp: number;
  kd: number;
  cap: number;
  active: boolean;
}>;

function passiveAxisFeedback(
  value: number,
  velocity: number,
  axis: JointAxisProfile,
  profile: JointProfile,
): PassiveAxisFeedback {
  const width = Math.max(1e-6, axis.maxRadians - axis.minRadians);
  const zone = Math.max(1e-5, width * profile.limitSoftZoneFraction);
  let error = 0;
  let amount = 0;
  if (value < axis.minRadians + zone) {
    amount = clamp((axis.minRadians + zone - value) / zone, 0, 2);
    error = zone * amount ** 3;
  } else if (value > axis.maxRadians - zone) {
    amount = clamp((value - (axis.maxRadians - zone)) / zone, 0, 2);
    error = -zone * amount ** 3;
  }
  return {
    error,
    // There is no passive position spring in the middle of the range. Keeping
    // kp there would still attenuate the damping term in the implicit solve.
    kp: amount > 0 ? axis.passiveStiffnessNmPerRad : 0,
    kd: axis.dampingNmsPerRad,
    cap: Math.max(
      axis.maxMotorTorqueNm * 0.5,
      axis.passiveStiffnessNmPerRad * zone * 2,
    ),
    active: amount !== 0 || Math.abs(velocity) >= 1e-5,
  };
}

/**
 * Run the same coupled, equal-and-opposite torque solve used by recovery for any
 * posture target. Forbidden joint coordinates are removed before the solve and
 * every permitted coordinate retains its own actuator ceiling afterward.
 */
export function applyCoupledJointMotors(
  bodies: ReadonlyMap<SegmentId, RigidBody>,
  commands: readonly JointMotorCommand[],
  dt: number,
  options: JointMotorSolveOptions = {},
): ReadonlyMap<SegmentId, JointMotorResult> {
  const intents: ReducedCoordinateMotorIntent[] = [];
  const prepared = new Map<SegmentId, {
    coordinates: Vec3;
    targetCoordinates: Vec3;
    coordinateError: Vec3;
    torqueAxes: Readonly<Record<JointCoordinate, Vec3>>;
    caps: Vec3;
    effortScale: number;
  }>();

  for (const command of commands) {
    const definition = SEGMENT_BY_ID.get(command.id);
    const profile = definition?.jointProfile;
    if (!definition?.parent || !profile) continue;
    const parent = bodies.get(definition.parent);
    const child = bodies.get(definition.id);
    if (!parent || !child) continue;

    const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
    const rawTarget = jointCoordinates(IDENTITY, command.targetLocalRotation, profile);
    const targetCoordinates = clampJointCoordinates(rawTarget, profile);
    const coordinateError = jointCoordinateTargetError(coordinates, targetCoordinates, profile);
    const kinematics = jointCoordinateKinematics(
      parent.angvel(), child.angvel(), parent.rotation(), coordinates, profile,
    );
    const strength = clamp(command.strengthScale, 0, 1);
    const caps: MutableVec3 = { x: 0, y: 0, z: 0 };

    const feedforwardCoordinates = worldTorqueCoordinates(
      command.feedforwardWorld ?? ZERO,
      profile.axes,
      kinematics.torqueAxesWorld,
    );
    intents.push({
      id: command.id,
      parent: definition.parent,
      axes: profile.axes.map((axis) => {
        const coordinate = axis.coordinate;
        const activeKp = Math.max(0, command.stiffness) * strength;
        const activeKd = Math.max(0, command.damping) * strength;
        const passive = options.passiveResistance
          ? passiveAxisFeedback(
            coordinates[coordinate], kinematics.rates[coordinate], axis, profile,
          )
          : null;
        const kp = activeKp + (passive?.kp ?? 0);
        const feedback = activeKp * coordinateError[coordinate]
          + (passive?.kp ?? 0) * (passive?.error ?? 0);
        const cap = Math.max(
          axis.maxMotorTorqueNm * strength,
          passive?.cap ?? 0,
        );
        caps[coordinate] = cap;
        return {
          coordinate,
          worldAxis: kinematics.torqueAxesWorld[coordinate],
          error: kp > 0 ? feedback / kp : 0,
          velocity: kinematics.rates[coordinate],
          kp,
          kd: activeKd + (passive?.kd ?? 0),
          feedforward: feedforwardCoordinates[coordinate] * strength,
          cap,
        };
      }),
    });
    prepared.set(command.id, {
      coordinates, targetCoordinates, coordinateError,
      torqueAxes: kinematics.torqueAxesWorld, caps,
      effortScale: Math.max(0, command.effortScale ?? 1),
    });
  }

  const solved = solveReducedCoordinateMotorTorques(
    intents,
    inverseInertias(bodies),
    dt,
    completeAssembly(bodies)
      ? { response: articulatedCoordinateResponse(bodies, options.supports) }
      : {},
  );
  const results = new Map<SegmentId, JointMotorResult>();
  for (const intent of intents) {
    const state = prepared.get(intent.id)!;
    const solution = solved.get(intent.id);
    const torqueCoordinates: MutableVec3 = { x: 0, y: 0, z: 0 };
    let saturationRatio = 0;
    for (const coordinate of COORDINATES) {
      const cap = state.caps[coordinate];
      if (cap <= 0) continue;
      const coupled = solution?.coordinateTorques[coordinate] ?? 0;
      const requestedCoupled = solution?.requestedCoordinateTorques[coordinate] ?? 0;
      const requested = coupled * state.effortScale;
      const unconstrained = requestedCoupled * state.effortScale;
      torqueCoordinates[coordinate] = clamp(requested, -cap, cap);
      saturationRatio = Math.max(
        saturationRatio,
        Math.abs(requested) / cap,
        Math.abs(unconstrained) / cap,
      );
    }
    const rawTorqueWorld = fromCoordinates(torqueCoordinates, state.torqueAxes);
    // Coordinate ceilings define an ellipsoidal actuator envelope only when
    // the conjugate axes are orthonormal. Combined anatomical rotations make
    // those axes oblique, so also enforce the documented aggregate vector cap.
    const aggregateCap = Math.hypot(...COORDINATES.map((coordinate) => state.caps[coordinate]));
    const torqueWorld = clampLength(rawTorqueWorld, aggregateCap);
    const child = bodies.get(intent.id)!;
    const parent = bodies.get(intent.parent!)!;
    // A sleeping dynamic assembly is already at equilibrium. Rapier wakes an
    // island even for a nominally non-waking impulse on some joint paths, so
    // omit the no-op actuation entirely until contact or a grab wakes it.
    if (!child.isSleeping() || !parent.isSleeping()) {
      child.applyTorqueImpulse(scale(torqueWorld, dt), false);
      parent.applyTorqueImpulse(scale(torqueWorld, -dt), false);
    }
    results.set(intent.id, {
      coordinates: state.coordinates,
      targetCoordinates: state.targetCoordinates,
      coordinateError: state.coordinateError,
      torqueWorld,
      saturationRatio,
    });
  }
  return results;
}

/**
 * Muscle/tissue resistance is independent of posture control. It is weakest in
 * the middle of the range and rises cubically through each outer soft zone.
 */
export function applyPassiveJointResistance(
  bodies: ReadonlyMap<SegmentId, RigidBody>,
  dt: number,
  options: Pick<JointMotorSolveOptions, "supports"> = {},
): ReadonlyMap<SegmentId, Vec3> {
  const intents: ReducedCoordinateMotorIntent[] = [];
  for (const definition of SEGMENT_BY_ID.values()) {
    const profile = definition.jointProfile;
    if (!definition.parent || !profile) continue;
    const parent = bodies.get(definition.parent);
    const child = bodies.get(definition.id);
    if (!parent || !child) continue;
    const coordinates = jointCoordinates(parent.rotation(), child.rotation(), profile);
    const kinematics = jointCoordinateKinematics(
      parent.angvel(), child.angvel(), parent.rotation(), coordinates, profile,
    );
    let resisting = false;
    const axes = profile.axes.map((axis) => {
      const coordinate = axis.coordinate;
      const passive = passiveAxisFeedback(
        coordinateValue(coordinates, coordinate),
        coordinateValue(kinematics.rates, coordinate),
        axis,
        profile,
      );
      resisting ||= passive.active;
      return {
        coordinate,
        worldAxis: kinematics.torqueAxesWorld[coordinate],
        error: passive.error,
        velocity: kinematics.rates[coordinate],
        kp: passive.kp,
        kd: passive.kd,
        cap: passive.cap,
      };
    });
    if (!resisting) continue;
    intents.push({ id: definition.id, parent: definition.parent, axes });
  }

  // Every intent and inertia tensor above came from the same pre-impulse body
  // snapshot. Solving the whole network at once prevents a proximal damping
  // impulse from becoming an amplified distal input later in the same frame.
  const solved = solveReducedCoordinateMotorTorques(
    intents,
    inverseInertias(bodies),
    dt,
    completeAssembly(bodies) && options.supports !== undefined
      ? { response: articulatedCoordinateResponse(bodies, options.supports) }
      : {},
  );
  const results = new Map<SegmentId, Vec3>();
  for (const intent of intents) {
    const profile = SEGMENT_BY_ID.get(intent.id)?.jointProfile;
    const aggregateCap = Math.hypot(...(profile?.axes.map((axis) => {
      const width = Math.max(1e-6, axis.maxRadians - axis.minRadians);
      const zone = Math.max(1e-5, width * profile.limitSoftZoneFraction);
      return Math.max(
        axis.maxMotorTorqueNm * 0.5,
        axis.passiveStiffnessNmPerRad * zone * 2,
      );
    }) ?? []));
    const torqueWorld = clampLength(solved.get(intent.id)?.torqueWorld ?? ZERO, aggregateCap);
    const child = bodies.get(intent.id)!;
    const parent = bodies.get(intent.parent!)!;
    child.applyTorqueImpulse(scale(torqueWorld, dt), true);
    parent.applyTorqueImpulse(scale(torqueWorld, -dt), true);
    results.set(intent.id, torqueWorld);
  }
  return results;
}
