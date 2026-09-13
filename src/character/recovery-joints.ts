import { lowestWorldPoint } from "../core/geometry";
import { SEGMENT_BY_ID } from "../core/humanoid";
import type { JointProfile, Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";
import {
  clampJointCoordinates,
  jointCoordinates,
  jointCoordinateTargetError,
  jointLimitErrorMagnitude,
  jointRotationFromCoordinates,
} from "./joint-coordinates";
import { quatMultiply, rotate, sub, worldPoint } from "./math";

const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };

export const RECOVERY_ARM_CHAIN = [
  "ShoulderGirdle",
  "UpperArm",
  "Forearm",
  "ForearmTwist",
  "Hand",
] as const;

export const RECOVERY_LEG_CHAIN = [
  "Thigh",
  "Shin",
  "Ankle",
  "Foot",
  "Forefoot",
] as const;

function profileFor(id: SegmentId): JointProfile {
  const profile = SEGMENT_BY_ID.get(id)?.jointProfile;
  if (!profile) throw new RangeError(`${id} does not have an articulated joint profile.`);
  return profile;
}

/** Build a body-local target from the same coordinates used by Rapier limits. */
export function recoveryJointRotation(id: SegmentId, coordinates: Vec3): Quat {
  const profile = profileFor(id);
  return jointRotationFromCoordinates(clampJointCoordinates(coordinates, profile), profile);
}

/** Clamp a body-local target through its asymmetric anatomical profile. */
export function limitRecoveryJoint(id: SegmentId, input: Quat): Quat {
  const profile = profileFor(id);
  const coordinates = jointCoordinates(IDENTITY, input, profile);
  return jointRotationFromCoordinates(clampJointCoordinates(coordinates, profile), profile);
}

/** Structural-limit excess in radians for diagnostics and target rejection. */
export function recoveryJointLimitError(id: SegmentId, input: Quat): number {
  const profile = profileFor(id);
  return jointLimitErrorMagnitude(jointCoordinates(IDENTITY, input, profile), profile);
}

/** Coordinate-space target blend; no Euler or quaternion component interpolation. */
export function blendRecoveryJointTargets(
  id: SegmentId,
  from: Quat,
  to: Quat,
  amount: number,
): Quat {
  const profile = profileFor(id);
  const start = jointCoordinates(IDENTITY, from, profile);
  const finish = clampJointCoordinates(jointCoordinates(IDENTITY, to, profile), profile);
  const delta = jointCoordinateTargetError(start, finish, profile);
  const t = Math.max(0, Math.min(1, amount));
  return jointRotationFromCoordinates(clampJointCoordinates({
    x: start.x + delta.x * t,
    y: start.y + delta.y * t,
    z: start.z + delta.z * t,
  }, profile), profile);
}

export function recoveryLimbIds(side: "left" | "right", arm: boolean): SegmentId[] {
  return (arm ? RECOVERY_ARM_CHAIN : RECOVERY_LEG_CHAIN)
    .map((suffix) => `${side}${suffix}` as SegmentId);
}

/** Forward geometry from the actual parent, including every expanded segment. */
export function reconstructRecoveryLimb(
  side: "left" | "right",
  arm: boolean,
  parent: Pick<SegmentPose, "position" | "rotation">,
  rotations: ReadonlyMap<SegmentId, Quat>,
): { poses: SegmentPose[]; floorClearanceM: number } {
  const poses: SegmentPose[] = [];
  let current = parent;
  let floorClearanceM = Infinity;
  for (const id of recoveryLimbIds(side, arm)) {
    const definition = SEGMENT_BY_ID.get(id)!;
    const profile = profileFor(id);
    const localRotation = rotations.get(id) ?? definition.restLocalRotation;
    const rotation = quatMultiply(current.rotation, limitRecoveryJoint(id, localRotation));
    const position = sub(
      worldPoint(current.position, current.rotation, profile.parentFrame.anchor),
      rotate(rotation, profile.childFrame.anchor),
    );
    const pose: SegmentPose = {
      id,
      position,
      rotation,
      linearVelocity: ZERO,
      angularVelocity: ZERO,
    };
    floorClearanceM = Math.min(
      floorClearanceM,
      lowestWorldPoint(definition.geometry, position, rotation).y,
    );
    poses.push(pose);
    current = pose;
  }
  return { poses, floorClearanceM };
}
