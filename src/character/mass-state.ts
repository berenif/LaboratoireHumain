import { convexVolumeCentroid } from "../core/geometry";
import { SEGMENTS, SEGMENT_BY_ID } from "../core/humanoid";
import type { SegmentId, SegmentPose, Vec3 } from "../core/types";
import { add, scale, worldPoint } from "./math";

const LOCAL_CENTERS: ReadonlyMap<SegmentId, Vec3> = new Map(
  SEGMENTS.map(definition => [definition.id, convexVolumeCentroid(definition.geometry)]),
);

/** Use measured Rapier state when present; geometry is only the planning fallback. */
export function segmentWorldMassCenter(pose: SegmentPose): Vec3 {
  if (pose.centerOfMass) return pose.centerOfMass;
  const local = LOCAL_CENTERS.get(pose.id);
  if (!local) throw new RangeError(`Unknown body segment: ${pose.id}`);
  return worldPoint(pose.position, pose.rotation, local);
}

/** Shared by standing and recovery so support and gravity use the same body mass. */
export function measureMassState(poses: Iterable<SegmentPose>): {
  position: Vec3; velocity: Vec3; massKg: number;
} {
  let position = { x: 0, y: 0, z: 0 }, velocity = { x: 0, y: 0, z: 0 }, massKg = 0;
  for (const pose of poses) {
    const mass = pose.massKg ?? SEGMENT_BY_ID.get(pose.id)?.massKg;
    if (mass === undefined) continue;
    position = add(position, scale(segmentWorldMassCenter(pose), mass));
    // Rapier linear velocity is already measured at each body's mass centre.
    velocity = add(velocity, scale(pose.linearVelocity, mass));
    massKg += mass;
  }
  return massKg > 0
    ? { position: scale(position, 1 / massKg), velocity: scale(velocity, 1 / massKg), massKg }
    : { position, velocity, massKg };
}
