import { Q, V3 } from "../core/math";
import type { PoseSnapshot, Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";

export function clampInterpolationAlpha(alpha: number): number {
  return Number.isFinite(alpha) ? Math.max(0, Math.min(1, alpha)) : 1;
}

export function rotateVector(rotation: Quat, vector: Vec3): Vec3 {
  const q = Q.normalize(rotation);
  const tx = 2 * (q.y * vector.z - q.z * vector.y);
  const ty = 2 * (q.z * vector.x - q.x * vector.z);
  const tz = 2 * (q.x * vector.y - q.y * vector.x);
  return {
    x: vector.x + q.w * tx + (q.y * tz - q.z * ty),
    y: vector.y + q.w * ty + (q.z * tx - q.x * tz),
    z: vector.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

export function transformLocalPoint(pose: SegmentPose, localPoint: Vec3): Vec3 {
  return V3.add(pose.position, rotateVector(pose.rotation, localPoint));
}

/** Returns a detached display snapshot. It is never fed back to simulation. */
export function interpolatePoseSnapshot(
  previous: PoseSnapshot,
  current: PoseSnapshot,
  alpha: number,
): PoseSnapshot {
  const t = clampInterpolationAlpha(alpha);
  const previousById = new Map<SegmentId, SegmentPose>(
    previous.segments.map((segment) => [segment.id, segment]),
  );
  const segments = current.segments.map((segment): SegmentPose => {
    const before = previousById.get(segment.id) ?? segment;
    return {
      id: segment.id,
      position: V3.lerp(before.position, segment.position, t),
      rotation: Q.nlerp(before.rotation, segment.rotation, t),
      linearVelocity: V3.lerp(before.linearVelocity, segment.linearVelocity, t),
      angularVelocity: V3.lerp(before.angularVelocity, segment.angularVelocity, t),
    };
  });
  return {
    ...current,
    rootPosition: V3.lerp(previous.rootPosition, current.rootPosition, t),
    rootRotation: Q.nlerp(previous.rootRotation, current.rootRotation, t),
    segments,
  };
}
