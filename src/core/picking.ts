import { SEGMENTS_BY_REGION, SEGMENT_BY_ID } from "./humanoid";
import { raycastConvex } from "./geometry";
import { V3 } from "./math";
import type { PickResult, Quat, Ray, RegionId, SegmentPose, Vec3 } from "./types";

// Surface distance always wins. Priority only resolves numerically coincident
// intersections, so an overlapping torso proxy cannot steal a visible hand.
const REGION_PRIORITY: readonly RegionId[] = [
  "leftHand", "rightHand", "leftFoot", "rightFoot", "head", "torso", "pelvis",
];
const TIE_EPSILON_M = 1e-7;

function inverseRotate(q: Quat, v: Vec3): Vec3 {
  const magnitude = Math.hypot(q.x, q.y, q.z, q.w) || 1;
  const inverse = { x: -q.x / magnitude, y: -q.y / magnitude, z: -q.z / magnitude, w: q.w / magnitude };
  const uv = V3.cross(inverse, v);
  return V3.add(v, V3.add(V3.scale(uv, 2 * inverse.w), V3.scale(V3.cross(inverse, uv), 2)));
}

/** Pick exact canonical surfaces while preserving the seven public regions. */
export function pickRegionProxies(ray: Ray, poses: readonly SegmentPose[]): PickResult | null {
  if (![...Object.values(ray.origin), ...Object.values(ray.direction)].every(Number.isFinite)
      || V3.length(ray.direction) < 1e-10) return null;
  const direction = V3.normalize(ray.direction);
  const poseMap = new Map(poses.map((pose) => [pose.id, pose]));
  let nearest: PickResult | null = null;

  for (const region of REGION_PRIORITY) {
    for (const segmentId of SEGMENTS_BY_REGION.get(region) ?? []) {
      const pose = poseMap.get(segmentId);
      const definition = SEGMENT_BY_ID.get(segmentId);
      if (!pose || !definition) continue;
      const originLocal = inverseRotate(pose.rotation, V3.sub(ray.origin, pose.position));
      const directionLocal = inverseRotate(pose.rotation, direction);
      const distance = raycastConvex(definition.geometry, originLocal, directionLocal);
      if (distance === null || !Number.isFinite(distance)
          || (nearest && distance >= nearest.distance - TIE_EPSILON_M)) continue;
      nearest = {
        region,
        segment: segmentId,
        distance,
        worldPoint: V3.add(ray.origin, V3.scale(direction, distance)),
        localAnchor: V3.add(originLocal, V3.scale(directionLocal, distance)),
      };
    }
  }
  return nearest;
}
