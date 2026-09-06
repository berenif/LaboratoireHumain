import { SEGMENT_BY_ID } from "../core/humanoid";
import { V3 } from "../core/math";
import type { PickResult, Quat, Ray, RegionId, SegmentPose, SegmentShape, Vec3 } from "../core/types";

// Surface distance always wins. Priority only resolves numerically coincident
// intersections, so an enlarged pelvis proxy cannot steal a visible hand.
const REGION_PRIORITY: ReadonlyArray<RegionId> = [
  "leftHand", "rightHand", "leftFoot", "rightFoot", "head", "torso", "pelvis",
];
const TIE_EPSILON_M = 1e-7;

function inverseRotate(q: Quat, v: Vec3): Vec3 {
  const qv = { x: -q.x, y: -q.y, z: -q.z };
  const uv = V3.cross(qv, v);
  return V3.add(v, V3.add(V3.scale(uv, 2 * q.w), V3.scale(V3.cross(qv, uv), 2)));
}

function sphereHit(origin: Vec3, direction: Vec3, radius: number): number | null {
  const b = V3.dot(origin, direction);
  const discriminant = b * b - V3.dot(origin, origin) + radius * radius;
  if (discriminant < 0) return null;
  const root = Math.sqrt(discriminant);
  const near = -b - root;
  const far = -b + root;
  return near >= 0 ? near : far >= 0 ? far : null;
}

function shapeHit(origin: Vec3, direction: Vec3, shape: SegmentShape): number | null {
  if (shape.kind === "sphere") return sphereHit(origin, direction, shape.radius);
  if (shape.kind === "box") {
    let near = -Infinity;
    let far = Infinity;
    for (const axis of ["x", "y", "z"] as const) {
      const extent = shape.halfExtents[axis];
      if (Math.abs(direction[axis]) < 1e-10) {
        if (Math.abs(origin[axis]) > extent) return null;
        continue;
      }
      const t1 = (-extent - origin[axis]) / direction[axis];
      const t2 = (extent - origin[axis]) / direction[axis];
      near = Math.max(near, Math.min(t1, t2));
      far = Math.min(far, Math.max(t1, t2));
      if (near > far) return null;
    }
    return near >= 0 ? near : far >= 0 ? far : null;
  }
  // Capsule is a finite local-Y cylinder with two spherical end caps.
  const hits: number[] = [];
  const a = direction.x * direction.x + direction.z * direction.z;
  const b = origin.x * direction.x + origin.z * direction.z;
  const c = origin.x * origin.x + origin.z * origin.z - shape.radius * shape.radius;
  const discriminant = b * b - a * c;
  if (a > 1e-12 && discriminant >= 0) {
    for (const t of [(-b - Math.sqrt(discriminant)) / a, (-b + Math.sqrt(discriminant)) / a]) {
      if (t >= 0 && Math.abs(origin.y + direction.y * t) <= shape.halfHeight) hits.push(t);
    }
  }
  for (const sign of [-1, 1]) {
    const t = sphereHit({ ...origin, y: origin.y - sign * shape.halfHeight }, direction, shape.radius);
    if (t !== null && sign * (origin.y + direction.y * t) >= shape.halfHeight) hits.push(t);
  }
  return hits.length ? Math.min(...hits) : null;
}

/** Pick the same oriented primitive surface that both renderers display. */
export function pickRegionProxies(ray: Ray, poses: ReadonlyArray<SegmentPose>): PickResult | null {
  if (![...Object.values(ray.origin), ...Object.values(ray.direction)].every(Number.isFinite)
      || V3.length(ray.direction) < 1e-10) return null;
  const direction = V3.normalize(ray.direction);
  const poseMap = new Map(poses.map((pose) => [pose.id, pose]));
  let nearest: PickResult | null = null;
  for (const region of REGION_PRIORITY) {
    const pose = poseMap.get(region);
    const definition = SEGMENT_BY_ID.get(region);
    if (!pose || !definition) continue;
    const originLocal = inverseRotate(pose.rotation, V3.sub(ray.origin, pose.position));
    const directionLocal = inverseRotate(pose.rotation, direction);
    const distance = shapeHit(originLocal, directionLocal, definition.shape);
    if (distance === null || !Number.isFinite(distance)
        || (nearest && distance >= nearest.distance - TIE_EPSILON_M)) continue;
    nearest = {
      region,
      segment: pose.id,
      distance,
      worldPoint: V3.add(ray.origin, V3.scale(direction, distance)),
      localAnchor: V3.add(originLocal, V3.scale(directionLocal, distance)),
    };
  }
  return nearest;
}
