import type { Collider } from "@dimforge/rapier3d-compat";
import { SEGMENT_BY_ID } from "../src/core/humanoid";
import type { PoseSnapshot, SegmentId, Vec3 } from "../src/core/types";

/** Independent QA reduction, not the controller's mass-state helper.
 * Current frames use Rapier directly when colliders are supplied. Previous
 * frames use their captured physical COM/mass. Point-mass sensor fixtures that
 * predate those fields retain their explicitly centered-position convention.
 */
export function measuredPhysicsMass(snapshot: Pick<PoseSnapshot, "segments">,
  colliders?: ReadonlyMap<SegmentId, Collider>): { position: Vec3; velocity: Vec3 } {
  const position = { x: 0, y: 0, z: 0 }, velocity = { x: 0, y: 0, z: 0 };
  let totalMass = 0;
  for (const pose of snapshot.segments) {
    const body = colliders?.get(pose.id)?.parent?.();
    const mass = body?.mass() ?? pose.massKg ?? SEGMENT_BY_ID.get(pose.id)?.massKg;
    const center = body?.worldCom() ?? pose.centerOfMass ?? pose.position;
    const speed = body?.linvel() ?? pose.linearVelocity;
    if (mass === undefined || !Number.isFinite(mass) || mass <= 0
      || ![...Object.values(center), ...Object.values(speed)].every(Number.isFinite)) {
      throw new Error(`Invalid independent mass sample: ${pose.id}`);
    }
    totalMass += mass;
    for (const axis of ["x", "y", "z"] as const) {
      position[axis] += mass * center[axis];
      velocity[axis] += mass * speed[axis];
    }
  }
  if (totalMass <= 0) throw new Error("Independent mass sample is empty");
  for (const axis of ["x", "y", "z"] as const) {
    position[axis] /= totalMass;
    velocity[axis] /= totalMass;
  }
  return { position, velocity };
}
