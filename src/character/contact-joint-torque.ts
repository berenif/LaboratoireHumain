import { SEGMENT_BY_ID } from "../core/humanoid";
import type { SegmentId, Vec3 } from "../core/types";
import { cross, scale, sub } from "./math";

export function isBelowJoint(segment: SegmentId, joint: SegmentId): boolean {
  let current: SegmentId | null = segment;
  while (current) {
    if (current === joint) return true;
    current = SEGMENT_BY_ID.get(current)?.parent ?? null;
  }
  return false;
}

/** One virtual-work mapping for allocation and execution: ground reaction
 * cancels the descendant load through an equal/opposite joint moment. */
export function contactJointTorque(joint: SegmentId, anchor: Vec3,
  segment: SegmentId, point: Vec3, force: Vec3): Vec3 {
  return isBelowJoint(segment, joint) ? cross(sub(point, anchor), scale(force, -1)) : { x: 0, y: 0, z: 0 };
}
