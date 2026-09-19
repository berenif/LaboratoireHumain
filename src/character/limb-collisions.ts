import RAPIER from "@dimforge/rapier3d-compat";

import { flattenGeometryIndices, flattenGeometryVertices } from "../core/geometry";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, Vec3 } from "../core/types";
import { add, clamp, length, quatInverse, rotate, scale, sub, worldPoint } from "./math";

type Side = "left" | "right";
type Limb = "arm" | "leg";
export interface CollisionPose { id: SegmentId; position: Vec3; rotation: Quat }
export type CollisionPoseMap = ReadonlyMap<SegmentId, CollisionPose>;
export interface TrunkClearance {
  /** Signed separation in metres. Negative values are penetration. */
  clearanceM: number;
  pair: readonly [SegmentId, SegmentId] | null;
}

const TRUNK_IDS = ["torso", "lumbar", "pelvis"] as const satisfies readonly SegmentId[];
const QUERY_DISTANCE_M = 0.025;
const TARGET_CLEARANCE_M = 0.004;
const shapes = new Map<SegmentId, RAPIER.ConvexPolyhedron>();
const boundingRadii = new Map([...SEGMENT_BY_ID].map(([id, definition]) => [
  id,
  Math.max(...definition.geometry.vertices.map(({ x, y, z }) => Math.hypot(x, y, z))),
]));
let shapesReady = false;

/** Call after Rapier.init. The query meshes are the same meshes used by the dynamic colliders. */
export function initializeLimbCollisionQueries(): void {
  if (shapesReady) return;
  for (const [id, definition] of SEGMENT_BY_ID) {
    shapes.set(id, new RAPIER.ConvexPolyhedron(
      flattenGeometryVertices(definition.geometry),
      flattenGeometryIndices(definition.geometry),
    ));
  }
  shapesReady = true;
}

function limbIds(side: Side, limb: Limb): SegmentId[] {
  return limb === "arm"
    ? [`${side}UpperArm`, `${side}Forearm`, `${side}ForearmTwist`, `${side}Hand`]
    : [`${side}Thigh`, `${side}Shin`, `${side}Ankle`, `${side}Foot`, `${side}Forefoot`];
}

function pairClearance(poses: CollisionPoseMap, firstId: SegmentId, secondId: SegmentId): number {
  const first = poses.get(firstId), second = poses.get(secondId);
  if (!first || !second || !shapesReady) return Infinity;
  if (SEGMENT_BY_ID.get(firstId)!.collisionExclusions.includes(secondId)) return Infinity;
  const broadRadius = boundingRadii.get(firstId)! + boundingRadii.get(secondId)! + QUERY_DISTANCE_M;
  if (length(sub(first.position, second.position)) > broadRadius) return Infinity;
  const contact = shapes.get(firstId)!.contactShape(
    first.position, first.rotation,
    shapes.get(secondId)!, second.position, second.rotation,
    QUERY_DISTANCE_M,
  );
  return contact?.distance ?? Infinity;
}

/** Check the completed limb, including joint-clamped distal rotations, against the trunk. */
export function limbTrunkClearance(poses: CollisionPoseMap, side: Side, limb: Limb): TrunkClearance {
  // Pure pose composition is also used without a physics world in a few tests.
  // Runtime creation initializes the exact Rapier convex queries first.
  if (!shapesReady) return { clearanceM: Infinity, pair: null };
  let clearanceM = Infinity;
  let pair: readonly [SegmentId, SegmentId] | null = null;
  for (const limbId of limbIds(side, limb)) {
    const limbPose = poses.get(limbId);
    if (!limbPose) continue;
    const definition = SEGMENT_BY_ID.get(limbId)!;
    for (const trunkId of TRUNK_IDS) {
      if (!poses.has(trunkId) || definition.collisionExclusions.includes(trunkId)) continue;
      const distance = pairClearance(poses, limbId, trunkId);
      if (distance < clearanceM) {
        clearanceM = distance;
        pair = [limbId, trunkId];
      }
    }
  }
  return { clearanceM, pair };
}

/** The active grabbed piece can meet any non-excluded segment, including the opposite limb. */
export function segmentBodyClearance(poses: CollisionPoseMap, segment: SegmentId): TrunkClearance {
  let clearanceM = Infinity;
  let pair: readonly [SegmentId, SegmentId] | null = null;
  for (const other of SEGMENT_BY_ID.keys()) {
    if (other === segment) continue;
    const distance = pairClearance(poses, segment, other);
    if (distance < clearanceM) { clearanceM = distance; pair = [segment, other]; }
  }
  return { clearanceM, pair };
}

/** Preview every enabled collision involving the reconstructed chain. */
export function limbBodyClearance(poses: CollisionPoseMap, side: Side, limb: Limb): TrunkClearance {
  let clearanceM = Infinity;
  let pair: readonly [SegmentId, SegmentId] | null = null;
  const seen = new Set<string>();
  for (const segment of limbIds(side, limb)) for (const other of SEGMENT_BY_ID.keys()) {
    if (segment === other) continue;
    const key = segment < other ? `${segment}|${other}` : `${other}|${segment}`;
    if (seen.has(key)) continue;
    seen.add(key);
    const distance = pairClearance(poses, segment, other);
    if (distance < clearanceM) { clearanceM = distance; pair = [segment, other]; }
  }
  return { clearanceM, pair };
}

export interface CollisionAwareTargetInput<T extends CollisionPoseMap> {
  side: Side;
  limb: Limb;
  start: Vec3;
  requested: Vec3;
  parent: CollisionPose;
  /** Rebuild every chain pose after clamping its joint target to its profile. */
  solve: (target: Vec3) => T;
  clearance?: (target: Vec3, poses: T) => TrunkClearance;
}
export interface CollisionAwareTarget<T extends CollisionPoseMap> {
  target: Vec3;
  poses: T;
  blocked: boolean;
  clearance: TrunkClearance;
}

/**
 * Preview a bounded path around the trunk. These are motor targets only;
 * Rapier remains the sole owner of segment movement and contact response.
 */
export function collisionAwareLimbTarget<T extends CollisionPoseMap>(
  input: CollisionAwareTargetInput<T>,
): CollisionAwareTarget<T> {
  const evaluate = (target: Vec3): CollisionAwareTarget<T> => {
    const poses = input.solve(target);
    return { target, poses, blocked: false,
      clearance: input.clearance?.(target, poses) ?? limbTrunkClearance(poses, input.side, input.limb) };
  };
  const clear = (candidate: CollisionAwareTarget<T>): boolean =>
    candidate.clearance.clearanceM >= TARGET_CLEARANCE_M;
  const requested = evaluate(input.requested);
  if (length(sub(input.requested, input.start)) < 0.02 && clear(requested)) return requested;
  const pathClear = (to: Vec3): boolean => [0.25, 0.5, 0.75, 1].every((t) =>
    clear(evaluate(add(input.start, scale(sub(to, input.start), t)))));
  if (clear(requested) && pathClear(input.requested)) return requested;

  const sign = input.side === "left" ? -1 : 1;
  const localRequested = rotate(quatInverse(input.parent.rotation), sub(input.requested, input.parent.position));
  const p = HUMAN_PROPORTIONS;
  const lateral = input.limb === "arm"
    ? p.torso.halfExtentsM.x + p.arm.upperRadiusM + 0.035
    : p.pelvis.halfExtentsM.x + p.leg.thighRadiusM + 0.03;
  const depth = input.limb === "arm"
    ? p.torso.halfExtentsM.z + p.arm.forearmRadiusM + 0.065
    : p.pelvis.halfExtentsM.z + p.leg.shinRadiusM + 0.07;
  const localHeight = input.limb === "arm"
    ? clamp(localRequested.y, -p.torso.halfExtentsM.y, p.torso.halfExtentsM.y)
    : localRequested.y;
  const waypoints = [depth, -depth].map((z) => worldPoint(
    input.parent.position, input.parent.rotation,
    { x: sign * lateral, y: localHeight, z },
  ));
  const routed = waypoints
    .map(evaluate)
    .filter((candidate) => clear(candidate) && pathClear(candidate.target))
    .sort((a, b) => {
      const difference = length(sub(a.target, input.requested)) - length(sub(b.target, input.requested));
      return Math.abs(difference) < 1e-6 ? 0 : difference;
    });
  if (routed.length) return { ...routed[0], blocked: true };

  // If every route is blocked, stop at the first attainable point along the
  // requested direction. The bisection makes the limit continuous near contact.
  let nearest = evaluate(input.start);
  if (!clear(nearest)) return { ...nearest, blocked: true };
  let lo = 0, hi = 1;
  for (let iteration = 0; iteration < 8; iteration += 1) {
    const fraction = (lo + hi) / 2;
    const candidate = evaluate(add(input.start, scale(sub(input.requested, input.start), fraction)));
    if (clear(candidate)) { lo = fraction; nearest = candidate; }
    else hi = fraction;
  }
  return { ...nearest, blocked: true };
}
