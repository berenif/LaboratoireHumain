import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";
import { add, clamp, cross, dot, length, normalize, quatFromAxisAngle, quatFromTo, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";
import { solveTwoBone } from "./pose";
import type { RecoverySide } from "./recovery-support";

const UP: Vec3 = { x: 0, y: 1, z: 0 }, RIGHT: Vec3 = { x: 1, y: 0, z: 0 }, FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const rotation = (a: Vec3): Quat => quatMultiply(quatMultiply(quatFromAxisAngle(UP, a.y), quatFromAxisAngle(RIGHT, a.x)), quatFromAxisAngle(FORWARD, a.z));

export interface RecoveryFootTarget {
  /** Foot box centre and world rotation; dynamic bodies remain untouched. */
  position: Vec3;
  rotation: Quat;
  hip: Vec3;
  knee: Vec3;
  ankle: Vec3;
  bend: Vec3;
  travelM: number;
  feasible: boolean;
  floorReachable: boolean;
  reachErrorM: number;
  floorClearanceM: number;
  jointLimitErrorRad: number;
  /** Legal relative rotations, suitable for bounded torque targets. */
  jointRotations: { thigh: Quat; shin: Quat; foot: Quat };
}

function boundedAngles(q: Quat, limit: Vec3): { angles: Vec3; error: number } {
  const x = Math.asin(clamp(2 * (q.w * q.x - q.y * q.z), -1, 1));
  const y = Math.atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.x * q.x + q.y * q.y));
  const z = Math.atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z));
  const wrap = (angle: number) => Math.atan2(Math.sin(angle), Math.cos(angle));
  const candidates = [{ x, y, z }, { x: x >= 0 ? Math.PI - x : -Math.PI - x, y: wrap(y + Math.PI), z: wrap(z + Math.PI) }];
  const error = (a: Vec3) => Math.hypot(Math.max(0, Math.abs(a.x) - limit.x), Math.max(0, Math.abs(a.y) - limit.y), Math.max(0, Math.abs(a.z) - limit.z));
  const chosen = error(candidates[1]) < error(candidates[0]) - 1e-10 ? candidates[1] : candidates[0];
  return { angles: { x: clamp(chosen.x, -limit.x, limit.x), y: clamp(chosen.y, -limit.y, limit.y), z: clamp(chosen.z, -limit.z, limit.z) }, error: error(chosen) };
}

/** Nearest legal floor placement from the measured hip, with an explicit unreachable fallback. */
export function reachableFootTarget(side: RecoverySide, poses: ReadonlyMap<SegmentId, SegmentPose>, heading: number): RecoveryFootTarget | null {
  const pelvis = poses.get("pelvis"), measuredFoot = poses.get(`${side}Foot`);
  if (!pelvis || !measuredFoot) return null;
  const thigh = SEGMENT_BY_ID.get(`${side}Thigh`)!, shin = SEGMENT_BY_ID.get(`${side}Shin`)!, foot = SEGMENT_BY_ID.get(`${side}Foot`)!;
  const hip = worldPoint(pelvis.position, pelvis.rotation, thigh.jointAnchorParent!);
  const firstLength = length(sub(thigh.jointAnchorChild!, shin.jointAnchorParent!));
  const secondLength = length(sub(shin.jointAnchorChild!, foot.jointAnchorParent!));
  const yaw = quatFromAxisAngle(UP, heading), sign = side === "left" ? -1 : 1;
  const outward = rotate(yaw, { x: sign, y: 0, z: 0 }), forward = rotate(yaw, FORWARD);
  const bend = rotate(pelvis.rotation, FORWARD);
  const fromHip = sub(measuredFoot.position, hip);
  // Stance width is measured from the pelvis heading frame; offsets from each
  // actual hip retain its measured roll/pitch displacement.
  const hipWidth = HUMAN_PROPORTIONS.pelvis.hipAnchorXM;
  const nearestWidth = clamp(dot(fromHip, outward) + hipWidth, 0.10, 0.20);
  const nearestForward = clamp(dot(fromHip, forward), 0, 0.40);
  const widths = [nearestWidth, 0.10, 0.12, 0.15, 0.18, 0.20];
  const forwards = [nearestForward, ...Array.from({ length: 41 }, (_, index) => index / 100)];
  const half = HUMAN_PROPORTIONS.foot.halfExtentsM;
  const boxExtent = (q: Quat) => Math.abs(rotate(q, RIGHT).y) * half.x + Math.abs(rotate(q, UP).y) * half.y + Math.abs(rotate(q, FORWARD).y) * half.z;
  let best: RecoveryFootTarget | null = null, bestScore = Infinity;
  for (const width of widths) for (const forwardOffset of forwards) {
    const horizontal = add(hip, add(scale(outward, width - hipWidth), scale(forward, forwardOffset)));
    const requestedPosition = { ...horizontal, y: half.y };
    const requestedAnkle = worldPoint(requestedPosition, yaw, foot.jointAnchorChild!);
    const solved = solveTwoBone(hip, requestedAnkle, firstLength, secondLength, bend);
    const axisA = normalize(sub(hip, solved.middle)), axisB = normalize(sub(solved.middle, solved.end));
    const hinge = normalize(cross(axisA, axisB), rotate(yaw, RIGHT));
    const base = quatFromTo(UP, axisA), baseX = rotate(base, RIGHT);
    const twist = Math.atan2(dot(axisA, cross(baseX, hinge)), dot(baseX, hinge));
    const desiredThigh = quatMultiply(quatFromAxisAngle(axisA, twist), base);
    const desiredKnee = Math.acos(clamp(dot(axisA, axisB), -1, 1));
    const desiredShin = quatMultiply(desiredThigh, quatFromAxisAngle(RIGHT, desiredKnee));
    const thighLimit = boundedAngles(quatMultiply(quatInverse(pelvis.rotation), desiredThigh), thigh.jointLimitRadians!);
    const ankleLimit = boundedAngles(quatMultiply(quatInverse(desiredShin), yaw), foot.jointLimitRadians!);
    const kneeAngle = clamp(desiredKnee, 0.025, shin.jointLimitRadians!.x);
    const jointLimitErrorRad = thighLimit.error + ankleLimit.error + Math.abs(kneeAngle - desiredKnee);
    const jointRotations = { thigh: rotation(thighLimit.angles), shin: quatFromAxisAngle(RIGHT, kneeAngle), foot: rotation(ankleLimit.angles) };
    // Reconstruct even the fallback from legal local rotations. Returning the
    // requested floor point after a clamped solve would conceal an unreachable target.
    const thighRotation = quatMultiply(pelvis.rotation, jointRotations.thigh);
    const shinRotation = quatMultiply(thighRotation, jointRotations.shin);
    const footRotation = quatMultiply(shinRotation, jointRotations.foot);
    const knee = add(hip, rotate(thighRotation, { x: 0, y: -firstLength, z: 0 }));
    const ankle = add(knee, rotate(shinRotation, { x: 0, y: -secondLength, z: 0 }));
    const position = sub(ankle, rotate(footRotation, foot.jointAnchorChild!));
    const reachErrorM = length(sub(position, requestedPosition));
    const thighExtent = HUMAN_PROPORTIONS.leg.thighRadiusM + (firstLength / 2 - HUMAN_PROPORTIONS.leg.thighRadiusM) * Math.abs(rotate(thighRotation, UP).y);
    const shinExtent = HUMAN_PROPORTIONS.leg.shinRadiusM + (secondLength / 2 - HUMAN_PROPORTIONS.leg.shinRadiusM) * Math.abs(rotate(shinRotation, UP).y);
    const floorClearanceM = Math.min(position.y - boxExtent(footRotation), (hip.y + knee.y) / 2 - thighExtent, (knee.y + ankle.y) / 2 - shinExtent);
    const floorReachable = reachErrorM < 1e-6 && floorClearanceM >= -1e-6;
    const feasible = floorReachable && jointLimitErrorRad < 1e-7;
    const travelM = length(sub(position, measuredFoot.position));
    const score = feasible ? travelM : 10 + reachErrorM * 5 + Math.max(0, -floorClearanceM) * 10 + jointLimitErrorRad + travelM * 0.05;
    if (!best || feasible && !best.feasible || feasible === best.feasible && score < bestScore - 1e-9) {
      bestScore = score;
      best = { position, rotation: footRotation, hip, knee, ankle, bend, travelM, feasible, floorReachable, reachErrorM, floorClearanceM, jointLimitErrorRad, jointRotations };
    }
  }
  return best;
}
