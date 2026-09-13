import { lowestWorldPoint } from "../core/geometry";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";
import {
  add,
  clamp,
  cross,
  dot,
  length,
  normalize,
  quatFromAxisAngle,
  quatFromTo,
  quatInverse,
  quatMultiply,
  rotate,
  scale,
  sub,
  worldPoint,
} from "./math";
import { solveTwoBone } from "./pose";
import {
  limitRecoveryJoint,
  reconstructRecoveryLimb,
  recoveryJointLimitError,
  recoveryJointRotation,
} from "./recovery-joints";
import type { RecoverySide } from "./recovery-support";

const UP: Vec3 = { x: 0, y: 1, z: 0 };
const RIGHT: Vec3 = { x: 1, y: 0, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };

export interface RecoveryLegJointRotations {
  thigh: Quat;
  shin: Quat;
  ankle: Quat;
  foot: Quat;
  forefoot: Quat;
}

export interface RecoveryFootTarget {
  kind: "floor-plant" | "intermediate-fold" | "blocked";
  parentRotation: Quat;
  /** Hindfoot centre and world rotation; dynamic bodies remain untouched. */
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
  jointRotations: RecoveryLegJointRotations;
}

export interface RecoveryLegTargetGeometry {
  position: Vec3;
  rotation: Quat;
  hip: Vec3;
  knee: Vec3;
  ankle: Vec3;
  floorClearanceM: number;
  reachErrorM: number;
  jointLimitErrorRad: number;
  jointRotations: RecoveryLegJointRotations;
}

function ids(side: RecoverySide): Readonly<{
  thigh: SegmentId;
  shin: SegmentId;
  ankle: SegmentId;
  foot: SegmentId;
  forefoot: SegmentId;
}> {
  return {
    thigh: `${side}Thigh`,
    shin: `${side}Shin`,
    ankle: `${side}Ankle`,
    foot: `${side}Foot`,
    forefoot: `${side}Forefoot`,
  };
}

function jointSpan(parent: SegmentId, child: SegmentId): number {
  const parentAnchor = SEGMENT_BY_ID.get(parent)!.jointProfile!.childFrame.anchor;
  const childAnchor = SEGMENT_BY_ID.get(child)!.jointProfile!.parentFrame.anchor;
  return length(sub(parentAnchor, childAnchor));
}

function rotationDot(a: Quat, b: Quat): number {
  return Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

/** Resolve roll about local +Y so the one-way local +X hinge reaches the distal axis. */
function hingeParentRotation(proximalAxis:Vec3,distalAxis:Vec3,fallbackHingeAxis:Vec3):Quat {
  const up=normalize(proximalAxis,UP);
  const hingeAxis=normalize(cross(up,distalAxis),fallbackHingeAxis);
  const alignUp=quatFromTo(UP,up);
  const baseHinge=rotate(alignUp,RIGHT);
  const roll=Math.atan2(dot(cross(baseHinge,hingeAxis),up),dot(baseHinge,hingeAxis));
  return quatMultiply(quatFromAxisAngle(up,roll),alignUp);
}

function rotationsMap(side: RecoverySide, rotations: RecoveryLegJointRotations): Map<SegmentId, Quat> {
  const chain = ids(side);
  return new Map<SegmentId, Quat>([
    [chain.thigh, rotations.thigh],
    [chain.shin, rotations.shin],
    [chain.ankle, rotations.ankle],
    [chain.foot, rotations.foot],
    [chain.forefoot, rotations.forefoot],
  ]);
}

/**
 * Solve a requested hindfoot pose through hip, knee, ankle, foot tilt, and toe
 * joints. Every result is reconstructed from clamped joint coordinates.
 */
export function solveRecoveryLegTarget(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  requestedPosition: Vec3,
  requestedRotation: Quat,
  bend?: Vec3,
): RecoveryLegTargetGeometry | null {
  const chain = ids(side);
  const pelvis = poses.get("pelvis");
  if (!pelvis) return null;
  const thighDefinition = SEGMENT_BY_ID.get(chain.thigh)!;
  const shinDefinition = SEGMENT_BY_ID.get(chain.shin)!;
  const ankleDefinition = SEGMENT_BY_ID.get(chain.ankle)!;
  const footDefinition = SEGMENT_BY_ID.get(chain.foot)!;
  const hip = worldPoint(
    pelvis.position,
    pelvis.rotation,
    thighDefinition.jointProfile!.parentFrame.anchor,
  );
  const firstLength = jointSpan(chain.thigh, chain.shin);
  const secondLength = jointSpan(chain.shin, chain.ankle);
  const bendDirection = bend ?? rotate(pelvis.rotation, FORWARD);
  const headingForward=rotate(requestedRotation,FORWARD);
  const headingRight=rotate(requestedRotation,RIGHT);
  const kneeAxis=shinDefinition.jointProfile!.axes.find(axis=>axis.coordinate==="x")!;
  const maximumBend=Math.max(Math.abs(kneeAxis.minRadians),Math.abs(kneeAxis.maxRadians));
  let ankleWorld=requestedRotation,footWorld=requestedRotation;
  let result: RecoveryLegTargetGeometry | null = null;

  for (let iteration = 0; iteration < 20; iteration += 1) {
    // Work backward from the requested hindfoot centre through both distal
    // joints to the exact point solved by the thigh/shin chain.
    const footJoint=worldPoint(requestedPosition,footWorld,
      footDefinition.jointProfile!.childFrame.anchor);
    const anklePosition=sub(footJoint,rotate(ankleWorld,
      footDefinition.jointProfile!.parentFrame.anchor));
    const requestedAnkle=worldPoint(anklePosition,ankleWorld,
      ankleDefinition.jointProfile!.childFrame.anchor);
    const solved = solveTwoBone(
      hip, requestedAnkle, firstLength, secondLength, bendDirection, maximumBend,
    );
    const axisA = normalize(sub(hip, solved.middle));
    const axisB = normalize(sub(solved.middle, solved.end));
    const desiredThighWorld = hingeParentRotation(axisA,axisB,headingRight);
    const desiredKnee = Math.acos(clamp(dot(axisA, axisB), -1, 1));

    const rawThigh = quatMultiply(quatInverse(pelvis.rotation), desiredThighWorld);
    const thigh = limitRecoveryJoint(chain.thigh, rawThigh);
    const thighWorld = quatMultiply(pelvis.rotation, thigh);
    const rawShin = quatFromAxisAngle(RIGHT, desiredKnee);
    const shin = limitRecoveryJoint(chain.shin, rawShin);
    const shinWorld = quatMultiply(thighWorld, shin);
    const actualShinAxis=rotate(shinWorld,UP);
    const ankleFlexion=-Math.atan2(dot(actualShinAxis,headingForward),dot(actualShinAxis,UP));
    const rawAnkle=quatFromAxisAngle(RIGHT,ankleFlexion);
    const ankle = limitRecoveryJoint(chain.ankle, rawAnkle);
    ankleWorld = quatMultiply(shinWorld, ankle);
    const ankleUp=rotate(ankleWorld,UP);
    const footTilt=Math.atan2(dot(ankleUp,headingRight),dot(ankleUp,UP));
    const rawFoot=quatFromAxisAngle(FORWARD,footTilt);
    const foot = limitRecoveryJoint(chain.foot, rawFoot);
    footWorld=quatMultiply(ankleWorld,foot);
    const forefoot = recoveryJointRotation(chain.forefoot, ZERO);
    const jointRotations = { thigh, shin, ankle, foot, forefoot };
    const rebuilt = reconstructRecoveryLimb(side, false, pelvis, rotationsMap(side, jointRotations));
    const thighPose = rebuilt.poses.find(({ id }) => id === chain.thigh)!;
    const shinPose = rebuilt.poses.find(({ id }) => id === chain.shin)!;
    const footPose = rebuilt.poses.find(({ id }) => id === chain.foot)!;
    const knee = worldPoint(
      thighPose.position,
      thighPose.rotation,
      SEGMENT_BY_ID.get(chain.shin)!.jointProfile!.parentFrame.anchor,
    );
    const anklePoint = worldPoint(
      shinPose.position,
      shinPose.rotation,
      SEGMENT_BY_ID.get(chain.ankle)!.jointProfile!.parentFrame.anchor,
    );
    const reachError = sub(requestedPosition, footPose.position);
    result = {
      position: footPose.position,
      rotation: footPose.rotation,
      hip,
      knee,
      ankle: anklePoint,
      floorClearanceM: rebuilt.floorClearanceM,
      reachErrorM: length(reachError),
      jointLimitErrorRad: recoveryJointLimitError(chain.thigh, rawThigh)
        + recoveryJointLimitError(chain.shin, rawShin)
        + recoveryJointLimitError(chain.ankle, rawAnkle)
        + recoveryJointLimitError(chain.foot, rawFoot),
      jointRotations,
    };
    if (result.reachErrorM < 1e-10) break;
  }
  return result;
}

/** Nearest legal floor placement from the measured hip. */
export function reachableFootTarget(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  heading: number,
  preferred?: Vec3,
): RecoveryFootTarget | null {
  const pelvis = poses.get("pelvis");
  const measuredFoot = poses.get(`${side}Foot`);
  if (!pelvis || !measuredFoot) return null;
  const chain = ids(side);
  const thigh = SEGMENT_BY_ID.get(chain.thigh)!;
  const foot = SEGMENT_BY_ID.get(chain.foot)!;
  const hip = worldPoint(pelvis.position, pelvis.rotation, thigh.jointProfile!.parentFrame.anchor);
  const yaw = quatFromAxisAngle(UP, heading);
  const sign = side === "left" ? -1 : 1;
  const outward = rotate(yaw, { x: sign, y: 0, z: 0 });
  const forward = rotate(yaw, FORWARD);
  const bend = rotate(pelvis.rotation, FORWARD);
  const fromHip = sub(preferred ?? measuredFoot.position, hip);
  const hipWidth = HUMAN_PROPORTIONS.pelvis.hipAnchorXM;
  const nearestWidth = clamp(dot(fromHip, outward) + hipWidth, 0.10, 0.20);
  const nearestForward = clamp(dot(fromHip, forward), -0.40, 0.10);
  const widths = [nearestWidth, 0.10, 0.12, 0.15, 0.18, 0.20];
  const forwards = [nearestForward, ...Array.from({ length: 51 }, (_, index) => -0.40 + index / 100)];
  const flatFloorOffset = -lowestWorldPoint(foot.geometry, ZERO, yaw).y;
  let best: RecoveryFootTarget | null = null;
  let bestScore = Infinity;

  for (const width of widths) for (const forwardOffset of forwards) {
    const horizontal = add(hip, add(
      scale(outward, width - hipWidth),
      scale(forward, forwardOffset),
    ));
    const requestedPosition = { ...horizontal, y: flatFloorOffset };
    const solved = solveRecoveryLegTarget(side, poses, requestedPosition, yaw, bend);
    if (!solved) continue;
    const floorReachable = solved.reachErrorM < 0.003 && solved.floorClearanceM >= -0.003;
    const feasible = floorReachable && solved.jointLimitErrorRad < 1e-5;
    const travelM = length(sub(solved.position, measuredFoot.position));
    const placementCost = preferred ? length(sub(solved.position, preferred)) + 0.01 * travelM : travelM;
    const score = feasible
      ? placementCost
      : 10 + solved.reachErrorM * 5 + Math.max(0, -solved.floorClearanceM) * 10
        + solved.jointLimitErrorRad + placementCost * 0.05;
    if (!best || feasible && !best.feasible || feasible === best.feasible && score < bestScore - 1e-9) {
      bestScore = score;
      best = {
        kind: feasible ? "floor-plant" : solved.floorClearanceM >= -0.003 ? "intermediate-fold" : "blocked",
        parentRotation: { ...pelvis.rotation },
        position: solved.position,
        rotation: solved.rotation,
        hip: solved.hip,
        knee: solved.knee,
        ankle: solved.ankle,
        bend,
        travelM,
        feasible,
        floorReachable,
        reachErrorM: solved.reachErrorM,
        floorClearanceM: solved.floorClearanceM,
        jointLimitErrorRad: solved.jointLimitErrorRad,
        jointRotations: solved.jointRotations,
      };
    }
  }
  return best;
}

/** Revalidate retained rotations against parent movement and exact geometry. */
export function revalidateRecoveryFootTarget(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  target: RecoveryFootTarget,
): boolean {
  const pelvis = poses.get("pelvis");
  if (!pelvis || target.kind === "blocked") return false;
  const rebuilt = reconstructRecoveryLimb(
    side,
    false,
    pelvis,
    rotationsMap(side, target.jointRotations),
  );
  const foot = rebuilt.poses.find(({ id }) => id === `${side}Foot`)!;
  return rebuilt.floorClearanceM >= -0.003
    && length(sub(foot.position, target.position)) < 0.02
    && rotationDot(foot.rotation, target.rotation) > Math.cos(0.03 / 2);
}
