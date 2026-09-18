import { SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, Vec3 } from "../core/types";
import { length, rotate, sub, worldPoint } from "./math";

export type LegSide = "left" | "right";
export type HindfootId = "leftFoot" | "rightFoot";

/** All balance destinations are WORLD hindfoot centres, not whole-foot centres. */
export const LEG_TARGET_FRAME = "world-hindfoot-center" as const;

function definition(id: SegmentId) {
  const value = SEGMENT_BY_ID.get(id);
  if (!value?.jointAnchorChild || !value.jointAnchorParent) {
    throw new Error(`Missing leg anchors: ${id}`);
  }
  return value;
}

export function legReachRadius(side: LegSide): number {
  const thigh = definition(`${side}Thigh`);
  const shin = definition(`${side}Shin`);
  const ankle = definition(`${side}Ankle`);
  // Match solveTwoBone's extension margin; geometry, not duplicated dimensions.
  return length(sub(thigh.jointAnchorChild!, shin.jointAnchorParent!))
    + length(sub(shin.jointAnchorChild!, ankle.jointAnchorParent!)) - 0.002;
}

/** Back-project a hindfoot centre through foot and ankle anchors to the shin tip. */
export function ankleFromHindfoot(
  side: LegSide,
  center: Vec3,
  footRotation: Quat,
  ankleRotation: Quat = footRotation,
): Vec3 {
  const foot = definition(`${side}Foot`);
  const ankle = definition(`${side}Ankle`);
  const footJoint = worldPoint(center, footRotation, foot.jointAnchorChild!);
  const ankleCenter = sub(footJoint, rotate(ankleRotation, foot.jointAnchorParent!));
  return worldPoint(ankleCenter, ankleRotation, ankle.jointAnchorChild!);
}

export function hindfootFromAnkle(
  side: LegSide,
  anklePoint: Vec3,
  footRotation: Quat,
  ankleRotation: Quat = footRotation,
): Vec3 {
  const foot = definition(`${side}Foot`);
  const ankle = definition(`${side}Ankle`);
  const ankleCenter = sub(anklePoint, rotate(ankleRotation, ankle.jointAnchorChild!));
  const footJoint = worldPoint(ankleCenter, ankleRotation, foot.jointAnchorParent!);
  return sub(footJoint, rotate(footRotation, foot.jointAnchorChild!));
}

export function legTargetReach(
  side: LegSide,
  hip: Vec3,
  hindfoot: Vec3,
  rotation: Quat,
) {
  const requestedAnkle = ankleFromHindfoot(side, hindfoot, rotation);
  const radiusM = legReachRadius(side);
  const distanceM = length(sub(requestedAnkle, hip));
  return {
    requestedAnkle,
    radiusM,
    distanceM,
    excessM: Math.max(0, distanceM - radiusM),
    // Radial feasibility alone is not proof of joint-limit feasibility/touchdown.
    radiallyReachable: distanceM <= radiusM + 1e-8,
  };
}
