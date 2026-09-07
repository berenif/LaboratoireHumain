import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type { Quat, RegionId, SegmentId, SegmentPose, Vec3 } from "../core/types";
import {
  add, clamp, clampLength, cross, dot, length, lerp, normalize,
  quatFromAxisAngle, quatFromTo, quatMultiply, rotate, scale, smooth01, sub, worldPoint,
} from "./math";

const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };

export type MutablePose = {
  id: SegmentId;
  position: Vec3;
  rotation: Quat;
  linearVelocity: Vec3;
  angularVelocity: Vec3;
};

export type StepMotion = {
  foot: "leftFoot" | "rightFoot";
  from: Vec3;
  to: Vec3;
  elapsed: number;
  duration: number;
};

export type LegSide = "left" | "right";

/** Inputs to the kinematic pose compositor; no physics or DOM ownership. */
export interface UprightPoseInput {
  rootTranslation: Vec3;
  reactionOffset: Vec3;
  simulationTime: number;
  activeGrab: {
    region: RegionId;
    target: Vec3;
    startTarget: Vec3;
    startSegmentPosition: Vec3;
  } | null;
  supportFeet: Readonly<Record<"leftFoot" | "rightFoot", Vec3>>;
  step: StepMotion | null;
  /** World yaw preserved across dynamic recovery and non-default starts. */
  heading?: number;
  /** Bounded pelvis lowering; the leg solver supplies the matching knee bend. */
  kneeFlexion?: number;
}

export interface UprightPoseResult {
  poses: Map<SegmentId, MutablePose>;
  leanRadians: number;
}

export function immutablePose(pose: MutablePose): SegmentPose {
  return {
    id: pose.id,
    position: { ...pose.position },
    rotation: { ...pose.rotation },
    linearVelocity: { ...pose.linearVelocity },
    angularVelocity: { ...pose.angularVelocity },
  };
}

export function restPoseMap(): Map<SegmentId, MutablePose> {
  const poses = new Map<SegmentId, MutablePose>();
  for (const definition of SEGMENTS) {
    const parent = definition.parent ? poses.get(definition.parent) : null;
    const rotation = parent
      ? quatMultiply(parent.rotation, definition.restLocalRotation)
      : definition.restLocalRotation;
    const position = parent && definition.jointAnchorParent && definition.jointAnchorChild
      ? sub(poseAnchor(parent, definition.jointAnchorParent), rotate(rotation, definition.jointAnchorChild))
      : definition.localOffset;
    poses.set(definition.id, {
      id: definition.id,
      position,
      rotation,
      linearVelocity: ZERO,
      angularVelocity: ZERO,
    });
  }
  return poses;
}

function makePose(id: SegmentId, position: Vec3, rotation: Quat = IDENTITY): MutablePose {
  return { id, position, rotation, linearVelocity: ZERO, angularVelocity: ZERO };
}

export function poseAnchor(pose: MutablePose, anchor: Vec3): Vec3 {
  return worldPoint(pose.position, pose.rotation, anchor);
}

function solveTwoBone(
  start: Vec3,
  requestedEnd: Vec3,
  firstLength: number,
  secondLength: number,
  preferredBend: Vec3,
): Readonly<{ middle: Vec3; end: Vec3 }> {
  const raw = sub(requestedEnd, start);
  const direction = normalize(raw, { x: 0, y: -1, z: 0 });
  const minimum = Math.abs(firstLength - secondLength) + 0.005;
  const maximum = firstLength + secondLength - 0.002;
  const distance = clamp(length(raw), minimum, maximum);
  const end = add(start, scale(direction, distance));
  const along = (firstLength * firstLength - secondLength * secondLength + distance * distance) / (2 * distance);
  const bendHeight = Math.sqrt(Math.max(0, firstLength * firstLength - along * along));
  let bend = sub(preferredBend, scale(direction, dot(preferredBend, direction)));
  if (length(bend) < 1e-4) bend = cross(direction, { x: 1, y: 0, z: 0 });
  bend = normalize(bend, FORWARD);
  return { middle: add(add(start, scale(direction, along)), scale(bend, bendHeight)), end };
}

export function midpoint(a: Vec3, b: Vec3): Vec3 {
  return scale(add(a, b), 0.5);
}

export function horizontal(v: Vec3): Vec3 {
  return { x: v.x, y: 0, z: v.z };
}

export function composeUprightPose(input: UprightPoseInput): UprightPoseResult {
  const output = new Map<SegmentId, MutablePose>();
  const rootTranslation = input.rootTranslation;
  const root: Vec3 = { x: rootTranslation.x, y: rootTranslation.y - clamp(input.kneeFlexion ?? 0, 0, 1) * 0.10, z: rootTranslation.z };
  const heading = quatFromAxisAngle(UP, input.heading ?? 0);
  const drag = input.activeGrab ? sub(input.activeGrab.target, input.activeGrab.startTarget) : ZERO;
  const horizontalReaction = horizontal(input.reactionOffset);
  const leanRadians = Math.atan(length(horizontalReaction) * 0.38);
  const pelvisTilt = quatMultiply(quatFromTo(UP, normalize({
    x: horizontalReaction.x * 0.25,
    y: 1,
    z: horizontalReaction.z * 0.25,
  })), heading);
  const torsoTilt = quatMultiply(quatFromTo(UP, normalize({
    x: horizontalReaction.x * 0.38,
    y: 1,
    z: horizontalReaction.z * 0.38,
  })), heading);

  const pelvis = makePose("pelvis", root, pelvisTilt);
  output.set("pelvis", pelvis);
  const torsoDefinition = SEGMENT_BY_ID.get("torso")!;
  const torsoJoint = poseAnchor(pelvis, torsoDefinition.jointAnchorParent!);
  const torso = makePose(
    "torso",
    sub(torsoJoint, rotate(torsoTilt, torsoDefinition.jointAnchorChild!)),
    torsoTilt,
  );
  output.set("torso", torso);

  const headDrag = input.activeGrab?.region === "head" ? clampLength(drag, 0.42) : ZERO;
  const headTilt = quatMultiply(quatFromTo(UP, normalize({
    x: horizontalReaction.x * 0.45 + headDrag.x * 1.05,
    y: 1 + headDrag.y * 0.2,
    z: horizontalReaction.z * 0.45 + headDrag.z * 1.05,
  })), heading);
  const neckDefinition = SEGMENT_BY_ID.get("neck")!;
  const neckJoint = poseAnchor(torso, neckDefinition.jointAnchorParent!);
  const neck = makePose("neck", sub(neckJoint, rotate(torsoTilt, neckDefinition.jointAnchorChild!)), torsoTilt);
  output.set("neck", neck);
  const headDefinition = SEGMENT_BY_ID.get("head")!;
  const headJoint = poseAnchor(neck, headDefinition.jointAnchorParent!);
  const head = makePose("head", sub(headJoint, rotate(headTilt, headDefinition.jointAnchorChild!)), headTilt);
  output.set("head", head);

  composeArm("left", torso, output, drag, input);
  composeArm("right", torso, output, drag, input);
  composeLeg("left", pelvis, output, drag, input);
  composeLeg("right", pelvis, output, drag, input);
  return { poses: output, leanRadians };
}

function composeArm(
  side: LegSide,
  torso: MutablePose,
  output: Map<SegmentId, MutablePose>,
  drag: Vec3,
  input: UprightPoseInput,
): void {
  const sign = side === "left" ? -1 : 1;
  const upperId = `${side}UpperArm` as SegmentId;
  const forearmId = `${side}Forearm` as SegmentId;
  const handId = `${side}Hand` as RegionId;
  const upperDefinition = SEGMENT_BY_ID.get(upperId)!;
  const forearmDefinition = SEGMENT_BY_ID.get(forearmId)!;
  const handDefinition = SEGMENT_BY_ID.get(handId)!;
  const shoulder = poseAnchor(torso, upperDefinition.jointAnchorParent!);
  const upperLength = length(sub(upperDefinition.jointAnchorChild!, forearmDefinition.jointAnchorParent!));
  const forearmLength = length(sub(forearmDefinition.jointAnchorChild!, handDefinition.jointAnchorParent!));
  const handCenterOffset = length(handDefinition.jointAnchorChild!);
  const idle = Math.sin(input.simulationTime * 1.7 + (side === "left" ? 0 : Math.PI)) * 0.018;
  let desiredHand: Vec3 = add(shoulder, rotate(torso.rotation, {
    x: sign * HUMAN_PROPORTIONS.arm.relaxedLateralOffsetM,
    y: -(upperLength + forearmLength + handCenterOffset - HUMAN_PROPORTIONS.arm.relaxedShorteningM),
    z: idle,
  }));
  if (input.activeGrab?.region === handId) {
    desiredHand = add(input.activeGrab.startSegmentPosition, clampLength(drag, 0.72));
  } else {
    const counterbalance = scale(horizontal(input.reactionOffset), -0.52);
    desiredHand = add(desiredHand, { ...counterbalance, y: length(counterbalance) * 0.8 });
  }

  const approximateAxis = normalize(sub(shoulder, desiredHand), UP);
  const requestedWrist = add(desiredHand, scale(approximateAxis, handCenterOffset));
  const solved = solveTwoBone(
    shoulder,
    requestedWrist,
    upperLength,
    forearmLength,
    rotate(quatFromAxisAngle(UP, input.heading ?? 0), normalize({ x: sign * 0.22, y: 0, z: 1 })),
  );
  const upperAxis = normalize(sub(shoulder, solved.middle), UP);
  const forearmAxis = normalize(sub(solved.middle, solved.end), UP);
  const yaw = quatFromAxisAngle(UP, input.heading ?? 0);
  const upperRotation = quatMultiply(quatFromTo(UP, upperAxis), yaw);
  const forearmRotation = quatMultiply(quatFromTo(UP, forearmAxis), yaw);
  const handRotation = forearmRotation;
  const upper = makePose(upperId, sub(shoulder, rotate(upperRotation, upperDefinition.jointAnchorChild!)), upperRotation);
  const forearm = makePose(forearmId, sub(solved.middle, rotate(forearmRotation, forearmDefinition.jointAnchorChild!)), forearmRotation);
  const hand = makePose(handId, sub(solved.end, rotate(handRotation, handDefinition.jointAnchorChild!)), handRotation);
  output.set(upperId, upper);
  output.set(forearmId, forearm);
  output.set(handId, hand);
}

function composeLeg(
  side: LegSide,
  pelvis: MutablePose,
  output: Map<SegmentId, MutablePose>,
  drag: Vec3,
  input: UprightPoseInput,
): void {
  const thighId = `${side}Thigh` as SegmentId;
  const shinId = `${side}Shin` as SegmentId;
  const footId = `${side}Foot` as "leftFoot" | "rightFoot";
  const thighDefinition = SEGMENT_BY_ID.get(thighId)!;
  const shinDefinition = SEGMENT_BY_ID.get(shinId)!;
  const footDefinition = SEGMENT_BY_ID.get(footId)!;
  const hip = poseAnchor(pelvis, thighDefinition.jointAnchorParent!);
  const thighLength = length(sub(thighDefinition.jointAnchorChild!, shinDefinition.jointAnchorParent!));
  const shinLength = length(sub(shinDefinition.jointAnchorChild!, footDefinition.jointAnchorParent!));
  let footPosition = input.supportFeet[footId];

  if (input.step?.foot === footId) {
    const progress = smooth01(input.step.elapsed / input.step.duration);
    const base = lerp(input.step.from, input.step.to, progress);
    const travel = length(horizontal(sub(input.step.to, input.step.from)));
    const clearance = clamp(
      HUMAN_PROPORTIONS.foot.stepClearanceBaseM + travel * HUMAN_PROPORTIONS.foot.stepClearancePerTravel,
      HUMAN_PROPORTIONS.foot.minStepClearanceM,
      HUMAN_PROPORTIONS.foot.maxStepClearanceM,
    );
    footPosition = { ...base, y: base.y + Math.sin(Math.PI * progress) * clearance };
  } else if (input.activeGrab?.region === footId) {
    footPosition = add(input.activeGrab.startSegmentPosition, clampLength(drag, 0.58));
    const soleHeight = footDefinition.shape.kind === "box"
      ? footDefinition.shape.halfExtents.y
      : HUMAN_PROPORTIONS.foot.halfExtentsM.y;
    footPosition = { ...footPosition, y: Math.max(soleHeight + 0.005, footPosition.y) };
  }

  const footRotation = quatFromAxisAngle(UP, input.heading ?? 0);
  const ankle = worldPoint(footPosition, footRotation, footDefinition.jointAnchorChild!);
  const solved = solveTwoBone(hip, ankle, thighLength, shinLength, rotate(footRotation, FORWARD));
  const thighAxis = normalize(sub(hip, solved.middle), UP);
  const shinAxis = normalize(sub(solved.middle, solved.end), UP);
  const thighRotation = quatMultiply(quatFromTo(UP, thighAxis), footRotation);
  const shinRotation = quatMultiply(quatFromTo(UP, shinAxis), footRotation);
  output.set(thighId, makePose(thighId, sub(hip, rotate(thighRotation, thighDefinition.jointAnchorChild!)), thighRotation));
  output.set(shinId, makePose(shinId, sub(solved.middle, rotate(shinRotation, shinDefinition.jointAnchorChild!)), shinRotation));
  // Keep the foot attached to the solved ankle when its target exceeds leg reach.
  const solvedFootPosition = sub(solved.end, rotate(footRotation, footDefinition.jointAnchorChild!));
  output.set(footId, makePose(footId, solvedFootPosition, footRotation));
}
