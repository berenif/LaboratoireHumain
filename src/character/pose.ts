import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type { JointCoordinate, Quat, RegionId, SegmentDefinition, SegmentId, SegmentPose, Vec3 } from "../core/types";
import {
  add, clamp, clampLength, cross, dot, length, lerp, normalize,
  quatFromAxisAngle, quatFromTo, quatMultiply, rotate, scale, smooth01, sub, worldPoint,
} from "./math";
import { jointCoordinates, jointRotationFromCoordinates } from "./joint-coordinates";

import { ankleFromHindfoot } from "./leg-target-frame";

const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const RIGHT: Vec3 = { x: 1, y: 0, z: 0 };

export type MutablePose = {
  id: SegmentId;
  massKg?: number;
  centerOfMass?: Vec3;
  position: Vec3;
  rotation: Quat;
  linearVelocity: Vec3;
  angularVelocity: Vec3;
};

export type StepMotion = {
  phase?: "unloading" | "swing" | "touchdown" | "loading";
  foot: "leftFoot" | "rightFoot";
  /** Immutable world-frame planning metadata; optional for external pose fixtures. */
  requested?: Vec3;
  heading?: number;
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
    massKg: pose.massKg,
    centerOfMass: pose.centerOfMass ? { ...pose.centerOfMass } : undefined,
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

function segmentDefinition(id: SegmentId): SegmentDefinition {
  const definition = SEGMENT_BY_ID.get(id);
  if (!definition) throw new Error(`Missing humanoid segment definition: ${id}`);
  return definition;
}

/** Place a child from the shared parent/child joint anchors without assuming its dimensions. */
function attachedPose(id: SegmentId, parent: MutablePose, rotation: Quat): MutablePose {
  const definition = segmentDefinition(id);
  if (!definition.jointAnchorParent || !definition.jointAnchorChild) {
    throw new Error(`Connected segment ${id} is missing a joint anchor`);
  }
  const joint = poseAnchor(parent, definition.jointAnchorParent);
  return makePose(id, sub(joint, rotate(rotation, definition.jointAnchorChild)), rotation);
}

/** Length of a segment between the joint that owns it and the next child joint. */
function jointSpan(id: SegmentId, childId: SegmentId): number {
  const definition = segmentDefinition(id);
  const childDefinition = segmentDefinition(childId);
  if (!definition.jointAnchorChild || !childDefinition.jointAnchorParent) {
    throw new Error(`Cannot measure joint span ${id} -> ${childId}`);
  }
  return length(sub(definition.jointAnchorChild, childDefinition.jointAnchorParent));
}

/** Keep generated targets comfortably within the same profile used by the physics joint. */
function boundedJointTarget(
  definition: SegmentDefinition,
  coordinate: JointCoordinate,
  requested: number,
  fallbackLimit: number,
  utilization = 0.78,
): number {
  const axis = definition.jointProfile?.axes.find((candidate) => candidate.coordinate === coordinate);
  const minimum = axis ? axis.minRadians * utilization : -fallbackLimit;
  const maximum = axis ? axis.maxRadians * utilization : fallbackLimit;
  return clamp(requested, Math.min(minimum, maximum), Math.max(minimum, maximum));
}

function jointTargetRotation(
  parentRotation: Quat,
  definition: SegmentDefinition,
  coordinates: Vec3,
): Quat {
  if (definition.jointProfile) {
    return quatMultiply(
      parentRotation,
      jointRotationFromCoordinates(coordinates, definition.jointProfile),
    );
  }
  const localTarget = quatMultiply(
    quatFromAxisAngle(RIGHT, coordinates.x),
    quatMultiply(
      quatFromAxisAngle(UP, coordinates.y),
      quatFromAxisAngle(FORWARD, coordinates.z),
    ),
  );
  return quatMultiply(parentRotation, quatMultiply(definition.restLocalRotation, localTarget));
}

function boundedWorldJointRotation(
  parentRotation: Quat,
  requestedRotation: Quat,
  definition: SegmentDefinition,
  utilization = 0.98,
): Quat {
  if (!definition.jointProfile) return requestedRotation;
  const requested = jointCoordinates(parentRotation, requestedRotation, definition.jointProfile);
  return jointTargetRotation(parentRotation, definition, {
    x: boundedJointTarget(definition, "x", requested.x, 0, utilization),
    y: boundedJointTarget(definition, "y", requested.y, 0, utilization),
    z: boundedJointTarget(definition, "z", requested.z, 0, utilization),
  });
}

/** Resolve bone roll using the actual anatomical hinge axis, not a knee-only +X assumption. */
export function hingeParentRotation(
  proximalAxis: Vec3,
  distalAxis: Vec3,
  fallbackHingeAxis: Vec3,
  localHingeAxis: Vec3 = RIGHT,
): Quat {
  const up = normalize(proximalAxis, UP);
  const hingeAxis = normalize(cross(up, distalAxis), fallbackHingeAxis);
  const alignUp = quatFromTo(UP, up);
  const baseHingeAxis = rotate(alignUp, localHingeAxis);
  const roll = Math.atan2(
    dot(cross(baseHingeAxis, hingeAxis), up),
    dot(baseHingeAxis, hingeAxis),
  );
  return quatMultiply(quatFromAxisAngle(up, roll), alignUp);
}

function maximumFlexion(definition: SegmentDefinition, fallback: number): number {
  const flexion = definition.jointProfile?.axes.find(({ coordinate }) => coordinate === "x");
  return flexion ? Math.max(Math.abs(flexion.minRadians), Math.abs(flexion.maxRadians)) : fallback;
}

export function poseAnchor(pose: MutablePose, anchor: Vec3): Vec3 {
  return worldPoint(pose.position, pose.rotation, anchor);
}

export function solveTwoBone(
  start: Vec3,
  requestedEnd: Vec3,
  firstLength: number,
  secondLength: number,
  preferredBend: Vec3,
  maximumBendRadians = Math.PI,
): Readonly<{ middle: Vec3; end: Vec3 }> {
  const raw = sub(requestedEnd, start);
  const direction = normalize(raw, { x: 0, y: -1, z: 0 });
  const bendLimit = clamp(maximumBendRadians, 0, Math.PI);
  const limitedMinimum = Math.sqrt(Math.max(
    0,
    firstLength * firstLength + secondLength * secondLength
      + 2 * firstLength * secondLength * Math.cos(bendLimit),
  ));
  const maximum = firstLength + secondLength - 0.002;
  const minimum = Math.min(maximum, Math.max(Math.abs(firstLength - secondLength) + 0.005, limitedMinimum));
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

  const pelvis = makePose("pelvis", root, pelvisTilt);
  output.set("pelvis", pelvis);
  const headingRight = rotate(heading, RIGHT);
  const headingForward = rotate(heading, FORWARD);
  const localReactionRight = dot(horizontalReaction, headingRight);
  const localReactionForward = dot(horizontalReaction, headingForward);
  const lumbarDefinition = segmentDefinition("lumbar");
  const lumbar = attachedPose("lumbar", pelvis, jointTargetRotation(pelvis.rotation, lumbarDefinition, {
    x: boundedJointTarget(lumbarDefinition, "x", localReactionForward * 0.065, 0.16),
    y: 0,
    z: boundedJointTarget(lumbarDefinition, "z", -localReactionRight * 0.065, 0.14),
  }));
  output.set("lumbar", lumbar);
  const torsoDefinition = segmentDefinition("torso");
  const torso = attachedPose("torso", lumbar, jointTargetRotation(lumbar.rotation, torsoDefinition, {
    x: boundedJointTarget(torsoDefinition, "x", localReactionForward * 0.065, 0.16),
    y: 0,
    z: boundedJointTarget(torsoDefinition, "z", -localReactionRight * 0.065, 0.14),
  }));
  output.set("torso", torso);

  const headDrag = input.activeGrab?.region === "head" ? clampLength(drag, 0.42) : ZERO;
  const headTilt = quatMultiply(quatFromTo(UP, normalize({
    x: horizontalReaction.x * 0.45 + headDrag.x * 1.05,
    y: 1 + headDrag.y * 0.2,
    z: horizontalReaction.z * 0.45 + headDrag.z * 1.05,
  })), heading);
  const neckDefinition = segmentDefinition("neck");
  const headDefinition = segmentDefinition("head");
  const desiredHeadCoordinates = headDefinition.jointProfile
    ? jointCoordinates(torso.rotation, headTilt, headDefinition.jointProfile)
    : ZERO;
  const neck = attachedPose("neck", torso, jointTargetRotation(torso.rotation, neckDefinition, {
    x: boundedJointTarget(neckDefinition, "x", desiredHeadCoordinates.x * 0.42, 0.25),
    y: boundedJointTarget(neckDefinition, "y", desiredHeadCoordinates.y * 0.42, 0.30),
    z: boundedJointTarget(neckDefinition, "z", desiredHeadCoordinates.z * 0.42, 0.22),
  }));
  output.set("neck", neck);
  const head = attachedPose("head", neck, jointTargetRotation(neck.rotation, headDefinition, {
    x: boundedJointTarget(headDefinition, "x", desiredHeadCoordinates.x * 0.58, 0.35),
    y: boundedJointTarget(headDefinition, "y", desiredHeadCoordinates.y * 0.58, 0.45),
    z: boundedJointTarget(headDefinition, "z", desiredHeadCoordinates.z * 0.58, 0.25),
  }));
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
  const girdleId = `${side}ShoulderGirdle` as SegmentId;
  const upperId = `${side}UpperArm` as SegmentId;
  const forearmId = `${side}Forearm` as SegmentId;
  const twistId = `${side}ForearmTwist` as SegmentId;
  const handId = `${side}Hand` as "leftHand" | "rightHand";
  const girdleDefinition = segmentDefinition(girdleId);
  const upperDefinition = segmentDefinition(upperId);
  const forearmDefinition = segmentDefinition(forearmId);
  const twistDefinition = segmentDefinition(twistId);
  const handDefinition = segmentDefinition(handId);
  const upperLength = jointSpan(upperId, forearmId);
  const forearmLength = jointSpan(forearmId, twistId) + jointSpan(twistId, handId);
  const idle = Math.sin(input.simulationTime * 1.7 + (side === "left" ? 0 : Math.PI)) * 0.018;
  const desiredHandAt = (shoulder: Vec3): Vec3 => {
    if (input.activeGrab?.region === handId) {
      return add(input.activeGrab.startSegmentPosition, clampLength(drag, 0.72));
    }
    const relaxed = add(shoulder, rotate(torso.rotation, {
      x: sign * HUMAN_PROPORTIONS.arm.relaxedLateralOffsetM,
      y: -(upperLength + forearmLength + length(handDefinition.jointAnchorChild!) - HUMAN_PROPORTIONS.arm.relaxedShorteningM),
      z: idle,
    }));
    const counterbalance = scale(horizontal(input.reactionOffset), -0.52);
    return add(relaxed, { ...counterbalance, y: length(counterbalance) * 0.8 });
  };

  // Let the shoulder girdle visibly share demanding reaches while keeping its
  // target inside the same asymmetric profile enforced by the constraint.
  const baseGirdleRotation = jointTargetRotation(torso.rotation, girdleDefinition, ZERO);
  let girdle = attachedPose(girdleId, torso, baseGirdleRotation);
  let shoulder = poseAnchor(girdle, upperDefinition.jointAnchorParent!);
  let desiredHand = desiredHandAt(shoulder);
  const initialReach = sub(desiredHand, shoulder);
  const initialDirection = normalize(initialReach, rotate(torso.rotation, { x: sign, y: -1, z: 0 }));
  const torsoUp = rotate(torso.rotation, UP);
  const torsoForward = rotate(torso.rotation, FORWARD);
  const reachActivity = input.activeGrab?.region === handId
    ? clamp(length(drag) / 0.28, 0, 1)
    : clamp(length(horizontal(input.reactionOffset)) * 1.1, 0, 0.8);
  const elevationMagnitude = reachActivity * clamp(
    0.045 + Math.max(0, dot(initialDirection, torsoUp) + 0.45) * 0.11,
    0,
    0.18,
  );
  const elevation = boundedJointTarget(girdleDefinition, "z", sign * elevationMagnitude, 0.18);
  const protraction = boundedJointTarget(
    girdleDefinition,
    "y",
    -sign * reachActivity * dot(initialDirection, torsoForward) * 0.14,
    0.14,
  );
  girdle = attachedPose(girdleId, torso, jointTargetRotation(torso.rotation, girdleDefinition, {
    x: 0,
    y: protraction,
    z: elevation,
  }));
  output.set(girdleId, girdle);
  shoulder = poseAnchor(girdle, upperDefinition.jointAnchorParent!);
  desiredHand = desiredHandAt(shoulder);

  const reach = sub(desiredHand, shoulder);
  const reachDirection = normalize(reach, rotate(torso.rotation, { x: sign, y: -1, z: 0 }));
  const forwardReach = clamp(dot(reachDirection, torsoForward), -1, 1);
  const upwardReach = clamp(dot(reachDirection, torsoUp), -1, 1);
  const torsoRight = rotate(torso.rotation, { x: 1, y: 0, z: 0 });
  const acrossBodyReach = Math.max(0, -sign * dot(reachDirection, torsoRight));
  const twistRadians = boundedJointTarget(
    twistDefinition,
    "y",
    sign * reachActivity * (0.18 + Math.max(0, forwardReach) * 0.48 + acrossBodyReach * 0.14),
    1.1,
  );
  const wristFlexion = boundedJointTarget(
    handDefinition,
    "x",
    reachActivity * clamp(-forwardReach * 0.16 + upwardReach * 0.10, -0.24, 0.24),
    0.32,
  );
  const wristDeviation = boundedJointTarget(
    handDefinition,
    "z",
    sign * reachActivity * clamp(acrossBodyReach * 0.12 - 0.035, -0.10, 0.12),
    0.18,
  );
  const yaw = quatFromAxisAngle(UP, input.heading ?? 0);
  // Elbows sit slightly behind/outside the arm, so flexion brings hands
  // forward, in the same body frame as the toes. Knees keep their +Z bend.
  const preferredBend = rotate(torso.rotation, normalize({ x: sign * 0.22, y: 0, z: -1 }));
  const elbowAxis = rotate(forearmDefinition.jointProfile!.parentFrame.rotation, RIGHT);
  const approximateAxis = normalize(sub(shoulder, desiredHand), UP);
  const approximateForearmRotation = quatMultiply(quatFromTo(UP, approximateAxis), yaw);
  const approximateTwistRotation = jointTargetRotation(approximateForearmRotation, twistDefinition, {
    x: 0,
    y: twistRadians,
    z: 0,
  });
  const approximateHandRotation = jointTargetRotation(approximateTwistRotation, handDefinition, {
    x: wristFlexion,
    y: 0,
    z: wristDeviation,
  });
  let requestedWrist = worldPoint(
    desiredHand,
    approximateHandRotation,
    handDefinition.jointAnchorChild!,
  );
  const elbowMaximum = maximumFlexion(forearmDefinition, Math.PI * 0.8) * 0.98;
  let solved = solveTwoBone(
    shoulder,
    requestedWrist,
    upperLength,
    forearmLength,
    preferredBend,
    elbowMaximum,
  );
  let upperAxis = normalize(sub(shoulder, solved.middle), UP);
  let forearmAxis = normalize(sub(solved.middle, solved.end), UP);
  let upperRotation = boundedWorldJointRotation(
    girdle.rotation,
    hingeParentRotation(upperAxis, forearmAxis, rotate(yaw, elbowAxis), elbowAxis),
    upperDefinition,
  );
  let elbowFlexion = boundedJointTarget(
    forearmDefinition,
    "x",
    Math.acos(clamp(dot(upperAxis, forearmAxis), -1, 1)),
    elbowMaximum,
    0.98,
  );
  let forearmRotation = jointTargetRotation(upperRotation, forearmDefinition, { x: elbowFlexion, y: 0, z: 0 });
  let twistRotation = jointTargetRotation(forearmRotation, twistDefinition, { x: 0, y: twistRadians, z: 0 });
  let handRotation = jointTargetRotation(twistRotation, handDefinition, {
    x: wristFlexion,
    y: 0,
    z: wristDeviation,
  });
  // A second pass preserves the requested hand centre after wrist articulation.
  requestedWrist = worldPoint(desiredHand, handRotation, handDefinition.jointAnchorChild!);
  solved = solveTwoBone(
    shoulder,
    requestedWrist,
    upperLength,
    forearmLength,
    preferredBend,
    elbowMaximum,
  );
  upperAxis = normalize(sub(shoulder, solved.middle), UP);
  forearmAxis = normalize(sub(solved.middle, solved.end), UP);
  upperRotation = boundedWorldJointRotation(
    girdle.rotation,
    hingeParentRotation(upperAxis, forearmAxis, rotate(yaw, elbowAxis), elbowAxis),
    upperDefinition,
  );
  elbowFlexion = boundedJointTarget(
    forearmDefinition,
    "x",
    Math.acos(clamp(dot(upperAxis, forearmAxis), -1, 1)),
    elbowMaximum,
    0.98,
  );
  forearmRotation = jointTargetRotation(upperRotation, forearmDefinition, { x: elbowFlexion, y: 0, z: 0 });
  twistRotation = jointTargetRotation(forearmRotation, twistDefinition, { x: 0, y: twistRadians, z: 0 });
  handRotation = jointTargetRotation(twistRotation, handDefinition, {
    x: wristFlexion,
    y: 0,
    z: wristDeviation,
  });
  const upper = makePose(upperId, sub(shoulder, rotate(upperRotation, upperDefinition.jointAnchorChild!)), upperRotation);
  output.set(upperId, upper);
  const forearm = attachedPose(forearmId, upper, forearmRotation);
  output.set(forearmId, forearm);
  const twist = attachedPose(twistId, forearm, twistRotation);
  output.set(twistId, twist);
  const hand = attachedPose(handId, twist, handRotation);
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
  const ankleId = `${side}Ankle` as SegmentId;
  const footId = `${side}Foot` as "leftFoot" | "rightFoot";
  const forefootId = `${side}Forefoot` as SegmentId;
  const thighDefinition = segmentDefinition(thighId);
  const shinDefinition = segmentDefinition(shinId);
  const ankleDefinition = segmentDefinition(ankleId);
  const footDefinition = segmentDefinition(footId);
  const hip = poseAnchor(pelvis, thighDefinition.jointAnchorParent!);
  const thighLength = jointSpan(thighId, shinId);
  const shinLength = jointSpan(shinId, ankleId);
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
    const soleHeight = -footDefinition.geometry.localBounds.min.y;
    footPosition = { ...footPosition, y: Math.max(soleHeight + 0.005, footPosition.y) };
  }

  const footHeading = input.step?.foot === footId ? input.step.heading ?? input.heading : input.heading;
  const headingRotation = quatFromAxisAngle(UP, footHeading ?? 0);
  const headingForward = rotate(headingRotation, FORWARD);
  const headingRight = rotate(headingRotation, { x: 1, y: 0, z: 0 });
  let ankleRotation = headingRotation;
  let footRotation = headingRotation;
  const requestedLegEnd = (): Vec3 => ankleFromHindfoot(
    side, footPosition, footRotation, ankleRotation,
  );
  const kneeMaximum = maximumFlexion(shinDefinition, Math.PI * 0.78) * 0.98;
  const solveLeg = () => solveTwoBone(
    hip,
    requestedLegEnd(),
    thighLength,
    shinLength,
    headingForward,
    kneeMaximum,
  );

  // Iterate the small ankle/foot offsets twice: ankle X removes sagittal shin
  // lean, then foot Z levels the residual side tilt. Each target is profile-bounded.
  let solved = solveLeg();
  for (let iteration = 0; iteration < 2; iteration += 1) {
    const thighAxis = normalize(sub(hip, solved.middle), UP);
    const shinAxis = normalize(sub(solved.middle, solved.end), UP);
    const thighRotation = boundedWorldJointRotation(
      pelvis.rotation,
      hingeParentRotation(thighAxis, shinAxis, headingRight),
      thighDefinition,
    );
    const kneeFlexion = boundedJointTarget(
      shinDefinition,
      "x",
      Math.acos(clamp(dot(thighAxis, shinAxis), -1, 1)),
      kneeMaximum,
      0.98,
    );
    const shinRotation = jointTargetRotation(thighRotation, shinDefinition, { x: kneeFlexion, y: 0, z: 0 });
    ankleRotation = boundedWorldJointRotation(
      shinRotation, headingRotation, ankleDefinition, 0.9,
    );
    const ankleUp = rotate(ankleRotation, UP);
    const footTilt = boundedJointTarget(
      footDefinition,
      "z",
      Math.atan2(dot(ankleUp, headingRight), dot(ankleUp, UP)),
      0.24,
      0.9,
    );
    footRotation = jointTargetRotation(ankleRotation, footDefinition, { x: 0, y: 0, z: footTilt });
    solved = solveLeg();
  }

  const thighAxis = normalize(sub(hip, solved.middle), UP);
  const shinAxis = normalize(sub(solved.middle, solved.end), UP);
  const thighRotation = boundedWorldJointRotation(
    pelvis.rotation,
    hingeParentRotation(thighAxis, shinAxis, headingRight),
    thighDefinition,
  );
  const kneeFlexion = boundedJointTarget(
    shinDefinition,
    "x",
    Math.acos(clamp(dot(thighAxis, shinAxis), -1, 1)),
    kneeMaximum,
    0.98,
  );
  const shinRotation = jointTargetRotation(thighRotation, shinDefinition, { x: kneeFlexion, y: 0, z: 0 });
  ankleRotation = boundedWorldJointRotation(
    shinRotation, headingRotation, ankleDefinition, 0.9,
  );
  const ankleUp = rotate(ankleRotation, UP);
  const footTilt = boundedJointTarget(
    footDefinition,
    "z",
    Math.atan2(dot(ankleUp, headingRight), dot(ankleUp, UP)),
    0.24,
    0.9,
  );
  footRotation = jointTargetRotation(ankleRotation, footDefinition, { x: 0, y: 0, z: footTilt });
  const thigh = makePose(thighId, sub(hip, rotate(thighRotation, thighDefinition.jointAnchorChild!)), thighRotation);
  output.set(thighId, thigh);
  const shin = attachedPose(shinId, thigh, shinRotation);
  output.set(shinId, shin);
  // Anchor every distal part to the solved endpoint so unreachable targets can
  // move the whole chain but can never stretch it.
  const ankle = attachedPose(ankleId, shin, ankleRotation);
  output.set(ankleId, ankle);
  const foot = attachedPose(footId, ankle, footRotation);
  output.set(footId, foot);
  const forefootDefinition = segmentDefinition(forefootId);
  output.set(forefootId, attachedPose(
    forefootId,
    foot,
    jointTargetRotation(foot.rotation, forefootDefinition, ZERO),
  ));
}
