import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type { JointCoordinate, Quat, RegionId, SegmentDefinition, SegmentId, SegmentPose, Vec3 } from "../core/types";
import {
  add, clamp, clampLength, cross, dot, length, lerp, normalize,
  quatFromAxisAngle, quatFromTo, quatMultiply, rotate, scale, smooth01, sub, worldPoint,
} from "./math";
import { jointCoordinates, jointRotationFromCoordinates } from "./joint-coordinates";
import { collisionAwareLimbTarget } from "./limb-collisions";
import { refineBoundedReach } from "./bounded-reach";

import { ankleFromHindfoot, transferPelvisHeight } from "./leg-target-frame";

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
  /** Rapier contacts have stayed below the unload force for the required window. */
  unloaded?: boolean;
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
  /** Solve leg reach from the current physical pelvis while its target lags. */
  measuredLegFrame?: boolean;
  simulationTime: number;
  activeGrab: {
    region: RegionId;
    target: Vec3;
    startTarget: Vec3;
    startSegmentPosition: Vec3;
  } | null;
  supportFeet: Readonly<Record<"leftFoot" | "rightFoot", Vec3>>;
  /** Measured plant rotations, retained while the foot stays on that support. */
  supportFootRotations?: Readonly<Partial<Record<"leftFoot" | "rightFoot", Quat>>>;
  /** Current Rapier poses used as the start of a collision-aware reach path. */
  measuredPoses?: ReadonlyMap<SegmentId, MutablePose>;
  /** Internal: build one clamped preview without recursively planning a route. */
  collisionPlanning?: false;
  step: StepMotion | null;
  /** World yaw preserved across dynamic recovery and non-default starts. */
  heading?: number;
  /** Optional bounded ribcage twist for pose composition and anatomy probes. */
  torsoTwistRadians?: number;
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
    poses.set(definition.id, makePose(definition.id, position, rotation));
  }
  return poses;
}

function makePose(id: SegmentId, position: Vec3, rotation: Quat = IDENTITY): MutablePose {
  // Mutable simulation/fixture poses must never alias anatomy or another pose.
  return { id, position: { ...position }, rotation: { ...rotation },
    linearVelocity: { ...ZERO }, angularVelocity: { ...ZERO } };
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
  // As the hinge straightens, its cross product loses a stable direction.
  // Bias only that near-singular range: retaining the exact cross-product
  // plane for ordinary bends lets reconstructed distal anchors land exactly.
  const rawHingeAxis = cross(up, distalAxis);
  const fallbackWeight = Math.max(0, 0.025 - length(rawHingeAxis));
  const hingeAxis = normalize(add(
    rawHingeAxis,
    scale(fallbackHingeAxis, fallbackWeight),
  ), fallbackHingeAxis);
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
  secondaryAxis: Vec3 = RIGHT,
  middleClearance?: Readonly<{ direction: Vec3; minimumOffset: number }>,
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
  const projectedPreference = sub(preferredBend, scale(direction, dot(preferredBend, direction)));
  const secondary = cross(direction, secondaryAxis);
  // The small secondary term gives a continuous bend plane when a requested
  // endpoint lines up with the bend preference near full extension.
  let bend = add(projectedPreference, scale(secondary, 0.01));
  bend = normalize(bend, FORWARD);
  const center = add(start, scale(direction, along));
  if (middleClearance && bendHeight > 1e-10) {
    // The elbow can move around this circle without moving the wrist or
    // changing either bone length. Choose the nearest bend that keeps a
    // relaxed arm outside the torso, including during counterbalance.
    const outward = normalize(middleClearance.direction);
    const projected = sub(outward, scale(direction, dot(outward, direction)));
    const projectionLength = length(projected);
    if (projectionLength > 1e-10) {
      const outwardBend = scale(projected, 1 / projectionLength);
      const current = dot(bend, outwardBend);
      const minimum = clamp((middleClearance.minimumOffset - along * dot(direction, outward))
        / (bendHeight * projectionLength), -1, 1);
      if (current < minimum) {
        const tangent = normalize(sub(bend, scale(outwardBend, current)), cross(direction, outwardBend));
        bend = add(scale(outwardBend, minimum), scale(tangent, Math.sqrt(Math.max(0, 1 - minimum * minimum))));
      }
    }
  }
  return { middle: add(center, scale(bend, bendHeight)), end };
}

export function midpoint(a: Vec3, b: Vec3): Vec3 {
  return scale(add(a, b), 0.5);
}

export function horizontal(v: Vec3): Vec3 {
  return { x: v.x, y: 0, z: v.z };
}

export function composeUprightPose(input: UprightPoseInput): UprightPoseResult {
  const grab = input.activeGrab;
  if (input.collisionPlanning !== false && grab && input.measuredPoses
      && ["leftHand", "rightHand", "leftFoot", "rightFoot"].includes(grab.region)) {
    const side: LegSide = grab.region.startsWith("left") ? "left" : "right";
    const limb = grab.region.endsWith("Hand") ? "arm" : "leg";
    const segmentId = grab.region as SegmentId;
    const measured = input.measuredPoses.get(segmentId);
    if (measured) {
      const previewInput: UprightPoseInput = { ...input, collisionPlanning: false };
      const preview = composeUprightPose(previewInput);
      const parent = preview.poses.get(limb === "arm" ? "torso" : "pelvis")!;
      let requested = add(grab.startSegmentPosition, clampLength(
        sub(grab.target, grab.startTarget), limb === "arm" ? 0.72 : 0.58,
      ));
      if (limb === "leg") {
        const soleHeight = -segmentDefinition(segmentId).geometry.localBounds.min.y;
        requested = { ...requested, y: Math.max(soleHeight + 0.005, requested.y) };
      }
      const solve = (target: Vec3): Map<SegmentId, MutablePose> => composeUprightPose({
        ...previewInput,
        activeGrab: {
          ...grab,
          target: add(grab.startTarget, sub(target, grab.startSegmentPosition)),
        },
      }).poses;
      const planned = collisionAwareLimbTarget({
        side, limb, start: measured.position, requested, parent, solve,
      });
      return { poses: planned.poses, leanRadians: preview.leanRadians };
    }
  }
  const output = new Map<SegmentId, MutablePose>();
  const rootTranslation = input.rootTranslation;
  const root = { x: rootTranslation.x, y: rootTranslation.y - clamp(input.kneeFlexion ?? 0, 0, 1) * 0.10, z: rootTranslation.z };
  const heading = quatFromAxisAngle(UP, input.heading ?? 0);
  const drag = input.activeGrab ? sub(input.activeGrab.target, input.activeGrab.startTarget) : ZERO;
  const horizontalReaction = horizontal(input.reactionOffset);
  const leanRadians = Math.atan(length(horizontalReaction) * 0.9);
  const pelvisTilt = quatMultiply(quatFromTo(UP, normalize({
    x: horizontalReaction.x * 0.5,
    y: 1,
    z: horizontalReaction.z * 0.5,
  })), heading);

  // Keep the anatomical reserve after lift and after a cancelled transfer too.
  // Dropping this ceiling at a phase boundary abruptly straightens the stance
  // knee and can leave the measured pelvis above every feasible landing.
  root.y = transferPelvisHeight(root, pelvisTilt, input.supportFeet, {
      leftFoot: input.supportFootRotations?.leftFoot ?? heading,
      rightFoot: input.supportFootRotations?.rightFoot ?? heading,
  });

  const pelvis = makePose("pelvis", root, pelvisTilt);
  output.set("pelvis", pelvis);
  const headingRight = rotate(heading, RIGHT);
  const headingForward = rotate(heading, FORWARD);
  const localReactionRight = dot(horizontalReaction, headingRight);
  const localReactionForward = dot(horizontalReaction, headingForward);
  const lumbarDefinition = segmentDefinition("lumbar");
  const lumbar = attachedPose("lumbar", pelvis, jointTargetRotation(pelvis.rotation, lumbarDefinition, {
    x: boundedJointTarget(lumbarDefinition, "x", localReactionForward * 0.45, 0.16),
    y: 0,
    z: boundedJointTarget(lumbarDefinition, "z", -localReactionRight * 0.45, 0.14),
  }));
  output.set("lumbar", lumbar);
  const torsoDefinition = segmentDefinition("torso");
  const torso = attachedPose("torso", lumbar, jointTargetRotation(lumbar.rotation, torsoDefinition, {
    x: boundedJointTarget(torsoDefinition, "x", localReactionForward * 0.45, 0.16),
    y: boundedJointTarget(torsoDefinition, "y", input.torsoTwistRadians ?? 0, 0.2),
    z: boundedJointTarget(torsoDefinition, "z", -localReactionRight * 0.45, 0.14),
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
  const movingPelvis = input.measuredLegFrame
    ? input.measuredPoses?.get("pelvis") ?? pelvis : pelvis;
  // Both planted soles and a committed swing are world targets. Posture/height
  // restoration is a bounded supporting-joint wrench; encoding a virtual root
  // in these leg angles would lift the opposite sole during transfer.
  composeLeg("left", movingPelvis, output, drag, input);
  composeLeg("right", movingPelvis, output, drag, input);
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
  // Elbows sit slightly behind/outside the arm, so flexion brings hands
  // forward, in the same body frame as the toes. Knees keep their +Z bend.
  const preferredBend = rotate(torso.rotation, normalize({ x: sign * 0.22, y: 0, z: -1 }));
  const elbowClearance = input.activeGrab?.region === handId ? undefined : {
    direction: rotate(torso.rotation, { x: sign, y: 0, z: 0 }),
    minimumOffset: HUMAN_PROPORTIONS.arm.relaxedLateralOffsetM,
  };
  const elbowAxis = rotate(forearmDefinition.jointProfile!.parentFrame.rotation, RIGHT);
  const approximateAxis = normalize(sub(shoulder, desiredHand), UP);
  const approximateForearmRotation = quatMultiply(
    quatFromTo(rotate(torso.rotation, UP), approximateAxis), torso.rotation,
  );
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
    rotate(torso.rotation, RIGHT),
    elbowClearance,
  );
  let upperAxis = normalize(sub(shoulder, solved.middle), UP);
  let forearmAxis = normalize(sub(solved.middle, solved.end), UP);
  let upperRotation = boundedWorldJointRotation(
    girdle.rotation,
    hingeParentRotation(upperAxis, forearmAxis, rotate(torso.rotation, elbowAxis), elbowAxis),
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
    rotate(torso.rotation, RIGHT),
    elbowClearance,
  );
  upperAxis = normalize(sub(shoulder, solved.middle), UP);
  forearmAxis = normalize(sub(solved.middle, solved.end), UP);
  upperRotation = boundedWorldJointRotation(
    girdle.rotation,
    hingeParentRotation(upperAxis, forearmAxis, rotate(torso.rotation, elbowAxis), elbowAxis),
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

  if (input.activeGrab?.region === handId) {
    // Clamping the unconstrained shoulder solution can displace the hand far
    // below its requested point. Solve the remaining position error within the
    // same limits instead of letting the posture motors oppose the grab spring.
    const measuredGirdle = input.measuredPoses?.get(girdleId);
    const measuredUpper = input.measuredPoses?.get(upperId);
    const measuredForearm = input.measuredPoses?.get(forearmId);
    // The measured configuration gives successive live solves a continuous
    // branch, including when the unconstrained elbow pole crosses a limit.
    const shoulderCoordinates = measuredGirdle && measuredUpper
      ? jointCoordinates(measuredGirdle.rotation, measuredUpper.rotation, upperDefinition.jointProfile!)
      : jointCoordinates(girdle.rotation, upperRotation, upperDefinition.jointProfile!);
    const initialElbow = measuredUpper && measuredForearm
      ? jointCoordinates(measuredUpper.rotation, measuredForearm.rotation, forearmDefinition.jointProfile!).x : elbowFlexion;
    const axes = ["x", "y", "z"] as const;
    const bounds = [...axes.map(coordinate => {
      const axis = upperDefinition.jointProfile!.axes.find(a => a.coordinate === coordinate)!;
      return [axis.minRadians * 0.98, axis.maxRadians * 0.98] as const;
    }), [0, elbowMaximum] as const];
    const chainAt = (coordinates: readonly number[]): MutablePose[] => {
      const nextUpper = attachedPose(upperId, girdle, jointTargetRotation(girdle.rotation, upperDefinition,
        { x: coordinates[0], y: coordinates[1], z: coordinates[2] }));
      const nextForearm = attachedPose(forearmId, nextUpper, jointTargetRotation(nextUpper.rotation, forearmDefinition,
        { x: coordinates[3], y: 0, z: 0 }));
      const nextTwist = attachedPose(twistId, nextForearm, jointTargetRotation(nextForearm.rotation, twistDefinition,
        { x: 0, y: twistRadians, z: 0 }));
      const nextHand = attachedPose(handId, nextTwist, jointTargetRotation(nextTwist.rotation, handDefinition,
        { x: wristFlexion, y: 0, z: wristDeviation }));
      return [nextUpper, nextForearm, nextTwist, nextHand];
    };
    const endpoint = (coordinates: readonly number[]) => chainAt(coordinates)[3].position;
    const refined = refineBoundedReach([shoulderCoordinates.x, shoulderCoordinates.y, shoulderCoordinates.z, initialElbow],
      bounds, desiredHand, endpoint);
    for (const pose of chainAt(refined)) output.set(pose.id, pose);
  }
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
  let ankleUtilization = 0.9;

  if (input.step?.foot === footId && input.step.elapsed >= 0) {
    const progress = clamp(input.step.elapsed / input.step.duration, 0, 1);
    const travel = length(horizontal(sub(input.step.to, input.step.from)));
    const nominalClearance = clamp(
      HUMAN_PROPORTIONS.foot.stepClearanceBaseM + travel * HUMAN_PROPORTIONS.foot.stepClearancePerTravel,
      HUMAN_PROPORTIONS.foot.minStepClearanceM,
      HUMAN_PROPORTIONS.foot.maxStepClearanceM,
    );
    const measuredFoot = input.measuredPoses?.get(footId);
    const flatSoleDrop = -footDefinition.geometry.localBounds.min.y;
    const lowestMeasuredSoleY = measuredFoot
      ? ([footId, forefootId] as const).flatMap(id => {
        const measured = input.measuredPoses?.get(id);
        if (!measured) return [];
        const geometry = segmentDefinition(id).geometry;
        return geometry.vertices.map(vertex => measured.position.y + rotate(measured.rotation, vertex).y);
      }).reduce((lowest, y) => Math.min(lowest, y), Number.POSITIVE_INFINITY)
      : Number.NaN;
    const measuredSoleDrop = Number.isFinite(lowestMeasuredSoleY)
      ? measuredFoot!.position.y - lowestMeasuredSoleY : flatSoleDrop;
    // A tilted loaded sole can keep touching at its toe or heel even after its
    // centre rises by the nominal shuffle clearance. Add only the measured
    // orientation's extra vertical extent, then keep that lift through the
    // unload gate.
    const clearance = Math.min(0.18, nominalClearance
      + Math.max(0, measuredSoleDrop - flatSoleDrop) + 0.04);
    // Separate lift, travel and lowering so the sole can release before
    // horizontal travel. BalanceController separately verifies actual unload
    // before any later loaded touchdown can complete the step.
    const travelProgress = progress < 0.25 ? 0
      : progress < 0.75 ? smooth01((progress - 0.25) / 0.5)
        : 1;
    const lift = progress < 0.25 ? smooth01(progress / 0.25) * clearance
      : progress < 0.75 ? clearance
        : (1 - smooth01((progress - 0.75) / 0.25)) * clearance;
    const base = lerp(input.step.from, input.step.to, travelProgress);
    footPosition = { ...base, y: base.y + lift };
    // A raised swing can use the full anatomical ankle range to level its sole.
    // Restore the planted target reserve continuously as the foot lowers.
    ankleUtilization += 0.1 * lift / clearance;
  } else if (input.activeGrab?.region === footId) {
    footPosition = add(input.activeGrab.startSegmentPosition, clampLength(drag, 0.58));
    const soleHeight = -footDefinition.geometry.localBounds.min.y;
    footPosition = { ...footPosition, y: Math.max(soleHeight + 0.005, footPosition.y) };
  }

  const currentHeadingRotation = quatFromAxisAngle(UP, input.heading ?? 0);
  const stepHeadingRotation = quatFromAxisAngle(UP, input.step?.heading ?? input.heading ?? 0);
  const plantedRotation = input.supportFootRotations?.[footId];
  const footHeading = input.step?.foot === footId
    ? input.step.elapsed >= 0 ? stepHeadingRotation : plantedRotation ?? stepHeadingRotation
    : plantedRotation ?? currentHeadingRotation;
  const headingRotation = footHeading;
  const headingForward = rotate(footHeading, FORWARD);
  const headingRight = rotate(footHeading, RIGHT);
  const pelvisForward = rotate(pelvis.rotation, FORWARD);
  const kneeBendForward = input.supportFootRotations?.[footId]
    && (input.step?.foot !== footId || input.step.elapsed < 0)
    ? headingForward : pelvisForward;
  const kneeHingeRight = input.supportFootRotations?.[footId]
    && (input.step?.foot !== footId || input.step.elapsed < 0)
    ? headingRight : rotate(pelvis.rotation, RIGHT);
  let ankleRotation = footHeading;
  let footRotation = footHeading;
  const requestedLegEnd = (): Vec3 => ankleFromHindfoot(
    side, footPosition, footRotation, ankleRotation,
  );
  const kneeMaximum = maximumFlexion(shinDefinition, Math.PI * 0.78) * 0.98;
  const solveLeg = () => solveTwoBone(
    hip,
    requestedLegEnd(),
    thighLength,
    shinLength,
    kneeBendForward,
    kneeMaximum,
    rotate(pelvis.rotation, RIGHT),
  );

  // Iterate the small ankle/foot offsets twice: ankle X removes sagittal shin
  // lean, then foot Z levels the residual side tilt. Each target is profile-bounded.
  let solved = solveLeg();
  for (let iteration = 0; iteration < 2; iteration += 1) {
    const thighAxis = normalize(sub(hip, solved.middle), UP);
    const shinAxis = normalize(sub(solved.middle, solved.end), UP);
    const thighRotation = boundedWorldJointRotation(
      pelvis.rotation,
      hingeParentRotation(thighAxis, shinAxis, kneeHingeRight),
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
      shinRotation, headingRotation, ankleDefinition, ankleUtilization,
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
    hingeParentRotation(thighAxis, shinAxis, kneeHingeRight),
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
    shinRotation, headingRotation, ankleDefinition, ankleUtilization,
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
  const stepProgress = input.step?.foot === footId
    ? clamp(input.step.elapsed / input.step.duration, 0, 1) : 1;
  const toeLift = input.step?.foot === footId && input.step.elapsed >= 0
    ? 0.55 * (stepProgress < 0.68 ? 1 : 1 - smooth01((stepProgress - 0.68) / 0.24))
    : 0;
  output.set(forefootId, attachedPose(
    forefootId,
    foot,
    jointTargetRotation(foot.rotation, forefootDefinition, {
      x: boundedJointTarget(forefootDefinition, "x", toeLift, 0.7, 0.9),
      y: 0,
      z: 0,
    }),
  ));
}

/** Preview the joint-bounded hindfoot centre at the end of a step on a measured pelvis. */
export function composeTouchdownHindfoot(
  side: LegSide,
  measuredPelvis: Pick<MutablePose, "position" | "rotation">,
  candidate: Vec3,
  headingRadians: number,
): Vec3 {
  const foot = `${side}Foot` as "leftFoot" | "rightFoot";
  const pelvis = makePose("pelvis", measuredPelvis.position, measuredPelvis.rotation);
  const poses = new Map<SegmentId, MutablePose>();
  composeLeg(side, pelvis, poses, ZERO, {
    rootTranslation: measuredPelvis.position,
    reactionOffset: ZERO,
    simulationTime: 0,
    activeGrab: null,
    supportFeet: { leftFoot: candidate, rightFoot: candidate },
    step: { foot, from: candidate, to: candidate, elapsed: 1, duration: 1,
      heading: headingRadians },
    heading: headingRadians,
  });
  return { ...poses.get(foot)!.position };
}
