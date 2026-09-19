import { lowestWorldPoint } from "../core/geometry";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, RecoveryDiagnostics, RecoveryPhase, SegmentId, SegmentPose, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, cross, dot, length, normalize, quatFromAxisAngle, quatFromTo, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";
import { hingeParentRotation } from "./pose";
import { measureMassState } from "./mass-state";
import {
  limitRecoveryJoint,
  reconstructRecoveryLimb,
  recoveryJointLimitError,
  recoveryJointRotation,
} from "./recovery-joints";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const RIGHT: Vec3 = { x: 1, y: 0, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const SIDES = ["left", "right"] as const;
const GEOMETRY_EPSILON = 1e-8;
// Keep the shared observed/planned hand lever from the concurrent repair.
const MAXIMUM_HAND_BRACE_LEVER_M = HUMAN_PROPORTIONS.arm.upperLengthM + HUMAN_PROPORTIONS.arm.forearmLengthM;

export type RecoveryRoute = "none" | "crouch" | "half-kneel" | "prone" | "roll";
export type RecoverySide = (typeof SIDES)[number];

const armSupports = SIDES.flatMap((side) => [
  `${side}Hand`, `${side}ForearmTwist`, `${side}Forearm`,
] as SegmentId[]);
const legSupports = SIDES.flatMap((side) => [
  `${side}Shin`, `${side}Ankle`, `${side}Foot`, `${side}Forefoot`,
] as SegmentId[]);
const footSupports = SIDES.flatMap((side) => [
  `${side}Foot`, `${side}Forefoot`,
] as SegmentId[]);

export const RECOVERY_SUPPORT_ELIGIBILITY: Record<RecoveryPhase, readonly SegmentId[]> = {
  none: [], protect: [], settle: [],
  roll: ["pelvis", "lumbar", "torso", ...armSupports, ...legSupports],
  brace: [...armSupports, ...legSupports],
  kneel: [...armSupports, ...legSupports],
  stand: footSupports,
};

export function isRecoveryFootSegment(id: SegmentId): boolean {
  const role = SEGMENT_BY_ID.get(id)?.role;
  return role === "hindfoot" || role === "forefoot";
}

export function isRecoveryArmSupportSegment(id: SegmentId): boolean {
  const role = SEGMENT_BY_ID.get(id)?.role;
  return role === "hand" || role === "forearm" || role === "forearm-twist";
}

export function isRecoveryLegSupportSegment(id: SegmentId): boolean {
  const role = SEGMENT_BY_ID.get(id)?.role;
  return role === "shin" || role === "ankle" || role === "hindfoot" || role === "forefoot";
}

/** One selection policy for current and prospective support; never grows contact patches. */
export function selectRecoveryContacts<T extends RecoverySupportContact>(
  contacts: readonly T[], poses: ReadonlyMap<SegmentId, SegmentPose> | undefined,
  phase: RecoveryPhase, stage: RecoveryDiagnostics["transferStage"], leadingSide: RecoverySide | null,
  excluded: ReadonlySet<SegmentId> = new Set(),
): T[] {
  const toeSide = phase === "kneel" && stage === "shift-weight" && leadingSide
    ? (leadingSide === "left" ? "right" : "left") : null;
  return contacts.filter(contact => contact.loadBearing && contact.forceN > 0 && contact.normalY >= .65
    && !excluded.has(contact.segment) && RECOVERY_SUPPORT_ELIGIBILITY[phase].includes(contact.segment)
    && (!isRecoveryFootSegment(contact.segment) || !poses || SEGMENT_BY_ID.get(contact.segment)?.side === toeSide
      || (!!poses.get(contact.segment) && rotate(poses.get(contact.segment)!.rotation, UP).y > .85)));
}

/** Solver contact points are preferable to a reconstructed collision-shape patch. */
export interface RecoverySupportContact extends Omit<SupportingContact, "points"> {
  points?: readonly Vec3[];
}

export interface RecoveryMassState {
  position: Vec3;
  velocity: Vec3;
  massKg: number;
}

export interface RecoverySupportGeometry {
  center: Vec3;
  polygon: Vec3[];
  projectedCenterOfMass: Vec3;
  marginM: number;
  loadedForceN: number;
  supporting: SegmentId[];
}

export const RECOVERY_PROJECTION_SECONDS = 0.15;

/** Each body's recorded linear velocity is its centre-of-mass velocity. */
export function recoveryMassState(poses: Iterable<SegmentPose>): RecoveryMassState {
  return measureMassState(poses);
}

function horizontal(value: Vec3): Vec3 { return { x: value.x, y: 0, z: value.z }; }
function cross2(a: Vec3, b: Vec3, c: Vec3): number {
  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

/** Counterclockwise hull in XZ; duplicate and collinear contacts cannot create area. */
export function recoverySupportHull(points: readonly Vec3[]): Vec3[] {
  const ordered = points.map(horizontal).sort((a, b) => a.x - b.x || a.z - b.z)
    .filter((point, index, all) => index === 0
      || Math.abs(point.x - all[index - 1].x) > GEOMETRY_EPSILON
      || Math.abs(point.z - all[index - 1].z) > GEOMETRY_EPSILON);
  if (ordered.length < 3) return ordered;
  const lower: Vec3[] = [], upper: Vec3[] = [];
  for (const point of ordered) {
    while (lower.length > 1 && cross2(lower[lower.length - 2], lower[lower.length - 1], point) <= GEOMETRY_EPSILON) lower.pop();
    lower.push(point);
  }
  for (const point of [...ordered].reverse()) {
    while (upper.length > 1 && cross2(upper[upper.length - 2], upper[upper.length - 1], point) <= GEOMETRY_EPSILON) upper.pop();
    upper.push(point);
  }
  return [...lower.slice(0, -1), ...upper.slice(0, -1)];
}

/** Signed shortest horizontal distance to the edge; positive means supported. */
export function recoverySupportMargin(point: Vec3, polygon: readonly Vec3[]): number {
  if (!polygon.length) return -1;
  let nearest = Infinity, inside = polygon.length >= 3;
  for (let index = 0; index < polygon.length; index++) {
    const a = polygon[index], b = polygon[(index + 1) % polygon.length];
    const edge = horizontal(sub(b, a)), delta = horizontal(sub(point, a));
    const amount = clamp(dot(delta, edge) / Math.max(dot(edge, edge), 1e-12), 0, 1);
    nearest = Math.min(nearest, length(sub(delta, scale(edge, amount))));
    if (cross2(a, b, point) < -GEOMETRY_EPSILON) inside = false;
  }
  // A line or point has no support area, even when COM lies exactly on it.
  return inside ? nearest : -Math.max(nearest, GEOMETRY_EPSILON);
}

function contactPatch(contact: RecoverySupportContact, pose: SegmentPose | undefined): readonly Vec3[] {
  if (contact.points?.length) return contact.points;
  const geometry = SEGMENT_BY_ID.get(contact.segment)?.geometry;
  if (!pose || !geometry) return [contact.point];
  const candidates = (geometry.supportPatch?.length ? geometry.supportPatch : geometry.vertices)
    .map(point => worldPoint(pose.position, pose.rotation, point));
  const minimumY = Math.min(...candidates.map(point => point.y));
  const floorY = contact.point.y;
  const patch = candidates.filter(point => point.y <= minimumY + 0.012 && Math.abs(point.y - floorY) <= 0.024);
  return patch.length ? patch : [contact.point];
}

/** Loaded solver geometry only; no convex hull expansion or unloaded limb targets. */
export function supportGeometry(
  contacts: readonly RecoverySupportContact[],
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  mass: RecoveryMassState,
  excludedSegments: readonly SegmentId[] = [],
): RecoverySupportGeometry {
  const excluded = new Set(excludedSegments);
  const loaded = contacts.filter(contact => contact.loadBearing && contact.forceN > 0
    && contact.normalY >= 0.65 && !excluded.has(contact.segment));
  const patches = loaded.map(contact => contactPatch(contact, poses.get(contact.segment)));
  const loadedForceN = loaded.reduce((sum, contact) => sum + contact.forceN, 0);
  const center = loadedForceN > 0
    ? scale(loaded.reduce((sum, contact, index) => {
      const points = patches[index];
      const patchCenter = points.length ? scale(points.reduce(add, ZERO), 1 / points.length) : contact.point;
      return add(sum, scale(patchCenter, contact.forceN));
    }, ZERO), 1 / loadedForceN)
    : horizontal(mass.position);
  const polygon = recoverySupportHull(patches.flatMap(points => [...points]));
  const projectedCenterOfMass = horizontal(add(mass.position, scale(mass.velocity, RECOVERY_PROJECTION_SECONDS)));
  return { center, polygon, projectedCenterOfMass, marginM: recoverySupportMargin(projectedCenterOfMass, polygon),
    loadedForceN, supporting: [...new Set(loaded.map(contact => contact.segment))] };
}

/** A deliberate release needs an actual remaining support area under future COM. */
export function canReleaseSupport(
  contacts: readonly RecoverySupportContact[],
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  mass: RecoveryMassState,
  excludedSegments: readonly SegmentId[],
): boolean {
  if (mass.massKg <= 0 || !excludedSegments.length) return false;
  const remaining = supportGeometry(contacts, poses, mass, excludedSegments);
  return remaining.supporting.length > 0 && remaining.polygon.length >= 3 && remaining.marginM >= 0;
}

export interface RecoveryRouteInput {
  poses: ReadonlyMap<SegmentId, SegmentPose>;
  contacts: readonly RecoverySupportContact[];
  /** Reachable centre targets supplied by the existing two-bone limb solver. */
  footTargets?: Partial<Record<RecoverySide, Vec3>>;
  /** Optional measured feet-only projected support margin. */
  supportMarginM?: number;
}

export interface RecoveryRouteChoice {
  route: Exclude<RecoveryRoute, "none">;
  leadingSide: RecoverySide;
  rollSide: RecoverySide;
}

function loadedContact(contacts: readonly RecoverySupportContact[], segment: SegmentId): RecoverySupportContact | undefined {
  return contacts.find(contact => contact.segment === segment && contact.loadBearing && contact.forceN > 0 && contact.normalY >= 0.65);
}

function armHeight(side: RecoverySide, poses: ReadonlyMap<SegmentId, SegmentPose>): number {
  return Math.min(...([`${side}Hand`, `${side}ForearmTwist`, `${side}Forearm`] as SegmentId[]).map(id => {
    const pose = poses.get(id), geometry = SEGMENT_BY_ID.get(id)?.geometry;
    return pose && geometry ? lowestWorldPoint(geometry, pose.position, pose.rotation).y : Infinity;
  }));
}

/**
 * Recovery paths need to be able to return to an already captured endpoint.
 * The posture helper intentionally keeps 2 mm of extension reserve, which is
 * useful for live IK but would make a captured plant drift on the final frame.
 */
function solveCapturedTwoBone(
  start: Vec3,
  requestedEnd: Vec3,
  firstLength: number,
  secondLength: number,
  preferredBend: Vec3,
  maximumBendRadians: number,
): Readonly<{ middle: Vec3; end: Vec3 }> {
  const raw = sub(requestedEnd, start);
  const direction = normalize(raw, { x: 0, y: -1, z: 0 });
  const bendLimit = clamp(maximumBendRadians, 0, Math.PI);
  const minimumFromLimit = Math.sqrt(Math.max(
    0,
    firstLength ** 2 + secondLength ** 2
      + 2 * firstLength * secondLength * Math.cos(bendLimit),
  ));
  const maximum = firstLength + secondLength;
  const minimum = Math.min(maximum, Math.max(
    Math.abs(firstLength - secondLength),
    minimumFromLimit,
  ));
  const distance = clamp(length(raw), minimum, maximum);
  const end = add(start, scale(direction, distance));
  if (maximum - distance <= 1e-12) {
    return { middle: add(start, scale(direction, firstLength)), end };
  }
  const along = distance > GEOMETRY_EPSILON
    ? (firstLength ** 2 - secondLength ** 2 + distance ** 2) / (2 * distance)
    : firstLength;
  // The factored triangle area reaches exact zero at full extension. The
  // difference of near-equal squares previously introduced heading-dependent
  // nanometre bends, amplified by acos near a straight elbow.
  const bendHeight = distance > GEOMETRY_EPSILON ? Math.sqrt(Math.max(0,
    (maximum - distance) * (maximum + distance)
      * (distance - Math.abs(firstLength - secondLength))
      * (distance + Math.abs(firstLength - secondLength)),
  )) / (2 * distance) : 0;
  let bend = sub(preferredBend, scale(direction, dot(preferredBend, direction)));
  if (length(bend) < 1e-4) bend = cross(direction, RIGHT);
  bend = normalize(bend, FORWARD);
  return {
    middle: add(add(start, scale(direction, along)), scale(bend, bendHeight)),
    end,
  };
}

function loadedFootContact(
  contacts: readonly RecoverySupportContact[],
  side: RecoverySide,
): RecoverySupportContact | undefined {
  return contacts
    .filter((contact) => SEGMENT_BY_ID.get(contact.segment)?.side === side
      && isRecoveryFootSegment(contact.segment))
    .map((contact) => loadedContact(contacts, contact.segment))
    .filter((contact): contact is RecoverySupportContact => !!contact)
    .sort((a, b) => b.forceN - a.forceN)[0];
}

/** Called once after settling, and retained by the controller until a retry. */
export function selectRecoveryRoute(input: RecoveryRouteInput): RecoveryRouteChoice {
  const { poses, contacts } = input;
  const feet = SIDES.map(side => ({ side, contact: loadedFootContact(contacts, side), pose: poses.get(`${side}Foot`) }))
    .filter(({ contact, pose }) => contact && pose);
  const travel = (side: RecoverySide): number => {
    const foot = poses.get(`${side}Foot`), target = input.footTargets?.[side];
    return foot && target ? length(sub(target, foot.position)) : Infinity;
  };
  // Stronger established support wins; ties use least movement, then stable side order.
  feet.sort((a, b) => b.contact!.forceN - a.contact!.forceN || travel(a.side) - travel(b.side)
    || SIDES.indexOf(a.side) - SIDES.indexOf(b.side));
  const leadingSide = feet[0]?.side ?? (travel("right") < travel("left") - 1e-8 ? "right" : "left");
  const armLoad = (side: RecoverySide): number => contacts.reduce((sum, contact) =>
    sum + (SEGMENT_BY_ID.get(contact.segment)?.side === side && isRecoveryArmSupportSegment(contact.segment)
      ? loadedContact(contacts, contact.segment)?.forceN ?? 0
      : 0), 0);
  const leftLoad = armLoad("left"), rightLoad = armLoad("right");
  const rollSide = rightLoad > leftLoad + 1e-8 ? "right" : leftLoad > rightLoad + 1e-8 ? "left"
    : armHeight("right", poses) < armHeight("left", poses) - 1e-8 ? "right" : "left";
  const mass = recoveryMassState(poses.values());
  const footMargin = input.supportMarginM ?? supportGeometry(
    contacts.filter(contact => isRecoveryFootSegment(contact.segment)), poses, mass,
  ).marginM;
  if (feet.length === 2 && footMargin >= 0) return { route: "crouch", leadingSide, rollSide };
  for (const { side } of feet) {
    const opposite = side === "left" ? "right" : "left";
    if (loadedContact(contacts, `${opposite}Shin`)) return { route: "half-kneel", leadingSide: side, rollSide };
  }
  const torso = poses.get("torso");
  const faceDown = torso && rotate(torso.rotation, { x: 0, y: 0, z: 1 }).y < -0.25;
  return { route: faceDown && leftLoad + rightLoad > 0 ? "prone" : "roll", leadingSide, rollSide };
}

export interface RecoveryArmBraceTarget {
  /** Hand centre and orientation. Capture these once; do not track a sliding hand. */
  position: Vec3;
  rotation: Quat;
  bend: Vec3;
  shoulder: Vec3;
  wrist: Vec3;
  elbow: Vec3;
  movementM: number;
  reachErrorM: number;
  floorClearanceM: number;
  floorReachable: boolean;
  jointLimitErrorRad: number;
  /** Local hand-from-forearm rotation, reusable through an unplanted trajectory. */
  wristRotation: Quat;
  trajectoryBend: Vec3;
}

/**
 * Numerical residue accepted while projecting a floor brace through the
 * bounded shoulder/elbow/wrist chain. The final command is still clamped by
 * the anatomical profiles and Rapier's structural constraints. The floor
 * allowance matches the contact/command clearance used by live recovery.
 */
export const RECOVERY_ARM_TARGET_TOLERANCE = Object.freeze({
  maximumReachErrorM: 0.004,
  maximumFloorPenetrationM: 0.012,
  maximumJointLimitErrorRad: 0.0005,
});

export function acceptableRecoveryArmBraceTarget(
  target: Pick<RecoveryArmBraceTarget, "reachErrorM" | "floorClearanceM" | "jointLimitErrorRad">,
  floorY = 0,
): boolean {
  return target.reachErrorM <= RECOVERY_ARM_TARGET_TOLERANCE.maximumReachErrorM
    && target.floorClearanceM >= floorY - RECOVERY_ARM_TARGET_TOLERANCE.maximumFloorPenetrationM
    && target.jointLimitErrorRad <= RECOVERY_ARM_TARGET_TOLERANCE.maximumJointLimitErrorRad;
}

/** A loaded sprawled arm cannot supply a useful upward brace about the shoulder. */
export function usableRecoveryArmSupport(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  contacts: readonly RecoverySupportContact[],
): boolean {
  const girdle = poses.get(`${side}ShoulderGirdle`);
  const upper = SEGMENT_BY_ID.get(`${side}UpperArm`)!;
  if (!girdle) return false;
  const shoulder = worldPoint(
    girdle.position,
    girdle.rotation,
    upper.jointProfile!.parentFrame.anchor,
  );
  return ([`${side}Hand`, `${side}ForearmTwist`, `${side}Forearm`] as SegmentId[]).some(id => {
    const contact = loadedContact(contacts, id), pose = poses.get(id);
    if (!contact || !pose) return false;
    const patch = contactPatch(contact, pose);
    const center = patch.length ? scale(patch.reduce(add, ZERO), 1 / patch.length) : contact.point;
    // The upper arm must remain over the brace rather than pulling on a hand
    // stretched behind the chest. A forearm can support only near its elbow.
    const maximumLever = id.endsWith("Hand")
      ? MAXIMUM_HAND_BRACE_LEVER_M
      : HUMAN_PROPORTIONS.arm.upperLengthM + 0.025;
    const minimumShoulderClearance = HUMAN_PROPORTIONS.arm.upperRadiusM
      + HUMAN_PROPORTIONS.arm.forearmRadiusM + .02;
    // A stale or synthetic load flag above the floor is not a support contact.
    return Math.abs(center.y) <= RECOVERY_ARM_TARGET_TOLERANCE.maximumFloorPenetrationM
      && length(horizontal(sub(center, shoulder))) <= maximumLever
      && shoulder.y - center.y >= minimumShoulderClearance;
  });
}

/** Nearest useful floor placement, solved from the measured shoulder in world space. */
export function reachableArmBraceTarget(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  heading: number,
  floorY = 0,
): RecoveryArmBraceTarget | null {
  const torso = poses.get("torso"), girdle = poses.get(`${side}ShoulderGirdle`);
  const hand = poses.get(`${side}Hand`);
  if (!torso || !girdle || !hand) return null;
  const girdleId = `${side}ShoulderGirdle` as SegmentId;
  const upperId = `${side}UpperArm` as SegmentId;
  const forearmId = `${side}Forearm` as SegmentId;
  const twistId = `${side}ForearmTwist` as SegmentId;
  const handId = `${side}Hand` as SegmentId;
  const upper = SEGMENT_BY_ID.get(upperId)!;
  const handDefinition = SEGMENT_BY_ID.get(handId)!;
  const shoulder = worldPoint(girdle.position, girdle.rotation, upper.jointProfile!.parentFrame.anchor);
  const sign = side === "left" ? -1 : 1, yaw = quatFromAxisAngle(UP, heading);
  const lateral = rotate(yaw, { x: sign, y: 0, z: 0 }), forward = rotate(yaw, { x: 0, y: 0, z: 1 });
  // The elbow folds toward the body's caudal side, not a fixed world-down
  // pole. This preserves natural flexion through prone and side orientations.
  const bend = normalize(add(scale(lateral, 0.20), scale(rotate(torso.rotation, UP), -0.65)));
  const delta = sub(hand.position, shoulder);
  const nearestLateral = clamp(dot(delta, lateral), 0.08, 0.25);
  const nearestForward = clamp(dot(delta, forward), -0.90, 0.15);
  const lateralOffsets = [nearestLateral, 0.08, 0.12, 0.16, 0.20, 0.25];
  const forwardOffsets = [nearestForward, ...Array.from({ length: 12 }, (_, index) => -0.90 + index * 0.10)];
  const handRadius = Math.max(...handDefinition.geometry.vertices.map(length));
  let best: RecoveryArmBraceTarget | null = null, bestScore = Infinity;
  for (const lateralOffset of lateralOffsets) for (const forwardOffset of forwardOffsets) for (const wristFlex of [-0.45, 0, 0.45]) {
    const horizontalCenter = add(shoulder, add(scale(lateral, lateralOffset), scale(forward, forwardOffset)));
    const wristRotation = recoveryJointRotation(handId, { x: wristFlex, y: 0, z: 0 });
    // With a fixed wrist, solve the complete arm to the hand centre exactly.
    // Only the scalar floor height remains implicit. Bisect it instead of
    // averaging orientations: the old fixed-count iteration could advertise
    // a reachable brace while leaving a millimetric endpoint/clearance error.
    let low = floorY + 0.002, high = low + handRadius;
    let requestedCenter = { ...horizontalCenter, y: (low + high) / 2 };
    let solved = solveRecoveryArmTarget(shoulder, requestedCenter, bend, wristRotation, heading);
    for (let iteration = 0; iteration < 40; iteration++) {
      requestedCenter = { ...horizontalCenter, y: (low + high) / 2 };
      solved = solveRecoveryArmTarget(shoulder, requestedCenter, bend, wristRotation, heading);
      const bottom = lowestWorldPoint(handDefinition.geometry, requestedCenter, solved.handRotation).y;
      if (bottom > floorY + 0.002) high = requestedCenter.y;
      else low = requestedCenter.y;
    }
    const rotation = solved.handRotation;
    const frame = { upper: solved.upperRotation, forearm: solved.forearmRotation };
    const rawGirdle = quatMultiply(quatInverse(torso.rotation), girdle.rotation);
    const girdleRotation = limitRecoveryJoint(girdleId, rawGirdle);
    const girdleWorld = quatMultiply(torso.rotation, girdleRotation);
    const rawUpper = quatMultiply(quatInverse(girdleWorld), frame.upper);
    const upperRotation = limitRecoveryJoint(upperId, rawUpper);
    const upperWorld = quatMultiply(girdleWorld, upperRotation);
    const rawForearm = quatMultiply(quatInverse(upperWorld), frame.forearm);
    const forearmRotation = limitRecoveryJoint(forearmId, rawForearm);
    const forearmWorld = quatMultiply(upperWorld, forearmRotation);
    const twistRotation = recoveryJointRotation(twistId, ZERO);
    const twistWorld = quatMultiply(forearmWorld, twistRotation);
    const rawHand = quatMultiply(quatInverse(twistWorld), rotation);
    const handRotation = limitRecoveryJoint(handId, rawHand);
    const rotations = new Map<SegmentId, Quat>([
      [girdleId, girdleRotation], [upperId, upperRotation], [forearmId, forearmRotation],
      [twistId, twistRotation], [handId, handRotation],
    ]);
    const rebuilt = reconstructRecoveryLimb(side, true, torso, rotations);
    const upperPose = rebuilt.poses.find(({ id }) => id === upperId)!;
    const handPose = rebuilt.poses.find(({ id }) => id === handId)!;
    const position = handPose.position;
    const actualWrist = worldPoint(position, handPose.rotation, handDefinition.jointProfile!.childFrame.anchor);
    const actualElbow = worldPoint(upperPose.position, upperPose.rotation,
      SEGMENT_BY_ID.get(forearmId)!.jointProfile!.parentFrame.anchor);
    const reachErrorM = length(sub(position, requestedCenter));
    const jointLimitErrorRad = recoveryJointLimitError(girdleId, rawGirdle)
      + recoveryJointLimitError(upperId, rawUpper)
      + recoveryJointLimitError(forearmId, rawForearm)
      + recoveryJointLimitError(handId, rawHand);
    const floorClearanceM = rebuilt.floorClearanceM;
    const geometricallyReachable = reachErrorM <= RECOVERY_ARM_TARGET_TOLERANCE.maximumReachErrorM
      && floorClearanceM >= floorY - RECOVERY_ARM_TARGET_TOLERANCE.maximumFloorPenetrationM;
    const movementM = length(sub(position, hand.position));
    const localPatch = handDefinition.geometry.supportPatch ?? handDefinition.geometry.vertices;
    const worldPatch = localPatch.map(point => worldPoint(position, handPose.rotation, point));
    const lowest = Math.min(...worldPatch.map(point => point.y));
    const patch = worldPatch.filter(point => point.y <= lowest + .003);
    const pressureCenter = patch.length ? scale(patch.reduce(add, ZERO), 1 / patch.length) : position;
    const contactLever = length(horizontal(sub(pressureCenter, shoulder)));
    const minimumShoulderClearance = HUMAN_PROPORTIONS.arm.upperRadiusM
      + HUMAN_PROPORTIONS.arm.forearmRadiusM + .02;
    const floorReachable = geometricallyReachable
      && contactLever <= MAXIMUM_HAND_BRACE_LEVER_M
      && shoulder.y - pressureCenter.y >= minimumShoulderClearance
      && jointLimitErrorRad <= RECOVERY_ARM_TARGET_TOLERANCE.maximumJointLimitErrorRad;
    // Judge the actual contacting edge, not just the hand centre. Keep a small
    // reserve for the measured contact to settle within the useful brace area.
    const supportCost = Math.max(0, contactLever - MAXIMUM_HAND_BRACE_LEVER_M) * 20;
    const score = (floorReachable ? 0 : 10 + reachErrorM) + jointLimitErrorRad * 10 + supportCost + movementM;
    if (score < bestScore - 1e-9) {
      bestScore = score;
      best = { position, rotation: handPose.rotation, bend, shoulder, wrist: actualWrist, elbow: actualElbow,
        movementM, reachErrorM, floorClearanceM, floorReachable, jointLimitErrorRad,
        wristRotation: handRotation,
        trajectoryBend: normalize(sub(sub(actualElbow, shoulder), scale(normalize(sub(position, shoulder)), dot(sub(actualElbow, shoulder), normalize(sub(position, shoulder)))))) };
    }
  }
  return best;
}

export interface RecoveryArmTargetGeometry {
  handPosition: Vec3;
  handRotation: Quat;
  upperRotation: Quat;
  forearmRotation: Quat;
  forearmTwistRotation: Quat;
  elbow: Vec3;
  wrist: Vec3;
  reachErrorM: number;
}

/** Follow a hand path as a two-bone chain with the wrist held relative to the forearm. */
export function solveRecoveryArmTarget(
  shoulder: Vec3,
  requestedHandCenter: Vec3,
  bend: Vec3,
  wristRotation: Quat,
  heading = 0,
): RecoveryArmTargetGeometry {
  const firstLength = HUMAN_PROPORTIONS.arm.upperLengthM;
  const forearmLength = HUMAN_PROPORTIONS.arm.forearmLengthM;
  const anchor = SEGMENT_BY_ID.get("leftHand")!.jointProfile!.childFrame.anchor;
  const localWrist = wristRotation;
  // With fixed local wrist flex, forearm + hand form one rigid effective bone.
  // Solving this exact geometry avoids a non-convergent wrist/orientation loop
  // when a clearance path passes close to the shoulder.
  const effectiveAxis = add({ x: 0, y: forearmLength, z: 0 }, rotate(localWrist, anchor));
  const effectiveLength = length(effectiveAxis);
  const effectiveOffset = quatFromTo(UP, normalize(effectiveAxis));
  const maximumBend = SEGMENT_BY_ID.get("leftForearm")!.jointProfile!.axes
    .find(({ coordinate }) => coordinate === "x")!.maxRadians;
  const solved = solveCapturedTwoBone(
    shoulder,
    requestedHandCenter,
    firstLength,
    effectiveLength,
    bend,
    maximumBend,
  );
  const axisA = normalize(sub(shoulder, solved.middle)), axisB = normalize(sub(solved.middle, solved.end));
  const yaw = quatFromAxisAngle(UP, heading);
  const localHinge = rotate(SEGMENT_BY_ID.get("leftForearm")!.jointProfile!.parentFrame.rotation, RIGHT);
  const upperRotation = hingeParentRotation(axisA, axisB, rotate(yaw, localHinge), localHinge);
  const effectiveRotation = quatMultiply(upperRotation, recoveryJointRotation("leftForearm", {
    x: Math.atan2(length(cross(axisA, axisB)), dot(axisA, axisB)), y: 0, z: 0,
  }));
  const forearmRotation = quatMultiply(effectiveRotation, quatInverse(effectiveOffset));
  const forearmTwistRotation = forearmRotation;
  const handRotation = quatMultiply(forearmRotation, localWrist);
  const wrist = sub(solved.middle, rotate(forearmRotation, { x: 0, y: forearmLength, z: 0 }));
  const handPosition = sub(wrist, rotate(handRotation, anchor));
  return { handPosition, handRotation, upperRotation, forearmRotation, forearmTwistRotation, elbow: solved.middle, wrist,
    reachErrorM: length(sub(handPosition, requestedHandCenter)) };
}



