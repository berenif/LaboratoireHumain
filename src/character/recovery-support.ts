import { HUMAN_PROPORTIONS, SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, SegmentPose, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, cross, dot, length, normalize, quatFromAxisAngle, quatFromTo, quatInverse, quatMultiply, quatNormalize, rotate, scale, sub, worldPoint } from "./math";
import { solveTwoBone } from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const SIDES = ["left", "right"] as const;
const GEOMETRY_EPSILON = 1e-8;

export type RecoveryRoute = "none" | "crouch" | "half-kneel" | "prone" | "roll";
export type RecoverySide = (typeof SIDES)[number];

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
  let position = ZERO, velocity = ZERO, massKg = 0;
  for (const pose of poses) {
    const mass = SEGMENT_BY_ID.get(pose.id)?.massKg ?? 0;
    position = add(position, scale(pose.position, mass));
    velocity = add(velocity, scale(pose.linearVelocity, mass));
    massKg += mass;
  }
  return massKg > 0
    ? { position: scale(position, 1 / massKg), velocity: scale(velocity, 1 / massKg), massKg }
    : { position: ZERO, velocity: ZERO, massKg: 0 };
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
  const shape = SEGMENT_BY_ID.get(contact.segment)?.shape;
  if (!pose || shape?.kind !== "box") return [contact.point];
  const half = shape.halfExtents;
  const corners: Vec3[] = [];
  for (const x of [-half.x, half.x]) for (const y of [-half.y, half.y]) for (const z of [-half.z, half.z]) {
    corners.push(worldPoint(pose.position, pose.rotation, { x, y, z }));
  }
  // Only the contacting face/edge contributes. A tilted foot must not acquire
  // a whole horizontal sole rectangle or a fictitious round support radius.
  const minimumY = Math.min(...corners.map(point => point.y));
  const floorY = contact.point.y;
  return corners.filter(point => point.y <= minimumY + 0.012 && Math.abs(point.y - floorY) <= 0.024);
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
  return Math.min(...([`${side}Hand`, `${side}Forearm`] as SegmentId[]).map(id => {
    const pose = poses.get(id), shape = SEGMENT_BY_ID.get(id)?.shape;
    if (!pose || !shape) return Infinity;
    if (shape.kind === "sphere") return pose.position.y - shape.radius;
    const axisY = Math.abs(rotate(pose.rotation, UP).y);
    if (shape.kind === "capsule") return pose.position.y - axisY * shape.halfHeight - shape.radius;
    const axisX = Math.abs(rotate(pose.rotation, { x: 1, y: 0, z: 0 }).y);
    const axisZ = Math.abs(rotate(pose.rotation, { x: 0, y: 0, z: 1 }).y);
    return pose.position.y - axisX * shape.halfExtents.x - axisY * shape.halfExtents.y - axisZ * shape.halfExtents.z;
  }));
}

/** Called once after settling, and retained by the controller until a retry. */
export function selectRecoveryRoute(input: RecoveryRouteInput): RecoveryRouteChoice {
  const { poses, contacts } = input;
  const feet = SIDES.map(side => ({ side, contact: loadedContact(contacts, `${side}Foot`), pose: poses.get(`${side}Foot`) }))
    .filter(({ contact, pose }) => contact && pose && rotate(pose.rotation, UP).y > 0.5);
  const travel = (side: RecoverySide): number => {
    const foot = poses.get(`${side}Foot`), target = input.footTargets?.[side];
    return foot && target ? length(sub(target, foot.position)) : Infinity;
  };
  // Stronger established support wins; ties use least movement, then stable side order.
  feet.sort((a, b) => b.contact!.forceN - a.contact!.forceN || travel(a.side) - travel(b.side)
    || SIDES.indexOf(a.side) - SIDES.indexOf(b.side));
  const leadingSide = feet[0]?.side ?? (travel("right") < travel("left") - 1e-8 ? "right" : "left");
  const armLoad = (side: RecoverySide): number => ["Hand", "Forearm"].reduce((sum, suffix) =>
    sum + (loadedContact(contacts, `${side}${suffix}` as SegmentId)?.forceN ?? 0), 0);
  const leftLoad = armLoad("left"), rightLoad = armLoad("right");
  const rollSide = rightLoad > leftLoad + 1e-8 ? "right" : leftLoad > rightLoad + 1e-8 ? "left"
    : armHeight("right", poses) < armHeight("left", poses) - 1e-8 ? "right" : "left";
  const mass = recoveryMassState(poses.values());
  const footMargin = input.supportMarginM ?? supportGeometry(
    contacts.filter(contact => contact.segment.endsWith("Foot")), poses, mass,
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
  floorReachable: boolean;
  jointLimitErrorRad: number;
  /** Local hand-from-forearm rotation, reusable through an unplanted trajectory. */
  wristRotation: Quat;
  trajectoryBend: Vec3;
}

/** A loaded sprawled arm cannot supply a useful upward brace about the shoulder. */
export function usableRecoveryArmSupport(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  contacts: readonly RecoverySupportContact[],
): boolean {
  const torso = poses.get("torso"), upper = SEGMENT_BY_ID.get(`${side}UpperArm`)!;
  if (!torso) return false;
  const shoulder = worldPoint(torso.position, torso.rotation, upper.jointAnchorParent!);
  return ([`${side}Hand`, `${side}Forearm`] as SegmentId[]).some(id => {
    const contact = loadedContact(contacts, id), pose = poses.get(id);
    if (!contact || !pose) return false;
    const patch = contactPatch(contact, pose);
    const center = patch.length ? scale(patch.reduce(add, ZERO), 1 / patch.length) : contact.point;
    // The upper arm must remain over the brace rather than pulling on a hand
    // stretched behind the chest. A forearm can support only near its elbow.
    const maximumLever = id.endsWith("Hand") ? 0.36 : HUMAN_PROPORTIONS.arm.upperLengthM + 0.025;
    return length(horizontal(sub(center, shoulder))) <= maximumLever
      && shoulder.y >= center.y - 0.025;
  });
}

/** Nearest useful floor placement, solved from the measured shoulder in world space. */
export function reachableArmBraceTarget(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  heading: number,
  floorY = 0,
): RecoveryArmBraceTarget | null {
  const torso = poses.get("torso"), hand = poses.get(`${side}Hand`);
  if (!torso || !hand) return null;
  const upper = SEGMENT_BY_ID.get(`${side}UpperArm`)!, handDefinition = SEGMENT_BY_ID.get(`${side}Hand`)!;
  const shoulder = worldPoint(torso.position, torso.rotation, upper.jointAnchorParent!);
  const sign = side === "left" ? -1 : 1, yaw = quatFromAxisAngle(UP, heading);
  const lateral = rotate(yaw, { x: sign, y: 0, z: 0 }), forward = rotate(yaw, { x: 0, y: 0, z: 1 });
  const half = HUMAN_PROPORTIONS.arm.handHalfExtentsM;
  // Backward relative to the measured torso is upward when prone. The lateral
  // component keeps mirrored elbows out of the chest and avoids yaw-only poles.
  const anatomical = add(scale(lateral, 0.20), scale(rotate(torso.rotation, { x: 0, y: 0, z: -1 }), 0.65));
  const bend = normalize({ ...anatomical, y: Math.max(0.25, anatomical.y) });
  const delta = sub(hand.position, shoulder);
  const nearestLateral = clamp(dot(delta, lateral), 0.08, 0.25);
  const nearestForward = clamp(dot(delta, forward), -0.20, 0.10);
  const lateralOffsets = [nearestLateral, 0.08, 0.12, 0.16, 0.20, 0.25];
  const forwardOffsets = [nearestForward, -0.20, -0.15, -0.10, -0.05, 0, 0.05, 0.10, 0.15, 0.20];
  const firstLength = HUMAN_PROPORTIONS.arm.upperLengthM, secondLength = HUMAN_PROPORTIONS.arm.forearmLengthM;
  const elbowLimit = SEGMENT_BY_ID.get(`${side}Forearm`)!.jointLimitRadians!.x;
  const minimumReach = Math.sqrt(firstLength ** 2 + secondLength ** 2 + 2 * firstLength * secondLength * Math.cos(elbowLimit));
  const armFrame = (elbow: Vec3, wrist: Vec3): { upper: Quat; forearm: Quat } => {
    const axisA = normalize(sub(shoulder, elbow)), axisB = normalize(sub(elbow, wrist));
    const hinge = normalize(scale(cross(axisA, axisB), -1), rotate(yaw, { x: 1, y: 0, z: 0 }));
    const base = quatFromTo(UP, axisA), baseX = rotate(base, { x: 1, y: 0, z: 0 });
    const twist = Math.atan2(dot(axisA, cross(baseX, hinge)), dot(baseX, hinge));
    const upper = quatMultiply(quatFromAxisAngle(axisA, twist), base);
    return { upper, forearm: quatMultiply(upper, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -Math.acos(clamp(dot(axisA, axisB), -1, 1)))) };
  };
  const limitError = (q: Quat, limit: Vec3): number => {
    const x = Math.asin(clamp(2 * (q.w * q.x - q.y * q.z), -1, 1));
    const y = Math.atan2(2 * (q.x * q.z + q.w * q.y), 1 - 2 * (q.x * q.x + q.y * q.y));
    const z = Math.atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z));
    const excess = (angles: Vec3): number => Math.hypot(Math.max(0, Math.abs(angles.x) - limit.x),
      Math.max(0, Math.abs(angles.y) - limit.y), Math.max(0, Math.abs(angles.z) - limit.z));
    const wrap = (angle: number): number => Math.atan2(Math.sin(angle), Math.cos(angle));
    return Math.min(excess({ x, y, z }), excess({ x: x >= 0 ? Math.PI - x : -Math.PI - x, y: wrap(y + Math.PI), z: wrap(z + Math.PI) }));
  };
  let best: RecoveryArmBraceTarget | null = null, bestScore = Infinity;
  for (const lateralOffset of lateralOffsets) for (const forwardOffset of forwardOffsets) for (const wristFlex of [-0.45, 0, 0.45]) {
    const horizontalCenter = add(shoulder, add(scale(lateral, lateralOffset), scale(forward, forwardOffset)));
    let rotation = hand.rotation, requestedWrist = shoulder;
    let solved = { middle: shoulder, end: shoulder } as Readonly<{ middle: Vec3; end: Vec3 }>;
    // Hand orientation and floor height depend on the forearm frame. Converge
    // them together instead of imposing an unreachable world wrist rotation.
    for (let iteration = 0; iteration < 48; iteration++) {
      const floorExtent = Math.abs(rotate(rotation, { x: 1, y: 0, z: 0 }).y) * half.x
        + Math.abs(rotate(rotation, UP).y) * half.y
        + Math.abs(rotate(rotation, { x: 0, y: 0, z: 1 }).y) * half.z;
      const requestedCenter = { ...horizontalCenter, y: floorY + floorExtent + 0.002 };
      requestedWrist = add(requestedCenter, rotate(rotation, handDefinition.jointAnchorChild!));
      const raw = sub(requestedWrist, shoulder), rawLength = length(raw);
      const limitedWrist = rawLength < minimumReach ? add(shoulder, scale(normalize(raw), minimumReach)) : requestedWrist;
      solved = solveTwoBone(shoulder, limitedWrist, firstLength, secondLength, bend);
      if (iteration === 47) break;
      let next = quatMultiply(armFrame(solved.middle, solved.end).forearm, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, wristFlex));
      if (rotation.x * next.x + rotation.y * next.y + rotation.z * next.z + rotation.w * next.w < 0)
        next = { x: -next.x, y: -next.y, z: -next.z, w: -next.w };
      rotation = quatNormalize({ x: rotation.x + next.x, y: rotation.y + next.y, z: rotation.z + next.z, w: rotation.w + next.w });
    }
    const position = sub(solved.end, rotate(rotation, handDefinition.jointAnchorChild!));
    const reachErrorM = length(sub(solved.end, requestedWrist));
    const frame = armFrame(solved.middle, solved.end);
    const jointLimitErrorRad = limitError(quatMultiply(quatInverse(torso.rotation), frame.upper), upper.jointLimitRadians!)
      + limitError(quatMultiply(quatInverse(frame.forearm), rotation), handDefinition.jointLimitRadians!);
    const floorReachable = reachErrorM < 1e-6 && solved.middle.y >= floorY + HUMAN_PROPORTIONS.arm.forearmRadiusM;
    const movementM = length(sub(position, hand.position));
    const corners: Vec3[] = [];
    for (const x of [-half.x, half.x]) for (const y of [-half.y, half.y]) for (const z of [-half.z, half.z])
      corners.push(worldPoint(position, rotation, { x, y, z }));
    const lowest = Math.min(...corners.map(point => point.y));
    const patch = corners.filter(point => point.y <= lowest + .003);
    const pressureCenter = scale(patch.reduce(add, ZERO), 1 / patch.length);
    const contactLever = length(horizontal(sub(pressureCenter, shoulder)));
    // Judge the actual contacting edge, not just the hand centre. Keep a small
    // reserve for the measured contact to settle within the useful brace area.
    const supportCost = Math.max(0, contactLever - .32) * 20;
    const score = (floorReachable ? 0 : 10 + reachErrorM) + jointLimitErrorRad * 10 + supportCost + movementM;
    if (score < bestScore - 1e-9) {
      bestScore = score;
      best = { position, rotation, bend, shoulder, wrist: solved.end, elbow: solved.middle,
        movementM, reachErrorM, floorReachable, jointLimitErrorRad,
        wristRotation: quatMultiply(quatInverse(frame.forearm), rotation),
        trajectoryBend: normalize(sub(sub(solved.middle, shoulder), scale(normalize(sub(position, shoulder)), dot(sub(solved.middle, shoulder), normalize(sub(position, shoulder)))))) };
    }
  }
  return best;
}

export interface RecoveryArmTargetGeometry {
  handPosition: Vec3;
  handRotation: Quat;
  upperRotation: Quat;
  forearmRotation: Quat;
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
  const firstLength = HUMAN_PROPORTIONS.arm.upperLengthM, forearmLength = HUMAN_PROPORTIONS.arm.forearmLengthM;
  const anchor = { x: 0, y: HUMAN_PROPORTIONS.arm.handHalfExtentsM.y, z: 0 };
  const wristAngle = 2 * Math.atan2(wristRotation.x, wristRotation.w);
  const wristFlex = clamp(Math.atan2(Math.sin(wristAngle), Math.cos(wristAngle)), -.6, .6);
  const localWrist = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, wristFlex);
  // With fixed local wrist flex, forearm + hand form one rigid effective bone.
  // Solving this exact geometry avoids a non-convergent wrist/orientation loop
  // when a clearance path passes close to the shoulder.
  const effectiveAxis = add({ x: 0, y: forearmLength, z: 0 }, rotate(localWrist, anchor));
  const effectiveLength = length(effectiveAxis), offsetAngle = Math.atan2(effectiveAxis.z, effectiveAxis.y);
  const minimumBend = Math.max(.025, .025 - offsetAngle), maximumBend = 2.35 - offsetAngle;
  const distanceAt = (angle: number): number => Math.sqrt(firstLength ** 2 + effectiveLength ** 2
    + 2 * firstLength * effectiveLength * Math.cos(angle));
  const delta = sub(requestedHandCenter, shoulder);
  const boundedCenter = add(shoulder, scale(normalize(delta), clamp(length(delta), distanceAt(maximumBend), distanceAt(minimumBend))));
  const solved = solveTwoBone(shoulder, boundedCenter, firstLength, effectiveLength, bend);
  const axisA = normalize(sub(shoulder, solved.middle)), axisB = normalize(sub(solved.middle, solved.end));
  const yaw = quatFromAxisAngle(UP, heading);
  const hinge = normalize(scale(cross(axisA, axisB), -1), rotate(yaw, { x: 1, y: 0, z: 0 }));
  const base = quatFromTo(UP, axisA), baseX = rotate(base, { x: 1, y: 0, z: 0 });
  const twist = Math.atan2(dot(axisA, cross(baseX, hinge)), dot(baseX, hinge));
  const upperRotation = quatMultiply(quatFromAxisAngle(axisA, twist), base);
  const effectiveRotation = quatMultiply(upperRotation, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -Math.acos(clamp(dot(axisA, axisB), -1, 1))));
  const forearmRotation = quatMultiply(effectiveRotation, quatFromAxisAngle({ x: 1, y: 0, z: 0 }, -offsetAngle));
  const handRotation = quatMultiply(forearmRotation, localWrist);
  const wrist = sub(solved.middle, rotate(forearmRotation, { x: 0, y: forearmLength, z: 0 }));
  const handPosition = sub(wrist, rotate(handRotation, anchor));
  return { handPosition, handRotation, upperRotation, forearmRotation, elbow: solved.middle, wrist,
    reachErrorM: length(sub(handPosition, requestedHandCenter)) };
}



