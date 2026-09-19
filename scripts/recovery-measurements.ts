import type { Collider, World } from "@dimforge/rapier3d-compat";
import { SEGMENT_BY_ID } from "../src/core/humanoid";
import type { PoseSnapshot, SegmentId, Vec3 } from "../src/core/types";
import { add, length, quatInverse, rotate, scale, sub, worldPoint } from "../src/character/math";
import { measuredPhysicsMass as massState } from "./physics-mass";
import { jointCoordinates } from "../src/character/joint-coordinates";

const DT = 1 / 60;
const horizontalDistance = (a: Vec3, b: Vec3) => Math.hypot(a.x - b.x, a.z - b.z);
const recoveryMotion = (state: PoseSnapshot["state"]): boolean =>
  state === "falling" || state === "fallen" || state === "recovering";
const isFootSupport = (segment: SegmentId): boolean => {
  const role = SEGMENT_BY_ID.get(segment)?.role;
  return role === "hindfoot" || role === "forefoot";
};
interface MeasuredContact { segment: SegmentId; points: Vec3[]; forceN: number }
interface CapturedPlant { target: Vec3; materialPoint: Vec3; worldPoint: Vec3 }
export interface RecoveryPhysicsMeasurements {
  ages: Map<SegmentId, number>;
  planted: Map<SegmentId, CapturedPlant>;
  previousContacts: MeasuredContact[];
  deliberatelyReleased: Set<SegmentId>;
  unloadedReleased: Set<SegmentId>;
  report: {
    maxUpwardAssistanceN: number; maxPelvisTorqueNm: number; maxPlantedDriftM: number; maxTargetMotionM: number;
    maximumKneeReverseRadians: number; maximumElbowReverseRadians: number; transferMeasurements: Array<{ time: number; stage: string; leadingLoadFraction: number; centerOfMass: Vec3; loadedSegments: SegmentId[] }>;
    minimumReleaseMarginM: number | null; releaseCount: number; loadedFootFrames: number; asymmetricallyLoadedFrames: number;
    maxLegExtension: Record<"left" | "right", number>; minLegExtension: Record<"left" | "right", number>;
    routeEntries: Array<{ time: number; route: string; leadingSide: string | null; loadedSegments: SegmentId[]; centerOfMass: Vec3; leadingLoadFraction: number }>;
    releaseEvents: Array<{ time: number; released: SegmentId[]; marginM: number; remaining: SegmentId[] }>;
  };
}
export function newRecoveryPhysicsMeasurements(): RecoveryPhysicsMeasurements {
  return { ages: new Map(), planted: new Map(), previousContacts: [], deliberatelyReleased: new Set(), unloadedReleased: new Set(), report: {
    maxUpwardAssistanceN: 0, maxPelvisTorqueNm: 0, maxPlantedDriftM: 0, maxTargetMotionM: 0,
    maximumKneeReverseRadians: 0, maximumElbowReverseRadians: 0, transferMeasurements: [],
    minimumReleaseMarginM: null, releaseCount: 0, loadedFootFrames: 0, asymmetricallyLoadedFrames: 0,
    maxLegExtension: { left: 0, right: 0 }, minLegExtension: { left: 1, right: 1 }, routeEntries: [], releaseEvents: [],
  } };
}
/** Independent signed distance to the convex hull of actual solved floor contact points. */
function margin(points: Vec3[], projection: Vec3): number {
  const sorted = points.map(p => ({ ...p })).sort((a, b) => a.x - b.x || a.z - b.z)
    .filter((p, i, all) => !i || horizontalDistance(p, all[i - 1]) > 1e-8);
  if (sorted.length < 3) return -1;
  const cross = (a: Vec3, b: Vec3, c: Vec3) => (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
  const lower: Vec3[] = [], upper: Vec3[] = [];
  for (const point of sorted) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0) lower.pop(); lower.push(point); }
  for (const point of [...sorted].reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0) upper.pop(); upper.push(point); }
  const hull = [...lower.slice(0, -1), ...upper.slice(0, -1)];
  if (hull.length < 3) return -1;
  return Math.min(...hull.map((a, i) => cross(a, hull[(i + 1) % hull.length], projection) / horizontalDistance(a, hull[(i + 1) % hull.length])));
}
/** Evidence for an arm-assisted brace at floor level, before the torso has risen. */
export function measuredProneBraceSupport(snapshot: PoseSnapshot, contacts: MeasuredContact[]): boolean {
  const torso=snapshot.segments.find(p=>p.id==="torso")!;
  if(rotate(torso.rotation,{x:0,y:0,z:1}).y>=-.25)return false;
  const supportedArm=contacts.some(contact=>{
    if(!/Hand|Forearm/.test(contact.segment))return false;
    const side=contact.segment.startsWith("left")?"left":"right";
    const upper=SEGMENT_BY_ID.get(`${side}UpperArm`)!;
    const parent=snapshot.segments.find(p=>p.id===upper.parent)!;
    const shoulder=worldPoint(parent.position,parent.rotation,upper.jointAnchorParent!);
    return contact.points.some(point=>shoulder.y>point.y+.035 && horizontalDistance(point,shoulder)<=(contact.segment.endsWith("Hand")?.36:.335));
  });
  const mass=massState(snapshot), projected=add(mass.position,scale(mass.velocity,.15));
  return supportedArm && contacts.some(c=>/torso|pelvis|Thigh|Shin/.test(c.segment) || isFootSupport(c.segment))
    && margin(contacts.flatMap(c=>c.points),projected)>=-1e-5;
}

export function measureRecoveryPhysics(measured: RecoveryPhysicsMeasurements, snapshot: PoseSnapshot, previous: PoseSnapshot,
  internals: { world: World; floorCollider: Collider; ragdollColliders: Map<SegmentId, Collider> }, violations: Set<string>): void {
  const d = snapshot.diagnostics.recovery, before = previous.diagnostics.recovery, report = measured.report;
  const contacts: MeasuredContact[] = [], rawLoaded = new Set<SegmentId>();
  if (snapshot.diagnostics.physicsOwnership !== "rapier-dynamic") {
    violations.add("Physics ownership left the continuous Rapier dynamic assembly");
  }
  if (!recoveryMotion(snapshot.state)) { measured.planted.clear(); measured.ages.clear(); measured.previousContacts = []; measured.deliberatelyReleased.clear(); measured.unloadedReleased.clear(); return; }
  const integrated = snapshot.simulationTime > previous.simulationTime;
  if (d.retries !== before.retries) { measured.deliberatelyReleased.clear(); measured.unloadedReleased.clear(); }
  for (const [segment, collider] of internals.ragdollColliders) {
    let impulse = 0;
    const points: Vec3[] = [];
    if (internals.floorCollider.isEnabled()) internals.world.contactPair(internals.floorCollider, collider, (manifold, flipped) => {
      if (manifold.normal().y * (flipped ? -1 : 1) < 0.65) return;
      for (let index = 0; index < manifold.numSolverContacts(); index++) {
        const point = manifold.solverContactPoint(index);
        if (point && manifold.solverContactDist(index) <= 0.012) points.push({ ...point });
      }
      for (let index = 0; index < manifold.numContacts(); index++) impulse += Math.max(0, manifold.contactImpulse(index));
    });
    const forceN = impulse / DT, loaded = points.length > 0 && forceN >= 3;
    if (loaded) rawLoaded.add(segment);
    const age = loaded ? (measured.ages.get(segment) ?? 0) + (integrated ? DT : 0) : 0;
    measured.ages.set(segment, age);
    if (loaded && age + 1e-8 >= 0.05) contacts.push({ segment, forceN, points });
  }
  if (before.phase === "settle" && d.phase === "brace" && d.route === "prone") {
    const entryMass = massState(previous), projection = add(entryMass.position, scale(entryMass.velocity, 0.15));
    const loaded = measured.previousContacts;
    const torso = previous.segments.find(p => p.id === "torso")!;
    if (!loaded.some(c => /Hand|Forearm/.test(c.segment)) || !loaded.some(c => /torso|Thigh|Shin|Foot/.test(c.segment))
      || rotate(torso.rotation, { x: 0, y: 0, z: 1 }).y >= -0.25 || margin(loaded.flatMap(c => c.points), projection) < -1e-5) {
      violations.add("Prone brace shortcut lacks independently measured settled arm/body support and projected balance");
    }
  }
  if (before.phase === "settle" && d.phase === "stand") {
    const entryMass = massState(previous), projection = add(entryMass.position, scale(entryMass.velocity, 0.15));
    const loadedFeet = measured.previousContacts.filter(c => isFootSupport(c.segment));
    const loadedSides = new Set(loadedFeet.map(c => c.segment.startsWith("left") ? "left" : "right"));
    const torso = previous.segments.find(p => p.id === "torso")!, pelvis = previous.segments.find(p => p.id === "pelvis")!;
    if (loadedSides.size !== 2 || loadedFeet.some(c => rotate(previous.segments.find(p => p.id === c.segment)!.rotation, { x: 0, y: 1, z: 0 }).y <= 0.85)
      || rotate(torso.rotation, { x: 0, y: 1, z: 0 }).y <= 0.88 || pelvis.position.y <= 0.55
      || margin(loadedFeet.flatMap(c => c.points), projection) < -1e-5) {
      violations.add("Crouch rise shortcut lacks independently measured planted soles, entry height, upright torso and projected balance");
    }
  }
  const currentMass = massState(snapshot, internals.ragdollColliders);
  // Dynamic activation itself resets diagnostics without integrating recovery.
  const recoveryIntegrated=integrated && recoveryMotion(previous.state);
  if (recoveryIntegrated && length(sub(d.centerOfMass, currentMass.position)) > 1e-7) violations.add("Recovery COM is not the independent mass-weighted body center");
  const expectedProjection = add(currentMass.position, scale(currentMass.velocity, 0.15));
  if (recoveryIntegrated && horizontalDistance(d.projectedCenterOfMass, expectedProjection) > 1e-7) violations.add("Recovery projected COM does not use the independent 0.15 s mass-weighted velocity");
  report.maxUpwardAssistanceN = Math.max(report.maxUpwardAssistanceN, d.assistanceForce.y);
  report.maxPelvisTorqueNm = Math.max(report.maxPelvisTorqueNm, length(d.assistanceTorque));
  if (length(d.assistanceForce) > 1e-8) violations.add("Recovery includes direct pelvis assistance force");
  if (length(d.assistanceTorque) > 1e-8) violations.add("Recovery includes direct pelvis assistance torque");
  const feet = contacts.filter(c => isFootSupport(c.segment));
  if (feet.length) report.loadedFootFrames++;
  const footLoad = (side: string) => contacts
    .filter(c => c.segment.startsWith(side) && isFootSupport(c.segment))
    .reduce((sum, contact) => sum + contact.forceN, 0);
  const totalFootLoad = footLoad("left") + footLoad("right");
  if (totalFootLoad > 3 && Math.abs(footLoad("left") - footLoad("right")) / totalFootLoad > 0.35) report.asymmetricallyLoadedFrames++;
  if (d.route !== "none" && (d.route !== before.route || d.retries !== before.retries || before.phase === "settle" && d.phase !== "settle")) {
    report.routeEntries.push({ time: snapshot.simulationTime, route: d.route, leadingSide: d.leadingSide,
      loadedSegments: measured.previousContacts.map(c => c.segment), centerOfMass: currentMass.position,
      leadingLoadFraction: totalFootLoad > 0 && d.leadingSide ? footLoad(d.leadingSide) / totalFootLoad : 0 });
  }
  if (d.transferStage !== "none" && d.transferStage !== before.transferStage) report.transferMeasurements.push({
    time: snapshot.simulationTime, stage: d.transferStage, leadingLoadFraction: totalFootLoad > 0 && d.leadingSide ? footLoad(d.leadingSide) / totalFootLoad : 0,
    centerOfMass: currentMass.position, loadedSegments: contacts.map(c => c.segment),
  });
  if (d.route !== "none" && before.route !== "none" && d.retries === before.retries && before.phase !== "settle" && d.phase !== "settle"
    && (d.route !== before.route || d.leadingSide !== before.leadingSide || d.rollSide !== before.rollSide)) violations.add("Recovery route or side changed before a retry");
  for (const side of ["left", "right"] as const) {
    const thighId = `${side}Thigh` as SegmentId, shinId = `${side}Shin` as SegmentId;
    const ankleId = `${side}Ankle` as SegmentId, forearmId = `${side}Forearm` as SegmentId;
    const thigh = snapshot.segments.find(p => p.id === thighId)!, shin = snapshot.segments.find(p => p.id === shinId)!;
    const thighDefinition = SEGMENT_BY_ID.get(thighId)!, shinDefinition = SEGMENT_BY_ID.get(shinId)!;
    const ankleDefinition = SEGMENT_BY_ID.get(ankleId)!;
    const hip = worldPoint(thigh.position, thigh.rotation, thighDefinition.jointAnchorChild!);
    const ankle = worldPoint(shin.position, shin.rotation, ankleDefinition.jointAnchorParent!);
    const legLength = length(sub(thighDefinition.jointAnchorChild!, shinDefinition.jointAnchorParent!))
      + length(sub(shinDefinition.jointAnchorChild!, ankleDefinition.jointAnchorParent!));
    const extension = length(sub(ankle, hip)) / legLength;
    report.maxLegExtension[side] = Math.max(report.maxLegExtension[side], extension);
    report.minLegExtension[side] = Math.min(report.minLegExtension[side], extension);
    if (["roll", "brace", "kneel", "stand"].includes(d.phase)) {
      const upperArm = snapshot.segments.find(p => p.id === `${side}UpperArm`)!;
      const forearm = snapshot.segments.find(p => p.id === forearmId)!;
      const knee = jointCoordinates(thigh.rotation, shin.rotation, shinDefinition.jointProfile!);
      const elbowDefinition = SEGMENT_BY_ID.get(forearmId)!;
      const elbow = jointCoordinates(upperArm.rotation, forearm.rotation, elbowDefinition.jointProfile!);
      report.maximumKneeReverseRadians = Math.max(report.maximumKneeReverseRadians, Math.max(0, -knee.x));
      report.maximumElbowReverseRadians = Math.max(report.maximumElbowReverseRadians, Math.max(0, -elbow.x));
    }
  }
  for (const planted of d.plantedTargets) {
    const p = snapshot.segments.find(p => p.id === planted.segment)!;
    const contact = contacts.find(c => c.segment === planted.segment);
    let captured = measured.planted.get(planted.segment);
    if (captured && before.retries === d.retries && !d.releasedSupports.includes(planted.segment)) {
      const targetMotion = length(sub(captured.target, planted.position));
      report.maxTargetMotionM = Math.max(report.maxTargetMotionM, targetMotion);
      if (targetMotion > 1e-7) violations.add("Established support target followed a moving limb");
    }
    if (!captured && contact) {
      const point = contact.points[0];
      captured = { target: { ...planted.position }, worldPoint: point, materialPoint: rotate(quatInverse(p.rotation), sub(point, p.position)) };
      measured.planted.set(planted.segment, captured);
    }
    if (captured && contact) {
      const drift = horizontalDistance(add(p.position, rotate(p.rotation, captured.materialPoint)), captured.worldPoint);
      report.maxPlantedDriftM = Math.max(report.maxPlantedDriftM, drift);
      if (drift > 0.08) violations.add("Loaded planted limb slid more than 0.08 m before release");
    }
  }
  for (const segment of measured.planted.keys()) {
    if (!d.plantedTargets.some(p => p.segment === segment) || !contacts.some(c => c.segment === segment) || d.releasedSupports.includes(segment) || d.retries !== before.retries) measured.planted.delete(segment);
  }
  if (d.releasedSupports.length) {
    const mass = massState(previous), projection = add(mass.position, scale(mass.velocity, 0.15));
    const remaining = measured.previousContacts.filter(c => !d.releasedSupports.includes(c.segment) && !measured.deliberatelyReleased.has(c.segment));
    const measuredMargin = margin(remaining.flatMap(c => c.points), projection);
    report.releaseCount++;
    report.minimumReleaseMarginM = Math.min(report.minimumReleaseMarginM ?? Infinity, measuredMargin);
    report.releaseEvents.push({ time: snapshot.simulationTime, released: [...d.releasedSupports], marginM: measuredMargin, remaining: remaining.map(c => c.segment) });
    if (measuredMargin < -1e-5) violations.add("Support intentionally released outside independently measured 0.15 s remaining support area");
  }
  if (integrated) {
    // A deliberately released support cannot authorize another release from its
    // old manifold. Restore it only after independently observed unload/replant.
    for (const segment of measured.deliberatelyReleased) {
      if (!rawLoaded.has(segment)) measured.unloadedReleased.add(segment);
      else if (measured.unloadedReleased.has(segment) && contacts.some(c => c.segment === segment)) {
        measured.deliberatelyReleased.delete(segment); measured.unloadedReleased.delete(segment);
      }
    }
    for (const segment of d.releasedSupports) { measured.deliberatelyReleased.add(segment); measured.unloadedReleased.delete(segment); }
    measured.previousContacts = contacts;
  }
}
