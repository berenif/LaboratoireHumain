import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { performance } from "node:perf_hooks";
import { measuredPhysicsMass as massState } from "./physics-mass";
import RAPIER, { type Collider, type RigidBody, type World } from "@dimforge/rapier3d-compat";
import { createEmbodiedCharacter } from "../src/character/index";
import { RECOVERY_LIMITS } from "../src/character/DynamicRecovery";
import { jointCoordinates, jointFrameAxesWorld, jointLimitErrorMagnitude, jointRotationFromCoordinates } from "../src/character/joint-coordinates";
import { add, clamp, length, lerp, normalize, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "../src/character/math";
import { createRapierJointLimitAdapter } from "../src/character/rapier-joint-adapter";
import { restPoseMap } from "../src/character/pose";
import { flattenGeometryIndices, flattenGeometryVertices } from "../src/core/geometry";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, SEGMENTS_BY_REGION, TOTAL_MASS_KG } from "../src/core/humanoid";
import { REGION_IDS, type CharacterController, type GrabCommand, type JointCoordinate, type MotionState, type PoseSnapshot, type Quat, type RecoveryPhase, type RegionId, type SegmentId, type SegmentPose, type Vec3 } from "../src/core/types";
import { SharedCameraProjection } from "../src/scene/camera";
import { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } from "./recovery-fixtures";
import { measureRecoveryPhysics, measuredProneBraceSupport, newRecoveryPhysicsMeasurements, type RecoveryPhysicsMeasurements } from "./recovery-measurements";
import { ACCEPTANCE, PULL_FIXTURES, NATIVE_REGRESSION_FIXTURES, RECOVERY_ACCEPTANCE, type PullFixture } from "./physics-fixtures";

await RAPIER.init();

const DT = ACCEPTANCE.fixedDtS, ZERO: Vec3 = { x: 0, y: 0, z: 0 }, UP: Vec3 = { x: 0, y: 1, z: 0 };
const KNOWN_FLOOR_PENETRATION_M = 0.035;
const LOCKED = new Set<MotionState>(["falling", "fallen", "recovering"]);
const FOOT_ROLES = new Set(["ankle", "hindfoot", "forefoot"]);
const isFootSegment = (id: SegmentId): boolean => FOOT_ROLES.has(SEGMENT_BY_ID.get(id)!.role);
const loadedFootSides = (contacts: PoseSnapshot["diagnostics"]["recovery"]["contacts"]): Set<"left" | "right"> => new Set(
  contacts
    .filter(contact => contact.loadBearing && isFootSegment(contact.segment))
    .flatMap(contact => SEGMENT_BY_ID.get(contact.segment)!.side ?? []),
);
const segmentsWithRoles = (...roles: string[]): SegmentId[] => SEGMENTS
  .filter(definition => roles.includes(definition.role))
  .map(definition => definition.id);
const ELIGIBLE: Record<RecoveryPhase, ReadonlyArray<SegmentId>> = {
  none: [], protect: [], settle: [],
  roll: segmentsWithRoles("pelvis", "lumbar", "ribcage", "shoulder-girdle", "upper-arm", "forearm", "forearm-twist", "hand", "thigh", "shin", "ankle", "hindfoot", "forefoot"),
  brace: segmentsWithRoles("forearm", "forearm-twist", "hand", "shin", "ankle", "hindfoot", "forefoot"),
  kneel: segmentsWithRoles("hand", "shin", "ankle", "hindfoot", "forefoot"),
  stand: segmentsWithRoles("ankle", "hindfoot", "forefoot"),
};
/** Private access is limited to read-only ownership checks and explicit acceptance fixtures. */
type Inspectable = CharacterController & {
  world: World; floorCollider: Collider; ragdollBodies: Map<SegmentId, RigidBody>; ragdollColliders: Map<SegmentId, Collider>;
  activateRagdoll?(direction: Vec3): void; finishRecovery?(): void;
};
type StateTransitionMeasurement = {kind:"fall-commit"|"recovery-complete";simulationTime:number;maxPositionJumpM:number;maxRotationJumpDegrees:number;maxLinearVelocityJumpMps:number;maxAngularVelocityJumpRadps:number;ownershipRetained:boolean};
type RecordData = {
  physicalRecovery: RecoveryPhysicsMeasurements; states: MotionState[]; phases: RecoveryPhase[]; violations: Set<string>;
  bodyRefs: Map<SegmentId, RigidBody>; colliderRefs: Map<SegmentId, Collider>;
  maxJointM: number; maxFloorM: number; maxLinearMps: number; maxAngularRadps: number; maxStepCount: number;
  maxJointLimitErrorRad: number; maxMotorSaturationRatio: number; maxMotorTorqueNm: number; maxContactCount: number;
  stateTransitions: StateTransitionMeasurement[];
  lockedFrames: number; supportedFrames: number; assistedFrames: number; firstFallS: number | null; recoveredS: number | null;
  final: PoseSnapshot; samples: number; floorPresent: boolean; protectiveElbowDegrees: number; protectiveHeadDegrees: number;
  protectiveTorsoDegrees: number; lateralNearArmTargetProgressDegrees: number; nearArmEntryErrorDegrees: number | null; lateralBracedContact: boolean; stableStandingS: number; landingContactS: number; orientations: Set<string>;
};
const results: Array<{ name: string; passed: boolean; durationMs: number; metrics: Record<string, unknown>; failures: string[] }> = [];
const trace: unknown[] = [], records = new Map<CharacterController, RecordData>();
let activeScenario = "";
const SCENARIO_PATTERN=process.env.PHYSICS_SCENARIO_PATTERN;
interface NativeStreamFixture {id:string;resetId:number;initialSnapshot:PoseSnapshot;updates:Array<{sequence:number;dt:number;command:GrabCommand|null}>;recordedTransfers:Array<{fallSequence:number;restoreSequence:number;recordedRecoverySeconds:number}>}
const NATIVE_STREAMS=JSON.parse(readFileSync(new URL("./fixtures/native-fixed-streams.json",import.meta.url),"utf8")) as {source:unknown;fixtures:NativeStreamFixture[]};

function pose(snapshot: PoseSnapshot, id: SegmentId): SegmentPose {
  const result = snapshot.segments.find(p => p.id === id);
  if (!result) throw new Error("Missing segment " + id);
  return result;
}
function angleDegrees(a: Quat, b: Quat): number {
  const cosine = Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (Math.hypot(a.x, a.y, a.z, a.w) * Math.hypot(b.x, b.y, b.z, b.w));
  return 2 * Math.acos(clamp(cosine, -1, 1)) * 180 / Math.PI;
}
function relative(snapshot: PoseSnapshot, child: SegmentId, parent: SegmentId): Quat {
  return quatMultiply(quatInverse(pose(snapshot, parent).rotation), pose(snapshot, child).rotation);
}
function jointError(snapshot: PoseSnapshot): number {
  let maximum = 0;
  for (const d of SEGMENTS) {
    if (!d.parent || !d.jointAnchorParent || !d.jointAnchorChild) continue;
    const parent = pose(snapshot, d.parent), child = pose(snapshot, d.id);
    maximum = Math.max(maximum, length(sub(worldPoint(parent.position, parent.rotation, d.jointAnchorParent), worldPoint(child.position, child.rotation, d.jointAnchorChild))));
  }
  return maximum;
}
const SEGMENT_SHAPES = new Map(SEGMENTS.map(definition => [
  definition.id,
  new RAPIER.ConvexPolyhedron(
    flattenGeometryVertices(definition.geometry),
    flattenGeometryIndices(definition.geometry),
  ),
]));
function floorError(snapshot: PoseSnapshot, character: CharacterController): number {
  const floor = (character as Inspectable).floorCollider;
  if (!floor.isEnabled()) return 0;
  let maximum=0;
  for (const definition of SEGMENTS) {
    const segment=pose(snapshot,definition.id);
    const contact=floor.contactShape(SEGMENT_SHAPES.get(definition.id)!,segment.position,segment.rotation,0);
    if(contact) maximum=Math.max(maximum,-contact.distance);
  }
  return maximum;
}
type LimitExercise = {
  segment: SegmentId;
  heading: number;
  direction: -1 | 1;
  coordinates: Vec3;
  limitErrorRad: number;
};
function exerciseStructuralLimit(
  segment: SegmentId,
  heading: number,
  direction: -1 | 1,
  combined = false,
): LimitExercise {
  const definition = SEGMENT_BY_ID.get(segment)!;
  const profile = definition.jointProfile!;
  const world = new RAPIER.World(ZERO);
  world.timestep = DT;
  world.numSolverIterations = 20;
  world.numInternalPgsIterations = 4;
  const parentRotation = quatFromAxisAngle(UP, heading);
  const childRotation = quatMultiply(parentRotation, jointRotationFromCoordinates(ZERO, profile));
  const parentPosition = ZERO;
  const childPosition = sub(
    worldPoint(parentPosition, parentRotation, profile.parentFrame.anchor),
    rotate(childRotation, profile.childFrame.anchor),
  );
  const parent = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(parentPosition.x,parentPosition.y,parentPosition.z)
    .setRotation(parentRotation).setAdditionalMassProperties(8,ZERO,{x:1,y:1,z:1},{x:0,y:0,z:0,w:1}).setCanSleep(false));
  const child = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(childPosition.x,childPosition.y,childPosition.z)
    .setRotation(childRotation).setAdditionalMassProperties(5,ZERO,{x:0.6,y:0.6,z:0.6},{x:0,y:0,z:0,w:1}).setCanSleep(false));
  const xHinge = profile.kind === "hinge" && profile.axes[0]?.coordinate === "x";
  const data = xHinge
    ? RAPIER.JointData.revoluteWithAxes(
      profile.parentFrame.anchor,
      profile.childFrame.anchor,
      rotate(profile.parentFrame.rotation,{x:1,y:0,z:0}),
      rotate(profile.childFrame.rotation,{x:1,y:0,z:0}),
    )
    : RAPIER.JointData.spherical(profile.parentFrame.anchor,profile.childFrame.anchor);
  const joint = world.createImpulseJoint(data,parent,child,true);
  joint.setLocalFrame1(profile.parentFrame.anchor,profile.parentFrame.rotation);
  joint.setLocalFrame2(profile.childFrame.anchor,profile.childFrame.rotation);
  const adapter=createRapierJointLimitAdapter(world);
  if(xHinge) adapter.constrain(joint,profile);
  else for(const coordinate of ["x","y","z"] as JointCoordinate[]){
      const axis=profile.axes.find(candidate=>candidate.coordinate===coordinate);
      adapter.constrainAxis(joint,axis??{coordinate,minRadians:0,maxRadians:0,passiveStiffnessNmPerRad:0,dampingNmsPerRad:0,maxMotorTorqueNm:0});
  }
  for(let frame=0;frame<240;frame++){
    const axes=jointFrameAxesWorld(parent.rotation(),profile);
    let torque=ZERO;
    for(const [index,axis] of profile.axes.entries()){
      const sign=combined ? (index%2===0?direction:-direction) : direction;
      torque=add(torque,scale(axes[axis.coordinate],sign*220));
    }
    child.applyTorqueImpulse(scale(torque,DT),true);
    parent.applyTorqueImpulse(scale(torque,-DT),true);
    world.step();
  }
  const coordinates=jointCoordinates(parent.rotation(),child.rotation(),profile);
  const result={segment,heading,direction,coordinates,limitErrorRad:jointLimitErrorMagnitude(coordinates,profile)};
  world.free();
  return result;
}
function motion(snapshot: PoseSnapshot): { linear: number; angular: number } {
  let linear = 0, angular = 0;
  for (const d of SEGMENTS) { const p = pose(snapshot, d.id); linear += d.massKg * length(p.linearVelocity) ** 2; angular += d.massKg * length(p.angularVelocity) ** 2; }
  return { linear: Math.sqrt(linear / TOTAL_MASS_KG), angular: Math.sqrt(angular / TOTAL_MASS_KG) };
}
function zeroExternal(snapshot: PoseSnapshot): boolean {
  const d = snapshot.diagnostics, g = d.grabControl;
  return !d.activeGrab && d.activePointerId === null && d.selectedRegion === null && !d.queuedTarget && d.appliedGrabForceN === 0
    && !g.active && g.storedUserForceN === 0 && g.storedUserTorqueNm === 0 && [g.force, g.torque, g.impulse, g.angularImpulse].every(v => v.x === 0 && v.y === 0 && v.z === 0);
}
const instrumentedBodies = new WeakSet<RigidBody>();
function establishAssemblyBaseline(r: RecordData, character: CharacterController): void {
  const internal = character as Inspectable;
  r.bodyRefs = new Map(internal.ragdollBodies);
  r.colliderRefs = new Map(internal.ragdollColliders);
  if (r.bodyRefs.size !== SEGMENTS.length || r.colliderRefs.size !== SEGMENTS.length) {
    r.violations.add("Continuous assembly does not contain all 25 anatomical segments");
  }
  for (const [id, body] of r.bodyRefs) {
    if (!body.isDynamic()) r.violations.add(`${id} is not a dynamic Rapier body`);
    if (instrumentedBodies.has(body)) continue;
    instrumentedBodies.add(body);
    for (const setter of ["setTranslation", "setRotation", "setLinvel", "setAngvel", "setBodyType", "setNextKinematicTranslation", "setNextKinematicRotation"] as const) {
      const original = body[setter] as (...args: unknown[]) => unknown;
      Object.defineProperty(body, setter, { configurable: true, value: function (...args: unknown[]) {
        r.violations.add(`Runtime physics state setter invoked for ${id}: ${setter}`);
        return original.apply(body, args);
      } });
    }
  }
}
function assertContinuousAssembly(r: RecordData, character: CharacterController): void {
  const internal = character as Inspectable;
  for (const definition of SEGMENTS) {
    const body = internal.ragdollBodies.get(definition.id);
    const collider = internal.ragdollColliders.get(definition.id);
    if (body !== r.bodyRefs.get(definition.id) || collider !== r.colliderRefs.get(definition.id)) {
      r.violations.add(`Rapier ownership changed without reset for ${definition.id}`);
    }
    if (!body?.isDynamic()) r.violations.add(`${definition.id} lost dynamic body type`);
  }
}
function measure(r: RecordData, snapshot: PoseSnapshot, character: CharacterController): void {
  const previous = r.final;
  r.samples++; r.final = snapshot;
  const d = snapshot.diagnostics, recovery = d.recovery;
  const physicalContactsBefore=r.physicalRecovery.previousContacts;
  measureRecoveryPhysics(r.physicalRecovery, snapshot, previous, character as Inspectable, r.violations);
  if (!d.finite || d.errors.length || snapshot.segments.some(p => ![...Object.values(p.position), ...Object.values(p.rotation), ...Object.values(p.linearVelocity), ...Object.values(p.angularVelocity)].every(Number.isFinite))) r.violations.add("Nonfinite pose/velocity or runtime error");
  if (r.states.at(-1) !== snapshot.state) r.states.push(snapshot.state);
  if (r.phases.at(-1) !== recovery.phase) r.phases.push(recovery.phase);
  r.maxJointM = Math.max(r.maxJointM, jointError(snapshot));
  if (r.floorPresent) r.maxFloorM = Math.max(r.maxFloorM, floorError(snapshot, character));
  r.maxLinearMps = Math.max(r.maxLinearMps, ...snapshot.segments.map(p => length(p.linearVelocity)));
  r.maxAngularRadps = Math.max(r.maxAngularRadps, ...snapshot.segments.map(p => length(p.angularVelocity)));
  r.maxStepCount = Math.max(r.maxStepCount, d.stepCount);
  r.maxJointLimitErrorRad = Math.max(r.maxJointLimitErrorRad, d.maxJointLimitErrorRad);
  r.maxMotorSaturationRatio = Math.max(r.maxMotorSaturationRatio, d.maxMotorSaturationRatio);
  r.maxMotorTorqueNm = Math.max(r.maxMotorTorqueNm, ...d.jointDiagnostics.map(joint => joint.motorTorqueNm));
  r.maxContactCount = Math.max(r.maxContactCount, d.contactDiagnostics.count);
  assertContinuousAssembly(r, character);
  if (d.physicsOwnership !== "rapier-dynamic") r.violations.add("Diagnostics lost continuous Rapier ownership");
  if (d.jointDiagnostics.length !== SEGMENTS.length - 1) r.violations.add("Joint diagnostics do not cover every articulated connection");
  for (const joint of d.jointDiagnostics) {
    const definition = SEGMENT_BY_ID.get(joint.segment)!;
    const profile = definition.jointProfile;
    if (!profile) { r.violations.add(`Unexpected joint diagnostics for ${joint.segment}`); continue; }
    const finiteJoint = [
      ...Object.values(joint.coordinates), ...Object.values(joint.targetCoordinates),
      ...Object.values(joint.limitError), joint.limitErrorMagnitudeRad,
      joint.motorTorqueNm, joint.motorSaturationRatio,
    ].every(Number.isFinite);
    if (!finiteJoint) r.violations.add(`Nonfinite joint diagnostics for ${joint.segment}`);
    const vectorCap = Math.hypot(...profile.axes.map(axis => axis.maxMotorTorqueNm));
    if (joint.motorTorqueNm > vectorCap + 1e-6) r.violations.add(`Motor torque exceeds anatomical profile for ${joint.segment}`);
  }
  const contacts = d.contactDiagnostics;
  if (![contacts.count, contacts.loadBearingCount, contacts.totalNormalForceN].every(Number.isFinite)
    || contacts.count < contacts.loadBearingCount || contacts.loadBearingCount < 0 || contacts.totalNormalForceN < 0) {
    r.violations.add("Invalid measured contact diagnostics");
  }
  if (LOCKED.has(snapshot.state)) {
    r.lockedFrames++; r.firstFallS ??= snapshot.simulationTime;
    if (d.bodyInputAvailable || !zeroExternal(snapshot)) r.violations.add("Body input/grab contribution survived lockout");
  } else if (!d.bodyInputAvailable) r.violations.add("Standing body input unavailable");
  if (r.firstFallS !== null && r.recoveredS === null && snapshot.state === "upright") r.recoveredS = snapshot.simulationTime;
  if (snapshot.support.swingFoot && snapshot.support.planted.includes(snapshot.support.swingFoot)) r.violations.add("Swinging foot counted as supporting");
  if (snapshot.support.planted.some(foot => pose(snapshot, foot).position.y > 0.10)) r.violations.add("Lifted foot counted as supporting");
  const currentMotion = motion(snapshot);
  const stableStanding = snapshot.state === "upright" && snapshot.support.planted.length === 2
    && currentMotion.linear <= RECOVERY_LIMITS.stableLinearMps && currentMotion.angular <= RECOVERY_LIMITS.stableAngularRadps
    && ["pelvis", "torso", "leftFoot", "rightFoot"].every(id => rotate(pose(snapshot,id as SegmentId).rotation,UP).y >= RECOVERY_LIMITS.stableUpDot);
  r.stableStandingS = stableStanding ? r.stableStandingS + Math.max(0,snapshot.simulationTime - previous.simulationTime) : 0;
  const aided = length(recovery.assistanceForce) > 1e-8 || length(recovery.assistanceTorque) > 1e-8;
  if (recovery.supporting.length) r.supportedFrames++;
  if (aided) {
    r.assistedFrames++;
    r.violations.add("Recovery reported forbidden direct pelvis assistance");
  }
  if (recovery.phase === "protect") {
    const supportedCrouch = loadedFootSides(recovery.contacts).size === 2
      && rotate(pose(snapshot,"torso").rotation, UP).y > 0.65 && pose(snapshot,"pelvis").position.y < 0.85;
    const loadedLandingContact = supportedCrouch || recovery.contacts.some(c => !isFootSegment(c.segment) && c.normalY >= RECOVERY_LIMITS.normalY && c.forceN >= RECOVERY_LIMITS.minimumLoadN);
    r.landingContactS = loadedLandingContact ? r.landingContactS + Math.max(0,snapshot.simulationTime - previous.simulationTime) : 0;
  }
  if (recovery.phase !== previous.diagnostics.recovery.phase) {
    const contacts = previous.diagnostics.recovery.contacts.filter(c => c.loadBearing);
    const feet = contacts.filter(c => isFootSegment(c.segment));
    const footSides = loadedFootSides(contacts);
    const shins = contacts.filter(c => c.segment === "leftShin" || c.segment === "rightShin");
    const hands = contacts.filter(c => /Hand|Forearm/.test(c.segment));
    const torsoUp = rotate(pose(previous,"torso").rotation, UP).y, pelvisHeight = pose(previous,"pelvis").position.y;
    if (previous.diagnostics.recovery.phase === "protect" && recovery.phase === "settle" && ((!previous.diagnostics.recovery.contacts.some(c => !isFootSegment(c.segment) && c.normalY >= RECOVERY_LIMITS.normalY && c.forceN >= RECOVERY_LIMITS.minimumLoadN) && !(footSides.size === 2 && torsoUp > 0.65 && pelvisHeight < 0.85)) || r.landingContactS + 1e-8 < RECOVERY_LIMITS.landingPersistenceS)) r.violations.add("Landing lacks 0.10 s consecutive loaded floor contact or balanced supported crouch");
    if (previous.diagnostics.recovery.phase === "settle" && ["roll", "brace", "kneel", "stand"].includes(recovery.phase) && (previous.diagnostics.recovery.settledTimeS + DT < RECOVERY_LIMITS.settlePersistenceS || !contacts.length)) r.violations.add("Recovery began without persistent settled contact");
    if (recovery.phase === "brace" && (!(hands.length || shins.length || feet.length) || (torsoUp <= 0.25 || pelvisHeight <= 0.28) && !measuredProneBraceSupport(previous,physicalContactsBefore))) r.violations.add("Brace phase advanced without contact/pose evidence");
    if (recovery.phase === "kneel" && (!(shins.length || feet.length) || torsoUp <= 0.65 || pelvisHeight <= 0.38)) r.violations.add("Kneel phase advanced without contact/pose evidence");
    if (recovery.phase === "stand" && (footSides.size !== 2 || torsoUp <= 0.88 || pelvisHeight <= 0.55)) r.violations.add("Stand phase advanced without contact/pose evidence");
  }
  for (const id of recovery.supporting) {
    const c = recovery.contacts.find(c => c.segment === id && c.loadBearing);
    if (!c || !ELIGIBLE[recovery.phase].includes(id) || c.normalY < RECOVERY_LIMITS.normalY || c.forceN < RECOVERY_LIMITS.minimumLoadN || c.persistenceS + 1e-8 < RECOVERY_LIMITS.loadPersistenceS) r.violations.add("Incidental contact classified as load-bearing phase support");
  }
  if (recovery.phase === "protect") {
    r.orientations.add(recovery.orientation);
    const identity = { x: 0, y: 0, z: 0, w: 1 };
    r.protectiveElbowDegrees = Math.max(r.protectiveElbowDegrees, ...(["left", "right"] as const).map(side => angleDegrees(relative(snapshot, (side + "Forearm") as SegmentId, (side + "UpperArm") as SegmentId), identity)));
    r.protectiveHeadDegrees = Math.max(r.protectiveHeadDegrees, angleDegrees(relative(snapshot, "head", "neck"), identity));
    r.protectiveTorsoDegrees = Math.max(r.protectiveTorsoDegrees,
      angleDegrees(relative(snapshot, "lumbar", "pelvis"), identity)
      + angleDegrees(relative(snapshot, "torso", "lumbar"), identity));
    if (recovery.orientation === "left" || recovery.orientation === "right") {
      const near = recovery.orientation, sign = near === "left" ? -1 : 1;
      const upperTarget = quatMultiply(quatFromAxisAngle({x:1,y:0,z:0},-1.25),quatFromAxisAngle({x:0,y:0,z:1},sign*1.1));
      const foreTarget = quatFromAxisAngle({x:1,y:0,z:0},-0.9);
      const upper = relative(snapshot,(near+"UpperArm") as SegmentId,(near+"ShoulderGirdle") as SegmentId), fore = relative(snapshot,(near+"Forearm") as SegmentId,(near+"UpperArm") as SegmentId);
      const error = angleDegrees(upper,upperTarget)+angleDegrees(fore,foreTarget);
      if (previous.diagnostics.recovery.phase !== "protect" || r.nearArmEntryErrorDegrees === null) r.nearArmEntryErrorDegrees=error;
      // Progress may bend an already outstretched arm inward. Requiring further outward travel would misclassify that brace.
      r.lateralNearArmTargetProgressDegrees=Math.max(r.lateralNearArmTargetProgressDegrees,r.nearArmEntryErrorDegrees-error);
      r.lateralBracedContact ||= angleDegrees(fore,identity)>=8 && recovery.contacts.some(c => (c.segment===near+"Hand" || c.segment===near+"Forearm" || c.segment===near+"ForearmTwist") && c.normalY>=RECOVERY_LIMITS.normalY && c.forceN>=RECOVERY_LIMITS.minimumLoadN);
    }
  }
  if (snapshot.sequence % 6 === 0) trace.push({ scenario: activeScenario, time: snapshot.simulationTime, state: snapshot.state, segments: snapshot.segments, diagnostics: d, jointErrorM: jointError(snapshot), floorErrorM: r.floorPresent ? floorError(snapshot, character) : 0 });
}

async function create(options: { heading?: number; position?: Vec3 } = {}): Promise<CharacterController> {
  const factory = createEmbodiedCharacter as unknown as (renderer: "canvas2d", options: { heading?: number; position?: Vec3 }) => Promise<CharacterController>;
  const character = await factory("canvas2d", options), initial = character.getSnapshot("canvas2d");
  const r: RecordData = { physicalRecovery: newRecoveryPhysicsMeasurements(), states: [], phases: [], violations: new Set(), bodyRefs: new Map(), colliderRefs: new Map(), maxJointM: 0, maxFloorM: 0, maxLinearMps: 0, maxAngularRadps: 0, maxStepCount: 0,
    maxJointLimitErrorRad: 0, maxMotorSaturationRatio: 0, maxMotorTorqueNm: 0, maxContactCount: 0,
    stateTransitions: [],
    lockedFrames: 0, supportedFrames: 0, assistedFrames: 0, firstFallS: null, recoveredS: null, final: initial, samples: 0, floorPresent: true,
    protectiveElbowDegrees: 0, protectiveHeadDegrees: 0, protectiveTorsoDegrees: 0, lateralNearArmTargetProgressDegrees: 0, nearArmEntryErrorDegrees: null, lateralBracedContact: false, stableStandingS: 0, landingContactS: 0, orientations: new Set() };
  records.set(character, r);
  establishAssemblyBaseline(r, character);
  const internal=character as Inspectable;
  for(const [method,kind] of [["activateRagdoll","fall-commit"],["finishRecovery","recovery-complete"]] as const){
    const original=internal[method];
    if(typeof original!=="function"){r.violations.add(`Missing state-only transition ${method}`);continue;}
    Object.defineProperty(internal,method,{configurable:true,writable:true,value:function(this:Inspectable,...args:unknown[]){
      const before=new Map([...this.ragdollBodies].map(([id,body])=>[id,{body,position:{...body.translation()},rotation:{...body.rotation()},linear:{...body.linvel()},angular:{...body.angvel()}}]));
      const simulationTime=this.getSnapshot("canvas2d").simulationTime;
      const result=(original as (...values:unknown[])=>unknown).apply(this,args);
      let maxPositionJumpM=0,maxRotationJumpDegrees=0,maxLinearVelocityJumpMps=0,maxAngularVelocityJumpRadps=0,ownershipRetained=true;
      for(const definition of SEGMENTS){const a=before.get(definition.id)!,body=this.ragdollBodies.get(definition.id)!;ownershipRetained&&=a.body===body;
        maxPositionJumpM=Math.max(maxPositionJumpM,length(sub(a.position,body.translation())));
        maxRotationJumpDegrees=Math.max(maxRotationJumpDegrees,angleDegrees(a.rotation,body.rotation()));
        maxLinearVelocityJumpMps=Math.max(maxLinearVelocityJumpMps,length(sub(a.linear,body.linvel())));
        maxAngularVelocityJumpRadps=Math.max(maxAngularVelocityJumpRadps,length(sub(a.angular,body.angvel())));
      }
      const measurement={kind,simulationTime,maxPositionJumpM,maxRotationJumpDegrees,maxLinearVelocityJumpMps,maxAngularVelocityJumpRadps,ownershipRetained};
      r.stateTransitions.push(measurement);
      if(!ownershipRetained || maxPositionJumpM>1e-10 || maxRotationJumpDegrees>ACCEPTANCE.trajectoryRotationToleranceDegrees || maxLinearVelocityJumpMps>1e-10 || maxAngularVelocityJumpRadps>1e-10) r.violations.add(`${kind} changed Rapier pose, velocity, or ownership`);
      return result;
    }});
  }
  const update = character.fixedUpdate.bind(character);
  character.fixedUpdate = (dt, command) => { update(dt, command); measure(r, character.getSnapshot("canvas2d"), character); };
  measure(r, initial, character); return character;
}
function advance(character: CharacterController, frames: number): void { for (let i = 0; i < frames; i++) character.fixedUpdate(DT, null); }
function begin(character: CharacterController, region: RegionId, pointerId: number, localAnchor: Vec3 = ZERO): { start: Vec3; command: GrabCommand } {
  const p = pose(character.getSnapshot("canvas2d"), region), start = worldPoint(p.position, p.rotation, localAnchor);
  return { start, command: { kind: "begin", pointerId, region, segment: region, localAnchor, worldTarget: start, timestampMs: character.getSnapshot("canvas2d").simulationTime * 1000 } };
}
function offsetAt(fixture: PullFixture, frame: number): Vec3 {
  const next = fixture.targets.findIndex(t => t.frame >= frame);
  if (next < 0) return fixture.targets.at(-1)!.offset;
  if (next === 0) return fixture.targets[0].offset;
  const a = fixture.targets[next - 1], b = fixture.targets[next]; return lerp(a.offset, b.offset, (frame - a.frame) / (b.frame - a.frame));
}
function commandAt(f: PullFixture, frame: number, start: Vec3, pointerId: number, release = f.releaseFrame): GrabCommand | null {
  if (frame === release) return { kind: "end", pointerId, timestampMs: frame * DT * 1000 };
  if (frame > release || f.holdWithoutCommandsAfterLastTarget && frame > f.targets.at(-1)!.frame) return null;
  return { kind: "move", pointerId, worldTarget: add(start, rotate(quatFromAxisAngle(UP, f.initial.heading), offsetAt(f, frame))), timestampMs: frame * DT * 1000 };
}
function describe(r: RecordData): Record<string, unknown> {
  return { physicalRecovery: r.physicalRecovery.report, states: r.states, phases: r.phases, samples: r.samples, maxJointSeparationM: r.maxJointM, maxFloorPenetrationM: r.maxFloorM,
    maxLinearMps: r.maxLinearMps, maxAngularRadps: r.maxAngularRadps, steps: r.maxStepCount, lockedFrames: r.lockedFrames, supportedFrames: r.supportedFrames, assistedFrames: r.assistedFrames,
    physicsOwnership: r.final.diagnostics.physicsOwnership, segmentCount: r.bodyRefs.size,
    maxJointLimitErrorRad: r.maxJointLimitErrorRad, maxMotorSaturationRatio: r.maxMotorSaturationRatio,
    maxMotorTorqueNm: r.maxMotorTorqueNm, maxContactCount: r.maxContactCount,
    stateTransitions: r.stateTransitions,
    firstFallS: r.firstFallS, recoveredS: r.recoveredS, recoveryDurationS: r.recoveredS !== null && r.firstFallS !== null ? r.recoveredS - r.firstFallS : null,
    finalState: r.final.state, finalRoot: r.final.rootPosition, finalRecovery: r.final.diagnostics.recovery,
    protectiveElbowDegrees: r.protectiveElbowDegrees, protectiveHeadDegrees: r.protectiveHeadDegrees, protectiveTorsoDegrees: r.protectiveTorsoDegrees,
    lateralNearArmTargetProgressDegrees: r.lateralNearArmTargetProgressDegrees, lateralBracedContact: r.lateralBracedContact, stableStandingS: r.stableStandingS, protectiveOrientations: [...r.orientations] };
}
async function scenario(name: string, run: (failures: string[]) => Promise<Record<string, unknown>>): Promise<void> {
  if(SCENARIO_PATTERN && !new RegExp(SCENARIO_PATTERN).test(name)) return;
  activeScenario = name; records.clear(); const start = performance.now(), failures: string[] = [];
  let metrics: Record<string, unknown> = {};
  try { metrics = await run(failures); } catch (error) { failures.push(error instanceof Error ? error.stack ?? error.message : String(error)); }
  const trajectories: Record<string, unknown>[] = [];
  for (const [character, r] of records) {
    failures.push(...r.violations);
    if (r.maxJointM > ACCEPTANCE.maxJointSeparationM) failures.push("Joint anchors separated " + r.maxJointM.toFixed(6) + " m > 0.08 m");
    if (r.maxFloorM > ACCEPTANCE.maxFloorPenetrationM) failures.push("Collider floor penetration " + r.maxFloorM.toFixed(6) + " m > 0.08 m");
    trajectories.push(describe(r)); character.dispose();
  }
  metrics.trajectories = trajectories;
  const result = { name, passed: failures.length === 0, durationMs: performance.now() - start, metrics, failures: [...new Set(failures)] };
  results.push(result); console.log((result.passed ? "PASS " : "FAIL ") + name + (result.failures.length ? ": " + result.failures.join("; ") : ""));
}
function requireRecovery(r: RecordData, failures: string[]): void {
  if (r.firstFallS === null) failures.push("Overpowering pull did not commit a fall");
  if (!r.states.includes("fallen")) failures.push("No contact-based landed/settled state observed");
  if (!r.states.includes("recovering")) failures.push("No dynamic recovering state observed");
  if (r.recoveredS === null || r.firstFallS === null || r.recoveredS - r.firstFallS > ACCEPTANCE.recoveryLimitS) failures.push("Automatic recovery did not complete within 25 simulated seconds");
  if (r.stableStandingS + 1e-8 < ACCEPTANCE.stableObservationS) failures.push("Recovered posture did not remain stably upright for one second");
  if (r.final.state !== "upright" || !r.final.diagnostics.bodyInputAvailable || r.final.support.planted.length !== 2) failures.push("Final state lacks stable standing and available body input");
  if (r.final.diagnostics.physicsOwnership !== "rapier-dynamic") failures.push("Recovery did not retain continuous Rapier ownership");
}
async function runPull(f: PullFixture, failures: string[]): Promise<Record<string, unknown>> {
  const c = await create(f.initial), r = records.get(c)!;
  advance(c,f.warmupFrames??0);
  const grab = begin(c, f.region, 41, f.localAnchor);
  if(f.initialTargetWorldOffset){grab.start=add(pose(c.getSnapshot("canvas2d"),f.region).position,f.initialTargetWorldOffset);grab.command.worldTarget=grab.start;}
  if(f.beginTimestampMs!==undefined)grab.command.timestampMs=f.beginTimestampMs;
  c.fixedUpdate(DT, grab.command);
  let release = f.releaseFrame, releasedInSwing = false;
  for (let frame = 1; frame <= f.observeUntilFrame; frame++) {
    if (f.releaseAtFirstSwing && !releasedInSwing && c.getSnapshot("canvas2d").support.swingFoot) { release = frame; releasedInSwing = true; }
    c.fixedUpdate(DT, commandAt(f, frame, grab.start, 41, release));
  }
  if (f.minimumSteps > r.maxStepCount) failures.push("Only " + r.maxStepCount + "/" + f.minimumSteps + " required corrective steps");
  if (f.releaseAtFirstSwing && !releasedInSwing) failures.push("No corrective swing existed for release fixture");
  if (f.outcome === "recoverable") {
    if (r.lockedFrames) failures.push("Recoverable pull entered falling/recovery");
    if (r.final.state !== "upright") failures.push("Recoverable pull finished in " + r.final.state);
  } else {
    requireRecovery(r, failures);
    if (f.direction && !r.orientations.has(f.direction)) failures.push("Missing " + f.direction + " protective orientation");
    if (f.direction === "forward" && r.protectiveElbowDegrees < 8) failures.push("Forward fall lacked bent protective arms");
    if (f.direction === "backward" && (r.protectiveHeadDegrees < 5 || r.protectiveTorsoDegrees < 8)) failures.push("Backward fall lacked head tuck and body bend");
    if ((f.direction === "left" || f.direction === "right") && r.lateralNearArmTargetProgressDegrees < 3 && !r.lateralBracedContact) failures.push("Near arm neither progressed 3 degrees toward its brace target nor established a bent-arm floor contact");
  }
  return { fixture: f, releasedAtFrame: release, releasedInSwing };
}

await scenario("expanded-anatomy-shared-geometry-and-ownership", async failures => {
  const c=await create(),internal=c as Inspectable,rest=restPoseMap();
  const ids=new Set(SEGMENTS.map(definition=>definition.id));
  if(SEGMENTS.length!==25 || ids.size!==25) failures.push("Anatomy does not contain 25 unique segments");
  if(Math.abs(TOTAL_MASS_KG-72.2)>1e-9) failures.push(`Anatomical mass is ${TOTAL_MASS_KG} kg, expected 72.2 kg`);
  let minimumY=Infinity,maximumY=-Infinity,maximumAnchorGap=0;
  for(const definition of SEGMENTS){
    const segment=rest.get(definition.id)!;
    if(definition.geometry.kind!=="convex" || definition.geometry.vertices.length<4 || !definition.geometry.triangles.length) failures.push(`Missing shared convex geometry for ${definition.id}`);
    for(const vertex of definition.geometry.vertices){const point=add(segment.position,rotate(segment.rotation,vertex));minimumY=Math.min(minimumY,point.y);maximumY=Math.max(maximumY,point.y);}
    if(definition.parent){
      const parent=rest.get(definition.parent)!;
      maximumAnchorGap=Math.max(maximumAnchorGap,length(sub(
        worldPoint(parent.position,parent.rotation,definition.jointAnchorParent!),
        worldPoint(segment.position,segment.rotation,definition.jointAnchorChild!),
      )));
      if(!definition.collisionExclusions.includes(definition.parent) || !SEGMENT_BY_ID.get(definition.parent)!.collisionExclusions.includes(definition.id)) failures.push(`Connected collision exclusion is not symmetric for ${definition.id}`);
    }
    const colliderShape=internal.ragdollColliders.get(definition.id)!.shape as RAPIER.ConvexPolyhedron;
    if(colliderShape.type!==RAPIER.ShapeType.ConvexPolyhedron || colliderShape.vertices.length!==flattenGeometryVertices(definition.geometry).length) failures.push(`Collider does not use canonical convex geometry for ${definition.id}`);
  }
  const stature=maximumY-minimumY;
  if(Math.abs(stature-HUMAN_PROPORTIONS.totalHeightM)>1e-9) failures.push(`Rest stature is ${stature} m, expected 1.84 m`);
  if(maximumAnchorGap>1e-9) failures.push(`Rest anatomy has a ${maximumAnchorGap} m anchor gap`);
  const actualMass=[...internal.ragdollBodies.values()].reduce((sum,body)=>sum+body.mass(),0);
  if(Math.abs(actualMass-TOTAL_MASS_KG)>1e-5) failures.push(`Rapier assembly mass is ${actualMass} kg, expected ${TOTAL_MASS_KG} kg`);
  const expectedNewSegments:SegmentId[]=["lumbar","leftShoulderGirdle","rightShoulderGirdle","leftForearmTwist","rightForearmTwist","leftAnkle","rightAnkle","leftForefoot","rightForefoot"];
  if(expectedNewSegments.some(id=>!ids.has(id))) failures.push("Expanded anatomical segments are incomplete");
  if(REGION_IDS.some(region=>!(SEGMENTS_BY_REGION.get(region)?.length))) failures.push("A selectable region has no anatomical segments");
  const canvas=c.getSnapshot("canvas2d"),webgl=c.getSnapshot("webgl");
  const rendererPoseDifference=Math.max(...SEGMENTS.map(definition=>{
    const a=pose(canvas,definition.id),b=pose(webgl,definition.id);
    return Math.max(length(sub(a.position,b.position)),angleDegrees(a.rotation,b.rotation));
  }));
  if(canvas.sequence!==webgl.sequence || rendererPoseDifference>ACCEPTANCE.trajectoryRotationToleranceDegrees) failures.push("Canvas2D and WebGL snapshots do not share one physical pose");
  return {segmentCount:SEGMENTS.length,massKg:TOTAL_MASS_KG,actualRapierMassKg:actualMass,statureM:stature,maximumAnchorGapM:maximumAnchorGap,rendererPoseDifference,regions:Object.fromEntries(REGION_IDS.map(region=>[region,SEGMENTS_BY_REGION.get(region)]))};
});

await scenario("solver-structural-joint-limits", async failures => {
  const exercises:LimitExercise[]=[];
  const oneWay:SegmentId[]=["leftForearm","rightForearm","leftShin","rightShin"];
  for(const [index,segment] of oneWay.entries()) for(const direction of [-1,1] as const) exercises.push(exerciseStructuralLimit(segment,[0,Math.PI/3,-Math.PI/4][index%3],direction));
  for(const segment of ["leftForearmTwist","rightForearmTwist","leftAnkle","rightAnkle"] as SegmentId[]) for(const direction of [-1,1] as const) exercises.push(exerciseStructuralLimit(segment,segment.startsWith("left")?Math.PI/3:-Math.PI/4,direction));
  for(const segment of ["leftUpperArm","rightUpperArm","leftThigh","rightThigh"] as SegmentId[]) exercises.push(exerciseStructuralLimit(segment,Math.PI/3,1,true));
  for(const exercise of exercises){
    const profile=SEGMENT_BY_ID.get(exercise.segment)!.jointProfile!;
    if(exercise.limitErrorRad>ACCEPTANCE.maximumStructuralLimitErrorRad) failures.push(`${exercise.segment} exceeded structural limits by ${exercise.limitErrorRad} rad`);
    for(const coordinate of ["x","y","z"] as JointCoordinate[]){
      const axis=profile.axes.find(candidate=>candidate.coordinate===coordinate);
      const value=exercise.coordinates[coordinate];
      if(!axis && Math.abs(value)>ACCEPTANCE.maximumStructuralLimitErrorRad) failures.push(`${exercise.segment} escaped its permitted rotation axes`);
      if(axis && (value<axis.minRadians-ACCEPTANCE.maximumStructuralLimitErrorRad || value>axis.maxRadians+ACCEPTANCE.maximumStructuralLimitErrorRad)) failures.push(`${exercise.segment} ${coordinate} coordinate escaped its asymmetric range`);
    }
    if(oneWay.includes(exercise.segment)){
      if(exercise.direction<0 && exercise.coordinates.x>0.08) failures.push(`${exercise.segment} bent in the forbidden hinge direction`);
      if(exercise.direction>0 && exercise.coordinates.x<0.2) failures.push(`${exercise.segment} did not flex under sustained permitted torque`);
    }
  }
  return {toleranceRad:ACCEPTANCE.maximumStructuralLimitErrorRad,exercises};
});

await scenario("frozen-recovery-threshold-contract", async failures => {
  for (const [key, value] of Object.entries(RECOVERY_ACCEPTANCE)) {
    if (RECOVERY_LIMITS[key as keyof typeof RECOVERY_LIMITS] !== value) failures.push("Recovery threshold changed after acceptance freeze: " + key);
  }
  return { expected: RECOVERY_ACCEPTANCE, implementation: RECOVERY_LIMITS };
});
await scenario("finite-floor-penetration-semantics", async failures => {
  const c=await create(),original=c.getSnapshot("canvas2d"),floor=(c as Inspectable).floorCollider;
  const foot=SEGMENT_BY_ID.get("leftFoot");
  if(!foot) throw new Error("Finite-floor fixture requires the left hindfoot geometry");
  const penetratingFootCenterY=-foot.geometry.localBounds.min.y-KNOWN_FLOOR_PENETRATION_M;
  const raised={...original,segments:original.segments.map(p=>({...p,position:add(p.position,{x:0,y:3,z:0})}))};
  const outside={...original,segments:original.segments.map(p=>({...p,position:add(p.position,{x:20,y:-2,z:0})}))};
  const penetrating={...raised,segments:raised.segments.map(p=>p.id==="leftFoot"?{...p,position:{x:0,y:penetratingFootCenterY,z:0},rotation:{x:0,y:0,z:0,w:1}}:p)};
  const clearance=floorError(raised,c),outsideDepth=floorError(outside,c),penetration=floorError(penetrating,c);
  floor.setEnabled(false);const disabled=floorError(penetrating,c);floor.setEnabled(true);
  if(clearance!==0 || outsideDepth!==0 || disabled!==0 || Math.abs(penetration-KNOWN_FLOOR_PENETRATION_M)>1e-6) failures.push("Finite floor query confuses clearance, absence, or outside-footprint geometry with penetration");
  return {clearanceM:clearance,outsideDepthM:outsideDepth,disabledDepthM:disabled,knownPenetrationM:penetration,expectedPenetrationM:KNOWN_FLOOR_PENETRATION_M,penetratingFootCenterY};
});
await scenario("idle-30-seconds", async failures => {
  const runs: Array<{ heading: number; pelvisDriftM: number; plantedFootDriftM: number; maxLinearMps: number; maxAngularRadps: number; steps: number; finalState: MotionState }> = [];
  for (const heading of [0, Math.PI / 3, -Math.PI / 4]) {
    const c = await create({ heading });
    advance(c, ACCEPTANCE.idleSettlingS * 60);
    const start = c.getSnapshot("canvas2d"), record = records.get(c)!;
    const startSteps=record.maxStepCount;
    let maxLinearMps=0,maxAngularRadps=0;
    for(let frame=0;frame<ACCEPTANCE.idleDurationS*60;frame++){
      c.fixedUpdate(DT,null);const sample=c.getSnapshot("canvas2d");
      maxLinearMps=Math.max(maxLinearMps,...sample.segments.map(segment=>length(segment.linearVelocity)));
      maxAngularRadps=Math.max(maxAngularRadps,...sample.segments.map(segment=>length(segment.angularVelocity)));
    }
    const end = c.getSnapshot("canvas2d");
    const pelvisDrift = Math.hypot(
      end.rootPosition.x-start.rootPosition.x,
      end.rootPosition.z-start.rootPosition.z,
    );
    const plantedFootDrift=Math.max(...(["leftFoot","leftForefoot","rightFoot","rightForefoot"] as SegmentId[]).map(id=>{
      const a=pose(start,id).position,b=pose(end,id).position;return Math.hypot(b.x-a.x,b.z-a.z);
    }));
    if (!(["upright","reacting"] as MotionState[]).includes(end.state) || pelvisDrift > ACCEPTANCE.idleRootDriftM || plantedFootDrift > ACCEPTANCE.idlePlantedFootDriftM
      || maxLinearMps > ACCEPTANCE.idleMaxLinearMps || maxAngularRadps > ACCEPTANCE.idleMaxAngularRadps
      || end.support.planted.length !== 2 || record.maxStepCount !== startSteps) {
      failures.push("Idle drifted, moved too quickly, stepped, or lost stable support at heading " + heading);
    }
    runs.push({
      heading,
      pelvisDriftM: pelvisDrift,
      plantedFootDriftM: plantedFootDrift,
      maxLinearMps,
      maxAngularRadps,
      steps: record.maxStepCount-startSteps,
      finalState: end.state,
    });
  }
  return { pelvisDriftM: Math.max(...runs.map(run => run.pelvisDriftM)), plantedFootDriftM: Math.max(...runs.map(run => run.plantedFootDriftM)), runs };
});
await scenario("seven-region-picking", async failures => {
  const c = await create(), snapshot = c.getSnapshot("canvas2d"), camera = new SharedCameraProjection().getState().position;
  const picked = REGION_IDS.map(region => {
    const hit=c.pick({ origin: camera, direction: normalize(sub(pose(snapshot, region).position, camera)) });
    const anchorErrorM=hit?length(sub(worldPoint(pose(snapshot,hit.segment).position,pose(snapshot,hit.segment).rotation,hit.localAnchor),hit.worldPoint)):Infinity;
    return {region,hitRegion:hit?.region,hitSegment:hit?.segment,anchorErrorM,groupContainsExactSegment:!!hit&&SEGMENTS_BY_REGION.get(region)?.includes(hit.segment)};
  });
  if (picked.some(result => result.region !== result.hitRegion || !result.groupContainsExactSegment || result.anchorErrorM>1e-7)) failures.push("Seven-region picking did not preserve the exact segment and local anchor");
  const forefoot=pose(snapshot,"leftForefoot"),forefootHit=c.pick({origin:add(forefoot.position,{x:0,y:-.5,z:0}),direction:UP});
  if(forefootHit?.segment!=="leftForefoot" || forefootHit.region!=="leftFoot") failures.push("Grouped foot picking did not retain the exact forefoot segment");
  return { picked,forefootHit };
});
for (const f of [...PULL_FIXTURES,...NATIVE_REGRESSION_FIXTURES]) await scenario(f.id, failures => runPull(f, failures));

for (const fixture of RECOVERY_POSE_FIXTURES) await scenario(fixture.id, async failures => {
  const c = await create({ heading: fixture.heading });
  seedRecoveryFixture(c, fixture);
  const initial = c.getSnapshot("canvas2d"), r = records.get(c)!;
  establishAssemblyBaseline(r,c);
  advance(c, (ACCEPTANCE.recoveryLimitS + ACCEPTANCE.stableObservationS + 1) * 60);
  requireRecovery(r, failures);
  if (!r.physicalRecovery.report.routeEntries.length) failures.push("Landed fixture never selected a support-driven route");
  return { fixture, initial, physical: r.physicalRecovery.report };
});

for(const fixture of NATIVE_STREAMS.fixtures) await scenario("captured-"+fixture.id,async failures=>{
  const c=await create(),r=records.get(c)!,initial=c.getSnapshot("canvas2d");
  for(const {id} of SEGMENTS){const expected=pose(fixture.initialSnapshot,id),actual=pose(initial,id);
    if(length(sub(expected.position,actual.position))>ACCEPTANCE.trajectoryPositionToleranceM || angleDegrees(expected.rotation,actual.rotation)>ACCEPTANCE.trajectoryRotationToleranceDegrees
      || length(sub(expected.linearVelocity,actual.linearVelocity))>ACCEPTANCE.trajectoryVelocityTolerance || length(sub(expected.angularVelocity,actual.angularVelocity))>ACCEPTANCE.trajectoryVelocityTolerance) failures.push("Native reset initial pose differs: "+id);
  }
  let lockedBeginAttempts=0,availableBeginAttempts=0,extraObservationFrames=0;
  for(const update of fixture.updates){
    if(update.dt!==DT)failures.push("Native stream does not use the frozen 60Hz timestep");
    if(update.command?.kind==="begin"){if(c.diagnostics().bodyInputAvailable)availableBeginAttempts++;else lockedBeginAttempts++;}
    c.fixedUpdate(update.dt,update.command);
    if(c.getSnapshot("canvas2d").sequence!==update.sequence) failures.push("Native fixed-step sequence mismatch");
  }
  // Preserve every recorded dt/command exactly, then observe any still-active final fall without new body input.
  while(extraObservationFrames<(ACCEPTANCE.recoveryLimitS+3)*60 && (LOCKED.has(c.diagnostics().state) || r.stableStandingS+1e-8<ACCEPTANCE.stableObservationS)){c.fixedUpdate(DT,null);extraObservationFrames++;}
  const falls=r.stateTransitions.filter(t=>t.kind==="fall-commit"),returns=r.stateTransitions.filter(t=>t.kind==="recovery-complete");
  for(const fall of falls){const returned=returns.find(t=>t.simulationTime>=fall.simulationTime);if(!returned || returned.simulationTime-fall.simulationTime>ACCEPTANCE.recoveryLimitS)failures.push("Captured native fall did not recover within25s");}
  requireRecovery(r,failures);
  return {fixture:fixture.id,recordedSteps:fixture.updates.length,extraObservationFrames,availableBeginAttempts,lockedBeginAttempts,observedFalls:falls.length,originalRecordedFalls:fixture.recordedTransfers.length,
    secondPressExpectation:"Accepted only when body input is available at its recorded fixed step; an input arriving during changed recovery timing must be rejected",originalSource:NATIVE_STREAMS.source};
});
await scenario("lockout-input-identical-trajectories", async failures => {
  const f = PULL_FIXTURES.find(f => f.id === "fast-forward")!, clean = await create(f.initial), attempted = await create(f.initial);
  const aGrab = begin(clean, f.region, 50, f.localAnchor), bGrab = begin(attempted, f.region, 50, f.localAnchor);
  clean.fixedUpdate(DT, aGrab.command); attempted.fixedUpdate(DT, bGrab.command);
  let maxPosition = 0, maxAngle = 0, maxVelocity = 0, attemptedFrames = 0, transitionMismatch = false;
  for (let frame = 1; frame <= f.observeUntilFrame; frame++) {
    const before = clean.getSnapshot("canvas2d");
    const locked = !before.diagnostics.bodyInputAvailable;
    const normal = commandAt(f, frame, aGrab.start, 50);
    clean.fixedUpdate(DT, normal);
    const intrusion: GrabCommand = frame % 3 === 0 ? { kind: "begin", pointerId: 900, region: "pelvis", segment: "pelvis", localAnchor: ZERO, worldTarget: { x: 4, y: 4, z: -4 }, timestampMs: frame * DT * 1000 }
      : { kind: "move", pointerId: frame % 2 ? 50 : 900, worldTarget: { x: Math.sin(frame) * 4, y: 3, z: Math.cos(frame) * 4 }, timestampMs: frame * DT * 1000 };
    attempted.fixedUpdate(DT, locked ? intrusion : normal);
    if (locked) attemptedFrames++;
    const a = clean.getSnapshot("canvas2d"), b = attempted.getSnapshot("canvas2d"); transitionMismatch ||= a.state !== b.state;
    transitionMismatch ||= a.diagnostics.bodyInputAvailable !== b.diagnostics.bodyInputAvailable
      || (a.diagnostics.balance === null) !== (b.diagnostics.balance === null);
    for (const d of SEGMENTS) { const pa = pose(a, d.id), pb = pose(b, d.id); maxPosition = Math.max(maxPosition, length(sub(pa.position, pb.position))); maxAngle = Math.max(maxAngle, angleDegrees(pa.rotation, pb.rotation)); maxVelocity = Math.max(maxVelocity, length(sub(pa.linearVelocity, pb.linearVelocity)), length(sub(pa.angularVelocity, pb.angularVelocity))); }
  }
  if (!attemptedFrames) failures.push("No locked input attempts exercised");
  if (transitionMismatch || maxPosition > ACCEPTANCE.trajectoryPositionToleranceM || maxAngle > ACCEPTANCE.trajectoryRotationToleranceDegrees || maxVelocity > ACCEPTANCE.trajectoryVelocityTolerance) failures.push("Lockout input changed segment trajectories or transitions");
  requireRecovery(records.get(clean)!, failures); requireRecovery(records.get(attempted)!, failures);
  const fresh = begin(attempted, "rightHand", 901); attempted.fixedUpdate(DT, fresh.command);
  if (!attempted.diagnostics().activeGrab) failures.push("Fresh press after recovery rejected"); attempted.clearBodyInput();
  return { attemptedFrames, maxPositionM: maxPosition, maxRotationDegrees: maxAngle, maxVelocityDifference: maxVelocity };
});

await scenario("floorless-center-of-mass-free-fall", async failures => {
  const c=await create(),r=records.get(c)!,internal=c as Inspectable;
  const start=massState(c.getSnapshot("canvas2d"), internal.ragdollColliders);
  internal.floorCollider.setEnabled(false);r.floorPresent=false;
  let previous=start,maximumUpwardVelocityCorrection=0,maximumHorizontalDrift=0;
  for(let frame=0;frame<60;frame++){
    c.fixedUpdate(DT,null);const snapshot=c.getSnapshot("canvas2d"),current=massState(snapshot,internal.ragdollColliders);
    maximumUpwardVelocityCorrection=Math.max(maximumUpwardVelocityCorrection,current.velocity.y-previous.velocity.y);
    maximumHorizontalDrift=Math.max(maximumHorizontalDrift,Math.hypot(current.position.x-start.position.x,current.position.z-start.position.z));
    if(snapshot.diagnostics.contactDiagnostics.loadBearingCount!==0) failures.push("Floorless assembly reported a load-bearing contact");
    if(length(snapshot.diagnostics.recovery.assistanceForce)>1e-8 || length(snapshot.diagnostics.recovery.assistanceTorque)>1e-8) failures.push("Floorless assembly reported direct pelvis assistance");
    previous=current;
  }
  const end=massState(c.getSnapshot("canvas2d"), internal.ragdollColliders);
  const velocityGain=end.velocity.y-start.velocity.y,drop=start.position.y-end.position.y;
  if(velocityGain>-6.5 || drop<3.2) failures.push("Internal motors arrested center-of-mass free fall");
  if(maximumUpwardVelocityCorrection>0.03) failures.push("Center-of-mass gained unsupported upward momentum");
  return {start,end,verticalVelocityChangeMps:velocityGain,dropM:drop,maximumUpwardVelocityCorrectionMps:maximumUpwardVelocityCorrection,maximumHorizontalDriftM:maximumHorizontalDrift};
});

await scenario("support-loss-clears-contacts-and-retries", async failures => {
  const f = PULL_FIXTURES.find(f => f.id === "fast-forward")!, c = await create(f.initial), r = records.get(c)!, internal = c as Inspectable;
  const grab = begin(c, f.region, 61, f.localAnchor); c.fixedUpdate(DT, grab.command);
  let supportedFrame = -1;
  for (let frame = 1; frame <= 25 * 60; frame++) { c.fixedUpdate(DT, commandAt(f, frame, grab.start, 61)); if (LOCKED.has(c.diagnostics().state) && c.diagnostics().recovery.supporting.length > 0) { supportedFrame = frame; break; } }
  if (supportedFrame < 0) { failures.push("No supported dynamic recovery reached"); return { supportedFrame }; }
  if (!internal.floorCollider) throw new Error("Floor collider unavailable for support-loss fixture");
  internal.floorCollider.setEnabled(false); r.floorPresent = false; c.fixedUpdate(DT, null);
  const immediate = c.diagnostics();
  if (length(immediate.recovery.assistanceForce) !== 0 || length(immediate.recovery.assistanceTorque) !== 0 || immediate.recovery.supporting.length) failures.push("Support loss retained contacts or forbidden pelvis assistance");
  internal.floorCollider.setEnabled(true); r.floorPresent = true;
  let supportReturnFrame = -1;
  for (let frame = 1; frame <= 300; frame++) {
    c.fixedUpdate(DT, null);
    const current = c.diagnostics();
    if (current.physicsOwnership === "rapier-dynamic" && current.recovery.supporting.length > 0) {
      supportReturnFrame = frame;
      break;
    }
  }
  if (supportReturnFrame < 0) failures.push("Floor restoration did not re-establish supported recovery");
  internal.floorCollider.setEnabled(false); r.floorPresent = false;
  const retriesBefore = c.diagnostics().recovery.retries;
  advance(c, Math.ceil(RECOVERY_LIMITS.supportLossS / DT) + 2);
  const unsupported = c.diagnostics();
  if (unsupported.physicsOwnership !== "rapier-dynamic" || unsupported.recovery.retries <= retriesBefore) failures.push("Support loss did not dynamically retry");
  advance(c, ACCEPTANCE.recoveryLimitS * 60);
  if (c.diagnostics().physicsOwnership !== "rapier-dynamic" || c.diagnostics().bodyInputAvailable || length(c.diagnostics().recovery.assistanceForce) !== 0 || length(c.diagnostics().recovery.assistanceTorque) !== 0) failures.push("Support-unavailable timeout forced standing or assistance");
  return { supportedFrame, supportReturnFrame, retriesBefore, retriesAfterLoss: unsupported.recovery.retries, afterLoss: immediate.recovery };
});

await scenario("supported-obstruction-stall-retries", async failures => {
  // Explicit initial rest pose, heading zero, central hand anchor, 1.25 m forward target reached in five ticks.
  const c = await create(), internal = c as Inspectable, grab = begin(c, "rightHand", 63, ZERO);
  c.fixedUpdate(DT, grab.command);
  let insertionFrame = -1;
  for (let frame = 1; frame <= 300; frame++) {
    c.fixedUpdate(DT, frame <= 5 ? { kind: "move", pointerId: 63, worldTarget: add(grab.start,{x:0,y:0.12*frame/5,z:1.25*frame/5}), timestampMs: frame*DT*1000 } : null);
    if (c.diagnostics().recovery.phase === "settle") { insertionFrame = frame; break; }
  }
  if (insertionFrame < 0) { failures.push("Obstruction fixture never reached contact-based settling"); return {insertionFrame}; }
  const ceilingBody = internal.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0,1.30,0));
  internal.world.createCollider(RAPIER.ColliderDesc.cuboid(4,0.05,4).setCollisionGroups((2<<16)|1),ceilingBody);
  const retriesBefore = c.diagnostics().recovery.retries;
  let maxStalledS = 0, supportedStallFrames = 0, forcedStanding = false;
  for (let frame = 0; frame < ACCEPTANCE.recoveryLimitS*60; frame++) {
    c.fixedUpdate(DT,null); const d = c.diagnostics();
    maxStalledS = Math.max(maxStalledS,d.recovery.stalledTimeS);
    if (d.recovery.supporting.length && d.recovery.stalledTimeS >= RECOVERY_LIMITS.stallS-0.2) supportedStallFrames++;
    forcedStanding ||= d.bodyInputAvailable || !LOCKED.has(d.state) || d.physicsOwnership !== "rapier-dynamic";
  }
  if (forcedStanding) failures.push("Obstructed recovery forced standing");
  if (c.diagnostics().recovery.retries <= retriesBefore || maxStalledS+DT+1e-8 < RECOVERY_LIMITS.stallS || !supportedStallFrames) failures.push("Supported obstructed phase did not reach stall threshold and dynamically retry");
  return { insertionFrame, ceiling:{center:{x:0,y:1.30,z:0},halfExtents:{x:4,y:0.05,z:4}}, observationS:25, retriesBefore, retriesAfter:c.diagnostics().recovery.retries,maxStalledS,supportedStallFrames };
});
await scenario("five-fall-recovery-cycles-without-reset", async failures => {
  const c = await create(), cycles: unknown[] = [];
  for (let cycle = 0; cycle < ACCEPTANCE.repeatedCycles; cycle++) {
    const f = PULL_FIXTURES.find(f => f.id === (cycle % 2 ? "fast-backward" : "fast-forward"))!, grab = begin(c, f.region, 100 + cycle, f.localAnchor), start = c.getSnapshot("canvas2d").simulationTime;
    c.fixedUpdate(DT, grab.command); let fell = false, finished = false;
    for (let frame = 1; frame <= ACCEPTANCE.recoveryLimitS * 60; frame++) { c.fixedUpdate(DT, commandAt(f, frame, grab.start, 100 + cycle)); const s = c.getSnapshot("canvas2d"); fell ||= LOCKED.has(s.state); if (fell && s.state === "upright" && records.get(c)!.stableStandingS + 1e-8 >= ACCEPTANCE.stableObservationS) { finished = true; break; } }
    if (!fell || !finished) failures.push("Cycle " + (cycle + 1) + " did not fall and recover within 25 s");
    advance(c, 60); const end = c.getSnapshot("canvas2d"); cycles.push({ cycle: cycle + 1, fell, finished, durationS: end.simulationTime - start, root: end.rootPosition, state: end.state }); if (!finished) break;
  }
  return { cycles };
});
for (const targetState of ["falling", "fallen", "recovering"] as const) await scenario("pause-reset-fresh-input-" + targetState, async failures => {
  const f = PULL_FIXTURES.find(f => f.id === "fast-forward")!, c = await create(), grab = begin(c, f.region, 71, f.localAnchor); c.fixedUpdate(DT, grab.command);
  for (let frame = 1; frame <= 25 * 60 && c.diagnostics().state !== targetState; frame++) c.fixedUpdate(DT, commandAt(f, frame, grab.start, 71));
  if (c.diagnostics().state !== targetState) { failures.push("Never reached " + targetState); return { targetState }; }
  const before = c.getSnapshot("canvas2d"); c.pause(); advance(c, 10); const paused = c.getSnapshot("canvas2d");
  if (paused.sequence !== before.sequence || paused.simulationTime !== before.simulationTime) failures.push("Pause advanced physics");
  c.resume(); c.fixedUpdate(DT, null); if (c.getSnapshot("canvas2d").sequence !== before.sequence + 1) failures.push("Resume did not restore one fixed step");
  c.reset(); establishAssemblyBaseline(records.get(c)!,c); const reset = c.diagnostics(); if (reset.state !== "upright" || reset.activeGrab || !reset.bodyInputAvailable || reset.fixedSteps !== 0 || reset.physicsOwnership!=="rapier-dynamic") failures.push("Reset did not restore clean dynamic standing");
  const fresh = begin(c, "leftHand", 72); c.fixedUpdate(DT, fresh.command); if (!c.diagnostics().activeGrab) failures.push("Fresh input rejected after Reset"); c.clearBodyInput(); return { targetState, pausedSequence: paused.sequence, resetState: reset.state };
});

if (!results.length) throw new Error("No physics scenarios matched " + (SCENARIO_PATTERN ?? "the configured selection"));
const failed = results.filter(r => !r.passed);
const report = { schema: 4, objective: "continuous-dynamic-humanoid-balance-fall-and-recovery", generatedAt: new Date().toISOString(), engine: "Rapier " + RAPIER.version(), node: process.version, fixedHz: 60,
  selection: SCENARIO_PATTERN ?? "all", acceptance: ACCEPTANCE, recoveryThresholds: RECOVERY_LIMITS, supportEligibility: ELIGIBLE, nativeCommandFixtureFile:"scripts/fixtures/native-fixed-streams.json", fixtures: [...PULL_FIXTURES,...NATIVE_REGRESSION_FIXTURES], landedPoseFixtures: RECOVERY_POSE_FIXTURES,
  measurement: { joints: "World-space anchor separation plus shared frame-coordinate limit error on every fixed update", floor: "Nonnegative penetration from an independent Rapier contactShape query between the actual finite enabled floor and each canonical convex polyhedron; clearance or missing floor is zero error", ownership: "The same 25 dynamic body and collider objects persist across standing, falling, and recovery; reset and explicit fixture seeding establish a new baseline", stateTransitions: "Fall commitment and recovery completion are wrapped at the same simulation instant to prove zero pose and velocity jump", trajectories: "Identical initial state, Rapier backend, build, and 1/60 s sequence; compare every segment every tick" },
  summary: { passed: failed.length === 0, passedScenarios: results.length - failed.length, failedScenarios: failed.length }, results };
mkdirSync("evidence", { recursive: true });
const outputPrefix=process.env.PHYSICS_OUTPUT_PREFIX ?? (SCENARIO_PATTERN?"evidence/physics-selected":"evidence/physics");
writeFileSync(outputPrefix+"-results.json", JSON.stringify(report, null, 2) + "\n");
writeFileSync(process.env.PHYSICS_OUTPUT_PREFIX ? outputPrefix+"-trace.ndjson" : SCENARIO_PATTERN?"evidence/physics-selected-trace.ndjson":"evidence/successor-trace.ndjson", trace.map(row => JSON.stringify(row)).join("\n") + "\n");
console.log(JSON.stringify(report.summary)); process.exitCode = failed.length ? 1 : 0;
