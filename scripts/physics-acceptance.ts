import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { performance } from "node:perf_hooks";
import RAPIER, { type Collider, type RigidBody, type World } from "@dimforge/rapier3d-compat";
import { createEmbodiedCharacter } from "../src/character/index";
import { RECOVERY_LIMITS } from "../src/character/DynamicRecovery";
import { add, clamp, clampLength, length, lerp, normalize, quatFromAxisAngle, quatInverse, quatMultiply, rotate, sub, worldPoint } from "../src/character/math";
import { SEGMENTS, TOTAL_MASS_KG } from "../src/core/humanoid";
import { REGION_IDS, type CharacterController, type GrabCommand, type MotionState, type PoseSnapshot, type Quat, type RecoveryPhase, type RegionId, type SegmentId, type SegmentPose, type Vec3 } from "../src/core/types";
import { SharedCameraProjection } from "../src/scene/camera";
import { ACCEPTANCE, PULL_FIXTURES, NATIVE_REGRESSION_FIXTURES, RECOVERY_ACCEPTANCE, type PullFixture } from "./physics-fixtures";

const DT = ACCEPTANCE.fixedDtS, ZERO: Vec3 = { x: 0, y: 0, z: 0 }, UP: Vec3 = { x: 0, y: 1, z: 0 };
const LOCKED = new Set<MotionState>(["falling", "fallen", "recovering"]);
const ELIGIBLE: Record<RecoveryPhase, ReadonlyArray<SegmentId>> = {
  none: [], protect: [], settle: [],
  roll: ["torso", "pelvis", "leftForearm", "rightForearm", "leftUpperArm", "rightUpperArm", "leftThigh", "rightThigh", "leftShin", "rightShin", "leftHand", "rightHand", "leftFoot", "rightFoot"],
  brace: ["leftHand", "rightHand", "leftForearm", "rightForearm", "leftShin", "rightShin", "leftFoot", "rightFoot"],
  kneel: ["leftShin", "rightShin", "leftFoot", "rightFoot", "leftHand", "rightHand"], stand: ["leftFoot", "rightFoot"],
};
/** Private adapter access is limited to transfer instrumentation and the explicit support-loss fixture. */
type Inspectable = CharacterController & { world: World; floorCollider: Collider; ragdollBodies: Map<SegmentId, RigidBody>; activateRagdoll(direction: Vec3): void; restoreUpright(): void };
type Transfer = { direction: string; simulationTime: number; translationM: number; rotationDegrees: number; bySegment: Array<{ id: SegmentId; translationM: number; rotationDegrees: number }> };
type RecordData = {
  states: MotionState[]; phases: RecoveryPhase[]; violations: Set<string>; transfers: Transfer[];
  maxJointM: number; maxFloorM: number; maxLinearMps: number; maxAngularRadps: number; maxStepCount: number;
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
const SEGMENT_SHAPES = new Map(SEGMENTS.map(d => [d.id, d.shape.kind === "sphere" ? new RAPIER.Ball(d.shape.radius)
  : d.shape.kind === "capsule" ? new RAPIER.Capsule(d.shape.halfHeight,d.shape.radius)
    : new RAPIER.Cuboid(d.shape.halfExtents.x,d.shape.halfExtents.y,d.shape.halfExtents.z)]));
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
function measure(r: RecordData, snapshot: PoseSnapshot, character: CharacterController): void {
  const previous = r.final;
  r.samples++; r.final = snapshot;
  const d = snapshot.diagnostics, recovery = d.recovery;
  if (!d.finite || d.errors.length || snapshot.segments.some(p => ![...Object.values(p.position), ...Object.values(p.rotation), ...Object.values(p.linearVelocity), ...Object.values(p.angularVelocity)].every(Number.isFinite))) r.violations.add("Nonfinite pose/velocity or runtime error");
  if (r.states.at(-1) !== snapshot.state) r.states.push(snapshot.state);
  if (r.phases.at(-1) !== recovery.phase) r.phases.push(recovery.phase);
  r.maxJointM = Math.max(r.maxJointM, jointError(snapshot));
  if (r.floorPresent) r.maxFloorM = Math.max(r.maxFloorM, floorError(snapshot, character));
  r.maxLinearMps = Math.max(r.maxLinearMps, ...snapshot.segments.map(p => length(p.linearVelocity)));
  r.maxAngularRadps = Math.max(r.maxAngularRadps, ...snapshot.segments.map(p => length(p.angularVelocity)));
  r.maxStepCount = Math.max(r.maxStepCount, d.stepCount);
  if (LOCKED.has(snapshot.state)) {
    r.lockedFrames++; r.firstFallS ??= snapshot.simulationTime;
    if (d.bodyInputAvailable || !zeroExternal(snapshot)) r.violations.add("Body input/grab contribution survived lockout");
    if (d.authority !== "ragdoll") r.violations.add("Locked state lost dynamic authority");
    const bodies = (character as Inspectable).ragdollBodies;
    if (bodies.size !== SEGMENTS.length || [...bodies.values()].some(b => !b.isDynamic())) r.violations.add("Falling/recovering segment is not dynamic");
  } else if (!d.bodyInputAvailable) r.violations.add("Standing body input unavailable");
  if (r.firstFallS !== null && r.recoveredS === null && snapshot.state === "upright") r.recoveredS = snapshot.simulationTime;
  if (snapshot.support.swingFoot && snapshot.support.planted.includes(snapshot.support.swingFoot)) r.violations.add("Swinging foot counted as supporting");
  if (d.authority === "character-motor" && snapshot.support.planted.some(foot => pose(snapshot, foot).position.y > 0.10)) r.violations.add("Lifted foot counted as supporting");
  const currentMotion = motion(snapshot);
  const stableStanding = snapshot.state === "upright" && snapshot.support.planted.length === 2
    && currentMotion.linear <= RECOVERY_LIMITS.stableLinearMps && currentMotion.angular <= RECOVERY_LIMITS.stableAngularRadps
    && ["pelvis", "torso", "leftFoot", "rightFoot"].every(id => rotate(pose(snapshot,id as SegmentId).rotation,UP).y >= RECOVERY_LIMITS.stableUpDot);
  r.stableStandingS = stableStanding ? r.stableStandingS + Math.max(0,snapshot.simulationTime - previous.simulationTime) : 0;
  const aided = length(recovery.assistanceForce) > 0 || length(recovery.assistanceTorque) > 0;
  if (recovery.supporting.length) r.supportedFrames++;
  if (aided) {
    r.assistedFrames++;
    if (!recovery.supporting.length) r.violations.add("Pelvis assistance without eligible support");
    if (length(recovery.assistanceForce) > recovery.assistanceForceCapN + 1e-8 || length(recovery.assistanceTorque) > recovery.assistanceTorqueCapNm + 1e-8) r.violations.add("Pelvis assistance exceeds force/torque cap");
  }
  if (recovery.maxMotorTorqueNm > ACCEPTANCE.maximumJointMotorTorqueNm + 1e-8) r.violations.add("Joint motor torque exceeds 110 Nm");
  if (recovery.phase === "protect") {
    const loadedLandingContact = recovery.contacts.some(c => !c.segment.endsWith("Foot") && c.normalY >= RECOVERY_LIMITS.normalY && c.forceN >= RECOVERY_LIMITS.minimumLoadN);
    r.landingContactS = loadedLandingContact ? r.landingContactS + Math.max(0,snapshot.simulationTime - previous.simulationTime) : 0;
  }
  if (recovery.phase !== previous.diagnostics.recovery.phase) {
    const contacts = previous.diagnostics.recovery.contacts.filter(c => c.loadBearing);
    const feet = contacts.filter(c => c.segment === "leftFoot" || c.segment === "rightFoot");
    const shins = contacts.filter(c => c.segment === "leftShin" || c.segment === "rightShin");
    const hands = contacts.filter(c => /Hand|Forearm/.test(c.segment));
    const torsoUp = rotate(pose(previous,"torso").rotation, UP).y, pelvisHeight = pose(previous,"pelvis").position.y;
    if (previous.diagnostics.recovery.phase === "protect" && recovery.phase === "settle" && (!previous.diagnostics.recovery.contacts.some(c => !c.segment.endsWith("Foot") && c.normalY >= RECOVERY_LIMITS.normalY && c.forceN >= RECOVERY_LIMITS.minimumLoadN) || r.landingContactS + 1e-8 < RECOVERY_LIMITS.landingPersistenceS)) r.violations.add("Landing lacks 0.10 s consecutive loaded non-foot floor contact");
    if (recovery.phase === "roll" && (previous.diagnostics.recovery.settledTimeS + DT < RECOVERY_LIMITS.settlePersistenceS || !contacts.length)) r.violations.add("Recovery began without persistent settled contact");
    if (recovery.phase === "brace" && (!(hands.length || shins.length || feet.length) || torsoUp <= 0.25 || pelvisHeight <= 0.28)) r.violations.add("Brace phase advanced without contact/pose evidence");
    if (recovery.phase === "kneel" && (!(shins.length || feet.length) || torsoUp <= 0.65 || pelvisHeight <= 0.38)) r.violations.add("Kneel phase advanced without contact/pose evidence");
    if (recovery.phase === "stand" && (feet.length !== 2 || torsoUp <= 0.88 || pelvisHeight <= 0.55)) r.violations.add("Stand phase advanced without contact/pose evidence");
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
    r.protectiveTorsoDegrees = Math.max(r.protectiveTorsoDegrees, angleDegrees(relative(snapshot, "torso", "pelvis"), identity));
    if (recovery.orientation === "left" || recovery.orientation === "right") {
      const near = recovery.orientation, sign = near === "left" ? -1 : 1;
      const upperTarget = quatMultiply(quatFromAxisAngle({x:1,y:0,z:0},-1.25),quatFromAxisAngle({x:0,y:0,z:1},sign*1.1));
      const foreTarget = quatFromAxisAngle({x:1,y:0,z:0},-0.9);
      const upper = relative(snapshot,(near+"UpperArm") as SegmentId,"torso"), fore = relative(snapshot,(near+"Forearm") as SegmentId,(near+"UpperArm") as SegmentId);
      const error = angleDegrees(upper,upperTarget)+angleDegrees(fore,foreTarget);
      if (previous.diagnostics.recovery.phase !== "protect" || r.nearArmEntryErrorDegrees === null) r.nearArmEntryErrorDegrees=error;
      // Progress may bend an already outstretched arm inward. Requiring further outward travel would misclassify that brace.
      r.lateralNearArmTargetProgressDegrees=Math.max(r.lateralNearArmTargetProgressDegrees,r.nearArmEntryErrorDegrees-error);
      r.lateralBracedContact ||= angleDegrees(fore,identity)>=8 && recovery.contacts.some(c => (c.segment===near+"Hand" || c.segment===near+"Forearm") && c.normalY>=RECOVERY_LIMITS.normalY && c.forceN>=RECOVERY_LIMITS.minimumLoadN);
    }
  }
  if (snapshot.sequence % 6 === 0) trace.push({ scenario: activeScenario, time: snapshot.simulationTime, state: snapshot.state, segments: snapshot.segments, diagnostics: d, jointErrorM: jointError(snapshot), floorErrorM: r.floorPresent ? floorError(snapshot, character) : 0 });
}

async function create(options: { heading?: number; position?: Vec3 } = {}): Promise<CharacterController> {
  const factory = createEmbodiedCharacter as unknown as (renderer: "canvas2d", options: { heading?: number; position?: Vec3 }) => Promise<CharacterController>;
  const character = await factory("canvas2d", options), initial = character.getSnapshot("canvas2d");
  const r: RecordData = { states: [], phases: [], violations: new Set(), transfers: [], maxJointM: 0, maxFloorM: 0, maxLinearMps: 0, maxAngularRadps: 0, maxStepCount: 0,
    lockedFrames: 0, supportedFrames: 0, assistedFrames: 0, firstFallS: null, recoveredS: null, final: initial, samples: 0, floorPresent: true,
    protectiveElbowDegrees: 0, protectiveHeadDegrees: 0, protectiveTorsoDegrees: 0, lateralNearArmTargetProgressDegrees: 0, nearArmEntryErrorDegrees: null, lateralBracedContact: false, stableStandingS: 0, landingContactS: 0, orientations: new Set() };
  records.set(character, r);
  const internals = character as Inspectable;
  for (const [method, direction] of [["activateRagdoll", "to-rapier"], ["restoreUpright", "to-procedural"]] as const) {
    const original = internals[method];
    if (typeof original !== "function") { r.violations.add("Missing " + method + " handoff instrumentation"); continue; }
    Object.defineProperty(internals, method, { configurable: true, writable: true, value: function (this: Inspectable, ...args: unknown[]) {
      const before = this.getSnapshot("canvas2d");
      if (direction==="to-procedural") {
        let actualLinear=0,actualAngular=0;
        for(const definition of SEGMENTS){const body=this.ragdollBodies.get(definition.id)!,cached=pose(before,definition.id);
          if(length(sub(body.linvel(),cached.linearVelocity))>ACCEPTANCE.trajectoryVelocityTolerance || length(sub(body.angvel(),cached.angularVelocity))>ACCEPTANCE.trajectoryVelocityTolerance) r.violations.add("Recovery handoff cached velocities differ from actual Rapier velocities");
          actualLinear+=definition.massKg*length(body.linvel())**2;actualAngular+=definition.massKg*length(body.angvel())**2;
        }
        if(Math.sqrt(actualLinear/TOTAL_MASS_KG)>RECOVERY_LIMITS.stableLinearMps || Math.sqrt(actualAngular/TOTAL_MASS_KG)>RECOVERY_LIMITS.stableAngularRadps) r.violations.add("Actual Rapier motion is unstable at recovery handoff");
      }
      const result = (original as (...args: unknown[]) => unknown).apply(this, args);
      const after = this.getSnapshot("canvas2d");
      const bySegment = SEGMENTS.map(({ id }) => ({ id, translationM: length(sub(pose(before, id).position, pose(after, id).position)), rotationDegrees: angleDegrees(pose(before, id).rotation, pose(after, id).rotation) }));
      r.transfers.push({ direction, simulationTime: before.simulationTime, translationM: Math.max(...bySegment.map(p => p.translationM)), rotationDegrees: Math.max(...bySegment.map(p => p.rotationDegrees)), bySegment });
      if (before.simulationTime !== after.simulationTime || before.sequence !== after.sequence) r.violations.add("Handoff measurement includes integration");
      if (direction === "to-procedural") {
        const recovery = before.diagnostics.recovery, m = motion(before);
        if (recovery.stableTimeS + DT + 1e-8 < RECOVERY_LIMITS.stablePersistenceS || !["leftFoot", "rightFoot"].every(id => recovery.contacts.some(c => c.segment === id && c.loadBearing))
          || m.linear > RECOVERY_LIMITS.stableLinearMps + 1e-8 || m.angular > RECOVERY_LIMITS.stableAngularRadps + 1e-8 || ["pelvis", "torso"].some(id => rotate(pose(before, id as SegmentId).rotation, UP).y < RECOVERY_LIMITS.stableUpDot)) r.violations.add("Upright transfer lacks persistent stable feet, low motion, or upright pose");
      }
      if (direction === "to-rapier") {
        for (const { id } of SEGMENTS) {
          const a = pose(before, id), b = pose(after, id);
          if (length(sub(b.linearVelocity, clampLength(a.linearVelocity, ACCEPTANCE.inheritedLinearVelocityMps))) > ACCEPTANCE.handoffVelocityTolerance
            || length(sub(b.angularVelocity, clampLength(a.angularVelocity, ACCEPTANCE.inheritedAngularVelocityRadps))) > ACCEPTANCE.handoffVelocityTolerance) r.violations.add("Transfer did not preserve bounded segment momentum");
          const body = this.ragdollBodies.get(id)!;
          for (const setter of ["setTranslation", "setRotation", "setLinvel", "setAngvel", "setBodyType", "setNextKinematicTranslation", "setNextKinematicRotation"] as const) {
            const originalSetter = body[setter] as (...args: unknown[]) => unknown;
            Object.defineProperty(body, setter, { configurable: true, value: function (...values: unknown[]) {
              r.violations.add("Dynamic recovery directly overwrote body pose/velocity/type via " + setter);
              return originalSetter.apply(body, values);
            } });
          }
          if (id === "pelvis") {
            const originalImpulse = body.applyImpulse.bind(body);
            Object.defineProperty(body, "applyImpulse", { configurable: true, value: (impulse: Vec3, wake: boolean) => {
              if (!r.floorPresent && length(impulse) > 0) r.violations.add("Unsupported pelvis received an assistance impulse");
              return originalImpulse(impulse, wake);
            } });
          }
        }
      }
      // Poses are captured before the operation; authority checks apply to its completed state.
      r.maxJointM = Math.max(r.maxJointM, jointError(before), jointError(after));
      if (r.floorPresent) r.maxFloorM = Math.max(r.maxFloorM, floorError(before, this), floorError(after, this));
      measure(r, after, this); return result;
    } });
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
  return { states: r.states, phases: r.phases, samples: r.samples, maxJointSeparationM: r.maxJointM, maxFloorPenetrationM: r.maxFloorM,
    maxLinearMps: r.maxLinearMps, maxAngularRadps: r.maxAngularRadps, steps: r.maxStepCount, lockedFrames: r.lockedFrames, supportedFrames: r.supportedFrames, assistedFrames: r.assistedFrames,
    firstFallS: r.firstFallS, recoveredS: r.recoveredS, recoveryDurationS: r.recoveredS !== null && r.firstFallS !== null ? r.recoveredS - r.firstFallS : null,
    transfers: r.transfers, finalState: r.final.state, finalRoot: r.final.rootPosition, finalRecovery: r.final.diagnostics.recovery,
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
    for (const t of r.transfers) {
      if (t.translationM > ACCEPTANCE.handoffTranslationM) failures.push(t.direction + " translation " + t.translationM + " m > 0.025 m");
      if (t.rotationDegrees > ACCEPTANCE.handoffRotationDegrees) failures.push(t.direction + " rotation " + t.rotationDegrees + " degrees > 3 degrees");
    }
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
  if (!r.transfers.some(t => t.direction === "to-rapier") || !r.transfers.some(t => t.direction === "to-procedural")) failures.push("Both authority handoffs were not measured");
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

await scenario("frozen-recovery-threshold-contract", async failures => {
  for (const [key, value] of Object.entries(RECOVERY_ACCEPTANCE)) {
    if (RECOVERY_LIMITS[key as keyof typeof RECOVERY_LIMITS] !== value) failures.push("Recovery threshold changed after acceptance freeze: " + key);
  }
  return { expected: RECOVERY_ACCEPTANCE, implementation: RECOVERY_LIMITS };
});
await scenario("finite-floor-penetration-semantics", async failures => {
  const c=await create(),original=c.getSnapshot("canvas2d"),floor=(c as Inspectable).floorCollider;
  const raised={...original,segments:original.segments.map(p=>({...p,position:add(p.position,{x:0,y:3,z:0})}))};
  const outside={...original,segments:original.segments.map(p=>({...p,position:add(p.position,{x:20,y:-2,z:0})}))};
  const penetrating={...raised,segments:raised.segments.map(p=>p.id==="leftFoot"?{...p,position:{x:0,y:0.03,z:0},rotation:{x:0,y:0,z:0,w:1}}:p)};
  const clearance=floorError(raised,c),outsideDepth=floorError(outside,c),penetration=floorError(penetrating,c);
  floor.setEnabled(false);const disabled=floorError(penetrating,c);floor.setEnabled(true);
  if(clearance!==0 || outsideDepth!==0 || disabled!==0 || Math.abs(penetration-0.035)>1e-6) failures.push("Finite floor query confuses clearance, absence, or outside-footprint geometry with penetration");
  return {clearanceM:clearance,outsideDepthM:outsideDepth,disabledDepthM:disabled,knownPenetrationM:penetration,expectedPenetrationM:0.035};
});
await scenario("idle-30-seconds", async failures => {
  const c = await create(), start = c.getSnapshot("canvas2d"); advance(c, ACCEPTANCE.idleDurationS * 60);
  const end = c.getSnapshot("canvas2d"), drift = length(sub(end.rootPosition, start.rootPosition));
  if (end.state !== "upright" || drift > ACCEPTANCE.idleRootDriftM || end.support.planted.length !== 2) failures.push("Idle drifted or lost stable support");
  return { driftM: drift };
});
await scenario("seven-region-picking", async failures => {
  const c = await create(), snapshot = c.getSnapshot("canvas2d"), camera = new SharedCameraProjection().getState().position;
  const picked = REGION_IDS.map(region => ({ region, hit: c.pick({ origin: camera, direction: normalize(sub(pose(snapshot, region).position, camera)) })?.region }));
  if (picked.some(p => p.region !== p.hit)) failures.push("Not all seven regions can be picked"); return { picked };
});
for (const f of [...PULL_FIXTURES,...NATIVE_REGRESSION_FIXTURES]) await scenario(f.id, failures => runPull(f, failures));

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
  const falls=r.transfers.filter(t=>t.direction==="to-rapier"),returns=r.transfers.filter(t=>t.direction==="to-procedural");
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
    const locked = LOCKED.has(clean.getSnapshot("canvas2d").state), normal = commandAt(f, frame, aGrab.start, 50);
    clean.fixedUpdate(DT, normal);
    const intrusion: GrabCommand = frame % 3 === 0 ? { kind: "begin", pointerId: 900, region: "pelvis", segment: "pelvis", localAnchor: ZERO, worldTarget: { x: 4, y: 4, z: -4 }, timestampMs: frame * DT * 1000 }
      : { kind: "move", pointerId: frame % 2 ? 50 : 900, worldTarget: { x: Math.sin(frame) * 4, y: 3, z: Math.cos(frame) * 4 }, timestampMs: frame * DT * 1000 };
    attempted.fixedUpdate(DT, locked ? intrusion : normal); if (locked) attemptedFrames++;
    const a = clean.getSnapshot("canvas2d"), b = attempted.getSnapshot("canvas2d"); transitionMismatch ||= a.state !== b.state;
    for (const d of SEGMENTS) { const pa = pose(a, d.id), pb = pose(b, d.id); maxPosition = Math.max(maxPosition, length(sub(pa.position, pb.position))); maxAngle = Math.max(maxAngle, angleDegrees(pa.rotation, pb.rotation)); maxVelocity = Math.max(maxVelocity, length(sub(pa.linearVelocity, pb.linearVelocity)), length(sub(pa.angularVelocity, pb.angularVelocity))); }
  }
  if (!attemptedFrames) failures.push("No locked input attempts exercised");
  if (transitionMismatch || maxPosition > ACCEPTANCE.trajectoryPositionToleranceM || maxAngle > ACCEPTANCE.trajectoryRotationToleranceDegrees || maxVelocity > ACCEPTANCE.trajectoryVelocityTolerance) failures.push("Lockout input changed segment trajectories or transitions");
  requireRecovery(records.get(clean)!, failures); requireRecovery(records.get(attempted)!, failures);
  const fresh = begin(attempted, "rightHand", 901); attempted.fixedUpdate(DT, fresh.command);
  if (!attempted.diagnostics().activeGrab) failures.push("Fresh press after recovery rejected"); attempted.clearBodyInput();
  return { attemptedFrames, maxPositionM: maxPosition, maxRotationDegrees: maxAngle, maxVelocityDifference: maxVelocity };
});

await scenario("support-loss-stops-assistance-and-retries", async failures => {
  const f = PULL_FIXTURES.find(f => f.id === "fast-forward")!, c = await create(f.initial), r = records.get(c)!, internal = c as Inspectable;
  const grab = begin(c, f.region, 61, f.localAnchor); c.fixedUpdate(DT, grab.command);
  let assistanceFrame = -1;
  for (let frame = 1; frame <= 25 * 60; frame++) { c.fixedUpdate(DT, commandAt(f, frame, grab.start, 61)); if (length(c.diagnostics().recovery.assistanceForce) > 0) { assistanceFrame = frame; break; } }
  if (assistanceFrame < 0) { failures.push("No supported pelvis assistance reached"); return { assistanceFrame }; }
  if (!internal.floorCollider) throw new Error("Floor collider unavailable for support-loss fixture");
  internal.floorCollider.setEnabled(false); r.floorPresent = false; c.fixedUpdate(DT, null);
  const immediate = c.diagnostics();
  if (length(immediate.recovery.assistanceForce) !== 0 || length(immediate.recovery.assistanceTorque) !== 0 || immediate.recovery.supporting.length) failures.push("Support loss did not stop assistance before the next integration");
  internal.floorCollider.setEnabled(true); r.floorPresent = true; advance(c, 60);
  internal.floorCollider.setEnabled(false); r.floorPresent = false;
  const retriesBefore = c.diagnostics().recovery.retries;
  advance(c, Math.ceil(RECOVERY_LIMITS.supportLossS / DT) + 2);
  const unsupported = c.diagnostics();
  if (unsupported.authority !== "ragdoll" || unsupported.recovery.retries <= retriesBefore) failures.push("Support loss did not dynamically retry");
  advance(c, ACCEPTANCE.recoveryLimitS * 60);
  if (c.diagnostics().authority !== "ragdoll" || c.diagnostics().bodyInputAvailable || length(c.diagnostics().recovery.assistanceForce) !== 0) failures.push("Support-unavailable timeout forced standing or assistance");
  return { assistanceFrame, retriesBefore, retriesAfterLoss: unsupported.recovery.retries, afterLoss: immediate.recovery };
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
    forcedStanding ||= d.bodyInputAvailable || d.authority !== "ragdoll";
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
  c.reset(); const reset = c.diagnostics(); if (reset.state !== "upright" || reset.activeGrab || !reset.bodyInputAvailable || reset.fixedSteps !== 0) failures.push("Reset did not restore clean standing");
  const fresh = begin(c, "leftHand", 72); c.fixedUpdate(DT, fresh.command); if (!c.diagnostics().activeGrab) failures.push("Fresh input rejected after Reset"); c.clearBodyInput(); return { targetState, pausedSequence: paused.sequence, resetState: reset.state };
});

const failed = results.filter(r => !r.passed);
const report = { schema: 3, objective: "support-and-momentum-balance-protective-fall-dynamic-recovery", generatedAt: new Date().toISOString(), engine: "Rapier " + RAPIER.version(), node: process.version, fixedHz: 60,
  selection: SCENARIO_PATTERN ?? "all", acceptance: ACCEPTANCE, recoveryThresholds: RECOVERY_LIMITS, supportEligibility: ELIGIBLE, nativeCommandFixtureFile:"scripts/fixtures/native-fixed-streams.json", fixtures: [...PULL_FIXTURES,...NATIVE_REGRESSION_FIXTURES],
  measurement: { joints: "World-space distance between paired anatomical anchors every fixed update and both same-instant handoffs", floor: "Nonnegative penetration from an independent Rapier contactShape query between the actual finite enabled floor and each oriented Ball/Capsule/Cuboid; clearance or missing floor is zero error", handoff: "All segment poses before/after transfer without integration; shortest relative quaternion angle", trajectories: "Identical initial state, Rapier backend, build, 1/60 s sequence; compare every segment every tick" },
  summary: { passed: failed.length === 0, passedScenarios: results.length - failed.length, failedScenarios: failed.length }, results };
mkdirSync("evidence", { recursive: true });
const outputPrefix=SCENARIO_PATTERN?"evidence/physics-selected":"evidence/physics";
writeFileSync(outputPrefix+"-results.json", JSON.stringify(report, null, 2) + "\n");
writeFileSync(SCENARIO_PATTERN?"evidence/physics-selected-trace.ndjson":"evidence/successor-trace.ndjson", trace.map(row => JSON.stringify(row)).join("\n") + "\n");
console.log(JSON.stringify(report.summary)); process.exitCode = failed.length ? 1 : 0;
