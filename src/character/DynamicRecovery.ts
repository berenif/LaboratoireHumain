import type { Collider, RigidBody, World } from "@dimforge/rapier3d-compat";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } from "../core/humanoid";
import type { MotionState, Quat, RecoveryDiagnostics, RecoveryPhase, SegmentId, SupportingContact, SegmentPose, Vec3 } from "../core/types";
import { add, angularVelocity, clamp, clampLength, length, lerp, dot, cross, normalize, quatFromTo, smooth01, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
import { applyCoupledJointMotors, applyPassiveJointResistance, type JointMotorCommand, type JointMotorResult } from "./joint-motors";
import { minimumSupportTorqueLimit, planContactLoads } from "./contact-loads";
import { jointCoordinates } from "./joint-coordinates";
import { blendRecoveryJointTargets, limitRecoveryJoint, reconstructRecoveryLimb, recoveryJointLimitError, recoveryJointRotation, recoveryLimbIds } from "./recovery-joints";
import { limbBodyClearance, limbTrunkClearance } from "./limb-collisions";
import { reachableFootTarget, revalidateRecoveryFootTarget, solveRecoveryLegTarget, type RecoveryFootTarget } from "./recovery-foot-targets";
import { solveRecoveryForefootAnchor } from "./recovery-forefoot-anchor";
import { acceptableRecoveryArmBraceTarget, isRecoveryArmSupportSegment, isRecoveryFootSegment, isRecoveryLegSupportSegment, RECOVERY_ARM_TARGET_TOLERANCE, recoveryMassState, selectRecoveryContacts, selectRecoveryRoute, supportGeometry, reachableArmBraceTarget, solveRecoveryArmTarget, usableRecoveryArmSupport, type RecoveryArmBraceTarget } from "./recovery-support";
const SIDES = ["left", "right"] as const;
type Side = typeof SIDES[number];
type Stage = RecoveryDiagnostics["transferStage"];
export type RecoveryReleaseResult = "blocked" | "newly-released" | "already-released";
type Plant = { position: Vec3; rotation: Quat; absentS: number; contact?:Vec3; contactPoints?:Vec3[] };
const plantSlipM = (plant: Plant, pose: SegmentPose): number => {
  const points = plant.contactPoints?.length ? plant.contactPoints : [plant.contact ?? plant.position];
  return Math.max(...points.map(priorPoint => {
    const materialPoint = add(pose.position,
      rotate(pose.rotation, rotate(quatInverse(plant.rotation), sub(priorPoint, plant.position))));
    return Math.hypot(materialPoint.x - priorPoint.x, materialPoint.z - priorPoint.z);
  }));
};
type FootMovement={side:Side;origin:Vec3;originRotation:Quat;target:RecoveryFootTarget;time:number;unloaded:boolean;paused:boolean};
const WEIGHT_N = TOTAL_MASS_KG * 9.81;
const FORWARD = { x: 0, y: 0, z: 1 };
/** Frozen acceptance parameters. Persistence always requires consecutive qualifying frames. */
export const RECOVERY_LIMITS = Object.freeze({
  normalY: 0.65, contactDistanceM: 0.012, minimumLoadN: 3,
  loadPersistenceS: 0.05, landingPersistenceS: 0.10, settlePersistenceS: 0.30,
  settleLinearMps: 0.65, settleAngularRadps: 1.8,
  // Frozen recovery completion contract. A product counter that requires a
  // full second of standing must observe that separately from this gate.
  stablePersistenceS: 0.55, stableLinearMps: 0.22, stableAngularRadps: 0.65,
  stableUpDot: 0.97, phaseMinimumS: 0.20, stallS: 3.0, supportLossS: 0.20,
  assistanceForceN: 0, assistanceTorqueNm: 0,
});
// The usable-brace clearance includes 20 mm of anatomical reserve. Planning
// may begin one solver contact band earlier; phase advancement still requires
// a freshly loaded brace satisfying the full measured-clearance predicate.
const PRONE_SHOULDER_CLEARANCE_M = HUMAN_PROPORTIONS.arm.upperRadiusM
  + HUMAN_PROPORTIONS.arm.forearmRadiusM + .02 - RECOVERY_LIMITS.contactDistanceM;

const armSupportSegments = (side: Side): SegmentId[] => [
  `${side}Forearm`, `${side}ForearmTwist`, `${side}Hand`,
];
const legSwingSegments = (side: Side): SegmentId[] => [
  `${side}Ankle`, `${side}Foot`, `${side}Forefoot`,
];
const segmentSide = (id: SegmentId): Side | null => SEGMENT_BY_ID.get(id)?.side ?? null;
const isLandingFootSegment = (id: SegmentId): boolean => {
  const role = SEGMENT_BY_ID.get(id)?.role;
  return role === "ankle" || role === "hindfoot" || role === "forefoot";
};
const supportingSideCount = (contacts: readonly SupportingContact[], predicate: (id: SegmentId) => boolean): number =>
  new Set(contacts.filter(contact => contact.loadBearing && predicate(contact.segment))
    .map(contact => segmentSide(contact.segment)).filter((side): side is Side => side !== null)).size;

/** Posture/speed part of standing; contact, phase and persistence remain caller gates. */
export function recoveryStandingPosture(
  poses: ReadonlyMap<SegmentId, SegmentPose>, motion: { linear: number; angular: number },
): boolean {
  const pelvis = poses.get("pelvis"), torso = poses.get("torso");
  return !!pelvis && !!torso && SIDES.every(side => {
    const foot = poses.get(`${side}Foot`);
    return !!foot && rotate(foot.rotation, UP).y > .97;
  }) && rotate(torso.rotation, UP).y >= RECOVERY_LIMITS.stableUpDot
    && rotate(pelvis.rotation, UP).y >= RECOVERY_LIMITS.stableUpDot
    && pelvis.position.y > .93 && motion.linear <= RECOVERY_LIMITS.stableLinearMps
    && motion.angular <= RECOVERY_LIMITS.stableAngularRadps;
}


function pitch(angle: number): Quat { return quatFromAxisAngle({ x: 1, y: 0, z: 0 }, angle); }
function rotation(x = 0, y = 0, z = 0): Quat {
  return quatMultiply(quatMultiply(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, y), pitch(x)), quatFromAxisAngle({ x: 0, y: 0, z: 1 }, z));
}

export function emptyRecoveryDiagnostics(): RecoveryDiagnostics {
  return { phase: "none", route: "none", leadingSide: null, rollSide: null, transferStage: "none",
    supportMarginM: -1, centerOfMass: ZERO, projectedCenterOfMass: ZERO, plantedTargets: [],
    releasedSupports: [], releaseMarginM: null, extension: 0, progressError: null, noSupportTimeS: 0, orientation: "forward", contacts: [], phaseTimeS: 0, settledTimeS: 0,
    stableTimeS: 0, stalledTimeS: 0, retries: 0, assistanceForce: ZERO, assistanceTorque: ZERO,
    assistanceForceCapN: RECOVERY_LIMITS.assistanceForceN, assistanceTorqueCapNm: RECOVERY_LIMITS.assistanceTorqueNm,
    maxMotorTorqueNm: 0, supporting: [], supportLoads: [] };
}

/** Rapier owns every dynamic transform. Plans only become bounded joint impulses. */
export class DynamicRecovery {
  private data = emptyRecoveryDiagnostics();
  private lastMotorResults: ReadonlyMap<SegmentId, JointMotorResult> = new Map();
  private lastPassiveTorques: ReadonlyMap<SegmentId, Vec3> = new Map();
  private heading = 0;
  private contactAges = new Map<SegmentId, number>();
  private landingTime = 0;
  private landingSupportedRise = false;
  private noSupportTime = 0;
  private bestError = Infinity;
  private bestStableTime = 0;
  private plants = new Map<SegmentId, Plant>();
  private slippedPlants = new Set<SegmentId>();
  private released = new Set<SegmentId>();
  private releasedUnloaded = new Set<SegmentId>();
  private placements = new Map<SegmentId, Vec3>();
  private footPlans=new Map<Side,RecoveryFootTarget>();
  private blockedFootSearchAge=new Map<Side,number>();
  private blockedFootSearchPelvis=new Map<Side,Vec3>();
  private footMovements=new Map<SegmentId,FootMovement>();
  private entry = new Map<SegmentId, Quat>();
  private bend = new Map<SegmentId, Vec3>();
  private stageTime = 0;
  private rootGoal: Vec3 = ZERO;
  private rootRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
  private rollTurned = false;
  private riseOrigin:Vec3=ZERO;
  private swingOrigin:Vec3=ZERO;
  private placingProneArms = false;
  private placementRootRotation:Quat={x:0,y:0,z:0,w:1};
  private placementTorsoRotation:Quat={x:0,y:0,z:0,w:1};
  private placementClearanceTorsoRotation:Quat={x:0,y:0,z:0,w:1};
  private shoulderClearanceEstablished=false;
  private lastArmSearchTimeS=-Infinity;
  private lastArmSearchClearanceM=-Infinity;
  private armBraces=new Map<Side,RecoveryArmBraceTarget>();
  private armStarts=new Map<Side,Vec3>();
  private armTimes=new Map<Side,number>();
  private movingArms=new Set<Side>();
  private rootEntry:Quat={x:0,y:0,z:0,w:1};
  private standOriginY:number=HUMAN_PROPORTIONS.pelvis.centerHeightM;
  private transferBlend=0;
  private prospectiveTime=0;
  private rollAxis:Vec3={x:0,y:0,z:1};
  private rollTarget:Quat={x:0,y:0,z:0,w:1};
  private pushOrigin:Vec3=ZERO;
  private pushRootRotation:Quat={x:0,y:0,z:0,w:1};

  reset(heading: number, direction: Vec3): void {
    this.data = emptyRecoveryDiagnostics(); this.data.phase = "protect";
    this.lastMotorResults = new Map();
    this.lastPassiveTorques = new Map();
    this.heading = heading;
    const local = rotate(quatInverse(quatFromAxisAngle(UP, heading)), direction);
    this.data.orientation = Math.abs(local.x) > Math.abs(local.z) ? (local.x < 0 ? "left" : "right") : (local.z < 0 ? "backward" : "forward");
    this.placingProneArms=false; this.armBraces.clear(); this.armStarts.clear(); this.armTimes.clear();this.movingArms.clear();
    this.shoulderClearanceEstablished=false;
    this.lastArmSearchTimeS=-Infinity;this.lastArmSearchClearanceM=-Infinity;
    this.footPlans.clear();this.blockedFootSearchAge.clear();this.blockedFootSearchPelvis.clear();
    this.footMovements.clear();
    this.contactAges.clear(); this.plants.clear(); this.slippedPlants.clear(); this.placements.clear(); this.released.clear(); this.releasedUnloaded.clear(); this.entry.clear(); this.bend.clear();
    this.landingTime = 0; this.landingSupportedRise = false; this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0; this.stageTime = 0; this.transferBlend = 0; this.prospectiveTime=0;
  }

  /** A new measured blow invalidates captured support before the next motor decision. */
  interruptForImpact(bodies: Map<SegmentId, RigidBody>, direction: Vec3): void {
    const contacts = this.data.contacts;
    const motion = this.motion(bodies);
    const torso = bodies.get("torso")!;
    const hadSupport = contacts.some(contact => contact.loadBearing);
    this.reset(this.heading, direction);
    // The collision may have left patches in contact, but no pre-impact load or
    // material anchor is allowed to authorize an immediate weight transfer.
    this.data.contacts = contacts.map(contact => ({ ...contact, persistenceS: 0, loadBearing: false }));
    const moving = motion.linear > RECOVERY_LIMITS.settleLinearMps
      || motion.angular > RECOVERY_LIMITS.settleAngularRadps
      || length(torso.linvel()) > RECOVERY_LIMITS.settleLinearMps;
    if (hadSupport && !moving) {
      this.data.phase = "settle";
      this.data.transferStage = "none";
    }
  }

  /** Called after Rapier integration and contact observation, once per physical step. */
  confirmPostStep(bodies: Map<SegmentId, RigidBody>, dt: number,
    requiredStableS: number = RECOVERY_LIMITS.stablePersistenceS): boolean {
    if (this.data.phase !== "stand") {
      this.data.stableTimeS = 0;
      return false;
    }
    const poses = this.poses(bodies);
    const feet = this.data.contacts.filter(contact => contact.loadBearing
      && !this.released.has(contact.segment) && isRecoveryFootSegment(contact.segment)
      && rotate(poses.get(contact.segment)!.rotation, UP).y > .85);
    const geometry = supportGeometry(feet, poses, recoveryMassState(poses.values()));
    const stable = supportingSideCount(feet, isRecoveryFootSegment) === 2
      && geometry.marginM >= 0 && recoveryStandingPosture(poses, this.motion(bodies));
    this.data.stableTimeS = stable ? this.data.stableTimeS + dt : 0;
    this.data.supportMarginM = geometry.marginM;
    return this.data.stableTimeS + 1e-9 >= requiredStableS;
  }

  diagnostics(): RecoveryDiagnostics {
    return { ...this.data, contacts: this.data.contacts.map(c => ({ ...c, point: { ...c.point }, points: c.points?.map(p => ({...p})) })),
      plantedTargets: this.data.plantedTargets.map(p => ({...p,position:{...p.position},rotation:{...p.rotation}})),
      supporting: [...this.data.supporting], releasedSupports: [...this.data.releasedSupports],
      supportLoads: this.data.supportLoads?.map(load=>({...load})),
      recoveryAxis: this.data.recoveryAxis ? {...this.data.recoveryAxis} : undefined,
      plannedSupportSources: this.data.plannedSupportSources ? [...this.data.plannedSupportSources] : undefined,
      establishedSupportSources: this.data.establishedSupportSources ? [...this.data.establishedSupportSources] : undefined,
      centerOfMass: {...this.data.centerOfMass}, projectedCenterOfMass: {...this.data.projectedCenterOfMass},
      assistanceForce: { ...this.data.assistanceForce }, assistanceTorque: { ...this.data.assistanceTorque } };
  }

  /** Actual posture-motor results for the last decision; empty in passive settling. */
  motorResults(): ReadonlyMap<SegmentId, JointMotorResult> { return this.lastMotorResults; }

  /** Passive-only settling impulses, without inventing a posture target. */
  passiveTorques(): ReadonlyMap<SegmentId, Vec3> { return this.lastPassiveTorques; }

  private poses(bodies: Map<SegmentId, RigidBody>): Map<SegmentId, SegmentPose> {
    return new Map([...bodies].map(([id, b]) => [id, { id, position: {...b.translation()}, rotation: {...b.rotation()},
      linearVelocity: {...b.linvel()}, angularVelocity: {...b.angvel()} }]));
  }

  /** Contact points are copied before another WASM query can reuse its scratch storage. */
  observe(world: World, floor: Collider | readonly Collider[], colliders: Map<SegmentId, Collider>, bodies: Map<SegmentId, RigidBody>, dt: number): void {
    const contacts: SupportingContact[] = [];
    const supportSurfaces = (Array.isArray(floor) ? floor : [floor]).filter(surface => surface.isEnabled());
    const surfaceHandles = new Set(supportSurfaces.map(surface => surface.handle));
    const sleepingEquilibrium = supportSurfaces.length > 0 && bodies.size > 0
      && [...bodies.values()].every(body => body.isSleeping());
    for (const [segment, collider] of colliders) {
      let normalY = 0, impulse = 0;
      const points: Vec3[] = [];
      const nearbySurfaces: Collider[] = [];
      if (supportSurfaces.length > 1) world.contactPairsWith(collider, surface => {
        if (surfaceHandles.has(surface.handle)) nearbySurfaces.push(surface);
      });
      for (const surface of supportSurfaces.length > 1 ? nearbySurfaces : supportSurfaces) world.contactPair(surface, collider, (manifold, flipped) => {
        const upward = manifold.normal().y * (flipped ? -1 : 1);
        let touching = false;
        for (let i = 0; i < manifold.numSolverContacts(); i++) {
          if (manifold.solverContactDist(i) <= RECOVERY_LIMITS.contactDistanceM && upward >= RECOVERY_LIMITS.normalY) {
            touching = true; normalY = Math.max(normalY, upward);
            const point = manifold.solverContactPoint(i);
            if (point) points.push({...point});
          }
        }
        if (touching) for (let i = 0; i < manifold.numContacts(); i++) impulse += Math.max(0, manifold.contactImpulse(i));
      });
      const forceN = impulse / dt;
      const loaded = points.length > 0 && forceN >= RECOVERY_LIMITS.minimumLoadN;
      const age = loaded ? (this.contactAges.get(segment) ?? 0) + dt : 0;
      this.contactAges.set(segment, age);
      if (points.length) contacts.push({ segment, normalY, forceN, measuredForceN: forceN,
        persistenceS: age, points,
        point: scale(points.reduce(add, ZERO), 1 / points.length), loadBearing: loaded && age + 1e-9 >= RECOVERY_LIMITS.loadPersistenceS });
    }
    // Sleeping rigid bodies produce no new impulse, but their solver manifolds
    // remain real floor contacts. Keep those patches available at static
    // equilibrium; a missing manifold is removed on this same observation.
    if (sleepingEquilibrium && contacts.length) {
      const staticShare = WEIGHT_N / contacts.length;
      for (const contact of contacts) {
        const prior = this.data.contacts.find(previous => previous.segment === contact.segment
          && previous.loadBearing && previous.forceN > 0);
        contact.sleepingEquilibrium = true;
        contact.forceN = prior?.forceN ?? staticShare;
        contact.persistenceS = Math.max(contact.persistenceS, prior?.persistenceS ?? 0,
          RECOVERY_LIMITS.loadPersistenceS);
        contact.loadBearing = true;
        this.contactAges.set(contact.segment, contact.persistenceS);
      }
    }
    this.data.contacts = contacts;
    const previousLoads = this.data.supportLoads ?? [];
    this.data.supportLoads = contacts.map(contact => ({
      segment: contact.segment,
      plannedForceN: contact.loadBearing && !this.released.has(contact.segment)
        ? previousLoads.find(load => load.segment === contact.segment)?.plannedForceN ?? 0 : 0,
      measuredForceN: contact.measuredForceN ?? contact.forceN,
    }));
    const poses = this.poses(bodies), motion = this.motion(bodies);
    const plantSpeed = (contact: SupportingContact): number => {
      const body = bodies.get(contact.segment)!;
      const velocity = add(body.linvel(), cross(body.angvel(), sub(contact.point, body.translation())));
      return Math.hypot(velocity.x, velocity.z);
    };
    const canCapturePlant = (contact: SupportingContact): boolean => {
      const velocity = bodies.get(contact.segment)!.linvel();
      return plantSpeed(contact) < 0.18 && Math.hypot(velocity.x, velocity.z) < 0.18;
    };
    const feet = contacts.filter(c => c.loadBearing && isLandingFootSegment(c.segment));
    const supportedCrouch = supportingSideCount(feet, isLandingFootSegment) === 2
      && rotate(bodies.get("torso")!.rotation(), UP).y > 0.65 && bodies.get("pelvis")!.translation().y < 0.85;
    const landed = contacts.some(c => !isLandingFootSegment(c.segment)
      && c.normalY >= RECOVERY_LIMITS.normalY && c.forceN >= RECOVERY_LIMITS.minimumLoadN)
      || supportedCrouch;
    this.landingTime = landed ? this.landingTime + dt : 0;
    const settled = contacts.some(c => c.forceN >= RECOVERY_LIMITS.minimumLoadN) && motion.linear <= RECOVERY_LIMITS.settleLinearMps && motion.angular <= RECOVERY_LIMITS.settleAngularRadps;
    this.data.settledTimeS = settled ? this.data.settledTimeS + dt : 0;
    // Old persistent load cannot requalify an anchor invalidated by slip. It
    // must first unload; the existing contact-age gate then requires fresh load.
    for (const id of this.slippedPlants) {
      if (!contacts.some(c => c.segment === id && c.forceN >= RECOVERY_LIMITS.minimumLoadN)) this.slippedPlants.delete(id);
    }
    for (const [id, plant] of this.plants) {
      const contact = contacts.find(c => c.segment === id && c.forceN >= RECOVERY_LIMITS.minimumLoadN);
      plant.absentS = contact ? 0 : plant.absentS + dt;
      // A sliding contact still carries measured load, but its old pose is no
      // longer a usable anchor for the recovery motors.
      // Track the entire captured material patch: rotation can slip an edge
      // while its centroid stays fixed. Keep the original 50 mm reserve.
      const slipped = plantSlipM(plant, poses.get(id)!) > 0.05;
      if (plant.absentS > 0.10 || !supportSurfaces.length || slipped) {
        this.plants.delete(id);
        if (slipped) this.slippedPlants.add(id);
      }
    }
    for (const id of this.released) {
      const movement=this.footMovements.get(id);
      const contact=movement
        ? contacts.filter(c=>segmentSide(c.segment)===movement.side && isRecoveryFootSegment(c.segment))
          .sort((a,b)=>b.forceN-a.forceN)[0]
        : contacts.find(c=>c.segment===id);
      if(!contact || contact.forceN<RECOVERY_LIMITS.minimumLoadN) this.releasedUnloaded.add(id);
      const armSide=segmentSide(id);
      if(!movement && armSide && this.movingArms.has(armSide) && armSupportSegments(armSide).includes(id))continue;
      if(movement) {
        if((!contact || contact.forceN<RECOVERY_LIMITS.minimumLoadN) && !movement.unloaded) {
          movement.unloaded=true;movement.time=0;movement.origin={...poses.get(id)!.position};movement.originRotation={...poses.get(id)!.rotation};
        }
        const pose=poses.get(id)!;
        const remaining=new Set(this.released);remaining.delete(id);
        const prospective=selectRecoveryContacts(contacts,poses,this.data.phase,this.data.transferStage,this.data.leadingSide,remaining);
        const geometry=supportGeometry(prospective,poses,recoveryMassState(poses.values()));
        const qualified=movement.unloaded && movement.time>=.42 && !!contact && contact.persistenceS>=.20
          && contact.forceN>WEIGHT_N*.08 && rotate(pose.rotation,UP).y>.95 && pose.position.y<.075
          && length(sub(pose.position,movement.target.position))<.14 && geometry.marginM>.015
          && recoveryMassState(poses.values()).velocity.y>-.10;
        if(qualified) {
          this.footMovements.delete(id);
          for(const segment of legSwingSegments(movement.side)) {
            this.released.delete(segment);this.releasedUnloaded.delete(segment);
          }
        }
        else if(movement.unloaded && contact?.loadBearing) movement.paused=length(sub(pose.position,movement.target.position))>=.14;
        continue;
      }
      if(this.releasedUnloaded.has(id) && contact?.loadBearing) {this.released.delete(id);this.releasedUnloaded.delete(id);}
    }
    for(const side of [...this.movingArms]) {
      const ids=armSupportSegments(side),target=this.armBraces.get(side),hand=poses.get(`${side}Hand`)!;
      const contactsForSide=contacts.filter(contact=>ids.includes(contact.segment));
      const fresh=ids.every(id=>this.releasedUnloaded.has(id))
        && contactsForSide.some(contact=>contact.loadBearing && contact.persistenceS>=RECOVERY_LIMITS.landingPersistenceS);
      if(!target || !fresh || (this.armTimes.get(side)??0)<.35
        || length(sub(hand.position,target.position))>=.14
        || !usableRecoveryArmSupport(side,poses,contactsForSide))continue;
      for(const id of ids) {this.released.delete(id);this.releasedUnloaded.delete(id);}
      for(const contact of contactsForSide) if(contact.loadBearing && !this.slippedPlants.has(contact.segment) && canCapturePlant(contact)) {
        const pose=poses.get(contact.segment)!;
        this.plants.set(contact.segment,{position:{...pose.position},rotation:{...pose.rotation},absentS:0,contact:{...contact.point},contactPoints:contact.points?.map(point=>({...point}))});
      }
      this.movingArms.delete(side);
    }
    for (const c of contacts) {
      if (!c.loadBearing || this.plants.has(c.segment) || this.released.has(c.segment)
        || this.slippedPlants.has(c.segment) || !canCapturePlant(c)
        || !(isRecoveryArmSupportSegment(c.segment) || isRecoveryLegSupportSegment(c.segment))) continue;
      const pose = poses.get(c.segment)!;
      this.plants.set(c.segment, { position: {...pose.position}, rotation: {...pose.rotation}, absentS: 0, contact:{...c.point},contactPoints:c.points?.map(point=>({...point})) });
    }
    this.data.plantedTargets = [...this.plants].map(([segment, p]) => ({segment, position:p.position, rotation:p.rotation,
      driftM: plantSlipM(p, poses.get(segment)!)}));
    const mass = recoveryMassState(poses.values()), geometry = supportGeometry(this.supporting(poses), poses, mass);
    this.data.centerOfMass = mass.position; this.data.projectedCenterOfMass = geometry.projectedCenterOfMass;
    this.data.supportMarginM = geometry.marginM;
    this.data.supporting = this.supporting(poses).map(c => c.segment);
    this.data.establishedSupportSources=[...geometry.supporting];
    // Retain the impulse actually applied this step, even if contact is lost during integration.
  }

  private supporting(poses?: Map<SegmentId,SegmentPose>): SupportingContact[] {
    return selectRecoveryContacts(this.data.contacts,poses,this.data.phase,this.data.transferStage,this.data.leadingSide,this.released);
  }
  private prospectiveSupport(phase: RecoveryPhase, stage: Stage, poses: Map<SegmentId,SegmentPose>, releases: SegmentId[] = []) {
    const excluded=new Set([...this.released,...releases]);
    const contacts=selectRecoveryContacts(this.data.contacts,poses,phase,stage,this.data.leadingSide,excluded);
    const geometry=supportGeometry(contacts,poses,recoveryMassState(poses.values()));
    const included=new Set(contacts.map(contact=>contact.segment));
    const excludedLoaded=this.data.contacts.filter(contact=>contact.forceN>=RECOVERY_LIMITS.minimumLoadN
      && contact.normalY>=RECOVERY_LIMITS.normalY && !included.has(contact.segment));
    return { contacts, geometry, excludedLoaded, balanced:geometry.polygon.length>=3 && geometry.marginM>=0,
      ready:geometry.polygon.length>=3 && geometry.marginM>=0 && excludedLoaded.length===0 };
  }
  private motion(bodies: Map<SegmentId, RigidBody>): { linear: number; angular: number } {
    let linear = 0, angular = 0;
    for (const definition of SEGMENTS) {
      const body = bodies.get(definition.id)!;
      linear += definition.massKg * length(body.linvel()) ** 2; angular += definition.massKg * length(body.angvel()) ** 2;
    }
    return { linear: Math.sqrt(linear / TOTAL_MASS_KG), angular: Math.sqrt(angular / TOTAL_MASS_KG) };
  }
  private captureEntry(bodies: Map<SegmentId, RigidBody>): void {
    this.entry.clear();
    this.rootEntry={...bodies.get("pelvis")!.rotation()};
    for (const d of SEGMENTS) if (d.parent) this.entry.set(d.id, quatMultiply(quatInverse(bodies.get(d.parent)!.rotation()), bodies.get(d.id)!.rotation()));
    this.stageTime = 0;
  }
  private stage(stage: Stage, bodies: Map<SegmentId, RigidBody>): void {
    if (stage === this.data.transferStage) return;
    const previousRootEntry={...this.rootEntry};
    this.data.transferStage = stage; this.captureEntry(bodies);
    this.prospectiveTime=0;
    if(stage === "shift-weight") this.transferBlend = 0;
    if(stage === "push-brace") {
      this.pushOrigin={...bodies.get("pelvis")!.translation()};
      this.pushRootRotation={...bodies.get("pelvis")!.rotation()};
    }
    if(stage==="extend") this.rootEntry=previousRootEntry;
    this.bestError = Infinity; this.data.stalledTimeS = 0;
  }
  private enter(phase: RecoveryPhase, bodies: Map<SegmentId, RigidBody>): void {
    this.data.phase = phase; this.data.phaseTimeS = 0; this.data.stalledTimeS = 0; this.data.stableTimeS = 0;
    if(phase==="stand") this.standOriginY=bodies.get("pelvis")!.translation().y;
    this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0; this.captureEntry(bodies);
    if (phase === "settle") { this.data.transferStage = "none"; this.data.route = "none"; this.released.clear(); this.releasedUnloaded.clear();this.footMovements.clear();this.movingArms.clear(); }
  }
  private chooseRoute(bodies: Map<SegmentId, RigidBody>): void {
    const poses = this.poses(bodies), pelvis = poses.get("pelvis")!;
    const bodyAxis=rotate(poses.get("torso")!.rotation,UP);
    if(bodyAxis.y<.5 && Math.hypot(bodyAxis.x,bodyAxis.z)>.25)this.heading=Math.atan2(bodyAxis.x,bodyAxis.z);
    const forward = rotate(poses.get("torso")!.rotation, FORWARD), right = rotate(poses.get("torso")!.rotation, {x:1,y:0,z:0});
    this.data.orientation = Math.abs(right.y) > Math.abs(forward.y) ? (right.y > 0 ? "left" : "right") : (forward.y > 0 ? "backward" : "forward");
    this.rootGoal = {...pelvis.position}; this.riseOrigin={...pelvis.position}; this.placements.clear(); this.released.clear(); this.rollTurned = false;
    this.footPlans.clear();this.blockedFootSearchAge.clear();this.blockedFootSearchPelvis.clear();
    const footTargets:Partial<Record<Side,Vec3>>={};
    for(const side of SIDES) {
      const plan=reachableFootTarget(side,poses,this.heading);
      if(plan) {this.footPlans.set(side,plan);if(plan.feasible)footTargets[side]=plan.position;}
      this.blockedFootSearchAge.set(side,0);
      this.blockedFootSearchPelvis.set(side,{...pelvis.position});
    }
    for (const side of SIDES) {
      const foot: SegmentId = `${side}Foot`, hand: SegmentId = `${side}Hand`;
      this.placements.set(foot, {...(this.footPlans.get(side)?.position??poses.get(foot)!.position)});
      const girdle = poses.get(`${side}ShoulderGirdle`)!;
      const shoulder = worldPoint(girdle.position, girdle.rotation,
        SEGMENT_BY_ID.get(`${side}UpperArm`)!.jointProfile!.parentFrame.anchor);
      const measuredHand = poses.get(hand)!.position;
      this.placements.set(hand, {x: measuredHand.x * 0.5 + shoulder.x * 0.5, y:0.066,z:measuredHand.z * 0.5 + shoulder.z * 0.5});
      for (const [upper,lower,terminal,isArm] of [[`${side}Thigh`,`${side}Shin`,`${side}Ankle`,false],[`${side}UpperArm`,`${side}Forearm`,`${side}ForearmTwist`,true]] as const) {
        const upperPose = poses.get(upper)!, lowerPose = poses.get(lower)!;
        const parentPose = poses.get(isArm ? `${side}ShoulderGirdle` : "pelvis")!;
        const start = worldPoint(parentPose.position,parentPose.rotation,SEGMENT_BY_ID.get(upper)!.jointProfile!.parentFrame.anchor);
        const middle = worldPoint(upperPose.position,upperPose.rotation,SEGMENT_BY_ID.get(lower)!.jointProfile!.parentFrame.anchor);
        const finish = worldPoint(lowerPose.position,lowerPose.rotation,SEGMENT_BY_ID.get(terminal)!.jointProfile!.parentFrame.anchor);
        const line = normalize(sub(finish,start));
        const measured = sub(sub(middle,start),scale(line,dot(sub(middle,start),line)));
        const parentRotation=parentPose.rotation;
        const localMeasured=rotate(quatInverse(parentRotation),measured);
        const natural={x:0,y:0,z:isArm?-1:1};
        this.bend.set(upper,length(localMeasured)>.03 && dot(localMeasured,natural)>0 ? normalize(add(scale(normalize(localMeasured),.2),natural)) : natural);
      }
    }
    const selected = selectRecoveryRoute({poses,contacts:this.data.contacts,footTargets});
    this.data.route = selected.route; this.data.leadingSide = selected.leadingSide; this.data.rollSide = selected.rollSide;
    const longitudinal=rotate(pelvis.rotation,UP);
    this.rollAxis=normalize({x:longitudinal.x,y:0,z:longitudinal.z},rotate(quatFromAxisAngle(UP,this.heading),FORWARD));
    const face=rotate(pelvis.rotation,FORWARD);
    const rollAngle=Math.atan2(-cross(this.rollAxis,face).y,-face.y);
    const selectedAngle=Math.abs(Math.abs(rollAngle)-Math.PI)<.05 ? (selected.rollSide==="left"?1:-1)*Math.abs(rollAngle) : rollAngle;
    this.rollTarget=quatMultiply(quatFromAxisAngle(this.rollAxis,selectedAngle),pelvis.rotation);
    this.data.recoveryHeading=this.heading;this.data.recoveryAxis={...this.rollAxis};
    this.armBraces.clear(); this.armStarts.clear(); this.armTimes.clear();this.movingArms.clear();
    this.placementRootRotation={...pelvis.rotation};
    this.placementTorsoRotation=quatMultiply(quatInverse(pelvis.rotation),poses.get("torso")!.rotation);
    // A prone shoulder can settle too close to the floor for any legal hand
    // brace. Extend the two bounded spinal joints before moving an arm. This
    // remains an internal posture target: chest/pelvis contact must supply the
    // external reaction that actually raises the shoulder.
    this.placementClearanceTorsoRotation=quatMultiply(this.placementTorsoRotation,pitch(-.36));
    this.placingProneArms=selected.route==="prone" && !SIDES.every(side=>usableRecoveryArmSupport(
      side,poses,this.data.contacts.filter(c=>isRecoveryArmSupportSegment(c.segment)),
    ));
    this.shoulderClearanceEstablished=false;
    this.lastArmSearchTimeS=-Infinity;this.lastArmSearchClearanceM=-Infinity;
    if(selected.route!=="crouch" && selected.route!=="half-kneel" && !this.placingProneArms) for(const side of SIDES) {
      const target=reachableArmBraceTarget(side,poses,this.heading);
      if(target && acceptableRecoveryArmBraceTarget(target)) this.armBraces.set(side,target);
      this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});
    }
    const up = rotate(poses.get("torso")!.rotation,UP).y;
    const lower = this.data.contacts.some(c=>c.loadBearing && isRecoveryLegSupportSegment(c.segment));
    const braceReady = this.data.contacts.some(c=>c.loadBearing
      && (isRecoveryArmSupportSegment(c.segment)||isRecoveryLegSupportSegment(c.segment))) && up>.25 && pelvis.position.y>.28;
    const standingFeet = this.data.contacts.filter(c=>c.loadBearing && isRecoveryFootSegment(c.segment)
      && rotate(poses.get(c.segment)!.rotation,UP).y>.85);
    const supportedRise = selected.route === "crouch" && supportingSideCount(standingFeet,isRecoveryFootSegment) === 2
      && up>.88 && pelvis.position.y>.55
      && supportGeometry(standingFeet,poses,recoveryMassState(poses.values())).marginM>=0;
    if (this.placingProneArms) {this.enter("roll",bodies);this.stage("arm-preparation",bodies);}
    else if (supportedRise) { this.enter("stand",bodies); this.stage("extend",bodies); }
    else if (lower && up>.65 && pelvis.position.y>.38 && this.prospectiveSupport("kneel","shift-weight",poses).ready) { this.enter("kneel",bodies); this.stage(selected.route === "crouch" ? "extend":"shift-weight",bodies); }
    else if (braceReady && this.prospectiveSupport("brace","tuck-knee",poses).ready) { this.enter("brace",bodies); this.stage("tuck-knee",bodies); }
    else if(selected.route==="prone") {this.enter("roll",bodies);this.prepareArms(bodies,poses);}
    else { this.enter("roll",bodies); this.stage("roll",bodies); }
  }

  private prepareArms(bodies:Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>):void {
    this.placingProneArms=true;
    this.placementRootRotation={...poses.get("pelvis")!.rotation};
    this.placementTorsoRotation=quatMultiply(quatInverse(this.placementRootRotation),poses.get("torso")!.rotation);
    this.placementClearanceTorsoRotation=quatMultiply(this.placementTorsoRotation,pitch(-.36));
    this.shoulderClearanceEstablished=false;
    this.lastArmSearchTimeS=-Infinity;this.lastArmSearchClearanceM=-Infinity;
    this.armBraces.clear();this.armStarts.clear();this.armTimes.clear();this.movingArms.clear();
    this.stage("arm-preparation",bodies);
  }

  private proneShoulderClearance(poses:ReadonlyMap<SegmentId,SegmentPose>):number {
    return Math.min(...SIDES.map(side=>{
      const girdle=poses.get(`${side}ShoulderGirdle`)!;
      return worldPoint(girdle.position,girdle.rotation,
        SEGMENT_BY_ID.get(`${side}UpperArm`)!.jointProfile!.parentFrame.anchor).y;
    }));
  }

  private armPlacementReady(side:Side,poses:Map<SegmentId,SegmentPose>):boolean {
    const target=this.armBraces.get(side);
    const contacts=this.data.contacts.filter(c=>isRecoveryArmSupportSegment(c.segment)
      && segmentSide(c.segment)===side && !this.released.has(c.segment));
    if(!usableRecoveryArmSupport(side,poses,contacts))return false;
    // A planted, persistently loaded forearm already supplies a physical
    // brace; there is no reason to demand a floor-safe hand relocation first.
    if(contacts.some(contact=>contact.loadBearing
      && contact.persistenceS>=RECOVERY_LIMITS.loadPersistenceS
      && this.plants.has(contact.segment)))return true;
    return !!target && length(sub(poses.get(`${side}Hand`)!.position,target.position))<.14;
  }
  private legalArmCommand(side:Side,desired:Vec3,poses:Map<SegmentId,SegmentPose>):Map<SegmentId,Quat>|null {
    const brace=this.armBraces.get(side),parent=poses.get("torso")!;
    if(!brace)return null;
    if(!parent)return null;
    const girdle:SegmentId=`${side}ShoulderGirdle`,upper:SegmentId=`${side}UpperArm`;
    const lower:SegmentId=`${side}Forearm`,twist:SegmentId=`${side}ForearmTwist`,end:SegmentId=`${side}Hand`;
    const measuredGirdle=poses.get(girdle)!;
    const rawGirdle=quatMultiply(quatInverse(parent.rotation),measuredGirdle.rotation);
    const girdleLocal=limitRecoveryJoint(girdle,rawGirdle);
    const girdleWorld=quatMultiply(parent.rotation,girdleLocal);
    const girdleProfile=SEGMENT_BY_ID.get(girdle)!.jointProfile!;
    const girdlePosition=sub(worldPoint(parent.position,parent.rotation,girdleProfile.parentFrame.anchor),
      rotate(girdleWorld,girdleProfile.childFrame.anchor));
    const shoulder=worldPoint(girdlePosition,girdleWorld,SEGMENT_BY_ID.get(upper)!.jointProfile!.parentFrame.anchor);
    const solved=solveRecoveryArmTarget(shoulder,desired,brace.trajectoryBend,brace.wristRotation,this.heading);
    const rawUpper=quatMultiply(quatInverse(girdleWorld),solved.upperRotation);
    const rawForearm=quatMultiply(quatInverse(solved.upperRotation),solved.forearmRotation);
    const rawTwist=quatMultiply(quatInverse(solved.forearmRotation),solved.forearmTwistRotation);
    const rawHand=quatMultiply(quatInverse(solved.forearmTwistRotation),solved.handRotation);
    const raw=new Map<SegmentId,Quat>([
      [girdle,rawGirdle],[upper,rawUpper],[lower,rawForearm],[twist,rawTwist],[end,rawHand],
    ]);
    const limited=new Map([...raw].map(([id,q])=>[id,limitRecoveryJoint(id,q)]));
    if([...raw].some(([id,q])=>recoveryJointLimitError(id,q)>RECOVERY_ARM_TARGET_TOLERANCE.maximumJointLimitErrorRad))return null;
    const geometry=reconstructRecoveryLimb(side,true,parent,limited);
    const hand=geometry.poses.find(pose=>pose.id===end)!;
    const complete=new Map<SegmentId,SegmentPose>(poses);
    for(const pose of geometry.poses)complete.set(pose.id,pose);
    return geometry.floorClearanceM>=-RECOVERY_ARM_TARGET_TOLERANCE.maximumFloorPenetrationM
      && limbTrunkClearance(complete,side,"arm").clearanceM>=.004
      && length(sub(hand.position,desired))<.02?limited:null;
  }
  private release(ids: SegmentId[], poses: Map<SegmentId,SegmentPose>): RecoveryReleaseResult {
    if (!ids.length) return "blocked";
    // Every authorization uses current measured balance, even after an earlier
    // release or when the requested limb has no current load to remove.
    const geometry = supportGeometry(this.supporting(poses),poses,recoveryMassState(poses.values()),ids);
    if (geometry.marginM < 0 || geometry.polygon.length < 3 || !geometry.supporting.length) return "blocked";
    const releasedArmLoad = ids.some(isRecoveryArmSupportSegment)
      ? this.supporting(poses).filter(contact=>ids.includes(contact.segment)
        && isRecoveryArmSupportSegment(contact.segment)).reduce((sum,contact)=>sum+contact.forceN,0)
      : 0;
    // A loaded hand or forearm stays braced until other measured patches have
    // accepted its share; a target position alone cannot authorize the lift.
    if(releasedArmLoad>0 && geometry.loadedForceN<releasedArmLoad*1.15)return "blocked";
    this.data.releaseMarginM = geometry.marginM;
    if(ids.every(id=>this.released.has(id))) return "already-released";
    const loaded = ids.filter(id => !this.released.has(id) && this.data.contacts.some(c=>c.segment===id && c.loadBearing));
    this.data.releasedSupports.push(...loaded);
    for (const id of ids) { if(this.released.has(id))continue; this.plants.delete(id); this.released.add(id); this.releasedUnloaded.delete(id); }
    return "newly-released";
  }

  private beginFootMovement(side:Side,poses:Map<SegmentId,SegmentPose>,releaseIds:SegmentId[],preferred?:Vec3):boolean {
    const id:SegmentId=`${side}Foot`;
    if(this.footMovements.has(id))return true;
    const target=reachableFootTarget(side,poses,this.heading,preferred);
    if(!target?.feasible || this.release(releaseIds,poses)==="blocked")return false;
    this.footPlans.set(side,target);this.placements.set(id,{...target.position});
    this.footMovements.set(id,{side,origin:{...poses.get(id)!.position},originRotation:{...poses.get(id)!.rotation},target,time:0,unloaded:false,paused:false});
    return true;
  }

  /** Decide using last integrated evidence, actuate, then the caller integrates once. */
  apply(bodies: Map<SegmentId, RigidBody>, dt: number): { state: MotionState; recovered: boolean } {
    this.lastMotorResults = new Map();
    this.lastPassiveTorques = new Map();
    this.data.phaseTimeS += dt; this.stageTime += dt;
    this.data.assistanceForce = ZERO; this.data.assistanceTorque = ZERO; this.data.maxMotorTorqueNm = 0;
    this.data.releasedSupports = []; this.data.releaseMarginM = null;
    if (!this.entry.size) this.captureEntry(bodies);
    const pelvis = bodies.get("pelvis")!, torso = bodies.get("torso")!, motion = this.motion(bodies);
    const poses = this.poses(bodies), mass = recoveryMassState(poses.values());
    const up = rotate(torso.rotation(), UP).y;
    const feet = this.data.contacts.filter(c=>c.loadBearing && !this.released.has(c.segment)
      && isRecoveryFootSegment(c.segment) && rotate(bodies.get(c.segment)!.rotation(),UP).y>.85);
    const footSideCount=supportingSideCount(feet,isRecoveryFootSegment);
    const qualifiedFootSide=(side:Side):boolean=>{
      const contacts=feet.filter(contact=>segmentSide(contact.segment)===side);
      return contacts.length>0 && Math.max(...contacts.map(contact=>contact.persistenceS))>=.12
        && contacts.reduce((sum,contact)=>sum+contact.forceN,0)>WEIGHT_N*.12
        && contacts.every(contact=>rotate(bodies.get(contact.segment)!.rotation(),UP).y>.97);
    };
    const shins = this.data.contacts.filter(c=>c.loadBearing && !this.released.has(c.segment) && c.segment.endsWith("Shin"));
    if (this.data.phase === "protect" && up > .65 && pelvis.translation().y > .38
      && pelvis.translation().y < .85
      && (footSideCount === 2 || footSideCount === 1 && shins.some(contact =>
        segmentSide(contact.segment) !== segmentSide(feet[0].segment)))) {
      this.landingSupportedRise = true;
    }
    // Hold a supported low entry during the brief landing observation. A
    // balanced two-foot crouch takes the direct physical route above as soon
    // as landing persistence qualifies, before its foot margin drifts away.
    const supportedEntryStillUpright = this.landingSupportedRise
      && up > .65 && pelvis.translation().y > .38 && pelvis.translation().y < .85;
    const readyToSettle = !supportedEntryStillUpright
      || this.data.settledTimeS + 1e-9 >= RECOVERY_LIMITS.settlePersistenceS;
    const ready = this.data.phaseTimeS >= RECOVERY_LIMITS.phaseMinimumS;
    const prospective=this.prospectiveSupport("brace","tuck-knee",poses);
    this.prospectiveTime=this.data.transferStage==="push-brace" && prospective.ready ? this.prospectiveTime+dt : 0;
    const supportedCrouchNow=footSideCount===2 && up>.88 && pelvis.translation().y>.55
      && supportGeometry(feet,poses,mass).marginM>=0
      && this.prospectiveSupport("stand","extend",poses).ready;
    if (this.data.phase === "protect" && supportedCrouchNow
      && this.landingTime + 1e-9 >= RECOVERY_LIMITS.landingPersistenceS) this.chooseRoute(bodies);
    else if (this.data.phase === "protect" && readyToSettle
      && this.landingTime + 1e-9 >= RECOVERY_LIMITS.landingPersistenceS) this.enter("settle",bodies);
    else if (this.data.phase === "settle" && this.data.settledTimeS >= RECOVERY_LIMITS.settlePersistenceS) this.chooseRoute(bodies);
    else if(this.data.phase==="roll" && this.placingProneArms && ready && this.movingArms.size===0
      && SIDES.every(side=>this.armPlacementReady(side,poses))) {
      this.placingProneArms=false;this.stage("push-brace",bodies);
    } else if (this.data.phase === "roll" && this.data.transferStage==="push-brace" && this.prospectiveTime>=RECOVERY_LIMITS.phaseMinimumS) {
      this.enter("brace",bodies); this.stage("tuck-knee",bodies);
    } else if (this.data.phase === "brace" && ready && (shins.length || footSideCount) && up>.65 && pelvis.translation().y>.38 && this.prospectiveSupport("kneel",footSideCount?"shift-weight":"plant-lead",poses).ready) {
      this.enter("kneel",bodies); this.stage(footSideCount ? "shift-weight":"plant-lead",bodies);
    } else if (this.data.phase === "kneel" && ready && footSideCount===2
      && SIDES.every(qualifiedFootSide)
      && up>.97 && rotate(pelvis.rotation(),UP).y>.95 && pelvis.translation().y>.93
      && motion.linear<.30 && motion.angular<.85 && supportGeometry(feet,poses,mass).marginM>.03
      && this.prospectiveSupport("stand","extend",poses).ready) {
      this.enter("stand",bodies); this.stage("extend",bodies);
    }
    let active = ["roll","brace","kneel","stand"].includes(this.data.phase);
    let supports = this.supporting(poses); this.data.supporting = supports.map(c=>c.segment);
    const geometry = supportGeometry(supports,poses,mass);
    this.data.supportMarginM = geometry.marginM;
    this.data.centerOfMass = mass.position; this.data.projectedCenterOfMass = geometry.projectedCenterOfMass;
    // Completion is decided by confirmPostStep using the state Rapier actually
    // integrated, including newly lost contacts and impact velocity.
    if (active) {
      this.transfer(bodies,poses,dt);
      supports=this.supporting(poses);
      const refreshed=supportGeometry(supports,poses,mass);
      this.data.supporting=refreshed.supporting; this.data.supportMarginM=refreshed.marginM;
      this.data.projectedCenterOfMass=refreshed.projectedCenterOfMass;
      this.data.establishedSupportSources=[...refreshed.supporting];
      this.data.plannedSupportSources=[...this.placements.keys()];
      this.noSupportTime = supports.length ? 0 : this.noSupportTime+dt;
      const targetHeight = this.data.phase==="roll" ? .32 : this.data.phase==="brace" ? .50 : this.data.phase==="kneel" ? .76 : .99;
      const lead = `${this.data.leadingSide}Foot` as SegmentId;
      const placementError = this.placements.has(lead) && !feet.some(c=>c.segment===lead) ? length(sub(poses.get(lead)!.position,this.placements.get(lead)!))*.2:0;
      let error = this.placingProneArms ? SIDES.reduce((sum,side)=>sum+length(sub(poses.get(`${side}Hand`)!.position,this.armBraces.get(side)?.position??poses.get(`${side}Hand`)!.position)),0)
        : Math.abs(targetHeight-pelvis.translation().y)+Math.max(0,1-up)*.3+placementError;
      this.data.stageAction=this.data.transferStage;
      this.data.progressMeasure="height, posture and placement error";
      this.data.blockingPredicate=null;
      if(this.data.transferStage==="roll") {
        error=length(angularVelocity(pelvis.rotation(),this.rollTarget,1));
        this.data.progressMeasure="orientation error about measured body axis";
        this.data.blockingPredicate="prone preparation orientation";
      } else if(this.data.transferStage==="arm-preparation") {
        error=Math.max(0,PRONE_SHOULDER_CLEARANCE_M-this.proneShoulderClearance(poses))
          + SIDES.reduce((sum,side)=>sum+(this.armBraces.has(side)?length(sub(poses.get(`${side}Hand`)!.position,this.armBraces.get(side)!.position)):0),0);
        this.data.stageAction=!this.shoulderClearanceEstablished?"raise shoulder clearance":"place arm braces";
        this.data.progressMeasure="shoulder clearance, hand target error and usable brace persistence";
        this.data.blockingPredicate=!this.shoulderClearanceEstablished?"support-driven shoulder clearance"
          :SIDES.some(side=>!this.armBraces.has(side))?"legal reachable arm target":"measured usable arm brace";
      } else if(this.data.transferStage==="push-brace") {
        const destination=this.prospectiveSupport("brace","tuck-knee",poses);
        error=Math.max(0,-destination.geometry.marginM)+destination.excludedLoaded.reduce((sum,c)=>sum+c.forceN,0)/WEIGHT_N
          +Math.max(0,RECOVERY_LIMITS.phaseMinimumS-this.prospectiveTime);
        this.data.progressMeasure="prospective balance, excluded load and support persistence";
        this.data.blockingPredicate=!destination.balanced?"prospective brace balance":destination.excludedLoaded.length?"excluded body contacts still loaded":"brace support persistence";
      } else if(this.data.transferStage==="extend" || this.data.transferStage==="relax") {
        error=Math.max(0,HUMAN_PROPORTIONS.pelvis.centerHeightM-pelvis.translation().y)
          +Math.max(0,1-up)+Math.max(0,1-rotate(pelvis.rotation(),UP).y)
          +Math.max(0,-refreshed.marginM)+Math.max(0,motion.linear-RECOVERY_LIMITS.stableLinearMps)*.1
          +Math.max(0,motion.angular-RECOVERY_LIMITS.stableAngularRadps)*.1;
        this.data.progressMeasure="standing height, orientation, balance and speed deficits";
        this.data.blockingPredicate=this.data.stableTimeS>0?"standing persistence":"standing posture, contact or speed";
      } else {
        this.data.blockingPredicate=this.data.transferGuard?.qualified?"safe support release":"limb support and placement qualification";
      }
      const stabilityProgress = this.data.stableTimeS>this.bestStableTime+1e-9;
      this.bestStableTime = Math.max(this.bestStableTime,this.data.stableTimeS);
      if (error<this.bestError-.015 || stabilityProgress) { this.bestError=error; this.data.stalledTimeS=0; } else this.data.stalledTimeS+=dt;
      this.data.progressError = error;
      if (this.noSupportTime>RECOVERY_LIMITS.supportLossS || this.data.stalledTimeS>RECOVERY_LIMITS.stallS) {
        this.data.retryReason=this.noSupportTime>RECOVERY_LIMITS.supportLossS?"eligible support lost":`stalled: ${this.data.blockingPredicate}`;
        this.data.retries++; this.enter("settle",bodies); this.data.settledTimeS=0; active=false;
      }
    }
    this.data.noSupportTimeS=this.noSupportTime;
    this.data.noSupportReason=this.noSupportTime>0?"no eligible loaded contact":null;
    this.actuate(bodies,poses,active,dt);
    const phase=this.data.phase;
    return {state:active?"recovering":phase==="settle"?"fallen":"falling",recovered:false};
  }

  private transfer(bodies: Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>,dt:number): void {
    const phase=this.data.phase, lead=this.data.leadingSide!, trail:Side=lead==="left"?"right":"left";
    const leadFoot:SegmentId=`${lead}Foot`, trailingFoot:SegmentId=`${trail}Foot`;
    const sideFootContacts=(side:Side)=>this.data.contacts.filter(c=>c.loadBearing
      && segmentSide(c.segment)===side && isRecoveryFootSegment(c.segment) && !this.released.has(c.segment)
      && rotate(poses.get(c.segment)!.rotation,UP).y>.85);
    const sideFootContact=(side:Side)=>sideFootContacts(side).sort((a,b)=>b.forceN-a.forceN)[0];
    const loaded=(id:SegmentId)=>isRecoveryFootSegment(id)
      ? !!sideFootContact(segmentSide(id)!)
      : !this.released.has(id) && this.data.contacts.some(c=>c.segment===id && c.loadBearing);
    const pelvis=poses.get("pelvis")!, yaw=quatFromAxisAngle(UP,this.heading), mass=recoveryMassState(poses.values());
    for(const side of SIDES) {
      if(this.plants.has(`${side}Foot`) || this.footMovements.has(`${side}Foot`))continue;
      const plan=this.footPlans.get(side);
      if(plan?.kind==="blocked") {
        const age=(this.blockedFootSearchAge.get(side)??0)+dt;
        this.blockedFootSearchAge.set(side,age);
        const previous=this.blockedFootSearchPelvis.get(side);
        if(age<.15 && previous && length(sub(pelvis.position,previous))<.03)continue;
      }
      if(!plan || !revalidateRecoveryFootTarget(side,poses,plan)) {
        const next=reachableFootTarget(side,poses,this.heading,this.placements.get(`${side}Foot`));
        this.blockedFootSearchAge.set(side,0);
        this.blockedFootSearchPelvis.set(side,{...pelvis.position});
        if(next) {this.footPlans.set(side,next);if(next.kind!=="blocked")this.placements.set(`${side}Foot`,{...next.position});}
      }
    }
    for(const [id,movement] of this.footMovements) {
      // A pause describes the previous step's evidence. Reassess it now;
      // retaining it forever would freeze a valid swing after a transient limit.
      movement.paused=false;
      if(!revalidateRecoveryFootTarget(movement.side,poses,movement.target)) {
        const next=reachableFootTarget(movement.side,poses,this.heading,movement.target.position);
        if(next?.feasible) {movement.target=next;this.footPlans.set(movement.side,next);this.placements.set(id,{...next.position});}
        else {movement.paused=true;continue;}
      }
      const contact=this.data.contacts.find(c=>c.segment===id && c.loadBearing);
      if(movement.paused && contact && length(sub(poses.get(id)!.position,movement.target.position))>=.14) {
        movement.origin={...poses.get(id)!.position};movement.originRotation={...poses.get(id)!.rotation};movement.time=0;movement.unloaded=false;movement.paused=false;
      }
      if(!movement.paused)movement.time+=dt;
    }
    const geometry=supportGeometry(this.supporting(poses),poses,mass);
    const trailingContact=sideFootContact(trail);
    const trailingPose=poses.get(trailingFoot), trailingTarget=this.placements.get(trailingFoot);
    const targetDistanceM=trailingPose && trailingTarget ? length(sub(trailingPose.position,trailingTarget)) : null;
    const footUpDot=trailingPose ? rotate(trailingPose.rotation,UP).y : 0;
    const footHeightM=trailingPose?.position.y ?? 0;
    const leadingLoadN=sideFootContacts(lead).reduce((sum,c)=>sum+c.forceN,0);
    const readyToLift=leadingLoadN>WEIGHT_N*.55 && mass.velocity.y>-.05 && pelvis.position.y>.65;
    const qualified=!!trailingContact
      && trailingContact.persistenceS>=.20 && trailingContact.forceN>WEIGHT_N*.08
      && footUpDot>.95 && footHeightM<.075 && targetDistanceM!==null && targetDistanceM<.14
      && geometry.marginM>.015 && mass.velocity.y>-.10;
    this.data.transferGuard={segment:trailingContact?.segment??trailingFoot,hasContact:!!trailingContact,
      persistenceS:trailingContact?.persistenceS??0,forceN:trailingContact?.forceN??0,
      footUpDot,footHeightM,targetDistanceM,supportMarginM:geometry.marginM,
      massVelocityYMps:mass.velocity.y,leadingLoadN,readyToLift,qualified};
    let center=geometry.center;
    let height=pelvis.position.y;
    let tilt=.0;
    if(phase==="roll" && this.placingProneArms) {
      this.rootGoal={...pelvis.position};this.rootRotation=this.placementRootRotation;
      const clearance=this.proneShoulderClearance(poses);
      if(!this.supporting(poses).length)return;
      // A clearance threshold only authorizes a search. It does not prove a
      // floor-safe hand target exists. Preserve any legal target through a
      // transient dip, and keep lifting the torso until a real target is found.
      const searchDue=this.stageTime-this.lastArmSearchTimeS>=.15
        || clearance-this.lastArmSearchClearanceM>=.025;
      if(clearance>=PRONE_SHOULDER_CLEARANCE_M && searchDue) {
        this.lastArmSearchTimeS=this.stageTime;
        this.lastArmSearchClearanceM=clearance;
        for(const side of SIDES) {
          if(this.movingArms.has(side))continue;
          const captured=this.armBraces.get(side);
          if(captured && this.legalArmCommand(side,captured.position,poses))continue;
          const target=reachableArmBraceTarget(side,poses,this.heading);
          if(target && acceptableRecoveryArmBraceTarget(target)) {
            this.armBraces.set(side,target);
            if(!captured) {
              this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});
              this.armTimes.set(side,0);
            }
          }
        }
      }
      this.shoulderClearanceEstablished=this.armBraces.size>0;
      if(!this.shoulderClearanceEstablished)return;
      const first=this.data.rollSide!,second:Side=first==="left"?"right":"left";
      const alreadyMoving=[...this.movingArms][0];
      const placementOrder:Side[]=alreadyMoving?[alreadyMoving]:[first,second].sort((a,b)=>(
        this.armBraces.get(a)?.movementM??Infinity)-(this.armBraces.get(b)?.movementM??Infinity)
      );
      for(const side of placementOrder) {
        if(this.armPlacementReady(side,poses)) continue;
        const target=this.armBraces.get(side);
        if(!target || !this.legalArmCommand(side,target.position,poses)) {
          this.data.blockingPredicate="legal reachable arm target";
          continue;
        }
        const moving=this.movingArms.has(side);
        if(moving || this.release(armSupportSegments(side),poses) !== "blocked") {
          if(!moving) {
            this.movingArms.add(side);
            this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});this.armTimes.set(side,0);
          }
          const t=(this.armTimes.get(side)??0)+dt;this.armTimes.set(side,t);
          const start=this.armStarts.get(side)??poses.get(`${side}Hand`)!.position;
          const p=lerp(start,target.position,smooth01(t/.8));
          const liftHeight=clamp(length(sub(start,target.position))*.35+.015,.02,.10);
          // The measured resting arm can contain small contact/constraint
          // residue that has no legal inverse-kinematic equivalent. Begin at
          // a legal lifted target and ramp its motor strength from zero; then
          // lower onto the captured brace during the second half.
          const lift=liftHeight*(1-smooth01((t-.45)/.35));
          let legal:Vec3|null=null;
          for(const fraction of [1,.75,.5,.25,0]) {
            const candidate={...p,y:p.y+lift*fraction};
            if(this.legalArmCommand(side,candidate,poses)){legal=candidate;break;}
          }
          if(legal)this.placements.set(`${side}Hand`,legal);
          else {this.armTimes.set(side,Math.max(0,t-dt));this.data.blockingPredicate="legal shoulder and clearance trajectory";}
          this.bend.set(`${side}UpperArm`,rotate(quatInverse(poses.get("torso")!.rotation),target.bend));
        }
        // Finish one fresh brace before asking it to support the other arm.
        break;
      }
      return;
    }    if(phase==="roll" && this.data.transferStage==="push-brace") {
      this.rootGoal={...this.pushOrigin,y:Math.min(.24,this.pushOrigin.y+.10)};
      const intended=quatMultiply(yaw,pitch(.48));
      const turn=angularVelocity(this.pushRootRotation,intended,1);
      const progress=smooth01(this.stageTime/1.20);
      this.rootRotation=quatMultiply(
        quatFromAxisAngle(normalize(turn),length(turn)*progress),this.pushRootRotation,
      );
      return;
    }    if(phase==="roll") {
      const forward=rotate(pelvis.rotation,FORWARD);
      this.rollTurned ||= forward.y<-.45;
      this.rootRotation=this.rollTarget;
      if(this.rollTurned) {this.prepareArms(bodies,poses);return;}
      if(this.data.route==="prone" || this.rollTurned) {
        for(const side of SIDES) {
          const foot:SegmentId=`${side}Foot`;
          if(rotate(poses.get(foot)!.rotation,UP).y<.5
            && this.release([`${side}Shin`,...legSwingSegments(side)],poses) !== "blocked") {
            this.placements.set(foot,{...add(pelvis.position,rotate(yaw,{x:(side==="left"?-1:1)*.14,y:0,z:side===lead?.10:-.36})),y:.05});
          }
        }
      }
    } else {
      if(phase==="brace") {
        height=.52; tilt=.48;
        if(this.data.transferStage!=="plant-lead") {
          const foot:SegmentId=`${trail}Foot`;
          if(this.release([`${trail}Shin`,...legSwingSegments(trail)],poses) !== "blocked")
            this.placements.set(foot,{...add(pelvis.position,rotate(yaw,{x:(trail==="left"?-1:1)*.16,y:0,z:-.36})),y:.066});
        }
      }
      else if(phase==="kneel") {
        const baseHeight=(loaded(leadFoot) || this.plants.has(leadFoot))?.80:.58;
        const extendBlend=this.data.transferStage==="extend"?smooth01(this.stageTime/1.20):0;
        height=baseHeight+(HUMAN_PROPORTIONS.pelvis.centerHeightM-baseHeight)*extendBlend;
        tilt=.18*(1-extendBlend);
      }
      else { const standBlend=smooth01(this.data.phaseTimeS/.80); height=this.standOriginY+(HUMAN_PROPORTIONS.pelvis.centerHeightM-this.standOriginY)*standBlend; tilt=.18*(1-standBlend); }
      const feet=this.data.contacts.filter(c=>isRecoveryFootSegment(c.segment) && loaded(c.segment));
      let trailingSupportBlend=0;
      if (supportingSideCount(feet,isRecoveryFootSegment)===2) {
        const ankle=(id:SegmentId)=>{const p=this.plants.get(id)??poses.get(id)!;return worldPoint(p.position,p.rotation,SEGMENT_BY_ID.get(id)!.jointAnchorChild!);};
        const leadingAnkle=ankle(leadFoot), trailingAnkle=ankle(trailingFoot);
        if(phase==="kneel" && this.data.transferStage==="shift-weight") {
          const leadingLoad=sideFootContacts(lead).reduce((sum,c)=>sum+c.forceN,0);
          const trailingLoad=sideFootContacts(trail).reduce((sum,c)=>sum+c.forceN,0);
          const loadShare=clamp(leadingLoad/Math.max(leadingLoad+trailingLoad,1e-6),0,1);
          // Keep the measured two-foot base as the virtual target while load
          // settles. Filtering the measured share limits reaction-driven
          // target jumps without adding support area or changing the release
          // qualification gates.
          this.transferBlend += clamp(loadShare-this.transferBlend,-dt/.24,dt/.24);
          center=lerp(scale(add(leadingAnkle,trailingAnkle),.5),leadingAnkle,smooth01(this.transferBlend));
        } else if(phase==="kneel" && this.data.transferStage==="bring-trailing") {
          const contact=sideFootContact(trail)!;
          trailingSupportBlend=smooth01(clamp((contact.persistenceS-.05)/.20,0,1))*clamp(contact.forceN/(WEIGHT_N*.20),0,1);
          center=lerp(leadingAnkle,scale(add(leadingAnkle,trailingAnkle),.5),trailingSupportBlend);
        } else if(phase==="kneel" && this.data.transferStage==="extend") {
          center=lerp(leadingAnkle,scale(add(leadingAnkle,trailingAnkle),.5),smooth01(this.stageTime/.35));
        } else center=scale(add(leadingAnkle,trailingAnkle),.5);
      } else if(loaded(leadFoot)) {
        const foot=this.plants.get(leadFoot)??poses.get(leadFoot)!;
        const ankle=worldPoint(foot.position,foot.rotation,SEGMENT_BY_ID.get(leadFoot)!.jointAnchorChild!);
        center=ankle;
      }
      // Shift the virtual root so the whole mass, including the bent torso, moves over support.
      // Transfer over the measured lower-limb base before asking it to raise
      // the pelvis. Height alone cannot start this correction from a genuine
      // half-kneel: its pelvis begins below 0.65 m and the planted sole unloads
      // while the controller waits for a rise that has not begun.
      const riseBlend=phase==="kneel"
        ?Math.max(smooth01((pelvis.position.y-.65)/.13),
          this.data.transferStage==="shift-weight"?smooth01(this.stageTime/.65):0)
        :1;
      const initialCorrection=clampLength({x:this.riseOrigin.x-pelvis.position.x,y:0,z:this.riseOrigin.z-pelvis.position.z},.05);
      // During the single-foot transfer, correct against the same 0.15 s COM
      // projection used by the release guard. Correcting only the measured COM
      // lets the swing-leg reaction reach the outside edge of the planted sole
      // before the pelvis target responds. Keep a small, heading-relative
      // reserve toward the body's midline and rear half of the supporting foot.
      const transferring=phase==="kneel" && this.data.transferStage==="bring-trailing" && loaded(leadFoot);
      const supportReserve=transferring
        ? scale(rotate(yaw,{x:this.data.leadingSide==="left"?.025:-.025,y:0,z:-.015}),1-trailingSupportBlend)
        : ZERO;
      const balanceTarget=add(center,supportReserve);
      const predictedMass=transferring ? add(mass.position,scale(mass.velocity,.15)) : mass.position;
      const balancePoint=phase==="kneel" && this.data.transferStage==="extend" ? pelvis.position : predictedMass;
      const centerCorrection=clampLength({x:balanceTarget.x-balancePoint.x,y:0,z:balanceTarget.z-balancePoint.z},.16);
      const correction=phase==="kneel"?add(scale(initialCorrection,1-riseBlend),scale(centerCorrection,riseBlend)):centerCorrection;
      const measured=pelvis.position;
      this.rootGoal={x:measured.x+correction.x,y:Math.min(height,measured.y+.10),z:measured.z+correction.z};
      const intendedRotation=quatMultiply(yaw,pitch(tilt)),rootDelta=angularVelocity(this.rootEntry,intendedRotation,1);
      this.rootRotation=phase==="kneel"?quatMultiply(quatFromAxisAngle(normalize(rootDelta),length(rootDelta)*riseBlend),this.rootEntry):intendedRotation;
      for(const side of SIDES) {
        const shin:SegmentId=`${side}Shin`, plant=this.plants.get(shin);
        if(!plant || this.released.has(shin)) continue;
        const knee=this.plantedKnee(side,plant);
        const anchor=SEGMENT_BY_ID.get(`${side}Thigh`)!.jointAnchorParent!;
        const hip=worldPoint(this.rootGoal,this.rootRotation,anchor),delta=sub(hip,knee);
        if(length(delta)>.418) this.rootGoal=sub(add(knee,scale(normalize(delta),.418)),rotate(this.rootRotation,anchor));
      }
      if(phase==="brace" && loaded(`${trail}Shin`)) this.stage("plant-lead",bodies);
      if((phase==="brace" || phase==="kneel") && this.data.transferStage==="plant-lead") {
        const shin=this.data.contacts.find(c=>c.segment===`${trail}Shin` && c.loadBearing && c.persistenceS>=.20);
        const arms=SIDES.some(side=>usableRecoveryArmSupport(side,poses,this.supporting(poses)));
        if(shin && arms && !this.footMovements.has(leadFoot) && !loaded(leadFoot))
          this.beginFootMovement(lead,poses,legSwingSegments(lead));
        if(loaded(leadFoot) && !this.footMovements.has(leadFoot))this.stage("shift-weight",bodies);
      }
      if(phase==="kneel" && loaded(leadFoot) && this.data.transferStage!=="extend") {
        if(!loaded(trailingFoot)) {
          if(this.data.transferStage!=="bring-trailing") this.stage("shift-weight",bodies);
          const toe=sideFootContact(trail);
          if(toe && this.plants.has(`${trail}Shin`)) this.release([`${trail}Shin`],poses);
          const leadLoad=sideFootContacts(lead).reduce((sum,c)=>sum+c.forceN,0);
          const readyToLift=leadLoad>WEIGHT_N*.55 && mass.velocity.y>-.05 && pelvis.position.y>.65;
          if(this.data.transferStage!=="bring-trailing" && readyToLift) {
            const foot=this.plants.get(leadFoot)??poses.get(leadFoot)!;
            const offset=rotate(yaw,{x:(trail==="left"?-1:1)*.24,y:0,z:-.04});
            if(this.beginFootMovement(trail,poses,[`${trail}Shin`,...legSwingSegments(trail)],{...add(foot.position,offset),y:.049})) {
              this.stage("bring-trailing",bodies);this.swingOrigin={...poses.get(trailingFoot)!.position};
            }
          }
        } else if(this.data.transferStage==="shift-weight") {
          const contact=sideFootContact(trail)!;
          const nearTarget=length(sub(poses.get(trailingFoot)!.position,this.placements.get(trailingFoot)!))<.14;
          const qualified=contact.persistenceS>=.20 && contact.forceN>WEIGHT_N*.08
            && rotate(poses.get(trailingFoot)!.rotation,UP).y>.95 && poses.get(trailingFoot)!.position.y<.075
            && nearTarget && geometry.marginM>.015 && mass.velocity.y>-.10;
          const leadLoad=sideFootContacts(lead).reduce((sum,c)=>sum+c.forceN,0);
          const readyToLift=leadLoad>WEIGHT_N*.55 && mass.velocity.y>-.05 && pelvis.position.y>.65;
          if(qualified && readyToLift && this.release([`${trail}Shin`],poses) !== "blocked") this.stage("extend",bodies);
        } else {
          const contact=sideFootContact(trail)!;
          const nearTarget=length(sub(poses.get(trailingFoot)!.position,this.placements.get(trailingFoot)!))<.14;
          if(this.data.transferStage==="bring-trailing" && contact.persistenceS>=.20 && contact.forceN>WEIGHT_N*.08
            && rotate(poses.get(trailingFoot)!.rotation,UP).y>.95 && poses.get(trailingFoot)!.position.y<.075
            && nearTarget && geometry.marginM>.015 && mass.velocity.y>-.10) this.stage("extend",bodies);
        }
      }
      if(phase==="kneel" && this.data.transferStage==="bring-trailing" && !!sideFootContact(trail)
        && length(sub(poses.get(trailingFoot)!.position,this.placements.get(trailingFoot)!))>.12)
        this.release(legSwingSegments(trail),poses);
      if(phase==="stand" && this.data.transferStage!=="relax" && loaded(leadFoot) && loaded(trailingFoot)
        && pelvis.position.y>.93 && rotate(pelvis.rotation,UP).y>.94 && rotate(poses.get("torso")!.rotation,UP).y>.96
        && geometry.marginM>.02 && mass.velocity.y>-.10) {
        const arms=this.data.contacts.filter(c=>c.loadBearing&&isRecoveryArmSupportSegment(c.segment)).map(c=>c.segment);
        if(!arms.length || this.release(arms,poses) !== "blocked") this.stage("relax",bodies);
      }
    }
    if(phase==="roll") this.rootGoal={...pelvis.position};
    this.data.extension=clamp((pelvis.position.y-.55)/.44,0,1);
    // A planned foot may be captured again once it has actually replanted.

    void dt;
  }

  private plantedKnee(side:Side,plant:Plant):Vec3 {
    const definition=SEGMENT_BY_ID.get(`${side}Shin`)!;
    if(this.data.phase!=="kneel" || side===this.data.leadingSide || !plant.contact)
      return worldPoint(plant.position,plant.rotation,definition.jointProfile!.childFrame.anchor);
    // Preserve the measured material point that established the knee contact.
    // This works for the shared convex shin geometry without assuming a capsule.
    const localContact=rotate(quatInverse(plant.rotation),sub(plant.contact,plant.position));
    const q=quatMultiply(quatFromAxisAngle(UP,this.heading),pitch(1.8));
    const targetPosition=sub(plant.contact,rotate(q,localContact));
    return worldPoint(targetPosition,q,definition.jointProfile!.childFrame.anchor);
  }
  private kneelTorso(poses:Map<SegmentId,SegmentPose>):Quat {
    const u=smooth01((poses.get("pelvis")!.position.y-.65)/.15);
    const extension=this.data.transferStage==="extend"?smooth01(this.stageTime/1.20):0;
    return rotation((.3-.22*u)*(1-extension),0,(this.data.leadingSide==="left"?1:-1)*(.20-.14*u)*(1-extension));
  }
  private torsoWorldTarget(poses: Map<SegmentId, SegmentPose>): Quat {
    const relative = this.placingProneArms ? this.placementClearanceTorsoRotation
      : this.data.phase === "kneel" ? this.kneelTorso(poses)
      : pitch(this.data.phase === "roll" ? -.3 : this.data.phase === "brace" ? -.12 : 0);
    return quatMultiply(this.rootRotation, relative);
  }
  private measuredLocalRotation(id:SegmentId,poses:ReadonlyMap<SegmentId,SegmentPose>):Quat {
    const definition=SEGMENT_BY_ID.get(id)!;
    const child=poses.get(id)!;
    const parent=definition.parent ? poses.get(definition.parent) : null;
    return parent ? quatMultiply(quatInverse(parent.rotation),child.rotation) : child.rotation;
  }
  private limbTargets(side:Side,arm:boolean,poses:Map<SegmentId,SegmentPose>,targets:Map<SegmentId,Quat>):void {
    const ids=recoveryLimbIds(side,arm);
    const yaw=quatFromAxisAngle(UP,this.heading),phase=this.data.phase;
    const setEntry=()=>{for(const id of ids) targets.set(id,this.entry.get(id)
      ?? SEGMENT_BY_ID.get(id)!.restLocalRotation);};
    if(arm) {
      const [girdle,upper,forearm,twist,hand]=ids;
      const setMeasured=()=>{for(const id of ids)
        targets.set(id,limitRecoveryJoint(id,this.measuredLocalRotation(id,poses)));};
      const materialPatch=(id:SegmentId,plant:Plant,pose:SegmentPose):Vec3=>
        worldPoint(pose.position,pose.rotation,
          rotate(quatInverse(plant.rotation),sub(plant.contact!,plant.position)));
      const lowerPlant=this.plants.get(forearm)??this.plants.get(twist);
      const handPlant=this.plants.get(hand);
      const brace=this.armBraces.get(side);
      let desired=this.placements.get(hand)??poses.get(hand)!.position;
      if(handPlant) desired=handPlant.position;
      if(!handPlant && (phase==="stand" || (phase==="kneel"&&this.data.transferStage==="extend"))) {
        const girdlePose=poses.get(girdle)!;
        const shoulder=worldPoint(girdlePose.position,girdlePose.rotation,
          SEGMENT_BY_ID.get(upper)!.jointProfile!.parentFrame.anchor);
        desired=add(shoulder,rotate(yaw,{x:(side==="left"?-1:1)*.04,y:-.70,z:.04}));
      }
      const torso=poses.get("torso")!;
      const girdleLocal=limitRecoveryJoint(girdle,this.entry.get(girdle)
        ?? this.measuredLocalRotation(girdle,poses));
      const girdleProfile=SEGMENT_BY_ID.get(girdle)!.jointProfile!;
      const girdleWorld=quatMultiply(torso.rotation,girdleLocal);
      const girdlePosition=sub(worldPoint(torso.position,torso.rotation,girdleProfile.parentFrame.anchor),
        rotate(girdleWorld,girdleProfile.childFrame.anchor));
      const shoulder=worldPoint(girdlePosition,girdleWorld,
        SEGMENT_BY_ID.get(upper)!.jointProfile!.parentFrame.anchor);
      if(this.placingProneArms && !handPlant && !lowerPlant && brace) {
        const legal=this.legalArmCommand(side,desired,poses);
        if(legal) for(const id of ids) targets.set(id,legal.get(id)!);
        else setEntry();
        return;
      }
      const forearmPlant=this.plants.get(forearm),twistPlant=this.plants.get(twist);
      if(!handPlant && forearmPlant && !this.released.has(forearm)) {
        const elbow=worldPoint(forearmPlant.position,forearmPlant.rotation,
          SEGMENT_BY_ID.get(forearm)!.jointProfile!.childFrame.anchor);
        const upperWorld=quatMultiply(quatFromTo(UP,normalize(sub(shoulder,elbow))),yaw);
        targets.set(girdle,girdleLocal);
        targets.set(upper,limitRecoveryJoint(upper,quatMultiply(quatInverse(girdleWorld),upperWorld)));
        targets.set(forearm,limitRecoveryJoint(forearm,
          quatMultiply(quatInverse(upperWorld),forearmPlant.rotation)));
        targets.set(twist,limitRecoveryJoint(twist,this.measuredLocalRotation(twist,poses)));
        targets.set(hand,limitRecoveryJoint(hand,this.measuredLocalRotation(hand,poses)));
        if(forearmPlant.contact) {
          const predicted=reconstructRecoveryLimb(side,true,torso,targets);
          const patch=materialPatch(forearm,forearmPlant,predicted.poses.find(pose=>pose.id===forearm)!);
          if(length(sub(patch,forearmPlant.contact))>.015 || predicted.floorClearanceM<-.003)
            setMeasured();
        }
        return;
      }
      if(!handPlant && twistPlant && !this.released.has(twist)) {
        for(const id of [girdle,upper,forearm])
          targets.set(id,limitRecoveryJoint(id,this.measuredLocalRotation(id,poses)));
        targets.set(twist,limitRecoveryJoint(twist,
          quatMultiply(quatInverse(poses.get(forearm)!.rotation),twistPlant.rotation)));
        targets.set(hand,limitRecoveryJoint(hand,this.measuredLocalRotation(hand,poses)));
        return;
      }
      if(phase==="roll" && this.data.transferStage==="push-brace"
        && !handPlant && !this.movingArms.has(side)) {
        // A hand that never established a material patch has no fixed floor
        // target. Holding its measured joint pose avoids folding an unanchored
        // wrist into the trunk as the torso moves.
        setMeasured();
        return;
      }
      const intermediatePlant=!this.released.has(forearm) && forearmPlant
        ?{id:forearm,plant:forearmPlant}
        :!this.released.has(twist) && twistPlant?{id:twist,plant:twistPlant}:null;
      let plantedBend:Vec3|null=null;
      if(handPlant && intermediatePlant) {
        const middle=worldPoint(intermediatePlant.plant.position,intermediatePlant.plant.rotation,
          SEGMENT_BY_ID.get(intermediatePlant.id)!.jointProfile!.childFrame.anchor);
        const ray=normalize(sub(desired,shoulder));
        const residue=sub(sub(middle,shoulder),scale(ray,dot(sub(middle,shoulder),ray)));
        if(length(residue)>.01)plantedBend=normalize(residue);
      }
      const wristLocal=handPlant?this.measuredLocalRotation(hand,poses)
        :brace?.wristRotation??recoveryJointRotation(hand,ZERO);
      const bend=plantedBend??brace?.trajectoryBend??rotate(girdleWorld,this.bend.get(upper)??{x:0,y:0,z:-1});
      const commandFor=(position:Vec3):Map<SegmentId,Quat>=>{
        const solved=solveRecoveryArmTarget(shoulder,position,bend,wristLocal,this.heading);
        return new Map<SegmentId,Quat>([
          [girdle,girdleLocal],
          [upper,limitRecoveryJoint(upper,quatMultiply(quatInverse(girdleWorld),solved.upperRotation))],
          [forearm,limitRecoveryJoint(forearm,quatMultiply(quatInverse(solved.upperRotation),solved.forearmRotation))],
          [twist,limitRecoveryJoint(twist,quatMultiply(quatInverse(solved.forearmRotation),solved.forearmTwistRotation))],
          [hand,limitRecoveryJoint(hand,quatMultiply(quatInverse(solved.forearmTwistRotation),solved.handRotation))],
        ]);
      };
      if(handPlant?.contact && !this.released.has(hand)) {
        let requested={...desired};
        let best:Map<SegmentId,Quat>|null=null,bestError=Infinity;
        for(let iteration=0;iteration<4;iteration++) {
          const candidate=commandFor(requested);
          const geometry=reconstructRecoveryLimb(side,true,torso,candidate);
          const complete=new Map<SegmentId,SegmentPose>(poses);
          for(const pose of geometry.poses)complete.set(pose.id,pose);
          const patch=materialPatch(hand,handPlant,geometry.poses.find(pose=>pose.id===hand)!);
          const error=length(sub(handPlant.contact,patch));
          if(geometry.floorClearanceM>=-.003
            && limbTrunkClearance(complete,side,"arm").clearanceM>=.004 && error<bestError) {
            best=candidate;bestError=error;
          }
          requested=add(requested,clampLength(sub(handPlant.contact,patch),.06));
        }
        if(best && bestError<=.015) for(const [id,rotation] of best)targets.set(id,rotation);
        else setMeasured();
      } else for(const [id,rotation] of commandFor(desired))targets.set(id,rotation);
      return;
    }

    const [thigh,shin,ankle,foot,forefoot]=ids;
    const plan=this.footPlans.get(side),shinPlant=this.plants.get(shin);
    const forefootPlant=this.plants.get(forefoot);
    if(phase==="roll" && (this.data.transferStage==="arm-preparation"
      || this.data.transferStage==="push-brace") && forefootPlant?.contact
      && !this.released.has(forefoot) && !this.slippedPlants.has(forefoot)) {
      // A briefly airborne captured toe remains a replant posture target for
      // the existing 0.10 s plant grace. It contributes no planned support
      // until a new Rapier load is observed.
      const anchored=solveRecoveryForefootAnchor(side,poses,{
        position:forefootPlant.position,rotation:forefootPlant.rotation,
        contact:forefootPlant.contact,
      });
      if(anchored) {
        const rotations=anchored.jointRotations;
        targets.set(thigh,rotations.thigh);targets.set(shin,rotations.shin);
        targets.set(ankle,rotations.ankle);targets.set(foot,rotations.foot);
        targets.set(forefoot,rotations.forefoot);
        return;
      }
    }
    if(!this.plants.get(foot) && !shinPlant && plan && !plan.feasible) {
      if(plan.kind==="intermediate-fold") {
        const rotations=plan.jointRotations;
        targets.set(thigh,rotations.thigh);targets.set(shin,rotations.shin);
        targets.set(ankle,rotations.ankle);targets.set(foot,rotations.foot);
        targets.set(forefoot,rotations.forefoot);
      } else setEntry();
      return;
    }
    if(shinPlant && !this.released.has(shin)) {
      const pelvis=poses.get("pelvis")!;
      const hip=worldPoint(pelvis.position,pelvis.rotation,
        SEGMENT_BY_ID.get(thigh)!.jointProfile!.parentFrame.anchor);
      const knee=this.plantedKnee(side,shinPlant);
      const shinWorld=this.data.phase==="kneel" && side!==this.data.leadingSide && shinPlant.contact
        ? quatMultiply(yaw,pitch(1.8)):shinPlant.rotation;
      const thighWorld=quatMultiply(quatFromTo(UP,normalize(sub(hip,knee))),yaw);
      targets.set(thigh,limitRecoveryJoint(thigh,quatMultiply(quatInverse(pelvis.rotation),thighWorld)));
      targets.set(shin,limitRecoveryJoint(shin,quatMultiply(quatInverse(thighWorld),shinWorld)));
      for(const id of [ankle,foot,forefoot])
        targets.set(id,limitRecoveryJoint(id,this.measuredLocalRotation(id,poses)));
      return;
    }
    let desired=this.placements.get(foot)??plan?.position??poses.get(foot)!.position;
    let desiredRotation=plan?.rotation??yaw;
    const footPlant=this.plants.get(foot);
    if(footPlant) {desired=footPlant.position;desiredRotation=footPlant.rotation;}
    if((phase==="brace"||phase==="kneel") && side!==this.data.leadingSide
      && this.data.transferStage!=="bring-trailing" && this.data.transferStage!=="extend" && !footPlant) {
      desired={...add(this.rootGoal,rotate(yaw,{x:(side==="left"?-1:1)*.14,y:0,z:-.38})),y:.07};
      desiredRotation=quatMultiply(yaw,pitch(.3));
    }
    if(side!==this.data.leadingSide && this.data.transferStage==="bring-trailing" && !footPlant) {
      const u=clamp(this.stageTime/.42,0,1),goal=this.placements.get(foot)??desired;
      const swing=lerp(this.swingOrigin,goal,smooth01(u));
      desired={...swing,y:swing.y+.12*Math.sin(Math.PI*u)};
    }
    const movement=this.footMovements.get(foot);
    if(movement) {
      const u=movement.unloaded?clamp(movement.time/.42,0,1):0;
      const swing=lerp(movement.origin,movement.target.position,smooth01(u));
      desired={...swing,y:swing.y+(movement.unloaded?.12*Math.sin(Math.PI*u):.12*smooth01(movement.time/.22))};
      const turn=angularVelocity(movement.originRotation,movement.target.rotation,1);
      desiredRotation=quatMultiply(quatFromAxisAngle(normalize(turn),length(turn)*smooth01(movement.time/.22)),movement.originRotation);
    }
    const solved=solveRecoveryLegTarget(side,poses,desired,desiredRotation,
      rotate(poses.get("pelvis")!.rotation,this.bend.get(thigh)??{x:0,y:0,z:1}));
    if(!solved) {setEntry();return;}
    const rotations=solved.jointRotations;
    targets.set(thigh,rotations.thigh);targets.set(shin,rotations.shin);
    targets.set(ankle,rotations.ankle);targets.set(foot,rotations.foot);
    targets.set(forefoot,rotations.forefoot);
  }

  private actuate(bodies:Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>,active:boolean,dt:number):void {
    // Unsupported settling has no posture motor. Rapier still enforces every
    // joint limit while passive resistance damps motion near those limits.
    if(this.data.phase==="settle") {
      this.lastPassiveTorques=applyPassiveJointResistance(bodies,dt);
      for(const torque of this.lastPassiveTorques.values())
        this.data.maxMotorTorqueNm=Math.max(this.data.maxMotorTorqueNm,length(torque));
      return;
    }
    const targets=new Map<SegmentId,Quat>();
    const holdCrouch=!active && poses.get("pelvis")!.position.y>.40 && poses.get("pelvis")!.position.y<.85
      && rotate(poses.get("torso")!.rotation,UP).y>.65 && SIDES.some(side=>poses.get(`${side}Foot`)!.position.y<.13);
    const freeProneLegs=active && this.data.phase==="roll"
      && (this.placingProneArms || this.data.transferStage==="push-brace")
      && SIDES.every(side=>!this.footMovements.has(`${side}Foot`)
        && recoveryLimbIds(side,false).every(id=>!this.plants.has(id))
        && !this.data.contacts.some(contact=>contact.loadBearing
          && segmentSide(contact.segment)===side
          && isRecoveryLegSupportSegment(contact.segment)));
    if(active) {
      // The pelvis is never actuated directly. Split trunk orientation over
      // both spinal joints so only contact reactions rotate the assembly.
      const measuredPelvis=poses.get("pelvis")!.rotation,desiredTorso=this.torsoWorldTarget(poses);
      const spineError=angularVelocity(measuredPelvis,desiredTorso,1);
      const desiredLumbarWorld=quatMultiply(
        quatFromAxisAngle(normalize(spineError),length(spineError)*.45),measuredPelvis,
      );
      const lumbar=limitRecoveryJoint("lumbar",quatMultiply(quatInverse(measuredPelvis),desiredLumbarWorld));
      const reachableLumbarWorld=quatMultiply(measuredPelvis,lumbar);
      targets.set("lumbar",lumbar);
      targets.set("torso",limitRecoveryJoint("torso",quatMultiply(quatInverse(reachableLumbarWorld),desiredTorso)));
      targets.set("neck",recoveryJointRotation("neck",ZERO));
      targets.set("head",recoveryJointRotation("head",ZERO));
      for(const side of SIDES) {
        const holdingProneBase=this.placingProneArms
          || (this.data.phase==="roll" && this.data.transferStage==="push-brace");
        const anchoredForefoot:SegmentId=`${side}Forefoot`;
        const anchoredShin:SegmentId=`${side}Shin`;
        const canAnchorProneForefoot=holdingProneBase
          && !!this.plants.get(anchoredForefoot)?.contact
          && !this.released.has(anchoredForefoot)
          && !this.slippedPlants.has(anchoredForefoot);
        const canAnchorProneShin=holdingProneBase
          && !!this.plants.get(anchoredShin)?.contact
          && !this.released.has(anchoredShin)
          && !this.slippedPlants.has(anchoredShin);
        if(holdingProneBase && !canAnchorProneForefoot && !canAnchorProneShin) for(const id of recoveryLimbIds(side,false))
          targets.set(id,this.entry.get(id)??SEGMENT_BY_ID.get(id)!.restLocalRotation);
        else this.limbTargets(side,false,poses,targets);
        if(this.data.route==="half-kneel" && (this.data.phase==="kneel" || (this.data.phase==="stand" && this.data.transferStage!=="relax"))) {
          for(const id of recoveryLimbIds(side,true))
            targets.set(id,this.entry.get(id)??SEGMENT_BY_ID.get(id)!.restLocalRotation);
        } else this.limbTargets(side,true,poses,targets);
      }
      if(freeProneLegs) for(const side of SIDES) {
        const thigh:SegmentId=`${side}Thigh`;
        const profile=SEGMENT_BY_ID.get(thigh)!.jointProfile!;
        const current=jointCoordinates({x:0,y:0,z:0,w:1},targets.get(thigh)!,profile);
        // Only airborne, unplanted legs may open away from the midline.
        // Their existing joints and bounded motors supply all motion.
        const outward=side==="left"?Math.min(current.z,-.08):Math.max(current.z,.08);
        targets.set(thigh,recoveryJointRotation(thigh,{...current,z:outward}));
      }
    } else {
      const backward=this.data.orientation==="backward";
      targets.set("lumbar",recoveryJointRotation("lumbar",{x:backward?.12:-.08,y:0,z:0}));
      targets.set("torso",recoveryJointRotation("torso",{x:backward?.18:-.10,y:0,z:0}));
      targets.set("neck",recoveryJointRotation("neck",{x:.12,y:0,z:0}));
      targets.set("head",recoveryJointRotation("head",{x:.20,y:0,z:0}));
      for(const side of SIDES) {
        const sign=side==="left"?-1:1;
        targets.set(`${side}ShoulderGirdle`,recoveryJointRotation(`${side}ShoulderGirdle`,{x:.12,y:0,z:.10}));
        targets.set(`${side}UpperArm`,rotation(backward?-.25:-1.25,0,this.data.orientation===side?sign*1.1:sign*.22));
        targets.set(`${side}Forearm`,recoveryJointRotation(`${side}Forearm`,{x:.9,y:0,z:0}));
        targets.set(`${side}ForearmTwist`,recoveryJointRotation(`${side}ForearmTwist`,ZERO));
        targets.set(`${side}Hand`,recoveryJointRotation(`${side}Hand`,{x:-.25,y:0,z:0}));
        targets.set(`${side}Thigh`,recoveryJointRotation(`${side}Thigh`,{x:-.20,y:0,z:0}));
        targets.set(`${side}Shin`,recoveryJointRotation(`${side}Shin`,{x:.70,y:0,z:0}));
        targets.set(`${side}Ankle`,recoveryJointRotation(`${side}Ankle`,{x:-.20,y:0,z:0}));
        targets.set(`${side}Foot`,recoveryJointRotation(`${side}Foot`,ZERO));
        targets.set(`${side}Forefoot`,recoveryJointRotation(`${side}Forefoot`,{x:.15,y:0,z:0}));
      }
      if(holdCrouch)
        for(const [id,q] of this.entry) targets.set(id,q);
    }
    const blend=smooth01(this.stageTime/(active?.55:.25));
    const loaded=this.data.contacts.filter(c=>c.forceN>=RECOVERY_LIMITS.minimumLoadN
      && (isRecoveryLegSupportSegment(c.segment)||isRecoveryArmSupportSegment(c.segment))
      && !this.released.has(c.segment));
    // Inverse statics supplies the load through joints. The equal/opposite impulses
    // create no external lift; only actual floor reactions can raise the mass.
    const measuredSupports=this.supporting(poses);
    const pushingBrace=this.data.phase==="roll" && this.data.transferStage==="push-brace";
    const plannedBraceSupports=pushingBrace || this.placingProneArms
      ?measuredSupports.filter(contact=>isRecoveryArmSupportSegment(contact.segment)
        || isRecoveryLegSupportSegment(contact.segment)):[];
    const supportsForLoad=active
      ?(pushingBrace || this.placingProneArms ?plannedBraceSupports:measuredSupports)
      :loaded.filter(c=>!isRecoveryFootSegment(c.segment) || rotate(poses.get(c.segment)!.rotation,UP).y>.5);
    const restingTrunk=this.data.contacts.filter(contact=>contact.loadBearing && contact.forceN>0
      && contact.normalY>=RECOVERY_LIMITS.normalY && !this.released.has(contact.segment)
      && (contact.segment==="pelvis" || contact.segment==="lumbar" || contact.segment==="torso"));
    // During prone arm preparation the measured distal patches must receive
    // the planned load so their bounded joint torques can unload the belly.
    // Rapier keeps every trunk contact; excluding it here changes no body pose.
    const supportingEnds=[...new Map([...supportsForLoad,...(this.placingProneArms?[]:restingTrunk)]
      .map(contact=>[contact.segment,contact])).values()];
    const massState=recoveryMassState(poses.values());
    const correction=active ? clampLength(sub(scale(sub(this.rootGoal,poses.get("pelvis")!.position),650),
      scale(massState.velocity,180)),160) : ZERO;
    const pushLoad=this.data.phase==="roll" && this.data.transferStage==="push-brace"
      ?clamp((this.rootGoal.y-poses.get("pelvis")!.position.y)*350-massState.velocity.y*90,0,80):0;
    const verticalLoad=active
      ?((this.data.phase==="kneel" || (this.data.route==="half-kneel" && this.data.phase==="stand"))
        ?WEIGHT_N+clamp((this.rootGoal.y-poses.get("pelvis")!.position.y)*1000-massState.velocity.y*250,0,.30*WEIGHT_N)
        :WEIGHT_N+pushLoad+(this.data.phase==="roll"?0:Math.max(0,this.rootGoal.y-poses.get("pelvis")!.position.y)*1000))
      :WEIGHT_N;
    const plan=planContactLoads(supportingEnds,massState.position,massState.velocity,
      {x:correction.x,y:verticalLoad,z:correction.z},
      {frictionCoefficient:1.2,maxHorizontalForceN:160,
        minimumMeasuredShareFraction:this.placingProneArms?.5:0,
        maxJointTorqueNm:minimumSupportTorqueLimit(supportingEnds)});
    this.data.supportLoads=this.data.contacts.map(contact=>{
      const planned=plan.loads.find(load=>load.segment===contact.segment);
      return {segment:contact.segment,plannedForceN:planned?.plannedForce.y??0,
        measuredForceN:contact.measuredForceN??contact.forceN};
    });
    const descends=(id:SegmentId,ancestor:SegmentId):boolean=>{let current:SegmentId|null=id;while(current){if(current===ancestor)return true;current=SEGMENT_BY_ID.get(current)!.parent;}return false;};
    const commands=new Map<SegmentId,Quat>();
    const anchorHoldingSides=new Set<Side>();
    if(active && this.data.phase==="roll" && (this.placingProneArms || this.data.transferStage==="push-brace"))
      for(const side of SIDES) {
        const forefoot:SegmentId=`${side}Forefoot`;
        const shin:SegmentId=`${side}Shin`;
        const forefootAnchored=this.plants.get(forefoot)?.contact
          && !this.released.has(forefoot) && !this.slippedPlants.has(forefoot);
        const shinAnchored=this.plants.get(shin)?.contact
          && !this.released.has(shin) && !this.slippedPlants.has(shin);
        if(forefootAnchored || shinAnchored)
          anchorHoldingSides.add(side);
      }
    for(const d of SEGMENTS) {
      if(!d.parent||!d.jointProfile)continue;
      const target=limitRecoveryJoint(d.id,targets.get(d.id)??d.restLocalRotation);
      const entry=this.entry.get(d.id)??target;
      const legRole=["thigh","shin","ankle","hindfoot","forefoot"].includes(d.role);
      const trailingBlend=active && this.data.transferStage==="bring-trailing"
        && d.side && d.side!==this.data.leadingSide && legRole?smooth01(this.stageTime/.22):blend;
      const blended=blendRecoveryJointTargets(d.id,entry,target,trailingBlend);
      const armRole=["shoulder-girdle","upper-arm","forearm","forearm-twist","hand"].includes(d.role);
      const moving=!!d.side && ((this.footMovements.has(`${d.side}Foot`) && legRole)
        || (anchorHoldingSides.has(d.side) && legRole)
        || (this.movingArms.has(d.side) && armRole));
      commands.set(d.id,moving?target:blended);
    }
    for(const side of SIDES) for(const arm of [false,true]) {
      const end:SegmentId=arm?`${side}Hand`:`${side}Foot`;
      const moving=active && (this.footMovements.has(end)
        || !arm && anchorHoldingSides.has(side)
        || arm && this.placingProneArms && this.released.has(end));
      const protecting=!active && this.data.phase==="protect" && arm;
      if(!moving && !protecting && !(active && arm))continue;
      const parent=poses.get(arm?"torso":"pelvis")!;
      const geometry=reconstructRecoveryLimb(side,arm,parent,commands);
      const complete=new Map<SegmentId,SegmentPose>(poses);
      for(const pose of geometry.poses)complete.set(pose.id,pose);
      if(arm && active) for(const otherSide of SIDES) if(otherSide!==side)
        for(const pose of reconstructRecoveryLimb(otherSide,true,parent,commands).poses)
          complete.set(pose.id,pose);
      const bodyClearance=arm
        ?limbBodyClearance(complete,side,"arm")
        :limbTrunkClearance(complete,side,"leg");
      if(geometry.floorClearanceM < -.012 || bodyClearance.clearanceM < .004) {
        this.data.blockingPredicate=bodyClearance.clearanceM < .004
          ? "commanded limb-body clearance after blending and limits"
          : "commanded collider clearance after blending and limits";
        for(const pose of geometry.poses) {
          const definition=SEGMENT_BY_ID.get(pose.id)!;
          commands.set(pose.id,limitRecoveryJoint(pose.id,quatMultiply(quatInverse(poses.get(definition.parent!)!.rotation),poses.get(pose.id)!.rotation)));
        }
        const movement=this.footMovements.get(end);if(movement)movement.paused=true;
      }
    }
    if(freeProneLegs) {
      const complete=new Map<SegmentId,SegmentPose>(poses);
      const pelvis=poses.get("pelvis")!;
      const legs=SIDES.map(side=>({side,geometry:reconstructRecoveryLimb(side,false,pelvis,commands)}));
      for(const leg of legs)for(const pose of leg.geometry.poses)complete.set(pose.id,pose);
      if(legs.some(({side,geometry})=>geometry.floorClearanceM<-.003
        || limbBodyClearance(complete,side,"leg").clearanceM<.01)) {
        this.data.blockingPredicate="commanded free-leg clearance after blending and limits";
        for(const side of SIDES)for(const id of recoveryLimbIds(side,false))
          commands.set(id,limitRecoveryJoint(id,this.measuredLocalRotation(id,poses)));
      }
    }
    const motorCommands:JointMotorCommand[]=[];
    for(const d of SEGMENTS) {
      if(!d.parent||!d.jointProfile) continue;
      const child=bodies.get(d.id)!;
      const large=["lumbar","ribcage","thigh","shin"].includes(d.role);
      const distal=["ankle","hindfoot","forefoot"].includes(d.role);
      const spine=["lumbar","ribcage"].includes(d.role);
      const head=["neck","head"].includes(d.role);
      const kp=active||holdCrouch?(spine?3600:large||distal?2400:head?1200:650):(large?35:12);
      const kd=active||holdCrouch?(large?85:distal?38:head?12:18):(large?6:1.6);
      const movingArmStrength=d.side && this.movingArms.has(d.side)
        && ["shoulder-girdle","upper-arm","forearm","forearm-twist","hand"].includes(d.role)
        ?smooth01((this.armTimes.get(d.side)??0)/.22):1;
      let feedforward=ZERO;
      if((active||holdCrouch) && plan.loads.length) {
        const joint=worldPoint(child.translation(),child.rotation(),d.jointProfile.childFrame.anchor);
        for(const load of plan.loads) if(descends(load.segment,d.id)) {
          let fraction=1;
          if(this.placingProneArms) {
            // Only a real, persistent distal patch can receive a planned
            // brace load. Feet as well as hands can press against the floor
            // while the belly unloads. Ramp from the measured reaction.
            const measured=this.data.contacts.find(contact=>contact.segment===load.segment
              && contact.loadBearing && (isRecoveryArmSupportSegment(contact.segment)
                || isRecoveryLegSupportSegment(contact.segment)));
            if(!measured)continue;
            fraction=Math.min(1,(measured.forceN*1.5+30)/Math.max(1,load.plannedForce.y));
          }
          feedforward=add(feedforward,cross(sub(load.point,joint),scale(load.plannedForce,-fraction)));
        }
        if(!this.placingProneArms)for(const [id,plant] of this.plants) if(descends(id,d.id) && !this.released.has(id) && this.data.contacts.some(c=>c.segment===id&&c.loadBearing)) {
          const pose=poses.get(id)!;
          const delta=sub(plant.position,pose.position);
          const spring=clampLength({x:delta.x*120-pose.linearVelocity.x*6,y:0,z:delta.z*120-pose.linearVelocity.z*6},15);
          feedforward=add(feedforward,cross(sub(pose.position,joint),spring));
        }
        if(!this.placingProneArms)for(const descendant of SEGMENTS) if(descends(descendant.id,d.id))
          feedforward=add(feedforward,cross(sub(poses.get(descendant.id)!.position,joint),{x:0,y:descendant.massKg*9.81,z:0}));
      }

      motorCommands.push({id:d.id,targetLocalRotation:commands.get(d.id)??d.restLocalRotation,
        stiffness:kp,damping:kd,strengthScale:(active?1:holdCrouch?.75:.28)*movingArmStrength,
        effortScale:this.placingProneArms && spine?2.2:1,
        feedforwardWorld:feedforward});
    }
    const results=applyCoupledJointMotors(bodies,motorCommands,dt,{passiveResistance:true});
    this.lastMotorResults=results;
    for(const result of results.values())
      this.data.maxMotorTorqueNm=Math.max(this.data.maxMotorTorqueNm,length(result.torqueWorld));
  }
}
