import type { Collider, RigidBody, World } from "@dimforge/rapier3d-compat";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } from "../core/humanoid";
import type { MotionState, Quat, RecoveryDiagnostics, RecoveryPhase, SegmentId, SupportingContact, SegmentPose, Vec3 } from "../core/types";
import { add, angularVelocity, clamp, clampLength, length, lerp, dot, cross, normalize, quatFromTo, smooth01, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
import { applyCoupledJointMotors, type JointMotorCommand } from "./joint-motors";
import { blendRecoveryJointTargets, limitRecoveryJoint, reconstructRecoveryLimb, recoveryJointLimitError, recoveryJointRotation, recoveryLimbIds } from "./recovery-joints";
import { reachableFootTarget, revalidateRecoveryFootTarget, solveRecoveryLegTarget, type RecoveryFootTarget } from "./recovery-foot-targets";
import { acceptableRecoveryArmBraceTarget, isRecoveryArmSupportSegment, isRecoveryFootSegment, isRecoveryLegSupportSegment, RECOVERY_ARM_TARGET_TOLERANCE, recoveryMassState, selectRecoveryContacts, selectRecoveryRoute, supportGeometry, reachableArmBraceTarget, solveRecoveryArmTarget, usableRecoveryArmSupport, type RecoveryArmBraceTarget } from "./recovery-support";
const SIDES = ["left", "right"] as const;
type Side = typeof SIDES[number];
type Stage = RecoveryDiagnostics["transferStage"];
export type RecoveryReleaseResult = "blocked" | "newly-released" | "already-released";
type Plant = { position: Vec3; rotation: Quat; absentS: number; contact?:Vec3 };
type FootMovement={side:Side;origin:Vec3;originRotation:Quat;target:RecoveryFootTarget;time:number;unloaded:boolean;paused:boolean};
const WEIGHT_N = TOTAL_MASS_KG * 9.81;
const FORWARD = { x: 0, y: 0, z: 1 };
/** Frozen acceptance parameters. Persistence always requires consecutive qualifying frames. */
export const RECOVERY_LIMITS = Object.freeze({
  normalY: 0.65, contactDistanceM: 0.012, minimumLoadN: 3,
  loadPersistenceS: 0.05, landingPersistenceS: 0.10, settlePersistenceS: 0.30,
  settleLinearMps: 0.65, settleAngularRadps: 1.8,
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
    maxMotorTorqueNm: 0, supporting: [] };
}

/** Rapier owns every dynamic transform. Plans only become bounded joint impulses. */
export class DynamicRecovery {
  private data = emptyRecoveryDiagnostics();
  private heading = 0;
  private contactAges = new Map<SegmentId, number>();
  private landingTime = 0;
  private noSupportTime = 0;
  private bestError = Infinity;
  private bestStableTime = 0;
  private plants = new Map<SegmentId, Plant>();
  private released = new Set<SegmentId>();
  private releasedUnloaded = new Set<SegmentId>();
  private placements = new Map<SegmentId, Vec3>();
  private footPlans=new Map<Side,RecoveryFootTarget>();
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
    this.heading = heading;
    const local = rotate(quatInverse(quatFromAxisAngle(UP, heading)), direction);
    this.data.orientation = Math.abs(local.x) > Math.abs(local.z) ? (local.x < 0 ? "left" : "right") : (local.z < 0 ? "backward" : "forward");
    this.placingProneArms=false; this.armBraces.clear(); this.armStarts.clear(); this.armTimes.clear();this.movingArms.clear();
    this.shoulderClearanceEstablished=false;
    this.footPlans.clear();
    this.footMovements.clear();
    this.contactAges.clear(); this.plants.clear(); this.placements.clear(); this.released.clear(); this.releasedUnloaded.clear(); this.entry.clear(); this.bend.clear();
    this.landingTime = 0; this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0; this.stageTime = 0; this.transferBlend = 0; this.prospectiveTime=0;
  }

  diagnostics(): RecoveryDiagnostics {
    return { ...this.data, contacts: this.data.contacts.map(c => ({ ...c, point: { ...c.point }, points: c.points?.map(p => ({...p})) })),
      plantedTargets: this.data.plantedTargets.map(p => ({...p,position:{...p.position},rotation:{...p.rotation}})),
      supporting: [...this.data.supporting], releasedSupports: [...this.data.releasedSupports],
      recoveryAxis: this.data.recoveryAxis ? {...this.data.recoveryAxis} : undefined,
      plannedSupportSources: this.data.plannedSupportSources ? [...this.data.plannedSupportSources] : undefined,
      establishedSupportSources: this.data.establishedSupportSources ? [...this.data.establishedSupportSources] : undefined,
      centerOfMass: {...this.data.centerOfMass}, projectedCenterOfMass: {...this.data.projectedCenterOfMass},
      assistanceForce: { ...this.data.assistanceForce }, assistanceTorque: { ...this.data.assistanceTorque } };
  }

  private poses(bodies: Map<SegmentId, RigidBody>): Map<SegmentId, SegmentPose> {
    return new Map([...bodies].map(([id, b]) => [id, { id, massKg: b.mass(), centerOfMass: {...b.worldCom()}, position: {...b.translation()}, rotation: {...b.rotation()},
      linearVelocity: {...b.linvel()}, angularVelocity: {...b.angvel()} }]));
  }

  /** Contact points are copied before another WASM query can reuse its scratch storage. */
  observe(world: World, floor: Collider, colliders: Map<SegmentId, Collider>, bodies: Map<SegmentId, RigidBody>, dt: number): void {
    const contacts: SupportingContact[] = [];
    for (const [segment, collider] of colliders) {
      let normalY = 0, impulse = 0;
      const points: Vec3[] = [];
      if (floor.isEnabled()) world.contactPair(floor, collider, (manifold, flipped) => {
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
      if (points.length) contacts.push({ segment, normalY, forceN, persistenceS: age, points,
        point: scale(points.reduce(add, ZERO), 1 / points.length), loadBearing: loaded && age + 1e-9 >= RECOVERY_LIMITS.loadPersistenceS });
    }
    this.data.contacts = contacts;
    const poses = this.poses(bodies), motion = this.motion(bodies);
    const feet = contacts.filter(c => c.loadBearing && isRecoveryFootSegment(c.segment));
    const supportedCrouch = supportingSideCount(feet, isRecoveryFootSegment) === 2
      && rotate(bodies.get("torso")!.rotation(), UP).y > 0.65 && bodies.get("pelvis")!.translation().y < 0.85;
    const landed = contacts.some(c => !isRecoveryFootSegment(c.segment) && c.forceN >= RECOVERY_LIMITS.minimumLoadN) || supportedCrouch;
    this.landingTime = landed ? this.landingTime + dt : 0;
    const settled = contacts.some(c => c.forceN >= RECOVERY_LIMITS.minimumLoadN) && motion.linear <= RECOVERY_LIMITS.settleLinearMps && motion.angular <= RECOVERY_LIMITS.settleAngularRadps;
    this.data.settledTimeS = settled ? this.data.settledTimeS + dt : 0;
    for (const [id, plant] of this.plants) {
      plant.absentS = contacts.some(c => c.segment === id && c.forceN >= RECOVERY_LIMITS.minimumLoadN) ? 0 : plant.absentS + dt;
      if (plant.absentS > 0.10 || !floor.isEnabled()) this.plants.delete(id);
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
      for(const contact of contactsForSide) if(contact.loadBearing) {
        const pose=poses.get(contact.segment)!;
        this.plants.set(contact.segment,{position:{...pose.position},rotation:{...pose.rotation},absentS:0,contact:{...contact.point}});
      }
      this.movingArms.delete(side);
    }
    for (const c of contacts) {
      if (!c.loadBearing || this.plants.has(c.segment) || this.released.has(c.segment)
        || !(isRecoveryArmSupportSegment(c.segment) || isRecoveryLegSupportSegment(c.segment))) continue;
      const pose = poses.get(c.segment)!;
      this.plants.set(c.segment, { position: {...pose.position}, rotation: {...pose.rotation}, absentS: 0, contact:{...c.point} });
    }
    this.data.plantedTargets = [...this.plants].map(([segment, p]) => ({segment, position:p.position, rotation:p.rotation,
      driftM: length(sub(poses.get(segment)!.position, p.position))}));
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
    this.footPlans.clear();
    const footTargets:Partial<Record<Side,Vec3>>={};
    for(const side of SIDES) {
      const plan=reachableFootTarget(side,poses,this.heading);
      if(plan) {this.footPlans.set(side,plan);if(plan.feasible)footTargets[side]=plan.position;}
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
    return !!target && usableRecoveryArmSupport(side,poses,this.data.contacts.filter(c=>
      isRecoveryArmSupportSegment(c.segment) && !this.released.has(c.segment)))
      && length(sub(poses.get(`${side}Hand`)!.position,target.position))<.14;
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
    return geometry.floorClearanceM>=-RECOVERY_ARM_TARGET_TOLERANCE.maximumFloorPenetrationM
      && length(sub(hand.position,desired))<.02?limited:null;
  }
  private release(ids: SegmentId[], poses: Map<SegmentId,SegmentPose>): RecoveryReleaseResult {
    if (!ids.length) return "blocked";
    // Every authorization uses current measured balance, even after an earlier
    // release or when the requested limb has no current load to remove.
    const geometry = supportGeometry(this.supporting(poses),poses,recoveryMassState(poses.values()),ids);
    if (geometry.marginM < 0 || geometry.polygon.length < 3 || !geometry.supporting.length) return "blocked";
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
    const ready = this.data.phaseTimeS >= RECOVERY_LIMITS.phaseMinimumS;
    const prospective=this.prospectiveSupport("brace","tuck-knee",poses);
    this.prospectiveTime=this.data.transferStage==="push-brace" && prospective.ready ? this.prospectiveTime+dt : 0;
    if (this.data.phase === "protect" && this.landingTime >= RECOVERY_LIMITS.landingPersistenceS) this.enter("settle",bodies);
    else if (this.data.phase === "settle" && this.data.settledTimeS >= RECOVERY_LIMITS.settlePersistenceS) this.chooseRoute(bodies);
    else if(this.data.phase==="roll" && this.placingProneArms && ready && this.movingArms.size===0
      && SIDES.some(side=>this.armPlacementReady(side,poses))) {
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
    const stable = this.data.phase === "stand" && footSideCount===2 && geometry.marginM>=0
      && recoveryStandingPosture(poses, motion);
    this.data.stableTimeS = stable ? this.data.stableTimeS+dt : 0;
    if (this.data.stableTimeS>=RECOVERY_LIMITS.stablePersistenceS) return {state:"recovering",recovered:true};
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
        this.data.blockingPredicate=stable?"standing persistence":"standing posture, contact or speed";
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
      if(!plan || !revalidateRecoveryFootTarget(side,poses,plan)) {
        const next=reachableFootTarget(side,poses,this.heading,this.placements.get(`${side}Foot`));
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
      if(!this.shoulderClearanceEstablished) {
        const grounded=this.supporting(poses).length>0;
        if(!grounded || this.proneShoulderClearance(poses)<PRONE_SHOULDER_CLEARANCE_M)return;
        this.shoulderClearanceEstablished=true;
        this.armBraces.clear();this.armStarts.clear();this.armTimes.clear();
        for(const side of SIDES) {
          const target=reachableArmBraceTarget(side,poses,this.heading);
          if(target && acceptableRecoveryArmBraceTarget(target))this.armBraces.set(side,target);
          this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});this.armTimes.set(side,0);
        }
      }
      const first=this.data.rollSide!,second:Side=first==="left"?"right":"left";
      const alreadyMoving=[...this.movingArms][0];
      const placementOrder:Side[]=alreadyMoving?[alreadyMoving]:[first,second].sort((a,b)=>(
        this.armBraces.get(a)?.movementM??Infinity)-(this.armBraces.get(b)?.movementM??Infinity)
      );
      for(const side of placementOrder) {
        if(this.armPlacementReady(side,poses)) continue;
        let target=this.armBraces.get(side);
        if(!target || !this.legalArmCommand(side,target.position,poses)) {
          const next=reachableArmBraceTarget(side,poses,this.heading);
          if(next && acceptableRecoveryArmBraceTarget(next)) {this.armBraces.set(side,next);target=next;}
          else {this.armBraces.delete(side);this.data.blockingPredicate="legal reachable arm target";continue;}
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
      const riseBlend=phase==="kneel"?smooth01((pelvis.position.y-.65)/.13):1;
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
      const wristLocal=brace?.wristRotation??recoveryJointRotation(hand,ZERO);
      const bend=plantedBend??brace?.trajectoryBend??rotate(girdleWorld,this.bend.get(upper)??{x:0,y:0,z:-1});
      const solved=solveRecoveryArmTarget(shoulder,desired,bend,wristLocal,this.heading);
      const rawUpper=quatMultiply(quatInverse(girdleWorld),solved.upperRotation);
      const rawForearm=quatMultiply(quatInverse(solved.upperRotation),solved.forearmRotation);
      const rawTwist=quatMultiply(quatInverse(solved.forearmRotation),solved.forearmTwistRotation);
      const rawHand=quatMultiply(quatInverse(solved.forearmTwistRotation),solved.handRotation);
      targets.set(girdle,girdleLocal);targets.set(upper,limitRecoveryJoint(upper,rawUpper));
      targets.set(forearm,limitRecoveryJoint(forearm,rawForearm));
      targets.set(twist,limitRecoveryJoint(twist,rawTwist));targets.set(hand,limitRecoveryJoint(hand,rawHand));
      return;
    }

    const [thigh,shin,ankle,foot,forefoot]=ids;
    const plan=this.footPlans.get(side),shinPlant=this.plants.get(shin);
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
    // Settling is an evidence-gathering phase, not a posture target. Continuing
    // the protective pose motors here pumps energy into floor contacts and can
    // prevent the measured low-motion window from ever becoming consecutive.
    // Structural joint limits and passive tissue damping are applied by the
    // owning character before this call, so the assembly remains constrained.
    if(this.data.phase==="settle")return;
    const targets=new Map<SegmentId,Quat>();
    const holdCrouch=!active && poses.get("pelvis")!.position.y>.40 && poses.get("pelvis")!.position.y<.85
      && rotate(poses.get("torso")!.rotation,UP).y>.65 && SIDES.some(side=>poses.get(`${side}Foot`)!.position.y<.13);
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
        if(holdingProneBase) for(const id of recoveryLimbIds(side,false))
          targets.set(id,this.entry.get(id)??SEGMENT_BY_ID.get(id)!.restLocalRotation);
        else this.limbTargets(side,false,poses,targets);
        if(this.data.route==="half-kneel" && (this.data.phase==="kneel" || (this.data.phase==="stand" && this.data.transferStage!=="relax"))) {
          for(const id of recoveryLimbIds(side,true))
            targets.set(id,this.entry.get(id)??SEGMENT_BY_ID.get(id)!.restLocalRotation);
        } else this.limbTargets(side,true,poses,targets);
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
    const plannedBraceSupports=pushingBrace
      ?measuredSupports.filter(contact=>isRecoveryArmSupportSegment(contact.segment)
        || isRecoveryLegSupportSegment(contact.segment)):[];
    const supportsForLoad=active
      ?(pushingBrace?plannedBraceSupports:measuredSupports)
      :loaded.filter(c=>!isRecoveryFootSegment(c.segment) || rotate(poses.get(c.segment)!.rotation,UP).y>.5);
    const supportingEnds=supportsForLoad;
    const massState=recoveryMassState(poses.values());
    const pressurePoints=supportingEnds.map(contact=>({...contact.point}));
    const totalContactLoad=supportsForLoad.reduce((sum,c)=>sum+c.forceN,0);
    let shares=supportingEnds.map(c=>active ? c.forceN/Math.max(1,totalContactLoad) : 1/Math.max(1,supportingEnds.length));
    const targetMass=add(massState.position,scale(massState.velocity,.15));
    for(let iteration=0;iteration<(active?8:32) && shares.length>1;iteration++) {
      const center=pressurePoints.reduce((sum,p,i)=>add(sum,scale(p,shares[i])),ZERO);
      const error={x:center.x-targetMass.x,y:0,z:center.z-targetMass.z};
      const average=pressurePoints.reduce(add,ZERO);
      shares=shares.map((w,i)=>Math.max(0,w-1.5*dot(sub(pressurePoints[i],scale(average,1/shares.length)),error)));
      const total=shares.reduce((a,b)=>a+b,0); shares=shares.map(w=>w/Math.max(total,1e-9));
    }
    const descends=(id:SegmentId,ancestor:SegmentId):boolean=>{let current:SegmentId|null=id;while(current){if(current===ancestor)return true;current=SEGMENT_BY_ID.get(current)!.parent;}return false;};
    const commands=new Map<SegmentId,Quat>();
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
        || (this.movingArms.has(d.side) && armRole));
      commands.set(d.id,moving?target:blended);
    }
    if(active) for(const side of SIDES) for(const arm of [false,true]) {
      const end:SegmentId=arm?`${side}Hand`:`${side}Foot`;
      if(!this.footMovements.has(end) && !(arm && this.placingProneArms && this.released.has(end)))continue;
      const parent=poses.get(arm?"torso":"pelvis")!;
      const geometry=reconstructRecoveryLimb(side,arm,parent,commands);
      if(geometry.floorClearanceM < -.012) {
        this.data.blockingPredicate="commanded collider clearance after blending and limits";
        for(const pose of geometry.poses) {
          const definition=SEGMENT_BY_ID.get(pose.id)!;
          commands.set(pose.id,limitRecoveryJoint(pose.id,quatMultiply(quatInverse(poses.get(definition.parent!)!.rotation),poses.get(pose.id)!.rotation)));
        }
        const movement=this.footMovements.get(end);if(movement)movement.paused=true;
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
      if((active||holdCrouch) && !this.placingProneArms && supportingEnds.length) {
        const joint=worldPoint(child.translation(),child.rotation(),d.jointProfile.childFrame.anchor);
        for(const [contactIndex,contact] of supportingEnds.entries()) if(descends(contact.segment,d.id)) {
          const point=pressurePoints[contactIndex];
          const correction=active ? clampLength(sub(scale(sub(this.rootGoal,poses.get("pelvis")!.position),650),scale(massState.velocity,180)),160) : ZERO;
          const pushLoad=this.data.phase==="roll" && this.data.transferStage==="push-brace"
            ?clamp((this.rootGoal.y-poses.get("pelvis")!.position.y)*350-massState.velocity.y*90,0,80):0;
          const verticalLoad=active
            ?((this.data.phase==="kneel" || (this.data.route==="half-kneel" && this.data.phase==="stand"))
              ?.8*WEIGHT_N+clamp((this.rootGoal.y-poses.get("pelvis")!.position.y)*250-massState.velocity.y*100,0,30)
              :WEIGHT_N+pushLoad+(this.data.phase==="roll"?0:Math.max(0,this.rootGoal.y-poses.get("pelvis")!.position.y)*1000))
            :WEIGHT_N;
          feedforward=add(feedforward,cross(sub(point,joint),{
            x:-correction.x*shares[contactIndex],y:-verticalLoad*shares[contactIndex],z:-correction.z*shares[contactIndex],
          }));
        }
        for(const [id,plant] of this.plants) if(descends(id,d.id) && !this.released.has(id) && this.data.contacts.some(c=>c.segment===id&&c.loadBearing)) {
          const pose=poses.get(id)!;
          const delta=sub(plant.position,pose.position);
          const spring=clampLength({x:delta.x*120-pose.linearVelocity.x*6,y:0,z:delta.z*120-pose.linearVelocity.z*6},15);
          feedforward=add(feedforward,cross(sub(pose.position,joint),spring));
        }
        for(const descendant of SEGMENTS) if(descends(descendant.id,d.id))
          feedforward=add(feedforward,cross(sub(poses.get(descendant.id)!.position,joint),{x:0,y:descendant.massKg*9.81,z:0}));
      }

      motorCommands.push({id:d.id,targetLocalRotation:commands.get(d.id)??d.restLocalRotation,
        stiffness:kp,damping:kd,strengthScale:(active?1:holdCrouch?.75:.28)*movingArmStrength,
        effortScale:this.placingProneArms && spine?(this.shoulderClearanceEstablished?1:2.2):1,
        feedforwardWorld:feedforward});
    }
    const results=applyCoupledJointMotors(bodies,motorCommands,dt);
    for(const result of results.values())
      this.data.maxMotorTorqueNm=Math.max(this.data.maxMotorTorqueNm,length(result.torqueWorld));
  }
}
