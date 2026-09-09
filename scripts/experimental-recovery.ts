import type { Collider, RigidBody, World } from "@dimforge/rapier3d-compat";
import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } from "../src/core/humanoid";
import type { MotionState, Quat, RecoveryDiagnostics, RecoveryPhase, SegmentId, SupportingContact, SegmentPose, Vec3 } from "../src/core/types";
import { add, angularVelocity, clamp, clampLength, length, lerp, dot, cross, normalize, quatFromTo, smooth01, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "../src/character/math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
import { solveTwoBone } from "../src/character/pose";
import { recoveryMassState, selectRecoveryRoute, supportGeometry, usableRecoveryArmSupport, reachableArmBraceTarget, solveRecoveryArmTarget, type RecoveryArmBraceTarget } from "../src/character/recovery-support";
import { solveRecoveryMotorTorques, type RecoveryMotorIntent } from "../src/character/recovery-motors";
const SIDES = ["left", "right"] as const;
type Side = typeof SIDES[number];
type Stage = RecoveryDiagnostics["transferStage"];
type Plant = { position: Vec3; rotation: Quat; absentS: number };
const WEIGHT_N = TOTAL_MASS_KG * 9.81;
const FORWARD = { x: 0, y: 0, z: 1 };
/** Frozen acceptance parameters. Persistence always requires consecutive qualifying frames. */
export const RECOVERY_LIMITS = Object.freeze({
  normalY: 0.65, contactDistanceM: 0.012, minimumLoadN: 3,
  loadPersistenceS: 0.05, landingPersistenceS: 0.10, settlePersistenceS: 0.30,
  settleLinearMps: 0.65, settleAngularRadps: 1.8,
  stablePersistenceS: 0.55, stableLinearMps: 0.22, stableAngularRadps: 0.65,
  stableUpDot: 0.97, phaseMinimumS: 0.20, stallS: 3.0, supportLossS: 0.20,
  assistanceForceN: 950, assistanceTorqueNm: 60,
});

const ELIGIBLE: Record<RecoveryPhase, ReadonlyArray<SegmentId>> = {
  none: [], protect: [], settle: [],
  roll: ["torso", "pelvis", "leftForearm", "rightForearm", "leftUpperArm", "rightUpperArm", "leftThigh", "rightThigh", "leftShin", "rightShin", "leftHand", "rightHand", "leftFoot", "rightFoot"],
  brace: ["leftHand", "rightHand", "leftForearm", "rightForearm", "leftShin", "rightShin", "leftFoot", "rightFoot"],
  kneel: ["leftShin", "rightShin", "leftFoot", "rightFoot", "leftHand", "rightHand"],
  stand: ["leftFoot", "rightFoot"],
};

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
export interface ProneExperimentParameters { rootTilt:number; spineTilt:number; rootHeight:number; armLoad:number; kneeBend:number; kneeHip:number; backShift:number }
export class DynamicRecovery {
  constructor(private experiment:ProneExperimentParameters={rootTilt:1.1,spineTilt:-.3,rootHeight:.35,armLoad:.22,kneeBend:2.1,kneeHip:-.15,backShift:.12}){}
  private kneeFlexing=false;
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
  private entry = new Map<SegmentId, Quat>();
  private bend = new Map<SegmentId, Vec3>();
  private stageTime = 0;
  private rootGoal: Vec3 = ZERO;
  private rootRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
  private rollTurned = false;
  private placingProneArms = false;
  private pressingProneChest = false;
  private placementRootRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
  private placementTorsoRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
  private armBraces = new Map<Side, RecoveryArmBraceTarget>();
  private armStarts = new Map<Side, Vec3>();
  private armTimes = new Map<Side, number>();

  reset(heading: number, direction: Vec3): void {
    this.data = emptyRecoveryDiagnostics(); this.data.phase = "protect";
    this.heading = heading;
    const local = rotate(quatInverse(quatFromAxisAngle(UP, heading)), direction);
    this.data.orientation = Math.abs(local.x) > Math.abs(local.z) ? (local.x < 0 ? "left" : "right") : (local.z < 0 ? "backward" : "forward");
    this.contactAges.clear(); this.plants.clear(); this.placements.clear(); this.released.clear(); this.releasedUnloaded.clear(); this.entry.clear(); this.bend.clear();
    this.landingTime = 0; this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0; this.stageTime = 0;
  }

  diagnostics(): RecoveryDiagnostics {
    return { ...this.data, contacts: this.data.contacts.map(c => ({ ...c, point: { ...c.point }, points: c.points?.map(p => ({...p})) })),
      plantedTargets: this.data.plantedTargets.map(p => ({...p,position:{...p.position},rotation:{...p.rotation}})),
      supporting: [...this.data.supporting], releasedSupports: [...this.data.releasedSupports],
      centerOfMass: {...this.data.centerOfMass}, projectedCenterOfMass: {...this.data.projectedCenterOfMass},
      assistanceForce: { ...this.data.assistanceForce }, assistanceTorque: { ...this.data.assistanceTorque } };
  }

  private poses(bodies: Map<SegmentId, RigidBody>): Map<SegmentId, SegmentPose> {
    return new Map([...bodies].map(([id, b]) => [id, { id, position: {...b.translation()}, rotation: {...b.rotation()},
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
    const feet = contacts.filter(c => c.loadBearing && c.segment.endsWith("Foot"));
    const supportedCrouch = feet.length === 2 && rotate(bodies.get("torso")!.rotation(), UP).y > 0.65 && bodies.get("pelvis")!.translation().y < 0.85;
    const landed = contacts.some(c => !c.segment.endsWith("Foot") && c.forceN >= RECOVERY_LIMITS.minimumLoadN) || supportedCrouch;
    this.landingTime = landed ? this.landingTime + dt : 0;
    const settled = contacts.some(c => c.forceN >= RECOVERY_LIMITS.minimumLoadN) && motion.linear <= RECOVERY_LIMITS.settleLinearMps && motion.angular <= RECOVERY_LIMITS.settleAngularRadps;
    this.data.settledTimeS = settled ? this.data.settledTimeS + dt : 0;
    for (const [id, plant] of this.plants) {
      plant.absentS = contacts.some(c => c.segment === id && c.forceN >= RECOVERY_LIMITS.minimumLoadN) ? 0 : plant.absentS + dt;
      if (plant.absentS > 0.10 || !floor.isEnabled()) this.plants.delete(id);
    }
    for (const id of this.released) {
      const contact=contacts.find(c=>c.segment===id);
      if(!contact || contact.forceN<RECOVERY_LIMITS.minimumLoadN) this.releasedUnloaded.add(id);
      if(this.releasedUnloaded.has(id) && contact?.loadBearing) {this.released.delete(id);this.releasedUnloaded.delete(id);}
    }
    for (const c of contacts) {
      if (!c.loadBearing || this.plants.has(c.segment) || this.released.has(c.segment) || !/Hand|Forearm|Shin|Foot/.test(c.segment)) continue;
      const pose = poses.get(c.segment)!;
      this.plants.set(c.segment, { position: {...pose.position}, rotation: {...pose.rotation}, absentS: 0 });
    }
    this.data.plantedTargets = [...this.plants].map(([segment, p]) => ({segment, position:p.position, rotation:p.rotation,
      driftM: length(sub(poses.get(segment)!.position, p.position))}));
    const mass = recoveryMassState(poses.values()), geometry = supportGeometry(this.supporting(), poses, mass);
    this.data.centerOfMass = mass.position; this.data.projectedCenterOfMass = geometry.projectedCenterOfMass;
    this.data.supportMarginM = geometry.marginM;
    this.data.supporting = this.supporting().map(c => c.segment);
    if (!this.data.supporting.length) { this.data.assistanceForce = ZERO; this.data.assistanceTorque = ZERO; }
  }

  private supporting(): SupportingContact[] {
    return this.data.contacts.filter(c => c.loadBearing && !this.released.has(c.segment) && ELIGIBLE[this.data.phase].includes(c.segment));
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
    for (const d of SEGMENTS) if (d.parent) this.entry.set(d.id, quatMultiply(quatInverse(bodies.get(d.parent)!.rotation()), bodies.get(d.id)!.rotation()));
    this.stageTime = 0;
  }
  private stage(stage: Stage, bodies: Map<SegmentId, RigidBody>): void {
    if (stage === this.data.transferStage) return;
    this.data.transferStage = stage; this.captureEntry(bodies); this.bestError = Infinity; this.data.stalledTimeS = 0;
  }
  private enter(phase: RecoveryPhase, bodies: Map<SegmentId, RigidBody>): void {
    this.data.phase = phase; this.data.phaseTimeS = 0; this.data.stalledTimeS = 0; this.data.stableTimeS = 0;
    this.noSupportTime = 0; this.bestError = Infinity; this.bestStableTime = 0; this.captureEntry(bodies);
    if (phase === "settle") { this.data.transferStage = "none"; this.data.route = "none"; this.released.clear(); }
  }
  private chooseRoute(bodies: Map<SegmentId, RigidBody>): void {
    const poses = this.poses(bodies), pelvis = poses.get("pelvis")!;
    const forward = rotate(poses.get("torso")!.rotation, FORWARD), right = rotate(poses.get("torso")!.rotation, {x:1,y:0,z:0});
    this.data.orientation = Math.abs(right.y) > Math.abs(forward.y) ? (right.y > 0 ? "left" : "right") : (forward.y > 0 ? "backward" : "forward");
    this.rootGoal = {...pelvis.position}; this.placements.clear(); this.released.clear(); this.rollTurned = false;
    const yaw = quatFromAxisAngle(UP, this.heading);
    const footTargets = {left: add(pelvis.position, rotate(yaw,{x:-0.12,y:0,z:0.10})), right: add(pelvis.position,rotate(yaw,{x:0.12,y:0,z:0.10}))};
    for (const side of SIDES) {
      const foot: SegmentId = `${side}Foot`, hand: SegmentId = `${side}Hand`;
      this.placements.set(foot, {...footTargets[side],y:0.049});
      const shoulder = worldPoint(poses.get("torso")!.position, poses.get("torso")!.rotation, SEGMENT_BY_ID.get(`${side}UpperArm`)!.jointAnchorParent!);
      const measuredHand = poses.get(hand)!.position;
      this.placements.set(hand, {x: measuredHand.x * 0.5 + shoulder.x * 0.5, y:0.066,z:measuredHand.z * 0.5 + shoulder.z * 0.5});
      for (const [upper,lower,end,isArm] of [[`${side}Thigh`,`${side}Shin`,foot,false],[`${side}UpperArm`,`${side}Forearm`,hand,true]] as const) {
        const upperPose = poses.get(upper)!, lowerPose = poses.get(lower)!, endPose = poses.get(end)!;
        const start = worldPoint(upperPose.position,upperPose.rotation,SEGMENT_BY_ID.get(upper)!.jointAnchorChild!);
        const middle = worldPoint(lowerPose.position,lowerPose.rotation,SEGMENT_BY_ID.get(lower)!.jointAnchorChild!);
        const finish = worldPoint(endPose.position,endPose.rotation,SEGMENT_BY_ID.get(end)!.jointAnchorChild!);
        const line = normalize(sub(finish,start));
        const measured = sub(sub(middle,start),scale(line,dot(sub(middle,start),line)));
        const parentRotation=poses.get(isArm?"torso":"pelvis")!.rotation;
        const localMeasured=rotate(quatInverse(parentRotation),measured);
        const natural={x:0,y:0,z:isArm?-1:1};
        this.bend.set(upper,length(localMeasured)>.03 && dot(localMeasured,natural)>0 ? normalize(add(scale(normalize(localMeasured),.2),natural)) : natural);
      }
    }
    const selected = selectRecoveryRoute({poses,contacts:this.data.contacts,footTargets});
    this.data.route = selected.route; this.data.leadingSide = selected.leadingSide; this.data.rollSide = selected.rollSide;
    this.placingProneArms = selected.route === "prone" && !SIDES.every(side=>usableRecoveryArmSupport(side,poses,this.data.contacts.filter(c=>c.segment.endsWith("Hand"))));
    this.pressingProneChest = selected.route === "prone"; this.kneeFlexing=false;
    this.placementRootRotation = {...pelvis.rotation};
    this.placementTorsoRotation = quatMultiply(quatInverse(pelvis.rotation),poses.get("torso")!.rotation);
    this.armBraces.clear(); this.armStarts.clear(); this.armTimes.clear();
    for(const side of SIDES) {
      const target = reachableArmBraceTarget(side,poses,this.heading);
      if(target?.floorReachable) this.armBraces.set(side,target);
      this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});
    }
    const up = rotate(poses.get("torso")!.rotation,UP).y;
    const lower = this.data.contacts.some(c=>c.loadBearing && /Shin|Foot/.test(c.segment));
    const braceReady = this.data.contacts.some(c=>c.loadBearing && /Hand|Forearm|Shin|Foot/.test(c.segment)) && up>.25 && pelvis.position.y>.28;
    if (this.placingProneArms) { this.enter("roll",bodies); this.stage("roll",bodies); }
    else if (lower && up>.65 && pelvis.position.y>.38) { this.enter("kneel",bodies); this.stage(selected.route === "crouch" ? "extend":"shift-weight",bodies); }
    else if (braceReady || (selected.route==="prone" && this.data.contacts.some(c=>c.loadBearing&&/Hand|Forearm/.test(c.segment)) && supportGeometry(this.data.contacts,poses,recoveryMassState(poses.values())).marginM>=0)) { this.enter("brace",bodies); this.stage("tuck-knee",bodies); }
    else { this.enter("roll",bodies); this.stage("roll",bodies); }
  }

  private armPlacementReady(side:Side,poses:Map<SegmentId,SegmentPose>):boolean {
    const target=this.armBraces.get(side);
    return !!target && usableRecoveryArmSupport(side,poses,this.data.contacts.filter(c=>c.segment.endsWith("Hand") && !this.released.has(c.segment)))
      && length(sub(poses.get(`${side}Hand`)!.position,target.position))<.14;
  }
  private release(ids: SegmentId[], poses: Map<SegmentId,SegmentPose>): boolean {
    if(ids.every(id=>this.released.has(id))) return false;
    const loaded = ids.filter(id => !this.released.has(id) && this.data.contacts.some(c=>c.segment===id && c.loadBearing));
    if (loaded.length || ids.some(id=>this.plants.has(id))) {
      const geometry = supportGeometry(this.supporting(),poses,recoveryMassState(poses.values()),ids);
      if (geometry.marginM < 0 || !geometry.supporting.length) return false;
      this.data.releaseMarginM = geometry.marginM; this.data.releasedSupports.push(...loaded);
    }
    for (const id of ids) { if(this.released.has(id)) continue; this.plants.delete(id); this.released.add(id); this.releasedUnloaded.delete(id); }
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
    const feet = this.data.contacts.filter(c=>c.loadBearing && !this.released.has(c.segment) && c.segment.endsWith("Foot") && rotate(bodies.get(c.segment)!.rotation(),UP).y>.85);
    const shins = this.data.contacts.filter(c=>c.loadBearing && !this.released.has(c.segment) && c.segment.endsWith("Shin"));
    const hands = this.data.contacts.filter(c=>c.loadBearing && !this.released.has(c.segment) && /Hand|Forearm/.test(c.segment));
    const ready = this.data.phaseTimeS >= RECOVERY_LIMITS.phaseMinimumS;
    if (this.data.phase === "protect" && this.landingTime >= RECOVERY_LIMITS.landingPersistenceS) this.enter("settle",bodies);
    else if (this.data.phase === "settle" && this.data.settledTimeS >= RECOVERY_LIMITS.settlePersistenceS) this.chooseRoute(bodies);
    else if (this.data.phase === "roll" && ready && ((this.placingProneArms && SIDES.every(side=>usableRecoveryArmSupport(side,poses,this.data.contacts.filter(c=>c.segment.endsWith("Hand")&&!this.released.has(c.segment))) && length(sub(poses.get(`${side}Hand`)!.position,this.armBraces.get(side)!.position))<.14)) || (!this.placingProneArms && (hands.length || shins.length || feet.length) && up>.25 && pelvis.translation().y>.28))) {
      this.release(["leftForearm","rightForearm"],poses);
      this.placingProneArms = false;
      this.enter("brace",bodies); this.stage("tuck-knee",bodies);
    } else if (this.data.phase === "brace" && ready && (shins.length || feet.length) && up>.65 && pelvis.translation().y>.38) {
      this.enter("kneel",bodies); this.stage(feet.length ? "shift-weight":"plant-lead",bodies);
    } else if (this.data.phase === "kneel" && ready && feet.length===2 && up>.88 && pelvis.translation().y>.55) {
      this.enter("stand",bodies); this.stage("extend",bodies);
    }
    let active = ["roll","brace","kneel","stand"].includes(this.data.phase);
    const supports = this.supporting(); this.data.supporting = supports.map(c=>c.segment);
    const geometry = supportGeometry(supports,poses,mass);
    this.data.supportMarginM = geometry.marginM;
    this.data.centerOfMass = mass.position; this.data.projectedCenterOfMass = geometry.projectedCenterOfMass;
    const stable = this.data.phase === "stand" && feet.length===2 && feet.every(c=>rotate(bodies.get(c.segment)!.rotation(),UP).y>.97)
      && up>=RECOVERY_LIMITS.stableUpDot && rotate(pelvis.rotation(),UP).y>=RECOVERY_LIMITS.stableUpDot
      && pelvis.translation().y>.93 && motion.linear<=RECOVERY_LIMITS.stableLinearMps && motion.angular<=RECOVERY_LIMITS.stableAngularRadps;
    this.data.stableTimeS = stable ? this.data.stableTimeS+dt : 0;
    if (this.data.stableTimeS>=RECOVERY_LIMITS.stablePersistenceS) return {state:"recovering",recovered:true};
    if (active) {
      this.transfer(bodies,poses,dt);
      this.noSupportTime = supports.length ? 0 : this.noSupportTime+dt;
      const targetHeight = this.data.phase==="roll" ? .32 : this.data.phase==="brace" ? .50 : this.data.phase==="kneel" ? .76 : .99;
      const lead = `${this.data.leadingSide}Foot` as SegmentId;
      const placementError = this.placements.has(lead) && !feet.some(c=>c.segment===lead) ? length(sub(poses.get(lead)!.position,this.placements.get(lead)!))*.2:0;
      const error = this.placingProneArms?SIDES.reduce((sum,side)=>sum+length(sub(poses.get(`${side}Hand`)!.position,this.armBraces.get(side)?.position??poses.get(`${side}Hand`)!.position)),0):Math.abs(targetHeight-pelvis.translation().y)+Math.max(0,1-up)*.3+placementError;
      const stabilityProgress = this.data.stableTimeS>this.bestStableTime+1e-9;
      this.bestStableTime = Math.max(this.bestStableTime,this.data.stableTimeS);
      if (error<this.bestError-.015 || stabilityProgress) { this.bestError=error; this.data.stalledTimeS=0; } else this.data.stalledTimeS+=dt;
      this.data.progressError = error;
      if (this.noSupportTime>RECOVERY_LIMITS.supportLossS || this.data.stalledTimeS>RECOVERY_LIMITS.stallS) {
        this.data.retries++; this.enter("settle",bodies); this.data.settledTimeS=0; active=false;
      }
    }
    this.data.noSupportTimeS=this.noSupportTime;
    this.actuate(bodies,poses,active,dt);
    if (active && supports.length) this.assist(bodies,poses,dt);
    const phase=this.data.phase;
    return {state:active?"recovering":phase==="settle"?"fallen":"falling",recovered:false};
  }

  private transfer(bodies: Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>,dt:number): void {
    const phase=this.data.phase, lead=this.data.leadingSide!, trail:Side=lead==="left"?"right":"left";
    const leadFoot:SegmentId=`${lead}Foot`, trailingFoot:SegmentId=`${trail}Foot`;
    const loaded=(id:SegmentId)=>!this.released.has(id) && this.data.contacts.some(c=>c.segment===id && c.loadBearing) && (!id.endsWith("Foot") || rotate(poses.get(id)!.rotation,UP).y>.85);
    const pelvis=poses.get("pelvis")!, yaw=quatFromAxisAngle(UP,this.heading), mass=recoveryMassState(poses.values());
    const geometry=supportGeometry(this.pressingProneChest?this.data.contacts:this.supporting(),poses,mass);
    let center=geometry.center;
    let height=pelvis.position.y;
    let tilt=.0;
    if(phase==="roll" && this.placingProneArms) {
      this.rootGoal={...pelvis.position};this.rootRotation=this.placementRootRotation;
      const first=this.data.rollSide!,second:Side=first==="left"?"right":"left";
      for(const side of [first,second]) {
        if(this.armPlacementReady(side,poses)) continue;
        const target=this.armBraces.get(side); if(!target)continue;
        const moving=this.released.has(`${side}Hand`);
        if(this.release([`${side}Hand`,`${side}Forearm`],poses) || (this.released.has(`${side}Hand`) && this.released.has(`${side}Forearm`))) {
          if(!moving) {this.armStarts.set(side,{...poses.get(`${side}Hand`)!.position});this.armTimes.set(side,0);}
          const t=(this.armTimes.get(side)??0)+dt;this.armTimes.set(side,t);
          const start=this.armStarts.get(side)??poses.get(`${side}Hand`)!.position;
          const p=lerp(start,target.position,smooth01(t/.8));
          const lift=.15*smooth01(t/.25)*(1-smooth01((t-.8)/.35));
          this.placements.set(`${side}Hand`,{...p,y:p.y+lift-.006*smooth01((t-.8)/.35)});
          this.bend.set(`${side}UpperArm`,rotate(quatInverse(poses.get("torso")!.rotation),target.bend));
        }
      }
      return;
    }    if(phase==="roll") {
      const forward=rotate(pelvis.rotation,FORWARD);
      this.rollTurned ||= forward.y<-.45;
      tilt=.90;
      this.rootRotation=quatMultiply(yaw,rotation(tilt,0,this.rollTurned||this.data.route==="prone"?0:(this.data.rollSide==="left"?1:-1)*1.25));
      if(this.data.route==="prone" || this.rollTurned) {
        for(const side of SIDES) {
          const foot:SegmentId=`${side}Foot`;
          if(rotate(poses.get(foot)!.rotation,UP).y<.5 && this.release([foot,`${side}Shin`],poses)) {
            this.placements.set(foot,{...add(pelvis.position,rotate(yaw,{x:(side==="left"?-1:1)*.14,y:0,z:side===lead?.10:-.36})),y:.05});
          }
        }
      }
    } else {
      if(phase==="brace") {
                if(this.pressingProneChest && this.data.transferStage==="tuck-knee") {
          if(this.release([`${trail}Foot`,`${trail}Shin`],poses) || (this.released.has(`${trail}Foot`)&&this.released.has(`${trail}Shin`))) this.kneeFlexing=true;
          if(loaded(`${trail}Shin`) && this.stageTime>.25) {this.stage("plant-lead",bodies);this.kneeFlexing=false;}
        }
        if(this.pressingProneChest && this.data.transferStage==="plant-lead") {
          const shin=this.plants.get(`${trail}Shin`);
          if(shin) center=add(worldPoint(shin.position,shin.rotation,SEGMENT_BY_ID.get(`${trail}Shin`)!.jointAnchorChild!),rotate(yaw,{x:0,y:0,z:this.experiment.backShift}));
          if(this.release([leadFoot,`${lead}Shin`],poses)) this.placements.set(leadFoot,{...add(pelvis.position,rotate(yaw,{x:(lead==="left"?-1:1)*.13,y:0,z:.12})),y:.049});
        }
        height=this.pressingProneChest ? Math.min(this.experiment.rootHeight,pelvis.position.y+.05) : .52;
        tilt=this.pressingProneChest ? this.experiment.rootTilt : .48;
        if(!this.pressingProneChest && this.data.transferStage!=="plant-lead") {
          const foot:SegmentId=`${trail}Foot`;
          if(this.release([foot,`${trail}Shin`],poses)) this.placements.set(foot,{...add(pelvis.position,rotate(yaw,{x:(trail==="left"?-1:1)*.16,y:0,z:-.36})),y:.066});
        }
      }
      else if(phase==="kneel") { height=loaded(leadFoot)?.80:.58; tilt=.18; }
      else { height=HUMAN_PROPORTIONS.pelvis.centerHeightM; tilt=0; }
      const feet=this.data.contacts.filter(c=>c.segment.endsWith("Foot") && loaded(c.segment));
      if (feet.length===2) {
        const ankles=feet.map(c=>{const p=this.plants.get(c.segment)??poses.get(c.segment)!;return worldPoint(p.position,p.rotation,SEGMENT_BY_ID.get(c.segment)!.jointAnchorChild!);});
        center=scale(ankles.reduce(add,ZERO),.5);
      } else if(loaded(leadFoot)) {
        const foot=this.plants.get(leadFoot)??poses.get(leadFoot)!;
        const ankle=worldPoint(foot.position,foot.rotation,SEGMENT_BY_ID.get(leadFoot)!.jointAnchorChild!);
        center=ankle;
      }
      // Shift the virtual root so the whole mass, including the bent torso, moves over support.
      const correction=clampLength({x:center.x-mass.position.x,y:0,z:center.z-mass.position.z},.16);
      const measured=pelvis.position;
      this.rootGoal={x:measured.x+correction.x,y:Math.min(height,measured.y+.10),z:measured.z+correction.z};
      const desiredRoot=quatMultiply(yaw,pitch(tilt));
      const rootDelta=angularVelocity(this.placementRootRotation,desiredRoot,1);
      this.rootRotation=this.pressingProneChest?quatMultiply(quatFromAxisAngle(normalize(rootDelta),length(rootDelta)*smooth01(this.data.phaseTimeS/1.5)),this.placementRootRotation):desiredRoot;
      for(const side of SIDES) {
        const shin:SegmentId=`${side}Shin`, plant=this.plants.get(shin);
        if(!plant || this.released.has(shin)) continue;
        const knee=worldPoint(plant.position,plant.rotation,SEGMENT_BY_ID.get(shin)!.jointAnchorChild!);
        const anchor=SEGMENT_BY_ID.get(`${side}Thigh`)!.jointAnchorParent!;
        const hip=worldPoint(this.rootGoal,this.rootRotation,anchor),delta=sub(hip,knee);
        if(length(delta)>.418) this.rootGoal=sub(add(knee,scale(normalize(delta),.418)),rotate(this.rootRotation,anchor));
      }
      if(phase==="brace" && loaded(`${trail}Shin`)) this.stage("plant-lead",bodies);
      if(phase==="kneel" && loaded(leadFoot) && this.data.transferStage!=="extend") {
        if(!loaded(trailingFoot)) {
          if(this.data.transferStage!=="bring-trailing") this.stage("shift-weight",bodies);
          const toe=this.data.contacts.find(c=>c.segment===trailingFoot && c.loadBearing && !this.released.has(trailingFoot));
          if(toe && this.plants.has(`${trail}Shin`)) this.release([`${trail}Shin`],poses);
          const leadLoad=this.data.contacts.find(c=>c.segment===leadFoot)?.forceN??0;
          const readyToLift=leadLoad>WEIGHT_N*.55 && mass.velocity.y>-.05 && (toe ? pelvis.position.y>.62 : pelvis.position.y>.52);
          if(this.data.transferStage!=="bring-trailing" && readyToLift && this.release([`${trail}Shin`,trailingFoot],poses)) {
            this.stage("bring-trailing",bodies);
            const foot=this.plants.get(leadFoot)??poses.get(leadFoot)!;
            const offset=rotate(yaw,{x:(trail==="left"?-1:1)*.24,y:0,z:-.04});
            this.placements.set(trailingFoot,{...add(foot.position,offset),y:.049});
          }
        } else this.stage("extend",bodies);
      }
      if(phase==="stand" && pelvis.position.y>.86) {
        const arms=this.data.contacts.filter(c=>c.loadBearing&&/Hand|Forearm/.test(c.segment)).map(c=>c.segment);
        if(this.release(arms,poses)) this.stage("relax",bodies);
      }
    }
    if(phase==="roll") this.rootGoal={...pelvis.position};
    this.data.extension=clamp((pelvis.position.y-.55)/.44,0,1);
    // A planned foot may be captured again once it has actually replanted.
    
    void dt;
  }

  private limbTargets(side:Side,arm:boolean,poses:Map<SegmentId,SegmentPose>,targets:Map<SegmentId,Quat>):void {
    const upper:SegmentId=arm?`${side}UpperArm`:`${side}Thigh`, lower:SegmentId=arm?`${side}Forearm`:`${side}Shin`, end:SegmentId=arm?`${side}Hand`:`${side}Foot`;
    const ud=SEGMENT_BY_ID.get(upper)!,ld=SEGMENT_BY_ID.get(lower)!,ed=SEGMENT_BY_ID.get(end)!;
    const yaw=quatFromAxisAngle(UP,this.heading), phase=this.data.phase;
    const rootRotation=this.rootRotation;
    const torsoRotation=quatMultiply(rootRotation,this.placingProneArms?this.placementTorsoRotation:this.pressingProneChest?pitch(this.experiment.spineTilt):phase==="kneel" ? rotation(.3,0,this.data.leadingSide==="left"?.20:-.20) : pitch(phase==="roll"?-.3:phase==="brace"?-.12:0));
    const torsoDef=SEGMENT_BY_ID.get("torso")!;
    const torsoPosition=sub(worldPoint(this.rootGoal,rootRotation,torsoDef.jointAnchorParent!),rotate(torsoRotation,torsoDef.jointAnchorChild!));
    const parentRotation=arm?torsoRotation:rootRotation;
    const start=worldPoint(arm?torsoPosition:this.rootGoal,parentRotation,ud.jointAnchorParent!);
    let endRotation=arm?(this.armBraces.get(side)?.rotation??quatMultiply(yaw,pitch(-1.1))):yaw;
    let desired=this.placements.get(end)!;
    const plant=this.plants.get(end);
    const lowerPlant=this.plants.get(lower);
    if(lowerPlant && !this.released.has(lower)) {
      const middle=worldPoint(lowerPlant.position,lowerPlant.rotation,ld.jointAnchorChild!);
      const upperWorld=quatMultiply(quatFromTo(UP,normalize(sub(start,middle))),yaw);
      targets.set(upper,quatMultiply(quatInverse(parentRotation),upperWorld));
      targets.set(lower,quatMultiply(quatInverse(upperWorld),lowerPlant.rotation));
      targets.set(end,poses.get(end) ? quatMultiply(quatInverse(poses.get(lower)!.rotation),poses.get(end)!.rotation) : pitch(0));
      return;
    }
    if(plant) {desired=plant.position;endRotation=plant.rotation;}
    if(arm && !plant && (phase==="stand" || (phase==="kneel"&&this.data.transferStage==="extend"))) {
      desired=add(start,rotate(yaw,{x:(side==="left"?-1:1)*.04,y:-.70,z:.04})); endRotation=yaw;
    }
    if(!arm && phase==="roll" && !plant) {
      desired=this.placements.get(end)!;
    }
    if(!arm && (phase==="brace"||phase==="kneel") && side!==this.data.leadingSide && this.data.transferStage!=="bring-trailing" && this.data.transferStage!=="extend" && !plant) {
      desired={...add(this.rootGoal,rotate(yaw,{x:(side==="left"?-1:1)*.14,y:0,z:-.38})),y:.07};
      endRotation=quatMultiply(yaw,pitch(.3));
    }
    if(arm && !plant && this.armBraces.has(side)) {
      const brace=this.armBraces.get(side)!;
      const solved=solveRecoveryArmTarget(start,desired,brace.trajectoryBend,brace.wristRotation,this.heading);
      targets.set(upper,quatMultiply(quatInverse(parentRotation),solved.upperRotation));
      targets.set(lower,quatMultiply(quatInverse(solved.upperRotation),solved.forearmRotation));
      targets.set(end,quatMultiply(quatInverse(solved.forearmRotation),solved.handRotation));
      return;
    }
    // The same reach-limited geometry solver is shared with procedural standing.
    const requested=worldPoint(desired,endRotation,ed.jointAnchorChild!);
    const solved=solveTwoBone(start,requested,length(sub(ud.jointAnchorChild!,ld.jointAnchorParent!)),length(sub(ld.jointAnchorChild!,ed.jointAnchorParent!)),rotate(parentRotation,this.bend.get(upper)??{x:0,y:0,z:arm?-1:1}));
    const axisA=normalize(sub(start,solved.middle)),axisB=normalize(sub(solved.middle,solved.end));
    const hinge=normalize(scale(cross(axisA,axisB),arm?-1:1),rotate(yaw,{x:1,y:0,z:0}));
    const base=quatFromTo(UP,axisA),baseX=rotate(base,{x:1,y:0,z:0});
    const twist=Math.atan2(dot(axisA,cross(baseX,hinge)),dot(baseX,hinge));
    const a=quatMultiply(quatFromAxisAngle(axisA,twist),base);
    const b=quatMultiply(a,pitch((arm?-1:1)*Math.acos(clamp(dot(axisA,axisB),-1,1))));
    targets.set(upper,quatMultiply(quatInverse(parentRotation),a));
    targets.set(lower,quatMultiply(quatInverse(a),b));
    targets.set(end,quatMultiply(quatInverse(b),endRotation));
    void poses;
  }

  private actuate(bodies:Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>,active:boolean,dt:number):void {
    const targets=new Map<SegmentId,Quat>();
    const holdCrouch=!active && poses.get("pelvis")!.position.y>.40 && poses.get("pelvis")!.position.y<.85
      && rotate(poses.get("torso")!.rotation,UP).y>.65 && SIDES.some(side=>poses.get(`${side}Foot`)!.position.y<.13);
    if(active) {
      targets.set("torso",this.placingProneArms?this.placementTorsoRotation:this.pressingProneChest?pitch(this.experiment.spineTilt):this.data.phase==="kneel" ? rotation(.3,0,this.data.leadingSide==="left"?.20:-.20) : pitch(this.data.phase==="roll"?-.3:this.data.phase==="brace"?-.12:0));
      for(const side of SIDES) {
        if(this.pressingProneChest && this.kneeFlexing && side!==this.data.leadingSide) {
          targets.set(`${side}Thigh`,pitch(this.experiment.kneeHip));targets.set(`${side}Shin`,pitch(this.experiment.kneeBend));targets.set(`${side}Foot`,pitch(-.6));
        } else if(this.placingProneArms || (this.pressingProneChest && this.data.transferStage==="tuck-knee")) for(const suffix of ["Thigh","Shin","Foot"] as const) targets.set(`${side}${suffix}`,this.entry.get(`${side}${suffix}`)!);
        else this.limbTargets(side,false,poses,targets);
        this.limbTargets(side,true,poses,targets);
      }
    } else {
      const backward=this.data.orientation==="backward";
      targets.set("torso",pitch(backward?.30:-.18)); targets.set("head",pitch(.32));
      for(const side of SIDES) {
        const sign=side==="left"?-1:1;
        targets.set(`${side}UpperArm`,rotation(backward?-.25:-1.25,0,this.data.orientation===side?sign*1.1:sign*.22));
        targets.set(`${side}Forearm`,pitch(-.9));targets.set(`${side}Thigh`,pitch(-.35));targets.set(`${side}Shin`,pitch(.70));
      }
      if(holdCrouch)
        for(const [id,q] of this.entry) targets.set(id,q);
    }
    const blend=smooth01(this.stageTime/(active?.55:.25));
    const loaded=this.data.contacts.filter(c=>c.forceN>=RECOVERY_LIMITS.minimumLoadN && (this.data.phase==="roll" || this.data.phase==="brace" || /Foot|Shin|Hand|Forearm/.test(c.segment)) && !this.released.has(c.segment) );
    // Inverse statics supplies the load through joints. The equal/opposite impulses
    // create no external lift; only actual floor reactions can raise the mass.
    const supportingEnds=loaded.filter(c=>!loaded.some(other=>other.segment!==c.segment && other.segment.slice(0,4)===c.segment.slice(0,4)
      && ((c.segment.endsWith("Shin")&&other.segment.endsWith("Foot"))||(c.segment.endsWith("Forearm")&&other.segment.endsWith("Hand")))));
    const massState=recoveryMassState(poses.values());
    const pressurePoints=supportingEnds.map(contact=>{
      const point={...contact.point};
      if(contact.segment.endsWith("Foot")) {
        const q=quatFromAxisAngle(UP,this.heading),local=rotate(quatInverse(q),sub(worldPoint(poses.get(contact.segment)!.position,poses.get(contact.segment)!.rotation,SEGMENT_BY_ID.get(contact.segment)!.jointAnchorChild!),contact.point));
        const patch=(contact.points??[contact.point]).map(p=>rotate(quatInverse(q),sub(p,contact.point)));
        const shift=rotate(q,{x:0,y:0,z:clamp(local.z,Math.min(...patch.map(p=>p.z)),Math.max(...patch.map(p=>p.z)))});
        point.x+=shift.x;point.z+=shift.z;
      }
      return point;
    });
    let shares=supportingEnds.map(()=>1/Math.max(1,supportingEnds.length));
    const targetMass=add(massState.position,scale(massState.velocity,.15));
    for(let iteration=0;iteration<32 && shares.length>1;iteration++) {
      const center=pressurePoints.reduce((sum,p,i)=>add(sum,scale(p,shares[i])),ZERO);
      const error={x:center.x-targetMass.x,y:0,z:center.z-targetMass.z};
      const average=pressurePoints.reduce(add,ZERO);
      shares=shares.map((w,i)=>Math.max(0,w-1.5*dot(sub(pressurePoints[i],scale(average,1/shares.length)),error)));
      const total=shares.reduce((a,b)=>a+b,0); shares=shares.map(w=>w/Math.max(total,1e-9));
    }
    const descends=(id:SegmentId,ancestor:SegmentId):boolean=>{let current:SegmentId|null=id;while(current){if(current===ancestor)return true;current=SEGMENT_BY_ID.get(current)!.parent;}return false;};
    const motors: RecoveryMotorIntent[]=[];
    for(const d of SEGMENTS) {
      if(!d.parent) continue;
      const child=bodies.get(d.id)!,parent=bodies.get(d.parent)!;
      let raw=targets.get(d.id)??{x:0,y:0,z:0,w:1}; if(raw.w<0) raw={x:-raw.x,y:-raw.y,z:-raw.z,w:-raw.w};
      // YXZ decomposition matches rotation(); limit only motor intent, never a body transform.
      let x=/Shin|Forearm/.test(d.id) ? 2*Math.atan2(raw.x,raw.w) : Math.asin(clamp(2*(raw.w*raw.x-raw.y*raw.z),-1,1));
      let y=/Shin|Forearm/.test(d.id)?0:Math.atan2(2*(raw.x*raw.z+raw.w*raw.y),1-2*(raw.x*raw.x+raw.y*raw.y));
      let z=/Shin|Forearm/.test(d.id)?0:Math.atan2(2*(raw.x*raw.y+raw.w*raw.z),1-2*(raw.x*raw.x+raw.z*raw.z));
      const limit=d.jointLimitRadians!;
      if(!/Shin|Forearm/.test(d.id)) {
        const wrap=(angle:number)=>Math.atan2(Math.sin(angle),Math.cos(angle));
        const alternate={x:x>=0?Math.PI-x:-Math.PI-x,y:wrap(y+Math.PI),z:wrap(z+Math.PI)};
        const cost=(a:Vec3)=>Math.max(0,Math.abs(a.x)-limit.x)**2+Math.max(0,Math.abs(a.y)-limit.y)**2+Math.max(0,Math.abs(a.z)-limit.z)**2+.01*(a.y*a.y+a.z*a.z);
        if(cost(alternate)<cost({x,y,z})) ({x,y,z}=alternate);
      }
      const target=rotation(clamp(x,d.id.endsWith("Shin")?.025:-limit.x,d.id.endsWith("Forearm")?-.025:limit.x),clamp(y,-limit.y,limit.y),clamp(z,-limit.z,limit.z));
      const entry=this.entry.get(d.id)??target;
      const delta=angularVelocity(entry,target,1);
      const blended=quatMultiply(quatFromAxisAngle(normalize(delta),length(delta)*blend),entry);
      const relative=quatMultiply(quatInverse(parent.rotation()),child.rotation());
      const error=rotate(parent.rotation(),angularVelocity(relative,blended,1));
      const large=/Thigh|Shin|torso/.test(d.id);
      const cap=d.id==="torso"?110:/Thigh|Shin/.test(d.id)?110:/UpperArm/.test(d.id)?30:/Forearm/.test(d.id)?20:/Foot/.test(d.id)?65:/neck|head/.test(d.id)?22:9;
      const kp=active||holdCrouch?(d.id==="torso"?1800:large||/Foot/.test(d.id)?2800:/neck|head/.test(d.id)?1500:500):(large?35:12);
      const kd=active||holdCrouch?(large?85:/Foot/.test(d.id)?35:/neck|head/.test(d.id)?12:16):(large?6:1.6);
      let feedforward=ZERO;
      if((active||holdCrouch) && supportingEnds.length && !this.placingProneArms && !this.pressingProneChest) {
        const joint=worldPoint(child.translation(),child.rotation(),d.jointAnchorChild!);
        for(const [contactIndex,contact] of supportingEnds.entries()) if(descends(contact.segment,d.id)) {
          const point=pressurePoints[contactIndex];
          const correction=active ? clampLength(sub(scale(sub(this.rootGoal,poses.get("pelvis")!.position),650),scale(massState.velocity,180)),160) : ZERO;
          feedforward=add(feedforward,cross(sub(point,joint),{x:-correction.x*shares[contactIndex],y:-(active ? (WEIGHT_N + (this.data.phase==="roll"?0:Math.max(0,this.rootGoal.y-poses.get("pelvis")!.position.y)*1000)) : WEIGHT_N)*shares[contactIndex],z:-correction.z*shares[contactIndex]}));
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

      if(this.pressingProneChest && this.data.phase==="brace" && /UpperArm|Forearm|Hand/.test(d.id)) {
        const joint=worldPoint(child.translation(),child.rotation(),d.jointAnchorChild!);
        for(const contact of supportingEnds) if(contact.segment.endsWith("Hand") && descends(contact.segment,d.id))
          feedforward=add(feedforward,cross(sub(contact.point,joint),{x:0,y:-WEIGHT_N*this.experiment.armLoad,z:0}));
        for(const descendant of SEGMENTS) if(descends(descendant.id,d.id))
          feedforward=add(feedforward,cross(sub(poses.get(descendant.id)!.position,joint),{x:0,y:descendant.massKg*9.81,z:0}));
      }
      motors.push({id:d.id,parent:d.parent,error,velocity:sub(poses.get(d.id)!.angularVelocity,poses.get(d.parent)!.angularVelocity),kp,kd,feedforward:scale(feedforward,this.pressingProneChest?smooth01(this.data.phaseTimeS/1.5):blend),cap});
    }
    if(active) motors.push({id:"pelvis",parent:null,error:angularVelocity(poses.get("pelvis")!.rotation,this.rootRotation,1),velocity:poses.get("pelvis")!.angularVelocity,kp:1200,kd:100,feedforward:ZERO,cap:60});
    const inertia=new Map([...bodies].map(([id,body])=>{const raw=body.effectiveWorldInvInertia();return [id,{m11:raw.m11,m12:raw.m12,m13:raw.m13,m22:raw.m22,m23:raw.m23,m33:raw.m33}] as const;}));
    const solved=solveRecoveryMotorTorques(motors,inertia,dt);
    for(const motor of motors) {
      const torque=solved.get(motor.id)!;
      bodies.get(motor.id)!.applyTorqueImpulse(scale(torque,dt),true);
      if(motor.parent) bodies.get(motor.parent)!.applyTorqueImpulse(scale(torque,-dt),true);
      if(motor.id==="pelvis") this.data.assistanceTorque=torque;
      else this.data.maxMotorTorqueNm=Math.max(this.data.maxMotorTorqueNm,length(torque));
    }
  }
  private assist(bodies:Map<SegmentId,RigidBody>,poses:Map<SegmentId,SegmentPose>,dt:number):void {
    const pelvis=bodies.get("pelvis")!, mass=recoveryMassState(poses.values());
    const geometry=supportGeometry(this.supporting(),poses,mass);
    const phase=this.data.phase;
    const enough=geometry.loadedForceN>WEIGHT_N*.25 && geometry.marginM>=-.08 && this.supporting().some(c=>/Foot|Shin/.test(c.segment));
    const error=sub(this.rootGoal,pelvis.translation()),velocity=pelvis.linvel();
    const force=phase==="roll"?ZERO:clampLength({x:error.x*500-mass.velocity.x*150,
      y:enough?clamp(error.y*600-velocity.y*100,0,WEIGHT_N*.2):0,z:error.z*500-mass.velocity.z*150},RECOVERY_LIMITS.assistanceForceN);
    pelvis.applyImpulse(scale(force,dt),true);
    this.data.assistanceForce=force;
  }
}















