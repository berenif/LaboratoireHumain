import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { createHash } from "node:crypto";
import type { RigidBody } from "@dimforge/rapier3d-compat";
import { DynamicRecovery } from "../src/character/DynamicRecovery";
import { createEmbodiedCharacter } from "../src/character/index";
import { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } from "./recovery-fixtures";
import { SEGMENTS } from "../src/core/humanoid";
import type { CharacterController, MotionState, Quat, RecoveryDiagnostics, SegmentId, SegmentPose, Vec3 } from "../src/core/types";
import { add, length, quatFromAxisAngle, quatInverse, quatMultiply, rotate, scale, sub } from "../src/character/math";

const DT = 1 / 60, ZERO: Vec3 = { x: 0, y: 0, z: 0 }, heading = Math.PI / 3;
const inverseYaw = quatInverse(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading));
const frames = Number(process.argv[2] ?? 240);
const sourceHash = createHash("sha256").update(readFileSync(new URL("../src/character/DynamicRecovery.ts",import.meta.url))).digest("hex");
interface ControllerInspection {
  heading: number; rootGoal: Vec3; rootRotation: Quat; entry: Map<SegmentId, Quat>; bend: Map<SegmentId, Vec3>;
  placements: Map<SegmentId, Vec3>; apply(bodies: Map<SegmentId, RigidBody>, dt: number): { state: MotionState; recovered: boolean };
  diagnostics(): RecoveryDiagnostics;
}
interface Internals { recovery: ControllerInspection; ragdollBodies: Map<SegmentId, RigidBody> }
interface Impulse { segment: SegmentId; kind: "torque" | "force"; value: Vec3 }
interface ControlFrame { frozen?: {maxTorqueDifferenceNm:number;maxForceDifferenceN:number;rootGoalDifferenceM:number;rootRotationDifferenceRadians:number;phases:string[];calls:Array<{segment:SegmentId;kind:string;difference:number}>}; before: SegmentPose[]; plan: { heading: number; rootGoal: Vec3; rootRotation: Quat; entry: Array<[SegmentId, Quat]>; bend: Array<[SegmentId, Vec3]>; placements: Array<[SegmentId, Vec3]> }; recovery: RecoveryDiagnostics; impulses: Impulse[] }

interface Inertia {m11:number;m12:number;m13:number;m22:number;m23:number;m33:number}
function multiplyInertia(m:Inertia,v:Vec3):Vec3 {return {x:m.m11*v.x+m.m12*v.y+m.m13*v.z,y:m.m12*v.x+m.m22*v.y+m.m23*v.z,z:m.m13*v.x+m.m23*v.y+m.m33*v.z};}
function turnInertia(m:Inertia,q:Quat):Inertia {
  const columns=[{x:1,y:0,z:0},{x:0,y:1,z:0},{x:0,y:0,z:1}].map(v=>rotate(q,multiplyInertia(m,rotate(quatInverse(q),v))));
  return {m11:columns[0].x,m12:columns[1].x,m13:columns[2].x,m22:columns[1].y,m23:columns[2].y,m33:columns[2].z};
}
function frozenControlComparison(source:ControllerInspection,bodies:Map<SegmentId,RigidBody>):NonNullable<ControlFrame["frozen"]> {
  const q=quatFromAxisAngle({x:0,y:1,z:0},heading);
  type Clone=ControllerInspection & {data:RecoveryDiagnostics;plants:Map<SegmentId,{position:Vec3;rotation:Quat;absentS:number}>};
  const originalState=Object.fromEntries(Object.entries(source).filter(([,value])=>typeof value!=="function"));
  const clones=[0,1].map(index=>{
    const c=Object.assign(new DynamicRecovery(),structuredClone(originalState)) as unknown as Clone;
    if(index) {
      c.heading+=heading;c.rootGoal=rotate(q,c.rootGoal);c.rootRotation=quatMultiply(q,c.rootRotation);
      c.placements=new Map([...c.placements].map(([id,p])=>[id,rotate(q,p)]));
      c.plants=new Map([...c.plants].map(([id,p])=>[id,{...p,position:rotate(q,p.position),rotation:quatMultiply(q,p.rotation)}]));
      c.data.contacts=c.data.contacts.map(contact=>({...contact,point:rotate(q,contact.point),points:contact.points?.map(p=>rotate(q,p))}));
      c.data.centerOfMass=rotate(q,c.data.centerOfMass);c.data.projectedCenterOfMass=rotate(q,c.data.projectedCenterOfMass);
    }
    return c;
  });
  const logs:Impulse[][]=[[],[]];
  for(const [index,c] of clones.entries()) {
    const fakes=new Map<SegmentId,RigidBody>();
    for(const [id,body] of bodies) {
      const turn=index?q:{x:0,y:0,z:0,w:1};
      const rotation=quatMultiply(turn,body.rotation()),position=rotate(turn,body.translation());
      let angular=rotate(turn,body.angvel()),linear=rotate(turn,body.linvel());
      const raw=body.effectiveWorldInvInertia(),copy={m11:raw.m11,m12:raw.m12,m13:raw.m13,m22:raw.m22,m23:raw.m23,m33:raw.m33};
      const inertia=index?turnInertia(copy,q):copy,invMass=1/body.mass();
      fakes.set(id,{translation:()=>position,rotation:()=>rotation,angvel:()=>angular,linvel:()=>linear,effectiveWorldInvInertia:()=>inertia,
        applyTorqueImpulse:(value:Vec3)=>{logs[index].push({segment:id,kind:"torque",value:{...value}});angular=add(angular,multiplyInertia(inertia,value));},
        applyImpulse:(value:Vec3)=>{logs[index].push({segment:id,kind:"force",value:{...value}});linear=add(linear,scale(value,invMass));},
      } as unknown as RigidBody);
    }
    c.apply(fakes,DT);
  }
  const calls=logs[0].map((call,index)=>({segment:call.segment,kind:call.kind,difference:logs[1][index]?length(scale(sub(call.value,rotate(inverseYaw,logs[1][index].value)),1/DT)):Infinity}));
  return {maxTorqueDifferenceNm:Math.max(0,...calls.filter(c=>c.kind==="torque").map(c=>c.difference)),maxForceDifferenceN:Math.max(0,...calls.filter(c=>c.kind==="force").map(c=>c.difference)),
    rootGoalDifferenceM:length(sub(clones[0].rootGoal,rotate(inverseYaw,clones[1].rootGoal))),rootRotationDifferenceRadians:angle(clones[0].rootRotation,quatMultiply(inverseYaw,clones[1].rootRotation)),phases:clones.map(c=>c.diagnostics().phase),calls};
}
const orientations = [0, heading];
const characters: CharacterController[] = [];
const captures: Array<ControlFrame | null> = [null, null];
let currentImpulses: Impulse[][] = [[], []];
function angle(a: Quat, b: Quat): number {
  const dot = Math.abs(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w) / (Math.hypot(a.x,a.y,a.z,a.w)*Math.hypot(b.x,b.y,b.z,b.w));
  return 2 * Math.acos(Math.min(1, dot));
}
for (const [index, yaw] of orientations.entries()) {
  const character = await createEmbodiedCharacter("canvas2d", { heading: yaw });
  const fixture = RECOVERY_POSE_FIXTURES.find(f => f.pose === "crouch" && f.side === "left" && f.heading === yaw)!;
  seedRecoveryFixture(character, fixture);
  const internals = character as unknown as Internals;
  for (const [segment, body] of internals.ragdollBodies) {
    const torque = body.applyTorqueImpulse.bind(body), force = body.applyImpulse.bind(body);
    body.applyTorqueImpulse = (value, wake) => { currentImpulses[index].push({segment,kind:"torque",value:{...value}}); torque(value,wake); };
    body.applyImpulse = (value, wake) => { currentImpulses[index].push({segment,kind:"force",value:{...value}}); force(value,wake); };
  }
  const recovery = internals.recovery, original = recovery.apply.bind(recovery);
  recovery.apply = (bodies, dt) => {
    const before = [...bodies].map(([id, body]) => ({id,position:{...body.translation()},rotation:{...body.rotation()},linearVelocity:{...body.linvel()},angularVelocity:{...body.angvel()}}));
    const frozen = index === 0 ? frozenControlComparison(recovery,bodies) : undefined;
    const result = original(bodies, dt);
    captures[index] = { frozen, before, recovery: recovery.diagnostics(), impulses: currentImpulses[index], plan: {
      heading: recovery.heading, rootGoal: {...recovery.rootGoal}, rootRotation: {...recovery.rootRotation},
      entry: [...recovery.entry].map(([id,q])=>[id,{...q}]), bend: [...recovery.bend].map(([id,v])=>[id,{...v}]),
      placements: [...recovery.placements].map(([id,v])=>[id,{...v}]),
    } };
    return result;
  };
  characters.push(character);
}
const rows: unknown[] = [], thresholds = [1e-6,1e-4,0.01,0.1,1];
const firstTorqueDifference = new Map<number, unknown>();
const outcome: Array<{heading:number;recoveredTimeS:number|null}> = orientations.map(heading=>({heading,recoveredTimeS:null}));
for (let frame = 1; frame <= frames; frame++) {
  currentImpulses = [[], []]; captures[0] = null; captures[1] = null;
  for (const [index, character] of characters.entries()) {
    character.fixedUpdate(DT, null);
    if(character.diagnostics().state === "upright") outcome[index].recoveredTimeS ??= character.getSnapshot("canvas2d").simulationTime;
  }
  const a = captures[0] as ControlFrame | null, b = captures[1] as ControlFrame | null;
  if(!a || !b) { rows.push({frame,states:characters.map(c=>c.diagnostics().state),pairedDynamic:false}); continue; }
  const aggregate = (capture:ControlFrame, undoYaw:boolean) => {
    const totals = new Map<SegmentId,Vec3>();
    for(const impulse of capture.impulses.filter(i=>i.kind==="torque")) totals.set(impulse.segment,add(totals.get(impulse.segment)??ZERO,undoYaw?rotate(inverseYaw,impulse.value):impulse.value));
    return totals;
  };
  const aa=aggregate(a,false),bb=aggregate(b,true);
  const bodyTorqueErrors=SEGMENTS.map(d=>({segment:d.id, differenceNm:length(scale(sub(aa.get(d.id)??ZERO,bb.get(d.id)??ZERO),1/DT)),baseNm:scale(aa.get(d.id)??ZERO,1/DT),rotatedBackNm:scale(bb.get(d.id)??ZERO,1/DT)})).sort((a,b)=>b.differenceNm-a.differenceNm);
  const poseErrors=a.before.map(p=>{const q=b.before.find(q=>q.id===p.id)!;return {segment:p.id,positionM:length(sub(p.position,rotate(inverseYaw,q.position))),rotationRadians:angle(p.rotation,quatMultiply(inverseYaw,q.rotation)),linearMps:length(sub(p.linearVelocity,rotate(inverseYaw,q.linearVelocity))),angularRadps:length(sub(p.angularVelocity,rotate(inverseYaw,q.angularVelocity)))};});
  const perCall=a.impulses.map((call,i)=>{const other=b.impulses[i];return {index:i,segment:call.segment,kind:call.kind,matching:other?.segment===call.segment&&other.kind===call.kind,difference:other?length(scale(sub(call.value,rotate(inverseYaw,other.value)),1/DT)):null};});
  const comparison={frame,timeS:frame*DT,phases:[a.recovery.phase,b.recovery.phase],routes:[a.recovery.route,b.recovery.route],stages:[a.recovery.transferStage,b.recovery.transferStage],
    maxBodyTorqueDifferenceNm:bodyTorqueErrors[0].differenceNm,bodyTorqueErrors,poseErrors,perCall,
    rootGoalDifferenceM:length(sub(a.plan.rootGoal,rotate(inverseYaw,b.plan.rootGoal))),rootRotationDifferenceRadians:angle(a.plan.rootRotation,quatMultiply(inverseYaw,b.plan.rootRotation)),
    localEntryRotationErrors:a.plan.entry.map(([id,q])=>({segment:id,radians:angle(q,b.plan.entry.find(([bid])=>bid===id)![1])})),
    base:a,rotated:b};
  for(const threshold of thresholds) if(!firstTorqueDifference.has(threshold)&&comparison.maxBodyTorqueDifferenceNm>threshold) firstTorqueDifference.set(threshold,{thresholdNm:threshold,frame,timeS:frame*DT,worstBody:bodyTorqueErrors[0],maxInputPositionM:Math.max(...poseErrors.map(p=>p.positionM)),maxInputRotationRadians:Math.max(...poseErrors.map(p=>p.rotationRadians)),maxInputAngularRadps:Math.max(...poseErrors.map(p=>p.angularRadps)),phases:comparison.phases,rootGoalDifferenceM:comparison.rootGoalDifferenceM,rootRotationDifferenceRadians:comparison.rootRotationDifferenceRadians,perCall:perCall.filter(p=>(p.difference??0)>threshold)});
  rows.push(comparison);
}
for(const character of characters)character.dispose();
const frozenRows=rows as Array<{frame:number;base?:ControlFrame}>;
const frozenSummary={maxTorqueDifferenceNm:Math.max(...frozenRows.map(row=>row.base?.frozen?.maxTorqueDifferenceNm??0)),firstAboveTolerance:frozenRows.filter(row=>(row.base?.frozen?.maxTorqueDifferenceNm??0)>1e-5).slice(0,3).map(row=>({frame:row.frame,...row.base?.frozen}))};
const summary={sourceHash,frozenSummary,fixture:"crouch-left",headings:orientations,frames,outcome,firstTorqueDifference:[...firstTorqueDifference.values()]};
mkdirSync("evidence",{recursive:true});writeFileSync("evidence/probe-equivariance.json",JSON.stringify({summary,rows},null,2));
console.log(JSON.stringify(summary,null,2));
