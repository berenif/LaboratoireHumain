import {writeFileSync} from "node:fs";
import {createEmbodiedCharacter} from "../src/character/index";
import {DynamicRecovery} from "./experimental-roll-recovery";
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from "./recovery-fixtures";
import {measureRecoveryPhysics,newRecoveryPhysicsMeasurements} from "./recovery-measurements";
import {SEGMENT_BY_ID} from "../src/core/humanoid";
import {rotate,worldPoint} from "../src/character/math";
import type {RigidBody,Collider,World} from "@dimforge/rapier3d-compat";
import type {SegmentId,Vec3,Quat} from "../src/core/types";
const results=[];
for(const fixture of RECOVERY_POSE_FIXTURES.filter(f=>(f.pose==="supine"||f.pose==="side")&&(!process.argv[2]||f.id.includes(process.argv[2])))) {
 const character=await createEmbodiedCharacter("canvas2d",{heading:fixture.heading});
 const internal=character as unknown as {recovery:DynamicRecovery;world:World;floorCollider:Collider;ragdollColliders:Map<SegmentId,Collider>;ragdollBodies:Map<SegmentId,RigidBody>};
 internal.recovery=new DynamicRecovery();seedRecoveryFixture(character,fixture);
 const controller=internal.recovery as unknown as {apply:DynamicRecovery["apply"];rootGoal:Vec3;rollTurned:boolean;placingProneArms:boolean;armBraces:Map<string,{position:Vec3;rotation:Quat}>};
 const originalApply=controller.apply.bind(controller);let maxRollRootLift=0;
 controller.apply=(bodies,dt)=>{const value=originalApply(bodies,dt);if(internal.recovery.diagnostics().phase==="roll")maxRollRootLift=Math.max(maxRollRootLift,controller.rootGoal.y-bodies.get("pelvis")!.translation().y);return value;};
 let previous=character.getSnapshot("canvas2d"),turnedS:number|null=null,readyHandsS:number|null=null,braceS:number|null=null,last="";
 const measured=newRecoveryPhysicsMeasurements(),violations=new Set<string>(),rows=[];
 let driftEvent:unknown=null,recordedDrift=0;
 for(let step=0;step<Number(process.argv[3]??1500);step++) {
  character.fixedUpdate(1/60,null);const snapshot=character.getSnapshot("canvas2d"),d=snapshot.diagnostics.recovery;
  measureRecoveryPhysics(measured,snapshot,previous,internal,violations);previous=snapshot;
  const poses=new Map(snapshot.segments.map(p=>[p.id,p])),torso=poses.get("torso")!;
  const forward=rotate(torso.rotation,{x:0,y:0,z:1}).y;
  for(const [segment,plant] of measured.planted){const pose=poses.get(segment)!;const p=worldPoint(pose.position,pose.rotation,plant.materialPoint);const drift=Math.hypot(p.x-plant.worldPoint.x,p.z-plant.worldPoint.z);if(drift>recordedDrift){recordedDrift=drift;driftEvent={segment,t:snapshot.simulationTime,phase:d.phase,placing:controller.placingProneArms,drift};}}
  const usable=["left","right"].filter(side=>{
   const contact=measured.previousContacts.find(c=>c.segment===`${side}Hand`);if(!contact)return false;
   const center=contact.points.reduce((a,p)=>({x:a.x+p.x/contact.points.length,y:a.y+p.y/contact.points.length,z:a.z+p.z/contact.points.length}),{x:0,y:0,z:0});
   const shoulder=worldPoint(torso.position,torso.rotation,SEGMENT_BY_ID.get(`${side}UpperArm` as SegmentId)!.jointAnchorParent!);
   return Math.hypot(center.x-shoulder.x,center.z-shoulder.z)<=.36 && Math.hypot(center.x-torso.position.x,center.z-torso.position.z)<=.52 && shoulder.y>=center.y-.025;
  });
  if(forward<-.5 && snapshot.rootPosition.y<.4 && turnedS===null)turnedS=snapshot.simulationTime;
  if(usable.length===2 && readyHandsS===null)readyHandsS=snapshot.simulationTime;
  const phase=d.phase+"/"+d.transferStage+"/"+d.retries;
  if(step%15===0||phase!==last){rows.push({t:snapshot.simulationTime,phase,route:d.route,leading:d.leadingSide,rollSide:d.rollSide,pelvis:snapshot.rootPosition,forward,usable,placing:controller.placingProneArms,turned:controller.rollTurned,targets:[...controller.armBraces],contacts:measured.previousContacts.map(c=>[c.segment,Math.round(c.forceN)]),drift:measured.report.maxPlantedDriftM});last=phase;}
  if(d.phase==="brace" && usable.length===2 && forward<-.5){braceS=snapshot.simulationTime;break;}
 }
 const result={fixture:fixture.id,turnedS,readyHandsS,braceS,maxRollRootLift,maxUp:measured.report.maxUpwardAssistanceN,drift:measured.report.maxPlantedDriftM,driftEvent,violations:[...violations],last:rows.at(-1)};results.push(result);console.log(JSON.stringify(result));
 writeFileSync(`evidence/roll-preparation-${fixture.id}.json`,JSON.stringify(rows,null,2));character.dispose();
}
writeFileSync(`evidence/roll-preparation-${process.argv[2]??"all"}-summary.json`,JSON.stringify(results,null,2));
