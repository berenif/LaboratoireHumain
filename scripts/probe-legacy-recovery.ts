import { writeFile } from "node:fs/promises";
import { createEmbodiedCharacter } from "../src/character/EmbodiedCharacter";
import type { Vec3 } from "../src/core/types";
import { DynamicRecovery } from "./experimental-legacy-recovery";
import { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } from "./recovery-fixtures";
const ZERO:Vec3={x:0,y:0,z:0};
const results=[];
for(const pose of ["half-kneel"]){
 const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.pose===pose&&f.side==="left"&&f.heading===0)!;
 for(const experiment of [{id:"direct-kneel-anchor-inertia",feedforward:0,armGain:100,rootGain:12000,rootDamping:80,preserveEntry:true,directKneel:true,jointInertia:true},{id:"direct-kneel-anchor-inertia-ff",feedforward:1,armGain:100,rootGain:12000,rootDamping:80,preserveEntry:true,directKneel:true,jointInertia:true}]){
  const c=await createEmbodiedCharacter("canvas2d",{heading:fixture.heading});seedRecoveryFixture(c,fixture);
  const r=new DynamicRecovery(experiment);r.reset(fixture.heading,ZERO);
  (c as unknown as {recovery:DynamicRecovery}).recovery=r;
  const entries:unknown[]=[],samples:unknown[]=[];let previous="",recoveredS:number|null=null,maxY=0,maxLift=0,maxRollLift=0,maxTorque=0,maxJoint=0,maxSeparation=0,maxPenetration=0;
  for(let frame=0;frame<25*60;frame++){
   c.fixedUpdate(1/60,null);const s=c.getSnapshot("canvas2d"),d=s.diagnostics.recovery;if(s.diagnostics.errors.length) throw new Error(s.diagnostics.errors.join("; "));const p=s.segments.find(p=>p.id==="pelvis")!,t=s.segments.find(p=>p.id==="torso")!;
   const up=(q:typeof p.rotation)=>1-2*(q.x*q.x+q.z*q.z);
   const event={time:(frame+1)/60,phase:d.phase,pelvisY:p.position.y,torsoUp:up(t.rotation),support:d.contacts.filter(c=>c.loadBearing).map(c=>c.segment),retries:d.retries};
   if(d.phase!==previous){entries.push(event);previous=d.phase;}if(frame%30===0)samples.push(event);
   maxY=Math.max(maxY,p.position.y);maxLift=Math.max(maxLift,d.assistanceForce.y);if(d.phase==="roll")maxRollLift=Math.max(maxRollLift,d.assistanceForce.y);maxTorque=Math.max(maxTorque,Math.hypot(d.assistanceTorque.x,d.assistanceTorque.y,d.assistanceTorque.z));maxJoint=Math.max(maxJoint,d.maxMotorTorqueNm);maxSeparation=Math.max(maxSeparation,s.diagnostics.maxJointSeparationM);maxPenetration=Math.max(maxPenetration,s.diagnostics.maxFloorPenetrationM);
   if(s.diagnostics.authority==="character-motor"){recoveredS=(frame+1)/60;break;}
  }
  c.dispose();const result={fixture:fixture.id,experiment,recoveredS,maxY,maxLift,maxRollLift,maxTorque,maxJoint,maxSeparation,maxPenetration,entries,samples};results.push(result);console.log(JSON.stringify({...result,samples:undefined}));
 }
}
await writeFile("evidence/recovery-20260908/legacy-anchor-experiment.json",JSON.stringify(results,null,2));
