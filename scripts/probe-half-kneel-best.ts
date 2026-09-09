import {writeFile} from "node:fs/promises";
import {gzipSync} from "node:zlib";
import {createEmbodiedCharacter} from "../src/character/EmbodiedCharacter";
import {TOTAL_MASS_KG, SEGMENT_BY_ID} from "../src/core/humanoid";
import {rotate, sub, length, dot} from "../src/character/math";
import type {SegmentId, Vec3} from "../src/core/types";
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from "./recovery-fixtures";
import {DynamicRecovery} from "./experimental-load-recovery";
const options={feedforwardScale:1,balancedRise:true,groundBudget:true,smoothTransfer:true,fastTrail:true,clearanceArc:true,retryToe:true,uprightRise:true,kneelHeight:.92,rootAdvance:.15,swingDuration:.65,stableSole:true,worldTorso:true};
const results=[];
for(const fixture of RECOVERY_POSE_FIXTURES.filter(f=>f.pose==="half-kneel" && f.support!=="weak")) {
 const c=await createEmbodiedCharacter("canvas2d",{heading:fixture.heading});seedRecoveryFixture(c,fixture);const recovery=new DynamicRecovery(options);recovery.reset(fixture.heading,{x:0,y:0,z:0});(c as unknown as{recovery:DynamicRecovery}).recovery=recovery;
 const snapshots=[],releases=[],plants=new Map<SegmentId,{position:Vec3;absent:number}>(),leadBends:number[]=[],entries=[];
 let recoveredS:number|null=null,maximumHeight=0,maxUp=0,maxRollUp=0,maxRootTorque=0,maxJointTorque=0,maxFootDrift=0,maxJointGap=0,maxFloorPenetration=0,maxComDifference=0,previous="",minimumLeadingShare=1;
 for(let frame=0;frame<25*60;frame++){
  c.fixedUpdate(1/60,null);const s=c.getSnapshot("canvas2d"),r=s.diagnostics.recovery,p=s.segments.find(p=>p.id==="pelvis")!,torso=s.segments.find(p=>p.id==="torso")!;
  if(s.diagnostics.errors.length)throw Error(s.diagnostics.errors.join(";"));snapshots.push(s);
  maximumHeight=Math.max(maximumHeight,p.position.y);maxUp=Math.max(maxUp,r.assistanceForce.y);if(r.phase==="roll")maxRollUp=Math.max(maxRollUp,r.assistanceForce.y);maxRootTorque=Math.max(maxRootTorque,length(r.assistanceTorque));maxJointTorque=Math.max(maxJointTorque,r.maxMotorTorqueNm);maxJointGap=Math.max(maxJointGap,s.diagnostics.maxJointSeparationM);maxFloorPenetration=Math.max(maxFloorPenetration,s.diagnostics.maxFloorPenetrationM);
  const com=s.segments.reduce((a,p)=>{const m=SEGMENT_BY_ID.get(p.id)!.massKg/TOTAL_MASS_KG;return{x:a.x+p.position.x*m,y:a.y+p.position.y*m,z:a.z+p.position.z*m};},{x:0,y:0,z:0});
  if(r.phase!=="none")maxComDifference=Math.max(maxComDifference,length(sub(com,r.centerOfMass)));
  for(const side of ["left","right"] as const){const id=`${side}Foot` as SegmentId,foot=s.segments.find(p=>p.id===id)!,contact=r.contacts.find(c=>c.segment===id&&c.loadBearing),plant=plants.get(id);
   if(contact&&!plant)plants.set(id,{position:{...foot.position},absent:0});
   if(plant){plant.absent=contact?0:plant.absent+1/60;if(plant.absent>.1 || r.releasedSupports.includes(id))plants.delete(id);else if(contact)maxFootDrift=Math.max(maxFootDrift,Math.hypot(foot.position.x-plant.position.x,foot.position.z-plant.position.z));}
  }
  if(r.transferStage==="bring-trailing"){
    const loads=r.contacts.filter(c=>c.loadBearing),total=loads.reduce((a,c)=>a+c.forceN,0),lead=loads.find(c=>c.segment===`${fixture.side}Foot`);if(total>3)minimumLeadingShare=Math.min(minimumLeadingShare,(lead?.forceN??0)/total);
  }
  const thigh=s.segments.find(p=>p.id===`${fixture.side}Thigh`)!,shin=s.segments.find(p=>p.id===`${fixture.side}Shin`)!;leadBends.push(Math.acos(Math.max(-1,Math.min(1,dot(rotate(thigh.rotation,{x:0,y:1,z:0}),rotate(shin.rotation,{x:0,y:1,z:0}))))));
  if(r.releasedSupports.length)releases.push({time:(frame+1)/60,ids:r.releasedSupports,margin:r.releaseMarginM,contacts:r.contacts.filter(c=>c.loadBearing).map(c=>({id:c.segment,force:c.forceN}))});
  if(`${r.phase}-${r.transferStage}`!==previous){entries.push({time:(frame+1)/60,phase:r.phase,stage:r.transferStage,pelvis:p.position.y,torsoUp:rotate(torso.rotation,{x:0,y:1,z:0}).y,feet:s.segments.filter(p=>p.id.endsWith("Foot")).map(p=>({id:p.id,position:p.position,up:rotate(p.rotation,{x:0,y:1,z:0}).y,force:r.contacts.find(c=>c.segment===p.id)?.forceN??0}))});previous=`${r.phase}-${r.transferStage}`;}
  if(s.diagnostics.authority==="character-motor" && recoveredS===null)recoveredS=(frame+1)/60;
  if(recoveredS!==null && (frame+1)/60>recoveredS+1)break;
 }
 const last=snapshots.at(-1)!,summary={fixture,options,recoveredS,maximumHeight,maxUp,maxRollUp,maxRootTorque,maxJointTorque,maxFootDrift,maxJointGap,maxFloorPenetration,maxComDifference,minimumLeadingShare,initialLeadBend:leadBends[0],finalLeadBend:leadBends.at(-1),stableOneSecond:recoveredS!==null&&last.diagnostics.authority==="character-motor",releases,entries};
 results.push(summary);console.log(JSON.stringify(summary));await writeFile(`evidence/recovery-20260908/world-torso-${fixture.id}.json.gz`,gzipSync(JSON.stringify({summary,snapshots})));c.dispose();
}
await writeFile("evidence/recovery-20260908/half-kneel-world-torso-results.json",JSON.stringify(results,null,2));



