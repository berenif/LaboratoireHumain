import {writeFile}from"node:fs/promises";
import{createEmbodiedCharacter}from"../src/character/EmbodiedCharacter";
import{RECOVERY_POSE_FIXTURES,seedRecoveryFixture}from"./recovery-fixtures";
import{DynamicRecovery}from"./experimental-load-recovery";
const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.id==="landed-half-kneel-left")!,results=[];
for(const experiment of [{id:"higher-kneel-budget",feedforwardScale:1,balancedRise:true,groundBudget:true,smoothTransfer:true,fastTrail:true,clearanceArc:true,retryToe:true,uprightRise:true,kneelHeight:.92,rootAdvance:.15},{id:"higher-kneel-actual-budget",feedforwardScale:1,balancedRise:true,groundBudget:true,smoothTransfer:true,fastTrail:true,clearanceArc:true,retryToe:true,uprightRise:true,kneelHeight:.92,rootAdvance:.15,actualBudget:true},{id:"higher-kneel-actual-budget-inertia",feedforwardScale:1,balancedRise:true,groundBudget:true,smoothTransfer:true,fastTrail:true,clearanceArc:true,retryToe:true,uprightRise:true,kneelHeight:.92,rootAdvance:.15,actualBudget:true,swingInertia:true}]){
 const c=await createEmbodiedCharacter("canvas2d",{heading:fixture.heading});seedRecoveryFixture(c,fixture);const recovery=new DynamicRecovery(experiment);recovery.reset(fixture.heading,{x:0,y:0,z:0});(c as unknown as{recovery:DynamicRecovery}).recovery=recovery;
 let maximumHeight=0,recoveredS:number|null=null,earlyFall=false,previous="";const samples:unknown[]=[],entries:unknown[]=[];
 for(let frame=0;frame<25*60;frame++){
  c.fixedUpdate(1/60,null);const s=c.getSnapshot("canvas2d"),r=s.diagnostics.recovery;if(s.diagnostics.errors.length)throw Error(s.diagnostics.errors.join(";"));
  const pelvis=s.segments.find(p=>p.id==="pelvis")!,torso=s.segments.find(p=>p.id==="torso")!;maximumHeight=Math.max(maximumHeight,pelvis.position.y);
  const summary={time:(frame+1)/60,phase:r.phase,stage:r.transferStage,height:pelvis.position.y,torsoUp:1-2*(torso.rotation.x**2+torso.rotation.z**2),margin:r.supportMarginM,load:r.contacts.filter(c=>c.forceN>3).map(c=>({segment:c.segment,forceN:c.forceN,point:c.point})),segments:s.segments.filter(p=>/pelvis|torso|Thigh|Shin|Foot/.test(p.id)),com:r.centerOfMass,projectedCom:r.projectedCenterOfMass,rootGoal:(recovery as unknown as{rootGoal:unknown}).rootGoal};
  if(`${r.phase}-${r.transferStage}`!==previous){entries.push(summary);previous=`${r.phase}-${r.transferStage}`;}
  if(frame%3===0&&frame<150)samples.push({...summary,motors:recovery.lastMotorTrace.filter(m=>/Thigh|Shin|Foot|pelvis|torso/.test(m.id))});
  if(s.diagnostics.authority==="character-motor"){recoveredS=(frame+1)/60;break;}
  if(frame>60&&pelvis.position.y<.25){earlyFall=true;break;}
 }
 const result={experiment,maximumHeight,recoveredS,earlyFall,entries,samples};results.push(result);console.log(JSON.stringify({...result,samples:undefined,entries:entries.map(e=>{const entry={...(e as Record<string,unknown>)};delete entry.segments;return entry;})}));c.dispose();
}
await writeFile("evidence/recovery-20260908/toe-ground-budget-experiment.json",JSON.stringify(results,null,2));






