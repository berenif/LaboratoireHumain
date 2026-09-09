/* eslint-disable @typescript-eslint/no-explicit-any -- Temporary diagnostic probe of private controller state; not shipped application code. */
import {usableRecoveryArmSupport} from "../src/character/recovery-support";
import {createEmbodiedCharacter} from '../src/character/index';
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from './recovery-fixtures';
import {writeFileSync} from 'node:fs';
const c=await createEmbodiedCharacter('canvas2d');
const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.id===(process.argv[2]??'landed-crouch-left'))!;
seedRecoveryFixture(c,fixture);
const rows=[];
let last='';
for(let i=0;i<Number(process.argv[3]??1500);i++){
 c.fixedUpdate(1/60,null);
 const s=c.getSnapshot('canvas2d'),r=s.diagnostics.recovery;
 rows.push(s);
 const phase=r.phase+' '+r.transferStage;
 if(i%15===0 || phase!==last){
  console.log(JSON.stringify({t:(i/60).toFixed(2),phase,route:r.route,lead:r.leadingSide,y:s.rootPosition.y,up:1-2*(s.rootRotation.x**2+s.rootRotation.z**2),margin:r.supportMarginM,contacts:r.contacts.filter(c=>c.loadBearing).map(c=>[c.segment,Math.round(c.forceN)]),force:r.assistanceForce,torque:r.assistanceTorque,drift:r.plantedTargets.map(t=>[t.segment,Number(t.driftM.toFixed(3))]),retry:r.retries,motor:r.maxMotorTorqueNm,knee:(c as any).recovery.entry.get("leftShin"),joint:s.segments.filter(p=>/leftShin|leftThigh/.test(p.id)).map(p=>[p.id,p.rotation])}));last=phase;
 }
 if(s.state==='upright')break;
}
writeFileSync('evidence/probe-'+fixture.id+'.json',JSON.stringify(rows));
console.log(JSON.stringify({rawReady:["left","right"].map(side=>usableRecoveryArmSupport(side as any,(c as any).recovery.poses((c as any).ragdollBodies),(c as any).recovery.data.contacts)),ready:["left","right"].map(side=>(c as any).recovery.armPlacementReady(side,(c as any).recovery.poses((c as any).ragdollBodies))),arms:[...(c as any).recovery.armBraces],released:[...(c as any).recovery.released],unloaded:[...(c as any).recovery.releasedUnloaded],hands:c.getSnapshot("canvas2d").segments.filter(p=>p.id.endsWith("Hand"))}));
c.dispose();
