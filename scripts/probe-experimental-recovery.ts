import {createEmbodiedCharacter} from '../src/character/index';
import {DynamicRecovery} from './experimental-recovery';
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from './recovery-fixtures';
import {writeFileSync} from 'node:fs';
import {rotate} from '../src/character/math';
const c=await createEmbodiedCharacter('canvas2d');
(c as unknown as { recovery: DynamicRecovery }).recovery=new DynamicRecovery();
const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.id===(process.argv[2]??'landed-prone-left'))!;
seedRecoveryFixture(c,fixture);
const rows=[];let last='';
for(let i=0;i<Number(process.argv[3]??900);i++){
 c.fixedUpdate(1/60,null);const s=c.getSnapshot('canvas2d'),r=s.diagnostics.recovery;
 const phase=r.phase+' '+r.transferStage; rows.push(s);
 if(i%60===0 || phase!==last){
 const relevant=['pelvis','torso','leftUpperArm','leftForearm','leftHand','leftThigh','leftShin','leftFoot','rightShin'];
 console.log(JSON.stringify({t:(i/60).toFixed(2),phase,route:r.route,lead:r.leadingSide,y:s.rootPosition.y,up:rotate(s.rootRotation,{x:0,y:1,z:0}).y,tu:rotate(s.segments.find(p=>p.id==='torso')!.rotation,{x:0,y:1,z:0}).y,margin:r.supportMarginM,contacts:r.contacts.filter(c=>c.loadBearing).map(c=>[c.segment,Math.round(c.forceN)]),assist:r.assistanceForce.y,retry:r.retries,drift:r.plantedTargets.map(t=>[t.segment,+t.driftM.toFixed(3)]),pos:s.segments.filter(p=>relevant.includes(p.id)).map(p=>[p.id,+p.position.y.toFixed(3),+p.position.z.toFixed(3)])}));last=phase;}
 if(s.state==='upright')break;
}
writeFileSync('evidence/experimental-'+fixture.id+'.json',JSON.stringify(rows));c.dispose();


