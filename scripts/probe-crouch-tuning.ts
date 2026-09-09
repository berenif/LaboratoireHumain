import {writeFileSync} from 'node:fs';
import {createEmbodiedCharacter} from '../src/character/index';
import {DynamicRecovery as ProductionDynamicRecovery} from '../src/character/DynamicRecovery';
import {DynamicRecovery,CROUCH_TUNING} from './experimental-crouch-recovery';
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from './recovery-fixtures';
import {measureRecoveryPhysics,newRecoveryPhysicsMeasurements} from './recovery-measurements';
import type {RigidBody,Collider,World} from '@dimforge/rapier3d-compat';
import type {SegmentId} from '../src/core/types';
const defaults={...CROUCH_TUNING};
const variants=[
 {id:'production',production:true},
 {id:'current-implicit'},
 {id:'symmetric-torso',torsoPitch:.15},
 {id:'direct-stand',directStand:true},
 {id:'direct-stand-damped',directStand:true,pelvisKd:180,largeKd:120},
 {id:'symmetric-explicit',torsoPitch:.15,implicit:false},
 {id:'direct-stand-explicit',directStand:true,implicit:false},
];
const results=[];
for(const variant of variants.filter(v=>!process.argv[2]||v.id.includes(process.argv[2]))) {
 Object.assign(CROUCH_TUNING,defaults,variant);
 const cases=[];
 for(const f of RECOVERY_POSE_FIXTURES.filter(f=>f.pose==='crouch')) {
  const c=await createEmbodiedCharacter('canvas2d',{heading:f.heading});
  const internal=c as unknown as {recovery:DynamicRecovery;world:World;floorCollider:Collider;ragdollColliders:Map<SegmentId,Collider>;ragdollBodies:Map<SegmentId,RigidBody>};
  if (!(variant as {production?:boolean}).production) internal.recovery=new DynamicRecovery();
  else internal.recovery=new ProductionDynamicRecovery() as unknown as DynamicRecovery;
  seedRecoveryFixture(c,f);
  let previous=c.getSnapshot('canvas2d'),recoveredS:number|null=null;
  const measurements=newRecoveryPhysicsMeasurements(),violations=new Set<string>();
  for(let step=0;step<1500;step++) {
   c.fixedUpdate(1/60,null);const snapshot=c.getSnapshot('canvas2d');
   measureRecoveryPhysics(measurements,snapshot,previous,internal,violations);previous=snapshot;
   if(snapshot.state==='upright'){recoveredS=snapshot.simulationTime;break;}
  }
  cases.push({id:f.id,recoveredS,driftM:measurements.report.maxPlantedDriftM,violations:[...violations],phase:previous.diagnostics.recovery.phase,retries:previous.diagnostics.recovery.retries,root:previous.rootPosition,maxUp:measurements.report.maxUpwardAssistanceN,torque:measurements.report.maxPelvisTorqueNm,kneeReverse:measurements.report.maximumKneeReverseRadians,elbowReverse:measurements.report.maximumElbowReverseRadians});c.dispose();
 }
 const result={variant,cases,passed:cases.every(c=>c.recoveredS!==null&&c.violations.length===0)};
 results.push(result);console.log(JSON.stringify(result));
 writeFileSync('evidence/crouch-tuning-'+(process.argv[2]??'all')+'.json',JSON.stringify(results,null,2));
}
