import test from 'node:test';
import assert from 'node:assert/strict';
import {checkpointShape,replacementCheckpointBound,replacementProjection,MiB} from './getup-qualification-checkpoint-budget.mjs';
const runtime=()=>({mode:'protocol',disposed:false,timeline:{generation:1,queue:{}},simulation:{generation:1,motion:'Recovering',first_failure:null,
 profile:{id:'getup-800hz-3plus1-repeat-cycle-v1',dt_s:.00125,tick_substeps:20,getup_repeated_recovery:'completed-cycle-confirmed-fall-v1',recovery_enabled:false,getup_policy_enabled:true},
 substeps_completed:500,getup_controller:{started_substep:480,reference:[[0]],teacher_frames:[],
 stable_substeps:0,student_decisions:[],status:{prototype_standing_passed:false,forecast_substeps:1020}}}});
function checkpoint(r,schema=2,head=null){
 const inner=JSON.stringify({runtime:JSON.stringify(r),roots:[0,1,2]});
 const aux=schema===4?JSON.stringify({format:'frozen-slow-correction-v1',inner_schema:2,auxiliary:inner,head}):inner;
 const parts=[Buffer.from('{}'),Buffer.alloc(100),Buffer.from(aux)];
 const out=[Buffer.from([schema,0,0,0])];for(const p of parts){const n=Buffer.alloc(8);n.writeBigUInt64LE(BigInt(p.length));out.push(n,p);}return Buffer.concat(out);
}
const lineage=s=>({modelSha256:s.modelSha256,profileSha256:s.profileSha256,generation:s.generation,profile:'getup-800hz-3plus1-repeat-cycle-v1'});
const bounded=s=>replacementCheckpointBound(s,{certifiedLineage:lineage(s)});
test('schema2 and null-head schema4 normalize both alternatives to identical escaping depth',()=>{
 const a=checkpointShape(checkpoint(runtime(),2)),b=checkpointShape(checkpoint(runtime(),4));
 assert(b.bytes>a.bytes);assert.equal(a.normalizedRuntimeBytes,b.normalizedRuntimeBytes);
 assert.equal(replacementCheckpointBound(a).bytes,replacementCheckpointBound(b).bytes);
});
test('fallback remains max retained-or-new history, not incremental fresh forecast',()=>{
 const shape=checkpointShape(checkpoint(runtime()));
 assert.equal(replacementCheckpointBound(shape).bytes,28+shape.modelBytes+20*MiB+16384);
 const large={...shape,normalizedRuntimeBytes:13*MiB};const bound=replacementCheckpointBound(large);
 assert.equal(bound.bytes,28+shape.modelBytes+21*MiB+24576+16384);
 assert(bound.bytes<large.normalizedRuntimeBytes+20*MiB);
});
test('strict incomplete779, near780/799 and completed upright/Reacting branch matrix',()=>{
 const s={...checkpointShape(checkpoint(runtime())),native:20000};
 assert.equal(bounded({...s,stable:779}).branch,'incomplete-below-success-threshold');
 for(const stable of [780,799,800]){
  const p=bounded({...s,stable});assert.equal(p.branch,'success-crossing-settling-barrier');
  assert.equal(p.emptyControllerBytes,65536);assert(p.bytes>=28+s.modelBytes+6*MiB+p.emptyReplacementBytes);
 }
 for(const motion of ['Upright','Reacting'])assert.equal(bounded({...s,completed:true,motion}).branch,'completed-upright-settling-barrier');
 for(const motion of ['Falling','Fallen','Recovering','Halted'])assert(!bounded({...s,completed:true,motion}).noReforecast);
 for(const change of [{source:null},{referenceEntries:0},{noFailure:false}])assert(!bounded({...s,...change}).noReforecast);
 assert(!replacementCheckpointBound(s).noReforecast,'Uncertified lineage never reduces bound');
});
test('queue, schema, missing boolean, alignment and changed generation/model/profile reject',()=>{
 const s=checkpointShape(checkpoint(runtime()));
 for(const change of [{native:501},{source:501},{stable:21},{lifecycleQueueEntries:1},{disposed:true}])
  assert.throws(()=>bounded({...s,...change}));
 for(const change of [{modelSha256:'bad'},{profileSha256:'bad'},{generation:2},{profile:'unknown'}])
  assert.throws(()=>replacementCheckpointBound(s,{certifiedLineage:{...lineage(s),...change}}),/lineage/);
 const r=runtime();delete r.simulation.getup_controller.status.prototype_standing_passed;
 assert.throws(()=>checkpointShape(checkpoint(r)));assert.throws(()=>checkpointShape(checkpoint(runtime(),3)));
 const queued=runtime();queued.timeline.queue={1:{action:'Reset'}};const q=checkpointShape(checkpoint(queued));
 assert.throws(()=>replacementCheckpointBound(q,{ticks:0}),/Queued/);
 assert.throws(()=>replacementCheckpointBound(q,{ticks:1}),/Queued/);
 for(const change of [{dt_s:.02},{tick_substeps:40},{getup_repeated_recovery:null},{recovery_enabled:true}]){
  const r=runtime();Object.assign(r.simulation.profile,change);
  assert.throws(()=>bounded(checkpointShape(checkpoint(r))),/lineage/);
 }
});
test('pending correction is finite/native-bound and distinct from lifecycle queue',()=>{
 const r=runtime();r.simulation.getup_controller.adaptation={pending:{native_substep:500,correction:Array(21).fill(0)}};
 assert(bounded(checkpointShape(checkpoint(r))).noReforecast);
 r.simulation.getup_controller.adaptation.pending.native_substep=480;
 assert.throws(()=>checkpointShape(checkpoint(r)),/Pending correction/);
});
test('recorded old guard counterexample has structural headroom, not a cap change',()=>{
 const s={...checkpointShape(checkpoint(runtime())),modelBytes:506781,normalizedRuntimeBytes:1260311,native:1340,source:340};
 const options={active:3844926,terminal:8163324,shapes:[s],advanced:0};
 assert.throws(()=>replacementProjection(options),error=>{
  assert.equal(error.projection.projectedActiveOutputBytes,33568499);
  assert.equal(error.projection.physicalDispatchAuthorized,false);return /BEFORE dispatch/.test(error.message);
 });
 const p=replacementProjection({...options,lineage:lineage(s)});
 assert.equal(p.replacement.bytes,10196688);assert.equal(p.projectedActiveOutputBytes,22270474);
 assert.equal(32*MiB-p.projectedActiveOutputBytes,11283958);
});
test('advanced runtime is replaced but retained disk copies and other runtimes stay additive',()=>{
 const s=checkpointShape(checkpoint(runtime()));const options={active:2000,terminal:5000,shapes:[s,s],advanced:0,lineage:lineage(s)};
 const p=replacementProjection(options);assert.equal(p.unchangedRuntimeBytes,s.bytes);
 assert.equal(p.projectedActiveOutputBytes,7000+s.bytes+p.replacement.bytes+65536);
 assert.equal(replacementProjection({...options,active:2000+s.bytes}).projectedActiveOutputBytes-p.projectedActiveOutputBytes,s.bytes);
 assert.throws(()=>replacementProjection({...options,limit:p.projectedActiveOutputBytes-1}),/BEFORE dispatch/);
 assert.equal(replacementProjection({...options,ticks:0}).replacement.bytes,s.bytes);
});
test('known partial-call failure still admits exact nonstepping terminal capture',()=>{
 const r=runtime();r.simulation.first_failure={check:'synthetic partial-native'};r.simulation.motion='Halted';
 r.simulation.substeps_completed=501;
 const s=checkpointShape(checkpoint(r));assert.throws(()=>bounded(s),/Unaligned/);
 const p=replacementProjection({active:1000,terminal:5000,shapes:[s],extra:s.bytes});
 assert.equal(p.physicalDispatchAuthorized,false);assert.equal(p.projectedActiveOutputBytes,6000+2*s.bytes+65536);
 assert.throws(()=>checkpointShape(checkpoint(runtime()).subarray(0,-1)));
});
// Deliberately worst-case scripted observer, NOT physics. Mirrors only the
// source-bound ordering/reset/settling proof; all contacts may instantly fail.
function scriptedObserver({complete=false,stable=0,motion='Recovering',settled=.299,successAt=-1,failureAt=-1}){
 let started=true,forecasts=0,resets=0;
 for(let i=0;i<20;i++){
  if(!started&&motion==='Fallen'){forecasts++;break;}
  if(i===failureAt)break;
  if(!started||complete){
   if(['Upright','Reacting','Recovering'].includes(motion)){motion='Falling';settled=0;}
   if(motion==='Falling'){settled=Math.fround(settled+Math.fround(.00125));if(settled>=.3)motion='Fallen';}
  }
  if(complete&&['Falling','Fallen'].includes(motion)){complete=false;started=false;resets++;continue;}
  if(!started)continue;
  if(i===successAt){stable=800;complete=true;settled=0;motion='Upright';}
  else if(!complete)motion='Recovering';
 }
 return {forecasts,resets,started};
}
test('success crossing then immediate fall cannot forecast; completed Falling counterexample can',()=>{
 for(const stable of [779,780,799])for(const successAt of [0,1,19])for(const failureAt of [-1,0,1,2,19]){
  const p=scriptedObserver({stable,successAt,failureAt});assert.equal(p.forecasts,0);assert(p.resets<=1);
 }
 for(const motion of ['Upright','Reacting'])assert.equal(scriptedObserver({complete:true,motion}).forecasts,0);
 assert.equal(scriptedObserver({complete:true,motion:'Falling'}).forecasts,1);
 assert.equal(scriptedObserver({complete:true,motion:'Fallen'}).forecasts,1);
});
