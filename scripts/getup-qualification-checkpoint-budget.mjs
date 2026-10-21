// Serialized-output admission only: this is NOT a transient heap/RSS bound.
// No WASM construction, physics, forecast or filesystem mutation here.
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
export const MiB=1024**2;
const integer=n=>Number.isSafeInteger(n)&&n>=0;
export function checkpointShape(input){
 const b=Buffer.from(input);assert(b.length<=64*MiB&&b.length>=28);
 let at=4;const schema=b.readUInt32LE(0);assert([1,2,4].includes(schema));
 const part=()=>{assert(at+8<=b.length);const n=Number(b.readBigUInt64LE(at));at+=8;
  assert(Number.isSafeInteger(n)&&n>=0&&at+n<=b.length);const x=b.subarray(at,at+n);at+=n;return x;};
 const model=part(),world=part(),outer=part();assert.equal(at,b.length);
 assert(world.length<=6*MiB,'Current serialized world exceeds reviewed complete-world envelope');
 let raw=outer.toString('utf8'),s=schema;
 if(s===4){const w=JSON.parse(raw);assert.equal(w.format,'frozen-slow-correction-v1');
  assert([1,2].includes(w.inner_schema));s=w.inner_schema;raw=w.auxiliary;}
 if(s===2){const w=JSON.parse(raw);assert(typeof w.runtime==='string'&&Array.isArray(w.roots));raw=w.runtime;}
 const runtime=JSON.parse(raw),sim=runtime.simulation,c=sim.getup_controller;
 const profile=sim.profile;
 assert(profile&&typeof profile==='object');
 const profileCertified=profile.id==='getup-800hz-3plus1-repeat-cycle-v1'&&profile.dt_s===.00125&&
  profile.tick_substeps===20&&profile.getup_repeated_recovery==='completed-cycle-confirmed-fall-v1'&&
  profile.recovery_enabled===false&&profile.getup_policy_enabled===true;
 assert(c&&integer(sim.substeps_completed)&&Array.isArray(c.student_decisions));
 assert(runtime.mode==='protocol'&&typeof runtime.disposed==='boolean');
 assert(runtime.timeline&&integer(runtime.timeline.generation)&&runtime.timeline.generation===sim.generation);
 assert(runtime.timeline.queue&&typeof runtime.timeline.queue==='object'&&!Array.isArray(runtime.timeline.queue));
 assert(['Upright','Reacting','Falling','Fallen','Recovering','Halted'].includes(sim.motion));
 assert(typeof c.status.prototype_standing_passed==='boolean'&&integer(c.stable_substeps));
 assert(c.started_substep===null||integer(c.started_substep));
 assert(Array.isArray(c.reference)&&c.reference.length<=501&&Array.isArray(c.teacher_frames)&&c.teacher_frames.length<=250&&c.student_decisions.length<=1100);
 const pending=c.adaptation?.pending;
 if(pending!==null&&pending!==undefined)assert(pending.native_substep===sim.substeps_completed&&
  Array.isArray(pending.correction)&&pending.correction.length===21&&pending.correction.every(Number.isFinite),
  'Pending correction must remain finite and bound to current native counter');
 // Normalize BOTH branches to the same two string-escaping layers. A null
 // adaptation head still activates schema4. Never subtract a schema2 size
 // from a schema4 envelope.16KiB separately covers root/envelope sidecars.
 const normalizedRuntimeBytes=Buffer.byteLength(JSON.stringify(JSON.stringify(raw)));
 return {schema,bytes:b.length,modelBytes:model.length,modelSha256:createHash('sha256').update(model).digest('hex'),worldBytes:world.length,
  normalizedRuntimeBytes,native:sim.substeps_completed,source:c.started_substep,
  stable:c.stable_substeps,completed:c.status.prototype_standing_passed,
  studentRecords:c.student_decisions.length,forecast:c.status.forecast_substeps,
  referenceEntries:c.reference.length,motion:sim.motion,generation:sim.generation,
  lifecycleQueueEntries:Object.keys(runtime.timeline.queue).length,disposed:runtime.disposed,
  noFailure:Object.hasOwn(sim,'first_failure')&&sim.first_failure===null,
  pendingCorrection:pending!==null&&pending!==undefined,profileCertified,
  profileSha256:createHash('sha256').update(JSON.stringify(profile)).digest('hex')};
}
export function replacementCheckpointBound(shape,{ticks=1,certifiedLineage=null}={}){
 assert([0,1].includes(ticks));
 assert(shape.lifecycleQueueEntries===0&&!shape.disposed,'Queued lifecycle/disposed runtime cannot be dispatched');
 if(ticks===0)return {bytes:shape.bytes,branch:'zero-tick-exact-current',noReforecast:false};
 if(shape.source!==null){assert(Number.isSafeInteger(shape.source)&&shape.source<=shape.native);
  assert.equal((shape.native-shape.source)%20,0,'Unaligned source archive cannot use one-decision bound');
  assert(integer(shape.stable)&&shape.stable<=shape.native-shape.source,'Stable count exceeds source lineage');}
 if(certifiedLineage!==null)assert(shape.profileCertified&&certifiedLineage.profileSha256===shape.profileSha256&&
  certifiedLineage.modelSha256===shape.modelSha256&&
  certifiedLineage.generation===shape.generation&&certifiedLineage.profile==='getup-800hz-3plus1-repeat-cycle-v1',
  'Runtime model/generation/profile lineage changed');
 let branch='retained-or-replacement-reference';
 const established=certifiedLineage!==null&&shape.source!==null&&shape.referenceEntries>0&&shape.noFailure;
 if(established&&shape.completed===false&&shape.stable+20<800)branch='incomplete-below-success-threshold';
 // On first success the observer resets both fall timers and sets Upright.
 // On a completed Upright/Reacting -> Falling transition it resets settled_s.
 // At most20*1.25ms cannot reach the0.3s Fallen threshold. Rearm may replace
 // Controller with an EMPTY Self::new, but cannot generate another reference.
 else if(established&&shape.completed===false)branch='success-crossing-settling-barrier';
 else if(established&&shape.completed===true&&['Upright','Reacting'].includes(shape.motion))
  branch='completed-upright-settling-barrier';
 const noReforecast=branch!=='retained-or-replacement-reference';
 const retained=shape.normalizedRuntimeBytes+24*1024+2*MiB+16*1024;
 const fresh=12*MiB+2*MiB+16*1024;
 // Observe's rearm replaces Controller with Self::new. It does not append a
 // second teacher history. Changed observation/status/terminal/first-control,
 // pending/receipt/head fields are included in the separate2MiB mutation cap.
 // Empty Controller:33 fields (16 controller+17 status),42 f32 slots and
 // bounded phase/refusal strings give raw<=7780B; two escaping layers<=31126B.
 // Round to64KiB and ADD to full retained runtime (no semantic subtraction).
 const emptyController=64*1024;
 const emptyReplacement=shape.normalizedRuntimeBytes+emptyController+2*MiB+16*1024;
 const auxiliary=noReforecast?(branch==='incomplete-below-success-threshold'?retained:
  Math.max(retained,emptyReplacement)):Math.max(retained,fresh);
 return {bytes:28+shape.modelBytes+6*MiB+auxiliary,
  branch,
  noReforecast,normalizedRetainedBytes:retained,freshReferenceBytes:fresh,
  emptyControllerBytes:emptyController,emptyReplacementBytes:emptyReplacement,
  completeWorldBytes:6*MiB,alignedStudentAppendBytes:24*1024,wrapperSidecarBytes:16*1024,
  certificateInputs:{source:shape.source,native:shape.native,stable:shape.stable,completed:shape.completed,
   motion:shape.motion,referenceEntries:shape.referenceEntries,noFailure:shape.noFailure,
   lifecycleQueueEntries:shape.lifecycleQueueEntries,generation:shape.generation,modelSha256:shape.modelSha256,
   profileSha256:shape.profileSha256}};
}
export function replacementProjection({active,terminal,shapes,advanced=-1,ticks=1,lineage=null,extra=0,limit=32*MiB}){
 for(const n of [active,terminal,extra])assert(Number.isSafeInteger(n)&&n>=0);
 assert(Number.isInteger(advanced)&&advanced>=-1&&advanced<shapes.length);
 const current=shapes.reduce((n,s)=>n+s.bytes,0);
 const replacement=advanced<0?null:replacementCheckpointBound(shapes[advanced],{ticks,certifiedLineage:lineage});
 const unchanged=current-(advanced<0?0:shapes[advanced].bytes);
 const projected=active+terminal+unchanged+(replacement?.bytes??0)+extra+65536;
 const proof={activeOutputBytes:active,terminalBaseBytes:terminal,currentRuntimeBytes:current,
  unchangedRuntimeBytes:unchanged,replacement,projectedActiveOutputBytes:projected,
  physicalDispatchAuthorized:advanced>=0&&projected<=limit,serializedOutputOnly:true};
 if(projected>limit){const error=new Error(`Qualification byte stop BEFORE dispatch: active=${active} terminal=${terminal} unchangedRuntime=${unchanged} replacement=${replacement?.bytes??0} extra=${extra} projected=${projected}`);
  error.projection=proof;throw error;}
 return proof;
}
