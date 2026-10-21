// Serialized-output admission only: this is NOT a transient heap/RSS bound.
// No WASM construction, physics, forecast or filesystem mutation here.
import assert from 'node:assert/strict';
export const MiB=1024**2;
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
 assert(c&&Number.isSafeInteger(sim.substeps_completed)&&Array.isArray(c.student_decisions));
 // Normalize BOTH branches to the same two string-escaping layers. A null
 // adaptation head still activates schema4. Never subtract a schema2 size
 // from a schema4 envelope.16KiB separately covers root/envelope sidecars.
 const normalizedRuntimeBytes=Buffer.byteLength(JSON.stringify(JSON.stringify(raw)));
 return {schema,bytes:b.length,modelBytes:model.length,worldBytes:world.length,
  normalizedRuntimeBytes,native:sim.substeps_completed,source:c.started_substep,
  stable:c.stable_substeps,completed:c.status.prototype_standing_passed===true,
  studentRecords:c.student_decisions.length,forecast:c.status.forecast_substeps};
}
export function replacementCheckpointBound(shape,{ticks=1,coldParityLineage=false}={}){
 assert([0,1].includes(ticks));
 if(ticks===0)return {bytes:shape.bytes,branch:'zero-tick-exact-current',noReforecast:false};
 if(shape.source!==null){assert(Number.isSafeInteger(shape.source)&&shape.source<=shape.native);
  assert.equal((shape.native-shape.source)%20,0,'Unaligned source archive cannot use one-decision bound');}
 const elapsed=shape.source===null?null:shape.native-shape.source;
 const noReforecast=coldParityLineage===true&&shape.source===480&&elapsed>=0&&elapsed+20<=400
  &&!shape.completed&&Number.isSafeInteger(shape.stable)&&shape.stable>=0&&shape.stable<=elapsed
  &&shape.forecast>0;
 const retained=shape.normalizedRuntimeBytes+24*1024+2*MiB+16*1024;
 const fresh=12*MiB+2*MiB+16*1024;
 // Observe's rearm replaces Controller with Self::new. It does not append a
 // second teacher history. Changed observation/status/terminal/first-control,
 // pending/receipt/head fields are included in the separate2MiB mutation cap.
 const auxiliary=noReforecast?retained:Math.max(retained,fresh);
 return {bytes:28+shape.modelBytes+6*MiB+auxiliary,
  branch:noReforecast?'certified-first400-no-reforecast':'retained-or-replacement-reference',
  noReforecast,normalizedRetainedBytes:retained,freshReferenceBytes:fresh,
  completeWorldBytes:6*MiB,alignedStudentAppendBytes:24*1024,wrapperSidecarBytes:16*1024};
}
export function replacementProjection({active,terminal,shapes,advanced=-1,ticks=1,lineage=false,extra=0,limit=32*MiB}){
 for(const n of [active,terminal,extra])assert(Number.isSafeInteger(n)&&n>=0);
 assert(Number.isInteger(advanced)&&advanced>=-1&&advanced<shapes.length);
 const current=shapes.reduce((n,s)=>n+s.bytes,0);
 const replacement=advanced<0?null:replacementCheckpointBound(shapes[advanced],{ticks,coldParityLineage:lineage});
 const unchanged=current-(advanced<0?0:shapes[advanced].bytes);
 const projected=active+terminal+unchanged+(replacement?.bytes??0)+extra+65536;
 assert(projected<=limit,`Qualification byte stop BEFORE dispatch: active=${active} terminal=${terminal} unchangedRuntime=${unchanged} replacement=${replacement?.bytes??0} extra=${extra} projected=${projected}`);
 return {activeOutputBytes:active,terminalBaseBytes:terminal,currentRuntimeBytes:current,
  unchangedRuntimeBytes:unchanged,replacement,projectedActiveOutputBytes:projected,
  physicalDispatchAuthorized:advanced>=0,serializedOutputOnly:true};
}
