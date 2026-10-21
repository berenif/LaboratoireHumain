import test from 'node:test';
import assert from 'node:assert/strict';
import {checkpointShape,replacementCheckpointBound,replacementProjection,MiB} from './getup-qualification-checkpoint-budget.mjs';
const runtime=()=>({simulation:{substeps_completed:500,getup_controller:{started_substep:480,
 stable_substeps:0,student_decisions:[],status:{prototype_standing_passed:false,forecast_substeps:1020}}}});
function checkpoint(r,schema=2,head=null){
 const inner=JSON.stringify({runtime:JSON.stringify(r),roots:[0,1,2]});
 const aux=schema===4?JSON.stringify({format:'frozen-slow-correction-v1',inner_schema:2,auxiliary:inner,head}):inner;
 const parts=[Buffer.from('{}'),Buffer.alloc(100),Buffer.from(aux)];
 const out=[Buffer.from([schema,0,0,0])];for(const p of parts){const n=Buffer.alloc(8);n.writeBigUInt64LE(BigInt(p.length));out.push(n,p);}return Buffer.concat(out);
}
test('schema2 and null-head schema4 normalize both alternatives to identical escaping depth',()=>{
 const a=checkpointShape(checkpoint(runtime(),2)),b=checkpointShape(checkpoint(runtime(),4));
 assert(b.bytes>a.bytes);assert.equal(a.normalizedRuntimeBytes,b.normalizedRuntimeBytes);
 assert.equal(replacementCheckpointBound(a).bytes,replacementCheckpointBound(b).bytes);
});
test('replacement is max retained-or-new history, not incremental fresh forecast',()=>{
 const shape=checkpointShape(checkpoint(runtime()));
 const small=replacementCheckpointBound(shape);assert.equal(small.bytes,28+shape.modelBytes+20*MiB+16384);
 const large={...shape,normalizedRuntimeBytes:13*MiB};const bound=replacementCheckpointBound(large);
 assert.equal(bound.bytes,28+shape.modelBytes+21*MiB+24576+16384);
 assert(bound.bytes<large.normalizedRuntimeBytes+20*MiB);
});
test('certified lineage only excludes reforecast through the400th real sample',()=>{
 const shape=checkpointShape(checkpoint(runtime()));
 assert(replacementCheckpointBound(shape,{coldParityLineage:true}).noReforecast);
 assert(!replacementCheckpointBound(shape).noReforecast);
 assert(replacementCheckpointBound({...shape,native:860},{coldParityLineage:true}).noReforecast);
 for(const change of [{native:880},{completed:true},{stable:21},{forecast:0},{source:460}])
  assert(!replacementCheckpointBound({...shape,...change},{coldParityLineage:true}).noReforecast);
 assert.throws(()=>replacementCheckpointBound({...shape,native:501}),/Unaligned/);
 assert.throws(()=>replacementCheckpointBound({...shape,source:501}));
});
test('advanced runtime is replaced while retained disk copies and other runtimes remain additive',()=>{
 const shape=checkpointShape(checkpoint(runtime()));const options={active:2000,terminal:5000,shapes:[shape,shape],advanced:0,lineage:true};
 const p=replacementProjection(options);
 assert.equal(p.unchangedRuntimeBytes,shape.bytes);
 assert.equal(p.projectedActiveOutputBytes,7000+shape.bytes+p.replacement.bytes+65536);
 const retained=replacementProjection({...options,active:2000+shape.bytes});
 assert.equal(retained.projectedActiveOutputBytes-p.projectedActiveOutputBytes,shape.bytes);
 assert.throws(()=>replacementProjection({...options,limit:p.projectedActiveOutputBytes-1}),/BEFORE dispatch/);
 assert.equal(replacementProjection({...options,ticks:0}).replacement.bytes,shape.bytes);
});
test('untracked checkpoint writes reserve exact bytes; malformed tuple rejected without native calls',()=>{
 const shape=checkpointShape(checkpoint(runtime()));
 const p=replacementProjection({active:1000,terminal:5000,shapes:[shape],extra:shape.bytes});
 assert.equal(p.physicalDispatchAuthorized,false);assert.equal(p.projectedActiveOutputBytes,6000+2*shape.bytes+65536);
 assert.throws(()=>checkpointShape(checkpoint(runtime()).subarray(0,-1)));
});
