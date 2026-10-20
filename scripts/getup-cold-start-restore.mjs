// Fresh-process operational loading only: no advance/step call is permitted.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {pathToFileURL} from 'node:url';
const [glue,wasm,...args]=process.argv.slice(2);
const option=args.indexOf('--controller');
const checkpoints=option<0?args:args.slice(0,option);
const expected=option<0?null:JSON.parse(readFileSync(args[option+1]));
assert(glue&&wasm&&checkpoints.length);
if(expected){assert.equal(checkpoints.length,1);assert(expected.bridge&&expected.state);}
const module=await import(pathToFileURL(glue));
module.initSync({module:readFileSync(wasm)});
const cases=[];
for(const path of checkpoints){
 const bytes=readFileSync(path),runtime=module.WorkerRuntime.from_getup_checkpoint(bytes);
 try{
  const before=JSON.parse(runtime.getup_fixture_report()),check=JSON.parse(runtime.verify_getup_checkpoint(bytes));
  assert(check.jointStateEqual&&check.handlesEqual&&check.snapshotEqual&&check.controllerEqual
   &&check.rootIntegratorStateEqual&&check.physicsIntegrations===0);
  assert.deepEqual(JSON.parse(runtime.getup_fixture_report()),before);
  if(expected){
   const c=JSON.parse(runtime.getup_adaptation_state());let preview=null;
   try{preview=JSON.parse(runtime.getup_adaptation_preview());}catch{}
   const restoredState={native:before.nativeSubsteps,forecast:c?.status.forecast_substeps??0,status:c?.status,
    fixture:before.state,impactEpoch:c?.impact_substep,sourceEpoch:c?.started_substep,preview,
    current:JSON.parse(runtime.getup_standing_diagnostic('null')),lastCorrection:c?.adaptation?.last??null};
   assert.deepEqual(restoredState,expected.state,'Independent runtime/controller logical state differs');
   // JS reward/hold bookkeeping is serialized separately; retain every field,
   // including the WeakMap snapshot, without inventing another physical call.
   const {label,previousC,longest,interval,protectedEvaluationIntegrations,protectedEvaluationWallMs,lastSnapshot}=structuredClone(expected.bridge);
   assert.equal(typeof label,'string');assert.equal(previousC.length,21);
   assert(previousC.every(Number.isFinite));assert(Number.isSafeInteger(longest)&&Number.isSafeInteger(interval));
   assert(Number.isSafeInteger(protectedEvaluationIntegrations)&&Number.isFinite(protectedEvaluationWallMs));
   assert(lastSnapshot&&Array.isArray(lastSnapshot.segments));
   const snapshots=new WeakMap();snapshots.set(runtime,lastSnapshot);
   assert.deepEqual({label,previousC,longest,interval,protectedEvaluationIntegrations,protectedEvaluationWallMs,
    lastSnapshot:snapshots.get(runtime)},expected.bridge);
  }
  cases.push({native:before.nativeSubsteps,checkpointBytes:bytes.length,passed:true,
   independentControllerStateVerified:!!expected});
 }finally{runtime.free();}
}
console.log(JSON.stringify({passed:true,physics_integrations:0,cases}));
