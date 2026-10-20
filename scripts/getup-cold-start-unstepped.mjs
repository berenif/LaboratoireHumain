// Read/write real WorkerRuntime checkpoints, with strictly ZERO physical steps.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const [workerArg,profileArg,outputArg]=process.argv.slice(2);
assert(workerArg&&profileArg&&outputArg,'worker profile output required');
const worker=resolve(workerArg),output=resolve(outputArg),profile=JSON.parse(readFileSync(profileArg));
mkdirSync(output,{recursive:true});
const m=await import(pathToFileURL(join(worker,'lh_worker.js')));
m.initSync({module:readFileSync(join(worker,'lh_worker_bg.wasm'))});
const hash=x=>createHash('sha256').update(x).digest('hex');
const receipt={schema:'lh-cold-start-unstepped-runtime/v1',physicsIntegrations:0,passed:false,cases:{}};
for(const name of ['back','belly']){
 const runtime=m.WorkerRuntime.getup_fixture_with_profile(name,JSON.stringify(profile));
 runtime.set_root_translation_compensation_enabled(true);
 const before=JSON.parse(runtime.getup_fixture_report());
 assert.equal(before.nativeSubsteps,0);
 const bytes=runtime.getup_checkpoint(),restored=m.WorkerRuntime.from_getup_checkpoint(bytes);
 // Rapier serializes hash maps whose iteration order may change on decode.
 // Archive bytes must roundtrip identically; reserialization is not canonical.
 const reserialized=restored.getup_checkpoint();
 assert.deepEqual(JSON.parse(restored.getup_fixture_report()),before);
 const verified=JSON.parse(runtime.verify_getup_checkpoint(bytes));
 assert(verified.jointStateEqual&&verified.handlesEqual&&verified.snapshotEqual
  &&verified.controllerEqual&&verified.rootIntegratorStateEqual&&verified.physicsIntegrations===0);
 const after=JSON.parse(runtime.getup_fixture_report());assert.deepEqual(after,before);
 writeFileSync(join(output,name+'-unstepped.bin'),bytes);
 receipt.cases[name]={nativeBefore:0,nativeAfter:after.nativeSubsteps,
  checkpointSha256:hash(bytes),checkpointBytes:bytes.length,roundtripStateExact:true,
  reserializationByteIdentical:hash(reserialized)===hash(bytes),verified};
 runtime.free();restored.free();
}
receipt.passed=true;writeFileSync(join(output,'unstepped-runtime.json'),JSON.stringify(receipt,null,2)+'\n');
console.log(JSON.stringify(receipt));
