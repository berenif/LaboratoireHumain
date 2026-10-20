// Validate remotely materialized bytes by loading the actual runtime, no steps.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {pathToFileURL} from 'node:url';
const [glue,wasm,...checkpoints]=process.argv.slice(2);
assert(glue&&wasm&&checkpoints.length);
const module=await import(pathToFileURL(glue));
module.initSync({module:readFileSync(wasm)});
const cases=[];
for(const path of checkpoints){
 const bytes=readFileSync(path),runtime=module.WorkerRuntime.from_getup_checkpoint(bytes);
 const before=JSON.parse(runtime.getup_fixture_report()),check=JSON.parse(runtime.verify_getup_checkpoint(bytes));
 assert(check.jointStateEqual&&check.handlesEqual&&check.snapshotEqual&&check.controllerEqual
  &&check.rootIntegratorStateEqual&&check.physicsIntegrations===0);
 assert.deepEqual(JSON.parse(runtime.getup_fixture_report()),before);
 cases.push({native:before.nativeSubsteps,checkpointBytes:bytes.length,passed:true});
 runtime.free();
}
console.log(JSON.stringify({passed:true,physics_integrations:0,cases}));
