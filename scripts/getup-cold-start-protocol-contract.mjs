// Cold-start uses new checkpoints/data. It cannot satisfy retained-input roles.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {createHash} from 'node:crypto';
const sha=x=>createHash('sha256').update(x).digest('hex');
export function coldSettings(config,candidatePath){
 const input=config.inputContract, cold=config.coldStart;
 assert.equal(cold.schema,'lh-getup-cold-start/v1');
 assert.equal(config.wallSeconds,3600);assert.equal(config.nativeIntegrations,300000);
 assert.equal(config.gamma,Math.exp(-.025/25));
 assert.equal(input.deadlineUnixMs-input.startedAtUnixMs,3600000);
 assert(Number.isSafeInteger(input.startedAtUnixMs)&&Date.now()<input.deadlineUnixMs);
 const proof=JSON.parse(readFileSync(cold.operationalProof));
 assert.equal(proof.schema,'lh-cold-start-operational-inputs/v1');
 assert(proof.passed&&proof.physicsIntegrations===0&&proof.privateDriveRoundtrip===true);
 assert.equal(proof.manifestSha256,sha(readFileSync(cold.manifest)));
 const manifest=JSON.parse(readFileSync(cold.manifest));
 assert.equal(manifest.schema,cold.schema);assert.equal(manifest.synthetic,false);
 for(const role of ['worker_wasm','worker_glue','profile','back_unstepped','belly_unstepped',
   'runtime_unstepped_receipt','cold_start_protocol','source_identity']){
  const file=manifest.files[role];assert(file&&file.sha256===sha(readFileSync(file.path)),`Cold input hash: ${role}`);
  assert.equal(input.files[role].path,file.path);assert.equal(input.files[role].sha256,file.sha256);
 }
 assert.equal(resolve(manifest.files.worker_wasm.path),join(resolve(candidatePath),'lh_worker_bg.wasm'));
 assert.equal(resolve(manifest.files.worker_glue.path),join(resolve(candidatePath),'lh_worker.js'));
 return {config,input,baselinePath:resolve(candidatePath),
  originalPath:input.checkpoints.original,profilePath:manifest.files.profile.path,cold:true};
}
