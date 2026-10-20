// Cold-start uses new checkpoints/data. It cannot satisfy retained-input roles.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {createHash} from 'node:crypto';
const sha=x=>createHash('sha256').update(x).digest('hex');
export function validateColdQualificationClaim(claim,amendment,ledger,config,now){
 assert.equal(claim.schema,'lh-cold-qualification-claim/v1');
 assert.equal(amendment.schema,'lh-cold-qualification-amendment/v1');
 assert.equal(amendment.boundary,'after-coldQualify-before-first-bootstrap-reset');
 assert.equal(claim.runId,amendment.runId);
 assert.equal(claim.maximumNewIntegrations,5020);assert.equal(claim.maximumPhysicalCalls,1);
 assert.equal(claim.operation,'bootstrap-belly-initialization');
 assert.equal(amendment.limitedUsefulOperationOnly,true);
 assert.equal(amendment.learnerSeed,20261020);assert.equal(amendment.genericResumeAuthorized,false);
 assert.equal(claim.originalDeadlineUnixMs,config.inputContract.deadlineUnixMs);
 assert.equal(ledger.deadlineUnixMs,claim.originalDeadlineUnixMs);
 assert.equal(Math.round(amendment.deadlineUnix*1000),ledger.deadlineUnixMs);
 assert.equal(Math.round(amendment.startedAtUnix*1000),config.inputContract.startedAtUnixMs);
 assert(now<ledger.deadlineUnixMs);assert.equal(amendment.deadlineUnix-amendment.startedAtUnix,3600);
 for(const [key,value] of Object.entries({nativeIntegrations:2380,realIntegrations:2380,
  forecastIntegrations:0,trainingDecisions:0,completedCalls:122,uncertainIntegrations:0,
  limit:300000,evaluationReserveIntegrations:147000}))assert.equal(ledger[key],value,key);
 assert(!ledger.pendingCall);assert.deepEqual(ledger.uncertainCalls,[]);
 assert.equal(ledger.bootstrapDecisions??0,0);assert.equal(ledger.iterativeDecisions??0,0);
}
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
