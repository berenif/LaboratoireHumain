// Pure file/config validation. No WASM import, Runtime, deadline or ledger starts here.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {resolve,join,dirname} from 'node:path';
import {createHash} from 'node:crypto';
const roles=['learning_audit','provenance_audit','replay','replay_metadata','original_seed',
 'nearby_seed','belly_seed','worker_wasm','worker_glue','baseline_wasm','baseline_glue',
 'profile','baseline_receipt','transitions'];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
export function rowsThroughNative(rows,beforeNative,deadlineNative){
 assert(Number.isSafeInteger(beforeNative)&&Number.isSafeInteger(deadlineNative));
 return rows.slice(0,Math.max(0,Math.min(rows.length,deadlineNative-beforeNative)));
}
export function guardProtocolCall(ledger,evaluationIntegrations,evaluationWallMs,now){
 assert(!ledger.pendingCall,'Prior uncertain call is terminal; no retry');
 assert(Number.isSafeInteger(ledger.nativeIntegrations)&&ledger.nativeIntegrations>=0);
 assert.equal(ledger.limit,300000);
 assert(now+evaluationWallMs<ledger.deadlineUnixMs,'Original absolute deadline/reserved wall');
 assert(ledger.nativeIntegrations+5020+evaluationIntegrations<=ledger.limit,'Aggregate integration cap');
}
export function beginProtocolCall(ledger,native,forecast){
 assert(!ledger.pendingCall);ledger.pendingCall={native,forecast,chargedUpperBound:5020};
 ledger.nativeIntegrations+=5020;ledger.uncertainIntegrations=(ledger.uncertainIntegrations??0)+5020;
}
export function finishProtocolCall(ledger,native,forecast){
 const pending=ledger.pendingCall;assert(pending);
 const realDelta=native-pending.native,forecastDelta=forecast-pending.forecast;
 assert(Number.isSafeInteger(realDelta)&&realDelta>=0&&Number.isSafeInteger(forecastDelta)&&forecastDelta>=0);
 assert(realDelta+forecastDelta<=5020,'Native call exceeded its conservatively charged maximum');
 ledger.nativeIntegrations+=realDelta+forecastDelta-5020;ledger.uncertainIntegrations-=5020;
 ledger.realIntegrations+=realDelta;ledger.forecastIntegrations+=forecastDelta;ledger.completedCalls++;
 delete ledger.pendingCall;
}
export function protocolSettings(output,candidatePath){
 const config=JSON.parse(readFileSync(join(output,'frozen-config.json')));
 const input=config.inputContract;
 if(!input)return {config,input:null,
  baselinePath:resolve('outputs/browser-capability-20261019/recovery-fixture/terminal-tail-ab/build/app-baseline-unadapted'),
  originalPath:resolve('outputs/browser-capability-20261019/recovery-fixture/teacher-feasibility/actual-back-teacher/teacher-export-before-checkpoint.bin'),
  profilePath:resolve('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json')};
 assert.equal(config.wallSeconds,3600);assert.equal(config.nativeIntegrations,300000);
 assert.equal(config.gamma,Math.exp(-.025/25));
 assert(Number.isSafeInteger(input.startedAtUnixMs));assert.equal(input.deadlineUnixMs-input.startedAtUnixMs,3600000);
 assert(Date.now()<input.deadlineUnixMs,'Original deadline expired before WASM import');
 for(const role of roles){
  const item=input.files?.[role];assert(item&&typeof item.path==='string'&&/^[a-f0-9]{64}$/.test(item.sha256),`Missing exact input ${role}`);
  assert.equal(sha(readFileSync(item.path)),item.sha256,`Input hash mismatch ${role}`);
 }
 const path=role=>resolve(input.files[role].path);
 assert.equal(path('worker_wasm'),join(resolve(candidatePath),'lh_worker_bg.wasm'));
 assert.equal(path('worker_glue'),join(resolve(candidatePath),'lh_worker.js'));
 assert.equal(path('baseline_glue'),join(dirname(path('baseline_wasm')),'lh_worker.js'));
 for(const [name,role] of [['original','original_seed'],['heldout','nearby_seed'],['belly','belly_seed']])
  assert.equal(resolve(input.checkpoints?.[name]??''),path(role));
 assert.equal(resolve(input.profile??''),path('profile'));
 assert.equal(resolve(input.baselineArtifactDirectory??''),dirname(path('baseline_wasm')));
 return {config,input,baselinePath:dirname(path('baseline_wasm')),originalPath:path('original_seed'),profilePath:path('profile')};
}
