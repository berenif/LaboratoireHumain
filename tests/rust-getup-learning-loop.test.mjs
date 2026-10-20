import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtempSync,mkdirSync,writeFileSync,readFileSync,existsSync,rmSync} from 'node:fs';
import {join} from 'node:path';
import {tmpdir} from 'node:os';
import {createHash} from 'node:crypto';
import {protocolSettings,guardProtocolCall,beginProtocolCall,finishProtocolCall,rowsThroughNative} from '../scripts/getup-learning-loop-protocol-contract.mjs';
const roles=['learning_audit','provenance_audit','replay','replay_metadata','original_seed','nearby_seed',
 'belly_seed','worker_wasm','worker_glue','baseline_wasm','baseline_glue','profile','baseline_receipt','transitions'];
function fixture(){
 const root=mkdtempSync(join(tmpdir(),'getup-contract-synthetic-')),worker=join(root,'worker'),baseline=join(root,'baseline');
 mkdirSync(worker);mkdirSync(baseline);
 const files={};
 for(const role of roles){
  const path=role==='worker_wasm'?join(worker,'lh_worker_bg.wasm'):role==='worker_glue'?join(worker,'lh_worker.js'):
   role==='baseline_wasm'?join(baseline,'lh_worker_bg.wasm'):role==='baseline_glue'?join(baseline,'lh_worker.js'):join(root,role);
  writeFileSync(path,`Synthetic config bytes only: ${role}`);
  files[role]={path,sha256:createHash('sha256').update(readFileSync(path)).digest('hex')};
 }
 const now=Date.now(),config={wallSeconds:3600,nativeIntegrations:300000,gamma:Math.exp(-.025/25),
  inputContract:{files,startedAtUnixMs:now,deadlineUnixMs:now+3600000,profile:files.profile.path,baselineArtifactDirectory:baseline,
   checkpoints:{original:files.original_seed.path,heldout:files.nearby_seed.path,belly:files.belly_seed.path}}};
 const save=()=>writeFileSync(join(root,'frozen-config.json'),JSON.stringify(config));save();
 return {root,worker,config,save,close:()=>rmSync(root,{recursive:true})};
}
test('retained protocol settings validate bytes before any runtime or ledger',()=>{
 const f=fixture();try{
  const settings=protocolSettings(f.root,f.worker);assert.equal(settings.originalPath,f.config.inputContract.checkpoints.original);
  assert.equal(settings.baselinePath,f.config.inputContract.baselineArtifactDirectory);
  assert(!existsSync(join(f.root,'budget-ledger.json')));
 }finally{f.close();}
});
test('every missing or changed role rejects before ledger/runtime startup',()=>{
 const f=fixture();try{
  for(const role of roles){
   const item=f.config.inputContract.files[role];delete f.config.inputContract.files[role];f.save();
   assert.throws(()=>protocolSettings(f.root,f.worker),/Missing exact input/);
   f.config.inputContract.files[role]={...item,sha256:'0'.repeat(64)};f.save();
   assert.throws(()=>protocolSettings(f.root,f.worker),/Input hash mismatch/);
   f.config.inputContract.files[role]=item;
  }
  assert(!existsSync(join(f.root,'budget-ledger.json')));
 }finally{f.close();}
});
test('expired epoch or nonretained seed path is rejected before WASM import',()=>{
 const f=fixture();try{
  f.config.inputContract.checkpoints.heldout=f.config.inputContract.checkpoints.original;f.save();
  assert.throws(()=>protocolSettings(f.root,f.worker));
  f.config.inputContract.checkpoints.heldout=f.config.inputContract.files.nearby_seed.path;
  f.config.inputContract.startedAtUnixMs=0;f.config.inputContract.deadlineUnixMs=3600000;f.save();
  assert.throws(()=>protocolSettings(f.root,f.worker),/deadline expired/);
 }finally{f.close();}
});
function ledger(){return {nativeIntegrations:1000,realIntegrations:1000,forecastIntegrations:0,completedCalls:0,limit:300000,deadlineUnixMs:3600000};}
test('an in-flight call is conservatively charged before execution and blocks retry',()=>{
 const l=ledger();guardProtocolCall(l,147000,1000,0);beginProtocolCall(l,480,0);
 assert.equal(l.nativeIntegrations,6020);assert.equal(l.uncertainIntegrations,5020);
 assert.throws(()=>guardProtocolCall(l,147000,1000,0),/no retry/);
 finishProtocolCall(l,500,960);
 assert.equal(l.nativeIntegrations,1980);assert.equal(l.realIntegrations,1020);assert.equal(l.forecastIntegrations,960);
 assert.equal(l.uncertainIntegrations,0);assert.equal(l.completedCalls,1);
});
test('counter faults retain the conservative charge rather than undercounting',()=>{
 for(const [native,forecast] of [[479,0],[500,5001]]){
  const l=ledger();beginProtocolCall(l,480,0);assert.throws(()=>finishProtocolCall(l,native,forecast));
  assert.equal(l.nativeIntegrations,6020);assert(l.pendingCall);
 }
});
test('protected final integrations and wall are admitted under the same hard cap',()=>{
 const l=ledger();l.nativeIntegrations=148000;
 assert.throws(()=>guardProtocolCall(l,147000,0,0),/integration cap/);
 l.nativeIntegrations=0;assert.throws(()=>guardProtocolCall(l,147000,1000,3599000),/reserved wall/);
 assert.throws(()=>guardProtocolCall(l,0,0,3600000),/deadline/);
});
test('fixed25 diagnostic excludes every extra hold native after its exact endpoint',()=>{
 const rows=Array.from({length:20},(_,i)=>i);
 assert.deepEqual(rowsThroughNative(rows,20540,20560),rows);
 assert.deepEqual(rowsThroughNative(rows,20560,20560),[]);
 assert.deepEqual(rowsThroughNative(rows,20550,20560),rows.slice(0,10));
});
