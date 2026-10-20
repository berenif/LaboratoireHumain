import {checkColdWorkerCapabilities} from './getup-cold-worker-contract.mjs';
// Actual WASM only. JSONL bridge for one pre-frozen local bounded experiment.
import {readFileSync,writeFileSync,appendFileSync,mkdirSync,readdirSync,statSync,existsSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import {createInterface} from 'node:readline';
import assert from 'node:assert/strict';
import {appendInterval} from './getup-wasm-adaptation-math.mjs';
import {protocolSettings,guardProtocolCall,beginProtocolCall,finishProtocolCall,rowsThroughNative} from './getup-learning-loop-protocol-contract.mjs';
import {validateColdQualificationClaim} from './getup-cold-start-protocol-contract.mjs';
import {qualificationPlan,enterQualificationStage,guardQualificationCall,beginQualificationSource,finishQualificationSource,validateQualificationGrowthSources,qualificationByteProjection,assertQualificationSolverBudget} from './getup-qualification-budget.mjs';
const root=process.cwd(), output=resolve(process.argv[2]);
const candidatePath=resolve(process.argv[3]);
const {config,input:inputContract,baselinePath,originalPath,profilePath}=protocolSettings(output,candidatePath);
const qualificationOnly=config.qualificationOnly===true;
if(qualificationOnly){assert.deepEqual(config.qualificationPlan,qualificationPlan);validateQualificationGrowthSources(root);}
const profile=JSON.parse(readFileSync(profilePath));
const hash=x=>createHash('sha256').update(x).digest('hex');
const load=async path=>{let m=await import(pathToFileURL(join(path,'lh_worker.js')).href);m.initSync({module:readFileSync(join(path,'lh_worker_bg.wasm'))});return m;};
const priorLedgerPath=join(output,'budget-ledger.json');
let priorLedger;try{priorLedger=JSON.parse(readFileSync(priorLedgerPath));}catch{}
let continuationClaim=null;
if(inputContract&&priorLedger){
 assert(config.coldStart,'No corrected-loop restart or ledger reuse');
 continuationClaim=JSON.parse(readFileSync(join(output,'continuation-claim.json')));
 assert.equal(continuationClaim.ledgerSha256,hash(readFileSync(priorLedgerPath)));
 const amendmentBytes=readFileSync(continuationClaim.amendmentPath);
 assert.equal(continuationClaim.amendmentSha256,hash(amendmentBytes));
 validateColdQualificationClaim(continuationClaim,JSON.parse(amendmentBytes),priorLedger,config,Date.now());
 writeFileSync(join(output,'continuation-bridge-consumed.json'),JSON.stringify({
  amendmentSha256:continuationClaim.amendmentSha256,originalLedgerSha256:continuationClaim.ledgerSha256,
  consumedAt:new Date().toISOString()}),{flag:'wx'});
}
const candidate=await load(candidatePath), baseline=config.coldStart?candidate:await load(baselinePath);
if(config.coldStart)checkColdWorkerCapabilities(JSON.parse(candidate.getup_build_capabilities()));
const ledger=priorLedger??{schema:1,startedAt:new Date(inputContract?.startedAtUnixMs??Date.now()).toISOString(),deadlineUnixMs:inputContract?.deadlineUnixMs??Date.now()+config.wallSeconds*1000,
 nativeIntegrations:0,realIntegrations:0,forecastIntegrations:0,completedCalls:0,uncertainCalls:[],limit:config.nativeIntegrations,
 trainingDecisions:0,releaseAccepted:false};
const save=()=>writeFileSync(join(output,'budget-ledger.json'),JSON.stringify(ledger,null,2)+'\n');save();
let protectedEvaluationIntegrations=0,protectedEvaluationWallMs=0;
let usefulAdmission=null,limitedPhysicalCalls=0;
let commandName=null;
const qualificationRuntimes=new Set();
let qualificationTerminalBase=null;
const track=w=>{if(qualificationOnly)qualificationRuntimes.add(w);return w;};
const free=w=>{qualificationRuntimes.delete(w);w.free();};
function activeOutputBytes(path=output){
 return readdirSync(path,{withFileTypes:true}).reduce((n,entry)=>{
  if(path===output&&entry.name==='recovery')return n;
  const p=join(path,entry.name);return n+(entry.isDirectory()?activeOutputBytes(p):statSync(p).size);
 },0);
}
function qualificationStorageGuard(extraOutput=0,physical=true){
 if(!qualificationOnly)return;
 assert(Number.isSafeInteger(qualificationTerminalBase)&&qualificationTerminalBase>=config.qualificationStorage.untouchedLearnerBytes+2*1024**2);
 const runtimes=[...qualificationRuntimes].map(w=>w.getup_checkpoint().length);
 const current=runtimes.reduce((a,b)=>a+b,0);
 const measured=activeOutputBytes();
 // Frozen model shape audit:12MiB controller +6MiB world/contact +2MiB
 // other auxiliary/head growth for ONE20-native/5000-forecast call.
 return qualificationByteProjection(measured,current,qualificationTerminalBase,extraOutput,physical);
}
function writeCheckpoint(path,bytes){
 if(qualificationOnly)qualificationStorageGuard(Math.max(0,bytes.length-(existsSync(path)?statSync(path).size:0)),false);
 writeFileSync(path,bytes);
}
const qualificationCounters=w=>{const c=report(w);return {sourceDecisions:c?.status.policy_decisions??0,
 weakDecisions:c?.status.teacher_decisions??0,forecast:c?.status.forecast_substeps??0,
 sourceEpoch:c?.started_substep??null,impactEpoch:c?.impact_substep??null};};
const guard=(reserve=5020)=>{if(inputContract)return guardProtocolCall(ledger,protectedEvaluationIntegrations,protectedEvaluationWallMs,Date.now());assert(Date.now()+protectedEvaluationWallMs<ledger.deadlineUnixMs,'Experiment60minute wall cap');assert(ledger.nativeIntegrations+reserve+protectedEvaluationIntegrations<=ledger.limit,'Aggregate integration cap');};
const report=w=>JSON.parse(w.getup_adaptation_state?w.getup_adaptation_state():w.getup_report());
const fixture=w=>JSON.parse(w.getup_fixture_report());
const call=(w,ticks=1,{firstSourceCall=false}={})=>{
 track(w);
 if(qualificationOnly)qualificationStorageGuard();
 if(continuationClaim){
  assert(usefulAdmission&&limitedPhysicalCalls===0,'One limited physical initialization only');
  assert(Date.now()<usefulAdmission.deadlineUnixMs,'Limited operation wall deadline');
  assert(ledger.nativeIntegrations+5020<=usefulAdmission.maximumCumulativeIntegrations);
  limitedPhysicalCalls++;
 }
 if(qualificationOnly)guardQualificationCall(ledger);
 guard();const beforeFixture=fixture(w),before=beforeFixture.nativeSubsteps, bf=report(w)?.status.forecast_substeps??0;
 const qualificationBefore=qualificationOnly?qualificationCounters(w):null;
 if(inputContract){beginProtocolCall(ledger,before,bf);save();}
 if(qualificationOnly){beginQualificationSource(ledger);save();}
 let result;const advanceStarted=performance.now();
 try{result=JSON.parse(w.advance_terminal_tail_observed(ticks));}
 catch(error){ledger.uncertainCalls.push({at:new Date().toISOString(),chargedUpperBound:5020,error:String(error)});if(!inputContract)ledger.nativeIntegrations+=5020;save();throw error;}
 const after=fixture(w).nativeSubsteps,af=report(w)?.status.forecast_substeps??0;
 const realDelta=after>=before?after-before:20,forecastDelta=af>=bf?af-bf:5000;
 if(continuationClaim)ledger.limitedCallTiming={
  advanceAndStateReadWallSeconds:(performance.now()-advanceStarted)/1000,
  realIntegrations:realDelta,forecastIntegrations:forecastDelta,
  category:forecastDelta?'mixed-controlled-and-forecast':'controlled-only',
  separateForecastDurationMeasured:false};
 if(inputContract)finishProtocolCall(ledger,after,af);
 else{ledger.realIntegrations+=realDelta;ledger.forecastIntegrations+=forecastDelta;ledger.nativeIntegrations+=realDelta+forecastDelta;ledger.completedCalls++;}save();
 // Known completed state must remain capturable even if a later route/budget
 // assertion rejects this first reset. Never obtain it with another step.
 lastSnapshots.set(w,result.reply.snapshot);
 if(qualificationOnly){
  const f=fixture(w),status=report(w)?.status;
  const audit={command:commandName,preNative:before,postNative:after,
   before:qualificationBefore,after:qualificationCounters(w),
   nativeTraceSha256:hash(JSON.stringify(result.nativeTrace)),nativeSamples:result.nativeTrace.length,
   failure:result.reply.failure??null,phase:status?.phase,solverBudget:f.solverBudget,
   motion:result.reply.snapshot.motion,preSolverBudget:beforeFixture.solverBudget,
   solverAuditInputs:{ticks,nativeRows:result.nativeTrace.length,firstSourceCall,
    precedingMotion:result.nativeTrace.at(-2)?.motion??null,
    lastCommandController:result.nativeTrace.at(-1)?.getup?.controller??null,
    lastPolicyPhase:result.nativeTrace.at(-1)?.getup?.policy?.phase??null,
    lastStartedAtSeconds:result.nativeTrace.at(-1)?.getup?.policy?.started_at_s??null,
    scope:result.reply.failure?'physical-failure-preserved':'last-integrated-and-selected-next'}};
  appendFileSync(join(output,'qualification-native.ndjson'),JSON.stringify(audit)+'\n');
  try{
   finishQualificationSource(ledger,qualificationBefore,audit.after);
   assert(!String(status?.phase).startsWith('physical-roll'),'Forbidden physical-roll route');
   assertQualificationSolverBudget(f.solverBudget,{motion:result.reply.snapshot.motion,
    preBudget:beforeFixture.solverBudget,nativeTrace:result.nativeTrace,ticks,
    failure:result.reply.failure??null,firstSourceCall});
  }
  catch(error){ledger.qualificationFailure=String(error);save();throw error;}
  save();
 }
 assert(after>=before&&af>=bf,'Physical/reference counter reset during a bounded single-cycle call');
 assert(!result.traceError,result.traceError);return result;
};
const lastSnapshots=new WeakMap();
const success=w=>report(w)?.status.prototype_standing_passed===true;
const preview=w=>{try{return JSON.parse(w.getup_adaptation_preview());}catch{return null;}};
function state(w){const f=fixture(w),c=report(w),d=JSON.parse(w.getup_standing_diagnostic('null'));
 return {native:f.nativeSubsteps,forecast:c?.status.forecast_substeps??0,status:c?.status,fixture:f.state,
  impactEpoch:c?.impact_substep,sourceEpoch:c?.started_substep,
  preview:preview(w),current:d,lastCorrection:c?.adaptation?.last??null};}
function potential(d){const p=d.predicates;
 const support=Math.min(1,Math.max(0,d.loadsN.reduce((a,b)=>a+b,0)/(40.84401321411133*9.81)));
 const foot=Math.min(1,Math.max(0,d.footIds.reduce((a,i)=>a+d.loadsN[i],0)/(40.84401321411133*9.81)));
 const upright=Math.max(0,Math.min(1,(d.torsoUp+1)/2));
 const head=Math.max(0,Math.min(1,d.headM/1.4));
 const maxv=Math.max(...d.motion.map(x=>x.linearMps));const maxw=Math.max(...d.motion.map(x=>x.angularRadps));
 const quiet=1/(1+maxv+maxw);
 return support*(.25*upright+.25*head+.25*foot+.25*quiet)*(p.onlyFeet?1:.5);
}
let active=null,label=null,previousC=Array(21).fill(0),longest=0,interval=0;
async function command(q){
 commandName=q.op;
 if(q.op==='qualificationStorage'){
  assert(qualificationOnly&&Number.isSafeInteger(q.terminalBaseBytes));
  qualificationTerminalBase=q.terminalBaseBytes;
  return qualificationStorageGuard(0,false);
 }
 if(q.op==='qualificationCapture'){
  assert(qualificationOnly);const files=[];
  qualificationStorageGuard(0,false);
  const unresolved=!!(ledger.pendingCall||ledger.uncertainCalls.length||ledger.uncertainIntegrations);
  for(const [index,w] of [...qualificationRuntimes].entries()){
   if(w===active&&!unresolved)continue;
   const path=join(output,`qualification-runtime-${index}.bin`),bytes=w.getup_checkpoint();
   // The complete tracked-runtime sum was already reserved above. Retire this
   // temporary comparator only after retaining its exact bytes.
   writeFileSync(path,bytes);files.push({path,sha256:hash(bytes),bytes:bytes.length,
    uncertain:w===active&&unresolved});
   if(w!==active)free(w);
  }
  return {physicsIntegrations:0,files,activeOutputBytes:activeOutputBytes(),
   activeKnown:!!active&&!unresolved,state:active&&!unresolved?state(active):null};
 }
 if(q.op==='qualificationStage'){
  assert(qualificationOnly);const value=enterQualificationStage(ledger,q.name);save();return value;
 }
 if(q.op==='admitUseful'){
  assert(continuationClaim&&!usefulAdmission&&limitedPhysicalCalls===0);
  assert.equal(q.maximumIntegrations,5020);
  assert(q.deadlineUnixMs>Date.now()&&q.deadlineUnixMs<=Date.now()+120000);
  assert(q.deadlineUnixMs<=ledger.deadlineUnixMs-protectedEvaluationWallMs);
  usefulAdmission={deadlineUnixMs:q.deadlineUnixMs,maximumCumulativeIntegrations:ledger.nativeIntegrations+5020};
  return usefulAdmission;
 }
 if(continuationClaim&&['coldPrepare','coldQualify','step','hold','heldout','qualify'].includes(q.op))
  assert.fail('Qualification amendment admits only one bootstrap reset, no other physics');
 if(q.op==='coldDrop'){assert(config.coldStart);if(active)free(active);active=null;label=null;previousC=Array(21).fill(0);longest=0;interval=0;return {physicsIntegrations:0};}
 if(q.op==='state'){assert(active);return state(active);}
 if(q.op==='bridgeState')return {label,previousC,longest,interval,protectedEvaluationIntegrations,protectedEvaluationWallMs,
  lastSnapshot:active?lastSnapshots.get(active)??null:null};
 if(q.op==='standingReceipt'){assert(active);const c=JSON.parse(active.getup_report());return {native:fixture(active).nativeSubsteps,stableNative:c.stable_substeps,anchors:c.stable_feet};}
 if(q.op==='reserve'){assert(Number.isSafeInteger(q.integrations)&&q.integrations>=0&&q.integrations<=ledger.limit);assert(Number.isFinite(q.wallSeconds)&&q.wallSeconds>=0);protectedEvaluationIntegrations=q.integrations;protectedEvaluationWallMs=q.wallSeconds*1000;ledger.evaluationReserveIntegrations=q.integrations;ledger.evaluationReserveWallSeconds=q.wallSeconds;save();return {protectedEvaluationIntegrations,protectedEvaluationWallMs};}
 if(q.op==='headparity'){return {correction:JSON.parse(candidate.getup_correction_head(JSON.stringify(q.head),JSON.stringify(q.features))),physicsIntegrations:0};}
 if(q.op==='identity')return {candidateWasmSha256:hash(readFileSync(join(candidatePath,'lh_worker_bg.wasm'))),baselineWasmSha256:hash(readFileSync(join(baselinePath,'lh_worker_bg.wasm'))),ledger};
 if(q.op==='coldPrepare'){
  assert(config.coldStart&&!active,'Separate cold-start preparation only');
  assert.equal(ledger.trainingDecisions,0);assert.equal(ledger.nativeIntegrations,0,'No preparation replay/restart');
  const result={schema:'lh-cold-fixtures/v1',historicalRecoveryClaimed:false,cases:{}};
  for(const name of ['original','heldout','belly']){
   const role=name==='belly'?'belly_unstepped':'back_unstepped';
   const w=track(candidate.WorkerRuntime.from_getup_checkpoint(readFileSync(inputContract.files[role].path)));
   const target=name==='original'?480:name==='heldout'?560:null;
   do{assert(fixture(w).nativeSubsteps+20<=(target??2000),'Preparation cap BEFORE dispatch; no2020 overshoot');
    const r=call(w);assert(!r.reply.failure,JSON.stringify(r.reply.failure));
    assert(fixture(w).nativeSubsteps<=2000,'Cold fixture preparation cap; no retry');
   }while(target?fixture(w).nativeSubsteps<target:!fixture(w).state.ready);
   const f=fixture(w),s=f.state;
   assert(s.preparing&&s.ready,'Fresh fixture readiness, no synthetic pose injection');
   if(name!=='belly')assert(s.torso_face_up>=.8&&Math.abs(s.torso_long_axis_up)<=.35&&s.head_height_m<=.5
    &&s.ground_load_n>=.8*f.massKg*9.81&&s.maximum_body_com_speed_mps<=.22&&s.maximum_angular_radps<=.65,
    'Fresh one-shot BACK admission failed; no retry');
   if(name==='heldout')assert(JSON.stringify(f.jointQ)!==JSON.stringify(result.cases.original.fixture.jointQ)
    ||JSON.stringify(f.jointDq)!==JSON.stringify(result.cases.original.fixture.jointDq),'Nearby state must differ');
   w.trigger_getup_fixture();call(w,0);
   const cp=w.getup_checkpoint();writeCheckpoint(inputContract.checkpoints[name],cp);
   result.cases[name]={fixture:f,checkpointSha256:hash(cp),checkpointBytes:cp.length,policyPerformanceObserved:false};
   free(w);
  }
  writeFileSync(join(output,'cold-fixtures.json'),JSON.stringify(result,null,2)+'\n');return result;
 }
 if(q.op==='coldQualify'){
  assert(config.coldStart&&!active);assert.equal(ledger.trainingDecisions,0);
  const seed=readFileSync(inputContract.checkpoints.original);
  const runs=[track(candidate.WorkerRuntime.from_getup_checkpoint(seed)),track(candidate.WorkerRuntime.from_getup_checkpoint(seed))];
  runs[1].set_getup_adaptation_head(JSON.stringify({weights:Array(21*1024).fill(0),bias:Array(21).fill(0)}));
  let resumed=null,samples=0,resumedSamples=0;
  const started=performance.now();
  for(let i=0;i<20;i++){
   const packets=runs.map(w=>call(w,1,{firstSourceCall:i===0}));assert.deepEqual(packets[0].nativeTrace,packets[1].nativeTrace,'Zero correction changed actual physical trace');
   if(qualificationOnly){
    packets.forEach(packet=>assert(!packet.reply.failure,JSON.stringify(packet.reply.failure)));
    for(const w of runs){
     const c=report(w);
     assert.equal(c.impact_substep,480,'Original BACK impact epoch preserved');
     assert.equal(c.started_substep,480,'Original BACK must enter WEAK/reference then SLOW without preparation route');
     assert(c.status.forecast_substeps>0&&c.status.reference_frames>0&&c.status.reference_seed_matches_real);
     assert(!c.status.phase.startsWith('physical-roll'));
    }
   }
   samples+=packets[0].nativeTrace.length;
   if(resumed){const packet=call(resumed);assert.deepEqual(packet.nativeTrace,packets[1].nativeTrace,'Actual next-state checkpoint parity');resumedSamples+=packet.nativeTrace.length;}
   if(i===9)resumed=track(candidate.WorkerRuntime.from_getup_checkpoint(runs[1].getup_checkpoint()));
  }
  runs.forEach(free);free(resumed);
  const result={passed:true,physicsIntegrations:ledger.nativeIntegrations,zeroCorrectionSamples:samples,
   resumedSamples,wallSeconds:(performance.now()-started)/1000,historicalBaselineClaimed:false};
  writeFileSync(join(output,'cold-adapter-parity.json'),JSON.stringify(result,null,2)+'\n');return result;
 }
 if(q.op==='coldRestore'){
  assert(config.coldStart);assert(q.bridge&&q.path);
  const bytes=readFileSync(q.path);assert.equal(hash(bytes),q.sha256);
  const restored=track(candidate.WorkerRuntime.from_getup_checkpoint(bytes));
  const check=JSON.parse(restored.verify_getup_checkpoint(bytes));
  assert(check.jointStateEqual&&check.handlesEqual&&check.snapshotEqual&&check.controllerEqual&&check.rootIntegratorStateEqual);
  if(active)free(active);active=restored;
  ({label,previousC,longest,interval,protectedEvaluationIntegrations,protectedEvaluationWallMs}=q.bridge);
  if(q.bridge.lastSnapshot)lastSnapshots.set(active,q.bridge.lastSnapshot);
  return {state:state(active),physicsIntegrations:0,ledger};
 }
 if(q.op==='qualify'){
  assert(!inputContract,'Corrected loop must reuse verified qualification; no extra collection');
  const seed=readFileSync(originalPath);assert.equal(hash(seed),config.originalCheckpointSha256);
  const runs=[baseline.WorkerRuntime.from_getup_checkpoint(seed),candidate.WorkerRuntime.from_getup_checkpoint(seed),candidate.WorkerRuntime.from_getup_checkpoint(seed)];
  runs[2].set_getup_adaptation_head(JSON.stringify({weights:Array(21*1024).fill(0),bias:Array(21).fill(0)}));
  const activeCheckpoint=runs[2].getup_checkpoint();
  assert.equal(new DataView(activeCheckpoint.buffer,activeCheckpoint.byteOffset,4).getUint32(0,true),4,'Active correction must use explicit outer schema4');
  assert.throws(()=>baseline.WorkerRuntime.from_getup_checkpoint(activeCheckpoint),'Older baseline must reject active correction checkpoint');
  let restored=null,compared=0;const began=performance.now();
  for(let i=0;i<config.parityDecisions;i++){
   const packets=runs.map(w=>call(w));if(restored){const rp=call(restored);assert.deepEqual(rp.nativeTrace,packets[2].nativeTrace,'Checkpoint resumed next physical state differs');}
   const shared=rows=>rows.map(({motorApplication,integrityLimits,footFloorClearanceM,...physical})=>physical);
   assert.deepEqual(shared(packets[0].nativeTrace),shared(packets[1].nativeTrace),'Current correction-disabled WASM differs from frozen baseline shared physical contract');
   assert.deepEqual(packets[1].nativeTrace,packets[2].nativeTrace,'Zero head differs from correction-disabled WASM');
   compared+=packets[0].nativeTrace.length;
   if(i===9){const cp=runs[2].getup_checkpoint();writeFileSync(join(output,'parity-decision-checkpoint.bin'),cp);
    const check=JSON.parse(runs[2].verify_getup_checkpoint(cp));assert(check.snapshotEqual&&check.controllerEqual&&check.rootIntegratorStateEqual);
    restored=candidate.WorkerRuntime.from_getup_checkpoint(cp);}
  }
  const pendingRuntime=candidate.WorkerRuntime.from_getup_checkpoint(runs[2].getup_checkpoint());
  pendingRuntime.set_getup_adaptation_head('null');pendingRuntime.queue_getup_correction(JSON.stringify(Array(21).fill(0)));
  const pendingCp=pendingRuntime.getup_checkpoint(),pendingCheck=JSON.parse(pendingRuntime.verify_getup_checkpoint(pendingCp));
  assert(pendingCheck.controllerEqual&&pendingCheck.snapshotEqual&&pendingCheck.rootIntegratorStateEqual);
  pendingRuntime.free();
  const result={passed:true,legacyBaselineRejectsActiveSchema4:true,pendingCorrectionCheckpointExact:true,comparedNativeSamples:compared,parityDecisions:config.parityDecisions,
   checkpointResumedSamples:Math.max(0,config.parityDecisions-10)*20,wallSeconds:(performance.now()-began)/1000,ledger:{...ledger}};
  writeFileSync(join(output,'adapter-parity.json'),JSON.stringify(result,null,2)+'\n');runs.forEach(w=>w.free());restored?.free();return result;
 }
 if(q.op==='heldout'){
  assert(!inputContract,'Retained heldout is mandatory; no substitute generation');
  assert(!readFileSync(join(output,'frozen-config.json')).includes('REPLACE'));
  const w=candidate.WorkerRuntime.getup_fixture_with_profile('back',JSON.stringify(profile));w.set_root_translation_compensation_enabled(true);
  for(let n=0;n<560;n+=20){const r=call(w);assert(!r.reply.failure,JSON.stringify(r.reply.failure));}
  const f=fixture(w),s=f.state;assert.equal(f.nativeSubsteps,560);
  assert(s.preparing&&s.ready&&s.torso_face_up>=.8&&Math.abs(s.torso_long_axis_up)<=.35&&s.head_height_m<=.5
   &&s.ground_load_n>=.8*f.massKg*9.81&&s.maximum_body_com_speed_mps<=.22&&s.maximum_angular_radps<=.65,'One-shot heldout BACK admission failed, no retry');
  const original=candidate.WorkerRuntime.from_getup_checkpoint(readFileSync(originalPath));const o=fixture(original);
  const different=JSON.stringify(f.jointQ)!==JSON.stringify(o.jointQ)||JSON.stringify(f.jointDq)!==JSON.stringify(o.jointDq);
  assert(different,'Heldout physical q/dq identical; no retry');
  const before=w.getup_checkpoint();w.trigger_getup_fixture();call(w,0);const cp=w.getup_checkpoint();
  writeFileSync(join(output,'heldout-back-560-before-trigger.bin'),before);writeFileSync(join(output,'heldout-back-560.bin'),cp);
  const result={admitted:true,narrowHoldout:'same initial construction, distinct naturally settled physical state at560 instead480, no broad generalization',
   physicalQorDqDifferent:different,fixture:f,checkpointSha256:hash(cp),policyPerformanceObserved:false};
  writeFileSync(join(output,'heldout-admission.json'),JSON.stringify(result,null,2)+'\n');w.free();original.free();return result;
 }
 if(q.op==='reset'){
  if(continuationClaim)assert(q.case==='belly'&&q.external===true&&q.prepared===true&&limitedPhysicalCalls===0);
  assert(['original','heldout','belly'].includes(q.case));
  if(active)free(active);const path=inputContract?inputContract.checkpoints[q.case]:q.case==='heldout'?join(output,'heldout-back-560.bin'):q.case==='belly'?join(output,'belly-seed.bin'):originalPath;
  if(q.case==='belly'&&!q.prepared&&!inputContract){const w=candidate.WorkerRuntime.getup_fixture_with_profile('belly',JSON.stringify(profile));w.set_root_translation_compensation_enabled(true);
   while(!fixture(w).state.ready){for(let i=0;i<4;i++){const r=call(w);assert(!r.reply.failure);}assert(fixture(w).nativeSubsteps<=2000);}
   w.trigger_getup_fixture();call(w,0);writeFileSync(path,w.getup_checkpoint());w.free();}
  active=track(candidate.WorkerRuntime.from_getup_checkpoint(readFileSync(path)));
  label=q.label;previousC=Array(21).fill(0);longest=0;interval=0;
  // First20native are original actor for all branches. No stale latched observation is offered to learner.
  const r=call(active);assert(!r.reply.failure,JSON.stringify(r.reply.failure));
  if(inputContract){const measured=appendInterval({longest,tail:interval},r.nativeTrace);longest=measured.longest;interval=measured.tail;}
  if(q.head)active.set_getup_adaptation_head(JSON.stringify(q.head));else if(q.external)active.set_getup_adaptation_head('null');
  return state(active);
 }
 if(q.op==='step'){
  if(inputContract&&q.training)assert(ledger.trainingDecisions<4096,'Fresh decision cap');
  if(config.coldStart&&q.training){
   assert(['bootstrap','iterative'].includes(q.collectionPhase),'Explicit cold decision allocation');
   const key=q.collectionPhase==='bootstrap'?'bootstrapDecisions':'iterativeDecisions';
   assert((ledger[key]??0)<(q.collectionPhase==='bootstrap'?1024:3072),'Cold phase decision cap');
  }
  assert(active);const pre=state(active);
  if(inputContract&&q.training)assert(pre.preview&&q.c&&q.episodeId&&q.policyId,'Fresh decision requires current preview/action/episode/policy identity');
  if(inputContract&&q.c)assert(pre.preview,'No new correction outside a source boundary');
  let c=q.c??Array(21).fill(0);if(q.c&&pre.preview)active.queue_getup_correction(JSON.stringify(c));
  const packet=call(active),post=state(active);
  const failure=packet.reply.failure??(!success(active)&&post.status.since_impact_s>=25?{check:"original25s-deadline",native:post.native}:null);
  if(!q.c&&post.lastCorrection?.native_substep===pre.native)c=post.lastCorrection.correction;
  const terminal=success(active)||!!failure;const phi0=potential(pre.current),phi1=terminal?0:potential(post.current);
  const energy=c.reduce((a,b)=>a+b*b,0)/21,slew=c.reduce((a,b,i)=>a+(b-previousC[i])**2,0)/21;
  const reward=100*Number(success(active)&&!failure)-100*Number(!!failure)+2*(config.gamma*phi1-phi0)-.0025-.01*energy-.005*slew;
  const measured=appendInterval({longest,tail:interval},packet.nativeTrace);longest=measured.longest;interval=measured.tail;
  const provenance={label,episodeId:q.episodeId??label,behaviorPolicyId:q.policyId??null,impactEpoch:pre.impactEpoch,sourceEpoch:pre.sourceEpoch,
   prePhase:pre.status.phase,preNative:pre.native,postNative:post.native,preObservation:pre.preview?.observation??null,
   postObservation:post.preview?.observation??null,referenceDecision:pre.preview?.referenceDecision??null,correction:c,
   appliedResidual:post.lastCorrection?.applied_residual??report(active).residual,originalResidual:pre.preview?.originalResidual??null,
   postPhysicalWords:packet.nativeTrace.at(-1)?.physicalObservationWords,postPoseWords:packet.nativeTrace.at(-1)?.poseWords,
   intervalPredicateWords:packet.nativeTrace.map(row=>['torso','bothFeetLoadedAndPersistent','footLoadFraction','onlyFeet','linearMotion','angularMotion'].reduce((mask,key,i)=>mask|(Number(row.current.predicates[key])<<i),0)),
   reward,terminal,success:success(active),failure,status:post.status};
  if(config.coldStart){
   const featureHash=values=>hash(Buffer.from(new Float32Array(values??Array(1024).fill(0)).buffer));
   provenance.sourceFeaturesSha256=featureHash(pre.preview?.features);
   provenance.nextFeaturesSha256=featureHash(post.preview?.features);
   provenance.training=q.training===true;provenance.collectionPhase=q.collectionPhase??'evaluation';
   provenance.nativeTraceSha256=hash(JSON.stringify(packet.nativeTrace));
  }
  appendFileSync(join(output,'transitions.ndjson'),JSON.stringify(provenance)+'\n');
  previousC=c;if(q.training&&!qualificationOnly){ledger.trainingDecisions++;if(config.coldStart){const key=q.collectionPhase==='bootstrap'?'bootstrapDecisions':'iterativeDecisions';ledger[key]=(ledger[key]??0)+1;}}save();
  return {state:post,reward,terminal,success:success(active),failure,longestMeasuredInterval:longest,native:post.native,
   ...(config.coldStart?{nativeTraceSha256:provenance.nativeTraceSha256}:{})};
 }
 if(q.op==='hold'){
  assert(active&&success(active),'Hold requires original800-sample success');
  const initialController=JSON.parse(active.getup_report()),initialNative=fixture(active).nativeSubsteps;
  const diagnosticDeadline=initialController.impact_substep+20000;
  const originalAnchors=initialController.stable_feet;assert(originalAnchors);
  if(inputContract){assert(initialController.stable_substeps>=800);assert(q.firstSuccessReceipt,'First800 success receipt is required');
   assert.equal(initialController.stable_substeps-q.firstSuccessReceipt.stableNative,initialNative-q.firstSuccessReceipt.native,'Original standing counter reset after first success');
   assert.deepEqual(originalAnchors,q.firstSuccessReceipt.anchors,'Original anchors changed after first success');}
  if(inputContract)writeCheckpoint(join(output,label+'-first800-checkpoint.bin'),active.getup_checkpoint());
  const initial=inputContract?lastSnapshots.get(active):call(active,0).reply.snapshot;assert(initial,'Retained latest snapshot required');
  const diagnostic=JSON.parse(active.getup_standing_diagnostic(JSON.stringify(originalAnchors)));
  const xyz=p=>[p.x,p.y,p.z],sub=(a,b)=>a.map((v,i)=>v-b[i]);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const rotate=(q,v)=>{const a=q.slice(0,3),c=cross(a,v),cc=cross(a,c);return v.map((x,i)=>x+2*(q[3]*c[i]+cc[i]));};
  const material=[];
  for(const foot of diagnostic.contactMotion.feet){const id=diagnostic.footIds.find(i=>initial.segments[i].id===foot.body)??(foot.body==='left_foot'?diagnostic.footIds[0]:diagnostic.footIds[1]);
   const pose=initial.segments[id],r=[pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w];
   for(const contact of foot.contacts.filter(c=>c.normalImpulseNs>0))material.push({id,world:xyz(contact.point),local:rotate([-r[0],-r[1],-r[2],r[3]],sub(xyz(contact.point),xyz(pose.position)))});
  }
  assert(material.length>=2,'Both feet need retained material contacts');
  const bits=new DataView(new ArrayBuffer(4));const float=w=>{bits.setUint32(0,w,true);return bits.getFloat32(0,true);};
  let maximumOriginDriftM=0,maximumMaterialSlipM=0,samples=0,checkedSamples=0,receivedSamples=0;
  let holdError=null;
  try{for(let tick=0;tick<1200;tick++){
   const beforeNative=fixture(active).nativeSubsteps;
   const packet=call(active);assert(!packet.reply.failure,JSON.stringify(packet.reply.failure));
   receivedSamples+=packet.nativeTrace.length;
   if(inputContract){const measured=appendInterval({longest,tail:interval},rowsThroughNative(packet.nativeTrace,beforeNative,diagnosticDeadline));longest=measured.longest;interval=measured.tail;}
   for(const row of packet.nativeTrace){
    checkedSamples++;
    assert(Object.entries(row.current.predicates).filter(([k])=>k!=='footDriftFromSuppliedAnchors').every(([,v])=>v===true),'Original standing predicate failed in30s hold');
    row.current.feet.forEach((foot,i)=>{const d=Math.hypot(...sub(xyz(foot),xyz(originalAnchors[i])));maximumOriginDriftM=Math.max(maximumOriginDriftM,d);assert(d<=.01,'Original10mm origin-anchor hold failed');});
    for(const anchor of material){const words=row.poseWords.slice(anchor.id*13,anchor.id*13+7).map(float),position=words.slice(0,3),r=words.slice(3,7);
     const point=rotate(r,anchor.local).map((x,i)=>x+position[i]);const slip=Math.hypot(point[0]-anchor.world[0],point[2]-anchor.world[2]);
     maximumMaterialSlipM=Math.max(maximumMaterialSlipM,slip);assert(slip<=.02,'Separate20mm material-point slip hold failed');}
    samples++;
   }
  }
  }catch(error){holdError=error;}
  if(holdError){
   const partial={passed:false,fullyPassedSamples:samples,checkedSamples,receivedSamples,
    maximumOriginDriftM,maximumMaterialSlipM,originalAnchors,
    initialNative,currentNative:fixture(active).nativeSubsteps,error:String(holdError),
    evidence:'Full-sample checks up to failure; remaining native samples in the failing packet are not claimed checked',
    materialMetric:'maximum anchored material-point excursion, not cumulative travel'};
   writeFileSync(join(output,label+'-30s-hold.json'),JSON.stringify(partial,null,2)+'\n');
   throw holdError;
  }
  const finalController=JSON.parse(active.getup_report()),finalNative=fixture(active).nativeSubsteps;
  if(inputContract){assert.equal(finalNative-initialNative,24000);assert.equal(finalController.stable_substeps-initialController.stable_substeps,24000,'Original native standing counter reset during hold');assert.deepEqual(finalController.stable_feet,originalAnchors);}
  const result={passed:true,samples,physicalSeconds:30,maximumOriginDriftM,maximumMaterialSlipM,
   diagnosticEndpointNative:inputContract?diagnosticDeadline:null,longestMeasuredInterval:longest,diagnosticTail:interval,
   materialMetric:'maximum anchored material-point excursion, not cumulative travel',
   originalStableNativeBefore:initialController.stable_substeps,originalStableNativeAfter:finalController.stable_substeps,
   originAnchorSource:'original controller stable_feet, preserved at first800-native success',materialAnchorSource:'positive-impulse foot material points at first800-native success'};
  if(inputContract){const bytes=active.getup_checkpoint();writeCheckpoint(join(output,label+'-30s-hold-checkpoint.bin'),bytes);result.finalCheckpointSha256=hash(bytes);}
  writeFileSync(join(output,label+'-30s-hold.json'),JSON.stringify(result,null,2)+'\n');return result;
 }
 if(q.op==='checkpoint'){assert(active);const bytes=active.getup_checkpoint();
  if(qualificationOnly&&q.name==='phase-runtime.bin'){
   qualificationStorageGuard(0,false);writeFileSync(join(output,q.name),bytes);
  }else writeCheckpoint(join(output,q.name),bytes);
  return {sha256:hash(bytes),bytes:bytes.length};}
 if(q.op==='ledger')return ledger;
 throw Error('Unknown op');
}
for await(const line of createInterface({input:process.stdin,crlfDelay:Infinity})){
 try{const q=JSON.parse(line),r=await command(q);process.stdout.write(JSON.stringify({ok:true,result:r})+'\n');}
 catch(error){save();process.stdout.write(JSON.stringify({ok:false,error:String(error),stack:error.stack,ledger})+'\n');}
}
active?.free();save();
