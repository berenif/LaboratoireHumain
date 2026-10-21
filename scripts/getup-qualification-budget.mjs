// Pure bounded qualification accounting; no WASM/runtime/physics on import.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
export const qualificationPlan=JSON.parse(readFileSync(new URL('./getup-qualification-plan.json',import.meta.url)));
// Frozen profile has recovery_enabled=false. Mirror lib.rs's selector, not a
// controller phase prefix: an awaiting-fall fixture is Falling and uses32.
export function qualificationSelectedPasses(motion,sourceActive){
 assert(['Upright','Reacting','Falling','Fallen','Recovering','Halted'].includes(motion),'Unknown motion');
 assert.equal(typeof sourceActive,'boolean');
 return motion==='Falling'||motion==='Fallen'||motion==='Recovering'&&sourceActive?32:20;
}
function commandSourceActive(getup){
 assert(getup&&getup.policy,'Missing native command observation');
 const {controller,policy}=getup;
 const phases=['awaiting-physical-fall','experimental-joint-balance-awaiting-fall',
  'teacher-reference','slow-following','slow-holding','experimental-joint-balance-holding'];
 assert(phases.includes(policy.phase),'Unreviewed native command phase');
 const started=policy.started_at_s!==null;
 assert(!started||Number.isFinite(policy.started_at_s),'Invalid source start');
 if(controller==='fixture-zero-control-preparation'){
  assert(!started,'Preparation cannot contain active source recovery');return false;
 }
 if(controller==='experimental-recovery-joint-balance-ready'){
  assert(!started&&policy.phase==='experimental-joint-balance-awaiting-fall');return false;
 }
 if(controller==='experimental-recovery-joint-balance-hold'){
  assert(started&&policy.phase==='experimental-joint-balance-holding');return false;
 }
 assert.equal(controller,'pretrained-WEAK-SLOW','Unreviewed command controller');
 return started;
}
export function assertQualificationSolverBudget(budget,{motion,preBudget,nativeTrace,ticks=1,failure=null,firstSourceCall=false}){
 // A physical refusal has primary diagnostic authority, not a later audit.
 // In particular a failing unobserved integration can have changed the current
 // scalar before Halted selectedNext becomes20. Do not infer it from zero rows.
 if(failure!==null)return {scope:'physical-failure-preserved',solverVerified:false};
 assert([20,32].includes(budget.currentInternalPgsIterations),'Unreviewed current solver passes');
 assert.equal(budget.selectedNextInternalPgsIterations,
  qualificationSelectedPasses(motion,budget.sourceRecoveryActive),'Selected-next solver mismatch');
 assert([0,1].includes(ticks)&&Array.isArray(nativeTrace)&&nativeTrace.length===ticks*20,
  'Successful call requires requested ticks worth of complete native rows');
 let expected;
 if(!nativeTrace.length){
  expected=preBudget.currentInternalPgsIterations; // ticks=0 does not integrate.
 }else{
  const previous=nativeTrace.at(-2),last=nativeTrace.at(-1);
  // getup.rs records controller ownership BEFORE physics; policy is refreshed
  // and motion observed AFTER physics. The last command owner and PREVIOUS
  // motion reconstruct the final integration's selector. A controller rearm
  // is separately rejected by the source-epoch/counter guard before this audit.
  expected=qualificationSelectedPasses(previous.motion,commandSourceActive(last.getup));
 }
 assert.equal(budget.currentInternalPgsIterations,expected,'Last-integrated solver mismatch');
 if(firstSourceCall)assert.equal(budget.currentInternalPgsIterations,32,'First WEAK/SLOW call requires32');
 return {scope:'last-integrated-and-selected-next',solverVerified:true,currentExpected:expected};
}
export function validateQualificationGrowthSources(root){
 for(const [path,digest] of Object.entries(qualificationPlan.growth_bound_source_sha256))
  assert.equal(createHash('sha256').update(readFileSync(`${root}/${path}`)).digest('hex'),digest,
   'Frozen geometry/controller serialization source changed: '+path);
 assert.equal(qualificationPlan.next_call_growth_bytes,null);
 assert.equal(qualificationPlan.schema,'lh-corrected-qualification-phases/v2');
 assert.deepEqual(qualificationPlan.growth_components_bytes,
  {controller:12*1024**2,world_contacts:6*1024**2,auxiliary_head:2*1024**2});
 const model=JSON.parse(readFileSync(`${root}/rust/crates/model/data/getup-v1.json`));
 const primitives=model.segments.flatMap(s=>s.reference.primitives);
 assert.equal(model.segments.length,25);assert.equal(primitives.length,19);
 assert(primitives.every(p=>['sphere','capsule'].includes(p.kind)));
 assert.equal(model.segments.filter(s=>s.reference.hinge).length+6,27);
 assert.equal(qualificationPlan.frozen_geometry.maximum_primitive_pairs,26*25/2);
}
export function qualificationByteProjection(active,current,terminal,extra=0,physical=true){
 for(const n of [active,current,terminal,extra])assert(Number.isSafeInteger(n)&&n>=0);
 assert(!physical,'Legacy additive physical bound closed; use replacementCheckpointBound');
 const growth=0;
 const projected=active+current+terminal+extra+growth+65536;
 assert(projected<=qualificationPlan.active_output_bytes,
  `Qualification byte stop BEFORE dispatch: active=${active} terminal=${terminal} currentRuntime=${current} nextGrowth=${growth} extra=${extra} projected=${projected}`);
 return {activeOutputBytes:active,currentRuntimeBytes:current,terminalBaseBytes:terminal,
  nextGrowthBytes:growth,projectedActiveOutputBytes:projected,physicalDispatchAuthorized:physical};
}
export function qualificationLimits(ledger){
 return ledger.qualificationMode==='preservation-only'?qualificationPlan.preservation_only:qualificationPlan;
}
export function enterQualificationStage(ledger,name){
 if(ledger.qualificationMode==='preservation-only')assert(['transition-parity','belly-preservation','heldout-preservation'].includes(name),'No repeated historical preparation/qualification');
 const phase=qualificationPlan.phases[name];assert(phase&&phase.normal>0,'Named physical qualification phase');
 assert(!ledger.pendingCall&&!ledger.qualificationSourcePending&&!ledger.qualificationFailure,'No uncertain/repeated qualification call');
 ledger.qualificationStages??=[];
 assert(!ledger.qualificationStages.some(x=>x.name===name),'No repeated physical phase');
 const stage={name,start:ledger.nativeIntegrations,normalEnd:ledger.nativeIntegrations+phase.normal,
  normalAllowance:phase.normal,forecastAllowance:phase.forecasts,forecasts:0};
 ledger.qualificationStages.push(stage);ledger.qualificationStage=stage;
 return stage;
}
export function guardQualificationCall(ledger){
 const stage=ledger.qualificationStage;
 assert(stage&&!ledger.qualificationFailure&&!ledger.qualificationSourcePending);
 assert(ledger.nativeIntegrations<stage.normalEnd,'Normal phase complete; shared reserve is not another rollout');
 assert(ledger.nativeIntegrations+5020<=qualificationLimits(ledger).maximum_qualification_integrations,'One shared reserve, never per-phase extension');
 assert((ledger.bootstrapDecisions??0)<qualificationLimits(ledger).source_decisions,'Inclusive qualification source-decision cap');
 assert.equal(ledger.evaluationReserveIntegrations,147000);
}
export function beginQualificationSource(ledger){
 assert(!ledger.qualificationSourcePending);
 ledger.qualificationSourcePending=1;
 ledger.trainingDecisions++;ledger.bootstrapDecisions=(ledger.bootstrapDecisions??0)+1;
}
export function finishQualificationSource(ledger,before,after){
 assert.equal(ledger.qualificationSourcePending,1);
 const source=after.sourceDecisions-before.sourceDecisions;
 const weak=after.weakDecisions-before.weakDecisions;
 // Native charge is already reconciled; source uncertainty remains charged if
 // controller counters reset, rather than interpreting reset as negative work.
 assert(Number.isSafeInteger(source)&&source>=0&&Number.isSafeInteger(weak)&&weak>=0,'Controller rearm/counter reset');
 ledger.trainingDecisions+=source-1;ledger.bootstrapDecisions+=source-1;
 ledger.qualificationWeakDecisions=(ledger.qualificationWeakDecisions??0)+weak;
 delete ledger.qualificationSourcePending;
 assert(source<=1&&ledger.bootstrapDecisions<=qualificationLimits(ledger).source_decisions,'Source-decision exposure exceeded frozen cap');
 const forecast=after.forecast-before.forecast,stage=ledger.qualificationStage;
 if(forecast>0)stage.forecasts++;
 assert(forecast>=0&&forecast<=5000&&stage.forecasts<=stage.forecastAllowance,'Unexpected forecast/rearm; stop physics');
 assert(before.sourceEpoch===null||before.sourceEpoch===after.sourceEpoch,'Source epoch rearmed; stop physics');
 assert(before.impactEpoch===after.impactEpoch,'Original impact epoch changed');
 assert(ledger.nativeIntegrations<=stage.normalEnd,'Shared reserve consumed; stop physics');
}

export function retainMeasuredHold(result,writeSummary,completeCheckpoint){
 // Measured samples survive a final-copy/anchor assertion failure. Only the
 // completed callback permits a passed, hash-bound physical proof.
 const pending={...result,passed:false,physicalSamplesChecked:true,checkpointBound:false};
 writeSummary(pending);
 try{
  const digest=completeCheckpoint();assert.equal(typeof digest,'string');assert.equal(digest.length,64);
  const completed={...result,passed:true,checkpointBound:true,finalCheckpointSha256:digest};
  writeSummary(completed);return completed;
 }catch(error){writeSummary({...pending,finalizationError:String(error)});throw error;}
}
