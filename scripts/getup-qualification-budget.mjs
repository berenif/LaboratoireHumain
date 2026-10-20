// Pure bounded qualification accounting; no WASM/runtime/physics on import.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
export const qualificationPlan=JSON.parse(readFileSync(new URL('./getup-qualification-plan.json',import.meta.url)));
export function assertQualificationSolverBudget(budget,phase,firstSourceCall=false){
 if(firstSourceCall||budget.sourceRecoveryActive)assert.equal(budget.currentInternalPgsIterations,32);
 if(String(phase).startsWith('experimental-joint-balance'))assert.equal(budget.currentInternalPgsIterations,20);
}
export function validateQualificationGrowthSources(root){
 for(const [path,digest] of Object.entries(qualificationPlan.growth_bound_source_sha256))
  assert.equal(createHash('sha256').update(readFileSync(`${root}/${path}`)).digest('hex'),digest,
   'Frozen geometry/controller serialization source changed: '+path);
 assert.equal(qualificationPlan.next_call_growth_bytes,20*1024**2);
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
 const growth=physical?qualificationPlan.next_call_growth_bytes:0;
 const projected=active+current+terminal+extra+growth+65536;
 assert(projected<=qualificationPlan.active_output_bytes,
  `Qualification byte stop BEFORE dispatch: active=${active} terminal=${terminal} currentRuntime=${current} nextGrowth=${growth} extra=${extra} projected=${projected}`);
 return {activeOutputBytes:active,currentRuntimeBytes:current,terminalBaseBytes:terminal,
  nextGrowthBytes:growth,projectedActiveOutputBytes:projected,physicalDispatchAuthorized:physical};
}
export function enterQualificationStage(ledger,name){
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
 assert(ledger.nativeIntegrations+5020<=qualificationPlan.maximum_qualification_integrations,'One shared reserve, never per-phase extension');
 assert((ledger.bootstrapDecisions??0)<qualificationPlan.source_decisions,'Inclusive qualification source-decision cap');
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
 assert(source<=1&&ledger.bootstrapDecisions<=1024,'Source-decision exposure exceeded frozen cap');
 const forecast=after.forecast-before.forecast,stage=ledger.qualificationStage;
 if(forecast>0)stage.forecasts++;
 assert(forecast>=0&&forecast<=5000&&stage.forecasts<=stage.forecastAllowance,'Unexpected forecast/rearm; stop physics');
 assert(before.sourceEpoch===null||before.sourceEpoch===after.sourceEpoch,'Source epoch rearmed; stop physics');
 assert(before.impactEpoch===after.impactEpoch,'Original impact epoch changed');
 assert(ledger.nativeIntegrations<=stage.normalEnd,'Shared reserve consumed; stop physics');
}
