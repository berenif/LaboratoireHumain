import test from 'node:test';
import assert from 'node:assert/strict';
import {enterQualificationStage,guardQualificationCall,beginQualificationSource,finishQualificationSource,qualificationPlan as p} from './getup-qualification-budget.mjs';
import {beginProtocolCall,finishProtocolCall} from './getup-learning-loop-protocol-contract.mjs';
const ledger=()=>({nativeIntegrations:0,realIntegrations:0,forecastIntegrations:0,
 trainingDecisions:0,bootstrapDecisions:0,completedCalls:0,uncertainIntegrations:0,evaluationReserveIntegrations:147000});
const counters=(source=0,forecast=0)=>({sourceDecisions:source,weakDecisions:forecast/20,
 forecast,sourceEpoch:480,impactEpoch:480});
test('qualification normal totals and one shared reserve are immutable',()=>{
 assert.equal(Object.values(p.phases).reduce((n,x)=>n+x.normal,0),137060);
 assert.equal(p.normal_integrations+p.shared_uncertainty_reserve,142080);
 assert(142080+147000<300000);assert.equal(p.optimizer_updates,0);
 const l=ledger();enterQualificationStage(l,'original-diagnostic');
 l.nativeIntegrations=l.qualificationStage.normalEnd-20;
 guardQualificationCall(l);beginProtocolCall(l,0,0);beginQualificationSource(l);
 finishProtocolCall(l,20,5000);
 assert.throws(()=>finishQualificationSource(l,counters(),counters(1,5000)),/reserve consumed/);
 assert.equal(l.nativeIntegrations,30000); // 5000 beyond the25000 normal bound, charged.
});
test('forecast at limit and late hold rearm stop without refund or retry',()=>{
 const l=ledger();enterQualificationStage(l,'belly-preservation');
 l.nativeIntegrations=43940;guardQualificationCall(l);
 beginProtocolCall(l,100,500);beginQualificationSource(l);finishProtocolCall(l,120,5500);
 assert.throws(()=>finishQualificationSource(l,counters(5,500),counters(6,5500)),/Unexpected forecast/);
 assert.equal(l.nativeIntegrations,48960);
});
test('uncertain call retains native and source maximum exposure',()=>{
 const l=ledger();enterQualificationStage(l,'transition-parity');guardQualificationCall(l);
 beginProtocolCall(l,100,0);beginQualificationSource(l);
 assert.equal(l.nativeIntegrations,5020);assert.equal(l.bootstrapDecisions,1);
 assert.throws(()=>guardQualificationCall(l));assert.throws(()=>enterQualificationStage(l,'transition-parity'));
});
test('source cap includes qualification parity while WEAK is separate',()=>{
 const l=ledger();enterQualificationStage(l,'zero-correction-qualification');guardQualificationCall(l);
 beginProtocolCall(l,480,0);beginQualificationSource(l);finishProtocolCall(l,500,5000);
 finishQualificationSource(l,{...counters(),sourceEpoch:null},counters(1,5000));
 assert.equal(l.bootstrapDecisions,1);assert.equal(l.qualificationWeakDecisions,250);
 l.bootstrapDecisions=1024;assert.throws(()=>guardQualificationCall(l),/source-decision/);
});
import {qualificationByteProjection,validateQualificationGrowthSources,assertQualificationSolverBudget} from './getup-qualification-budget.mjs';
test('early valid terminal handoff uses20; only first source call and active recovery require32',()=>{
 const source={sourceRecoveryActive:true,currentInternalPgsIterations:32};
 const terminal={sourceRecoveryActive:false,currentInternalPgsIterations:20};
 assertQualificationSolverBudget(source,'source-policy',true);
 assertQualificationSolverBudget(terminal,'experimental-joint-balance',false);
 assert.throws(()=>assertQualificationSolverBudget(terminal,'experimental-joint-balance',true));
 assert.throws(()=>assertQualificationSolverBudget({...source,currentInternalPgsIterations:20},'source-policy'));
 assert.throws(()=>assertQualificationSolverBudget({...terminal,currentInternalPgsIterations:32},'experimental-joint-balance'));
});
test('measured-current20MiB bound reserves complete terminal output before dispatch',()=>{
 validateQualificationGrowthSources(process.cwd());
 const proof=qualificationByteProjection(2*1024**2,640000,5500000+2*1024**2);
 assert(proof.projectedActiveOutputBytes<32*1024**2);
 assert.equal(proof.nextGrowthBytes,20*1024**2);
 assert.throws(()=>qualificationByteProjection(10*1024**2,640000,5500000+2*1024**2),/BEFORE dispatch/);
 const capture=qualificationByteProjection(10*1024**2,640000,5500000+2*1024**2,0,false);
 assert.equal(capture.nextGrowthBytes,0);assert.equal(capture.physicalDispatchAuthorized,false);
});
