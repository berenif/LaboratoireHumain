import test from 'node:test';
import assert from 'node:assert/strict';
import {enterQualificationStage,guardQualificationCall,beginQualificationSource,finishQualificationSource,qualificationPlan as p} from './getup-qualification-budget.mjs';
import {beginProtocolCall,finishProtocolCall} from './getup-learning-loop-protocol-contract.mjs';
const ledger=()=>({nativeIntegrations:0,realIntegrations:0,forecastIntegrations:0,
 trainingDecisions:0,bootstrapDecisions:0,completedCalls:0,uncertainIntegrations:0,evaluationReserveIntegrations:147000});
const counters=(source=0,forecast=0)=>({sourceDecisions:source,weakDecisions:forecast/20,
 forecast,sourceEpoch:480,impactEpoch:480});
test('qualification normal totals and one shared reserve are immutable',()=>{
 assert.equal(Object.values(p.phases).reduce((n,x)=>n+x.normal,0),112060);
 assert.equal(p.normal_integrations+p.shared_uncertainty_reserve,117080);
 assert(117080+147000<300000);assert.equal(p.optimizer_updates,0);
 const l=ledger();enterQualificationStage(l,'heldout-preservation');
 l.nativeIntegrations=l.qualificationStage.normalEnd-20;
 guardQualificationCall(l);beginProtocolCall(l,0,0);beginQualificationSource(l);
 finishProtocolCall(l,20,5000);
 assert.throws(()=>finishQualificationSource(l,counters(),counters(1,5000)),/reserve consumed/);
 assert.equal(l.nativeIntegrations,54000); // 5000 beyond49000 normal, still charged.
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
import {qualificationByteProjection,validateQualificationGrowthSources,assertQualificationSolverBudget,
 qualificationSelectedPasses} from './getup-qualification-budget.mjs';
import {readFileSync} from 'node:fs';
const budget=(current,next,active=false)=>({currentInternalPgsIterations:current,
 selectedNextInternalPgsIterations:next,sourceRecoveryActive:active});
const row=(motion,controller,phase,started=null)=>({motion,getup:{controller,
 policy:{phase,started_at_s:started}}});
const fixture=motion=>row(motion,'fixture-zero-control-preparation','experimental-joint-balance-awaiting-fall');
const source=(motion,phase='slow-following')=>row(motion,'pretrained-WEAK-SLOW',phase,.6);
const terminal=motion=>row(motion,'experimental-recovery-joint-balance-hold','experimental-joint-balance-holding',.6);
const trace=(...rows)=>[...Array(20-rows.length).fill(rows[0]),...rows];
test('complete motion by source-state selector matrix matches frozen Rust profile',()=>{
 for(const motion of ['Upright','Reacting','Falling','Fallen','Recovering','Halted'])
  for(const active of [false,true])
   assert.equal(qualificationSelectedPasses(motion,active),
    ['Falling','Fallen'].includes(motion)||motion==='Recovering'&&active?32:20);
 assert.throws(()=>qualificationSelectedPasses('future-motion',false));
 assert.throws(()=>qualificationSelectedPasses('Recovering',null));
 const profile=JSON.parse(readFileSync('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json'));
 assert.equal(profile.recovery_enabled,false);assert.equal(profile.internal_pgs_iterations,20);
 assert.equal(profile.fall_internal_pgs_iterations,32);
 assert.equal(profile.getup_solver_budget,'consistent-recovery-phase-v1');
});
test('fixture awaiting-fall regression selects32 without treating its phase as terminal',()=>{
 for(const motion of ['Upright','Reacting','Falling','Fallen']){
  const n=qualificationSelectedPasses(motion,false);
  const context={motion,preBudget:budget(n,n),nativeTrace:trace(fixture(motion),fixture(motion))};
  assertQualificationSolverBudget(budget(n,n),context);
  assert.throws(()=>assertQualificationSolverBudget(budget(n===20?32:20,n),context),/Last-integrated/);
 }
});
test('WEAK command, SLOW following/holding and early final-sample handoff use correct current versus next',()=>{
 for(const phase of ['teacher-reference','slow-following','slow-holding']){
  const context={motion:'Recovering',preBudget:budget(32,32),firstSourceCall:true,
   nativeTrace:trace(source('Recovering',phase),source('Recovering',phase))};
  assertQualificationSolverBudget(budget(32,32,true),context);
  assert.throws(()=>assertQualificationSolverBudget(budget(20,32,true),context),/Last-integrated/);
 }
 // Binding occurs in controller.observe AFTER integration; command diagnostics
 // still say SLOW. Current32 is valid while the next selection is already20.
 const handoff={motion:'Recovering',preBudget:budget(32,32,true),
  nativeTrace:trace(source('Recovering','slow-holding'),source('Recovering','slow-holding'))};
 assertQualificationSolverBudget(budget(32,20,false),handoff);
 assert.throws(()=>assertQualificationSolverBudget(budget(20,20,false),handoff),/Last-integrated/);
});
test('post-handoff, standing and post-integration motion changes do not alias current with next',()=>{
 for(const motion of ['Recovering','Upright','Reacting']){
  const context={motion,preBudget:budget(20,20),
   nativeTrace:trace(terminal(motion),terminal(motion))};
  assertQualificationSolverBudget(budget(20,20),context);
  assert.throws(()=>assertQualificationSolverBudget(budget(32,20),context),/Last-integrated/);
  assert.throws(()=>assertQualificationSolverBudget(budget(20,20),{...context,firstSourceCall:true}),/First WEAK/);
 }
 assertQualificationSolverBudget(budget(20,32),{motion:'Falling',preBudget:budget(20,20),
  nativeTrace:trace(terminal('Upright'),terminal('Falling'))});
});
test('zero-step trigger preserves current; partial physical failure stays primary, not fabricated solver proof',()=>{
 const zero={motion:'Fallen',preBudget:budget(20,20),nativeTrace:[],ticks:0};
 assertQualificationSolverBudget(budget(20,32),zero);
 assert.throws(()=>assertQualificationSolverBudget(budget(20,32),{...zero,ticks:1}),/complete native rows/);
 assert.throws(()=>assertQualificationSolverBudget(budget(32,32),{...zero,nativeTrace:trace(fixture('Fallen'))}),/complete native rows/);
 assert.throws(()=>assertQualificationSolverBudget(budget(32,32),{motion:'Falling',preBudget:budget(20,32),
  nativeTrace:[fixture('Falling')]}),/complete native rows/);
 for(const n of [0,1,19]){
  const refused=assertQualificationSolverBudget(budget(32,20),{motion:'Halted',
   preBudget:budget(20,32),nativeTrace:Array(n).fill(fixture('Falling')),
   failure:{check:'getup-policy',reason:'explicit refusal'}});
  assert.equal(refused.solverVerified,false);assert.equal(refused.scope,'physical-failure-preserved');
 }
});
test('unknown phases/controllers and malformed motion/source cannot pass solver audit',()=>{
 const context={motion:'Recovering',preBudget:budget(32,32),
  nativeTrace:trace(source('Recovering'),source('Recovering'))};
 for(const controller of ['experimental-physical-right-flank-roll','experimental-recovery-bumpless-joint-balance-hold']){
  const bad=structuredClone(context);bad.nativeTrace.at(-1).getup.controller=controller;
  assert.throws(()=>assertQualificationSolverBudget(budget(32,32,true),bad),/Unreviewed/);
 }
 const bad=structuredClone(context);bad.nativeTrace.at(-1).getup.policy.phase='future-phase';
 assert.throws(()=>assertQualificationSolverBudget(budget(32,32,true),bad),/Unreviewed/);
 assert.throws(()=>assertQualificationSolverBudget(budget(32,20,true),context),/Selected-next/);
 assert.throws(()=>assertQualificationSolverBudget(budget(32,32,null),context));
});
test('frozen source orders command diagnostic then solver selection then observations then native trace',()=>{
 const apply=readFileSync('rust/crates/sim/src/getup.rs','utf8');
 assert(apply.indexOf('controller.command(s, &physical)')<apply.indexOf('s.diagnostics.getup = Some',apply.indexOf('controller.command(s, &physical)')));
 const sim=readFileSync('rust/crates/sim/src/lib.rs','utf8');
 const selected=sim.indexOf('self.world.integration_parameters.num_internal_pgs_iterations =',sim.indexOf('pub fn advance_tick_observed'));
 assert(selected<sim.indexOf('self.observe_motion(&loads_n)',selected));
 assert(sim.indexOf('controller.observe(self',selected)<sim.indexOf('observer(self',selected));
 const runtime=readFileSync('rust/crates/sim/src/runtime.rs','utf8');
 assert(runtime.includes('"getup":snapshot.diagnostics.getup'));
});
test('legacy additive physical guard is closed; nonphysical exact capture stays bounded',()=>{
 validateQualificationGrowthSources(process.cwd());
 assert.throws(()=>qualificationByteProjection(0,0,0),/Legacy additive/);
 const capture=qualificationByteProjection(10*1024**2,640000,5500000+2*1024**2,0,false);
 assert.equal(capture.nextGrowthBytes,0);assert.equal(capture.physicalDispatchAuthorized,false);
});

import {retainMeasuredHold} from './getup-qualification-budget.mjs';
test('late hold checkpoint byte rejection retains measured counts and extrema without passed proof',()=>{
 const result={samples:24000,physicalSeconds:30,maximumOriginDriftM:.004,maximumMaterialSlipM:.009};
 const retained=[];
 assert.throws(()=>retainMeasuredHold(result,x=>retained.push(x),()=>{throw Error('byte guard final copy');}),/byte guard/);
 assert.equal(retained.at(-1).passed,false);assert.equal(retained.at(-1).checkpointBound,false);
 assert.equal(retained.at(-1).samples,24000);assert.equal(retained.at(-1).maximumMaterialSlipM,.009);
 assert.equal(retained.at(-1).finalCheckpointSha256,undefined);
 const passed=retainMeasuredHold(result,x=>retained.push(x),()=> 'a'.repeat(64));
 assert.equal(passed.passed,true);assert.equal(passed.checkpointBound,true);
});
