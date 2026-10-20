// Continue the SAME retained physical world to the fixed original impact25s endpoint.
import {readFileSync,writeFileSync,appendFileSync,mkdirSync} from 'node:fs';
import {resolve,join} from 'node:path';import {pathToFileURL} from 'node:url';import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';import {appendInterval} from './getup-wasm-adaptation-math.mjs';
const output=resolve(process.argv[2]),label=process.argv[3],checkpoint=resolve(process.argv[4]),worker=resolve(process.argv[5]);
const mainOutput=resolve(process.argv[6]),deadline=JSON.parse(readFileSync(join(mainOutput,'budget-ledger.json'))).deadlineUnixMs;
mkdirSync(output,{recursive:true});const sha=x=>createHash('sha256').update(x).digest('hex');
const m=await import(pathToFileURL(join(worker,'lh_worker.js')).href);m.initSync({module:readFileSync(join(worker,'lh_worker_bg.wasm'))});
const bytes=readFileSync(checkpoint),w=m.WorkerRuntime.from_getup_checkpoint(bytes),controller=JSON.parse(w.getup_adaptation_state());
assert(controller.status.prototype_standing_passed,'Only continue already-successful censored same-world checkpoint');
const initial=JSON.parse(w.getup_fixture_report()).nativeSubsteps,impact=controller.impact_substep,end=impact+20000;
assert(initial<=end&&end-initial<=20000);const forecast=controller.status.forecast_substeps;
const fullInitial=JSON.parse(w.getup_report()),epoch={started:controller.started_substep,impact:controller.impact_substep,lastDecision:controller.last_decision,policyDecisions:controller.status.policy_decisions,phase:controller.status.phase,terminalActivation:fullInitial.terminal_balance?.activated_native_substep,headSha256:sha(JSON.stringify(fullInitial.adaptation?.head??null))};
let metric={longest:0,tail:0},preNative;
for(const line of readFileSync(join(mainOutput,'transitions.ndjson'),'utf8').trim().split('\n')){
 const row=JSON.parse(line);if(row.label!==label)continue;preNative=row.postNative;
 for(const word of row.intervalPredicateWords){const pass=word===63;metric.tail=pass?metric.tail+1:0;metric.longest=Math.max(metric.longest,metric.tail);}
}
assert.equal(preNative,initial,'Retained metric endpoint does not match exact checkpoint');
const ledger={schema:1,label,checkpointSha256:sha(bytes),startNative:initial,impactNative:impact,endNative:end,nativeIntegrations:0,realIntegrations:0,forecastIntegrations:0,limit:20000,deadlineUnixMs:deadline,releaseAccepted:false};
const save=()=>writeFileSync(join(output,label+'-continuation-ledger.json'),JSON.stringify(ledger,null,2)+'\n');save();
try{
 while(initial+ledger.nativeIntegrations<end){
  assert(Date.now()<deadline,'Original experiment absolute deadline');assert(ledger.nativeIntegrations+5020<=ledger.limit+5000);
  const mainLedger=JSON.parse(readFileSync(join(mainOutput,'budget-ledger.json')));
  const others=['baseline-heldout','candidate-original','candidate-heldout'].filter(name=>name!==label).reduce((total,name)=>{try{return total+JSON.parse(readFileSync(join(output,name+'-continuation-ledger.json'))).nativeIntegrations;}catch{return total;}},0);
  assert(mainLedger.nativeIntegrations+others+ledger.nativeIntegrations+5020<=600000,'Combined main/external600k integration cap');
  const nativeBefore=JSON.parse(w.getup_fixture_report()).nativeSubsteps;
  let packet;try{packet=JSON.parse(w.advance_terminal_tail_observed(1));}catch(error){ledger.nativeIntegrations+=5020;ledger.uncertainOutstandingUpperBound=5020;save();throw error;}
  const after=JSON.parse(w.getup_adaptation_state()),nativeAfter=JSON.parse(w.getup_fixture_report()).nativeSubsteps;
  const delta=nativeAfter-nativeBefore;assert(delta>=0);ledger.realIntegrations+=delta;ledger.forecastIntegrations=after.status.forecast_substeps>=forecast?after.status.forecast_substeps-forecast:5000;ledger.nativeIntegrations=ledger.realIntegrations+ledger.forecastIntegrations;save();
  assert.equal(after.started_substep,epoch.started,'Source start epoch changed');assert.equal(after.impact_substep,epoch.impact,'Original impact epoch changed');
  assert.equal(after.last_decision,epoch.lastDecision,'Held source decision changed');assert.equal(after.status.policy_decisions,epoch.policyDecisions,'Unexpected new policy decision');assert.equal(after.status.phase,epoch.phase,'Recovery controller phase changed');
  assert.equal(packet.nativeTrace.length,delta,'Successful observed rows differ from physical native integrations');
  assert(!packet.traceError);assert(!packet.reply.failure,JSON.stringify(packet.reply.failure));assert.equal(after.status.forecast_substeps,forecast,'Continuation generated unexpected new forecast');metric=appendInterval(metric,packet.nativeTrace);
  appendFileSync(join(output,label+'-continuation.ndjson'),JSON.stringify({native:packet.nativeTrace.at(-1)?.nativeSubsteps,
   intervalPredicateWords:packet.nativeTrace.map(row=>['torso','bothFeetLoadedAndPersistent','footLoadFraction','onlyFeet','linearMotion','angularMotion'].reduce((mask,key,i)=>mask|(Number(row.current.predicates[key])<<i),0)),
   finalPoseWords:packet.nativeTrace.at(-1)?.poseWords,finalPhysicalWords:packet.nativeTrace.at(-1)?.physicalObservationWords,status:after.status})+'\n');
 }
 const final=JSON.parse(w.getup_adaptation_state());assert.equal(JSON.parse(w.getup_fixture_report()).nativeSubsteps,end);
 const fullFinal=JSON.parse(w.getup_report());assert.equal(fullFinal.terminal_balance?.activated_native_substep,epoch.terminalActivation);assert.equal(sha(JSON.stringify(fullFinal.adaptation?.head??null)),epoch.headSha256);
 const finalBytes=w.getup_checkpoint();writeFileSync(join(output,label+'-fixed25-final.bin'),finalBytes);
 const result={...ledger,complete:true,longestMeasuredInterval:metric.longest,tailInterval:metric.tail,status:final.status,
  firstSuccessObservedNative:initial,finalCheckpointSha256:sha(finalBytes),unchangedControllerEpoch:epoch,metricCorrection:'same physical trajectory continued after first800success to fixedimpact+25s; no new candidate/head/reference/threshold selection'};
 writeFileSync(join(output,label+'-fixed25-summary.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
}finally{save();w.free();}
