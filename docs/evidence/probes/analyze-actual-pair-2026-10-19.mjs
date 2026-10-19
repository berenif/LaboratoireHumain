// Read-only retained-data analysis. No inference, world, browser, or integration.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {createHash} from 'node:crypto';
const lane=resolve('outputs/browser-capability-20261019/recovery-fixture/right-ankle-ablation'),pair=join(lane,'actual-back-pair');
const H=b=>createHash('sha256').update(b).digest('hex'),canon=v=>Array.isArray(v)?v.map(canon):v&&typeof v==='object'?Object.fromEntries(Object.keys(v).sort().map(k=>[k,canon(v[k])])):v;
const equal=(a,b)=>JSON.stringify(canon(a))===JSON.stringify(canon(b));
const receipt=p=>{const b=readFileSync(p);return{path:p,sha256:H(b),bytes:b.length}};
const decode=w=>Array.from(new Float32Array(new Uint32Array(w).buffer));
const readRows=p=>readFileSync(p,'utf8').trim().split('\n').map(JSON.parse);
const rows={},controllers={},results={};
for(const arm of ['baseline','candidate']){
 rows[arm]=readRows(join(pair,arm,'native-trace.ndjson'));controllers[arm]=JSON.parse(readFileSync(join(pair,arm,'stop-controller.json')));results[arm]=JSON.parse(readFileSync(join(pair,arm,'result.json')));
 assert.equal(results[arm].observationValid,true);assert.equal(rows[arm].length,2381);rows[arm].forEach((r,i)=>assert.equal(r.nativeSubsteps,i));
 assert.equal(results[arm].controllerCaptures.stop.native,2380);assert.equal(results[arm].controllerCaptures.stop.physicsIntegrations,0);
 for(let n=1;n<=2380;n++){
  const m=rows[arm][n].motorApplication,wanted=arm==='candidate'&&n>=2181&&n<=2200;assert.equal(m.commandInputNativeSubstep,n-1);assert.equal(m.overrideApplied,wanted);assert.deepEqual(m.overriddenActuatorIndices,wanted?[8]:[]);
  for(let i=0;i<21;i++){assert.equal(m.appliedTorqueNm[i],wanted&&i===8?0:m.intendedTorqueNm[i]);assert.equal(m.appliedControls[i],wanted&&i===8?0:m.intendedControls[i]);}
 }
}
const a=rows.baseline,b=rows.candidate;
for(let n=0;n<=2180;n++)assert.ok(equal(a[n],b[n]),'Full observable prefix n'+n);
const firstDiff=a.findIndex((r,i)=>!equal(r,b[i]));assert.equal(firstDiff,2181);
const teacherKeys=['teacher_initial','teacher_frames','teacher_native_dt_s','teacher_failure','reference'];
for(const k of teacherKeys)assert.ok(equal(controllers.baseline[k],controllers.candidate[k]),'Same full archive '+k);
for(const arm of ['baseline','candidate']){
 const c=controllers[arm];assert.equal(c.reference.length,c.status.reference_frames+1);assert.equal(c.reference.length,103);assert.equal(c.status.reference_frames,102);assert.equal(c.teacher_frames.length,51);assert.equal(c.status.forecast_substeps,1020);assert.equal(c.impact_substep,480);
}
const priorPath=resolve('outputs/browser-capability-20261019/recovery-fixture/terminal-tail-ab/actual-back-pair/baseline/native-trace.ndjson');
const prior=readRows(priorPath);let priorMatched=0;
for(let n=0;n<=2380;n++){
 const retained=Object.fromEntries(Object.keys(prior[n]).map(k=>[k,a[n][k]]));assert.ok(equal(retained,prior[n]),'Retained preablation baseline same observable schema n'+n);priorMatched++;
}
const ranges={pre20:[2161,2180],ablation:[2181,2200],restored:[2201,2380],postIntervention:[2181,2380]};
function metrics(rs,first,last){
 let workPre=0,workAngle=0,maximumLinear=0,maximumAngular=0,leftLoadMin=Infinity,leftLoadMax=-Infinity,rightLoadMin=Infinity,rightLoadMax=-Infinity,leftGapMin=Infinity,leftGapMax=-Infinity,torsoMin=Infinity,torsoMax=-Infinity;
 const overrides=[],leftUnloaded=[],leftSeparated=[];
 for(let n=first;n<=last;n++){
  const r=rs[n],p=decode(rs[n-1].physicalObservationWords),q=decode(r.physicalObservationWords),d=r.current,tau=r.motorApplication.appliedTorqueNm[8];
  workPre+=tau*p[46+7]/800;workAngle+=tau*(q[7]-p[7]);
  maximumLinear=Math.max(maximumLinear,...d.motion.map(x=>x.linearMps));maximumAngular=Math.max(maximumAngular,...d.motion.map(x=>x.angularRadps));
  const load=d.loadsN[d.footIds[0]],gap=r.footFloorClearanceM[0];leftLoadMin=Math.min(leftLoadMin,load);leftLoadMax=Math.max(leftLoadMax,load);rightLoadMin=Math.min(rightLoadMin,d.loadsN[d.footIds[1]]);rightLoadMax=Math.max(rightLoadMax,d.loadsN[d.footIds[1]]);leftGapMin=Math.min(leftGapMin,gap);leftGapMax=Math.max(leftGapMax,gap);torsoMin=Math.min(torsoMin,d.torsoUp);torsoMax=Math.max(torsoMax,d.torsoUp);
  if(r.motorApplication.overrideApplied)overrides.push(n);if(load<3)leftUnloaded.push(n);if(gap>0)leftSeparated.push(n);
 }
 return{nativeRange:[first,last],rows:last-first+1,actualOverrides:overrides,rightAnkleActiveMotorWorkPreNativeDqEstimateJ:workPre,rightAnkleActiveMotorWorkAngleIncrementEstimateJ:workAngle,maximumCurrentBodyLinearMps:maximumLinear,maximumCurrentBodyAngularRadps:maximumAngular,leftLoadN:[leftLoadMin,leftLoadMax],rightLoadN:[rightLoadMin,rightLoadMax],leftClearanceM:[leftGapMin,leftGapMax],torsoUp:[torsoMin,torsoMax],leftUnloadedRows:leftUnloaded.length,leftSeparatedRows:leftSeparated.length};
}
function events(rs){
 const runs=[];let start=null;for(let n=2181;n<=2380;n++){const unloaded=rs[n].current.loadsN[rs[n].current.footIds[0]]<3;if(unloaded&&start===null)start=n;if(!unloaded&&start!==null){runs.push({first:start,last:n-1,length:n-start});start=null}}if(start!==null)runs.push({first:start,last:2380,length:2381-start});
 const sustained=runs.find(x=>x.length>=20);let first20=null;if(sustained)first20=sustained.first+19;
 const samples=Object.fromEntries([2180,2181,2200,2201,2353,2355,2356,2357,2358,2375,2377,2380].map(n=>{const r=rs[n],physical=decode(r.physicalObservationWords);return[n,{native:n,time:n/800,leftLoadN:r.current.loadsN[r.current.footIds[0]],rightLoadN:r.current.loadsN[r.current.footIds[1]],leftClearanceM:r.footFloorClearanceM[0],rightClearanceM:r.footFloorClearanceM[1],torsoUp:r.current.torsoUp,com:r.current.com,comVelocity:r.current.comVelocity,comSupported:r.current.predicates.comInSupport,captureSupported:r.current.predicates.captureInSupport,rightAnkleQ:physical[7],rightAnkleDq:physical[53],intendedAnkleTorqueNm:r.motorApplication.intendedTorqueNm[8],appliedAnkleTorqueNm:r.motorApplication.appliedTorqueNm[8],integrityCumulativeMaxima:r.integrityLimits}]}));
 return{firstPositiveClearanceNativeAfterIntervention:rs.find(r=>r.nativeSubsteps>=2181&&r.footFloorClearanceM?.[0]>0)?.nativeSubsteps??null,allPostInterventionUnloadingRuns:runs,firstBelow3Native:runs[0]?.first??null,firstSustainedRun:sustained??null,firstCompleted20Native:first20,clearanceAtFirstBelow3M:runs.length?rs[runs[0].first].footFloorClearanceM[0]:null,clearanceAtFirstCompleted20M:first20===null?null:rs[first20].footFloorClearanceM[0],sustainedRunClearanceM:sustained?[Math.min(...rs.slice(sustained.first,sustained.last+1).map(x=>x.footFloorClearanceM[0])),Math.max(...rs.slice(sustained.first,sustained.last+1).map(x=>x.footFloorClearanceM[0]))]:null,samples};
}
const summary={at:new Date().toISOString(),status:'completed-isolated20-native-right-ankle-ablation-bounded-diagnostic',recordingCompleteByIndependentReadOnlyAnalysis:true,physicalInterventionValidated:true,policyAdmitted:false,noAdditionalRun:true,
 prefix:{range:[0,2180],rows:2181,exclusions:[],fullObservableEquality:true,firstActualDifferenceNative:firstDiff,baselinePrefixSha256:H(JSON.stringify(canon(a.slice(0,2181))))},
 archive:{teacherInitialAndAll51FramesEqual:true,complete103ReferenceTuplesEqual:true,referenceFramesExcludingAppended:102,totalReferenceTuples:103,forecastIntegrations:1020,originalImpactNative:480},
 priorBaselineParity:{matchedNativeRowsIncluding0:priorMatched,commonRetainedSchemaExact:true,priorReceipt:receipt(priorPath),excludedNewFields:['motorApplication','footFloorClearanceM','integrityLimits','native0.snapshot']},
 arms:Object.fromEntries(['baseline','candidate'].map(arm=>[arm,{rows:2381,stopNative:2380,stopPhysicalSeconds:2.975,actualOverrideCount:arm==='candidate'?20:0,windows:Object.fromEntries(Object.entries(ranges).map(([k,[f,l]])=>[k,metrics(rows[arm],f,l)])),events:events(rows[arm]),noNativeOrWorkerFault:true,receipts:{result:receipt(join(pair,arm,'result.json')),trace:receipt(join(pair,arm,'native-trace.ndjson')),controller:receipt(join(pair,arm,'stop-controller.json'))}}])),
 interpretation:'The one-step-budget motor-torque intervention does not remove sustained left load release by the fixed bound: first20 completes2375 baseline vs2377 candidate,2native/2.5ms later. Their clearance becomes positive at2361/2363 and is positive at each first20-sample detection; the initial unloaded samples still have nonpositive clearance. Thus load release and separation are kept distinct; no long recovery or policy admission inferred.',
 workConvention:'Actual active scalar at equal/opposite motor-body application×pre-native generalizedqpos7 dq×1/800, signed estimate. Optional tau×qangleincrement shown separately. Neither net constrained/passive torque nor COMforce; passive/contact torques remained active.',
 rawRecorderBookkeeping:{recordingComplete:false,error:'Final comparator incorrectly asserted103 totalreference tuples ==102 excluding appendedtarget. Original raw report preserved. Independent retained-data check uses exact+1 relation, no replay or altered gate.'},
 rawPairReceipt:receipt(join(pair,'result.json'))};
writeFileSync(join(lane,'ankle-ablation-summary.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({status:summary.status,prefix:summary.prefix,archive:summary.archive,arms:Object.fromEntries(Object.entries(summary.arms).map(([arm,x])=>[arm,{actualOverrideCount:x.actualOverrideCount,events:{firstBelow3Native:x.events.firstBelow3Native,firstSustainedRun:x.events.firstSustainedRun,firstCompleted20Native:x.events.firstCompleted20Native,clearanceAtFirstBelow3M:x.events.clearanceAtFirstBelow3M,clearanceAtFirstCompleted20M:x.events.clearanceAtFirstCompleted20M,sustainedRunClearanceM:x.events.sustainedRunClearanceM},windows:x.windows}]))},null,2));
