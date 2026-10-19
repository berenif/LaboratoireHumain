// Read-only analysis of the already completed bounded paired run. No browser/physics.
import assert from 'node:assert/strict';
import { readFileSync,writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { resolve,join } from 'node:path';
const lane=resolve('outputs/browser-capability-20261019/recovery-fixture/terminal-tail-ab');
const pair=join(lane,'actual-back-pair'), H=b=>createHash('sha256').update(b).digest('hex');
const canonical=v=>Array.isArray(v)?v.map(canonical):v&&typeof v==='object'?Object.fromEntries(Object.keys(v).sort().map(k=>[k,canonical(v[k])])):v;
const same=(a,b)=>JSON.stringify(canonical(a))===JSON.stringify(canonical(b));
const receipt=path=>{const b=readFileSync(path);return{path,sha256:H(b),bytes:b.length}};
const raw=JSON.parse(readFileSync(join(pair,'result.json')));
const arms={};
for(const arm of ['baseline','candidate']){
 const path=join(pair,arm,'native-trace.ndjson'),rows=readFileSync(path,'utf8').trim().split('\n').map(JSON.parse);
 const result=JSON.parse(readFileSync(join(pair,arm,'result.json'))),controller=JSON.parse(readFileSync(join(pair,arm,'stop-controller.json')));
 assert.equal(result.observationValid,true);assert.equal(result.evidence.lastNative,2920);assert.equal(result.evidence.triageDeadlineNative,2920);assert.equal(rows.length,2921);
 rows.forEach((r,i)=>assert.equal(r.nativeSubsteps,i));
 assert.equal(result.controllerCaptures.stop.native,result.triage.deadlineNative);assert.equal(result.controllerCaptures.stop.physicsIntegrations,0);
 assert.equal(result.evidence.error,null);assert.deepEqual(result.errors,[]);assert.equal(result.triage.passed,false);assert.equal(result.triage.currentConsecutiveNative,0);
 arms[arm]={rows,result,controller};
}
const a=arms.baseline,b=arms.candidate;
const prefixFields=['poseWords','physicalObservationWords','rootTranslationIntegratorStateJson'];
const firstMismatch=(get)=>{for(let n=0;n<a.rows.length;n++)if(!same(get(a.rows[n]),get(b.rows[n])))return n;return null};
const firstDifference=(x,y,path='')=>{if(same(x,y))return null;if(x&&y&&typeof x==='object'&&typeof y==='object'){for(const k of new Set([...Object.keys(x),...Object.keys(y)])){const f=firstDifference(x[k],y[k],path+(Array.isArray(x)?`[${k}]`:(path?'.':'')+k));if(f)return f;}}return{path,baseline:x,candidate:y}};
const firstRowMismatch=firstMismatch(x=>x);
const prefixEnd=2420;
for(let n=0;n<=prefixEnd;n++)assert.ok(same(a.rows[n],b.rows[n]),'Exact full native prefix n'+n);
const controllerComparison={
 sameTeacherInitial:same(a.controller.teacher_initial,b.controller.teacher_initial),
 sameAll51TeacherFrames:same(a.controller.teacher_frames,b.controller.teacher_frames),
 teacherFrameCount:[a.controller.teacher_frames.length,b.controller.teacher_frames.length],
 sameTeacherNativeDt:a.controller.teacher_native_dt_s===b.controller.teacher_native_dt_s,
 teacherNativeDt:a.controller.teacher_native_dt_s,
 sameForecastCount:a.controller.status.forecast_substeps===b.controller.status.forecast_substeps,
 forecastNativeSteps:a.controller.status.forecast_substeps,
 sameInterpolated102FramePrefix:same(a.controller.reference.slice(0,-1),b.controller.reference.slice(0,-1)),
 referenceFramesExcludingAppended:[a.controller.status.reference_frames,b.controller.status.reference_frames],
 totalTargetTuples:[a.controller.reference.length,b.controller.reference.length],
 candidateAppendedTupleExactlyDuplicatesFinalInterpolated:same(b.controller.reference.at(-1),b.controller.reference.at(-2)),
 baselineAppendedTuple:a.controller.reference.at(-1),candidateAppendedTuple:b.controller.reference.at(-1),
 sameOriginalImpact:[a.controller.impact_substep,b.controller.impact_substep],
 sameStartedSubstep:[a.controller.started_substep,b.controller.started_substep],
 sameReferenceAndTeacherCounts:a.controller.reference.length===b.controller.reference.length&&a.controller.status.teacher_decisions===b.controller.status.teacher_decisions,
};
for(const k of ['sameTeacherInitial','sameAll51TeacherFrames','sameTeacherNativeDt','sameForecastCount','sameInterpolated102FramePrefix','candidateAppendedTupleExactlyDuplicatesFinalInterpolated','sameReferenceAndTeacherCounts'])assert.equal(controllerComparison[k],true,k);
function runs(rows,test,minNative=480){let start=null,out=[];for(const r of rows){if(r.nativeSubsteps<minNative)continue;const yes=test(r);if(yes&&start===null)start=r.nativeSubsteps;if(!yes&&start!==null){out.push({first:start,last:r.nativeSubsteps-1,length:r.nativeSubsteps-start});start=null}}if(start!==null)out.push({first:start,last:rows.at(-1).nativeSubsteps,length:rows.at(-1).nativeSubsteps-start+1});return out}
function sample(r){const d=r.current,feet=d.footIds.map(i=>d.loadsN[i]),motion=d.motion;return{
 native:r.nativeSubsteps,time:r.nativeSubsteps/800,commandInputNative:r.commandInputNativeSubstep??null,
 phase:r.getup?.policy?.phase??null,referenceIndex:r.getup?.policy?.reference_index??null,
 headM:d.headM,torsoUp:d.torsoUp,feetLoadN:feet,footLoadN:feet.reduce((a,b)=>a+b,0),bodyWeightN:r.weightN??null,
 nativePredicates:d.predicates,nonfootMaxLoadN:Math.max(...d.loadsN.filter((_,i)=>!d.footIds.includes(i))),
 com:d.com,comVelocity:d.comVelocity,capture:d.capture,solverSupportPoints:d.solverSupportPoints,
 maximumCurrentLinearMps:Math.max(...motion.map(x=>x.linearMps)),maximumCurrentAngularRadps:Math.max(...motion.map(x=>x.angularRadps)),
 trackingErrorRad:r.getup?.policy?.reference_tracking_error_rad??null,referenceComErrorM:r.getup?.policy?.reference_com_error_m??null,
 stableStandingS:r.getup?.policy?.stable_standing_s??0,originalStableFeet:r.originalStableFeet??null,
 footContacts:d.contactMotion.feet.map(f=>({body:f.body,loadN:f.loadN,loaded:f.loaded,currentPoints:f.contacts.length,impulseWeightedTangentSpeedMps:f.impulseWeightedTangentSpeedMps})),
 requestedControls:r.getup?.motor_controls??null};}
const summaryArms={};
for(const [arm,{rows,result,controller}]of Object.entries(arms)){
 const progress=r=>r.current.footIds.every(i=>r.current.loadsN[i]>=3)&&['footLoadFraction','onlyFeet','torso','comInSupport','captureInSupport'].every(k=>r.current.predicates[k]===true);
 const fractionRuns=runs(rows,r=>r.current.predicates.footLoadFraction===true);
 summaryArms[arm]={outcome:result.outcome,observationValid:true,triage:result.triage,allPostNativeSamplesCovered:rows.length-1,native0Receipt:true,
 exactStopNative:rows.at(-1).nativeSubsteps,extraUncreditedNativeSteps:result.evidence.extraUncreditedRows,
 firstHoldingNative:result.evidence.firstHoldingNative,firstHoldingCommandInputNative:result.evidence.firstHoldingCommandInputNative,
 progressRuns:runs(rows,progress),footFractionRuns:fractionRuns,bothFootCurrentMinimumRuns:runs(rows,r=>r.current.footIds.every(i=>r.current.loadsN[i]>=3)),
 torsoNativePassRuns:runs(rows,r=>r.current.predicates.torso===true),comSupportRuns:runs(rows,r=>r.current.predicates.comInSupport===true),captureSupportRuns:runs(rows,r=>r.current.predicates.captureInSupport===true),
 allFeetUnloadedRuns:runs(rows,r=>r.current.footIds.every(i=>r.current.loadsN[i]===0)),
 materialSlipDiagnosticOnly:result.evidence.materialSlip,
 snapshots:Object.fromEntries([2280,2300,2420,2421,2520,2521,2560,2920].map(n=>[String(n),sample(rows[n])])),
 firstPass:false,twentyFiveSecondRecoveryRun:false,thirtySecondHoldRun:false,performanceClaim:false,
 receipts:{result:receipt(join(pair,arm,'result.json')),nativeTrace:receipt(join(pair,arm,'native-trace.ndjson')),stopController:receipt(join(pair,arm,'stop-controller.json'))},
 checkpointLimitation:arm==='candidate'?'Export/restore deliberately refused; live native0 words/current/carry proof only.':'Binary world/model/embedded runtime activation proof passed.',
 };
}
// Independent same-target baseline observer purity: retained earlier unit-root
// BACK pose/velocity words at every40Hz publication through this short bound.
const oldPath=resolve('outputs/browser-capability-20261019/recovery-fixture/unit-root-rotation/back-thirty/result.json');
const old=JSON.parse(readFileSync(oldPath));let checked=0;
for(const s of old.evidence.samples){const n=Math.round(s.simulation_time_s*800);if(n>2920)break;
 const v=s.segments.flatMap(p=>['position','rotation','linearVelocity','angularVelocity'].flatMap(k=>(k==='rotation'?['x','y','z','w']:['x','y','z']).map(a=>p[k][a])));
 const words=Array.from(new Uint32Array(new Float32Array(v).buffer));assert.ok(same(words,a.rows[n].poseWords),'Baseline retained pose prefix n'+n);checked++;}
const summary={at:new Date().toISOString(),status:'completed-negative-endpoint-only-bounded-back-triage',recordingComplete:true,candidateAdmitted:false,
 scope:'One fresh actual-WASM BACK baseline and one fresh candidate; no retries/sweep; stopped exactly3.65physical seconds under predeclared current80-native support streak gate.',
 controllerComparison,physicalPrefix:{fullRowsIncludingCurrentContactsPolicyControlsCarryEqualThroughNative:prefixEnd,rowsCompared:prefixEnd+1,
 firstPermittedAffectedNative:2421,firstActualFullRowDifferenceNative:firstRowMismatch,firstActualFullRowDifference:firstDifference(a.rows[firstRowMismatch],b.rows[firstRowMismatch]),
 firstPoseWordsDifferenceNative:firstMismatch(x=>x.poseWords),firstFreshPhysicalObservationWordsDifferenceNative:firstMismatch(x=>x.physicalObservationWords),firstExactCarryStringDifferenceNative:firstMismatch(x=>x.rootTranslationIntegratorStateJson),firstRequestedControlsDifferenceNative:firstMismatch(x=>x.getup?.motor_controls),
 normalizedPrefixSha256:H(a.rows.slice(0,prefixEnd+1).map(r=>JSON.stringify(canonical(r))).join('\n')+'\n')},
 baselineObserverPurity:{retainedUnitRootRun:receipt(oldPath),compared40HzPublications:checked,every325PoseVelocityWordExact:true,claim:'Same-target emitted body pose/velocity parity only; not cross-target trajectory or hidden full-world bitwise equivalence.'},
 arms:summaryArms,rawRecorderBookkeepingLimitation:{automaticComparisonAvailable:raw.comparison.available,why:'Negative stop ends scheduling before queued triage capture is serviced. Both unconditional stop-controller exports exist read-only at exact2920 cutoff, so this independent analysis uses those same-state captures without replay.',originalRawReportPreserved:true},
 causalFinding:'Removing the sole appended canonical tuple creates its first real difference only at2421 as designed, but does not earn any qualifying supported80-native streak by2920. It cannot cause the earlier shared pre2421 load/support trajectory. No claim about25s outcome is made.',
 rootPlannerNextStep:'Endpoint-only candidate rejected/unadmitted. Preserve frozen pair and hand upstream current support/motion/command diagnosis to physics owner; no automatic parameter sweep.',
 rawPairResult:receipt(join(pair,'result.json'))};
writeFileSync(join(lane,'terminal-tail-ab-summary.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({status:summary.status,controllerComparison,physicalPrefix:summary.physicalPrefix,arms:Object.fromEntries(Object.entries(summaryArms).map(([k,v])=>[k,{triage:v.triage,footFractionRuns:v.footFractionRuns,bothFootCurrentMinimumRuns:v.bothFootCurrentMinimumRuns,allFeetUnloadedRuns:v.allFeetUnloadedRuns,comSupportRuns:v.comSupportRuns,captureSupportRuns:v.captureSupportRuns}])),baselineObserverPurity:summary.baselineObserverPurity},null,2));
