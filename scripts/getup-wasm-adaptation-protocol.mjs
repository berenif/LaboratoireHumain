// Actual WASM only. JSONL bridge for one pre-frozen local bounded experiment.
import {readFileSync,writeFileSync,appendFileSync,mkdirSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import {createInterface} from 'node:readline';
import assert from 'node:assert/strict';
import {appendInterval} from './getup-wasm-adaptation-math.mjs';
const root=process.cwd(), output=resolve(process.argv[2]); mkdirSync(output,{recursive:true});
const candidatePath=resolve(process.argv[3]);
const baselinePath=resolve('outputs/browser-capability-20261019/recovery-fixture/terminal-tail-ab/build/app-baseline-unadapted');
const originalPath=resolve('outputs/browser-capability-20261019/recovery-fixture/teacher-feasibility/actual-back-teacher/teacher-export-before-checkpoint.bin');
const profile=JSON.parse(readFileSync('docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json'));
const hash=x=>createHash('sha256').update(x).digest('hex');
const load=async path=>{let m=await import(pathToFileURL(join(path,'lh_worker.js')).href);m.initSync({module:readFileSync(join(path,'lh_worker_bg.wasm'))});return m;};
const candidate=await load(candidatePath), baseline=await load(baselinePath);
const config=JSON.parse(readFileSync(join(output,'frozen-config.json')));
const priorLedgerPath=join(output,'budget-ledger.json');
let priorLedger;try{priorLedger=JSON.parse(readFileSync(priorLedgerPath));}catch{}
const ledger=priorLedger??{schema:1,startedAt:new Date().toISOString(),deadlineUnixMs:Date.now()+config.wallSeconds*1000,
 nativeIntegrations:0,realIntegrations:0,forecastIntegrations:0,completedCalls:0,uncertainCalls:[],limit:config.nativeIntegrations,
 trainingDecisions:0,releaseAccepted:false};
const save=()=>writeFileSync(join(output,'budget-ledger.json'),JSON.stringify(ledger,null,2)+'\n');save();
let protectedEvaluationIntegrations=0,protectedEvaluationWallMs=0;
const guard=(reserve=5020)=>{assert(Date.now()+protectedEvaluationWallMs<ledger.deadlineUnixMs,'Experiment60minute wall cap');assert(ledger.nativeIntegrations+reserve+protectedEvaluationIntegrations<=ledger.limit,'Aggregate integration cap');};
const report=w=>JSON.parse(w.getup_adaptation_state?w.getup_adaptation_state():w.getup_report());
const fixture=w=>JSON.parse(w.getup_fixture_report());
const call=(w,ticks=1)=>{
 guard();const before=fixture(w).nativeSubsteps, bf=report(w)?.status.forecast_substeps??0;
 let result;
 try{result=JSON.parse(w.advance_terminal_tail_observed(ticks));}
 catch(error){ledger.uncertainCalls.push({at:new Date().toISOString(),chargedUpperBound:5020,error:String(error)});ledger.nativeIntegrations+=5020;save();throw error;}
 const after=fixture(w).nativeSubsteps,af=report(w)?.status.forecast_substeps??0;
 const realDelta=after>=before?after-before:20,forecastDelta=af>=bf?af-bf:5000;
 ledger.realIntegrations+=realDelta;ledger.forecastIntegrations+=forecastDelta;ledger.nativeIntegrations+=realDelta+forecastDelta;ledger.completedCalls++;save();
 assert(after>=before&&af>=bf,'Physical/reference counter reset during a bounded single-cycle call');
 assert(!result.traceError,result.traceError);return result;
};
const success=w=>report(w)?.status.prototype_standing_passed===true;
const preview=w=>{try{return JSON.parse(w.getup_adaptation_preview());}catch{return null;}};
function state(w){const f=fixture(w),c=report(w),d=JSON.parse(w.getup_standing_diagnostic('null'));
 return {native:f.nativeSubsteps,forecast:c?.status.forecast_substeps??0,status:c?.status,fixture:f.state,
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
 if(q.op==='reserve'){protectedEvaluationIntegrations=q.integrations;protectedEvaluationWallMs=q.wallSeconds*1000;ledger.evaluationReserveIntegrations=q.integrations;ledger.evaluationReserveWallSeconds=q.wallSeconds;save();return {protectedEvaluationIntegrations,protectedEvaluationWallMs};}
 if(q.op==='headparity'){return {correction:JSON.parse(candidate.getup_correction_head(JSON.stringify(q.head),JSON.stringify(q.features))),physicsIntegrations:0};}
 if(q.op==='identity')return {candidateWasmSha256:hash(readFileSync(join(candidatePath,'lh_worker_bg.wasm'))),baselineWasmSha256:hash(readFileSync(join(baselinePath,'lh_worker_bg.wasm'))),ledger};
 if(q.op==='qualify'){
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
  active?.free();const path=q.case==='heldout'?join(output,'heldout-back-560.bin'):q.case==='belly'?join(output,'belly-seed.bin'):originalPath;
  if(q.case==='belly'&&!q.prepared){const w=candidate.WorkerRuntime.getup_fixture_with_profile('belly',JSON.stringify(profile));w.set_root_translation_compensation_enabled(true);
   while(!fixture(w).state.ready){for(let i=0;i<4;i++){const r=call(w);assert(!r.reply.failure);}assert(fixture(w).nativeSubsteps<=2000);}
   w.trigger_getup_fixture();call(w,0);writeFileSync(path,w.getup_checkpoint());w.free();}
  active=candidate.WorkerRuntime.from_getup_checkpoint(readFileSync(path));
  label=q.label;previousC=Array(21).fill(0);longest=0;interval=0;
  // First20native are original actor for all branches. No stale latched observation is offered to learner.
  const r=call(active);assert(!r.reply.failure,JSON.stringify(r.reply.failure));
  if(q.head)active.set_getup_adaptation_head(JSON.stringify(q.head));else if(q.external)active.set_getup_adaptation_head('null');
  return state(active);
 }
 if(q.op==='step'){
  assert(active);const pre=state(active);
  let c=q.c??Array(21).fill(0);if(q.c&&pre.preview)active.queue_getup_correction(JSON.stringify(c));
  const packet=call(active),post=state(active);
  const failure=packet.reply.failure??(!success(active)&&post.status.since_impact_s>=25?{check:"original25s-deadline",native:post.native}:null);
  if(!q.c&&post.lastCorrection?.native_substep===pre.native)c=post.lastCorrection.correction;
  const terminal=success(active)||!!failure;const phi0=potential(pre.current),phi1=terminal?0:potential(post.current);
  const energy=c.reduce((a,b)=>a+b*b,0)/21,slew=c.reduce((a,b,i)=>a+(b-previousC[i])**2,0)/21;
  const reward=100*Number(success(active)&&!failure)-100*Number(!!failure)+2*(config.gamma*phi1-phi0)-.0025-.01*energy-.005*slew;
  const measured=appendInterval({longest,tail:interval},packet.nativeTrace);longest=measured.longest;interval=measured.tail;
  const provenance={label,preNative:pre.native,postNative:post.native,preObservation:pre.preview?.observation??null,
   postObservation:post.preview?.observation??null,referenceDecision:pre.preview?.referenceDecision??null,correction:c,
   appliedResidual:post.lastCorrection?.applied_residual??report(active).residual,originalResidual:pre.preview?.originalResidual??null,
   postPhysicalWords:packet.nativeTrace.at(-1)?.physicalObservationWords,postPoseWords:packet.nativeTrace.at(-1)?.poseWords,
   intervalPredicateWords:packet.nativeTrace.map(row=>['torso','bothFeetLoadedAndPersistent','footLoadFraction','onlyFeet','linearMotion','angularMotion'].reduce((mask,key,i)=>mask|(Number(row.current.predicates[key])<<i),0)),
   reward,terminal,success:success(active),failure,status:post.status};
  appendFileSync(join(output,'transitions.ndjson'),JSON.stringify(provenance)+'\n');
  previousC=c;if(q.training)ledger.trainingDecisions++;save();
  return {state:post,reward,terminal,success:success(active),failure,longestMeasuredInterval:longest,native:post.native};
 }
 if(q.op==='hold'){
  assert(active&&success(active),'Hold requires original800-sample success');
  const originalAnchors=JSON.parse(active.getup_report()).stable_feet;assert(originalAnchors);
  const initial=call(active,0).reply.snapshot,diagnostic=JSON.parse(active.getup_standing_diagnostic(JSON.stringify(originalAnchors)));
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
  let maximumOriginDriftM=0,maximumMaterialSlipM=0,samples=0;
  for(let tick=0;tick<1200;tick++){
   const packet=call(active);assert(!packet.reply.failure,JSON.stringify(packet.reply.failure));
   for(const row of packet.nativeTrace){
    assert(Object.entries(row.current.predicates).filter(([k])=>k!=='footDriftFromSuppliedAnchors').every(([,v])=>v===true),'Original standing predicate failed in30s hold');
    row.current.feet.forEach((foot,i)=>{const d=Math.hypot(...sub(xyz(foot),xyz(originalAnchors[i])));maximumOriginDriftM=Math.max(maximumOriginDriftM,d);assert(d<=.01,'Original10mm origin-anchor hold failed');});
    for(const anchor of material){const words=row.poseWords.slice(anchor.id*13,anchor.id*13+7).map(float),position=words.slice(0,3),r=words.slice(3,7);
     const point=rotate(r,anchor.local).map((x,i)=>x+position[i]);const slip=Math.hypot(point[0]-anchor.world[0],point[2]-anchor.world[2]);
     maximumMaterialSlipM=Math.max(maximumMaterialSlipM,slip);assert(slip<=.02,'Separate20mm material-point slip hold failed');}
    samples++;
   }
  }
  const result={passed:true,samples,physicalSeconds:30,maximumOriginDriftM,maximumMaterialSlipM,
   originAnchorSource:'original controller stable_feet, preserved at first800-native success',materialAnchorSource:'positive-impulse foot material points at first800-native success'};
  writeFileSync(join(output,label+'-30s-hold.json'),JSON.stringify(result,null,2)+'\n');return result;
 }
 if(q.op==='checkpoint'){assert(active);const bytes=active.getup_checkpoint();writeFileSync(join(output,q.name),bytes);return {sha256:hash(bytes),bytes:bytes.length};}
 if(q.op==='ledger')return ledger;
 throw Error('Unknown op');
}
for await(const line of createInterface({input:process.stdin,crlfDelay:Infinity})){
 try{const q=JSON.parse(line),r=await command(q);process.stdout.write(JSON.stringify({ok:true,result:r})+'\n');}
 catch(error){save();process.stdout.write(JSON.stringify({ok:false,error:String(error),stack:error.stack,ledger})+'\n');}
}
active?.free();save();
