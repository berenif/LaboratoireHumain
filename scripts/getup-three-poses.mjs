// Three fixed fixtures, unchanged pretrained controller, all preparation counted.
import assert from 'node:assert/strict';
import { readFileSync, writeFileSync, appendFileSync, openSync, closeSync, fsyncSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { initSync, WorkerRuntime, getup_trial_profile } from '../rust/dist/app/lh_worker.js';
const sha=x=>createHash('sha256').update(x).digest('hex');
const save=(path,value)=>writeFileSync(path,JSON.stringify(value,null,2)+'\n');
initSync({module:readFileSync('rust/dist/app/lh_worker_bg.wasm')});
const cases=['back','belly','side','side-supported'];
if(process.argv[2]==='--preflight') {
  const out=process.argv[3];mkdirSync(out,{recursive:true});const results=[];
  for(const kind of cases) {
    const runtime=WorkerRuntime.getup_fixture(kind);
    const reply=JSON.parse(runtime.advance(0));
    const report=JSON.parse(runtime.getup_fixture_report());
    assert.equal(reply.snapshot.simulation_time_s,0);assert.equal(report.nativeSubsteps,0);
    assert.equal(report.state.kind,kind);assert.equal(report.state.preparing,true);assert.equal(report.state.ready,false);
    assert.equal(reply.snapshot.segments.length,25);
    assert.ok(reply.snapshot.diagnostics.getup.motor_controls.every(x=>x===0));
    assert.throws(()=>runtime.trigger_getup_fixture());
    const bytes=runtime.getup_checkpoint();const verification=JSON.parse(runtime.verify_getup_checkpoint(bytes));
    const restored=WorkerRuntime.from_getup_checkpoint(bytes);
    assert.equal(restored.getup_fixture_report(),runtime.getup_fixture_report());
    assert.deepEqual(JSON.parse(restored.advance(0)).snapshot.segments,reply.snapshot.segments);
    writeFileSync(`${out}/preflight-${kind}.bin`,bytes);
    results.push({kind,report,checkpoint:verification,checkpointSha256:sha(bytes),initialHeadM:reply.snapshot.segments.find(p=>p.id==='head').position.y});
    restored.free();runtime.free();
  }
  save(`${out}/preflight.json`,{passed:true,physicsIntegrations:0,cases:results,profile:JSON.parse(getup_trial_profile()),
    previousHistoryNativePerSecond:8400/43.61662435800099,
    estimatedSettlingAndTypicalRecoveryWallSecondsPerCase:(1600+540+6000)/(8400/43.61662435800099)});
} else {
  const ledger=JSON.parse(readFileSync(process.argv[2]));const kind=process.argv[3];const out=process.argv[4];
  assert.equal(ledger.status,'running');assert.ok(cases.includes(kind));
  const admission=ledger.entries.find(e=>e.case===kind&&e.status==='running');assert.ok(admission);
  const deadline=Date.parse(admission.deadlineUtc);openSync(`${out}/started-once`,'wx');
  const runtime=WorkerRuntime.getup_fixture(kind);
  const result={case:kind,phase:'preparation',fixtureValid:false,triggerNative:null,counters:{real:0,forecast:0,total:0},
    checkpoints:[],standingPassed:false,qualification24Cases:false};
  let reply=JSON.parse(runtime.advance(0));
  const fixture=()=>JSON.parse(runtime.getup_fixture_report());
  const checkpoint=label=>{
    const bytes=runtime.getup_checkpoint();const path=`${out}/checkpoint-${label}.bin`;
    const fd=openSync(path,'wx');try{writeFileSync(fd,bytes);fsyncSync(fd);}finally{closeSync(fd);}
    const verification=JSON.parse(runtime.verify_getup_checkpoint(bytes));
    result.checkpoints.push({path,sha256:sha(bytes),...verification});
    writeFileSync(`${out}/reference-report.json`,runtime.getup_report()+'\n');save(`${out}/result.json`,result);
  };
  const record=()=>{
    const s=reply.snapshot,p=s.diagnostics.getup.policy,f=fixture();
    const real=Math.round(s.simulation_time_s*800),forecast=p.forecast_substeps;
    appendFileSync(`${out}/trace.ndjson`,JSON.stringify({...s,fixture:f.state,casePhase:result.phase})+'\n');
    result.final=reply;result.fixture=f.state;
    const prior=result.counters;result.counters={real,forecast,total:real+forecast};
    save(`${out}/result.json`,result);
    assert.equal(s.simulation_time_s,real/800);assert.equal(f.nativeSubsteps,real);
    assert.ok(real>=prior.real&&forecast>=prior.forecast&&forecast<=5000);
    assert.equal(s.stamp.generation,1);assert.equal(s.segments.length,25);
    assert.equal(s.diagnostics.direct_pelvis_force_n,0);assert.equal(s.diagnostics.direct_pelvis_torque_nm,0);
    assert.ok(s.diagnostics.requested_motor_effort_nm<=920.001);
    if(result.phase==='preparation')assert.equal(forecast,0);
  };
  const preparationLimit=kind==='side-supported'?3200:1600;
  let nextCheckpoint=1600;
  try {
    record();checkpoint('initial');
    while(true) {
      if(Date.now()>=deadline-2000){result.outcome='incomplete-wall-budget';break;}
      const state=fixture().state,p=reply.snapshot.diagnostics.getup.policy;
      if(reply.failure) {
        result.outcome=result.phase==='preparation'?'invalid-fixture-integrity'
          :p.phase==='reference-rejected'?'controller-reference-failure':'controller-physical-integrity-failure';
        result.failure=reply.failure;break;
      }
      if(result.phase==='preparation') {
        if(state.ready) {
          result.fixtureValid=true;result.fixtureAtTrigger=state;
          if(deadline-Date.now()<30000){result.outcome='incomplete-budget-before-controller';break;}
          const before=reply.snapshot.segments;
          runtime.trigger_getup_fixture();reply=JSON.parse(runtime.advance(0));
          assert.deepEqual(reply.snapshot.segments,before);
          result.triggerNative=fixture().state.trigger_substep;result.phase='recovery';
          record();checkpoint('trigger');
        } else if(result.counters.real>=preparationLimit) {result.outcome='invalid-fixture-not-settled-or-orientation';break;}
      } else {
        if(p.prototype_standing_passed) {result.standingPassed=true;result.outcome='success';break;}
        if(result.counters.real-result.triggerNative>=20000) {result.outcome='controller-deadline-without-standing';break;}
      }
      const realLimit=result.phase==='preparation'?preparationLimit:result.triggerNative+20000;
      const ticks=Math.min(kind.startsWith('side')&&result.phase==='preparation'?1:4,Math.floor((realLimit-result.counters.real)/20),Math.floor((nextCheckpoint-result.counters.real)/20));
      assert.ok(ticks>0);
      reply=JSON.parse(runtime.advance(ticks));record();
      if(result.counters.real>=nextCheckpoint) {checkpoint(String(result.counters.real));nextCheckpoint+=1600;}
      if(!reply.failure)assert.equal(reply.completed_ticks,ticks);
    }
    result.harnessCompleted=true;
  } catch(error) {result.harnessCompleted=false;result.harnessError=String(error);process.exitCode=1;}
  finally {
    try{checkpoint('final');}catch(error){result.finalCheckpointError=String(error);process.exitCode=1;}
    result.finishedUtc=new Date().toISOString();save(`${out}/result.json`,result);runtime.free();
  }
  console.log(JSON.stringify(result));
}
