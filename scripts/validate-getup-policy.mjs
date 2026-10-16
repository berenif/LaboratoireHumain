// One reserved real history and one bounded WEAK branch. Never renews a ledger.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
const ledgerPath=process.argv[2]??'docs/evidence/getup-policy-2026-10-16.json';
const ledger=JSON.parse(readFileSync(ledgerPath));
const output=ledger.outputDirectory??'outputs/getup-policy-2026-10-16';
assert.equal(ledger.status,'reserved');
assert.equal(ledger.nativeReservation,23650);
assert.equal(ledger.wallReservationSeconds,900);
assert.equal(ledger.nativeChargedAfter,ledger.nativeChargedBefore+ledger.nativeReservation);
assert.equal(ledger.wallChargedAfterSeconds,ledger.wallChargedBeforeSeconds+ledger.wallReservationSeconds);
assert.ok(ledger.nativeChargedAfter<=ledger.nativeCeiling);
assert.ok(ledger.wallChargedAfterSeconds<=ledger.wallCeilingSeconds);
for(const [path,hash] of Object.entries(ledger.inputs))assert.equal(createHash('sha256').update(readFileSync(path)).digest('hex'),hash,`Changed frozen input: ${path}`);
assert.equal(createHash('sha256').update(readFileSync(ledger.priorClosedRecord)).digest('hex'),ledger.priorClosedRecordSha256);
const prior=JSON.parse(readFileSync(ledger.priorClosedRecord));
assert.equal(prior.status,'closed');
assert.equal(ledger.nativeChargedBefore,prior.nativeChargedAfter);
assert.equal(ledger.wallChargedBeforeSeconds,prior.wallChargedAfterSeconds);
openSync(`${output}/started-once`,'wx');mkdirSync(`${output}/frames`);
const started=performance.now();
const result={startedUtc:new Date().toISOString(),deadlineUtc:new Date(Date.now()+900000).toISOString(),maximumMainNativeSteps:22400,maximumForecastSteps:1250,maximumWallSeconds:900,frames:0,qualification:false};
let browser,page,value,lastNative=0,lastForecast=0;
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','rust/dist/app','0'],{stdio:['ignore','pipe','inherit']});
const timer=setTimeout(()=>{browser?.close();server.kill();},900000);
const save=()=>writeFileSync(`${output}/result.json`,JSON.stringify(result,null,2)+'\n');save();
const retainReport=async()=>{
 const report=await page.evaluate(()=>window.__lhRust.getupReport());
 const serialized=JSON.stringify(report)+'\n';
 writeFileSync(`${output}/reference-report.json`,serialized);
 result.referenceReport={sha256:createHash('sha256').update(serialized).digest('hex'),bytes:Buffer.byteLength(serialized),forecastSteps:report.status.forecast_substeps,teacherFrames:report.teacher_frames.length,studentDecisions:report.student_decisions.length};
 save();return report;
};
try{
 const origin=await new Promise((resolve,reject)=>{server.stdout.on('data',data=>{const url=String(data).match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];if(url)resolve(url);});server.once('error',reject);server.once('exit',code=>reject(Error(`Preview exited ${code}`)));});
 browser=await chromium.launch(browserLaunchOptions(true));page=await browser.newPage({viewport:{width:1280,height:900}});result.browser=browser.version();
 const errors=[];page.on('pageerror',e=>errors.push(String(e)));
 await page.addInitScript(()=>{window.__lhGetUpTrial=true;let api;Object.defineProperty(window,'__lhRust',{get:()=>api,set:value=>{api=value;api.setManual(true);}});});
 await page.goto(`${origin}?renderer=webgl2`);await page.waitForFunction(()=>window.__lhRust?.diagnostics.ready,null,{timeout:45000});
 await page.locator('#camera-focus').click();await page.mouse.move(800,450);await page.mouse.wheel(0,450);
 const record=async()=>{
  value=await page.evaluate(()=>window.__lhRust.diagnostics);const snapshot=value.snapshot,g=snapshot.diagnostics.getup,p=g?.policy;
  result.mainNativeSteps=Math.round(snapshot.simulation_time_s*800);result.forecastSteps=p?.forecast_substeps??0;
  result.totalNativeSteps=result.mainNativeSteps+result.forecastSteps;
  result.final={tick:value.tick,time:value.time,motion:value.motion,failure:value.failure,counters:snapshot.counters,getup:g,diagnostics:snapshot.diagnostics};
  appendFileSync(`${output}/trace.ndjson`,JSON.stringify(snapshot)+'\n');save();
  // Persist the read-only branch report before assertions or screenshot work can fail.
  const report=await retainReport();
  assert.equal(report.status.forecast_substeps,result.forecastSteps);
  assert.equal(report.student_decisions.length,p?.policy_decisions);
  assert.ok(g&&p);assert.equal(g.learned_policy_loaded,true);assert.equal(g.source_bodies,16);assert.equal(g.active_hinges,21);
  assert.equal(value.workerInstances,1);assert.equal(value.generation,1);assert.equal(snapshot.segments.length,25);
  assert.ok(result.mainNativeSteps>=lastNative&&result.mainNativeSteps<=22400);
  assert.ok(result.forecastSteps>=lastForecast&&result.forecastSteps<=1250);assert.ok(result.totalNativeSteps<=23650);
  assert.equal(value.time,result.mainNativeSteps/800);assert.ok(snapshot.counters.strikes<=1);
  assert.equal(snapshot.diagnostics.direct_pelvis_force_n,0);assert.equal(snapshot.diagnostics.direct_pelvis_torque_nm,0);
  if(!value.failure){assert.equal(g.motor_controls.length,21);assert.ok(g.motor_controls.every(x=>Number.isFinite(x)&&Math.abs(x)<=1));}
  assert.ok(snapshot.diagnostics.requested_motor_effort_nm<=920.001);
  await page.screenshot({path:`${output}/frames/${String(result.frames++).padStart(4,'0')}.jpg`,type:'jpeg',quality:75});
  lastNative=result.mainNativeSteps;lastForecast=result.forecastSteps;save();
 };
 await record();assert.equal(value.tick,0);
 while(value.tick<1120){
  assert.ok(performance.now()-started<895000,'Wall deadline');
  if(value.tick===80){await page.locator('#strike').click();await page.waitForFunction(()=>!window.__lhRust.diagnostics.outstanding&&!window.__lhRust.diagnostics.queueDepth);}
  const reply=await page.evaluate(()=>window.__lhRust.advance(2));await record();
  if(reply.failure){result.physicalRefusal=reply.failure;break;}
  assert.equal(reply.completed_ticks,2);
  if(value.snapshot.diagnostics.getup.policy.prototype_standing_passed){result.standingPassed=true;break;}
 }
 const report=await retainReport();
 assert.equal(report.status.forecast_substeps,result.forecastSteps);
 if(result.physicalRefusal&&result.forecastSteps>0){
  assert.ok(report.teacher_initial,'Missing exact branch seed');
  assert.ok(report.teacher_frames.length>0,'Missing teacher decision trace');
  if(result.physicalRefusal.offending?.includes('Teacher Rapier integrity failure'))assert.ok(report.teacher_failure,'Missing native teacher failure');
 }
 assert.deepEqual(errors,[]);assert.equal(value.snapshot.counters.strikes,1);
 result.harnessCompleted=true;result.standingPassed??=false;result.errors=errors;
 result.outcome=result.standingPassed?'prototype-stable-standing':result.physicalRefusal?'stopped-at-refusal':'bounded-history-without-stable-standing';
}catch(error){result.harnessCompleted=false;result.harnessError=String(error);process.exitCode=1;}
finally{
 result.finishedUtc=new Date().toISOString();result.wallSeconds=(performance.now()-started)/1000;save();
 clearTimeout(timer);await browser?.close();server.kill();
 console.log(JSON.stringify({outcome:result.outcome,harnessCompleted:result.harnessCompleted,harnessError:result.harnessError,mainNativeSteps:result.mainNativeSteps,forecastSteps:result.forecastSteps,totalNativeSteps:result.totalNativeSteps,wallSeconds:result.wallSeconds,standingPassed:result.standingPassed,physicalRefusal:result.physicalRefusal}));
}
