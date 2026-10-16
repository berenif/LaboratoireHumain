// Bounded real browser smoke. Construction/tests/encoding perform no hidden stepping.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
const output='outputs/getup-body-replacement-2026-10-16';
const ledger=JSON.parse(readFileSync('docs/evidence/getup-body-replacement-2026-10-16.json'));
assert.equal(ledger.status,'reserved');
for(const [path,hash] of Object.entries(ledger.inputs))assert.equal(createHash('sha256').update(readFileSync(path)).digest('hex'),hash,`Changed frozen input: ${path}`);
openSync(`${output}/started-once`,'wx');mkdirSync(`${output}/frames`);
const result={startedUtc:new Date().toISOString(),maximumTicks:160,maximumWallSeconds:180,frames:0,learnedPolicyIntegrated:false,qualification:false};
const started=performance.now();let browser;
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','rust/dist/app','0'],{stdio:['ignore','pipe','inherit']});
const timer=setTimeout(()=>{browser?.close();server.kill();},180000);
try{
 const origin=await new Promise((resolve,reject)=>{server.stdout.on('data',data=>{const url=String(data).match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];if(url)resolve(url);});server.once('error',reject);server.once('exit',code=>reject(Error(`Preview exited ${code}`)));});
 browser=await chromium.launch(browserLaunchOptions(true));const page=await browser.newPage({viewport:{width:1280,height:900}});result.browser=browser.version();
 const errors=[];page.on('pageerror',e=>errors.push(String(e)));
 await page.addInitScript(()=>{let api;Object.defineProperty(window,'__lhRust',{get:()=>api,set:value=>{api=value;api.setManual(true);}});});
 await page.goto(`${origin}?renderer=webgl2`);await page.waitForFunction(()=>window.__lhRust?.diagnostics.ready,null,{timeout:45000});
 await page.locator('#camera-focus').click();
 const read=()=>page.evaluate(()=>window.__lhRust.diagnostics);let value=await read();assert.equal(value.tick,0);
 const record=async()=>{
  value=await read();result.nativeSteps=value.snapshot.stamp.sequence-1;
  assert.ok(result.nativeSteps<=3200);
  result.final={tick:value.tick,time:value.time,motion:value.motion,failure:value.failure,counters:value.snapshot.counters,diagnostics:value.snapshot.diagnostics};
  appendFileSync(`${output}/trace.ndjson`,JSON.stringify(value.snapshot)+'\n');
  writeFileSync(`${output}/result.json`,JSON.stringify(result,null,2)+'\n');
  const g=value.snapshot.diagnostics.getup;
  assert.ok(g);assert.equal(g.source_bodies,16);assert.equal(g.active_hinges,21);assert.equal(g.coordinate_frames,9);assert.equal(g.learned_policy_loaded,false);
  assert.equal(g.joint_angles_rad.length,21);assert.equal(g.observation.length,67);assert.ok(g.motor_controls.every(x=>Number.isFinite(x)&&Math.abs(x)<=1));
  assert.equal(value.time,result.nativeSteps/800);assert.equal(value.workerInstances,1);
  assert.equal(value.snapshot.segments.filter(p=>!p.id.startsWith('@')).length,16);
  assert.ok(value.snapshot.diagnostics.requested_motor_effort_nm<=920.001);
  await page.screenshot({path:`${output}/frames/${String(result.frames++).padStart(4,'0')}.jpg`,type:'jpeg',quality:75});
  writeFileSync(`${output}/result.json`,JSON.stringify(result,null,2)+'\n');
 };
 await record();
 while(value.tick<160){
  assert.ok(performance.now()-started<175000,'Wall deadline');
  if(value.tick===80){await page.locator('#strike').click();await page.waitForFunction(()=>!window.__lhRust.diagnostics.outstanding&&!window.__lhRust.diagnostics.queueDepth);}
  const reply=await page.evaluate(()=>window.__lhRust.advance(2));await record();
  assert.equal(reply.failure,null);assert.equal(reply.completed_ticks,2);
 }
 assert.deepEqual(errors,[]);assert.equal(value.snapshot.counters.strikes,1);assert.equal(value.snapshot.counters.recoveries,0);
 result.passed=true;result.errors=errors;
}catch(error){result.passed=false;result.error=String(error);process.exitCode=1;}
finally{result.finishedUtc=new Date().toISOString();result.wallSeconds=(performance.now()-started)/1000;writeFileSync(`${output}/result.json`,JSON.stringify(result,null,2)+'\n');clearTimeout(timer);await browser?.close();server.kill();console.log(JSON.stringify(result));}
