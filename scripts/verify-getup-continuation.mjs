// Pure by default. --run is the ONE approved 4000-step browser continuation.
import fs from 'node:fs';import assert from 'node:assert/strict';import {execFileSync} from 'node:child_process';import {gzipSync,gunzipSync} from 'node:zlib';
import {supportStop} from '../web/mujoco/local-getup/continuation.js';
import {CycleObserver} from '../web/mujoco/local-getup/cycle.js';
const {chromium}=await import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs');
const run=process.argv.includes('--run'),dir='web/proofs/getup-slow-continuation',before=187459;
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
assert(disk()+6000000<1500000000,'Need 6 MB evidence reserve');
fs.mkdirSync(dir,{recursive:true});const json=p=>JSON.parse(fs.readFileSync(p));
const write=(name,value)=>{fs.writeFileSync(`${dir}/${name}.tmp`,JSON.stringify(value));fs.renameSync(`${dir}/${name}.tmp`,`${dir}/${name}`);};
if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson.gz`),'Existing/uncertain run: inspect, never repeat');
assert.equal(supportStop({7:200,10:200},[-.001,-.001],1.45,1),null);
assert.match(supportStop({7:200,10:2},[-.001,0],1.45,1),/Perte/);
assert.match(supportStop({},[.002,.002],1.45,1),/envol/);
assert.match(supportStop({7:200,10:200,4:3},[0,0],1.45,1),/non plantaire/);
const old=gunzipSync(fs.readFileSync('web/proofs/getup-browser-cycle/poses.ndjson.gz')).toString().trim().split('\n').map(JSON.parse).at(-1);
const obs=new CycleObserver(40.844*9.81);assert(obs.observe(old).calm);
assert(!obs.observe({...old,physical:old.physical.map((v,i)=>i===37?.051:v)}).calm);
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox','--disk-cache-size=1','--media-cache-size=1']});
let fd,ledger;
try{
 const page=await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});const errors=[];page.on('pageerror',e=>errors.push(String(e)));
 await page.goto('http://127.0.0.1:5199/web/dist/index.html');await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert(await page.locator('#getup-panel').isHidden());assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
 await page.goto('http://127.0.0.1:5199/web/dist/index.html?getup=local&continue=checkpoint');await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert(await page.locator('#getup-start').isDisabled());assert(await page.locator('#getup-continue').isEnabled());
 const pure=await page.evaluate(async()=>{
  const {loadCycleAssets}=await import('/web/mujoco/local-getup/cycle.js');
  const {SlowContinuation,loadFinalCheckpoint,FINAL_STATE_SHA}=await import('/web/mujoco/local-getup/continuation.js');
  const {physicalObservation,observation,controls}=await import('/web/mujoco/local-getup/slow.js');
  const {Physics}=await import('/web/dist/engine.js');const mj=await(await import('/web/dist/vendor/mujoco.js')).default();
  const assets=await loadCycleAssets(),checkpoint=await loadFinalCheckpoint(),p=new Physics(mj,assets.xml);p.model.opt.timestep=.00125;
  const c=new SlowContinuation(p,assets,{checkpoint}),same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
  const q=Array.from(p.data.qpos),v=Array.from(p.data.qvel),t=p.data.time;
  const expected=controls(assets.terminal.slice(0,21),assets.actor.infer(observation(physicalObservation(p),assets.terminal,assets.terminal)).action,q.slice(7),v.slice(6),true);
  c.command();const actual=Array.from(p.data.ctrl);p.forward();
  const gearError=Math.max(...p.joints.map(j=>Math.abs(p.data.qfrc_actuator[j.v]-p.data.ctrl[j.a]*j.gear)));
  const firstCommand={actual,exactHistoricalHolding:same(actual,expected),maxRatio:Math.max(...actual.map(Math.abs)),gearError,rootZero:p.data.qfrc_actuator.slice(0,6).every(x=>x===0)};
  // Exercise only the existing user-force writer; no integration. The impulse
  // below is a scheduled-force sum, not a measured physical impulse.
  p.smallPush('forward');let forceSum=0,forceExact=true;
  for(let i=0;i<120;i++){p.data.xfrc_applied.fill(0);p.applyPush();forceSum+=p.data.xfrc_applied[6]*p.dt;forceExact&&=p.data.xfrc_applied.every((v,i)=>v===(i===6?15:0));}
  const pulseEnded=!p.push;p.release();p.push=null;
  const result={ticks:p.steps,checkpoint:FINAL_STATE_SHA,fullRestorationExact:same(c.initialState,checkpoint.state),initialTime:t,stateLength:c.initialState.length,initialCalm:c.observer.last.calm,firstCommand,forceExact,pulseEnded,scheduledImpulseNs:forceSum,
   physicalStateUnchanged:same(q,Array.from(p.data.qpos))&&same(v,Array.from(p.data.qvel))&&t===p.data.time,
   nextTicks:c.nextTicks,rootAndOtherForcesZero:p.data.qfrc_applied.every(x=>x===0)&&p.data.xfrc_applied.every(x=>x===0)};
  // Synthetic controller counters only: no world stepping. A lost quiet window
  // must prevent the pulse; a failure on the last tick must never become PASS.
  c.ticks=800;c.calmTicks=780;let refused=false;try{c.advance();}catch{refused=true;}
  result.noPushWithoutCalm=refused&&!p.push&&p.steps===0;
  c.ticks=4000;c.calmTicks=800;c.pushAppliedTicks=120;p.lastForce=15;c.finish('Erreur synthétique au dernier tick');
  result.errorAtBudgetIsFailure=!c.summary().observedHold&&p.lastForce===0;
  c.dispose();
  // Attach to a living completed scenario: no checkpoint/restore at all.
  const live=new SlowContinuation(p,assets,{previousSummary:checkpoint.summary});
  result.liveAttachmentUnchanged=same(q,Array.from(p.data.qpos))&&same(v,Array.from(p.data.qvel))&&t===p.data.time&&!live.restoredExact;
  live.dispose();p.dispose();return result;
 });
 assert(pure.ticks===0&&pure.fullRestorationExact&&pure.initialCalm&&pure.stateLength===233&&pure.nextTicks===20);
 assert(pure.firstCommand.exactHistoricalHolding&&pure.firstCommand.maxRatio<=1&&pure.firstCommand.gearError<2e-4&&pure.firstCommand.rootZero);
 assert(pure.forceExact&&pure.pulseEnded&&Math.abs(pure.scheduledImpulseNs-2.25)<1e-12&&pure.physicalStateUnchanged&&pure.rootAndOtherForcesZero&&pure.liveAttachmentUnchanged&&pure.noPushWithoutCalm&&pure.errorAtBudgetIsFailure);
 assert.equal(errors.length,0);
 if(!run){
  // Browser scheduling test with advance replaced by a NO-INTEGRATION stub.
  // Reproduces the busy-control refresh path without another physical trial.
  await page.evaluate(async()=>{
   const {SlowContinuation}=await import('/web/mujoco/local-getup/continuation.js');window.__pureAdvances=0;
   SlowContinuation.prototype.advance=function(){window.__pureAdvances++;return this.last;};
  });
  await page.click('#getup-continue');await page.waitForFunction(()=>window.__pureAdvances>=2);
  await page.click('#pause');assert(await page.evaluate(()=>window.__lhMujoco.state.paused));
  const calls=await page.evaluate(()=>window.__pureAdvances);await page.click('#zoom-in');await page.click('#camera-reset');await page.waitForTimeout(100);
  assert.equal(await page.evaluate(()=>window.__pureAdvances),calls);assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
  await page.click('#pause');await page.waitForFunction(c=>window.__pureAdvances>c,calls);await page.click('#pause');
  assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);pure.pauseCameraZeroIntegrationStub=true;
  write('pure.json',{pure,criteria:'same calm thresholds; native-step physical stop; one 120-step 15N+X pulse after 800 calm steps; last 800 steps calm; 4000 maximum',diskGrowth:disk()});console.log(JSON.stringify({pure,ticks:0}));
 }
 else{
  assert(fs.existsSync(`${dir}/pure.json`),'Pure checks must pass before runtime');
  ledger={source:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),confirmedBefore:before,priorUnknownPreserved:true,maximumNewTicks:4000,newTicks:0,pendingMaxTicks:0,completed:false,pure};
  fs.writeFileSync(`${dir}/ledger.json`,JSON.stringify(ledger),{flag:'wx'});fd=fs.openSync(`${dir}/poses.ndjson.gz`,'wx');
  await page.exposeFunction('__lhGetupAudit',message=>{
   if(message.kind==='reserve'){assert.equal(ledger.newTicks,message.ticks);assert(message.ticks+message.pending<=4000);ledger.pendingMaxTicks=message.pending;write('ledger.json',ledger);}
   else if(message.kind==='start'||message.kind==='sample'){
    fs.writeSync(fd,gzipSync(JSON.stringify(message.row)+'\n'));fs.fsyncSync(fd);
    ledger.newTicks=message.summary.ticks;ledger.pendingMaxTicks=0;ledger.summary=message.summary;ledger.confirmedTotal=before+ledger.newTicks;
    if(message.kind==='start'){ledger.identities=message.identities;assert.deepEqual(message.initialState,json('web/proofs/getup-browser-cycle/final-state.json').state);ledger.fullStartExact=true;}
    write('ledger.json',ledger);
   }else if(message.kind==='final'){
    ledger.newTicks=message.summary.ticks;ledger.pendingMaxTicks=0;ledger.summary=message.summary;ledger.confirmedTotal=before+ledger.newTicks;ledger.completed=true;
    write('final-state.json',{state:message.state,spec:message.spec,summary:message.summary});write('ledger.json',ledger);
   }
  });
  await page.click('#getup-continue');
  await page.waitForFunction(()=>window.__lhMujoco.state.getup?.ticks>=400||window.__lhMujoco.state.getup?.done,{},{timeout:30000});
  if(!await page.evaluate(()=>window.__lhMujoco.state.getup.done)){
   await page.click('#pause');assert(await page.evaluate(()=>window.__lhMujoco.state.paused));
   const pausedTicks=await page.evaluate(()=>window.__lhMujoco.state.steps);
   await page.click('#zoom-in');await page.click('#camera-reset');await page.waitForTimeout(100);
   assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),pausedTicks);ledger.pauseCameraCheckedAt=pausedTicks;write('ledger.json',ledger);
   await page.click('#pause');
  }
  await page.waitForFunction(()=>window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed,{},{timeout:120000});
  await page.waitForTimeout(150);const state=await page.evaluate(()=>window.__lhMujoco.state);
  assert(ledger.completed&&ledger.pendingMaxTicks===0&&state.paused&&state.steps===ledger.newTicks,'Final accounting/paused state');
  assert(await page.locator('#mode').isDisabled());assert(await page.locator('#push').isDisabled());assert(await page.locator('#getup-continue').isDisabled());
  await page.click('#zoom-in');await page.click('#camera-reset');assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),ledger.newTicks);
  await page.screenshot({path:`${dir}/ui-final.png`});
  write('ui.json',{state,errors,persistentReport:await page.locator('#getup-result').textContent(),diskGrowth:disk()});
  console.log(JSON.stringify({completed:ledger.completed,newTicks:ledger.newTicks,confirmedTotal:ledger.confirmedTotal,summary:ledger.summary,diskGrowth:disk()}));
 }
}finally{if(fd!==undefined)fs.closeSync(fd);await browser.close();}
