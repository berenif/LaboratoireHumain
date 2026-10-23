// Existing UI journey only. No controller replacement. --run is a single trial.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import {execFileSync} from 'node:child_process';import {gzipSync,gunzipSync} from 'node:zlib';
const {chromium}=await import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs');
const dir='web/proofs/getup-live-journey',run=process.argv.includes('--run'),before=191459,max=14200;
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
const sha=b=>crypto.createHash('sha256').update(b).digest('hex'),json=p=>JSON.parse(fs.readFileSync(p));
const write=(name,value)=>{fs.writeFileSync(`${dir}/${name}.tmp`,JSON.stringify(value));fs.renameSync(`${dir}/${name}.tmp`,`${dir}/${name}`);};
assert(disk()+7000000<1500000000,'Need 7 MB for compact poses/state/media');
fs.mkdirSync(dir,{recursive:true});
if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson.gz`),'Existing/uncertain run: never repeat');
for(const f of ['app.js','engine.js','index.html'])assert.equal(sha(fs.readFileSync(`web/dist/${f}`)),sha(fs.readFileSync(`web/mujoco/${f}`)),'Built/source mismatch: '+f);
assert(json('web/proofs/getup-slow-continuation/ledger.json').confirmedTotal===before);
assert(json('web/proofs/getup-slow-continuation/pure.json').pure.pauseCameraZeroIntegrationStub);
const source=execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim();
const protocol={cycleTicks:10200,continuationTicks:4000,maximumTicks:max,push:'same torso +X15N,120 native steps after800 calm steps',pause:'during pulse at40 applied native steps; camera then resume; full physical state and pulse counter must remain fixed',continuity:'one initial known-entry restoration; same Physics object and full233-state handoff; no intermediate setter/reset; SLOW sole owner',success:'both existing observedHold results, live handoff, actual pause/resume,120 force ticks, last second calm, final pause'};
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox','--disk-cache-size=1','--media-cache-size=1']});
let fd,ledger,cycleFinal,offset=0,latestState,page;
try{
 page=await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});const errors=[];
 page.on('pageerror',e=>errors.push(String(e)));
 await page.goto('http://127.0.0.1:5199/web/dist/index.html?getup=local');await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
 assert(await page.locator('#getup-start').isEnabled());assert(await page.locator('#getup-continue').isDisabled());
 if(!run){
  const preflight={source,protocol,ticks:0,priorPure:'getup-slow-continuation/pure.json',diskGrowth:disk(),sourceAndBuiltFilesIdentical:true,initialUI:'paused; cycle enabled; continuation disabled'};
  write('preflight.json',preflight);console.log(JSON.stringify(preflight));
 }else{
  assert(fs.existsSync(`${dir}/preflight.json`),'Preflight required');
  ledger={source,protocol,confirmedBefore:before,priorUnknownPreserved:true,newTicks:0,confirmedTotal:before,cycleTicks:0,continuationTicks:0,pendingMaxTicks:0,completed:false};
  fs.writeFileSync(`${dir}/ledger.json`,JSON.stringify(ledger),{flag:'wx'});fd=fs.openSync(`${dir}/poses.ndjson.gz`,'wx');
  await page.exposeFunction('__journeyAudit',message=>{
   const continuing=!!message.summary?.continuation;
   if(message.kind==='start'&&continuing){
    assert(cycleFinal&&ledger.cycleTicks===10200&&ledger.cycleSummary.observedHold);
    assert.deepEqual(message.initialState,cycleFinal.state,'Full state changed at live handoff');
    assert.equal(message.summary.restoredExact,false);assert.equal(message.identities.checkpoint,null);
    offset=10200;ledger.liveHandoff={full233StateExact:true,restoredCheckpoint:false,samePhysics:message.worldSame,physicsSteps:message.physicsSteps,t:message.row.t};
   }
   assert(message.worldSame&&message.setStateCalls===0,'World changed or intermediate state setter');
   if(message.kind==='reserve'){
    assert.equal(offset+message.ticks,ledger.newTicks);assert(ledger.newTicks+message.pending<=max);
    ledger.pendingMaxTicks=message.pending;write('ledger.json',ledger);
   }else if(message.kind==='start'||message.kind==='sample'){
    const total=offset+message.summary.ticks;assert.equal(message.physicsSteps,total);assert(total<=max&&total>=ledger.newTicks);
    // Duplicate handoff pose is retained in raw evidence with zero elapsed time.
    fs.writeSync(fd,gzipSync(JSON.stringify({...message.row,globalTicks:total})+'\n'));fs.fsyncSync(fd);
    ledger.newTicks=total;ledger.confirmedTotal=before+total;ledger.pendingMaxTicks=0;
    if(continuing){ledger.continuationTicks=message.summary.ticks;ledger.continuationSummary=message.summary;}
    else{ledger.cycleTicks=message.summary.ticks;ledger.cycleSummary=message.summary;}
    if(message.kind==='start'&&!continuing)ledger.identities=message.identities;
    write('ledger.json',ledger);
   }else if(message.kind==='final'){
    assert.equal(offset+message.summary.ticks,ledger.newTicks);ledger.pendingMaxTicks=0;
    const state={state:message.state,spec:message.spec,summary:message.summary};
    if(!continuing){cycleFinal=state;write('cycle-final.json',state);ledger.cycleSummary=message.summary;}
    else{write('final-state.json',state);ledger.continuationSummary=message.summary;ledger.completed=true;}
    write('ledger.json',ledger);
   }
  });
  await page.evaluate(async()=>{
   const {GetUpCycle}=await import('/web/mujoco/local-getup/cycle.js');
   const original=GetUpCycle.prototype.snapshot;
   const audit={world:null,current:null,setStateCalls:0,pauseTriggered:false,pauseClickAccepted:false};window.__journey=audit;
   // Read-only instrumentation around the existing snapshot. No alternate
   // advance, inference, control or physical path is installed for this trial.
   GetUpCycle.prototype.snapshot=function(){
    if(!audit.world){
     audit.world=this.p;
     // The sole initial restoration already occurred before this first snapshot.
     this.p.mj.mj_setState=()=>{audit.setStateCalls++;throw Error('Intermediate restoration forbidden');};
    }
    if(audit.world!==this.p)throw Error('Physics object replaced between phases');
    audit.current=this;return original.call(this);
   };
   window.__lhGetupAudit=async message=>{
    await window.__journeyAudit({...message,physicsSteps:audit.current.p.steps,worldSame:audit.current.p===audit.world,setStateCalls:audit.setStateCalls});
    if(message.kind==='sample'&&message.summary.continuation&&message.summary.pushAppliedTicks===40&&!audit.pauseTriggered){
     audit.pauseTriggered=true;
     // Queue an actual button activation AFTER advance/audit have completed and
     // busy has cleared, BEFORE the next scheduled physics frame.
     requestAnimationFrame(()=>{const button=document.querySelector('#pause');audit.pauseClickEnabled=!button.disabled;button.click();audit.pauseClickAccepted=window.__lhMujoco.state.paused;audit.pauseWallStart=performance.now();});
    }
   };
   window.__journeyInspect=()=>{const c=audit.current;return {state:c.stateVector(),physicsSteps:c.p.steps,ticks:c.ticks,pushAppliedTicks:c.pushAppliedTicks||0,push:c.p.push?structuredClone(c.p.push):null,time:c.p.data.time,setStateCalls:audit.setStateCalls,sameWorld:c.p===audit.world,paused:window.__lhMujoco.state.paused,camera:window.__lhMujoco.state.camera};};
  });
  // The one cycle launch; its only restoration is the already-known initial entry.
  await page.click('#getup-start');
  await page.waitForFunction(()=>(window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed)&&!document.querySelector('#reset').disabled,{},{timeout:180000});
  assert(!await page.evaluate(()=>window.__lhMujoco.state.failed),await page.locator('#error').textContent());
  assert(cycleFinal&&ledger.cycleSummary.observedHold,'Cycle did not qualify; no continuation');
  const pausedCycle=await page.evaluate(()=>window.__journeyInspect());assert(pausedCycle.paused);
  await page.waitForTimeout(150);assert.deepEqual(await page.evaluate(()=>window.__journeyInspect()),pausedCycle);
  ledger.cyclePause={physicsFrozen:true,t:pausedCycle.time,ticks:pausedCycle.physicsSteps};write('ledger.json',ledger);
  assert(await page.locator('#getup-continue').isEnabled());await page.click('#getup-continue');
  await page.waitForFunction(()=>window.__journey.pauseClickAccepted||((window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed)&&!document.querySelector('#reset').disabled),{},{timeout:90000});
  assert(await page.evaluate(()=>window.__journey.pauseClickAccepted),'Did not pause during pulse');
  const paused=await page.evaluate(()=>window.__journeyInspect());assert.equal(paused.pushAppliedTicks,40);assert.equal(paused.physicsSteps,11040);assert(paused.paused&&paused.push);
  await page.click('#zoom-in');const camera=await page.evaluate(()=>window.__lhMujoco.state.camera);assert.notDeepEqual(camera,paused.camera);
  await page.click('#camera-reset');await page.waitForTimeout(250);
  const stayed=await page.evaluate(()=>window.__journeyInspect());assert.deepEqual(stayed,paused,'Physical/pulse state advanced while paused');
  ledger.pushPause={atGlobalTick:paused.physicsSteps,atContinuationTick:paused.ticks,appliedForceTicks:paused.pushAppliedTicks,fullPhysicalStateFrozen:true,pendingPulseFrozen:true,cameraChanged:true,wallMs:await page.evaluate(()=>performance.now()-window.__journey.pauseWallStart)};
  write('pause.json',{before:paused,after:stayed,verification:ledger.pushPause});write('ledger.json',ledger);
  await page.click('#pause');
  await page.waitForFunction(()=>(window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed)&&!document.querySelector('#reset').disabled,{},{timeout:90000});
  latestState=await page.evaluate(()=>window.__journeyInspect());
  assert(ledger.completed&&ledger.pendingMaxTicks===0&&latestState.paused&&latestState.physicsSteps===ledger.newTicks);
  const frozen=JSON.stringify(latestState);await page.waitForTimeout(150);assert.equal(JSON.stringify(await page.evaluate(()=>window.__journeyInspect())),frozen);
  assert.equal(errors.length,0);assert.equal(ledger.continuationSummary.pushAppliedTicks,120);
  ledger.finalPauseFrozen=true;ledger.passed=ledger.cycleSummary.observedHold&&ledger.continuationSummary.observedHold&&ledger.liveHandoff.full233StateExact&&ledger.pushPause.fullPhysicalStateFrozen;
  assert(ledger.newTicks<=max);write('ledger.json',ledger);
  await page.screenshot({path:`${dir}/ui-final.png`});
  write('ui.json',{state:await page.evaluate(()=>window.__lhMujoco.state),report:await page.locator('#getup-result').textContent(),previous:await page.locator('#getup-previous').textContent(),errors});
  console.log(JSON.stringify({passed:ledger.passed,newTicks:ledger.newTicks,confirmedTotal:ledger.confirmedTotal,liveHandoff:ledger.liveHandoff,pushPause:ledger.pushPause,summary:ledger.continuationSummary,diskGrowth:disk()}));
 }
}catch(error){
 if(ledger){
  ledger.harnessError=String(error);write('ledger.json',ledger);
  try{
   // Stop through the existing blur/pause handler, then let any already-reserved
   // decision finish before reading. Never restart or restore after a failure.
   await page.evaluate(()=>window.dispatchEvent(new Event('blur')));
   await page.waitForFunction(()=>window.__lhMujoco.state.paused&&!document.querySelector('#reset').disabled,{},{timeout:5000});
   const stopped=await page.evaluate(()=>({inspection:window.__journeyInspect(),summary:window.__lhMujoco.state.getup}));
   write('stopped-state.json',stopped);ledger.stoppedStateSaved=true;write('ledger.json',ledger);
  }catch(saveError){ledger.stopSaveError=String(saveError);write('ledger.json',ledger);}
 }
 throw error;
}finally{if(fd!==undefined)fs.closeSync(fd);await browser.close();}
