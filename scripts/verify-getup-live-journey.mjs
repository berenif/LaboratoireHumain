// Existing UI journey only. No controller replacement. --run is a single trial.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import {execFileSync} from 'node:child_process';import {gzipSync,gunzipSync} from 'node:zlib';
// The previous cycle remains done while the asynchronous continuation import
// is pending. Its terminal flag is not a continuation event.
function continuationEventReady(){
 const s=window.__lhMujoco.state;
 if(document.querySelector('#reset').disabled)return false;
 return !!(s.failed||(s.getup?.continuation===true&&(window.__journey.pauseClickAccepted||s.getup.done)));
}
if(process.argv.includes('--check-waits')){
 const evaluate=new Function('window','document',`return (${continuationEventReady.toString()})();`);
 const check=(getup,{failed=false,disabled=false,pause=false}={})=>evaluate({__lhMujoco:{state:{getup,failed}},__journey:{pauseClickAccepted:pause}},{querySelector:()=>({disabled})});
 assert.equal(check({done:true}),false,'Old completed cycle must not terminate continuation wait');
 assert.equal(check({done:true},{disabled:true}),false);
 assert.equal(check({continuation:true,done:false}),false);
 assert.equal(check({continuation:true,done:false},{pause:true}),true);
 assert.equal(check({continuation:true,done:false},{pause:true,disabled:true}),false);
 assert.equal(check({continuation:true,done:true}),true);
 assert.equal(check({done:true},{failed:true}),true,'Load failure must still escape wait');
 console.log(JSON.stringify({waitRegression:true,cases:7,ticks:0}));process.exit(0);
}
const {chromium}=await import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs');
const restored=process.argv.includes('--restored-pause'),fullFinal=process.argv.includes('--full-journey-final');
assert(!(restored&&fullFinal),'Select one authorized observation only');
const dir=fullFinal?'web/proofs/getup-full-journey-final':restored?'web/proofs/getup-restored-pause':'web/proofs/getup-live-journey',run=process.argv.includes('--run'),before=fullFinal?205659:restored?201659:191459,max=restored?4000:14200;
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
const sha=b=>crypto.createHash('sha256').update(b).digest('hex'),json=p=>JSON.parse(fs.readFileSync(p));
const write=(name,value)=>{fs.writeFileSync(`${dir}/${name}.tmp`,JSON.stringify(value));fs.renameSync(`${dir}/${name}.tmp`,`${dir}/${name}`);};
assert(disk()+(restored?5000000:7000000)<1500000000,'Evidence reserve exceeds disk budget');
fs.mkdirSync(dir,{recursive:true});
if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson.gz`),'Existing/uncertain run: never repeat');
for(const f of ['app.js','engine.js','index.html'])assert.equal(sha(fs.readFileSync(`web/dist/${f}`)),sha(fs.readFileSync(`web/mujoco/${f}`)),'Built/source mismatch: '+f);
assert(json(fullFinal?'web/proofs/getup-restored-pause/ledger.json':restored?'web/proofs/getup-live-journey/ledger.json':'web/proofs/getup-slow-continuation/ledger.json').confirmedTotal===before);
assert(json('web/proofs/getup-slow-continuation/pure.json').pure.pauseCameraZeroIntegrationStub);
const checkpoint=json('web/proofs/getup-browser-cycle/final-state.json');
if(restored){assert.deepEqual(checkpoint,json('web/proofs/getup-live-journey/cycle-final.json'));assert.deepEqual(checkpoint.state,json('web/proofs/getup-live-journey/stopped-state.json').inspection.state);}
const source=execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim();
const protocol={cycleTicks:restored?0:10200,continuationTicks:4000,maximumTicks:max,push:'same torso +X15N,120 native steps after800 calm steps',pause:'during pulse at40 applied native steps; camera then resume; full physical state and pulse counter must remain fixed',continuity:restored?'EXPLICITLY RESTORED endpoint, no fall/get-up replay; no intermediate setter/reset; SLOW sole owner; does NOT validate the interrupted live journey':'one initial known-entry restoration; same Physics object and full233-state handoff; no intermediate setter/reset; SLOW sole owner',success:restored?'exact233-state restoration, actual pause/resume,120 force ticks, last second calm, final pause':'both existing observedHold results, live handoff, actual pause/resume,120 force ticks, last second calm, final pause'};
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox','--disk-cache-size=1','--media-cache-size=1']});
let fd,ledger,cycleFinal,offset=0,latestState,page;
try{
 page=await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});const errors=[];
 page.on('pageerror',e=>errors.push(String(e)));
 await page.goto('http://127.0.0.1:5199/web/dist/index.html?getup=local'+(restored?'&continue=checkpoint':''));await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
 assert.equal(await page.locator('#getup-start').isEnabled(),!restored);assert.equal(await page.locator('#getup-continue').isEnabled(),restored);
 if(!run){
  let restoration=null,reportVisibility=null;
  if(restored)restoration=await page.evaluate(async()=>{
   const {loadCycleAssets}=await import('/web/mujoco/local-getup/cycle.js');
   const {SlowContinuation,loadFinalCheckpoint,FINAL_STATE_SHA}=await import('/web/mujoco/local-getup/continuation.js');
   const {Physics}=await import('/web/dist/engine.js');
   const mj=await(await import('/web/dist/vendor/mujoco.js')).default(),assets=await loadCycleAssets(),checkpoint=await loadFinalCheckpoint(),p=new Physics(mj,assets.xml);p.model.opt.timestep=.00125;
   // Only constructor/forward/state reads, never advance or an integrator.
   const c=new SlowContinuation(p,assets,{checkpoint}),result={checkpoint:FINAL_STATE_SHA,initialState:c.initialState,exact:c.initialState.every((v,i)=>v===checkpoint.state[i]),ticks:p.steps,calm:c.observer.last.calm,restored:c.restoredExact};
   c.dispose();p.dispose();return result;
  });
  if(restored){assert(restoration.exact&&restoration.restored&&restoration.calm&&restoration.ticks===0);assert.deepEqual(restoration.initialState,checkpoint.state);}
  if(fullFinal){
   // UI-only layout fixture in this disposable, paused preflight page. No
   // controller is started; this text is never recorded as a physical result.
   await page.evaluate(()=>{const r=document.querySelector('#getup-result');r.hidden=false;r.textContent='Vérification de cadrage uniquement\nAucun essai lancé\nBilan final lisible\nÉtat en pause\nSans intégration\nFin de la vérification';});
   await page.locator('#getup-result').scrollIntoViewIfNeeded();
   reportVisibility=await page.evaluate(()=>{const r=document.querySelector('#getup-result').getBoundingClientRect(),p=document.querySelector('.panel').getBoundingClientRect();return r.top>=Math.max(0,p.top)&&r.bottom<=Math.min(innerHeight,p.bottom)&&r.left>=0&&r.right<=innerWidth;});
   assert(reportVisibility);assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
  }
  const preflight={source,protocol,ticks:0,restoration,reportVisibility,harnessSha256:sha(fs.readFileSync('scripts/verify-getup-live-journey.mjs')),priorPure:'getup-slow-continuation/pure.json',diskGrowth:disk(),sourceAndBuiltFilesIdentical:true,initialUI:restored?'paused; cycle disabled; explicit checkpoint continuation enabled':'paused; cycle enabled; continuation disabled'};
  write('preflight.json',preflight);console.log(JSON.stringify(preflight));
 }else{
  assert(fs.existsSync(`${dir}/preflight.json`),'Preflight required');
  if(fullFinal){const pf=json(`${dir}/preflight.json`);assert(pf.reportVisibility&&pf.ticks===0);assert.equal(pf.harnessSha256,sha(fs.readFileSync('scripts/verify-getup-live-journey.mjs')),'Harness changed after preflight');}
  ledger={source,protocol,confirmedBefore:before,priorUnknownPreserved:true,newTicks:0,confirmedTotal:before,cycleTicks:0,continuationTicks:0,pendingMaxTicks:0,completed:false};
  fs.writeFileSync(`${dir}/ledger.json`,JSON.stringify(ledger),{flag:'wx'});fd=fs.openSync(`${dir}/poses.ndjson.gz`,'wx');
  await page.exposeFunction('__journeyAudit',message=>{
   const continuing=!!message.summary?.continuation;
   if(message.kind==='start'&&continuing){
    if(restored){
     assert.deepEqual(message.initialState,checkpoint.state,'Full checkpoint restoration differs');assert.equal(message.initialState.length,233);
     assert.equal(message.summary.restoredExact,true);assert.equal(message.identities.checkpoint,sha(fs.readFileSync('web/proofs/getup-browser-cycle/final-state.json')));
     assert.equal(message.physicsSteps,0);ledger.fullStartExact=true;ledger.identities=message.identities;
    }else{
     assert(cycleFinal&&ledger.cycleTicks===10200&&ledger.cycleSummary.observedHold);
     assert.deepEqual(message.initialState,cycleFinal.state,'Full state changed at live handoff');
     assert.equal(message.summary.restoredExact,false);assert.equal(message.identities.checkpoint,null);
     offset=10200;ledger.liveHandoff={full233StateExact:true,restoredCheckpoint:false,samePhysics:message.worldSame,physicsSteps:message.physicsSteps,t:message.row.t};
    }
   }
   assert(message.worldSame&&message.setStateCalls===0,'World changed or intermediate state setter');
   if(message.kind==='reserve'){
    assert.equal(offset+message.ticks,ledger.newTicks);assert(ledger.newTicks+message.pending<=max);
    if(fullFinal&&ledger.newTicks%400===0){const growth=disk();ledger.peakMeasuredDiskGrowth=Math.max(ledger.peakMeasuredDiskGrowth||0,growth);assert(growth+500000<Math.min(1500000000,1490710528+7000000),'Disk reserve exhausted');}
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
  // The restored variant NEVER launches the fall/get-up cycle.
  if(!restored){await page.click('#getup-start');
  await page.waitForFunction(()=>(window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed)&&!document.querySelector('#reset').disabled,{},{timeout:180000});
  assert(!await page.evaluate(()=>window.__lhMujoco.state.failed),await page.locator('#error').textContent());
  assert(cycleFinal&&ledger.cycleSummary.observedHold,'Cycle did not qualify; no continuation');
  const pausedCycle=await page.evaluate(()=>window.__journeyInspect());assert(pausedCycle.paused);
  await page.waitForTimeout(150);assert.deepEqual(await page.evaluate(()=>window.__journeyInspect()),pausedCycle);
  ledger.cyclePause={physicsFrozen:true,t:pausedCycle.time,ticks:pausedCycle.physicsSteps};write('ledger.json',ledger);
  }
  assert(await page.locator('#getup-continue').isEnabled());await page.click('#getup-continue');
  await page.waitForFunction(continuationEventReady,{},{timeout:90000});
  assert(await page.evaluate(()=>window.__journey.pauseClickAccepted),'Did not pause during pulse');
  const paused=await page.evaluate(()=>window.__journeyInspect());assert.equal(paused.pushAppliedTicks,40);assert.equal(paused.physicsSteps,(restored?0:10200)+840);assert(paused.paused&&paused.push);
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
  ledger.finalPauseFrozen=true;const physicalAndPausePassed=(restored?ledger.fullStartExact:ledger.cycleSummary.observedHold&&ledger.liveHandoff.full233StateExact)&&ledger.continuationSummary.observedHold&&ledger.pushPause.fullPhysicalStateFrozen;
  assert(ledger.newTicks<=max);write('ledger.json',ledger);
  if(fullFinal){
   await page.locator('#getup-result').scrollIntoViewIfNeeded();
   const reportVisible=await page.evaluate(()=>{const r=document.querySelector('#getup-result').getBoundingClientRect(),p=document.querySelector('.panel').getBoundingClientRect();return r.top>=Math.max(0,p.top)&&r.bottom<=Math.min(innerHeight,p.bottom)&&r.left>=0&&r.right<=innerWidth;});
   assert(reportVisible,'Final report is outside the captured viewport');assert.equal(JSON.stringify(await page.evaluate(()=>window.__journeyInspect())),frozen);
   ledger.reportVisible=true;ledger.peakMeasuredDiskGrowth=Math.max(ledger.peakMeasuredDiskGrowth||0,disk());write('ledger.json',ledger);
  }
  await page.screenshot({path:`${dir}/ui-final.png`});
  write('ui.json',{state:await page.evaluate(()=>window.__lhMujoco.state),report:await page.locator('#getup-result').textContent(),previous:await page.locator('#getup-previous').textContent(),errors});
  // Publish the combined verdict only after the required visible report and
  // capture are actually preserved. A capture/assertion failure is not PASS.
  ledger.passed=physicalAndPausePassed&&(!fullFinal||ledger.reportVisible);write('ledger.json',ledger);
  console.log(JSON.stringify({passed:ledger.passed,newTicks:ledger.newTicks,confirmedTotal:ledger.confirmedTotal,liveHandoff:ledger.liveHandoff,pushPause:ledger.pushPause,summary:ledger.continuationSummary,diskGrowth:disk()}));
 }
}catch(error){
 if(ledger){
  ledger.passed=false;ledger.harnessError=String(error);write('ledger.json',ledger);
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
