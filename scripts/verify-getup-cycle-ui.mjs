// Default: replay/pure/browser wiring only. --run: exactly ONE approved UI cycle.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {spawn,execFileSync} from 'node:child_process';
import {gzipSync} from 'node:zlib';
import {CycleObserver} from '../web/mujoco/local-getup/cycle.js';
const {chromium}=await import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs');
const run=process.argv.includes('--run'),dir='web/proofs/getup-browser-cycle';
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
assert(disk()+8000000<1500000000,'Insufficient reserve for compact evidence');
fs.mkdirSync(dir,{recursive:true});
const json=p=>JSON.parse(fs.readFileSync(p));
const write=(name,value)=>{fs.writeFileSync(`${dir}/${name}.tmp`,JSON.stringify(value));fs.renameSync(`${dir}/${name}.tmp`,`${dir}/${name}`);};
if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson.gz`),'Existing/uncertain run: inspect; never repeat');
const rows=fs.readFileSync('web/proofs/getup-continuous-slow/poses.ndjson','utf8').trim().split('\n').map(JSON.parse);
const weight=40.844*9.81,observer=new CycleObserver(weight);rows.forEach(r=>observer.observe(r));
const replay=observer.summary();assert(replay.observedHold&&replay.calmSamples===85&&replay.newFlightSamples===0&&Math.abs(replay.receptionTime-18.85)<1e-8);
assert.equal(json('web/proofs/getup-continuous-slow/ledger.json').passed,false);
// Real recorded air, moved only in a synthetic observer test after reception:
// must be classified as NEW flight, never as the initial waiting phase.
observer.observe({...rows.find(r=>Math.abs(r.t-18.8)<1e-8),t:22.525,phase:'SLOW-holding'});
assert.equal(observer.summary().newFlightSamples,1);assert.equal(observer.summary().calmSamples,0);
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','.','5199'],{stdio:['ignore','pipe','inherit']});
await new Promise((resolve,reject)=>{server.stdout.once('data',resolve);server.once('error',reject);server.once('exit',c=>reject(Error(`Server exit ${c}`)));});
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox','--disk-cache-size=1','--media-cache-size=1']});
let fd,ledger;
try{
 const page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
 const errors=[];page.on('pageerror',e=>errors.push(String(e)));
 const requests=[];page.on('request',r=>requests.push(r.url()));
 await page.goto('http://127.0.0.1:5199/web/dist/index.html');await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert(await page.locator('#getup-panel').isHidden());assert(!requests.some(u=>u.includes('student.bin')));
 await page.goto('http://127.0.0.1:5199/web/dist/index.html?getup=local');await page.waitForFunction(()=>window.__lhMujoco?.state);
 assert(await page.locator('#getup-panel').isVisible());
 // No stepping: reset, selection, camera and grab/release still work paused.
 await page.selectOption('#mode','balance');await page.click('#reset');await page.click('#reset');
 const pos=await page.evaluate(()=>window.__lhMujoco.state.screen.find(p=>p.name==='left_hand'));
 const rect=await page.locator('#simulation').boundingBox();
 await page.mouse.move(rect.x+pos.x,rect.y+pos.y);await page.mouse.down();
 assert(await page.evaluate(()=>!!window.__lhMujoco.state.grab));await page.mouse.up();
 assert.equal(await page.evaluate(()=>window.__lhMujoco.state.grab),null);
 await page.click('#zoom-in');await page.click('#camera-reset');await page.selectOption('#mode','passive');
 assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),0);
 const pure=await page.evaluate(async()=>{
  const {loadCycleAssets,GetUpCycle}=await import('/web/mujoco/local-getup/cycle.js');
  const {Physics}=await import('/web/dist/engine.js');const load=(await import('/web/dist/vendor/mujoco.js')).default;
  const assets=await loadCycleAssets(),mj=await load(),p=new Physics(mj,assets.xml),cycle=new GetUpCycle(p,assets);
  const initial=cycle.last,spec=assets.entry.spec,state=cycle.stateVector();
  const same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
  const ok=same(initial.qpos,assets.entry.qpos)&&same(initial.qvel,assets.entry.qvel)&&initial.t===assets.entry.time&&p.steps===0;
  const result={ok,initialTime:initial.t,stateLength:state.length,spec,identities:assets.identities,ticks:p.steps,nextTicks:cycle.nextTicks};
  // Evaluate the first real SLOW command on the existing fallen checkpoint;
  // forward/actuation only, no mj_step/mj_step1/mj_step2 integration.
  const fallen=await(await fetch('/web/proofs/getup-continuous-slow/fallen-entry.json')).json();
  const {physicalObservation,observation,controls}=await import('/web/mujoco/local-getup/slow.js');
  mj.mj_setState(p.model,p.data,fallen.state,fallen.spec);p.model.opt.timestep=.00125;
  const q=Array.from(p.data.qpos),dq=Array.from(p.data.qvel),now=cycle.at(0);
  const residual=assets.actor.infer(observation(physicalObservation(p),now,cycle.at(5))).action;
  p.data.ctrl.set(controls(now.slice(0,21),residual,q.slice(7),dq.slice(6),false));p.forward();
  result.firstSlowCommand={finiteBounded:p.data.ctrl.every(v=>Number.isFinite(v)&&Math.abs(v)<=1),
   gearError:Math.max(...p.joints.map(j=>Math.abs(p.data.qfrc_actuator[j.v]-p.data.ctrl[j.a]*j.gear))),
   rootUnactuated:p.data.qfrc_actuator.slice(0,6).every(v=>v===0),stateUnchanged:same(q,Array.from(p.data.qpos))&&same(dq,Array.from(p.data.qvel))};
  cycle.dispose();p.dispose();return result;
 });
 assert(pure.ok&&pure.ticks===0&&pure.stateLength===233&&pure.nextTicks===5);
 assert(pure.firstSlowCommand.finiteBounded&&pure.firstSlowCommand.gearError<2e-4&&pure.firstSlowCommand.rootUnactuated&&pure.firstSlowCommand.stateUnchanged);
 if(!run){
  assert.equal(errors.length,0);write('pure.json',{pure,replay,ui:'default unchanged; local opt-in; paused reset/mode/camera/grab/release checked',ticks:0,diskGrowth:disk()});
  await page.evaluate(()=>document.querySelector('#getup-panel').scrollIntoView({block:'nearest'}));await page.screenshot({path:`${dir}/ui-ready.png`});console.log(JSON.stringify({pure,replay,ticks:0}));
 }else{
  assert(fs.existsSync(`${dir}/pure.json`),'Pure checks required first');
  ledger={confirmedBefore:177259,priorUnknownPreserved:true,newTicks:0,pendingMaxTicks:0,completed:false,source:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),maximumNewTicks:10200,pure};
  fs.writeFileSync(`${dir}/ledger.json`,JSON.stringify(ledger),{flag:'wx'});fd=fs.openSync(`${dir}/poses.ndjson.gz`,'wx');
  await page.exposeFunction('__lhGetupAudit',message=>{
   if(message.kind==='reserve'){
    assert.equal(ledger.newTicks,message.ticks);assert(message.ticks+message.pending<=10200);
    ledger.pendingMaxTicks=message.pending;write('ledger.json',ledger);
   }else if(message.kind==='start'||message.kind==='sample'){
    fs.writeSync(fd,gzipSync(JSON.stringify(message.row)+'\n'));fs.fsyncSync(fd);
    ledger.newTicks=message.summary.ticks;ledger.pendingMaxTicks=0;ledger.summary=message.summary;ledger.confirmedTotal=177259+ledger.newTicks;
    if(message.kind==='start')ledger.identities=message.identities;
    write('ledger.json',ledger);
   }else if(message.kind==='final'){
    ledger.newTicks=message.summary.ticks;ledger.pendingMaxTicks=0;ledger.summary=message.summary;ledger.confirmedTotal=177259+ledger.newTicks;ledger.completed=true;
    write('final-state.json',{state:message.state,spec:message.spec,summary:message.summary});write('ledger.json',ledger);
   }
  });
  // The only UI launch in this script. Neither hooks nor acceptance tests step the world.
  await page.click('#getup-start');
  await page.waitForFunction(()=>window.__lhMujoco.state.getup?.done||window.__lhMujoco.state.failed,{},{timeout:180000});
  await page.waitForTimeout(150);
  const state=await page.evaluate(()=>window.__lhMujoco.state);assert(!state.failed,await page.locator('#error').textContent());
  assert(ledger.completed&&ledger.pendingMaxTicks===0&&state.paused&&state.steps===ledger.newTicks);
  assert(await page.locator('#mode').isDisabled());assert(await page.locator('#push').isDisabled());assert(await page.locator('#small-push').isDisabled());
  const finalSteps=state.steps;await page.waitForTimeout(1200);assert.equal(await page.evaluate(()=>window.__lhMujoco.state.steps),finalSteps);
  await page.screenshot({path:`${dir}/ui-final.png`});
  ledger.ui={paused:true,terminalTicksFrozen:true,controlsLocked:true,result:await page.locator('#getup-result').textContent(),errors};ledger.diskGrowth=disk();write('ledger.json',ledger);
  // Explicit reset after the saved terminal state; zero new integrations, MVP available.
  await page.click('#reset');const reset=await page.evaluate(()=>window.__lhMujoco.state);
  assert(reset.paused&&reset.time===0&&!reset.getup&&reset.steps===finalSteps);assert(await page.locator('#mode').isEnabled());
  ledger.ui.resetReturnsToMVP=true;write('ledger.json',ledger);
  assert.equal(errors.length,0);console.log(JSON.stringify(ledger));
 }
}finally{if(fd!==undefined)fs.closeSync(fd);await browser.close();server.kill();}
