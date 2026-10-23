import fs from 'node:fs';import assert from 'node:assert/strict';import{spawn}from'node:child_process';
const{chromium}=await import('/opt/codex/cua_node/lib/node_modules/playwright/index.mjs');
const server=spawn(process.execPath,['scripts/serve-rust-preview.mjs','.','5198'],{stdio:['ignore','pipe','inherit']});await new Promise(r=>server.stdout.once('data',r));
const browser=await chromium.launch({headless:true,executablePath:'/usr/bin/chromium',args:['--no-sandbox']});
try{const page=await browser.newPage();await page.goto('http://127.0.0.1:5198/web/dist/index.html');await page.waitForFunction(()=>window.__lhMujoco?.state);
 const r=await page.evaluate(async()=>{
  const{DenseActor,physicalObservation,SlowHolding}=await import('/web/mujoco/local-getup/slow.js'),{Physics}=await import('/web/dist/engine.js'),load=(await import('/web/dist/vendor/mujoco.js')).default;
  const actor=new DenseActor(await(await fetch('/rust/crates/sim/data/getup-policy/student.bin')).arrayBuffer()),fixtures=await(await fetch('/rust/crates/sim/data/getup-policy/parity.json')).json();
  let raw=0,action=0;const start=performance.now();for(const f of fixtures.actors.filter(x=>x.actor==='student')){const out=actor.infer(f.input);raw=Math.max(raw,...out.raw.map((v,i)=>Math.abs(v-f.raw[i])));action=Math.max(action,...out.action.map((v,i)=>Math.abs(v-f.action[i])));}
  const inferenceMsPerDecision=(performance.now()-start)/16;
  const p=new Physics(await load(),await(await fetch('/web/dist/humanoid.xml')).text()),f=(await(await fetch('/docs/fixtures/getup-step/source-SLOW-final-frame.json')).json()).frame;
  p.data.qpos.set(f.qpos);p.data.qvel.set(f.qvel);const physical=physicalObservation(p),physicalError=Math.max(...physical.map((v,i)=>Math.abs(v-f.physicalObservation[i]))),standing=(await(await fetch('/rust/crates/model/data/getup-v1.json')).json()).referenceMetadata.standingPose;
  const hold=new SlowHolding(p,actor,standing);hold.command();const bounded=p.data.ctrl.every(x=>Number.isFinite(x)&&Math.abs(x)<=1),ticks=p.steps;p.dispose();
  return {raw,action,physicalError,inferenceMsPerDecision,bounded,ticks,uiTicks:window.__lhMujoco.state.steps};
 });assert(r.raw<2e-4&&r.action<5e-5&&r.physicalError<2e-5&&r.bounded&&r.ticks===0&&r.uiTicks===0);console.log(JSON.stringify(r));fs.writeFileSync('web/proofs/slow-local-browser.json',JSON.stringify(r));
}finally{await browser.close();server.kill();}
