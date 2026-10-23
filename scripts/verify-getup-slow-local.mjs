import fs from 'node:fs';import assert from 'node:assert/strict';import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{DenseActor,physicalObservation,controls,observation,target,SlowHolding}from'../web/mujoco/local-getup/slow.js';
const read=f=>JSON.parse(fs.readFileSync(f)),bin=f=>{const b=fs.readFileSync(f);return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
const fixture=read('rust/crates/sim/data/getup-policy/parity.json'),actors=Object.fromEntries(['teacher','student'].map(n=>[n,new DenseActor(bin(`rust/crates/sim/data/getup-policy/${n}.bin`))]));
const r={integrations:0,raw:0,action:0,pd:0,physical:0};
for(const f of fixture.actors){const out=actors[f.actor].infer(f.input);r.raw=Math.max(r.raw,...out.raw.map((v,i)=>Math.abs(v-f.raw[i])));r.action=Math.max(r.action,...out.action.map((v,i)=>Math.abs(v-f.action[i])));}
for(const f of fixture.pd){const u=controls(f.reference,f.residual,f.q,f.dq,f.holding);r.pd=Math.max(r.pd,...u.map((v,i)=>Math.abs(v-f.controls[i])));}
assert(r.raw<=2e-4&&r.action<=5e-5&&r.pd<=2e-7);
const mj=await load(),p=new Physics(mj,fs.readFileSync('web/dist/humanoid.xml','utf8')),saved=read('docs/fixtures/getup-step/source-SLOW-final-frame.json').frame;
// Static fixture clone only; never advanced or used as a new physical trajectory.
p.data.qpos.set(saved.qpos);p.data.qvel.set(saved.qvel);const q=Array.from(p.data.qpos),dq=Array.from(p.data.qvel),physical=physicalObservation(p);
r.physical=Math.max(...physical.map((v,i)=>Math.abs(v-saved.physicalObservation[i])));assert(r.physical<2e-5,JSON.stringify(r));
assert.deepEqual(Array.from(p.data.qpos),q);assert.deepEqual(Array.from(p.data.qvel),dq);
const standing=read('rust/crates/model/data/getup-v1.json').referenceMetadata.standingPose;
const hold=new SlowHolding(p,actors.student,standing);for(let i=0;i<41;i++)hold.command();assert.equal(hold.decisions,3);assert.equal(hold.nativeTicks,41);assert.equal(p.steps,0);assert(p.data.ctrl.every(v=>Math.abs(v)<=1));
assert.equal(observation(physical,target(standing),target(standing)).length,118);
r.fixtures={actors:fixture.actors.length,pd:fixture.pd.length,physical:1};r.commandsWithoutIntegration=41;console.log(JSON.stringify(r));fs.writeFileSync('web/proofs/slow-local-parity.json',JSON.stringify(r));p.dispose();
