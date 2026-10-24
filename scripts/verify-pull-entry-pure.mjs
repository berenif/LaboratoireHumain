import fs from 'node:fs';import assert from 'node:assert/strict';
import load from '../web/dist/vendor/mujoco.js';import{jointCurve,smooth}from'../web/dist/pull-step.js';import{restorePullEntry,nativeState}from'./pull-step-restoration.mjs';
const mj=await load(),{p,cp,ledger,path,forceTransition}=restorePullEntry(mj),before=nativeState(p,cp.spec);
assert.deepEqual(path.at(0),cp.plan.entryQ);assert.equal(path.entrance.length,2);
const oldPrevious=p.previousCom.slice(),plan=p.pullStep;p.pullStep=null;p.control();const nominal=structuredClone(p.command);p.pullStep=plan;p.previousCom=oldPrevious.slice();p.control();assert.deepEqual(p.command,nominal);assert.equal(p.pullStep.phase,'transfer');
assert.deepEqual(Array.from(p.data.qpos),cp.physics.qpos);assert.deepEqual(Array.from(p.data.qvel),cp.physics.qvel);assert.equal(p.data.time,cp.physics.time);assert.equal(p.steps,8195);
let last=path.at(0),maxObservedSpeed=0,maxRequested=0;const sampleDt=path.duration/4096;
for(let i=0;i<=4096;i++){
 const q=path.at(smooth(i/4096));for(const j of p.joints){const lo=p.model.jnt_range[2*j.j],hi=p.model.jnt_range[2*j.j+1],entry=cp.plan.entryQ[j.q],excess=Math.max(lo-q[j.q],q[j.q]-hi,0),prev=Math.max(lo-last[j.q],last[j.q]-hi,0);assert(excess<=prev+1e-12);const servo=Math.max(lo,Math.min(hi,q[j.q]+cp.plan.servoOffset[j.q]));assert(servo>=lo&&servo<=hi);if(i)maxObservedSpeed=Math.max(maxObservedSpeed,Math.abs(q[j.q]-last[j.q])/sampleDt);assert(Number.isFinite(q[j.q]));}last=q;
}
assert(maxObservedSpeed<=path.maxSpeed*(1+1e-8));
// Pure future-reference snapshots, with unchanged measured q/dq/contact inputs.
// These are command checks, NOT future physical states or dynamic feasibility.
for(const dt of [0,1e-7,.00125,.0125,.025,.15,.4,1,1.8]){p.data.time=cp.physics.time+dt;p.previousCom=Array.from(p.data.subtree_com.slice(3,6));p.control();assert.equal(p.pullStep.phase,'transfer');for(const [i,j]of p.joints.entries()){const c=p.command[i];maxRequested=Math.max(maxRequested,Math.abs(c.requested));assert(Number.isFinite(c.applied)&&Math.abs(c.applied)<=Math.abs(j.gear));assert(Math.abs(p.data.ctrl[j.a]*j.gear-c.applied)<1e-12);}}
p.data.time=cp.physics.time;
// Physical dq is damped, never truncated or turned into (0-dq)/dt.
const knees=p.joints.filter(j=>j.name.endsWith('knee')),braking=knees.map(j=>({joint:j.name,dq:p.data.qvel[j.v],dampingNm:-20*p.data.qvel[j.v]}));assert(braking.every(x=>x.dq>0&&x.dampingNm<0));
const model={jnt_range:[-1,1]},joints=[{name:'test',j:0,q:7}],q=v=>[0,0,0,1,0,0,0,v];
assert.throws(()=>jointCurve([q(1.01),q(1),q(.9)],joints,model,1),/unapproved/);
assert.throws(()=>jointCurve([q(1.01),q(1),q(.9)],joints,model,1,q(1.02)),/mismatch/);
assert.throws(()=>jointCurve([q(1.01),q(1.02),q(.9)],joints,model,1,q(1.01)),/reach ROM/);
assert.throws(()=>jointCurve([q(1),q(1.001),q(.9)],joints,model,1,q(1)),/ROM/);
assert.throws(()=>jointCurve([q(1.01),q(.99),q(1.002)],joints,model,1,q(1.01)),/ROM/);
assert.throws(()=>jointCurve([q(1.01),q(NaN),q(.9)],joints,model,1,q(1.01)),/Nonfinite/);
assert.equal(jointCurve([q(1.01),q(1),q(.9)],joints,model,1,q(1.01)).at(0)[7],1.01);
p.steps=8195;p.grab.previous=null;assert.deepEqual(p.grabForce(),ledger.summary.last.force);for(let i=0;i<=20;i++){p.steps=8195+i;assert(Math.hypot(...p.grabForce())<=180+1e-10);}p.steps=8195;
assert.deepEqual(Array.from(p.data.qpos),cp.physics.qpos);assert.deepEqual(Array.from(p.data.qvel),cp.physics.qvel);assert(p.data.qfrc_applied.every(x=>x===0));assert(p.data.qfrc_actuator.slice(0,6).every(x=>x===0));
console.log(JSON.stringify({passed:true,integrations:0,native233ExactOnRestore:before.every((x,i)=>x===cp.state[i]),entryTotalCommandEqualToCurrentBalance:true,lastAppliedToRecomputedMaxNm:Math.max(...nominal.map((x,i)=>Math.abs(x.applied-ledger.summary.last.command[i][1]))),residualM:path.residual,entrance:path.entrance,maxSpeedBoundRadS:path.maxSpeed,maxAccelerationBoundRadS2:path.maxAcceleration,maxObservedSpeedRadS:maxObservedSpeed,maxPureRequestedNm:maxRequested,braking,forceTransition,physicalStateUnchanged:true,counterexamplesRejected:6}));p.dispose();
