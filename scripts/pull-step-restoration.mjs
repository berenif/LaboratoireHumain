// Pure preparation of the explicitly restored 001 entry, not a physical replay.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{gunzipSync}from'node:zlib';
import{Physics}from'../web/dist/engine.js';import{PULL_PROTOCOL as c,smooth}from'../web/dist/pull-step.js';
export const INPUT='web/proofs/pull-step-001',EXPECTED='bdc3c9bd7167a8bc23ea583b55628d0c8d5ee878dd7a6fe3336a03bac41c743c';
export const sha=b=>crypto.createHash('sha256').update(b).digest('hex');
export function nativeState(p,spec){const b=new p.mj.DoubleBuffer(p.mj.mj_stateSize(p.model,spec));p.mj.mj_getState(p.model,p.data,b,spec);const s=Array.from(b.GetView());b.delete();return s;}
export function restorePullEntry(mj,{loadFeedback=false}={}){
 const bytes=fs.readFileSync(INPUT+'/final.json.gz');assert.equal(sha(bytes),EXPECTED);
 const cp=JSON.parse(gunzipSync(bytes)),ledger=JSON.parse(fs.readFileSync(INPUT+'/ledger.json')),xml=fs.readFileSync('web/dist/humanoid.xml','utf8');
 assert.equal(sha(xml),cp.modelSHA256);assert.equal(cp.state.length,233);assert.equal(ledger.confirmedTotal,228054);assert(ledger.completed&&ledger.pendingMaxTicks===0);assert.equal(cp.physics.steps,8195);
 const p=new Physics(mj,xml);mj.mj_setState(p.model,p.data,cp.state,cp.spec);p.forward();
 // Recompute derived fields only; forward can alter warmstart. Restore all 233
 // integration values again, retaining the current derived kinematics.
 mj.mj_setState(p.model,p.data,cp.state,cp.spec);assert.deepEqual(nativeState(p,cp.spec),cp.state);
 Object.assign(p,{mode:cp.physics.mode,fallen:cp.physics.fallen,steps:cp.physics.steps,resets:cp.physics.resets,reactions:structuredClone(cp.physics.reactions),previousCom:cp.previousCom.slice(),balance:structuredClone(cp.physics.balance)});
 p.command=p.joints.map((j,i)=>({name:j.name,requested:ledger.summary.last.command[i][0],applied:ledger.summary.last.command[i][1]}));
 p.enablePullStep({loadFeedback});Object.assign(p.pullStep,structuredClone(cp.plan));p.pullStep.loadFeedback=loadFeedback;
 // A failed optional pure proposal is replaced transactionally. Physical state,
 // original phase clock, entry shape, destination and deadline remain intact.
 const path=p.pullStep.ik.path(cp.plan.entryQ,cp.plan.transferGoal,c.transferSeconds,cp.plan.entryQ);
 Object.assign(p.pullStep,{phase:'transfer',reason:null,events:[{phase:'transfer',t:cp.plan.entry}],path,pathStart:cp.plan.entry,phaseStart:cp.plan.entry,progress:0,reference:null});
 const local=[.09,-.09,.09],startStep=p.steps,from=ledger.summary.last.force.slice();assert(Math.hypot(...from)<=180);
 // Missing grab.previous is NOT reconstructed. Start a disclosed new history:
 // first velocity from native J*qvel, then actual successive point samples.
 p.grab={body:c.body,local,target:cp.physics.grab.target.slice(),previous:null,bridge:{startStep,ticks:20,from}};
 const normal=p.grabForce.bind(p);
 p.grabForce=()=>{
  const g=p.grab;if(!g)return [0,0,0];let current;
  if(g.previous===null){
   const jp=new mj.DoubleBuffer(3*p.model.nv),jr=new mj.DoubleBuffer(3*p.model.nv);mj.mj_jacBody(p.model,p.data,jp,jr,g.body);
   const linear=[0,1,2].map(k=>Array.from(jp.GetView().slice(k*p.model.nv,(k+1)*p.model.nv)).reduce((s,x,i)=>s+x*p.data.qvel[i],0)),angular=[0,1,2].map(k=>Array.from(jr.GetView().slice(k*p.model.nv,(k+1)*p.model.nv)).reduce((s,x,i)=>s+x*p.data.qvel[i],0));jp.delete();jr.delete();
   const point=p.worldPoint(g.body,g.local),r=point.map((x,i)=>x-p.data.xpos[3*g.body+i]),cross=[angular[1]*r[2]-angular[2]*r[1],angular[2]*r[0]-angular[0]*r[2],angular[0]*r[1]-angular[1]*r[0]];
   current=point.map((x,i)=>120*(g.target[i]-x)-12*(linear[i]+cross[i]));const n=Math.hypot(...current);if(n>180)current=current.map(x=>x*180/n);
  }else current=normal();
  const s=smooth((p.steps-startStep)/20);return current.map((x,i)=>(1-s)*from[i]+s*x);
 };
 assert.deepEqual(nativeState(p,cp.spec),cp.state);
 const grabStart=[cp.physics.grab.target[0]-c.targetOffsetX*smooth((p.data.time-10)/c.rampSeconds),...cp.physics.grab.target.slice(1)];
 return {p,cp,ledger,path,grabStart,forceTransition:{ticks:20,seconds:.025,from,body:c.body,local,method:'convex quintic from recorded last force to current unchanged grab law; new point history, first velocity native J*qvel',historicalGrabMemoryRestored:false}};
}
