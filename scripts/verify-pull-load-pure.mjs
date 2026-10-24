// No mj_step, integration, forecast or file output. Fixed-state forward dynamics
// compare the same complete command with/without the experimental load feedback.
import fs from 'node:fs';import assert from 'node:assert/strict';import{gunzipSync}from'node:zlib';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{loadExchange,jointCurve}from'../web/dist/pull-step.js';import{nativeState}from'./pull-step-restoration.mjs';
const mj=await load(),xml=fs.readFileSync('web/dist/humanoid.xml','utf8'),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),sub=(a,b)=>a.map((x,i)=>x-b[i]);
const cases=[];
for(const name of ['001','002']){
 const cp=JSON.parse(gunzipSync(fs.readFileSync(`web/proofs/pull-step-${name}/final.json.gz`))),ledger=JSON.parse(fs.readFileSync(`web/proofs/pull-step-${name}/ledger.json`)),p=new Physics(mj,xml);
 mj.mj_setState(p.model,p.data,cp.state,cp.spec);p.forward();mj.mj_setState(p.model,p.data,cp.state,cp.spec);assert.deepEqual(nativeState(p,cp.spec),cp.state);
 const pose=Array.from(p.data.qpos),velocity=Array.from(p.data.qvel),time=p.data.time,force=ledger.summary.last.force.slice(),local=[.09,-.09,.09];
 p.mode='balance';p.reactions=structuredClone(cp.physics.reactions);p.enablePullStep();Object.assign(p.pullStep,structuredClone(cp.plan));p.pullStep.loadFeedback=false;
 if(name==='001'){Object.assign(p.pullStep,{phase:'transfer',reason:null,path:p.pullStep.ik.path(cp.plan.entryQ,cp.plan.transferGoal,1.8,cp.plan.entryQ),pathStart:cp.plan.entry,phaseStart:cp.plan.entry});}
 else p.pullStep.path={...cp.path,...jointCurve(cp.path.nodes,p.joints,p.model,cp.path.duration,cp.plan.entryQ)};
 p.grab={body:15,local,target:cp.physics.grab.target.slice(),previous:p.worldPoint(15,local)};p.grabForce=()=>force;
 // Diagnostic baseline uses the recorded current COM velocity in both branches;
 // it is explicitly not a reproduction of old pre-step controller memory.
 const com=Array.from(p.data.subtree_com.slice(3,6)),vcom=ledger.summary.last.velocity;
 p.previousCom=com.map((x,i)=>x-vcom[i]*p.dt);p.control();
 if(name==='002'){
  assert.equal(p.pullStep.phase,'failed');assert.equal(p.pullStep.reason,'torso tilt exceeded 20 degrees');assert.equal(p.steps,0);assert.equal(p.data.time,time);assert.deepEqual(Array.from(p.data.qpos),pose);assert.deepEqual(Array.from(p.data.qvel),velocity);
  cases.push({state:name,time,physicalSteps:0,terminalGuard:'torso tilt exceeded20degrees',candidateNotEvaluated:true,reason:'A controller preview beyond this physical stop would fall back to nominal Balance; it is not an admissible continuation command.'});p.dispose();continue;
 }
 const baseline=p.command.map(x=>x.requested),servo=p.pullStep.reference.q.slice(),point=p.worldPoint(15,local),mass=Array.from(p.model.body_mass).reduce((a,b)=>a+b,0);
 p.data.xfrc_applied.fill(0);p.apply(15,point,force);p.forward();p.readReactions();
 const geometry={...p.reactions};
 const existingCommand=p.command.map(x=>x.applied);p.pullStep.loadFeedback=true;p.previousCom=com.map((x,i)=>x-vcom[i]*p.dt);p.control();const entryDifference=Math.max(...p.command.map((x,i)=>Math.abs(x.applied-existingCommand[i])));assert(entryDifference<1e-12);p.pullStep.loadFeedback=false;
 const goal=[com[0],p.data.xpos[3*7+1],com[2]],r=loadExchange(p,servo,goal,com,vcom);
 assert(r.active);for(const[i,j]of p.joints.entries())assert(Math.abs(r.wanted[i]*(/hip|knee|ankle/.test(j.name)?300:200)-r.requestedTau[i])<1e-12);
 assert(Math.hypot(...r.deltaForce)<1e-10);assert(Math.hypot(...r.deltaMoment)>1); // intentionally nonzero, never hidden as "balanced"
 const jp=new mj.DoubleBuffer(3*p.model.nv),jr=new mj.DoubleBuffer(3*p.model.nv),M=new mj.DoubleBuffer(p.model.nv*p.model.nv),dq=Array.from({length:p.model.nv},(_,i)=>Math.sin(i+1)*.01),general=Array(p.model.nv).fill(0);r.unactuatedGeneralizedNotApplied.forEach((x,i)=>general[i]=x);p.joints.forEach((j,i)=>general[j.v]=r.requestedTau[i]);let work=0;
 for(const x of r.changes){mj.mj_jac(p.model,p.data,jp,jr,x.position,x.body);const view=jp.GetView(),pv=[0,1,2].map(k=>dq.reduce((s,v,i)=>s+view[k*p.model.nv+i]*v,0));work-=(x.targetN-x.measuredN)*dot(x.normal,pv);}
 assert(Math.abs(dot(general,dq)-work)<1e-12);
 function measure(){
  let totalF=force.slice(),H=cross(sub(point,com),force),frictionRatio=0;
  for(const x of p.reactions.points){const h=cross(sub(x.position,com),x.forceWorld);totalF=totalF.map((v,i)=>v+x.forceWorld[i]);H=H.map((v,i)=>v+h[i]);const normal=dot(x.forceWorld,x.normal),tangent=x.forceWorld.map((v,i)=>v-normal*x.normal[i]);frictionRatio=Math.max(frictionRatio,Math.hypot(...tangent)/(x.friction[0]*x.load));}
  mj.mj_fullM(p.model,p.data,M);const matrix=M.GetView();mj.mj_jac(p.model,p.data,jp,jr,point,15);const j=jp.GetView(),res=[];
  for(let i=0;i<6;i++){let x=p.data.qfrc_bias[i]-p.data.qfrc_passive[i]-p.data.qfrc_actuator[i]-p.data.qfrc_constraint[i]-p.data.qfrc_applied[i];for(let k=0;k<p.model.nv;k++)x+=matrix[i*p.model.nv+k]*p.data.qacc[k];for(let k=0;k<3;k++)x-=j[k*p.model.nv+i]*force[k];res.push(x);}
  assert(Math.max(...res.map(Math.abs))<1e-7);assert(frictionRatio<=1+1e-6);assert(p.data.qfrc_applied.every(x=>x===0));assert(p.data.qfrc_actuator.slice(0,6).every(x=>x===0));
  for(const j of p.joints)assert(Number.isFinite(p.data.qfrc_actuator[j.v])&&Math.abs(p.data.qfrc_actuator[j.v])<=Math.abs(j.gear)+1e-10);
  assert.deepEqual(Array.from(p.data.qpos),pose);assert.deepEqual(Array.from(p.data.qvel),velocity);assert.equal(p.data.time,time);assert.equal(p.steps,0);
  return {loads:p.reactions.normalLoads,comAcceleration:totalF.map((x,i)=>x/mass-(i===2?9.81:0)),angularMomentumRate:H,rootAngularAcceleration:Array.from(p.data.qacc.slice(3,6)),baseEquationResidual:res,frictionRatio};
 }
 const before=measure(),variants=[];
 // Two prescribed checks, not a gain sweep: the first25ms filter update and
 // its unfiltered steady request at the same state. Neither advances a state.
 for(const [label,prior]of [['firstFilteredUpdate',null],['fullRequestedCorrection',r.wanted]]){
  p.reactions=geometry;const correction=loadExchange(p,servo,goal,com,vcom,prior);for(const[i,j]of p.joints.entries()){assert(correction.q[j.q]>=p.model.jnt_range[2*j.j]-1e-12&&correction.q[j.q]<=p.model.jnt_range[2*j.j+1]+1e-12);p.data.ctrl[j.a]=Math.max(-1,Math.min(1,(baseline[i]+correction.realizedReferenceTau[i])/j.gear));}
  p.forward();p.readReactions();variants.push({label,...measure()});
 }
 p.reactions={normalLoads:{7:400},points:geometry.points.filter(x=>x.body===7)};assert.equal(loadExchange(p,servo,goal,com,vcom).active,false);
 mj.mj_setState(p.model,p.data,cp.state,cp.spec);assert.deepEqual(nativeState(p,cp.spec),cp.state);
 cases.push({state:name,time,physicalSteps:0,baseline:'recomputed using recorded COM velocity, frozen measured grab force',target:{ayWanted:r.ayWanted,ayAfterCopProjection:r.ayCommanded,right:r.rightTarget,left:r.leftTarget,copWanted:r.copWanted,copCommand:r.copCommand,deltaMoment:r.deltaMoment,unactuatedGeneralizedNotApplied:r.unactuatedGeneralizedNotApplied},before,variants,entryCommandDifferenceNm:entryDifference,virtualWorkError:dot(general,dq)-work});jp.delete();jr.delete();M.delete();p.dispose();
}
const entry=cases[0],full=entry.variants[1],filtered=entry.variants[0];assert(entry.target.ayAfterCopProjection<0);assert(full.comAcceleration[1]>entry.before.comAcceleration[1]);assert(full.angularMomentumRate[0]>entry.before.angularMomentumRate[0]+10);
console.log(JSON.stringify({checksPassed:true,readiness:false,integrations:0,forecasts:0,reason:'Normal-load correction changes reactions but fails the lateral acceleration / roll moment task; no physical trial authorized by this pure gate',cases}));
