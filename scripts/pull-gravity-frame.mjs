// Experimental equilibrium task coordinates only. No dynamics calls or force writes.
import assert from 'node:assert/strict';
import {CoordinatedReference,worldFeatures} from './pull-dynamic-stance.mjs';
import {RecoveryReference,recoveryAssembly} from './pull-entry-recovery.mjs';
import {SubstepReference} from './pull-substep-recovery.mjs';
import {hermite,flat,inv,rot,mm,sub} from './pull-stance-geometry.mjs';
import {jointCurve} from '../web/mujoco/pull-step.js';
import {SOURCE_TASKS,orientationError} from './pull-whole-body.mjs';
export function gravityPosition(stance,feature,Rg){
 const columns=feature.jp[0].map((_,i)=>inv(Rg,sub(feature.jp.map(r=>r[i]),stance.jp.map(r=>r[i]))));
 return {position:inv(Rg,sub(feature.pos,stance.pos)),velocity:inv(Rg,sub(feature.v,stance.v)),bias:inv(Rg,sub(feature.bp,stance.bp)),jac:[0,1,2].map(k=>columns.map(c=>c[k]))};
}
// The geometric curve was aligned to this ORIGINAL measured stance transform.
// Ranchor and Rg are constant. No current foot roll or virtual base motion is credited.
export function gravityTargets(sample,Ranchor,Rg){
 const map=v=>inv(Rg,rot(Ranchor,v));
 return {com:Object.fromEntries(['position','velocity','acceleration'].map(k=>[k,map(sample.com[k])])),torso:{rotation:mm(Ranchor,sample.torso.rotation),velocity:rot(Ranchor,sample.torso.velocity),acceleration:rot(Ranchor,sample.torso.acceleration)}};
}
export class GravityReference extends CoordinatedReference{
 constructor(p,nodes,saved){
  super(p,nodes);assert(saved.phase==='transfer'&&saved.recoveryDone&&!saved.zPolynomial,'Exact recovered transfer checkpoint required');
  for(const k of ['feet','com','goal','torso','joint'])this[k]=structuredClone(saved.bindings[k]);
  for(const k of ['phase','start','phaseStart','transferDeadline','unloaded','clearDwell','events','entryControls','globalStart','recovery','recoveryDone','recoveredAt','closed'])this[k]=structuredClone(saved[k]);
  assert(this.recovery.entries.every(e=>this.closed[e.name]),'Closed exceptions cannot reopen');
  this.servoNodes=structuredClone(saved.servoNodes);this.servoCurve=jointCurve(this.servoNodes,p.joints,p.model,1.8);this.servoJets=hermite(this.servoNodes);
  this.Ranchor=this.feet[0].rotation.slice();this.Rg=flat(this.Ranchor);
 }
 at(t){return RecoveryReference.prototype.at.call(this,t);}
 age(){return SubstepReference.prototype.age.call(this);}
 open(j){return SubstepReference.prototype.open.call(this,j);}
 checkState(){return SubstepReference.prototype.checkState.call(this);}
 advance(m,u,dt){
  // Readiness uses the same gravity coordinates as equilibrium tasks.
  const original=this.sample,target=gravityTargets(original,this.Ranchor,this.Rg);
  this.sample={...original,com:target.com};
  try{return super.advance({...m,relativeCOM:m.gravityCOM,relativeVelocity:m.gravityVelocity},u,dt);}finally{this.sample=original;}
 }
 snapshot(){return {...SubstepReference.prototype.snapshot.call(this),equilibriumFrame:{kind:'fixed original heading, world gravity vertical',Rg:this.Rg,Ranchor:this.Ranchor,currentFootRollExcluded:true}};}
}
export function gravityAssembly(p,interaction,ref){
 const q=recoveryAssembly(p,interaction,ref),f=worldFeatures(p),now=gravityPosition(f.stance,f.com,ref.Rg),target=gravityTargets(ref.sample,ref.Ranchor,ref.Rg),c=SOURCE_TASKS.com,torso=SOURCE_TASKS.torso;
 q.tasks=q.tasks.filter(t=>!t.label.startsWith('relative-com-')&&!t.label.startsWith('relative-torso-'));
 for(let k=0;k<3;k++)q.tasks.push({label:'gravity-com-'+k,jac:now.jac[k],bias:now.bias[k],desired:target.com.acceleration[k]+c.kp*(target.com.position[k]-now.position[k])+c.kd*(target.com.velocity[k]-now.velocity[k]),weight:c.weight,scale:9.81});
 const error=orientationError(target.torso.rotation,f.torso.R);
 for(let k=0;k<3;k++)q.tasks.push({label:'gravity-torso-'+k,jac:f.torso.jr[k],bias:f.torso.br[k],desired:target.torso.acceleration[k]+torso.kp*error[k]+torso.kd*(target.torso.velocity[k]-f.torso.w[k]),weight:torso.weight,scale:q.scales.angularAcceleration});
 q.gravity={Rg:ref.Rg,Ranchor:ref.Ranchor,target,current:now,worldCOM:f.com,worldTorso:f.torso};
 return q;
}
export function worldLateral(q,raw,assessment){
 const row=q.gravity.worldCOM.jp[1],bias=q.gravity.worldCOM.bp[1],dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),variance=q.Minv.reduce((s,col,i)=>s+row[i]*dot(col,row),0);
 const native=dot(row,raw.qacc)+bias,euler=dot(row,raw.euler)+bias,error=Math.sqrt(Math.max(0,variance))*assessment.eta+128*Number.EPSILON*Math.max(1,Math.abs(native));
 return {native,euler,error,passed:native+error<0};
}
