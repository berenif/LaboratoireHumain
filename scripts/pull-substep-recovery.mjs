// Prospective bounded recovery, distinct from the preserved failed parabola.
import assert from 'node:assert/strict';
import {CoordinatedReference} from './pull-dynamic-stance.mjs';
import {RecoveryReference,recoveryAssembly,eulerMap,envelope,RECOVERY} from './pull-entry-recovery.mjs';
import {jointCurve} from '../web/mujoco/pull-step.js';
import {hermite} from './pull-stance-geometry.mjs';
import {SOURCE_TASKS} from './pull-whole-body.mjs';
export {recoveryAssembly,eulerMap};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),eps=Number.EPSILON,h=.00125;
export function boundedBrake(q,v,limit,peak,N){
 assert([q,v,limit,peak].every(Number.isFinite)&&Number.isInteger(N)&&N>0&&N<=96&&peak>=limit);
 const H=N*h,deadline=2*(q-limit+H*v)/(h*h*N*(N+1)),velocity=v/H;
 let peakBrake=-Infinity,critical=0;for(let k=1;k<=N;k++){const b=2*(q-peak+k*h*v)/(h*h*k*(k+1));if(b>peakBrake){peakBrake=b;critical=k;}}
 return {b:Math.max(deadline,velocity,peakBrake),deadline,velocity,peak:peakBrake,critical,N};
}
export class SubstepReference extends CoordinatedReference{
 constructor(p,nodes,saved,bindings){
  super(p,nodes);assert(saved.phase==='recovery'&&!saved.zPolynomial,'This continuation is only the recorded pre-transfer state');
  for(const k of ['feet','com','goal','torso','joint'])this[k]=structuredClone(bindings[k]);
  for(const k of ['phase','start','phaseStart','transferDeadline','unloaded','clearDwell','events','entryControls','globalStart','recovery','recoveryDone'])this[k]=structuredClone(saved[k]);
  assert(Math.round((p.data.time-this.recovery.start)/h)===2);this.closed={};
  this.servoNodes=structuredClone(saved.servoNodes);this.servoCurve=jointCurve(this.servoNodes,p.joints,p.model,1.8);this.servoJets=hermite(this.servoNodes);
 }
 at(t){return RecoveryReference.prototype.at.call(this,t);}
 age(){return Math.round((this.p.data.time-this.recovery.start)/h);}
 open(j){return this.phase==='recovery'&&!this.closed[j.name]&&this.recovery.entries.find(e=>e.name===j.name);}
 checkState(){
  const d=this.p.data,m=this.p.model,n=this.age(),violations=[],knees=[];
  if(this.phase==='recovery'&&n>96)violations.push({kind:'original recovery deadline',n});
  for(const j of this.p.joints){const q=d.qpos[j.q],v=d.qvel[j.v],lo=m.jnt_range[2*j.j],hi=m.jnt_range[2*j.j+1],e=this.open(j),arith=128*eps*Math.max(1,Math.abs(q),Math.abs(v));
   if(q<lo-arith||q>(e?e.peak.q:hi)+arith)violations.push({joint:j.name,kind:e?'fixed inherited peak':'new/recurring ROM excess',q,v,lo,hi,peak:e?e.peak.q:undefined});
   const inherited=this.recovery.entries.find(e=>e.name===j.name);if(inherited){const old=envelope(inherited,Math.min(96,n));knees.push({name:j.name,q,v,closed:!!this.closed[j.name],historicalParabolaResidual:{q:q-old.q,v:v-old.v}});}
  }return {passed:!violations.length,n,violations,knees};
 }
 advance(m,controls,dt){
  if(this.phase==='recovery'){
   for(const e of this.recovery.entries)if(!this.closed[e.name]&&this.p.data.qpos[e.q]<=e.limit&&this.p.data.qvel[e.v]<=0){this.closed[e.name]={t:m.t,n:this.age()};this.events.push({phase:'joint-exception-closed',joint:e.name,t:m.t});}
   if(this.recovery.entries.every(e=>this.closed[e.name])){this.recoveryDone=true;this.recoveredAt=m.t;this.start=m.t;this.transferDeadline=m.t+2.3;this.enter('transfer',m.t,controls,{recoveryTicks:this.age()});}
   else if(this.age()>=96)return 'Bounded recovery not completed at original96-tick deadline';return null;
  }return super.advance(m,controls,dt);
 }
 snapshot(){return {...super.snapshot(),bindings:Object.fromEntries(['feet','com','goal','torso','joint'].map(k=>[k,this[k]])),globalStart:this.globalStart,recovery:this.recovery,recoveryDone:this.recoveryDone,recoveredAt:this.recoveredAt,servoNodes:this.servoNodes,closed:this.closed,contract:'fixed original peaks/deadline; historical parabola diagnostic only'};}
}
export function accelerationChecks(p,ref){
 const checks=[],d=p.data,m=p.model;
 for(const j of p.joints){const q=d.qpos[j.q],v=d.qvel[j.v],lo=m.jnt_range[2*j.j],hi=m.jnt_range[2*j.j+1],e=ref.open(j),N=96-ref.age();
  for(const side of [-1,1]){
   let bound,kind,detail;if(e&&side===1){detail=boundedBrake(q,v,e.limit,e.peak.q,N);bound=-detail.b;kind='bounded recovery peak/deadline/velocity';}
   else{const H=SOURCE_TASKS.stoppingHorizon,limit=side===1?hi:lo;bound=2*side*(limit-q-H*v)/(H*(H+h));kind='Euler endpoint predictor';}
   checks.push({joint:j.name,v:j.v,q:j.q,side,bound,kind,detail});
   checks.push({joint:j.name,v:j.v,q:j.q,side,bound:side*((side===1?(e?e.peak.q:hi):lo)-q-h*v)/(h*h),kind:'next integrated joint position'});
  }
 }return checks;
}
export function substepRows(p,q,ref){
 const a0=eulerMap(q.T,q.a0),A=q.A.map(a=>eulerMap(q.T,a));q.rows=q.rows.filter(r=>!r.label.startsWith('joint-stopping-'));
 const checks=accelerationChecks(p,ref);checks.forEach((r,i)=>q.rows.push({label:`substep-${i}-${r.joint}-${r.side}`,a:A.map(a=>r.side*a[r.v]),bound:r.bound-r.side*a0[r.v],kind:r.kind}));q.substepChecks=checks;return q;
}
export function accelerationErrors(q,assessed,raw){return raw.euler.map((a,i)=>{
 const row=q.T.map(col=>col[i]),MinvRow=q.Minv.reduce((s,col,k)=>s.map((v,j)=>v+col[j]*row[k]),Array(27).fill(0)),variance=dot(row,MinvRow);assert(variance>=-1e-12);
 return Math.sqrt(Math.max(0,variance))*assessed.eta+128*eps*Math.max(1,Math.abs(a));
});}
export function substepAdmission(p,q,raw,ref,assessed){
 const errors=accelerationErrors(q,assessed,raw),checks=accelerationChecks(p,ref).map(r=>({...r,actual:r.side*raw.euler[r.v],error:errors[r.v],residual:r.side*raw.euler[r.v]+errors[r.v]-r.bound}));
 return {passed:checks.every(c=>c.residual<=RECOVERY.admissionTolerance),checks,violations:checks.filter(c=>c.residual>RECOVERY.admissionTolerance),errors};
}
export function prediction(raw,errors,joints){return {q:raw.qpos.slice(),v:raw.qvel.map((v,i)=>v+h*raw.euler[i]),errors,hinges:joints.map(j=>({name:j.name,q:j.q,v:j.v,next:raw.qpos[j.q]+h*(raw.qvel[j.v]+h*raw.euler[j.v])}))};}
export function checkPrediction(pred,d,stepErrors){
 const velocity=[],position=[];for(let i=0;i<27;i++){const error=Math.abs(d.qvel[i]-pred.v[i]),allowance=h*(pred.errors[i]+stepErrors[i])+128*eps*Math.max(1,Math.abs(d.qvel[i]),Math.abs(pred.v[i]));velocity.push({i,error,allowance});}
 for(const j of pred.hinges){const error=Math.abs(d.qpos[j.q]-j.next),allowance=h*h*(pred.errors[j.v]+stepErrors[j.v])+128*eps*Math.max(1,Math.abs(d.qpos[j.q]),Math.abs(j.next));position.push({joint:j.name,error,allowance});}
 return {passed:velocity.every(x=>x.error<=x.allowance)&&position.every(x=>x.error<=x.allowance),maxVelocityError:Math.max(...velocity.map(x=>x.error)),maxPositionError:Math.max(...position.map(x=>x.error)),violations:[...velocity,...position].filter(x=>x.error>x.allowance)};
}
