// Explicit, state-specific recovery. References only; native motor control owns forces.
import assert from 'node:assert/strict';
import {CoordinatedReference, dynamicAssembly} from './pull-dynamic-stance.mjs';
import {hermite} from './pull-stance-geometry.mjs';
import {jointCurve} from '../web/mujoco/pull-step.js';
import {SOURCE_TASKS} from './pull-whole-body.mjs';
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), eps=Number.EPSILON;
export const RECOVERY={ticks:96,dt:.00125,seconds:.12,admissionTolerance:1e-8};
// T and Minv are stored as COLUMNS, as in nativeAssembly, not rows.
export const eulerMap=(T,a)=>a.map((_,i)=>T.reduce((s,col,k)=>s+col[i]*a[k],0));
export function envelope(e,n){assert(Number.isInteger(n)&&n>=0&&n<=96);const t=n*RECOVERY.dt;return {q:e.q0+t*e.v0-.5*e.b*RECOVERY.dt**2*n*(n+1),v:e.v0-t*e.b};}
export function recoveryDefinition(p){
 const {model:m,data:d}=p,entries=[];
 for(const j of p.joints){const lo=m.jnt_range[2*j.j],hi=m.jnt_range[2*j.j+1],q=d.qpos[j.q],v=d.qvel[j.v];
  if(q<lo||q>hi){assert(['right_knee','left_knee'].includes(j.name)&&q>hi&&v>0,'Only the two recorded outward upper-knee excesses are authorized');const H=RECOVERY.seconds,b=2*(q-hi+H*v)/(H*(H+RECOVERY.dt)),e={name:j.name,q:j.q,v:j.v,q0:q,v0:v,limit:hi,b};const peak=Array.from({length:97},(_,n)=>({...envelope(e,n),n})).reduce((a,b)=>a.q>b.q?a:b);entries.push({...e,peak});}
 }
 assert.equal(entries.length,2);return {start:d.time,entries,ticks:96,dt:.00125,seconds:.12};
}
export class RecoveryReference extends CoordinatedReference{
 constructor(p,nodes){
  super(p,nodes);this.recovery=recoveryDefinition(p);this.globalStart=this.start;this.phase='recovery';this.events=[{phase:'recovery',t:this.start}];this.recoveryDone=false;
  // The measured geometric anchor stays unchanged. Servo position AND its
  // derivatives follow an independent admissible curve, never a clipped q/dq pair.
  this.servoNodes=nodes.map(q=>q.slice());for(const q of this.servoNodes)for(const j of p.joints)q[j.q]=Math.max(p.model.jnt_range[2*j.j],Math.min(p.model.jnt_range[2*j.j+1],q[j.q]));
  this.servoCurve=jointCurve(this.servoNodes,p.joints,p.model,1.8);this.servoJets=hermite(this.servoNodes);
 }
 at(t){const elapsed=this.phase==='recovery'?0:Math.max(0,Math.min(1.8,t-this.start));const sample=super.at(this.start+elapsed),servo=this.servoJets.time(elapsed,1.8);sample.servo=servo;sample.servoResidual=this.p.joints.map(j=>({name:j.name,desired:sample.q[j.q],commanded:servo.q[j.q],residual:sample.q[j.q]-servo.q[j.q]})).filter(x=>x.residual!==0);return sample;}
 checkState(){
  const {data:d,model:m}=this.p,n=Math.round((d.time-this.recovery.start)/RECOVERY.dt),violations=[];
  for(const j of this.p.joints){const q=d.qpos[j.q],v=d.qvel[j.v],lo=m.jnt_range[2*j.j],hi=m.jnt_range[2*j.j+1],e=this.recovery.entries.find(e=>e.name===j.name),arith=128*eps*Math.max(1,Math.abs(q),Math.abs(v));
   if(this.phase==='recovery'&&e){assert(n>=0&&n<=96,'Recovery deadline exceeded');const env=envelope(e,n);if(q>env.q+arith||v>env.v+arith||q<lo-arith)violations.push({joint:j.name,kind:'fixed recovery envelope',n,q,v,envelope:env});}
   else if(q<lo-arith||q>hi+arith)violations.push({joint:j.name,kind:'new or recurring physical ROM excess',q,v,lo,hi});
  }
  return {passed:!violations.length,n,violations};
 }
 advance(m,controls,dt){
  if(this.phase==='recovery'){
   const n=Math.round((m.t-this.recovery.start)/RECOVERY.dt),inside=this.recovery.entries.every(e=>this.p.data.qpos[e.q]<=e.limit&&this.p.data.qvel[e.v]<=0);
   if(inside){this.recoveryDone=true;this.recoveredAt=m.t;this.start=m.t;this.transferDeadline=m.t+2.3;this.enter('transfer',m.t,controls,{recoveryTicks:n});}
   else if(n>=96)return 'Initial knee recovery not observed by fixed96-tick deadline';
   return null;
  }
  return super.advance(m,controls,dt);
 }
 snapshot(){return {...super.snapshot(),globalStart:this.globalStart,recovery:this.recovery,recoveryDone:this.recoveryDone,recoveredAt:this.recoveredAt,servoNodes:this.servoNodes};}
}
export function recoveryAssembly(p,int,ref){
 const taskRef=ref.phase==='recovery'?{...ref,phase:'transfer',at:ref.at.bind(ref),vertical:ref.vertical.bind(ref)}:ref;
 const q=dynamicAssembly(p,int,taskRef),s=ref.sample.servo,c=SOURCE_TASKS.posture;q.target.phase=ref.phase;
 for(const j of p.joints){const t=q.tasks.find(t=>t.label==='posture-'+j.name);t.desired=s.ddq[j.q]+c.kp*(s.q[j.q]-p.data.qpos[j.q])+c.kd*(s.dq[j.q]-p.data.qvel[j.v]);}
 q.target.geometryPosture=q.target.posture;q.target.servoResidual=ref.sample.servoResidual;q.target.posture=p.joints.map(j=>({name:j.name,q:s.q[j.q],dq:s.dq[j.q],ddq:s.ddq[j.q]}));return q;
}
function bound(p,ref,j,side){
 const e=ref.phase==='recovery'&&ref.recovery.entries.find(e=>e.name===j.name);
 if(e&&side===1)return {bound:-e.b,kind:'fixed initial recovery braking'};
 const H=SOURCE_TASKS.stoppingHorizon,h=p.model.opt.timestep,d=p.data,limit=p.model.jnt_range[2*j.j+(side===1?1:0)];
 return {bound:2*side*(limit-d.qpos[j.q]-H*d.qvel[j.v])/(H*(H+h)),kind:'constant Euler acceleration endpoint predictor'};
}
export function recoveryRows(p,q,ref){
 const a0=eulerMap(q.T,q.a0),A=q.A.map(a=>eulerMap(q.T,a));q.rows=q.rows.filter(r=>!r.label.startsWith('joint-stopping-'));
 for(const j of p.joints)for(const side of [-1,1]){const b=bound(p,ref,j,side);q.rows.push({label:`euler-stopping-${j.name}-${side}`,a:A.map(a=>side*a[j.v]),bound:b.bound-side*a0[j.v],kind:b.kind});}
 q.recovery={phase:ref.phase,definition:ref.recovery,eulerColumnStorage:true};return q;
}
export function recoveryAdmission(p,q,raw,ref,assessed){
 const checks=[],violations=[];for(const j of p.joints){
  const row=q.T.map(col=>col[j.v]),variance=dot(row,q.Minv.reduce((sum,col,k)=>sum.map((v,i)=>v+col[i]*row[k]),Array(27).fill(0)));assert(variance>=-1e-12);
  const error=Math.sqrt(Math.max(0,variance))*assessed.eta+128*eps*Math.max(1,Math.abs(raw.euler[j.v]));
  for(const side of [-1,1]){const b=bound(p,ref,j,side),actual=side*raw.euler[j.v],residual=actual+error-b.bound,c={joint:j.name,side,kind:b.kind,actual,error,bound:b.bound,residual};checks.push(c);if(residual>RECOVERY.admissionTolerance)violations.push(c);}
 }
 return {passed:!violations.length,checks,violations};
}
