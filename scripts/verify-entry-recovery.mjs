// Algebra/saved-data checks only. No MuJoCo import or dynamic call.
import assert from 'node:assert/strict';
import fs from 'node:fs';import {gunzipSync} from 'node:zlib';
import {envelope,eulerMap,RECOVERY,RecoveryReference,recoveryRows,recoveryAdmission} from './pull-entry-recovery.mjs';
const max=a=>Math.max(...a.map(Math.abs)),sub=(a,b)=>a.map((v,i)=>v-b[i]);
export function algebra(){
 const read=n=>JSON.parse(gunzipSync(fs.readFileSync('web/proofs/pull-dynamic-stance-001-verified/'+n+'.json.gz'))),raw=read('initial-native'),q=read('initial-qp'),old=read('last-comparison'),entries=[];
 for(const [name,v]of [['right_knee',12],['left_knee',18]]){const q0=raw.qpos[v+1],v0=raw.qvel[v],limit=2*Math.PI/180,H=.12,b=2*(q0-limit+H*v0)/(H*(H+.00125)),e={name,q:v+1,v,q0,v0,limit,b};entries.push(e);assert(Math.abs(envelope(e,96).q-limit)<1e-16);let pos=q0,vel=v0;for(let n=1;n<=96;n++){vel-=b*.00125;pos+=vel*.00125;const env=envelope(e,n);assert(max([pos-env.q,vel-env.v])<1e-15);}assert(envelope(e,1).q>q0,'Outward initial velocity cannot be called monotone ROM recovery');}
 const b=entries.map(e=>e.b),peaks=entries.map(e=>Math.max(...Array.from({length:97},(_,n)=>envelope(e,n).q))*180/Math.PI);assert(max(sub(b,[.6620299538253861,.6971730675970672]))<1e-14);assert(max(sub(peaks,[2.095811867310513,2.103205093482908]))<1e-12);
 const mapped=eulerMap(q.T,raw.qacc),mapError=max(sub(mapped,raw.euler));assert(mapError<2e-14);const wrong= q.T.map(col=>col.reduce((s,v,i)=>s+v*raw.qacc[i],0));assert(max(sub(wrong,raw.euler))>1e-3,'Transpose counterexample');
 const affine=q.a0.map((v,k)=>v+q.A.reduce((s,col,i)=>s+col[k]*old.unblendedControls[i],0)),ap=eulerMap(q.T,affine);assert(entries.every(e=>ap[e.v]>-e.b),'Old raw-qacc admitted candidate is not Euler recovery admitted');
 assert(entries.every(e=>raw.euler[e.v]>0));assert.throws(()=>envelope(entries[0],97));assert.throws(()=>envelope(entries[0],NaN));
 // A convex mix with an inadmissible old endpoint is not automatically admissible.
 const u0=1,u1=-1,threshold=-1;assert(u1<=threshold&&.5*u0+.5*u1>threshold);
 return {passed:true,tests:8,b,peakDegrees:peaks,mappingError:mapError,oldCandidateEulerKnees:entries.map(e=>ap[e.v]),savedDataOnly:true,forwards:0,ticks:0};
}
export function referenceChecks(p,nodes){
 const before={q:Array.from(p.data.qpos),v:Array.from(p.data.qvel)},ref=new RecoveryReference(p,nodes);let derivativeError=0,maxServoExcess=0;
 try{const t0=ref.at(p.data.time);assert(t0.servoResidual.length===2);assert(t0.servo.dq.every(v=>v===0));assert(ref.recovery.entries.every(e=>p.data.qvel[e.v]===e.v0));
  for(let k=0;k<=384;k++){const t=1.8*k/384,x=ref.servoJets.time(t,1.8);for(const j of p.joints)maxServoExcess=Math.max(maxServoExcess,p.model.jnt_range[2*j.j]-x.q[j.q],x.q[j.q]-p.model.jnt_range[2*j.j+1]);}
  for(const t of [.131,.431,.811,1.171,1.571]){const h=1e-6,x=ref.servoJets.time(t,1.8),a=ref.servoJets.time(t-h,1.8),b=ref.servoJets.time(t+h,1.8);derivativeError=Math.max(derivativeError,max(sub(b.q,a.q).map((v,i)=>v/(2*h)-x.dq[i])),max(sub(b.dq,a.dq).map((v,i)=>v/(2*h)-x.ddq[i])));}
  assert(maxServoExcess<1e-12&&derivativeError<1e-6);assert.deepEqual(Array.from(p.data.qpos),before.q);assert.deepEqual(Array.from(p.data.qvel),before.v);
  // Recovery progress stays frozen even if wall/simulation time is requested ahead.
  assert.deepEqual(ref.at(p.data.time+.1).q,t0.q);
  const fake={...ref,p:{...p,data:{time:p.data.time+.00125,qpos:Float64Array.from(p.data.qpos),qvel:Float64Array.from(p.data.qvel)}}};
  for(const e of ref.recovery.entries){const env=envelope(e,1);fake.p.data.qpos[e.q]=env.q;fake.p.data.qvel[e.v]=env.v;}assert(RecoveryReference.prototype.checkState.call(fake).passed);
  const e=ref.recovery.entries[0];fake.p.data.qvel[e.v]+=.00001;assert(!RecoveryReference.prototype.checkState.call(fake).passed);fake.p.data.qvel[e.v]-=.00001;fake.p.data.qpos[e.q]+=.00001;assert(!RecoveryReference.prototype.checkState.call(fake).passed);
  fake.phase='transfer';assert(!RecoveryReference.prototype.checkState.call(fake).passed,'Initial exception must not survive recovery');
  return {passed:true,tests:9,servoMaximumExcess:maxServoExcess,servoDerivativeError:derivativeError,recovery:ref.recovery,servoSpeedBound:ref.servoCurve.maxSpeed,servoAccelerationBound:ref.servoCurve.maxAcceleration,physicalDqPreserved:true};
 }finally{ref.dispose();}
}
if(process.argv.includes('--algebra'))console.log(JSON.stringify(algebra()));
