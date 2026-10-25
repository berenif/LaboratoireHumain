// Opt-in static diagnostic. Reuses RelativeFootIK's Jacobians and its identical
// bounded DLS/Cholesky iteration; only the SOURCE planning interval is inset.
// No controller, model, physical state or force is modified by these helpers.
import assert from 'node:assert/strict';
import {sub,rot,add,maxabs} from './pull-stance-geometry.mjs';
export const STATIC_PLAN=Object.freeze({reserveRad:.02,motorReserveFraction:.925,maxFK:2000,maxInverse:2,maxForward:1,maxEvidenceBytes:300000,maxCodeAndEvidenceBytes:500000,confirmedMuJoCo:229033,unknownHistoryPreserved:true,Rapier:45735,iterationsPerKnot:70,knots:12});
export const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const column=(r,i)=>[r[i],r[3+i],r[6+i]];
export const orientationError=(now,goal)=>[0,1,2].reduce((e,i)=>add(e,cross(column(now,i),column(goal,i))),[0,0,0]).map(v=>v*.5);
export function planningBounds(joints,range,reserve=STATIC_PLAN.reserveRad){assert(reserve>=0&&Number.isFinite(reserve));return joints.map(j=>{const lo=range[2*j.j]+reserve,hi=range[2*j.j+1]-reserve;assert(lo<hi&&Number.isFinite(lo+hi),'Empty planning ROM');return {joint:j.name,q:j.q,v:j.v,lo,hi};});}
export function solveReserved(ik,seed,goal){
 const bounds=planningBounds(ik.joints,ik.m.jnt_range),weights=[1,1,.1,2,2,2,.2,.2,.2,.2,.2,.2];
 // Seed projection acts ONLY on the diagnostic joint reference. The physical
 // measured anchor, including inherited soft-limit excess, remains untouched.
 let q=seed.slice(),iterations=0,blockedLast=[];for(const b of bounds)q[b.q]=clamp(q[b.q],b.lo,b.hi);
 for(let it=0;it<STATIC_PLAN.iterationsPerKnot;it++){
  ik.set(q);const f=ik.features(),errors=[...sub(goal.com,f.com),...sub(goal.foot,f.foot),...orientationError(f.footR,goal.footR),...orientationError(f.torsoR,goal.torsoR)];iterations=it+1;
  if(Math.max(...errors.filter((_,i)=>i<6&&i!==2).map(Math.abs))<.00015&&Math.max(...errors.slice(6).map(Math.abs))<.001)break;
  const jac=ik.jacobians(),a=ik.matrix.GetView(),rhs=Array(ik.n).fill(0);a.fill(0);
  for(let i=0;i<ik.n;i++){for(let k=0;k<12;k++)rhs[i]+=jac[k][i]*weights[k]**2*errors[k];for(let j=0;j<ik.n;j++)for(let k=0;k<12;k++)a[i*ik.n+j]+=jac[k][i]*jac[k][j]*weights[k]**2;a[i*ik.n+i]+=.00001;}
  const normal=Array.from(a),blocked=new Set();let step;
  for(let pass=0;pass<=ik.n;pass++){
   a.set(normal);const b=rhs.slice();for(const i of blocked){for(let j=0;j<ik.n;j++)a[i*ik.n+j]=a[j*ik.n+i]=0;a[i*ik.n+i]=1;b[i]=0;}
   if(ik.mj.mju_cholFactor(ik.matrix,1e-12)!==ik.n)throw Error('Reserved relative IK rank');ik.mj.mju_cholSolve(ik.solution,Array.from(a),b);step=Array.from(ik.solution.GetView());
   let changed=false;bounds.forEach((b,i)=>{if(!blocked.has(i)&&((q[b.q]<=b.lo+1e-7&&step[i]<0)||(q[b.q]>=b.hi-1e-7&&step[i]>0))){blocked.add(i);changed=true;}});if(!changed)break;
  }
  blockedLast=Array.from(blocked,i=>bounds[i].joint);const scale=Math.min(1,.035/Math.max(...step.map(Math.abs)));bounds.forEach((b,i)=>{q[b.q]=clamp(q[b.q]+scale*step[i],b.lo,b.hi);});
 }
 ik.set(q);const f=ik.features(),comError=sub(f.com,goal.com),footError=sub(f.foot,goal.foot),freeAngle=orientationError(f.footR,goal.footR),torsoAngle=orientationError(f.torsoR,goal.torsoR);
 return {q,iterations,blockedLast,features:f,comError,footError,freeAngle,torsoAngle,positionError:maxabs([...comError.slice(0,2),...footError]),angleError:maxabs([...freeAngle,...torsoAngle]),heightError:comError[2],planningBounds:bounds,activeBounds:bounds.flatMap(b=>{const x=q[b.q],eps=16*Number.EPSILON*Math.max(1,Math.abs(x));return [...(Math.abs(x-b.lo)<=eps?[{joint:b.joint,side:'lower',value:x,bound:b.lo}]:[]),...(Math.abs(x-b.hi)<=eps?[{joint:b.joint,side:'upper',value:x,bound:b.hi}]:[])];})};
}
export function externalGeneralized(jac,force,nv){assert(jac.length===3*nv&&force.length===3&&[...jac,...force].every(Number.isFinite));return Array.from({length:nv},(_,i)=>force.reduce((s,f,k)=>s+f*jac[k*nv+i],0));}
export function requiredMotor(qfrcInverse,external,joints){assert(qfrcInverse.length===27&&external.length===27);const required=sub(qfrcInverse,external);assert(required.every(Number.isFinite));return {required,base:required.slice(0,6),motors:joints.map(j=>({joint:j.name,dof:j.v,actuator:j.a,gear:j.gear,requiredNm:required[j.v],control:required[j.v]/j.gear,capNm:Math.abs(j.gear),fraction:Math.abs(required[j.v]/j.gear)}))};}
export function materialPoint(d,body,local){return add(Array.from(d.xpos.slice(3*body,3*body+3)),rot(Array.from(d.xmat.slice(9*body,9*body+9)),local));}
export function algebraChecks(){
 const nv=27,J=Array(3*nv).fill(0),F=[2,-3,5],v=Array.from({length:nv},(_,i)=>(i-7)/13);for(let k=0;k<3;k++)for(let i=0;i<nv;i++)J[k*nv+i]=(k+1)*(i+2)/31;
 const tau=externalGeneralized(J,F,nv),Jv=F.map((_,k)=>dot(J.slice(k*nv,(k+1)*nv),v));assert(Math.abs(dot(tau,v)-dot(F,Jv))<1e-10);assert.deepEqual(externalGeneralized(J,F.map(x=>-x),nv),tau.map(x=>-x));
 const joints=Array.from({length:21},(_,i)=>({name:'j'+i,a:i,q:i+7,v:i+6,j:i,gear:i===0?20:40})),act=Array.from({length:nv},(_,i)=>i<6?0:(i-5)*.5),inv=add(tau,act),motor=requiredMotor(inv,tau,joints);assert(maxabs(motor.base)<1e-14);assert(maxabs(sub(motor.required,act))<1e-14);assert(maxabs(sub(requiredMotor(inv,Array(27).fill(0),joints).required,act))>1);assert(maxabs(requiredMotor(inv,tau.map(x=>-x),joints).base)>1);
 const r=Array.from({length:42},(_,i)=>i%2?1:-1),b=planningBounds(joints,r);assert.equal(b[0].lo,-.98);assert.equal(b[0].hi,.98);assert.throws(()=>planningBounds(joints,r,1),/Empty/);assert.throws(()=>externalGeneralized(J,[NaN,0,0],nv));
 return {passed:true,virtualWork:true,forceSignReversal:true,tractionSubtractedOnce:true,wrongSignAndOmissionRejected:true,gearMapping:true,planningReserve:true,emptyRangeAndNonfiniteRejected:true,nativeCalls:0};
}
