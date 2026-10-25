// Diagnostic only: no mj_forward/mj_step, no change to controls or native qacc.
// MuJoCo 3.17 Newton: grad = M*a - smooth - J'force; K = M + J_A'D_AJ_A.
import assert from 'node:assert/strict';
import {cholSolve,dot} from './pull-constrained-ls.mjs';
import {denseConstraintJacobian} from './pull-native-step-map.mjs';
const maxabs=a=>Math.max(0,...a.map(Math.abs));
export function closeArrays(a,b,label){
 assert.equal(a.length,b.length,label+' length');
 const error=maxabs(a.map((v,i)=>v-b[i])),limit=128*Number.EPSILON*Math.max(1,maxabs(a),maxabs(b));
 assert(error<=limit,label+' mismatch');return {error,limit};
}
export function stationaryCertificate({M,J,D,aref,states,qacc,forces,residual,h,damping,tolerance},solve){
 const n=qacc.length;assert.equal(M.length,n*n);assert.equal(J.length,D.length);
 assert([...M,...J.flat(),...D,...aref,...qacc,...forces,...residual,...damping,h,tolerance].every(Number.isFinite));
 assert(tolerance>0&&h>0&&D.every(x=>x>0));assert(states.every(x=>x===0||x===1));
 const K=M.map((v,k)=>v+J.reduce((s,j,i)=>s+(states[i]===1?D[i]*j[Math.floor(k/n)]*j[k%n]:0),0));
 const correction=solve(K,residual.map(v=>-v)),corrected=qacc.map((v,i)=>v+correction[i]);
 const backward=residual.map((v,i)=>v+dot(K.slice(i*n,(i+1)*n),correction));
 const backwardLimit=128*Number.EPSILON*Math.max(1,maxabs(residual),...K.map((_,k)=>Math.abs(K[k]*correction[k%n])))*n;
 assert(maxabs(backward)<=backwardLimit,'Stationary correction backward error');
 const trace=M.reduce((s,v,k)=>s+(k%(n+1)===0?v:0),0);assert(trace>0);
 const decrement=Math.max(0,-.5*dot(residual,correction)/trace);
 assert(decrement<=tolerance,'Stationarity uncertainty exceeds native cost tolerance');
 const lambdas=J.map((j,i)=>-D[i]*(dot(j,corrected)-aref[i]));
 const lambdaRoundoff=J.map((j,i)=>128*Number.EPSILON*D[i]*Math.max(1,Math.abs(aref[i])+j.reduce((s,v,k)=>s+Math.abs(v*corrected[k]),0)));
 assert(lambdas.every((v,i)=>states[i]===1?v>lambdaRoundoff[i]:v<=-lambdaRoundoff[i]),'Stationary correction changes branch or lacks sign margin');
 const correctedForces=lambdas.map((v,i)=>states[i]===1?v:0);
 const H=M.map((v,k)=>v+(k%(n+1)===0?h*damping[k%(n)]:0));
 const deltaEuler=solve(H,correction.map((_,i)=>dot(M.slice(i*n,(i+1)*n),correction)));
 return {correction,corrected,deltaEuler,deltaVelocity:deltaEuler.map(v=>h*v),correctedForces,correctedLambdas:lambdas,
  deltaForces:correctedForces.map((v,i)=>v-forces[i]),decrement,trace,backwardMax:maxabs(backward),backwardLimit,
  residualMax:maxabs(residual),normalizedGradient:Math.hypot(...residual)/trace,
  minActiveForce:Math.min(...lambdas.filter((_,i)=>states[i]===1))};
}
export function numericalMargin(value,uncertainty,tolerance){
 assert(Number.isFinite(uncertainty)&&uncertainty>=0);return value+uncertainty<=tolerance;
}
export function reconstructArrays(mj,model,data,raw){
 const n=model.nv,buf=new mj.DoubleBuffer(n*n);mj.mj_fullM(model,data,buf);const M=Array.from(buf.GetView());
 const copy=new mj.MjData(model);copy.qpos.set(data.qpos);mj.mj_kinematics(model,copy);mj.mj_comPos(model,copy);mj.mj_makeM(model,copy);mj.mj_fullM(model,copy,buf);
 const massCheck=closeArrays(M,Array.from(buf.GetView()),'Reconstructed mass');copy.delete();buf.delete();
 const jp=new mj.DoubleBuffer(3*n),jr=new mj.DoubleBuffer(3*n),contactJ={};
 for(const c of raw.contacts){assert(c.geom1===0&&c.dim===3);mj.mj_jac(model,data,jp,jr,c.position,c.body);const a=Array.from(jp.GetView());contactJ[c.index]=[0,1,2].map(k=>Array.from({length:n},(_,j)=>c.frame[3*k]*a[j]+c.frame[3*k+1]*a[n+j]+c.frame[3*k+2]*a[2*n+j]));}
 jp.delete();jr.delete();const counts={},J=raw.arrays.efc_type.map((type,i)=>{
  const id=raw.arrays.efc_id[i];if(type===3){const row=Array(n).fill(0),q=data.qpos[model.jnt_qposadr[id]];row[model.jnt_dofadr[id]]=q-model.jnt_range[2*id]<model.jnt_range[2*id+1]-q?1:-1;return row;}
  assert.equal(type,6);const k=counts[id]||0;counts[id]=k+1;const dim=1+Math.floor(k/2),sign=k%2?-1:1,c=raw.contacts[id];return contactJ[id][0].map((v,j)=>v+sign*c.friction[dim-1]*contactJ[id][dim][j]);
 });
 const forceCheck=closeArrays(Array.from({length:n},(_,i)=>J.reduce((s,j,k)=>s+j[i]*raw.arrays.efc_force[k],0)),raw.arrays.qfrc_constraint,'Reconstructed constraint force');
 return {M,J,massCheck,forceCheck};
}
export function nativeCertificate(mj,model,data,raw,residual){
 const {M,J,massCheck,forceCheck}=reconstructArrays(mj,model,data,raw);
 const jacobianCheck=closeArrays(J.flat(),denseConstraintJacobian(model,data).flat(),'Reconstructed Jacobian');
 const result=stationaryCertificate({M,J,D:raw.arrays.efc_D,aref:raw.arrays.efc_aref,states:raw.resolved.states,qacc:raw.arrays.qacc,forces:raw.arrays.efc_force,residual,h:model.opt.timestep,damping:Array.from(model.dof_damping),tolerance:model.opt.tolerance},(a,b)=>cholSolve(mj,a,b));
 // Map pyramid edge corrections back to actual contact-frame reactions.
 const contactDelta=raw.contacts.map(c=>[0,0,0]),counts={};
 for(let i=0;i<J.length;i++)if(raw.arrays.efc_type[i]===6){const id=raw.arrays.efc_id[i],k=counts[id]||0;counts[id]=k+1;contactDelta[id][0]+=result.deltaForces[i];contactDelta[id][1+Math.floor(k/2)]+=(k%2?-1:1)*raw.contacts[id].friction[Math.floor(k/2)]*result.deltaForces[i];}
 return {...result,massCheck,jacobianCheck,forceCheck,contactDelta};
}
