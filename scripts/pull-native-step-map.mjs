// MuJoCo 3.17 diagnostic adapters, no forward/step and no model changes.
// See engine_forward.c:mj_EulerSkip/fwdConstraint and engine_solver.c:PrimalPointers.
import assert from 'node:assert/strict';
import{cholSolve,dot}from'./pull-constrained-ls.mjs';
export const enumValue=x=>x?.value??x;
export function remapIslandStates(d){
 const n=d.nefc;if(!d.nisland)return{states:Array.from(d.efc_state).slice(0,n),source:'efc_state',map:null};
 const to=Array.from(d.map_efc2iefc).slice(0,n),from=Array.from(d.map_iefc2efc).slice(0,n);
 assert.equal(new Set(to).size,n,'Island mapping not a permutation');
 const states=to.map((j,i)=>{assert(Number.isInteger(j)&&j>=0&&j<n&&from[j]===i,'Island inverse map mismatch');assert.equal(d.iefc_type[j],d.efc_type[i]);assert.equal(d.iefc_id[j],d.efc_id[i]);assert(Math.abs(d.iefc_force[j]-d.efc_force[i])<=1e-9,'Island/global force mismatch');return d.iefc_state[j];});
 return{states,source:'iefc_state via map_efc2iefc',map:to};
}
export function denseConstraintJacobian(model,d){
 const n=model.nv,J=[];const sparse=enumValue(model.opt.jacobian)===1||(enumValue(model.opt.jacobian)===2&&n>=60);
 for(let i=0;i<d.nefc;i++){
  const row=Array(n).fill(0);if(sparse){for(let j=d.efc_J_rowadr[i];j<d.efc_J_rowadr[i]+d.efc_J_rownnz[i];j++)row[d.efc_J_colind[j]]=d.efc_J[j];}else for(let j=0;j<n;j++)row[j]=d.efc_J[i*n+j];J.push(row);
 }return J;
}
export function branchResponse(model,d,states){
 const J=denseConstraintJacobian(model,d),rows=J.map((row,i)=>{
  assert([3,6].includes(d.efc_type[i]),'Only current joint limits and pyramidal contacts supported');
  assert([0,1].includes(states[i]),'Unsupported branch state');
  const jar=dot(row,d.qacc)-d.efc_aref[i],D=d.efc_D[i],lambda=-D*jar;
  assert(D>0&&Number.isFinite(D));assert(Math.abs(Math.max(0,lambda)-d.efc_force[i])<=1e-7,'Native branch force law mismatch');
  assert(states[i]===(jar>=0?0:1),'Resolved state disagrees with native jar');
  return{label:`branch-${i}-type${d.efc_type[i]}-id${d.efc_id[i]}`,unit:'N',type:d.efc_type[i],id:d.efc_id[i],state:states[i],jar,D,lambda,value:states[i]===1?-lambda:lambda};
 });return rows;
}
export function eulerAcceleration(mj,model,d,qacc=Array.from(d.qacc)){
 assert.equal(enumValue(model.opt.integrator),0,'Euler only');assert.equal(model.opt.disableflags,0);assert.equal(model.opt.enableflags,0);assert.equal(d.efm_active,0);
 assert(Array.from(model.dof_dampingpoly).every(x=>x===0),'Polynomial damping unsupported');assert(Array.from(model.jnt_actuatorid).every(x=>x===-1),'Joint servo damping unsupported');
 const buf=new mj.DoubleBuffer(model.nv**2);mj.mj_fullM(model,d,buf);const M=Array.from(buf.GetView());buf.delete();
 const damping=Array.from(model.dof_damping),H=M.map((v,i)=>v+(Math.floor(i/model.nv)===i%model.nv?model.opt.timestep*damping[i%model.nv]:0));
 const rhs=qacc.map((_,i)=>dot(M.slice(i*model.nv,(i+1)*model.nv),qacc));return cholSolve(mj,H,rhs);
}
