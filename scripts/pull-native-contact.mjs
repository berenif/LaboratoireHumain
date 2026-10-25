// Native pyramidal-contact/limit response at fixed q,v and external input.
// No forward/step, no state writes, no force injection. Motor controls are the only decisions.
import assert from'node:assert/strict';import{dot,sum}from'./pull-constrained-ls.mjs';import{denseConstraintJacobian,remapIslandStates}from'./pull-native-step-map.mjs';import{assemble,SOURCE_TASKS,forceRows}from'./pull-whole-body.mjs';import{constrainedDirectLS}from'./pull-direct-qr.mjs';
const maxabs=a=>Math.max(0,...a.map(Math.abs)),eps=Number.EPSILON;
// A persistent factor; buffers live only for this closure invocation.
function solver(mj,H){const n=Math.sqrt(H.length),b=new mj.DoubleBuffer(H.length);b.GetView().set(H);let L;try{assert.equal(mj.mju_cholFactor(b,1e-12),n);L=Array.from(b.GetView());}finally{b.delete();}return rhs=>{const out=new mj.DoubleBuffer(n);try{mj.mju_cholSolve(out,L,rhs);return Array.from(out.GetView());}finally{out.delete();}};}
export function contactLaw(J,D,aref,a){return J.map((j,i)=>Math.max(0,-D[i]*(dot(j,a)-aref[i])));}
export function residual(M,J,D,aref,a,smooth){const f=contactLaw(J,D,aref,a),n=a.length;return {f,r:a.map((_,i)=>dot(M.slice(i*n,(i+1)*n),a)-smooth[i]-sum(J.map((j,k)=>j[i]*f[k])))};}
export function errorBound(M,J,D,aref,a,smooth,solveM){const {f,r}=residual(M,J,D,aref,a,smooth),v=solveM(r),square=dot(r,v);assert(Number.isFinite(square)&&square>=-128*eps*Math.max(1,dot(r,r)));return {eta:Math.sqrt(Math.max(0,square)),r,forces:f};}
export function nativeAssembly(p,interaction,ref,assembly=assemble){
 const old=assembly(p,interaction,ref),{model:m,data:d,mj}=p,nv=27,J=denseConstraintJacobian(m,d),D=Array.from(d.efc_D).slice(0,d.nefc),aref=Array.from(d.efc_aref).slice(0,d.nefc),states=remapIslandStates(d).states,types=Array.from(d.efc_type).slice(0,d.nefc);
 assert(types.every(t=>t===3||t===6)&&states.every(s=>s===0||s===1)&&D.every(v=>v>0&&Number.isFinite(v)),'Unsupported native constitutive law');assert(!d.efm_active&&m.opt.disableflags===0&&m.opt.enableflags===0);
 const smooth=old.rhs.map((v,i)=>v-old.limits[i]),active=states.map(s=>s===1),K=old.M.map((v,k)=>v+sum(J.map((j,i)=>active[i]?D[i]*j[Math.floor(k/nv)]*j[k%nv]:0))),solveK=solver(mj,K),solveM=solver(mj,old.M),constant=smooth.map((v,k)=>v+sum(J.map((j,i)=>active[i]?j[k]*D[i]*aref[i]:0))),a0=solveK(constant),A=old.B.slice(0,21).map(solveK);
 const l0=J.map((j,i)=>active[i]?-D[i]*(dot(j,a0)-aref[i]):0),LA=J.map((j,i)=>A.map(a=>active[i]?-D[i]*dot(j,a):0));
 const forceMap=old.contacts.map(c=>{assert(c.geom1===0&&c.dim===3);const adr=c.constraintAddress;assert(Array.from({length:4},(_,i)=>types[adr+i]===6&&d.efc_id[adr+i]===c.index).every(Boolean));return [Array.from({length:d.nefc},(_,i)=>i>=adr&&i<adr+4?1:0),Array.from({length:d.nefc},(_,i)=>i===adr?c.friction[0]:i===adr+1?-c.friction[0]:0),Array.from({length:d.nefc},(_,i)=>i===adr+2?c.friction[1]:i===adr+3?-c.friction[1]:0)];}).flat();
 const F0=forceMap.map(row=>dot(row,l0)/old.mg),FA=forceMap.map(row=>Array.from({length:21},(_,k)=>sum(row.map((v,i)=>v*LA[i][k]))/old.mg));
 const tasks=old.tasks.map(t=>({...t,a:A.map(a=>dot(t.jac,a)),offset:dot(t.jac,a0)+t.bias})),C=tasks.map(t=>t.a.map(v=>v*t.weight/t.scale)),target=tasks.map(t=>(t.desired-t.offset)*t.weight/t.scale);
 for(let i=0;i<21;i++){C.push(Array.from({length:21},(_,j)=>i===j?Math.sqrt(SOURCE_TASKS.motorRegularizer):0));target.push(0);}
 for(let i=0;i<F0.length;i++){C.push(FA[i].map(v=>v*Math.sqrt(SOURCE_TASKS.forceRegularizer)));target.push(-F0[i]*Math.sqrt(SOURCE_TASKS.forceRegularizer));}
 for(let i=0;i<F0.length;i++){const k=i%3,w=k?SOURCE_TASKS.shearWeight:SOURCE_TASKS.normalWeight;C.push(FA[i].map(v=>v*w));target.push(w*((k?0:1/old.contacts.length)-F0[i]));}
 const rows=forceRows(old.contacts,old.n).map(r=>({label:r.label,a:Array.from({length:21},(_,j)=>sum(r.a.slice(21).map((v,i)=>v*FA[i][j]))),bound:r.bound-dot(r.a.slice(21),F0)}));
 for(const row of old.rows.filter(r=>r.label.startsWith('motor-')))rows.push({...row,a:row.a.slice(0,21)});
 for(let k=0;k<21;k++){const j=p.joints[k],h=SOURCE_TASKS.stoppingHorizon,q=d.qpos[j.q],v=d.qvel[j.v];for(const s of[-1,1]){const limit=m.jnt_range[2*j.j+(s===1?1:0)],acc=2*s*(limit-q-h*v)/(h*h);rows.push({label:`joint-stopping-${j.name}-${s}`,a:A.map(col=>s*col[j.v]),bound:acc-s*a0[j.v]});}}
 const Minv=Array.from({length:27},(_,k)=>solveM(Array.from({length:27},(_,i)=>i===k?1:0))),H=old.M.map((v,k)=>v+(k%28===0?m.opt.timestep*m.dof_damping[k%27]:0)),solveE=solver(mj,H),T=Array.from({length:27},(_,k)=>solveE(old.M.filter((_,i)=>i%27===k)));
 return {...old,n:21,C,d:target,rows,tasks,A,a0,smooth,J,D,aref,states,types,active,K,F0,FA,forceMap,Minv,T,contactLawAssumption:'Exact piecewise-affine native law for present pyramidal contacts and limits at fixed physical state; no fixed-branch admission requirement',H:undefined,b:undefined};
}
function quadInv(q,row){return Math.max(0,sum(row.map((v,i)=>v*dot(q.Minv[i],row))));}
export function assess(q,raw){
 const actuator=q.B.slice(0,21),smooth=q.smooth.map((v,i)=>v+sum(actuator.map((b,j)=>b[i]*raw.controls[j]))),solveM=r=>q.Minv.map(col=>dot(col,r)),bound=errorBound(q.M,q.J,q.D,q.aref,raw.qacc,smooth,solveM),lambda=bound.forces,recorded=raw.edgeForces;
 assert(recorded.length===lambda.length);const lawError=maxabs(recorded.map((v,i)=>v-lambda[i])),arithmetic=128*eps*Math.max(1,...recorded.map(Math.abs),...lambda.map(Math.abs));assert(lawError<=arithmetic,'Reconstructed native force law');
 const force=q.forceMap.map(row=>dot(row,lambda)/q.mg),forceError=q.forceMap.map(row=>sum(row.map((v,i)=>Math.abs(v)*q.D[i]*Math.sqrt(quadInv(q,q.J[i]))))*bound.eta/q.mg),taskValues=q.tasks.map(t=>dot(t.jac,raw.qacc)+t.bias),taskErrors=q.tasks.map(t=>Math.sqrt(quadInv(q,t.jac))*bound.eta),velocityErrors=q.tasks.slice(0,6).map(t=>{const row=q.T.map(col=>dot(t.jac,col));return Math.sqrt(quadInv(q,row))*bound.eta*.00125;});
 const residuals=q.tasks.map((t,i)=>(taskValues[i]-t.desired)*t.weight/t.scale),errors=q.tasks.map((t,i)=>taskErrors[i]*t.weight/t.scale);
 for(let j=0;j<21;j++){residuals.push(Math.sqrt(SOURCE_TASKS.motorRegularizer)*raw.controls[j]);errors.push(0);}
 for(let i=0;i<force.length;i++){residuals.push(Math.sqrt(SOURCE_TASKS.forceRegularizer)*force[i]);errors.push(Math.sqrt(SOURCE_TASKS.forceRegularizer)*forceError[i]);}
 for(let i=0;i<force.length;i++){const k=i%3,w=k?SOURCE_TASKS.shearWeight:SOURCE_TASKS.normalWeight;residuals.push(w*(force[i]-(k?0:1/q.contacts.length)));errors.push(w*forceError[i]);}
 const cost=.5*dot(residuals,residuals),costError=sum(residuals.map((v,i)=>Math.abs(v)*errors[i]+.5*errors[i]**2))+128*eps*Math.max(1,cost);
 return {eta:bound.eta,maxDynamicResidual:maxabs(bound.r),lawError,force,forceError,taskValues,taskErrors,velocityErrors,cost,costError,costLower:Math.max(0,cost-costError),costUpper:cost+costError};
}
export function solveNative(q){const start=performance.now(),s=constrainedDirectLS(q.C,q.d,q.rows,{maxPivots:64,tolerance:1e-10});return {solve:s,solveMs:performance.now()-start};}
export function compare(q,s,raw,centre){
 assert(s.solved);const u=s.x,a=q.a0.map((v,i)=>v+sum(q.A.map((c,j)=>c[i]*u[j]))),predictedEdges=q.J.map((j,i)=>q.active[i]?-q.D[i]*(dot(j,a)-q.aref[i]):0),law=contactLaw(q.J,q.D,q.aref,a),scale=Math.max(1,...predictedEdges.map(Math.abs),...law.map(Math.abs)),lawMismatch=maxabs(law.map((v,i)=>v-predictedEdges[i])),selfConsistent=lawMismatch<=128*eps*scale,actual=assess(q,raw),predictedTasks=q.tasks.map(t=>dot(t.jac,a)+t.bias),differences=predictedTasks.map((v,i)=>Math.abs(v-actual.taskValues[i])),round=q.tasks.map((t,i)=>128*eps*Math.max(1,Math.abs(predictedTasks[i]),Math.abs(actual.taskValues[i]))),modelBound=errorBound(q.M,q.J,q.D,q.aref,a,q.smooth.map((v,i)=>v+sum(q.B.slice(0,21).map((b,j)=>b[i]*u[j]))),r=>q.Minv.map(col=>dot(col,r))),predictionErrors=q.tasks.map((t,i)=>Math.sqrt(quadInv(q,t.jac))*(actual.eta+modelBound.eta)+round[i]);
 const accurate=differences.every((v,i)=>v<=predictionErrors[i]),improves=actual.costUpper<=centre.costLower,failures=[];
 if(!selfConsistent)failures.push('affine branch not self-consistent with native law');if(!accurate)failures.push('native prediction outside certified error');if(!improves)failures.push('native cost improvement not qualified');
 return {passed:failures.length===0,failures,selfConsistent,lawMismatch,predictedAcceleration:a,predictedTasks,actual,differences,predictionErrors,improves,centreCost:centre.cost,centreCostError:centre.costError};
}
