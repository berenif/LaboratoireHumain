// Small diagnostic active-set solve using the existing MuJoCo Cholesky only.
// min .5*x'Hx-b'x, A*x<=c. No dynamics, model mutation, or runtime controller.
export const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
export function cholSolve(mj,matrix,rhs){
 const n=rhs.length;if(!n)return [];
 const a=new mj.DoubleBuffer(n*n),x=new mj.DoubleBuffer(n);a.GetView().set(matrix);
 try{if(mj.mju_cholFactor(a,1e-12)!==n)throw Error('dependent or non-positive Cholesky system');mj.mju_cholSolve(x,Array.from(a.GetView()),rhs);return Array.from(x.GetView());}
 finally{a.delete();x.delete();}
}
export function constrainedLS(mj,H,b,rows,{maxPivots=64,tolerance=1e-10}={}){
 const n=b.length,xFree=cholSolve(mj,H,b),scaled=rows.map(r=>{const norm=Math.hypot(...r.a);return {...r,norm,a:r.a.map(x=>x/(norm||1)),bound:r.bound/(norm||1)};});
 for(const r of scaled)if(r.norm===0&&r.bound< -tolerance)return{solved:false,reason:'inconsistent zero row',row:r.label,pivots:0};
 const inv=scaled.map(r=>cholSolve(mj,H,r.a)),Q=scaled.map(r=>inv.map(v=>dot(r.a,v))),rhs=scaled.map(r=>dot(r.a,xFree)-r.bound),lambda=rows.map(()=>0),active=[];
 let pivots=0;const history=[];
 const current=()=>xFree.map((v,j)=>v-lambda.reduce((s,l,i)=>s+l*inv[i][j],0));
 const stopped=reason=>({solved:false,reason,pivots,active:active.map(i=>rows[i].label),x:current(),history});
 while(true){
  const violation=rhs.map((r,i)=>r-dot(Q[i],lambda));let entering=-1,biggest=tolerance;
  violation.forEach((v,i)=>{if(!active.includes(i)&&v>biggest){biggest=v;entering=i;}});
  if(entering<0){
   const x=current(),primal=Math.max(0,...scaled.map(r=>dot(r.a,x)-r.bound)),stationarity=Array.from({length:n},(_,j)=>dot(H.slice(j*n,(j+1)*n),x)-b[j]+lambda.reduce((s,l,i)=>s+l*scaled[i].a[j],0));
   const dual=Math.min(0,...lambda),complementarity=Math.max(0,...lambda.map((l,i)=>Math.abs(l*(dot(scaled[i].a,x)-scaled[i].bound))));
   return {solved:primal<=tolerance&&dual>=-tolerance&&Math.max(...stationarity.map(Math.abs))<=1e-8&&complementarity<=1e-8,x,pivots,active:active.map(i=>rows[i].label),lambda:lambda.map((l,i)=>l/(scaled[i].norm||1)),normalizedPrimalViolation:primal,dualViolation:dual,stationarity:Math.max(...stationarity.map(Math.abs)),complementarity,history};
  }
  if(pivots>=maxPivots)return stopped('pivot budget');active.push(entering);pivots++;history.push({action:'add',row:rows[entering].label});
  while(true){
   let trial;try{trial=cholSolve(mj,active.flatMap(i=>active.map(j=>Q[i][j])),active.map(i=>rhs[i]));}catch{return stopped('dependent active rows; feasibility undetermined');}
   if(trial.every(x=>x>tolerance)){lambda.fill(0);active.forEach((i,k)=>lambda[i]=trial[k]);break;}
   let fraction=1;active.forEach((i,k)=>{if(trial[k]<=tolerance&&lambda[i]-trial[k]>0)fraction=Math.min(fraction,lambda[i]/(lambda[i]-trial[k]));});
   active.forEach((i,k)=>lambda[i]+=fraction*(trial[k]-lambda[i]));
   const removed=active.filter(i=>lambda[i]<=tolerance);if(!removed.length)return stopped('active-set numerical stagnation');
   for(const i of removed){if(pivots>=maxPivots)return stopped('pivot budget');active.splice(active.indexOf(i),1);lambda[i]=0;pivots++;history.push({action:'drop',row:rows[i].label});}
  }
 }
}
