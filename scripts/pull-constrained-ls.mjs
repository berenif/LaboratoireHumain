// Small diagnostic active-set QP, using the existing MuJoCo Cholesky only.
// min .5*x'Hx-b'x, A*x<=c. No dynamics, model mutation, or applied forces.
export function sum(values){let s=0,c=0;for(const v of values){const t=s+v;c+=Math.abs(s)>=Math.abs(v)?(s-t)+v:(v-t)+s;s=t;}return s+c;}
export const dot=(a,b)=>sum(a.map((v,i)=>v*b[i]));
const maxabs=a=>Math.max(0,...a.map(Math.abs));
function factor(mj,matrix,n){
 const a=new mj.DoubleBuffer(n*n),x=new mj.DoubleBuffer(n);a.GetView().set(matrix);
 try{if(mj.mju_cholFactor(a,1e-12)!==n)throw Error('dependent or non-positive Cholesky system');}
 catch(e){a.delete();x.delete();throw e;}
 const fact=Array.from(a.GetView());return {solve(rhs){mj.mju_cholSolve(x,fact,rhs);const out=Array.from(x.GetView());if(!out.every(Number.isFinite))throw Error('nonfinite Cholesky solution');return out;},delete(){a.delete();x.delete();}};
}
export function cholSolve(mj,matrix,rhs){if(!rhs.length)return [];const f=factor(mj,matrix,rhs.length);try{return f.solve(rhs);}finally{f.delete();}}
export function certificate(H,b,rows,x,lambda,tolerance=1e-10){
 const n=b.length,rawPrimal=rows.map(r=>dot(r.a,x)-r.bound),norms=rows.map(r=>Math.hypot(...r.a)||1),stationarityVector=Array.from({length:n},(_,j)=>sum([...H.slice(j*n,(j+1)*n).map((h,i)=>h*x[i]),-b[j],...rows.map((r,i)=>r.a[j]*lambda[i])]));
 const denominators=Array.from({length:n},(_,j)=>sum([...H.slice(j*n,(j+1)*n).map((h,i)=>Math.abs(h*x[i])),Math.abs(b[j]),...rows.map((r,i)=>Math.abs(r.a[j]*lambda[i]))]));
 const relative=stationarityVector.map((r,i)=>denominators[i]===0?(r===0?0:Infinity):Math.abs(r)/denominators[i]);
 const primal=Math.max(0,...rawPrimal.map((v,i)=>v/norms[i])),dual=Math.min(0,...lambda.map((v,i)=>v*norms[i])),complementarity=Math.max(0,...lambda.map((l,i)=>Math.abs(l*rawPrimal[i]))),backwardError=maxabs(relative);
 const failures=[];if(![...x,...lambda,...rawPrimal,...stationarityVector,...denominators].every(Number.isFinite))failures.push('nonfinite KKT');if(primal>tolerance)failures.push('original normalized primal');if(dual< -tolerance)failures.push('dual sign');if(complementarity>1e-8)failures.push('complementarity');if(!(backwardError<=tolerance))failures.push('componentwise backward stationarity');
 return {passed:failures.length===0,failures,normalizedPrimalViolation:primal,dualViolation:dual,stationarity:maxabs(stationarityVector),complementarity,backwardError,stationarityVector,stationarityDenominators:denominators,originalConstraintResiduals:rawPrimal};
}
export function constrainedLS(mj,H,b,rows,{maxPivots=64,tolerance=1e-10,maxRefinements=3}={}){
 const n=b.length,history=[],refinements=[];let pivots=0,hf,af,z,lambda=rows.map(()=>0),active=[],D,scaled,Q,inv,xFree,rhs,hessianFactorizations=0,activeFactorizations=0;
 const result=(reason,cert)=>{const x=z?.map((v,j)=>v*D[j]),originalLambda=scaled?lambda.map((v,i)=>v/scaled[i].norm):undefined;return {solved:!reason,reason,x,pivots,active:active.map(i=>rows[i].label),lambda:originalLambda,history,refinements,hessianFactorizations,activeFactorizations,scaling:D,...cert};};
 try{
  if(!n||H.length!==n*n||rows.some(r=>r.a.length!==n)||![...H,...b,...rows.flatMap(r=>[...r.a,r.bound])].every(Number.isFinite))return result('invalid/nonfinite QP');
  if(Array.from({length:n},(_,i)=>H[i*n+i]).some(h=>h<=0))return result('non-positive Hessian diagonal');
  D=Array.from({length:n},(_,i)=>1/Math.sqrt(H[i*n+i]));
  const hs=H.map((h,k)=>h*D[Math.floor(k/n)]*D[k%n]),bs=b.map((v,i)=>v*D[i]);
  if(hs.some((h,k)=>Math.abs(h-hs[k%n*n+Math.floor(k/n)])>1e-12*Math.max(1,Math.abs(h))))return result('asymmetric Hessian');
  scaled=rows.map(r=>{const a=r.a.map((v,i)=>v*D[i]),norm=Math.hypot(...a)||1;return {...r,norm,a:a.map(v=>v/norm),bound:r.bound/norm};});
  for(const r of scaled)if(r.a.every(v=>v===0)&&r.bound< -tolerance)return result('inconsistent zero row');
  hf=factor(mj,hs,n);hessianFactorizations++;xFree=hf.solve(bs);z=xFree.slice();
  inv=scaled.map(r=>hf.solve(r.a));Q=scaled.map(r=>inv.map(v=>dot(r.a,v)));
  // Symmetric Schur matrix: both entries approximate the same exact scalar.
  for(let i=0;i<Q.length;i++)for(let j=0;j<i;j++)Q[i][j]=Q[j][i]=(Q[i][j]+Q[j][i])*.5;
  rhs=scaled.map(r=>dot(r.a,xFree)-r.bound);
  const current=()=>xFree.map((v,j)=>sum([v,...lambda.map((l,i)=>-l*inv[i][j])]));
  while(true){
   const violation=rhs.map((r,i)=>r-dot(Q[i],lambda));let entering=-1,biggest=tolerance;violation.forEach((v,i)=>{if(!active.includes(i)&&v>biggest){biggest=v;entering=i;}});
   if(entering<0){
    z=current();let cert=certificate(H,b,rows,z.map((v,j)=>v*D[j]),lambda.map((v,i)=>v/scaled[i].norm),tolerance);
    for(let iteration=0;!cert.passed&&iteration<maxRefinements;iteration++){
     const rd=cert.stationarityVector.map((v,j)=>v*D[j]),rp=active.map(i=>dot(scaled[i].a,z)-scaled[i].bound),hinv=hf.solve(rd);
     const dl=active.length?af.solve(active.map((i,k)=>rp[k]-dot(scaled[i].a,hinv))):[],dz=hf.solve(rd.map((v,j)=>-sum([v,...active.map((i,k)=>scaled[i].a[j]*dl[k])])));
     const nextZ=z.map((v,j)=>v+dz[j]),nextLambda=lambda.slice();active.forEach((i,k)=>nextLambda[i]+=dl[k]);
     const next=certificate(H,b,rows,nextZ.map((v,j)=>v*D[j]),nextLambda.map((v,i)=>v/scaled[i].norm),tolerance),score=c=>Math.max(c.normalizedPrimalViolation/tolerance,-c.dualViolation/tolerance,c.complementarity/1e-8,c.backwardError/tolerance);
     refinements.push({iteration:iteration+1,before:score(cert),after:score(next),maxCommandCorrection:Math.max(0,...dz.slice(0,21).map((v,j)=>Math.abs(v*D[j])))});
     if(!next.passed&&!(score(next)<score(cert)))return result('KKT refinement without improvement',cert);
     z=nextZ;lambda=nextLambda;cert=next;
    }
    return result(cert.passed?undefined:'KKT certificate failed: '+cert.failures.join(', '),cert);
   }
   if(pivots>=maxPivots){z=current();return result('pivot budget');}active.push(entering);pivots++;history.push({action:'add',row:rows[entering].label});
   while(true){
    if(af)af.delete();af=null;
    if(active.length){af=factor(mj,active.flatMap(i=>active.map(j=>Q[i][j])),active.length);activeFactorizations++;}
    const trial=active.length?af.solve(active.map(i=>rhs[i])):[];
    if(trial.every(x=>x>tolerance)){lambda.fill(0);active.forEach((i,k)=>lambda[i]=trial[k]);break;}
    let fraction=1;active.forEach((i,k)=>{if(trial[k]<=tolerance&&lambda[i]-trial[k]>0)fraction=Math.min(fraction,lambda[i]/(lambda[i]-trial[k]));});active.forEach((i,k)=>lambda[i]+=fraction*(trial[k]-lambda[i]));
    const removed=active.filter(i=>lambda[i]<=tolerance);if(!removed.length){z=current();return result('active-set numerical stagnation');}
    for(const i of removed){if(pivots>=maxPivots){z=current();return result('pivot budget');}active.splice(active.indexOf(i),1);lambda[i]=0;pivots++;history.push({action:'drop',row:rows[i].label});}
   }
  }
 }catch(e){return result('factorization/nonfinite failure; feasibility undetermined: '+String(e));}
 finally{hf?.delete();af?.delete();}
}
