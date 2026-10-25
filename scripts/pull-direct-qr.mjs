// Direct constrained least squares. Householder CPQR and active-face nullspace.
// No C'C, H inverse, Schur matrix, native dynamics, or forces applied.
import {dot,sum} from './pull-constrained-ls.mjs';
const maxabs=a=>Math.max(0,...a.map(Math.abs));
export function qrFactor(input){
 const m=input.length,n=input[0]?.length||0;if(!m||!n||m<n||input.some(r=>r.length!==n||!r.every(Number.isFinite)))throw Error('invalid/nonfinite QR dimensions');
 const R=input.map(r=>r.slice()),permutation=Array.from({length:n},(_,i)=>i),reflectors=[],scale=Math.max(...permutation.map(j=>Math.hypot(...R.map(r=>r[j])))),rankThreshold=Number.EPSILON*Math.max(m,n)*scale;
 for(let k=0;k<n;k++){
  let pivot=k,best=-1;for(let j=k;j<n;j++){const norm=Math.hypot(...R.slice(k).map(r=>r[j]));if(norm>best){best=norm;pivot=j;}}
  if(!(best>rankThreshold))throw Error(`ambiguous/dependent QR rank at${k}/${n}: ${best} <= ${rankThreshold}`);
  if(pivot!==k){for(const row of R)[row[k],row[pivot]]=[row[pivot],row[k]];[permutation[k],permutation[pivot]]=[permutation[pivot],permutation[k]];}
  const v=R.slice(k).map(r=>r[k]),first=v[0],alpha=first>=0?-best:best,tau=(alpha-first)/alpha;v[0]-=alpha;const v0=v[0];for(let i=0;i<v.length;i++)v[i]/=v0;
  for(let j=k;j<n;j++){const s=tau*dot(v,R.slice(k).map(r=>r[j]));for(let i=k;i<m;i++)R[i][j]-=s*v[i-k];}
  R[k][k]=alpha;for(let i=k+1;i<m;i++)R[i][k]=0;reflectors.push({v,tau});
 }
 function apply(vector,transpose){if(vector.length!==m)throw Error('QR vector mapping');const out=vector.slice();for(let t=0;t<n;t++){const k=transpose?t:n-1-t,{v,tau}=reflectors[k],s=tau*dot(v,out.slice(k));for(let i=k;i<m;i++)out[i]-=s*v[i-k];}return out;}
 function upper(rhs,transpose=false){if(rhs.length!==n)throw Error('triangular mapping');const x=Array(n).fill(0);for(let t=0;t<n;t++){const i=transpose?t:n-1-t,terms=[];for(let j=transpose?0:i+1;j<(transpose?i:n);j++)terms.push((transpose?R[j][i]:R[i][j])*x[j]);x[i]=(rhs[i]-sum(terms))/R[i][i];}if(!x.every(Number.isFinite))throw Error('nonfinite triangular solution');return x;}
 return {m,n,R,permutation,rankThreshold,minDiagonal:Math.min(...R.slice(0,n).map((r,i)=>Math.abs(r[i]))),applyQ:v=>apply(v,false),applyQT:v=>apply(v,true),upper,solve(rhs){
   const permuted=upper(apply(rhs,true).slice(0,n)),x=Array(n);permutation.forEach((j,i)=>x[j]=permuted[i]);
   // Bounded LS refinement: original residual/gradient, two triangular R solves.
   // No normal/Schur matrix is formed; the direct QR factor is reused.
   const gradient=y=>{const r=input.map((row,i)=>dot(row,y)-rhs[i]);return Array.from({length:n},(_,j)=>sum(input.map((row,i)=>row[j]*r[i])));};
   let g=gradient(x);for(let iteration=0;iteration<3&&maxabs(g)>0;iteration++){
    const w=upper(permutation.map(j=>-g[j]),true),dp=upper(w),trial=x.slice();permutation.forEach((j,i)=>trial[j]+=dp[i]);const next=gradient(trial);if(!(maxabs(next)<maxabs(g)))break;for(let j=0;j<n;j++)x[j]=trial[j];g=next;
   }return x;
  }};
}
export function directCertificate(C,d,rows,x,lambda,tolerance=1e-10){
 const n=x.length,residual=C.map((r,i)=>dot(r,x)-d[i]),raw=rows.map(r=>dot(r.a,x)-r.bound),rowNorm=rows.map(r=>Math.hypot(...r.a)||1),cxBound=C.map((r,i)=>sum(r.map((v,j)=>Math.abs(v*x[j])))+Math.abs(d[i]));
 const gradient=Array.from({length:n},(_,j)=>sum([...C.map((r,i)=>r[j]*residual[i]),...rows.map((r,i)=>r.a[j]*lambda[i])])),den=Array.from({length:n},(_,j)=>sum([...C.map((r,i)=>Math.abs(r[j])*cxBound[i]),...rows.map((r,i)=>Math.abs(r.a[j]*lambda[i]))]));
 const backwardError=Math.max(0,...gradient.map((v,i)=>den[i]===0?(v===0?0:Infinity):Math.abs(v)/den[i])),primal=Math.max(0,...raw.map((v,i)=>v/rowNorm[i])),dual=Math.min(0,...lambda.map((v,i)=>v*rowNorm[i])),complementarity=Math.max(0,...lambda.map((v,i)=>Math.abs(v*raw[i]))),failures=[];
 if(![...x,...lambda,...residual,...gradient,...den,...raw].every(Number.isFinite))failures.push('nonfinite certificate');if(primal>tolerance)failures.push('original normalized primal');if(dual< -tolerance)failures.push('dual sign');if(complementarity>1e-8)failures.push('complementarity');if(!(backwardError<=tolerance))failures.push('componentwise backward stationarity');
 return {passed:failures.length===0,failures,normalizedPrimalViolation:primal,dualViolation:dual,stationarity:maxabs(gradient),backwardError,complementarity,cost:.5*dot(residual,residual),stationarityVector:gradient,originalConstraintResiduals:raw};
}
export function constrainedDirectLS(C,d,rows,{maxPivots=64,tolerance=1e-10}={}){
 const n=C[0]?.length||0,history=[],active=[];let pivots=0,qrFactorizations=0,z,x,D,lambda=rows.map(()=>0),scaled;
 const output=(reason,cert)=>({solved:!reason,reason,method:'Householder CPQR/direct active-face nullspace',x,pivots,active:active.map(i=>rows[i].label),lambda:scaled?lambda.map((v,i)=>v/scaled[i].norm):undefined,history,qrFactorizations,scaling:D,...cert});
 try{
  if(!n||C.length!==d.length||C.some(r=>r.length!==n)||rows.some(r=>r.a.length!==n)||![...C.flat(),...d,...rows.flatMap(r=>[...r.a,r.bound])].every(Number.isFinite))return output('invalid/nonfinite direct LS');
  const norms=Array.from({length:n},(_,j)=>Math.hypot(...C.map(r=>r[j])));if(norms.some(v=>v===0))return output('unresolved zero cost column');D=norms.map(v=>1/v);
  const Cs=C.map(r=>r.map((v,j)=>v*D[j]));scaled=rows.map(r=>{const a=r.a.map((v,j)=>v*D[j]),norm=Math.hypot(...a)||1;return {...r,norm,a:a.map(v=>v/norm),bound:r.bound/norm,originalNorm:Math.hypot(...r.a)||1};});
  if(scaled.some(r=>r.a.every(v=>v===0)&&r.bound< -tolerance))return output('inconsistent zero row');
  const factor=a=>{qrFactorizations++;return qrFactor(a);},free=factor(Cs);z=free.solve(d);
  function face(){
   if(!active.length)return {z:free.solve(d),lambda:[]};
   const k=active.length;if(k>n)throw Error('dependent active constraints');const E=active.map(i=>scaled[i].a),eq=factor(Array.from({length:n},(_,j)=>E.map(r=>r[j]))),qColumns=Array.from({length:n},(_,j)=>eq.applyQ(Array.from({length:n},(_,i)=>i===j?1:0)));
   const yp=eq.upper(eq.permutation.map(j=>scaled[active[j]].bound),true),zp=Array.from({length:n},(_,i)=>dot(qColumns.slice(0,k).map(v=>v[i]),yp)),Z=qColumns.slice(k),rhs=d.map((v,i)=>v-dot(Cs[i],zp));
   const coordinates=Z.length?factor(Cs.map(r=>Z.map(col=>dot(r,col)))).solve(rhs):[],trial=zp.map((v,i)=>sum([v,...Z.map((col,j)=>col[i]*coordinates[j])]));
   // Correct active-equality reconstruction roundoff in the decision vector,
   // BEFORE certification/admission. No motor clipping or physical-state write.
   for(let iteration=0;iteration<3;iteration++){const er=eq.permutation.map(j=>scaled[active[j]].bound-dot(E[j],trial));if(maxabs(er)===0)break;const dy=eq.upper(er,true),correction=Array.from({length:n},(_,i)=>dot(qColumns.slice(0,k).map(col=>col[i]),dy));for(let i=0;i<n;i++)trial[i]+=correction[i];}
   const r=Cs.map((row,i)=>dot(row,trial)-d[i]),g=Array.from({length:n},(_,j)=>sum(Cs.map((row,i)=>row[j]*r[i]))),lp=eq.upper(qColumns.slice(0,k).map(col=>-dot(col,g))),l=Array(k);eq.permutation.forEach((j,i)=>l[j]=lp[i]);return {z:trial,lambda:l};
  }
  while(true){
   x=z.map((v,j)=>v*D[j]);let entering=-1,biggest=tolerance;rows.forEach((r,i)=>{const violation=(dot(r.a,x)-r.bound)/scaled[i].originalNorm;if(!active.includes(i)&&violation>biggest){biggest=violation;entering=i;}});
   if(entering<0){const cert=directCertificate(C,d,rows,x,lambda.map((v,i)=>v/scaled[i].norm),tolerance);return output(cert.passed?undefined:'direct KKT certificate failed: '+cert.failures.join(', '),cert);}
   if(pivots>=maxPivots)return output('active-set pivot budget');active.push(entering);pivots++;history.push({action:'add',row:rows[entering].label});
   while(true){
    const trial=face();if(trial.lambda.every(v=>v>=0)){z=trial.z;lambda.fill(0);active.forEach((i,k)=>lambda[i]=trial.lambda[k]);break;}
    let fraction=1;active.forEach((i,k)=>{if(trial.lambda[k]<0)fraction=Math.min(fraction,lambda[i]/(lambda[i]-trial.lambda[k]));});
    z=z.map((v,j)=>v+fraction*(trial.z[j]-v));active.forEach((i,k)=>lambda[i]+=fraction*(trial.lambda[k]-lambda[i]));
    const removed=active.filter((i,k)=>trial.lambda[k]<0&&lambda[i]<=0);if(!removed.length)return output('active-set numerical stagnation');
    for(const i of removed){if(pivots>=maxPivots)return output('active-set pivot budget');active.splice(active.indexOf(i),1);lambda[i]=0;pivots++;history.push({action:'drop',row:rows[i].label});}
   }
  }
 }catch(e){return output('QR/dependency failure; feasibility undetermined: '+String(e));}
}
