// Pure evaluator contracts; no WASM, native calls, optimizer or controller writes.
import assert from 'node:assert/strict';
import {qrFactor} from './pull-direct-qr.mjs';
export const diagnosticText=x=>JSON.stringify(x,(_,v)=>typeof v==='number'&&!Number.isFinite(v)?{nonfiniteBinary64:String(v)}:v);
export function recordBeforeGuard(snapshot,record,guard){record(snapshot);return guard(snapshot);}
const bits=new DataView(new ArrayBuffer(8));
export function nextUp(x){
 if(Number.isNaN(x)||x===Infinity)return x;if(x===0)return Number.MIN_VALUE;
 bits.setFloat64(0,x);bits.setBigUint64(0,bits.getBigUint64(0)+(x>0?1n:-1n));return bits.getFloat64(0);
}
export const nextDown=x=>-nextUp(-x);
const point=x=>[x,x],add=(a,b)=>[nextDown(a[0]+b[0]),nextUp(a[1]+b[1])],neg=a=>[-a[1],-a[0]],minus=(a,b)=>add(a,neg(b));
function times(a,b){const v=[a[0]*b[0],a[0]*b[1],a[1]*b[0],a[1]*b[1]];return[nextDown(Math.min(...v)),nextUp(Math.max(...v))];}
const upperAbs=a=>Math.max(Math.abs(a[0]),Math.abs(a[1]));
function isum(values){return values.reduce(add,point(0));}
function idot(a,b){return isum(a.map((v,i)=>times(v,b[i])));}
const midpoint=a=>a[0]/2+a[1]/2;
const maxAbs=a=>Math.max(0,...a.map(upperAbs));
const finite=a=>a.every(Number.isFinite);
const matrixRows=(a,n)=>Array.from({length:n},(_,i)=>a.slice(i*n,(i+1)*n));
const normInf=A=>Math.max(...A.map(row=>isum(row.map(v=>point(upperAbs(v))))[1]));

// Validated Neumann certificate: rho=||I-BH||inf<1 implies
// ||H^-1||inf <= ||B||inf/(1-rho). B is merely an approximate inverse;
// outward-rounded interval residuals, not QR accuracy claims, certify the bound.
export function linearCertificate(H,rhs,x,inverseApprox){
 const n=x.length;
 if(!n||H.length!==n*n||rhs.length!==n||![...H,...rhs].every(a=>a.length===2&&a[0]<=a[1])||!finite([...H.flat(),...rhs.flat(),...x]))return{status:'inconclusive',reason:'invalid/nonfinite interval system'};
 try{
  const rows=matrixRows(H,n),mid=rows.map(row=>row.map(midpoint)),factor=inverseApprox?null:qrFactor(mid);
  const columns=inverseApprox?null:Array.from({length:n},(_,j)=>factor.solve(Array.from({length:n},(_,i)=>i===j?1:0)));
  const B=inverseApprox??Array.from({length:n},(_,i)=>columns.map(col=>col[i]));
  if(B.length!==n||B.some(row=>row.length!==n||!finite(row)))return{status:'inconclusive',reason:'invalid inverse approximation'};
  const R=B.map((row,i)=>Array.from({length:n},(_,j)=>minus(point(i===j?1:0),idot(row.map(point),rows.map(r=>r[j])))));
  const rho=normInf(R),bnorm=normInf(B.map(row=>row.map(point))),den=nextDown(1-rho);
  if(!(rho<1&&den>0))return{status:'inconclusive',reason:'inverse defect not bounded below one',rho};
  const inverseNormUpper=nextUp(bnorm/den),conditionUpper=nextUp(normInf(rows)*inverseNormUpper);
  const residual=rows.map((row,i)=>minus(idot(row,x.map(point)),rhs[i]));
  const residualUpper=maxAbs(residual),errorUpper=nextUp(inverseNormUpper*residualUpper);
  if(!finite([rho,inverseNormUpper,conditionUpper,residualUpper,errorUpper]))return{status:'inconclusive',reason:'interval overflow'};
  return{status:'certified',scope:'exact real linear system on supplied binary64 data/intervals only',rho,inverseNormUpper,conditionUpper,residual,residualUpper,errorUpper};
 }catch(e){return{status:'inconclusive',reason:'inverse/rank certificate unavailable: '+String(e)};}
}

export function validateJointMapping(joints,nq,nv){
 assert.equal(nq,28);assert.equal(nv,27);assert.equal(joints.length,21);
 assert.deepEqual(joints.map(j=>j.q).slice().sort((a,b)=>a-b),Array.from({length:21},(_,i)=>i+7),'Hinge qpos mapping excludes free quaternion');
 assert.deepEqual(joints.map(j=>j.v).slice().sort((a,b)=>a-b),Array.from({length:21},(_,i)=>i+6),'Hinge dof mapping excludes free base');
}
export function hingeChecks(beforeQ,afterQ,afterV,joints,h){
 return joints.map(j=>{const expected=add(point(beforeQ[j.q]),times(point(h),point(afterV[j.v]))),actual=afterQ[j.q];return{name:j.name,q:j.q,v:j.v,expected,actual,passed:actual>=expected[0]&&actual<=expected[1]};});
}

export function eulerDiagnostic({M,damping,qacc,h,beforeV,afterV,smooth,constraint,nativeVelocityErrorBound=null}){
 const n=qacc.length;
 if(!n||M.length!==n*n||[damping,beforeV,afterV,smooth,constraint].some(a=>a?.length!==n)||!(h>0)||!finite([...M,...damping,...qacc,h,...beforeV,...afterV,...smooth,...constraint]))return{status:'inconclusive',reason:'missing/nonfinite Euler inputs'};
 let H,rhs,force,massResidual;
 try{
  H=M.map((v,k)=>k%(n+1)===0?add(point(v),times(point(h),point(damping[k%n]))):point(v));
  rhs=matrixRows(M,n).map(row=>idot(row.map(point),qacc.map(point)));
  force=smooth.map((v,i)=>add(point(v),point(constraint[i])));
  massResidual=rhs.map((v,i)=>minus(v,force[i]));
  const effective=qrFactor(matrixRows(H,n).map(row=>row.map(midpoint))).solve(rhs.map(midpoint));
  const reconstruction=linearCertificate(H,rhs,effective);
  const forceEquation=linearCertificate(H,force,effective);
  const reconstructionResidual=matrixRows(H,n).map((row,i)=>minus(idot(row,effective.map(point)),rhs[i]));
  const checks=effective.map((a,k)=>{
   const ideal=add(point(beforeV[k]),times(point(h),point(a))),expected=beforeV[k]+h*a,error=Math.abs(expected-afterV[k]);
   // Bound arithmetic using interval endpoints; retain legacy error for diagnosis only.
   const arithmeticUpper=nextUp(Math.max(Math.abs(expected-ideal[0]),Math.abs(ideal[1]-expected)));
   const reconstructionVelocityErrorUpper=reconstruction.status==='certified'?nextUp(h*reconstruction.errorUpper):null;
   const forceEquationVelocityErrorUpper=forceEquation.status==='certified'?nextUp(h*forceEquation.errorUpper):null;
   let allowed=null,verdict='inconclusive';
   // An independent native solve-error bound is REQUIRED for an identity verdict.
   // Never derive it from the observed discrepancy or use a user-chosen tolerance.
   if(forceEquationVelocityErrorUpper!==null&&nativeVelocityErrorBound?.length===n&&finite(nativeVelocityErrorBound)&&nativeVelocityErrorBound.every(x=>x>=0)){
    allowed=nextUp(nextUp(forceEquationVelocityErrorUpper+arithmeticUpper)+nativeVelocityErrorBound[k]);
    const delta=minus(point(expected),point(afterV[k])),lower=delta[0]>0?delta[0]:delta[1]<0?-delta[1]:0;
    const upper=upperAbs(delta);
    verdict=lower>allowed?'demonstrated-anomaly':upper<=allowed?'certified':'inconclusive';
   }
   return{k,expected,actual:afterV[k],error,legacy128EpsilonBound:128*Number.EPSILON*Math.max(1,Math.abs(expected),Math.abs(afterV[k])),arithmeticUpper,reconstructionVelocityErrorUpper,forceEquationVelocityErrorUpper,nativeVelocityErrorBound:nativeVelocityErrorBound?.[k]??null,allowed,verdict};
  });
  const status=checks.some(c=>c.verdict==='demonstrated-anomaly')?'demonstrated-anomaly':checks.every(c=>c.verdict==='certified')?'certified':'inconclusive';
  return{status,reason:status==='inconclusive'?'No certified engine agreement: missing/invalid independent native-error bound or linear certificate. No independent bound on native implicit-solve error; reconstruction certificate alone does not certify engine agreement':undefined,assumptions:'Euler with constant linear diagonal damping; supplied M/qacc/forces must refer to the same pre-integration solve; no quaternion position identity',effective,H,rhs,force,massResidual,reconstructionResidual,reconstruction,forceEquation,checks};
 }catch(e){return{status:'inconclusive',reason:'Euler reconstruction unavailable: '+String(e),H,rhs,force,massResidual};}
}

// Thresholds are the existing physical definitions: <1 N for 20 consecutive
// substeps is lost support; a nonfoot ground contact is loaded only ABOVE .1 N.
export class SupportWindows{
 constructor(history=[]){assert(history.length<20&&finite(history)&&history.every(f=>f>=0));this.loads=history.slice();this.margin=1;this.run=history.reduce((r,f)=>f<1?r+1:0,0);this.maxRun=this.run;}
 update(load){assert(Number.isFinite(load)&&load>=0,'Nonfinite/negative native load');this.loads.push(load);if(this.loads.length>20)this.loads.shift();this.run=load<1?this.run+1:0;this.maxRun=Math.max(this.maxRun,this.run);
  const windowMargin=this.loads.length===20?Math.max(...this.loads)-1:null;
  if(windowMargin!==null)this.margin=Math.min(this.margin,windowMargin);
  return{value:this.margin,windowMargin,run:this.run,maxRun:this.maxRun,violated:this.maxRun>=20,window:this.loads.slice()};
 }
}
export function nonplantarMargin(points){
 assert(points.every(p=>Number.isFinite(p.load)&&p.load>=0));
 const maximum=Math.max(0,...points.filter(p=>![7,10].includes(p.body)).map(p=>p.load));
 return{value:(.1-maximum)/.1,maximumContactLoad:maximum,violated:maximum>.1};
}
export function romMargins(q,v,lo,hi){
 assert(finite([q,v,lo,hi])&&hi>lo);const round=128*Number.EPSILON*Math.max(1,Math.abs(q),Math.abs(v));
 return{lower:(q-lo+round)/(hi-lo),upper:(hi-q+round)/(hi-lo),raw:{q,v,lo,hi,round}};
}
export function outcome(rows,numericalStatus){
 assert(['certified','inconclusive','demonstrated-anomaly'].includes(numericalStatus));assert(rows.every(r=>Number.isFinite(r.value)));const violations=rows.filter(r=>r.value<0),physicalFeasible=!violations.length;
 return{physicalFeasible,feasible:physicalFeasible&&numericalStatus==='certified',status:numericalStatus==='demonstrated-anomaly'?'demonstrated-anomaly':!physicalFeasible?'finite-infeasible':numericalStatus==='certified'?'finite-feasible':'numerical-inconclusive',numericalStatus,violations,maxNormalizedViolation:Math.max(0,...rows.map(r=>-r.value))};
}
export function evaluationResponse(obj,steps=400){
 assert(obj.evaluated&&obj.summary.steps===steps&&obj.constraints.length===obj.labels.length&&finite(obj.constraints));
 assert(['finite-infeasible','finite-feasible','numerical-inconclusive'].includes(obj.summary.status));
 assert(!obj.summary.feasible||obj.summary.physicalFeasible&&obj.summary.numericalStatus==='certified');
 return obj; // Negative physical constraints must reach the optimizer unchanged.
}
