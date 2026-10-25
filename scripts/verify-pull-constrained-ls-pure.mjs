// Algebra only: loads Cholesky, creates no MuJoCo model/data, calls no dynamics.
import assert from 'node:assert/strict';import load from '../web/dist/vendor/mujoco.js';
import {constrainedLS,dot} from './pull-constrained-ls.mjs';
const mj=await load();let cases=0;const close=(a,b)=>assert(Math.hypot(...a.map((v,i)=>v-b[i]))<1e-7);
function check(H,b,rows,expected){const r=constrainedLS(mj,H,b,rows);assert(r.solved,JSON.stringify(r));close(r.x,expected);assert(rows.every(z=>dot(z.a,r.x)<=z.bound+1e-8));assert(r.pivots<=64);cases++;return r;}
check([2,0,0,3],[4,-9],[],[2,-3]);
check([1,0,0,1],[2,-3],[{a:[1,0],bound:.5,label:'upper'},{a:[0,-1],bound:1,label:'lower'}],[.5,-1]);
check([1,0,0,1],[0,0],[{a:[-1,0],bound:-1,label:'initially-infeasible'}],[1,0]);
check([1,0,0,1],[2,2],[{a:[1,1],bound:1,label:'coupled'},{a:[-1,0],bound:0,label:'x-positive'}],[.5,.5]);
check([1],[2],[{a:[1],bound:1,label:'loose'},{a:[2],bound:1,label:'strict'}],[.5]);
const impossible=constrainedLS(mj,[1],[0],[{a:[1],bound:0,label:'upper'},{a:[-1],bound:-1,label:'lower'}]);assert(!impossible.solved);assert(impossible.pivots<=64);cases++;
const capped=constrainedLS(mj,[1],[2],[{a:[1],bound:1,label:'bound'}],{maxPivots:0});assert(!capped.solved);assert.equal(capped.pivots,0);cases++;
// Independent 2D active-face enumeration, fixed inputs; no tuning/physics.
for(let k=0;k<12;k++){
 const b=[Math.sin(k*2.1)*3,Math.cos(k*1.7)*3],H=[2,.3,.3,1],rows=[{a:[1,0],bound:.5,label:'x+'},{a:[-1,0],bound:.7,label:'x-'},{a:[0,1],bound:.8,label:'y+'},{a:[0,-1],bound:.4,label:'y-'},{a:[1,1],bound:.6,label:'sum'}];
 const inv=v=>[(v[0]-.3*v[1])/1.91,(-.3*v[0]+2*v[1])/1.91],free=inv(b),candidates=[free];
 for(const row of rows){const z=inv(row.a),l=(dot(row.a,free)-row.bound)/dot(row.a,z);if(l>=0)candidates.push(free.map((x,i)=>x-l*z[i]));}
 for(let i=0;i<rows.length;i++)for(let j=i+1;j<rows.length;j++){const a=rows[i],d=rows[j],det=a.a[0]*d.a[1]-a.a[1]*d.a[0];if(Math.abs(det)>1e-12)candidates.push([(a.bound*d.a[1]-a.a[1]*d.bound)/det,(a.a[0]*d.bound-a.bound*d.a[0])/det]);}
 const cost=x=>.5*(2*x[0]*x[0]+.6*x[0]*x[1]+x[1]*x[1])-dot(b,x),feasible=candidates.filter(x=>rows.every(r=>dot(r.a,x)<=r.bound+1e-10)).sort((a,b)=>cost(a)-cost(b));
 check(H,b,rows,feasible[0]);
}
console.log(JSON.stringify({algebraTestsPassed:cases,dynamicsEvaluations:0,integrations:0,forecasts:0,impossibleHandling:impossible.reason}));
// A 45-variable manufactured KKT optimum: no reference solver shares the adapter.
const n=45,star=Array.from({length:n},(_,i)=>.2*Math.sin(i+.2)),H=Array(n*n).fill(0);
for(let i=0;i<n;i+=2){const a=10**((i%9)-4),d=10**(((i+4)%9)-4);if(i===n-1){H[i*n+i]=a;break;}H[i*n+i]=.36*a+.64*d;H[(i+1)*n+i+1]=.64*a+.36*d;H[i*n+i+1]=H[(i+1)*n+i]=.48*(a-d);}
const face=Array.from({length:8},(_,i)=>({a:Array.from({length:n},(_,j)=>j===i?(i%2?1:-1):0),bound:(i%2?1:-1)*star[i],label:'known-'+i}));
const rhs=Array.from({length:n},(_,i)=>dot(H.slice(i*n,(i+1)*n),star)+face.reduce((s,r,k)=>s+r.a[i]*(.1+.01*k),0));
const original=check(H,rhs,face,star);assert(original.backwardError<=1e-10);assert.equal(original.hessianFactorizations,1);assert(original.refinements.length<=3);
for(const costScale of[1e-6,1e6])check(H.map(v=>v*costScale),rhs.map(v=>v*costScale),face,star);
const units=Array.from({length:n},(_,i)=>10**((i%7)-3));
check(H.map((v,k)=>v*units[Math.floor(k/n)]*units[k%n]),rhs.map((v,i)=>v*units[i]),face.map(r=>({...r,a:r.a.map((v,i)=>v*units[i])})),star.map((v,i)=>v/units[i]));
assert(!constrainedLS(mj,[NaN],[0],[]).solved);assert(!constrainedLS(mj,[0],[0],[]).solved);assert(!constrainedLS(mj,[1],[0],[{a:[0],bound:-1,label:'zero'}]).solved);cases+=3;
const {certificate}=await import('./pull-constrained-ls.mjs');
assert(certificate([1],[0],[],[0],[]).passed,'Zero denominator has exact zero residual');
assert(!certificate([1e12],[1e12],[{a:[1],bound:1,label:'motor-cap'}],[1.001],[0]).passed,'Large objective cannot hide a real motor-cap violation');
assert(constrainedLS(mj,[1],[2],[{a:[1],bound:1,label:'face'},{a:[2],bound:2,label:'same-face'}]).solved,'Redundant inactive row does not imply impossibility');cases+=3;
console.log(JSON.stringify({extendedAlgebraTestsPassed:cases,nativeForwards:0,ticks:0,manufactured45:original,interpretation:'Backward error and primal constraints, not a bound on physical contact realization'}));
// Inject linear-solve error only in algebra, to exercise refinement and its stop.
function faulty(always){let calls=0;return {DoubleBuffer:mj.DoubleBuffer,mju_cholFactor:(...a)=>mj.mju_cholFactor(...a),mju_cholSolve(out,...args){mj.mju_cholSolve(out,...args);if(always||calls++===0)out.GetView()[0]+=.01;}};}
const refined=constrainedLS(faulty(false),[1],[2],[]);assert(refined.solved);assert.equal(refined.refinements.length,1);close(refined.x,[2]);
const stagnant=constrainedLS(faulty(true),[1],[2],[]);assert(!stagnant.solved);assert(stagnant.reason.includes('without improvement'));assert(stagnant.refinements.length<=3);
assert(!constrainedLS(mj,[1,2,2,1],[0,0],[]).solved,'Indefinite Hessian must not be regularized into acceptance');
console.log(JSON.stringify({additionalFaultTestsPassed:3,refinement:refined.refinements,stagnation:stagnant.reason,nativeForwards:0,ticks:0}));
