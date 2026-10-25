// Deterministic algebra only; no MuJoCo load, model, forward or integration.
import assert from 'node:assert/strict';
import {qrFactor,constrainedDirectLS,directCertificate} from './pull-direct-qr.mjs';
import {dot} from './pull-constrained-ls.mjs';
let cases=0;const close=(a,b,tol=1e-8)=>assert(Math.max(...a.map((v,i)=>Math.abs(v-b[i])))<tol,JSON.stringify({a,b}));
const C=[[2,1],[0,3],[1,-2]],f=qrFactor(C),Q=Array.from({length:3},(_,j)=>f.applyQ([0,0,0].map((_,i)=>i===j?1:0)));
for(let j=0;j<2;j++)close(C.map(r=>r[f.permutation[j]]),Array.from({length:3},(_,i)=>dot(Q.map(c=>c[i]),f.R.map(r=>r[j]))));
for(let j=0;j<3;j++)for(let i=0;i<3;i++)assert(Math.abs(dot(Q[i],Q[j])-(i===j?1:0))<1e-14);cases++;
function check(C,d,rows,expected){const r=constrainedDirectLS(C,d,rows);assert(r.solved,JSON.stringify(r));close(r.x,expected);assert(r.pivots<=64&&r.backwardError<=1e-10);cases++;return r;}
check(C,C.map(r=>dot(r,[.3,-.2])),[],[.3,-.2]);
check([[1,0],[0,1]],[2,-3],[{a:[1,0],bound:.5,label:'upper'},{a:[0,-1],bound:1,label:'lower'}],[.5,-1]);
check([[1,0],[0,1]],[0,0],[{a:[-1,0],bound:-1,label:'initially-infeasible'}],[1,0]);
check([[1,0],[0,1]],[2,2],[{a:[1,1],bound:1,label:'sum'}],[.5,.5]);
check([[1]],[2],[{a:[1],bound:1,label:'loose'},{a:[2],bound:1,label:'strict'}],[.5]);
const impossible=constrainedDirectLS([[1]],[0],[{a:[1],bound:0,label:'upper'},{a:[-1],bound:-1,label:'lower'}]);assert(!impossible.solved);cases++;
const capped=constrainedDirectLS([[1]],[2],[{a:[1],bound:1,label:'cap'}],{maxPivots:0});assert(!capped.solved&&capped.pivots===0);cases++;
// Independent 2D edge/vertex enumeration for coupled objective C'C=[2,.3;.3,1].
for(let k=0;k<12;k++){
 const C=[[Math.sqrt(2),.3/Math.sqrt(2)],[0,Math.sqrt(.955)]],d=[3*Math.sin(k*2.1),3*Math.cos(k*1.7)],b=[Math.sqrt(2)*d[0],.3/Math.sqrt(2)*d[0]+Math.sqrt(.955)*d[1]],rows=[{a:[1,0],bound:.5,label:'x+'},{a:[-1,0],bound:.7,label:'x-'},{a:[0,1],bound:.8,label:'y+'},{a:[0,-1],bound:.4,label:'y-'},{a:[1,1],bound:.6,label:'sum'}],inv=v=>[(v[0]-.3*v[1])/1.91,(-.3*v[0]+2*v[1])/1.91],free=inv(b),points=[free];
 for(const row of rows){const z=inv(row.a),l=(dot(row.a,free)-row.bound)/dot(row.a,z);if(l>=0)points.push(free.map((v,i)=>v-l*z[i]));}
 for(let i=0;i<rows.length;i++)for(let j=i+1;j<rows.length;j++){const a=rows[i],z=rows[j],det=a.a[0]*z.a[1]-a.a[1]*z.a[0];if(Math.abs(det)>1e-12)points.push([(a.bound*z.a[1]-a.a[1]*z.bound)/det,(a.a[0]*z.bound-a.bound*z.a[0])/det]);}
 const cost=x=>C.reduce((s,r,i)=>s+(dot(r,x)-d[i])**2,0),feasible=points.filter(x=>rows.every(r=>dot(r.a,x)<=r.bound+1e-10)).sort((a,b)=>cost(a)-cost(b));check(C,d,rows,feasible[0]);
}
const n=45,star=Array.from({length:n},(_,i)=>.2*Math.sin(i+.2)),diagonal=Array.from({length:n},(_,i)=>10**(i%9-4)),design=diagonal.map((v,i)=>Array.from({length:n},(_,j)=>j===i?v:0)),target=diagonal.map((v,i)=>v*star[i]);
check(design,target,[],star);
const face=Array.from({length:8},(_,i)=>({a:Array.from({length:n},(_,j)=>i===j?1:0),bound:star[i]-.1,label:'known-'+i})),expected=star.map((v,i)=>i<8?v-.1:v);check(design,target,face,expected);
// C scales by sqrt(costScale): objective scales1e-6 and1e6.
for(const scale of[1e-3,1e3])check(design.map(r=>r.map(v=>v*scale)),target.map(v=>v*scale),face,expected);
const extreme=constrainedDirectLS(design.map(r=>r.map(v=>v*1e6)),target.map(v=>v*1e6),face);assert(!extreme.solved&&extreme.reason.includes('complementarity'),'Absolute complementarity must conservatively reject unqualified huge-scale roundoff, not relax its threshold');cases++;
const units=Array.from({length:n},(_,i)=>10**(i%7-3));check(design.map(r=>r.map((v,i)=>v*units[i])),target,face.map(r=>({...r,a:r.a.map((v,i)=>v*units[i])})),expected.map((v,i)=>v/units[i]));
assert(!constrainedDirectLS([[1,2],[2,4]],[0,0],[]).solved);assert(!constrainedDirectLS([[NaN]],[0],[]).solved);assert(!constrainedDirectLS([[1]],[0],[{a:[0],bound:-1,label:'zero'}]).solved);cases+=3;
assert(directCertificate([[1]],[0],[],[0],[]).passed);assert(!directCertificate([[1e6]],[1e6],[{a:[1],bound:1,label:'motor'}],[1.001],[0]).passed);cases+=2;
check([[1]],[2],[{a:[1],bound:1,label:'bound'},{a:[2],bound:2,label:'duplicate'}],[1]);
console.log(JSON.stringify({passed:true,algebraCases:cases,ticks:0,nativeForwards:0,impossible:impossible.reason}));
