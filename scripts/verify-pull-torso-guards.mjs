// Pure algebra and saved data; no engine import or native evaluation.
import assert from 'node:assert/strict';import fs from 'node:fs';
import{torsoKinematics,axisResiduals,robustNonpositiveLimit,robustNonpositiveRows,recenteredControlBound}from'./pull-torso-guards.mjs';
let tests=0;const near=(a,b,t=1e-12)=>assert(Math.abs(a-b)<=t);
const R=[Math.cos(.2),0,Math.sin(.2),0,1,0,-Math.sin(.2),0,Math.cos(.2)];
for(const [w,a]of[[[.1,.2,.3],[1,-2,3]],[[-.4,.2,-.7],[-2,4,9]],[[0,0,1],[3,5,10]]]){
 const t=torsoKinematics(R,w,a);near(t.orientationSpeedEnergyDerivative,w[0]*a[0]+w[1]*a[1]);near(t.up.reduce((s,v,i)=>s+v*t.upDot[i],0),0);near(t.up.reduce((s,v,i)=>s+v*t.upDD[i],0),-t.upDot.reduce((s,v)=>s+v*v,0));tests++;
}
// Finite differences of R*exp([omega]t), purely analytic, no physical state.
const w=[.1,.2,-.3],norm=Math.hypot(...w),k=w.map(x=>x/norm),ez=[0,0,1],cross=[k[1],-k[0],0];
const up=t=>{const v=ez.map((v,i)=>v*Math.cos(norm*t)+cross[i]*Math.sin(norm*t)+k[i]*k[2]*(1-Math.cos(norm*t)));return[0,1,2].map(i=>R.slice(3*i,3*i+3).reduce((s,x,j)=>s+x*v[j],0));};
const dt=1e-4,t=torsoKinematics(R,w,[0,0,0]),plus=up(dt),minus=up(-dt),zero=up(0);
for(let i=0;i<3;i++){near((plus[i]-minus[i])/(2*dt),t.upDot[i],1e-9);near((plus[i]-2*zero[i]+minus[i])/(dt*dt),t.upDD[i],3e-8);}tests++;
for(const w of[[.1,-.2,0],[-.1,.2,-.3]]){
 const h=.00125,brake=w.map(x=>-x/h);assert(axisResiduals(w,brake,h).every(x=>x.value<=0));assert(axisResiduals(w,w.map(x=>-2*x/h),h).every(x=>x.value<=1e-15));assert(axisResiduals(w,w.map(x=>-3*x/h),h).some(x=>x.value>0));tests++;
}
assert(axisResiduals([0,0,0],[1,0,0],.00125).some(x=>x.value>0));tests++;
// Scalar cancellation passes yet transverse and axial acceleration must fail.
const cancellation=torsoKinematics(R,[1,1,1],[1,-2,3]);assert(cancellation.orientationSpeedEnergyDerivative<0);assert(axisResiduals([1,1,1],[1,-2,3],.00125).some(x=>x.value>0));tests++;
const center=.0002857760814277035,r=.2,tol=1e-10,limit=robustNonpositiveLimit(center,r,tol);
near(limit,-center/4);assert(limit+Math.max(tol,r*Math.abs(limit-center))<=1e-18);assert(limit/2+r*Math.abs(limit/2-center)>0);tests++;
assert.throws(()=>robustNonpositiveLimit(center,1,tol));assert.throws(()=>robustNonpositiveLimit(NaN,r,tol));tests++;
for(const c of[-center,0,center]){
 const rows=robustNonpositiveRows(c,r,tol),limit=robustNonpositiveLimit(c,r,tol);
 for(const gp of[-.01,-center,-tol,0,center,.01,limit-1e-12,limit+1e-12]){
  const exact=gp+Math.max(tol,r*Math.abs(gp-c))<=0;
  assert.equal(rows.every(row=>row.factor*gp<=row.bound),exact);
  assert.equal(gp<=limit,exact);
 }
 tests++;
}
// Recentring cannot create a second .1 of authority in the same direction.
near(recenteredControlBound(.2,.3,.1,1),0);near(recenteredControlBound(.2,.3,.1,-1),.2);near(recenteredControlBound(.95,.99,.1,1),.01);tests++;
const saved=JSON.parse(fs.readFileSync('web/proofs/pull-step-constrained-recheck/result.json'));
assert(axisResiduals([.002344881537359025,.04398065031674101,-.12027056274652875],saved.candidate.qacc.slice(3,6),.00125).filter(x=>x.value>1e-10).length===2);tests++;
console.log(JSON.stringify({algebraTests:tests,modelsCreated:0,nativeEvaluations:0,ticks:0,forecasts:0,robustScalarLimit:limit}));
