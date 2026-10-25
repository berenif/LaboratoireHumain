// Native prediction helpers. No controller is installed in the application.
import assert from 'node:assert/strict';
import {smooth} from '../web/mujoco/pull-step.js';
import {sub,mul,norm,inv} from './pull-stance-geometry.mjs';
export const PLAN=Object.freeze({seconds:.5,dt:.00125,steps:400,knotSeconds:[0,.25,.5],variables:42,maxEvaluations:128,maxPredictiveSteps:51200,maxWallSeconds:900,maxBytes:500000,maxGrowth:1530000000,confirmedBefore:229565,unknownHistoryPreserved:true,objective:'mean over400 substeps and21 actuators of squared normalized motor command; no additional cost or weight',initialization:'both free command nodes repeat last actually applied command',interpolation:'convex quintic smoothstep, C2 command at fixed0/.25/.5s knots; not a joint-position trajectory',constraints:'native path ROM/caps/20deg/25mm/support loss25ms/nonplantar/fall; terminal world AND gravity-heading relative progress5mm, both lateral velocities<=0, tilt<=entry, torso omega norm<=.2',predictor120ms:'historical diagnostic only; candidate actual path checked at every native substep',forces:'unchanged material-point grab law updated once per native step, no root actuation or reaction injection',forecastPolicy:'all400 substeps on a disposable copy, including infeasible paths, unless demonstrated anomaly, evaluator failure or compute budget; numerical identity inconclusiveness is recorded separately; path violations reject the entire prediction, never applied live'});
export function command(u0,x,t){
 assert(u0.length===21&&x.length===42&&[...u0,...x].every(v=>Number.isFinite(v)&&Math.abs(v)<=1));
 assert(t>=0&&t<=.5);const second=t>.25,a=second?x.slice(0,21):u0,b=second?x.slice(21):x.slice(0,21),s=smooth((t-(second?.25:0))/.25);
 return a.map((v,i)=>(1-s)*v+s*b[i]);
}
export function grabInput(p,cp){
 const g=p.grab;g.target=[cp.experimental.grabStart[0]+.25*smooth((p.data.time-10)/.5),...cp.experimental.grabStart.slice(1)];
 const point=p.worldPoint(15,g.local);assert(g.previous?.length===3,'Complete grab history required');
 const velocity=mul(sub(point,g.previous),800);let force=point.map((x,i)=>120*(g.target[i]-x)-12*velocity[i]);
 if(norm(force)>180)force=mul(force,180/norm(force));const blend=smooth((p.steps-g.bridge.startStep)/20);force=force.map((x,i)=>(1-blend)*g.bridge.from[i]+blend*x);
 return {point,force,velocity,target:g.target.slice(),blend,bodyCOM:Array.from(p.data.xipos.slice(45,48))};
}
export function terminal(initial,last,Rg){
 const worldGain=initial.com[1]-last.com[1],relativeGain=initial.relative[1]-last.relative[1],omega=norm(last.omega);
 const rows=[['world progress5mm',(worldGain-.005)/.005],['relative progress5mm',(relativeGain-.005)/.005],['world lateral velocity reversed',-last.velocity[1]/.05],['relative lateral velocity reversed',-last.relativeVelocity[1]/.05],['torso tilt no greater than entry',(initial.tilt-last.tilt)/(20*Math.PI/180)],['torso angular speed<=.2',(.2-omega)/.2]];
 return {worldGain,relativeGain,worldLateralVelocity:last.velocity[1],relativeLateralVelocity:last.relativeVelocity[1],tilt:last.tilt,entryTilt:initial.tilt,torsoAngularSpeed:omega,rows,passed:rows.every(([,g])=>g>=0)};
}
// Legacy arithmetic-only diagnostic; never an engine-consistency certificate.
export function velocityIdentity(before,actual,effective){
 const arithmetic=128*Number.EPSILON;
 return effective.map((a,k)=>{const expected=before[k]+PLAN.dt*a,err=Math.abs(expected-actual[k]),bound=arithmetic*Math.max(1,Math.abs(expected),Math.abs(actual[k]));return {k,expected,actual:actual[k],err,bound};});
}
export function tests(){
 const u0=Array(21).fill(.2),x=[...Array(21).fill(-1),...Array(21).fill(1)];
 assert.deepEqual(command(u0,x,0),u0);assert.deepEqual(command(u0,x,.25),x.slice(0,21));assert.deepEqual(command(u0,x,.5),x.slice(21));
 for(let n=0;n<=400;n++)assert(command(u0,x,n/800).every(v=>Math.abs(v)<=1));
 assert.throws(()=>command(u0,[1.00000000001,...x.slice(1)],.1));assert.throws(()=>command(u0,x,-.01));
 // A common translation is excluded from useful relative progress.
 const R=[1,0,0,0,1,0,0,0,1],a={com:[0,.1,1],relative:[0,.1,1],velocity:[0,0,0],relativeVelocity:[0,0,0],tilt:.1,omega:[0,0,0]},b={...a,com:[0,.09,1]};
 assert(!terminal(a,b,R).passed);assert(terminal(a,{...b,relative:[0,.09,1]},R).passed);
 assert(!terminal(a,{...b,relative:[0,.09,1],omega:[0,0,.20001]},R).passed);
 const cache=new Map(),key=x=>Buffer.from(new Float64Array(x).buffer).toString('hex');let evaluations=0,steps=0;
 for(const q of [[0],[0],[Number.MIN_VALUE]])if(!cache.has(key(q))){assert(evaluations<2);cache.set(key(q),++evaluations);for(let i=0;i<400;i++){assert(steps<800);steps++;}}
 assert.equal(evaluations,2);assert.equal(steps,800);assert.throws(()=>{assert(evaluations<2);});
 const exact=1+PLAN.dt*.5;assert(velocityIdentity([1],[exact],[.5]).every(c=>c.err<=c.bound));const bad=velocityIdentity([1,1],[exact,exact+1e-9],[.5,.5]);assert.equal(bad.length,2);assert(bad[1].err>bad[1].bound&&bad[0].err<=bad[0].bound);
 return {checks:12,passed:true,nativeSteps:0,nativeForwards:0,syntheticCountersOnly:true};
}
