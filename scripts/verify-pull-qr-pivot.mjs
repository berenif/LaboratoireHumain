// Algebra only. The blocker is selected before floating-point interpolation.
import assert from 'node:assert/strict';import {dualBlocker,constrainedDirectLS} from './pull-direct-qr.mjs';
let tests=0;
const a=.22059650160372257,b=.6278999091591686,f=a/(a+b),positive={a,b,f,r:a+f*(-b-a)};
assert(positive);{const {a,b,r}=positive,s=dualBlocker([0],[a],[-b]);assert(s.blocker===0&&r>0);tests++;}
for(const active of [[2,0],[0,2]]){const s=dualBlocker(active,[1,0,1],[-1,-1]);assert(s.blocker===0&&s.fraction===.5);tests++;}
assert.deepEqual(dualBlocker([0],[0],[-1]).fraction,0);tests++;
const zero=constrainedDirectLS([[1]],[1],[{a:[1],bound:1,label:'zero-multiplier'}]);assert(zero.solved&&zero.lambda[0]===0);tests++;
// This small QP needs a dual drop, and the cap deliberately stops after that segment.
const rows=[{a:[1,0],bound:.5,label:'x'},{a:[-1,0],bound:.7,label:'-x'},{a:[0,1],bound:.8,label:'y'},{a:[0,-1],bound:.4,label:'-y'},{a:[1,1],bound:.6,label:'sum'}];
const C2=[[2,.3],[.1,.25]],d2=[2.589628099946621,-.3865334828865739];
const stopped=constrainedDirectLS(C2,d2,rows,{maxPivots:2});
assert(!stopped.solved&&stopped.lastSegment);
assert(stopped);for(let i=0;i<2;i++)assert.equal(stopped.x[i],stopped.scaling[i]*(stopped.lastSegment.oldZ[i]+stopped.lastSegment.fraction*(stopped.lastSegment.trialZ[i]-stopped.lastSegment.oldZ[i])));tests++;
console.log(JSON.stringify({passed:true,tests,positiveInterpolationResidual:positive.r,ticks:0,nativeForwards:0}));
