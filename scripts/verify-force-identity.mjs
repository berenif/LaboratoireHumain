// Pure counterexamples; do not load MuJoCo or integrate a state.
import assert from 'node:assert/strict';import fs from 'node:fs';import{gunzipSync}from'node:zlib';
import {Physics,cross,sub} from '../web/dist/engine.js';import {accumulateInteraction}from'./pull-external-force.mjs';
export function verify(){
 const cp=JSON.parse(gunzipSync(fs.readFileSync('web/proofs/pull-substep-recovery-001/final-with-pending.json.gz'))),pending=cp.experimental.applicationStatus.pendingValidatedInput,body=15;
 const make=()=>({xipos:new Float64Array(48),xfrc_applied:new Float64Array(96)}),same=(a,b)=>assert.deepEqual(Array.from(a),Array.from(b));let cases=0;
 for(const int of [pending,{point:[1,-2,3],bodyCOM:[.4,-.5,.6],force:[-1,2,-3]},{point:[0,0,0],bodyCOM:[0,0,0],force:[-0,0,-1]},{point:[1,1,1],bodyCOM:[1,1,1],force:[0,-0,1]}]){
  const copy=make(),live=make();live.xipos.set(int.bodyCOM,45);accumulateInteraction(copy,body,int);Physics.prototype.apply.call({data:live},body,int.point,int.force);same(copy.xfrc_applied,live.xfrc_applied);cases++;
 }
 const torque=cross(sub(pending.point,pending.bodyCOM),pending.force);assert(Object.is(torque[0],-0));const direct=make();direct.xfrc_applied.set([...pending.force,...torque],90);const accumulated=make();accumulateInteraction(accumulated,body,pending);assert.throws(()=>same(direct.xfrc_applied,accumulated.xfrc_applied));assert(Object.is(accumulated.xfrc_applied[93],0));
 const altered=Float64Array.from(accumulated.xfrc_applied);altered[90]+=1e-12;assert.throws(()=>same(altered,accumulated.xfrc_applied));altered.set(accumulated.xfrc_applied);altered[93]=Number.MIN_VALUE;assert.throws(()=>same(altered,accumulated.xfrc_applied));
 const status=cp.experimental.applicationStatus;assert(status.lastIntegratedStep===8232&&status.recoveryTicksRemaining===59&&status.globalTicksRemaining===2603&&status.nextTickWasNOTIntegrated);assert.deepEqual(status.pendingValidatedControls,cp.experimental.previousControls);assert.deepEqual(status.pendingValidatedInput,cp.experimental.pendingInput);
 return {passed:true,tests:8,forceCases:cases,identityComparison:'unchanged assert.deepEqual, bit-exact including signed zero',nonzeroDifferenceRejected:true,smallestPositiveDifferenceRejected:true,pendingCandidateExact:true,remainingRecoveryTicks:59,globalTicksRemaining:2603,forwards:0,ticks:0};
}
if(process.argv.includes('--algebra'))console.log(JSON.stringify(verify()));
