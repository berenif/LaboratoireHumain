// Algebra and saved-state reconstruction only: ZERO forward, ZERO step.
import fs from 'node:fs';import assert from 'node:assert/strict';import {gunzipSync} from 'node:zlib';
import load from '../web/dist/vendor/mujoco.js';import {cholSolve,dot} from './pull-constrained-ls.mjs';
import {closeArrays,stationaryCertificate,numericalMargin,reconstructArrays} from './pull-numeric-certificate.mjs';
import {nativeState} from './pull-step-restoration.mjs';
const scalar={M:[2],J:[[1]],D:[3],aref:[2],states:[1],qacc:[1],forces:[3],residual:[1e-5],h:.00125,damping:[.2],tolerance:1e-8},solve=(a,b)=>[b[0]/a[0]];
const a=stationaryCertificate(scalar,solve);assert(Math.abs(a.correction[0]+2e-6)<1e-20);assert(a.deltaVelocity[0]<0);assert(a.correctedForces[0]>0);
assert.throws(()=>stationaryCertificate({...scalar,residual:[1]},solve),/tolerance/);
assert.throws(()=>stationaryCertificate({...scalar,aref:[1+1e-8],residual:[-1e-5]},solve),/branch/);
assert.throws(()=>stationaryCertificate({...scalar,residual:[NaN]},solve));
assert.throws(()=>stationaryCertificate(scalar,(a,b)=>[-b[0]/a[0]]),/backward/);
assert.throws(()=>closeArrays([1,2],[2,1],'mapping'),/mismatch/);
assert(numericalMargin(-.1,.01,0));assert(!numericalMargin(-.1,.11,0));assert(!numericalMargin(-1e-12,1e-10,0));
const dir='web/proofs/pull-step-euler-window/',read=p=>JSON.parse(gunzipSync(fs.readFileSync(dir+p))),cp=read('final.json.gz'),raw=read('last-native.json.gz'),audit=JSON.parse(fs.readFileSync(dir+'audit.json'));
const mj=await load(),model=mj.MjModel.from_xml_string(fs.readFileSync('web/dist/humanoid.xml','utf8')),data=new mj.MjData(model);mj.mj_setState(model,data,cp.state,cp.spec);mj.mj_kinematics(model,data);mj.mj_comPos(model,data);mj.mj_makeM(model,data);
const {M,J,massCheck,forceCheck}=reconstructArrays(mj,model,data,raw);closeArrays(J.map((j,i)=>dot(j,raw.arrays.qacc)-raw.arrays.efc_aref[i]),raw.branches.map(b=>b.jar),'Saved branch Jacobian');
const result=stationaryCertificate({M,J,D:raw.arrays.efc_D,aref:raw.arrays.efc_aref,states:raw.resolved.states,qacc:raw.arrays.qacc,forces:raw.arrays.efc_force,residual:audit.dynamicsResidual,h:model.opt.timestep,damping:Array.from(model.dof_damping),tolerance:model.opt.tolerance},(a,b)=>cholSolve(mj,a,b));
assert(Math.abs(result.decrement-5.676850123495e-10)<1e-20);assert(Math.abs(result.minActiveForce-.00128404606056)<1e-12);assert(result.backwardMax<1e-15);assert(Math.max(...result.deltaVelocity.map(Math.abs))<4.255e-7);assert.deepEqual(nativeState({mj,model,data},cp.spec),cp.state);
console.log(JSON.stringify({forwardCalls:0,ticks:0,counterexamples:'mapping, large residual, branch crossing, nonfinite, insufficient margin rejected',saved233Unchanged:true,massCheck,forceCheck,savedCertificate:{decrement:result.decrement,minActiveForce:result.minActiveForce,maxVelocityCorrection:Math.max(...result.deltaVelocity.map(Math.abs)),backwardError:result.backwardMax}}));data.delete();model.delete();
