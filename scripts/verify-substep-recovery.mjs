// Saved-data/algebra only; no forward, integration or new optimization.
import fs from 'node:fs';import assert from 'node:assert/strict';import{gunzipSync}from'node:zlib';
import {SubstepReference,boundedBrake,prediction,checkPrediction,eulerMap} from './pull-substep-recovery.mjs';
const read=n=>JSON.parse(gunzipSync(fs.readFileSync('web/proofs/pull-entry-recovery-001/'+n+'.json.gz'))),h=.00125;
export function verify(){
 const cp=read('final'),raw=read('last-native'),qp=read('last-qp'),R=cp.experimental.reference.recovery,records=gunzipSync(fs.readFileSync('web/proofs/pull-entry-recovery-001/journal.ndjson.gz')).toString().trim().split('\n').map(JSON.parse).filter(r=>r.event==='physical-envelope-before-guard');let tests=0;
 const age=Math.round((cp.physics.time-R.start)/h);assert(age===2&&96-age===94);tests++;
 const brakes=R.entries.map(e=>{const q=cp.physics.qpos[e.q],v=cp.physics.qvel[e.v],r=boundedBrake(q,v,e.limit,e.peak.q,94);for(let k=1;k<=94;k++)assert(q+k*h*v-.5*r.b*h*h*k*(k+1)<=e.peak.q+1e-16);assert(q+94*h*v-.5*r.b*h*h*94*95<=e.limit&&v-94*h*r.b<=0);return r;});assert(Math.abs(brakes[0].b-.6738604225711338)<1e-13&&Math.abs(brakes[1].b-.7068670336660066)<1e-13);tests++;
 for(const v of [-.02,.02]){const limit=.03,q=.031,r=boundedBrake(q,v,limit,.04,1),vn=v-h*r.b,qn=q+h*vn;assert(qn<=limit+1e-16&&vn<=1e-14);}tests++;
 for(const N of [0,-1,97,NaN])assert.throws(()=>boundedBrake(.03,.02,.02,.04,N));assert.throws(()=>boundedBrake(NaN,0,0,1,2));tests++;
 const fake=Object.create(SubstepReference.prototype);Object.assign(fake,{phase:'recovery',closed:{},events:[],recovery:structuredClone(R),p:{data:{qpos:cp.physics.qpos.slice(),qvel:cp.physics.qvel.slice(),time:cp.physics.time},model:{jnt_range:R.entries.flatMap(e=>[-3,e.limit])},joints:R.entries.map((e,j)=>({name:e.name,j,q:e.q,v:e.v}))}});
 assert(fake.checkState().passed,'New bounded contract accepts state that failed old parabola');tests++;
 const e=R.entries[0];fake.p.data.qpos[e.q]=e.peak.q+.00001;assert(!fake.checkState().passed);fake.p.data.qpos[e.q]=cp.physics.qpos[e.q];tests++;
 fake.p.data.qpos[e.q]=e.limit-.001;fake.p.data.qvel[e.v]=-.01;fake.advance({t:fake.p.data.time},Array(21).fill(0),h);assert(fake.closed[e.name]);fake.p.data.qpos[e.q]=e.limit+.000001;assert(!fake.checkState().passed,'Closed exception never reopens');tests++;
 const q1=R.entries[1];fake.p.data.qpos[q1.q]=q1.limit-.001;fake.p.data.qvel[q1.v]=-.01;fake.p.data.qpos[e.q]=e.limit-.001;fake.advance({t:fake.p.data.time},Array(21).fill(0),h);assert(fake.phase==='transfer'&&fake.recoveryDone&&fake.recovery.start===R.start&&fake.start===cp.physics.time);tests++;
 const mapped=eulerMap(qp.T,raw.qacc),mappingError=Math.max(...mapped.map((v,i)=>Math.abs(v-raw.euler[i])));assert(mappingError<128*Number.EPSILON*Math.max(...raw.qacc.map(Math.abs)));const wrong=qp.T.map(col=>col.reduce((s,x,i)=>s+x*raw.qacc[i],0));assert(Math.max(...wrong.map((v,i)=>Math.abs(v-raw.euler[i])))>.001);tests++;
 const js=R.entries.map(e=>({name:e.name,q:e.q,v:e.v})),pred=prediction(raw,Array(27).fill(1e-12),js),fakeData={qpos:pred.q.slice(),qvel:pred.v.slice()};for(const j of pred.hinges)fakeData.qpos[j.q]=j.next;assert(checkPrediction(pred,fakeData,Array(27).fill(1e-12)).passed);fakeData.qvel[12]+=.00001;assert(!checkPrediction(pred,fakeData,Array(27).fill(1e-12)).passed);tests++;
 for(const e of R.entries){assert(Math.abs(records[0].knees.find(x=>x.name===e.name).v-pred.v[e.v])<1e-14);const stale=records[0].knees.find(x=>x.name===e.name).v+h*raw.euler[e.v];assert(Math.abs(records[1].knees.find(x=>x.name===e.name).v-stale)>1e-4);}tests++;
 const sameInput=(a,b)=>assert.deepEqual(a,b),before={state:cp.state,force:cp.experimental.pendingInput.force,control:cp.experimental.previousControls};sameInput(before,structuredClone(before));const changed=structuredClone(before);changed.force[0]+=.01;assert.throws(()=>sameInput(before,changed));changed.force[0]-=.01;changed.control[0]+=.01;assert.throws(()=>sameInput(before,changed));tests++;
 return {passed:true,tests,zeroForwards:true,zeroTicks:true,age,remaining:94,brakes,mappingError,oldParabolaFAILPreserved:true};
}
if(process.argv.includes('--algebra'))console.log(JSON.stringify(verify()));
