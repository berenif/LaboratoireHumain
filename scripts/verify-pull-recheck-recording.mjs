// Saved-data checks only: no MuJoCo import, optimizer, forward call or tick.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {gunzipSync} from 'node:zlib';
const root='web/proofs/', dir=root+'pull-step-constrained-recheck/';
const read=p=>JSON.parse(fs.readFileSync(p));
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const valueHash=x=>crypto.createHash('sha256').update(JSON.stringify(x)).digest('hex');
const previous=read(root+'pull-step-constrained-pure/result.json');
const baseline=read(root+'pull-step-coupled-pure/result.json');
const result=read(dir+'result.json'), raw=read(dir+'candidate-native.json'), input=read(dir+'candidate-input.json');
const checkpoint=JSON.parse(gunzipSync(fs.readFileSync(root+'pull-step-001/final.json.gz')));
assert.equal(sha(root+'pull-step-constrained-pure/result.json'),'16d1cd01736ccf4f0ac15d88744c62c46e5db16b20c57759781ed7f030fe2793');
assert.equal(sha(root+'pull-step-constrained-pure/evaluations.ndjson'),'27f534dd3c9f40e5699b40d384f71a387439f5cb1ee8eb6f63096db00e9e5328');
assert.equal(sha(root+'pull-step-001/final.json.gz'),baseline.inputHash);
assert.equal(checkpoint.state.length,233);
assert.equal(input.stateValues,233);
assert.equal(input.stateSHA256,valueHash(checkpoint.state));
assert.deepEqual(input.controls,baseline.baseline.controls.map((v,i)=>v+previous.solve.x[i]));
assert.equal(input.controlsSHA256,valueHash(input.controls));
assert.deepEqual(raw.controls,input.controls);
assert.deepEqual(raw.arrays.qpos,checkpoint.physics.qpos);
assert.deepEqual(raw.arrays.qvel,checkpoint.physics.qvel);
assert.equal(raw.time,checkpoint.physics.time);
assert.equal(result.counts.attempted,1); assert.equal(result.counts.completed,1);
assert.equal(result.optimizationCalls,0); assert.equal(result.solveMs,0);
assert.equal(result.counts.integrations,0); assert.equal(result.counts.forecasts,0);
const journal=fs.readFileSync(dir+'evaluations.ndjson','utf8').trim().split('\n').map(JSON.parse);
assert.deepEqual(journal.map(x=>[x.event,x.evaluation,x.label]),[['begin',1,'candidate'],['end',1,'candidate']]);
assert.equal(raw.event,'native-before-guards');
for(const key of ['efc_force','efc_type','efc_id','efc_state','efc_pos','efc_vel','efc_aref','efc_b'])assert.equal(raw.arrays[key].length,34);
assert.equal(raw.contacts.length,8); assert.equal(raw.arrays.qacc.length,27);
assert.deepEqual(raw.arrays.qacc,result.candidate.qacc);
assert.deepEqual(result.candidate.regimeChanges,[{row:0,type:3,id:7,before:1,after:0}]);
for(let i=2;i<34;i++)assert.equal(raw.arrays.efc_state[i],baseline.baseline.efcState[i]);
const conditions=result.nativeConstraints;
assert.equal(conditions.length,92); assert.equal(new Set(conditions.map(c=>c.label)).size,92);
assert.deepEqual(conditions.filter(c=>!c.pass).map(c=>c.label),['torso-orientation-speed-energy-rate']);
for(const c of conditions)assert.equal(c.pass,c.value<=c.tolerance);
// Independently reconstruct contact facets, normal loads and geometric jets.
const check=(label,value)=>assert(Math.abs(conditions.find(c=>c.label===label).value-value)<1e-11,label);
let right=0;
for(const c of raw.contacts){
 const [normal,t1,t2]=c.forceLocal;
 if(c.body===7)right+=normal;
 check(`contact-${c.index}-negative-normal`,-normal);
 for(const a of[-1,1])for(const b of[-1,1])check(`contact-${c.index}-friction-${a}-${b}`,a*t1/c.friction[0]+b*t2/c.friction[1]-normal);
}
check('right-load-shortfall',100-right);
for(const g of previous.geometryRows){
 const acc=g.bias+g.jac.reduce((sum,j,i)=>sum+j*raw.arrays.qacc[i],0);
 check(g.label,g.gap+.00125*g.velocity+.5*.00125**2*acc);
}
for(let i=0;i<21;i++)for(const sign of[-1,1])check(`motor-${i}-${sign}`,sign*(input.controls[i]-baseline.baseline.controls[i])-Math.min(.1,1-sign*baseline.baseline.controls[i]));
const t=result.candidate.torso;
check('torso-orientation-speed-energy-rate',t.upDot.reduce((sum,v,i)=>sum+v*t.upDD[i],0));
assert.equal(result.readiness,false); assert.equal(result.nativeInequalitiesPass,false);
assert.equal(result.localPredictionPass,true);
assert.equal(result.candidate.groundContactRegimeUnchanged,true);
assert(raw.arrays.qfrc_actuator.slice(0,6).every(x=>x===0));
for(const [i,j] of baseline.joints.entries()){
 const effort=raw.arrays.qfrc_actuator[j.v];
 assert(Math.abs(effort-input.controls[i]*j.gear)<1e-10);
 assert(Math.abs(effort)<=Math.abs(j.gear)+1e-10);
}
const proofBytes=fs.readdirSync(dir).reduce((sum,f)=>sum+fs.statSync(dir+f).size,0);
assert(proofBytes<=100000);
console.log(JSON.stringify({savedDataAudit:'PASS',conditions:92,passing:91,oldEvidenceUnchanged:true,exactCandidate:true,telemetryRetainedForRejectedCandidate:true,newNativeEvaluations:0,newTicks:0,proofBytes}));
