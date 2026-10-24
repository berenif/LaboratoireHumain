// Audit existing bytes only. No model, solver or native evaluation is imported.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{gunzipSync}from'node:zlib';
import{axisResiduals,torsoKinematics,robustNonpositiveLimit}from'./pull-torso-guards.mjs';
const dir='web/proofs/pull-step-axis-pure/',read=p=>JSON.parse(fs.readFileSync(p)),gz=p=>JSON.parse(gunzipSync(fs.readFileSync(p)));
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const protocol=read(dir+'protocol.json'),result=gz(dir+'result.json.gz');
const events=gunzipSync(fs.readFileSync(dir+'evaluations.ndjson.gz')).toString().trim().split('\n').map(JSON.parse);
const cp=gz('web/proofs/pull-step-001/final.json.gz'),old=read('web/proofs/pull-step-coupled-pure/result.json');
const center=read('web/proofs/pull-step-constrained-recheck/result.json'),centerRaw=read('web/proofs/pull-step-constrained-recheck/candidate-native.json');
assert.equal(sha('web/proofs/pull-step-coupled-pure/result.json'),protocol.originalSHA256);
assert.equal(sha('web/proofs/pull-step-constrained-recheck/result.json'),protocol.centerSHA256);
assert.equal(sha('web/proofs/pull-step-constrained-recheck/candidate-native.json'),protocol.centerNativeSHA256);
assert.equal(sha('web/proofs/pull-step-001/final.json.gz'),result.inputSHA256);
assert.equal(cp.state.length,233);assert.equal(result.counts.completed,22);assert.equal(result.counts.integrations,0);assert.equal(result.counts.forecasts,0);
const begin=events.filter(e=>e.event==='begin'),native=events.filter(e=>e.event==='native-before-guards'),end=events.filter(e=>e.event==='end');
assert.equal(begin.length,22);assert.equal(native.length,22);assert.equal(end.length,22);
for(let i=0;i<22;i++){
 const b=begin[i],n=native[i],e=end[i];assert.equal(b.evaluation,i+1);assert.equal(n.evaluation,i+1);assert.equal(e.evaluation,i+1);
 assert(events.indexOf(b)<events.indexOf(n)&&events.indexOf(n)<events.indexOf(e));
 assert.deepEqual(n.arrays.qpos,cp.physics.qpos);assert.deepEqual(n.arrays.qvel,cp.physics.qvel);assert.equal(n.time,cp.physics.time);
 assert.deepEqual(n.controls,b.controls);assert.deepEqual(n.arrays.efc_state,centerRaw.arrays.efc_state);assert.deepEqual(e.regimeChanges,[]);
 assert(n.controls.every((v,k)=>Math.abs(v)<=1+1e-10&&Math.abs(v-old.baseline.controls[k])<=.1+1e-10));
 assert(n.arrays.qfrc_actuator.slice(0,6).every(v=>v===0));assert(n.arrays.qfrc_applied.every(v=>v===0));
 assert.deepEqual(n.arrays.xfrc_applied,centerRaw.arrays.xfrc_applied);
 for(const [k,j]of old.joints.entries())assert(Math.abs(n.arrays.qfrc_actuator[j.v]-j.gear*n.controls[k])<1e-10);
 if(i<21){const expected=center.candidate.controls.slice();expected[i]+=protocol.steps[i];assert.deepEqual(n.controls,expected);}
}
const candidate=native[21],conditions=result.nativeConstraints;assert.equal(conditions.length,98);assert.equal(new Set(conditions.map(c=>c.label)).size,98);
assert(conditions.every(c=>c.pass&&c.value<=c.tolerance));assert(result.linearConstraints.every(c=>c.pass));assert(result.linearization.every(c=>c.pass));assert(result.readiness);
const near=(a,b,t=1e-11)=>assert(Math.abs(a-b)<=t),check=(label,v)=>near(conditions.find(c=>c.label===label).value,v);
let right=0;for(const c of candidate.contacts){const [n,t1,t2]=c.forceLocal;if(c.body===7)right+=n;check(`contact-${c.index}-negative-normal`,-n);for(const a of[-1,1])for(const b of[-1,1])check(`contact-${c.index}-friction-${a}-${b}`,a*t1/c.friction[0]+b*t2/c.friction[1]-n);}
check('right-load-shortfall',100-right);
for(const g of center.geometryRows){const acc=g.bias+g.jac.reduce((s,v,i)=>s+v*candidate.arrays.qacc[i],0);check(g.label,g.gap+.00125*g.velocity+.5*.00125**2*acc);}
for(const [k,v]of axisResiduals(cp.physics.qvel.slice(3,6),candidate.arrays.qacc.slice(3,6),.00125).entries())check(v.label,v.value);
for(let i=0;i<21;i++)for(const sign of[-1,1])check(`motor-${i}-${sign}`,sign*(candidate.controls[i]-old.baseline.controls[i])-Math.min(.1,1-sign*old.baseline.controls[i]));
check('lateral-progress',result.candidate.output[1]+1e-6);for(const sign of[-1,1])check('global-roll-'+sign,sign*result.candidate.output[3]-Math.abs(old.baseline.output[3]));
// Independent quaternion -> body rotation reconstruction, no native kinematics.
const [w,x,y,z]=cp.physics.qpos.slice(3,7),R=[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
const torso=torsoKinematics(R,cp.physics.qvel.slice(3,6),candidate.arrays.qacc.slice(3,6));check('torso-tilt-acceleration',torso.thetaDD);check('torso-orientation-speed-energy-rate',torso.orientationSpeedEnergyDerivative);
near(protocol.scalarLimit,robustNonpositiveLimit(center.candidate.features[1],.2,1e-10));
const gp=result.proposal.prediction[1],allowance=Math.max(1e-10,.2*Math.abs(gp-center.candidate.features[1]));near(allowance,result.proposal.marginAllowance);assert(gp+allowance<=1e-10);assert(Math.abs(torso.orientationSpeedEnergyDerivative-gp)<=allowance);
const delta=candidate.controls.map((v,i)=>v-old.baseline.controls[i]);near(result.regularizationCost,.0001*delta.reduce((s,v)=>s+v*v,0));
assert(result.solve.pivots<=64);assert(result.candidate.allDynamicsResidualMax<=1e-7);
const proofBytes=fs.readdirSync(dir).reduce((s,f)=>s+fs.statSync(dir+f).size,0);assert(proofBytes<=100000);
console.log(JSON.stringify({savedAudit:'PASS',nativeRecords:22,conditionsIndependentlyReconstructed:98,unchangedOldHashes:true,proofBytes,newNativeEvaluations:0,newTicks:0}));
