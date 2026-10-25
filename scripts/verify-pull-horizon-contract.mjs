// Pure algebra and archived bytes only. Importing this test never loads WASM.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import {diagnosticText,recordBeforeGuard,linearCertificate,eulerDiagnostic,hingeChecks,validateJointMapping,SupportWindows,nonplantarMargin,romMargins,outcome,evaluationResponse,nextUp,nextDown} from './pull-horizon-contract.mjs';
import {tests as legacyTests,command} from './pull-native-horizon.mjs';
let cases=0;const test=(name,fn)=>{fn();cases++;return name;},names=[];
const add=(name,fn)=>names.push(test(name,fn));
const joints=Array.from({length:21},(_,i)=>({q:i+7,v:i+6,name:'j'+i}));
add('27 dofs/28 qpos: exclude base quaternion from hinge positions',()=>{
 validateJointMapping(joints,28,27);assert.throws(()=>validateJointMapping(joints.map((j,i)=>i?j:{...j,q:3}),28,27));assert.throws(()=>validateJointMapping(joints.map((j,i)=>i?j:{...j,v:3}),28,27));
 const q=Array(28).fill(0),after=q.slice(),v=Array(27).fill(0);q[3]=1;after[3]=.9;after[4]=Math.sqrt(.19);v[3]=99;
 v[6]=.5;after[7]=.00125*.5;assert(hingeChecks(q,after,v,joints,.00125).every(c=>c.passed));after[7]+=.001;assert(!hingeChecks(q,after,v,joints,.00125)[0].passed);
});
add('directed IEEE intervals at signs, zero and subnormals',()=>{
 assert(nextDown(1)<1&&nextUp(1)>1);assert(nextDown(-1)<-1&&nextUp(-1)>-1);assert.equal(nextUp(-Number.MIN_VALUE),-0);assert.equal(nextDown(0),-Number.MIN_VALUE);assert.equal(nextUp(0),Number.MIN_VALUE);assert.equal(nextUp(Infinity),Infinity);
});
add('known residual and independently certified inverse enclosure',()=>{
 const c=linearCertificate([[2,2],[0,0],[0,0],[4,4]],[[2,2],[8,8]],[1.125,2]);
 assert.equal(c.status,'certified');assert(c.errorUpper>=.125&&c.errorUpper<.126);assert(c.conditionUpper>=2);assert(c.residual[0][0]<=.25&&c.residual[0][1]>=.25);
});
add('failed inverse defect/rank produces inconclusive, no inflated threshold',()=>{
 assert.equal(linearCertificate([[1,1]],[[1,1]],[1],[[0]]).status,'inconclusive');
 assert.equal(linearCertificate([[0,0]],[[1,1]],[0]).status,'inconclusive');
 assert.equal(linearCertificate([[Infinity,Infinity]],[[1,1]],[0]).status,'inconclusive');
});
add('damped Euler analytic scalar includes damping exactly once',()=>{
 const input={M:[2],damping:[4],qacc:[6],h:.25,beforeV:[1],afterV:[2],smooth:[12],constraint:[0]};
 const c=eulerDiagnostic({...input,nativeVelocityErrorBound:[0]});assert.equal(c.status,'certified');assert(Math.abs(c.effective[0]-4)<1e-14);assert(c.forceEquation.errorUpper<1e-12);
 const wrong=eulerDiagnostic({...input,afterV:[2.5],nativeVelocityErrorBound:[0]});assert.equal(wrong.status,'demonstrated-anomaly');assert(wrong.checks[0].error>.49);
});
add('all 27 generalized velocity components, free base damping zero',()=>{
 const n=27,h=.00125,M=Array.from({length:n*n},(_,k)=>k%(n+1)===0?2:0),damping=Array.from({length:n},(_,k)=>k<6?0:4),beforeV=Array.from({length:n},(_,k)=>k/100),afterV=beforeV.map((v,k)=>v+h*12/(2+h*damping[k]));
 const d=eulerDiagnostic({M,damping,qacc:Array(n).fill(6),h,beforeV,afterV,smooth:Array(n).fill(12),constraint:Array(n).fill(0),nativeVelocityErrorBound:Array(n).fill(0)});
 assert.equal(d.status,'certified');assert.equal(d.checks.length,27);assert(d.checks.slice(0,6).every(c=>Math.abs(c.expected-beforeV[c.k]-h*6)<1e-14));
});
add('no independent native error bound means explicitly inconclusive',()=>{
 const c=eulerDiagnostic({M:[2],damping:[4],qacc:[6],h:.25,beforeV:[1],afterV:[2],smooth:[12],constraint:[0]});
 assert.equal(c.reconstruction.status,'certified');assert.equal(c.status,'inconclusive');assert.equal(c.checks[0].allowed,null);
});
add('mass equation residual is exposed independently of reconstruction residual',()=>{
 const c=eulerDiagnostic({M:[2],damping:[4],qacc:[6],h:.25,beforeV:[1],afterV:[1+11/12],smooth:[11],constraint:[0],nativeVelocityErrorBound:[0]});
 assert(c.massResidual[0][0]<=1&&c.massResidual[0][1]>=1);assert(c.forceEquation.errorUpper>=1/3);assert(c.reconstruction.errorUpper<1e-12);
});
add('coupled SPD and known inverse: certificate encloses a known error',()=>{
 const c=linearCertificate([[2,2],[1,1],[1,1],[2,2]],[[3,3],[3,3]],[1.01,1],[[2/3,-1/3],[-1/3,2/3]]);
 assert.equal(c.status,'certified');assert(c.errorUpper>=.01);assert(c.rho<1e-12);
});
add('ROM infeasibility reaches optimizer as finite negative constraints',()=>{
 const r=romMargins(1.01,0,-1,1);assert(r.upper<0);const rows=[{label:'ROM upper',value:r.upper}];
 const summary={...outcome(rows,'certified'),steps:400};assert.equal(summary.status,'finite-infeasible');assert.equal(summary.feasible,false);
 const reply={evaluated:true,summary,constraints:[r.upper],labels:['ROM upper']};assert.equal(evaluationResponse(reply).constraints[0],r.upper);
});
add('outcome separates numerical qualification, physical infeasibility and anomaly',()=>{
 assert.equal(outcome([{value:1}],'inconclusive').status,'numerical-inconclusive');assert.equal(outcome([{value:-1}],'inconclusive').status,'finite-infeasible');assert.equal(outcome([{value:1}],'demonstrated-anomaly').status,'demonstrated-anomaly');assert.equal(outcome([{value:0}],'certified').feasible,true);
 assert.throws(()=>evaluationResponse({evaluated:true,summary:{steps:400,status:'numerical-inconclusive',feasible:true,physicalFeasible:true,numericalStatus:'inconclusive'},constraints:[0],labels:['x']}));
});
add('19 lost substeps permitted; 20th yields strict negative load margin',()=>{
 const s=new SupportWindows();for(let k=0;k<19;k++)assert(!s.update(.9).violated);const r=s.update(.9);assert(r.violated);assert(r.value<0);assert.equal(r.maxRun,20);
});
add('exact 1 N restores support; separated runs do not accumulate',()=>{
 const s=new SupportWindows();for(let k=0;k<19;k++)s.update(.999);assert(!s.update(1).violated);for(let k=0;k<19;k++)assert(!s.update(.999).violated);assert(s.margin>=0);assert.equal(s.maxRun,19);
});
add('window margin has useful force magnitude, not a dwell rounding artifact',()=>{
 const a=new SupportWindows(),b=new SupportWindows();for(let k=0;k<20;k++){a.update(.9);b.update(.8);}assert(a.margin>b.margin);assert(a.margin<0&&b.margin<0);
 const s=new SupportWindows(Array(19).fill(.9));assert(s.update(.9).violated);assert.throws(()=>new SupportWindows(Array(20).fill(.9)));
});
add('window iff 20 consecutive loads below 1 N (all 10-bit test sequences)',()=>{
 for(let word=0;word<1024;word++){
  const s=new SupportWindows(),loads=Array.from({length:40},(_,k)=>(word>>(k%10))&1?1:.75);let run=0,failed=false;
  for(const f of loads){run=f<1?run+1:0;failed||=run>=20;const r=s.update(f);assert.equal(r.violated,failed);assert.equal(r.value<0,failed);}
 }
});
add('nonplantar per-contact threshold is >.1 N; feet exempt',()=>{
 for(const f of [.099,.1,.101]){const r=nonplantarMargin([{body:1,load:f},{body:7,load:300},{body:10,load:200}]);assert.equal(r.violated,f>.1);assert.equal(r.value<0,f>.1);}
 assert(!nonplantarMargin([{body:1,load:.06},{body:1,load:.06}]).violated); // Existing per-contact rule, NOT invented body-sum rule.
 assert(nonplantarMargin([]).value>0);assert.throws(()=>nonplantarMargin([{body:1,load:NaN}]));
});
add('raw diagnostic survives a throwing guard and preserves nonfinite/signed-zero evidence',()=>{
 const state=[0,-0,NaN,Infinity,-Infinity],snapshot={beforeState:state,beforeStateBinary64:Buffer.from(new Float64Array(state).buffer).toString('base64')};let text;
 assert.throws(()=>recordBeforeGuard(snapshot,s=>text=diagnosticText(s),()=>{throw Error('demonstrated test fault');}),/demonstrated test fault/);
 const saved=JSON.parse(text);assert.equal(saved.beforeState[2].nonfiniteBinary64,'NaN');assert.equal(saved.beforeState[3].nonfiniteBinary64,'Infinity');assert.equal(saved.beforeState[4].nonfiniteBinary64,'-Infinity');
 const buf=Buffer.from(saved.beforeStateBinary64,'base64');assert(Object.is(buf.readDoubleLE(8),-0));assert(Number.isNaN(buf.readDoubleLE(16)));
});
add('unmodified archived executed bytes and historical closure',()=>{
 const source=fs.readFileSync('web/proofs/pull-gravity-frame-001/final.json.gz');assert.equal(crypto.createHash('sha256').update(source).digest('hex'),'3bf174a791649512792b8dea38acbacb54094e509d75b526d26ea43dc1311304');
 const root='web/proofs/pull-native-horizon-001-qualified/',manifest=JSON.parse(fs.readFileSync(root+'executed-sources.json'));
 for(const info of Object.values(manifest)){const bytes=gunzipSync(fs.readFileSync(root+info.archive));assert.equal(crypto.createHash('sha256').update(bytes).digest('hex'),info.sha256);}
 const closure=JSON.parse(fs.readFileSync(root+'closure.json'));assert.equal(closure.newPredictiveSteps,90);assert.equal(closure.complete400stepEvaluations,0);assert.equal(closure.newRealTicks,0);
});
add('legacy interpolation/caching checks are algebra only',()=>{
 assert(legacyTests().passed);const u=Array(21).fill(.2);assert.deepEqual(command(u,[...u,...u],0),u);
});
const result={passed:true,cases,names,nativeLibraryLoaded:false,nativeForwards:0,nativeSteps:0,forecastSteps:0,optimizerCalls:0,historyPreserved:true};
console.log(JSON.stringify(result));
