// One recentered, bounded fixed-state diagnostic. NEVER imported by the app.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';
import{gunzipSync,gzipSync,createGzip,constants as zlibConstants}from'node:zlib';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{nativeState}from'./pull-step-restoration.mjs';
import{constrainedLS,dot}from'./pull-constrained-ls.mjs';
import{torsoKinematics,axisResiduals,robustNonpositiveLimit,recenteredControlBound}from'./pull-torso-guards.mjs';
const PROTOCOL={schema:1,maxDirectEvaluations:22,perturbations:21,candidates:1,maxPivots:64,integrations:0,forecasts:0,maxEvidenceBytes:100000,
 originalSHA256:'3182efd12ebca1f59b0c0144bbf3d54b90fd76927aa2ed80cfc92064f163df53',
 centerSHA256:'e3d7b13b84cc503d2cedb7ebbe28ab3efdb8915b8b7c70981a1ae6837ae3d02a',
 centerNativeSHA256:'1c537202749fce9115edd93620a078b1052e253eaf67eb7cb66a94fd753a4d3e',
 epsilon:.001,lambda:.0001,trust:.1,normalizedSolverTolerance:1e-10,dt:.00125,
 objective:'original target, weights, scales; new sensitivities at saved native candidate; regularize TOTAL control change from original u0',
 perturbation:'one sided .001, positive if inside ORIGINAL trust/caps, otherwise negative; fixed before evaluations, never replace a failed perturbation',
 conditions:'all 92 old conditions plus six |omega_i+dt*alpha_i|<=|omega_i|; local conservative braking guard, not universal balance law',
 margin:'g_pred+max(tolerance,.2*abs(g_pred-g_center))<=0; prediction allowance must still be checked natively',
 linearErrorRelative:.2,linearErrorAbsolute:.0001,
 tolerances:{torso:1e-8,torsoEnergy:1e-10,force:1e-7,gap:1e-9,control:1e-10,base:1e-7,axis:1e-10},
 failure:'stop on changed sensitivity regime, solver failure or failed native candidate; no retry or tuning',
 record:'all raw native outputs gzip-streamed and flushed BEFORE guards; dynamics counters distinct from simulation ticks'};
if(process.argv.includes('--protocol')){console.log(JSON.stringify(PROTOCOL,null,2));process.exit(0);}
const sha=b=>crypto.createHash('sha256').update(b).digest('hex');
function checked(path,hash){const b=fs.readFileSync(path);assert.equal(sha(b),hash,path);return JSON.parse(b);}
const old=checked('web/proofs/pull-step-coupled-pure/result.json',PROTOCOL.originalSHA256);
const centerResult=checked('web/proofs/pull-step-constrained-recheck/result.json',PROTOCOL.centerSHA256);
const centerNative=checked('web/proofs/pull-step-constrained-recheck/candidate-native.json',PROTOCOL.centerNativeSHA256);
const raw=fs.readFileSync('web/proofs/pull-step-001/final.json.gz');assert.equal(sha(raw),old.inputHash);const cp=JSON.parse(gunzipSync(raw));assert.equal(cp.state.length,233);
const terminal=fs.readFileSync('web/proofs/pull-step-002/final.json.gz');assert.equal(sha(terminal),old.protocol.input002);assert(old.terminalGuard.rejected);
const xml=fs.readFileSync('web/dist/humanoid.xml','utf8');assert.equal(sha(xml),old.modelSHA256);
const center=centerResult.candidate,u0=old.baseline.controls,uc=center.controls,offset=uc.map((v,i)=>v-u0[i]);
assert.deepEqual(centerNative.controls,uc);assert.deepEqual(centerNative.arrays.qacc,center.qacc);
assert.deepEqual(centerNative.arrays.qpos,cp.physics.qpos);assert.deepEqual(centerNative.arrays.qvel,cp.physics.qvel);assert.equal(centerNative.time,cp.physics.time);
assert(offset.every(v=>Math.abs(v)<=PROTOCOL.trust+PROTOCOL.tolerances.control));
const steps=uc.map((v,i)=>v+PROTOCOL.epsilon<=Math.min(1,u0[i]+PROTOCOL.trust)+PROTOCOL.tolerances.control?PROTOCOL.epsilon:-PROTOCOL.epsilon);
assert(steps.every((s,i)=>Math.abs(uc[i]+s)<=1&&Math.abs(uc[i]+s-u0[i])<=PROTOCOL.trust+PROTOCOL.tolerances.control));
const scalarLimit=robustNonpositiveLimit(center.features[1],PROTOCOL.linearErrorRelative,PROTOCOL.tolerances.torsoEnergy);
const folder='web/proofs/pull-step-axis-pure';assert(!fs.existsSync(folder),'Existing evidence: inspect, never retry');fs.mkdirSync(folder);
const bytes=()=>fs.readdirSync(folder).reduce((s,f)=>s+fs.statSync(folder+'/'+f).size,0);
function save(name,value){const b=gzipSync(JSON.stringify(value)+'\n',{level:9});assert(bytes()+b.length<=PROTOCOL.maxEvidenceBytes);fs.writeFileSync(folder+'/'+name,b,{flag:'wx'});}
fs.writeFileSync(folder+'/protocol.json',JSON.stringify({...PROTOCOL,steps,scalarLimit,stateSHA256:sha(JSON.stringify(cp.state)),controlsSHA256:sha(JSON.stringify(uc))},null,2)+'\n',{flag:'wx'});
// One streaming gzip, synchronously written on each output chunk, flushed after
// every record. An interrupted stream is recoverable with Z_SYNC_FLUSH decoding.
const fd=fs.openSync(folder+'/evaluations.ndjson.gz','wx'),zip=createGzip({level:9});let ioError=null;
zip.on('error',e=>{ioError=e;});zip.on('data',chunk=>{try{assert(bytes()+chunk.length<=PROTOCOL.maxEvidenceBytes);fs.writeSync(fd,chunk);}catch(e){ioError=e;}});
async function record(value){assert(!ioError);await new Promise((resolve,reject)=>zip.write(JSON.stringify(value)+'\n',e=>e?reject(e):resolve()));await new Promise((resolve,reject)=>zip.flush(zlibConstants.Z_SYNC_FLUSH,e=>e?reject(e):resolve()));assert(!ioError,String(ioError));fs.fsyncSync(fd);}
async function closeJournal(){await new Promise((resolve,reject)=>{zip.once('end',resolve);zip.once('error',reject);zip.end();});fs.fsyncSync(fd);fs.closeSync(fd);assert(!ioError,String(ioError));}
const before=performance.now(),mj=await load(),model=mj.MjModel.from_xml_string(xml),joints=old.joints,force=old.baseline.force,point=old.baseline.point;
assert.equal(mj.mj_versionString(),old.engine);assert.equal(model.opt.timestep,PROTOCOL.dt);assert.equal(model.opt.cone.value??model.opt.cone,0);assert.equal(model.nv,27);assert.equal(model.nu,21);
const sub=(a,b)=>a.map((x,i)=>x-b[i]),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],maxabs=x=>Math.max(...x.map(Math.abs)),norm=x=>Math.hypot(...x),rot=(R,v)=>[0,1,2].map(k=>dot(Array.from(R.slice(3*k,3*k+3)),v));
const factors=old.scales.map((x,i)=>old.weights[i]/x),mass=old.scales[6]/9.81,zeroState=JSON.stringify(cp.state),metadata=[];
let attempted=0,completed=0,nativeMs=0,geometryRows=centerResult.geometryRows,frames=centerResult.frames,baselineFeatures,final;
function scratch(){const p=Object.create(Physics.prototype),data=new mj.MjData(model);Object.assign(p,{mj,model,data,joints,steps:0,contactBuffer:new mj.DoubleBuffer(6),names:[]});mj.mj_setState(model,data,cp.state,cp.spec);assert.deepEqual(nativeState(p,cp.spec),cp.state);mj.mj_kinematics(model,data);mj.mj_comPos(model,data);mj.mj_comVel(model,data);assert.deepEqual(nativeState(p,cp.spec),cp.state);return p;}
function dispose(p){p.contactBuffer.delete();p.data.delete();}
function torso(p,qacc){return torsoKinematics(Array.from(p.data.xmat.slice(9,18)),Array.from(p.data.qvel.slice(3,6)),qacc.slice(3,6));}
function features(p,qacc,contactForces){
 const t=torso(p,qacc),values=[t.thetaDD,t.orientationSpeedEnergyDerivative];
 for(let i=0;i<frames.length;i++){
  const f=contactForces[i],frame=frames[i].frame,fn=dot(f,frame.slice(0,3)),t1=dot(f,frame.slice(3,6))/frames[i].friction[0],t2=dot(f,frame.slice(6,9))/frames[i].friction[1];
  values.push(-fn);for(const a of[-1,1])for(const b of[-1,1])values.push(a*t1+b*t2-fn);
 }
 const right=contactForces.reduce((s,f,i)=>s+(frames[i].body===7?dot(f,frames[i].frame.slice(0,3)):0),0);values.push(100-right);
 const geometry=geometryRows.map(g=>{const a=dot(g.jac,qacc)+g.bias,v=g.velocity,gap=g.gap,approx=gap+model.opt.timestep*v+.5*model.opt.timestep**2*a;values.push(approx);return{label:g.label,gap,velocity:v,acceleration:a,localGapNextSubstep:approx};});
 values.push(...axisResiduals(Array.from(p.data.qvel.slice(3,6)),qacc.slice(3,6),model.opt.timestep).map(x=>x.value));
 return{values,torso:t,geometry,rightLoad:right};
}
async function evaluate(label,controls){
 assert(attempted<PROTOCOL.maxDirectEvaluations);assert(controls.every(x=>Number.isFinite(x)&&Math.abs(x)<=1+PROTOCOL.tolerances.control));
 assert(controls.every((v,i)=>Math.abs(v-u0[i])<=PROTOCOL.trust+PROTOCOL.tolerances.control),'Original trust envelope exceeded');
 assert(bytes()+20000<=PROTOCOL.maxEvidenceBytes,'Reserve final telemetry/result before any call');
 const p=scratch(),d=p.data;d.ctrl.set(controls);d.xfrc_applied.fill(0);p.apply(15,point,force);assert(d.qfrc_applied.every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(0,90)).every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(96)).every(x=>x===0));
 await record({event:'begin',evaluation:++attempted,label,controls});const t=performance.now();mj.mj_forward(model,d);const ms=performance.now()-t;nativeMs+=ms;completed++;
 // Candidate telemetry is durable BEFORE any physical/regime comparison. No
 // guard can erase a completed native response again. No integration here.
 const native={time:d.time,ncon:d.ncon,nefc:d.nefc,controls:Array.from(d.ctrl),contacts:[],arrays:{}};
 for(const key of['qpos','qvel','qacc','qacc_warmstart','qfrc_bias','qfrc_passive','qfrc_actuator','qfrc_constraint','qfrc_applied','xfrc_applied','actuator_force'])native.arrays[key]=Array.from(d[key]);
 for(const key of['efc_state','efc_type','efc_id','efc_force','efc_pos','efc_margin','efc_vel','efc_aref','efc_b'])native.arrays[key]=Array.from(d[key]).slice(0,d.nefc);
 const nc=d.contact;
 for(let i=0;i<d.ncon;i++){const c=nc.get(i);mj.mj_contactForce(model,d,i,p.contactBuffer);native.contacts.push({index:i,geom1:c.geom1,geom2:c.geom2,body:model.geom_bodyid[c.geom2],dim:c.dim,position:Array.from(c.pos),frame:Array.from(c.frame),friction:Array.from(c.friction),distance:c.dist,efcAddress:c.efc_address,forceLocal:Array.from(p.contactBuffer.GetView())});c.delete();}nc.delete();
 await record({event:'native-before-guards',evaluation:attempted,label,nativeMs:ms,...native});
 p.readReactions();assert.deepEqual(Array.from(d.qpos),cp.physics.qpos);assert.deepEqual(Array.from(d.qvel),cp.physics.qvel);assert.equal(d.time,cp.physics.time);assert.equal(p.steps,0);
 const F=force.slice(),H=cross(sub(point,old.baseline.com),force);for(const c of p.reactions.points){const h=cross(sub(c.position,old.baseline.com),c.forceWorld);for(let i=0;i<3;i++){F[i]+=c.forceWorld[i];H[i]+=h[i];}}
 const output=[...F.map((x,i)=>x/mass-(i===2?9.81:0)),...H,p.reactions.normalLoads[7]||0,p.reactions.normalLoads[10]||0];
 const contacts=d.contact,signature=[];for(let i=0;i<d.ncon;i++){const c=contacts.get(i);signature.push([c.geom1,c.geom2,c.dim,...c.pos]);c.delete();}contacts.delete();
 const regimeChanges=native.arrays.efc_state.flatMap((state,i)=>state===centerNative.arrays.efc_state[i]?[]:[{row:i,type:native.arrays.efc_type[i],id:native.arrays.efc_id[i],before:centerNative.arrays.efc_state[i],after:state}]);
 assert.deepEqual(signature,old.baseline.contactSignature,'Contact geometry changed; raw output retained');
 if(label!=='candidate')assert.deepEqual(native.arrays.efc_state,centerNative.arrays.efc_state,'Sensitivity regime changed; raw output retained, stop');
 assert.equal(native.contacts.length,frames.length);native.contacts.forEach((c,i)=>{assert.equal(c.body,frames[i].body);assert.deepEqual(c.position,old.baseline.contacts[i].position);});
 const M=new mj.DoubleBuffer(729),jp=new mj.DoubleBuffer(81),jr=new mj.DoubleBuffer(81);mj.mj_fullM(model,d,M);mj.mj_jac(model,d,jp,jr,point,15);const mat=M.GetView(),J=jp.GetView(),res=[];
 for(let i=0;i<27;i++){let v=d.qfrc_bias[i]-d.qfrc_passive[i]-d.qfrc_actuator[i]-d.qfrc_constraint[i]-d.qfrc_applied[i];for(let k=0;k<27;k++)v+=mat[27*i+k]*d.qacc[k];for(let k=0;k<3;k++)v-=J[27*k+i]*force[k];res.push(v);}
 const applied=joints.map(j=>d.qfrc_actuator[j.v]);assert(d.qfrc_actuator.slice(0,6).every(x=>x===0));assert(applied.every((x,i)=>Math.abs(x-controls[i]*joints[i].gear)<1e-10));assert(applied.every((x,i)=>Math.abs(x)<=Math.abs(joints[i].gear)+1e-10));assert(maxabs(res)<=PROTOCOL.tolerances.base);
 const qacc=Array.from(d.qacc),contactForces=native.contacts.map(c=>[0,1,2].map(k=>c.frame[k]*c.forceLocal[0]+c.frame[3+k]*c.forceLocal[1]+c.frame[6+k]*c.forceLocal[2])),f=features(p,qacc,contactForces);
 const jd=new mj.DoubleBuffer(81),rd=new mj.DoubleBuffer(81),contactKinematics=native.contacts.map(c=>{mj.mj_jac(model,d,jp,jr,c.position,c.body);mj.mj_jacDot(model,d,jd,rd,c.position,c.body);const velocity=[0,1,2].map(k=>dot(Array.from(jp.GetView().slice(27*k,27*k+27)),d.qvel)),acceleration=[0,1,2].map(k=>dot(Array.from(jp.GetView().slice(27*k,27*k+27)),qacc)+dot(Array.from(jd.GetView().slice(27*k,27*k+27)),d.qvel));return{index:c.index,body:c.body,distance:c.distance,velocity,acceleration,normalVelocity:dot(velocity,c.frame.slice(0,3)),normalAcceleration:dot(acceleration,c.frame.slice(0,3)),meaning:'instantaneous material point at contact position; distinct from limiting capsule endpoint gap'};});jd.delete();rd.delete();
 const result={label,controls,contactKinematics,regimeChanges,groundContactRegimeUnchanged:regimeChanges.every(r=>r.type<5),jointLimitRegimeChanges:regimeChanges.filter(r=>r.type===3),contactGeometryUnchanged:true,physicalPoseVelocityTimeUnchanged:true,output,qacc,contactForces,features:f.values,torso:f.torso,geometry:f.geometry,appliedNm:applied,baseResidual:res.slice(0,6),allDynamicsResidualMax:maxabs(res),nativeMs:ms};
 await record({event:'end',evaluation:attempted,label,output,features:f.values,regimeChanges,baseResidualMax:maxabs(res),nativeMs:ms});
 for(const b of[M,jp,jr])b.delete();dispose(p);return result;
}
try{
 const p=scratch();baselineFeatures=features(p,center.qacc,center.contactForces);
 assert.deepEqual(baselineFeatures.values.slice(0,47),center.features);assert.deepEqual(nativeState(p,cp.spec),cp.state);dispose(p);
 metadata.push({label:'torso-tilt-acceleration',unit:'rad/s^2',tolerance:PROTOCOL.tolerances.torso},{label:'torso-orientation-speed-energy-rate',unit:'1/s^3',tolerance:PROTOCOL.tolerances.torsoEnergy});
 frames.forEach((_,i)=>{metadata.push({label:`contact-${i}-negative-normal`,unit:'N',tolerance:PROTOCOL.tolerances.force});for(const a of[-1,1])for(const b of[-1,1])metadata.push({label:`contact-${i}-friction-${a}-${b}`,unit:'N',tolerance:PROTOCOL.tolerances.force});});
 metadata.push({label:'right-load-shortfall',unit:'N',tolerance:PROTOCOL.tolerances.force});geometryRows.forEach(g=>metadata.push({label:g.label,unit:'m',tolerance:PROTOCOL.tolerances.gap}));
 axisResiduals(cp.physics.qvel.slice(3,6),center.qacc.slice(3,6),PROTOCOL.dt).forEach(v=>metadata.push({label:v.label,unit:'rad/s',tolerance:PROTOCOL.tolerances.axis}));
 assert.equal(metadata.length,53);
 final={protocol:PROTOCOL,modelSHA256:old.modelSHA256,inputSHA256:old.inputHash,centerSource:PROTOCOL.centerSHA256,baselineFeatures,scalarLimit,readiness:false,scope:'recentered local braking witness, never a step or dynamic support proof'};
 const derivative=metadata.map(()=>Array(21).fill(0)),A=Array.from({length:8},()=>Array(21).fill(0));
 for(let i=0;i<21;i++){
  const controls=uc.slice();controls[i]+=steps[i];const response=await evaluate(`sensitivity-${i}`,controls);
  for(let k=0;k<metadata.length;k++)derivative[k][i]=(response.features[k]-baselineFeatures.values[k])/steps[i];
  for(let k=0;k<8;k++)A[k][i]=(response.output[k]-center.output[k])/steps[i]*factors[k];
 }
 const rhs=old.target.map((v,i)=>(v-center.output[i])*factors[i]),H=Array(441).fill(0),b=Array(21).fill(0);
 for(let i=0;i<21;i++){b[i]=A.reduce((s,row,k)=>s+row[i]*rhs[k],0)-PROTOCOL.lambda*offset[i];for(let j=0;j<21;j++)H[21*i+j]=A.reduce((s,row)=>s+row[i]*row[j],0)+(i===j?PROTOCOL.lambda:0);}
 const rows=metadata.map((m,k)=>({...m,a:derivative[k],bound:(k===1?scalarLimit:0)-baselineFeatures.values[k],modelLimit:k===1?scalarLimit:0}));
 for(let i=0;i<21;i++)for(const sign of[-1,1])rows.push({label:`motor-${i}-${sign}`,a:Array.from({length:21},(_,j)=>j===i?sign:0),bound:recenteredControlBound(u0[i],uc[i],PROTOCOL.trust,sign),unit:'control',tolerance:PROTOCOL.tolerances.control});
 rows.push({label:'lateral-progress',a:A[1].map(x=>x/factors[1]),bound:-1e-6-center.output[1],unit:'m/s^2',tolerance:1e-8});
 for(const sign of[-1,1])rows.push({label:'global-roll-'+sign,a:A[3].map(x=>sign*x/factors[3]),bound:Math.abs(old.baseline.output[3])-sign*center.output[3],unit:'Nm',tolerance:1e-7});
 assert.equal(rows.length,98);
 const solveStart=performance.now(),solve=constrainedLS(mj,H,b,rows,{maxPivots:PROTOCOL.maxPivots,tolerance:PROTOCOL.normalizedSolverTolerance}),solveMs=performance.now()-solveStart;
 const linearConstraints=solve.x?rows.map(r=>({label:r.label,unit:r.unit,residual:dot(r.a,solve.x)-r.bound,tolerance:r.tolerance,pass:dot(r.a,solve.x)-r.bound<=r.tolerance})):[];
 Object.assign(final,{solve,solveMs,linearConstraints});
 if(!solve.solved||!linearConstraints.every(x=>x.pass)){final.reason='Algebra not qualified; no native candidate evaluated';}
 else{
  const delta=solve.x,controls=uc.map((v,i)=>v+delta[i]),totalDelta=controls.map((v,i)=>v-u0[i]);
  const prediction=baselineFeatures.values.map((v,k)=>v+dot(derivative[k],delta)),outputPrediction=center.output.map((v,k)=>v+dot(A[k],delta)/factors[k]);
  const marginAllowance=Math.max(PROTOCOL.tolerances.torsoEnergy,PROTOCOL.linearErrorRelative*Math.abs(prediction[1]-baselineFeatures.values[1]));
  final.proposal={controls,totalDelta,deltaFromCenter:delta,prediction,outputPrediction,marginAllowance,robustMarginResidual:prediction[1]+marginAllowance};
  await record({event:'proposal-before-native',...final.proposal});assert(final.proposal.robustMarginResidual<=PROTOCOL.tolerances.torsoEnergy);
  const candidate=await evaluate('candidate',controls);
  const nativeConstraints=metadata.map((m,k)=>({...m,value:candidate.features[k],limit:0,pass:candidate.features[k]<=m.tolerance}));
  rows.slice(metadata.length).forEach(r=>{let value=dot(r.a,delta)-r.bound;if(r.label==='lateral-progress')value=candidate.output[1]+1e-6;if(r.label.startsWith('global-roll-'))value=(r.label==='global-roll-1'?1:-1)*candidate.output[3]-Math.abs(old.baseline.output[3]);nativeConstraints.push({label:r.label,unit:r.unit,value,limit:0,tolerance:r.tolerance,pass:value<=r.tolerance});});
  const linearization=candidate.features.map((v,k)=>{const error=Math.abs(v-prediction[k]),limit=Math.max(metadata[k].tolerance,PROTOCOL.linearErrorRelative*Math.abs(prediction[k]-baselineFeatures.values[k]));return{label:metadata[k].label,error,limit,pass:error<=limit};});
  const commonError=norm(sub(candidate.output,outputPrediction).map((x,i)=>x*factors[i])),commonLimit=Math.max(PROTOCOL.linearErrorAbsolute,PROTOCOL.linearErrorRelative*norm(sub(outputPrediction,center.output).map((x,i)=>x*factors[i])));
  const taskCost=output=>dot(output.map((v,i)=>(v-old.target[i])*factors[i]),output.map((v,i)=>(v-old.target[i])*factors[i]));
  Object.assign(final,{candidate,nativeConstraints,linearization,commonError,commonLimit,centerTaskCost:taskCost(center.output),candidateTaskCost:taskCost(candidate.output),regularizationCost:PROTOCOL.lambda*dot(totalDelta,totalDelta),nativeInequalitiesPass:nativeConstraints.every(x=>x.pass),localPredictionPass:linearization.every(x=>x.pass)&&commonError<=commonLimit,readiness:nativeConstraints.every(x=>x.pass)&&linearization.every(x=>x.pass)&&commonError<=commonLimit&&candidate.regimeChanges.length===0,reason:'One native candidate only; any failed criterion stops, no retry or controller activation'});
 }
 final.terminal002={reusedRejection:old.terminalGuard,inputHashChecked:true,newEvaluations:0};assert.equal(JSON.stringify(cp.state),zeroState);
}catch(error){final={...final,readiness:false,error:String(error),stack:error.stack,reason:'Stopped within the declared budget; no replacement candidate'};process.exitCode=1;}
finally{model.delete();await closeJournal();}
Object.assign(final,{counts:{attempted,completed,centerReused:1,newBaselineEvaluations:0,integrations:0,forecasts:0,confirmedMuJoCo:228794,unknownHistoryPreserved:true,Rapier:45735},cost:{nativeMs,elapsedMs:performance.now()-before}});
save('result.json.gz',final);
console.log(JSON.stringify({readiness:final.readiness,reason:final.reason,error:final.error,counts:final.counts,solve:final.solve&&{solved:final.solve.solved,pivots:final.solve.pivots,reason:final.solve.reason},failedLinear:final.linearConstraints?.filter(x=>!x.pass),failedNative:final.nativeConstraints?.filter(x=>!x.pass),failedPrediction:final.linearization?.filter(x=>!x.pass),candidate:final.candidate?.output,torso:final.candidate?.torso,regime:final.candidate?.regimeChanges,cost:final.cost,evidenceBytes:bytes()}));
