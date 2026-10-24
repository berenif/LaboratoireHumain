// One constrained fixed-state witness. This never imports/starts a runtime loop.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{gunzipSync}from'node:zlib';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{nativeState}from'./pull-step-restoration.mjs';
import{constrainedLS,dot}from'./pull-constrained-ls.mjs';
const RECHECK=process.argv.includes('--recheck-saved-candidate');
const PROTOCOL={schema:1,maxDirectEvaluations:22,perturbations:21,candidates:1,maxPivots:64,integrations:0,forecasts:0,maxEvidenceBytes:100000,
 previousResultSHA256:'3182efd12ebca1f59b0c0144bbf3d54b90fd76927aa2ed80cfc92064f163df53',
 epsilon:.001,lambda:.0001,trust:.1,normalizedSolverTolerance:1e-10,
 objective:'unchanged previous 8-output Jacobian, target, scales, weights and regularization',
 torso:'thetaDD<=0 while thetaDot>0; dot(upDot,upDD)<=0; world upright vector, actual q/dq',
 contact:'native normal>=0, retained right sum>=100N, unchanged pyramidal friction, present contacts only',
 geometry:'each currently loaded right capsule endpoint: gap+dt*gapDot+.5*dt^2*gapDD<=0; dt=actual .00125s; local Taylor guard, NOT a contact-persistence proof',
 caps:'motor and +/-0.1 trust bounds participate in solve; no post-solve scaling/clipping',
 comparisonTolerance:1e-9,linearErrorRelative:.2,linearErrorAbsolute:.0001,
 tolerances:{torso:1e-8,torsoEnergy:1e-10,force:1e-7,gap:1e-9,control:1e-10,base:1e-7},
 noReinterpretation:'positive contact normal acceleration is not itself loss of support; no an<=0 restriction',
 failure:'stop on common-output/contact-regime mismatch or solver failure; one candidate native recheck only'};
if(RECHECK)Object.assign(PROTOCOL,{maxDirectEvaluations:1,perturbations:0,candidates:1,optimizationCalls:0,savedCandidateSHA256:'16d1cd01736ccf4f0ac15d88744c62c46e5db16b20c57759781ed7f030fe2793',savedJournalSHA256:'27f534dd3c9f40e5699b40d384f71a387439f5cb1ee8eb6f63096db00e9e5328',failure:'persist native telemetry before comparison; one exact saved candidate only, no retry'});
if(process.argv.includes('--protocol')){console.log(JSON.stringify(PROTOCOL,null,2));process.exit(0);}
const sha=x=>crypto.createHash('sha256').update(x).digest('hex'),folder=RECHECK?'web/proofs/pull-step-constrained-recheck':'web/proofs/pull-step-constrained-pure',journal=folder+'/evaluations.ndjson';
assert(!fs.existsSync(folder),'Existing evidence: inspect, do not retry');fs.mkdirSync(folder);fs.writeFileSync(folder+'/protocol.json',JSON.stringify(PROTOCOL,null,2)+'\n',{flag:'wx'});fs.writeFileSync(journal,'',{flag:'wx'});
const bytes=()=>fs.readdirSync(folder).reduce((s,x)=>s+fs.statSync(folder+'/'+x).size,0);
function record(file,value){const text=JSON.stringify(value)+'\n';assert(bytes()+Buffer.byteLength(text)<=PROTOCOL.maxEvidenceBytes);fs.appendFileSync(file,text);}
const oldRaw=fs.readFileSync('web/proofs/pull-step-coupled-pure/result.json');assert.equal(sha(oldRaw),PROTOCOL.previousResultSHA256);const old=JSON.parse(oldRaw);
const savedRaw=RECHECK?fs.readFileSync('web/proofs/pull-step-constrained-pure/result.json'):null;
if(RECHECK)assert.equal(sha(savedRaw),PROTOCOL.savedCandidateSHA256);
const saved=RECHECK?JSON.parse(savedRaw):null;
if(RECHECK){assert(saved.solve.solved&&saved.solve.x.length===21);assert.equal(saved.counts.completed,22);assert(!saved.candidate,'Use already saved telemetry instead');assert.deepEqual(saved.protocol.tolerances,PROTOCOL.tolerances);}
const savedJournalRaw=RECHECK?fs.readFileSync('web/proofs/pull-step-constrained-pure/evaluations.ndjson'):null;
if(RECHECK)assert.equal(sha(savedJournalRaw),PROTOCOL.savedJournalSHA256);
const savedJournal=RECHECK?savedJournalRaw.toString().trim().split('\n').map(JSON.parse):[];
const oldJournal=fs.readFileSync('web/proofs/pull-step-coupled-pure/evaluations.ndjson','utf8').trim().split('\n').map(JSON.parse);
const raw=fs.readFileSync('web/proofs/pull-step-001/final.json.gz');assert.equal(sha(raw),old.inputHash);const cp=JSON.parse(gunzipSync(raw));
const terminalRaw=fs.readFileSync('web/proofs/pull-step-002/final.json.gz');assert.equal(sha(terminalRaw),old.protocol.input002);assert(old.terminalGuard.rejected);
const xml=fs.readFileSync('web/dist/humanoid.xml','utf8');assert.equal(sha(xml),old.modelSHA256);
const before=performance.now(),mj=await load(),model=mj.MjModel.from_xml_string(xml),u0=old.baseline.controls,joints=old.joints,force=old.baseline.force,point=old.baseline.point;
assert.equal(mj.mj_versionString(),old.engine);assert.equal(model.opt.timestep,.00125);assert.equal(model.opt.cone.value??model.opt.cone,0,'Only current pyramidal cone is covered');
assert.equal(model.nv,27);assert.equal(model.nu,21);assert.equal(old.lambda,PROTOCOL.lambda);assert.equal(old.protocol.maxControlDelta,PROTOCOL.trust);
const sub=(a,b)=>a.map((x,i)=>x-b[i]),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],maxabs=x=>Math.max(...x.map(Math.abs)),norm=x=>Math.hypot(...x),rot=(R,v)=>[0,1,2].map(k=>dot(Array.from(R.slice(3*k,3*k+3)),v));
const factors=old.scales.map((x,i)=>old.weights[i]/x),mass=old.scales[6]/9.81,zeroState=JSON.stringify(cp.state),metadata=[];
let attempted=0,completed=0,nativeMs=0,geometryRows,frames,baselineFeatures,final;
function scratch(){const p=Object.create(Physics.prototype),data=new mj.MjData(model);Object.assign(p,{mj,model,data,joints,steps:0,contactBuffer:new mj.DoubleBuffer(6),names:[]});mj.mj_setState(model,data,cp.state,cp.spec);assert.deepEqual(nativeState(p,cp.spec),cp.state);mj.mj_kinematics(model,data);mj.mj_comPos(model,data);mj.mj_comVel(model,data);assert.deepEqual(nativeState(p,cp.spec),cp.state);return p;}
function dispose(p){p.contactBuffer.delete();p.data.delete();}
function torso(p,qacc){const R=p.data.xmat.slice(9,18),w=Array.from(p.data.qvel.slice(3,6)),n=rot(R,[0,0,1]),nd=rot(R,cross(w,[0,0,1])),add=cross(qacc.slice(3,6),[0,0,1]),centripetal=cross(w,cross(w,[0,0,1])),ndd=rot(R,add.map((x,i)=>x+centripetal[i])),theta=Math.acos(Math.max(-1,Math.min(1,n[2]))),thetaDot=-nd[2]/Math.sin(theta);assert(theta>.001&&thetaDot>0);return{up:n,upDot:nd,upDD:ndd,theta,thetaDot,thetaDD:-(ndd[2]+Math.cos(theta)*thetaDot**2)/Math.sin(theta),orientationSpeedEnergyDerivative:dot(nd,ndd)};}
function features(p,qacc,contactForces){
 const t=torso(p,qacc),values=[t.thetaDD,t.orientationSpeedEnergyDerivative];
 for(let i=0;i<frames.length;i++){
  const f=contactForces[i],frame=frames[i].frame,fn=dot(f,frame.slice(0,3)),t1=dot(f,frame.slice(3,6))/frames[i].friction[0],t2=dot(f,frame.slice(6,9))/frames[i].friction[1];
  values.push(-fn);for(const a of[-1,1])for(const b of[-1,1])values.push(a*t1+b*t2-fn);
 }
 const right=contactForces.reduce((s,f,i)=>s+(frames[i].body===7?dot(f,frames[i].frame.slice(0,3)):0),0);values.push(100-right);
 const geometry=geometryRows.map(g=>{const a=dot(g.jac,qacc)+g.bias,v=g.velocity,gap=g.gap,approx=gap+model.opt.timestep*v+.5*model.opt.timestep**2*a;values.push(approx);return{label:g.label,gap,velocity:v,acceleration:a,localGapNextSubstep:approx};});
 return{values,torso:t,geometry,rightLoad:right};
}
function evaluate(label,controls){
 assert(attempted<PROTOCOL.maxDirectEvaluations);assert(controls.every(x=>Number.isFinite(x)&&Math.abs(x)<=1+PROTOCOL.tolerances.control));
 const p=scratch(),d=p.data;d.ctrl.set(controls);d.xfrc_applied.fill(0);p.apply(15,point,force);assert(d.qfrc_applied.every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(0,90)).every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(96)).every(x=>x===0));
 record(journal,{event:'begin',evaluation:++attempted,label});const t=performance.now();mj.mj_forward(model,d);const ms=performance.now()-t;nativeMs+=ms;completed++;
 // Candidate telemetry is durable BEFORE any physical/regime comparison. No
 // guard can erase a completed native response again. No integration here.
 const native={time:d.time,ncon:d.ncon,nefc:d.nefc,controls:Array.from(d.ctrl),contacts:[],arrays:{}};
 for(const key of['qpos','qvel','qacc','qacc_warmstart','qfrc_bias','qfrc_passive','qfrc_actuator','qfrc_constraint','qfrc_applied','xfrc_applied','actuator_force'])native.arrays[key]=Array.from(d[key]);
 for(const key of['efc_state','efc_type','efc_id','efc_force','efc_pos','efc_margin','efc_vel','efc_aref','efc_b'])native.arrays[key]=Array.from(d[key]).slice(0,d.nefc);
 const nc=d.contact;
 for(let i=0;i<d.ncon;i++){const c=nc.get(i);mj.mj_contactForce(model,d,i,p.contactBuffer);native.contacts.push({index:i,geom1:c.geom1,geom2:c.geom2,body:model.geom_bodyid[c.geom2],dim:c.dim,position:Array.from(c.pos),frame:Array.from(c.frame),friction:Array.from(c.friction),distance:c.dist,efcAddress:c.efc_address,forceLocal:Array.from(p.contactBuffer.GetView())});c.delete();}nc.delete();
 if(label==='candidate')record(folder+'/candidate-native.json',{event:'native-before-guards',evaluation:attempted,nativeMs:ms,...native});
 p.readReactions();assert.deepEqual(Array.from(d.qpos),cp.physics.qpos);assert.deepEqual(Array.from(d.qvel),cp.physics.qvel);assert.equal(d.time,cp.physics.time);assert.equal(p.steps,0);
 const F=force.slice(),H=cross(sub(point,old.baseline.com),force);for(const c of p.reactions.points){const h=cross(sub(c.position,old.baseline.com),c.forceWorld);for(let i=0;i<3;i++){F[i]+=c.forceWorld[i];H[i]+=h[i];}}
 const output=[...F.map((x,i)=>x/mass-(i===2?9.81:0)),...H,p.reactions.normalLoads[7]||0,p.reactions.normalLoads[10]||0];
 const contacts=d.contact,signature=[];for(let i=0;i<d.ncon;i++){const c=contacts.get(i);signature.push([c.geom1,c.geom2,c.dim,...c.pos]);c.delete();}contacts.delete();
 const regimeChanges=native.arrays.efc_state.flatMap((state,i)=>state===old.baseline.efcState[i]?[]:[{row:i,type:native.arrays.efc_type[i],id:native.arrays.efc_id[i],before:old.baseline.efcState[i],after:state}]);
 assert.deepEqual(signature,old.baseline.contactSignature,'Contact geometry changed; raw output retained');
 if(!RECHECK)assert.deepEqual(native.arrays.efc_state,old.baseline.efcState,'Native constraint regime changed; raw output retained');
 assert.equal(native.contacts.length,frames.length);native.contacts.forEach((c,i)=>{assert.equal(c.body,frames[i].body);assert.deepEqual(c.position,old.baseline.contacts[i].position);});
 const M=new mj.DoubleBuffer(729),jp=new mj.DoubleBuffer(81),jr=new mj.DoubleBuffer(81);mj.mj_fullM(model,d,M);mj.mj_jac(model,d,jp,jr,point,15);const mat=M.GetView(),J=jp.GetView(),res=[];
 for(let i=0;i<27;i++){let v=d.qfrc_bias[i]-d.qfrc_passive[i]-d.qfrc_actuator[i]-d.qfrc_constraint[i]-d.qfrc_applied[i];for(let k=0;k<27;k++)v+=mat[27*i+k]*d.qacc[k];for(let k=0;k<3;k++)v-=J[27*k+i]*force[k];res.push(v);}
 const applied=joints.map(j=>d.qfrc_actuator[j.v]);assert(d.qfrc_actuator.slice(0,6).every(x=>x===0));assert(applied.every((x,i)=>Math.abs(x-controls[i]*joints[i].gear)<1e-10));assert(applied.every((x,i)=>Math.abs(x)<=Math.abs(joints[i].gear)+1e-10));assert(maxabs(res)<=PROTOCOL.tolerances.base);
 const qacc=Array.from(d.qacc),contactForces=native.contacts.map(c=>[0,1,2].map(k=>c.frame[k]*c.forceLocal[0]+c.frame[3+k]*c.forceLocal[1]+c.frame[6+k]*c.forceLocal[2])),f=features(p,qacc,contactForces);
 const jd=new mj.DoubleBuffer(81),rd=new mj.DoubleBuffer(81),contactKinematics=native.contacts.map(c=>{mj.mj_jac(model,d,jp,jr,c.position,c.body);mj.mj_jacDot(model,d,jd,rd,c.position,c.body);const velocity=[0,1,2].map(k=>dot(Array.from(jp.GetView().slice(27*k,27*k+27)),d.qvel)),acceleration=[0,1,2].map(k=>dot(Array.from(jp.GetView().slice(27*k,27*k+27)),qacc)+dot(Array.from(jd.GetView().slice(27*k,27*k+27)),d.qvel));return{index:c.index,body:c.body,distance:c.distance,velocity,acceleration,normalVelocity:dot(velocity,c.frame.slice(0,3)),normalAcceleration:dot(acceleration,c.frame.slice(0,3)),meaning:'instantaneous material point at contact position; distinct from limiting capsule endpoint gap'};});jd.delete();rd.delete();
 const result={label,controls,contactKinematics,regimeChanges,groundContactRegimeUnchanged:regimeChanges.every(r=>r.type<5),jointLimitRegimeChanges:regimeChanges.filter(r=>r.type===3),contactGeometryUnchanged:true,physicalPoseVelocityTimeUnchanged:true,output,qacc,contactForces,features:f.values,torso:f.torso,geometry:f.geometry,appliedNm:applied,baseResidual:res.slice(0,6),allDynamicsResidualMax:maxabs(res),nativeMs:ms};
 record(journal,{event:'end',evaluation:attempted,label,output,qacc,contactForces,features:f.values,baseResidualMax:maxabs(res),nativeMs:ms});
 for(const b of[M,jp,jr])b.delete();dispose(p);return result;
}
try{
 // Pure kinematics and collision detection, no new baseline forward solve.
 const p=scratch(),d=p.data;mj.mj_collision(model,d);frames=[];const contacts=d.contact;
 for(let i=0;i<d.ncon;i++){const c=contacts.get(i);assert.equal(c.geom1,0);assert.equal(c.dim,3);const body=model.geom_bodyid[c.geom2];assert([7,10].includes(body));frames.push({body,geom:c.geom2,frame:Array.from(c.frame),friction:Array.from(c.friction),distance:c.dist,position:Array.from(c.pos)});c.delete();}contacts.delete();
 for(const key of['qpos','qvel','qacc','qacc_warmstart','qfrc_bias','qfrc_passive','qfrc_actuator','qfrc_constraint','qfrc_applied','xfrc_applied','actuator_force','efc_state','efc_type','efc_id','efc_force','efc_pos','efc_margin','efc_vel','efc_aref','efc_b'])assert(d[key]!=null&&typeof d[key].length==='number','Native telemetry field unavailable: '+key);
 assert.equal(frames.length,old.baseline.contacts.length);frames.forEach((f,i)=>{assert.deepEqual(f.position,old.baseline.contacts[i].position);assert(Math.abs(f.distance-2*f.position[2])<1e-12);});
 geometryRows=[];const jp=new mj.DoubleBuffer(81),jr=new mj.DoubleBuffer(81),jd=new mj.DoubleBuffer(81),rd=new mj.DoubleBuffer(81);
 for(const f of frames.filter(f=>f.body===7)){
  const pos=Array.from(d.geom_xpos.slice(3*f.geom,3*f.geom+3)),R=d.geom_xmat.slice(9*f.geom,9*f.geom+9),axis=[R[2],R[5],R[8]],half=model.geom_size[3*f.geom+1],radius=model.geom_size[3*f.geom],ends=[-1,1].map(s=>pos.map((v,i)=>v+s*half*axis[i]));
  const center=ends.sort((a,b)=>Math.hypot(a[0]-f.position[0],a[1]-f.position[1])-Math.hypot(b[0]-f.position[0],b[1]-f.position[1]))[0];assert(Math.hypot(center[0]-f.position[0],center[1]-f.position[1])<1e-10);assert(Math.abs(center[2]-radius-f.distance)<1e-10);
  mj.mj_jac(model,d,jp,jr,center,7);mj.mj_jacDot(model,d,jd,rd,center,7);const jac=Array.from(jp.GetView().slice(54,81));geometryRows.push({label:'right-gap-'+geometryRows.length,center,radius,gap:center[2]-radius,jac,bias:dot(Array.from(jd.GetView().slice(54,81)),d.qvel),velocity:dot(jac,d.qvel)});
 }
 for(const b of[jp,jr,jd,rd])b.delete();baselineFeatures=features(p,old.baseline.qacc,old.baseline.contacts.map(c=>c.forceWorld));assert.deepEqual(nativeState(p,cp.spec),cp.state);dispose(p);
 metadata.push({label:'torso-tilt-acceleration',unit:'rad/s^2',tolerance:PROTOCOL.tolerances.torso},{label:'torso-orientation-speed-energy-rate',unit:'1/s^3',tolerance:PROTOCOL.tolerances.torsoEnergy});
 frames.forEach((_,i)=>{metadata.push({label:'contact-'+i+'-negative-normal',unit:'N',tolerance:PROTOCOL.tolerances.force});for(const a of[-1,1])for(const b of[-1,1])metadata.push({label:`contact-${i}-friction-${a}-${b}`,unit:'N',tolerance:PROTOCOL.tolerances.force});});
 metadata.push({label:'right-load-shortfall',unit:'N',tolerance:PROTOCOL.tolerances.force});geometryRows.forEach(g=>metadata.push({label:g.label,unit:'m',tolerance:PROTOCOL.tolerances.gap}));assert.equal(metadata.length,baselineFeatures.values.length);
 const derivative=metadata.map(()=>Array(21).fill(0)),commonErrors=[];
 for(let i=0;i<21;i++){
  const u=u0.slice();u[i]+=PROTOCOL.epsilon;const response=RECHECK?savedJournal.find(e=>e.event==='end'&&e.label==='plus-'+i):evaluate('plus-'+i,u),previous=oldJournal.find(e=>e.event==='end'&&e.label==='plus-'+i);assert(response&&previous);
  const error=maxabs(sub(response.output,previous.output));commonErrors.push(error);assert(error<=PROTOCOL.comparisonTolerance,'Common outputs differ from saved perturbation');
  for(let k=0;k<metadata.length;k++)derivative[k][i]=(response.features[k]-baselineFeatures.values[k])/PROTOCOL.epsilon;
 }
 const A=old.leastSquares.jacobianScaled,rhs=old.target.map((v,i)=>(v-old.baseline.output[i])*factors[i]),H=Array(441).fill(0),b=Array(21).fill(0);
 for(let i=0;i<21;i++){b[i]=A.reduce((s,row,k)=>s+row[i]*rhs[k],0);for(let j=0;j<21;j++)H[21*i+j]=A.reduce((s,row)=>s+row[i]*row[j],0)+(i===j?PROTOCOL.lambda:0);}
 const rows=metadata.map((m,k)=>({label:m.label,a:derivative[k],bound:-baselineFeatures.values[k],unit:m.unit,tolerance:m.tolerance}));
 for(let i=0;i<21;i++)for(const sign of[-1,1])rows.push({label:`motor-${i}-${sign}`,a:Array.from({length:21},(_,j)=>j===i?sign:0),bound:Math.min(PROTOCOL.trust,1-sign*u0[i]),unit:'control',tolerance:PROTOCOL.tolerances.control});
 const ayRow=A[1].map(x=>x/factors[1]);rows.push({label:'lateral-progress',a:ayRow,bound:-1e-6-old.baseline.output[1],unit:'m/s^2',tolerance:1e-8});
 for(const sign of[-1,1])rows.push({label:'global-roll-'+sign,a:A[3].map(x=>sign*x/factors[3]),bound:Math.abs(old.baseline.output[3])-sign*old.baseline.output[3],unit:'Nm',tolerance:1e-7});
 const solveStart=performance.now(),solve=RECHECK?saved.solve:constrainedLS(mj,H,b,rows,{maxPivots:PROTOCOL.maxPivots,tolerance:PROTOCOL.normalizedSolverTolerance}),solveMs=RECHECK?0:performance.now()-solveStart;
 const linearConstraints=solve.x?rows.map(r=>({label:r.label,unit:r.unit,residual:dot(r.a,solve.x)-r.bound,tolerance:r.tolerance,pass:dot(r.a,solve.x)-r.bound<=r.tolerance})):[];
 final={protocol:PROTOCOL,modelSHA256:old.modelSHA256,inputSHA256:old.inputHash,baselineSource:PROTOCOL.previousResultSHA256,baselineFeatures,geometryRows,frames,solve,solveMs,linearConstraints,commonOutputMaxError:Math.max(...commonErrors),readiness:false,scope:'one constrained fixed-state witness; not contact persistence or a step'};
 if(RECHECK){assert.deepEqual(geometryRows,saved.geometryRows);assert.deepEqual(frames,saved.frames);assert.deepEqual(baselineFeatures,saved.baselineFeatures);assert.deepEqual(linearConstraints,saved.linearConstraints);assert.equal(cp.state.length,233);assert.equal(attempted,0);final.savedCandidateSource=PROTOCOL.savedCandidateSHA256;final.optimizationCalls=0;}
 if(!solve.solved||!linearConstraints.every(x=>x.pass)){final.reason='Constrained algebra did not supply a feasible candidate; no native candidate evaluated';}
 else{
  const delta=solve.x,controls=u0.map((v,i)=>v+delta[i]);
  if(RECHECK)record(folder+'/candidate-input.json',{stateSHA256:sha(Buffer.from(JSON.stringify(cp.state))),stateValues:cp.state.length,sourceCheckpointSHA256:old.inputHash,savedCandidateSource:PROTOCOL.savedCandidateSHA256,construction:'same IEEE754 u0[i] + saved.solve.x[i], no optimizer or clipping',controls,controlsSHA256:sha(Buffer.from(JSON.stringify(controls))),force,point,body:15,delta});
  const candidate=evaluate('candidate',controls);
  const nativeConstraints=metadata.map((m,k)=>({label:m.label,unit:m.unit,value:candidate.features[k],limit:0,tolerance:m.tolerance,pass:candidate.features[k]<=m.tolerance}));
  rows.slice(metadata.length).forEach(r=>{let value=dot(r.a,delta)-r.bound;if(r.label==='lateral-progress')value=candidate.output[1]+1e-6;if(r.label.startsWith('global-roll-')){const sign=r.label==='global-roll-1'?1:-1;value=sign*candidate.output[3]-Math.abs(old.baseline.output[3]);}nativeConstraints.push({label:r.label,unit:r.unit,value,limit:0,tolerance:r.tolerance,pass:value<=r.tolerance});});
  const prediction=baselineFeatures.values.map((v,k)=>v+dot(derivative[k],delta)),errors=candidate.features.map((v,k)=>Math.abs(v-prediction[k])),linearization=errors.map((e,k)=>({label:metadata[k].label,error:e,limit:Math.max(metadata[k].tolerance,PROTOCOL.linearErrorRelative*Math.abs(prediction[k]-baselineFeatures.values[k])),pass:e<=Math.max(metadata[k].tolerance,PROTOCOL.linearErrorRelative*Math.abs(prediction[k]-baselineFeatures.values[k]))}));
  const outputPrediction=old.baseline.output.map((v,k)=>v+dot(A[k],delta)/factors[k]),commonError=norm(sub(candidate.output,outputPrediction).map((x,i)=>x*factors[i])),commonLimit=Math.max(PROTOCOL.linearErrorAbsolute,PROTOCOL.linearErrorRelative*norm(sub(outputPrediction,old.baseline.output).map((x,i)=>x*factors[i])));
  const cost=dot(candidate.output.map((v,i)=>(v-old.target[i])*factors[i]),candidate.output.map((v,i)=>(v-old.target[i])*factors[i]));
  Object.assign(final,{candidate,nativeConstraints,nativeInequalitiesPass:nativeConstraints.every(x=>x.pass),localPredictionPass:linearization.every(x=>x.pass)&&commonError<=commonLimit,linearization,commonCandidateError:commonError,commonCandidateLimit:commonLimit,candidateTaskCost:cost,baselineTaskCost:old.leastSquares.baselineTaskCost,regularizationCost:PROTOCOL.lambda*dot(delta,delta),readiness:nativeConstraints.every(x=>x.pass)&&linearization.every(x=>x.pass)&&commonError<=commonLimit&&candidate.groundContactRegimeUnchanged,reason:'Native candidate checked once; joint-limit changes reported separately; no controller activation or dynamic test'});
 }
 final.terminal002={reusedRejection:old.terminalGuard,inputHashChecked:true,newEvaluations:0};assert.equal(JSON.stringify(cp.state),zeroState);
}catch(error){final={...final,readiness:false,error:String(error),stack:error.stack,reason:'Stopped within fixed budget; no automatic retry'};process.exitCode=1;}
finally{model.delete();}
Object.assign(final,{counts:{attempted,completed,baselineReused:1,newBaselineEvaluations:0,integrations:0,forecasts:0,confirmedMuJoCo:228794,unknownHistoryPreserved:true,Rapier:45735},cost:{nativeMs,elapsedMs:performance.now()-before}});
record(folder+'/result.json',final);console.log(JSON.stringify({readiness:final.readiness,reason:final.reason,error:final.error,counts:final.counts,solve:final.solve&&{solved:final.solve.solved,pivots:final.solve.pivots,reason:final.solve.reason},failedLinear:final.linearConstraints?.filter(x=>!x.pass),failedNative:final.nativeConstraints?.filter(x=>!x.pass),candidate:final.candidate?.output,torso:final.candidate?.torso,cost:final.cost,evidenceBytes:bytes()}));
