// One fixed-state diagnostic, NEVER a runtime controller. No mj_step/forecast.
// The first exclusive journal creation locks this one execution against retry.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import load from '../web/dist/vendor/mujoco.js';
import {Physics} from '../web/dist/engine.js';
import {PullStep,PULL_PROTOCOL} from '../web/dist/pull-step.js';
import {nativeState} from './pull-step-restoration.mjs';

export const PROTOCOL=Object.freeze({
 schema:1,kind:'one-local-command-not-a-walking-controller',maxEvaluations001:44,
 evaluations:{baseline:1,perturbations:42,candidate:1},integrations:0,forecasts:0,
 input001:'bdc3c9bd7167a8bc23ea583b55628d0c8d5ee878dd7a6fe3336a03bac41c743c',
 input002:'e00b3428d6ec07351f1f869641ff7990bf11a109086578ba9b73c04a9d30c9fd',
 baseline:'last recorded applied complete command, not recomputed Balance',
 savedControl:'restored ctrl is a later unintegrated proposal; verify it against checkpoint.physics.command, then explicitly install the recorded applied baseline',
 force:'recorded last force at current world position of body15 local point [.09,-.09,.09]; same for all evaluations',
 inputEpsilon:.001,lambda:.0001,maxControlDelta:.1,
 outputs:['comAx','comAy','comAz','HdotX','HdotY','HdotZ','rightNormal','leftNormal'],
 scales:'g for acceleration; mass*g*COM-height-above-contact-plane for moment; mass*g for loads',
 weights:[1,1,1,1,1,1,.25,.25],
 target:'preserve baseline ax/az/HdotY/HdotZ; previous declared projected ay and right/left normal goals; HdotX=0',
 solve:'one regularized least-squares direction, one analytic common scale to motor caps and 0.1 control trust radius; no search or parameter retuning',
 guards:{baseResidual:1e-7,capTolerance:1e-10,frictionTolerance:1e-6,
 lateralMustBeNegative:true,lateralImprovement:1e-6,rollNonworseningNm:1e-6,
 linearErrorRelative:.2,linearErrorAbsolute:.0001,midpointCurvatureRelative:.1,midpointCurvatureAbsolute:.00001,
 minimumRetainedN:100,minimumOtherN:1,maximumEvidenceBytes:100000},
 note:'No future contact, no root force, no state projection. Contact-regime changes make this local linearization unqualified.'
});
if(process.argv.includes('--protocol')){console.log(JSON.stringify(PROTOCOL,null,2));process.exit(0);}
const folder='web/proofs/pull-step-coupled-pure',journal=folder+'/evaluations.ndjson';
assert(!fs.existsSync(folder),'Evidence already exists: inspect instead of retrying');
fs.mkdirSync(folder);fs.writeFileSync(folder+'/protocol.json',JSON.stringify(PROTOCOL,null,2)+'\n',{flag:'wx'});
fs.writeFileSync(journal,'',{flag:'wx'});
const sha=x=>crypto.createHash('sha256').update(x).digest('hex');
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),sub=(a,b)=>a.map((x,i)=>x-b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=x=>Math.hypot(...x),maxabs=x=>Math.max(...x.map(Math.abs));
const bytes=()=>fs.readdirSync(folder).reduce((s,f)=>s+fs.statSync(folder+'/'+f).size,0);
function record(file,value){const line=JSON.stringify(value)+'\n';assert(bytes()+Buffer.byteLength(line)<=PROTOCOL.guards.maximumEvidenceBytes);fs.appendFileSync(file,line);}
const modelText=fs.readFileSync('web/dist/humanoid.xml','utf8'),modelHash=sha(modelText);
const sources=['web/mujoco/engine.js','web/mujoco/pull-step.js'];
for(const source of sources)assert.equal(sha(fs.readFileSync(source)),sha(fs.readFileSync(source.replace('/mujoco/','/dist/'))),'Stale dist');
function input(id){const raw=fs.readFileSync(`web/proofs/pull-step-${id}/final.json.gz`);assert.equal(sha(raw),PROTOCOL['input'+id]);const cp=JSON.parse(gunzipSync(raw));assert.equal(cp.modelSHA256,modelHash);assert.equal(cp.state.length,233);return cp;}
const cp=input('001'),terminal=input('002');
const old=JSON.parse(fs.readFileSync('web/proofs/pull-step-load-pure/result.json','utf8')).cases[0];
const ledger=JSON.parse(fs.readFileSync('web/proofs/pull-step-001/ledger.json','utf8'));
const frozen=JSON.stringify(cp),force=ledger.summary.last.force.slice(),local=[.09,-.09,.09];
const start=performance.now(),mj=await load(),model=mj.MjModel.from_xml_string(modelText);
assert.equal(model.nv,27);assert.equal(model.nu,21);assert.equal(model.nq,28);
const joints=Array.from({length:model.nu},(_,a)=>{const j=model.actuator_trnid[2*a];return{a,j,name:mj.mj_id2name(model,mj.mjtObj.mjOBJ_JOINT.value,j),q:model.jnt_qposadr[j],v:model.jnt_dofadr[j],gear:model.actuator_gear[6*a]};});
assert.equal(new Set(joints.map(j=>j.v)).size,21);assert(joints.every(j=>j.v>=6&&j.gear!==0));
const mass=Array.from(model.body_mass).reduce((a,b)=>a+b,0),g=9.81;
assert.deepEqual(Array.from(model.opt.gravity),[0,0,-g]);
const u0=joints.map((j,i)=>ledger.summary.last.command[i][1]/j.gear);
assert(u0.every(x=>Math.abs(x)+PROTOCOL.inputEpsilon<=1),'Central perturbation unavailable within caps: no replacement search');
let calls=0;const records=[],callTimes=[];
function scratch(state){
 const data=new mj.MjData(model),p=Object.create(Physics.prototype);
 Object.assign(p,{mj,model,data,joints,steps:0,contactBuffer:new mj.DoubleBuffer(6),names:[],reactions:null});
 mj.mj_setState(model,data,state.state,state.spec);assert.deepEqual(nativeState(p,state.spec),state.state);
 // Kinematics only: no contact dynamics evaluation hidden in construction.
 mj.mj_kinematics(model,data);mj.mj_comPos(model,data);
 assert.deepEqual(nativeState(p,state.spec),state.state);
 return p;
}
function dispose(p){p.contactBuffer.delete();p.data.delete();}
function physical(p,state){assert.deepEqual(Array.from(p.data.qpos),state.physics.qpos);assert.deepEqual(Array.from(p.data.qvel),state.physics.qvel);assert.equal(p.data.time,state.physics.time);assert.equal(p.steps,0);}
function evaluate(label,u){
 assert(calls<PROTOCOL.maxEvaluations001);assert(u.length===21&&u.every(x=>Number.isFinite(x)&&Math.abs(x)<=1+1e-12));
 const p=scratch(cp),d=p.data;physical(p,cp);
 assert(maxabs(joints.map((j,i)=>d.ctrl[j.a]*j.gear-cp.physics.command[i].applied))<1e-12,'Saved native control must match the checkpoint pending proposal');
 const beforeQ=Array.from(d.qpos),beforeV=Array.from(d.qvel),beforeTime=d.time;
 const point=p.worldPoint(15,local);d.xfrc_applied.fill(0);p.apply(15,point,force);d.ctrl.set(u);
 assert(d.qfrc_applied.every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(0,15*6)).every(x=>x===0));assert(Array.from(d.xfrc_applied.slice(16*6)).every(x=>x===0));
 const reserved=++calls;record(journal,{event:'begin',evaluation:reserved,label});
 const t0=performance.now();mj.mj_forward(model,d);const ms=performance.now()-t0;callTimes.push(ms);p.readReactions();
 physical(p,cp);assert.deepEqual(Array.from(d.qpos),beforeQ);assert.deepEqual(Array.from(d.qvel),beforeV);assert.equal(d.time,beforeTime);
 const com=Array.from(d.subtree_com.slice(3,6)),F=force.slice(),H=cross(sub(point,com),force);
 const M=new mj.DoubleBuffer(27*27),jp=new mj.DoubleBuffer(81),jr=new mj.DoubleBuffer(81),jdot=new mj.DoubleBuffer(81),rdot=new mj.DoubleBuffer(81);
 let frictionRatio=0;const contactAcceleration=[];
 for(const c of p.reactions.points){
  const moment=cross(sub(c.position,com),c.forceWorld);for(let k=0;k<3;k++){F[k]+=c.forceWorld[k];H[k]+=moment[k];}
  const normal=dot(c.forceWorld,c.normal),tangent=sub(c.forceWorld,c.normal.map(x=>x*normal));
  frictionRatio=Math.max(frictionRatio,norm(tangent)/(c.friction[0]*normal));
  mj.mj_jac(model,d,jp,jr,c.position,c.body);mj.mj_jacDot(model,d,jdot,rdot,c.position,c.body);
  const J=jp.GetView(),D=jdot.GetView(),a=[0,1,2].map(k=>dot(Array.from(J.slice(27*k,27*k+27)),d.qacc)+dot(Array.from(D.slice(27*k,27*k+27)),d.qvel));
  contactAcceleration.push({body:c.body,position:c.position,acceleration:a,normalAcceleration:dot(a,c.normal),velocity:[0,1,2].map(k=>dot(Array.from(J.slice(27*k,27*k+27)),d.qvel))});
 }
 mj.mj_fullM(model,d,M);mj.mj_jac(model,d,jp,jr,point,15);const matrix=M.GetView(),Jgrab=jp.GetView(),residual=[];
 for(let i=0;i<27;i++){let r=d.qfrc_bias[i]-d.qfrc_passive[i]-d.qfrc_actuator[i]-d.qfrc_constraint[i]-d.qfrc_applied[i];for(let k=0;k<27;k++)r+=matrix[27*i+k]*d.qacc[k];for(let k=0;k<3;k++)r-=Jgrab[27*k+i]*force[k];residual.push(r);}
 const applied=joints.map(j=>d.qfrc_actuator[j.v]),expected=joints.map(j=>u[j.a]*j.gear),mappingError=maxabs(sub(applied,expected));
 assert(d.qfrc_actuator.slice(0,6).every(x=>x===0));assert(d.qfrc_applied.every(x=>x===0));assert(mappingError<1e-10);
 const a=F.map((x,i)=>x/mass-(i===2?g:0)),loads=[p.reactions.normalLoads[7]||0,p.reactions.normalLoads[10]||0],output=[...a,...H,...loads];
 const contacts=d.contact,signature=[];for(let i=0;i<d.ncon;i++){const c=contacts.get(i);signature.push([c.geom1,c.geom2,c.dim,...c.pos]);c.delete();}contacts.delete();
 const efcState=Array.from(d.efc_state).slice(0,d.nefc),active=p.reactions.points.map(c=>[c.body,...c.position]);
 const result={label,output,comAcceleration:a,angularMomentumRate:H,loads,com,point,force,contacts:p.reactions.points,contactAcceleration,qacc:Array.from(d.qacc),appliedNm:applied,controls:u,baseResidual:residual.slice(0,6),allDynamicsResidualMax:maxabs(residual),frictionRatio,mappingError,contactSignature:signature,activeContacts:active,efcState,evaluationMs:ms};
 assert(output.every(Number.isFinite));records.push(result);
 record(journal,{event:'end',evaluation:reserved,label,output,baseResidualMax:maxabs(result.baseResidual),frictionRatio,evaluationMs:ms,efcStateHash:sha(JSON.stringify(efcState))});
 for(const b of[M,jp,jr,jdot,rdot])b.delete();dispose(p);assert.equal(JSON.stringify(cp),frozen);return result;
}
let final;
try{
 // Existing physical guard is evaluated on terminal002, with kinematics only.
 const p=scratch(terminal);Object.assign(p,{fallen:false,balance:{tilt:Math.acos(Math.max(-1,Math.min(1,p.data.xmat[17])))},reactions:structuredClone(terminal.physics.reactions)});
 const controller=Object.create(PullStep.prototype);Object.assign(controller,structuredClone(terminal.plan),{p});
 const result=controller.step(terminal.physics.balance.com,terminal.physics.balance.velocity,[7,10].map(b=>Array.from(p.data.xpos.slice(3*b,3*b+3))));
 assert.equal(result,null);assert.equal(controller.reason,'torso tilt exceeded 20 degrees');assert.equal(controller.phase,'failed');assert.deepEqual(nativeState(p,terminal.spec),terminal.state);physical(p,terminal);
 const terminalGuard={rejected:true,reason:controller.reason,tiltRad:p.balance.tilt,forwardEvaluations:0,state233Unchanged:true};dispose(p);
 const baseline=evaluate('baseline',u0);
 const h=baseline.com[2]-baseline.contacts.reduce((s,c)=>s+c.position[2]*c.load,0)/baseline.loads.reduce((a,b)=>a+b,0);assert(h>.3);
 const scales=[g,g,g,mass*g*h,mass*g*h,mass*g*h,mass*g,mass*g];
 const target=[baseline.output[0],old.target.ayAfterCopProjection,baseline.output[2],0,baseline.output[4],baseline.output[5],old.target.right,old.target.left];
 const factors=scales.map((s,i)=>PROTOCOL.weights[i]/s),scaled=(v)=>v.map((x,i)=>x*factors[i]);
 record(journal,{event:'frozen-objective',target,scales,weights:PROTOCOL.weights,lambda:PROTOCOL.lambda,maxControlDelta:PROTOCOL.maxControlDelta});
 const jac=Array.from({length:8},()=>Array(21).fill(0)),curvature=[];
 for(let j=0;j<21;j++){
  const minus=u0.slice(),plus=u0.slice();minus[j]-=PROTOCOL.inputEpsilon;plus[j]+=PROTOCOL.inputEpsilon;
  const lo=evaluate('minus-'+j,minus),hi=evaluate('plus-'+j,plus);
  for(let k=0;k<8;k++)jac[k][j]=(hi.output[k]-lo.output[k])/(2*PROTOCOL.inputEpsilon)*factors[k];
  const mid=scaled(baseline.output.map((v,k)=>(hi.output[k]+lo.output[k])/2-v)),span=scaled(sub(hi.output,lo.output)).map(x=>x/2);
  curvature.push({joint:j,error:norm(mid),limit:Math.max(PROTOCOL.guards.midpointCurvatureAbsolute,PROTOCOL.guards.midpointCurvatureRelative*norm(span))});
 }
 const rhs=scaled(sub(target,baseline.output)),normal=Array(441).fill(0),b=Array(21).fill(0);
 for(let i=0;i<21;i++){b[i]=jac.reduce((s,row,k)=>s+row[i]*rhs[k],0);for(let j=0;j<21;j++)normal[21*i+j]=jac.reduce((s,row)=>s+row[i]*row[j],0)+(i===j?PROTOCOL.lambda:0);}
 const mat=new mj.DoubleBuffer(441),solution=new mj.DoubleBuffer(21);mat.GetView().set(normal);
 const factorRank=mj.mju_cholFactor(mat,1e-12);assert.equal(factorRank,21);mj.mju_cholSolve(solution,Array.from(mat.GetView()),b);const direction=Array.from(solution.GetView());mat.delete();solution.delete();
 assert(direction.every(Number.isFinite));let fraction=1;
 for(let i=0;i<21;i++)if(direction[i]!==0)fraction=Math.min(fraction,PROTOCOL.maxControlDelta/Math.abs(direction[i]),(direction[i]>0?1-u0[i]:1+u0[i])/Math.abs(direction[i]));
 assert(fraction>=0&&fraction<=1);const delta=direction.map(x=>x*fraction),controls=u0.map((x,i)=>x+delta[i]);
 const prediction=baseline.output.map((v,k)=>v+dot(jac[k],delta)/factors[k]);
 const candidate=evaluate('candidate',controls);assert.equal(calls,44);
 const error=norm(scaled(sub(candidate.output,prediction))),predictedChange=norm(scaled(sub(prediction,baseline.output)));
 const linearErrorLimit=Math.max(PROTOCOL.guards.linearErrorAbsolute,PROTOCOL.guards.linearErrorRelative*predictedChange);
 const dynamicsPassed=records.every(r=>maxabs(r.baseResidual)<=PROTOCOL.guards.baseResidual&&r.allDynamicsResidualMax<=PROTOCOL.guards.baseResidual);
 const capsPassed=records.every(r=>r.appliedNm.every((x,i)=>Number.isFinite(x)&&Math.abs(x)<=Math.abs(joints[i].gear)+PROTOCOL.guards.capTolerance));
 const frictionPassed=records.every(r=>r.frictionRatio<=1+PROTOCOL.guards.frictionTolerance);
 const sameRegime=records.every(r=>JSON.stringify(r.contactSignature)===JSON.stringify(baseline.contactSignature)&&JSON.stringify(r.efcState)===JSON.stringify(baseline.efcState)&&JSON.stringify(r.activeContacts)===JSON.stringify(baseline.activeContacts));
 const baselineTaskCost=dot(rhs,rhs),actualError=scaled(sub(target,candidate.output)),candidateTaskCost=dot(actualError,actualError);
 const guards={finiteAndMapping:records.every(r=>r.mappingError<1e-10),caps:capsPassed,base:dynamicsPassed,friction:frictionPassed,
  retainedSupport:candidate.loads[0]>=PROTOCOL.guards.minimumRetainedN&&candidate.loads[1]>=PROTOCOL.guards.minimumOtherN&&candidate.contacts.every(c=>[7,10].includes(c.body)),
  lateralSign:candidate.output[1]<0&&candidate.output[1]<baseline.output[1]-PROTOCOL.guards.lateralImprovement,
  rollMoment:Math.abs(candidate.output[3])<=Math.abs(baseline.output[3])+PROTOCOL.guards.rollNonworseningNm,
  taskImproved:candidateTaskCost<baselineTaskCost,linearization:error<=linearErrorLimit,
  contactRegime:sameRegime,curvature:curvature.every(x=>x.error<=x.limit),terminal002:terminalGuard.rejected,sourceUnchanged:JSON.stringify(cp)===frozen};
 final={schema:1,readiness:Object.values(guards).every(Boolean),scope:'one fixed-state bounded motor candidate only; no walking or dynamic trial qualification',guards,rejections:Object.entries(guards).filter(([,v])=>!v).map(([k])=>k),
  protocol:PROTOCOL,inputHash:PROTOCOL.input001,modelSHA256:modelHash,engine:mj.mj_versionString(),counts:{baseline:1,perturbations:42,candidate:1,total001:calls,terminal002Forward:0,integrations:0,forecasts:0,confirmedMuJoCo:228794,historicalUnknownPreserved:true,Rapier:45735},
  target,scales,weights:PROTOCOL.weights,lambda:PROTOCOL.lambda,baseline,candidate,joints,
  pendingToLastAppliedMaxNm:maxabs(cp.physics.command.map((c,i)=>c.applied-u0[i]*joints[i].gear)),
  leastSquares:{jacobianScaled:jac,direction,commonBoundFraction:fraction,delta,linearPrediction:prediction,linearError:error,linearErrorLimit,predictedChange,baselineTaskCost,candidateTaskCost,regularizationCost:PROTOCOL.lambda*dot(delta,delta),normalEquationResidual:maxabs(normal.reduce((a,_,i)=>i%21?a:(a.push(dot(normal.slice(i,i+21),direction)-b[i/21]),a),[])),curvature},
  terminalGuard,cost:{nativeEvaluationsMs:callTimes.reduce((a,b)=>a+b,0),maxEvaluationMs:Math.max(...callTimes),elapsedMs:performance.now()-start}};
}catch(error){final={readiness:false,error:String(error),stack:error.stack,evaluationsAttempted:calls,integrations:0,forecasts:0,scope:'preparation or bounded pure check failed; no retry in this invocation'};process.exitCode=1;}
finally{model.delete();}
record(folder+'/result.json',final);console.log(JSON.stringify({readiness:final.readiness,error:final.error,guards:final.guards,rejections:final.rejections,counts:final.counts,baseline:final.baseline?.output,candidate:final.candidate?.output,cost:final.cost,evidenceBytes:bytes()}));
