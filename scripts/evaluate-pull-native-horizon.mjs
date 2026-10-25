// Persistent MuJoCo3.17 shooting evaluator. Every mj_step is a counted forecast.
import fs from 'node:fs';import readline from 'node:readline';import assert from 'node:assert/strict';import crypto from 'node:crypto';import {gzipSync,gunzipSync} from 'node:zlib';import {execFileSync} from 'node:child_process';
import load from '../web/dist/vendor/mujoco.js';import {Physics} from '../web/dist/engine.js';
import {PLAN,command,grabInput,terminal,tests,velocityIdentity} from './pull-native-horizon.mjs';
import {nativeState} from './pull-step-restoration.mjs';import {accumulateInteraction} from './pull-external-force.mjs';import {eulerAcceleration} from './pull-native-step-map.mjs';
import {worldFeatures} from './pull-dynamic-stance.mjs';import {inv,sub,norm} from './pull-stance-geometry.mjs';
const out=process.argv[2];assert(out&&fs.existsSync(out+'/protocol.json')&&!fs.existsSync(out+'/worker-ledger.json'),'New prepared campaign only');
const source='web/proofs/pull-gravity-frame-001/final.json.gz',sha=b=>crypto.createHash('sha256').update(b).digest('hex'),bytes=fs.readFileSync(source),cp=JSON.parse(gunzipSync(bytes)),xml=fs.readFileSync('web/dist/humanoid.xml','utf8');
assert.equal(sha(bytes),'3bf174a791649512792b8dea38acbacb54094e509d75b526d26ea43dc1311304');assert.equal(sha(xml),cp.modelSHA256);
assert.equal(execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),'e066a3b8557f82ab2ed81f9b9380b0a4f86dab50');
const growth=()=>Number(execFileSync('du',['-sx','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488+13123543;
assert(growth()+500000<PLAN.maxGrowth,'Global reserve');
const ledger={evaluations:0,predictiveSteps:0,pendingPredictiveSteps:0,realTicks:0,explicitForwards:0,confirmedBefore:229565,unknownHistoryPreserved:true,completed:false,sourceSHA256:sha(bytes),growthBefore:growth(),nativeMs:0,kinematicPasses:0};
const proofBytes=()=>fs.readdirSync(out).reduce((s,n)=>s+fs.statSync(out+'/'+n).size,0);
function save(name,x){const b=gzipSync(JSON.stringify(x),{level:9});assert(proofBytes()+b.length<350000,'Compact evidence reserve');fs.writeFileSync(out+'/'+name,b);}
function persist(){fs.writeFileSync(out+'/worker-ledger.tmp',JSON.stringify(ledger));fs.renameSync(out+'/worker-ledger.tmp',out+'/worker-ledger.json');}
persist();const pure=tests(),engine=await load();
const mj=new Proxy(engine,{get(o,k){const f=o[k];if(['mj_forward','mj_forwardSkip','mj_inverse','mj_step1','mj_step2','mjd_transitionFD','mjd_inverseFD'].includes(k))return()=>{throw Error('Undeclared native entry point '+k);};return f;}});
const model=mj.MjModel.from_xml_string(xml),joints=Array.from({length:model.nu},(_,a)=>{const j=model.actuator_trnid[2*a];return {a,j,q:model.jnt_qposadr[j],v:model.jnt_dofadr[j],name:mj.mj_id2name(model,mj.mjtObj.mjOBJ_JOINT.value,j),gear:model.actuator_gear[6*a]};}),names=Array.from({length:model.nbody},(_,i)=>mj.mj_id2name(model,mj.mjtObj.mjOBJ_BODY.value,i));
assert.equal(mj.mj_versionString(),cp.engine);assert.equal(model.nv,27);assert.equal(model.nu,21);assert.equal(model.opt.timestep,PLAN.dt);assert.equal(model.opt.integrator.value??model.opt.integrator,0);assert.equal(cp.state.length,233);
assert(cp.experimental.reference.recoveryDone&&Object.keys(cp.experimental.reference.closed).length===2);assert(!cp.experimental.applicationStatus.nativeIntegrationPending);assert(!cp.experimental.applicationStatus.pendingValidatedControls);
const u0=cp.experimental.applicationStatus.lastAppliedControls.slice(),Rg=cp.experimental.reference.equilibriumFrame?.Rg??JSON.parse(gunzipSync(fs.readFileSync('web/proofs/pull-gravity-frame-001/last-qp.json.gz'))).gravity.Rg;
const work=Object.create(Physics.prototype);Object.assign(work,{mj,model,joints,names,data:new mj.MjData(model),contactBuffer:new mj.DoubleBuffer(6),steps:cp.physics.steps});assert.equal(work.dt,PLAN.dt);
const velocityBuffer=new mj.DoubleBuffer(6);
const pointJP=new mj.DoubleBuffer(81),pointJR=new mj.DoubleBuffer(81);
function kin(){mj.mj_kinematics(model,work.data);mj.mj_comPos(model,work.data);mj.mj_comVel(model,work.data);mj.mj_subtreeVel(model,work.data);ledger.kinematicPasses++;}
function restore(){mj.mj_setState(model,work.data,cp.state,cp.spec);assert.deepEqual(nativeState(work,cp.spec),cp.state);work.grab=structuredClone(cp.grab);work.steps=cp.physics.steps;kin();assert.deepEqual(nativeState(work,cp.spec),cp.state);assert.deepEqual(work.grab,cp.grab);assert.deepEqual(Array.from(work.data.ctrl),u0);}
function bodyVelocity(body){mj.mj_objectVelocity(model,work.data,mj.mjtObj.mjOBJ_BODY.value,body,velocityBuffer,0);return Array.from(velocityBuffer.GetView());}
function measure(reactions){const d=work.data,com=Array.from(d.subtree_com.slice(3,6)),stance=Array.from(d.xpos.slice(21,24)),tv=bodyVelocity(1),velocity=Array.from(d.subtree_linvel.slice(3,6)),geoms=work.geometries();mj.mj_jac(model,d,pointJP,pointJR,stance,7);const jp=pointJP.GetView(),sv=[0,1,2].map(k=>Array.from(jp.slice(k*27,k*27+27)).reduce((s,x,i)=>s+x*d.qvel[i],0));return {t:d.time,com,velocity,stance,stanceVelocity:sv,relative:inv(Rg,sub(com,stance)),relativeVelocity:inv(Rg,sub(velocity,sv)),tilt:Math.acos(Math.max(-1,Math.min(1,d.xmat[17]))),omega:tv.slice(0,3),torsoHeight:d.xpos[5],up:d.xmat[17],loads:[reactions.normalLoads[7]||0,reactions.normalLoads[10]||0],geometryAt:d.time,reactionsAt:reactions.atTime,clear:[7,10].map(b=>Math.min(...geoms.filter(g=>g.body===b).map(g=>Math.min(g.a[2],g.b[2])-g.radius)))};}
function pose(m){return {...m,positions:Array.from(work.data.xpos),rotations:Array.from(work.data.xquat)};}
let best=null,inFlight=null;
try{
 restore();const restored=nativeState(work,cp.spec),int=grabInput(work,cp);assert.deepEqual(int,cp.experimental.pendingInput,'Exact first grab including velocity history');
 const a=new mj.MjData(model),b=new mj.MjData(model);mj.mj_setState(model,a,cp.state,cp.spec);mj.mj_setState(model,b,cp.state,cp.spec);a.xfrc_applied.fill(0);b.xfrc_applied.fill(0);accumulateInteraction(a,15,int);const tmp=work.data;work.data=b;kin();work.apply(15,int.point,int.force);work.data=tmp;assert.deepEqual(Array.from(a.xfrc_applied),Array.from(b.xfrc_applied));a.delete();b.delete();
 const features=worldFeatures(work),initial=measure(cp.lastResolvedReactions),arithmetic=128*Number.EPSILON;
 assert(Math.max(...initial.velocity.map((x,i)=>Math.abs(x-features.com.v[i])))<arithmetic);assert(Math.max(...initial.omega.map((x,i)=>Math.abs(x-features.torso.w[i])))<arithmetic);
 assert.deepEqual(initial.stanceVelocity,features.stance.v);
 assert.deepEqual(nativeState(work,cp.spec),restored);assert.deepEqual(work.grab,cp.grab);assert(joints.every(j=>j.v>=6&&j.gear>0));
 const ranges=joints.map(j=>[model.jnt_range[2*j.j],model.jnt_range[2*j.j+1]]);assert(ranges.every(([l,h])=>h>l));
 const declaration={...PLAN,source,time:cp.physics.time,sourceSHA256:sha(bytes),modelSHA256:cp.modelSHA256,u0,joints,ranges,Rg,initial,closed:cp.experimental.reference.closed,clocks:{recoveredAt:cp.experimental.reference.recoveredAt,transferStart:cp.experimental.reference.start,transferDeadline:cp.experimental.reference.transferDeadline,absoluteEnd:cp.experimental.absoluteEnd},firstGrab:int,objectiveNormalization:'400*21, normalized motor controls (not mechanical work)',physicalConstraintsScaledBy:'existing angle20deg, drift25mm, dwell25ms, actual joint range; terminal5mm/.05mps/20deg/.2radps. Positive scaling only; no tolerance relaxation.'};
 save('pure.json.gz',{...pure,native233Exact:true,grabExact:true,externalArithmeticIdentical:true,velocityAPIvsJacobian:true,rootActuatorAbsent:true,sourcePreserved:true});save('declaration.json.gz',declaration);persist();
 process.stdout.write(JSON.stringify({ready:true,pure,declaration})+'\n');
 for await(const line of readline.createInterface({input:process.stdin,crlfDelay:Infinity})){
  const request=JSON.parse(line);if(request.action==='finish'){ledger.completed=true;ledger.reason=request.reason;ledger.growthAfter=growth();ledger.bestEvaluation=best?.index??null;assert.equal(sha(fs.readFileSync(source)),ledger.sourceSHA256);persist();process.stdout.write(JSON.stringify({finished:true,ledger,best:best?.summary??null})+'\n');break;}
  assert(request.action==='evaluate');assert(ledger.evaluations<PLAN.maxEvaluations&&ledger.predictiveSteps+400<=PLAN.maxPredictiveSteps);assert(Date.now()<request.deadlineEpochMs,'Campaign deadline');
  const x=request.x;assert(x.length===42&&x.every(v=>Number.isFinite(v)&&Math.abs(v)<=1),'Input outside original motor caps');restore();assert.deepEqual(grabInput(work,cp),int);
  const index=++ledger.evaluations,beforeSteps=ledger.predictiveSteps,frames=[pose(initial)],constraints=new Map(),margins=new Map(),firstViolations={};let effort=0,lost=cp.experimental.lostSeconds,maxLost=lost,maxJump=0,prior=u0.slice(),last=initial,maxVE=0,maxQE=0,minRight=Infinity,minLeft=Infinity,maxTilt=initial.tilt,maxDrift=0,atCapSamples=Array(21).fill(0);
  inFlight={index,x,u0,frames,initial,sourceSHA256:sha(bytes),label:'INCOMPLETE native prediction, never applied live',partial:true};
  function constrain(label,value,raw){const old=constraints.get(label);if(old===undefined||value<old){constraints.set(label,value);margins.set(label,raw);}if(value<0&&!firstViolations[label])firstViolations[label]={step:work.steps-cp.physics.steps,t:work.data.time,value,raw};}
  for(let n=0;n<400;n++){
   assert(Date.now()<request.deadlineEpochMs,'Campaign wall deadline during prediction');assert(ledger.predictiveSteps<PLAN.maxPredictiveSteps);const d=work.data,u=command(u0,x,n*PLAN.dt),interaction=grabInput(work,cp),q0=Array.from(d.qpos),v0=Array.from(d.qvel),t0=d.time;
   assert(u.every(v=>Number.isFinite(v)&&Math.abs(v)<=1),'Interpolation cap inconsistency');for(let j=0;j<21;j++){effort+=u[j]*u[j]/(400*21);maxJump=Math.max(maxJump,Math.abs((u[j]-prior[j])*joints[j].gear));if(Math.abs(u[j])>=1-1e-10)atCapSamples[j]++;}prior=u;
   Object.assign(inFlight,{attemptedStep:n+1,beforeQ:q0,beforeV:v0,beforeTime:t0,controls:u,interaction,lastMeasured:last,firstViolations});
   d.ctrl.set(u);d.qfrc_applied.fill(0);d.xfrc_applied.fill(0);accumulateInteraction(d,15,interaction);work.grab.previous=interaction.point.slice();ledger.pendingPredictiveSteps=1;persist();const clock=performance.now();mj.mj_step(model,d);ledger.nativeMs+=performance.now()-clock;ledger.predictiveSteps++;work.steps++;ledger.pendingPredictiveSteps=0;
   assert([...d.qpos,...d.qvel,...d.qacc].every(Number.isFinite),'Native nonfinitude');assert(Math.abs(d.time-(t0+PLAN.dt))<=arithmetic*Math.max(1,Math.abs(d.time)),'Native time reset/inconsistency');
   assert(d.qfrc_actuator.slice(0,6).every(v=>v===0)&&d.qfrc_applied.every(v=>v===0),'Root/generalized assistance');
   for(const [j,k]of joints.entries()){const expected=u[j]*k.gear,actual=d.qfrc_actuator[k.v];assert(Math.abs(actual-expected)<=arithmetic*Math.max(1,Math.abs(expected)),'Actual motor mapping mismatch');constrain('motor-'+k.name,(k.gear-Math.abs(actual))/k.gear,{applied:actual,cap:k.gear});}
   // qacc/M are the pre-integration solved fields; damping is implicit in Euler.
   const effective=eulerAcceleration(mj,model,d),velocityChecks=velocityIdentity(v0,Array.from(d.qvel),effective);
   Object.assign(inFlight,{effective,qacc:Array.from(d.qacc),afterQ:Array.from(d.qpos),afterV:Array.from(d.qvel),velocityChecks});
   // Preserve complete diagnostic values before checking identity. No threshold change.
   save('last-integrated-diagnostic.json.gz',{...inFlight,frames:undefined});
   for(const c of velocityChecks){maxVE=Math.max(maxVE,c.err);assert(c.err<=c.bound,'Euler velocity inconsistency');}
   for(const k of joints){const expected=q0[k.q]+PLAN.dt*d.qvel[k.v],err=Math.abs(expected-d.qpos[k.q]);maxQE=Math.max(maxQE,err);assert(err<=arithmetic*Math.max(1,Math.abs(expected)),'Euler hinge-position inconsistency');}
   work.readReactions();work.reactions.atTime=t0;for(const p of work.reactions.points){assert(p.forceWorld.every(Number.isFinite)&&p.load>=0,'Invalid native reaction');const cf=work.reactions.supportedBodies.filter(b=>![7,10].includes(b));constrain('nonplantar load',cf.length?-1:1,{supportedBodies:cf});}
   if(!work.reactions.points.length)constrain('nonplantar load',1,{supportedBodies:[]});
   kin();last=measure(work.reactions);minRight=Math.min(minRight,last.loads[0]);minLeft=Math.min(minLeft,last.loads[1]);lost=last.loads[0]<1?lost+PLAN.dt:0;maxLost=Math.max(maxLost,lost);maxTilt=Math.max(maxTilt,last.tilt);const drift=norm(sub(last.stance,cp.experimental.initialFeet[0]).slice(0,2));maxDrift=Math.max(maxDrift,drift);
   constrain('torso tilt20deg',(20*Math.PI/180-last.tilt)/(20*Math.PI/180),{rad:last.tilt});constrain('retained foot drift25mm',(.025-drift)/.025,{m:drift});constrain('retained support lost25ms',(.025-lost)/.025,{seconds:lost,load:last.loads[0]});constrain('torso height .65m',(last.torsoHeight-.65)/.65,{m:last.torsoHeight});constrain('torso up .4',(last.up-.4)/.4,{up:last.up});
   for(const [i,k]of joints.entries()){const q=d.qpos[k.q],v=d.qvel[k.v],[lo,hi]=ranges[i],round=arithmetic*Math.max(1,Math.abs(q),Math.abs(v));constrain('ROM lower '+k.name,(q-lo+round)/(hi-lo),{q,v,lo,hi,round});constrain('ROM upper '+k.name,(hi-q+round)/(hi-lo),{q,v,lo,hi,round});}
   if((n+1)%20===0)frames.push(pose(last));
  }
  assert.equal(ledger.predictiveSteps-beforeSteps,400);const end=terminal(initial,last,Rg);for(const [label,value]of end.rows)constrain('terminal '+label,value,end);
  const rows=[...constraints].map(([label,value])=>({label,value,raw:margins.get(label)})),violations=rows.filter(x=>x.value<0),strictDwell=maxLost<.025,feasible=violations.length===0&&strictDwell,deficit=Math.max(0,...rows.map(x=>-x.value));
  const summary={index,steps:400,objective:effort,feasible,maxNormalizedViolation:deficit,violations,firstViolations,terminal:end,final:last,minRight,minLeft,maxTilt,maxDrift,maxLost,strictDwell,maxTorqueStepNm:maxJump,atCapSamples,maximumVelocityIdentityError:maxVE,maximumHingeIdentityError:maxQE,realTicks:0,predictiveSteps:ledger.predictiveSteps};
  if(!best||(feasible&&!best.summary.feasible)||(feasible===best.summary.feasible&&(deficit<best.summary.maxNormalizedViolation||(deficit===best.summary.maxNormalizedViolation&&effort<best.summary.objective)))){best={index,summary};save('best-trajectory.json.gz',{summary,x,u0,frames,finalState:nativeState(work,cp.spec),finalGrab:work.grab,closed:cp.experimental.reference.closed,originalPhaseClocks:declaration.clocks,sourceSHA256:sha(bytes),label:'PREDICTION native0.5s on disposable copy; never applied live',phase:'transfer prediction only; no phase/contact imposed'});}
  fs.appendFileSync(out+'/evaluations.ndjson',JSON.stringify({index,objective:effort,feasible,violation:deficit,terminal:end,firstFault:Object.entries(firstViolations)[0]??null})+'\n');ledger.bestEvaluation=best.index;persist();
  inFlight=null;
  process.stdout.write(JSON.stringify({evaluated:true,summary,constraints:rows.map(x=>x.value),labels:rows.map(x=>x.label)})+'\n');
 }
}catch(e){ledger.error=String(e);ledger.completed=false;ledger.uncertainPrediction=ledger.pendingPredictiveSteps!==0;persist();save('engine-failure.json.gz',{error:String(e),stack:e.stack,state:nativeState(work,cp.spec),grab:work.grab,ledger,best,inFlight});process.stdout.write(JSON.stringify({fatal:true,error:String(e),ledger})+'\n');process.exitCode=1;}
finally{pointJP.delete();pointJR.delete();velocityBuffer.delete();work.contactBuffer.delete();work.data.delete();model.delete();}
