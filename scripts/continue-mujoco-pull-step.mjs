// Exclusive, prospective ONE-run protocol. No forecasts or automatic retries.
import fs from 'node:fs';import assert from 'node:assert/strict';import {gzipSync} from 'node:zlib';import {execFileSync} from 'node:child_process';import crypto from 'node:crypto';
import load from '../web/dist/vendor/mujoco.js';import {Physics} from '../web/dist/engine.js';import {PULL_PROTOCOL as c,smooth} from '../web/dist/pull-step.js';
import{restorePullEntry,nativeState,EXPECTED}from'./pull-step-restoration.mjs';
const out=process.argv[2];assert(out&&!fs.existsSync(out),'Require a NEW proof directory; an existing attempt must never be rerun');
const sha=x=>crypto.createHash('sha256').update(x).digest('hex'),bytes=()=>Number(execFileSync('du',['-sx','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0]);
const baseline=419647488,parentGrowth=13123543,diskBefore=bytes(),aggregateBefore=diskBefore-baseline+parentGrowth;
assert(aggregateBefore+2_000_000<1_530_000_000,'Disk reserve');
for(const f of ['engine.js','pull-step.js'])assert(fs.readFileSync('web/dist/'+f).equals(fs.readFileSync('web/mujoco/'+f)),'stale dist '+f);
const previous=JSON.parse(fs.readFileSync('web/proofs/pull-step-001/ledger.json'));assert.equal(previous.confirmedTotal,228054);assert(previous.completed&&previous.pendingMaxTicks===0);
fs.mkdirSync(out);const lock=fs.openSync(out+'/started.json','wx');fs.writeSync(lock,JSON.stringify({pid:process.pid,source:execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim(),protocol:c,aggregateBefore,wallStart:new Date().toISOString()}));fs.closeSync(lock);
const ledger={confirmedBefore:228054,priorUnknownPreserved:true,rapierFrozen:45735,newTicks:0,pendingMaxTicks:0,completed:false,phase:'preflight',protocol:c,aggregateBefore};
const save=()=>{fs.writeFileSync(out+'/ledger.tmp',JSON.stringify(ledger));fs.renameSync(out+'/ledger.tmp',out+'/ledger.json');};save();
const mj=await load(),xml=fs.readFileSync('web/dist/humanoid.xml','utf8'),restoration=restorePullEntry(mj),{p,cp,path,forceTransition}=restoration,obs=new mj.MjData(p.model),contactSpec=cp.spec;
const startSteps=p.steps,maxNewTicks=4605;assert.equal(startSteps,8195);assert.deepEqual(nativeState(p,contactSpec),cp.state);
ledger.restoration={inputSHA256:EXPECTED,native233Exact:true,forceTransition,startSteps,startTime:p.data.time,deadline:16,releaseTime:13.8,remainingTicks:maxNewTicks,entryPathResidual:path.residual,maxReferenceSpeedBound:path.maxSpeed};
const buffer=new mj.DoubleBuffer(mj.mj_stateSize(p.model,contactSpec));
const proofPath=out+'/trace.ndjson.gz';fs.closeSync(fs.openSync(proofPath,'wx'));let previousCom=previous.summary.last.com.slice(),grabStart=restoration.grabStart,phaseLast=null,stop='budget',last=null,interacted=true,maxRatio=0,maxTilt=0,minimumLoad=Infinity,maxClear=-Infinity,poseRows=0,calm=0,slip=0,feetAtGrab=previous.summary.last.feet.map(f=>f.slice());
const rounded=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='number'?Math.round(v*1e7)/1e7:v));
function measure(){
 obs.qpos.set(p.data.qpos);mj.mj_kinematics(p.model,obs);mj.mj_comPos(p.model,obs);
 const com=Array.from(obs.subtree_com.slice(3,6)),velocity=previousCom?com.map((x,i)=>(x-previousCom[i])/p.dt):[0,0,0];previousCom=com;
 const geoms=Physics.prototype.geometries.call({model:p.model,data:obs,names:p.names}),feet=[7,10].map(b=>Array.from(obs.xpos.slice(b*3,b*3+3))),clear=[7,10].map(b=>Math.min(...geoms.filter(g=>g.body===b).map(g=>Math.min(g.a[2],g.b[2])-g.radius))),loads=[p.reactions.normalLoads[7]||0,p.reactions.normalLoads[10]||0],tilt=Math.acos(Math.max(-1,Math.min(1,obs.xmat[17])));
 return {t:p.data.time,ticks:p.steps,phase:p.pullStep.phase,com,velocity,feet,clear,loads,tilt,geoms,force:p.lastForceVector||[0,0,0],grab:p.grab?{body:p.grab.body,point:p.worldPoint(p.grab.body,p.grab.local),target:p.grab.target}:null,q:Array.from(p.data.qpos.slice(7)),dq:Array.from(p.data.qvel.slice(6)),shape:p.pullStep.qref?.slice(7)||null,servo:p.pullStep.reference?.q.slice(7)||null,servoLimitResidual:p.pullStep.servoLimitResidual||[],loadShare:p.pullStep.loadShare,command:p.command.map((j,i)=>[j.requested,j.applied,p.data.qfrc_actuator[p.joints[i].v]]),reactionsAt:p.reactions.atTime,geometryAt:p.data.time,controller:p.pullStep.snapshot()};
}
function sample(row){
 fs.appendFileSync(proofPath,gzipSync(JSON.stringify(rounded(row))+'\n',{level:6}));poseRows++;assert(fs.statSync(proofPath).size<1_500_000,'Trace reserve exceeded');save();
}
try{
 assert.equal(p.model.nu,21);assert.equal(p.dt,.00125);assert.equal(p.model.nq,28);assert.equal(p.model.nv,27);
 assert.equal(sha(xml),'7a6e3dfc83a9e30bebe81e3e8c6e6b146f8447d1dbfaacd83de90f4937ea5765');
 ledger.phase='restored-transfer';ledger.joints=p.joints.map(j=>({name:j.name,gear:j.gear,q:j.q,v:j.v}));sample(measure());
 for(let k=0;k<maxNewTicks;k++){
  const i=p.steps;
  if(k%20===0){ledger.pendingMaxTicks=Math.min(20,maxNewTicks-k);save();}
  if(i<11040){const s=smooth((i-8000)*p.dt/c.rampSeconds);p.moveGrab([grabStart[0]+c.targetOffsetX*s,grabStart[1],grabStart[2]]);}
  if(i===11040){p.release();ledger.releaseTime=p.data.time;}
  if(p.step()===false){stop=p.pullStep.reason||'controller terminal before integration';ledger.pendingMaxTicks=0;save();break;}
  ledger.newTicks++;ledger.pendingMaxTicks--;
  last=measure();
  assert.equal(p.steps,startSteps+ledger.newTicks);assert(p.data.qfrc_applied.every(x=>x===0));assert(p.data.qfrc_actuator.slice(0,6).every(x=>x===0));
  for(let b=0;b<p.model.nbody;b++)if(b!==c.body)assert(p.data.xfrc_applied.slice(6*b,6*b+6).every(x=>x===0),'Unexpected external body force');
  for(const j of p.joints){const actual=p.data.qfrc_actuator[j.v];maxRatio=Math.max(maxRatio,Math.abs(actual/j.gear));assert(Math.abs(actual)<=Math.abs(j.gear)+1e-8);assert(Math.abs(actual-p.data.ctrl[j.a]*j.gear)<1e-8);}
  if(i>=8000){maxTilt=Math.max(maxTilt,last.tilt);minimumLoad=Math.min(minimumLoad,last.loads[1]);maxClear=Math.max(maxClear,last.clear[1]);slip=Math.max(slip,Math.hypot(...[0,1].map(k=>last.feet[0][k]-feetAtGrab[0][k])));}
  calm=!p.grab&&last.loads.every(x=>x>30)&&last.tilt<.2&&Math.hypot(...last.velocity)<.03?calm+p.dt:0;
  const terminal=p.fallen||p.pullStep.phase==='failed'||p.pullStep.phase==='complete'||p.pullStep.phase==='cancelled';
  if(p.steps%20===0||last.phase!==phaseLast||terminal){sample(last);phaseLast=last.phase;}
  if(p.steps>=800&&p.reactions.supportedBodies.some(b=>![7,10].includes(b))){stop='loaded non-foot contact';break;}
  if(p.fallen||last.tilt>c.tiltMaxRad&&i>=800){stop='physical fall/tilt';break;}
  if(terminal){stop=p.pullStep.reason||p.pullStep.phase;break;}
 }
}catch(e){stop=String(e);ledger.error=String(e);}
finally{
 ledger.pendingMaxTicks=0;ledger.newTicks=p.steps-startSteps;ledger.confirmedTotal=ledger.confirmedBefore+ledger.newTicks;ledger.completed=true;ledger.stop=stop;ledger.phase=p.pullStep.phase;
 ledger.summary={passed:p.pullStep.complete&&calm>=1,interacted,actualSeconds:p.data.time,poseRows,minimumFreeLoadN:Number.isFinite(minimumLoad)?minimumLoad:null,maxWholeFreeFootClearanceM:Number.isFinite(maxClear)?maxClear:null,maxRetainedFootDriftM:slip,maxTiltRad:maxTilt,maxActualTorqueRatio:maxRatio,finalCalmSeconds:calm,controller:p.pullStep.snapshot(),last:last?{...last,geoms:undefined}:null};
 // Save native state before cleanup. Restoration check is pure on a new MjData,
 // no reset or replay of any part of this physical run.
 mj.mj_getState(p.model,p.data,buffer,contactSpec);const state=Array.from(buffer.GetView()),restored=new mj.MjData(p.model);mj.mj_setState(p.model,restored,state,contactSpec);mj.mj_getState(p.model,restored,buffer,contactSpec);assert.deepEqual(Array.from(buffer.GetView()),state);restored.delete();
 const checkpoint={engine:mj.mj_versionString(),modelSHA256:sha(xml),spec:contactSpec,state,physics:p.state(),previousCom:p.previousCom,grab:p.grab,forceTransition,plan:p.pullStep?Object.fromEntries(Object.entries(p.pullStep).filter(([k])=>!['p','ik','path'].includes(k))):null,path:p.pullStep?.path?{...p.pullStep.path,at:undefined}:null};
 const native=gzipSync(JSON.stringify(checkpoint));fs.writeFileSync(out+'/final.json.gz',native);ledger.finalCheckpointSHA256=sha(native);ledger.restoreExact=true;
 if(last)sample(last);ledger.proofBytes=fs.readdirSync(out).reduce((n,f)=>n+fs.statSync(out+'/'+f).size,0);assert(ledger.proofBytes<1_600_000,'Reserve for visual proof');ledger.diskAfter=bytes();ledger.aggregateAfter=ledger.diskAfter-baseline+parentGrowth;assert(ledger.aggregateAfter<1_530_000_000);save();
 console.log(JSON.stringify({stop,ticks:ledger.newTicks,scenarioTicks:p.steps,confirmedTotal:ledger.confirmedTotal,summary:ledger.summary,checkpoint:ledger.finalCheckpointSHA256,aggregateAfter:ledger.aggregateAfter}));
 p.dispose();obs.delete();buffer.delete();
}
