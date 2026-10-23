// One continuous cycle. Reuses b9d2180 model loading, inference and audited stepping.
// The ONLY state installation is the saved WEAK standing entry, before this cycle.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{execFileSync}from'node:child_process';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{DenseActor,physicalObservation,ORDER}from'../web/mujoco/local-getup/slow.js';
const dir='web/proofs/getup-continuous-weak',sourceDir='web/proofs/getup-weak-seed10',run=process.argv.includes('--run'),hash=x=>crypto.createHash('sha256').update(x).digest('hex');
const json=p=>JSON.parse(fs.readFileSync(p)),write=(n,v)=>{fs.writeFileSync(`${dir}/${n}.tmp`,JSON.stringify(v));fs.renameSync(`${dir}/${n}.tmp`,`${dir}/${n}`);};
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
assert(disk()+16000000<1500000000,'16MB capture/video reserve does not fit');
if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson`),'Existing cycle or uncertain previous launch: never retry');
const prior=json(`${sourceDir}/ledger.json`),sourcePreflight=json(`${sourceDir}/preflight.json`),entryBytes=fs.readFileSync(`${sourceDir}/weak-final.json`),entry=JSON.parse(entryBytes);
assert(prior.completed&&prior.pendingMaxTicks===0&&prior.weak.passed);assert.equal(hash(entryBytes),'8b9ef9f76bd26ec5047c5b1d13ecbf4715820ef3c9a8324568dd7cdbc9473281');
assert.deepEqual(entry.qpos,prior.weak.final.qpos);assert.deepEqual(entry.qvel,prior.weak.final.qvel);assert.equal(entry.time,prior.weak.final.t);
const original=fs.readFileSync('rust/crates/model/data/getup-humanoid.xml','utf8'),xml=original.replace(/<include\b[^>]*\/>/g,'').replace(/\s+material="[^"]*"/g,'').replace('<option timestep=".00125"/>','<option timestep=".005" integrator="Euler"/>');assert.equal(hash(xml),sourcePreflight.weakXMLSHA256);
const mj=await load(),p=new Physics(mj,xml),d=p.data,m=p.model;assert.equal(mj.mj_versionString(),sourcePreflight.engine);assert.equal(p.dt,.005);assert.equal(m.nu,21);assert.equal(m.nv,27);p.joints.forEach((j,a)=>assert.equal(j.q-7,ORDER[a]));
assert.equal(mj.mj_stateSize(m,entry.spec),entry.state.length);mj.mj_setState(m,d,entry.state,entry.spec);
const buffer=new mj.DoubleBuffer(entry.state.length);mj.mj_getState(m,d,buffer,entry.spec);assert.deepEqual(Array.from(buffer.GetView()),entry.state);assert.deepEqual(Array.from(d.qpos),entry.qpos);assert.deepEqual(Array.from(d.qvel),entry.qvel);assert.equal(d.time,entry.time);
p.forward();p.readReactions();const initialObservation=physicalObservation(p),weight=Array.from(m.body_mass).reduce((a,b)=>a+b,0)*9.81;
assert(initialObservation[21]>1.4&&initialObservation[36]>Math.cos(Math.PI/6),'Entry must already be standing');
const binary=fs.readFileSync('rust/crates/sim/data/getup-policy/teacher.bin'),actor=new DenseActor(binary.buffer.slice(binary.byteOffset,binary.byteOffset+binary.byteLength)),power=.663420431;
const protocol={physicsHz:200,policyHz:40,power,passiveTicks:400,recoveryMaximumTicks:1250,holdingTicks:750,maximumCycleTicks:2400,preparationTicks:0,
 fall:'head <0.9m AND (torso vertical <0.5 OR nonfoot load >=3N), for 4 consecutive 40Hz samples (0.1s); must also hold at passive end',
 standing:'head >1.4m, torso tilt <30deg, feet load >=half body weight, no nonfoot load >=3N, for 10 consecutive 40Hz samples (0.25s); either/both feet permitted',
 hold:'3.75s WEAK only after standing admission; stop on persistent measured fall (same 0.1s predicate); success requires standing at end, no drift veto',
 integrity:'finite q/dq; |dq|<10000; gear*ctrl equality <=2e-4Nm; all21caps; no root actuation or external forces; time continuous'};
const preflight={protocol,engine:mj.mj_versionString(),entryPath:`${sourceDir}/weak-final.json`,entrySHA256:hash(entryBytes),modelSHA256:hash(xml),actorSHA256:hash(binary),sourceCommit:'b9d2180835e306fb6eb8baed9f65a16e168fca25',time:d.time,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),initialObservation,loads:p.reactions.normalLoads,gears:p.joints.map(j=>j.gear),weight,restoredIntegrationStateExactly:true,diskGrowth:disk()};
if(!run){write('preflight.json',preflight);buffer.delete();p.dispose();console.log(JSON.stringify({ready:true,preparationTicks:0,entryTime:entry.time,entrySHA256:preflight.entrySHA256,head:initialObservation[21],tiltDeg:Math.acos(initialObservation[36])*180/Math.PI,diskGrowth:preflight.diskGrowth,protocol}));process.exit(0);}
const saved=json(`${dir}/preflight.json`);assert.deepEqual(saved.protocol,protocol);assert.equal(saved.entrySHA256,preflight.entrySHA256);assert.equal(saved.modelSHA256,preflight.modelSHA256);assert.equal(saved.actorSHA256,preflight.actorSHA256);
const report={confirmedBefore:165444,priorUnknownPreserved:true,preparationTicks:0,newTicks:0,pendingMaxTicks:0,phase:'passive-fall',completed:false,passed:false,protocol,entrySHA256:preflight.entrySHA256,entryTime:entry.time,transitions:[],maxTorqueErrorNm:0,maxTorqueFraction:0};
const save=()=>write('ledger.json',report);save();const fd=fs.openSync(`${dir}/poses.ndjson`,'wx');
let phase=report.phase,fallCount=0,standCount=0,last=null;
function snapshot(){const o=physicalObservation(p);return {phase,t:d.time,cycleTime:d.time-entry.time,physical:o,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),com:Array.from(d.subtree_com.slice(3,6)),loads:p.reactions.normalLoads,ctrl:Array.from(d.ctrl),bodypos:Array.from(d.xpos),bodyquat:Array.from(d.xquat),geoms:p.geometries()};}
function record(){last=snapshot();fs.writeSync(fd,JSON.stringify(last)+'\n');fs.fsyncSync(fd);const o=last.physical,nonfoot=Object.entries(last.loads).some(([id,n])=>![7,10].includes(+id)&&n>=3);last.fall=o[21]<.9&&(o[36]<.5||nonfoot);last.standing=o[21]>1.4&&o[36]>Math.cos(Math.PI/6)&&(last.loads[7]||0)+(last.loads[10]||0)>=weight*.5&&!nonfoot;return last;}
function checkpoint(name){mj.mj_getState(m,d,buffer,entry.spec);write(name,{spec:entry.spec,state:Array.from(buffer.GetView()),qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),time:d.time,phase,newTicks:report.newTicks,modelSHA256:hash(xml)});}
function enter(next){const q=Array.from(d.qpos),v=Array.from(d.qvel),t=d.time;phase=report.phase=next;report.transitions.push({phase,time:t,cycleTime:t-entry.time,qpos:q,qvel:v});fallCount=0;standCount=0;assert.deepEqual(Array.from(d.qpos),q);assert.deepEqual(Array.from(d.qvel),v);save();}
function tick(){assert(report.newTicks<protocol.maximumCycleTicks);report.pendingMaxTicks=1;save();mj.mj_step2(m,d);report.newTicks++;report.pendingMaxTicks=0;p.steps++;p.readReactions();
 for(const j of p.joints){const actual=d.qfrc_actuator[j.v],wanted=d.ctrl[j.a]*j.gear;report.maxTorqueErrorNm=Math.max(report.maxTorqueErrorNm,Math.abs(actual-wanted));report.maxTorqueFraction=Math.max(report.maxTorqueFraction,Math.abs(actual/j.gear));assert(Math.abs(actual)<=Math.abs(j.gear)+1e-8);assert(Math.abs(actual-wanted)<=2e-4);}
 assert(d.qfrc_actuator.slice(0,6).every(v=>v===0));assert(d.xfrc_applied.every(v=>v===0));assert(d.qfrc_applied.every(v=>v===0));mj.mj_step1(m,d);assert([...d.qpos,...d.qvel].every(Number.isFinite));assert(Math.max(...d.qvel.map(Math.abs))<1e4);assert(Math.abs(d.time-entry.time-report.newTicks*.005)<1e-8);save();}
function decision(active){if(active)d.ctrl.set(actor.infer([...physicalObservation(p),power]).action);else d.ctrl.fill(0);for(let i=0;i<5;i++)tick();record();fallCount=last.fall?fallCount+1:0;standCount=last.standing?standCount+1:0;if(fallCount>=4&&report.firstFallTime===undefined)report.firstFallTime=last.t;if(standCount>=10&&active&&report.firstStandingTime===undefined)report.firstStandingTime=last.t;save();}
try{
 mj.mj_step1(m,d);record();
 for(let k=0;k<80;k++)decision(false);
 checkpoint('after-passive.json');report.passiveFinal=last;
 if(fallCount<4){report.stop='scenario not established: no sustained fall at 2s';}
 else{
  enter('WEAK-recovery');checkpoint('recovery-entry.json');let recovered=false;
  for(let k=0;k<250;k++){decision(true);if(standCount>=10){recovered=true;report.recoveryTicks=(k+1)*5;break;}}
  if(!recovered)report.stop='WEAK recovery deadline 6.25s: standing not observed';
  else{
   enter('WEAK-holding');checkpoint('holding-entry.json');let ended=false;
   for(let k=0;k<150;k++){decision(true);if(fallCount>=4){report.stop='measured fall during WEAK holding';ended=true;break;}}
   if(!ended){report.passed=last.standing;report.stop=report.passed?'continuous fall/getup/3.75s holding completed':'holding horizon ended without standing posture';}
  }
 }
 report.final=last;checkpoint('final-state.json');report.completed=true;
}catch(e){report.error=String(e);report.stop='integrity failure';checkpoint('error-state.json');throw e;}
finally{report.confirmedTotal=report.confirmedBefore+report.newTicks;report.diskGrowth=disk();save();fs.closeSync(fd);buffer.delete();p.dispose();console.log(JSON.stringify({completed:report.completed,passed:report.passed,stop:report.stop,error:report.error,preparationTicks:0,newTicks:report.newTicks,confirmedTotal:report.confirmedTotal,firstFallTime:report.firstFallTime,firstStandingTime:report.firstStandingTime,transitions:report.transitions.map(({phase,time,cycleTime})=>({phase,time,cycleTime})),maxTorqueFraction:report.maxTorqueFraction,maxTorqueErrorNm:report.maxTorqueErrorNm,diskGrowth:report.diskGrowth}));}
