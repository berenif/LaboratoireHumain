// Approved single cycle: same WEAK entry, passive fall, then existing SLOW in this SAME world.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{execFileSync}from'node:child_process';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{DenseActor,physicalObservation,controls,observation,target,ORDER}from'../web/mujoco/local-getup/slow.js';
const dir='web/proofs/getup-continuous-slow',run=process.argv.includes('--run'),json=p=>JSON.parse(fs.readFileSync(p)),hash=x=>crypto.createHash('sha256').update(x).digest('hex');
const write=(n,x)=>{fs.writeFileSync(`${dir}/${n}.tmp`,JSON.stringify(x));fs.renameSync(`${dir}/${n}.tmp`,`${dir}/${n}`);},disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
assert(disk()+16000000<1500000000,'Capture/video reserve16MB unavailable');if(run)assert(!fs.existsSync(`${dir}/ledger.json`)&&!fs.existsSync(`${dir}/poses.ndjson`),'Existing/uncertain trial: no retry');
const old='web/proofs/getup-continuous-weak',oldReport=json(`${old}/ledger.json`),sourceBytes=fs.readFileSync(`${old}/poses.ndjson`),rows=sourceBytes.toString().trim().split('\n').map(JSON.parse),teacherRows=rows.filter(r=>r.phase==='WEAK-recovery').map(r=>({...r,com_ori:r.physical.slice(34,37)}));
assert(oldReport.completed&&!oldReport.passed&&oldReport.pendingMaxTicks===0&&oldReport.confirmedTotal===167059);
// Existing collector (threshold+one decision) and exact source cubic; no new teacher integration.
const ref=JSON.parse(execFileSync('python3',['scripts/getup-step/local_seed_reference.py','reference'],{input:JSON.stringify(teacherRows),maxBuffer:8*1024*1024}));
const n=ref.data.qpos.length,lastTeacher=teacherRows[ref.measuredFrames-1];assert.equal(n,Math.ceil(ref.measuredFrames/.5));
const referenceAudit={qualification:'candidate from FAILED WEAK cycle, not stable teacher',sourceSHA256:hash(sourceBytes),measuredFrames:ref.measuredFrames,frames:n,speed:.5,startTime:teacherRows[0].t,thresholdTime:teacherRows[ref.measuredFrames-2].t,endTime:lastTeacher.t,endLoads:lastTeacher.loads,endFootClearance:{right:Math.min(...lastTeacher.geoms.filter(g=>g.body===7).map(g=>Math.min(g.a[2],g.b[2])-g.radius)),left:Math.min(...lastTeacher.geoms.filter(g=>g.body===10).map(g=>Math.min(g.a[2],g.b[2])-g.radius))},movingSeconds:n*.025,terminal:'existing metadata standingPose + sentinel1.4 + vertical001'};
const oldPre=json('web/proofs/getup-weak-seed10/preflight.json'),entryBytes=fs.readFileSync('web/proofs/getup-weak-seed10/weak-final.json'),entry=JSON.parse(entryBytes);assert.equal(hash(entryBytes),oldReport.entrySHA256);
const original=fs.readFileSync('rust/crates/model/data/getup-humanoid.xml','utf8'),xml=original.replace(/<include\b[^>]*\/>/g,'').replace(/\s+material="[^"]*"/g,'').replace('<option timestep=".00125"/>','<option timestep=".005" integrator="Euler"/>');assert.equal(hash(xml),oldPre.weakXMLSHA256);
const mj=await load(),p=new Physics(mj,xml),m=p.model,d=p.data;assert.equal(mj.mj_versionString(),oldPre.engine);assert.equal(p.dt,.005);p.joints.forEach((j,a)=>assert.equal(j.q-7,ORDER[a]));
mj.mj_setState(m,d,entry.state,entry.spec);const buffer=new mj.DoubleBuffer(entry.state.length);mj.mj_getState(m,d,buffer,entry.spec);assert.deepEqual(Array.from(buffer.GetView()),entry.state);
// Test setter without integrating: changing dt must not rewrite ANY integration state.
m.opt.timestep=.00125;assert.equal(p.dt,.00125);mj.mj_getState(m,d,buffer,entry.spec);assert.deepEqual(Array.from(buffer.GetView()),entry.state);m.opt.timestep=.005;
p.forward();p.readReactions();physicalObservation(p);
const b=fs.readFileSync('rust/crates/sim/data/getup-policy/student.bin'),actor=new DenseActor(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));assert.equal(actor.dims[0],118);
const standing=json('rust/crates/model/data/getup-v1.json').referenceMetadata.standingPose,terminal=target(standing),at=k=>k>=n?terminal:[...ref.data.qpos[k].slice(7),ref.data.com[k][2],...ref.data.com_ori[k]];
for(let k=0;k<n;k++)assert(at(k).every(Number.isFinite));
const weight=Array.from(m.body_mass).reduce((a,b)=>a+b,0)*9.81;
const protocol={passiveTicks:400,passiveDt:.005,slowDt:.00125,policyHz:40,movingDecisions:n,holdingDecisions:150,maximumNewTicks:400+(n+150)*20,
 fall:'head<.9m and (torso vertical<.5 or nonfoot load>=3N) for4 consecutive40Hz samples; require fallen at passive end',
 reception:'both feet>=3N; summed feet>=.8bodyweight; no nonfoot>=3N',
 calm:'reception AND head>1.4m AND torso tilt<20deg AND COM speed<.05m/s AND root angular speed<.5rad/s, last1s continuously sampled',
 hold:'full3.75s; reject any airborne episode after reference (zero solved support plus both capsule minima>.001m); stop on sustained measured fall; no foot-drift veto',
 state:'single world; only entry restored; timestep change preserves full integration state; qvel retiming affects reference arrays ONLY'};
const preflight={referenceAudit,protocol,entrySHA256:hash(entryBytes),modelSHA256:hash(xml),actorSHA256:hash(b),entryTime:entry.time,engine:mj.mj_versionString(),diskGrowth:disk(),preparationTicks:0};
if(!run){write('reference.json',{...ref,audit:referenceAudit});write('preflight.json',preflight);console.log(JSON.stringify(preflight));buffer.delete();p.dispose();process.exit(0);}
const pre=json(`${dir}/preflight.json`);assert.deepEqual(pre.protocol,protocol);assert.deepEqual(pre.referenceAudit,referenceAudit);assert.equal(pre.entrySHA256,preflight.entrySHA256);assert.equal(pre.actorSHA256,preflight.actorSHA256);
const report={confirmedBefore:167059,priorUnknownPreserved:true,newTicks:0,passiveTicks:0,slowTicks:0,pendingMaxTicks:0,phase:'passive-fall',completed:false,passed:false,preflight,transitions:[],maxTorqueRatio:0,maxTorqueErrorNm:0,holdingSamples:0,calmSamples:0,maxCalmSamples:0,airborneHoldingSamples:0,nonfootHoldingSamples:0};
const save=()=>write('ledger.json',report);save();const fd=fs.openSync(`${dir}/poses.ndjson`,'wx');let phase=report.phase,last,fallCount=0;
function snapshot(){const o=physicalObservation(p);return{phase,t:d.time,cycleTime:d.time-entry.time,physical:o,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),com:Array.from(d.subtree_com.slice(3,6)),loads:p.reactions.normalLoads,ctrl:Array.from(d.ctrl),bodypos:Array.from(d.xpos),bodyquat:Array.from(d.xquat),geoms:p.geometries()};}
function record(){last=snapshot();fs.writeSync(fd,JSON.stringify(last)+'\n');fs.fsyncSync(fd);return last;}
const nonfoot=r=>Object.entries(r.loads).some(([id,v])=>![7,10].includes(+id)&&v>=3);
const fall=r=>r.physical[21]<.9&&(r.physical[36]<.5||nonfoot(r));
function checkpoint(name){mj.mj_getState(m,d,buffer,entry.spec);write(name,{spec:entry.spec,state:Array.from(buffer.GetView()),time:d.time,timestep:p.dt,phase,newTicks:report.newTicks,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel),modelSHA256:hash(xml)});}
function tick(){assert(report.newTicks<protocol.maximumNewTicks);report.pendingMaxTicks=1;save();mj.mj_step2(m,d);report.newTicks++;report.pendingMaxTicks=0;if(p.dt===.005)report.passiveTicks++;else report.slowTicks++;p.readReactions();
 for(const j of p.joints){const actual=d.qfrc_actuator[j.v],wanted=d.ctrl[j.a]*j.gear;report.maxTorqueRatio=Math.max(report.maxTorqueRatio,Math.abs(actual/j.gear));report.maxTorqueErrorNm=Math.max(report.maxTorqueErrorNm,Math.abs(actual-wanted));assert(Math.abs(actual)<=Math.abs(j.gear)+1e-8&&Math.abs(actual-wanted)<=2e-4);}
 assert(d.qfrc_actuator.slice(0,6).every(x=>x===0)&&d.qfrc_applied.every(x=>x===0)&&d.xfrc_applied.every(x=>x===0));mj.mj_step1(m,d);assert([...d.qpos,...d.qvel].every(Number.isFinite));assert(Math.max(...d.qvel.map(Math.abs))<1e4);assert(Math.abs(d.time-entry.time-report.passiveTicks*.005-report.slowTicks*.00125)<1e-8);save();}
function enter(next){phase=report.phase=next;report.transitions.push({phase,time:d.time,qpos:Array.from(d.qpos),qvel:Array.from(d.qvel)});save();}
try{
 mj.mj_step1(m,d);record();
 for(let k=0;k<80;k++){d.ctrl.fill(0);for(let j=0;j<5;j++)tick();record();fallCount=fall(last)?fallCount+1:0;}
 checkpoint('fallen-entry.json');report.passiveFinal=last;
 // Same entry, same passive protocol: compare the actual fallen state before the different controller.
 const oldFallen=json(`${old}/after-passive.json`);assert.deepEqual(Array.from(d.qpos),oldFallen.qpos);assert.deepEqual(Array.from(d.qvel),oldFallen.qvel);assert.equal(d.time,oldFallen.time);report.passiveMatchesPreviousExactly=true;
 if(fallCount<4)report.stop='scenario not established';
 else{
  mj.mj_getState(m,d,buffer,entry.spec);const before=Array.from(buffer.GetView());m.opt.timestep=.00125;assert.equal(p.dt,.00125);mj.mj_getState(m,d,buffer,entry.spec);assert.deepEqual(Array.from(buffer.GetView()),before);report.rateChangePreservedFullState=true;mj.mj_step1(m,d);enter('SLOW-moving');fallCount=0;
  for(let k=0;k<n+150;k++){
   const holding=k>=n;if(k===n){enter('SLOW-holding');checkpoint('holding-entry.json');fallCount=0;}
   const now=at(k),residual=actor.infer(observation(physicalObservation(p),now,at(k+5))).action;
   for(let j=0;j<20;j++){d.ctrl.set(controls(now.slice(0,21),residual,Array.from(d.qpos.slice(7)),Array.from(d.qvel.slice(6)),holding));tick();}
   record();
   if(holding){
    report.holdingSamples++;const o=last.physical,feet=[7,10].map(id=>last.loads[id]||0),clear=[7,10].map(id=>Math.min(...last.geoms.filter(g=>g.body===id).map(g=>Math.min(g.a[2],g.b[2])-g.radius)));
    const air=Object.values(last.loads).reduce((a,b)=>a+b,0)<.1&&clear.every(x=>x>.001);if(air)report.airborneHoldingSamples++;if(nonfoot(last))report.nonfootHoldingSamples++;
    const reception=feet.every(x=>x>=3)&&feet[0]+feet[1]>=.8*weight&&!nonfoot(last);
    const calm=reception&&o[21]>1.4&&o[36]>Math.cos(Math.PI/9)&&Math.hypot(...o.slice(37,40))<.05&&Math.hypot(...last.qvel.slice(3,6))<.5;
    report.calmSamples=calm?report.calmSamples+1:0;report.maxCalmSamples=Math.max(report.maxCalmSamples,report.calmSamples);fallCount=fall(last)?fallCount+1:0;
    if(fallCount>=4){report.stop='measured fall during SLOW holding';break;}
   }
  }
  if(!report.stop){report.passed=report.holdingSamples===150&&report.calmSamples>=40&&report.airborneHoldingSamples===0&&report.nonfootHoldingSamples===0;report.stop=report.passed?'continuous SLOW recovery and hold completed':'holding ended without full reception/calm/no-flight criteria';}
 }
 report.final=last;checkpoint('final-state.json');report.completed=true;
}catch(e){report.error=String(e);report.stop='integrity/precondition failure';checkpoint('error-state.json');throw e;}
finally{report.confirmedTotal=report.confirmedBefore+report.newTicks;report.diskGrowth=disk();save();fs.closeSync(fd);buffer.delete();p.dispose();console.log(JSON.stringify({completed:report.completed,passed:report.passed,stop:report.stop,error:report.error,newTicks:report.newTicks,passiveTicks:report.passiveTicks,slowTicks:report.slowTicks,confirmedTotal:report.confirmedTotal,holdingSamples:report.holdingSamples,calmSamples:report.calmSamples,airborneHoldingSamples:report.airborneHoldingSamples,maxTorqueRatio:report.maxTorqueRatio,passiveMatchesPreviousExactly:report.passiveMatchesPreviousExactly,rateChangePreservedFullState:report.rateChangePreservedFullState,diskGrowth:report.diskGrowth}));}
