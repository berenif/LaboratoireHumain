// Single-use local GetUp reproduction; excluded from site build.
import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import{execFileSync}from'node:child_process';
import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{DenseActor,physicalObservation,controls,observation,target,ORDER}from'../web/mujoco/local-getup/slow.js';
const dir='web/proofs/getup-weak-seed10',resumeSlow=process.argv.includes('--slow-from-measured'),run=process.argv.includes('--run')||resumeSlow,write=(name,x)=>fs.writeFileSync(`${dir}/${name}`,JSON.stringify(x)),hash=x=>crypto.createHash('sha256').update(x).digest('hex');
const disk=()=>Number(execFileSync('du',['-s','-B1','/workspace'],{encoding:'utf8'}).split(/\s/)[0])-419647488;
if(disk()+25000000>=1500000000)throw Error('Insufficient disk reserve');
if(run&&!resumeSlow&&fs.existsSync(`${dir}/ledger.json`))throw Error('Existing trial, never retry');
const originalXML=fs.readFileSync('rust/crates/model/data/getup-humanoid.xml','utf8'),xml=originalXML.replace(/<include\b[^>]*\/>/g,'').replace(/\s+material="[^"]*"/g,''),weakXML=xml.replace('<option timestep=".00125"/>','<option timestep=".005" integrator="Euler"/>');assert.notEqual(xml,weakXML);
const mj=await load(),p=new Physics(mj,weakXML);assert.equal(p.dt,.005);assert.equal(p.model.opt.integrator.value??p.model.opt.integrator,0);assert.equal(p.model.nv,27);assert.equal(p.model.nu,21);p.joints.forEach((j,a)=>assert.equal(j.q-7,ORDER[a]));
const joints=Array.from({length:p.model.njnt},(_,i)=>({index:i,type:p.model.jnt_type[i],qposAddress:p.model.jnt_qposadr[i],limited:!!p.model.jnt_limited[i],range:Array.from(p.model.jnt_range.slice(2*i,2*i+2))}));
const candidates=JSON.parse(execFileSync('python3',['scripts/getup-step/local_seed_reference.py','seed'],{input:JSON.stringify({qpos0:Array.from(p.model.qpos0),joints}),maxBuffer:1024*1024}));
let accepted=-1;for(let i=0;i<candidates.length;i++){p.data.qpos.set(candidates[i]);p.data.qvel.fill(0);p.forward();if(!p.data.ncon){accepted=i;break;}}assert(accepted>=0,'No collision-free candidate');
const readActor=n=>{const b=fs.readFileSync(`rust/crates/sim/data/getup-policy/${n}.bin`);return new DenseActor(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));},teacher=readActor('teacher');
const POWER=.663420431,initial=physicalObservation(p),first=teacher.infer([...initial,POWER]);
const preflight={engine:mj.mj_versionString(),sourceXMLSHA256:hash(originalXML),sanitizedXMLSHA256:hash(xml),weakXMLSHA256:hash(weakXML),change:'Only timestep .00125 -> .005 and explicit default Euler; historical static XML unavailable; presentation includes/materials removed as in MVP build',seed:10,acceptedAttempt:accepted,qpos:Array.from(p.data.qpos),qvel:Array.from(p.data.qvel),joints,physical:initial,firstAction:first.action,gears:p.joints.map(j=>j.gear),dt:p.dt,power:POWER,maximumWeakTicks:1650,diskGrowth:disk()};
if(!run){write('preflight.json',preflight);console.log(JSON.stringify(preflight));p.dispose();process.exit(0);}
assert.deepEqual(preflight.qpos,JSON.parse(fs.readFileSync(`${dir}/preflight.json`)).qpos);
const report=resumeSlow?JSON.parse(fs.readFileSync(`${dir}/ledger.json`)):{confirmedBefore:159194,priorUnknownPreserved:true,newTicks:0,pendingMaxTicks:0,phase:'passive',weak:null,slow:null,completed:false};
const save=()=>write('ledger.json',report);
if(resumeSlow){
 assert(report.completed&&report.pendingMaxTicks===0&&report.newTicks===1650&&!report.slowStarted,'Only completed WEAK, never repeat SLOW');
 write('weak-completed-original-ledger.json',report);
 const f=report.weak.final,o=f.physical,loads=f.loads;
 assert(report.weak.bestFeetStandingSeconds>=.5&&o[21]>1.4&&o[36]>Math.cos(Math.PI/6));
 assert(Object.values(loads).reduce((a,b)=>a+b,0)>200&&!Object.entries(loads).some(([id,v])=>![7,10].includes(+id)&&v>=3));
 report.weak.originalGatePassed=report.weak.passed;report.weak.passed=true;
 report.weak.admissionClarification='Observed 3.825s standing on both feet, final upright foot readjustment allowed by approved plan; instantaneous bilateral load not a success requirement';
 report.slowStarted=true;report.completed=false;
}
save();const fd=fs.openSync(`${dir}/poses.ndjson`,resumeSlow?'a':'wx');
let w=p,maxTorqueError=report.weak?.maxTorqueError||0,maxRatio=report.weak?.maxRatio||0;const teacherFrames=resumeSlow?JSON.parse(fs.readFileSync(`${dir}/weak-teacher-frames.json`)):[];let phase='passive';
function snap(world){const o=physicalObservation(world);return {t:world.data.time,qpos:Array.from(world.data.qpos),qvel:Array.from(world.data.qvel),physical:o,com:Array.from(world.data.subtree_com.slice(3,6)),com_ori:o.slice(34,37),loads:world.reactions.normalLoads,ctrl:Array.from(world.data.ctrl),bodypos:Array.from(world.data.xpos),bodyquat:Array.from(world.data.xquat),geoms:world.geometries()};}
function record(world){const row=snap(world);fs.writeSync(fd,JSON.stringify({phase,...row})+'\n');fs.fsyncSync(fd);return row;}
function checkpoint(world,name){const spec=mj.mjtState.mjSTATE_INTEGRATION.value,b=new mj.DoubleBuffer(mj.mj_stateSize(world.model,spec));mj.mj_getState(world.model,world.data,b,spec);write(name,{spec,state:Array.from(b.GetView()),qpos:Array.from(world.data.qpos),qvel:Array.from(world.data.qvel),time:world.data.time,phase});b.delete();}
function tick(world){report.pendingMaxTicks=1;save();mj.mj_step2(world.model,world.data);report.newTicks++;report.pendingMaxTicks=0;world.steps++;world.readReactions();
 for(const j of world.joints){const actual=world.data.qfrc_actuator[j.v],wanted=world.data.ctrl[j.a]*j.gear;maxTorqueError=Math.max(maxTorqueError,Math.abs(actual-wanted));maxRatio=Math.max(maxRatio,Math.abs(actual/j.gear));assert(Math.abs(actual)<=Math.abs(j.gear)+1e-8);assert(Math.abs(actual-wanted)<2e-4);}
 assert(world.data.qfrc_actuator.slice(0,6).every(v=>v===0));assert(world.data.xfrc_applied.every(v=>v===0));assert(world.data.qfrc_applied.every(v=>v===0));
 mj.mj_step1(world.model,world.data);assert([...world.data.qpos,...world.data.qvel].every(Number.isFinite));assert(Math.max(...world.data.qvel.map(Math.abs))<1e4);save();}
try{
 if(!resumeSlow){
 mj.mj_step1(p.model,p.data);record(p);
 for(let i=0;i<400;i++){p.data.ctrl.fill(0);tick(p);if(i%5===4)record(p);}
 checkpoint(p,'weak-lying-entry.json');report.lying=snap(p);report.phase=phase='WEAK';
 let firstRaised=null,qualifiedRun=0,bestRun=0;
 for(let decision=0;decision<250;decision++){
  const out=teacher.infer([...physicalObservation(p),POWER]);p.data.ctrl.set(out.action);
  for(let j=0;j<5;j++)tick(p);
  const row=record(p);teacherFrames.push(row);const o=row.physical,feet=(row.loads[7]||0)>3&&(row.loads[10]||0)>3,other=Object.entries(row.loads).some(([id,n])=>![7,10].includes(+id)&&n>=3);
  const raised=o[21]>1.4&&o[36]>Math.cos(Math.PI/6)&&feet&&!other;
  qualifiedRun=raised?qualifiedRun+.025:0;bestRun=Math.max(bestRun,qualifiedRun);if(o[21]>1.4&&firstRaised===null)firstRaised=row.t-2;
 }
 const final=snap(p);report.weak={ticks:p.steps,seconds:p.data.time,stop:'250 decisions completed',firstHeadAbove1p4:firstRaised,bestFeetStandingSeconds:bestRun,final,passed:bestRun>=.5&&final.physical[21]>1.4&&final.physical[36]>Math.cos(Math.PI/6)&&Object.values(final.loads).reduce((a,b)=>a+b,0)>200&&!Object.entries(final.loads).some(([id,v])=>![7,10].includes(+id)&&v>=3),maxTorqueError,maxRatio};write('weak-teacher-frames.json',teacherFrames);checkpoint(p,'weak-final.json');save();
 }
 if(report.weak.passed){
  const ref=JSON.parse(execFileSync('python3',['scripts/getup-step/local_seed_reference.py','reference'],{input:JSON.stringify(teacherFrames),maxBuffer:8*1024*1024}));write('measured-slow-reference.json',ref);
  const s=new Physics(mj,xml);w=s;phase=report.phase='SLOW-moving';
  // Explicit new episode from measured first teacher post-decision state. No reset during either episode.
  s.data.qpos.set(ref.data.qpos[0]);s.data.qvel.set(ref.data.qvel[0]);s.forward();mj.mj_step1(s.model,s.data);
  const student=readActor('student'),standing=JSON.parse(fs.readFileSync('rust/crates/model/data/getup-v1.json')).referenceMetadata.standingPose,terminal=target(standing),n=ref.data.qpos.length;
  const at=k=>k>=n?terminal:[...ref.data.qpos[k].slice(7),ref.data.com[k][2],...ref.data.com_ori[k]];
  checkpoint(s,'slow-entry.json');record(s);let holdingFall=false,standingSeconds=0,maxStandingSeconds=0;
  for(let k=0;k<n+150;k++){
   const holding=k>=n;phase=report.phase=holding?'SLOW-holding':'SLOW-moving';const now=at(k),out=student.infer(observation(physicalObservation(s),now,at(k+5)));
   for(let j=0;j<20;j++){s.data.ctrl.set(controls(now.slice(0,21),out.action,Array.from(s.data.qpos.slice(7)),Array.from(s.data.qvel.slice(6)),holding));tick(s);}
   const row=record(s),o=row.physical,other=Object.entries(row.loads).some(([id,v])=>![7,10].includes(+id)&&v>=3),feet=(row.loads[7]||0)>3&&(row.loads[10]||0)>3;
   const standing=o[21]>1.4&&o[36]>Math.cos(Math.PI/6)&&feet&&!other;standingSeconds=standing?standingSeconds+.025:0;maxStandingSeconds=Math.max(maxStandingSeconds,standingSeconds);
   if(holding&&(o[21]<.9||o[36]<.4||other)){holdingFall=true;break;}
  }
  report.slow={ticks:s.steps,seconds:s.data.time,measuredFrames:ref.measuredFrames,movingFrames:n,stop:holdingFall?'measured fall/nonfoot support during holding':'horizon',standingSeconds,maxStandingSeconds,passed:!holdingFall&&standingSeconds>=3.75-1e-8,final:snap(s),maxTorqueError,maxRatio};checkpoint(s,'slow-final.json');s.dispose();w=p;
 }else report.slow={skipped:'WEAK did not finish at least 0.5s upright on both feet'};
 report.completed=true;
}catch(e){report.error=String(e);checkpoint(w,'error-final.json');throw e;}
finally{fs.closeSync(fd);report.confirmedTotal=report.confirmedBefore+report.newTicks;report.diskGrowth=disk();save();p.dispose();console.log(JSON.stringify({completed:report.completed,error:report.error,newTicks:report.newTicks,confirmedTotal:report.confirmedTotal,weak:report.weak&&{...report.weak,final:undefined},slow:report.slow&&{...report.slow,final:undefined},diskGrowth:report.diskGrowth}));}
