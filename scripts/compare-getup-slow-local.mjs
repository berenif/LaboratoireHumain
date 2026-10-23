import fs from 'node:fs';import assert from 'node:assert/strict';import crypto from 'node:crypto';import load from '../web/dist/vendor/mujoco.js';import{Physics}from'../web/dist/engine.js';import{DenseActor,SlowHolding,physicalObservation}from'../web/mujoco/local-getup/slow.js';
const output='web/proofs/slow-holding-comparison.json';if(fs.existsSync(output))throw Error('Existing comparison; no retry');
const mj=await load(),xml=fs.readFileSync('web/dist/humanoid.xml','utf8'),p=new Physics(mj,xml);
const report={confirmedBefore:147714,priorUnknownPreserved:true,newTicks:0,pendingMaxTicks:0,protocol:{entry:'balance265a638, 8000 real ticks from unchanged reset',branchTicks:3000,seconds:3.75,holding:true,power:1,policyHz:40,physicsHz:800,tiltLimitDeg:20,slipLimitM:.03,calmSpeedMps:.03,finalCalmSeconds:1,stop:'fall, nonfoot support, 30mm slip, nonfinite or horizon'},runs:[]};
const save=()=>fs.writeFileSync(output,JSON.stringify(report));save();
function step(world){report.pendingMaxTicks=1;save();world.step();report.newTicks++;report.pendingMaxTicks=0;}
const binary=fs.readFileSync('rust/crates/sim/data/getup-policy/student.bin');const actor=new DenseActor(binary.buffer.slice(binary.byteOffset,binary.byteOffset+binary.byteLength));
const standing=JSON.parse(fs.readFileSync('rust/crates/model/data/getup-v1.json')).referenceMetadata.standingPose;
const worlds=[];
try{
 p.setMode('balance');for(let i=0;i<8000;i++)step(p);save();
 const spec=mj.mjtState.mjSTATE_INTEGRATION.value,buffer=new mj.DoubleBuffer(mj.mj_stateSize(p.model,spec));mj.mj_getState(p.model,p.data,buffer,spec);const state=Array.from(buffer.GetView());
 report.entry={time:p.data.time,stateSHA256:crypto.createHash('sha256').update(JSON.stringify(state)).digest('hex'),state,spec,com:p.balance.com,loads:p.reactions.normalLoads,previousCom:p.previousCom};
 for(const name of ['balance265a638','SLOW-terminal']){
  const w=new Physics(mj,xml);worlds.push(w);w.setMode('balance');
  // Explicit comparison fork: complete MuJoCo integration state, including warmstart.
  // No qpos/dq assignment or reset is performed during either branch.
  mj.mj_setState(w.model,w.data,state,spec);mj.mj_getState(w.model,w.data,buffer,spec);assert.deepEqual(Array.from(buffer.GetView()),state);w.forward();w.previousCom=[...p.previousCom];
  const hold=name==='SLOW-terminal'?new SlowHolding(w,actor,standing):null;if(hold)w.control=()=>hold.command();
  const feet=[7,10].map(id=>Array.from(w.data.xpos.slice(id*3,id*3+3))),run={name,ticks:0,stop:'horizon',maxTiltDeg:0,maxSlipM:0,maxTorqueRatio:0,maxSaturationSeconds:0,calmSeconds:0,samples:[]};report.runs.push(run);let saturation=0;
  for(let i=0;i<3000;i++){
   step(w);run.ticks++;
   for(const j of w.joints){const ratio=Math.abs(w.data.qfrc_actuator[j.v]/j.gear);run.maxTorqueRatio=Math.max(run.maxTorqueRatio,ratio);assert(ratio<=1+1e-9);}assert(w.data.qfrc_actuator.slice(0,6).every(x=>x===0));assert(w.data.xfrc_applied.every(x=>x===0));
   saturation=w.data.ctrl.some(x=>Math.abs(x)>=.999999)?saturation+w.dt:0;run.maxSaturationSeconds=Math.max(run.maxSaturationSeconds,saturation);
   // Same observation cadence in both branches; native reactions were saved immediately after solve.
   if(i%20===19){const o=physicalObservation(w),com=Array.from(w.data.subtree_com.slice(3,6)),speed=Math.hypot(...o.slice(37,40)),tilt=Math.acos(Math.max(-1,Math.min(1,o[36])))*180/Math.PI;
    const slip=Math.max(...[7,10].map((id,f)=>Math.hypot(w.data.xpos[id*3]-feet[f][0],w.data.xpos[id*3+1]-feet[f][1]))),loads=w.reactions.normalLoads;
    run.maxTiltDeg=Math.max(run.maxTiltDeg,tilt);run.maxSlipM=Math.max(run.maxSlipM,slip);
    const calm=tilt<20&&speed<.03&&(loads[7]||0)>1&&(loads[10]||0)>1&&slip<.03;
    run.calmSeconds=calm?run.calmSeconds+.025:0;
    const row={t:w.data.time,com,velocity:o.slice(37,40),tilt,slip,loads,ctrl:Array.from(w.data.ctrl),calmSeconds:run.calmSeconds};if(i%200===199||!calm)run.samples.push(row);
    if(o[21]<.9||tilt>66)run.stop='measured fall';else if(w.reactions.supportedBodies.some(id=>![7,10].includes(id)))run.stop='nonfoot support';else if(slip>.03)run.stop='foot slip >30mm';
    if(run.stop!=='horizon'){run.final=row;break;}
   }
   if(i%200===199)save();
  }
  run.decisions=hold?.decisions??null;run.lastSource=hold?.last??null;run.passed=run.stop==='horizon'&&run.calmSeconds>=1&&run.maxTiltDeg<20;run.finalQpos=Array.from(w.data.qpos);run.finalQvel=Array.from(w.data.qvel);save();
 }
 buffer.delete();report.completed=true;
}finally{report.confirmedTotal=report.confirmedBefore+report.newTicks;save();for(const w of worlds)w.dispose();p.dispose();console.log(JSON.stringify({newTicks:report.newTicks,confirmedTotal:report.confirmedTotal,runs:report.runs.map(({samples,lastSource,finalQpos,finalQvel,...r})=>r)}));}
