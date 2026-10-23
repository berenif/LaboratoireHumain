import fs from 'node:fs';import assert from 'node:assert/strict';
import load from '../web/dist/vendor/mujoco.js';import {Physics} from '../web/dist/engine.js';
const root=process.cwd()+'/',mj=await load(),p=new Physics(mj,fs.readFileSync(root+'web/dist/humanoid.xml','utf8')),buf=new mj.DoubleBuffer(6);
const path=root+'web/proofs/balance-ledger.json',ledger=JSON.parse(fs.readFileSync(path));
const report={protocol:{forceN:15,duration:.15,point:'torso COM',pushTime:10,end:15,tiltMaxDeg:20,finalSpeed:.03,footSlipMax:.03,saturationContinuousMax:.25,footOnlyAfter:1},runs:[]};
assert.deepEqual(JSON.parse(fs.readFileSync(root+'web/proofs/balance-protocol.json')),report.protocol);
const resultPath=root+'web/proofs/balance-validation-final.json';if(fs.existsSync(resultPath))throw Error('Result already exists; preserve it and choose an explicit new destination.');
for(const direction of [[1,0,0],[-1,0,0],[0,1,0]]){
 p.reset();p.setMode('balance');let feet=null,satTime=0;const run={direction,ticks:0,maxTilt:0,maxSlip:0,maxSaturationTime:0,nonFoot:[],minimumFootLoad:[Infinity,Infinity],maximumAppliedRatio:0,samples:[]};ledger.runs.push(run);report.runs.push(run);
 try{for(let i=0;i<12000;i++){
 if(i===8000)p.push={body:1,remaining:.15,force:direction.map(x=>x*15)};
 p.step();run.ticks++;ledger.newTicks++;
 const b=p.balance;if(i===800)feet=[7,10].map(k=>Array.from(p.data.xpos.slice(k*3,k*3+3)));
 if(i>=800){run.maxTilt=Math.max(run.maxTilt,b.tilt);satTime=b.saturated.length?satTime+p.dt:0;run.maxSaturationTime=Math.max(run.maxSaturationTime,satTime);for(let f=0;f<2;f++)run.maxSlip=Math.max(run.maxSlip,Math.hypot(...[0,1].map(k=>p.data.xpos[[7,10][f]*3+k]-feet[f][k])));}
 const contacts=p.data.contact,loads={};for(let c=0;c<p.data.ncon;c++){const contact=contacts.get(c),a=p.model.geom_bodyid[contact.geom1],b=p.model.geom_bodyid[contact.geom2];mj.mj_contactForce(p.model,p.data,c,buf);const f=buf.GetView()[0];if(!a||!b){const body=a||b;loads[body]=(loads[body]||0)+f;if(i>=800&&f>.1&&![7,10].includes(body)&&!run.nonFoot.includes(body))run.nonFoot.push(body);}contact.delete();}contacts.delete();if(i>=800)for(let f=0;f<2;f++)run.minimumFootLoad[f]=Math.min(run.minimumFootLoad[f],loads[[7,10][f]]||0);
 for(const j of p.joints){const ratio=Math.abs(p.data.qfrc_actuator[j.v]/j.gear);run.maximumAppliedRatio=Math.max(run.maximumAppliedRatio,ratio);assert(ratio<=1+1e-10);}
 assert(Array.from(p.data.qfrc_actuator.slice(0,6)).every(x=>x===0));
 assert(p.command.every(c=>Math.abs(c.applied)<=Math.abs(p.joints.find(j=>j.name===c.name).gear)));
 assert(p.data.qfrc_applied.every(x=>x===0));
 if(i%400===0){run.samples.push({t:p.data.time,com:b.com,v:b.velocity,tilt:b.tilt,loads,saturated:b.saturated});fs.writeFileSync(path,JSON.stringify(ledger));}
 if(p.fallen){run.fallen=true;break;}
 }
 run.finalSpeed=Math.hypot(...p.balance.velocity);run.passed=!run.fallen&&run.maxTilt<20*Math.PI/180&&run.maxSlip<.03&&run.maxSaturationTime<.25&&run.finalSpeed<.03&&!run.nonFoot.length&&run.minimumFootLoad.every(f=>f>1);
 }finally{fs.writeFileSync(path,JSON.stringify(ledger));fs.writeFileSync(resultPath,JSON.stringify(report));console.log(JSON.stringify({...run,samples:run.samples.filter((_,i)=>i%5===0)}));}
}
p.dispose();buf.delete();
