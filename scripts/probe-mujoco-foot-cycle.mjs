import fs from 'node:fs';import load from '../web/dist/vendor/mujoco.js';import {Physics} from '../web/dist/engine.js';
const file=process.argv[2];if(!file||fs.existsSync(file))throw Error('Provide a new result path');
const mj=await load(),p=new Physics(mj,fs.readFileSync('web/dist/humanoid.xml','utf8'));p.setMode('balance');
const result={baselineConfirmed:102954,priorUnknownPreserved:true,ticks:0,samples:[],complete:false};
const save=()=>fs.writeFileSync(file,JSON.stringify(result));save();
try{for(let i=0;i<20000;i++){
 if(i===8000){result.started=p.startFootCycle();if(!result.started)break;}
 p.step();result.ticks++;
 if(i%200===0||p.cycle?.phase==='failed'||p.cycle?.phase==='complete'){
 result.samples.push({t:p.data.time,phase:p.cycle?.phase,reason:p.cycle?.reason,com:p.balance.com,v:p.balance.velocity,tilt:p.balance.tilt,loads:p.reactions.normalLoads,clear:p.footClearance(10),supportClear:p.footClearance(7),feet:[7,10].map(b=>Array.from(p.data.xpos.slice(3*b,3*b+3))),q:Array.from(p.data.qpos.slice(7,22)),sat:p.balance.saturated,command:p.command.filter(j=>/hip_x|ankle_x/.test(j.name))});save();}
 if(p.fallen||p.cycle?.phase==='failed'||p.cycle?.phase==='complete')break;
 }}finally{result.complete=true;result.final=p.state();save();console.log(JSON.stringify({ticks:result.ticks,cycle:p.cycle,samples:result.samples.filter(s=>s.t>9.9)}));p.dispose();}
