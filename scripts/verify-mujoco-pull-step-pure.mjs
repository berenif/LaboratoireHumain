// Pure FK / command / transition checks. mj_step is never called.
import fs from 'node:fs';import assert from 'node:assert/strict';import {execFileSync} from 'node:child_process';import {pathToFileURL} from 'node:url';import crypto from 'node:crypto';
import load from '../web/dist/vendor/mujoco.js';import {Physics} from '../web/dist/engine.js';import {RelativeFootIK,PullStep,PULL_PROTOCOL} from '../web/dist/pull-step.js';
const oldSource=execFileSync('git',['show','5dc93fa:web/mujoco/engine.js'],{encoding:'utf8'}).replace("'./vendor/mujoco.js'",JSON.stringify(pathToFileURL(process.cwd()+'/web/dist/vendor/mujoco.js').href));
const {Physics:Old}=await import('data:text/javascript;base64,'+Buffer.from(oldSource).toString('base64'));
const mj=await load(),xml=fs.readFileSync('web/dist/humanoid.xml','utf8'),p=new Physics(mj,xml),old=new Old(mj,xml),ik=new RelativeFootIK(p);
const physical=()=>JSON.stringify({t:p.data.time,q:Array.from(p.data.qpos),v:Array.from(p.data.qvel),x:Array.from(p.data.xfrc_applied),steps:p.steps});
const initial=physical(),q0=Array.from(p.data.qpos);let maxCommandDifference=0,maxJacobianError=0;
for(let pose=0;pose<3;pose++){
 for(const w of [p,old]){w.setMode('balance');w.data.qpos.set(q0);for(const j of w.joints){if(pose)w.data.qpos[j.q]=Math.max(w.model.jnt_range[2*j.j],Math.min(w.model.jnt_range[2*j.j+1],.025*Math.sin(j.a+pose)));w.data.qvel[j.v]=pose*.02*Math.cos(j.a);}w.forward();w.control();}
 maxCommandDifference=Math.max(maxCommandDifference,...p.command.map((c,i)=>Math.abs(c.requested-old.command[i].requested)));assert.deepEqual(p.command,old.command);
}
p.data.qpos.set(q0);p.data.qvel.fill(0);p.forward();p.previousCom=null;
ik.set(q0);const f=ik.features(),jac=ik.jacobians();
for(let i=0;i<ik.joints.length;i++){const q=q0.slice();q[ik.joints[i].q]+=1e-6;ik.set(q);const changed=ik.features(),actual=[...changed.com,...changed.foot],base=[...f.com,...f.foot];for(let k=0;k<6;k++)maxJacobianError=Math.max(maxJacobianError,Math.abs((actual[k]-base[k])/1e-6-jac[k][i]));}assert(maxJacobianError<1e-5);
const shifted=q0.slice();shifted[0]+=3;shifted[1]-=2;shifted[2]+=.5;ik.set(shifted);const translated=ik.features();assert(Math.max(...translated.com.map((x,i)=>Math.abs(x-f.com[i])),...translated.foot.map((x,i)=>Math.abs(x-f.foot[i])))<1e-12);
let q=q0.slice();const paths=[];
for(let stage=0;stage<5;stage++){
 ik.set(q);const a=ik.features();let goal={...a};
 if(stage===0)goal.com=[a.com[0],0,a.com[2]-.005];
 if(stage===1)goal.foot=[a.foot[0],a.foot[1],a.foot[2]+.02];
 if(stage===2)goal.foot=[a.foot[0]+.06,a.foot[1],a.foot[2]];
 if(stage===3)goal.foot=[a.foot[0],a.foot[1],a.foot[2]-.02];
 if(stage===4)goal.com=[a.foot[0]/2,a.foot[1]/2,f.com[2]];
 const path=ik.path(q,goal,[1.8,.4,.6,.5,.6][stage]);assert.deepEqual(path.at(0),q);q=path.at(1);ik.set(q);const final=ik.features();paths.push({stage,residualM:path.residual,maxReferenceSpeed:path.maxSpeed,com:final.com,foot:final.foot,conditionalClearanceDeltaM:final.foot[2]-f.foot[2],heightChangeM:final.com[2]-f.com[2]});assert(path.residual<.002);
}
assert.equal(p.steps,0);assert.equal(physical(),initial);
// Synthetic load/velocity fixture tests activation and command continuity;
// these observations are NOT passed off as a simulated standing state.
p.setMode('balance');p.control();const before=p.command.map(c=>c.applied);p.enablePullStep();p.pullStep.quietCom=Array.from(p.data.subtree_com.slice(3,6));p.pullStep.quietCom[0]-=.02;p.reactions={normalLoads:{7:200,10:200},supportedBodies:[7,10]};const hand=p.worldPoint(15,[.09,-.09,.09]);p.beginGrab(15,hand);p.moveGrab([hand[0]+.25,hand[1],hand[2]]);p.control();assert.equal(p.pullStep.phase,'transfer');assert.deepEqual(p.command.map(c=>c.applied),before);assert(p.data.qfrc_applied.every(x=>x===0));assert(p.data.xfrc_applied.every(x=>x===0));
const appliedCap=()=>p.command.forEach((c,i)=>assert(Number.isFinite(c.applied)&&Math.abs(c.applied)<=Math.abs(p.joints[i].gear)));
appliedCap();p.release();p.control();assert.equal(p.pullStep.phase,'cancel-return');assert.equal(p.steps,0);appliedCap();
// A terminal planning failure must not integrate one last tick.
p.pullStep.fail('synthetic terminal fixture');const count=p.steps,time=p.data.time;assert.equal(p.step(),false);assert.equal(p.steps,count);assert.equal(p.data.time,time);
for(const [file,sha] of [['cycle.js','983d62ff367f2c77a99a298b8fee75f8565e1d1227dc0c8deedf0227341171c3'],['continuation.js','1fd083a81cd761a015a5561d0c548a1d9cbb8deb3f806e9008b5a8d5b4902f17']]){let b=fs.readFileSync('web/mujoco/local-getup/'+file);const marker=file==='cycle.js'?'export class CycleObserver':'export class SlowContinuation';b=b.subarray(b.indexOf(marker));assert.equal(crypto.createHash('sha256').update(b).digest('hex'),sha);}
const report={passed:true,integrations:0,conditionalFKNotDynamicProof:true,maxCommandDifference,maxJacobianError,paths,protocol:PULL_PROTOCOL,checks:['baseline Balance 21 commands identical at three pure fixtures','measured-shape and compliant servo references separated','native Jacobian finite differences','relative translation invariance','finite interpolated FK + ROM','full command continuity at trigger','release cancels before lift','failure stops before mj_step','SLOW modules unchanged']};console.log(JSON.stringify(report));ik.dispose();p.dispose();old.dispose();
