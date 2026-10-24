// Official MuJoCo WASM. External body forces are exclusively user grab/push.
// Balance writes only the 21 actuator controls; the root remains unactuated.
import loadMujoco from './vendor/mujoco.js';
import {PullStep} from './pull-step.js';
export const add=(a,b)=>a.map((x,i)=>x+b[i]), sub=(a,b)=>a.map((x,i)=>x-b[i]), mul=(a,k)=>a.map(x=>x*k);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const rotate=(m,v)=>[0,1,2].map(i=>m[i*3]*v[0]+m[i*3+1]*v[1]+m[i*3+2]*v[2]);
export const inverseRotate=(m,v)=>[0,1,2].map(i=>m[i]*v[0]+m[3+i]*v[1]+m[6+i]*v[2]);
export class Physics {
 static async create(){const mj=await loadMujoco();const response=await fetch('./humanoid.xml');if(!response.ok)throw Error('Modèle indisponible');return new Physics(mj,await response.text());}
 constructor(mj,xml){this.mj=mj;this.model=mj.MjModel.from_xml_string(xml);this.data=new mj.MjData(this.model);this.steps=0;this.resets=0;this.grab=null;this.push=null;this.lastForce=0;this.names=Array.from({length:this.model.nbody},(_,i)=>mj.mj_id2name(this.model,mj.mjtObj.mjOBJ_BODY.value,i));this.contactBuffer=new mj.DoubleBuffer(6);this.mode='passive';this.joints=Array.from({length:this.model.nu},(_,a)=>{const j=this.model.actuator_trnid[2*a];return {a,j,name:mj.mj_id2name(this.model,mj.mjtObj.mjOBJ_JOINT.value,j),q:this.model.jnt_qposadr[j],v:this.model.jnt_dofadr[j],gear:this.model.actuator_gear[6*a]};});this.reset();}
 get dt(){return this.model.opt.timestep;}
 worldPoint(body,local){return add(Array.from(this.data.xpos.slice(body*3,body*3+3)),rotate(this.data.xmat.slice(body*9,body*9+9),local));}
 beginGrab(body,point){if(!Number.isInteger(body)||body<1||body>=this.model.nbody)throw Error('Corps invalide');this.grab={body,local:inverseRotate(this.data.xmat.slice(body*9,body*9+9),sub(point,Array.from(this.data.xpos.slice(body*3,body*3+3)))),target:[...point],previous:[...point]};}
 moveGrab(point){if(this.grab&&point.length===3&&point.every(Number.isFinite))this.grab.target=[...point];}
 release(){if(this.grab)this.pullStep?.release();this.grab=null;this.lastForce=0;this.data.xfrc_applied.fill(0);}
 smallPush(direction='forward'){const vectors={forward:[15,0,0],backward:[-15,0,0],lateral:[0,15,0]};if(!vectors[direction])throw Error('Direction invalide');this.push={body:1,remaining:.15,force:vectors[direction]};}
 pushBody(body=1){this.push={body,remaining:.06,force:[110,0,25]};}
 apply(body,point,force){const torque=cross(sub(point,Array.from(this.data.xipos.slice(body*3,body*3+3))),force);for(let i=0;i<3;i++){this.data.xfrc_applied[body*6+i]+=force[i];this.data.xfrc_applied[body*6+3+i]+=torque[i];}}
 applyPush(){if(this.push){this.apply(this.push.body,Array.from(this.data.xipos.slice(this.push.body*3,this.push.body*3+3)),this.push.force);this.push.remaining-=this.dt;if(this.push.remaining<=0)this.push=null;}}
 enablePullStep(){if(this.mode!=='balance'||this.pullStep)throw Error('Pull step requires fresh Balance');this.pullStep=new PullStep(this);}
 grabForce(){if(!this.grab)return [0,0,0];const g=this.grab,p=this.worldPoint(g.body,g.local),v=mul(sub(p,g.previous),1/this.dt);let f=sub(mul(sub(g.target,p),120),mul(v,12));const n=Math.hypot(...f);return n>180?mul(f,180/n):f;}
 setMode(mode){if(this.pullStep){this.pullStep.dispose();this.pullStep=null;}if(!['passive','balance'].includes(mode))throw Error('Mode invalide');this.mode=mode;this.cycle=null;this.fallen=false;this.previousCom=null;}
 // Optional experiment. Existing passive/balance paths are untouched without a cycle.
 startFootCycle(){
  if(this.mode!=='balance'||this.fallen||this.cycle&&!['complete','failed'].includes(this.cycle.phase))return false;
  const b=this.balance,loads=this.reactions.normalLoads;
  if(!b||Math.hypot(...b.velocity)>.03||b.tilt>.2||(loads[7]||0)<50||(loads[10]||0)<50)return false;
  this.cycle={phase:'transfer',started:this.data.time,phaseStart:this.data.time,unloadedTime:0,clearTime:0,stableTime:0,lift:0,shift:0,reason:null};return true;
 }
 footClearance(body){return Math.min(...this.geometries().filter(g=>g.body===body).map(g=>Math.min(g.a[2],g.b[2])-g.radius));}
 cycleReference(com,vel,feet){
  const c=this.cycle;if(!c||c.phase==='complete')return null;if(c.phase==='failed')return c.lastReference||null;
  const t=this.data.time-c.phaseStart,loads=this.reactions.normalLoads;
  const smooth=x=>{x=Math.max(0,Math.min(1,x));return x*x*(3-2*x);};
  const enter=phase=>{c.phase=phase;c.phaseStart=this.data.time;};
  if(this.fallen){c.phase='failed';c.reason='fall';return null;}
  if(c.phase==='transfer'){
   c.shift=smooth(t/2);c.unloadedTime=(loads[10]||0)<15&&(loads[7]||0)>200?c.unloadedTime+this.dt:0;
   if(c.unloadedTime>=.1)enter('lift');else if(t>4){c.phase='failed';c.reason='unloading not observed';}
  }else if(c.phase==='lift'){
   c.lift=smooth(t/1);const clear=this.footClearance(10);
   c.clearTime=clear>=.008&&(loads[10]||0)<1?c.clearTime+this.dt:0;
   if(c.clearTime>=.2)enter('lower');else if(t>2){c.phase='failed';c.reason='clearance not observed';}
  }else if(c.phase==='lower'){
   c.lift=1-smooth(t/1);
   if(t>=1&&(loads[10]||0)>30)enter('return');else if(t>2){c.phase='failed';c.reason='loaded recontact absent';}
  }else if(c.phase==='return'){
   c.shift=1-smooth(t/2);if(t>=2)enter('settle');
  }else if(c.phase==='settle'){
   const stable=(loads[7]||0)>50&&(loads[10]||0)>50&&Math.hypot(...vel)<.03&&this.balance.tilt<.2;
   c.stableTime=stable?c.stableTime+this.dt:0;
   if(c.stableTime>=1)enter('complete');else if(t>4){c.phase='failed';c.reason='double support not calm';}
  }
  const dy=(feet[0][1]-(feet[0][1]+feet[1][1])/2)*c.shift;
  // Shorten the free leg during the last half of transfer; no contact is removed.
  const shortening=smooth((c.shift-.5)*2);
  const lift=c.phase==='transfer'||c.phase==='lift'?shortening:c.phase==='lower'?c.lift:0;
  if(c.phase==='failed')return c.lastReference||null;
  return c.lastReference={dy,roll:dy/Math.max(.5,com[2]),lift,shift:c.shift};
 }
 // Gains: posture Nm/rad and Nm·s/rad; COM feedback Nm/m and Nm·s/m.
 // World axes: Z up, X sagittal, Y lateral. Ankle X axis is oblique.
 // No integral state, inverse dynamics, gravity force or root compensation.
 control(){
  const d=this.data,m=this.model;this.commandPrevious=this.command;this.command=[];
  const com=Array.from(d.subtree_com.slice(3,6));
  const vel=this.previousCom?mul(sub(com,this.previousCom),1/this.dt):[0,0,0];this.previousCom=com;
  const feet=[7,10].map(b=>Array.from(d.xpos.slice(3*b,3*b+3)));
  const support=mul(add(feet[0],feet[1]),.5),r=d.xmat.slice(9,18);
  this.balance={com,velocity:vel,support,tilt:Math.acos(Math.max(-1,Math.min(1,r[8]))),saturated:[]};
  if(this.mode!=='balance'||this.fallen)return;
  if(d.time>.5&&(d.xpos[5]<.65||r[8]<.4)){this.fallen=true;if(this.cycle){this.cycle.phase='failed';this.cycle.reason='measured fall';}return;}
  const cycle=this.cycleReference(com,vel,feet);
  let pull=null;if(this.pullStep){try{pull=this.pullStep.step(com,vel,feet);}catch(e){this.pullStep.fail(String(e));pull=this.pullStep.reference||null;}}
  // Midpoint of measured foot frames: double-support reference, not a fixed world anchor.
  const targetSupport=pull?.support||support;
  const forward=200*(com[0]-targetSupport[0])+80*vel[0];
  const lateral=200*(com[1]-targetSupport[1]-(cycle?.dy||0))+80*vel[1];
  for(const j of this.joints){
   const leg=/hip|knee|ankle/.test(j.name);let kp=leg?300:200,kd=leg?20:15,target=0;
   if(j.name.includes('elbow'))target=-.15;
   if(cycle){
    if(j.name.endsWith('ankle_x'))target=cycle.roll/.8944271909999159;
    if(j.name==='right_hip_x')target=-cycle.roll*.5;
    if(j.name==='left_hip_x')target=cycle.roll*.5;
    if(j.name==='left_hip_y')target=-.2*cycle.lift;
    if(j.name==='left_knee')target=-.4*cycle.lift;
    if(j.name==='left_ankle_y')target=-.2*cycle.lift;
    target=Math.max(m.jnt_range[2*j.j],Math.min(m.jnt_range[2*j.j+1],target));
   }
   if(pull)target=pull.q[j.q];
   let torque=kp*(target-d.qpos[j.q])-kd*d.qvel[j.v];
   const weight=pull?(j.name.startsWith('right_')?2*pull.loadShare:2*(1-pull.loadShare)):cycle?(j.name.startsWith('right_')?1+cycle.shift:1-cycle.shift):1;
   if(j.name.endsWith('ankle_y'))torque+=weight*forward;
   if(j.name.endsWith('ankle_x'))torque-=weight*lateral/.8944271909999159;
   if(j.name.includes('hip_y'))torque+=10*r[2];
   // Convex transition of TOTAL motor requests, then the unchanged final cap.
   if(pull&&pull.blend<1){
    const nominal=j.name.includes('elbow')?-.15:0;let before=kp*(nominal-d.qpos[j.q])-kd*d.qvel[j.v];
    if(j.name.endsWith('ankle_y'))before+=200*(com[0]-support[0])+80*vel[0];
    if(j.name.endsWith('ankle_x'))before-=(200*(com[1]-support[1])+80*vel[1])/.8944271909999159;
    if(j.name.includes('hip_y'))before+=10*r[2];torque=(1-pull.blend)*before+pull.blend*torque;
   }
   const applied=Math.max(-Math.abs(j.gear),Math.min(Math.abs(j.gear),torque));
   d.ctrl[j.a]=applied/j.gear;this.command.push({name:j.name,requested:torque,applied});
   if(applied!==torque)this.balance.saturated.push(j.name);
  }
 }
 step(){const d=this.data;d.xfrc_applied.fill(0);d.ctrl.fill(0);this.lastForce=0;this.control();if(this.pullStep&&this.fallen)this.pullStep.fail('measured fall');if(this.pullStep?.phase==='failed')return false;
  if(this.grab){const g=this.grab,p=this.worldPoint(g.body,g.local),f=this.grabForce();this.lastForce=Math.hypot(...f);this.lastForceVector=f;this.apply(g.body,p,f);g.previous=p;}else this.lastForceVector=[0,0,0];
  this.applyPush();
  if(!Array.from(d.ctrl).every(Number.isFinite))throw Error('Commande non finie : simulation arrêtée');
  this.mj.mj_step(this.model,d);this.steps++;this.readReactions();if(!Array.from(d.qpos).every(Number.isFinite)||!Array.from(d.qvel).every(Number.isFinite))throw Error('État physique non fini : simulation arrêtée');return true;
 }
 // Read solved impulses as forces for diagnostics only; never feed them back as applied forces.
 readReactions(){
  const contacts=this.data.contact,loads={};const supported=[];
  for(let i=0;i<this.data.ncon;i++){
   const c=contacts.get(i),a=this.model.geom_bodyid[c.geom1],b=this.model.geom_bodyid[c.geom2];
   if(a===0||b===0){
    this.mj.mj_contactForce(this.model,this.data,i,this.contactBuffer);
    const force=this.contactBuffer.GetView(),body=a||b;
    loads[body]=(loads[body]||0)+force[0];
    if(force[0]>.1&&!supported.includes(body))supported.push(body);
   }
   c.delete();
  }
  contacts.delete();this.reactions={normalLoads:loads,supportedBodies:supported,atTime:this.data.time};
 }
 forward(){this.mj.mj_forward(this.model,this.data);}
 reset(){if(this.pullStep){this.pullStep.dispose();this.pullStep=null;}this.cycle=null;this.reactions={normalLoads:{},supportedBodies:[],atTime:0};this.balance=null;this.previousCom=null;this.fallen=false;this.command=[];this.grab=null;this.push=null;this.lastForce=0;this.mj.mj_resetData(this.model,this.data);this.forward();this.resets++;}
 geometries(){const m=this.model,d=this.data,result=[];for(let i=0;i<m.ngeom;i++){const body=m.geom_bodyid[i];if(!body)continue;const p=Array.from(d.geom_xpos.slice(i*3,i*3+3)),mat=d.geom_xmat.slice(i*9,i*9+9),radius=m.geom_size[i*3],half=m.geom_type[i]===3?m.geom_size[i*3+1]:0,axis=[mat[2],mat[5],mat[8]];result.push({id:i,body,name:this.names[body],a:sub(p,mul(axis,half)),b:add(p,mul(axis,half)),radius});}return result;}
 state(){return {pullStep:this.pullStep?.snapshot()||null,cycle:this.cycle,mode:this.mode,fallen:this.fallen,balance:this.balance,reactions:this.reactions,command:this.command,engine:'MuJoCo '+this.mj.mj_versionString(),time:this.data.time,steps:this.steps,resets:this.resets,contacts:this.data.ncon,grab:this.grab?{body:this.grab.body,target:this.grab.target}:null,force:this.lastForce,qpos:Array.from(this.data.qpos),qvel:Array.from(this.data.qvel),physicalBodies:Array.from(this.model.body_mass).filter(x=>x>0).length,actuators:this.model.nu};}
 dispose(){this.release();this.pullStep?.dispose();this.contactBuffer.delete();this.data.delete();this.model.delete();}
}
