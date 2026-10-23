// Official MuJoCo WASM. External body forces are exclusively user grab/push.
// Balance writes only the 21 actuator controls; the root remains unactuated.
import loadMujoco from './vendor/mujoco.js';
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
 release(){this.grab=null;this.lastForce=0;this.data.xfrc_applied.fill(0);}
 smallPush(direction='forward'){const vectors={forward:[15,0,0],backward:[-15,0,0],lateral:[0,15,0]};if(!vectors[direction])throw Error('Direction invalide');this.push={body:1,remaining:.15,force:vectors[direction]};}
 pushBody(body=1){this.push={body,remaining:.06,force:[110,0,25]};}
 apply(body,point,force){const torque=cross(sub(point,Array.from(this.data.xipos.slice(body*3,body*3+3))),force);for(let i=0;i<3;i++){this.data.xfrc_applied[body*6+i]+=force[i];this.data.xfrc_applied[body*6+3+i]+=torque[i];}}
 setMode(mode){if(!['passive','balance'].includes(mode))throw Error('Mode invalide');this.mode=mode;this.fallen=false;this.previousCom=null;}
 // Gains: posture Nm/rad and Nm·s/rad; COM feedback Nm/m and Nm·s/m.
 // World axes: Z up, X sagittal, Y lateral. Ankle X axis is oblique.
 // No integral state, inverse dynamics, gravity force or root compensation.
 control(){
  const d=this.data,m=this.model;this.command=[];
  const com=Array.from(d.subtree_com.slice(3,6));
  const vel=this.previousCom?mul(sub(com,this.previousCom),1/this.dt):[0,0,0];this.previousCom=com;
  const feet=[7,10].map(b=>Array.from(d.xpos.slice(3*b,3*b+3)));
  const support=mul(add(feet[0],feet[1]),.5),r=d.xmat.slice(9,18);
  this.balance={com,velocity:vel,support,tilt:Math.acos(Math.max(-1,Math.min(1,r[8]))),saturated:[]};
  if(this.mode!=='balance'||this.fallen)return;
  if(d.time>.5&&(d.xpos[5]<.65||r[8]<.4)){this.fallen=true;return;}
  // Midpoint of measured foot frames: double-support reference, not a fixed world anchor.
  const forward=200*(com[0]-support[0])+80*vel[0];
  const lateral=200*(com[1]-support[1])+80*vel[1];
  for(const j of this.joints){
   const leg=/hip|knee|ankle/.test(j.name);let kp=leg?300:200,kd=leg?20:15,target=0;
   if(j.name.includes('elbow'))target=-.15;
   let torque=kp*(target-d.qpos[j.q])-kd*d.qvel[j.v];
   if(j.name.endsWith('ankle_y'))torque+=forward;
   if(j.name.endsWith('ankle_x'))torque-=lateral/.8944271909999159;
   if(j.name.includes('hip_y'))torque+=10*r[2];
   const applied=Math.max(-Math.abs(j.gear),Math.min(Math.abs(j.gear),torque));
   d.ctrl[j.a]=applied/j.gear;this.command.push({name:j.name,requested:torque,applied});
   if(applied!==torque)this.balance.saturated.push(j.name);
  }
 }
 step(){const d=this.data;d.xfrc_applied.fill(0);d.ctrl.fill(0);this.lastForce=0;this.control();
  if(this.grab){const g=this.grab,p=this.worldPoint(g.body,g.local),velocity=mul(sub(p,g.previous),1/this.dt);let f=sub(mul(sub(g.target,p),120),mul(velocity,12));const length=Math.hypot(...f);if(length>180)f=mul(f,180/length);this.lastForce=Math.hypot(...f);this.apply(g.body,p,f);g.previous=p;}
  if(this.push){this.apply(this.push.body,Array.from(d.xipos.slice(this.push.body*3,this.push.body*3+3)),this.push.force);this.push.remaining-=this.dt;if(this.push.remaining<=0)this.push=null;}
  if(!Array.from(d.ctrl).every(Number.isFinite))throw Error('Commande non finie : simulation arrêtée');
  this.mj.mj_step(this.model,d);this.steps++;this.readReactions();if(!Array.from(d.qpos).every(Number.isFinite)||!Array.from(d.qvel).every(Number.isFinite))throw Error('État physique non fini : simulation arrêtée');
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
 reset(){this.reactions={normalLoads:{},supportedBodies:[],atTime:0};this.balance=null;this.previousCom=null;this.fallen=false;this.command=[];this.grab=null;this.push=null;this.lastForce=0;this.mj.mj_resetData(this.model,this.data);this.forward();this.resets++;}
 geometries(){const m=this.model,d=this.data,result=[];for(let i=0;i<m.ngeom;i++){const body=m.geom_bodyid[i];if(!body)continue;const p=Array.from(d.geom_xpos.slice(i*3,i*3+3)),mat=d.geom_xmat.slice(i*9,i*9+9),radius=m.geom_size[i*3],half=m.geom_type[i]===3?m.geom_size[i*3+1]:0,axis=[mat[2],mat[5],mat[8]];result.push({id:i,body,name:this.names[body],a:sub(p,mul(axis,half)),b:add(p,mul(axis,half)),radius});}return result;}
 state(){return {mode:this.mode,fallen:this.fallen,balance:this.balance,reactions:this.reactions,command:this.command,engine:'MuJoCo '+this.mj.mj_versionString(),time:this.data.time,steps:this.steps,resets:this.resets,contacts:this.data.ncon,grab:this.grab?{body:this.grab.body,target:this.grab.target}:null,force:this.lastForce,qpos:Array.from(this.data.qpos),qvel:Array.from(this.data.qvel),physicalBodies:Array.from(this.model.body_mass).filter(x=>x>0).length,actuators:this.model.nu};}
 dispose(){this.release();this.contactBuffer.delete();this.data.delete();this.model.delete();}
}
