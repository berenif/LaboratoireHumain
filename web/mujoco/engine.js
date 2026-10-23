// Official MuJoCo WASM. All external forces below are explicit user actions.
import loadMujoco from './vendor/mujoco.js';
export const add=(a,b)=>a.map((x,i)=>x+b[i]), sub=(a,b)=>a.map((x,i)=>x-b[i]), mul=(a,k)=>a.map(x=>x*k);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const rotate=(m,v)=>[0,1,2].map(i=>m[i*3]*v[0]+m[i*3+1]*v[1]+m[i*3+2]*v[2]);
export const inverseRotate=(m,v)=>[0,1,2].map(i=>m[i]*v[0]+m[3+i]*v[1]+m[6+i]*v[2]);
export class Physics {
 static async create(){const mj=await loadMujoco();const response=await fetch('./humanoid.xml');if(!response.ok)throw Error('Modèle indisponible');return new Physics(mj,await response.text());}
 constructor(mj,xml){this.mj=mj;this.model=mj.MjModel.from_xml_string(xml);this.data=new mj.MjData(this.model);this.steps=0;this.resets=0;this.grab=null;this.push=null;this.lastForce=0;this.names=Array.from({length:this.model.nbody},(_,i)=>mj.mj_id2name(this.model,mj.mjtObj.mjOBJ_BODY.value,i));this.reset();}
 get dt(){return this.model.opt.timestep;}
 worldPoint(body,local){return add(Array.from(this.data.xpos.slice(body*3,body*3+3)),rotate(this.data.xmat.slice(body*9,body*9+9),local));}
 beginGrab(body,point){if(!Number.isInteger(body)||body<1||body>=this.model.nbody)throw Error('Corps invalide');this.grab={body,local:inverseRotate(this.data.xmat.slice(body*9,body*9+9),sub(point,Array.from(this.data.xpos.slice(body*3,body*3+3)))),target:[...point],previous:[...point]};}
 moveGrab(point){if(this.grab&&point.length===3&&point.every(Number.isFinite))this.grab.target=[...point];}
 release(){this.grab=null;this.lastForce=0;this.data.xfrc_applied.fill(0);}
 pushBody(body=1){this.push={body,remaining:.06,force:[110,0,25]};}
 apply(body,point,force){const torque=cross(sub(point,Array.from(this.data.xipos.slice(body*3,body*3+3))),force);for(let i=0;i<3;i++){this.data.xfrc_applied[body*6+i]+=force[i];this.data.xfrc_applied[body*6+3+i]+=torque[i];}}
 step(){const d=this.data;d.xfrc_applied.fill(0);d.ctrl.fill(0);this.lastForce=0;
  if(this.grab){const g=this.grab,p=this.worldPoint(g.body,g.local),velocity=mul(sub(p,g.previous),1/this.dt);let f=sub(mul(sub(g.target,p),120),mul(velocity,12));const length=Math.hypot(...f);if(length>180)f=mul(f,180/length);this.lastForce=Math.hypot(...f);this.apply(g.body,p,f);g.previous=p;}
  if(this.push){this.apply(this.push.body,Array.from(d.xipos.slice(this.push.body*3,this.push.body*3+3)),this.push.force);this.push.remaining-=this.dt;if(this.push.remaining<=0)this.push=null;}
  this.mj.mj_step(this.model,d);this.steps++;if(!Array.from(d.qpos).every(Number.isFinite)||!Array.from(d.qvel).every(Number.isFinite))throw Error('État physique non fini : simulation arrêtée');
 }
 forward(){this.mj.mj_forward(this.model,this.data);}
 reset(){this.grab=null;this.push=null;this.lastForce=0;this.mj.mj_resetData(this.model,this.data);this.forward();this.resets++;}
 geometries(){const m=this.model,d=this.data,result=[];for(let i=0;i<m.ngeom;i++){const body=m.geom_bodyid[i];if(!body)continue;const p=Array.from(d.geom_xpos.slice(i*3,i*3+3)),mat=d.geom_xmat.slice(i*9,i*9+9),radius=m.geom_size[i*3],half=m.geom_type[i]===3?m.geom_size[i*3+1]:0,axis=[mat[2],mat[5],mat[8]];result.push({id:i,body,name:this.names[body],a:sub(p,mul(axis,half)),b:add(p,mul(axis,half)),radius});}return result;}
 state(){return {engine:'MuJoCo '+this.mj.mj_versionString(),time:this.data.time,steps:this.steps,resets:this.resets,contacts:this.data.ncon,grab:this.grab?{body:this.grab.body,target:this.grab.target}:null,force:this.lastForce,qpos:Array.from(this.data.qpos),qvel:Array.from(this.data.qvel),physicalBodies:Array.from(this.model.body_mass).filter(x=>x>0).length,actuators:this.model.nu};}
 dispose(){this.release();this.data.delete();this.model.delete();}
}
