// Algebra and scratch kinematics only. No forward, inverse, step, or physical writes.
import assert from 'node:assert/strict';
import {gravityPosition,gravityTargets} from './pull-gravity-frame.mjs';
import {worldFeatures} from './pull-dynamic-stance.mjs';
import {rot,inv,mm,flat,sub,maxabs} from './pull-stance-geometry.mjs';
export function verify(p,ref,kin){
 const beforeQ=Array.from(p.data.qpos),beforeV=Array.from(p.data.qvel),I=[1,0,0,0,1,0,0,0,1],z=[0,0,0],row=v=>v.map(x=>[x]),base={pos:z,v:z,bp:z,jp:row(z)},f={pos:[.04,.09,.82],v:[.2,-.1,.3],bp:[.1,-.4,.2],jp:row([.3,-.2,.1])};let tests=0;
 const a=gravityPosition(base,f,I),offset=[1,-2,.5],moving={...base,pos:offset,v:[.1,.2,.3],bp:[.2,.3,.4],jp:row([.7,.8,.9])},translated={...f,pos:f.pos.map((x,i)=>x+offset[i]),v:f.v.map((x,i)=>x+moving.v[i]),bp:f.bp.map((x,i)=>x+moving.bp[i]),jp:row([1,.6,1])};
 const b=gravityPosition(moving,translated,I);for(const k of ['position','velocity','bias'])assert(maxabs(sub(a[k],b[k]))<1e-14);assert(maxabs(sub(a.jac.flat(),b.jac.flat()))<1e-14);tests++;
 const angle=.1,R=[1,0,0,0,Math.cos(angle),-Math.sin(angle),0,Math.sin(angle),Math.cos(angle)],rolled=gravityPosition(base,{...f,pos:rot(R,f.pos)},I);assert(Math.abs(rolled.position[1]-a.position[1])>.08,'Gravity must see common roll around support');tests++;
 assert.deepEqual(gravityPosition({...base,R},f,I),a,'Foot orientation must not rotate task axes');tests++;
 const converted=gravityTargets(ref.at(p.data.time),ref.Ranchor,ref.Rg);assert(converted.torso.rotation.every(Number.isFinite));assert(ref.Rg[2]===0&&ref.Rg[5]===0&&ref.Rg[6]===0&&ref.Rg[7]===0&&ref.Rg[8]===1);tests++;
 const scratch={...p,data:new p.mj.MjData(p.model)},at=dt=>{scratch.data.qpos.set(beforeQ);scratch.data.qvel.set(beforeV);p.mj.mj_integratePos(p.model,scratch.data.qpos,new Float64Array(beforeV),dt);kin(scratch);const x=worldFeatures(scratch);return gravityPosition(x.stance,x.com,ref.Rg);};
 let velocityError,accelerationError,jetError=0;
 try{const h=.0001,l=at(-h),m=at(0),r=at(h);velocityError=maxabs(m.velocity.map((x,k)=>(r.position[k]-l.position[k])/(2*h)-x));accelerationError=maxabs(m.bias.map((x,k)=>(r.position[k]-2*m.position[k]+l.position[k])/(h*h)-x));assert(velocityError<1e-7&&accelerationError<2e-5);tests++;
  for(const t of [.13,.43,.81,1.17,1.57]){const sample=dt=>gravityTargets(ref.at(ref.start+t+dt),ref.Ranchor,ref.Rg),l=sample(-h),m=sample(0),r=sample(h);for(const [position,velocity,acceleration] of [[l.com.position,m.com.velocity,r.com.position],[l.com.velocity,m.com.acceleration,r.com.velocity]])jetError=Math.max(jetError,maxabs(sub(sub(acceleration,position).map(x=>x/(2*h)),velocity)));}assert(jetError<1e-5);tests++;
  // Reference conversion is constant, so its world orientation jets rotate identically.
  const s=ref.at(p.data.time),t=gravityTargets(s,ref.Ranchor,ref.Rg);assert.deepEqual(t.torso.velocity,rot(ref.Ranchor,s.torso.velocity));assert.deepEqual(t.torso.acceleration,rot(ref.Ranchor,s.torso.acceleration));tests++;
  assert.deepEqual(Array.from(p.data.qpos),beforeQ);assert.deepEqual(Array.from(p.data.qvel),beforeV);ref.at(p.data.time);
  return {passed:true,tests,velocityError,accelerationError,referenceJetError:jetError,physicalStateUnchanged:true,forwards:0,ticks:0};
 }finally{scratch.data.delete();}
}
