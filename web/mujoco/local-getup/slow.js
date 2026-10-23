// Local research adapter. Not copied by build-mujoco-mvp.mjs or delivered with the site.
// Numerical contract: getup/actor.rs, getup.rs, scripts/getup-step/source_math.py.
export const ORDER=[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20];
const finite=(v,n)=>{if(v.length!==n||!Array.from(v).every(Number.isFinite))throw Error(`Expected ${n} finite values`);return v;};
export class DenseActor {
 constructor(buffer){
  const view=new DataView(buffer);if(new TextDecoder().decode(new Uint8Array(buffer,0,8))!=='LHGPW001')throw Error('Actor header');
  this.dims=[0,1,2,3].map(i=>view.getUint32(8+4*i,true));
  if(![68,118].includes(this.dims[0])||this.dims.slice(1).join(',')!=='1024,1024,42')throw Error('Actor dimensions');
  const count=this.dims.slice(1).reduce((n,out,i)=>n+(this.dims[i]+1)*out,0);
  if(buffer.byteLength!==24+count*4)throw Error('Actor size');
  this.values=new Float32Array(count);for(let i=0;i<count;i++)this.values[i]=view.getFloat32(24+4*i,true);
  if(!this.values.every(Number.isFinite))throw Error('Actor nonfinite');
 }
 infer(observation){
  let x=Float32Array.from(finite(observation,this.dims[0]));const power=x[x.length-1];
  if(!x.every(Number.isFinite)||power<0||power>1)throw Error('Actor observation');
  let offset=0;
  for(let layer=0;layer<3;layer++){
   const width=this.dims[layer],height=this.dims[layer+1],next=new Float32Array(height),bias=offset+width*height;
   for(let row=0;row<height;row++){let sum=this.values[bias+row];for(let col=0;col<width;col++)sum+=this.values[offset+row*width+col]*x[col];const f=Math.fround(sum);next[row]=layer<2?Math.max(0,f):f;}
   offset=bias+height;x=next;
  }
  if(!x.every(Number.isFinite))throw Error('Actor output');
  return {raw:Array.from(x),action:Array.from(x.slice(0,21),v=>Math.fround(Math.fround(Math.tanh(v))*power))};
 }
}
export function target(standing){return [...finite(standing,21),1.4,0,0,1];}
export function observation(physical,now,ahead){return Array.from(Float32Array.from([...finite(physical,67),...finite(now,25),...finite(ahead,25),1]));}
export function controls(ref,residual,q,dq,holding=true){
 [ref,residual,q,dq].forEach(x=>finite(x,21));if(residual.some(v=>Math.abs(v)>1))throw Error('Residual range');
 const kd=holding?.2:.1;return ORDER.map(i=>Math.max(-1,Math.min(1,ref[i]+residual[i]-q[i]-kd*dq[i])));
}
export function physicalObservation(p){
 const {mj,model:m,data:d}=p;
 // Kinematics evaluated at CURRENT q/dq. No integration, normalization of qpos or pose installation.
 mj.mj_forward(m,d);mj.mj_subtreeVel(m,d);
 const ids=['torso','head','left_hand','left_foot','right_hand','right_foot'].map(n=>p.names.indexOf(n));if(ids.some(i=>i<0))throw Error('Body mapping');
 const torso=ids[0],r=d.xmat.slice(9*torso,9*torso+9),origin=d.xpos.slice(3*torso,3*torso+3);
 const inv=v=>[0,1,2].map(i=>r[i]*v[0]+r[3+i]*v[1]+r[6+i]*v[2]);
 const ext=ids.slice(2).flatMap(id=>inv([0,1,2].map(k=>d.xpos[3*id+k]-origin[k])));
 // MuJoCo free-joint translation velocity is world-frame; angular velocity is body-local.
 return finite([...d.qpos.slice(7),d.xpos[3*ids[1]+2],...ext,r[6],r[7],r[8],...d.subtree_linvel.slice(3*torso,3*torso+3),...d.qvel.slice(0,6),...d.qvel.slice(6)],67);
}
export class SlowHolding {
 constructor(p,actor,standing){
  if(actor.dims[0]!==118||p.model.nq!==28||p.model.nv!==27||p.model.nu!==21||Math.abs(p.dt-.00125)>1e-12)throw Error('SLOW model contract');
  p.joints.forEach((j,a)=>{if(j.q-7!==ORDER[a])throw Error('Actuator mapping');});
  this.p=p;this.actor=actor;this.reference=target(standing);this.nativeTicks=0;this.decisions=0;this.residual=null;this.last=null;
 }
 command(){
  const p=this.p,d=p.data;
  if(this.nativeTicks%20===0){
   const physical=physicalObservation(p),input=observation(physical,this.reference,this.reference),prediction=this.actor.infer(input);
   this.residual=prediction.action;this.last={physical,input,raw:prediction.raw,residual:this.residual};this.decisions++;
  }
  const u=controls(this.reference.slice(0,21),this.residual,Array.from(d.qpos.slice(7)),Array.from(d.qvel.slice(6)),true);
  d.ctrl.set(u);p.command=p.joints.map(j=>({name:j.name,requested:u[j.a]*j.gear,applied:u[j.a]*j.gear}));this.nativeTicks++;
 }
}
