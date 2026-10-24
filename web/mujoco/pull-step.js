// Local, opt-in experiment. Joint REFERENCES only; Physics.control owns all torques.
// All FK/Jacobian work uses a separate MjData. The physical root is never written.
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x)),add=(a,b)=>a.map((x,i)=>x+b[i]),sub=(a,b)=>a.map((x,i)=>x-b[i]),mul=(a,k)=>a.map(x=>x*k);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const rot=(r,v)=>[0,1,2].map(i=>r[3*i]*v[0]+r[3*i+1]*v[1]+r[3*i+2]*v[2]);
const inv=(r,v)=>[0,1,2].map(i=>r[i]*v[0]+r[3+i]*v[1]+r[6+i]*v[2]);
const relR=(a,b)=>[0,1,2].flatMap(i=>[0,1,2].map(j=>a[i]*b[j]+a[3+i]*b[3+j]+a[6+i]*b[6+j]));
const column=(r,i)=>[r[i],r[3+i],r[6+i]];
const orientationError=(now,goal)=>mul([0,1,2].reduce((e,i)=>add(e,cross(column(now,i),column(goal,i))),[0,0,0]),.5);
export const PULL_PROTOCOL=Object.freeze({prepareSeconds:10,interactionSeconds:6,maxTicks:12800,body:15,targetOffsetX:.25,rampSeconds:.5,releaseAfter:3.8,forceTriggerN:8,captureAdvanceTriggerM:.008,velocityTriggerMps:.02,transferSeconds:1.8,unloadDeadline:2.3,unloadN:15,unloadDwell:.075,retainedN:100,clearTargetM:.02,clearRequiredM:.008,clearDwell:.05,liftSeconds:.4,liftDeadline:.7,transportSeconds:.6,lowerSeconds:.5,landingDeadline:.8,returnSeconds:.6,calmSeconds:1,footSlipMaxM:.025,tiltMaxRad:20*Math.PI/180,speedCalmMps:.03,taskResidualM:.002});
export const smooth=x=>{x=clamp(x,0,1);return x*x*x*(10+x*(-15+6*x));};
// Exact cubic extrema/derivative checks, not a sampled enlarged ROM tolerance.
// Only an explicitly supplied, byte-identical measured entry may start outside.
export function jointCurve(nodes,joints,model,duration,measuredAnchor=null){
 if(!(duration>0&&Number.isFinite(duration)))throw Error('Reference duration');
 const n=nodes.length-1;
 if(measuredAnchor&&nodes[0].some((v,i)=>v!==measuredAnchor[i]))throw Error('Measured anchor mismatch');
 const tangent=(i,j)=>{if(i===0)return nodes[1][j]-nodes[0][j];if(i===n)return nodes[n][j]-nodes[n-1][j];const l=nodes[i][j]-nodes[i-1][j],r=nodes[i+1][j]-nodes[i][j];return l*r<=0?0:2*l*r/(l+r);};
 const poly=(i,j)=>{const a=nodes[i][j],b=nodes[i+1][j],m=tangent(i,j),k=tangent(i+1,j);return [a,m,3*(b-a)-2*m-k,2*(a-b)+m+k];};
 const val=(c,t)=>c[0]+t*(c[1]+t*(c[2]+t*c[3]));
 const derivative=(c,t)=>c[1]+t*(2*c[2]+3*c[3]*t);
 const roots=(a,b,c)=>{if(Math.abs(a)<1e-15)return Math.abs(b)<1e-15?[]:[-c/b];const d=b*b-4*a*c;return d<0?[]:[(-b-Math.sqrt(d))/(2*a),(-b+Math.sqrt(d))/(2*a)];};
 let maxDu=0,maxDuu=0;const entrance=[];
 for(const j of joints){
  const lo=model.jnt_range[2*j.j],hi=model.jnt_range[2*j.j+1],q0=nodes[0][j.q],side=q0<lo?-1:q0>hi?1:0;
  if(side&&!measuredAnchor)throw Error('Reference ROM: unapproved entry '+j.name);
  if(side)entrance.push({joint:j.name,measured:q0,bound:side<0?lo:hi,excess:side<0?lo-q0:q0-hi});
  for(let i=0;i<n;i++){
   const c=poly(i,j.q);if(!c.every(Number.isFinite))throw Error('Nonfinite reference');
   const extrema=[0,1,...roots(3*c[3],2*c[2],c[1]).filter(t=>t>0&&t<1)];
   const dExtrema=[0,1,...(Math.abs(c[3])>1e-15?[-c[2]/(3*c[3])].filter(t=>t>0&&t<1):[])];
   const bridge=i===0&&side!==0;
   if(bridge&&!(nodes[1][j.q]>=lo-1e-9&&nodes[1][j.q]<=hi+1e-9))throw Error('Entry does not reach ROM');
   for(const t of extrema){const q=val(c,t);if(q<Math.min(lo,bridge?q0:lo)-1e-9||q>Math.max(hi,bridge?q0:hi)+1e-9)throw Error('Reference ROM: '+j.name);}
   if(bridge&&dExtrema.some(t=>side*derivative(c,t)>1e-12))throw Error('Entry excess increases: '+j.name);
   maxDu=Math.max(maxDu,...dExtrema.map(t=>Math.abs(derivative(c,t))*n));
   maxDuu=Math.max(maxDuu,Math.abs(2*c[2])*n*n,Math.abs(2*c[2]+6*c[3])*n*n);
  }
 }
 const at=u=>{const x=clamp(u,0,1)*n,i=Math.min(n-1,Math.floor(x)),t=x-i;return nodes[i].map((v,j)=>j<7?v:val(poly(i,j),t));};
 // Composition q(s(t/T)): exact cubic extrema and global bounds of quintic
 // s'=1.875, |s''|=10/sqrt(3). No physical velocity cap is invented here.
 return {at,maxSpeed:maxDu*1.875/duration,maxAcceleration:(maxDuu*1.875**2+maxDu*10/Math.sqrt(3))/duration**2,entrance,firstAdmittedProgress:1/n};
}
export class RelativeFootIK {
 constructor(p,stance=7,free=10){
  this.p=p;this.m=p.model;this.mj=p.mj;this.d=new p.mj.MjData(p.model);this.stance=stance;this.free=free;
  this.joints=p.joints.filter(j=>/abdomen|hip|knee|ankle/.test(j.name));this.n=this.joints.length;
  this.jp=new p.mj.DoubleBuffer(3*this.m.nv);this.jr=new p.mj.DoubleBuffer(3*this.m.nv);this.jc=new p.mj.DoubleBuffer(3*this.m.nv);this.matrix=new p.mj.DoubleBuffer(this.n*this.n);this.solution=new p.mj.DoubleBuffer(this.n);
 }
 set(q){this.d.qpos.set(q);this.mj.mj_kinematics(this.m,this.d);this.mj.mj_comPos(this.m,this.d);}
 pos(b){return Array.from(this.d.xpos.slice(3*b,3*b+3));}
 mat(b){return Array.from(this.d.xmat.slice(9*b,9*b+9));}
 features(){const s=this.pos(this.stance),r=this.mat(this.stance);return {com:inv(r,sub(Array.from(this.d.subtree_com.slice(3,6)),s)),foot:inv(r,sub(this.pos(this.free),s)),footR:relR(r,this.mat(this.free)),torsoR:relR(r,this.mat(1))};}
 bodyJac(b){this.mj.mj_jacBody(this.m,this.d,this.jp,this.jr,b);return {p:Array.from(this.jp.GetView()),r:Array.from(this.jr.GetView())};}
 jacobians(){
  const nv=this.m.nv,stance=this.bodyJac(this.stance),free=this.bodyJac(this.free),torso=this.bodyJac(1),r=this.mat(this.stance),origin=this.pos(this.stance);
  this.mj.mj_jacSubtreeCom(this.m,this.d,this.jc,1);const com=Array.from(this.jc.GetView());
  const col=(j,v)=>[j[v],j[nv+v],j[2*nv+v]];
  const relative=(j,point)=>this.joints.map(({v})=>inv(r,sub(sub(col(j,v),col(stance.p,v)),cross(col(stance.r,v),sub(point,origin)))));
  const angular=j=>this.joints.map(({v})=>inv(r,sub(col(j,v),col(stance.r,v))));
  const transpose=cols=>[0,1,2].map(i=>cols.map(c=>c[i]));
  return [...transpose(relative(com,Array.from(this.d.subtree_com.slice(3,6)))),...transpose(relative(free.p,this.pos(this.free))),...transpose(angular(free.r)),...transpose(angular(torso.r))];
 }
 solve(seed,goal){
  let q=seed.slice(),errors=null;
  for(let it=0;it<70;it++){
   this.set(q);const f=this.features();errors=[...sub(goal.com,f.com),...sub(goal.foot,f.foot),...orientationError(f.footR,goal.footR),...orientationError(f.torsoR,goal.torsoR)];
   if(Math.max(...errors.filter((_,i)=>i<6&&i!==2).map(Math.abs))<.00015&&Math.max(...errors.slice(6).map(Math.abs))<.001)break;
   const jac=this.jacobians(),weights=[1,1,.1,2,2,2,.2,.2,.2,.2,.2,.2],a=this.matrix.GetView(),rhs=Array(this.n).fill(0);a.fill(0);
   for(let i=0;i<this.n;i++){
    for(let k=0;k<12;k++)rhs[i]+=jac[k][i]*weights[k]**2*errors[k];
    for(let j=0;j<this.n;j++)for(let k=0;k<12;k++)a[i*this.n+j]+=jac[k][i]*jac[k][j]*weights[k]**2;
    a[i*this.n+i]+=.00001;
   }
   const normal=Array.from(a),blocked=new Set();let step;
   // Re-solve after excluding outward motion at an active ROM bound. Clipping
   // an unconstrained direction would repeatedly spend motion on a blocked hip.
   for(let pass=0;pass<=this.n;pass++){
    a.set(normal);const b=rhs.slice();for(const i of blocked){for(let j=0;j<this.n;j++)a[i*this.n+j]=a[j*this.n+i]=0;a[i*this.n+i]=1;b[i]=0;}
    if(this.mj.mju_cholFactor(this.matrix,1e-12)!==this.n)throw Error('Relative IK rank');
    this.mj.mju_cholSolve(this.solution,Array.from(a),b);step=Array.from(this.solution.GetView());
    let changed=false;this.joints.forEach((j,i)=>{if(!blocked.has(i)&&((q[j.q]<=this.m.jnt_range[2*j.j]+1e-7&&step[i]<0)||(q[j.q]>=this.m.jnt_range[2*j.j+1]-1e-7&&step[i]>0))){blocked.add(i);changed=true;}});if(!changed)break;
   }
   const scale=Math.min(1,.035/Math.max(...step.map(Math.abs)));
   this.joints.forEach((j,i)=>{q[j.q]=clamp(q[j.q]+scale*step[i],this.m.jnt_range[2*j.j],this.m.jnt_range[2*j.j+1]);});
  }
  this.set(q);const f=this.features();const positionError=Math.max(...sub(goal.com,f.com).slice(0,2).map(Math.abs),...sub(goal.foot,f.foot).map(Math.abs)),angleError=Math.max(...orientationError(f.footR,goal.footR).map(Math.abs),...orientationError(f.torsoR,goal.torsoR).map(Math.abs));
  return {q,positionError,angleError,heightError:f.com[2]-goal.com[2],features:f};
 }
 path(seed,goal,duration,measuredAnchor=null){
  this.set(seed);const initial=this.features(),nodes=[seed.slice()],reports=[];
  for(let k=1;k<=12;k++){
   const s=k/12,g={...goal,com:add(initial.com,mul(sub(goal.com,initial.com),s)),foot:add(initial.foot,mul(sub(goal.foot,initial.foot),s))};
   const result=this.solve(nodes.at(-1),g);reports.push(result);if(result.positionError>PULL_PROTOCOL.taskResidualM||result.angleError>.015||Math.abs(result.heightError)>.03)throw Error(`Relative FK not admitted: ${result.positionError.toFixed(6)} m / ${result.angleError.toFixed(6)} rad`);nodes.push(result.q);
  }
  // COM height is a secondary posture objective (30 mm envelope), not an
  // equality imposing an impossible straight-legged transfer. Horizontal COM
  // and free-foot pose remain measured FK tasks with explicit residuals.
  // Continuous monotone Hermite curve, then quintic time progress; intermediate
  // waypoints are NOT stops. Check FK also between nodes, on this same curve.
  const curve=jointCurve(nodes,this.p.joints,this.m,duration,measuredAnchor),at=curve.at;
  let residual=0;
  for(let i=0;i<=96;i++){
   const u=i/96,q=at(u);this.set(q);const f=this.features();
   residual=Math.max(residual,...sub(f.com,add(initial.com,mul(sub(goal.com,initial.com),u))).slice(0,2).map(Math.abs),...sub(f.foot,add(initial.foot,mul(sub(goal.foot,initial.foot),u))).map(Math.abs));
  }
  if(residual>PULL_PROTOCOL.taskResidualM)throw Error('Interpolated relative FK residual '+residual);
  return {...curve,duration,initial,goal,nodes,residual};
 }
 dispose(){for(const b of [this.d,this.jp,this.jr,this.jc,this.matrix,this.solution])b.delete();}
}
// Pure-gate FAILED on001: keep this candidate opt-in/off by default. Normal
// redistribution alone produced the wrong lateral acceleration and roll moment.
// Experimental load feedback: desired ground reactions are goals, never forces
// injected into the world. The existing PD remains the only motor owner.
export function loadExchange(p,servo,goalCom,com,velocity,priorDelta=null){
 const points=(p.reactions.points||[]).filter(x=>[7,10].includes(x.body)&&x.load>0),loads=p.reactions.normalLoads;
 const zero=p.joints.map(()=>0),off={active:false,delta:zero,q:servo.slice(),reason:'await solved bilateral contact points'};
 if(!(loads[7]>1&&loads[10]>1)||!points.some(x=>x.body===7)||!points.some(x=>x.body===10))return off;
 if(points.some(x=>x.normal[2]<.999))return {...off,reason:'flat-ground normal model unavailable'};
 const N=loads[7]+loads[10],mass=Array.from(p.model.body_mass).reduce((a,b)=>a+b,0),h=com[2]-points.reduce((s,x)=>s+x.position[2]*x.load/N,0);
 if(!(h>.3&&N>1))return {...off,reason:'invalid height/load'};
 const centroid=b=>points.filter(x=>x.body===b).reduce((a,x)=>add(a,mul(x.position,x.load/loads[b])),[0,0,0]);
 const right=centroid(7),left=centroid(10),dy=left[1]-right[1];if(!(dy>.02))return {...off,reason:'lateral support geometry degenerate'};
 const omega=Math.sqrt(9.81/h),ay=omega*omega*(goalCom[1]-com[1])-2*omega*velocity[1];
 const external=p.grabForce(),application=p.grab?p.worldPoint(p.grab.body,p.grab.local):com,externalMoment=cross(sub(application,com),external);
 // Approximate zero roll-angular-momentum-rate target. The external arm moment
 // and ground tangential reaction required by ay are included, not silently zero.
 const groundY=mass*ay-external[1],copWanted=com[1]+(-h*groundY-externalMoment[0])/N;
 const rightWanted=N*(left[1]-copWanted)/dy;
 const rightTarget=clamp(rightWanted,Math.min(PULL_PROTOCOL.retainedN,N),N),leftTarget=N-rightTarget;
 const copCommand=(rightTarget*right[1]+leftTarget*left[1])/N;
 const groundYCommanded=(-N*(copCommand-com[1])-externalMoment[0])/h,ayCommanded=(groundYCommanded+external[1])/mass;
 const jp=new p.mj.DoubleBuffer(3*p.model.nv),jr=new p.mj.DoubleBuffer(3*p.model.nv),generalized=Array(p.model.nv).fill(0),deltaForce=[0,0,0],deltaMoment=[0,0,0];
 const changes=[];
 for(const point of points){
  const total=loads[point.body],target=point.body===7?rightTarget:leftTarget,dN=(target-total)*point.load/total,df=mul(point.normal,dN);
  p.mj.mj_jac(p.model,p.data,jp,jr,point.position,point.body);const j=jp.GetView();
  for(let v=0;v<p.model.nv;v++)for(let k=0;k<3;k++)generalized[v]-=j[k*p.model.nv+v]*df[k];
  const moment=cross(sub(point.position,com),df);for(let k=0;k<3;k++){deltaForce[k]+=df[k];deltaMoment[k]+=moment[k];}
  changes.push({body:point.body,position:point.position,normal:point.normal,measuredN:point.load,targetN:point.load+dN});
 }
 jp.delete();jr.delete();
 // -Jc^T delta-f has Nm units. Kp^-1 maps only its ACTUATED part to radians.
 // The nonzero base part is diagnostic, never written to qfrc_applied.
 const wanted=p.joints.map(j=>generalized[j.v]/(/hip|knee|ankle/.test(j.name)?300:200));
 // Reuse the already declared25ms command-entry timescale, no integral state.
 const blend=1-Math.exp(-p.dt/.025),delta=wanted.map((x,i)=>(priorDelta?.[i]||0)+blend*(x-(priorDelta?.[i]||0))),q=servo.slice(),projection=[];
 for(const [i,j]of p.joints.entries()){const raw=q[j.q]+delta[i];q[j.q]=clamp(raw,p.model.jnt_range[2*j.j],p.model.jnt_range[2*j.j+1]);projection.push(raw-q[j.q]);}
 return {active:true,q,delta,wanted,requestedTau:p.joints.map(j=>generalized[j.v]),realizedReferenceTau:p.joints.map(j=>(/hip|knee|ankle/.test(j.name)?300:200)*(q[j.q]-servo[j.q])),unactuatedGeneralizedNotApplied:generalized.slice(0,6),deltaForce,deltaMoment,ayWanted:ay,ayCommanded,groundYWanted:groundY,groundYCommanded,externalMoment,copWanted,copCommand,copResidual:copWanted-copCommand,rightTarget,leftTarget,measured:[loads[7],loads[10]],changes,projection,omega,desiredMomentRateX:0,targetIsApproximate:true};
}
export class PullStep {
 constructor(p,{loadFeedback=false}={}){this.loadFeedback=loadFeedback;this.p=p;this.phase='armed';this.events=[];this.quietCom=null;this.stance=7;this.free=10;this.ik=new RelativeFootIK(p);this.lastTick=-1;this.stable=0;this.saturation=0;this.released=false;this.complete=false;}
 event(phase){this.phase=phase;this.phaseStart=this.p.data.time;this.events.push({phase,t:this.phaseStart});this.dwell=0;}
 fail(reason){if(!this.reason){this.reason=reason;this.event('failed');}return null;}
 release(){this.released=true;}
 snapshot(){return {phase:this.phase,reason:this.reason||null,events:this.events,stance:this.stance,free:this.free,destination:this.destination,stable:this.stable,clearDwell:this.clearDwell||0,measuredAdvance:this.measuredAdvance||0,flightAdvance:this.flightAdvance||0,loadFeedback:this.loadFeedback,loadIntent:this.loadIntent?{active:this.loadIntent.active,reason:this.loadIntent.reason,ayWanted:this.loadIntent.ayWanted,ayCommanded:this.loadIntent.ayCommanded,rightTarget:this.loadIntent.rightTarget,leftTarget:this.loadIntent.leftTarget,measured:this.loadIntent.measured,copWanted:this.loadIntent.copWanted,copCommand:this.loadIntent.copCommand,deltaMoment:this.loadIntent.deltaMoment,requestedTau:this.loadIntent.requestedTau,realizedReferenceTau:this.loadIntent.realizedReferenceTau,unactuatedGeneralizedNotApplied:this.loadIntent.unactuatedGeneralizedNotApplied}:null,referenceResidual:this.path?.residual,maxReferenceSpeed:this.path?.maxSpeed,progress:this.progress,servoLimitResidual:this.servoLimitResidual||[],loadShare:this.loadShare,complete:this.complete,released:this.released};}
 makePath(goal,duration,measuredAnchor=null){this.path=this.ik.path(this.qref,goal,duration,measuredAnchor);this.pathStart=this.p.data.time;this.progress=0;}
 step(com,velocity,feet){
  const p=this.p,d=p.data,c=PULL_PROTOCOL,t=d.time,loads=p.reactions.normalLoads;
  if(['failed','complete','cancelled'].includes(this.phase))return this.reference||null;
  if(this.phase==='armed'){
   if(p.balance.tilt<c.tiltMaxRad&&Math.hypot(...velocity)<.03&&(loads[7]||0)>50&&(loads[10]||0)>50&&!p.grab)this.quietCom=com.slice();
   const force=p.grabForce(),capture=com[0]+velocity[0]*Math.sqrt(Math.max(.3,com[2])/9.81);
   if(!this.quietCom||!p.grab||![11,12,13,14,15,16].includes(p.grab.body)||force[0]<c.forceTriggerN||force[0]<Math.abs(force[1])||!(capture-this.quietCom[0]>c.captureAdvanceTriggerM||velocity[0]>c.velocityTriggerMps))return null;
   if((loads[7]||0)<50||(loads[10]||0)<50)return this.fail('initial double support absent');
   this.qref=Array.from(d.qpos);this.entryQ=this.qref.slice();this.servoOffset=Array.from(d.qpos,()=>0);for(const j of p.joints)this.servoOffset[j.q]=(j.name.includes('elbow')?-.15:0)-d.qpos[j.q];this.entryTorques=p.commandPrevious?.map(x=>x.applied)||p.joints.map(()=>0);this.entry=t;this.stanceOrigin=feet[0].slice();this.freeOrigin=feet[1].slice();this.ik.set(this.qref);this.initial=this.ik.features();
   const r=Array.from(d.xmat.slice(63,72));this.loadShare=.5;this.stanceR=r;this.feedbackOffset=inv(r,sub(mul(add(feet[0],feet[1]),.5),com));
   // Placement is linked to the measured forward capture displacement and force.
   // It is a bounded first step, not a claim that capture-point geometry proves viability.
   this.advance=clamp(capture-feet[1][0]+force[0]/(40.84*9.81)*.2,.04,.09);
   this.destination=add(feet[1],[this.advance,0,0]);
   // Support-relative COM goal: centre of actual two-capsule stance footprint.
   // Keep initial sagittal COM coordinate, limited to interior of sole length.
   this.transferGoal={...this.initial,com:[clamp(this.initial.com[0],-.04,.10),0,this.initial.com[2]-.005]};
   try{this.makePath(this.transferGoal,c.transferSeconds,this.entryQ);}catch(e){return this.fail(String(e));}
   this.event('transfer');
  }
  if(p.fallen)return this.fail('measured fall');
  if(p.reactions.supportedBodies.some(b=>![7,10].includes(b)))return this.fail('loaded non-foot contact');
  const stanceSlip=Math.hypot(feet[0][0]-this.stanceOrigin[0],feet[0][1]-this.stanceOrigin[1]);
  if(stanceSlip>c.footSlipMaxM)return this.fail('retained foot drift exceeded 25 mm');
  if((loads[7]||0)<1){this.supportLost=(this.supportLost||0)+p.dt;if(this.supportLost>=.025)return this.fail('retained support lost');}else this.supportLost=0;
  if(p.balance.tilt>c.tiltMaxRad)return this.fail('torso tilt exceeded 20 degrees');
  if(this.released&&this.phase==='transfer'){this.event('cancel-return');this.makePath(this.initial,c.returnSeconds);}
  const elapsed=t-this.phaseStart;
  if(this.phase==='transfer'){
   this.dwell=(loads[10]||0)<c.unloadN&&(loads[7]||0)>c.retainedN?this.dwell+p.dt:0;
   if(this.dwell>=c.unloadDwell){this.ik.set(this.qref);const f=this.ik.features(),worldMin=p.footClearance(10),up=inv(this.stanceR,[0,0,Math.max(0,c.clearTargetM-worldMin)]);this.event('lift');this.makePath({...f,com:this.transferGoal.com,foot:add(f.foot,up)},c.liftSeconds);}
   else if(elapsed>=c.unloadDeadline)return this.fail('unloading not observed by deadline');
  }else if(this.phase==='lift'){
   this.clearDwell=p.footClearance(10)>=c.clearRequiredM&&(loads[10]||0)<1?(this.clearDwell||0)+p.dt:0;
   if(this.clearDwell>=c.clearDwell){this.flightStart=feet[1].slice();this.ik.set(this.qref);const f=this.ik.features();this.event('transport');this.makePath({...f,foot:add(f.foot,inv(this.stanceR,[this.advance,0,0]))},c.transportSeconds);}
   else if(elapsed>=c.liftDeadline)return this.fail('whole sole clearance not observed');
  }else if(this.phase==='transport'&&elapsed>=c.transportSeconds){this.ik.set(this.qref);const f=this.ik.features();this.event('lower');this.makePath({...f,foot:sub(f.foot,inv(this.stanceR,[0,0,c.clearTargetM]))},c.lowerSeconds);
  }else if(this.phase==='lower'){
   this.dwell=(loads[10]||0)>30?this.dwell+p.dt:0;
   if(this.dwell>=.05){this.measuredAdvance=feet[1][0]-this.freeOrigin[0];this.flightAdvance=feet[1][0]-this.flightStart[0];this.ik.set(this.qref);const f=this.ik.features();this.event('return');this.makePath({...f,com:[f.foot[0]/2,f.foot[1]/2,this.initial.com[2]]},c.returnSeconds);}
   else if(elapsed>=c.landingDeadline)return this.fail('loaded reception absent');
  }else if(['return','cancel-return'].includes(this.phase)&&elapsed>=c.returnSeconds)this.event('settle');
  if(this.phase==='settle'){
   const calm=this.released&&(loads[7]||0)>30&&(loads[10]||0)>30&&Math.hypot(...velocity)<c.speedCalmMps&&p.balance.tilt<.2;
   this.stable=calm?this.stable+p.dt:0;
   if(this.stable>=1){this.complete=!!(this.clearDwell>=c.clearDwell&&this.measuredAdvance>=.03&&this.flightAdvance>=.03);this.event(this.complete?'complete':'cancelled');}
  }
  const s=smooth((t-this.pathStart)/this.path.duration);this.progress=s;this.qref=this.path.at(s);
  // Same time progress for joints and COM feedback, expressed in ACTUAL stance frame.
  const desired=add(this.path.initial.com,mul(sub(this.path.goal.com,this.path.initial.com),s));
  const support=add(feet[0],rot(Array.from(d.xmat.slice(63,72)),add(desired,this.feedbackOffset)));
  const servoQ=this.qref.map((x,i)=>x+this.servoOffset[i]);
  // Preserve the established PD preload: desired SHAPE delta modifies the
  // existing Balance reference, never erases its equilibrium effort. These
  // compliant servo targets are distinct from the FK geometry prediction.
  this.servoLimitResidual=[];for(const j of p.joints){const desired=servoQ[j.q];servoQ[j.q]=clamp(desired,p.model.jnt_range[j.j*2],p.model.jnt_range[j.j*2+1]);if(desired!==servoQ[j.q])this.servoLimitResidual.push({joint:j.name,desired,commanded:servoQ[j.q],residual:desired-servoQ[j.q]});}
  const total=(loads[7]||0)+(loads[10]||0);if(total>1)this.loadShare+=(1-Math.exp(-p.dt/.05))*((loads[7]||0)/total-this.loadShare);
  if(this.loadFeedback&&this.phase==='transfer'){
   const goalWorld=add(feet[0],rot(Array.from(d.xmat.slice(63,72)),this.transferGoal.com));
   this.loadIntent=loadExchange(p,servoQ,goalWorld,com,velocity,this.loadDelta);this.loadDelta=this.loadIntent.delta;
   for(const j of p.joints)servoQ[j.q]=this.loadIntent.q[j.q];
  }else if(this.loadFeedback){
   // Fade the previous reference offset instead of dropping it at a phase edge.
   this.loadDelta=(this.loadDelta||p.joints.map(()=>0)).map(x=>x*Math.exp(-p.dt/.025));
   for(const [i,j]of p.joints.entries())servoQ[j.q]=clamp(servoQ[j.q]+this.loadDelta[i],p.model.jnt_range[2*j.j],p.model.jnt_range[2*j.j+1]);
   this.loadIntent={active:false,reason:'no bilateral transfer reaction requested'};
  }
  this.reference={q:servoQ,geometryQ:this.qref,support,loadShare:this.loadShare,blend:clamp((t-this.entry)/.025,0,1),entryTorques:this.entryTorques};return this.reference;
 }
 dispose(){this.ik.dispose();}
}
