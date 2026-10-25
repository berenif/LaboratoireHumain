// Experimental source_step_probe task port. No simulation, state writes, or forces applied.
// Contact forces are auxiliary predictions; native soft-contact response remains authoritative.
import assert from 'node:assert/strict';
import {dot} from './pull-constrained-ls.mjs';
import {constrainedDirectLS} from './pull-direct-qr.mjs';
import {denseConstraintJacobian} from './pull-native-step-map.mjs';
export const SOURCE_TASKS=Object.freeze({com:{kp:45,kd:13,weight:12},torso:{kp:40,kd:12,weight:4},foot:{kp:100,kd:20,stanceWeight:25,swingWeight:20},posture:{kp:20,kd:8,weight:.2},motorRegularizer:1e-3,accelerationRegularizer:1e-5,forceRegularizer:1e-8,shearWeight:.1,normalWeight:.001,stoppingHorizon:.12,transferSeconds:1.8,liftSeconds:.7,unloadDeadline:2.3,liftDeadline:.7,transitionSeconds:.025});
export const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),mul=(a,s)=>a.map(v=>v*s),norm=a=>Math.hypot(...a),maxabs=a=>Math.max(0,...a.map(Math.abs));
export function clock(t,T){const x=Math.max(0,Math.min(1,t/T));return [10*x**3-15*x**4+6*x**5,(30*x*x-60*x**3+30*x**4)/T,(60*x-180*x*x+120*x**3)/(T*T)];}
export function orientationError(target,current){
 const R=Array.from({length:9},(_,i)=>dot(target.slice(Math.floor(i/3)*3,Math.floor(i/3)*3+3),current.slice(i%3*3,i%3*3+3))),c=Math.max(-1,Math.min(1,(R[0]+R[4]+R[8]-1)/2)),theta=Math.acos(c),v=[R[7]-R[5],R[2]-R[6],R[3]-R[1]];
 assert(theta<Math.PI-.01,'SO3 logarithm outside qualified local domain');return mul(v,theta<1e-7?.5:theta/(2*Math.sin(theta)));
}
export function capCheck(u){assert.equal(u.length,21);assert(u.every(x=>Number.isFinite(x)&&Math.abs(x)<=1+1e-10),'Actual motor cap/nonfinite');}
export function forceRows(contacts,n){
 const rows=[];contacts.forEach((c,i)=>{assert(c.friction[0]>0&&c.friction[1]>0&&c.dim===3);const start=21+3*i;
  for(const a of[-1,1])for(const b of[-1,1]){const row=Array(n).fill(0);row[start]=-1;row[start+1]=a/c.friction[0];row[start+2]=b/c.friction[1];rows.push({label:`contact-${i}-cone-${a}-${b}`,a:row,bound:0});}
  const row=Array(n).fill(0);row[start]=-1;rows.push({label:`contact-${i}-normal`,a:row,bound:0});
 });return rows;
}
export function contactRecords(p){
 const {mj,model:m,data:d}=p,cs=d.contact,out=[];
 try{for(let i=0;i<d.ncon;i++){const c=cs.get(i);try{const a=m.geom_bodyid[c.geom1],b=m.geom_bodyid[c.geom2];if(a!==0&&b!==0)continue;const body=a||b;
  mj.mj_contactForce(m,d,i,p.contactBuffer);out.push({index:i,geom1:c.geom1,geom2:c.geom2,body,sign:a===0?1:-1,dim:c.dim,position:Array.from(c.pos),frame:Array.from(c.frame),friction:Array.from(c.friction),distance:c.dist,force:Array.from(p.contactBuffer.GetView()),constraintAddress:c.efc_address});
 }finally{c.delete();}}}finally{cs.delete();}return out;
}
export function makeReference(p){
 const d=p.data,contacts=contactRecords(p),right=contacts.filter(c=>c.body===7&&c.constraintAddress>=0);assert(right.length,'No current right-foot contact');
 const com=Array.from(d.subtree_com.slice(3,6)),goal=com.slice();for(let k=0;k<2;k++)goal[k]=right.reduce((s,c)=>s+c.position[k],0)/right.length;
 // A convex combination of current stance points: inside their hull, including degeneracies.
 return {start:d.time,transferDeadline:d.time+SOURCE_TASKS.unloadDeadline,phase:'transfer',phaseStart:d.time,com,goal,torso:Array.from(d.xmat.slice(9,18)),feet:[7,10].map(b=>({body:b,position:Array.from(d.xpos.slice(3*b,3*b+3)),rotation:Array.from(d.xmat.slice(9*b,9*b+9))})),joint:p.joints.map(j=>Math.max(p.model.jnt_range[2*j.j],Math.min(p.model.jnt_range[2*j.j+1],d.qpos[j.q]))),unloaded:0,clearDwell:0,events:[{phase:'transfer',t:d.time}],targetConstruction:{method:'equal convex weights of current native right-foot points',points:right.map(c=>c.position),weights:right.map(()=>1/right.length)},entryControls:Array.from(d.ctrl)};
}
export function targets(ref,time,clearance){
 const c=SOURCE_TASKS,[s,v,a]=clock(time-ref.start,c.transferSeconds),delta=sub(ref.goal,ref.com),moving=ref.phase==='transfer';
 const out={com:{position:moving?add(ref.com,mul(delta,s)):ref.goal,velocity:moving?mul(delta,v):[0,0,0],acceleration:moving?mul(delta,a):[0,0,0]},feet:structuredClone(ref.feet),torso:ref.torso,joints:ref.joint};
 for(const f of out.feet){f.velocity=[0,0,0];f.acceleration=[0,0,0];}
 if(ref.phase==='lift'){const [s,v,a]=clock(time-ref.phaseStart,c.liftSeconds),f=out.feet[1];f.position[2]+=ref.liftDelta*s;f.velocity[2]=ref.liftDelta*v;f.acceleration[2]=ref.liftDelta*a;}
 return out;
}
export function updatePhase(ref,m,dt){
 if(ref.phase==='transfer'){
  ref.unloaded=m.loads[1]<15&&m.loads[0]>100?ref.unloaded+dt:0;
  if(ref.unloaded>=.075&&m.t-ref.start>=SOURCE_TASKS.transferSeconds){ref.phase='lift';ref.phaseStart=m.t;ref.liftDelta=Math.max(0,.02-m.clear[1]);ref.feet[1].position=m.feet[1].slice();ref.events.push({phase:'lift',t:m.t,loads:m.loads});}
  else if(m.t>=ref.transferDeadline)return 'unloading not observed by deadline';
 }else if(ref.phase==='lift'){
  ref.clearDwell=m.clear[1]>=.008&&m.loads[1]<1?ref.clearDwell+dt:0;
  if(ref.clearDwell>=.1)return 'qualified whole-sole lift';
  if(m.t-ref.phaseStart>=SOURCE_TASKS.liftDeadline)return 'whole sole clearance not observed';
 }return null;
}
function jacobians(p,point,body){const {mj,model:m,data:d}=p,b=Array.from({length:4},()=>new mj.DoubleBuffer(3*m.nv));try{mj.mj_jac(m,d,b[0],b[1],point,body);mj.mj_jacDot(m,d,b[2],b[3],point,body);return {p:Array.from(b[0].GetView()),r:Array.from(b[1].GetView()),pd:Array.from(b[2].GetView()),rd:Array.from(b[3].GetView())};}finally{b.forEach(x=>x.delete());}}
// Exact direct rows of the existing quadratic objective, including regularizers.
export function directCost(qp){
 const {n,tasks,contacts}=qp,C=tasks.map(t=>t.a.map(v=>v*t.weight/t.scale)),d=tasks.map(t=>(t.desired-t.offset)*t.weight/t.scale);
 for(let i=0;i<n;i++){const row=Array(n).fill(0);row[i]=Math.sqrt(i<21?SOURCE_TASKS.motorRegularizer:SOURCE_TASKS.forceRegularizer);C.push(row);d.push(0);}
 for(let i=0;i<contacts.length;i++)for(let k=0;k<3;k++){const w=k?SOURCE_TASKS.shearWeight:SOURCE_TASKS.normalWeight,row=Array(n).fill(0);row[21+3*i+k]=w;C.push(row);d.push(k?0:w/contacts.length);}
 return {C,d};
}
export function assemble(p,interaction,ref){
 const {mj,model:m,data:d,joints}=p,nv=m.nv;assert.equal(nv,27);assert.equal(joints.length,21);
 const buf=new mj.DoubleBuffer(nv*nv);mj.mj_fullM(m,d,buf);const M=Array.from(buf.GetView());buf.delete();
 const mass=Array.from(m.body_mass).reduce((s,x)=>s+x,0),mg=mass*9.81,height=ref.com[2],rotScale=9.81/height;
 const contacts=contactRecords(p).filter(c=>c.constraintAddress>=0&&[7,10].includes(c.body));assert(contacts.length,'No present foot constraint');const n=21+3*contacts.length;
 const B=Array.from({length:n},()=>Array(nv).fill(0));for(let a=0;a<21;a++)B[a][joints[a].v]=joints[a].gear;
 contacts.forEach((c,i)=>{const j=jacobians(p,c.position,c.body);c.J=Array.from({length:3},(_,k)=>Array.from({length:nv},(_,a)=>c.sign*(c.frame[3*k]*j.p[a]+c.frame[3*k+1]*j.p[nv+a]+c.frame[3*k+2]*j.p[2*nv+a])));for(let k=0;k<3;k++)B[21+3*i+k]=mul(c.J[k],mg);});
 const grabJ=jacobians(p,interaction.point,15).p,external=Array.from({length:nv},(_,i)=>dot([grabJ[i],grabJ[nv+i],grabJ[2*nv+i]],interaction.force));
 const Jefc=denseConstraintJacobian(m,d),limits=Array(nv).fill(0),limitRows=[];
 for(let k=0;k<d.nefc;k++)if(d.efc_type[k]===3){const f=d.efc_force[k];limitRows.push({joint:d.efc_id[k],force:f,jacobian:Jefc[k]});for(let i=0;i<nv;i++)limits[i]+=Jefc[k][i]*f;}
 const rhs=Array.from({length:nv},(_,i)=>d.qfrc_passive[i]-d.qfrc_bias[i]+external[i]+limits[i]);
 const factor=new mj.DoubleBuffer(nv*nv),solution=new mj.DoubleBuffer(nv);factor.GetView().set(M);let A,a0;
 try{assert.equal(mj.mju_cholFactor(factor,1e-12),nv);const fact=Array.from(factor.GetView()),solve=r=>{mj.mju_cholSolve(solution,fact,r);return Array.from(solution.GetView());};a0=solve(rhs);A=B.map(solve);}finally{factor.delete();solution.delete();}
 // qdd = a0 + sum A_j*x_j; x=(normalized motor controls, contact-frame forces/BW).
 const H=Array(n*n).fill(0),b=Array(n).fill(0),tasks=[],rows=forceRows(contacts,n),target=targets(ref,d.time);
 for(let j=0;j<n;j++)H[j*n+j]=j<21?SOURCE_TASKS.motorRegularizer:SOURCE_TASKS.forceRegularizer;
 function least(label,jac,desired,weight,scale=1,bias=0){const a=A.map(col=>dot(jac,col)),offset=dot(jac,a0)+bias,goal=(desired-offset)/scale,w=weight/scale;
  for(let i=0;i<n;i++){b[i]+=a[i]*w*weight*goal;for(let j=0;j<n;j++)H[i*n+j]+=a[i]*a[j]*w*w;}
  tasks.push({label,jac,desired,weight,scale,bias,a,offset});
 }
 const jc=Array(3*nv).fill(0),jcd=Array(3*nv).fill(0);
 for(let body=1;body<m.nbody;body++){const weight=m.body_mass[body]/mass;if(!weight)continue;const j=jacobians(p,Array.from(d.xipos.slice(3*body,3*body+3)),body);for(let k=0;k<3*nv;k++){jc[k]+=weight*j.p[k];jcd[k]+=weight*j.pd[k];}}
 const com=Array.from(d.subtree_com.slice(3,6));
 for(let k=0;k<3;k++){const jac=jc.slice(k*nv,(k+1)*nv),vel=dot(jac,d.qvel),c=SOURCE_TASKS.com;least('com-'+k,jac,target.com.acceleration[k]+c.kp*(target.com.position[k]-com[k])+c.kd*(target.com.velocity[k]-vel),c.weight,9.81,dot(jcd.slice(k*nv,(k+1)*nv),d.qvel));}
 const torso=jacobians(p,Array.from(d.xpos.slice(3,6)),1),err=orientationError(ref.torso,Array.from(d.xmat.slice(9,18)));
 for(let k=0;k<3;k++){const j=torso.r.slice(k*nv,(k+1)*nv),c=SOURCE_TASKS.torso;least('torso-'+k,j,c.kp*err[k]-c.kd*dot(j,d.qvel),c.weight,rotScale,dot(torso.rd.slice(k*nv,(k+1)*nv),d.qvel));}
 for(const f of target.feet){const js=jacobians(p,Array.from(d.xpos.slice(3*f.body,3*f.body+3)),f.body),pos=Array.from(d.xpos.slice(3*f.body,3*f.body+3)),error=orientationError(f.rotation,Array.from(d.xmat.slice(9*f.body,9*f.body+9))),c=SOURCE_TASKS.foot,w=f.body===10&&ref.phase==='lift'?c.swingWeight:c.stanceWeight;
  for(let k=0;k<3;k++){const j=js.p.slice(k*nv,(k+1)*nv);least(`foot-${f.body}-position-${k}`,j,f.acceleration[k]+c.kp*(f.position[k]-pos[k])+c.kd*(f.velocity[k]-dot(j,d.qvel)),w,9.81,dot(js.pd.slice(k*nv,(k+1)*nv),d.qvel));const jr=js.r.slice(k*nv,(k+1)*nv);least(`foot-${f.body}-rotation-${k}`,jr,c.kp*error[k]-c.kd*dot(jr,d.qvel),w,rotScale,dot(js.rd.slice(k*nv,(k+1)*nv),d.qvel));}
 }
 for(let a=0;a<21;a++){const j=joints[a],jac=Array(nv).fill(0);jac[j.v]=1;const c=SOURCE_TASKS.posture;least('posture-'+j.name,jac,c.kp*(target.joints[a]-d.qpos[j.q])-c.kd*d.qvel[j.v],c.weight,rotScale);}
 for(let i=0;i<nv;i++){const jac=Array(nv).fill(0);jac[i]=1;least('acceleration-regularizer-'+i,jac,0,Math.sqrt(SOURCE_TASKS.accelerationRegularizer),i<3?9.81:rotScale);}
 for(let i=0;i<contacts.length;i++)for(let k=0;k<3;k++){const idx=21+3*i+k,w=k?SOURCE_TASKS.shearWeight:SOURCE_TASKS.normalWeight,goal=k?0:1/contacts.length;H[idx*n+idx]+=w*w;b[idx]+=w*w*goal;}
 for(let a=0;a<21;a++){for(const s of[-1,1]){const row=Array(n).fill(0);row[a]=s;rows.push({label:`motor-${a}-${s}`,a:row,bound:1});}
  const j=joints[a],h=SOURCE_TASKS.stoppingHorizon,q=d.qpos[j.q],v=d.qvel[j.v];for(const s of[-1,1]){const limit=m.jnt_range[2*j.j+(s===1?1:0)],acc=2*(s*(limit-q-h*v))/(h*h);rows.push({label:`joint-stopping-${j.name}-${s}`,a:A.map(col=>s*col[j.v]),bound:acc-s*a0[j.v]});}
 }
 assert([...H,...b,...rows.flatMap(r=>[...r.a,r.bound])].every(Number.isFinite),'Nonfinite assembly');
 return {...directCost({n,tasks,contacts}),n,M,B,A,a0,rhs,external,limits,limitRows,contacts,mass,mg,rows,H,b,tasks,target,scales:{linearAcceleration:9.81,angularAcceleration:rotScale,force:mg,motor:'gear'},modelAssumption:'Joint-limit reactions frozen at current native centre for this command only; soft-contact force realization is NOT imposed by this QP'};
}
export function solveWholeBody(p,interaction,ref,beforeSolve=()=>{}){
 const qp=assemble(p,interaction,ref);beforeSolve(qp);const started=performance.now(),solve=constrainedDirectLS(qp.C,qp.d,qp.rows,{maxPivots:64,tolerance:1e-10}),solveMs=performance.now()-started;
 const prediction={solve,solveMs,variables:qp.n,contacts:qp.contacts.map(({J,...c})=>c),target:qp.target,scales:qp.scales,limitRows:qp.limitRows,modelAssumption:qp.modelAssumption};
 if(!solve.solved)return {prediction,qp};
 const x=solve.x,controls=x.slice(0,21);capCheck(controls);const acceleration=qp.a0.map((v,i)=>v+qp.A.reduce((s,col,j)=>s+col[i]*x[j],0)),residual=Array.from({length:27},(_,i)=>dot(qp.M.slice(i*27,(i+1)*27),acceleration)-qp.rhs[i]-qp.B.reduce((s,col,j)=>s+col[i]*x[j],0));
 assert(maxabs(residual)<1e-7,'Full27 dynamics elimination residual');
 Object.assign(prediction,{controls,acceleration,dynamicsResidual:residual,forces:qp.contacts.map((c,i)=>({body:c.body,position:c.position,forceLocal:x.slice(21+3*i,24+3*i).map(v=>v*qp.mg)})),tasks:qp.tasks.map(t=>({label:t.label,desired:t.desired,obtained:dot(t.jac,acceleration)+t.bias,residual:(dot(t.jac,acceleration)+t.bias-t.desired)/t.scale,weight:t.weight})),constraints:qp.rows.map(r=>({label:r.label,residual:dot(r.a,x)-r.bound})),cost:solve.cost});
 return {prediction,qp};
}
