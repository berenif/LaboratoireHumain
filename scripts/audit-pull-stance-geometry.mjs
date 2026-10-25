// Audit SAVED reference, no new optimization or dynamics. Never runs Physics.
import fs from'node:fs';import assert from'node:assert/strict';import{gunzipSync}from'node:zlib';import crypto from'node:crypto';
import load from'../web/dist/vendor/mujoco.js';import{RelativeFootIK,jointCurve}from'../web/mujoco/pull-step.js';
import{PLAN,hermite,worldReference,norm,sub,maxabs,margin,flat,mm}from'./pull-stance-geometry.mjs';
const dir=process.argv[2],ref=JSON.parse(fs.readFileSync(dir+'/reference.json')),result=JSON.parse(fs.readFileSync(dir+'/result.json'));assert(result.passed);assert(!fs.existsSync(dir+'/audit.json'),'Audit already recorded');
const cp=JSON.parse(gunzipSync(fs.readFileSync('web/proofs/pull-step-001/final.json.gz'))),native=await load(),counts={kinematics:0,comPos:0,jacobians:0,dynamicForwards:0,ticks:0,forecasts:0};const mj=new Proxy(native,{get(o,k){const v=o[k];if(typeof v!=='function')return v;if(/^mj_(forward|step|inverse|fwd|inv|collision|constraint|makeConstraint|resetData)/.test(k))return()=>{throw Error('Forbidden dynamics '+k);};if(['mj_kinematics','mj_comPos','mj_jacBody','mj_jacSubtreeCom'].includes(k))return(...args)=>{counts[k==='mj_kinematics'?'kinematics':k==='mj_comPos'?'comPos':'jacobians']++;assert(counts.kinematics+result.counts.kinematics<=PLAN.maxKinematics);return Reflect.apply(v,o,args);};return v;}});
const model=mj.MjModel.from_xml_string(fs.readFileSync('web/dist/humanoid.xml','utf8')),data=new mj.MjData(model);mj.mj_setState(model,data,cp.state,cp.spec);const state=()=>{const b=new mj.DoubleBuffer(mj.mj_stateSize(model,cp.spec));try{mj.mj_getState(model,data,b,cp.spec);return Array.from(b.GetView());}finally{b.delete();}},before=state(),p={mj,model,data,joints:ref.jointOrder},ik=new RelativeFootIK(p),curve=jointCurve(ref.nodes,p.joints,model,ref.duration,ref.nodes[0]),jets=hermite(ref.nodes);
let tests=0;const reject=(fn,re)=>{assert.throws(fn,re);tests++;},near=(a,b,tol)=>assert(maxabs(sub(a,b))<tol);
try{
 const toyModel={jnt_range:[-1,1]},toyJ=[{name:'joint',j:0,q:7}],q=x=>[0,0,0,1,0,0,0,x];
 reject(()=>jointCurve([q(1.01),q(1),q(.9)],toyJ,toyModel,1),/unapproved/);
 reject(()=>jointCurve([q(1.01),q(1),q(.9)],toyJ,toyModel,1,q(1.02)),/mismatch/);
 reject(()=>jointCurve([q(1.01),q(1.02),q(.9)],toyJ,toyModel,1,q(1.01)),/reach ROM/);
 reject(()=>jointCurve([q(1.01),q(.99),q(1.002)],toyJ,toyModel,1,q(1.01)),/ROM/);
 reject(()=>jointCurve([q(1),q(NaN)],toyJ,toyModel,1),/Nonfinite/);
 reject(()=>flat([0,0,1,0,1,0,1,0,0]),/singularity/);
 reject(()=>margin([[0,0],[1,0]],[.5,0]),/Degenerate/);
 assert(margin([[0,0],[1,0],[1,1],[0,1]],[1.01,.5])<0);tests++;
 // Invariance to a rigid transform of the common free base, including rotation.
 const seed=ref.nodes[0],a=.4,c=Math.cos(a/2),s=Math.sin(a/2),shift=seed.slice(),w=seed[3],x=seed[4],y=seed[5],z=seed[6];shift.splice(3,4,c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w);shift[0]+=1;shift[1]-=2;
 ik.set(seed);const initial=ik.features();ik.set(shift);const shifted=ik.features();for(const name of ['com','foot','footR','torsoR'])near(initial[name],shifted[name],1e-12);tests++;
 // Whole-foot bounds use the same endpoint+chord theorem, now including all
 // collider offsets in L. The retained frame is an explicit conditional anchor.
 const chain=Array.from({length:model.nbody},(_,b)=>norm(Array.from(model.body_pos.slice(3*b,3*b+3)))).reduce((s,x)=>s+x,0),geom=Array.from({length:model.ngeom},(_,g)=>norm(Array.from(model.geom_pos.slice(3*g,3*g+3)))+norm(Array.from(model.geom_size.slice(3*g,3*g+3)))),comOffsets=Array.from({length:model.nbody},(_,b)=>norm(Array.from(model.body_ipos.slice(3*b,3*b+3)))),reach=2*(chain+Math.max(...geom.slice(1),...comOffsets));
 let sampledClear=[Infinity,Infinity],lowerClear=[Infinity,Infinity],monotoneExcess=true,previous=seed.slice(),largestExcessIncrease=0,minPostEntryMargin=Infinity,intervals=0,peakSpeed=0;const at=u=>{const v=jets.at(u);ik.set(v.q);return worldReference(ik,ref.definition.stance);};
 for(let i=0;i<PLAN.nodes;i++){
  const b=jets.bounds(i),M=reach*(b.sumDuu+b.sumDu*b.sumDu);let n=1;while(M/(8*(PLAN.nodes*n)**2)>PLAN.positionToleranceM/8)n*=2;assert(intervals+n<=PLAN.maxIntervals);const allowance=M/(8*(PLAN.nodes*n)**2);let prior=at(i/PLAN.nodes);
  for(let k=1;k<=n;k++){
   const u=(i+k/n)/PLAN.nodes,next=at(u);for(let foot=0;foot<2;foot++){const clear=w=>Math.min(...w.feet[foot].capsules.flatMap(g=>[g.a[2]-g.radius,g.b[2]-g.radius]));const minimum=Math.min(clear(prior),clear(next));sampledClear[foot]=Math.min(sampledClear[foot],minimum);lowerClear[foot]=Math.min(lowerClear[foot],minimum-allowance);}
   const pos=curve.at(u);for(const j of p.joints){const lo=model.jnt_range[2*j.j],hi=model.jnt_range[2*j.j+1],excess=x=>Math.max(lo-x,x-hi,0),increase=excess(pos[j.q])-excess(previous[j.q]);largestExcessIncrease=Math.max(largestExcessIncrease,increase);if(increase>1e-12)monotoneExcess=false;if(u>=1/PLAN.nodes)minPostEntryMargin=Math.min(minPostEntryMargin,pos[j.q]-lo,hi-pos[j.q]);}previous=pos;prior=next;intervals++;
  }
 }
 assert(monotoneExcess,'Inherited soft-limit excess grows');assert(minPostEntryMargin>=-1e-10);
 // Report absolute seed/final collision bottoms; inherited floor compression
 // is exposed. The lower bound does not assert zero penetration or contact.
 const first=at(0),last=at(1),initialClear=first.feet.map(f=>Math.min(...f.capsules.flatMap(g=>[g.a[2]-g.radius,g.b[2]-g.radius]))),finalClear=last.feet.map(f=>Math.min(...f.capsules.flatMap(g=>[g.a[2]-g.radius,g.b[2]-g.radius])));
 const knotDeltas=p.joints.map(j=>({joint:j.name,deltaRad:ref.nodes.at(-1)[j.q]-seed[j.q],finalRad:ref.nodes.at(-1)[j.q]}));
 assert.deepEqual(state(),before);near(data.qpos,cp.physics.qpos,1e-15);near(data.qvel,cp.physics.qvel,1e-15);
 const report={passed:true,savedReferenceSHA256:crypto.createHash('sha256').update(fs.readFileSync(dir+'/reference.json')).digest('hex'),newCandidateSolves:0,counterexamples:tests,counts,countsAllGeometry:{kinematics:counts.kinematics+result.counts.kinematics,comPos:counts.comPos+result.counts.comPos,jacobians:counts.jacobians+result.counts.jacobians,dynamicForwards:0,ticks:0,forecasts:0},native233Unchanged:true,ROM:{analyticalHermiteExtremaChecked:true,measuredExcessMonotone:true,largestExcessIncreaseRad:largestExcessIncrease,minPostFirstKnotMarginRad:minPostEntryMargin},wholeSole:{initialClearM:initialClear,finalClearM:finalClear,sampledMinimumM:sampledClear,continuousLowerBoundM:lowerClear,intervals,reachM:reach,interpretation:'Inherited compressed-floor geometry remains explicit; geometric lower bounds do not establish contacts, loads or zero penetration.'},jointChanges:knotDeltas,referenceStartsAtMeasuredQ:true,initialMeasuredDqPreserved:true,continuityLimit:'Desired velocity at t0 is zero; measured dq is nonzero. No motor handoff is certified.'};
 const text=JSON.stringify(report)+'\n';assert(fs.readdirSync(dir).reduce((s,f)=>s+fs.statSync(dir+'/'+f).size,0)+Buffer.byteLength(text)<PLAN.maxProofBytes);fs.writeFileSync(dir+'/audit.json',text,{flag:'wx'});console.log(JSON.stringify(report));
}finally{ik.dispose();data.delete();model.delete();}
