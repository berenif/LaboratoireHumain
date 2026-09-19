import { mkdirSync, writeFileSync } from 'node:fs';
import { createEmbodiedCharacter } from '../src/character/index.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { jointAngularVelocityCoordinates } from '../src/character/joint-coordinates.ts';
import { planContactLoads } from '../src/character/contact-loads.ts';
import { add, angularVelocity, clampLength, rotate, scale, sub, worldPoint } from '../src/character/math.ts';

// Exact slow-hand-forward fixture, with pre-integration decisions and post-step
// measurements separated so target, load, and torque timing stays unambiguous.
const dt = 1 / 60;
const prefix = process.env.PROBE_OUTPUT_PREFIX ?? 'evidence/balance-transition';
const count = Number(process.env.PROBE_FRAMES ?? 420);
const c = await createEmbodiedCharacter('canvas2d');
if (process.env.PROBE_MEASURED_HEADING) {
  const standing=c.updateStanding.bind(c);
  c.updateStanding = dt => {
    const f=rotate(c.poses.get('pelvis').rotation,{x:0,y:0,z:1});
    c.heading=Math.atan2(f.x,f.z);
    return standing(dt);
  };
}
if (process.env.PROBE_RECAPTURE_STEP) {
  const update = c.balance.update.bind(c.balance);
  c.balance.update = input => {
    const prior=c.balance.step?.elapsed;
    const result=update(input);
    if (result.step && prior<0 && result.step.elapsed>=0) {
      const d=result.diagnostics, from=input.poses.get(result.step.foot).position;
      const desired={x:d.capturePoint.x+(result.step.foot==='leftFoot'?-.09:.09),y:.045,z:d.capturePoint.z+.027};
      const travel=clampLength({...sub(desired,from),y:0},.43);
      c.balance.step.from={...from};c.balance.step.to={...add(from,travel),y:.045};
      result.step=structuredClone(c.balance.step);
    }
    return result;
  };
}
if (process.env.PROBE_SUPPORT_MODEL === 'normal') {
  const support = c.motorSupportConstraints.bind(c);
  c.motorSupportConstraints = () => support().map(s=>({...s,directions:[...s.directions,{x:0,y:1,z:0}]}));
}
if (process.env.PROBE_SUPPORT_MODEL === 'none') c.motorSupportConstraints = () => [];
if (process.env.PROBE_SUPPORT_MODEL?.startsWith('measured')) {
  c.motorSupportConstraints = () => c.lastContacts.filter(c=>c.loadBearing && /Foot|Forefoot/.test(c.segment))
    .map(s=>({segment:s.segment,points:[s.point], ...(process.env.PROBE_SUPPORT_MODEL==='measured-tangent'
      ? {directions:[{x:1,y:0,z:0},{x:0,y:0,z:1}]}:{})}));
}
let targetPoses = null, decision = null;
const original = c.motorCommands.bind(c);
c.motorCommands = targets => {
  if (process.env.PROBE_ROOT_PRESSURE && c.contactLoadPlan.loads.length) {
    const pelvis=c.poses.get('pelvis'), gain=Number(process.env.PROBE_ROOT_PRESSURE);
    const moment=add(scale(angularVelocity(pelvis.rotation,targets.get('pelvis').rotation,1),gain),scale(pelvis.angularVelocity,-gain*.15));
    const shift={x:moment.z/(72.2*9.81),y:0,z:-moment.x/(72.2*9.81)};
    const allowed=new Set(c.contactLoadPlan.loads.map(l=>l.segment));
    c.contactLoadPlan=planContactLoads(c.lastContacts.filter(c=>allowed.has(c.segment)),
      add(c.balanceData.centerOfMass,shift),c.balanceData.centerOfMassVelocity,c.contactLoadPlan.requestedForce,
      {frictionCoefficient:1.2,maxHorizontalForceN:72.2*3.6,maxJointTorqueNm:10000});
  }
  if (process.env.PROBE_VERTICAL_LOAD) {
    const pelvis=c.poses.get('pelvis');
    const factor=Math.max(0,Math.min(1.35,1+(40*(targets.get('pelvis').position.y-pelvis.position.y)-12*pelvis.linearVelocity.y)/9.81));
    for(const l of c.contactLoadPlan.loads)l.plannedForce={...l.plannedForce,y:l.plannedForce.y*factor};
  }
  if (process.env.PROBE_FEASIBLE_WRENCH) {
    const p=c.contactLoadPlan, com=c.balanceData.centerOfMass;
    const fy=p.loads.reduce((sum,l)=>sum+l.plannedForce.y,0);
    const height=Math.max(.3,com.y-p.pressurePoint.y);
    for (const l of p.loads) l.plannedForce={...l.plannedForce,
      x:((com.x-p.pressurePoint.x)*fy/height)*l.plannedForce.y/Math.max(fy,1),
      z:((com.z-p.pressurePoint.z)*fy/height)*l.plannedForce.y/Math.max(fy,1)};
  }
  if (process.env.PROBE_HORIZONTAL_LOAD) {
    const factor = Number(process.env.PROBE_HORIZONTAL_LOAD);
    for (const load of c.contactLoadPlan.loads) load.plannedForce={...load.plannedForce,x:load.plannedForce.x*factor,z:load.plannedForce.z*factor};
  }
  targetPoses = [...targets.values()];
  decision = structuredClone({ step: c.step, balance: c.balanceData, contacts: c.lastContacts,
    loadPlan: c.contactLoadPlan, transferAge: c.balance.transferReadyAge,
    touchdownAge: c.balance.touchdownAge, cooldown: c.balance.cooldown,
    actual: [...c.poses.values()] });
  const commands = original(targets);
  if (process.env.PROBE_LEG_GAIN) for (const command of commands) {
    if (/Thigh|Shin|Ankle|Foot|Forefoot/.test(command.id)) {
      const factor=Number(process.env.PROBE_LEG_GAIN);
      command.stiffness*=factor;command.damping*=Math.sqrt(factor);
    }
  }
  if (process.env.PROBE_ROOT_ATTITUDE) {
    const pelvis=c.poses.get('pelvis');
    const gain=Number(process.env.PROBE_ROOT_ATTITUDE);
    const moment=add(scale(angularVelocity(pelvis.rotation,targets.get('pelvis').rotation,1),gain),scale(pelvis.angularVelocity,-gain*.2));
    const total=c.contactLoadPlan.loads.reduce((sum,l)=>sum+l.measuredForceN,0);
    for(const command of commands.filter(cmd=>/Thigh/.test(cmd.id))){
      const side=command.id.startsWith('left')?'left':'right';
      const share=c.contactLoadPlan.loads.filter(l=>l.segment.startsWith(side)).reduce((sum,l)=>sum+l.measuredForceN,0)/Math.max(total,1);
      command.feedforwardWorld=sub(command.feedforwardWorld,scale(moment,share));
    }
  }
  if (process.env.PROBE_KNEE_GAIN) for (const command of commands) {
    if (/Shin/.test(command.id)) {
      const factor=Number(process.env.PROBE_KNEE_GAIN);
      command.stiffness*=factor;command.damping*=Math.sqrt(factor);
    }
  }
  return commands;
};
const initial = c.getSnapshot('canvas2d').segments.find(p => p.id === 'rightHand');
const anchor = { x: .025, y: .015, z: .01 };
const start = worldPoint(initial.position, initial.rotation, anchor);
c.fixedUpdate(dt, { kind:'begin', pointerId:41, region:'rightHand', segment:'rightHand',
  localAnchor:anchor, worldTarget:start, timestampMs:0 });
const rows = [];
for (let frame = 1; frame <= count; frame++) {
  decision = null;
  targetPoses = null;
  const beforePoses = [...c.poses.values()].map(pose => structuredClone(pose));
  const command = frame === 150 ? { kind:'end', pointerId:41, timestampMs:frame*dt*1000 }
    : frame < 150 ? { kind:'move', pointerId:41,
      worldTarget:add(start, {x:0, y:.03*Math.min(frame,90)/90, z:.70*Math.min(frame,90)/90}),
      timestampMs:frame*dt*1000 } : null;
  c.fixedUpdate(dt, command);
  const s = c.getSnapshot('canvas2d');
  if (process.env.PROBE_PROGRESS && frame % 30 === 0) console.log(JSON.stringify({
    frame, state:s.state, phase:s.diagnostics.recovery.phase,
    stage:s.diagnostics.recovery.transferStage, blocking:s.diagnostics.recovery.blockingPredicate,
  }));
  const legs = s.diagnostics.jointDiagnostics.filter(j => /Thigh|Shin|Ankle|Foot|Forefoot/.test(j.segment))
    .map(j => {
      const d = SEGMENT_BY_ID.get(j.segment), parent = c.ragdollBodies.get(d.parent), body = c.ragdollBodies.get(j.segment);
      const motor=c.lastMotorResults.get(j.segment), beforeParent=beforePoses.find(p=>p.id===d.parent), beforeChild=beforePoses.find(p=>p.id===j.segment);
      return { ...j, velocity:jointAngularVelocityCoordinates(parent.angvel(),body.angvel(),parent.rotation(),j.coordinates,d.jointProfile),
        beforeCoordinates:motor?.coordinates,
        beforeVelocity:motor&&beforeParent&&beforeChild?jointAngularVelocityCoordinates(beforeParent.angularVelocity,beforeChild.angularVelocity,beforeParent.rotation,motor.coordinates,d.jointProfile):null,
        torqueWorld:motor?.torqueWorld,
        caps:d.jointProfile.axes.map(a=>({coordinate:a.coordinate,capNm:a.maxMotorTorqueNm})) };
    });
  rows.push({ frame, time:s.simulationTime, state:s.state, decision,
    support:s.support, contacts:c.lastContacts, targets:targetPoses,
    actual:s.segments, joints:legs, diagnostics:s.diagnostics });
}
mkdirSync('evidence', {recursive:true});
writeFileSync(`${prefix}.ndjson`, rows.map(r=>JSON.stringify(r)).join('\n')+'\n');
const transitions = rows.filter((r,i)=>!i || r.state!==rows[i-1].state || r.decision?.step?.foot!==rows[i-1].decision?.step?.foot
  || (r.decision?.step?.elapsed>=0)!==(rows[i-1].decision?.step?.elapsed>=0));
const summary = {frames:count, final:rows.at(-1).state, firstFall:rows.find(r=>r.state==='falling')?.frame,
  worstCollision:rows.reduce((a,r)=>r.diagnostics.maxSelfPenetrationM>a.depth?{depth:r.diagnostics.maxSelfPenetrationM,pair:r.diagnostics.selfPenetrationPair,frame:r.frame}:a,{depth:0}),
  transitions:transitions.map(r=>({frame:r.frame,state:r.state,step:r.decision?.step,support:r.support,
    pelvis:r.actual.find(p=>p.id==='pelvis').position, balance:r.decision?.balance}))};
writeFileSync(`${prefix}.json`,JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify(summary));
c.dispose();
