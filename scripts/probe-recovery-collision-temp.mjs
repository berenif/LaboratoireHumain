import { createEmbodiedCharacter } from '../src/character/index.ts';
import { jointRotationFromCoordinates } from '../src/character/joint-coordinates.ts';
import { limbBodyClearance } from '../src/character/limb-collisions.ts';
import { reconstructRecoveryLimb, recoveryLimbIds } from '../src/character/recovery-joints.ts';
import { add, cross, lerp, length, sub, worldPoint } from '../src/character/math.ts';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';

const dt=1/60, id='leftHand';
const c=await createEmbodiedCharacter('canvas2d');
let s=c.getSnapshot('canvas2d');
const startPose=s.segments.find(p=>p.id==='rightHand');
const anchor={x:.025,y:.015,z:.01};
const start=worldPoint(startPose.position,startPose.rotation,anchor);
c.fixedUpdate(dt,{kind:'begin',pointerId:41,region:'rightHand',segment:'rightHand',localAnchor:anchor,worldTarget:start,timestampMs:0});
let worst=null;
const frames=[];
for(let frame=1;frame<=420;frame++) {
  const offset={x:0,y:.03,z:.70};
  const command=frame===150?{kind:'end',pointerId:41,timestampMs:frame*dt*1000}
    :frame<150?{kind:'move',pointerId:41,worldTarget:add(start,{x:0,y:offset.y*Math.min(frame,90)/90,z:offset.z*Math.min(frame,90)/90}),timestampMs:frame*dt*1000}:null;
  c.fixedUpdate(dt,command);
  s=c.getSnapshot('canvas2d');
  const torso=c.ragdollColliders.get('torso'), hand=c.ragdollColliders.get(id);
  const contact=torso.contactCollider(hand,.06);
  if(!contact || contact.distance>=0) continue;
  const torsoBody=c.ragdollBodies.get('torso'), handBody=c.ragdollBodies.get(id);
  const velocityAt=(body,point)=>add(body.linvel(),cross(body.angvel(),sub(point,body.translation())));
  const relativeSpeed=length(sub(velocityAt(handBody,contact.point2),velocityAt(torsoBody,contact.point1)));
  let impulse=0,normal=null;
  c.world.contactPair(torso,hand,(manifold,flipped)=>{
    normal=manifold.normal();
    for(let i=0;i<manifold.numContacts();i++)impulse+=Math.max(0,manifold.contactImpulse(i));
  });
  const poses=new Map(s.segments.map(p=>[p.id,p]));
  const rotations=new Map();
  for(const segment of recoveryLimbIds('left',true)){
    const j=s.diagnostics.jointDiagnostics.find(j=>j.segment===segment);
    rotations.set(segment,jointRotationFromCoordinates(j.targetCoordinates,SEGMENT_BY_ID.get(segment).jointProfile));
  }
  const predicted=reconstructRecoveryLimb('left',true,poses.get('torso'),rotations);
  const planned=new Map(poses);
  for(const pose of predicted.poses)planned.set(pose.id,pose);
  const event={frame,time:s.simulationTime,state:s.state,phase:s.diagnostics.recovery.phase,
    penetration:-contact.distance,relativeSpeed,impulse,normal,
    measuredClearance:limbBodyClearance(poses,'left','arm'),
    plannedClearance:limbBodyClearance(planned,'left','arm'),
    blocking:s.diagnostics.recovery.blockingPredicate,
    handSkin:hand.contactSkin(),handLinvel:handBody.linvel(),torsoLinvel:torsoBody.linvel(),
    jointTargets:s.diagnostics.jointDiagnostics.filter(j=>j.segment.startsWith('left')&&['leftShoulderGirdle','leftUpperArm','leftForearm','leftForearmTwist','leftHand'].includes(j.segment)).map(j=>({id:j.segment,target:j.targetCoordinates,actual:j.coordinates,torque:j.motorTorqueNm}))};
  frames.push(event);
  if(!worst || event.penetration>worst.penetration)worst=event;
}
console.log(JSON.stringify({worst,near:frames.filter(f=>Math.abs(f.frame-(worst?.frame??-100))<=3),finalState:s.state}));
c.dispose();
