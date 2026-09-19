/* eslint-disable @typescript-eslint/no-explicit-any -- One-off measured recovery diagnosis. */
import {createEmbodiedCharacter} from '../src/character/index';
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from './recovery-fixtures';
import {usableRecoveryArmSupport} from '../src/character/recovery-support';
import {SEGMENT_BY_ID} from '../src/core/humanoid';
import {lowestWorldPoint} from '../src/core/geometry';
import {solveRecoveryForefootAnchor} from '../src/character/recovery-forefoot-anchor';
import {add,quatInverse,rotate,sub} from '../src/character/math';
import {reconstructRecoveryLimb} from '../src/character/recovery-joints';

const c=await createEmbodiedCharacter('canvas2d');
const fixture=RECOVERY_POSE_FIXTURES.find(f=>f.id==='landed-prone-left')!;
seedRecoveryFixture(c,fixture);
let previousPhase='';
for(let i=0;i<180;i++){
  c.fixedUpdate(1/60,null);
  const summary=c.getSnapshot('canvas2d');
  const phase=`${summary.diagnostics.recovery.phase}/${summary.diagnostics.recovery.transferStage}`;
  const changed=phase!==previousPhase;
  if((i<30 ? i%15!==0 : i%15!==0) && !changed)continue;
  previousPhase=phase;
  const r=(c as any).recovery;
  const poses=r.poses((c as any).ragdollBodies);
  const s=summary;
  const targets=Object.fromEntries([...r.armBraces].map(([side,t]:[string,any])=>[side,{
    reach:t.reachErrorM,floor:t.floorClearanceM,limit:t.jointLimitErrorRad,
    movement:t.movementM,legal:r.legalArmCommand(side,t.position,poses)!==null,
  }]));
  const p=r.prospectiveSupport('brace','tuck-knee',poses);
  const rightShin=s.segments.find((x:any)=>x.id==='rightShin')!;
  const toePlant=r.plants.get('rightForefoot');
  const anchor=toePlant?.contact?solveRecoveryForefootAnchor('right',poses,{
    position:toePlant.position,rotation:toePlant.rotation,contact:toePlant.contact}):null;
  console.log(JSON.stringify({t:(i+1)/60,phase:s.diagnostics.recovery.phase,
    stage:s.diagnostics.recovery.transferStage,clearance:r.proneShoulderClearance(poses),
    lead:s.diagnostics.recovery.leadingSide,rollSide:s.diagnostics.recovery.rollSide,
    pelvis:s.rootPosition.y,margin:s.diagnostics.recovery.supportMarginM,
    rightShinLow:lowestWorldPoint(SEGMENT_BY_ID.get('rightShin')!.geometry,
      rightShin.position,rightShin.rotation).y,
    toePlant:!!toePlant,
    toeContact:r.data.contacts.find((x:any)=>x.segment==='rightForefoot')&&{
      force:r.data.contacts.find((x:any)=>x.segment==='rightForefoot').forceN,
      persistence:r.data.contacts.find((x:any)=>x.segment==='rightForefoot').persistenceS,
      loaded:r.data.contacts.find((x:any)=>x.segment==='rightForefoot').loadBearing},
    anchor:anchor&&{error:anchor.patchErrorM,floor:anchor.floorClearanceM},
    toeDrift:r.data.plantedTargets.find((x:any)=>x.segment==='rightForefoot')?.driftM,
    leftShinPlant:r.plants.has('leftShin'),
    leftShinDrift:r.data.plantedTargets.find((x:any)=>x.segment==='leftShin')?.driftM,
    armPlants:['leftForearm','leftHand','rightForearm','rightHand'].map(id=>({
      id,plant:r.plants.has(id),drift:r.data.plantedTargets.find((x:any)=>x.segment===id)?.driftM,
      contact:r.data.contacts.find((x:any)=>x.segment===id)?.forceN,
      patchY:(()=>{const p=r.plants.get(id),pose=poses.get(id);return p?.contact&&pose
        ?add(pose.position,rotate(pose.rotation,rotate(quatInverse(p.rotation),sub(p.contact,p.position)))).y:null})(),
      low:lowestWorldPoint(SEGMENT_BY_ID.get(id as any)!.geometry,
        poses.get(id)!.position,poses.get(id)!.rotation).y})),
    blocking:r.data.blockingPredicate,
    armMotors:[...r.motorResults()].filter(([id]:[string])=>
      /rightUpperArm|rightForearm|rightHand|leftUpperArm|leftForearm|leftHand/.test(id)).map(([id,m]:[string,any])=>
      [id,Number(Math.hypot(m.torqueWorld.x,m.torqueWorld.y,m.torqueWorld.z).toFixed(1)),
        Number(m.saturationRatio.toFixed(2)),Number(m.coordinateError.x.toFixed(3))]),
    armTargets:(['left','right'] as const).map(side=>{
      const targets=new Map();r.limbTargets(side,true,poses,targets);
      const geometry=reconstructRecoveryLimb(side,true,poses.get('torso'),targets);
      return {side,floor:geometry.floorClearanceM,patches:geometry.poses.filter((p:any)=>{
        return !!r.plants.get(p.id)?.contact;
      }).map((p:any)=>{const plant=r.plants.get(p.id);
        return [p.id,add(p.position,rotate(p.rotation,rotate(quatInverse(plant.rotation),sub(plant.contact,plant.position)))).y];})};
    }),
    leftShinContact:r.data.contacts.find((x:any)=>x.segment==='leftShin')&&{
      force:r.data.contacts.find((x:any)=>x.segment==='leftShin').forceN,
      persistence:r.data.contacts.find((x:any)=>x.segment==='leftShin').persistenceS,
      loaded:r.data.contacts.find((x:any)=>x.segment==='leftShin').loadBearing},
    motors:[...r.motorResults()].filter(([id]:[string])=>
      /rightThigh|rightShin|rightAnkle/.test(id)).map(([id,m]:[string,any])=>
      [id,Number(Math.hypot(m.torqueWorld.x,m.torqueWorld.y,m.torqueWorld.z).toFixed(1)),
        Number(m.saturationRatio.toFixed(2)),Number(m.coordinateError.x.toFixed(3))]),
    prospective:{ready:p.ready,balanced:p.balanced,margin:p.geometry.marginM,
      supporting:p.geometry.supporting,excluded:p.excludedLoaded.map((x:any)=>x.segment)},
    supportLoads:s.diagnostics.recovery.supportLoads?.filter((x:any)=>
      /Shin|Forefoot|Foot|Hand|Forearm/.test(x.segment)).map((x:any)=>
      [x.segment,Number(x.plannedForceN.toFixed(1)),Number(x.measuredForceN.toFixed(1))]),
    latched:r.shoulderClearanceEstablished,braces:[...r.armBraces.keys()],targets,
    ready:['left','right'].map(side=>[side,r.armPlacementReady(side,poses),
      usableRecoveryArmSupport(side as 'left'|'right',poses,r.data.contacts),
      r.data.contacts.filter((x:any)=>x.loadBearing&&x.segment.startsWith(side)&&/Hand|Forearm/.test(x.segment)).map((x:any)=>[x.segment,x.persistenceS,r.plants.has(x.segment)])]),
    arms:s.diagnostics.recovery.contacts.filter((x:any)=>x.loadBearing&&/Hand|Forearm/.test(x.segment)).map((x:any)=>[x.segment,x.forceN]),
    legs:s.diagnostics.recovery.contacts.filter((x:any)=>x.loadBearing&&/Shin|Ankle|Foot|Forefoot/.test(x.segment)).map((x:any)=>[x.segment,x.forceN]),
    belly:s.diagnostics.recovery.contacts.filter((x:any)=>x.loadBearing&&/torso|lumbar|pelvis/.test(x.segment)).map((x:any)=>[x.segment,x.forceN])}));
}
c.dispose();
