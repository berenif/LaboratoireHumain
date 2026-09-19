/* eslint-disable @typescript-eslint/no-explicit-any -- Temporary recovery geometry diagnosis. */
import {createEmbodiedCharacter} from '../src/character/index';
import {RECOVERY_POSE_FIXTURES,seedRecoveryFixture} from './recovery-fixtures';
import {solveRecoveryForefootAnchor} from '../src/character/recovery-forefoot-anchor';
import {SEGMENT_BY_ID} from '../src/core/humanoid';
import {lowestWorldPoint} from '../src/core/geometry';
import {jointCoordinates,jointRotationFromCoordinates,clampJointCoordinates} from '../src/character/joint-coordinates';
import {reconstructRecoveryLimb} from '../src/character/recovery-joints';
import {limbTrunkClearance} from '../src/character/limb-collisions';
import {add,length,quatInverse,rotate,sub} from '../src/character/math';
import type {SegmentId,SegmentPose} from '../src/core/types';
import {supportGeometry,recoveryMassState} from '../src/character/recovery-support';

const c=await createEmbodiedCharacter('canvas2d');
seedRecoveryFixture(c,RECOVERY_POSE_FIXTURES.find(f=>f.id==='landed-prone-left')!);
for(let i=0;i<32;i++)c.fixedUpdate(1/60,null);
const r=(c as any).recovery;
const poses=r.poses((c as any).ragdollBodies) as Map<SegmentId,SegmentPose>;
for(const side of ['left','right'] as const){
  const forefoot=`${side}Forefoot` as SegmentId;
  const without=supportGeometry(r.supporting(poses),poses,recoveryMassState(poses.values()),
    [`${side}Foot`,`${side}Forefoot`,`${side}Ankle`,`${side}Shin`] as SegmentId[]);
  const plant=r.plants.get(forefoot);
  if(!plant?.contact)continue;
  const anchor=solveRecoveryForefootAnchor(side,poses,plant);
  if(!anchor)continue;
  const ids=['Thigh','Shin','Ankle','Foot','Forefoot'].map(name=>`${side}${name}` as SegmentId);
  const profiles=ids.map(id=>SEGMENT_BY_ID.get(id)!.jointProfile!);
  const names=['thigh','shin','ankle','foot','forefoot'] as const;
  const base=names.map((name,index)=>jointCoordinates({x:0,y:0,z:0,w:1},anchor.jointRotations[name],profiles[index]));
  const localPatch=rotate(quatInverse(plant.rotation),sub(plant.contact,plant.position));
  const initialShin=poses.get(`${side}Shin`)!;
  const initialLow=lowestWorldPoint(SEGMENT_BY_ID.get(initialShin.id)!.geometry,initialShin.position,initialShin.rotation).y;
  let best:any=null,eligible=0;
  const freeHeights:any[]=[];
  for(let thighStep=-4;thighStep<=4;thighStep++)for(let shinStep=-4;shinStep<=4;shinStep++)for(let ankleStep=-6;ankleStep<=6;ankleStep++){
    const values=base.map(x=>({...x}));
    values[0].x+=thighStep*.03;values[1].x+=shinStep*.03;values[2].x+=ankleStep*.05;
    const rotations=new Map(ids.map((id,index)=>[id,jointRotationFromCoordinates(clampJointCoordinates(values[index],profiles[index]),profiles[index])]));
    const rebuilt=reconstructRecoveryLimb(side,false,poses.get('pelvis')!,rotations);
    if(rebuilt.floorClearanceM<-.003)continue;
    const complete=new Map(poses);for(const pose of rebuilt.poses)complete.set(pose.id,pose);
    if(limbTrunkClearance(complete,side,'leg').clearanceM<.004)continue;
    const toe=complete.get(forefoot)!;
    const patch=add(toe.position,rotate(toe.rotation,localPatch));
    const patchError=length(sub(patch,plant.contact));
    if(patchError>.015)continue;
    eligible++;
    const shin=complete.get(`${side}Shin`)!;
    const low=lowestWorldPoint(SEGMENT_BY_ID.get(shin.id)!.geometry,shin.position,shin.rotation).y;
    const score=Math.abs(low-.002)+patchError*.2;
    if(!best||score<best.score)best={score,low,patchError,
      steps:[thighStep,shinStep,ankleStep],shinPosition:shin.position,
      angles:values.map(v=>v.x)};
  }
  for(const elevation of [0,.05,.10,.15,.20]){
  let freeBest:any=null,freeEligible=0;
  const trialPoses=new Map(poses);
  for(const id of ['pelvis','lumbar','ribcage','torso','neck','head'] as SegmentId[]){
    const p=poses.get(id);if(p)trialPoses.set(id,{...p,position:{...p.position,y:p.position.y+elevation}});
  }
  for(let hipStep=-4;hipStep<=12;hipStep++)for(let kneeStep=2;kneeStep<=18;kneeStep++)for(let ankleStep=-6;ankleStep<=8;ankleStep+=2){
    const values=base.map(x=>({...x}));
    values[0].x=hipStep*.1;values[1].x=kneeStep*.1;values[2].x=ankleStep*.1;
    const rotations=new Map(ids.map((id,index)=>[id,jointRotationFromCoordinates(clampJointCoordinates(values[index],profiles[index]),profiles[index])]));
    const rebuilt=reconstructRecoveryLimb(side,false,trialPoses.get('pelvis')!,rotations);
    if(rebuilt.floorClearanceM<-.003)continue;
    const complete=new Map(trialPoses);for(const pose of rebuilt.poses)complete.set(pose.id,pose);
    if(limbTrunkClearance(complete,side,'leg').clearanceM<.004)continue;
    const shin=complete.get(`${side}Shin`)!;
    const low=lowestWorldPoint(SEGMENT_BY_ID.get(shin.id)!.geometry,shin.position,shin.rotation).y;
    if(low>.012)continue;
    freeEligible++;
    const desiredZ=poses.get('pelvis')!.position.z-.2;
    const score=Math.abs(shin.position.z-desiredZ)+Math.abs(low-.002)*.4;
    if(!freeBest||score<freeBest.score)freeBest={score,low,shinPosition:shin.position,
      steps:[hipStep,kneeStep,ankleStep],angles:values.map(v=>v.x),floor:rebuilt.floorClearanceM};
  }
  freeHeights.push({elevation,freeEligible,freeBest});
  }
  console.log(JSON.stringify({side,pelvis:poses.get('pelvis')!.position,initialLow,
    releaseMargin:without.marginM,releaseSupporting:without.supporting,eligible,best,
    freeHeights}));
}
c.dispose();
