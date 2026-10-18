#!/usr/bin/env python3
"""Replay measured source step criteria from saved data, with zero integrations."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
def audit(folder):
 summary=json.loads((folder/'summary.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());mass=model['totalMassKg'];entry=summary['entry'];initial_q=np.array(entry['qpos']);heading=Rotation.from_quat(initial_q[[4,5,6,3]]).as_matrix()[:,0];heading[2]=0;heading/=np.linalg.norm(heading);origin=np.array(entry['feet'][0]['position']);gears=np.zeros(21)
 for part in model['segments']:
  h=part['reference'].get('hinge')
  if h:gears[h['qposIndex']]=h['gear']
 slip=np.zeros(2);liftrun=maxlift=0;landrun=0;landed=None;maxclear=-1;maxmotor=0.;rows=[];minimum_joint=float('inf');minimum_finaljoint=float('inf');maxpen=0.;rays=set();warnings=0;nonfoot=0.;flight=0
 with(folder/'frames.ndjson').open()as stream:
  for index,line in enumerate(stream):
   row=json.loads(line);assert row['sourceStep']==entry['sourceStep']+index+1;assert abs(row['elapsedS']-(index+1)*.00125)<1e-11
   feet=row['feet'];slip+=np.array([f['loadedSlipSpeedMps']for f in feet])*.00125;assert np.max(abs(slip-np.array(row['loadedSlipDistanceM'])))<1e-11
   forward=float((np.array(feet[0]['position'])-origin)@heading);assert abs(forward-row['forwardFootDisplacementM'])<1e-11
   for foot in feet:
    assert len(foot['groundRays'])==4
    for ray in foot['groundRays']:assert ray['hit']and ray['ownBodyExcluded']and ray['geomId']==0
    expected=min(ray['distanceM']-.2-.027 for ray in foot['groundRays']);assert abs(expected-foot['clearance'])<1e-12
   js=row['jointGroundRays'];assert len(js)==22 and len({r['joint']for r in js})==22
   for ray in js:assert ray['hit']and ray['ownBodyExcluded']and ray['geomId']==0;rays.add(ray['joint'])
   qualified_lift=feet[0]['clearance']>=.025 and feet[0]['loadN']<3 and feet[1]['loadN']>=.8*mass*9.81
   liftrun=liftrun+1 if qualified_lift else 0;maxlift=max(maxlift,liftrun);maxclear=max(maxclear,feet[0]['clearance'])
   if maxlift>=80:
    landrun=landrun+1 if feet[0]['loadN']>=3 and feet[1]['loadN']>=3 and feet[0]['clearance']<=.003 and forward>=.06 else 0
    if landed is None and landrun>=80:landed=row['elapsedS']
   maxmotor=max(maxmotor,float(np.max(abs(np.array(row['actualMotorTorquesNm']))/gears)));minimum_joint=min(minimum_joint,min(row['jointLimitMarginsRad']));maxpen=max(maxpen,row['maximumSelfPenetrationM']);warnings+=row['warnings'];nonfoot=max(nonfoot,row['nonfootLoadN']);flight+=all(f['loadN']<3 for f in feet);rows.append(row)
 assert len(rows)==summary['actualIntegrations'];tail=rows[-4000:];assert len(tail)==4000
 anchors=np.array([f['position']for f in tail[0]['feet']]);mindistance=float('inf');maxdrift=0.;maxlinear=maxangular=maxtilt=0.;quiet=0
 for row in tail:
  feet=row['feet'];com=np.array(row['com']);capture=com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81);eq=ConvexHull(np.unique(np.array(row['supportPointsXY']),axis=0)).equations;margin=float(min(np.min(-(eq[:,:2]@com[:2]+eq[:,2])),np.min(-(eq[:,:2]@capture+eq[:,2]))));drift=float(np.max(np.linalg.norm(np.array([f['position']for f in feet])-anchors,axis=1)));mindistance=min(mindistance,margin);maxdrift=max(maxdrift,drift);minimum_finaljoint=min(minimum_finaljoint,min(row['jointLimitMarginsRad']));q=np.array(row['qpos']);R=Rotation.from_quat(q[[4,5,6,3]]).as_matrix();head_height=q[2]+.19*R[2,2]
  conditions=[all(f['loadN']>=3 for f in feet),sum(f['loadN']for f in feet)>=.8*mass*9.81,row['nonfootLoadN']<3,row['torsoTiltDeg']<=5,row['maximumBodyLinearMps']<=.22,row['maximumBodyAngularRadps']<=.65,margin>=.003,.780625<=com[2]<=.829976,drift<=.01,row['maximumSelfPenetrationM']<=.005,head_height>1.4,row['warnings']==0]
  quiet+=all(conditions);maxlinear=max(maxlinear,row['maximumBodyLinearMps']);maxangular=max(maxangular,row['maximumBodyAngularRadps']);maxtilt=max(maxtilt,row['torsoTiltDeg'])
 finalforward=float((np.array(rows[-1]['feet'][0]['position'])-origin)@heading)
 result=dict(measuredForwardStepAndFinal5sQuietPassed=bool(maxlift>=80 and landed is not None and quiet==4000 and finalforward>=.06 and max(slip)<=.01 and maxmotor<=.95+1e-9 and not flight and nonfoot<3 and not warnings and maxpen<=.005),actualIntegrations=len(rows),physicsSimSeconds=len(rows)*.00125,additionalIntegrationsForAudit=0,finalForwardPlacementM=finalforward,maximumWholeFootClearanceM=maxclear,maximumConsecutive25mmLiftWithStanceLoadS=maxlift*.00125,measuredLanding100msAcquisitionAtS=landed,finalQuietSamplesPassed=quiet,minimumFinalComCaptureMarginM=mindistance,maximumFinalAnchorDriftM=maxdrift,maximumFinalBodyLinearMps=maxlinear,maximumFinalBodyAngularRadps=maxangular,maximumFinalTorsoTiltDeg=maxtilt,loadedSlipDistanceM=slip.tolist(),maximumActualMotorFraction=maxmotor,minimumJointMarginAllTrajectoryRad=minimum_joint,minimumInitialJointMarginRad=entry['jointLimitMinimumMarginRad'],minimumFinal4000JointMarginRad=minimum_finaljoint,maximumSelfPenetrationM=maxpen,flightSamples=flight,nonfootLoadMaximumN=nonfoot,warnings=warnings,jointRaysPerSample=22,jointNames=sorted(rays),journalSHA256=hashlib.sha256((folder/'frames.ndjson').read_bytes()).hexdigest(),summarySHA256=hashlib.sha256((folder/'summary.json').read_bytes()).hexdigest())
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();result=audit(a.folder);(a.folder/'replay-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
