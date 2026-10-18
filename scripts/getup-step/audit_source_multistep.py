#!/usr/bin/env python3
"""Audit saved multi-step data only. No engine import or integrations."""
import argparse,hashlib,json,math
from collections import OrderedDict,deque
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def audit(folder):
 summary=json.loads((folder/'multistep-summary.json').read_text());initial=json.loads((folder/'summary.json').read_text())['entry'];model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());gears=np.zeros(21);names=['']*21
 for part in model['segments']:
  h=part['reference'].get('hinge')
  if h:gears[h['qposIndex']]=h['gear'];names[h['qposIndex']]=h['name']
 phases=OrderedDict();last_slip=np.zeros(2);quiet=deque(maxlen=4000);counts=0;minimum=1.;worst=None;maxmotor=0.;maxself=0.;warnings=0;nonfoot=0.;flight=0;rayfail=0;phase_clear={};references={}
 for step in summary['steps']:
  references[step['index']]=json.loads((folder/f"controller-step-{step['index']:02d}.json").read_text())
 with(folder/'frames.ndjson').open()as f:
  for line in f:
   row=json.loads(line);counts+=1;assert row['sourceStep']==counts and abs(row['elapsedS']-counts*.00125)<1e-12;assert len(row['jointGroundRays'])==22
   rayfail+=sum(not r['hit']or not r['ownBodyExcluded']for r in row['jointGroundRays']);warnings=max(warnings,row['warnings']);nonfoot=max(nonfoot,row['nonfootLoadN']);flight+=int(all(x['loadN']<3 for x in row['feet']));maxself=max(maxself,row['maximumSelfPenetrationM']);maxmotor=max(maxmotor,float(max(abs(np.array(row['actualMotorTorquesNm']))/gears)));margin=row['jointLimitMinimumMarginRad']
   if margin<minimum:minimum=margin;worst=dict(timeS=row['elapsedS'],step=row['stepIndex'],phase=row['phase'],joint=names[int(np.argmin(row['jointLimitMarginsRad']))],marginRad=margin)
   key=(row['stepIndex'],row['phase']);phase=phases.setdefault(key,dict(step=key[0],phase=key[1],samples=0,loadedSlipIncrementM=np.zeros(2),minimumJointMarginRad=1.));current=np.array(row['loadedSlipDistanceM']);phase['loadedSlipIncrementM']+=current-last_slip;last_slip=current;phase['samples']+=1;phase['minimumJointMarginRad']=min(phase['minimumJointMarginRad'],margin)
   side=row['swingSide'];swing=row['feet'][side];stance=row['feet'][1-side];c=phase_clear.setdefault(row['stepIndex'],dict(maximumWholeFootClearanceM=-1.,maximumLoadedClear25mmConsecutiveSamples=0,currentClear=0));c['maximumWholeFootClearanceM']=max(c['maximumWholeFootClearanceM'],swing['clearance']);c['currentClear']=c['currentClear']+1 if swing['clearance']>=.025 and swing['loadN']<3 and stance['loadN']>=.8*model['totalMassKg']*9.81 else 0;c['maximumLoadedClear25mmConsecutiveSamples']=max(c['maximumLoadedClear25mmConsecutiveSamples'],c['currentClear'])
   ref=references[row['stepIndex']];actual_forward=float((np.array(swing['position'])-np.array(ref['targets'][side][0]))@np.array(ref['heading']));assert abs(actual_forward-row['swingForwardDisplacementM'])<1e-12
   quiet.append(row)
 assert counts==summary['actualIntegrations']and abs(counts*.00125-summary['physicsSimSeconds'])<1e-12;assert np.max(abs(last_slip-summary['globalLoadedSlipM']))<1e-12
 def isquiet(row):
  feet=row['feet'];return row['phase']=='stand'and all(f['loadN']>=3 for f in feet)and sum(f['loadN']for f in feet)>=.8*model['totalMassKg']*9.81 and row['torsoTiltDeg']<=5 and row['maximumBodyLinearMps']<=.22 and row['maximumBodyAngularRadps']<=.65 and row['minimumComCaptureMarginM']is not None and row['minimumComCaptureMarginM']>=.003 and .780625<=row['com'][2]<=.829976 and row['standingAnchorDriftM']<=.01 and row['maximumSelfPenetrationM']<=.005
 finalquiet=len(quiet)==4000 and all(isquiet(r)for r in quiet);quietmin=min(r['jointLimitMinimumMarginRad']for r in quiet);motion=summary['passed']and finalquiet and nonfoot<3 and not flight and not warnings and not rayfail and maxself<=.005 and maxmotor<=.95000001 and max(last_slip)<=.01 and all(s.get('achievedForwardM',-1)>=s['requestedDistanceM']for s in summary['steps'])
 result=dict(savedDataOnly=True,physicsIntegrations=0,motionAndQuietGatesPass=motion,strictAllSampleJointLimitsPass=minimum>=0,strictAllGatesPass=motion and minimum>=0,actualRecordedIntegrations=counts,physicsSimSeconds=counts*.00125,finalQuietSamples=4000 if finalquiet else 0,finalQuietMinimumJointReserveRad=quietmin,maximumMotorFraction=maxmotor,maximumSelfPenetrationM=maxself,maximumNonfootLoadN=nonfoot,flightSamples=flight,warnings=warnings,missingOrSelfEligibleJointRayCount=rayfail,globalLoadedSlipM=last_slip.tolist(),inheritedInitialJointMarginRad=initial['jointLimitMinimumMarginRad'],worstJointSample=worst,steps=[dict(index=s['index'],side=s['swingSide'],requestedDistanceM=s['requestedDistanceM'],achievedForwardM=s['achievedForwardM'],**{k:v for k,v in phase_clear[s['index']].items()if k!='currentClear'})for s in summary['steps']],phases=[{**p,'loadedSlipIncrementM':p['loadedSlipIncrementM'].tolist()}for p in phases.values()],hashes={n:digest(folder/n)for n in ['frames.ndjson','control.json','multistep-summary.json','summary.json','final.bin']})
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();r=audit(a.folder);text=json.dumps(r,indent=2)+'\n';(a.output or a.folder/'saved-data-audit.json').write_text(text);print(text);raise SystemExit(0 if r['motionAndQuietGatesPass']else 1)
