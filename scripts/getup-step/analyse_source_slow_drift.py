#!/usr/bin/env python3
"""Pure FK/recorded-frame audit of source SLOW drift; no simulator import."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from source_math import P
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('pure_source_fk',ROOT/'scripts/getup-step-reference.py');fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)

def main():
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);a=p.parse_args();journal=a.base/'slow-continuation/frames.ndjson';rows=[json.loads(x)for x in journal.read_text().splitlines()];kin=fk.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));data=[]
    for row in rows[::20]+[rows[-1]]:
        q=np.array(row['qpos']);root=np.eye(4);root[:3,3]=P.T@q[:3];root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P;poses=kin.forward(q[7:],root);feet=np.array([P@poses[name][:3,3]for name in ['left_foot','right_foot']]);data.append(dict(timeS=row['timeS'],feetSourceM=feet,rootSourceM=q[:3],q=q[7:],headM=row['physicalObservation'][21],tiltDeg=math.degrees(math.acos(np.clip(row['physicalObservation'][36],-1,1)))))
    by_time={round(x['timeS'],6):x for x in data};window=[]
    for start in np.arange(3,8,.5):
        first=by_time[round(float(start),6)];last=by_time[round(float(start+.5),6)];delta=last['feetSourceM']-first['feetSourceM'];window.append(dict(startS=float(start),endS=float(start+.5),footDeltaSourceM=delta.tolist(),footSpeedMmps=(np.linalg.norm(delta,axis=1)*2000).tolist(),rootDeltaSourceM=(last['rootSourceM']-first['rootSourceM']).tolist(),footSeparationM=float(np.linalg.norm(last['feetSourceM'][0]-last['feetSourceM'][1]))))
    finalrows=rows[-4000:];failures={k:sum(not r['sourceQuiet']['conditions'][k]for r in finalrows)for k in finalrows[0]['sourceQuiet']['conditions']};first_loss=next((r for r in finalrows if not r['sourceQuiet']['conditions']['fixedAnchorDrift']),None)
    result=dict(physicsIntegrations=0,journalSHA256=hashlib.sha256(journal.read_bytes()).hexdigest(),sourceFrames=len(rows)-1,uniqueConsecutiveIDs=[r['sourceStep']for r in rows]==list(range(6401)),final5sFailureCounts=failures,firstAnchorDriftViolationTimeS=None if first_loss is None else first_loss['timeS'],finalFixedAnchorDriftM=finalrows[-1]['sourceQuiet']['anchorDriftM'],halfSecondWindows=window,interpretation='fixed-source hold drifts; no retrospective anchor reset or quiet qualification; half-second foot-origin transport is measured by exact pure FK')
    (a.base/'slow-drift-analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
