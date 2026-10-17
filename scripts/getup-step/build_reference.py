#!/usr/bin/env python3
"""Validate/sample the offline forward candidate; zero physics, no teacher claim."""
import argparse,json
from pathlib import Path
import numpy as np
from body_reference import BodyReference,ROOT
from native_env import atomic_json,sha256
from reference import UniformReference

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--acquisition',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    plan=json.loads(a.source.read_text());acq=json.loads(a.acquisition.read_text());modelpath=ROOT/'rust/crates/model/data/getup-v1.json';ref=BodyReference(plan,acq,json.loads(modelpath.read_text()))
    data={k:[]for k in ['qpos','qvel','com','com_ori']};checks=[];samples=[];max_r_error=0.;max_r_position=0.;max_r_rotation=0.;max_foot_error=0.;min_margin=1.
    for phase,item in ref.phases.items():
        for t in np.arange(0,item['duration']+.00001,.025):
            sample=ref.sample(phase,float(t));samples.append((phase,float(t),sample))
            q=sample['q'];min_margin=min(min_margin,float(np.min(np.r_[q[:20]-ref.limits[:20,0],ref.limits[:20,1]-q[:20]])))
            max_r_error=max(max_r_error,float(np.max(abs(sample['rightFoot']-ref.right))))
            max_r_position=max(max_r_position,float(np.linalg.norm(sample['rightFoot'][:3,3]-ref.right[:3,3])))
            from body_reference import proper_rotation,Rotation
            max_r_rotation=max(max_r_rotation,float(np.linalg.norm(Rotation.from_matrix(proper_rotation(sample['rightFoot'][:3,:3])@proper_rotation(ref.right[:3,:3]).T).as_rotvec())))
            u=__import__('body_reference').smooth_clock(t,item['duration'])[0]
            max_foot_error=max(max_foot_error,float(np.max(abs(sample['leftFoot'][:3,3]-item['position'](u)))))
            if t<item['duration']:
                for key in data:data[key].append(sample[key].tolist())
        phase_samples=[s for ph,_,s in samples if ph==phase]
        checks.append(dict(phase=phase,minimumWholeCapsuleClearanceM=min(s['clearance']for s in phase_samples),minimumCOMHeightM=min(s['com'][2]for s in phase_samples),maximumCOMHeightM=max(s['com'][2]for s in phase_samples),maximumJointSpeedRadps=max(np.max(abs(s['qvel'][6:]))for s in phase_samples)))
    final=ref.sample('settle',3.)
    # Explicit t=13..18s terminal rows, valid root quaternions and zero twists.
    for _ in range(201):
        for key in data:data[key].append((np.zeros(27)if key=='qvel'else final[key]).tolist())
    terminal=np.r_[final['q'],final['com'][2],final['com_ori']];UniformReference(data,terminal,kind='planned-unqualified')
    result=dict(kind='planned-unqualified',physicsIntegrations=0,sourceSHA256=sha256(a.source),acquisitionSHA256=sha256(a.acquisition),modelSHA256=sha256(modelpath),generatorSHA256=sha256(__file__),bodyReferenceSHA256=sha256(Path(__file__).with_name('body_reference.py')),timelineSeconds={**ref.DURATIONS,'hold':5.},trajectoryIncludesInitialAndFinalRows=True,uniformPolicyDtSeconds=.025,trajectory=data,terminal=terminal.tolist(),entryHybridReferenceException=ref.exception,initialMarginExceptions=[7,11,20],maximumDenseWeightedIKError=ref.maximum_dense_ik_error,maximumRightFootMatrixDeviation=max_r_error,maximumRightFootPositionDeviationM=max_r_position,maximumRightFootRotationDeviationRad=max_r_rotation,entryBridgeFootMotion="free reference bridge matches measured twist, not an exactly planted-foot constraint; no native writes",maximumLeftTargetPositionErrorM=max_foot_error,minimumNonElbowNativeMarginRad=min_margin,phaseChecks=checks,unqualified=['static witness is not a teacher or sourcePD authority','initial bridge has uncredited native stop boundary exception','contact gates and actual source PD response not demonstrated','runtime root and COM derivatives are source-frame coherent but native dynamics remain authoritative'])
    a.output.parent.mkdir(parents=True,exist_ok=True);atomic_json(a.output,result)
    print(json.dumps({k:v for k,v in result.items()if k not in ['trajectory','terminal']},indent=2))
if __name__=='__main__':main()
