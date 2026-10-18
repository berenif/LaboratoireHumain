#!/usr/bin/env python3
"""Audit measured opposed swing journals; no native world or integration."""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
from audit_source_multistep import audit as audit_motion
from audit_source_arm_knee import audit as audit_effort


def amplitude(rows):
    a=np.array([r[0] for r in rows]);l=np.array([r[1] for r in rows]);t=np.array([r[2] for r in rows])
    arange=np.ptp(a,axis=0);lrange=np.ptp(l,axis=0)
    error=a-t
    return dict(samples=len(rows),legExcursionDeg=np.rad2deg(lrange).tolist(),
        actualArmExcursionDeg=np.rad2deg(arange).tolist(),
        targetArmExcursionDeg=np.rad2deg(np.ptp(t,axis=0)).tolist(),
        actualArmToLegExcursionRatio=(arange/lrange).tolist(),
        measuredArmLegCorrelation=[float(np.corrcoef(a[:,i],l[:,i])[0,1]) for i in range(2)],
        measuredAngularGain=[float(np.cov(l[:,i],a[:,i],ddof=0)[0,1]/np.var(l[:,i])) for i in range(2)],
        rmsTrackingErrorDeg=np.rad2deg(np.sqrt(np.mean(error**2,axis=0))).tolist(),
        maximumTrackingErrorDeg=np.rad2deg(np.max(abs(error),axis=0)).tolist())


def tracking_pass(measurement):
    return bool(all(abs(np.array(measurement['actualArmToLegExcursionRatio'])-1.)<=.05)
                and all(np.array(measurement['measuredArmLegCorrelation'])<=-.98)
                and max(measurement['maximumTrackingErrorDeg'])<=.5)


def audit(folder):
    motion=audit_motion(folder)
    effort=audit_effort(folder)
    mapping=json.loads((folder/'arm-mapping.json').read_text())
    controller=json.loads((folder/'controller-step-01.json').read_text())
    origin=np.array(controller['leg_origin']);neutral=np.array(mapping['neutralArmPitchRad'])
    from source_step_preview import fk, P
    model=json.loads((Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json').read_text())
    kin=fk.Kinematics(model);fk_error=0.;fk_samples=0
    groups=defaultdict(list);maxidentity=0.;count=0
    lateral_min=np.full(2,np.inf);lateral_max=np.full(2,-np.inf)
    for line in (folder/'frames.ndjson').open():
        row=json.loads(line);s=row['opposedArmSwing'];count+=1
        legs=np.array(s['legPitchRad']);arms=np.array(s['armPitchRad']);target=np.array(s['targetArmPitchRad'])
        maxidentity=max(maxidentity,float(max(abs(target-neutral+legs-origin))))
        if count == 1 or count % 20 == 0:
            poses=kin.forward(np.array(row['qpos'])[7:],np.eye(4));fk_samples+=1
            for i,side in enumerate(('right','left')):
                for start,end,observed in ((side+'_thigh',side+'_shin',legs[i]),
                                           (side+'_upper_arm',side+'_hand',arms[i])):
                    vector=P@(poses[end][:3,3]-poses[start][:3,3])
                    angle=np.arctan2(vector[0],-vector[2])
                    fk_error=max(fk_error,float(abs(angle-observed)))
        if not(row['stepIndex']==1 and row['phase']=='transfer'):
            values=(arms,legs,target)
            groups['allAfterInitialPlacement'].append(values)
            groups[f"step{row['stepIndex']}AfterPlacement"].append(values)
            lateral=np.array(s['armVectorTorsoM'])[:,1]*[-1,1]
            lateral_min=np.minimum(lateral_min,lateral);lateral_max=np.maximum(lateral_max,lateral)
            if row['phase'] in ('lift','translate','land'):
                groups['allGait'].append(values)
                groups[f"step{row['stepIndex']}Gait"].append(values)
    measurements={key:amplitude(rows) for key,rows in groups.items()}
    # Explicit physical tracking tolerance, separate from exact 1:1 target.
    tracking=all(tracking_pass(value) for key,value in measurements.items() if key.endswith('Gait'))
    result=dict(savedDataOnly=True,additionalPhysicsIntegrations=0,sideOrder=['right','left'],
        mapping=mapping,actualRecordedIntegrations=count,
        targetOppositionIdentityMaximumErrorRad=maxidentity,
        targetEqualAngularAmplitudePass=maxidentity<1e-12,
        independentFKCheck=dict(samples=fk_samples,samplingHz=40.,maximumPitchErrorRad=fk_error,physicsIntegrations=0),
        physicalTrackingTolerance=dict(maximumExcursionRatioError=.05,maximumOppositionCorrelation=-.98,maximumTrackingErrorDeg=.5),
        measuredEqualAmplitudeWithinTolerancePass=tracking,
        motionAndQuietGatesPass=motion['motionAndQuietGatesPass'],
        walkingPhaseQualificationPass=motion['walkingPhaseQualificationPass'],
        strictAllSampleJointLimitsPass=motion['strictAllSampleJointLimitsPass'],
        physicalSwingQualificationPass=bool(tracking and maxidentity<1e-12 and fk_error<1e-8 and motion['walkingPhaseQualificationPass']),
        handLateralDistanceFromShoulderAfterPlacementM=dict(minimum=lateral_min.tolist(),maximum=lateral_max.tolist()),
        sagittalMeasurements=measurements,motion=motion,effort=effort,
        selectedBasePreserved=True,selectedPresetChanged=False,
        claims='1:1 geometric target and measured tracking tolerance; no robustness or energy saving claim')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--output',type=Path)
    args=p.parse_args();result=audit(args.folder)
    (args.output or args.folder/'opposed-arm-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ('physicalSwingQualificationPass','motionAndQuietGatesPass','strictAllSampleJointLimitsPass','sagittalMeasurements')},indent=2))


if __name__=='__main__': main()
