#!/usr/bin/env python3
"""Reduce measured large-step journals, including stopped attempts; no physics."""
import argparse
import json
from collections import defaultdict,deque
from pathlib import Path
import numpy as np
from source_step_probe import ROOT
from source_opposed_arm_swing import pitch
from audit_source_opposed_arm_swing import amplitude,tracking_pass
from source_upright_large_steps import body_orientation,TILT_LIMIT_DEG
from source_step_preview import fk,P
from scipy.spatial.transform import Rotation
from audit_journal_io import digest, json_array_items


def audit(folder):
    config=json.loads((folder/'large-step-config.json').read_text()) if (folder/'large-step-config.json').exists() else {}
    summary=json.loads((folder/'multistep-summary.json').read_text()) if (folder/'multistep-summary.json').exists() else {}
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());kin=fk.Kinematics(model)
    gears=np.zeros(21);names=['']*21
    for part in model['segments']:
        h=part['reference'].get('hinge')
        if h:gears[h['qposIndex']]=h['gear'];names[h['qposIndex']]=h['name']
    count=flight=warnings=invalidrays=0;penetration=nonfoot=motor=0.;lastoutside=None;minjoint=1.;walkingmin=1.;capture=None;firstlift=None
    post_entry_joint_violations=undefined_capture=negative_capture=0;first_capture_violation=None
    groups=defaultdict(list);orientations=defaultdict(list);quiet=deque(maxlen=4000);last=None;fkerror=0.;fksamples=0;fkcompared=0;steps={};work=0.;references={};landing_runs=defaultdict(int);upright_samples=0
    for line in (folder/'frames.ndjson').open():
        row=json.loads(line);count+=1
        assert row['sourceStep']==count and abs(row['elapsedS']-count*.00125)<1e-12
        q=np.array(row['qpos']);v=np.array(row['qvel']);torque=np.array(row['actualMotorTorquesNm'])
        motor=max(motor,float(max(abs(torque/gears))));work+=float(sum(abs(torque*v[6:]))*.00125)
        warnings=max(warnings,row['warnings']);nonfoot=max(nonfoot,row['nonfootLoadN']);flight+=all(f['loadN']<3 for f in row['feet']);penetration=max(penetration,row['maximumSelfPenetrationM'])
        invalidrays+=sum(not r['hit'] or not r['ownBodyExcluded'] for r in row['jointGroundRays'])
        invalidrays+=sum(not r['hit'] or not r['ownBodyExcluded'] for f in row['feet'] for r in f['groundRays'])
        minjoint=min(minjoint,row['jointLimitMinimumMarginRad'])
        if row['jointLimitMinimumMarginRad']<0:lastoutside=row['elapsedS']
        if row['elapsedS']>.06375 and row['jointLimitMinimumMarginRad']<0:post_entry_joint_violations+=1
        if firstlift is None and row['phase']=='lift':firstlift=row['elapsedS']
        if firstlift is not None:walkingmin=min(walkingmin,row['jointLimitMinimumMarginRad'])
        margin=row['minimumComCaptureMarginM']
        if margin is not None:capture=margin if capture is None else min(capture,margin)
        else:undefined_capture+=1
        if margin is not None and margin<0:negative_capture+=1
        if first_capture_violation is None and (margin is None or margin<0):first_capture_violation=dict(timeS=row['elapsedS'],phase=row['phase'],marginM=margin)
        if 'uprightMeasured' in row:
            upright_samples+=1
            for body,value in row['uprightMeasured'].items():
                orientations[body].append([value[k]for k in ('pitchDeg','rollDeg','tiltDeg','yawDeg')])
                if row['phase']in ('lift','translate','land'):orientations[body+'Gait'].append([value[k]for k in ('pitchDeg','rollDeg','tiltDeg','yawDeg')])
        if not(row['stepIndex']==1 and row['phase']=='transfer'):
            swing=row['opposedArmSwing'];values=(np.array(swing['armPitchRad']),np.array(swing['legPitchRad']),np.array(swing['targetArmPitchRad']))
            if row['phase']in ('lift','translate','land'):
                groups['allGait'].append(values);groups[f"step{row['stepIndex']}Gait"].append(values)
        current=steps.setdefault(row['stepIndex'],dict(maximumForwardM=-1.,finalForwardM=0.,maximumSupportRelativeForwardM=-1.,maximumStanceOriginDisplacementM=0.,maximumStanceOriginHorizontalDisplacementM=0.,landingQualified=False,qualifiedLandingForwardM=None))
        current['maximumForwardM']=max(current['maximumForwardM'],row['swingForwardDisplacementM']);current['finalForwardM']=row['swingForwardDisplacementM']
        if row['stepIndex']not in references:references[row['stepIndex']]=json.loads((folder/f"controller-step-{row['stepIndex']:02d}.json").read_text())
        side=row['swingSide'];separation=float((np.array(row['feet'][side]['position'])-np.array(row['feet'][1-side]['position']))@np.array(references[row['stepIndex']]['heading']))
        current['maximumSupportRelativeForwardM']=max(current['maximumSupportRelativeForwardM'],separation)
        reference=references[row['stepIndex']];observed=float((np.array(row['feet'][side]['position'])-np.array(reference['targets'][side][0]))@np.array(reference['heading']))
        assert abs(observed-row['swingForwardDisplacementM'])<1e-12
        stance_delta=np.array(row['feet'][1-side]['position'])-np.array(reference['targets'][1-side][0])
        current['maximumStanceOriginDisplacementM']=max(current['maximumStanceOriginDisplacementM'],float(np.linalg.norm(stance_delta)))
        current['maximumStanceOriginHorizontalDisplacementM']=max(current['maximumStanceOriginHorizontalDisplacementM'],float(np.linalg.norm(stance_delta[:2])))
        current['finalStanceOriginHorizontalDisplacementM']=float(np.linalg.norm(stance_delta[:2]))
        qualifies=row['phase']=='land'and all(f['loadN']>=3 for f in row['feet'])and row['feet'][side]['clearance']<=.003 and observed>=row['requestedForwardM']
        landing_runs[row['stepIndex']]=landing_runs[row['stepIndex']]+1 if qualifies else 0
        landing_duration=reference.get('phase_durations',{}).get('land',reference.get('trajectory_time_scale',1.))
        if landing_runs[row['stepIndex']]>=80 and row['phaseElapsedS']>=landing_duration and not current['landingQualified']:
            current['landingQualified']=True;current['qualifiedLandingForwardM']=observed;current['qualifiedLandingTimeS']=row['elapsedS']
        if count==1 or count%20==0:
            root=np.eye(4);root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P;root[:3,3]=P.T@q[:3];poses=kin.forward(q[7:],root);fksamples+=1
            for body in ('torso','pelvis'):
                independent=body_orientation(P@poses[body][:3,:3]@P.T)
                if 'uprightMeasured'in row:
                    fkcompared+=1
                    fkerror=max(fkerror,max(abs((independent[k]-row['uprightMeasured'][body][k]+180.)%360.-180.)for k in independent))
        quiet.append(row);last=row
    assert count and last is not None
    note_path=folder/'control.json';notes=json_array_items(note_path)if note_path.exists() else ()
    minimumseparation=min((p['separationM']for n in notes for p in n['selfAvoidance']),default=None)
    orientation_summary={}
    for body,values in orientations.items():
        a=np.array(values);orientation_summary[body]=dict(maximumAbsPitchDeg=float(max(abs(a[:,0]))),maximumAbsRollDeg=float(max(abs(a[:,1]))),maximumTiltDeg=float(max(a[:,2])),rmsTiltDeg=float(np.sqrt(np.mean(a[:,2]**2))),finalPitchDeg=float(a[-1,0]),finalRollDeg=float(a[-1,1]),finalTiltDeg=float(a[-1,2]))
    amplitudes={k:amplitude(values)for k,values in groups.items()}
    def isquiet(r):
        return r['phase']=='stand' and all(f['loadN']>=3 for f in r['feet'])and sum(f['loadN']for f in r['feet'])>=.8*model['totalMassKg']*9.81 and r['torsoTiltDeg']<=5 and r['maximumBodyLinearMps']<=.22 and r['maximumBodyAngularRadps']<=.65 and r['minimumComCaptureMarginM']is not None and r['minimumComCaptureMarginM']>=.003 and .780625<=r['com'][2]<=.829976 and r.get('standingAnchorDriftM',999)<=.01 and r['maximumSelfPenetrationM']<=.005
    basequiet=len(quiet)==4000 and all(isquiet(r)for r in quiet)
    uprightquiet=basequiet and all(r.get('uprightMeasured',{}).get('pelvis',{}).get('tiltDeg',999)<=5 and r.get('uprightMeasured',{}).get('torsoRelativePelvis',{}).get('tiltDeg',999)<=5 for r in quiet)
    cone=all(orientation_summary.get(body,{}).get('maximumTiltDeg',999)<=TILT_LIMIT_DEG+1e-8 for body in ('torso','pelvis'))
    result=dict(schema=1,savedDataOnly=True,physicsIntegrations=0,actualRecordedIntegrations=count,physicsSimSeconds=count*.00125,
        stopped=summary.get('stopped','incomplete-process-journal'),runnerCompleted=summary.get('passed',False),config=config,
        steps=summary.get('steps',[]),measuredPerStep=steps,maximumMotorFraction=motor,minimumWalkingJointReserveDeg=float(np.rad2deg(walkingmin))if firstlift is not None else None,
        minimumAllSampleJointReserveDeg=float(np.rad2deg(minjoint)),lastOutsideJointLimitTimeS=lastoutside,strictAllSampleJointLimitsPass=minjoint>=0,
        globalLoadedSlipM=last['loadedSlipDistanceM'],minimumComCaptureMarginM=capture,maximumSelfPenetrationM=penetration,
        postEntryJointViolationSamples=post_entry_joint_violations,undefinedCaptureSamples=undefined_capture,negativeCaptureSamples=negative_capture,firstCaptureViolation=first_capture_violation,
        observedMeasuredRuntimeGuardsPass=post_entry_joint_violations==0 and undefined_capture==0 and negative_capture==0,
        maximumNonfootLoadN=nonfoot,flightSamples=int(flight),warnings=warnings,invalidRaySamples=invalidrays,
        minimumCommandSampledSelfSeparationM=minimumseparation,mechanicalAbsoluteWorkJ=work,
        orientation=orientation_summary,observed12DegreeTiltBoundPass=cone,finalBaseQuietSamples=4000 if basequiet else 0,
        finalUprightQuietSamples=4000 if uprightquiet else 0,armAmplitude=amplitudes,
        armTrackingPass=bool(amplitudes)and all(tracking_pass(m)for m in amplitudes.values()),
        independentFKCheck=dict(samples=fksamples,comparedBodyTelemetry=fkcompared,allFramesHaveUprightTelemetry=upright_samples==count,maximumWrappedOrientationErrorDeg=fkerror,coveragePass=upright_samples==count and fkcompared==2*fksamples,physicsIntegrations=0),
        hashes={name:digest(folder/name)for name in ('frames.ndjson','control.json','final.bin')if(folder/name).exists()})
    if summary.get('passed'):
        from audit_source_multistep import audit as original_audit
        result['originalMotionAudit']=original_audit(folder)
    result['allRecordedStepsHaveMeasuredLanding']=bool(steps)and all(step['landingQualified']for step in steps.values())
    result['auditedMotionPass']=bool(result['allRecordedStepsHaveMeasuredLanding']and result.get('originalMotionAudit',{}).get('walkingPhaseQualificationPass',False)and result['observedMeasuredRuntimeGuardsPass'])
    result['auditedOpposedStepPass']=bool(result['auditedMotionPass']and result['armTrackingPass']and result['independentFKCheck']['coveragePass']and fkerror<1e-8)
    result['auditedUprightStepPass']=bool(config.get('uprightEnabled')and result['auditedMotionPass']and result['observedMeasuredRuntimeGuardsPass']and uprightquiet and cone and result['armTrackingPass']and result['independentFKCheck']['coveragePass']and fkerror<1e-8)
    cadence_path=folder/'cadenced-step-config.json'
    if cadence_path.exists():
        cadence=json.loads(cadence_path.read_text())
        expected=cadence['requestedSteps']
        records=summary.get('steps',[])
        complete=len(records)==expected and list(steps)==list(range(1,expected+1))
        alternating=complete and all(s['swingSide']==('left'if i%2==0 else'right')for i,s in enumerate(records))
        continuous=result.get('originalMotionAudit',{}).get('positionVelocityContinuityMaxErrorM',999)<1e-10 and result.get('originalMotionAudit',{}).get('hingePositionVelocityContinuityMaxErrorRad',999)<1e-10
        result['cadencedSequence']=dict(requestedSteps=expected,recordedSteps=len(records),completeAlternatingSequence=alternating,continuousRecordedIntegration=continuous,finalFiveSecondsQuiet=basequiet,physicalTimeS=count*.00125,landingTimesS=[steps[k].get('qualifiedLandingTimeS')for k in steps],cadenceConfig=cadence)
        result['auditedCadencedStepsPass']=bool(result['auditedOpposedStepPass']and alternating and continuous and basequiet)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=audit(a.folder);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:r[k]for k in ('stopped','actualRecordedIntegrations','globalLoadedSlipM','orientation')},indent=2))
