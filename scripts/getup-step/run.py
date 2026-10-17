#!/usr/bin/env python3
"""Source parity and explicitly bounded native positive control; no training."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from actors import DenseActor
import source_math as m
from native_env import ROOT,PilotLedger,SourceNative,atomic_json,sha256


def source_target():
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    q=model.get('referenceMetadata',{}).get('standingPose')
    if q is None or len(q)!=21:
        raise ValueError('model lacks the source standingPose21 metadata')
    return m.target(q,1.4,[0,0,1.])


def frame_parity(physical,snapshot):
    parts={x['id']:x for x in snapshot['segments']}
    xyz=lambda d:np.array([d[k]for k in 'xyz'])
    torso=parts['torso'];rot=Rotation.from_quat([torso['rotation'][k]for k in 'xyzw']).as_matrix()
    expected={21:parts['head']['position']['y']}
    errors=[abs(physical[21]-expected[21])]
    for i,name in enumerate(['left_hand','left_foot','right_hand','right_foot']):
        point=m.P@rot.T@(xyz(parts[name]['position'])-xyz(torso['position']))
        errors.extend(abs(physical[22+3*i:25+3*i]-point))
    errors.extend(abs(physical[34:37]-m.P@rot.T@np.array([0.,1.,0.])))
    return float(max(errors))


def parity(native):
    before=native.command(dict(op='observe'));snap=native.command(dict(op='snapshot'))
    ref=source_target();teacher=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin');student=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin')
    report=dict(scope='source-interface parity, no stepping claim',rawMaxError=0.,actionMaxError=0.,controlMaxError=0.,frameMaxError=0.)
    for holding in [False,True]:
        state=native.command(dict(op='sourceState',now=ref.tolist(),ahead=ref.tolist(),teacher_power=.663420431,holding=holding))
        p=np.array(state['physicalObservation']);obs=m.student_observation(p,ref,ref);tobs=m.teacher_observation(p,.663420431)
        np.testing.assert_array_equal(obs,np.array(state['studentObservation'],dtype=np.float32));np.testing.assert_array_equal(tobs,np.array(state['teacherObservation'],dtype=np.float32))
        for actor,x,name in [(student,obs,'student'),(teacher,tobs,'teacher')]:
            prediction=actor.infer(x)
            report['rawMaxError']=max(report['rawMaxError'],float(max(abs(prediction['raw']-state[name]['raw']))))
            report['actionMaxError']=max(report['actionMaxError'],float(max(abs(prediction['action']-state[name]['action']))))
        expected=m.slow_controls(ref[:21],state['student']['action'],p[:21],p[46:67],holding)
        report['controlMaxError']=max(report['controlMaxError'],float(max(abs(expected-state['studentMotorControlsActuatorOrder']))))
        report['frameMaxError']=max(report['frameMaxError'],frame_parity(p,snap))
    assert report['rawMaxError']<2e-4 and report['actionMaxError']<5e-5
    assert report['controlMaxError']<2e-6 and report['frameMaxError']<2e-6
    for bad in [dict(op='stepSourceTeacher',controls=[2.]*21),dict(op='stepSourceStudent',reference=[0.]*21,residual=[2.]*21,holding=False),dict(op='stepSourceStudent',reference=[0.]*20,residual=[0.]*21,holding=False)]:
        native.command(bad,expect_ok=False)
    assert snap==native.command(dict(op='snapshot'))
    assert before==native.command(dict(op='observe'))
    cp=native.output/'parity-checkpoint.bin'
    native.checkpoint(cp,dict(phase='parity-only',reference=ref.tolist(),clock=None))
    assert native.command(dict(op='load',path=str(cp.resolve())))==before
    assert native.substeps==0 and native.physics==0
    report.update(invalidCommandsRejected=3,readOnlySnapshotParity=True,checkpointRoundtrip=True,nativeSubsteps=0,physicsWallSeconds=0.)
    return report


def positive_control(native):
    """Preregistered short known-GetUp interface test, not a step trial.

    4 legacy + 4 source holding decisions; 3 moving decisions (one entry and
    two exact repeats); 3 constant-torque calibration decisions. Total280 native frames.
    Reloads are explicitly independent parity forks, never a success rollout.
    """
    ref=source_target();initial=native.output/'initial.bin'
    native.checkpoint(initial,dict(phase='P1-parity-entry',reference=ref.tolist()))
    records=[];torque_errors=[]
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());gears=np.zeros(21)
    for part in model['segments']:
        hinge=part.get('reference',{}).get('hinge')
        if hinge:gears[hinge['actuatorIndex']]=hinge['gear']
    def state(holding):return native.command(dict(op='sourceState',now=ref.tolist(),ahead=ref.tolist(),holding=holding))
    def checked(request):
        result=native.command(request);assert not result['failure'] and not result['terminated'] and not result['truncated']
        rows=result['native_foot_samples'];assert len(rows)==20
        assert all(not x['nonfoot_support'] and max(x['loads_n'])>=3. for x in rows)
        records.append(result);return result
    legacy=[]
    for _ in range(4):
        x=state(True);legacy.append(checked(dict(op='step',action=x['student']['action'])))
    legacy_snapshot=native.command(dict(op='snapshot'))
    native.command(dict(op='load',path=str(initial.resolve())))
    for expected in legacy:
        x=state(True);result=checked(dict(op='stepSourceStudent',reference=ref[:21].tolist(),residual=x['student']['action'],holding=True))
        assert result['state']==expected['state']
        assert result['native_foot_samples']==expected['native_foot_samples']
    source_snapshot=native.command(dict(op='snapshot'))
    def canonical_snapshot(x):
        x=copy.deepcopy(x)
        for name in ['controller_ms','update_ms']:x['diagnostics'].pop(name,None)
        x['diagnostics']['getup']['controller']='declared-interface-mode'
        return x
    assert canonical_snapshot(legacy_snapshot)==canonical_snapshot(source_snapshot)
    # Moving damping and full schema3 next-decision evolution parity.
    native.command(dict(op='load',path=str(initial.resolve())))
    x=state(False);request=dict(op='stepSourceStudent',reference=ref[:21].tolist(),residual=x['student']['action'],holding=False,audit=True)
    checked(request);moving=native.output/'moving.bin';native.checkpoint(moving,dict(phase='P1-moving',reference=ref.tolist(),holding=False))
    x=state(False);request=dict(op='stepSourceStudent',reference=ref[:21].tolist(),residual=x['student']['action'],holding=False,audit=True)
    a=checked(request);native.command(dict(op='load',path=str(moving.resolve())));b=checked(request)
    assert a['state']==b['state'] and a['native_foot_samples']==b['native_foot_samples']
    # Audit each command with its actual held residual from the persisted journal.
    for line in (native.output/'protocol.ndjson').read_text().splitlines():
        row=json.loads(line);req=row['request'];response=row['response']
        if req.get('op')!='stepSourceStudent' or not req.get('audit')or not response['ok']:continue
        frames=response['result']['native_dynamics_audit']
        for before,after in zip(frames,frames[1:]):
            q=np.array(before['physicalObservation']);expected=m.slow_controls(req['reference'],req['residual'],q[:21],q[46:67],req['holding'])*gears
            torque_errors.append(float(max(abs(expected-after['motorTorquesNmActuatorOrder']))))
    # Teacher bridge uses a declared supported initial control for calibration;
    # this is not a successful learned torque-teacher rollout.
    native.command(dict(op='load',path=str(initial.resolve())))
    x=state(True);controls=x['studentMotorControlsActuatorOrder']
    request=dict(op='stepSourceTeacher',controls=controls,audit=True)
    checked(request)
    teacher_entry=native.output/'teacher-entry.bin';native.checkpoint(teacher_entry,dict(phase='P1-teacher-interface-calibration',controls=controls))
    a=checked(request);native.command(dict(op='load',path=str(teacher_entry.resolve())));b=checked(request)
    assert a['state']==b['state'] and a['native_foot_samples']==b['native_foot_samples']
    for result in [a,b]:
        for after in result['native_dynamics_audit'][1:]:torque_errors.append(float(max(abs(np.array(controls)*gears-after['motorTorquesNmActuatorOrder']))))
    assert max(torque_errors)<2e-4
    return dict(scope='P1 short source GetUp positive control and independent parity forks; no step/learning claim',
        holdingLegacySourceEvolutionExact=True,movingCheckpointEvolutionExact=True,teacherCheckpointEvolutionExact=True,
        maximumNativeTorqueEquationErrorNm=max(torque_errors),nativeSubsteps=native.substeps,
        physicsWallSeconds=native.physics,flightSamples=0,nonfootSamples=0,
        allHoldingNativeStanding=all(x['standing_without_foot_anchor']for result in records[:8]for x in result['native_foot_samples']))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode',choices=['parity','positive-control'])
    ap.add_argument('--binary',type=Path,required=True);ap.add_argument('--checkpoint',type=Path,required=True)
    ap.add_argument('--ledger',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();ledger=PilotLedger(a.ledger)
    native=SourceNative(a.binary,a.checkpoint,a.output,ledger,stage='knownSourceParity',physics_cap=0. if a.mode=='parity' else 3.)
    try:
        report=parity(native) if a.mode=='parity' else positive_control(native);report.update(binarySHA256=sha256(a.binary),checkpointSHA256=sha256(a.checkpoint),scriptSHA256=sha256(__file__))
        atomic_json(a.output/'summary.json',report);print(json.dumps(report,indent=2))
    finally:native.close()
if __name__=='__main__':main()
