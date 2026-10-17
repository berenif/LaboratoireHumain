#!/usr/bin/env python3
"""Bounded quiet-state weight-transfer positive control; not a recovery fix.

The root transform in the retained plan is mathematical FK only. Physics gets
ordinary existing SLOW residuals plus smooth joint-angle offsets, never a root
position, pose, velocity, force, contact constraint or changed motor gain.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('probe',ROOT/'scripts/probe-getup-recovery-step.py')
probe = importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def transfer_weight(elapsed, hold_until=3.):
    if elapsed <= 1.5:return probe.smooth(elapsed/1.5)
    if elapsed <= hold_until:return 1.
    return 1-probe.smooth((elapsed-hold_until)/1.)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--physics-budget',type=float,required=True);p.add_argument('--deadline',required=True)
    p.add_argument('--control',choices=['source-bias','conditioned-reference'],default='source-bias')
    p.add_argument('--seconds',type=float,default=11.)
    p.add_argument('--hold-until',type=float,default=3.)
    p.add_argument('--execute',action='store_true');a = p.parse_args()
    deadline = datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp()
    assert time.time() < deadline <= datetime.datetime(2026,10,17,13,6,tzinfo=datetime.timezone.utc).timestamp()
    assert 0 <= a.physics_budget <= 90
    assert 10 <= a.seconds <= 15 and abs(round(a.seconds*40)-a.seconds*40)<1e-8
    assert 3 <= a.hold_until <= 7 and a.hold_until+6 <= a.seconds
    handoff_tick = round((a.hold_until+1)*40)
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close()
    plans = json.loads(a.plan.read_text());plan = plans[0]
    assert plan['loweringM'] == -.01 and plan['abdomenRollDeltaRad'] == .12
    delta = np.array(plan['deltaQ']);assert np.max(np.abs(delta)) < .3
    path_file = a.plan.parent/'ik-path-feasibility.json'
    path = json.loads(path_file.read_text());assert path['maximumFootPositionErrorM'] < .0001
    weights = np.array([r['weight'] for r in path['pathSamples']])
    deltas = np.array([r['deltaQ'] for r in path['pathSamples']])
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    nominal = np.array(model['referenceMetadata']['standingPose'])
    config = {'purpose':'quiet-state deliberate transfer positive control; not original shuffling fix or learned skill',
        'physicsCapSeconds':a.physics_budget,'deadlineUTC':a.deadline,'secondsPerCase':a.seconds,
        'rampSeconds':1.5,'holdUntilS':a.hold_until,'returnUntilS':a.hold_until+1,'sourceHandoffTick':handoff_tick,
        'jointBiasDeltaRad':delta.tolist(),'plan':plan,'control':a.control,
        'sourceComposition':'frozen-reactive-SLOW-plus-fixed-pose-displacement' if a.control=='source-bias' else 'frozen-SLOW-original-now-ahead-reference-conditioning',
        'noRootOrFootConstraints':True,'noPhysicsPoseVelocityWrites':True,'checkpointSha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'planFileSha256':hashlib.sha256(a.plan.read_bytes()).hexdigest(),
        'pathFileSha256':hashlib.sha256(path_file.read_bytes()).hexdigest(),'pathFootPositionErrorMaxM':path['maximumFootPositionErrorM'],
        'trajectory':'41 fixed-feet IK knots along virtual-root/abdomen trajectory; no linear whole-pose interpolation',
        'nativeBinarySha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
        'acceptance':'Measure actual COM/capture approach, persistent right load share and left unload, foot slip, flight and exact source handoff. '
            'No lift or recovery improvement is claimed by this transfer-only test.'}
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    native = probe.Native(a.binary,a.checkpoint,a.physics_budget,deadline,a.output)
    report = {'status':'preflight-only','cases':[]}
    try:
        initial = native.command({'op':'observe'});diag = native.command({'op':'diagnostic'})
        assert initial['metrics']['native_standing_without_foot_anchor']
        assert np.linalg.norm(np.array(initial['observation'][:21])-np.array(json.loads((a.plan.parent/'plan.json').read_text())['initial']['observation'][:21])) < 1e-6
        if a.execute:
            report['status'] = 'running-transfer-positive-control'
            for label in ['baseline','hybrid']:
                state = native.command({'op':'reset'});initial_diag = native.command({'op':'diagnostic'})
                contacts = initial_diag['contactMotion']['feet'][1]['contacts']
                target = np.average([probe.reference.xyz(c['point'])[[0,2]] for c in contacts],axis=0,
                    weights=[c['normalImpulseNs'] for c in contacts])
                samples = [];frames = [];history = [];notes = [];stopped = None;right_run = longest_right = 0
                for tick in range(round(a.seconds*40)):
                    source = np.array(native.command({'op':'initializedAction'})['action'])
                    weight = transfer_weight(tick/40,a.hold_until) if label == 'hybrid' else 0.
                    bias = np.array([np.interp(weight,weights,deltas[:,i]) for i in range(21)])
                    raw = source+bias;action = np.clip(raw,-1.,1.)
                    reference_control_error = 0.;reference_clipped = False
                    if label == 'hybrid' and a.control == 'conditioned-reference':
                        ahead_weight = transfer_weight(tick/40+.125,a.hold_until)
                        if weight > 0 or ahead_weight > 0:
                            ahead_bias = np.array([np.interp(ahead_weight,weights,deltas[:,i]) for i in range(21)])
                            now = np.r_[nominal+bias,1.4,0,0,1].tolist()
                            ahead = np.r_[nominal+ahead_bias,1.4,0,0,1].tolist()
                            response = native.command({'op':'referenceAction','now':now,'ahead':ahead})
                            action = np.array(response['action']);raw = np.array(response['effectiveTarget'])-nominal
                            reference_control_error = response['referenceControlMaxError'];reference_clipped = response['actionClipped']
                    tr = native.command({'op':'step','action':action.tolist()});state = tr['state']
                    native_samples = tr['native_foot_samples'];samples.extend(native_samples)
                    diagnostic = native.command({'op':'diagnostic'})
                    com = probe.reference.xyz(diagnostic['com'])[[0,2]];capture = probe.reference.xyz(diagnostic['capture'])[[0,2]]
                    for s in native_samples:
                        right_fraction = s['loads_n'][1]/max(1e-9,sum(s['loads_n']))
                        good = right_fraction >= .85 and s['loads_n'][1] >= .7*model['totalMassKg']*9.81
                        right_run = right_run+1 if good else 0;longest_right = max(longest_right,right_run)
                    history.append({'tick':tick+1,'elapsedS':state['elapsed_s'],'captureInSupport':diagnostic['predicates']['captureInSupport'],
                        'comInSupport':diagnostic['predicates']['comInSupport'],'capture':diagnostic['capture'],'com':diagnostic['com'],
                        'supportPoints':diagnostic['solverSupportPoints'],'comDistanceToInitialRightSupportM':float(np.linalg.norm(com-target)),
                        'captureDistanceToInitialRightSupportM':float(np.linalg.norm(capture-target)),
                        'rightLoadFraction':state['metrics']['foot_loads_n'][1]/max(1e-9,sum(state['metrics']['foot_loads_n']))})
                    notes.append({'phase':'transfer' if tick/40<a.hold_until else ('return' if tick<handoff_tick else 'done'),
                        'transferWeight':weight,'overrideMaxRad':float(np.max(np.abs(action-source))),
                        'sourceEffectiveTargetRad':(nominal+source).tolist(),'representedEffectiveTargetRad':(nominal+action).tolist(),
                        'customReferenceControlMaxError':reference_control_error,'customReferenceActionClipped':reference_clipped,
                        'maximumRepresentationErrorRad':float(np.max(np.abs(action-raw)))})
                    if tick%2 == 0:frames.append(native.command({'op':'snapshot'}))
                    if tr['terminated'] or tr['truncated']:
                        stopped = tr['failure'] or 'episode-ended';break
                metrics = probe.analyse(samples)
                metrics.update({'label':label,'stopped':stopped,'final':state,'initialDiagnostic':initial_diag,
                    'finalDiagnostic':diagnostic,'maximumContinuousRightLoadShareAbove85PercentS':longest_right/800,
                    'minimumComDistanceToInitialRightSupportM':min(r['comDistanceToInitialRightSupportM'] for r in history),
                    'minimumCaptureDistanceToInitialRightSupportM':min(r['captureDistanceToInitialRightSupportM'] for r in history),
                    'maximumRightLoadFraction':max(r['rightLoadFraction'] for r in history),
                    'maximumFootOriginDisplacementM':[max(np.linalg.norm(probe.reference.xyz(s['positions'][i])-probe.reference.xyz(initial_diag['feet'][i])) for s in samples) for i in range(2)],
                    'controllerEvents':[{'event':'source-handoff','tick':handoff_tick}] if len(notes)>handoff_tick and notes[handoff_tick]['overrideMaxRad']==0 else []})
                (a.output/f'{label}.json').write_text(json.dumps({'metrics':metrics,'frames':frames,'controlNotes':notes,'supportHistory':history},indent=2)+'\n')
                native.command({'op':'checkpoint','path':str((a.output/f'{label}-final.bin').resolve())})
                report['cases'].append(metrics)
                print(json.dumps({k:metrics[k] for k in ['label','stopped','maximumContinuousRightLoadShareAbove85PercentS',
                    'minimumComDistanceToInitialRightSupportM','maximumRightLoadFraction','flightNativeSamples']}),flush=True)
            report['status'] = 'completed-transfer-positive-control-not-qualification'
    except BaseException as error:
        report['status'] = 'failed-or-incomplete-transfer' if a.execute else 'failed-preflight';report['error'] = repr(error);raise
    finally:
        try:native.close()
        except Exception as error:report['closeError'] = repr(error);report['status'] = 'failed-or-incomplete-transfer'
        report.update({'physicsWallSeconds':native.wall,'nativeSubsteps':native.native,'uncertainOutstandingRequests':native.uncertain})
        (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':main()
