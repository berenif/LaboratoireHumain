#!/usr/bin/env python3
"""Frozen imported teacher68 stance response. No training or step claims."""
import argparse,json
from pathlib import Path
import numpy as np
from actors import DenseActor
from native_env import PilotLedger,SourceNative,atomic_json,sha256,ROOT
from source_math import ORDER,teacher_observation
from run import source_target

POWER=.663420431

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--seed',type=Path,required=True);p.add_argument('--ledger',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ledger=PilotLedger(a.ledger);n=SourceNative(a.binary,a.seed,a.output,ledger,stage='frozenReference',physics_cap=3.)
    try:
        target=source_target();entry=n.command(dict(op='sourceState',now=target.tolist(),ahead=target.tolist(),teacher_power=POWER,holding=True));c0=np.array(entry['comSource']);before=n.command(dict(op='observe'))
        actorpath=ROOT/'rust/crates/sim/data/getup-policy/teacher.bin';actor=DenseActor.load(actorpath);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());gear=np.zeros(21)
        for part in model['segments']:
            h=part['reference'].get('hinge')
            if h:gear[h['qposIndex']]=h['gear']
        gear=gear[ORDER]
        config=dict(scope='frozen imported WEAK teacher68 on actual supported quiet GetUp endpoint; no learned step/teacher success claim',teacherPower=POWER,maximumNativeFrames=200,maximumSimSeconds=.25,physicsCapSeconds=3.,actionInterface='original normalized actuator torque, power once',actorSHA256=sha256(actorpath),binarySHA256=sha256(a.binary),seedSHA256=sha256(a.seed),stop='native failure, any flight/nonfoot, loaded slip>10mm, COM drift>20mm, any native height outside original envelope, parity mismatch',trainingUpdates=0,referenceInputsUsedByTeacher=False)
        atomic_json(a.output/'config.json',config);n.checkpoint(a.output/'entry.bin',dict(phase='frozen-teacher-stance-diagnostic',teacherPower=POWER,observedNativeSamples=0))
        records=[];samples=[];stopped=None;raw_error=0.;action_error=0.;torque_error=0.;drift=0.
        for _ in range(10):
            state=n.command(dict(op='sourceState',now=target.tolist(),ahead=target.tolist(),teacher_power=POWER,holding=True));pred=actor.infer(teacher_observation(state['physicalObservation'],POWER));raw_error=max(raw_error,float(np.max(abs(pred['raw']-state['teacher']['raw']))));action_error=max(action_error,float(np.max(abs(pred['action']-state['teacher']['action']))))
            if raw_error>2e-4 or action_error>5e-5:
                stopped='teacher-inference-parity-rejected-before-integration';break
            controls=state['teacher']['action'];result=n.command(dict(op='stepSourceTeacher',controls=controls,audit=True));records.append(result);samples.extend(result['native_foot_samples'])
            for frame in result['native_dynamics_audit'][1:]:torque_error=max(torque_error,float(np.max(abs(np.array(controls)*gear-frame['motorTorquesNmActuatorOrder']))))
            after=n.command(dict(op='sourceState',now=target.tolist(),ahead=target.tolist(),teacher_power=POWER,holding=True));drift=float(np.linalg.norm((np.array(after['comSource'])-c0)[:2]))
            if result['failure']or result['terminated']or result['truncated']:stopped=result['failure']or'native-ended'
            elif any(max(s['loads_n'])<3 for s in result['native_foot_samples']):stopped='observed-flight'
            elif any(s['nonfoot_support']for s in result['native_foot_samples']):stopped='observed-nonfoot'
            elif max(result['state']['loaded_slip_distance_m'])>.01:stopped='loaded-slip-over10mm'
            elif drift>.02:stopped='COM-drift-over20mm'
            elif any(not .780625<=s['com_height_m']<=.829976 for s in result['native_foot_samples']):stopped='height-envelope-exit'
            elif torque_error>2e-4:stopped='teacher-torque-parity-error'
            if stopped:break
        n.checkpoint(a.output/'final.bin',dict(phase='frozen-teacher-stance-diagnostic',teacherPower=POWER,observedNativeSamples=len(samples),stopped=stopped))
        report=dict(scope=config['scope'],stopped=stopped,teacherPower=POWER,observedSimSeconds=len(samples)/800.,nativeSubsteps=n.substeps,physicsWallSeconds=n.physics,maximumRaw42Error=raw_error,maximumAction21Error=action_error,maximumNativeTorqueEquationErrorNm=torque_error,flightSamples=sum(max(s['loads_n'])<3 for s in samples),nonfootSamples=sum(s['nonfoot_support']for s in samples),COMDriftM=drift,final=records[-1]['state']if records else before,actorSHA256=sha256(actorpath),binarySHA256=sha256(a.binary),scriptSHA256=sha256(__file__),taskDataCollected=False,optimizerUpdates=0,stepQualified=False)
        atomic_json(a.output/'summary.json',report);print(json.dumps(report,indent=2))
    finally:n.close();ledger.close()
if __name__=='__main__':main()
