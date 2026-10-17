#!/usr/bin/env python3
"""Short zero-residual source-PD baseline; no trained policy/task success claim."""
import argparse,json
from pathlib import Path
import numpy as np
from native_env import PilotLedger,SourceNative,atomic_json,sha256
from source_math import slow_controls,ORDER
from reference import UniformReference,native_pose_to_source_qpos
from scipy.spatial.transform import Rotation
from run import source_target
from hybrid_state import save_task_checkpoint,restore_task_checkpoint


def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--seed',type=Path,required=True);p.add_argument('--entry',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--ledger',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raw=json.loads(a.reference.read_text());ref=UniformReference(raw['trajectory'],raw['terminal'],kind='planned-unqualified');ledger=PilotLedger(a.ledger)
    n=SourceNative(a.binary,a.seed,a.output,ledger,stage='frozenReference',physics_cap=3.)
    try:
        before=n.command(dict(op='load',path=str(a.entry.resolve())));diag=n.command(dict(op='diagnostic'));native0=diag['nativeSubsteps'];snap0=n.command(dict(op='snapshot'));now,ahead=ref.now_ahead(0);state0=n.command(dict(op='sourceState',now=now.tolist(),ahead=ahead.tolist(),holding=False))
        physical=np.array(state0['physicalObservation']);np.testing.assert_allclose(now[:20],physical[:20],atol=1e-7,rtol=0.)
        torso=next(x for x in snap0['segments']if x['id']=='torso');rotation=Rotation.from_quat([torso['rotation'][k]for k in'xyzw']).as_matrix();actual_qpos=native_pose_to_source_qpos([torso['position'][k]for k in'xyz'],rotation,physical[:21]);planned_qpos=np.array(raw['trajectory']['qpos'][0]);planned_qvel=np.array(raw['trajectory']['qvel'][0]);actual_qvel=physical[40:67]
        if planned_qpos[3:7]@actual_qpos[3:7]<0:actual_qpos[3:7]*=-1
        binding=dict(rootAndNonElbowPositionMaxError=float(np.max(abs(planned_qpos[:27]-actual_qpos[:27]))),rootAndNonElbowVelocityMaxError=float(np.max(abs(planned_qvel[:26]-actual_qvel[:26]))),elbowReferencePositionDifference=float(planned_qpos[27]-actual_qpos[27]),elbowReferenceVelocityDifference=float(planned_qvel[26]-actual_qvel[26]),comMaxErrorM=float(np.max(abs(np.array(raw['trajectory']['com'][0])-state0['comSource']))),verticalMaxError=float(np.max(abs(now[22:]-physical[34:37]))))
        if binding['rootAndNonElbowPositionMaxError']>2e-6 or binding['rootAndNonElbowVelocityMaxError']>2e-6 or binding['comMaxErrorM']>2e-5 or binding['verticalMaxError']>2e-6:raise ValueError('planned bridge does not bind to actual native entry')
        exception=raw['entryHybridReferenceException']
        if abs(binding['elbowReferencePositionDifference']-exception['positionDifference'])>1e-12 or abs(binding['elbowReferenceVelocityDifference']-exception['velocityDifference'])>1e-8:raise ValueError('elbow exception differs from admitted recorded seed')
        c0=np.array(state0['comSource']);heading=np.array([.97177241,0.,.23592030]);heading/=np.linalg.norm(heading)
        controller=dict(schema=1,phase='prepare',referenceSHA256=sha256(a.reference),configSHA256=sha256(a.config),nativeId=native0,phaseWallStartNativeId=native0,referenceClock=dict(tau=0.,rate=1.,last_native_substep=native0,target_rate=1.,response_seconds=.1),unloadRun=0,clearanceRun=0,touchdownRun=0,loadRun=0,finalQuietRun=0,awardedEvents=[],entryHeading=heading.tolist(),entryFootOrigin=[before['metrics']['foot_positions'][0][k]for k in'xyz'],physicalSlipAnchors=dict(ownedByExactNativeCheckpoint=True,loadedSlipAtEntry=before['loaded_slip_distance_m']),postLandingAnalysisAnchors=None,lastAction=before['metrics']['controls'],RNG=dict(algorithm='numpy.PCG64',seed=20261017),scope='zero residual PD diagnostic only; no task data or trained teacher')
        save_task_checkpoint(n,a.output/'entry.bin',controller,reference_path=a.reference,config_path=a.config);restored=restore_task_checkpoint(n,a.output/'entry.bin',reference_path=a.reference,config_path=a.config);assert restored==controller
        config=dict(scope='short actual whole-body preparation reference with source movingPD and residual=0; not trained GetUp/student/teacher',maximumSimSeconds=.25,maximumNativeFrames=200,physicsCapSeconds=3.,dampingSeconds=.1,referenceSHA256=sha256(a.reference),entrySHA256=sha256(a.entry),stop='physical/native failure, flight, nonfoot, slip>10mm, COM drift>20mm, original height envelope exit',frozenActorApplied=False,referenceHeightActual=True,actualEntryBinding=binding)
        atomic_json(a.output/'config.json',config);records=[];samples=[];snapshots=[snap0];stopped=None;max_torque_error=0.
        model=json.loads((Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json').read_text());gears=np.zeros(21)
        for part in model['segments']:
            h=part['reference'].get('hinge')
            if h:gears[h['qposIndex']]=h['gear']
        gears=gears[ORDER]
        for k in range(10):
            now,ahead=ref.now_ahead(k);result=n.command(dict(op='stepSourceStudent',reference=now[:21].tolist(),residual=[0.]*21,holding=False,audit=True));records.append(result);samples.extend(result['native_foot_samples']);snapshots.append(n.command(dict(op='snapshot')))
            for b,after in zip(result['native_dynamics_audit'],result['native_dynamics_audit'][1:]):
                physical=np.array(b['physicalObservation']);expected=slow_controls(now[:21],np.zeros(21),physical[:21],physical[46:67],False)*gears;max_torque_error=max(max_torque_error,float(np.max(abs(expected-after['motorTorquesNmActuatorOrder']))))
            last=n.command(dict(op='sourceState',now=now.tolist(),ahead=ahead.tolist(),holding=False));drift=float(np.linalg.norm((np.array(last['comSource'])-c0)[:2]))
            if result['failure']or result['terminated']or result['truncated']:stopped=result['failure']or'native-ended'
            elif any(max(s['loads_n'])<3 for s in result['native_foot_samples']):stopped='observed-flight'
            elif any(s['nonfoot_support']for s in result['native_foot_samples']):stopped='observed-nonfoot'
            elif max(result['state']['loaded_slip_distance_m'])>.01:stopped='loaded-slip-over-10mm'
            elif drift>.02:stopped='COM-drift-over20mm'
            elif any(not .780625<=s['com_height_m']<=.829976 for s in result['native_foot_samples']):stopped='height-envelope-exit'
            elif max_torque_error>2e-4:stopped='source-PD-torque-parity-error'
            if stopped:break
        # A parity error stops BEFORE another command and is saved as a failure.
        controller.update(nativeId=native0+len(samples),lastAction=records[-1]['state']['metrics']['controls']);controller['referenceClock'].update(tau=len(samples)/800.,last_native_substep=native0+len(samples))
        save_task_checkpoint(n,a.output/'final.bin',controller,reference_path=a.reference,config_path=a.config)
        atomic_json(a.output/'actual-snapshots.json',snapshots)
        report=dict(scope=config['scope'],stopped=stopped,observedSimSeconds=len(samples)/800.,nativeSubsteps=n.substeps,physicsWallSeconds=n.physics,maximumNativeTorqueEquationErrorNm=max_torque_error,flightSamples=sum(max(s['loads_n'])<3 for s in samples),nonfootSamples=sum(s['nonfoot_support']for s in samples),final=records[-1]['state'],actualCOMDriftM=drift,referenceSHA256=sha256(a.reference),sourceCodeSHA256=sha256(__file__),binarySHA256=sha256(a.binary),taskDataCollected=False,optimizerUpdates=0,stepQualified=False,actualEntryBinding=binding)
        atomic_json(a.output/'summary.json',report);print(json.dumps(report,indent=2))
    finally:n.close();ledger.close()
if __name__=='__main__':main()
