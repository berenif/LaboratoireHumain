#!/usr/bin/env python3
"""Bounded frozen torque-teacher initialization on an actual SLOW source endpoint.

A fresh200Hz world receives measured q/qvel from the800Hz source. Geometry is
matched at zero steps; contact/solver caches are explicitly not transferred.
"""
import argparse,ctypes as ct,datetime as dt,fcntl,json,signal,time
from pathlib import Path
import numpy as np
from actors import DenseActor
from source_math import teacher_observation,ORDER
from source_engine import World,torch_actor
from source_slow_control import SlowWorld
from source_positive_control import validate_source_frame
from native_env import ROOT,sha256,atomic_json


def input_hashes(base):
    paths=[Path(__file__),ROOT/'scripts/getup-step/source_engine.py',ROOT/'scripts/getup-step/source_slow_control.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'scripts/getup-step/source_positive_control.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin',base/'source_bridge.so',base/'source_bridge_v2.so',base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',base/'slow-continuation/final.bin',base/'slow-continuation/final.controller.json',*sorted((base/'source-model').glob('*.xml'))]
    return{str(p.resolve()):sha256(p)for p in paths}


def prepare(base,powers):
    source=SlowWorld(base);teacher=World(base);actor=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin');infer=torch_actor(actor)
    try:
        source.restore((base/'slow-continuation/final.bin').read_bytes());q=source.arr(0,(28,)).copy();v=source.arr(1,(27,)).copy();teacher.lib.lh_set_state(teacher.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),v.ctypes.data_as(ct.POINTER(ct.c_double)))
        static=(base/'source-model/humanoid_static.xml').read_bytes();high=(base/'source-model/humanoid_high_freq.xml').read_bytes()
        if high.replace(b'timestep=".00125"',b'timestep=".005"')!=static:raise ValueError('models differ beyond source timestep')
        position_error=float(np.max(abs(source.arr(3,(17,3))-teacher.arr(3,(17,3)))));rotation_error=float(np.max(abs(source.arr(5,(17,3,3))-teacher.arr(5,(17,3,3)))));observation_error=float(np.max(abs(source.observation()-teacher.observation())))
        if max(position_error,rotation_error,observation_error)>1e-10:raise ValueError('source stance q/qvel/FK parity failed')
        predictions=[]
        for power in powers:
            obs=teacher_observation(teacher.observation(),power);raw,action=infer(obs);pred=actor.infer(obs)
            if np.max(abs(raw-pred['raw']))>2e-4 or np.max(abs(action-pred['action']))>5e-5:raise ValueError('teacher inference parity failed')
            predictions.append(dict(power=power,raw42=raw.tolist(),controlsActuatorOrder=action.tolist()))
        pre=dict(passed=True,physicsIntegrations=0,sourceSLOWStep=source.dim(6),sourceSLOWTimeS=source.lib.lh_time(source.h),sourceInitialStrictQuietQualified=False,sourceDtSeconds=.005,priorDtSeconds=.00125,powers=powers,maximumSourceStepsPerCase=50,maximumSimSecondsPerCase=.25,maximumAllCasesSourceSteps=50*len(powers),maximumAllCasesSimSeconds=.25*len(powers),allCasesPhysicsCapSeconds=1.,positionErrorM=position_error,rotationMatrixError=rotation_error,physical67Error=observation_error,qpos=q.tolist(),qvel=v.tolist(),initialPhysical67=teacher.observation().tolist(),predictions=predictions,cacheTransfer=False,powerOneAdmission='only if WEAK completes all50sourceframes without any predeclared stop and shared1sphysicscap remains',scope='training-only frozen initializer calibration, not quiet or step qualification; power1 is not a separate strong-trained checkpoint',inputHashes=input_hashes(base),stop='any bothfeet-unloaded/nonfoot/selfpenetration>5mm/heightoutsideoriginalenvelope/loadedslip>10mm/COMdrift>20mm/nonfinite/warning/parity; checked every200Hz integration')
        atomic_json(base/'teacher-stance-preflight.json',pre);print(json.dumps({k:v for k,v in pre.items()if k not in['inputHashes','qpos','qvel','initialPhysical67','predictions']},indent=2))
    finally:source.close();teacher.close()


def run(base):
    lock=(base/'phase.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);pre=json.loads((base/'teacher-stance-preflight.json').read_text());ledger=json.loads((base/'ledger.json').read_text())
    for path,value in pre['inputHashes'].items():
        if sha256(path)!=value:raise ValueError('stance input changed after preflight')
    if not pre['passed']or ledger.get('requestOutstanding')or ledger['status']!='closed-source-SLOW-positive-control':raise ValueError('stance comparison not admitted')
    (base/'source-teacher-stance-started-once').open('x').close();out=base/'teacher-stance';out.mkdir();deadline=dt.datetime.fromisoformat(ledger['deadlineUtc'].replace('Z','+00:00'));prior_steps=ledger['sourceMuJoCoSteps'];prior_physics=ledger['newPhysicsWallSeconds'];prior_sim=ledger['sourceSimSeconds'];physics=prior_physics;completed_steps=0;cases=[];actor=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin');infer=torch_actor(actor)
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());gear=np.zeros(21)
    for part in model['segments']:
        h=part['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    def alarm(*_):raise TimeoutError('stance calibration resource bound')
    old_handler=signal.signal(signal.SIGALRM,alarm)
    try:
        for power in pre['powers']:
            w=World(base);rows=[];commands=[];stopped=None;slip=np.zeros(2);max_raw=max_action=max_torque=0.
            try:
                q=np.array(pre['qpos']);v=np.array(pre['qvel']);w.lib.lh_set_state(w.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),v.ctypes.data_as(ct.POINTER(ct.c_double)))
                if not np.array_equal(w.observation(),np.array(pre['initialPhysical67'])):raise ValueError('stance initialization parity failed')
                com0=w.arr(6,(17,3))[w.ids['torso']].copy()
                with(out/f'power-{power}-frames.ndjson').open('x')as stream:
                    for decision in range(10):
                        obs=teacher_observation(w.observation(),power);raw,u=infer(obs);pred=actor.infer(obs);max_raw=max(max_raw,float(np.max(abs(raw-pred['raw']))));max_action=max(max_action,float(np.max(abs(u-pred['action']))))
                        if max_raw>2e-4 or max_action>5e-5:raise ValueError('stance inference parity rejected')
                        commands.append(dict(decision=decision,power=power,physical67=w.observation().tolist(),controlsActuatorOrder=u.tolist()));w.arr(2,(21,))[:]=u
                        for hold in range(5):
                            remaining=min(1.-(physics-prior_physics),ledger['newAggregatePhysicsLimitSeconds']-physics,(deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()-.05)
                            if remaining<=0:raise TimeoutError('stance calibration budget exhausted')
                            expected=w.dim(6)+1;ledger.update(status='running-source-teacher-stance',requestOutstanding=True,pendingSourceStepUpperBound=prior_steps+completed_steps+expected,pendingPhysicsReserveSeconds=remaining);atomic_json(base/'ledger.json',ledger)
                            t=time.perf_counter();signal.setitimer(signal.ITIMER_REAL,remaining)
                            try:actual=w.lib.lh_step_audited(w.h,50)
                            finally:signal.setitimer(signal.ITIMER_REAL,0);physics+=time.perf_counter()-t
                            ledger.update(requestOutstanding=False,newPhysicsWallSeconds=physics,sourceMuJoCoSteps=prior_steps+completed_steps+w.dim(6),sourceSimSeconds=prior_sim+(completed_steps+w.dim(6))*.005);atomic_json(base/'ledger.json',ledger)
                            if actual!=expected:raise ValueError('stance counted step failed')
                            r=w.metrics();r.update(decision=decision,hold=hold,power=power,controlsActuatorOrder=u.tolist());slip+=np.array(r['loadedSlipSpeedMps'])*.005;r['loadedSlipM']=slip.tolist();com=np.array(r['comSource']);r['COMDriftM']=float(np.linalg.norm((com-com0)[:2]));max_torque=max(max_torque,float(np.max(abs(np.array(r['qfrcActuator'])[6:]-gear*u[np.argsort(ORDER)]))));validate_source_frame(r,max_torque);rows.append(r);stream.write(json.dumps(r,allow_nan=False)+'\n');stream.flush()
                            if max(r['loadsN'])<3:stopped='bothfeet-unloaded'
                            elif r['nonfootSupport']:stopped='nonfoot-support'
                            elif r['maximumSelfPenetrationM']>.005:stopped='self-penetration-over5mm'
                            elif not .780625<=r['comSource'][2]<=.829976:stopped='height-envelope-exit'
                            elif max(slip)>.01:stopped='loaded-slip-over10mm'
                            elif r['COMDriftM']>.02:stopped='COM-drift-over20mm'
                            if stopped:break
                        if stopped:break
                stopped=stopped or'completed-bounded50sourceframes'
            except BaseException as error:stopped=f'{type(error).__name__}: {error}'
            finally:
                signal.setitimer(signal.ITIMER_REAL,0);completed_steps+=w.dim(6);case=dict(power=power,stopped=stopped,sourceSteps=w.dim(6),sourceSimSeconds=w.lib.lh_time(w.h),loadedSlipM=slip.tolist(),maxRaw42Error=max_raw,maxAction21Error=max_action,maxTorqueEquationErrorNm=max_torque,final=None if not rows else rows[-1],sourceStrictQuietQualified=False,stepQualified=False);cases.append(case);atomic_json(out/f'power-{power}-commands.json',commands);atomic_json(out/f'power-{power}-summary.json',case);w.close()
            if stopped!='completed-bounded50sourceframes':break
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,old_handler);ledger.update(status='closed-source-teacher-stance',requestOutstanding=False,newPhysicsWallSeconds=physics,sourceMuJoCoSteps=prior_steps+completed_steps,sourceSimSeconds=prior_sim+completed_steps*.005,sourceTeacherStanceSteps=completed_steps);atomic_json(base/'ledger.json',ledger);report=dict(scope=pre['scope'],physicsWallSeconds=physics-prior_physics,phaseAggregatePhysicsWallSeconds=physics,newSourceSteps=completed_steps,phaseAggregateSourceSteps=prior_steps+completed_steps,cases=cases,optimizerUpdates=0,taskDataCollected=False,inputHashes=pre['inputHashes']);atomic_json(out/'summary.json',report);fcntl.flock(lock,fcntl.LOCK_UN);lock.close();print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','cases']},indent=2));print(json.dumps([{k:v for k,v in c.items()if k!='final'}for c in cases],indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--base',type=Path,required=True);p.add_argument('--include-power-one',action='store_true');a=p.parse_args()
    if a.mode=='prepare':prepare(a.base,[.663420431,1.]if a.include_power_one else[.663420431])
    else:run(a.base)
if __name__=='__main__':main()
