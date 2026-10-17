#!/usr/bin/env python3
"""One original-engine WEAK get-up diagnostic, with independently bounded physics.

Fresh static-model task RandomState(seed), exact dm_control1.0.1 randomizer,
80 zero-action40Hz decisions, then250 deterministic frozen WEAK decisions.
The RNG position is explicit; original main.py constructor-reset consumption is
NOT claimed to match. This is neither a Rapier task rollout nor step training.
"""
import argparse,ctypes as ct,datetime as dt,fcntl,json,math,signal,time
from pathlib import Path
import numpy as np
from actors import DenseActor
from native_env import ROOT,atomic_json,sha256
from source_engine import POWER,World,fingerprints,frozen_actor
from source_math import ORDER,teacher_observation,teacher_reward

SETTLE_DECISIONS=80
TEACHER_DECISIONS=250
MAXIMUM_STEPS=(SETTLE_DECISIONS+TEACHER_DECISIONS)*5


def rng_json(rng):
    name,keys,pos,has_gauss,cached=rng.get_state()
    return dict(name=name,keys=keys.tolist(),position=pos,hasGaussian=has_gauss,cachedGaussian=cached)


def joint_info(world):
    world.lib.lh_joint_info.argtypes=[ct.c_void_p,ct.c_int,ct.POINTER(ct.c_double)]
    world.lib.lh_joint_info.restype=ct.c_int
    rows=[];buf=np.zeros(5)
    for i in range(world.dim(5)):
        if world.lib.lh_joint_info(world.h,i,buf.ctypes.data_as(ct.POINTER(ct.c_double)))!=0:raise ValueError('joint info rejected')
        rows.append(dict(index=i,type=int(buf[0]),qposAddress=int(buf[1]),limited=bool(buf[2]),range=buf[3:].tolist()))
    if [r['type']for r in rows]!=[0]+[3]*21 or not all(r['limited']for r in rows[1:]):raise ValueError('unexpected source joint schema')
    return rows


def sample_pose(q0,joints,rng):
    q=np.asarray(q0,dtype=np.float64).copy()
    if q.shape!=(28,)or not np.isfinite(q).all():raise ValueError('invalid source qpos0')
    for joint in joints:
        addr=joint['qposAddress']
        if joint['type']==0:
            quat=rng.rand(4);q[addr+3:addr+7]=quat/np.linalg.norm(quat)
        elif joint['limited']:q[addr]=rng.uniform(*joint['range'])
        else:raise ValueError('source randomizer schema unsupported')
    return q


def reset_pose(world,seed):
    """No integration: source reset + collision-rejected exact randomizer."""
    if type(seed)is not int or not 0<=seed<2**32:raise ValueError('invalid RandomState seed')
    world.lib.lh_reset_data(world.h);forwards=1
    q0=world.arr(0,(28,)).copy();rng=np.random.RandomState(seed);joints=joint_info(world);attempts=[]
    for attempt in range(10000):
        q=sample_pose(q0,joints,rng);v=np.zeros(27)
        world.lib.lh_set_state(world.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),v.ctypes.data_as(ct.POINTER(ct.c_double)));forwards+=1
        attempts.append(dict(attempt=attempt,qpos=q.tolist(),contacts=world.dim(7)))
        if world.dim(7)==0:break
    else:raise ValueError('collision-free source reset exceeded10000 attempts')
    if world.dim(6)!=0 or world.lib.lh_time(world.h)!=0 or not np.array_equal(world.arr(1,(27,)),v):raise ValueError('reset integrated or changed velocity')
    return dict(seed=seed,rng='numpy.random.RandomState MT19937; fresh static-model stream, not original main.py constructor-stream position',rngAfter=rng_json(rng),jointInfo=joints,qpos=q.tolist(),qvel=v.tolist(),attempts=attempts,explicitStaticForwardCalls=forwards)


def inputs(base):
    result=fingerprints(base)
    for path in [Path(__file__),ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py']:
        result[str(path.resolve())]=sha256(path)
    result[str((base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3').resolve())]=sha256(base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3')
    return result


def prepare(base,seed):
    world=World(base);other=World(base)
    try:
        if any(world.lib.lh_foot_slip_speed(world.h,i)!=0 or other.lib.lh_foot_slip_speed(other.h,i)!=0 for i in range(2)):raise ValueError('fresh audit state is not per-world zero')
        reset=reset_pose(world,seed);physical=world.observation();infer,dense=frozen_actor();raw,action=infer(physical);pred=dense.infer(teacher_observation(physical,POWER))
        raw_error=float(np.max(abs(raw-pred['raw'])));action_error=float(np.max(abs(action-pred['action'])))
        before=world.arr(0,(28,)).copy();rejected=world.lib.lh_step_audited(world.h,0)==-1
        if not rejected or world.dim(6)!=0 or not np.array_equal(before,world.arr(0,(28,))):raise ValueError('zero-step guard failed')
        if raw_error>2e-4 or action_error>5e-5:raise ValueError('actor implementation parity failed')
        if any(world.lib.lh_foot_slip_speed(world.h,i)!=0 or other.lib.lh_foot_slip_speed(other.h,i)!=0 for i in range(2)):raise ValueError('reset audit lifecycle failed')
        report=dict(passed=True,physicsIntegrations=0,sourceEngineVersion=213,sourceDtSeconds=.005,policyDtSeconds=.025,sourcePower=POWER,sourceReset=reset,initialPhysicalObservation=physical.tolist(),initialRaw42=raw.tolist(),initialAction21=action.tolist(),maximumRaw42ParityError=raw_error,maximumAction21ParityError=action_error,initialSolvedLoadsAreUnavailable=True,zeroStepGuardRejectedWithoutMutation=True,sourceBridgeFreshAndResetAuditZero=True,maximumSourceSteps=MAXIMUM_STEPS,maximumSourceSimSeconds=MAXIMUM_STEPS*.005,settleDecisions=SETTLE_DECISIONS,teacherDecisions=TEACHER_DECISIONS,physicsCapSeconds=10.,inputHashes=inputs(base),qualification='source-native reset-algorithm positive control; no transferred native state, source-reset RNG-position equivalence, full-step or training claim')
        atomic_json(base/'positive-control-preflight.json',report)
        print(json.dumps({k:v for k,v in report.items()if k not in ['sourceReset','initialPhysicalObservation','initialRaw42','initialAction21','inputHashes']},indent=2))
    finally:world.close();other.close()


def validate_source_frame(row,max_torque):
    velocity=np.asarray(row['qvel'],dtype=np.float64)
    if velocity.shape!=(27,)or not np.isfinite(velocity).all():raise ValueError('invalid source velocity packet')
    if row['sourceWarnings']or not math.isfinite(max_torque)or max_torque>2e-4 or np.max(abs(velocity))>1e4:
        raise ValueError('source warning/torque/nonfinite safety rejected')


def utcnow():return dt.datetime.now(dt.timezone.utc)


def run(base,corrected_restart=False):
    lock=(base/'phase.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ledger=json.loads((base/'ledger.json').read_text());pre=json.loads((base/'positive-control-preflight.json').read_text())
    if not pre['passed']or pre['physicsIntegrations']!=0 or pre['maximumSourceSteps']!=MAXIMUM_STEPS:raise ValueError('preflight not admitted')
    for path,value in pre['inputHashes'].items():
        if sha256(path)!=value:raise ValueError('source input changed after preflight: '+path)
    deadline=dt.datetime.fromisoformat(ledger['deadlineUtc'].replace('Z','+00:00'))
    if utcnow()>=deadline or ledger.get('requestOutstanding'):raise ValueError('phase expired or unresolved')
    prior_physics=ledger['newPhysicsWallSeconds'];prior_steps=ledger['sourceMuJoCoSteps'];out=base
    if corrected_restart:
        failed=json.loads((base/'attempt-01-validation-abort/positive-control-summary.json').read_text())
        if ledger['status']!='closed-source-positive-control'or prior_steps!=1 or failed['sourceMuJoCoSteps']!=1 or failed['stopped']!="TypeError: bad operand type for abs(): 'list'":raise ValueError('not the explicitly admitted validation-bug restart')
        (base/'positive-control-corrected-started-once').open('x').close();out=base/'corrected-attempt';out.mkdir()
    else:
        if prior_physics!=0 or prior_steps!=0:raise ValueError('source allowance already used')
        (base/'positive-control-started-once').open('x').close()
    ledger.update(status='running-original-source-positive-control',maximumSourceSteps=prior_steps+MAXIMUM_STEPS,maximumSourceSimSeconds=(prior_steps+MAXIMUM_STEPS)*.005,correctedRestartAfterValidationBug=corrected_restart,originalSourceProtocol=dict(seed=pre['sourceReset']['seed'],settleDecisions=80,teacherDecisions=250,teacherPower=POWER,allIntegrationsCharged=True),requestOutstanding=False)
    atomic_json(base/'ledger.json',ledger)
    world=World(base);infer,dense=frozen_actor();records=[];commands=[];physics=prior_physics;stopped=None;max_raw=max_action=max_torque=0.;slip=np.zeros(2);started=time.monotonic();final=None
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());gear=np.zeros(21)
    for part in model['segments']:
        h=part['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    def flush(status):
        ledger.update(status=status,newPhysicsWallSeconds=physics,sourceMuJoCoSteps=prior_steps+world.dim(6),sourceSimSeconds=(prior_steps+world.dim(6))*.005,requestOutstanding=False)
        atomic_json(base/'ledger.json',ledger)
    def alarm(*_):raise TimeoutError('remaining source physics allocation elapsed')
    previous_handler=signal.signal(signal.SIGALRM,alarm)
    try:
        q=np.array(pre['sourceReset']['qpos']);v=np.array(pre['sourceReset']['qvel']);world.lib.lh_set_state(world.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),v.ctypes.data_as(ct.POINTER(ct.c_double)))
        if not np.array_equal(world.observation(),np.array(pre['initialPhysicalObservation']))or world.dim(7)!=0:raise ValueError('source initial reset restore parity failed')
        with (out/'positive-control-frames.ndjson').open('x')as stream:
            initial=world.metrics();stream.write(json.dumps(dict(stage='source-reset-before-settling',**initial),allow_nan=False)+'\n');stream.flush()
            for decision in range(SETTLE_DECISIONS+TEACHER_DECISIONS):
                if utcnow()>=deadline:stopped='phase-wall-deadline';break
                stage='zero-action-settling'if decision<SETTLE_DECISIONS else'frozen-WEAK-getup'
                physical=world.observation();control=np.zeros(21)
                if stage=='frozen-WEAK-getup':
                    raw,control=infer(physical);pred=dense.infer(teacher_observation(physical,POWER));max_raw=max(max_raw,float(np.max(abs(raw-pred['raw']))));max_action=max(max_action,float(np.max(abs(control-pred['action']))))
                    if max_raw>2e-4 or max_action>5e-5:stopped='actor-parity-rejected-before-next-integration';break
                    if np.max(abs(control))>POWER+1e-7:raise ValueError('power applied incorrectly')
                commands.append(dict(decision=decision,stage=stage,timeS=world.lib.lh_time(world.h),physicalObservation=physical.tolist(),controlsActuatorOrder=control.tolist()))
                world.arr(2,(21,))[:]=control
                for hold in range(5):
                    remaining_physics=ledger['newAggregatePhysicsLimitSeconds']-physics
                    remaining_wall=(deadline-utcnow()).total_seconds()-.05
                    remaining=min(remaining_physics,remaining_wall)
                    if remaining<=0:raise TimeoutError('source budget/deadline exhausted')
                    expected=world.dim(6)+1
                    # Crash leaves an unresolved marker; do not reopen/reclaim it.
                    # SIGALRM is cooperative across the synchronous C call; retain
                    # measured elapsed even if one native call overruns the alarm.
                    ledger.update(requestOutstanding=True,pendingSourceStepUpperBound=prior_steps+expected,pendingPhysicsReserveSeconds=remaining)
                    atomic_json(base/'ledger.json',ledger)
                    t=time.perf_counter();signal.setitimer(signal.ITIMER_REAL,remaining)
                    try:actual=world.lib.lh_step_audited(world.h,MAXIMUM_STEPS)
                    finally:
                        signal.setitimer(signal.ITIMER_REAL,0);physics+=time.perf_counter()-t
                    flush('running-original-source-positive-control')
                    if actual!=expected:raise ValueError('source counted-step cap/parity failed')
                    row=world.metrics();row.update(stage=stage,decision=decision,hold=hold,controlsActuatorOrder=control.tolist())
                    if abs(row['timeS']-expected*.005)>1e-9:raise ValueError('source elapsed clock mismatch')
                    expected_torque=gear*control[np.argsort(ORDER)]
                    max_torque=max(max_torque,float(np.max(abs(np.array(row['qfrcActuator'])[6:]-expected_torque))))
                    slip+=np.array(row['loadedSlipSpeedMps'])*.005;row['integratedLoadedSlipM']=slip.tolist();row['torqueEquationErrorNm']=max_torque
                    records.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();final=row
                    validate_source_frame(row,max_torque)
                row=records[-1];obs=np.array(row['physicalObservation']);positions=world.arr(3,(17,3));feet=positions[[world.ids['left_foot'],world.ids['right_foot']],:2];separation=float(np.linalg.norm(feet[0]-feet[1]))
                commands[-1]['postIntervalSourceReward']=teacher_reward(com_height=row['comSource'][2],torso_up=obs[36],head_height=obs[21],horizontal_velocity=obs[37:39],foot_separation=separation)
                commands[-1]['postIntervalHeadHeightM']=float(obs[21])
        stopped=stopped or'completed-source250decision-horizon'
    except BaseException as error:
        signal.setitimer(signal.ITIMER_REAL,0);stopped=f'{type(error).__name__}: {error}'
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,previous_handler)
        if not ledger.get('requestOutstanding'):flush('closed-source-positive-control')
        else:
            # The synchronous C invocation has returned if Python reaches here.
            flush('closed-source-positive-control-after-bounded-interruption')
        teacher=[c for c in commands if c['stage']=='frozen-WEAK-getup'and'postIntervalSourceReward'in c]
        teacher_rows=[r for r in records if r['stage']=='frozen-WEAK-getup']
        report=dict(scope=pre['qualification'],stopped=stopped,sourceResetSeed=pre['sourceReset']['seed'],resetStaticForwardCalls=1,sourceMuJoCoSteps=world.dim(6),sourceSimSeconds=world.lib.lh_time(world.h),physicsWallSeconds=physics-prior_physics,phaseAggregatePhysicsWallSeconds=physics,phaseAggregateSourceSteps=prior_steps+world.dim(6),correctedRestart=corrected_restart,processWallSeconds=time.monotonic()-started,settlingNativeSteps=sum(r['stage']=='zero-action-settling'for r in records),teacherNativeSteps=len(teacher_rows),teacherCompletedDecisions=len(teacher),sourceTeacherReturn=sum(c['postIntervalSourceReward']for c in teacher),maximumTeacherHeadHeightM=max([r['physicalObservation'][21]for r in teacher_rows],default=None),firstTeacherHeadAbove1p4TimeS=next((r['timeS']-2 for r in teacher_rows if r['physicalObservation'][21]>1.4),None),finalHeadHeightM=None if final is None else final['physicalObservation'][21],finalTorsoUp=None if final is None else final['physicalObservation'][36],maximumRaw42ParityError=max_raw,maximumAction21ParityError=max_action,maximumTorqueEquationErrorNm=max_torque,integratedLoadedSlipM=slip.tolist(),nonfootSupportSamples=sum(r['nonfootSupport']for r in teacher_rows),flightSamples=sum(sum(r['allBodyFloorLoadsN'])<3 for r in teacher_rows),bothFeetUnloadedSamples=sum(max(r['loadsN'])<3 for r in teacher_rows),final=final,inputHashes=pre['inputHashes'],taskStepQualified=False,optimizerUpdates=0,qualification='one source-engine get-up positive control only; original >60 curriculum requires10 test returns and is not certified by one trial; flight/nonfoot during get-up are reported, not quiet-stance failures')
        atomic_json(out/'positive-control-commands.json',commands);atomic_json(out/'positive-control-summary.json',report);world.close();fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
        print(json.dumps({k:v for k,v in report.items()if k not in['final','inputHashes']},indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--base',type=Path,required=True);p.add_argument('--seed',type=int,default=10);p.add_argument('--corrected-restart',action='store_true');a=p.parse_args()
    if a.mode=='prepare':prepare(a.base,a.seed)
    else:run(a.base,a.corrected_restart)
if __name__=='__main__':main()
