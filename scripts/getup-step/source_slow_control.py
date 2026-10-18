#!/usr/bin/env python3
"""Original800Hz SLOW positive control from the measured WEAK collector.

No task training. Exact source cubic/PD/sentinel behavior; declared240-decision
hold observation window. The static teacher cache is never copied across models.
"""
import argparse,ctypes as ct,datetime as dt,fcntl,hashlib,json,math,signal,time
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull,QhullError
from actors import DenseActor
from native_env import ROOT,atomic_json,sha256
from source_engine import World as TeacherWorld,old,torch_actor
from source_math import ORDER,slow_controls,student_observation,source_terminal
from reference import source_retime,UniformReference
from source_positive_control import validate_source_frame

HOLD_DECISIONS=240
DT=.00125


class SlowWorld(TeacherWorld):
    def __init__(self,base):
        old.World.__init__(self,base/'source-model/humanoid_high_freq.xml',base/'source_bridge_v2.so')
        for name,args,ret in [('lh_step_audited',[ct.c_void_p,ct.c_int],ct.c_int),('lh_foot_slip_speed',[ct.c_void_p,ct.c_int],ct.c_double),('lh_support_count',[ct.c_void_p],ct.c_int),('lh_body_velocity',[ct.c_void_p,ct.c_int,ct.POINTER(ct.c_double)],None),('lh_checkpoint_size',[ct.c_void_p],ct.c_int),('lh_checkpoint_export',[ct.c_void_p,ct.c_void_p,ct.c_int],ct.c_int),('lh_checkpoint_import',[ct.c_void_p,ct.c_void_p,ct.c_int],ct.c_int)]:
            f=getattr(self.lib,name);f.argtypes=args;f.restype=ret
        if abs(self.lib.lh_dt(self.h)-DT)>1e-12:raise ValueError('not original800Hz SLOW XML')
        self.checkpoint_binding=bytes.fromhex(sha256(base/'source-model/humanoid_high_freq.xml'))+bytes.fromhex(sha256(base/'source_bridge_v2.so'))+bytes.fromhex(sha256(base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    def install(self,q,v):
        q=np.asarray(q,dtype=float);v=np.asarray(v,dtype=float)
        if q.shape!=(28,)or v.shape!=(27,)or not np.isfinite(q).all()or not np.isfinite(v).all()or abs(np.linalg.norm(q[3:7])-1)>1e-6:raise ValueError('invalid real SLOW reset state')
        self.lib.lh_set_state(self.h,q.ctypes.data_as(ct.POINTER(ct.c_double)),v.ctypes.data_as(ct.POINTER(ct.c_double)))
    def checkpoint(self):
        size=self.lib.lh_checkpoint_size(self.h)
        if not 0<size<32*1024**2:raise ValueError('invalid source checkpoint size')
        out=ct.create_string_buffer(size)
        if self.lib.lh_checkpoint_export(self.h,out,size)!=size:raise ValueError('source checkpoint export failed')
        payload=out.raw
        return b'LHSC002!'+self.checkpoint_binding+hashlib.sha256(payload).digest()+payload
    def restore(self,blob):
        if len(blob)<136 or blob[:8]!=b'LHSC002!'or blob[8:104]!=self.checkpoint_binding or hashlib.sha256(blob[136:]).digest()!=blob[104:136]:raise ValueError('source checkpoint model/library/hash mismatch')
        payload=blob[136:];data=ct.create_string_buffer(payload,len(payload))
        if self.lib.lh_checkpoint_import(self.h,data,len(payload))!=len(payload):raise ValueError('source checkpoint restore rejected')


def reference_from_measured_teacher(base):
    rows=[json.loads(s)for s in(base/'corrected-attempt/positive-control-frames.ndjson').read_text().splitlines()]
    frames=[];end_next=False
    for r in rows:
        if r['stage']!='frozen-WEAK-getup'or r['hold']!=4:continue
        frames.append(r)
        if end_next:break
        if r['physicalObservation'][21]>1.4:end_next=True
    if not end_next or len(frames)!=40:raise ValueError('actual source collector did not match40-frame admission')
    trajectory=dict(qpos=np.array([r['qpos']for r in frames]),qvel=np.array([r['qvel']for r in frames]),com=np.array([r['comSource']for r in frames]),com_ori=np.array([r['physicalObservation'][34:37]for r in frames]))
    data=source_retime(trajectory,.5)
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());standing=np.array(model['referenceMetadata']['standingPose'])
    terminal=np.r_[standing,1.4,0.,0.,1.]
    ref=UniformReference(data,terminal,kind='source-fixture')
    if ref.length!=80:raise ValueError('SLOW length differs from source speed.5')
    return ref,standing,frames


class SourceQuiet:
    def __init__(self):self.run=0;self.maximum=0;self.anchors=None;self.feet_run=np.zeros(2,dtype=int)
    def observe(self,w,row,holding,final_window_active=True):
        physical=np.array(row['physicalObservation']);loads=np.array(row['allBodyFloorLoadsN']);feet=[w.ids['left_foot'],w.ids['right_foot']];mass=float(w.arr(10,(17,)).sum());com=np.array(row['comSource']);capture=com[:2]+physical[37:39]*math.sqrt(max(com[2],0)/9.81)
        count=w.lib.lh_support_count(w.h);points=w.arr(21,(128,))[:2*count].reshape(-1,2).copy();margin=None
        if len(points)>=3:
            try:
                hull=ConvexHull(np.unique(points,axis=0));eq=hull.equations;margin=float(min(np.min(-(eq[:,:2]@com[:2]+eq[:,2])),np.min(-(eq[:,:2]@capture+eq[:,2]))))
            except QhullError:pass
        velocities=[]
        for i in range(1,17):
            v=np.zeros(6);w.lib.lh_body_velocity(w.h,i,v.ctypes.data_as(ct.POINTER(ct.c_double)));velocities.append(v)
        velocities=np.array(velocities);linear=float(np.max(np.linalg.norm(velocities[:,3:],axis=1)));angular=float(np.max(np.linalg.norm(velocities[:,:3],axis=1)));positions=w.arr(3,(17,3))[feet].copy();self.feet_run=np.where(loads[feet]>=3,self.feet_run+1,0)
        conditions=dict(holding=bool(holding),declaredFinalWindow=bool(final_window_active),head=bool(physical[21]>1.4),upright5deg=bool(physical[36]>=math.cos(math.radians(5))),bothFeet=bool(np.all(self.feet_run>=40)),totalFootLoad=bool(sum(loads[feet])>=.8*mass*9.81),footOnly=bool(max(loads[i]for i in range(1,17)if i not in feet)<3),linearSpeed=linear<=.22,angularSpeed=angular<=.65,balance=margin is not None and margin>=.003,height=.780625<=com[2]<=.829976,selfPenetration=row['maximumSelfPenetrationM']<=.005)
        conditions={key:bool(value)for key,value in conditions.items()}
        if all(conditions.values())and self.anchors is None:self.anchors=positions.copy()
        drift=None if self.anchors is None else float(np.max(np.linalg.norm(positions-self.anchors,axis=1)));conditions['fixedAnchorDrift']=drift is not None and drift<=.01
        quiet=all(conditions.values());self.run=self.run+1 if quiet else 0;self.maximum=max(self.maximum,self.run)
        return dict(qualified=quiet,conditions=conditions,quietConsecutiveSamples=self.run,quietSeconds=self.run*DT,minimumComCaptureMarginM=margin,maximumBodyLinearMps=linear,maximumBodyAngularRadps=angular,anchorDriftM=drift,solvedPositiveForcePointsCurrentXY=points.tolist())


def inputs(base):
    files=[Path(__file__),ROOT/'scripts/getup-step/source_engine.py',ROOT/'scripts/getup-step/source_positive_control.py',ROOT/'scripts/getup-step/source_bridge.c',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/reference.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',base/'source_bridge_v2.so',base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',base/'source-model/humanoid_high_freq.xml',base/'corrected-attempt/positive-control-frames.ndjson',*sorted((base/'source-model').glob('*.xml'))]
    return{str(p.resolve()):sha256(p)for p in files}


def prepare(base,continuation=False):
    ref,standing,frames=reference_from_measured_teacher(base);w=SlowWorld(base);clone=SlowWorld(base);actor=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin')
    try:
        w.install(ref.data['qpos'][0],ref.data['qvel'][0])
        if continuation:
            prior=json.loads((base/'slow/final.controller.json').read_text());ledger=json.loads((base/'ledger.json').read_text())
            if prior['sourceStep']!=1 or ledger['source800HzSteps']!=1 or ledger.get('requestOutstanding'):raise ValueError('not the measured one-step serialization continuation')
            w.restore((base/'slow/final.bin').read_bytes())
            if w.dim(6)!=1 or abs(w.lib.lh_time(w.h)-DT)>1e-12:raise ValueError('restored SLOW first-step state mismatch')
        now,ahead=ref.now_ahead(0);obs=student_observation(w.observation(),now,ahead);pred=actor.infer(obs);raw,action=torch_actor(actor)(obs)
        if np.max(abs(raw-pred['raw']))>2e-4 or np.max(abs(action-pred['action']))>5e-5:raise ValueError('SLOW zero-step actor parity rejected')
        u=slow_controls(now[:21],pred['action'],w.arr(0,(28,))[7:],w.arr(1,(27,))[6:])
        packet=w.metrics();packet['sourceQuiet']=SourceQuiet().observe(w,packet,False,False);json.dumps(packet,allow_nan=False)
        state=w.checkpoint();clone.restore(state)
        if clone.metrics()!=w.metrics()or not np.array_equal(clone.observation(),w.observation())or clone.checkpoint()!=state:raise ValueError('full source zero-step cache restore mismatch')
        for invalid in [state[:-1],bytes([state[0]^1])+state[1:]]:
            try:clone.restore(invalid)
            except ValueError:pass
            else:raise ValueError('invalid source checkpoint accepted')
        if clone.checkpoint()!=state:raise ValueError('invalid checkpoint rejection mutated state')
        if clone.lib.lh_step_audited(clone.h,0)!=-1 or clone.dim(6)!=w.dim(6):raise ValueError('SLOW zero-budget guard failed')
        report=dict(passed=True,physicsIntegrations=0,continuation=continuation,loadedSourceStep=w.dim(6),sourceTeacherCollectedFrames=len(frames),collectorLastTeacherTimeS=frames[-1]['timeS']-2,retimingSpeed=.5,movingDecisions=ref.length,holdDecisions=HOLD_DECISIONS,settlingHoldDecisions=HOLD_DECISIONS-200,declaredFinalQuietDecisions=200,maximumSourceSteps=(ref.length+HOLD_DECISIONS)*20,maximumSourceSimSeconds=(ref.length+HOLD_DECISIONS)*.025,modelDtS=DT,sourceTerminalObservationHeight=1.4,studentPower=1.,dampingMovingSeconds=.1,dampingHoldingSeconds=.2,initialReferenceIsActualTeacherPostActionFrame=True,teacherCacheCopiedAcrossModels=False,checkpointBytes=len(state),fullCacheZeroStepRestoreEqual=True,continuedEvolutionParityQualified=False,initialPhysicalObservation=w.observation().tolist(),initialRaw42=pred['raw'].tolist(),initialResidual=pred['action'].tolist(),initialControlsActuatorOrder=u.tolist(),inputHashes=inputs(base),qualification='original-source SLOW positive control from actual WEAK get-up reference, not task teacher/step')
        atomic_json(base/('slow-continuation-preflight.json'if continuation else'slow-preflight.json'),report);np.savez(base/'slow-source-reference.npz',**ref.data,terminal=ref.terminal);print(json.dumps({k:v for k,v in report.items()if k not in ['inputHashes','initialPhysicalObservation','initialRaw42','initialResidual','initialControlsActuatorOrder']},indent=2))
    finally:w.close();clone.close()


def run(base,continuation=False):
    pre=json.loads((base/('slow-continuation-preflight.json'if continuation else'slow-preflight.json')).read_text());ledger=json.loads((base/'ledger.json').read_text());lock=(base/'phase.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    for path,value in pre['inputHashes'].items():
        if sha256(path)!=value:raise ValueError('SLOW input changed after preflight: '+path)
    if not pre['passed']or pre['physicsIntegrations']!=0 or ledger.get('requestOutstanding')or ledger['status']!=('closed-source-SLOW-positive-control'if continuation else'closed-source-positive-control'):raise ValueError('SLOW comparison admission failed')
    deadline=dt.datetime.fromisoformat(ledger['deadlineUtc'].replace('Z','+00:00'))
    if dt.datetime.now(dt.timezone.utc)>=deadline:raise ValueError('phase expired')
    (base/('source-slow-continuation-started-once'if continuation else'source-slow-started-once')).open('x').close();out=base/('slow-continuation'if continuation else'slow');out.mkdir()
    restored_steps=1 if continuation else 0
    prior_steps=ledger['sourceMuJoCoSteps']-restored_steps;prior_physics=ledger['newPhysicsWallSeconds'];prior_sim=ledger['sourceSimSeconds']-restored_steps*DT;physics=prior_physics
    ref,standing,teacher_frames=reference_from_measured_teacher(base);maximum=(ref.length+HOLD_DECISIONS)*20;w=SlowWorld(base);quiet=SourceQuiet();actor=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin');infer=torch_actor(actor)
    gear=np.zeros(21);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    for part in model['segments']:
        hinge=part['reference'].get('hinge')
        if hinge:gear[hinge['qposIndex']]=hinge['gear']
    ledger.update(status='running-source-SLOW-positive-control',maximumSourceSteps=prior_steps+maximum,maximumAdditionalSource800HzSteps=maximum,source200HzSteps=prior_steps,source800HzSteps=restored_steps,sourceSLOWProtocol=dict(movingDecisions=ref.length,holdDecisions=HOLD_DECISIONS,sourceTerminalTargetHeight=1.4,sourceDtSeconds=DT,modelCacheTransfer=False));atomic_json(base/'ledger.json',ledger)
    records=[];commands=[];stopped=None;max_raw=max_action=max_torque=0.;started=time.monotonic();last=None;last_standing=None
    def flush(status):
        ledger.update(status=status,newPhysicsWallSeconds=physics,sourceMuJoCoSteps=prior_steps+w.dim(6),source800HzSteps=w.dim(6),sourceSimSeconds=prior_sim+w.lib.lh_time(w.h),requestOutstanding=False);atomic_json(base/'ledger.json',ledger)
    def alarm(*_):raise TimeoutError('remaining SLOW physics/deadline bound')
    handler=signal.signal(signal.SIGALRM,alarm)
    try:
        w.install(ref.data['qpos'][0],ref.data['qvel'][0])
        if continuation:
            w.restore((base/'slow/final.bin').read_bytes());saved=json.loads((base/'slow/final.controller.json').read_text());qs=saved['quietState'];quiet.run=qs['run'];quiet.maximum=qs['maximum'];quiet.anchors=None if qs['anchors']is None else np.array(qs['anchors']);quiet.feet_run=np.array(qs['feetRun'],dtype=int);commands=json.loads((base/'slow/commands.json').read_text())
            if w.dim(6)!=1 or len(commands)!=1:raise ValueError('partial command continuation mismatch')
        if not np.array_equal(w.observation(),np.array(pre['initialPhysicalObservation'])):raise ValueError('SLOW initial observation changed')
        packet=w.metrics();packet['sourceQuiet']=SourceQuiet().observe(w,packet,False,False);json.dumps(packet,allow_nan=False)
        (out/'entry.bin').write_bytes(w.checkpoint())
        with(out/'frames.ndjson').open('x')as stream:
            if continuation:
                initial=json.loads((base/'slow/frames.ndjson').read_text().splitlines()[0]);stream.write(json.dumps(initial,allow_nan=False)+'\n')
                recovered=w.metrics();recovered.update(decision=0,hold=0,holding=False,controlsActuatorOrder=w.arr(2,(21,)).copy().tolist(),recoveredFromExactCheckpoint=True);temporary=SourceQuiet();recovered['sourceQuiet']=temporary.observe(w,recovered,False,False);recovered['torqueEquationErrorNm']=0.;records.append(recovered);stream.write(json.dumps(recovered,allow_nan=False)+'\n')
                if temporary.run!=quiet.run or not np.array_equal(temporary.feet_run,quiet.feet_run):raise ValueError('recovered first observer state differs')
            else:stream.write(json.dumps(dict(stage='SLOW-reset',**w.metrics()),allow_nan=False)+'\n')
            for decision in range(restored_steps//20,ref.length+HOLD_DECISIONS):
                now,ahead=ref.now_ahead(decision)
                if continuation and decision==0:
                    obs=np.array(commands[0]['sourceObservation118'],dtype=np.float32);raw,residual=infer(obs)
                    if not np.array_equal(residual,np.array(commands[0]['residualQposOrder'],dtype=np.float32)):raise ValueError('held partial-command residual changed')
                else:obs=student_observation(w.observation(),now,ahead);raw,residual=infer(obs)
                pred=actor.infer(obs);max_raw=max(max_raw,float(np.max(abs(raw-pred['raw']))));max_action=max(max_action,float(np.max(abs(residual-pred['action']))))
                if max_raw>2e-4 or max_action>5e-5:raise ValueError('SLOW inference parity failed before command')
                holding=decision>=ref.length;reference=standing if holding else ref.data['qpos'][decision,7:]
                if not(continuation and decision==0):commands.append(dict(decision=decision,holding=holding,sourceObservation118=obs.tolist(),residualQposOrder=residual.tolist(),referenceQposOrder=reference.tolist()))
                for hold in range(restored_steps%20 if continuation and decision==0 else 0,20):
                    remaining=min(ledger['newAggregatePhysicsLimitSeconds']-physics,(deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()-.05)
                    if remaining<=0:raise TimeoutError('source phase budget exhausted')
                    control=slow_controls(reference,residual,w.arr(0,(28,))[7:],w.arr(1,(27,))[6:],holding);w.arr(2,(21,))[:]=control;expected=w.dim(6)+1
                    ledger.update(requestOutstanding=True,pendingSourceStepUpperBound=prior_steps+expected,pendingPhysicsReserveSeconds=remaining);atomic_json(base/'ledger.json',ledger)
                    t=time.perf_counter();signal.setitimer(signal.ITIMER_REAL,remaining)
                    try:actual=w.lib.lh_step_audited(w.h,maximum)
                    finally:signal.setitimer(signal.ITIMER_REAL,0);physics+=time.perf_counter()-t
                    flush('running-source-SLOW-positive-control')
                    if actual!=expected:raise ValueError('SLOW counted step failed')
                    row=w.metrics();row.update(decision=decision,hold=hold,holding=holding,controlsActuatorOrder=control.tolist())
                    if abs(row['timeS']-expected*DT)>1e-9:raise ValueError('SLOW clock mismatch')
                    expected_torque=gear*control[np.argsort(ORDER)];max_torque=max(max_torque,float(np.max(abs(np.array(row['qfrcActuator'])[6:]-expected_torque))));validate_source_frame(row,max_torque)
                    standing_metrics=quiet.observe(w,row,holding,decision>=ref.length+HOLD_DECISIONS-200);row['sourceQuiet']=standing_metrics;row['torqueEquationErrorNm']=max_torque;last=row;last_standing=standing_metrics;records.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
                term=source_terminal(decision+1,ref.length,last['comSource'][2],ref.data['com'][min(decision,ref.length-1),2])
                # Source physical terminal remains; its N+150 administrative
                # horizon is explicitly replaced by this bounded N+240 window.
                if term['terminated']:stopped='source-physical-height-terminal';break
        stopped=stopped or'completed80moving-plus240hold'
    except BaseException as error:
        signal.setitimer(signal.ITIMER_REAL,0);stopped=f'{type(error).__name__}: {error}'
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,handler);flush('closed-source-SLOW-positive-control')
        blob=w.checkpoint();(out/'final.bin').write_bytes(blob)
        controller=dict(schema=1,kind='deterministic source SLOW comparison only',nextDecision=w.dim(6)//20,nextHold=w.dim(6)%20,movingDecisions=ref.length,holdDecisions=HOLD_DECISIONS,sourceStep=w.dim(6),sourceTimeS=w.lib.lh_time(w.h),quietState=dict(run=quiet.run,maximum=quiet.maximum,anchors=None if quiet.anchors is None else quiet.anchors.tolist(),feetRun=quiet.feet_run.tolist()),lastCommand=None if not commands else commands[-1],modelSHA256=sha256(base/'source-model/humanoid_high_freq.xml'),bridgeSHA256=sha256(base/'source_bridge_v2.so'),actorSHA256=sha256(ROOT/'rust/crates/sim/data/getup-policy/student.bin'),checkpointSHA256=sha256(out/'final.bin'),noBudgetRefund=True,continuedEvolutionParityQualified=False)
        atomic_json(out/'final.controller.json',controller)
        report=dict(stopped=stopped,exactPartialCommandContinuation=continuation,previouslyChargedSLOWSteps=restored_steps,source800HzSteps=w.dim(6),sourceSimSeconds=w.lib.lh_time(w.h),physicsWallSeconds=physics-prior_physics,phaseAggregatePhysicsWallSeconds=physics,phaseAggregateAllSourceSteps=prior_steps+w.dim(6),processWallSeconds=time.monotonic()-started,movingDecisions=ref.length,declaredHoldDecisions=HOLD_DECISIONS,observedPolicyCommands=len(commands),maximumRaw42ParityError=max_raw,maximumAction21ParityError=max_action,maximumTorqueEquationErrorNm=max_torque,finalHeadHeightM=None if last is None else last['physicalObservation'][21],finalTorsoTiltDeg=None if last is None else math.degrees(math.acos(np.clip(last['physicalObservation'][36],-1,1))),finalQuietConsecutiveSamples=quiet.run,maximumQuietConsecutiveSamples=quiet.maximum,final4000ConsecutiveQuietQualified=quiet.run>=4000,finalStanding=last_standing,fullCacheCheckpointBytes=len(blob),fullCacheCheckpointSHA256=sha256(out/'final.bin'),continuedEvolutionParityQualified=False,sourceModelCacheTransfer=False,stepQualified=False,optimizerUpdates=0,scope='original SLOW source reference/cubic/PD with explicit240-decision hold diagnostic; no task training or Rapier result',inputHashes=pre['inputHashes'])
        atomic_json(out/'commands.json',commands);atomic_json(out/'summary.json',report);w.close();fcntl.flock(lock,fcntl.LOCK_UN);lock.close();print(json.dumps({k:v for k,v in report.items()if k!='inputHashes'},indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run','prepare-continuation','continue']);p.add_argument('--base',type=Path,required=True);a=p.parse_args()
    if a.mode.startswith('prepare'):prepare(a.base,a.mode=='prepare-continuation')
    else:run(a.base,a.mode=='continue')
if __name__=='__main__':main()
