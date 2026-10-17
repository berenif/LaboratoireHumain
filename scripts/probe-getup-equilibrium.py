#!/usr/bin/env python3
"""Explicit model-equilibrium + frozen GetUp local-feedback positive control.

All contact forces/limit reactions are offline inverse-statics calculations.
The physical process receives only its ordinary 21 bounded motor residuals.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('probe',ROOT/'scripts/probe-getup-recovery-step.py')
probe = importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


class FrozenActor:
    def __init__(self):
        b = (ROOT/'rust/crates/sim/data/getup-policy/student.bin').read_bytes()
        assert b[:8] == b'LHGPW001'
        dims = struct.unpack('<4I',b[8:24]);assert dims == (118,1024,1024,42)
        v = np.frombuffer(b[24:],dtype='<f4');offset = 0;self.layers = []
        for a,z in zip(dims,dims[1:]):
            w = v[offset:offset+a*z].reshape(z,a);offset += a*z
            bias = v[offset:offset+z];offset += z;self.layers.append((w,bias))
        assert offset == len(v)
    def infer(self,observation):
        x = np.asarray(observation,dtype=np.float32)
        for i,(w,b) in enumerate(self.layers):
            x = w@x+b
            if i<2:x = np.maximum(x,0)
        return np.tanh(x[:21])


def synthetic_physical(kin,q,root):
    poses = kin.forward(q,root);rot = root[:3,:3]/np.cbrt(np.linalg.det(root[:3,:3]))
    physical = np.zeros(67);physical[:21] = q;physical[21] = poses['head'][1,3]
    for i,f in enumerate(['left_hand','left_foot','right_hand','right_foot']):
        physical[22+3*i:25+3*i] = (rot.T@(poses[f][:3,3]-root[:3,3]))[[2,0,1]]
    physical[34:37] = (rot.T@np.array([0.,1.,0.]))[[2,0,1]]
    return physical


def transfer_schedule(elapsed,weights,qs):
    """Separate controller handoff from slow joint-arc path progression."""
    arc = np.r_[0.,np.cumsum(np.max(abs(np.diff(qs,axis=0)),axis=1))]
    if elapsed<1.:return 0.,probe.smooth(elapsed)
    if elapsed<4.:progress = probe.smooth((elapsed-1.)/3.)
    elif elapsed<6.:progress = 1.
    elif elapsed<9.:progress = 1.-probe.smooth((elapsed-6.)/3.)
    else:return 0.,1.-probe.smooth(elapsed-9.) if elapsed<10. else 0.
    return float(np.interp(progress*arc[-1],arc,weights)),1.


class GuardedTransfer:
    """Pause planned progress when actual capture/load have not followed."""
    def __init__(self,weights,qs,rows):
        self.weights = weights;self.arc = np.r_[0.,np.cumsum(np.max(abs(np.diff(qs,axis=0)),axis=1))]
        self.coms = np.array([r['com'] for r in rows]);self.normals = np.array([r['normalForcesN'] for r in rows])
        self.clock = 0.;self.blocked = 0.;self.return_start = None;self.return_progress = 0.;self.abort = None;self.log = []
    def schedule(self,elapsed,diagnostic,metrics,last_samples):
        if self.abort is not None:return self.abort[1],self.abort[2]*max(0.,1.-probe.smooth((elapsed-self.abort[0])/.05))
        if elapsed<1.:return 0.,probe.smooth(elapsed)
        progress = probe.smooth(self.clock/3.)
        weight = float(np.interp(progress*self.arc[-1],self.arc,self.weights))
        if self.return_start is None:
            target = np.array([np.interp(weight,self.weights,self.coms[:,i]) for i in range(3)])
            com_error = np.linalg.norm((probe.reference.xyz(diagnostic['com'])-target)[[0,2]])
            capture_error = np.linalg.norm((probe.reference.xyz(diagnostic['capture'])-target)[[0,2]])
            normals = np.array([np.interp(weight,self.weights,self.normals[:,i]) for i in range(self.normals.shape[1])])
            share = metrics['foot_loads_n'][1]/max(1e-9,sum(metrics['foot_loads_n']))
            planned_share = sum(normals[1:])/sum(normals)
            safe = com_error<=.015 and capture_error<=.02 and share>=planned_share-.06
            self.blocked = 0. if safe else self.blocked+.025
            if safe:self.clock = min(3.,self.clock+.025)
            if self.blocked>=1. or elapsed>=4.:
                self.return_start = elapsed;self.return_progress = progress
                self.log.append({'event':'return','timeS':elapsed,'weight':weight,'blockedS':self.blocked,'comErrorM':float(com_error),'captureErrorM':float(capture_error)})
            return weight,1.
        t = elapsed-self.return_start
        progress = self.return_progress*(1.-probe.smooth(t/3.))
        weight = float(np.interp(progress*self.arc[-1],self.arc,self.weights))
        return weight,1. if t<3. else max(0.,1.-probe.smooth(t-3.))
    def inspect_native(self,elapsed,weight,blend,last_samples):
        if self.abort is None and blend>0 and any(all(v<3 for v in s['loads_n']) for s in last_samples):
            self.abort = (elapsed,weight,blend);self.log.append({'event':'native-flight-model-release','timeS':elapsed})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['binary','checkpoint','path','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--mode',choices=['zero','transfer','guarded-transfer'],required=True)
    p.add_argument('--physics-budget',type=float,required=True);p.add_argument('--deadline',required=True)
    p.add_argument('--execute',action='store_true');a = p.parse_args()
    deadline = datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp()
    assert time.time()<deadline<=datetime.datetime(2026,10,17,13,6,tzinfo=datetime.timezone.utc).timestamp()
    assert 0<=a.physics_budget<=90
    if a.mode!='zero':
        schedule_proof = json.loads((a.path.parent/'equilibrium-schedule-validation.json').read_text())
        assert schedule_proof['scriptSha256']==hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        assert schedule_proof['pathSha256']==hashlib.sha256(a.path.read_bytes()).hexdigest()
        assert schedule_proof['maximumFootOriginErrorM']<.001
        assert schedule_proof['maximumRequiredMotorMismatchNm']<.05
        assert schedule_proof['maximumRootWrenchMomentResidualNm']<.025
        assert schedule_proof['maximumReferenceJointSpeedRadPerS']<.23
        assert schedule_proof['minimumNormalN']>=-1e-5
    proof = json.loads(a.path.read_text());assert proof['complete'] and len(proof['rows'])==41
    assert proof['maximumEqualityError']<1e-4 and proof['minimumInequalityMargin']>-.0001
    rows = proof['rows'];weights = np.array([r['weight'] for r in rows]);qs = np.array([r['q'] for r in rows])
    if a.mode=='guarded-transfer':assert [c['body'] for c in proof['contacts']]==['left_foot','right_foot','right_foot']
    roots = np.array([r['root'] for r in rows]);taus = np.array([r['torquesNm'] for r in rows]);leans = np.array([r['rootLeanRad'] for r in rows])
    kin = probe.reference.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))
    for r in rows:
        assert min(r['normalForcesN'])>=-1e-5 and max(abs(v) for v in r['rootMomentResidualNm'])<1e-4
        assert r['maximumMotorFraction']<=.9001 and r['maximumNominalResidual']<=.9001
        for i,v in enumerate(r['limitReactionsNm']):
            if abs(v)>1e-6:assert v>=0 and abs(r['q'][i]-kin.limits[i,0])<1e-5
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close()
    actor = FrozenActor();standing = np.r_[kin.nominal,1.4,0,0,1]
    config = {'mode':a.mode,'controller':'model-equilibrium-plus-recentered-frozen-SLOW-feedback-untrained',
        'formula':'source(actual)+blend*(qref-qnom+motorFF/gear-source(zero-velocity-FKref))',
        'physicsCapSeconds':a.physics_budget,'deadlineUTC':a.deadline,'simulatedSeconds':8 if a.mode=='zero' else 16,
        'contactAndLimitForcesAppliedToPhysics':False,'onlyOrdinaryBoundedMotorResiduals':True,
        'checkpointSha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),'pathSha256':hashlib.sha256(a.path.read_bytes()).hexdigest(),
        'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'nativeBinarySha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
        'pathProofEqualityMaxError':proof['maximumEqualityError'],'pathProofInequalityMinMargin':proof['minimumInequalityMargin'],
        'qualification':False,'zeroHoldWindowS':[1.,6.], 'zeroHoldAcceptance':{'continuousAnchoredStandingS':5.,'flightSamples':0,'nonfootSupport':False,'maximumLoadedSlipIntegralM':.01,'maximumHorizontalCOMDriftM':.02},
        'transferTimelineS':{'fullModelHandoff':1.,'progressionEnd':4.,'goalHoldEnd':6.,'returnEnd':9.,'sourceHandoff':10.,'end':16.},
        'maximumReferenceJointSpeedBoundRadPerS':float(np.sum(np.max(abs(np.diff(qs,axis=0)),axis=1))*.5),
        'guardedProgress':{'comErrorM':.015,'captureErrorM':.02,'maximumRightLoadShareLag':.06,'maximumContinuousPauseS':1.},
        'acceptance':'Zero shift: full-model hold 1-6s must retain >=5s native anchored standing, zero flight/nonfoot/failure, each loaded-slip <=1cm, horizontal COM drift <=2cm. Transfer: actual capture/load shift, no support or integrity failure, return >=5s anchored standing. No stepping claim.'}
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    native = probe.Native(a.binary,a.checkpoint,a.physics_budget,deadline,a.output);report = {'status':'preflight-only'}
    samples = [];frames = [];notes = [];history = []
    try:
        state = native.command({'op':'observe'});initial_diag = native.command({'op':'diagnostic'});snapshot = native.command({'op':'snapshot'})
        assert state['metrics']['native_standing_without_foot_anchor']
        seed_q_error = float(np.max(np.abs(qs[0]-state['observation'][:21])));assert seed_q_error<1e-5
        original_root = probe.reference.transform(snapshot['segments'][0]['position'],snapshot['segments'][0]['rotation'])
        seed_root_error = float(np.max(np.abs(roots[0]-original_root)));assert seed_root_error<1e-5
        synthetic_error = float(np.max(np.abs(synthetic_physical(kin,state['observation'][:21],original_root)[:37]-state['observation'][:37])));assert synthetic_error<2e-5
        actual = np.array(native.command({'op':'initializedAction'})['action'])
        native_input = np.r_[state['observation'][:117],state['observation'][119]]
        parity = float(np.max(np.abs(actor.infer(native_input)-actual)));assert parity<5e-5
        config['nativeNumpyActorParityMaxError'] = parity
        config.update({'seedJointErrorRad':seed_q_error,'seedRootError':seed_root_error,'syntheticObservationFirst37MaxError':synthetic_error})
        (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
        stopped = None;right_run = longest_right = 0;hold_samples = [];hold_com = [];return_samples = [];last_samples = [];guard = GuardedTransfer(weights,qs,rows)
        if a.execute:
            report['status'] = 'running-positive-control'
            for tick in range(config['simulatedSeconds']*40):
                elapsed = tick/40
                if a.mode=='zero':
                    weight = 0.;blend = probe.smooth(elapsed) if elapsed<1 else (1-probe.smooth(elapsed-6) if elapsed>=6 else 1.)
                elif a.mode=='guarded-transfer':
                    weight,blend = guard.schedule(elapsed,initial_diag if tick==0 else diagnostic,state['metrics'],last_samples)
                    guard.inspect_native(elapsed,weight,blend,last_samples)
                else:
                    weight,blend = transfer_schedule(elapsed,weights,qs)
                qref = np.array([np.interp(weight,weights,qs[:,i]) for i in range(21)])
                ff = np.array([np.interp(weight,weights,taus[:,i]) for i in range(21)])
                rootref = original_root.copy();rootref[:3,3] = [np.interp(weight,weights,roots[:,i,3]) for i in range(3)]
                lean = np.interp(weight,weights,leans);c,s = np.cos(lean),np.sin(lean)
                rootref[:3,:3] = original_root[:3,:3]@np.array([[c,-s,0],[s,c,0],[0,0,1]])
                physical = synthetic_physical(kin,qref,rootref)
                ref_policy = actor.infer(np.r_[physical,standing,standing,1.])
                source = np.array(native.command({'op':'initializedAction'})['action'])
                correction = qref-kin.nominal+ff/kin.gears-ref_policy
                raw = source+blend*correction;action = np.clip(raw,-1.,1.)
                if a.mode=='guarded-transfer' and max(abs(action-raw))>1e-6:
                    guard.log.append({'event':'unrepresentable-action-source-abort','timeS':elapsed});guard.abort = (elapsed-.05,weight,0.)
                    action = source.copy();blend = 0.
                tr = native.command({'op':'step','action':action.tolist()});state = tr['state'];samples.extend(tr['native_foot_samples'])
                last_samples = tr['native_foot_samples']
                diagnostic = native.command({'op':'diagnostic'})
                if a.mode=='zero' and 1.<=elapsed<6.:
                    hold_samples.extend(tr['native_foot_samples']);hold_com.append(probe.reference.xyz(diagnostic['com']))
                if a.mode!='zero' and elapsed>=10.:return_samples.extend(tr['native_foot_samples'])
                for row in tr['native_foot_samples']:
                    right_fraction = row['loads_n'][1]/max(1e-9,sum(row['loads_n']))
                    good = right_fraction>=.85 and row['loads_n'][1]>=.7*kin.model['totalMassKg']*9.81
                    right_run = right_run+1 if good else 0;longest_right = max(longest_right,right_run)
                history.append({'tick':tick+1,'elapsedS':state['elapsed_s'],'com':diagnostic['com'],'capture':diagnostic['capture'],
                    'captureInSupport':diagnostic['predicates']['captureInSupport'],'comInSupport':diagnostic['predicates']['comInSupport'],
                    'supportPoints':diagnostic['solverSupportPoints'],'rightLoadFraction':state['metrics']['foot_loads_n'][1]/max(1e-9,sum(state['metrics']['foot_loads_n']))})
                notes.append({'phase':'transfer' if blend>0 else 'done','referenceWeight':weight,'modelBlend':blend,
                    'maximumRepresentationErrorRad':float(max(abs(action-raw))),'overrideMaxRad':float(max(abs(action-source))),
                    'modelFeedforwardTorquesNm':ff.tolist(),'referencePolicyAction':ref_policy.tolist()})
                if tick%2==0:frames.append(native.command({'op':'snapshot'}))
                if tr['terminated'] or tr['truncated']:stopped = tr['failure'] or 'episode-ended';break
            metrics = probe.analyse(samples);metrics.update({'label':'hybrid','stopped':stopped,'final':state,
                'initialDiagnostic':initial_diag,'finalDiagnostic':diagnostic,'maximumContinuousRightLoadShareAbove85PercentS':longest_right/800,
                'maximumRightLoadFraction':max(r['rightLoadFraction'] for r in history),
                'maximumActionClippingRad':max(r['maximumRepresentationErrorRad'] for r in notes)})
            metrics['controllerEvents'] = guard.log if a.mode=='guarded-transfer' else []
            if a.mode=='zero':
                hold_metrics = probe.analyse(hold_samples)
                hold_metrics['nativeSamples'] = len(hold_samples)
                hold_metrics['maximumHorizontalCOMDriftM'] = float(max((np.linalg.norm((v-probe.reference.xyz(initial_diag['com']))[[0,2]]) for v in hold_com),default=float('inf')))
                hold_metrics['passed'] = bool(stopped is None and len(hold_samples)==4000 and hold_metrics['maximumContinuousNativeStandingWithFootAnchorS']>=5. and hold_metrics['flightNativeSamples']==0 and not hold_metrics['nonfootSupportObserved'] and max(hold_metrics['loadedContactSlipIntegralM'])<=.01 and hold_metrics['maximumHorizontalCOMDriftM']<=.02)
                metrics['fullModelHold'] = hold_metrics
            else:
                return_metrics = probe.analyse(return_samples)
                return_metrics['nativeSamples'] = len(return_samples)
                return_metrics['passedStanding'] = bool(stopped is None and return_metrics['maximumContinuousNativeStandingWithFootAnchorS']>=5. and return_metrics['flightNativeSamples']==0 and not return_metrics['nonfootSupportObserved'])
                metrics['sourceReturnWindow10To16S'] = return_metrics
            (a.output/'hybrid.json').write_text(json.dumps({'metrics':metrics,'frames':frames,'controlNotes':notes,'supportHistory':history},indent=2)+'\n')
            native.command({'op':'checkpoint','path':str((a.output/'hybrid-final.bin').resolve())})
            report.update({'status':'completed-positive-control-not-qualification','metrics':metrics})
            print(json.dumps({k:metrics[k] for k in ['stopped','maximumContinuousRightLoadShareAbove85PercentS','maximumRightLoadFraction','maximumContinuousNativeStandingWithFootAnchorS','flightNativeSamples']}),flush=True)
    except BaseException as error:
        report['status'] = 'failed-or-incomplete-positive-control';report['error'] = repr(error);raise
    finally:
        try:native.close()
        except Exception as error:report['closeError'] = repr(error);report['status'] = 'failed-or-incomplete-positive-control'
        report.update({'physicsWallSeconds':native.wall,'nativeSubsteps':native.native,'uncertainOutstandingRequests':native.uncertain})
        (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
