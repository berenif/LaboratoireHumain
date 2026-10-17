#!/usr/bin/env python3
"""One contact-gated hybrid recovery-step probe, never a learned-policy claim.

The frozen GetUp SLOW remains reactive on all joints. Only one naturally unloaded
leg may blend toward an unassisted, limit-respecting IK reference. A paired
baseline uses the exact same checkpoint, native clocks and disturbance schedule.
Execution requires a separately authorized wall allowance and future deadline.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
import signal
import subprocess
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('step_reference', ROOT/'scripts/getup-step-reference.py')
reference = importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)
DT = .025


class Native:
    def __init__(self, binary, checkpoint, allowance, deadline, output):
        self.allowance = allowance;self.deadline = deadline;self.output = output
        self.wall = 0.;self.native = 0;self.uncertain = []
        self.trace = (output/'protocol.ndjson').open('w')
        self.proc = subprocess.Popen([str(binary.resolve()), str(checkpoint.resolve()), str(allowance), '--upright-entry'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=(output/'native-stderr.log').open('w'), text=True)

    def command(self, request):
        limit = min(5., self.deadline-time.time())
        assert limit > 0, 'Total compute deadline reached'
        def timeout(_signum, _frame):
            raise TimeoutError('Native response deadline')
        old = signal.signal(signal.SIGALRM, timeout);signal.setitimer(signal.ITIMER_REAL, limit)
        started = time.perf_counter()
        try:
            self.proc.stdin.write(json.dumps(request)+'\n');self.proc.stdin.flush()
            row = json.loads(self.proc.stdout.readline())
        except Exception:
            if request['op'] in ['step','stepMotionProfile','stepMotionProfileAudit']:
                charged = time.perf_counter()-started;self.wall += charged
                self.uncertain.append({'chargedWallSeconds':charged,'nativeSubstepsUpperBound':20})
            self.proc.kill();self.ledger();raise
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0);signal.signal(signal.SIGALRM, old)
        assert row['processPhysicsWallS'] >= self.wall and row['processNativeSubsteps'] >= self.native
        self.wall = row['processPhysicsWallS'];self.native = row['processNativeSubsteps']
        self.trace.write(json.dumps({'request':request,'response':row})+'\n');self.trace.flush();self.ledger()
        assert row['ok'], row
        return row['result']

    def ledger(self):
        (self.output/'budget-ledger.json').write_text(json.dumps({'physicsWallSeconds':self.wall,
            'nativeSubsteps':self.native,'authorizedNewAllowanceSeconds':self.allowance,
            'remainingSeconds':self.allowance-self.wall,'uncertainOutstandingRequests':self.uncertain,
            'utcDeadline':datetime.datetime.fromtimestamp(self.deadline,datetime.timezone.utc).isoformat()},indent=2)+'\n')

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)
        finally:
            if self.proc.poll() is None:self.proc.kill()
            self.trace.close();self.ledger()


def smooth(x):
    x = np.clip(x, 0., 1.)
    return float(x*x*(3-2*x))


class OneStep:
    def __init__(self, kin, strategy='original'):
        self.kin = kin;self.phase = 'waiting';self.side = None;self.elapsed = 0.;self.blend_elapsed = 0.
        self.strategy = strategy
        self.log = [];self.landing = None;self.clearance_seen = False;self.last_goal = None;self.handoff_logged = False

    def action(self, native, state, baseline, tick, last_samples, diagnostic=None):
        m = state['metrics'];q = np.array(state['observation'][:21]);base = np.array(baseline)
        self.current_diagnostic = diagnostic
        note = {'phase':self.phase,'side':self.side,'overrideMaxRad':0.}
        if self.phase == 'waiting':
            eligible = [i for i in range(2) if m['foot_loads_n'][i] < 3.
                        and m['foot_loads_n'][1-i] >= .7*self.kin.model['totalMassKg']*9.81]
            if eligible:
                diag = diagnostic or native.command({'op':'diagnostic'})
                self.current_diagnostic = diag
                if self.strategy.startswith('rising-'):
                    # Reject contact chatter and a descending return-to-floor.
                    # The preceding actual native samples must already show
                    # an unloaded swing and continuously loaded opposite foot.
                    eligible = [i for i in eligible if len(last_samples) >= 10
                        and all(s['loads_n'][i] < 3. and s['loads_n'][1-i] >= 3. for s in last_samples[-10:])
                        and diag['contactMotion']['feet'][i]['comVelocityMps']['y'] >= .03]
                if not eligible:
                    return base,note
                if not diag['predicates']['captureInSupport']:
                    self.side = eligible[0];self.foot = ['left_foot','right_foot'][self.side]
                    self.idx = np.arange(9,15) if self.side == 0 else np.arange(3,9)
                    if self.strategy.startswith('rising-'):
                        self.idx = np.array([11,12,13] if self.side == 0 else [5,6,7])
                    snapshot = native.command({'op':'snapshot'})
                    pose = next(p for p in snapshot['segments'] if p['id'] == self.foot)
                    self.origin = reference.xyz(pose['position']);self.orientation = reference.transform(pose['position'],pose['rotation'])
                    delta = reference.xyz(diag['capture'])-self.origin;delta[1] = 0
                    if self.strategy.startswith('rising-'):
                        root = reference.transform(snapshot['segments'][0]['position'],snapshot['segments'][0]['rotation'])
                        heading = self.kin.sagittal_heading(q,root,self.foot)
                        delta = heading*np.dot(delta,heading)
                    length = np.linalg.norm(delta)
                    if length >= .02:
                        self.landing = self.origin+delta*min(.06/length,1.)
                        self.phase = 'lift';self.elapsed = 0.
                        self.log.append({'event':'natural-unload-trigger','tick':tick,'side':self.side,
                            'loadsN':m['foot_loads_n'],'origin':self.origin.tolist(),'landing':self.landing.tolist(),
                            'diagnostic':diag})
        if self.phase in ['waiting','done']:
            return base, note
        if self.phase in ['lift','translate'] and self.blend_elapsed >= DT and any(
                s['loads_n'][self.side] >= 3. for s in last_samples):
            self.release('native-early-recontact',tick)
        if self.phase in ['lift','translate','land'] and any(
                s['loads_n'][1-self.side] < 3. for s in last_samples):
            self.release('native-stance-support-lost',tick)
        self.elapsed += DT;self.blend_elapsed += DT
        target = self.origin.copy();weight = smooth(self.blend_elapsed/.05)
        if self.phase == 'lift':
            rise_s = .05 if self.strategy.startswith('rising-') else .12
            target[1] += .06*smooth(self.elapsed/rise_s)
            if self.elapsed >= .05 and m['foot_clearance_m'][self.side] >= .015:
                self.clearance_seen = True;self.phase = 'translate';self.elapsed = 0.
                self.log.append({'event':'clearance-gate','tick':tick,'clearanceM':m['foot_clearance_m'][self.side],
                                 'diagnostic':diagnostic})
            elif self.elapsed > .225:
                self.release('no-clearance-before-timeout',tick)
            elif self.elapsed > DT and m['foot_loads_n'][self.side] >= 3.:
                self.release('early-recontact-before-clearance',tick)
        if self.phase == 'translate':
            if m['foot_clearance_m'][self.side] < .008:
                self.release('clearance-lost-before-landing',tick)
            else:
                progress = smooth(self.elapsed/.15)
                target = self.origin+(self.landing-self.origin)*progress
                target[1] = self.origin[1]+.06
                if self.elapsed >= .15:
                    self.phase = 'land';self.elapsed = 0.
        if self.phase == 'land':
            target = self.landing.copy();target[1] = self.origin[1]+.06*(1-smooth(self.elapsed/.15))
            if self.elapsed >= DT and m['foot_loads_n'][self.side] >= 3.:
                self.release('landing-contact',tick)
            elif self.elapsed > .3:
                self.release('landing-timeout',tick)
        if self.phase not in ['release','done'] and m['foot_loads_n'][1-self.side] < 3.:
            self.release('stance-support-lost',tick)
        if self.phase == 'release':
            weight = 1-smooth(self.elapsed/.05)
            goal = self.last_goal if self.last_goal is not None else q
            if self.elapsed >= .05:self.phase = 'done';weight = 0.
            error = np.zeros(6)
        else:
            snapshot = native.command({'op':'snapshot'})
            root = reference.transform(snapshot['segments'][0]['position'],snapshot['segments'][0]['rotation'])
            kwargs = {'indices':self.idx} if self.strategy.startswith('rising-') else {}
            goal, predicted_pose, error = self.kin.leg_ik(q,root,self.foot,target,self.orientation,max_delta=.8,**kwargs)
            self.last_goal = goal.copy()
        action = base.copy()
        wanted = goal[self.idx]-self.kin.nominal[self.idx]
        if self.strategy == 'rising-additive':
            wanted = base[self.idx]+goal[self.idx]-q[self.idx]
        requested = base[self.idx]*(1-weight)+wanted*weight
        action[self.idx] = np.clip(requested,-1.,1.)
        representation_error = action[self.idx]-requested
        dq = np.array(state['observation'][46:67])
        source_torques = np.clip(self.kin.nominal[self.idx]+base[self.idx]-q[self.idx]-.2*dq[self.idx],-1,1)*self.kin.gears[self.idx]
        represented_torques = np.clip(self.kin.nominal[self.idx]+action[self.idx]-q[self.idx]-.2*dq[self.idx],-1,1)*self.kin.gears[self.idx]
        override = float(np.max(np.abs(action-base)))
        if self.phase == 'done' and not self.handoff_logged:
            assert override == 0., 'Source handoff must restore its exact current action'
            self.log.append({'event':'source-handoff','tick':tick,'firstZeroOverrideTick':tick,
                             'diagnostic':diagnostic})
            self.handoff_logged = True
        note = {'phase':self.phase,'side':self.side,'phaseElapsedS':self.elapsed,'blend':weight,
                'overriddenQposIndices':self.idx.tolist(),
                'target':target.tolist(),'IKPositionErrorM':float(np.linalg.norm(error[:3])),
                'overrideMaxRad':override,
                'requestedEffectiveTargetsRad':(self.kin.nominal[self.idx]+requested).tolist(),
                'representedEffectiveTargetsRad':(self.kin.nominal[self.idx]+action[self.idx]).tolist(),
                'representationErrorsRad':representation_error.tolist(),
                'sourceDecisionMotorTorquesNm':source_torques.tolist(),
                'representedDecisionMotorTorquesNm':represented_torques.tolist(),
                'maximumRepresentationErrorRad':float(np.max(np.abs(representation_error))),
                'otherJointMaxDifference':float(max(abs(action[i]-base[i]) for i in range(21) if i not in self.idx))}
        if self.phase not in ['release','done']:
            note['predictedIKWholeFootClearanceM'] = self.kin.bottom(self.foot,predicted_pose)
        return action,note

    def release(self, reason, tick):
        self.log.append({'event':reason,'tick':tick,'diagnostic':self.current_diagnostic})
        self.phase = 'release';self.elapsed = 0.


def analyse(samples):
    events = [];active = [None,None];quiet = longest = anchored = longest_anchored = 0
    feet_anchor = None;slip = np.zeros(2)
    for s in samples:
        quiet = quiet+1 if s['standing_without_foot_anchor'] else 0;longest = max(longest,quiet)
        if s['standing_without_foot_anchor']:
            if feet_anchor is None:feet_anchor = [reference.xyz(v) for v in s['positions']]
            anchored_ok = all(np.linalg.norm(reference.xyz(v)-a) <= .01 for v,a in zip(s['positions'],feet_anchor))
        else:
            anchored_ok = False
        anchored = anchored+1 if anchored_ok else 0
        if not anchored_ok:feet_anchor = None
        longest_anchored = max(longest_anchored,anchored)
        slip += np.array(s['loaded_contact_slip_mps'])/800
        for i in range(2):
            if s['loads_n'][i] < 3.:
                if active[i] is None:
                    active[i] = {'side':i,'origin':s['positions'][i],'startNative':s['native_substep'],
                                 'maximumClearanceM':0.,'minimumOtherLoadN':float('inf')}
                a = active[i];a['maximumClearanceM'] = max(a['maximumClearanceM'],s['clearance_m'][i])
                a['minimumOtherLoadN'] = min(a['minimumOtherLoadN'],s['loads_n'][1-i])
            elif active[i] is not None:
                a = active[i];a['landing'] = s['positions'][i];a['endNative'] = s['native_substep']
                delta = reference.xyz(a['landing'])-reference.xyz(a['origin'])
                a['horizontalDisplacementM'] = float(np.linalg.norm(delta[[0,2]]))
                a['durationS'] = (a['endNative']-a['startNative'])/800
                events.append(a);active[i] = None
    return {'unloadedRepositionEvents':events,'maximumContinuousNativeStandingWithoutFootAnchorS':longest/800,
            'maximumContinuousNativeStandingWithFootAnchorS':longest_anchored/800,
            'loadedContactSlipIntegralM':slip.tolist(),'nonfootSupportObserved':any(s['nonfoot_support'] for s in samples),
            'flightNativeSamples':sum(all(v<3 for v in s['loads_n']) for s in samples)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--physics-budget',type=float,required=True)
    p.add_argument('--deadline',required=True);p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--case',choices=['baseline','hybrid','paired'],default='paired')
    p.add_argument('--strategy',choices=['original','rising-sagittal','rising-additive'],default='original')
    p.add_argument('--execute',action='store_true');a = p.parse_args()
    deadline = datetime.datetime.fromisoformat(a.deadline.replace('Z','+00:00')).timestamp()
    assert 0 <= a.physics_budget <= 360 and 0 < a.seconds <= 10
    assert deadline > time.time() and deadline-time.time() <= 1800
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'started-once').open('x').close()
    kin = reference.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))
    config = {'physicsAllowanceSeconds':a.physics_budget,'totalDeadlineUTC':a.deadline,'simulatedSecondsPerCase':a.seconds,
              'case':a.case,'strategy':a.strategy,'checkpointSha256':hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
              'nativeBinarySha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
              'probeSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'referenceScriptSha256':hashlib.sha256((ROOT/'scripts/getup-step-reference.py').read_bytes()).hexdigest(),
              'controller':'frozen-SLOW-with-one-contact-gated-IK-swing-leg-intervention-untrained',
              'clearanceGateM':.015,'plannedFootOriginRaiseM':.06,'maximumCaptureDirectedPlacementM':.06,
              'naturalUnloadGateN':3.,'initialStanceGateBodyweightFraction':.7,'blendS':.05,
              'minimumNativeUnloadS':.0125 if a.strategy.startswith('rising-') else 0.,
              'minimumRisingFootComSpeedMps':.03 if a.strategy.startswith('rising-') else None,
              'liftReferenceRiseS':.05 if a.strategy.startswith('rising-') else .12,
              'commandComposition':'source-plus-IK-displacement' if a.strategy == 'rising-additive' else 'IK-effective-target-blend',
              'disturbanceSchedule':[],'noRootOrFootConstraints':True,'frozenWeightsChanged':False,
              'matchedPostWindow':'From 0.5s after hybrid source handoff to the common trial end; mechanism proof requires >=5s remaining.',
              'acceptance':'A real >=1.5cm cleared >=5cm unloaded reposition for this 6cm-target mechanism probe, with persistent stance support, '
                           'no flight/nonfoot/integrity failure, improved support/capture, then >=5s unchanged native standing; '
                           'loaded slip and quiet behavior compared with same-checkpoint baseline. Not a learned step.'}
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    native = Native(a.binary,a.checkpoint,a.physics_budget,deadline,a.output)
    report = {'status':'preflight-only','cases':[]};case_histories = {}
    try:
        initial = native.command({'op':'observe'})
        initialized = native.command({'op':'initializedAction'})['action']
        nominal = np.r_[kin.nominal,1.4,0,0,1].tolist()
        equivalent = native.command({'op':'referenceAction','now':nominal,'ahead':nominal})
        assert np.max(np.abs(np.array(initialized)-equivalent['action'])) < 5e-6
        if not a.execute:
            native.command({'op':'diagnostic'})
        else:
            report['status'] = 'running-probe'
            for label in (['baseline','hybrid'] if a.case == 'paired' else [a.case]):
                state = native.command({'op':'reset'});controller = OneStep(kin,a.strategy)
                samples = [];frames = [];notes = [];stopped = None;last_samples = [];support_history = []
                diagnostic = native.command({'op':'diagnostic'});initial_diagnostic = diagnostic
                for tick in range(round(a.seconds/DT)):
                    baseline = native.command({'op':'initializedAction'})['action']
                    action,note = controller.action(native,state,baseline,tick,last_samples,diagnostic) if label == 'hybrid' else (np.array(baseline),{})
                    tr = native.command({'op':'step','action':action.tolist()})
                    state = tr['state'];last_samples = tr['native_foot_samples'];samples.extend(last_samples);notes.append(note)
                    diagnostic = native.command({'op':'diagnostic'})
                    support_history.append({'tick':tick+1,'elapsedS':state['elapsed_s'],
                        'captureInSupport':diagnostic['predicates']['captureInSupport'],
                        'comInSupport':diagnostic['predicates']['comInSupport'],
                        'capture':diagnostic['capture'],'com':diagnostic['com'],
                        'supportPoints':diagnostic['solverSupportPoints']})
                    if tick%2 == 0:frames.append(native.command({'op':'snapshot'}))
                    if tr['terminated'] or tr['truncated']:
                        stopped = tr['failure'] or 'episode-ended';break
                metrics = analyse(samples);metrics.update({'label':label,'stopped':stopped,'final':state,'controllerEvents':controller.log,
                    'initialDiagnostic':initial_diagnostic,'finalDiagnostic':diagnostic})
                report['cases'].append(metrics)
                case_histories[label] = (support_history,samples,initial_diagnostic['nativeSubsteps'])
                (a.output/f'{label}.json').write_text(json.dumps({'metrics':metrics,'frames':frames,'controlNotes':notes,
                    'supportHistory':support_history},indent=2)+'\n')
                native.command({'op':'checkpoint','path':str((a.output/f'{label}-final.bin').resolve())})
                print(json.dumps({'label':label,'stopped':stopped,'physicsWallSeconds':native.wall,
                                  'maximumStandingS':metrics['maximumContinuousNativeStandingWithoutFootAnchorS'],
                                  'events':controller.log}),flush=True)
            if all(label in case_histories for label in ['baseline','hybrid']):
                hybrid = next(c for c in report['cases'] if c['label'] == 'hybrid')
                handoff = next((e for e in hybrid['controllerEvents'] if e['event'] == 'source-handoff'),None)
                if handoff:
                    start = handoff['tick']*DT+.5
                    end = min(case_histories[k][0][-1]['elapsedS'] for k in ['baseline','hybrid'])
                    matched = {'startS':start,'endS':end,'durationS':max(0.,end-start),'cases':{}}
                    for label,(history,foot_samples,initial_native) in case_histories.items():
                        rows = [r for r in history if start <= r['elapsedS'] <= end]
                        foot_rows = [s for s in foot_samples if start <= (s['native_substep']-initial_native)/800 <= end]
                        matched['cases'][label] = {'captureInsideFractionAt40Hz':sum(r['captureInSupport'] for r in rows)/max(1,len(rows)),
                            'comInsideFractionAt40Hz':sum(r['comInSupport'] for r in rows)/max(1,len(rows)),
                            'sampleCount40Hz':len(rows),**analyse(foot_rows)}
                    matched['captureInsideFractionDeltaHybridMinusBaseline'] = (
                        matched['cases']['hybrid']['captureInsideFractionAt40Hz']-matched['cases']['baseline']['captureInsideFractionAt40Hz'])
                    report['matchedPostInterventionWindow'] = matched
            report['status'] = 'completed-probe-not-qualification'
    except BaseException as error:
        report['status'] = 'failed-or-incomplete-probe' if a.execute else 'failed-preflight'
        report['error'] = repr(error)
        raise
    finally:
        try:
            native.close()
        except Exception as error:
            report['closeError'] = repr(error);report['status'] = 'failed-or-incomplete-probe' if a.execute else 'failed-preflight'
        report.update({'physicsWallSeconds':native.wall,'nativeSubsteps':native.native,
                                     'uncertainOutstandingRequests':native.uncertain,
                                     'remainingPhysicsSeconds':a.physics_budget-native.wall})
        (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
