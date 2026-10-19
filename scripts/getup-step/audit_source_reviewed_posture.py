#!/usr/bin/env python3
"""Explicit saved-data v3 posture pilot profile atop unchanged physical reducers.

The default reviewed-walk entry point rejects these tasks. This policy supplies
only independently reconstructed reference jets and a separate fresh-history gate.
"""
import argparse
import copy
import json
from collections import deque
from pathlib import Path

import numpy as np

import audit_source_reviewed_walk as core
from audit_source_robust_support import support_observation, forecast_math, triangle

PROFILE = 'reviewed-coupled-posture-pilot-v3'
POSTURE_SOURCES = frozenset(('source_reviewed_posture.py', 'source_posture_retarget.py',
    'coupled_posture_reference.py', 'coupled_posture_kinematics.py',
    'coupled_posture_admission.py', 'source_bounded_com_settle.py'))


class PosturePolicy:
    profile = PROFILE

    def __init__(self, folder):
        self.folder = Path(folder)
        self.static = {}; self.settles = {}; self.manifests = {}; self.initial_checks = {}
        self.history = {}; self.admitted = {}; self.checks = {}; self.segments = {}; self.return_bindings = {}
        self.model = json.loads((Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json').read_text())
        self.mass = self.model['totalMassKg']; self.native = None
        initial=json.loads((self.folder/'summary.json').read_text())
        preset=json.loads(Path(__file__).with_name('source-step-selected-v1.json').read_text())
        self.bindings=dict(modelSHA256=initial['modelSHA256'],helperSHA256=preset['helperSHA256'])
        from audit_source_posture_references import audit_posture_static_receipt
        bound_limits=core.limits_from_bound_xml(self.folder.parents[1]/'model-session/source-model/humanoid_high_freq.xml',self.model)
        for path in sorted(self.folder.glob('posture-static-step-*.json')):
            index = int(path.stem.rsplit('-', 1)[1]); value = json.loads(path.read_text())
            self.static[index] = value
            self.initial_checks[index] = audit_posture_static_receipt(value, model_data=self.model,joint_limits=bound_limits)
            self.manifests[index] = json.loads((self.folder/f'posture-risk-step-{index:02d}'/'manifest.json').read_text())
            self.settles[index] = json.loads((self.folder/f'posture-settle-step-{index:02d}.json').read_text())
        if not self.static: raise ValueError('missing posture static evidence')
        self.input_hashes = {str(path.relative_to(self.folder)): core.file_sha(path)
            for pattern in ('posture-static-step-*.json','posture-settle-step-*.json','posture-risk-step-*/*')
            for path in self.folder.glob(pattern) if path.is_file()}

    def _record(self, checks):
        for key, value in checks.items(): self.checks[key] = self.checks.get(key, True) and bool(value)
        return checks

    def _index(self, config, causal):
        active = config.get('coupledPosture')
        if active is None: return None
        source = float(causal['arm']['nativeSourceTimeS'])
        matches = [i for i, static in self.static.items() if core.equal(active['plan'], static['frozenPlan'])
            and source >= static['requestedSourceTimeS']-1e-10]
        if len(matches) != 1: raise ValueError('posture context does not match one frozen step plan')
        return matches[0]

    def references(self, config, causal, elapsed, phase, hypothetical):
        from audit_source_posture_references import posture_reference_jets
        index = self._index(config, causal)
        c = copy.deepcopy(config); ca = copy.deepcopy(causal)
        c.pop('coupledPosture', None); c.pop('boundedSettle', None)
        if index is not None and phase == 'settle':
            # The ordinary continuous quintic is explicitly replaced only by
            # the independently reconstructed bounded profile below.
            c['mode'] = 'baseline-context'
        expected = core.independent_references(c, ca, elapsed, phase, hypothetical)
        if index is None: return expected
        if phase == 'settle':
            if index not in self.segments: raise ValueError('bounded settle not independently initialized')
            expected['com'] = self.segments[index]['expected'](causal['arm']['nativeSourceTimeS'])
        changed = posture_reference_jets(config, causal, elapsed, phase,
            expected['com'], expected['joints'], expected['orientations'])
        for key in ('com','joints','orientations'):
            if key in changed: expected[key] = changed[key]
        if phase=='settle': expected['correctionClock']=np.array([1.,0.,0.])
        return expected

    def correction_clock(self,note,original):
        active=note.get('referenceContext',{}).get('config',{}).get('coupledPosture')
        return (1.,0.,0.) if active is not None and note['phase']=='settle' else original

    def bound_reference(self, reference, receipt):
        result = copy.deepcopy(reference)
        active = receipt.get('config', {}).get('coupledPosture')
        if active is not None:
            target = np.asarray(active['plan']['candidate']['target']['com']['position'])
            delta = np.asarray(active['completedBaselineCorrectionM'])
            result['com1'] = (target-delta).tolist()
        return result

    def context_checks(self, receipt, note, row, reference):
        from audit_source_posture_references import audit_posture_return
        c, ca = receipt['config'], receipt['causal']; active = c.get('coupledPosture')
        checks = dict(explicitPostureFeatureEveryCommand=note.get('reviewedWalk', {}).get('featureProfile') == PROFILE)
        index = note['stepIndex']; static = self.static[index]
        now = receipt['source_time']; start = static['requestedSourceTimeS']
        checks['postureActivatesOnlyAtBoundNominalEndpoint'] = (active is not None) == (now >= start-1e-10)
        if active is None:
            checks['noPostureOverlapBeforeActivation']=ca.get('coupledPostureOverlap') is None
            return self._record(checks)
        overlap=ca.get('coupledPostureOverlap')
        frozen=getattr(self,'overlap_bindings',{})
        checks['postureOverlapFrozenOnce']=index not in frozen or core.equal(frozen[index],overlap)
        frozen.setdefault(index,copy.deepcopy(overlap));self.overlap_bindings=frozen
        target = np.asarray(static['candidate']['target']['com']['position'])
        baseline_delta = np.asarray(ca['robustCorrection']['deltaM'])
        correction_binding={k:ca['robustCorrection'][k]for k in ('selected','deltaM','startTransferS','durationS')}
        baseline_bindings=getattr(self,'baseline_bindings',{})
        checks['completedBaselineCorrectionNeverRetargeted']=index not in baseline_bindings or core.equal(baseline_bindings[index],correction_binding)
        baseline_bindings.setdefault(index,copy.deepcopy(correction_binding));self.baseline_bindings=baseline_bindings
        checks.update(explicitPostureProfile=active.get('profile') == PROFILE,
            frozenStaticIndependentChecks=self.initial_checks[index]['passed'] is True,
            immutableFrozenPlan=core.equal(active['plan'], static['frozenPlan']),
            frozenStaticInputDigest=active['frozenCandidateInputSHA256'] == static['inputSHA256'],
            completedBaselineOffsetExactlyOnce=core.equal(active['completedBaselineCorrectionM'], baseline_delta)
                and core.equal(c['com1'], target-baseline_delta),
            noInheritedPostureAdmission=active.get('originalAdmissionInheritedForPosture') is False,
            sameProvisionalGeometry=active['provisionalSurvivors'] == static['input']['support']['surviving_corners']
                == note['robustSupportAdmission']['survivingCorners'],
            inheritedReserveDistinction=core.equal(active['inheritedTransition'], static['inheritedTransition']),
            baselineCorrectionCompleteBeforeActivation=ca['robustCorrection']['selected'] is True
                and start >= static['input']['sourceTimeS']-1e-10
                and start-static['input']['sourceTimeS'] <= 1e-10
                and (receipt['phase']!='transfer' or receipt['elapsed'] >= ca['robustCorrection']['startTransferS']+ca['robustCorrection']['durationS']-1e-10))
        if abs(now-start) < 1e-9:
            initial = static['input']['initialCoupledReference']; base = copy.deepcopy(c)
            base['com1'] = reference['com1']; base.pop('coupledPosture', None)
            expected = core.independent_references(base, ca, receipt['elapsed'], receipt['phase'])
            obs = support_observation(row, reference['stance_side'])
            overlap=core.audit_overlap_certificate(ca['coupledPostureOverlap'],
                np.asarray(static['candidate']['target']['com']['position'])-np.asarray(initial['com']['position']),
                1.52,[0.,0.,0.],0.,1.)
            checks['independentWholePostureCOMEnvelope']=overlap['valid'] and overlap['independent']['passed']
            checks['activationInitialCompleteOffsetBoundToBaseline'] = core.jet_matches(initial['com'], expected['com'], 3)
            checks['activationInitialLowerBoundToBaseline'] = all(core.equal(initial['lower_joints'][key], expected['joints'][j][:15])
                for j,key in enumerate(('position','velocity','acceleration')))
            support = static['input']['support']
            checks['staticInputUsesActualActivationGeometry'] = (static['requestedSourceStep'] == row['sourceStep']
                and core.equal(support['loaded_points_xy'], row['supportPointsXY'])
                and core.equal(support['stance_points_xyz'], obs['matchedCorners'])
                and core.equal(support['present_corners'], obs['present'].tolist())
                and receipt['elapsed'] >= reference['phase_durations']['transfer']-1e-10)
        from scipy.spatial import ConvexHull
        actual=[ConvexHull(np.unique(np.asarray(row['supportPointsXY']),axis=0)).equations]
        if receipt['phase'] in ('transfer','lift','translate','land'):
            obs=support_observation(row,reference['stance_side'])
            actual.append(triangle(obs,active['provisionalSurvivors'],actual=True))
        equations=np.vstack(actual);p,v,a=core.jet(receipt['com'],3)
        capture=p[:2]+np.sqrt(p[2]/9.81)*v[:2]
        margin=min(np.min(-(equations[:,:2]@p[:2]+equations[:,2])),np.min(-(equations[:,:2]@capture+equations[:,2])))
        checks['everyPostureReferenceUsesCurrentActualHullAndRequiredTriangle']=margin>=.003-1e-10
        checks['everyPostureCompleteJetUsesOriginalEnvelope']=(np.linalg.norm(v)<=.19260552331163003+1e-10
            and np.linalg.norm(a)<=.3901818436690813+1e-10)
        if receipt['phase'] in ('settle','stand'):

            ret = active['originalRestReturn']
            if index not in self.return_bindings:
                self.return_bindings[index] = copy.deepcopy(ret)
            checks['measuredRestReturnFrozenOnce'] = core.equal(ret, self.return_bindings[index])
            retproof = audit_posture_return(c, ca, receipt['elapsed'], receipt['phase'], self.model)
            checks['independentMeasuredRestReturn'] = retproof['passed']
        if receipt['phase'] == 'settle':
            # Initialize before independent_references is called by audit_context.
            checks.update(self.settle_checks(receipt, row, None))
        return self._record(checks)

    def settle_checks(self, receipt, row, state):
        from audit_source_bounded_settle import audit_bounded_settle_receipt
        index = receipt['step_index']; c = receipt['config']; ca = receipt['causal']; saved = self.settles[index]
        checks = {}
        if index not in self.segments:
            start = float(receipt['source_time'])-float(receipt['elapsed'])
            p,v,a = core.segment_jet(ca['continuousCom']['path'], start)
            initial = (np.asarray(c['com1'])+np.asarray(ca['robustCorrection']['deltaM'])+p,v,a)
            final = (np.asarray(c['finalCom']),np.zeros(3),np.zeros(3))
            proof = audit_bounded_settle_receipt(saved, initial, final, c['phase_durations']['settle'], row['supportPointsXY'])
            self.segments[index] = proof
            checks['boundedSettleBoundToMeasuredHandoff'] = (saved['plannedAtNativeStep'] == row['sourceStep']
                and core.close(saved['plannedAtSourceTimeS'], row['sourceTimeS'])
                and core.close(saved['path']['startSourceTimeS'], start)
                and all(saved.get(k)==v for k,v in self.bindings.items()))
        checks['independentAnalyticBoundedSettle'] = self.segments[index]['passed']
        declared = c.get('boundedSettle', {})
        checks['boundedSettleCertificateBoundOnce'] = (declared.get('fullCertificateSHA256') == self.input_hashes[f'posture-settle-step-{index:02d}.json']
            and declared.get('version') == saved['version'] and declared.get('profile') == saved['profile']
            and all(declared.get(k) is True for k in ('currentActualHullOnly','liveValidationEveryCommand',
                'durationUnchanged','measuredTargetUnchanged','frozenCapsUnchanged'))
            and core.equal(ca['continuousCom']['settle'], saved['path']))
        return self._record(checks)

    def reference_diagnostic(self,receipt):
        return (receipt['diagnostics']['coupledPosture'] if receipt['config'].get('coupledPosture') is not None
            else receipt['causal']['continuousCom']['diagnostic'])

    def expected_com(self, note, reference):
        r = note['referenceContext']
        return self.references(r['config'], r['causal'], r['elapsed'], r['phase'], False)['com'][0]

    def goal(self, note, reference):
        active = note.get('referenceContext', {}).get('config', {}).get('coupledPosture')
        if active is not None: return np.asarray(active['plan']['candidate']['target']['com']['position'])
        return np.asarray(reference['com1'])+np.asarray(note['robustSupportAdmission'].get('comCorrectionM', np.zeros(3)))

    def ready(self, note, row):
        active = note.get('referenceContext', {}).get('config', {}).get('coupledPosture')
        if active is None: return False
        index = note['stepIndex']; plan = active['plan']['com']
        return (row['sourceTimeS'] >= self.manifests[index]['historySourceEndS']-1e-10
            and row['sourceTimeS'] >= plan['start']+plan['duration']-1e-10
            and min(row['jointLimitMarginsRad'][i] for i in (7,13)) >= .035)

    def observe(self, note, row, reference, elapsed, native):
        self.native = native; index = note['stepIndex']; receipt = note['referenceContext']
        active = receipt['config'].get('coupledPosture'); checks = {}
        if active is None:
            checks['noFreshPostureAdmissionBeforeChangedTasks'] = note.get('coupledPosture', {}).get('freshPostureAdmission') is False
            return self._record(checks)
        if self.admitted.get(index):
            return self._record(dict(freshPostureAdmissionNeverDisappears=note.get('coupledPosture',{}).get('freshPostureAdmission') is True))
        if note['phase'] != 'transfer':
            return self._record(dict(noLiftBeforeFreshPostureAdmission=False))
        history = self.history.setdefault(index, deque()); loaded = row['feet'][reference['stance_side']]['loadN'] >= .8*self.mass*9.81
        if not loaded: history.clear()
        eligible = False
        if loaded:
            forecast = note['robustSupportAdmission'].get('currentPostureUnloadForecast', {})
            calculation = forecast_math(forecast,row,reference)
            checks.update({'postureForecast:'+k:v for k,v in calculation['checks'].items()})
            witness = self.replay_window('posture', note,row,reference,forecast)
            checks['postureRetainedNativeForecastReplay'] = witness.get('passed') is True
            now = row['sourceTimeS']; obs = support_observation(row,reference['stance_side'])
            if history and now-history[-1][0] > .0050001: history.clear()
            history.append((now,calculation['risk'],obs['present'],loaded,row['sourceStep']))
            while len(history)>1 and history[1][0] < now-.1-1e-10: history.popleft()
            if len(history)>=21 and now-history[0][0]>=.1-1e-9:
                votes=np.bincount([h[1] for h in history if h[1]>=0],minlength=4)
                weak=int(np.argmax(votes)); ids=[i for i in range(4) if i!=weak]
                eligible=(ids==active['provisionalSurvivors'] and votes[weak]/len(history)>=.8
                    and all(h[3] and all(h[2][ids]) for h in history))
        declared = note.get('coupledPosture', {}).get('freshPostureAdmission')
        checks['postureAdmissionAtFirstFreshEligibleCommand'] = declared is bool(eligible)
        if eligible:
            m=self.manifests[index]; ids=[h[4] for h in history]
            checks['completeUniquePostureNativeWindow'] = self.coverage_window('posture',index,ids)
            checks['freshPostureManifestMatchesCausalWindow'] = (m['profile']==PROFILE and m['originalAdmissionInherited'] is False
                and m['originalCorrectionCompleteBeforeActivation'] is True and m['physicsIntegrations']==0
                and m['candidateKey']==self.static[index]['candidateKey']
                and core.close(m['historySourceStartS'],history[0][0]) and core.close(m['historySourceEndS'],history[-1][0])
                and m['historySamples']==len(history) and m['historyPredictedRiskCorners']==[h[1] for h in history]
                and core.close(m['riskAgreementFrequency'],votes[weak]/len(history))
                and m['survivingCorners']==active['provisionalSurvivors'] and core.close(m['selectedAtTransferS'],elapsed)
                and m['allSurvivorsActuallySupportedThroughoutHistory'] is True)
            self.admitted[index]=True
        return self._record(checks)

    # The native replay implementation is shared; only manifest dictionaries
    # switch between baseline and fresh-posture windows in the same scratch world.
    _WINDOW_KEYS=('manifests','expected','samples','seen','covered')
    def _window(self, name, call):
        native=self.native; saved={k:getattr(native,k) for k in self._WINDOW_KEYS}
        try:
            for k,v in self.windows[name].items(): setattr(native,k,v)
            return call()
        finally:
            self.windows[name]={k:getattr(native,k) for k in self._WINDOW_KEYS}
            for k,v in saved.items():setattr(native,k,v)

    def replay_window(self,name,*args):return self._window(name,lambda:self.native(*args))
    def coverage_window(self,name,index,ids):return self._window(name,lambda:self.native.coverage(index,ids))

    def install_native_windows(self,native):
        self.native=native; data=dict(manifests={},expected={},samples={},seen={},covered=set())
        for index,m in self.manifests.items():
            native._require(m.get('schema')==1 and m.get('profile')==PROFILE,'wrong posture manifest profile')
            for key,digest in native.bindings.items():native._require(m.get(key)==digest,'posture manifest '+key+' mismatch')
            samples=m.get('samples',[]);ids=[s['sourceStep'] for s in samples];times=[s['sourceTimeS'] for s in samples]
            native._require(21<=len(ids)<=22 and all(type(i)is int for i in ids)
                and all(b-a==4 for a,b in zip(ids,ids[1:]))
                and all(abs(b-a-.005)<1e-9 for a,b in zip(times,times[1:]))
                and .1-1e-9<=times[-1]-times[0]<=.105+1e-9,'invalid posture native window')
            folder=self.folder/f'posture-risk-step-{index:02d}'
            for sample in samples:
                sid=sample['sourceStep'];filename=f'source-{sid:06d}.native-payload.zlib';path=folder/filename
                native._require(sample.get('filename')==filename and path.is_file() and not path.is_symlink()
                    and 0<path.stat().st_size<=native.MAX_COMPRESSED_BYTES
                    and type(sample.get('payloadBytes'))is int and 0<sample['payloadBytes']<=native.MAX_RAW_BYTES
                    and core.is_sha(sample.get('payloadSHA256')) and core.file_sha(path)==sample.get('compressedSHA256'),
                    'invalid posture native payload binding')
                data['samples'][(index,sid)]=(sample,path)
            native._require({p.name for p in folder.glob('*.native-payload.zlib')}=={s['filename'] for s in samples},'extra posture native files')
            data['manifests'][index]=m;data['expected'][index]=ids;data['seen'][index]=[]
        self.windows={'posture':data}

    def declarations(self,config,receipt,cadence):
        return (config.get('profile')==receipt.get('profile')==PROFILE and config.get('requestedSteps') in (1,2)
            and all(config.get(k)is True and receipt.get(k)is True for k in ('coupledPostureEnabled','inheritedPhysicalGatesUnchanged'))
            and config.get('minimumPostureAnkleReadyRad')==receipt.get('minimumPostureAnkleReadyRad')==.035
            and config.get('oldProfileQualificationClaim')is False and receipt.get('oldProfileQualificationClaim')is False
            and receipt.get('finalPlanQualificationClaim')is False
            and cadence.get('reviewedReferenceProfile',{}).get('featureProfile')==PROFILE
            and POSTURE_SOURCES<=set(receipt.get('codeSHA256AtStart',{})))

    def finish(self,result):
        native=self._window('posture',lambda:self.native.summary())
        checks=dict(self.checks)
        checks['allPostureStepsFreshlyAdmitted']=set(self.admitted)==set(self.static)
        checks['allStaticFamiliesIndependentlyBound']=all(p['passed']for p in self.initial_checks.values())
        checks['allBoundedSettlesIndependentlyReconstructed']=set(self.segments)==set(self.static) and all(p['passed']for p in self.segments.values())
        checks['allFreshPostureNativeWitnessesReplayed']=native['passed']
        checks['postureEvidenceUnchangedDuringAudit']=all((self.folder/name).is_file()and core.file_sha(self.folder/name)==sha for name,sha in self.input_hashes.items())
        result['checks']['explicitIndependentPostureFeatureReplay']=all(checks.values())
        passed=all(result['checks'].values())
        result.update(profile=PROFILE,reviewedWalkPilotPhysicalPass=False,reviewedWalkDiagnosticPass=False,
            reviewedWalkTenStepPhysicalPass=False,reviewedWalkFinalPlanPass=False,
            coupledPosturePilotPhysicalPass=passed and result['requestedSteps']==1,
            coupledPostureDiagnosticPass=passed and result['requestedSteps']==2,
            finalTenStepQualificationPass=False,visualImprovementQualificationPass=False,
            oldProfileQualificationClaim=False,
            postureReplay=dict(passed=all(checks.values()),checks=checks,failedChecks=[k for k,v in checks.items()if not v],
                nativeFreshHistory=native,staticProofs=self.initial_checks,
                boundedSettleProofs=json.loads(json.dumps({i:{k:v for k,v in p.items()if k!='expected'}for i,p in self.segments.items()},default=lambda value:value.tolist())),
                hashes=self.input_hashes,physicsIntegrations=0))
        result['postureAuditorSHA256']={name:core.file_sha(Path(__file__).with_name(name))for name in
            ('audit_source_reviewed_posture.py','audit_source_posture_references.py','audit_source_bounded_settle.py')}
        result['failedChecks']=[k for k,v in result['checks'].items()if not v]
        return result


def audit_clock_ownership(notes, rows, entry, policy):
    """Focused all-command ownership proof; no QP, engine or integration.

    The completed offset enters bounded settle's initial jet once. Its retained
    robust clock is inert diagnostic state, exactly one until stand resets it.
    All earlier analytic clocks retain the original independent calculation.
    """
    stream=iter(rows);previous=entry;checks={};count=settle_count=stand_count=0;bindings={}
    def check(name,value):checks[name]=checks.get(name,True)and bool(value)
    for note in notes:
        count+=1;r=note['referenceContext'];c=r['config'];ca=r['causal'];phase=note['phase'];index=note['stepIndex']
        while previous['sourceStep']<r['source_step']:previous=next(stream)
        check('ownershipBoundToPrecedingNativeSample',previous['sourceStep']==r['source_step'] and core.close(previous['sourceTimeS'],r['source_time']))
        correction=ca['robustCorrection'];elapsed=r['elapsed'];original=np.zeros(3)
        if correction['selected']:
            if phase=='transfer':original=core.clock(elapsed-correction['startTransferS'],correction['durationS'])
            elif phase=='settle':
                s,v,a=core.clock(elapsed,c['phase_durations']['settle']);original=np.array([1-s,-v,-a])
            elif phase!='stand':original=np.array([1.,0.,0.])
        active=c.get('coupledPosture');owned=active is not None and phase=='settle'
        expected=policy.correction_clock(note,original)
        if owned:
            settle_count+=1
            check('settleDiagnosticClockExactlyConstant',correction['clock']==[1.,0.,0.]
                and note['robustSupportAdmission']['correctionClock']==[1.,0.,0.])
            for name,value in policy.settle_checks(r,previous,None).items():check(name,value)
            check('completeSettleOnlyIndependentBoundedPath',core.jet_matches(r['com'],policy.segments[index]['expected'](r['source_time']),3))
            target=np.asarray(policy.static[index]['candidate']['target']['com']['position'])
            delta=np.asarray(correction['deltaM']);path=ca['continuousCom']['path']
            start=policy.settles[index]['path']['startSourceTimeS'];advance=core.segment_jet(path,start)
            check('completedOffsetInSettleOriginExactlyOnce',core.equal(np.asarray(c['com1'])+delta,target)
                and core.equal(policy.settles[index]['exactInitialJet'][0],target+advance[0])
                and core.equal(active['completedBaselineCorrectionM'],delta))
            binding=dict(delta=correction['deltaM'],start=correction['startTransferS'],duration=correction['durationS'])
            check('completedCorrectionIdentityNeverChanges',index not in bindings or core.equal(bindings[index],binding))
            bindings.setdefault(index,binding)
        else:
            check('allEarlierOriginalAnalyticClockGatesUnchanged',core.equal(correction['clock'],original)
                and core.equal(note['robustSupportAdmission']['correctionClock'],original))
        check('newProfileClockCallbackMatchesIndependentPolicy',core.equal(correction['clock'],expected))
        if active is not None and phase=='stand':
            stand_count+=1
            check('standClockExactlyZeroAndMeasuredCOMRest',correction['clock']==[0.,0.,0.]
                and note['robustSupportAdmission']['correctionClock']==[0.,0.,0.]
                and core.jet_matches(r['com'],(np.asarray(c['finalCom']),np.zeros(3),np.zeros(3)),3))
    check('nonemptyOwnedSettleAndFinalRest',count>0 and settle_count>0 and stand_count>0)
    return dict(passed=all(checks.values()),checks=checks,failedChecks=[k for k,v in checks.items()if not v],
        actualControlNotes=count,ownedSettleCommands=settle_count,standCommands=stand_count,
        semantics='The complete bounded settle owns COM; completed baseline offset enters its initial jet once. Retained robust clock is constant diagnostic-only state; no faded delta is separately applied.',
        physicsIntegrations=0,nativeReplays=0)


def audit(folder):
    return core.audit(folder,reference_policy=PosturePolicy(folder))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.folder);args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k]for k in ('coupledPosturePilotPhysicalPass','coupledPostureDiagnosticPass','failedChecks')},indent=2))
    raise SystemExit(0 if result['coupledPosturePilotPhysicalPass']or result['coupledPostureDiagnosticPass']else 1)

if __name__=='__main__':main()
