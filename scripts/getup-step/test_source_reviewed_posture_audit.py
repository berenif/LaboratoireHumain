"""Narrow fresh-history/profile corruption tests. No native world or physics."""
import copy
from collections import deque
from unittest.mock import patch
import unittest
import numpy as np

from audit_source_reviewed_posture import PosturePolicy, PROFILE, POSTURE_SOURCES, audit_clock_ownership


def history_fixture():
    p=PosturePolicy.__new__(PosturePolicy);p.mass=10.;p.native=None;p.checks={};p.history={};p.admitted={}
    p.static={1:dict(candidateKey='a'*64)}
    p.manifests={1:dict(profile=PROFILE,originalAdmissionInherited=False,originalCorrectionCompleteBeforeActivation=True,
        physicsIntegrations=0,candidateKey='a'*64,historySourceStartS=4.,historySourceEndS=4.1,historySamples=21,
        historyPredictedRiskCorners=[0]*21,riskAgreementFrequency=1.,survivingCorners=[1,2,3],
        selectedAtTransferS=4.1,allSurvivorsActuallySupportedThroughoutHistory=True)}
    p.replay_window=lambda *args:dict(passed=True)
    p.coverage_window=lambda name,index,ids:ids==list(range(3200,3281,4))
    return p


def sample(n, admitted=False, load=100.):
    active=dict(provisionalSurvivors=[1,2,3],plan=dict(com=dict(start=4.,duration=1.52)))
    row=dict(sourceStep=3200+4*n,sourceTimeS=4.+.005*n,feet=[dict(loadN=load),dict(loadN=0.)],jointLimitMarginsRad=[.1]*21)
    note=dict(stepIndex=1,phase='transfer',referenceContext=dict(config=dict(coupledPosture=active)),
        robustSupportAdmission=dict(currentPostureUnloadForecast={}),coupledPosture=dict(freshPostureAdmission=admitted))
    return note,row,dict(stance_side=0)


class FreshHistoryTests(unittest.TestCase):
    def observe(self,p,n,admitted=False,load=100.,risk=0,present=None):
        note,row,ref=sample(n,admitted,load)
        with patch('audit_source_reviewed_posture.forecast_math',return_value=dict(checks={'arithmetic':True},risk=risk)),patch(
                'audit_source_reviewed_posture.support_observation',return_value={'present':np.ones(4,dtype=bool)if present is None else np.array(present)}):
            return p.observe(note,row,ref,4.+n*.005,None)

    def test_complete_new_21_commands_required(self):
        p=history_fixture()
        for n in range(20):self.assertTrue(all(self.observe(p,n).values()))
        self.assertNotIn(1,p.admitted)
        self.assertTrue(all(self.observe(p,20,True).values()));self.assertTrue(p.admitted[1])

    def test_inherited_admission_cannot_skip_new_history(self):
        p=history_fixture();proof=self.observe(p,0,True)
        self.assertFalse(proof['postureAdmissionAtFirstFreshEligibleCommand']);self.assertNotIn(1,p.admitted)

    def test_load_loss_resets_history(self):
        p=history_fixture()
        for n in range(15):self.observe(p,n)
        self.observe(p,15,load=0.)
        self.assertEqual(len(p.history[1]),0)
        self.assertFalse(self.observe(p,20,True)['postureAdmissionAtFirstFreshEligibleCommand'])

    def test_gap_resets_history(self):
        p=history_fixture()
        for n in range(10):self.observe(p,n)
        self.observe(p,12)
        self.assertEqual(len(p.history[1]),1)

    def test_disagreement_or_wrong_triangle_never_admits(self):
        for risk in (-1,1):
            p=history_fixture()
            for n in range(21):proof=self.observe(p,n,n==20,risk=risk)
            self.assertFalse(proof['postureAdmissionAtFirstFreshEligibleCommand']);self.assertNotIn(1,p.admitted)

    def test_missing_actual_survivor_never_admits(self):
        p=history_fixture()
        for n in range(21):proof=self.observe(p,n,n==20,present=[True,True,False,True])
        self.assertFalse(proof['postureAdmissionAtFirstFreshEligibleCommand'])

    def test_retained_native_window_missing_fails(self):
        p=history_fixture();p.coverage_window=lambda *args:False
        for n in range(21):proof=self.observe(p,n,n==20)
        self.assertFalse(proof['completeUniquePostureNativeWindow'])

    def test_manifest_profile_or_history_mutation_fails(self):
        for key,value in [('profile','five-reviewed-walk'),('historySamples',20),('originalAdmissionInherited',True),('candidateKey','b'*64)]:
            p=history_fixture();p.manifests[1][key]=value
            for n in range(21):proof=self.observe(p,n,n==20)
            self.assertFalse(proof['freshPostureManifestMatchesCausalWindow'])

    def test_fresh_admission_cannot_disappear(self):
        p=history_fixture();p.admitted[1]=True
        self.assertFalse(self.observe(p,21,False)['freshPostureAdmissionNeverDisappears'])

    def test_posture_duration_and_original_ankle_gate_remain(self):
        p=history_fixture();note,row,_=sample(21,True)
        row['sourceTimeS']=5.519;self.assertFalse(p.ready(note,row))
        row['sourceTimeS']=5.52;self.assertTrue(p.ready(note,row))
        row['jointLimitMarginsRad'][7]=.03499;self.assertFalse(p.ready(note,row))


class ClockOwnershipTests(unittest.TestCase):
    def fixture(self):
        from types import SimpleNamespace
        target=np.array([.1,.1,.8]);delta=np.array([.001,0.,0.]);path=dict(startSourceTimeS=0.,durationS=1.,coefficients=np.zeros((6,3)).tolist())
        notes=[];rows=[]
        for i,(phase,now,elapsed)in enumerate((('settle',1.1,.1),('stand',2.52,0.))):
            clock=[1.,0.,0.]if phase=='settle'else[0.,0.,0.]
            active=dict(completedBaselineCorrectionM=delta.tolist())
            c=dict(com1=(target-delta).tolist(),finalCom=target.tolist(),coupledPosture=active,phase_durations=dict(settle=1.52))
            ca=dict(robustCorrection=dict(selected=True,deltaM=delta.tolist(),startTransferS=0.,durationS=1.,clock=clock),continuousCom=dict(path=path))
            com=dict(position=target.tolist(),velocity=[0.]*3,acceleration=[0.]*3)
            r=dict(source_step=i,source_time=now,elapsed=elapsed,config=c,causal=ca,com=com)
            notes.append(dict(phase=phase,stepIndex=1,referenceContext=r,robustSupportAdmission=dict(correctionClock=clock)))
            rows.append(dict(sourceStep=i,sourceTimeS=now))
        p=PosturePolicy.__new__(PosturePolicy)
        p.static={1:dict(candidate=dict(target=dict(com=dict(position=target.tolist()))))}
        p.settles={1:dict(path=dict(startSourceTimeS=1.),exactInitialJet=[target.tolist(),[0.]*3,[0.]*3])}
        p.segments={1:dict(expected=lambda now:(target,np.zeros(3),np.zeros(3)))}
        p.settle_checks=lambda *args:dict(independentBoundedPath=True)
        return notes,rows,p

    def test_owned_constant_clock_and_zero_stand(self):
        notes,rows,p=self.fixture();self.assertTrue(audit_clock_ownership(notes,rows[1:],rows[0],p)['passed'])

    def test_faded_clock_double_offset_or_nonrest_stand_fail(self):
        for mutation in ('fade','double','stand'):
            notes,rows,p=self.fixture()
            if mutation=='fade':notes[0]['referenceContext']['causal']['robustCorrection']['clock']=[.9,-.1,.1]
            elif mutation=='double':notes[0]['referenceContext']['com']['position'][0]+=.001
            else:notes[1]['referenceContext']['com']['velocity'][0]=.001
            self.assertFalse(audit_clock_ownership(notes,rows[1:],rows[0],p)['passed'])


class DeclarationTests(unittest.TestCase):
    def fixture(self):
        c=dict(profile=PROFILE,requestedSteps=1,coupledPostureEnabled=True,inheritedPhysicalGatesUnchanged=True,
            minimumPostureAnkleReadyRad=.035,oldProfileQualificationClaim=False,finalPlanQualificationClaim=False,
            codeSHA256AtStart=dict.fromkeys(POSTURE_SOURCES,'a'*64))
        return c,copy.deepcopy(c),dict(reviewedReferenceProfile=dict(featureProfile=PROFILE))

    def test_settle_clock_ownership_is_feature_only(self):
        p=PosturePolicy.__new__(PosturePolicy);old=(.4,-.2,.1)
        note=dict(phase='settle',referenceContext=dict(config=dict(coupledPosture=dict(profile=PROFILE))))
        self.assertEqual(p.correction_clock(note,old),(1.,0.,0.))
        note['phase']='transfer';self.assertEqual(p.correction_clock(note,old),old)
        note['phase']='settle';note['referenceContext']['config'].pop('coupledPosture')
        self.assertEqual(p.correction_clock(note,old),old)

    def test_feature_diagnostic_is_explicit_and_default_uses_original(self):
        p=PosturePolicy.__new__(PosturePolicy)
        receipt=dict(config=dict(coupledPosture={'profile':PROFILE}),diagnostics=dict(coupledPosture={'fresh':True}),
            causal=dict(continuousCom=dict(diagnostic={'baseline':True})))
        self.assertEqual(p.reference_diagnostic(receipt),{'fresh':True})
        receipt['config'].pop('coupledPosture')
        self.assertEqual(p.reference_diagnostic(receipt),{'baseline':True})

    def test_explicit_profile_and_source_policy(self):
        p=PosturePolicy.__new__(PosturePolicy);c,r,k=self.fixture();self.assertTrue(p.declarations(c,r,k))
        for key,value in [('profile','five-reviewed-walk'),('minimumPostureAnkleReadyRad',.034),('oldProfileQualificationClaim',True),('coupledPostureEnabled',False),('requestedSteps',10)]:
            changed=copy.deepcopy(c);changed[key]=value;self.assertFalse(p.declarations(changed,r,k))
        r['codeSHA256AtStart'].pop('source_bounded_com_settle.py');self.assertFalse(p.declarations(c,r,k))


if __name__=='__main__':unittest.main()
