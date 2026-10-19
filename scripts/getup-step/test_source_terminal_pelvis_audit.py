"""Pure saved-data corruption tests; never integrates a gait or a native world."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import audit_source_reviewed_walk as core
from audit_source_terminal_pelvis import (ACTIVATION_FIELDS, DURATION, MAX_ANGLE_RAD,
    PROFILE, THRESHOLDS, PelvisFK, TerminalPelvisPolicy, active_context_checks,
    audit_activation, audit_terminal_journal, terminal_geometry, terminal_orientation,
    terminal_policy)
from test_source_reviewed_walk_audit import fixture, rehash


def activation_fixture():
    R = Rotation.from_euler('ZYX', [.4, .3, -.1]).as_matrix()
    yaw, upright, axis, endpoint = terminal_geometry(R)
    history = [dict(sourceStep=sid, sourceTimeS=sid*core.DT, feetLoadN=[100., 100.])
        for sid in range(80, 161)]
    landing = dict(index=1, landingNativeStep=80, landingTimeS=.1,
        requestedDistanceM=.18, achievedAtLoadAcquisitionM=.181,
        actualLandingConsecutiveSamples=80, actualLandingFeetLoadN=[100., 100.])
    a = dict(schema=1, profile=PROFILE, body=4, stepIndex=1, finalStepIndex=1,
        sourceStep=160, sourceTimeS=.2, standEntry=True, initialRotationSource=R.tolist(),
        yawRad=yaw, uprightRotationSource=upright.tolist(), axisWorldRad=axis.tolist(),
        boundedEndpointRotationSource=endpoint.tolist(), correctionAngleRad=float(np.linalg.norm(axis)),
        durationS=4., maximumWeight=.5,
        measuredFinalLanding=dict(sourceStep=80, sourceTimeS=.1, stepIndex=1,
            landingConsecutiveSamples=80, feetLoadN=[100.,100.], forwardM=.181, requestedForwardM=.18),
        loadedHistory=dict(minimumDurationS=.1, minimumEachFootLoadN=3., minimumTotalWeightFraction=.8,
            bodyWeightN=98.1, sourceStartStep=80, sourceEndStep=160,
            sourceStartTimeS=.1, sourceEndTimeS=.2, durationS=.1, samples=history))
    a['activationSHA256'] = core.value_sha(a)
    anchor = dict(sourceStep=160, sourceTimeS=.2, stepIndex=1, phase='settle',
        actualPelvisRotationSource=R.tolist())
    return a, anchor, landing, history, R


def rehash_activation(value):
    value['activationSHA256'] = core.value_sha({k:v for k,v in value.items() if k != 'activationSHA256'})


def terminal_receipt(activation, sid, phase='stand'):
    r = dict(source_step=sid, source_time=sid*core.DT, step_index=1, phase=phase,
        config={}, causal={}, diagnostics={}, orientations=[dict(body=1)])
    if phase == 'stand':
        now = sid*core.DT; elapsed = now-activation['sourceTimeS']
        complete = elapsed >= 4.-1e-10
        active = dict(profile=PROFILE, activation=copy.deepcopy(activation),
            activationSHA256=activation['activationSHA256'], clock=core.clock(elapsed, 4.).tolist(),
            elapsedS=elapsed, complete=complete, endpointHeld=complete,
            orientation=terminal_orientation(activation, now))
        r['causal']['terminalPelvis'] = active
        r['diagnostics']['terminalPelvis'] = copy.deepcopy(active)
        r['orientations'].append(copy.deepcopy(active['orientation']))
    return r


def journal_fixture():
    a, anchor, landing, history, R = activation_fixture()
    end = 7360
    record = {k:v for k,v in landing.items() if not k.startswith('actual')}
    record.update(completedNativeStep=end, quietConsecutiveS=5.)
    summary = dict(steps=[record], actualIntegrations=end, finalQuietS=5., passed=True)
    base = dict(com=[0.,0.,.8], comVelocity=[0.,0.,0.], torsoTiltDeg=1.,
        maximumBodyLinearMps=0., maximumBodyAngularRadps=0., maximumSelfPenetrationM=0.,
        supportPointsXY=[[-.2,-.2],[-.2,.2],[.2,-.2],[.2,.2]],
        feet=[dict(loadN=100.,position=[0.,.1,.027],clearance=0.),
            dict(loadN=100.,position=[0.,-.1,.027],clearance=0.)],
        actualPelvisRotationSource=R.tolist(), stepIndex=1, swingSide=0,
        swingForwardDisplacementM=.181)
    entry = dict(base,sourceStep=0,sourceTimeS=0.,phase='land')
    rows=[]; notes=[]
    for sid in range(1,end+1):
        phase='land' if sid<=80 else 'settle' if sid<=160 else 'stand'
        row=dict(base,sourceStep=sid,sourceTimeS=sid*core.DT,phase=phase)
        eligible=sid>3360
        count=max(0,sid-3360)
        row['terminalPelvis']=dict(profile=PROFILE,enabled=True,activated=phase=='stand',
            activationSHA256=a['activationSHA256'] if phase=='stand' else None,
            activationSourceStep=160 if phase=='stand' else None,
            activationSourceTimeS=.2 if phase=='stand' else None,
            elapsedS=sid*core.DT-.2 if phase=='stand' else None,
            rampComplete=sid>=3360 if phase=='stand' else False,
            quietEligible=eligible,quietCountedSamples=count,
            endpointCommandSourceStep=3360 if eligible else None,
            endpointCommandSourceTimeS=4.2 if eligible else None)
        if phase=='stand':row['quietConsecutiveS']=count*core.DT
        rows.append(row)
        if (sid-1)%4==0:
            source=sid-1
            notes.append(dict(t=source*core.DT,phase=phase,stepIndex=1,
                reviewedWalk=dict(featureProfile=PROFILE),referenceContext=terminal_receipt(a,source,phase)))
    return a,entry,rows,notes,summary,R


class GeometryAndActivation(unittest.TestCase):
    def test_goal_is_same_yaw_and_clipped_not_amplified(self):
        a,_,_,_,R=activation_fixture()
        yaw,upright,axis,end=terminal_geometry(R)
        self.assertAlmostEqual(yaw,.4)
        self.assertAlmostEqual(np.linalg.norm(axis),math.radians(6.))
        np.testing.assert_allclose(upright,Rotation.from_euler('Z',.4).as_matrix(),atol=1e-14)
        small=Rotation.from_euler('ZYX',[.4,.01,-.02]).as_matrix()
        _,goal,axis,end=terminal_geometry(small)
        self.assertLess(np.linalg.norm(axis),MAX_ANGLE_RAD)
        np.testing.assert_allclose(end,goal,atol=1e-14)
    def test_same_quintic_drives_orientation_derivatives_and_weight(self):
        a,*_=activation_fixture()
        for elapsed,scalar in ((0.,0.),(2.,.5),(4.,1.),(20.,1.)):
            with self.subTest(elapsed=elapsed):
                o=terminal_orientation(a,a['sourceTimeS']+elapsed)
                self.assertEqual(o['body'],4);self.assertAlmostEqual(o['weight'],.5*scalar)
                axis=np.asarray(a['axisWorldRad']);clock=core.clock(elapsed,4.)
                np.testing.assert_allclose(o['velocity'],axis*clock[1],atol=1e-15)
                np.testing.assert_allclose(o['acceleration'],axis*clock[2],atol=1e-15)
        self.assertEqual(terminal_orientation(a,4.2),terminal_orientation(a,20.2))
        with self.assertRaises(ValueError):terminal_orientation(a,.19)
    def test_activation_is_independently_bound(self):
        a,anchor,landing,history,R=activation_fixture()
        result=audit_activation(a,anchor,1,landing,history,10.,lambda row:R)
        self.assertTrue(result['passed'],result)
        self.assertEqual(set(a),ACTIVATION_FIELDS)
    def test_rehashed_activation_corruptions_rejected(self):
        for field,change in (
            ('sourceStep',lambda a:a.__setitem__('sourceStep',159)),
            ('body',lambda a:a.__setitem__('body',1)),
            ('axis',lambda a:a['axisWorldRad'].__setitem__(0,a['axisWorldRad'][0]+.001)),
            ('weight',lambda a:a.__setitem__('maximumWeight',.6)),
            ('load',lambda a:a['loadedHistory']['samples'][0]['feetLoadN'].__setitem__(0,2.)),
            ('gap',lambda a:a['loadedHistory']['samples'][1].__setitem__('sourceStep',82)),
            ('short',lambda a:a['loadedHistory'].__setitem__('durationS',.099)),
            ('landing',lambda a:a['measuredFinalLanding'].__setitem__('landingConsecutiveSamples',81)),
            ('extra',lambda a:a.__setitem__('extra',True))):
            with self.subTest(field=field):
                a,anchor,landing,history,R=activation_fixture();history=copy.deepcopy(history)
                change(a);rehash_activation(a)
                self.assertFalse(audit_activation(a,anchor,1,landing,history,10.,lambda row:R)['passed'])
    def test_actual_anchor_fk_cannot_be_replaced_with_self_consistent_claim(self):
        a,anchor,landing,history,R=activation_fixture()
        changed=Rotation.from_euler('ZYX',[.5,.3,-.1]).as_matrix()
        a['initialRotationSource']=changed.tolist()
        yaw,upright,axis,end=terminal_geometry(changed)
        a.update(yawRad=yaw,uprightRotationSource=upright.tolist(),axisWorldRad=axis.tolist(),
            boundedEndpointRotationSource=end.tolist(),correctionAngleRad=float(np.linalg.norm(axis)))
        rehash_activation(a)
        result=audit_activation(a,anchor,1,landing,history,10.,lambda row:R)
        self.assertFalse(result['checks']['terminalAnchorIsActualBody4FK'])


class ContextScope(unittest.TestCase):
    def setUp(self): self.a,*_=activation_fixture()
    def test_default_profile_rejects_declared_feature_without_policy(self):
        d=fixture();d['note']['reviewedWalk']['featureProfile']=PROFILE
        result=core.audit_context(d['receipt'],d['note'],d['row'],d['reference'])
        self.assertFalse(result['checks']['explicitReferenceFeatureScope'])
    def test_preterminal_no_op_callbacks_preserve_original_reference_owners(self):
        policy=TerminalPelvisPolicy.__new__(TerminalPelvisPolicy);policy.activation=self.a
        d=fixture();r=d['receipt'];expected=core.independent_references(r['config'],r['causal'],0.,'transfer')
        actual=policy.references(r['config'],r['causal'],0.,'transfer',False)
        self.assertTrue(core.equal(actual,expected))
        self.assertIs(policy.bound_reference(d['reference'],r),d['reference'])
        clock=(.2,.3,.4);self.assertIs(policy.correction_clock(d['note'],clock),clock)
        self.assertTrue(policy.ready(d['note'],d['row']));self.assertEqual(policy.settle_checks(r,d['row'],{}),{})
    def test_valid_stand_context_and_changed_clock_weight_anchor_rejected(self):
        r=terminal_receipt(self.a,1760);note=dict(reviewedWalk=dict(featureProfile=PROFILE))
        self.assertTrue(all(active_context_checks(r,note,self.a).values()))
        for key in ('weight','clock','anchor','torso','diagnostic','phase'):
            with self.subTest(key=key):
                value=copy.deepcopy(r)
                if key=='weight':value['orientations'][1]['weight']+=.01
                elif key=='clock':value['causal']['terminalPelvis']['clock'][0]+=.01
                elif key=='anchor':value['causal']['terminalPelvis']['activation']['sourceStep']-=1
                elif key=='torso':value['orientations'][0]['body']=2
                elif key=='diagnostic':value['diagnostics']['terminalPelvis']['endpointHeld']=True
                else:value['phase']='settle'
                self.assertFalse(all(active_context_checks(value,note,self.a).values()))
    def test_overlay_cannot_appear_before_stand(self):
        r=terminal_receipt(self.a,1760);r['phase']='transfer'
        self.assertFalse(active_context_checks(r,dict(reviewedWalk=dict(featureProfile=PROFILE)),self.a)['terminalOverlayOnlyInFinalStand'])


class QuietJournal(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.data=journal_fixture()
    def result(self, rows=None, notes=None, summary=None):
        a,entry,original_rows,original_notes,original_summary,R=self.data
        return audit_terminal_journal(iter(rows if rows is not None else original_rows),
            iter(notes if notes is not None else original_notes),entry,a,
            summary if summary is not None else original_summary,10.,lambda row:R)
    def test_fresh_4000_samples_after_endpoint_command_pass(self):
        result=self.result();self.assertTrue(result['passed'],result['failedChecks'])
        self.assertEqual(result['finalQuietSamples'],4000)
        self.assertEqual(result['firstEndpointCommand']['sourceStep'],3360)
        self.assertEqual(result['physicsIntegrations'],0)
    def test_pre_ramp_quiet_credit_is_rejected(self):
        rows=copy.deepcopy(self.data[2]);rows[3359]['quietConsecutiveS']=.00125
        rows[3359]['terminalPelvis']['quietCountedSamples']=1
        self.assertFalse(self.result(rows=rows)['checks']['freshTerminalQuietCountMatchesEveryNativeSample'])
    def test_3999_samples_is_not_five_seconds(self):
        rows=self.data[2][:-1];summary=copy.deepcopy(self.data[4]);summary.update(actualIntegrations=len(rows),finalQuietS=3999*core.DT)
        summary['steps'][0].update(completedNativeStep=len(rows),quietConsecutiveS=3999*core.DT)
        self.assertFalse(self.result(rows=rows,summary=summary)['checks']['terminalFinal4000NativeSamplesQuietAfterRamp'])
    def test_endpoint_command_must_be_held_before_quiet_can_count(self):
        notes=copy.deepcopy(self.data[3]);notes[3360//4]['referenceContext']['source_time']-=.005
        self.assertFalse(self.result(notes=notes)['checks']['terminalQuietEligibilityAfterHeldEndpointCommand'])
    def test_late_quiet_break_and_tampered_fk_are_rejected(self):
        rows=copy.deepcopy(self.data[2]);rows[-1]['maximumBodyAngularRadps']=.66
        self.assertFalse(self.result(rows=rows)['checks']['terminalFinal4000NativeSamplesQuietAfterRamp'])
        rows=copy.deepcopy(self.data[2]);rows[-1]['actualPelvisRotationSource']=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
        self.assertFalse(self.result(rows=rows)['checks']['everyTerminalActualPelvisRotationMatchesIndependentFK'])


class IndependentFK(unittest.TestCase):
    def test_bound_xml_order_and_axes_reconstruct_body4_without_native(self):
        xml='''<mujoco><worldbody><body name="torso"><freejoint/>
<body name="head"/><body name="lower_waist" quat="1 0 -.002 0">
<joint name="abdomen_z" axis="0 0 1"/><joint name="abdomen_y" axis="0 1 0"/>
<body name="pelvis" quat="1 0 -.002 0"><joint name="abdomen_x" axis="1 0 0"/></body>
</body></body></worldbody></mujoco>'''
        model=dict(segments=[dict(reference=dict(hinge=dict(name=name,qposIndex=i)))
            for i,name in enumerate(('abdomen_z','abdomen_y','abdomen_x'))])
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'model.xml';p.write_text(xml);fk=PelvisFK(p,model)
            q=np.zeros(28);root=Rotation.from_euler('ZYX',[.4,.1,.2]);q[3:7]=root.as_quat()[[3,0,1,2]];q[7:10]=[.3,-.2,.1]
            fixed=Rotation.from_quat([0.,-.002,0.,1.]).as_matrix()
            expected=root.as_matrix()@fixed@Rotation.from_euler('Z',.3).as_matrix()@Rotation.from_euler('Y',-.2).as_matrix()@fixed@Rotation.from_euler('X',.1).as_matrix()
            np.testing.assert_allclose(fk(dict(qpos=q.tolist())),expected,atol=1e-14)


class NativeWitnessProtocol(unittest.TestCase):
    """Mock the isolated import/QP interface; the tests never load a simulator."""
    def setup_witness(self):
        p=TerminalPelvisPolicy.__new__(TerminalPelvisPolicy)
        a,*_=activation_fixture();a['sourceTimeS']=-.5;rehash_activation(a)
        p.activation=a;p.manifest_errors=[];p.seen=set();p.samples={};p.native_results=[]
        p.manifest=dict(activationSHA256=a['activationSHA256'])
        d=fixture();r=d['receipt'];r.update(phase='stand',elapsed=2.)
        planned=core.independent_references(r['config'],r['causal'],2.,'stand')
        for name in ('com','joints'):
            r[name]={key:value.tolist() for key,value in zip(('position','velocity','acceleration'),planned[name])}
        r['feet']=[dict(translation={key:value.tolist() for key,value in zip(('position','velocity','acceleration'),foot[:3])},
            orientation=dict(body=body,rotation=foot[3].reshape(-1).tolist(),velocity=foot[4].tolist(),
                acceleration=foot[5].tolist(),kp=80.,kd=20.,weight=4.))
            for body,foot in zip((7,10),planned['feet'])]
        r['orientations']=planned['orientations']+[terminal_orientation(a,0.)]
        active=dict(profile=PROFILE,activation=copy.deepcopy(a),activationSHA256=a['activationSHA256'],
            clock=core.clock(.5,4.).tolist(),elapsedS=.5,complete=False,endpointHeld=False,
            orientation=copy.deepcopy(r['orientations'][1]))
        r['causal']['terminalPelvis']=active;r['diagnostics']['terminalPelvis']=copy.deepcopy(active);rehash(r)
        note=d['note'];note.update(phase='stand');note['reviewedWalk']['featureProfile']=PROFILE
        row=d['row'];row['qvel']=[0.]*27
        payload=b'exact native bytes';R=np.array(a['initialRotationSource']);p.fk=lambda value:R
        world=SimpleNamespace(h=None,l=SimpleNamespace(lh_checkpoint_size=lambda h:len(payload),
            lh_checkpoint_import=lambda h,buf,n:n,lh_dim=lambda h,k:0,lh_time=lambda h:0.),
            arr=lambda index,shape: np.asarray(row['qpos']) if index==0 else np.zeros(27) if index==1 else np.repeat(R[None],17,axis=0))
        def compare(name,actual,expected,shape,tolerance):
            if not core.close(actual,expected,tolerance):raise ValueError(name+' mismatch')
        p.native=SimpleNamespace(bindings={},_check_sources=lambda:None,_ensure_world=lambda:world,
            _payload=lambda sample,path:payload,_compare=compare)
        sample=dict(thresholdS=.5,sourceStep=0,sourceTimeS=0.,elapsedS=.5,phase='stand',
            contextSHA256=r['contextSHA256'],actualMotorControlSHA256=note['actualMotorControlSHA256'],
            qpAssemblySHA256=copy.deepcopy(note['qpAssemblySHA256']))
        p.samples={0:(sample,Path('/unused'))}
        return p,note,row,d['reference'],payload

    def replay(self,values,solve=None,export=None):
        p,note,row,reference,payload=values
        solve=solve or (lambda *args:(np.zeros(21),dict(qpAssemblySHA256=note['qpAssemblySHA256'])))
        with patch('source_step_probe.Controller.solve',side_effect=solve), \
             patch('source_arm_knee_mobility.native_bytes',side_effect=export or (lambda w:payload)), \
             patch.object(core,'independent_native_constraints',return_value=[]):
            return p.replay_terminal_witness(note,row,reference)

    def test_exact_payload_context_command_assembly_and_zero_integrations_pass(self):
        values=self.setup_witness();result=self.replay(values)
        self.assertTrue(result['passed'],result)
        self.assertTrue(result['nativeBytesCounterTimeUnchanged'])
        self.assertEqual(result['physicsIntegrations'],0)
        self.assertEqual(values[0].seen,{0})
    def test_changed_orientation_command_digest_assembly_and_threshold_fail(self):
        for defect in ('orientation','command','assembly','threshold','activation'):
            with self.subTest(defect=defect):
                values=self.setup_witness();p,note,row,reference,payload=values
                if defect=='orientation':note['referenceContext']['orientations'][1]['weight']+=.01
                elif defect=='command':note['actualMotorControl'][0]=.001
                elif defect=='assembly':p.samples[0][0]['qpAssemblySHA256']={'P':'wrong'}
                elif defect=='threshold':p.samples[0][0]['thresholdS']=.1
                else:p.manifest['activationSHA256']='0'*64
                self.assertFalse(self.replay(values)['passed'])
                self.assertEqual(p.seen,set())
    def test_native_bytes_must_remain_identical_after_qp(self):
        values=self.setup_witness();state={'solved':False};payload=values[-1]
        def solve(*args):
            state['solved']=True
            return np.zeros(21),dict(qpAssemblySHA256=values[1]['qpAssemblySHA256'])
        result=self.replay(values,solve=solve,export=lambda w:b'changed' if state['solved'] else payload)
        self.assertFalse(result['passed']);self.assertIn('changed native bytes',result['error'])
    def test_replay_is_once_per_retained_source(self):
        values=self.setup_witness();self.assertTrue(self.replay(values)['passed'])
        result=self.replay(values);self.assertFalse(result['passed']);self.assertIn('duplicated',result['error'])


class Qualification(unittest.TestCase):
    def policy(self):
        p=TerminalPelvisPolicy.__new__(TerminalPelvisPolicy);p.checks={};p.input_hashes={'evidence':'hash'}
        p.folder=Path('/unused');p.journal=dict(passed=True);p.manifest_errors=[]
        p.samples={1:None,2:None,3:None};p.seen={1,2,3};p.native_results=[dict(passed=True)]*3
        return p
    def result(self):
        return dict(checks=dict(physical=True),requestedSteps=1,hashes={},evidenceLimit='')
    def test_all_old_and_final_flags_stay_false_with_truthful_separate_pass(self):
        p=self.policy()
        with patch.object(Path,'is_file',return_value=True),patch.object(core,'file_sha',return_value='hash'):
            result=p.finish(self.result())
        self.assertTrue(result['terminalPelvisPilotPhysicalPass'])
        self.assertTrue(result['terminalPelvisReferenceAndQuietPass'])
        for key in ('reviewedWalkPilotPhysicalPass','reviewedWalkTenStepPhysicalPass',
            'reviewedWalkFinalPlanPass','terminalPelvisFinalPlanPass','finalVisualComparisonIndependentlyAudited'):
            self.assertIs(result[key],False,key)
    def test_missing_changed_task_witness_defers_physical_pass(self):
        p=self.policy();p.seen={1,2}
        with patch.object(Path,'is_file',return_value=True),patch.object(core,'file_sha',return_value='hash'):
            result=p.finish(self.result())
        self.assertFalse(result['terminalPelvisPilotPhysicalPass'])
        self.assertFalse(result['terminalNativeWitnessReplay']['passed'])
        self.assertEqual(result['terminalNativeWitnessReplay']['verifiedWitnesses'],2)
    def test_unrelated_core_physical_failure_cannot_be_overridden(self):
        p=self.policy();value=self.result();value['checks']['physical']=False
        with patch.object(Path,'is_file',return_value=True),patch.object(core,'file_sha',return_value='hash'):
            result=p.finish(value)
        self.assertFalse(result['terminalPelvisPilotPhysicalPass'])
    def test_audit_always_calls_full_core_with_explicit_policy(self):
        with patch('audit_source_terminal_pelvis.TerminalPelvisPolicy',return_value='policy'), \
             patch.object(core,'audit',return_value={'full':'proof'}) as call:
            from audit_source_terminal_pelvis import audit
            self.assertEqual(audit('/tmp/evidence'),{'full':'proof'})
            call.assert_called_once_with('/tmp/evidence',reference_policy='policy')


if __name__=='__main__':unittest.main()
