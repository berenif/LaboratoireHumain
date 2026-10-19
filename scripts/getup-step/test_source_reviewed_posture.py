"""Focused pure/mock posture tests: no optimizer, world, solve, or integration."""
from collections import deque
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation
import source_reviewed_posture as p
import source_reviewed_walk as r
from source_posture_retarget import RetargetResult
from source_reference_context import ReferenceContext,Jet as CJ,Orientation,canonical
from coupled_posture_reference import Jet,OrientationJet,CoupledSample,Candidate,CoupledPlan,digest
from coupled_posture_kinematics import ModelFK,pose_jet_from_source,pose_jet_to_source

MODEL=Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json'
def pelvis(lower,R):
    model=ModelFK.from_model(json.loads(MODEL.read_text()))
    root=pose_jet_from_source(Jet.rest([0.,0.,0.]),R,np.zeros(3),np.zeros(3))
    return np.array(pose_jet_to_source(model.forward(Jet.rest(np.r_[lower,np.zeros(6)]),root)['pelvis']).rotation).reshape(3,3)
def fixture():
    q=np.zeros(15);q[[7,13]]=-.98;g=q.copy();g[[7,13]]=-.9
    initial=CoupledSample(Jet.rest([.03,.03,.8]),Jet.rest(q),OrientationJet.rest(1,np.eye(3)),OrientationJet.rest(4,pelvis(q,np.eye(3))))
    target=CoupledSample(Jet.rest([.032,.03,.8]),Jet.rest(g),OrientationJet.rest(1,Rotation.from_euler('y',.01).as_matrix()),OrientationJet.rest(4,pelvis(g,np.eye(3))))
    return initial,Candidate('pure-test','a'*64,target),np.tile([-1.,1.],(21,1))
def context(initial,now=4.,phase='transfer',elapsed=4.):
    return ReferenceContext(2,'reviewed-reference-v1',round(now/.00125),now,1,phase,elapsed,0,CJ.make(*initial.com.arrays()),
        CJ.make(*(np.r_[a,np.zeros(6)] for a in initial.lower_joints.arrays())),CJ.make(np.zeros(6),np.zeros(6),np.zeros(6)),
        (100.,20.,2.),(Orientation.make(1,*initial.thorax.arrays()),),(),(),False,canonical({}),
        canonical(dict(com1=list(initial.com.position),mode='continuous-com')),canonical({}))

class PostureReferences(unittest.TestCase):
    def controller(self):
        initial,candidate,limits=fixture();state=SimpleNamespace(now=4.)
        points=np.array([[-.1,-.1,0.],[-.1,.1,0.],[.1,-.1,0.],[.1,.1,0.]])
        def arr(k,s):
            a=np.zeros(s)
            if k==21:a[:8]=points[:,:2].reshape(-1)
            return a
        c=p.PostureController.__new__(p.PostureController)
        c.w=SimpleNamespace(h=None,limits=limits,arr=arr,l=SimpleNamespace(lh_time=lambda h:state.now,
            lh_dim=lambda h,k:round(state.now/.00125),lh_support_count=lambda h:4))
        c.install_reviewed(1,'continuous-com-causal');c.phase_duration=lambda phase:dict(transfer=4.,settle=1.52,lift=.8,translate=1.2,land=1.)[phase]
        c.stance_side=1;c.rootR=np.eye(3);c.q0=np.r_[initial.lower_joints.position,np.zeros(6)];c.com1=np.array(initial.com.position)
        return c,state,initial,candidate,points
    def activate(self,c,initial,candidate,points,directory):
        c.robust_support['selected']=True;c._robust_clock=(1.,0.,0.)
        c._robust_delta=np.array([.001,-.0005,0.]);c._robust_survivors=[1,2,3]
        c._robust_start=2.4;c._robust_duration=1.;c.com0=np.array(initial.com.position)-c._robust_delta
        c._robust_history.append((2.4,0,np.ones(4,dtype=bool),True))
        inputs=dict(initialCoupledReference=asdict(initial));result=RetargetResult(True,candidate,canonical(dict(input=inputs,inputSHA256=digest(inputs))))
        c._posture_evidence_path=Path(directory)/'static.json'
        with patch.object(p,'actual_stance_patch',return_value=(points,np.ones(4,dtype=bool),np.zeros(4))),patch.object(p,'retarget_current_reference',return_value=result),patch.object(p,'validate_reference',return_value={}),patch.object(c,'_forecast_unload_risk',return_value=(0,dict(role='provisional-posture-geometry'))):
            return c.transform_reviewed_reference_context(context(initial))
    def test_inherited_02_to_06_does_not_relax_generic_06_factory(self):
        initial,candidate,limits=fixture()
        with self.assertRaisesRegex(ValueError,'ankle'):CoupledPlan.between(4.,1.52,initial,candidate,joint_limits=limits,lumbar_anchor=[0.]*3)
        plan,receipt=p.inherited_plan(4.,initial,candidate,limits)
        for now,want in ((4.,initial),(5.52,candidate.target)):
            got=plan.at(now)
            self.assertEqual(got.com,want.com);self.assertEqual(got.lower_joints,want.lower_joints)
            for name in ('thorax','pelvis'):
                for a,b in zip(getattr(got,name).arrays(),getattr(want,name).arrays()):assert_allclose(a,b,atol=2e-15)
        assert_allclose(np.array(receipt['wholePathReferenceReserveFloorRad'])[[7,13]],[.02,.02]);self.assertTrue(receipt['genericWholePath06GuardUnchanged'])
    def test_activation_is_c2_and_first_context_binds_new_com1(self):
        c,state,initial,candidate,points=self.controller()
        with tempfile.TemporaryDirectory() as d:out=self.activate(c,initial,candidate,points,d)
        assert_allclose(out.com.arrays(),initial.com.arrays(),atol=1e-14)
        assert_allclose(out.joints.position[:15],initial.lower_joints.position,atol=1e-14)
        assert_allclose(json.loads(out.config_json)['com1'],c.com1,atol=0.)
        self.assertEqual(len(c._robust_history),1);self.assertTrue(c.robust_support['selected'])
        self.assertEqual(len(c._posture_history),0);self.assertFalse(c._posture_admitted)
        assert_allclose(c.com1+c._robust_delta,candidate.target.com.position,atol=1e-14)
        self.assertEqual([o.weight for o in out.orientations if o.body==4],[0.])
    def test_baseline_observer_retained_until_posture_activation(self):
        c,*_=self.controller()
        with patch.object(r.ReviewedController,'_observe_support') as observe:
            c._observe_support(2.4,'transfer');observe.assert_called_once()
            c._posture_plan=object();c._posture_admitted=True
            c._observe_support(4.1,'transfer');observe.assert_called_once()
    def test_completed_baseline_offset_is_embedded_once(self):
        c,state,initial,candidate,points=self.controller()
        with tempfile.TemporaryDirectory() as d:self.activate(c,initial,candidate,points,d)
        state.now=4.6
        for actual,base in zip(c.corrected_com_reference(4.6,'transfer'),c._posture_plan.com.at(4.6).arrays()):
            assert_allclose(actual,base,atol=1e-14)
        self.assertEqual(c._robust_clock,(1.,0.,0.))
        state.now=5.52
        lift=c.corrected_com_reference(0.,'lift')
        assert_allclose(lift[0],candidate.target.com.position,atol=1e-14)
        assert_allclose(lift[1:],np.zeros((2,3)),atol=1e-14)
    def test_fresh_admission_and_completion_both_precede_original_lift_gate(self):
        c,state,initial,candidate,_=self.controller();c._posture_plan,_=p.inherited_plan(4.,initial,candidate,c.w.limits)
        with patch.object(r.ReviewedController,'robust_support_ready',return_value=True) as old:
            state.now=5.52;self.assertFalse(c.robust_support_ready({}));old.assert_not_called()
            c._posture_admitted=True;state.now=5.;self.assertFalse(c.robust_support_ready({}));old.assert_not_called()
            state.now=5.52;self.assertTrue(c.robust_support_ready({}));old.assert_called_once()
    def test_no_new_correction_after_activation(self):
        c,state,initial,candidate,_=self.controller();c._posture_plan,_=p.inherited_plan(4.,initial,candidate,c.w.limits)
        with self.assertRaisesRegex(ValueError,'cannot be retargeted'):c.admit_robust_reference(4.1,np.zeros(3),1.)
    def test_activation_requires_completed_baseline_correction(self):
        c,state,initial,candidate,points=self.controller()
        with patch.object(p,'retarget_current_reference',side_effect=AssertionError('premature retarget')):
            raw=context(initial);self.assertIs(c.transform_reviewed_reference_context(raw),raw)
            c.robust_support['selected']=True;c._robust_clock=(.9,.1,.1)
            self.assertIs(c.transform_reviewed_reference_context(raw),raw)
    def test_settle_return_c2_and_joint_pelvis_fk_consistency(self):
        c,state,initial,candidate,points=self.controller()
        with tempfile.TemporaryDirectory() as d:self.activate(c,initial,candidate,points,d)
        c.q0[0]=.1;c.robust_support['selected']=True
        c.corrected_com_reference=lambda t,phase:(np.array([.03,.03,.8]),np.zeros(3),np.zeros(3))
        with patch.object(p,'actual_stance_patch',return_value=(points,np.ones(4,dtype=bool),np.zeros(4))),patch.object(p,'validate_reference',return_value={}):
            state.now=10.;a=c.transform_reviewed_reference_context(context(initial,10.,'settle',0.))
            assert_allclose(a.joints.position[:15],candidate.target.lower_joints.position,atol=1e-13)
            for o in a.orientations:
                if o.body in (1,4):assert_allclose(np.array(o.rotation).reshape(3,3),getattr(candidate.target,'thorax' if o.body==1 else 'pelvis').arrays()[0],atol=1e-12)
            state.now=11.52;b=c.transform_reviewed_reference_context(context(initial,11.52,'stand',0.))
        o=next(o for o in b.orientations if o.body==4)
        assert_allclose(np.array(o.rotation).reshape(3,3),pelvis(np.array(b.joints.position[:15]),c.rootR),atol=1e-12)
        for o in b.orientations:
            assert_allclose(o.velocity,[0.]*3,atol=1e-14);assert_allclose(o.acceleration,[0.]*3,atol=1e-14)

class RoleAndProfile(unittest.TestCase):
    def test_provisional_forecast_cannot_enter_fresh_native_witness_history(self):
        c=r.ReviewedController.__new__(r.ReviewedController);response=lambda *a:None
        c.w=SimpleNamespace(h=None,l=SimpleNamespace(lh_dim=lambda h,k:3200,lh_time=lambda h:4.,lh_motor_response=response),
            arr=lambda i,s:np.zeros(s),read=lambda name,shape,*args:np.zeros(shape),feet=lambda:[dict(body=1),dict(body=2)])
        c.install_reviewed(1,'continuous-com');c.stance_side=1;c.arm_posture=c.arm_velocity=c.arm_acceleration=np.zeros(6)
        c._mapping=SimpleNamespace(ranges=np.zeros((2,2)));ctx=SimpleNamespace(receipt=lambda:dict(contextSHA256='a'*64));c._prepared_reference_context=ctx
        forces=np.zeros((8,3));forces[:,2]=[1.,2.,3.,4.]*2
        with patch.object(r,'native_bytes',return_value=b'fixed native bytes'),patch.object(r,'prospective_unload_context',return_value=ctx),patch.object(r.Controller,'solve',return_value=(np.zeros(21),dict(predictedForcesN=forces.tolist(),qpAssemblySHA256={}))):
            c._forecast_unload_risk(4.,np.zeros((4,3)),np.array([.001,0.,0.,0.]),role='provisional-posture-geometry',reference_context=ctx)
            self.assertEqual(len(c._robust_payloads),0)
            c._forecast_unload_risk(4.,np.zeros((4,3)),np.array([.001,0.,0.,0.]))
            self.assertEqual(len(c._robust_payloads),1);self.assertEqual(c._robust_payloads[0]['sourceStep'],3200)
    def test_saved_feature_profile_is_separate_and_unqualified(self):
        from audit_source_reviewed_walk import PROFILE as qualified
        live=p.PostureSteps.__new__(p.PostureSteps)
        with tempfile.TemporaryDirectory() as d:
            live.folder=Path(d)
            def save(live,reason):
                for name in ('reviewed-walk-config.json','reviewed-walk-run.json'):(live.folder/name).write_text(canonical(dict(profile=qualified)))
                (live.folder/'cadenced-step-config.json').write_text(canonical(dict(reviewedReferenceProfile={})))
            with patch.object(r.ReviewedSteps,'save',save):live.save('synthetic')
            for name in ('reviewed-walk-config.json','reviewed-walk-run.json'):
                data=json.loads((live.folder/name).read_text());self.assertEqual(data['profile'],p.PROFILE);self.assertNotEqual(data['profile'],qualified)
                self.assertFalse(data['finalPlanQualificationClaim']);self.assertFalse(data['oldProfileQualificationClaim'])


class FreshPostureHistory(unittest.TestCase):
    """Synthetic read-only observer witnesses, never native physics evidence."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.c,self.state,initial,candidate,self.points=PostureReferences().controller()
        c=self.c;c._posture_plan,_=p.inherited_plan(4.,initial,candidate,c.w.limits)
        c._posture_provisional=(1,2,3);c._posture_evidence_path=self.root/'static.json'
        c.robust_support.update(selected=True,survivingCorners=[1,2,3],historySourceStartS=2.3,historySamples=21)
        c._robust_survivors=[1,2,3];c._robust_delta=np.array([.001,0.,0.]);self.original_delta=c._robust_delta.copy()
        c._robust_history.append(('old baseline history',));c._robust_payloads=deque(maxlen=22)
        c._robust_payloads.append(dict(sourceStep=2000,sourceTimeS=2.5,payloadSHA256='a'*64,payloadBytes=3,compressed=b'old'))
        c.w.mass=10.;c.w.feet=lambda:[dict(body=7),dict(body=10)]
        old_arr=c.w.arr;self.loaded=True
        def arr(k,shape):
            if k==20:
                a=np.zeros(shape);a[10]=100. if self.loaded else 20.;return a
            return old_arr(k,shape)
        c.w.arr=arr;c.w.base=self.root;c.w.xml=self.root/'model.xml';c.w.helper=self.root/'helper.so'
        for path in (c.w.xml,c.w.helper,self.root/'source_bridge_v2.so',self.root/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'):
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'synthetic identity only')
        self.risk=0;self.present=np.ones(4,dtype=bool);self.omit=False
        def forecast(t,corners,clearance):
            if not self.omit:c._robust_payloads.append(dict(sourceStep=round(self.state.now/.00125),sourceTimeS=self.state.now,payloadSHA256='b'*64,payloadBytes=3,compressed=b'new'))
            return self.risk,dict(role='causal-observer',candidateKey=c._posture_plan.candidate.key)
        c._forecast_unload_risk=forecast
    def observe(self,i):
        self.state.now=4.+i*.005
        with patch.object(p,'actual_stance_patch',return_value=(self.points,self.present.copy(),np.zeros(4))):self.c._observe_support(self.state.now,'transfer')
    def test_fresh_21_commands_not_old_selection_authorize_lift(self):
        for i in range(20):self.observe(i);self.assertFalse(self.c._posture_admitted)
        self.observe(20);self.assertTrue(self.c._posture_admitted)
        admission=self.c.robust_support['postureFreshAdmission'];self.assertFalse(admission['originalAdmissionInherited'])
        self.assertEqual(admission['historySamples'],21);self.assertEqual(admission['historySourceStartS'],4.)
        self.assertEqual(len(admission['samples']),21);self.assertEqual(len({w['sourceStep'] for w in admission['samples']}),21)
        self.assertTrue(all(w['sourceStep']>=3200 for w in admission['samples']))
        self.assertEqual(list(self.c._robust_history),[('old baseline history',)])
        assert_allclose(self.c._robust_delta,self.original_delta,atol=0.)
    def test_actual_unloading_resets_changed_task_history(self):
        for i in range(10):self.observe(i)
        self.loaded=False;self.observe(10);self.assertEqual(len(self.c._posture_history),0)
        self.loaded=True
        for i in range(11,31):self.observe(i);self.assertFalse(self.c._posture_admitted)
        self.observe(31);self.assertTrue(self.c._posture_admitted)
        self.assertEqual(self.c.robust_support['postureFreshAdmission']['historySourceStartS'],4.055)
    def test_gap_resets_history(self):
        for i in range(10):self.observe(i)
        self.observe(20);self.assertEqual(len(self.c._posture_history),1);self.assertFalse(self.c._posture_admitted)
    def test_risk_disagreement_and_triangle_mismatch_never_inherit_old_admission(self):
        for i in range(21):
            self.risk=-1 if i<5 else 0;self.observe(i)
        self.assertFalse(self.c._posture_admitted)
        self.c._posture_history.clear()
        for i in range(21,42):self.risk=1;self.observe(i)
        self.assertFalse(self.c._posture_admitted);self.assertEqual(self.c._robust_survivors,[1,2,3])
    def test_actual_survivor_loss_blocks_new_admission(self):
        for i in range(21):
            self.present[:]=True
            if i==10:self.present[2]=False
            self.observe(i)
        self.assertFalse(self.c._posture_admitted)
    def test_missing_unique_native_witness_coverage_fails_closed(self):
        for i in range(20):
            self.omit=i==10;self.observe(i)
        self.omit=False
        with self.assertRaisesRegex(ValueError,'unique exact native witnesses'):self.observe(20)
        self.assertFalse(self.c._posture_admitted)



class BoundedSettleIntegration(unittest.TestCase):
    """Exercise the real pure planner/validator through the controller seam."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.c,self.state,self.initial,self.candidate,_=PostureReferences().controller()
        c=self.c;c._posture_plan=object();c._posture_admitted=True
        c._robust_delta=np.array([.001,-.0005,0.]);c._posture_baseline_delta=c._robust_delta.copy()
        c.robust_support['selected']=True;c._robust_clock=(1.,0.,0.)
        c.com0=np.array(self.initial.com.position)-c._robust_delta
        c.com1=np.array(self.candidate.target.com.position)-c._robust_delta
        self.offset=np.array([.010,-.0025,0.]);self.incoming_v=np.zeros(3)
        c._com_path=SimpleNamespace(evaluate=lambda start:(self.offset.copy(),self.incoming_v.copy(),np.zeros(3)))
        self.complete_start=np.array(self.candidate.target.com.position)+self.offset
        c.finalCom=self.complete_start+[-.18192656090102183,0.,0.]
        self.measured_target=c.finalCom.copy();self.hull=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
        def arr(k,shape):
            out=np.zeros(shape)
            if k==21:out[:8]=self.hull.reshape(-1)
            return out
        c.w.arr=arr;c.w.xml=self.root/'model.xml';c.w.helper=self.root/'helper.so'
        for path in (c.w.xml,c.w.helper):path.write_bytes(b'synthetic source identity only')
        c._posture_evidence_path=self.root/'posture-static-step-01.json'
        self.state.now=10.00375
    def evaluate(self,now):
        self.state.now=now
        return self.c.corrected_com_reference(now-10.,'settle')
    def test_complete_offset_once_exact_target_duration_and_certificate_bound_once(self):
        with patch.object(p,'build_bounded_settle',wraps=p.build_bounded_settle) as build,patch.object(p,'validate_reference',wraps=p.validate_reference) as live:
            self.evaluate(10.00375);receipt_path=self.root/'posture-settle-step-01.json';frozen=receipt_path.read_bytes()
            self.evaluate(10.2);self.evaluate(11.3)
        self.assertEqual(build.call_count,1);self.assertEqual(live.call_count,3)
        args=build.call_args.args;self.assertEqual(args[0],10.);self.assertEqual(args[1],1.52)
        assert_allclose(args[2][0],self.complete_start,atol=1e-16)
        assert_allclose(args[2][1:],np.zeros((2,3)),atol=0.)
        assert_allclose(args[3][0],self.measured_target,atol=0.)
        self.assertIsNot(args[3][0],self.c.finalCom)
        assert_allclose(args[4],self.hull,atol=0.)
        self.assertEqual(receipt_path.read_bytes(),frozen)
        self.assertEqual(self.c._bounded_settle_receipt_sha,p.sha(receipt_path))
        saved=json.loads(frozen);self.assertEqual(saved,self.c._bounded_settle_receipt)
        self.assertEqual(saved['profile'],'seven-cubic-constant-jerk')
        self.assertTrue(saved['durationUnchanged']);self.assertTrue(saved['measuredTargetUnchanged']);self.assertTrue(saved['frozenCapsUnchanged'])
        self.assertFalse(saved['certificate']['futureSupportClaim'])
        self.assertEqual(saved['plannedAtNativeStep'],round(10.00375/.00125))
    def test_live_changed_hull_rejects_without_replanning_or_projection(self):
        with patch.object(p,'build_bounded_settle',wraps=p.build_bounded_settle) as build:
            self.evaluate(10.00375);original=self.c._com_settle
            self.hull[:]=[[2.,2.],[3.,2.],[3.,3.],[2.,3.]]
            with self.assertRaisesRegex(ValueError,'current support inset'):self.evaluate(10.2)
            self.assertEqual(build.call_count,1);self.assertIs(self.c._com_settle,original)
    def test_settle_to_stand_has_unchanged_measured_position_and_zero_jets(self):
        self.evaluate(10.00375)
        at_end=self.evaluate(11.52)
        self.state.now=11.52;stand=self.c.corrected_com_reference(0.,'stand')
        for got in (at_end,stand):
            assert_allclose(got[0],self.measured_target,atol=2e-16,rtol=0.)
            assert_allclose(got[1:],np.zeros((2,3)),atol=0.)
        before=self.evaluate(11.52-1e-7)
        assert_allclose(before[0],at_end[0],atol=1e-13,rtol=0.)
        assert_allclose(before[1],np.zeros(3),atol=1e-12)
        assert_allclose(before[2],np.zeros(3),atol=1e-5)
    def test_legacy_quintic_object_identity_is_retained_when_it_fits(self):
        self.c.finalCom=self.complete_start+[-.03,0.,0.]
        with patch.object(p,'build_bounded_settle',wraps=p.build_bounded_settle) as build:self.evaluate(10.00375)
        self.assertIs(self.c._com_settle,build.call_args.kwargs['legacy'])
        self.assertEqual(self.c._bounded_settle_receipt['profile'],'legacy-quintic-exact-identity')
        self.assertFalse(self.c._bounded_settle_receipt['endpointDerivativesCanonicalized'])
    def test_nonzero_incoming_jets_reject_before_installing_a_settle(self):
        self.incoming_v[0]=1.01e-12
        with self.assertRaisesRegex(ValueError,'zero endpoint derivatives'):self.evaluate(10.00375)
        self.assertFalse(hasattr(self.c,'_com_settle'))
        self.assertFalse((self.root/'posture-settle-step-01.json').exists())


if __name__=='__main__':unittest.main()
