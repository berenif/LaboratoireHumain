import dataclasses
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
REPO = Path(__file__).resolve().parents[2]
ARTIFACTS = Path('/workspace/shared/reviewed-walk/coupled-posture-design')
sys.path.insert(0,str(REPO/'scripts/getup-step'))
from coupled_posture_reference import *
from coupled_posture_kinematics import (ModelFK, ArmMap, MatrixJet, P,
    pose_jet_from_source,pose_jet_to_source,BaseFrameSample)
from coupled_posture_admission import StaticWitness, verify_static_point
from source_reference_context import Jet as ExistingJet, Orientation, ReferenceContext, SCHEMA, REFERENCE_VERSION


def sample(com=(0.,0.,.8),lower=None,yaw=.7,pitch=.03,roll=.05):
    lower = np.zeros(15) if lower is None else lower
    return CoupledSample(Jet.rest(com),Jet.rest(lower),
        OrientationJet.rest(1,rotation([yaw,pitch,roll])),
        OrientationJet.rest(4,rotation([yaw,pitch+.02,-roll])))


def candidate(target=None,name='test'):
    return Candidate(name,'a'*64,sample() if target is None else target)


def plan(initial=None,target=None,start=3.,duration=2.):
    initial = sample() if initial is None else initial
    target = sample(com=(.002,-.001,.8),lower=np.ones(15)*.01,pitch=.05) if target is None else target
    return CoupledPlan.between(start,duration,initial,candidate(target),
        joint_limits=np.tile([-2.,2.],(21,1)),lumbar_anchor=np.zeros(3))


def support(now=3.,step=2400,phase='transfer',triangle=False):
    return CurrentSupport(step,now,1,phase,((-1.,-1.),(1.,-1.),(1.,1.),(-1.,1.)),
        ((-1.,-1.,0.),(1.,-1.,0.),(1.,1.,0.),(-1.,1.,0.)),(True,)*4,
        (0,1,2),triangle)


def context(now=3.,step=2400,phase='transfer'):
    zero3 = ExistingJet.make(np.zeros(3),np.zeros(3),np.zeros(3))
    arms = ExistingJet.make(np.arange(6)*.01,np.arange(6)*.02,np.arange(6)*.03)
    return ReferenceContext(SCHEMA,REFERENCE_VERSION,step,now,1,phase,0.,0,zero3,
        ExistingJet.make(np.zeros(21),np.zeros(21),np.zeros(21)),arms,(100.,20.,2.),
        (Orientation.make(1,np.eye(3),np.zeros(3),np.zeros(3)),),(),(),False,
        '{"arm":{"original":true},"seek":"retained"}','{}','{}')


class PolynomialTests(unittest.TestCase):
    def test_arbitrary_endpoint_jets_and_derivatives(self):
        initial = Jet([1.,2.],[.3,-.2],[.1,.4]); final = Jet([2.,1.],[-.1,.6],[-.4,.2])
        p = C2Segment.between(4.,2.5,initial,final)
        self.assertIs(p.at(4.),initial); self.assertIs(p.at(6.5),final)
        t,h = 5.1,1e-5
        a,b,c = [p.at(x).arrays() for x in (t-h,t,t+h)]
        assert_allclose((c[0]-a[0])/(2*h),b[1],atol=1e-9)
        assert_allclose((c[1]-a[1])/(2*h),b[2],atol=1e-9)
        with self.assertRaises(ValueError):p.at(7.)
        with self.assertRaises(ValueError):p.at(3.9)

    def test_wait_is_exact_rest_forever(self):
        p = plan()
        for t in (5.,5.000001,50.,5000.):
            self.assertEqual(p.at(t),p.at(5.))
            self.assertFalse(any(p.at(t).com.velocity))
            self.assertFalse(any(p.at(t).com.acceleration))

    def test_handoff_joins_nonzero_jets_and_keeps_lumbar_anchor(self):
        p = plan(); now = 3.7
        target = sample(com=(.003,.002,.8),lower=np.ones(15)*.015,pitch=.04)
        q = p.handoff(now,2.,candidate(target,'settle'))
        for name in ('com','lower_joints'):
            self.assertEqual(getattr(q.at(now),name),getattr(p.at(now),name))
        for name in ('thorax','pelvis'):
            for a,b in zip(getattr(q.at(now),name).arrays(),getattr(p.at(now),name).arrays()):
                assert_allclose(a,b,atol=2e-15)
        self.assertEqual(p.lumbar_anchor,q.lumbar_anchor)
        r = q.handoff(8.,2.,candidate(sample(),'stand'))
        self.assertEqual(r.at(8.).com,q.at(8.).com)

    def test_bounds_catch_between_endpoint_violation(self):
        a = Jet(np.zeros(15),np.r_[10.,np.zeros(14)],np.zeros(15))
        x = dataclasses.replace(sample(),lower_joints=a)
        with self.assertRaisesRegex(ValueError,'joint polynomial'):
            plan(initial=x,target=sample())
        with self.assertRaisesRegex(ValueError,'lumbar'):
            plan(target=sample(lower=np.r_[.1,np.zeros(14)]))
        with self.assertRaisesRegex(ValueError,'COM jet'):
            plan(target=sample(com=(1.,0.,.8)),duration=1.)

    def test_immutability_and_defensive_arrays(self):
        a = np.array([1.,2.]); j = Jet(a,a,a);a[0]=5.
        self.assertEqual(j.position[0],1.)
        j.arrays()[0][0]=8.;self.assertEqual(j.position[0],1.)
        with self.assertRaises(dataclasses.FrozenInstanceError):j.position=(3.,4.)
        with self.assertRaises(ValueError):Jet([math.nan],[0.],[0.])


class OrientationTests(unittest.TestCase):
    def test_known_forward_roll_heading(self):
        for yaw in (0.,.7,-2.2):
            R = rotation([yaw,math.radians(4.),math.radians(5.)])
            assert_allclose(chart(R),[yaw,math.radians(4.),math.radians(5.)],atol=1e-15)
            forward = np.array([math.cos(yaw),math.sin(yaw),0.])
            self.assertGreater(float(forward@R[:,2]),0.)

    def test_noncommuting_angular_jet_and_inverse(self):
        x = Jet([.7,.1,-.15],[.2,-.3,.4],[.1,.3,-.2])
        R,w,a = angular_world(x); back = angular_chart(R,w,a)
        for aa,bb in zip(x.arrays(),back.arrays()):assert_allclose(aa,bb,atol=1e-15)
        h=1e-5;p,v,acc=x.arrays()
        wm = angular_world(Jet(p-v*h+.5*acc*h*h,v-acc*h,acc))[1]
        wp = angular_world(Jet(p+v*h+.5*acc*h*h,v+acc*h,acc))[1]
        Rm = rotation(p-v*h+.5*acc*h*h);Rp=rotation(p+v*h+.5*acc*h*h)
        assert_allclose(Rotation.from_matrix(Rp@Rm.T).as_rotvec()/(2*h),w,atol=2e-10)
        assert_allclose((wp-wm)/(2*h),a,atol=2e-10)

    def test_retained_heading_and_refuse_singular_or_untransportable(self):
        p = plan()
        for t in np.linspace(3.,5.,19):
            for o in (p.at(t).thorax,p.at(t).pelvis):
                self.assertAlmostEqual(chart(o.arrays()[0])[0],.7,places=14)
        with self.assertRaisesRegex(ValueError,'retained heading'):
            plan(target=sample(yaw=.8))
        with self.assertRaisesRegex(ValueError,'singular'):
            chart(rotation([.7,math.pi/2,0.]))
        o=OrientationJet(1,frozen(rotation([.7,.03,.05])),(0.,0.,.1),(0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'nonstationary'):
            plan(initial=dataclasses.replace(sample(),thorax=o))

    def test_soft_slack_reports_unmet_forward_preference(self):
        ref=OrientationJet.rest(1,rotation([.7,.08,.03]))
        d=orientation_residual(ref,rotation([.7,.02,.04]),np.zeros(3),
            achieved_angular_acceleration=np.zeros(3))
        self.assertAlmostEqual(d['forwardPitchPreferenceMissRad'],.06)
        self.assertGreater(np.linalg.norm(d['softAccelerationSlackWorldRadps2']),0.)


class HookTests(unittest.TestCase):
    def test_disabled_is_identical_even_with_invalid_other_arguments(self):
        c=context();self.assertIs(prepare_context(c,plan='bad',support='bad'),c)
        self.assertEqual(c.receipt(),prepare_context(c).receipt())

    def test_existing_context_roundtrip_delayed_arms_and_no_mutation(self):
        c=context();before=c.receipt();p=plan(); opt=OptIn(p.candidate.name,p.candidate.key)
        d=prepare_context(c,opt_in=opt,plan=p,support=support())
        self.assertEqual(c.receipt(),before)
        self.assertIs(d.arms,c.arms);self.assertIs(d.feet,c.feet)
        self.assertIs(d.capture_constraints,c.capture_constraints)
        self.assertEqual(d.causal_json,c.causal_json)
        self.assertEqual([o.body for o in d.orientations],[1,4])
        self.assertEqual(d.joints.position[15:],c.arms.position)
        self.assertEqual(d.joints.velocity[15:],c.arms.velocity)
        self.assertEqual(d.joints.acceleration[15:],c.arms.acceleration)
        self.assertEqual(ReferenceContext.from_receipt(d.receipt()),d)
        self.assertEqual(prepare_context(c,opt_in=opt,plan=p,support=support()),d)

    def test_reject_stale_support_candidate_and_changed_actual_hull(self):
        c=context();p=plan();o=OptIn(p.candidate.name,p.candidate.key)
        with self.assertRaisesRegex(ValueError,'stale'):
            prepare_context(c,opt_in=o,plan=p,support=support(now=3.1))
        with self.assertRaisesRegex(ValueError,'mismatch'):
            prepare_context(c,opt_in=OptIn('wrong','a'*64),plan=p,support=support())
        s=dataclasses.replace(support(),loaded_points_xy=((2.,2.),(3.,2.),(2.,3.)))
        with self.assertRaisesRegex(ValueError,'CURRENT'):
            prepare_context(c,opt_in=o,plan=p,support=s)
        # The initial and final targets inside the old hull do not exempt a later
        # changed support geometry from current evaluation.
        with self.assertRaisesRegex(ValueError,'surviving'):
            dataclasses.replace(support(triangle=True),present_corners=(False,True,True,True))

    def test_triangle_and_capture_are_both_checked(self):
        s=support(triangle=True)
        with self.assertRaisesRegex(ValueError,'CURRENT'):
            s.validate_com(Jet.rest([-.5,.5,.8]))
        s=support()
        with self.assertRaisesRegex(ValueError,'CURRENT'):
            s.validate_com(Jet([.99,0.,.8],[.1,0.,0.],[0.,0.,0.]))


@unittest.skipUnless((ARTIFACTS/'arm-map-table.json').is_file(), 'archived pure FK witnesses absent')
class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=json.loads((REPO/'rust/crates/model/data/getup-v1.json').read_text())
        cls.fk=ModelFK.from_model(cls.model)
        cls.arm=ArmMap.from_archived_table(json.loads((ARTIFACTS/'arm-map-table.json').read_text()))
        cls.endpoints=[json.loads((ARTIFACTS/f'free-com-candidate-step-{i:02d}.json').read_text()) for i in (1,2)]

    def anchor(self,d):
        q=np.array(d['qposWitness']);root=np.eye(4)
        root[:3,:3]=P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P
        root[:3,3]=P.T@q[:3]
        poses=self.fk.forward(Jet.rest(d['qReference']),MatrixJet.rest(root))
        return poses[('left_foot','right_foot')[d['stanceSide']]]

    def evaluate(self,d,lower):
        return self.fk.coupled(lower,d['stanceSide'],self.anchor(d),self.arm,
            d['legPitchOriginRightLeftRad'])

    def test_archived_endpoints_exact_future_arms_com_and_body_mapping(self):
        for d in self.endpoints:
            r=self.evaluate(d,Jet.rest(d['qReference'][:15]))
            assert_allclose(r.full_joints.position,d['qReference'],atol=3e-13)
            assert_allclose(r.future_thigh_right_left.position,d['thighPitchRightLeftRad'],atol=3e-13)
            assert_allclose(r.com_source.position,d['comReferenceSource'],atol=3e-13)
            for o,key in ((r.thorax,'torso'),(r.pelvis,'pelvis')):
                assert_allclose(o.arrays()[0],d['bodyReferencesSource'][key],atol=3e-13)
            self.assertFalse(any(r.com_source.velocity));self.assertFalse(any(r.com_source.acceleration))

    def test_right_left_permutation_and_exact_analytic_fk_jets(self):
        d=self.endpoints[0];q=np.array(d['qReference'][:15]);z=np.zeros(15)
        base=self.fk.thigh_jet(Jet.rest(q)).position
        for index,side in ((5,0),(11,1)):
            altered=q.copy();altered[index]+=.001
            change=np.array(self.fk.thigh_jet(Jet.rest(altered)).position)-base
            self.assertAlmostEqual(change[side],-.001,delta=2e-5)
            self.assertAlmostEqual(change[1-side],0.,places=12)
        v=np.linspace(-.02,.03,15);a=np.linspace(.01,-.01,15);h=2e-5
        center=self.evaluate(d,Jet(q,v,a));minus=self.evaluate(d,Jet(q-v*h+.5*a*h*h,v-a*h,a));plus=self.evaluate(d,Jet(q+v*h+.5*a*h*h,v+a*h,a))
        for key in ('com_source','future_thigh_right_left','future_arm_delta_right_left','full_joints'):
            m,c,p=[getattr(r,key).arrays() for r in (minus,center,plus)]
            assert_allclose((p[0]-m[0])/(2*h),c[1],atol=2e-10)
            assert_allclose((p[1]-m[1])/(2*h),c[2],atol=2e-10)
        for key in ('thorax','pelvis'):
            m,c,p=[getattr(r,key).arrays() for r in (minus,center,plus)]
            assert_allclose(Rotation.from_matrix(p[0]@m[0].T).as_rotvec()/(2*h),c[1],atol=2e-10)
            assert_allclose((p[1]-m[1])/(2*h),c[2],atol=2e-10)

    def test_original_verified_range_reserve_no_extrapolation(self):
        lo=self.arm.ranges[0][0]
        with self.assertRaisesRegex(ValueError,'no clipping'):
            self.arm.evaluate(Jet.rest([lo+.019,0.]))
        self.arm.evaluate(Jet.rest([lo+.02,0.]))
        with self.assertRaises(ValueError):self.arm.evaluate(Jet.rest([lo-1e-7,0.]),reserve=0.)

    def test_moving_root_stance_spatial_conversion_and_handoff(self):
        translation=Jet([.3,-.2,.1],[.02,-.01,.03],[.01,.02,-.01])
        R=rotation([.8,.1,-.2]);w=np.array([.02,-.03,.04]);a=np.array([-.01,.02,.03])
        pose=pose_jet_from_source(translation,R,w,a);back=pose_jet_to_source(pose)
        self.assertEqual(back.translation.position,translation.position)
        assert_allclose(back.rotation,frozen(R),atol=1e-15)
        assert_allclose(back.angular_velocity_world,w,atol=1e-15)
        assert_allclose(back.angular_acceleration_world,a,atol=1e-15)
        identity=pose@pose.inverse()
        assert_allclose(identity.T,np.eye(4),atol=1e-15)
        assert_allclose(identity.Tdot,np.zeros((4,4)),atol=1e-15)
        assert_allclose(identity.Tddot,np.zeros((4,4)),atol=1e-15)
        d=self.endpoints[0];anchor=self.anchor(d);anchor_source=pose_jet_to_source(anchor)
        moving=pose_jet_from_source(anchor_source.translation,np.array(anchor_source.rotation).reshape(3,3),w,a)
        lower=Jet(d['qReference'][:15],np.linspace(-.02,.02,15),np.linspace(.01,-.01,15))
        result=self.fk.coupled(lower,d['stanceSide'],moving,self.arm,d['legPitchOriginRightLeftRad'])
        frame=result.base_frame(d['stanceSide'])
        self.assertIsInstance(frame,BaseFrameSample)
        for found,expected in zip(frame.stance.arrays(),moving.arrays()):
            assert_allclose(found,expected,atol=4e-15)
        self.assertGreater(np.linalg.norm(frame.root.Tdot),0.)
        self.assertGreater(np.linalg.norm(result.com_source.velocity),0.)

    def test_pose_jet_rejects_missing_angular_curvature(self):
        T=np.eye(4);D=np.zeros((4,4));D[:3,:3]=[[0.,-.1,0.],[.1,0.,0.],[0.,0.,0.]]
        with self.assertRaisesRegex(ValueError,'rigid'):
            MatrixJet(frozen(T),frozen(D),frozen(np.zeros((4,4))))


class AdmissionTests(unittest.TestCase):
    def witness(self,s):
        return StaticWitness(s.fingerprint(),support().key,2400,3.,'m'*64,'h'*64,True,.91,True,.91,
            .04,tuple(s.lower_joints.position)+(0.,)*6,(.1,.1),0.,0.,True,0.,0.,
            s.com.position,s.thorax.rotation,s.pelvis.rotation,1e-14,0.)

    def check(self,s,w):
        return verify_static_point(s,w,support(),joint_limits=np.tile([-2.,2.],(21,1)),
            lumbar_anchor=np.zeros(3),model_sha256='m'*64,helper_sha256='h'*64,retained_headings=(.7,.7))

    def test_strict_thresholds_and_no_dynamic_promotion(self):
        s=sample();w=self.witness(s);r=self.check(s,w)
        self.assertTrue(r['staticPointPassed']);self.assertFalse(r['fullPathProven'])
        self.assertFalse(r['finiteTimePhysicalFeasibilityProven'])
        for changed in (dict(strict_four_point_motor_fraction=.925),
                        dict(minimum_clean_self_gap_m=.025),
                        dict(source_step=2399),dict(future_arm_identity_error_rad=1e-12)):
            with self.assertRaises(ValueError):self.check(s,dataclasses.replace(w,**changed))

    def test_soft_forward_goal_slack_is_reported_without_false_achievement(self):
        s=sample(pitch=.08);w=self.witness(s)
        w=dataclasses.replace(w,fk_thorax_rotation=frozen(rotation([.7,.02,.05])))
        r=self.check(s,w)
        self.assertAlmostEqual(r['orientationPreferenceSlacks'][0]['forwardPitchSlackRad'],.06)
        self.assertTrue(r['staticPointPassed'])


class CurrentSeedArtifactTests(unittest.TestCase):
    def test_fresh_seed_rejection_is_preserved_without_physics(self):
        path=Path('/workspace/shared/reviewed-walk/coupled-posture-reference/current-seed-admission.json')
        if not path.exists():self.skipTest('fresh static audit has not been requested/run')
        d=json.loads(path.read_text())
        self.assertFalse(d['staticEndpointAdmitted'])
        self.assertFalse(d['staticEndpointChecks']['preservedActiveReferenceFeet'])
        self.assertTrue(all(v for k,v in d['staticEndpointChecks'].items()
            if k!='preservedActiveReferenceFeet'))
        self.assertFalse(d['seedAdjusted']);self.assertFalse(d['pilotReady'])
        self.assertEqual(d['nativeIntegrations'],0);self.assertEqual(d['liveRunnerStateInstalls'],0)
        self.assertEqual(d['nativeStepAttempts'],0)

    def test_handoff_proof_distinguishes_inherited_ankle_reference(self):
        path=Path('/workspace/shared/reviewed-walk/coupled-posture-reference/current-seed-admission.json')
        if not path.exists():self.skipTest('fresh static audit has not been requested/run')
        d=json.loads(path.read_text());h=d['handoff']
        self.assertEqual(h['declaredDurationS'],1.52)
        self.assertTrue(h['entireCOMEnvelopePass'])
        self.assertTrue(h['entireCurrentFixedSupportCOMCaptureInsetPass'])
        self.assertGreaterEqual(h['minimumCurrentFixedHullTriangleCOMCaptureMarginM'],.003)
        self.assertAlmostEqual(h['inheritedInitialReferenceAnkleMarginsRightLeftRad'][0],.02)
        self.assertFalse(h['entireAnkle06ReferenceReservePass'])
        self.assertTrue(h['entireLowerJoint02InteriorPass'])
        self.assertIsNotNone(h['genericCoupledPlanRejection'])
        self.assertFalse(h['wholeFKContactCollisionDynamicPathProven'])


if __name__=='__main__':
    unittest.main(verbosity=2)
