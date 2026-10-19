import copy
import math
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from phase_leg_reference import *
from reviewed_phase_leg_fixtures import load_model,source_limits,MovingMathFixture,source_fixed_fixture


@unittest.skipUnless(Path('/workspace/shared/cadence-robust-8-ten/model-session/source-model/humanoid_high_freq.xml').is_file(), 'pinned executed source XML absent')
class LegReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin=load_model(); cls.limits=source_limits(); cls.paths=[]
        for side in (0,1):
            frame=MovingMathFixture(cls.kin,side,cls.limits)
            path=PhaseLegTrajectory(cls.kin,side,frame,cls.limits,[1.,0.,0.],joint_reserve=.02,
                max_stance_flexion_rad=1.5)
            receipt=path.prepare(9)
            assert receipt['accepted'],receipt
            cls.paths.append(path)

    def test_both_real_knee_indices_and_signs(self):
        self.assertEqual(KNEES,(12,6))
        for side,path in enumerate(self.paths):
            self.assertEqual(path.prepared_receipt['kneeQposIndex'],(12,6)[side])
            peak=path.geometry_sample('lift',.8)
            self.assertGreater(math.degrees(peak.knee_flexion_from_lift_entry_rad),10.)
            self.assertLess(peak.joints.position[3],path.q_entry[KNEES[side]])
            finish=path.geometry_sample('land',1.)
            self.assertGreater(finish.joints.position[3],peak.joints.position[3])

    def test_matched_cartesian_foot_and_rigid_foot_clearance(self):
        for path in self.paths:
            for phase,t in [('lift',.37),('translate',.47),('land',.31)]:
                sample=path.geometry_sample(phase,t); goal=path.target(phase,t)
                for actual,target in zip(sample.foot_model.arrays(),goal.arrays()):
                    np.testing.assert_allclose(actual,target,atol=2e-8,rtol=0.)
                self.assertFalse(dict(sample.diagnostics)['armsReplaced'])
            self.assertGreater(path.geometry_sample('translate',.6).planned_whole_foot_clearance_m,.039999)

    def test_analytic_joint_derivatives_against_finite_differences(self):
        h=1e-4
        for path in self.paths:
            for phase,t in [('lift',.31),('translate',.48),('land',.41)]:
                before=path.geometry_sample(phase,t-h); now=path.geometry_sample(phase,t); after=path.geometry_sample(phase,t+h)
                np.testing.assert_allclose(now.joints.velocity,(np.array(after.joints.position)-before.joints.position)/(2*h),atol=2e-6,rtol=0.)
                np.testing.assert_allclose(now.joints.acceleration,(np.array(after.joints.velocity)-before.joints.velocity)/(2*h),atol=2e-5,rtol=0.)

    def test_fk_product_rule_derivatives_moving_root(self):
        fk=FKJets(self.kin); q=np.array(self.paths[0].q_entry)
        v=np.linspace(-.03,.04,21); a=np.linspace(.02,-.01,21)
        R=Rotation.from_euler('ZYX',[.1,.04,-.03]).as_matrix()
        root=source_pose_jet(Jet([.1,.2,1.3],[.02,-.01,.03],[.01,.03,-.02]),R,[.03,-.02,.01],[.01,.02,-.01])
        out=fk.forward(Jet(q,v,a),root); h=1e-5
        T,V,A=root.arrays()
        # Root second-order Taylor matrix is sufficient for derivative checks.
        plus=self.kin.forward(q+v*h+.5*a*h*h,T+V*h+.5*A*h*h)
        minus=self.kin.forward(q-v*h+.5*a*h*h,T-V*h+.5*A*h*h)
        for name in FEET:
            p,d,dd=out[name].arrays()
            np.testing.assert_allclose(d,(plus[name]-minus[name])/(2*h),atol=2e-9,rtol=0.)
            np.testing.assert_allclose(dd,(plus[name]-2*p+minus[name])/h**2,atol=2e-5,rtol=0.)

    def test_endpoint_and_unbounded_wait_C2(self):
        boundaries=[(('lift',.8),('lift_wait',1e6)),(('lift_wait',1e6),('translate',0.)),
            (('translate',1.2),('translate_wait',1e6)),(('translate_wait',1e6),('land',0.)),
            (('land',1.),('land_wait',1e6)),(('land_wait',1e6),('settle',0.)),(('settle',1e6),('stand',0.))]
        for path in self.paths:
            for lhs,rhs in boundaries:
                l=path.geometry_sample(*lhs); r=path.geometry_sample(*rhs)
                for a,b in zip(l.joints.arrays(),r.joints.arrays()): np.testing.assert_allclose(a,b,atol=2e-9,rtol=0.)
                for a,b in zip(l.foot_model.arrays(),r.foot_model.arrays()): np.testing.assert_allclose(a,b,atol=2e-9,rtol=0.)
                self.assertEqual(l.joints.velocity,(0.,)*6)
                self.assertEqual(l.joints.acceleration,(0.,)*6)

    def test_no_horizontal_advance_before_measured_eighty(self):
        path=self.paths[0]
        for t in (0.,.4,.8,100.):
            foot=source_jet(path.geometry_sample('lift',t).foot_model)[0]
            np.testing.assert_allclose(foot.position[:2],path.entry_source[0].position[:2],atol=2e-9,rtol=0.)
        with self.assertRaises(AdmissionRequired): path.evaluate('translate',.1)
        samples=tuple(MeasuredAdvanceSample(i,.025,2.999,80.)for i in range(80))
        accepted=AdvanceWindow(samples,100.,79)
        path.evaluate('translate',.1,advance_window=accepted)
        for window in [AdvanceWindow(samples[:-1],100.,78),AdvanceWindow(samples,100.,80),
                       AdvanceWindow(samples[:-1]+(MeasuredAdvanceSample(79,.0249999,0.,80.),),100.,79),
                       AdvanceWindow(samples[:-1]+(MeasuredAdvanceSample(79,.025,3.,80.),),100.,79),
                       AdvanceWindow(samples[:-1]+(MeasuredAdvanceSample(79,.025,0.,79.9999),),100.,79)]:
            with self.assertRaises(AdmissionRequired):path.evaluate('translate',.1,advance_window=window)

    def test_disabled_identity_and_enabled_only_swing_patch(self):
        path=self.paths[0]; sample=path.geometry_sample('lift',.4)
        baseline=Jet(np.arange(21),np.arange(21)*2,np.arange(21)*3)
        feet=(object(),object())
        q,f=overlay_references(baseline,feet,None,enabled=False)
        self.assertIs(q,baseline);self.assertIs(f,feet)
        actual,actualfeet=overlay_references(baseline,feet,sample,enabled=True)
        self.assertIs(actualfeet[1],feet[1]); self.assertIs(actualfeet[0],sample.foot_model)
        for old,new in zip(baseline.arrays(),actual.arrays()):
            mask=np.ones(21,bool);mask[list(sample.joint_indices)]=False
            self.assertTrue(np.array_equal(old[mask],new[mask]))
            self.assertTrue(np.array_equal(old[15:],new[15:]))

    def test_explicit_preference_is_not_silently_reduced(self):
        frame=MovingMathFixture(self.kin,0,self.limits)
        path=PhaseLegTrajectory(self.kin,0,frame,self.limits,[1.,0.,0.],joint_reserve=.02,
            knee_preference_rad=math.radians(40.),max_stance_flexion_rad=1.5)
        receipt=path.prepare(5)
        self.assertFalse(receipt['accepted']);self.assertIn('not silently reduced',receipt['reason'])

    def test_incompatible_stance_jets_rejected(self):
        frame=MovingMathFixture(self.kin,0,self.limits);good=frame('lift',0.)
        T,V,A=good.stance_pose_model.arrays();V[0,3]=.001
        bad=BaseFrameSample(good.root_model,good.joints,MatrixJet(T,V,A))
        with self.assertRaises(ReferenceRejected):
            PhaseLegTrajectory(self.kin,0,lambda p,t:bad,self.limits,[1.,0.,0.],max_stance_flexion_rad=1.5)

    @unittest.skipUnless(Path('/workspace/shared/reviewed-walk/causal-selection-seven-01/run/generation-001/controller-step-02.json').is_file(), 'saved source family absent')
    def test_saved_source_fixed_frame_is_rejected_honestly(self):
        for i in (1,2):
            kin,side,frame,limits,heading=source_fixed_fixture(i)
            path=PhaseLegTrajectory(kin,side,frame,limits,heading,joint_reserve=.01,max_stance_flexion_rad=1.5)
            receipt=path.prepare(5)
            self.assertFalse(receipt['accepted']);self.assertFalse(receipt['sourceAdmitted'])
            self.assertEqual(receipt['nativeStateInstallations'],0)
            self.assertIn('IK/foot pose',receipt['reason'])

    def test_inputs_and_history_are_not_mutated(self):
        path=self.paths[0];qentry=path.q_entry.copy(); receipt=copy.deepcopy(path.prepared_receipt)
        a=path.geometry_sample('translate',.41);b=path.geometry_sample('translate',.41)
        self.assertEqual(a,b);self.assertTrue(np.array_equal(qentry,path.q_entry));self.assertEqual(receipt,path.prepared_receipt)


    def test_reference_context_adapter_preserves_measured_arm_identity(self):
        from dataclasses import dataclass
        @dataclass(frozen=True)
        class Orientation:
            body: int
            rotation: tuple
            velocity: tuple
            acceleration: tuple
            kp: float=40.
        @dataclass(frozen=True)
        class Foot:
            translation: Jet
            orientation: Orientation
        @dataclass(frozen=True)
        class Context:
            joints: Jet
            feet: tuple
            arms: Jet
            com: Jet
            phase: str
        arms=Jet.rest([.1,.2,.3,.4,.5,.6]); foot=Foot(Jet.rest([0.,0.,0.]),Orientation(10,frozen(np.eye(3)),(0.,)*3,(0.,)*3))
        sample=self.paths[0].geometry_sample('lift',.4)
        p,R,w,a=sample.foot_source()
        swingfoot=Foot(p,Orientation(10,frozen(R),frozen(w),frozen(a)))
        context=Context(sample.base_frame.joints,(swingfoot,foot),arms,Jet.rest([.1,.2,.8]),'lift')
        self.assertIs(overlay_reference_context(context,None,enabled=False),context)
        changed=overlay_reference_context(context,sample,enabled=True)
        self.assertIs(changed.arms,arms);self.assertIs(changed.com,context.com)
        self.assertIs(changed.feet[1],foot);self.assertEqual(changed.feet[0].orientation.body,10)
        self.assertEqual(changed.feet[0].orientation.kp,40.)
        self.assertEqual(changed.joints.position[15:],context.joints.position[15:])
        from dataclasses import replace
        incompatible=replace(context,feet=(replace(swingfoot,translation=Jet.rest([0.,0.,0.])),foot))
        with self.assertRaises(ReferenceRejected):overlay_reference_context(incompatible,sample,enabled=True)


    @unittest.skipUnless(Path('/workspace/shared/reviewed-walk/phase-leg-reference/saved-pose-calibration.json').is_file(), 'saved actual FK calibration absent')
    def test_saved_actual_native_pose_calibration(self):
        import json
        from pathlib import Path
        calibration=json.loads((Path('/workspace/shared/reviewed-walk/phase-leg-reference')/'saved-pose-calibration.json').read_text())
        self.assertGreaterEqual(len(calibration['records']),4)
        self.assertLess(calibration['maximumFootPositionErrorM'],1e-10)
        self.assertLess(calibration['maximumWholeFootClearanceErrorM'],1e-10)
        self.assertTrue(all(not r['ankleReservesAsserted']for r in calibration['records']))
        from reviewed_phase_leg_fixtures import source_root
        for row in json.loads((Path('/workspace/shared/reviewed-walk/phase-leg-reference')/'saved-pose-inputs.json').read_text()):
            poses=self.kin.forward(np.array(row['qpos'])[7:],source_root(row['qpos']).arrays()[0])
            for side,name in enumerate(FEET):
                np.testing.assert_allclose(P@poses[name][:3,3],row['feet'][side]['position'],atol=1e-10,rtol=0.)
                self.assertAlmostEqual(self.kin.bottom(name,poses[name]),row['feet'][side]['clearance'],places=10)


    def test_incoming_lift_velocity_cannot_be_reset(self):
        frame=MovingMathFixture(self.kin,0,self.limits)
        def bad(phase,t):
            sample=frame(phase,t);p,v,a=sample.joints.arrays();v[12]=.01
            return BaseFrameSample(sample.root_model,Jet(p,v,a),sample.stance_pose_model)
        path=PhaseLegTrajectory(self.kin,0,bad,self.limits,[1.,0.,0.],max_stance_flexion_rad=1.5)
        receipt=path.prepare(5)
        self.assertFalse(receipt['accepted']);self.assertIn('no C2 reset',receipt['reason'])

    def test_native_world_and_state_constructors_are_never_called(self):
        from unittest.mock import patch
        def forbidden(*args,**kwargs):raise AssertionError('native process/library construction forbidden')
        with patch('subprocess.Popen',side_effect=forbidden),patch('ctypes.CDLL',side_effect=forbidden):
            path=self.paths[0]
            result=path.geometry_sample('lift',.3)
            self.assertGreater(result.knee_flexion_from_lift_entry_rad,0.)


    def test_shared_coupled_posture_frame_protocol(self):
        import sys
        from pathlib import Path
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from coupled_posture_kinematics import MatrixJet as SharedPose, BaseFrameSample as SharedFrame, pose_jet_from_source
        from coupled_posture_reference import Jet as SharedJet
        frame=self.paths[0].frame_path('translate',.4)
        shared=SharedFrame(SharedPose(*frame.root_model.arrays()),SharedJet(*frame.joints.arrays()),SharedPose(*frame.stance_pose_model.arrays()))
        ours=BaseFrameSample.from_coupled_frame(shared)
        for a,b in zip(ours.root_model.arrays(),frame.root_model.arrays()): np.testing.assert_array_equal(a,b)
        position=Jet([.1,.2,.8],[.03,-.02,.01],[.02,.01,-.04]);rotation=Rotation.from_euler('ZYX',[.4,.1,-.2]).as_matrix()
        a=source_pose_jet(position,rotation,[.02,.03,-.01],[.01,-.02,.03])
        b=pose_jet_from_source(SharedJet(*position.arrays()),rotation,[.02,.03,-.01],[.01,-.02,.03])
        for x,y in zip(a.arrays(),b.arrays()):np.testing.assert_allclose(x,y,atol=1e-15,rtol=0.)

    def test_prepared_frame_baseline_mismatch_is_rejected(self):
        from dataclasses import dataclass
        @dataclass(frozen=True)
        class Context:
            joints: Jet
        with self.assertRaises(ReferenceRejected):
            overlay_reference_context(Context(Jet.rest(np.zeros(21))),self.paths[0].geometry_sample('lift',.4),enabled=True)


    def test_contact_seek_is_part_of_the_compatible_foot_leg_jet(self):
        path=self.paths[0]; nominal=path.geometry_sample('land',1.)
        seek=(.0004,.0003,-.01)
        adjusted=path.geometry_sample('land',1.,landing_seek=seek)
        p0=nominal.foot_source()[0];p1=adjusted.foot_source()[0]
        for old,new,delta in zip(p0.arrays(),p1.arrays(),seek):
            np.testing.assert_allclose(new,old-np.array([0.,0.,delta]),atol=2e-8,rtol=0.)
        self.assertNotEqual(nominal.joints,adjusted.joints)
        with self.assertRaises(ReferenceRejected):path.geometry_sample('land',1.,landing_seek=(.00301,0.,0.))
        with self.assertRaises(ReferenceRejected):path.geometry_sample('lift',.2,landing_seek=seek)


if __name__=='__main__': unittest.main(verbosity=2)
