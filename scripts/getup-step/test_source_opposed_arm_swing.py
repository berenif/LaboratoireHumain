"""Zero-integration mirrored-axis, causal mapping and unchanged-base checks."""
import json
import unittest
from unittest.mock import patch
import numpy as np
import source_opposed_arm_swing as swing
from source_step_probe import World, Controller, ROOT
from source_arm_knee_mobility import native_bytes, arms_along_body_goal, KNEE_GOAL_RAD
from plan_source_arm_knee import plan_posture

BASE = ROOT/'outputs/getup-source-step/hip-mobility-pair01/wider-session'
HELPER = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'


@unittest.skipUnless(BASE.exists() and HELPER.exists(), 'local original source assets absent')
class OpposedSwing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w = World(BASE, HELPER); cls.w.restore(BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.w)
        cls.q = cls.w.arr(0, (28,))[7:].copy()
        cls.goal = arms_along_body_goal(cls.w)
        cls.mapping = swing.SagittalArmMap(cls.q, cls.w.limits, cls.goal)
        cls.legs = swing.limb_telemetry(cls.w, 'leg')[0]
        cls.reference = plan_posture(cls.w, 1, True, .9, cls.goal, KNEE_GOAL_RAD)
        cls.kin = swing.fk.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))

    @classmethod
    def tearDownClass(cls):
        assert native_bytes(cls.w) == cls.before
        assert cls.w.l.lh_dim(cls.w.h, 6) == 0
        cls.w.close()

    def controller(self, placement=False):
        c = swing.OpposedArmController.from_controller(Controller(self.w, 0, .032, True),
                self.reference, self.mapping, self.legs, placement)
        c.trajectory_time_scale = 2.
        c.joint_reserve = .01
        return c

    def vectors(self, arms):
        q = self.q.copy(); q[15:] = arms
        poses = self.kin.forward(q, np.eye(4))
        return np.array([swing.P@(poses[side+'_hand'][:3,3]-poses[side+'_upper_arm'][:3,3])
                         for side in swing.SIDES])

    def test_both_oblique_mirrored_shoulders_produce_equal_geometric_angle(self):
        for delta in (np.array([-.1,.1]), np.array([.17,-.13]), np.array([.035,.035])):
            arms, _, _ = self.mapping.evaluate(delta)
            vectors = self.vectors(arms)
            np.testing.assert_allclose(swing.pitch(vectors)-self.mapping.neutral_pitch, delta, atol=2e-8)
            np.testing.assert_allclose(vectors[:,1],np.array(self.mapping.neutral_vectors)[:,1],atol=2e-8)
            np.testing.assert_allclose(np.linalg.norm(vectors,axis=1),np.linalg.norm(self.mapping.neutral_vectors,axis=1),atol=2e-8)
            self.assertTrue(np.all(arms >= self.w.limits[15:,0]+.02-1e-8))
            self.assertTrue(np.all(arms <= self.w.limits[15:,1]-.02+1e-8))

    def test_mapping_derivatives_match_finite_difference(self):
        delta = np.array([.07,-.04]); eps=1e-5
        q, dq, ddq = self.mapping.evaluate(delta)
        plus = self.mapping.evaluate(delta+eps)[0]; minus = self.mapping.evaluate(delta-eps)[0]
        np.testing.assert_allclose((plus-minus)/(2*eps),dq,atol=1e-8)
        np.testing.assert_allclose((plus-2*q+minus)/eps**2,ddq,atol=8e-6)

    def test_unverified_amplitude_is_rejected_without_clipping(self):
        with self.assertRaisesRegex(AssertionError, 'verified arm map'):
            self.mapping.evaluate(np.array([.46,0.]))
        self.assertFalse(self.mapping.receipt()['clippingEnabled'])

    def test_first_transfer_starts_at_measured_arm_position(self):
        c = self.controller(True); _,info=c.solve(0.,'transfer')
        np.testing.assert_array_equal(c.arm_posture,self.q[15:])
        np.testing.assert_array_equal(c.arm_velocity,np.zeros(6))
        self.assertEqual(info['opposedArmSwing']['placementFraction'],0.)
        _,info=c.solve(4.,'transfer')
        self.assertEqual(info['opposedArmSwing']['placementFraction'],1.)
        np.testing.assert_allclose(c.arm_posture,self.goal,atol=1e-12)

    def test_measured_ipsilateral_legs_causally_drive_opposite_arm_and_motor(self):
        c=self.controller(); baseline,base_info=c.solve(1.,'translate')
        telemetry=swing.limb_telemetry
        for side in (0,1):
            def changed(w,kind):
                value,rate,vec=telemetry(w,kind)
                if kind=='leg':value=value.copy();value[side]+=.04
                return value,rate,vec
            with patch.object(swing,'limb_telemetry',side_effect=changed):
                c=self.controller(); adjusted,info=c.solve(1.,'translate')
            expected=np.zeros(2);expected[side]=-.04
            np.testing.assert_allclose(info['opposedArmSwing']['targetArmPitchChangeRad'],expected,atol=1e-12)
            self.assertGreater(max(abs(adjusted-baseline)),1e-5)
            self.assertLessEqual(max(abs(adjusted)),.95000001)
            self.assertLess(info['dynamicsResidual'],1e-7)
            self.assertTrue(info['opposedArmSwing']['feedbackUsesMeasuredLeg'])

    def test_velocity_reference_is_causal_and_uses_same_minus_one_gain(self):
        telemetry=swing.limb_telemetry
        def changed(w,kind):
            value,rate,vec=telemetry(w,kind)
            if kind=='leg':rate=np.array([.02,-.03])
            return value,rate,vec
        with patch.object(swing,'limb_telemetry',side_effect=changed):
            c=self.controller();c.solve(1.,'translate')
        jac=self.mapping.evaluate(np.zeros(2))[1]
        np.testing.assert_allclose(c.arm_velocity,jac*np.repeat([-.02,.03],3),atol=1e-12)

    def test_live_state_and_selected_physics_are_unchanged_by_planning(self):
        self.assertEqual(native_bytes(self.w),self.before)
        self.assertEqual(self.w.l.lh_dim(self.w.h,6),0)
        self.assertEqual(swing.sha(self.w.xml),'216d5c0159abf07139f5556c59e244ae77cb7557ba5256e301ebc5e1c8bbc399')
        self.assertTrue(self.reference['staticMotorContactFeasible'])
        self.assertLess(self.reference['minimumRequiredMotorFraction'],.925)
        self.assertGreater(self.reference['minimumSelfSeparationM'],.025)


class SavedAmplitudeAudit(unittest.TestCase):
    def measurement(self, gain):
        from audit_source_opposed_arm_swing import amplitude
        legs=np.column_stack([np.linspace(-.1,.1,100),np.linspace(.12,-.12,100)])
        return amplitude([(gain*l,l,-l) for l in legs])

    def test_equal_opposed_motion_passes(self):
        from audit_source_opposed_arm_swing import tracking_pass
        result=self.measurement(-1.)
        self.assertTrue(tracking_pass(result))
        np.testing.assert_allclose(result['actualArmToLegExcursionRatio'],[1.,1.])
        np.testing.assert_allclose(result['measuredAngularGain'],[-1.,-1.])

    def test_six_percent_amplitude_error_is_not_hidden(self):
        from audit_source_opposed_arm_swing import tracking_pass
        self.assertFalse(tracking_pass(self.measurement(-1.06)))

    def test_equal_amplitude_without_opposition_fails(self):
        from audit_source_opposed_arm_swing import tracking_pass
        self.assertFalse(tracking_pass(self.measurement(1.)))


if __name__=='__main__': unittest.main()
