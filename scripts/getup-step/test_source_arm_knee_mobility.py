"""Zero-integration feasibility and causal arm-feedback tests."""
import unittest
import numpy as np
from source_arm_knee_mobility import (World, Controller, ROOT, native_bytes,
    arms_along_body_goal, balance_arm_offset, ArmKneeController,
    KNEE_GOAL_RAD, ARM_OFFSET_LIMIT_RAD, ARM_TARGET_RATE_RADPS, plan_posture)
BASE = ROOT/'outputs/getup-source-engine/session-20261017T233404Z'
HELPER = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'


@unittest.skipUnless(BASE.exists() and HELPER.exists(), 'local original source assets absent')
class ArmKneeMobility(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        seed = World(BASE, HELPER)
        try:
            seed.restore(BASE/'slow-continuation/final.bin')
            cls.entry = seed.packet()
        finally:
            seed.close()

    def setUp(self):
        self.w = World(BASE, HELPER)
        self.w.install_cold(self.entry)
        self.before = native_bytes(self.w)

    def tearDown(self):
        self.assertEqual(self.w.l.lh_dim(self.w.h, 6), 0)
        self.assertEqual(native_bytes(self.w), self.before)
        self.w.close()

    def test_original_knees_already_flexed_and_modest_preference_is_27_5deg(self):
        actual = -np.rad2deg(np.array(self.entry['qpos'])[7:][[6, 12]])
        np.testing.assert_allclose(actual, [20.1436, 25.4480], atol=.001)
        np.testing.assert_allclose(-np.rad2deg(KNEE_GOAL_RAD), [27.5, 27.5])

    def test_each_isolated_reference_has_motor_and_collision_reserve(self):
        goal = arms_along_body_goal(self.w)
        for name, arms, knees in [('arms', goal, None), ('knees', None, KNEE_GOAL_RAD)]:
            with self.subTest(variant=name):
                r = plan_posture(self.w, 1, True, .9, arms, knees)
                self.assertTrue(r['staticMotorContactFeasible'])
                self.assertLess(r['minimumRequiredMotorFraction'], .925)
                self.assertGreater(r['minimumSelfSeparationM'], .025)
                self.assertGreaterEqual(r['minimumJointReserveRad'], .02-1e-10)
                c = ArmKneeController.from_controller(Controller(self.w, 0, .032, True), r, arms is not None, knees is not None)
                c.trajectory_time_scale = 2.
                u, info = c.solve(0.)
                self.assertLessEqual(max(abs(u)), .95000001)
                self.assertLess(info['dynamicsResidual'], 1e-7)
                self.assertEqual(len(self.w.packet()['jointGroundRays']), 22)
                self.assertTrue(all(f['groundRays'] for f in self.w.packet()['feet']))

    def test_com_velocity_and_torso_rate_each_causally_change_bounded_offset(self):
        c = Controller(self.w, 0, .032, True)
        target = c.com0.copy()
        baseline, _ = balance_arm_offset(self.w, c, target)
        arr = self.w.arr
        for sensor in ('com', 'velocity', 'angular'):
            def changed(kind, shape):
                a = arr(kind, shape)
                if kind == 6 and sensor == 'com':
                    a = a.copy(); a[1, 0] += .005
                if kind == 1 and sensor in ('velocity', 'angular'):
                    a = a.copy(); a[0 if sensor == 'velocity' else 3] += .02
                return a
            self.w.arr = changed
            offset, _ = balance_arm_offset(self.w, c, target)
            with self.subTest(sensor=sensor):
                self.assertGreater(max(abs(offset-baseline)), 1e-5)
                self.assertLessEqual(max(abs(offset)), ARM_OFFSET_LIMIT_RAD)
        self.w.arr = arr

    def test_balance_feedback_changes_real_motor_commands_with_target_slew_bound(self):
        goal = arms_along_body_goal(self.w)
        r = plan_posture(self.w, 1, True, .9, goal, None)
        def controller():
            c = ArmKneeController.from_controller(Controller(self.w, 0, .032, True), r, True, False)
            c.trajectory_time_scale = 2.
            c.last_offset_time = self.w.l.lh_time(self.w.h)-.005
            return c
        c = controller(); baseline, _ = c.solve(.5)
        arr = self.w.arr
        def changed(kind, shape):
            a = arr(kind, shape)
            if kind == 6:
                a = a.copy(); a[1, 0] += .005
            return a
        self.w.arr = changed
        c = controller(); adjusted, info = c.solve(.5)
        self.w.arr = arr
        self.assertGreater(max(abs(adjusted-baseline)), 1e-7)
        self.assertLessEqual(max(abs(adjusted)), .95000001)
        self.assertLessEqual(max(abs(c.balance_offset)), ARM_TARGET_RATE_RADPS*.005+1e-14)
        self.assertTrue(info['postureExperiment'])


if __name__ == '__main__':
    unittest.main()
