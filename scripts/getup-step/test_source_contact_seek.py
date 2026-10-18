"""Contact-seek target rules; none of these tests claim a qualified motion."""
import unittest
from types import SimpleNamespace
import numpy as np
from source_upright_large_steps import ContactSeekingController, UprightController


class ContactSeekReference(unittest.TestCase):
    def setUp(self):
        class ReadOnlyContactWorld:
            mass = 40.
            h = None
            now = 0.
            clearance = .0001
            loads = np.array([0., 40.*9.81])

            def __init__(self):
                self.l = SimpleNamespace(lh_time=lambda h: self.now)

            def feet(self):
                return [dict(body=i, clearance=self.clearance) for i in (0, 1)]

            def arr(self, key, shape):
                assert key == 20
                return self.loads

        self.c = ContactSeekingController.__new__(ContactSeekingController)
        self.c.w = ReadOnlyContactWorld()
        self.c.swing_side, self.c.stance_side, self.c.swing_body = 0, 1, 0
        self.c.trajectory_time_scale = 2.

    def sample(self, time, phase='land', elapsed=None):
        self.c.w.now = time
        return self.c.landing_adjustment(time if elapsed is None else elapsed, phase)

    def test_no_seek_before_nominal_descent_or_when_not_near_ground(self):
        for phase in ('transfer', 'lift', 'translate', 'settle', 'stand'):
            self.assertEqual(self.sample(3., phase), (0., 0., 0.))
        self.assertEqual(self.sample(3.005, elapsed=1.999), (0., 0., 0.))
        self.c.w.clearance = .0030001
        self.assertEqual(self.sample(3.01), (0., 0., 0.))

    def test_only_actual_load_controls_search_with_existing_stance_admission(self):
        self.sample(2.)
        self.c.w.loads[1] = .8*self.c.w.mass*9.81-1e-6
        self.assertEqual(self.sample(2.005), (0., 0., 0.))
        self.c.w.loads[1] = .8*self.c.w.mass*9.81
        depth, rate, acc = self.sample(2.01)
        self.assertGreater(depth, 0.)
        self.assertGreater(rate, 0.)
        self.assertAlmostEqual(acc, self.c.SEEK_ACCEL_MPS2)

    def test_measured_contact_brakes_search_and_keeps_acquired_depth(self):
        self.sample(2.)
        for i in range(1, 81):
            self.sample(2.+i*.005)
        before = self.c._seek_depth
        self.c.w.loads[0] = self.c.SEEK_LOAD_N
        for i in range(81, 111):
            self.sample(2.+i*.005)
        depth = self.c._seek_depth
        self.assertGreaterEqual(depth, before)
        self.assertLessEqual(depth-before, .000151)
        self.assertEqual(self.c._seek_rate, 0.)
        self.assertEqual(self.sample(3.), (depth, 0., 0.))
        self.assertEqual(self.sample(3.005, 'settle'), (depth, 0., 0.))

    def test_phase_exit_brakes_with_admitted_low_load_and_integrates_travel(self):
        self.sample(2.)
        for i in range(1, 61):
            self.sample(2.+i*.005)
        before, previous = self.c._seek_depth, self.c._seek_rate
        self.c.w.loads[0] = 5.  # Original landing admission is 3N, seek goal12N.
        depth, rate, acc = self.sample(2.305, 'settle')
        self.assertGreater(depth, before)
        self.assertAlmostEqual(depth-before, .5*(previous+rate)*.005)
        self.assertLessEqual(abs(acc), self.c.SEEK_ACCEL_MPS2+1e-12)
        for i in range(62, 100):
            depth, rate, acc = self.sample(2.+i*.005, 'settle')
            self.assertLessEqual(abs(acc), self.c.SEEK_ACCEL_MPS2+1e-12)
        self.assertEqual(rate, 0.)
        self.assertEqual(self.sample(3., 'stand'), (depth, 0., 0.))

    def test_same_time_repeat_preserves_full_nonzero_acceleration_jet(self):
        self.sample(2.)
        jet = self.sample(2.005)
        self.assertGreater(jet[2], 0.)
        self.assertEqual(self.sample(2.005), jet)
        with self.assertRaises(ValueError):
            self.sample(2.004)

    def test_reference_search_is_bounded_and_does_not_accumulate_on_repeat_reads(self):
        self.sample(2.)
        previous_depth, previous_rate = 0., 0.
        for i in range(1, 601):
            now = 2.+i*.005
            depth, rate, acc = self.sample(now)
            self.assertGreaterEqual(depth, previous_depth)
            self.assertLessEqual(depth, self.c.SEEK_MAX_M)
            self.assertGreaterEqual(rate, 0.)
            self.assertLessEqual(rate, self.c.SEEK_SPEED_MPS)
            self.assertLessEqual(abs(acc), self.c.SEEK_ACCEL_MPS2+1e-12)
            self.assertEqual(self.sample(now), (depth, rate, acc))
            previous_depth, previous_rate = depth, rate
        self.assertEqual(depth, self.c.SEEK_MAX_M)
        self.assertAlmostEqual(rate, 0., delta=1e-15)
        self.assertTrue(self.c.contact_seek['originalLandingDeadlinePreserved'])


class IsolatedAnkleTransferTask(unittest.TestCase):
    def test_exact_reserved_joint_reference_is_transfer_only_and_torque_task(self):
        class ReadOnlyKinematics:
            def arr(self, key, shape):
                if key == 0:
                    q = np.zeros(28); q[7+7], q[7+13] = -.32, -.25
                    return q
                if key == 1:
                    v = np.zeros(27); v[6+7], v[6+13] = .02, -.03
                    return v
                if key == 5:
                    return np.repeat(np.eye(3)[None], 17, axis=0)
                raise AssertionError(key)

            def read(self, name, shape, body):
                return np.zeros(shape)

        c = UprightController.__new__(UprightController)
        c.w = ReadOnlyKinematics(); c.pelvis_yaw = 0.; c.rootR = np.eye(3)
        c.upright_start = c.upright_references = {str(b): np.eye(3).tolist() for b in (1, 4)}
        c.minimum_ankle_ready_rad = .035
        c.q0 = np.zeros(21); c.q0[[7, 13]] = [-.28, -.20]
        for phase in ('transfer', 'lift', 'translate', 'land', 'settle', 'stand'):
            tasks = []
            c.add_upright_tasks(0., phase, 72,
                lambda J, target, weight: tasks.append((J, target, weight)), [], [])
            self.assertEqual(len(tasks), 5 if phase == 'transfer' else 4)
            if phase == 'transfer':
                J, desired, weight = tasks[-1]
                np.testing.assert_array_equal(J, np.eye(27)[[13, 19]])
                np.testing.assert_allclose(desired, [20*.04-8*.02, 20*.05+8*.03])
                self.assertEqual(weight, 3.)


if __name__ == '__main__':
    unittest.main()
