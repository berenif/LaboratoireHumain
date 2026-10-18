"""Focused zero-integration checks for the opt-in upright/large-step extension.

These tests check the QP construction and native kinematics, not successful
stepping. A completed dynamic run is still required to establish feasibility.
"""
import importlib.util
import math
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from source_arm_knee_mobility import KNEE_GOAL_RAD, arms_along_body_goal, native_bytes
from plan_source_arm_knee import plan_posture
import source_opposed_arm_swing as swing
from source_step_probe import Controller, ROOT, World
from source_multistep_probe import posture_ready, unloaded_support_ready
from source_upright_large_steps import (
    DEADBAND_DEG, TILT_LIMIT_DEG, INHERITED_ENTRY_LIMIT_END_S,
    ExtendedArmMap, LargeStep, OpposedArmMultiStep,
    UprightController, body_orientation, cone_jet,
)


BASE = ROOT/'outputs/getup-source-step/upright-large-20261018/pilot-upright-120mm/model-session'
HELPER = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
FROZEN_COMMIT = '970273f'


class OrientationMath(unittest.TestCase):
    def test_pure_rotation_telemetry_and_yaw_independent_tilt(self):
        for axis, field in (('x', 'rollDeg'), ('y', 'pitchDeg')):
            for angle in (-9., 9.):
                R = Rotation.from_euler(axis, angle, degrees=True).as_matrix()
                measured = body_orientation(R)
                self.assertAlmostEqual(measured[field], angle, places=11)
                self.assertAlmostEqual(measured['tiltDeg'], abs(angle), places=11)
                yawed = Rotation.from_euler('z', 117., degrees=True).as_matrix()@R
                self.assertAlmostEqual(body_orientation(yawed)[field], angle, places=11)
                self.assertAlmostEqual(body_orientation(yawed)['tiltDeg'], abs(angle), places=11)
        measured = body_orientation(Rotation.from_euler('ZYX', [37., -8., 6.], degrees=True).as_matrix())
        np.testing.assert_allclose([measured[k] for k in ('yawDeg', 'pitchDeg', 'rollDeg')],
                                   [37., -8., 6.], atol=1e-12)
        self.assertAlmostEqual(body_orientation(Rotation.from_euler('z', 143., degrees=True).as_matrix())['tiltDeg'], 0.)

    def test_constant_omega_cone_jet_matches_finite_difference(self):
        rng = np.random.default_rng(531)
        for _ in range(30):
            up = Rotation.from_rotvec(rng.normal(size=3)*.13).apply([0., 0., 1.])
            J = rng.normal(size=(3, 27))*.1
            velocity = rng.normal(size=27)
            omega = J@velocity
            row, derivative, bias = cone_jet(up, J, omega, np.zeros(3))
            eps = 1e-4
            plus = Rotation.from_rotvec(omega*eps).apply(up)[2]
            minus = Rotation.from_rotvec(-omega*eps).apply(up)[2]
            self.assertAlmostEqual(derivative, (plus-minus)/(2*eps), delta=2e-8)
            self.assertAlmostEqual(bias, (plus-2*up[2]+minus)/eps**2, delta=6e-8)
            np.testing.assert_allclose(row, np.cross(up, [0., 0., 1.])@J, atol=1e-14)

    def test_angular_bias_has_correct_sign(self):
        up = Rotation.from_euler('xy', [7., -5.], degrees=True).apply([0., 0., 1.])
        J = np.eye(3, 27)
        omega = np.array([.3, -.2, .1])
        angular_bias = np.array([-.7, .4, .2])
        acceleration = np.linspace(-.4, .5, 27)
        row, _, bias = cone_jet(up, J, omega, angular_bias)
        direct = (np.cross(J@acceleration+angular_bias, up)
                  + np.cross(omega, np.cross(omega, up)))[2]
        self.assertAlmostEqual(float(row@acceleration+bias), direct, places=13)

    @staticmethod
    def captured_tasks(angle, axis='x', omega=None, angular_bias=None,
                       t=0., phase='stand', start=None, reference=None, actual=None):
        """Capture the real hook without a solver or any mutable native world."""
        class KinematicsOnly:
            def arr(self, key, shape):
                if key == 1:
                    v = np.zeros(27); v[:3] = np.zeros(3) if omega is None else omega
                    return v
                if key == 5:
                    rotations = np.repeat(np.eye(3)[None], 17, axis=0)
                    rotations[[1, 4]] = Rotation.from_euler(axis, angle, degrees=True).as_matrix()
                    if actual is not None:
                        for body in (1, 4):
                            rotations[body] = actual[str(body)]
                    return rotations
                raise AssertionError(key)

            def read(self, name, shape, body):
                if name == 'lh_body_jac':
                    out = np.zeros((6, 27)); out[3:, :3] = np.eye(3)
                    return out
                if name == 'lh_jdot':
                    return np.r_[np.zeros(3), np.zeros(3) if angular_bias is None else angular_bias]
                raise AssertionError(name)

        controller = UprightController.__new__(UprightController)
        controller.w = KinematicsOnly()
        controller.rootR = np.eye(3)
        controller.pelvis_yaw = 0.
        controller.upright_start = ({str(body): np.eye(3).tolist() for body in (1, 4)}
                                    if start is None else start)
        controller.upright_references = ({str(body): np.eye(3).tolist() for body in (1, 4)}
                                         if reference is None else reference)
        tasks, inequalities, bounds = [], [], []
        controller.add_upright_tasks(t, phase, 72,
                                    lambda J, target, weight: tasks.append((J, target, weight)),
                                    inequalities, bounds)
        return tasks, inequalities, bounds, controller.upright_control

    def test_soft_deadband_and_restorative_pitch_roll_sign(self):
        for axis, index in (('x', 0), ('y', 1)):
            tasks, _, _, _ = self.captured_tasks(DEADBAND_DEG*.8, axis)
            for _, target, _ in tasks[:2]:
                np.testing.assert_allclose(target, np.zeros(2), atol=1e-13)
            for sign in (-1., 1.):
                tasks, _, _, measured = self.captured_tasks(sign*5., axis)
                expected = np.zeros(2)
                expected[index] = -sign*40.*math.radians(5.-DEADBAND_DEG)
                for _, target, _ in tasks[:2]:
                    np.testing.assert_allclose(target, expected, atol=1e-12)
                self.assertEqual(len(measured), 2)
                self.assertTrue(all(row['softDeadbandDeg'] == DEADBAND_DEG for row in measured))

    def test_task_subtracts_measured_rate_and_jdot_bias(self):
        omega = np.array([.2, -.1, .3]); bias = np.array([.4, -.2, .1])
        tasks, _, _, _ = self.captured_tasks(0., omega=omega, angular_bias=bias)
        for _, target, _ in tasks[:2]:
            np.testing.assert_allclose(target, (-12.*omega-bias)[:2], atol=1e-13)

    def test_soft_yaw_hold_remains_separate_from_upright_tilt(self):
        tasks, _, _, _ = self.captured_tasks(9., 'z')
        self.assertEqual(len(tasks), 4)
        for _, target, _ in tasks[:2]:
            np.testing.assert_allclose(target, np.zeros(2), atol=1e-13)
        for _, target, _ in tasks[2:]:
            np.testing.assert_allclose(target, [-40.*math.radians(9.)], atol=1e-12)

    def test_cone_boundary_admits_restoring_and_rejects_outward_acceleration(self):
        for sign in (-1., 1.):
            _, inequalities, bounds, measured = self.captured_tasks(sign*TILT_LIMIT_DEG)
            restoring = np.zeros(72); restoring[0] = -sign
            outward = -restoring
            for row, bound in zip(inequalities, bounds):
                self.assertLessEqual(float(row@restoring), bound+1e-12)
                self.assertGreater(float(row@outward), bound+1e-5)
                np.testing.assert_array_equal(row[27:], np.zeros(45))
            self.assertTrue(all(abs(row['coneMargin']) < 1e-12 for row in measured))

    def test_transfer_starts_continuously_at_measured_orientation(self):
        initial = Rotation.from_euler('ZYX', [27., -2., 5.], degrees=True).as_matrix()
        start = {str(body): initial.tolist() for body in (1, 4)}
        reference = {str(body): Rotation.from_euler('ZYX', [27., -8. if body==1 else -3.21, 0.],
                                                   degrees=True).as_matrix().tolist() for body in (1, 4)}
        tasks, _, _, measured = self.captured_tasks(0., t=0., phase='transfer',
                                                   start=start, reference=reference, actual=start)
        for _, target, _ in tasks[:2]:
            np.testing.assert_allclose(target, np.zeros(2), atol=1e-12)
        for row in measured:
            self.assertEqual(row['target'], body_orientation(initial))

    def test_gait_tracks_reserved_reference_and_settle_returns_to_world_up(self):
        start = {str(body): Rotation.from_euler('ZYX', [27., -1., 2.], degrees=True).as_matrix().tolist()
                 for body in (1, 4)}
        reference = {str(body): Rotation.from_euler('ZYX', [27., -8. if body==1 else -3.21, 0.],
                                                   degrees=True).as_matrix().tolist() for body in (1, 4)}
        for phase, t in (('transfer', 4.), ('lift', 0.), ('translate', 1.), ('land', 2.), ('settle', 0.)):
            _, _, _, measured = self.captured_tasks(0., t=t, phase=phase, start=start, reference=reference)
            for body, row in zip((1, 4), measured):
                expected = body_orientation(np.array(reference[str(body)]))
                np.testing.assert_allclose([row['target'][key] for key in expected], list(expected.values()), atol=2e-12)
        for phase, t in (('settle', 4.), ('stand', 0.), ('stand', 9.)):
            _, _, _, measured = self.captured_tasks(0., t=t, phase=phase, start=start, reference=reference)
            for row in measured:
                self.assertAlmostEqual(row['target']['tiltDeg'], 0., delta=2e-6)
                self.assertAlmostEqual(row['target']['pitchDeg'], 0., delta=1e-12)
                self.assertAlmostEqual(row['target']['rollDeg'], 0., delta=1e-12)
                self.assertAlmostEqual(row['target']['yawDeg'], 27., delta=1e-12)

    def test_transfer_feedforward_rates_have_correct_sign_and_acceleration(self):
        from source_step_probe import clock
        initial = Rotation.from_euler('ZYX', [17., -1., 2.], degrees=True).as_matrix()
        target = Rotation.from_euler('ZYX', [17., -8., -2.], degrees=True).as_matrix()
        axis = Rotation.from_matrix(target@initial.T).as_rotvec()
        t = 1.25; s, ds, dds = clock(t, 4.)
        desired = Rotation.from_rotvec(axis*s).as_matrix()@initial
        start = {str(body): initial.tolist() for body in (1, 4)}
        reference = {str(body): target.tolist() for body in (1, 4)}
        actual = {str(body): desired.tolist() for body in (1, 4)}
        bias = np.array([.4, -.2, .1])
        tasks, _, _, _ = self.captured_tasks(0., omega=axis*ds, angular_bias=bias,
                                             t=t, phase='transfer', start=start,
                                             reference=reference, actual=actual)
        for _, rhs, _ in tasks[:2]:
            np.testing.assert_allclose(rhs, (axis*dds-bias)[:2], atol=1e-12)
        eps = 1e-4
        def pose_at(time):
            fraction = clock(time, 4.)[0]
            return Rotation.from_rotvec(axis*fraction).as_matrix()@initial
        angular_rate = Rotation.from_matrix(pose_at(t+eps)@pose_at(t-eps).T).as_rotvec()/(2*eps)
        np.testing.assert_allclose(angular_rate, axis*ds, atol=2e-10)
        rate_plus = Rotation.from_matrix(pose_at(t+2*eps)@pose_at(t).T).as_rotvec()/(2*eps)
        rate_minus = Rotation.from_matrix(pose_at(t)@pose_at(t-2*eps).T).as_rotvec()/(2*eps)
        np.testing.assert_allclose((rate_plus-rate_minus)/(2*eps), axis*dds, atol=2e-8)

    @staticmethod
    def decorated(torso_roll, pelvis_roll, phase, enabled=True,
                  elapsed=.1, capture=.01, joint_margin=.02):
        class RotationsOnly:
            def arr(self, key, shape):
                assert key == 5
                rotations = np.repeat(np.eye(3)[None], 17, axis=0)
                rotations[1] = Rotation.from_euler('x', torso_roll, degrees=True).as_matrix()
                rotations[4] = Rotation.from_euler('x', pelvis_roll, degrees=True).as_matrix()
                return rotations
        live = LargeStep.__new__(LargeStep)
        live.world = RotationsOnly(); live.upright_enabled = enabled
        live.quiet_run = 4000; live.failure = None
        row = dict(phase=phase, quietConsecutiveS=5., elapsedS=elapsed,
                   minimumComCaptureMarginM=capture, jointLimitMinimumMarginRad=joint_margin)
        with patch.object(OpposedArmMultiStep, 'decorate', lambda self, row: row):
            row = live.decorate(row)
        return live, row

    def test_sampled_tilt_exceeding_cone_is_explicit_failure(self):
        for torso, pelvis in ((TILT_LIMIT_DEG+.01, 0.), (0., TILT_LIMIT_DEG+.01)):
            live, _ = self.decorated(torso, pelvis, 'translate')
            self.assertEqual(live.failure, 'measured-upright-tilt-12deg')
        live, _ = self.decorated(0., TILT_LIMIT_DEG, 'translate')
        self.assertIsNone(live.failure)
        reference, _ = self.decorated(0., TILT_LIMIT_DEG+1., 'translate', enabled=False)
        self.assertIsNone(reference.failure)

    def test_final_quiet_requires_upright_pelvis_and_trunk_relative_pelvis(self):
        for torso, pelvis in ((0., 6.), (-4., 4.)):
            live, row = self.decorated(torso, pelvis, 'stand')
            self.assertEqual(live.quiet_run, 0)
            self.assertEqual(row['quietConsecutiveS'], 0.)
        live, row = self.decorated(2., 3., 'stand')
        self.assertEqual(live.quiet_run, 4000)
        self.assertEqual(row['quietConsecutiveS'], 5.)

    def test_measured_capture_guard_starts_after_exact_zero_time(self):
        for enabled in (False, True):
            for capture in (None, -1e-12):
                initial, _ = self.decorated(0., 0., 'transfer', enabled,
                                             elapsed=0., capture=capture)
                self.assertIsNone(initial.failure)
                live, _ = self.decorated(0., 0., 'transfer', enabled,
                                          elapsed=.00125, capture=capture)
                self.assertEqual(live.failure, 'measured-current-support-capture-outside')
            boundary, _ = self.decorated(0., 0., 'transfer', enabled,
                                          elapsed=.00125, capture=0.)
            self.assertIsNone(boundary.failure)

    def test_measured_joint_guard_preserves_only_exact_inherited_startup_window(self):
        self.assertEqual(INHERITED_ENTRY_LIMIT_END_S, .06375)
        for enabled in (False, True):
            for elapsed in (0., INHERITED_ENTRY_LIMIT_END_S-.00125, INHERITED_ENTRY_LIMIT_END_S):
                live, _ = self.decorated(0., 0., 'transfer', enabled,
                                          elapsed=elapsed, joint_margin=-1e-6)
                self.assertIsNone(live.failure)
            live, _ = self.decorated(0., 0., 'transfer', enabled,
                                      elapsed=INHERITED_ENTRY_LIMIT_END_S+.00125,
                                      joint_margin=-1e-12)
            self.assertEqual(live.failure, 'measured-new-joint-limit-overrun')
            boundary, _ = self.decorated(0., 0., 'transfer', enabled,
                                          elapsed=INHERITED_ENTRY_LIMIT_END_S+.00125,
                                          joint_margin=0.)
            self.assertIsNone(boundary.failure)


class ConservativeAnkleReadiness(unittest.TestCase):
    @staticmethod
    def ready_row():
        return dict(com=[0., 0., .8], comVelocity=[0., 0., 0.],
                    jointLimitMinimumMarginRad=.02, jointLimitMarginsRad=[.02]*21,
                    maximumBodyLinearMps=0., maximumBodyAngularRadps=0.,
                    feet=[dict(loadN=0.), dict(loadN=100.)])

    def test_default_posture_readiness_does_not_add_conservative_ankle_requirement(self):
        row = self.ready_row(); controller = SimpleNamespace(com1=np.array(row['com']))
        self.assertTrue(posture_ready(row, controller))
        controller.minimum_ankle_ready_rad = .035
        self.assertFalse(posture_ready(row, controller))

    def test_measured_pitch_ankle_reserves_must_both_meet_unload_threshold(self):
        row = self.ready_row()
        controller = SimpleNamespace(com1=np.array(row['com']), swing_side=0, stance_side=1,
                                     minimum_ankle_ready_rad=.035)
        row['jointLimitMarginsRad'][7] = row['jointLimitMarginsRad'][13] = .035
        self.assertTrue(posture_ready(row, controller))
        self.assertTrue(unloaded_support_ready(row, controller, 10.))
        for index in (7, 13):
            row['jointLimitMarginsRad'][index] = .035-1e-12
            self.assertFalse(posture_ready(row, controller))
            self.assertFalse(unloaded_support_ready(row, controller, 10.))
            row['jointLimitMarginsRad'][index] = .035

    def test_default_posture_readiness_matches_frozen_consistent_measurements(self):
        source = subprocess.check_output(
            ['git', 'show', FROZEN_COMMIT+':scripts/getup-step/source_multistep_probe.py'],
            cwd=ROOT, text=True)
        with tempfile.TemporaryDirectory(prefix='upright-frozen-readiness-') as folder:
            path = Path(folder)/'frozen_source_multistep_probe.py'; path.write_text(source)
            spec = importlib.util.spec_from_file_location('frozen_source_multistep_probe', path)
            frozen = importlib.util.module_from_spec(spec); spec.loader.exec_module(frozen)
            for change in ({}, {'com': [.009, 0., .8]}, {'comVelocity': [.05, 0., 0.]},
                           {'maximumBodyLinearMps': .051}, {'maximumBodyAngularRadps': .201},
                           {'jointLimitMinimumMarginRad': .004, 'jointLimitMarginsRad': [.004]*21}):
                row = self.ready_row(); row.update(change)
                controller = SimpleNamespace(com1=np.array([0., 0., .8]))
                self.assertEqual(posture_ready(row, controller), frozen.posture_ready(row, controller))


@unittest.skipUnless(BASE.exists() and HELPER.exists(), 'selected native session absent')
class NativeUprightQP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = World(BASE, HELPER)
        cls.world.restore(BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.world)
        cls.steps = cls.world.l.lh_dim(cls.world.h, 6)

    @classmethod
    def tearDownClass(cls):
        try:
            assert native_bytes(cls.world) == cls.before
            assert cls.world.l.lh_dim(cls.world.h, 6) == cls.steps
        finally:
            cls.world.close()

    def upright_controller(self, distance=.122):
        original = Controller(self.world, 0, distance, True, maximum_distance=.302)
        controller = UprightController.__new__(UprightController)
        controller.__dict__.update(original.__dict__)
        controller.trajectory_time_scale = 2.
        controller.joint_reserve = .01
        pelvis = self.world.arr(5, (17, 3, 3))[4]
        controller.pelvis_yaw = math.atan2(pelvis[1, 0], pelvis[0, 0])
        controller.upright_start = {str(body): self.world.arr(5, (17, 3, 3))[body].copy().tolist()
                                   for body in (1, 4)}
        controller.upright_references = self.reserved_reference()['uprightBodyReferencesSource']
        return controller

    def reserved_reference(self):
        cls = type(self)
        if not hasattr(cls, 'reference'):
            cls.arm_goal = arms_along_body_goal(self.world)
            cls.reference = plan_posture(self.world, 1, True, .9, cls.arm_goal,
                                         KNEE_GOAL_RAD, upright=1., ankle_reserve=.06)
        return cls.reference

    def test_planner_optin_reference_and_unchanged_default_parity(self):
        reference = self.reserved_reference()
        self.assertTrue(reference['staticOnly'])
        self.assertEqual(reference['physicsIntegrations'], 0)
        self.assertTrue(reference['staticMotorContactFeasible'])
        self.assertLess(reference['minimumRequiredMotorFraction'], .925)
        self.assertGreater(reference['minimumSelfSeparationM'], .025)
        self.assertEqual(reference['uprightPreferenceWeight'], 1.)
        self.assertEqual(reference['ankleReferenceReserveRad'], .06)
        q = np.array(reference['qReference'])
        margins = np.minimum(q-self.world.limits[:, 0], self.world.limits[:, 1]-q)
        self.assertTrue(np.all(margins[[7, 13]] >= .06-1e-8))
        self.assertAlmostEqual(reference['minimumJointReserveRad'], float(min(margins)), places=13)
        self.assertEqual(set(reference['uprightBodyReferencesSource']), {'1', '4'})
        for matrix in reference['uprightBodyReferencesSource'].values():
            self.assertLess(body_orientation(np.array(matrix))['tiltDeg'], TILT_LIMIT_DEG)
        source = subprocess.check_output(
            ['git', 'show', FROZEN_COMMIT+':scripts/getup-step/plan_source_arm_knee.py'],
            cwd=ROOT, text=True)
        with tempfile.TemporaryDirectory(prefix='upright-frozen-planner-') as folder:
            path = Path(folder)/'frozen_plan_source_arm_knee.py'; path.write_text(source)
            spec = importlib.util.spec_from_file_location('frozen_plan_source_arm_knee', path)
            frozen = importlib.util.module_from_spec(spec); spec.loader.exec_module(frozen)
            current = plan_posture(self.world, 1, True, .9, self.arm_goal, KNEE_GOAL_RAD)
            previous = frozen.plan_posture(self.world, 1, True, .9, self.arm_goal, KNEE_GOAL_RAD)
            self.assertNotIn('uprightBodyReferencesSource', current)
            self.assertNotIn('uprightPreferenceWeight', current)
            self.assertNotIn('ankleReferenceReserveRad', current)
            self.assertEqual(current, previous)
        self.assertEqual(native_bytes(self.world), self.before)

    def test_actual_native_ids_and_selected_model_are_pinned(self):
        self.assertEqual(self.world.l.lh_id(self.world.h, 1, b'torso'), 1)
        self.assertEqual(self.world.l.lh_id(self.world.h, 1, b'pelvis'), 4)
        from source_step_probe import sha
        self.assertEqual(sha(self.world.xml), '216d5c0159abf07139f5556c59e244ae77cb7557ba5256e301ebc5e1c8bbc399')
        self.assertEqual(self.steps, 0)

    def test_default_distance_guard_is_preserved_and_expansion_is_opt_in(self):
        for distance in (.122, .18, .24, .302):
            with self.assertRaises(AssertionError):
                Controller(self.world, 0, distance, True)
            controller = Controller(self.world, 0, distance, True, maximum_distance=.302)
            self.assertEqual(controller.step_distance, distance)
        with self.assertRaises(AssertionError):
            Controller(self.world, 0, .303, True, maximum_distance=.302)
        with self.assertRaises(AssertionError):
            Controller(self.world, 0, .302, True, maximum_distance=.303)

    def test_upright_solve_preserves_native_bytes_and_dynamics_motor_ray_gates(self):
        controller = self.upright_controller()
        for t in (0., 1., 4.):
            # This isolates the real upright hook in the unchanged torque QP;
            # the opposed-arm mapping has its own existing focused tests.
            command, info = Controller.solve(controller, t, 'transfer')
            self.assertTrue(np.isfinite(command).all())
            self.assertLessEqual(float(max(abs(command))), .95000001)
            self.assertLess(info['dynamicsResidual'], 1e-7)
            self.assertTrue(controller.self_avoidance)
            self.assertIsInstance(info['selfAvoidance'], list)
            self.assertTrue(info['jointGroundAdjustments'])
            self.assertTrue(all(row['hit'] and row['ownBodyExcluded']
                                for row in self.world.joint_ground_rays()))
            forces = np.array(info['predictedForcesN'])
            self.assertTrue(np.all(forces[:, 2] >= -1e-7))
            self.assertTrue(np.all(abs(forces[:, :2]) <= .6*forces[:, 2, None]+1e-7))
            self.assertEqual([row['body'] for row in controller.upright_control], ['torso', 'pelvis'])
            self.assertEqual(native_bytes(self.world), self.before)
            self.assertEqual(self.world.l.lh_dim(self.world.h, 6), self.steps)

    def test_native_cone_jet_matches_scratch_fk_finite_difference(self):
        q = self.world.arr(0, (28,)).copy()
        rng = np.random.default_rng(412)
        v = rng.normal(size=27)*.12; acceleration = rng.normal(size=27)*.2
        center = World(BASE, HELPER); scratch = World(BASE, HELPER)
        try:
            center.install_cold(dict(qpos=q.tolist(), qvel=v.tolist()))
            eps = 2e-4
            samples = {}
            root = Rotation.from_quat(q[[4, 5, 6, 3]])
            for sign in (-1, 1):
                dt = sign*eps; trial = q.copy()
                trial[:3] += dt*v[:3]+.5*dt**2*acceleration[:3]
                quaternion = (root*Rotation.from_rotvec(dt*v[3:6]+.5*dt**2*acceleration[3:6])).as_quat()
                trial[3:7] = quaternion[[3, 0, 1, 2]]
                trial[7:] += dt*v[6:]+.5*dt**2*acceleration[6:]
                scratch.install_cold(dict(qpos=trial.tolist(), qvel=(v+dt*acceleration).tolist()))
                samples[sign] = scratch.arr(5, (17, 3, 3)).copy()
            for body in (1, 4):
                J = center.read('lh_body_jac', (6, 27), body)[3:]
                angular_bias = center.read('lh_jdot', (6,), body)[3:]
                up = center.arr(5, (17, 3, 3))[body, :, 2]
                row, derivative, bias = cone_jet(up, J, J@v, angular_bias)
                plus, minus = samples[1][body, 2, 2], samples[-1][body, 2, 2]
                self.assertAlmostEqual(derivative, (plus-minus)/(2*eps), delta=2e-8)
                self.assertAlmostEqual(float(row@acceleration+bias),
                                       (plus-2*up[2]+minus)/eps**2, delta=2e-5)
            self.assertEqual(center.l.lh_dim(center.h, 6), 0)
            self.assertEqual(scratch.l.lh_dim(scratch.h, 6), 0)
        finally:
            center.close(); scratch.close()

    def test_unextended_solver_matches_frozen_commit_exactly(self):
        source = subprocess.check_output(
            ['git', 'show', FROZEN_COMMIT+':scripts/getup-step/source_step_probe.py'],
            cwd=ROOT, text=True)
        with tempfile.TemporaryDirectory(prefix='upright-frozen-') as folder:
            path = Path(folder)/'frozen_source_step_probe.py'; path.write_text(source)
            spec = importlib.util.spec_from_file_location('frozen_source_step_probe', path)
            frozen = importlib.util.module_from_spec(spec); spec.loader.exec_module(frozen)
            for self_avoidance in (False, True):
                current = Controller(self.world, 0, .062, self_avoidance)
                previous = frozen.Controller(self.world, 0, .062, self_avoidance)
                for phase in ('transfer', 'lift', 'translate', 'land', 'settle', 'stand'):
                    for t in (0., .5, 1.5):
                        actual, actual_info = current.solve(t, phase)
                        expected, expected_info = previous.solve(t, phase)
                        np.testing.assert_array_equal(actual, expected)
                        self.assertEqual(actual_info, expected_info)
        self.assertEqual(native_bytes(self.world), self.before)


@unittest.skipUnless(BASE.exists() and HELPER.exists(), 'selected native session absent')
class ExtendedArmMapping(unittest.TestCase):
    """Geometric map qualification, separate from dynamic step feasibility."""
    @classmethod
    def setUpClass(cls):
        cls.world = World(BASE, HELPER)
        cls.world.restore(BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.world)
        cls.q = cls.world.arr(0, (28,))[7:].copy()
        cls.goal = arms_along_body_goal(cls.world)
        cls.default = swing.SagittalArmMap(cls.q, cls.world.limits, cls.goal)
        cls.extended = ExtendedArmMap(cls.q, cls.world.limits, cls.goal)
        cls.legs = swing.limb_telemetry(cls.world, 'leg')[0]
        cls.reference = plan_posture(cls.world, 1, True, .9, cls.goal, KNEE_GOAL_RAD)
        import json
        cls.kin = swing.fk.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))
        source = subprocess.check_output(
            ['git', 'show', FROZEN_COMMIT+':scripts/getup-step/source_opposed_arm_swing.py'],
            cwd=ROOT, text=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='upright-frozen-arm-map-')
        path = Path(cls.temporary.name)/'frozen_source_opposed_arm_swing.py'; path.write_text(source)
        spec = importlib.util.spec_from_file_location('frozen_source_opposed_arm_swing', path)
        cls.frozen = importlib.util.module_from_spec(spec); spec.loader.exec_module(cls.frozen)
        cls.old_map = cls.frozen.SagittalArmMap(cls.q, cls.world.limits, cls.goal)

    @classmethod
    def tearDownClass(cls):
        try:
            assert native_bytes(cls.world) == cls.before
            assert cls.world.l.lh_dim(cls.world.h, 6) == 0
        finally:
            cls.world.close(); cls.temporary.cleanup()

    def vectors(self, arms):
        q = self.q.copy(); q[15:] = arms
        poses = self.kin.forward(q, np.eye(4))
        return np.array([swing.P@(poses[side+'_hand'][:3, 3]-poses[side+'_upper_arm'][:3, 3])
                         for side in swing.SIDES])

    def test_extended_endpoints_have_exact_fk_and_original_joint_reserve(self):
        minimum_reserve = math.inf
        for right_endpoint, left_endpoint in ((0, 0), (1, 1), (0, 1), (1, 0)):
            delta = np.array([self.extended.ranges[0, right_endpoint],
                              self.extended.ranges[1, left_endpoint]])
            arms, _, _ = self.extended.evaluate(delta)
            vectors = self.vectors(arms)
            expected = np.array([Rotation.from_rotvec([0., -angle, 0.]).apply(neutral)
                                 for angle, neutral in zip(delta, self.extended.neutral_vectors)])
            np.testing.assert_allclose(vectors, expected, atol=1e-7, rtol=0.)
            np.testing.assert_allclose(swing.pitch(vectors)-self.extended.neutral_pitch, delta,
                                       atol=2e-7, rtol=0.)
            reserve = np.minimum(arms-self.world.limits[15:, 0], self.world.limits[15:, 1]-arms)
            minimum_reserve = min(minimum_reserve, float(min(reserve)))
        self.assertGreaterEqual(minimum_reserve, .02-1e-8)
        self.assertLess(self.extended.maximum_fit_error_m, 1e-7)

    def test_extended_interpolation_preserves_joint_reserve_between_nodes(self):
        minimum_reserve = math.inf
        for delta in np.linspace(self.extended.ranges[:, 0], self.extended.ranges[:, 1], 1001):
            arms = self.extended.evaluate(delta)[0]
            reserve = np.minimum(arms-self.world.limits[15:, 0], self.world.limits[15:, 1]-arms)
            minimum_reserve = min(minimum_reserve, float(min(reserve)))
        self.assertGreaterEqual(minimum_reserve, .02-1e-8)

    def test_extended_interpolation_derivatives_match_finite_differences(self):
        eps = 1e-5
        for fraction in (.01, .5, .99):
            delta = self.extended.ranges[:, 0]+fraction*np.ptp(self.extended.ranges, axis=1)
            value, first, second = self.extended.evaluate(delta)
            plus = self.extended.evaluate(delta+eps)[0]
            minus = self.extended.evaluate(delta-eps)[0]
            np.testing.assert_allclose((plus-minus)/(2*eps), first, atol=1e-8, rtol=0.)
            np.testing.assert_allclose((plus-2*value+minus)/eps**2, second, atol=2e-5, rtol=0.)

    def test_extended_out_of_domain_rejects_without_clipping(self):
        for side in (0, 1):
            for bound, direction in ((0, -1.), (1, 1.)):
                delta = np.zeros(2); delta[side] = self.extended.ranges[side, bound]+direction*1e-6
                with self.assertRaisesRegex(AssertionError, 'verified arm map'):
                    self.extended.evaluate(delta)
        receipt = self.extended.receipt()
        self.assertFalse(receipt['clippingEnabled'])
        self.assertEqual(receipt['angularGain'], -1.)
        self.assertEqual(receipt['sideOrder'], ['right', 'left'])
        self.assertEqual(receipt['verifiedSwingDeltaRangeBySideRad'], self.extended.ranges.tolist())
        self.assertTrue(np.all(self.extended.ranges[:, 0] < -.45))
        self.assertTrue(np.all(self.extended.ranges[:, 1] > .45))
        self.assertNotIn('verifiedSwingDeltaRangeRad', receipt)

    def test_literal_default_grid_coefficients_receipt_and_domain_match_frozen(self):
        np.testing.assert_array_equal(self.default.grid, self.old_map.grid)
        self.assertEqual(self.default.receipt(), self.old_map.receipt())
        self.assertFalse(self.default.extended)
        for current, previous in zip(self.default.curves, self.old_map.curves):
            np.testing.assert_array_equal(current.x, previous.x)
            np.testing.assert_array_equal(current.c, previous.c)
        for delta in ([-.45, .45], [.37, -.31], [0., 0.]):
            for current, previous in zip(self.default.evaluate(np.array(delta)),
                                          self.old_map.evaluate(np.array(delta))):
                np.testing.assert_array_equal(current, previous)
        for delta in ([-.450001, 0.], [0., .450001]):
            with self.assertRaisesRegex(AssertionError, 'verified arm map'):
                self.default.evaluate(np.array(delta))

    def test_default_opposed_arm_targets_and_motor_commands_match_frozen_exactly(self):
        origin = self.legs+np.array([.13, -.08])
        current = swing.OpposedArmController.from_controller(
            Controller(self.world, 0, .032, True), self.reference, self.default, origin, True)
        previous = self.frozen.OpposedArmController.from_controller(
            Controller(self.world, 0, .032, True), self.reference, self.old_map, origin, True)
        for controller in (current, previous):
            controller.trajectory_time_scale = 2.; controller.joint_reserve = .01
        for phase, t in (('transfer', 0.), ('transfer', 2.), ('transfer', 4.), ('translate', 1.)):
            actual, actual_info = current.solve(t, phase)
            expected, expected_info = previous.solve(t, phase)
            np.testing.assert_array_equal(actual, expected)
            self.assertEqual(actual_info, expected_info)
            self.assertLessEqual(float(max(abs(actual))), .95000001)
            self.assertLess(actual_info['dynamicsResidual'], 1e-7)
        self.assertEqual(native_bytes(self.world), self.before)


if __name__ == '__main__':
    unittest.main()
