"""Pure robust-support geometry/history/reference tests; no native steps."""
import ast
import copy
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import pickle
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

import numpy as np
from scipy.spatial import ConvexHull

import source_robust_support_steps as robust
from source_cadenced_ten_steps import CadencedController
from source_multistep_probe import posture_ready
from source_step_probe import ROOT, World, clock


CORNERS = np.array([[0., 0., 0.], [.1, 0., 0.], [.1, .1, 0.], [0., .1, 0.]])
SURVIVORS = [1, 2, 3]


class FakeWorld:
    def __init__(self):
        self.now = 0.
        self.mass = 40.
        self.h = None
        self.velocity = np.zeros(27)
        self.com = np.array([.049, .049, .8])
        self.loads = np.zeros(17); self.loads[10] = .9*self.mass*9.81
        self.bias = np.array([.001, -.002, .003])
        self.supports = CORNERS[:, :2].copy()
        self.l = SimpleNamespace(lh_time=lambda _: self.now,
                                 lh_support_count=lambda _: len(self.supports))
    def feet(self):
        foot = dict(points=CORNERS+np.array([0., 0., .027]), radius=.027, body=10)
        return [foot, foot]
    def ground_height(self, point):
        return 0.
    def arr(self, key, shape):
        if key == 1:
            return self.velocity
        if key == 6:
            out = np.zeros(shape); out[1] = self.com; return out
        if key == 20:
            return self.loads
        if key == 21:
            out = np.zeros(shape); out[:2*len(self.supports)] = self.supports.reshape(-1); return out
        raise AssertionError('unexpected read '+str(key))
    def read(self, name, shape, *args):
        if name == 'lh_com_jac':
            return np.eye(3, 27)
        if name == 'lh_jdot':
            return self.bias.copy()
        raise AssertionError('unexpected kinematic request '+name)


def controller():
    c = robust.RobustSupportController.__new__(robust.RobustSupportController)
    c.w = FakeWorld()
    c.stance_side = 1
    c.com1 = c.w.com.copy()
    c.finalCom = np.array([.06, .055, .8])
    c.phase_durations = dict(transfer=2., settle=1.52)
    # Geometry/history tests supply an already-agreed causal risk. Predictor
    # construction and exact native purity are tested separately below.
    c._forecast_unload_risk = lambda t, corners, clearance: (
        int(np.argmax(clearance)), {'mockForecast': True})
    return c


def selected_controller():
    c = controller()
    for now in np.linspace(1.9, 2., 21):
        c.w.now = float(now)
        c.adjust_com_reference(float(now), 'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))
    if not c.robust_support['selected']:
        raise AssertionError('test fixture did not select')
    return c


class InsetProjection(unittest.TestCase):
    def setUp(self):
        self.eq = ConvexHull(CORNERS[SURVIVORS, :2]).equations

    def test_feasible_point_is_preserved_without_aliasing(self):
        point = np.array([.075, .075])
        result = robust.project_inset(point, self.eq, .004)
        np.testing.assert_array_equal(result, point)
        self.assertIsNot(result, point)

    def test_nearest_edge_projection_is_analytic(self):
        point = np.array([.049, .049])
        result = robust.project_inset(point, self.eq, .004)
        expected = np.repeat((.1+.004*np.sqrt(2.))/2., 2)
        np.testing.assert_allclose(result, expected, atol=1e-14)
        self.assertGreaterEqual(float(np.min(-(self.eq[:, :2]@result+self.eq[:, 2]))), .004-1e-12)

    def test_nearest_vertex_intersection_is_analytic(self):
        result = robust.project_inset(np.array([.2, .2]), self.eq, .004)
        np.testing.assert_allclose(result, [.096, .096], atol=1e-14)

    def test_empty_inset_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'empty robust support inset'):
            robust.project_inset(np.array([.05, .05]), self.eq, .1)

    def test_current_support_endpoint_matching_has_no_virtual_corners(self):
        w = FakeWorld(); w.supports = CORNERS[[1, 2, 3], :2].copy()
        corners, present, clearances = robust.actual_stance_patch(w, 1)
        np.testing.assert_array_equal(corners, CORNERS)
        np.testing.assert_array_equal(present, [False, True, True, True])
        np.testing.assert_array_equal(clearances, np.zeros(4))
        w.supports[0] += [.001001, 0.]
        self.assertFalse(robust.actual_stance_patch(w, 1)[1][1])

    def test_polygon_vertices_follow_actual_support_xy_not_nominal_endpoints(self):
        w = FakeWorld()
        displacement = np.array([[.0004, -.0002], [-.0003, .0002],
                                 [.0001, .0005], [-.0002, -.0003]])
        w.supports += displacement
        matched, present, clearance = robust.actual_stance_patch(w, 1)
        np.testing.assert_array_equal(present, np.ones(4, dtype=bool))
        np.testing.assert_array_equal(matched[:, :2], w.supports)
        self.assertFalse(np.array_equal(matched[:, :2], CORNERS[:, :2]))
        # Height ranking remains that of the physical endpoint, independent of
        # the small XY association displacement used to construct the polygon.
        np.testing.assert_array_equal(clearance, np.zeros(4))

    def test_duplicate_actual_point_associations_cannot_form_two_corners(self):
        w = FakeWorld()
        geometric = CORNERS.copy(); geometric[1] = [.0005, 0., 0.]
        w.feet = lambda: [dict(points=geometric+[0., 0., .027], radius=.027, body=10)]*2
        w.supports = np.array([[.00025, 0.], [.1, .1], [0., .1]])
        matched, present, _ = robust.actual_stance_patch(w, 1)
        np.testing.assert_array_equal(matched[0, :2], matched[1, :2])
        np.testing.assert_array_equal(present, [False, False, True, True])
        self.assertEqual(len(np.unique(matched[present, :2], axis=0)), int(present.sum()))


class CausalWeakCornerHistory(unittest.TestCase):
    def observe(self, c, times, *, weak_votes=None, bad_survivor=None, weak_present=True):
        for index, now in enumerate(times):
            c.w.now = float(now)
            present = np.ones(4, dtype=bool); present[0] = weak_present
            if index == bad_survivor:
                present[1] = False
            weak = 0 if weak_votes is None else weak_votes[index]
            clearance = np.zeros(4); clearance[weak] = .001
            with patch.object(robust, 'actual_stance_patch',
                              return_value=(CORNERS.copy(), present, clearance)):
                c.adjust_com_reference(float(now), 'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))

    def test_selection_requires_nominal_transfer_and_full_prior_hundred_ms(self):
        c = controller()
        self.observe(c, np.linspace(1.8, 1.9, 21))
        self.assertFalse(c.robust_support['selected'])
        c = controller()
        self.observe(c, np.linspace(2., 2.095, 20))
        self.assertFalse(c.robust_support['selected'])
        self.observe(c, [2.1])
        self.assertTrue(c.robust_support['selected'])
        self.assertEqual(c.robust_support['historySamples'], 21)
        self.assertLessEqual(c.robust_support['historySourceEndS'], c.w.now)
        self.assertGreaterEqual(c.w.now-c.robust_support['historySourceStartS'], .1-1e-9)

    def test_eighty_percent_vote_threshold_is_applied_to_actual_history(self):
        for votes, expected in (([0]*17+[1]*4, True), ([0]*16+[1]*5, False)):
            with self.subTest(weak_votes=votes.count(0)):
                c = controller(); self.observe(c, np.linspace(1.9, 2., 21), weak_votes=votes)
                self.assertEqual(c.robust_support['selected'], expected)

    def test_any_missing_survivor_in_history_blocks_selection(self):
        c = controller(); self.observe(c, np.linspace(1.9, 2., 21), bad_survivor=10)
        self.assertFalse(c.robust_support['selected'])

    def test_weak_corner_itself_need_not_be_supported(self):
        c = controller(); self.observe(c, np.linspace(1.9, 2., 21), weak_present=False)
        self.assertTrue(c.robust_support['selected'])
        self.assertEqual(c.robust_support['weakCorner'], 0)
        self.assertEqual(c.robust_support['survivingCorners'], SURVIVORS)

    def test_sub_eighty_percent_stance_load_blocks_selection(self):
        c = controller(); c.w.loads[10] = .8*c.w.mass*9.81-1e-6
        self.observe(c, np.linspace(1.9, 2., 21))
        self.assertFalse(c.robust_support['selected'])

    def test_selection_is_frozen_and_does_not_read_future_observations(self):
        c = selected_controller(); before = copy.deepcopy(c.robust_support)
        history = list(c._robust_history)
        c.w.now = 999.
        with patch.object(robust, 'actual_stance_patch', side_effect=AssertionError('future read')):
            c.adjust_com_reference(3., 'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))
        self.assertEqual(c.robust_support, before)
        self.assertEqual(len(c._robust_history), len(history))

    def test_required_shift_beyond_twelve_mm_blocks_admission_and_keeps_observing(self):
        c = controller(); c.com1[:2] = [.01, .01]
        self.observe(c, np.linspace(1.9, 2., 21))
        self.assertFalse(c.robust_support['selected'])
        self.assertEqual(c.robust_support['lastRejectedWeakCorner'], 0)
        self.assertGreater(c.robust_support['lastRejectedCorrectionM'], .012)
        self.assertIn('continue observing', c.robust_support['lastAdmissionRejection'])
        self.assertFalse(hasattr(c, '_robust_delta'))
        self.assertFalse(c.robust_support_ready(dict(com=[.01, .01, .8], comVelocity=[0., 0., 0.])))
        last_observation = c._robust_history[-1][0]
        self.observe(c, [2.005])
        self.assertGreater(c._robust_history[-1][0], last_observation)
        self.assertFalse(c.robust_support['selected'])

    def test_rejected_oversize_can_recover_after_causal_weak_corner_changes(self):
        c = controller(); c.com1[:2] = [.01, .01]
        self.observe(c, np.linspace(1.9, 2., 21))
        self.assertFalse(c.robust_support['selected'])
        # Excluding corner2 instead leaves this target inside the inset. A few
        # new votes cannot select it prematurely or read ahead in the window.
        self.observe(c, np.linspace(2.005, 2.02, 4), weak_votes=[2]*4)
        self.assertFalse(c.robust_support['selected'])
        observed_times = []
        def observed_patch(world, side):
            observed_times.append(world.now)
            clearance = np.zeros(4); clearance[2] = .001
            return CORNERS.copy(), np.ones(4, dtype=bool), clearance
        with patch.object(robust, 'actual_stance_patch', side_effect=observed_patch):
            for now in np.linspace(2.025, 2.125, 21):
                c.w.now = float(now)
                c.adjust_com_reference(float(now), 'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))
                if c.robust_support['selected']:
                    self.assertLessEqual(c.robust_support['historySourceEndS'], now)
        self.assertTrue(c.robust_support['selected'])
        self.assertEqual(c.robust_support['weakCorner'], 2)
        self.assertGreaterEqual(c.robust_support['selectedAtTransferS'], 2.08)
        self.assertGreaterEqual(c.robust_support['riskAgreementFrequency'], .8)
        self.assertEqual(observed_times[-1], c.robust_support['historySourceEndS'])
        self.assertTrue(all(a <= b for a, b in zip(observed_times, observed_times[1:])))
        self.assertEqual(c.robust_support['supportPatchConstruction'],
                         'three-unique-nearest-actual-support-xy')
        self.assertLessEqual(np.linalg.norm(c._robust_delta), .012)

    def test_low_load_gap_cannot_recycle_earlier_risk_history(self):
        c = controller(); self.observe(c, np.linspace(1.9, 1.995, 20))
        c.w.loads[10] = .8*c.w.mass*9.81-1e-6
        self.observe(c, [2.])
        self.assertEqual(len(c._robust_history), 0)
        c.w.loads[10] = .9*c.w.mass*9.81
        self.observe(c, [2.005])
        self.assertFalse(c.robust_support['selected'])
        self.observe(c, np.linspace(2.01, 2.12, 23))
        self.assertTrue(c.robust_support['selected'])
        self.assertGreaterEqual(c.robust_support['selectedAtTransferS'], 2.105-1e-9)

    def test_observation_gap_cannot_recycle_a_prior_hundred_ms_window(self):
        c = controller(); self.observe(c, np.linspace(1.9, 1.995, 20))
        self.observe(c, [2.01])  # A missing200Hz observation breaks continuity.
        self.assertFalse(c.robust_support['selected'])
        self.assertEqual(len(c._robust_history), 1)

    def test_disagreement_votes_are_not_removed_from_frequency_denominator(self):
        for agreed, expected in ((17, True), (16, False)):
            with self.subTest(agreed=agreed):
                c = controller(); calls = []
                def forecast(t, corners, clearance):
                    calls.append(t)
                    return (0 if len(calls) <= agreed else -1), {'mockForecast': True}
                c._forecast_unload_risk = forecast
                self.observe(c, np.linspace(1.9, 2., 21))
                self.assertEqual(c.robust_support['selected'], expected)


class SmoothReferenceAndAdmission(unittest.TestCase):
    def test_shift_is_bounded_and_c2_at_start_and_end(self):
        c = selected_controller()
        self.assertLessEqual(np.linalg.norm(c._robust_delta), .012)
        self.assertGreaterEqual(c._robust_duration, 1.)
        speeds, accelerations = [], []
        for time in np.linspace(0., c._robust_duration, 1001):
            _, ds, dds = clock(time, c._robust_duration)
            speeds.append(np.linalg.norm(ds*c._robust_delta))
            accelerations.append(np.linalg.norm(dds*c._robust_delta))
        self.assertLessEqual(max(speeds), .025+1e-12)
        self.assertLessEqual(max(accelerations), .15+1e-12)
        for time, position in ((0., c.com1), (c._robust_duration, c.com1+c._robust_delta)):
            target, velocity, acceleration = c.adjust_com_reference(
                c._robust_start+time, 'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))
            np.testing.assert_allclose(target, position, atol=1e-14)
            np.testing.assert_allclose(velocity, 0., atol=1e-14)
            np.testing.assert_allclose(acceleration, 0., atol=1e-13)

    def test_lift_land_settle_and_stand_offsets_have_c2_phase_boundaries(self):
        c = selected_controller()
        transfer_end = c.adjust_com_reference(c._robust_start+c._robust_duration,
            'transfer', c.com1.copy(), np.zeros(3), np.zeros(3))
        for phase, t in (('lift', 0.), ('translate', 0.), ('land', 0.), ('settle', 0.)):
            result = c.adjust_com_reference(t, phase, c.com1.copy(), np.zeros(3), np.zeros(3))
            for actual, expected in zip(result, transfer_end):
                np.testing.assert_allclose(actual, expected, atol=1e-13)
        settle_end = c.adjust_com_reference(c.phase_duration('settle'), 'settle',
            c.finalCom.copy(), np.zeros(3), np.zeros(3))
        stand = c.adjust_com_reference(0., 'stand', c.finalCom.copy(), np.zeros(3), np.zeros(3))
        for actual, expected in zip(settle_end, stand):
            np.testing.assert_allclose(actual, expected, atol=1e-13)
        np.testing.assert_array_equal(stand[0], c.finalCom)

    def test_admission_requires_finished_clock_actual_survivors_and_three_mm(self):
        c = selected_controller()
        eq = ConvexHull(CORNERS[SURVIVORS, :2]).equations
        row = dict(com=[.06, .06, .8], comVelocity=[0., 0., 0.])
        self.assertFalse(c.robust_support_ready(row))
        c._robust_clock = (1., 0., 0.)
        self.assertTrue(c.robust_support_ready(row))
        for reserve, expected in ((.003001, True), (.002999, False)):
            point = robust.project_inset([.049, .049], eq, reserve)
            row['com'][:2] = point.tolist()
            self.assertEqual(c.robust_support_ready(row), expected)
        c.w.supports = CORNERS[[0, 2, 3], :2]
        self.assertFalse(c.robust_support_ready(row))

    def test_transfer_goal_adds_offset_without_modifying_nominal_goal(self):
        c = selected_controller(); before = c.com1.copy()
        np.testing.assert_array_equal(c.transfer_ready_goal(), c.com1+c._robust_delta)
        np.testing.assert_array_equal(c.com1, before)


class MovingReserveBarriers(unittest.TestCase):
    def test_cp_and_com_inequalities_include_exact_reserve_derivatives(self):
        c = selected_controller(); c._robust_clock = (.37, .6, -.2)
        v = np.zeros(27); v[:3] = [.01, -.005, .002]
        J = np.eye(3, 27); com = c.w.com.copy()
        rows, bounds = [], []
        with patch.object(CadencedController, 'add_capture_constraints', return_value=None):
            c.add_capture_constraints('transfer', 72, J, com, v, rows, bounds)
        self.assertEqual(len(rows), 6)
        tau = math.sqrt(com[2]/9.81); tau_rate = v[2]/(2.*9.81*tau)
        acceleration = np.linspace(-.2, .3, 27)
        equations = ConvexHull(CORNERS[SURVIVORS, :2]).equations
        dr = .003-c._robust_initial_reserve
        reserve = c._robust_initial_reserve+.37*dr
        rdot, rddot = .6*dr, -.2*dr
        cp = com[:2]+tau*v[:2]
        for index, face in enumerate(equations):
            normal, offset = face[:2], face[2]
            cp_margin = -float(normal@cp+offset)
            com_margin = -float(normal@com[:2]+offset)
            cp_residual = normal@((1.+tau_rate)*v[:2]+tau*(J[:2]@acceleration+c.w.bias[:2]))+rdot-8.*(cp_margin-reserve)
            com_residual = normal@(J[:2]@acceleration+c.w.bias[:2])+16.*normal@v[:2]+16.*rdot+rddot-64.*(com_margin-reserve)
            self.assertAlmostEqual(float(rows[2*index][:27]@acceleration-bounds[2*index]), cp_residual)
            self.assertAlmostEqual(float(rows[2*index+1][:27]@acceleration-bounds[2*index+1]), com_residual)
            np.testing.assert_array_equal(rows[2*index][27:], np.zeros(45))

    def test_disappeared_survivor_never_adds_virtual_triangle_constraints(self):
        c = selected_controller(); c.w.supports = CORNERS[[0, 2, 3], :2]
        rows, bounds = [], []
        with patch.object(CadencedController, 'add_capture_constraints', return_value=None):
            c.add_capture_constraints('lift', 72, np.eye(3, 27), c.w.com, c.w.velocity, rows, bounds)
        self.assertEqual(rows, []); self.assertEqual(bounds, [])
        self.assertEqual(c.robust_support_barrier, [])


class CausalUnloadPredictor(unittest.TestCase):
    def test_predictor_bypasses_arm_history_and_keeps_shared_inputs_unchanged(self):
        c = controller(); w = c.w
        c.swing_side = 0
        c.floorTargets = [.0265, .0266]
        c.targets = [(np.array([0., 0., .027]), np.eye(3))]*2
        c.radii = [.027, .027]
        c.q0 = np.arange(21, dtype=float)
        c.arm_posture, c.arm_velocity, c.arm_acceleration = np.arange(6.), np.arange(6.)/10., np.arange(6.)/20.
        c.previous_time, c.previous_leg_rate = 17., np.array([.1, -.2])
        c._mapping = SimpleNamespace(ranges=np.array([[-1., 1.]]))
        c.unload_admitted = False
        native_response = np.zeros(33); native_response[:4] = [.2, .1, -.2, -.1]
        w.l.lh_motor_response = lambda *args: None
        w.l.lh_dim = lambda *args: 1600
        old_read, old_arr = w.read, w.arr
        def read(name, shape, *args):
            if name == 'lh_motor_response':
                return native_response.copy()
            if name == 'lh_body_jac':
                return np.zeros((6, 27))
            if name == 'lh_jdot' and shape == (6,):
                return np.zeros(6)
            if name == 'lh_point_jac':
                point = np.ctypeslib.as_array(args[1], shape=(3,))
                index = int(np.argmin(np.linalg.norm(CORNERS-point, axis=1)))
                result = np.zeros((3, 27)); result[2, index] = 1.
                return result
            return old_read(name, shape, *args)
        w.read = read
        w.arr = lambda key, shape: np.zeros(shape) if key == 3 else old_arr(key, shape)
        before = {key: copy.deepcopy(value) for key, value in vars(c).items()
                  if key not in ('w', '_forecast_unload_risk')}
        normal = np.array([20., 30., 40., 50.])
        proposed = np.linspace(-.2, .2, 21)
        calls = []
        def solve(probe, t, phase):
            calls.append((t, phase))
            self.assertIs(type(probe), CadencedController)
            self.assertIs(probe.w, w)
            self.assertTrue(probe.unload_admitted)
            self.assertIsNot(probe.floorTargets, c.floorTargets)
            self.assertEqual(probe.floorTargets[0], .0271)
            self.assertEqual(probe.floorTargets[1], c.floorTargets[1])
            # Shared arrays are read-only inputs to the actual base solve.
            self.assertIs(probe.q0, c.q0)
            forces = np.zeros((8, 3)); forces[4:, 2] = normal
            probe.capture_barrier = []; probe.arm_range_barrier = []
            return proposed.copy(), {'predictedForcesN': forces.tolist()}
        from source_opposed_arm_swing import OpposedArmController
        corners, clearance = CORNERS.copy(), np.zeros(4)
        with patch.object(robust.Controller, 'solve', solve), \
             patch.object(robust, 'native_bytes', return_value=b'fake-exact-native-state'), \
             patch.object(OpposedArmController, 'solve', side_effect=AssertionError('arm history called')):
            risk, telemetry = robust.RobustSupportController._forecast_unload_risk(c, 2., corners, clearance)
            self.assertEqual(risk, 0)
            self.assertEqual(telemetry['pressureRiskCorner'], 0)
            self.assertEqual(telemetry['clearanceRiskCorner'], 0)
            self.assertFalse(telemetry['proposedControlApplied'])
            self.assertEqual(telemetry['physicsRolloutIntegrations'], 0)
            np.testing.assert_allclose(telemetry['localClearanceForecastM'], .005*native_response[:4])
            native_response[:4] = [.1, .3, -.2, -.1]
            risk, telemetry = robust.RobustSupportController._forecast_unload_risk(c, 2., corners, clearance)
            self.assertEqual(risk, -1)
            self.assertNotEqual(telemetry['pressureRiskCorner'], telemetry['clearanceRiskCorner'])
        self.assertEqual(calls, [(2., 'transfer'), (2., 'transfer')])
        for key, value in before.items():
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(getattr(c, key), value)
            elif key == 'targets':
                for actual, expected in zip(c.targets, value):
                    for a, b in zip(actual, expected):
                        np.testing.assert_array_equal(a, b)
            elif key == '_mapping':
                np.testing.assert_array_equal(c._mapping.ranges, value.ranges)
            else:
                self.assertEqual(getattr(c, key), value)
        self.assertEqual(set(vars(c))-set(before)-{'w', '_forecast_unload_risk'}, {'_robust_payloads'})
        self.assertEqual(len(c._robust_payloads), 2)
        self.assertEqual(c._robust_payloads.maxlen, 22)
        for record in c._robust_payloads:
            self.assertEqual(zlib.decompress(record['compressed']), b'fake-exact-native-state')
            self.assertEqual(record['sourceStep'], 1600)
            self.assertEqual(record['payloadSHA256'], hashlib.sha256(b'fake-exact-native-state').hexdigest())
        np.testing.assert_array_equal(corners, CORNERS)
        np.testing.assert_array_equal(clearance, np.zeros(4))


@unittest.skipUnless(all((Path(f'/tmp/cadence-robust-8-pilot{i}')/'run/generation-001/final.bin').exists()
                         for i in (1, 2)), 'optional preserved native pilot states absent')
class NativeUnloadPredictorPurity(unittest.TestCase):
    """Two current-state forward solves; never mj_step or control application."""
    def test_saved_pilot_states_agree_without_native_or_controller_mutations(self):
        from source_arm_knee_mobility import native_bytes
        from source_upright_large_steps import ExtendedArmMap
        reports = []
        for pilot in (1, 2):
            with self.subTest(pilot=pilot):
                folder = Path(f'/tmp/cadence-robust-8-pilot{pilot}')
                journal = folder/'run/generation-001'
                record = json.loads((journal/'controller-step-01.json').read_text())
                note = json.loads((journal/'control.json').read_text())[-1]
                mapping = json.loads((journal/'arm-mapping.json').read_text())
                helper = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
                w = World(folder/'model-session', helper)
                try:
                    w.restore(journal/'final.bin')
                    counter = w.l.lh_dim(w.h, 6); now = w.l.lh_time(w.h)
                    c = CadencedController.__new__(CadencedController)
                    for key, value in record.items():
                        if isinstance(value, list):
                            if key == 'targets':
                                value = [tuple(np.array(x) for x in pair) for pair in value]
                            elif not value or not isinstance(value[0], dict):
                                value = np.array(value)
                        setattr(c, key, value)
                    c.w = w
                    c._mapping = ExtendedArmMap(np.array(record['posture_start']), w.limits, mapping['goalRad'])
                    c.previous_time = now+(note['t']-counter*.00125)
                    c.previous_leg_rate = np.array(note['opposedArmSwing']['measuredLegPitchRateRadps'])
                    seek = note['contactSeek']
                    c._seek_time, c._seek_depth, c._seek_rate = c.previous_time, seek['depthM'], seek['rateMps']
                    c._seek_jet = (c._seek_depth, c._seek_rate, seek['accelerationMps2'])
                    c.contact_seek = seek.copy()
                    native_before = native_bytes(w)
                    # Populate exactly once the current outer arm targets. No
                    # robust hook is present on this temporary base controller.
                    c.solve(counter*.00125, 'transfer')
                    self.assertEqual(native_bytes(w), native_before)
                    c.__class__ = robust.RobustSupportController
                    corners, _, clearance = robust.actual_stance_patch(w, c.stance_side)
                    before_keys = set(vars(c))
                    before = pickle.dumps({key: value for key, value in vars(c).items() if key != 'w'}, protocol=5)
                    ro = C.CDLL(str(folder/'contact-inspector.so'))
                    ro.lh_ro_model_fingerprint.argtypes = [C.c_void_p]
                    ro.lh_ro_model_fingerprint.restype = C.c_uint64
                    fingerprint = ro.lh_ro_model_fingerprint(w.h)
                    held = w.arr(2, (21,)).copy()
                    risk, telemetry = c._forecast_unload_risk(counter*.00125, corners, clearance)
                    self.assertEqual(risk, 0)
                    self.assertEqual((telemetry['pressureRiskCorner'], telemetry['clearanceRiskCorner']), (0, 0))
                    self.assertEqual(native_bytes(w), native_before)
                    self.assertEqual(w.l.lh_dim(w.h, 6), counter)
                    self.assertEqual(ro.lh_ro_model_fingerprint(w.h), fingerprint)
                    np.testing.assert_array_equal(w.arr(2, (21,)), held)
                    self.assertEqual(set(vars(c))-before_keys, {'_robust_payloads'})
                    self.assertEqual(pickle.dumps({key: value for key, value in vars(c).items()
                        if key not in ('w', '_robust_payloads')}, protocol=5), before)
                    self.assertEqual(c._robust_payloads.maxlen, 22)
                    self.assertEqual(len(c._robust_payloads), 1)
                    witness = c._robust_payloads[0]
                    self.assertEqual(zlib.decompress(witness['compressed']), native_before)
                    self.assertEqual(witness['payloadSHA256'], hashlib.sha256(native_before).hexdigest())
                    self.assertEqual(witness['sourceStep'], counter)
                    self.assertEqual(witness['sourceTimeS'], now)
                    self.assertEqual(witness['payloadBytes'], len(native_before))
                    self.assertLessEqual(telemetry['proposedMotorMaximumFraction'], .95000001)
                    reports.append(dict(pilot=pilot, sourceCounter=counter, nativeTimeS=now,
                        nativeBytesModelCounterControlAndSolverInputsUnchanged=True,
                        onlyControllerMutation='exact native witness ring append',
                        physicsIntegrations=0, **telemetry))
                finally:
                    w.close()
        Path('/tmp/robust-support-predictor-purity.json').write_text(json.dumps(
            dict(physicsIntegrations=0, controlApplications=0, states=reports), indent=2)+'\n')


class DefaultHookParity(unittest.TestCase):
    def test_plain_posture_readiness_matches_pre_hook_function(self):
        if subprocess.run(['git', 'cat-file', '-e', '699d22c^{commit}'], cwd=ROOT,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
            self.skipTest('historical 699d22c parity source is not retained in this checkout')
        text = subprocess.check_output(['git', 'show', '699d22c:scripts/getup-step/source_multistep_probe.py'],
                                       cwd=ROOT, text=True)
        function = next(node for node in ast.parse(text).body
                        if isinstance(node, ast.FunctionDef) and node.name == 'posture_ready')
        namespace = {'np': np, 'math': math}
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<frozen-posture-ready>', 'exec'), namespace)
        previous = namespace['posture_ready']; rng = np.random.default_rng(3157)
        for _ in range(300):
            c = SimpleNamespace(com1=np.array([0., 0., .8]))
            row = dict(com=[*rng.normal(scale=.01, size=2), .8],
                comVelocity=[*rng.normal(scale=.03, size=2), 0.],
                jointLimitMinimumMarginRad=float(rng.uniform(0., .02)),
                jointLimitMarginsRad=rng.uniform(0., .02, size=21).tolist(),
                maximumBodyLinearMps=float(rng.uniform(0., .1)),
                maximumBodyAngularRadps=float(rng.uniform(0., .4)))
            self.assertEqual(bool(posture_ready(row, c)), bool(previous(row, c)))


if __name__ == '__main__':
    unittest.main()
