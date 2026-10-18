"""Pure COM polynomial/support tests. Never opens or steps a native world."""
import copy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import ConvexHull, QhullError

import source_continuous_com as continuous
import source_reviewed_walk as reviewed
from source_continuous_com import (
    MAX_TOTAL_ACCEL_MPS2, MAX_TOTAL_SPEED_MPS, QuinticSegment,
    actual_equations, plan_forward, validate_reference)
from source_reference_context import nominal_com
from source_step_probe import clock


ZERO = np.zeros(3)


class PureWorld:
    """Only the read-only geometry/time interface used by the COM functions."""
    def __init__(self):
        self.time = 100.
        self.h = None
        self.corners = np.array([[0., 0., .027], [.1, 0., .027],
                                 [0., .1, .027], [.1, .1, .027]])
        self.supports = self.corners[:, :2].copy()
        self.l = SimpleNamespace(lh_support_count=lambda h: len(self.supports),
                                 lh_time=lambda h: self.time)

    def arr(self, which, shape):
        if which != 21 or shape != (128,):
            raise AssertionError('COM pure test requested an unexpected native array')
        result = np.zeros(128)
        result[:2*len(self.supports)] = self.supports.ravel()
        return result

    def feet(self):
        foot = dict(points=self.corners.copy(), radius=.027)
        return [copy.deepcopy(foot), copy.deepcopy(foot)]

    def ground_height(self, point):
        return 0.


def controller(origin=(.05, .045, .8)):
    c = reviewed.ReviewedController.__new__(reviewed.ReviewedController)
    c.w = PureWorld()
    c.com0 = np.array(origin, dtype=float)
    c.com1 = np.array(origin, dtype=float)
    c.finalCom = np.array([.065, .055, .8])
    c.heading = np.array([1., 0., 0.])
    c.stance_side = 0
    c.robust_support = dict(selected=True)
    c._robust_survivors = [0, 1, 2]
    c._robust_delta = np.array([.0003, -.0002, 0.])
    c._robust_start, c._robust_duration = 2., 1.
    c._reviewed_mode = 'continuous-com'
    c.phase_durations = dict(transfer=2., lift=.8, translate=1.2, land=1., settle=1.52)
    return c


def broad_equations():
    return ConvexHull(np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])).equations


class HermiteJets(unittest.TestCase):
    def test_general_position_velocity_acceleration_boundaries(self):
        initial = (np.array([.1, -.2, .8]), np.array([.02, -.03, .004]),
                   np.array([.01, -.015, .002]))
        final = (np.array([.15, -.1, .81]), np.array([-.005, .01, -.002]),
                 np.array([-.02, .005, -.001]))
        segment = QuinticSegment.between(7., 1.7, initial, final)
        for now, expected in ((7., initial), (8.7, final)):
            for value, target in zip(segment.evaluate(now), expected):
                np.testing.assert_allclose(value, target, atol=2e-14, rtol=0.)

    def test_analytic_jet_matches_finite_difference_inside_segment(self):
        segment = QuinticSegment.between(1., 2.2,
            ([.1, -.2, .8], [.03, 0., -.002], [.01, .004, 0.]),
            ([.15, -.18, .79], ZERO, ZERO))
        h = 1e-5
        for t in (1.17, 1.7, 2.5, 3.09):
            before, centre, after = [segment.evaluate(x) for x in (t-h, t, t+h)]
            np.testing.assert_allclose((after[0]-before[0])/(2*h), centre[1], atol=2e-10)
            np.testing.assert_allclose((after[1]-before[1])/(2*h), centre[2], atol=2e-10)

    def test_zero_terminal_jets_remain_c2_during_extended_wait(self):
        segment = QuinticSegment.between(5., 2.2,
            ([.1, .2, .8], [.02, -.01, 0.], [.005, .003, 0.]),
            ([.14, .18, .8], ZERO, ZERO))
        endpoint = segment.evaluate(7.2)
        for now in (7.2, 8., 17.2, 1000.):
            for value, expected in zip(segment.evaluate(now), endpoint):
                np.testing.assert_array_equal(value, expected)
            np.testing.assert_allclose(segment.evaluate(now)[1:], 0., atol=2e-15)

    def test_initial_zero_jets_are_stable_before_start(self):
        segment = QuinticSegment.between(5., 1., (ZERO, ZERO, ZERO), ([.01, 0., 0.], ZERO, ZERO))
        for now in (-100., 4.99, 5.):
            for value in segment.evaluate(now):
                np.testing.assert_array_equal(value, ZERO)

    def test_receipt_round_trip_reconstructs_same_polynomial(self):
        original = QuinticSegment.between(1.2, 2.2, (ZERO, ZERO, ZERO), ([.01, -.0025, 0.], ZERO, ZERO))
        receipt = original.receipt()
        rebuilt = QuinticSegment(receipt['startSourceTimeS'], receipt['durationS'],
                                 tuple(tuple(row) for row in receipt['coefficients']))
        for now in (1.2, 1.9, 3.4, 99.):
            for x, y in zip(original.evaluate(now), rebuilt.evaluate(now)):
                np.testing.assert_array_equal(x, y)

    def test_nonpositive_duration_is_rejected(self):
        for duration in (0., -1., float('nan')):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                QuinticSegment.between(0., duration, (ZERO, ZERO, ZERO), (ZERO, ZERO, ZERO))


class WholePolynomialCapture(unittest.TestCase):
    def test_lateral_bound_uses_exact_capture_extremum_not_endpoint(self):
        c = controller((0., 0., .8)); c._robust_delta = ZERO.copy()
        # Only 2.5mm available in either lateral direction above the 3mm inset.
        equations = np.array([[1., 0., -1.], [-1., 0., -1.],
                              [0., 1., -.0055], [0., -1., -.0055]])
        with patch.object(continuous, 'actual_equations', return_value=equations):
            segment, receipt = plan_forward(c, 100.)
        ratio = math.sqrt(.8/9.81)/2.2
        u = ((1.-4.*ratio)+math.sqrt((1.-4.*ratio)**2+8.*ratio))/2.
        s = 10*u**3-15*u**4+6*u**5
        alpha = s+ratio*30*u*u*(1.-u)**2
        self.assertGreater(alpha, 1.04)
        self.assertAlmostEqual(receipt['lateralIntervalM'][1], .0025/alpha, places=14)
        self.assertAlmostEqual(receipt['lateralIntervalM'][0], -.0025/alpha, places=14)
        self.assertLess(abs(receipt['lateralM']), .0025)
        tau = math.sqrt(.8/9.81)
        max_capture = max(abs((p+tau*v)[1]) for p, v, _ in
                          (segment.evaluate(100.+x) for x in np.linspace(0., 2.2, 2001)))
        self.assertLessEqual(max_capture, .0025+1e-14)
        p, v, _ = segment.evaluate(100.+u*2.2)
        self.assertAlmostEqual(abs((p+tau*v)[1]), .0025, places=14)

    def test_planner_reads_only_current_support_and_is_deterministic(self):
        c = controller()
        before = copy.deepcopy(vars(c.w))
        first, a = plan_forward(c, 100.)
        second, b = plan_forward(c, 100.)
        self.assertEqual(first, second)
        self.assertEqual(a, b)
        np.testing.assert_array_equal(c.w.supports, before['supports'])
        np.testing.assert_array_equal(c.w.corners, before['corners'])
        self.assertEqual(c.w.time, 100.)
        self.assertEqual(a['plannedAtSourceTimeS'], 100.)

    def test_causal_lateral_selection_obeys_hull_and_selected_triangle(self):
        c = controller()
        segment, receipt = plan_forward(c, 100.)
        self.assertLess(receipt['lateralM'], 0.)
        self.assertLessEqual(abs(receipt['lateralM']), .012)
        self.assertAlmostEqual(segment.evaluate(102.2)[0][0], .010, places=14)
        origin = c.com1+c._robust_delta
        for t in np.linspace(0., 2.2, 501):
            p, v, a = segment.evaluate(100.+t)
            diagnostic = validate_reference(c, 'translate' if t<1.2 else 'land', (origin+p, v, a))
            self.assertGreaterEqual(diagnostic['referenceSupportMarginM'], .003-1e-12)

    def test_fixed_forward_request_with_no_lateral_solution_is_rejected(self):
        c = controller((.095, .05, .8)); c._robust_delta = ZERO.copy()
        c._robust_survivors = [1, 2, 3]
        with self.assertRaisesRegex(ValueError, 'no support-bounded'):
            plan_forward(c, 100.)


class CurrentSupportAndTotalEnvelope(unittest.TestCase):
    def test_missing_surviving_support_is_rejected(self):
        c = controller(); c.w.supports = c.w.supports[[0, 1, 3]]
        with self.assertRaisesRegex(ValueError, 'current selected actual triangle'):
            actual_equations(c, 'translate')

    def test_unselected_triangle_is_rejected(self):
        c = controller(); c.robust_support['selected'] = False
        with self.assertRaisesRegex(ValueError, 'current selected actual triangle'):
            actual_equations(c, 'land')

    def test_degenerate_or_absent_actual_hull_is_rejected(self):
        for supports in (np.empty((0, 2)), np.array([[0., 0.], [.1, 0.], [.2, 0.]])):
            c = controller(); c.w.supports = supports
            with self.subTest(supports=supports.tolist()), self.assertRaises((ValueError, QhullError)):
                actual_equations(c, 'settle')

    def test_settle_uses_actual_hull_without_installing_old_triangle(self):
        c = controller(); c.robust_support['selected'] = False
        eq = actual_equations(c, 'settle')
        self.assertEqual(len(eq), 4)
        expected = ConvexHull(c.w.supports).equations
        np.testing.assert_array_equal(eq, expected)

    def test_total_speed_sum_is_audited_even_if_each_component_is_bounded(self):
        c = controller()
        first = np.array([.11, 0., 0.]); second = first.copy()
        self.assertLess(np.linalg.norm(first), MAX_TOTAL_SPEED_MPS)
        self.assertLess(np.linalg.norm(second), MAX_TOTAL_SPEED_MPS)
        with patch.object(continuous, 'actual_equations', return_value=broad_equations()), \
             self.assertRaisesRegex(ValueError, 'frozen witness envelope'):
            validate_reference(c, 'settle', ([0., 0., .8], first+second, ZERO))

    def test_total_acceleration_sum_is_audited(self):
        c = controller(); component = np.array([.21, 0., 0.])
        self.assertLess(np.linalg.norm(component), MAX_TOTAL_ACCEL_MPS2)
        with patch.object(continuous, 'actual_equations', return_value=broad_equations()), \
             self.assertRaisesRegex(ValueError, 'frozen witness envelope'):
            validate_reference(c, 'settle', ([0., 0., .8], ZERO, component+component))

    def test_envelope_boundaries_pass_and_larger_total_jets_fail(self):
        c = controller()
        with patch.object(continuous, 'actual_equations', return_value=broad_equations()):
            validate_reference(c, 'settle', ([0., 0., .8], [MAX_TOTAL_SPEED_MPS, 0., 0.],
                                           [MAX_TOTAL_ACCEL_MPS2, 0., 0.]))
            for v, a in (([MAX_TOTAL_SPEED_MPS+1e-8, 0., 0.], ZERO),
                         (ZERO, [MAX_TOTAL_ACCEL_MPS2+1e-8, 0., 0.])):
                with self.assertRaisesRegex(ValueError, 'frozen witness envelope'):
                    validate_reference(c, 'settle', ([0., 0., .8], v, a))

    def test_capture_outside_inset_is_rejected_even_with_inside_position(self):
        c = controller()
        with self.assertRaisesRegex(ValueError, 'outside current support inset'):
            validate_reference(c, 'settle', ([.05, .05, .8], [.18, 0., 0.], ZERO))

    def test_nonfinite_jet_is_rejected(self):
        c = controller()
        for component in range(3):
            for value in (float('nan'), float('inf'), -float('inf')):
                jet = [np.array([.05, .05, .8]), ZERO.copy(), ZERO.copy()]
                jet[component][0] = value
                with self.subTest(component=component, value=value), \
                     self.assertRaises(ValueError):
                    validate_reference(c, 'settle', jet)

    def test_nonpositive_planned_height_is_rejected(self):
        c = controller()
        for height in (0., -.1):
            with self.subTest(height=height), self.assertRaises(ValueError):
                validate_reference(c, 'settle', ([.05, .05, height], ZERO, ZERO))


class ReviewedPhaseContinuity(unittest.TestCase):
    def setUp(self):
        self.c = controller()
        self.delta = np.array([.01, -.0025, 0.])
        self.path = QuinticSegment.between(100., 2.2, (ZERO, ZERO, ZERO), (self.delta, ZERO, ZERO))
        self.plan_patch = patch.object(reviewed, 'plan_forward', return_value=(self.path, {'pureFixture': True}))
        self.planner = self.plan_patch.start()
        self.eq_patch = patch.object(continuous, 'actual_equations', return_value=broad_equations())
        self.eq_patch.start()
        self.addCleanup(self.plan_patch.stop)
        self.addCleanup(self.eq_patch.stop)

    def evaluate(self, now, phase, elapsed):
        self.c.w.time = now
        return self.c.corrected_com_reference(elapsed, phase)

    def assertJetEqual(self, left, right, tolerance=2e-13):
        for a, b in zip(left, right):
            np.testing.assert_allclose(a, b, atol=tolerance, rtol=0.)

    def test_translate_entry_matches_previous_total_c2_jet(self):
        previous = self.evaluate(100., 'lift', .8)
        initial = self.evaluate(100., 'translate', 0.)
        self.assertJetEqual(previous, initial)
        self.assertEqual(self.planner.call_count, 1)

    def test_translate_land_boundary_preserves_all_jets_without_clock_reset(self):
        self.evaluate(100., 'translate', 0.)
        before = self.evaluate(101.2, 'translate', 1.2)
        after = self.evaluate(101.2, 'land', 0.)
        self.assertJetEqual(before, after)
        self.assertGreater(np.linalg.norm(after[1]), 0.)
        self.assertEqual(self.planner.call_count, 1)

    def test_repeated_evaluation_does_not_replan_or_project(self):
        expected = self.evaluate(100.4, 'translate', .4)
        receipt = self.c._com_path.receipt()
        for _ in range(3):
            self.assertJetEqual(self.evaluate(100.4, 'translate', .4), expected)
        self.assertEqual(self.planner.call_count, 1)
        self.assertEqual(self.c._com_path.receipt(), receipt)
        with patch.object(continuous, 'actual_equations', return_value=
                          np.array([[1., 0., -.001], [-1., 0., -.1],
                                    [0., 1., -.1], [0., -1., -.1]])), \
             self.assertRaisesRegex(ValueError, 'outside current support inset'):
            self.evaluate(100.5, 'translate', .5)
        self.assertEqual(self.planner.call_count, 1)
        self.assertEqual(self.c._com_path.receipt(), receipt)

    def test_corrected_branch_checks_sum_not_each_reference_component(self):
        self.c.w.time = 100.55
        _, path_velocity, path_acceleration = self.path.evaluate(100.55)
        self.assertGreater(np.linalg.norm(path_velocity), .001)
        self.assertGreater(np.linalg.norm(path_acceleration), .001)
        for order, path_component, limit in ((1, path_velocity, MAX_TOTAL_SPEED_MPS),
                                              (2, path_acceleration, MAX_TOTAL_ACCEL_MPS2)):
            nominal = [self.c.com1.copy(), ZERO.copy(), ZERO.copy()]
            nominal[order] = path_component/np.linalg.norm(path_component)*(limit-.001)
            self.assertLess(np.linalg.norm(nominal[order]), limit)
            self.assertLess(np.linalg.norm(path_component), limit)
            with self.subTest(order=order), \
                 patch.object(reviewed, 'nominal_com', return_value=tuple(nominal)), \
                 self.assertRaisesRegex(ValueError, 'frozen witness envelope'):
                self.c.corrected_com_reference(.55, 'translate')
    def test_extended_land_wait_holds_terminal_jet_without_drift(self):
        self.evaluate(100., 'translate', 0.)
        end = self.evaluate(102.2, 'land', 1.)
        for now in (102.5, 103.7, 110.):
            self.assertJetEqual(self.evaluate(now, 'land', now-101.2), end)
        np.testing.assert_allclose(end[1:], 0., atol=2e-14)

    def test_settle_starts_from_total_jet_and_preserves_measured_target(self):
        self.evaluate(100., 'translate', 0.)
        final_before = self.c.finalCom.copy()
        land = self.evaluate(102.2, 'land', 1.)
        settle = self.evaluate(102.2, 'settle', 0.)
        self.assertJetEqual(land, settle)
        self.assertJetEqual(self.evaluate(103.72, 'settle', 1.52), (final_before, ZERO, ZERO))
        np.testing.assert_array_equal(self.c.finalCom, final_before)

    def test_late_settle_starts_at_actual_boundary_and_preserves_wait(self):
        self.evaluate(100., 'translate', 0.)
        land = self.evaluate(103., 'land', 1.8)
        settle = self.evaluate(103., 'settle', 0.)
        self.assertJetEqual(land, settle)
        self.assertEqual(self.c._com_settle.start, 103.)
        self.assertJetEqual(self.evaluate(104.52, 'settle', 1.52),
                            (self.c.finalCom, ZERO, ZERO))

    def test_general_nonzero_boundary_jet_is_carried_into_settle(self):
        self.evaluate(100., 'translate', 0.)
        prior = self.evaluate(102., 'land', .8)
        self.assertGreater(np.linalg.norm(prior[1]), 0.)
        current = self.evaluate(102., 'settle', 0.)
        self.assertJetEqual(prior, current)

    def test_settle_components_sum_to_total_with_original_correction_fade(self):
        self.evaluate(100., 'translate', 0.)
        self.evaluate(102.2, 'settle', 0.)
        # Both components use the same original 1.52s clock. This catches
        # retaining the correction a second time after replanning total COM.
        nominal_start = self.c.com1+self.delta
        for t in (0., .15, .76, 1.2, 1.52):
            s, ds, dds = clock(t, 1.52)
            change = self.c.finalCom-nominal_start
            expected = (nominal_start+change*s+self.c._robust_delta*(1.-s),
                        change*ds-self.c._robust_delta*ds,
                        change*dds-self.c._robust_delta*dds)
            self.assertJetEqual(self.evaluate(102.2+t, 'settle', t), expected)

    def test_settle_stand_and_extended_stand_are_c2(self):
        self.evaluate(100., 'translate', 0.)
        self.evaluate(102.2, 'settle', 0.)
        end = self.evaluate(103.72, 'settle', 1.52)
        self.assertJetEqual(self.evaluate(103.72, 'stand', 0.), end)
        self.assertJetEqual(self.evaluate(200., 'stand', 96.28), end)

    def test_land_without_translate_history_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'phase history unavailable'):
            self.evaluate(101.2, 'land', 0.)

    def test_baseline_mode_remains_unchanged_and_does_not_plan(self):
        self.c._reviewed_mode = 'baseline-context'
        for phase, t in (('transfer', 2.5), ('translate', .4), ('land', .7),
                         ('settle', .76), ('stand', 0.)):
            nominal = nominal_com(self.c, t, phase)
            s, ds, dds = ((clock(t-self.c._robust_start, self.c._robust_duration))
                         if phase=='transfer' else
                         tuple(-x if i else 1.-x for i, x in enumerate(clock(t, 1.52)))
                         if phase=='settle' else (0., 0., 0.) if phase=='stand' else (1., 0., 0.))
            expected = tuple(x+y*self.c._robust_delta for x, y in zip(nominal, (s, ds, dds)))
            self.assertJetEqual(self.evaluate(100.+t, phase, t), expected)
        self.assertEqual(self.planner.call_count, 0)


if __name__ == '__main__':
    unittest.main()
