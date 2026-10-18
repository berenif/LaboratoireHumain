"""Pure projected-COM geometry/reference tests. No native world or integrations."""
import copy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import source_continuous_com as continuous
import source_reviewed_walk as reviewed

ZERO = np.zeros(3)
EMPTY_A = np.empty((0, 2))
EMPTY_B = np.empty(0)


class ReadOnlyWorld:
    """Only current geometry/time reads; no installation or stepping interface."""
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
            raise AssertionError('unexpected native read in pure projected-COM test')
        out = np.zeros(128)
        out[:2*len(self.supports)] = self.supports.ravel()
        return out

    def feet(self):
        foot = dict(points=self.corners.copy(), radius=.027)
        return [copy.deepcopy(foot), copy.deepcopy(foot)]

    def ground_height(self, point):
        return 0.


def controller(origin=(0., 0., .8)):
    c = reviewed.ReviewedController.__new__(reviewed.ReviewedController)
    c.w = ReadOnlyWorld()
    c.com0 = np.array(origin, dtype=float)
    c.com1 = np.array(origin, dtype=float)
    c._robust_delta = ZERO.copy()
    c.heading = np.array([1., 0., 0.])
    c.stance_side = 0
    c.swing_side = 1
    c.targets = [[np.array([-.02, 0., .027]), np.eye(3)],
                 [np.array([-.08, 0., .027]), np.eye(3)]]
    c.landing = np.array([.06, 0., .0265])
    c.finalCom = np.array([.065, .005, .8])
    c.robust_support = dict(selected=True)
    c._robust_survivors = [0, 1, 2]
    c._robust_start, c._robust_duration = 2., 1.
    c._reviewed_mode = 'continuous-com-projected'
    c.phase_durations = dict(transfer=2., lift=.8, translate=1.2, land=1., settle=1.52)
    return c


def broad_equations():
    return np.array([[1., 0., -1.], [-1., 0., -1.],
                     [0., 1., -1.], [0., -1., -1.]])


def capture_factor(height=.8, duration=2.2):
    # Independent closed-form non-endpoint stationary point of s+tau/T*s'.
    r = math.sqrt(height/9.81)/duration
    u = ((1.-4.*r)+math.sqrt((1.-4.*r)**2+8.*r))/2.
    s = 10.*u**3-15.*u**4+6.*u**5
    f = s+r*30.*u*u*(1.-u)**2
    return u, f


class EndpointProjection(unittest.TestCase):
    def project(self, goal, A=EMPTY_A, b=EMPTY_B, radius=10.):
        result = np.asarray(continuous.project_endpoint(goal, A, b, radius))
        self.assertEqual(result.shape, (2,))
        self.assertTrue(np.isfinite(result).all())
        return result

    def test_feasible_desired_point_is_unchanged(self):
        np.testing.assert_array_equal(self.project([.2, -.3]), [.2, -.3])

    def test_single_line_is_orthogonal_projection(self):
        np.testing.assert_allclose(self.project([2., 3.], [[1., 0.]], [1.]), [1., 3.], atol=1e-12)

    def test_two_active_lines_give_polygon_vertex(self):
        np.testing.assert_allclose(self.project([2., 3.], [[1., 0.], [0., 1.]], [.2, .4]), [.2, .4], atol=1e-12)

    def test_oblique_line_preserves_tangential_component(self):
        np.testing.assert_allclose(self.project([2., 0.], [[1., 1.]], [1.]), [1.5, -.5], atol=1e-12)

    def test_disk_only_radial_projection(self):
        np.testing.assert_allclose(self.project([3., 4.], radius=1.), [.6, .8], atol=1e-12)

    def test_line_circle_intersection_is_needed(self):
        expected = [.2, math.sqrt(.96)]
        result = self.project([2., 2.], [[1., 0.]], [.2], 1.)
        np.testing.assert_allclose(result, expected, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(result), 1., places=12)

    def test_oppositely_oriented_line_circle_case(self):
        np.testing.assert_allclose(self.project([-2., 3.], [[-1., 0.]], [-.8], 1.), [.8, .6], atol=1e-12)

    def test_tangent_line_and_circle_single_feasible_point(self):
        np.testing.assert_allclose(self.project([0., 2.], [[-1., 0.]], [-1.], 1.), [1., 0.], atol=1e-12)

    def test_two_line_vertex_survives_disk_filter(self):
        A = [[-1., 0.], [0., -1.]]
        np.testing.assert_allclose(self.project([0., 0.], A, [-.6, -.8], 1.), [.6, .8], atol=1e-12)

    def test_redundant_zero_and_duplicate_faces_are_harmless(self):
        np.testing.assert_allclose(self.project([2., .3], [[0., 0.], [1., 0.], [2., 0.]], [0., 1., 2.]), [1., .3], atol=1e-12)

    def test_face_order_and_positive_normal_scaling_do_not_bias_solution(self):
        goal = np.array([2., 2.]); A = np.array([[1., 0.], [0., 1.], [-1., 0.]])
        b = np.array([.2, .9, 2.]); expected = self.project(goal, A, b, 1.)
        order = [2, 0, 1]; scale = np.array([3., .5, 7.])
        np.testing.assert_allclose(self.project(goal, A[order]*scale[:, None], b[order]*scale, 1.), expected, atol=1e-12)

    def test_mixed_extreme_normal_scales_keep_the_same_vertex(self):
        A = np.diag([-1e-9, -1e9])
        np.testing.assert_allclose(self.project([0., 0.], A, [-.5e-9, -.5e9], 2.),
                                   [.5, .5], atol=1e-12)

    def test_overflowing_finite_normalization_fails_closed(self):
        with np.errstate(over='ignore', invalid='ignore'), self.assertRaises(ValueError):
            continuous.project_endpoint([0., 0.], [[1e-300, 0.]], [1e300], 1.)

    def test_overflow_prone_finite_goals_fail_closed_or_project_correctly(self):
        for goal, expected in (([1e200, 0.], [1., 0.]),
                               ([1e308, 1e308], [1/math.sqrt(2), 1/math.sqrt(2)])):
            with self.subTest(goal=goal), np.errstate(over='ignore', invalid='ignore'):
                try:
                    result = continuous.project_endpoint(goal, EMPTY_A, EMPTY_B, 1.)
                except ValueError:
                    continue
                np.testing.assert_allclose(result, expected, atol=1e-12)

    def test_inputs_are_not_modified_and_result_is_deterministic(self):
        goal = np.array([2., 2.]); A = np.array([[1., 0.]]); b = np.array([.2])
        before = [a.copy() for a in (goal, A, b)]
        first = self.project(goal, A, b, 1.)
        np.testing.assert_array_equal(first, self.project(goal, A, b, 1.))
        for a, old in zip((goal, A, b), before):
            np.testing.assert_array_equal(a, old)

    def test_disjoint_halfplanes_are_rejected(self):
        with self.assertRaises(ValueError):
            self.project([0., 0.], [[1., 0.], [-1., 0.]], [0., -1.], 2.)

    def test_halfplane_outside_disk_is_rejected(self):
        with self.assertRaises(ValueError):
            self.project([0., 0.], [[-1., 0.]], [-1.01], 1.)

    def test_impossible_zero_normal_is_rejected(self):
        with self.assertRaises(ValueError):
            self.project([0., 0.], [[0., 0.]], [-.01], 1.)

    def test_malformed_shapes_are_rejected(self):
        cases = [([0.], EMPTY_A, EMPTY_B), ([0., 0., 0.], EMPTY_A, EMPTY_B),
                 ([[0., 0.]], EMPTY_A, EMPTY_B), ([0., 0.], [[1., 0., 0.]], [1.]),
                 ([0., 0.], [1., 0.], [1.]), ([0., 0.], [[1., 0.]], []),
                 ([0., 0.], [[1., 0.]], [[1.]])]
        for goal, A, b in cases:
            with self.subTest(goal=goal, A=A, b=b), self.assertRaises(ValueError):
                continuous.project_endpoint(goal, A, b, 1.)

    def test_nonfinite_values_and_negative_radius_are_rejected(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            for goal, A, b, radius in (([value, 0.], EMPTY_A, EMPTY_B, 1.),
                                      ([0., 0.], [[value, 0.]], [1.], 1.),
                                      ([0., 0.], [[1., 0.]], [value], 1.),
                                      ([0., 0.], EMPTY_A, EMPTY_B, value)):
                with self.subTest(value=value, radius=radius), self.assertRaises(ValueError):
                    continuous.project_endpoint(goal, A, b, radius)
        with self.assertRaises(ValueError):
            continuous.project_endpoint([0., 0.], EMPTY_A, EMPTY_B, -.1)


class ProjectedPlanner(unittest.TestCase):
    def plan(self, c, equations=None, start=100.):
        with patch.object(continuous, 'actual_equations', return_value=broad_equations() if equations is None else equations):
            segment, receipt = continuous.plan_projected(c, start)
        self.assertIsInstance(segment, continuous.QuinticSegment)
        self.assertIsInstance(receipt, dict)
        return segment, receipt

    def endpoint(self, segment):
        return segment.evaluate(segment.start+segment.duration)[0]

    def test_goal_is_planned_landing_plus_current_stance_midpoint_and35mm(self):
        c = controller(); segment, _ = self.plan(c)
        np.testing.assert_allclose(self.endpoint(segment), [.055, 0., 0.], atol=1e-12)
        c.targets[c.swing_side][0][:2] = [900., -900.]
        np.testing.assert_allclose(self.endpoint(self.plan(c)[0]), [.055, 0., 0.], atol=1e-12)

    def test_future_measured_final_com_cannot_change_the_planned_endpoint(self):
        c = controller(); c.finalCom = np.array([100., -100., 4.])
        first, receipt1 = self.plan(c)
        c.finalCom = np.array([-200., 300., .2])
        second, receipt2 = self.plan(c)
        np.testing.assert_array_equal(self.endpoint(first), self.endpoint(second))
        self.assertEqual(receipt1, receipt2)

    def test_landing_and_current_stance_targets_each_causally_affect_goal(self):
        for part in ('landing', 'stance'):
            c = controller()
            if part == 'landing': c.landing[0] += .02
            else: c.targets[c.stance_side][0][0] += .02
            with self.subTest(part=part):
                np.testing.assert_allclose(self.endpoint(self.plan(c)[0]), [.065, 0., 0.], atol=1e-12)

    def test_rotated_heading_preserves_source_coordinate_mapping(self):
        c = controller(); c.heading = np.array([0., 1., 0.])
        c.targets[0][0] = np.array([-.003, -.02, .027])
        c.landing = np.array([-.003, .06, .0265])
        np.testing.assert_allclose(self.endpoint(self.plan(c)[0]), [-.003, .055, 0.], atol=1e-12)

    def test_sub4mm_origin_face_cannot_lose_any_margin(self):
        c = controller(); c.landing[1] = .01
        equations = broad_equations(); equations[0, 2] = -.00375
        segment, _ = self.plan(c, equations)
        np.testing.assert_allclose(self.endpoint(segment), [0., .005, 0.], atol=1e-12)
        self.assertGreaterEqual(.00375-self.endpoint(segment)[0], .00375-1e-12)

    def test_exact3mm_origin_is_admitted_without_moving_the_origin(self):
        c = controller(); equations = broad_equations(); equations[0, 2] = -.003
        segment, _ = self.plan(c, equations)
        np.testing.assert_allclose(segment.evaluate(100.), [ZERO, ZERO, ZERO], atol=1e-14)
        self.assertLessEqual(self.endpoint(segment)[0], 1e-12)

    def test_origin_below_unchanged3mm_is_rejected(self):
        c = controller(); equations = broad_equations(); equations[0, 2] = -.00299
        with self.assertRaises(ValueError): self.plan(c, equations)

    def test_exact_whole_capture_extremum_keeps4mm_not_just_endpoint(self):
        c = controller(); equations = broad_equations(); equations[0, 2] = -.014
        segment, _ = self.plan(c, equations); u, factor = capture_factor()
        endpoint = self.endpoint(segment)
        self.assertAlmostEqual(endpoint[0], .010/factor, places=12)
        p, v, _ = segment.evaluate(100.+u*2.2)
        self.assertAlmostEqual(.014-(p+math.sqrt(.8/9.81)*v)[0], .004, places=12)
        self.assertGreater(.014-endpoint[0], .004)

    def test_lateral_design_cap_is12mm(self):
        c = controller(); c.landing[1] = .2
        self.assertAlmostEqual(self.endpoint(self.plan(c)[0])[1], .012, places=12)
        c.landing[1] = -.2
        self.assertAlmostEqual(self.endpoint(self.plan(c)[0])[1], -.012, places=12)

    def test_same_summed_jet_disk_bounds_endpoint_for_short_phase(self):
        c = controller(); c.phase_durations['translate'] = c.phase_durations['land'] = .05
        segment, _ = self.plan(c); T = .1
        radius = min(continuous.MAX_TOTAL_SPEED_MPS*T/1.875,
                     continuous.MAX_TOTAL_ACCEL_MPS2*T*T/(10/math.sqrt(3)))
        self.assertAlmostEqual(np.linalg.norm(self.endpoint(segment)), radius, places=12)
        _, v, _ = segment.evaluate(100.+T/2)
        _, _, a = segment.evaluate(100.+T*(3-math.sqrt(3))/6)
        self.assertLessEqual(np.linalg.norm(v), continuous.MAX_TOTAL_SPEED_MPS+1e-12)
        self.assertAlmostEqual(np.linalg.norm(a), continuous.MAX_TOTAL_ACCEL_MPS2, places=10)

    def test_original_robust_correction_is_used_in_origin_and_not_modified(self):
        c = controller(); c._robust_delta = np.array([.002, -.001, .003]); before = c._robust_delta.copy()
        c.com1 -= c._robust_delta
        np.testing.assert_allclose(self.endpoint(self.plan(c)[0]), [.055, 0., 0.], atol=1e-12)
        np.testing.assert_array_equal(c._robust_delta, before)

    def test_start_end_and_extended_wait_jets_are_c2(self):
        segment, _ = self.plan(controller())
        for now in (100.,): np.testing.assert_allclose(segment.evaluate(now), [ZERO, ZERO, ZERO], atol=1e-13)
        end = segment.evaluate(102.2)
        np.testing.assert_allclose(end[1:], 0., atol=1e-13)
        for now in (103., 200.):
            for a, b in zip(segment.evaluate(now), end): np.testing.assert_array_equal(a, b)

    def test_unavailable_actual_geometry_propagates_without_fallback(self):
        with patch.object(continuous, 'actual_equations', side_effect=ValueError('current actual support unavailable')), self.assertRaises(ValueError):
            continuous.plan_projected(controller(), 100.)

    def test_unselected_actual_triangle_rejects_planning(self):
        c = controller((.04, .04, .8)); c.robust_support['selected'] = False
        with self.assertRaises(ValueError): continuous.plan_projected(c, 100.)

    def test_plan_rejects_nonfinite_origin_heading_goal_and_equations(self):
        for field in ('origin', 'heading', 'landing', 'stance', 'equations'):
            c = controller(); equations = broad_equations()
            if field == 'origin': c.com1[0] = float('nan')
            elif field == 'heading': c.heading[0] = float('nan')
            elif field == 'landing': c.landing[0] = float('nan')
            elif field == 'stance': c.targets[0][0][0] = float('nan')
            else: equations[0, 0] = float('nan')
            with self.subTest(field=field), self.assertRaises(ValueError): self.plan(c, equations)


# Embedded actual-point-derived halfplanes and expected facewise-capped optima
# from com-endpoint-design.json. No shared artifacts/native assets are needed.
FROZEN_FIXTURES = [{'index': 1,
  'origin': [0.3173790977634888, 0.06675971215762727, 0.7925636051974451],
  'heading': [-0.43365641241267067, -0.9010783073481303, 0.0],
  'stance_side': 1,
  'landing': [0.43748313151143214, -0.19616379066069006, 0.0265],
  'stance_target': [0.3253183487011271, 0.10000097728588249, 0.02863416314941853],
  'equations': [[-0.4072747659504183, -0.9133056799451277, 0.0834910519522433],
                [0.9478049328825572, -0.3188507632161025, -0.2858480397955062],
                [-0.8705787388714383, 0.49202912456989384, 0.2102873515343075],
                [0.40727476595041734, 0.9133056799451281, -0.29349105120188324],
                [0.9478049328825572, -0.3188507632161025, -0.2858480397955062],
                [-0.9733812449693555, 0.22919195435683992, 0.28988123071356403],
                [0.40727476595041734, 0.9133056799451281, -0.29349105120188324]],
  'expected_local': [0.023368035224385283, 0.005159317835189758]},
 {'index': 2,
  'origin': [0.42018632971005576, -0.22332243736006374, 0.7846315249436375],
  'heading': [-0.43317314160847714, -0.901310728544292, 0.0],
  'stance_side': 0,
  'landing': [0.24918078008570965, -0.06547750709731116, 0.0265],
  'stance_target': [0.43784200405721035, -0.19631225188768406, 0.02691447555154742],
  'equations': [[-0.46159354147080967, -0.8870915412021669, -0.11243538534595274],
                [0.9268586351030597, -0.3754105359934553, -0.5063993657144605],
                [-0.839332537353274, 0.5436183327851583, 0.46798961785379456],
                [0.4615935414708081, 0.8870915412021677, -0.09756461442771668],
                [0.7850550021477459, -0.6194260598350724, -0.47232347544935704],
                [-0.839332537353274, 0.5436183327851583, 0.46798961785379456],
                [0.4615935414708081, 0.8870915412021677, -0.09756461442771668]],
  'expected_local': [-0.001420429912505355, -0.0018418261738641223]}]


class FrozenActualGeometries(unittest.TestCase):
    def test_both_frozen_endpoints_match_exact_facewise_projection(self):
        for fixture in FROZEN_FIXTURES:
            c = controller(fixture['origin']); c.heading = np.array(fixture['heading'])
            c.stance_side = fixture['stance_side']; c.swing_side = 1-c.stance_side
            c.landing = np.array(fixture['landing'])
            c.targets[c.stance_side][0] = np.array(fixture['stance_target'])
            c.finalCom = np.array([99., -99., 5.])  # Deliberately unrelated future measured target.
            eq = np.array(fixture['equations']); origin = np.array(fixture['origin'])
            with self.subTest(step=fixture['index']), patch.object(continuous, 'actual_equations', return_value=eq):
                segment, _ = continuous.plan_projected(c, 100.)
                delta = segment.evaluate(102.2)[0]; h = c.heading[:2]; left = np.array([-h[1], h[0]])
                np.testing.assert_allclose([delta[:2]@h, delta[:2]@left], fixture['expected_local'], atol=2e-11)
                reserve = np.minimum(.004, -(eq[:, :2]@origin[:2]+eq[:, 2]))
                u, _ = capture_factor(origin[2]); p, v, _ = segment.evaluate(100.+u*2.2)
                for point in (origin[:2], origin[:2]+delta[:2], origin[:2]+p[:2]+math.sqrt(origin[2]/9.81)*v[:2]):
                    self.assertTrue(np.all(-(eq[:, :2]@point+eq[:, 2]) >= reserve-2e-11))


class ProjectedModeContinuity(unittest.TestCase):
    def setUp(self):
        self.c = controller((.05, .045, .8))
        self.delta = np.array([.01, -.0025, 0.])
        self.path = continuous.QuinticSegment.between(100., 2.2,
            (ZERO, ZERO, ZERO), (self.delta, ZERO, ZERO))
        self.projected_patch = patch.object(reviewed, 'plan_projected', return_value=(self.path, {'pureFixture': True}), create=True)
        self.planner = self.projected_patch.start(); self.addCleanup(self.projected_patch.stop)
        self.equations_patch = patch.object(continuous, 'actual_equations', return_value=broad_equations())
        self.equations_patch.start(); self.addCleanup(self.equations_patch.stop)

    def evaluate(self, now, phase, elapsed):
        self.c.w.time = now
        return self.c.corrected_com_reference(elapsed, phase)

    def assertJetEqual(self, a, b):
        for x, y in zip(a, b): np.testing.assert_allclose(x, y, atol=2e-13, rtol=0.)

    def test_new_mode_calls_projected_planner_once_and_preserves_all_handoffs(self):
        self.assertJetEqual(self.evaluate(100., 'lift', .8), self.evaluate(100., 'translate', 0.))
        self.assertEqual(self.planner.call_count, 1)
        self.assertJetEqual(self.evaluate(101.2, 'translate', 1.2), self.evaluate(101.2, 'land', 0.))
        self.assertJetEqual(self.evaluate(102.2, 'land', 1.), self.evaluate(102.2, 'settle', 0.))
        self.assertJetEqual(self.evaluate(103.72, 'settle', 1.52), (self.c.finalCom, ZERO, ZERO))
        self.assertJetEqual(self.evaluate(103.72, 'stand', 0.), (self.c.finalCom, ZERO, ZERO))
        self.assertEqual(self.planner.call_count, 1)

    def test_no_per_evaluation_reprojection_or_support_fallback(self):
        self.evaluate(100., 'translate', 0.)
        receipt = self.c._com_path.receipt()
        first = self.evaluate(100.5, 'translate', .5)
        self.assertJetEqual(first, self.evaluate(100.5, 'translate', .5))
        with patch.object(continuous, 'actual_equations', return_value=np.array([[1., 0., -.001], [-1., 0., -.1], [0., 1., -.1], [0., -1., -.1]])), self.assertRaises(ValueError):
            self.evaluate(100.6, 'translate', .6)
        self.assertEqual(self.planner.call_count, 1)
        self.assertEqual(self.c._com_path.receipt(), receipt)

    def test_original_continuous_mode_keeps_old_forward_planner(self):
        self.c._reviewed_mode = 'continuous-com'
        with patch.object(reviewed, 'plan_forward', return_value=(self.path, {'old': True})) as original:
            self.evaluate(100., 'translate', 0.)
            self.assertEqual(original.call_count, 1)
        self.assertEqual(self.planner.call_count, 0)

    def test_baseline_context_never_invokes_projected_planner(self):
        self.c._reviewed_mode = 'baseline-context'
        self.evaluate(100., 'translate', 0.)
        self.assertEqual(self.planner.call_count, 0)
        self.assertFalse(hasattr(self.c, '_com_path'))


if __name__ == '__main__':
    unittest.main()
