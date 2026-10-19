"""Portable pure math tests; zero real optimizers, native worlds, or physics.

Use the repository scripts/getup-step on PYTHONPATH, and point
REVIEWED_WALK_REPO_ROOT at its checkout when this file is outside the repo.
"""
import copy
import unittest
from unittest.mock import patch
import numpy as np

from coupled_posture_reference import Jet
from coupled_posture_kinematics import MatrixJet, hinge_jet
from wholebody_cycle_reference import WholeBodyCycle, StanceRealizedIK
from wholebody_math_fixture import make_math_fixture
from analytic_com_geometry import AnalyticCOMGeometry, _multiply, _inverse, _hinge


def assert_jet_bits(test, left, right):
    for a, b in zip(left.arrays(), right.arrays()):
        np.testing.assert_array_equal(a.view(np.uint64), b.view(np.uint64))


def assert_sample_bits(test, left, right):
    for a, b in ((left.lower_joints, right.lower_joints), (left.com, right.com),
                 (left.geometric.full_joints, right.geometric.full_joints),
                 (left.geometric.future_thigh_right_left, right.geometric.future_thigh_right_left),
                 (left.geometric.future_arm_delta_right_left, right.geometric.future_arm_delta_right_left)):
        assert_jet_bits(test, a, b)
    test.assertEqual(left.diagnostics, right.diagnostics)


class AnalyticCOMGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = make_math_fixture()
        cls.knots = np.tile(np.asarray(cls.fixture.lower.position)[
            cls.fixture.realizer.independent_indices], (3, 1))

    def path(self, realizer=None, knots=None):
        f = self.fixture
        return WholeBodyCycle(realizer or f.realizer, f.context, f.limits, f.heading,
                              self.knots if knots is None else knots,
                              design_only=True, entry_elapsed=f.context.elapsed)

    def test_plain_products_inverse_and_hinge_are_original_bitwise(self):
        left = hinge_jet(.37, -.21, .43)
        right = hinge_jet(-.14, .39, -.17)
        for x, y in zip((left @ right).arrays(), _multiply(left.arrays(), right.arrays())):
            np.testing.assert_array_equal(x.view(np.uint64), y.view(np.uint64))
        for x, y in zip(left.inverse().arrays(), _inverse(left.arrays())):
            np.testing.assert_array_equal(x.view(np.uint64), y.view(np.uint64))
        for x, y in zip(left.arrays(), _hinge(.37, -.21, .43)):
            np.testing.assert_array_equal(x.view(np.uint64), y.view(np.uint64))

    def test_complete_analytic_jets_arm_bridge_and_runtime_arms_bitwise(self):
        f = self.fixture
        path = self.path()
        fast = AnalyticCOMGeometry(path.realizer)
        cases = [('lift', 0., {}), ('lift', .27, {}), ('lift', .58, {}),
                 ('lift', .27, dict(runtime_arms=f.context.arms)),
                 ('lift', .27, dict(landing_adjustment=(.0002, .0003, -.0004))),
                 ('lift_wait', 0., {})]
        for phase, fraction, options in cases:
            with self.subTest(phase=phase, fraction=fraction, options=options):
                t = path.durations[0] * fraction
                assert_sample_bits(self, path.sample(phase, t, **options),
                                   fast.sample(path, phase, t, **options))

    def test_position_seed_and_original_jacobian_are_used_without_model_forward(self):
        path = self.path()
        fast = AnalyticCOMGeometry(path.realizer)
        expected = path.sample('lift', .19)
        with patch.object(path.realizer, 'position', wraps=path.realizer.position) as position, \
                patch.object(path.realizer.fk, 'spatial_jacobian', wraps=path.realizer.fk.spatial_jacobian) as jacobian, \
                patch.object(type(path.realizer.model), 'forward', side_effect=AssertionError('repeated validated ModelFK forbidden')):
            actual = fast.sample(path, 'lift', .19)
        assert_sample_bits(self, expected, actual)
        self.assertEqual(position.call_count, 1)
        self.assertGreaterEqual(jacobian.call_count, 1)
        self.assertEqual(position.call_args.args[2], path.lower0.position)

    def test_static_cache_readonly_and_call_order_independent(self):
        path = self.path()
        fast = AnalyticCOMGeometry(path.realizer)
        before = path.entry.receipt()
        expected = fast.sample(path, 'lift', .18)
        fast.sample(path, 'lift', .31)
        assert_sample_bits(self, expected, fast.sample(path, 'lift', .18))
        self.assertEqual(before, path.entry.receipt())
        with self.assertRaises(ValueError):
            fast._parts[1][3][0][0, 0] = 8.

    def test_copy_realizer_preserves_caller_owned_map_policy(self):
        path = self.path()
        original = path.realizer.arm_map
        calls = []
        class MapRecorder:
            def evaluate(self, delta):
                calls.append(delta)
                return original.evaluate(delta)
        cloned = copy.copy(path)
        cloned.realizer = copy.copy(path.realizer)
        cloned.realizer.arm_map = MapRecorder()
        fast = AnalyticCOMGeometry(path.realizer)
        actual = fast.sample(cloned, 'lift', .2)
        self.assertEqual(len(calls), 1)
        assert_sample_bits(self, path.sample('lift', .2), actual)
        self.assertIs(path.realizer.arm_map, original)

    def test_original_map_rejection_propagates_without_clipping(self):
        path = self.path()
        fast = AnalyticCOMGeometry(path.realizer)
        class RejectingMap:
            def evaluate(self, delta):
                raise ValueError('original reserved map rejection')
        cloned = copy.copy(path)
        cloned.realizer = copy.copy(path.realizer)
        cloned.realizer.arm_map = RejectingMap()
        for sampler in (cloned.sample, lambda phase, t: fast.sample(cloned, phase, t)):
            with self.assertRaisesRegex(ValueError, 'original reserved map rejection'):
                sampler('lift', .2)

    def test_opposite_stance_uses_its_own_ancestor_cache(self):
        f = self.fixture
        r = StanceRealizedIK(f.kin, 0, MatrixJet.rest(f.feet_model[0]), f.arm_map, f.leg_origin)
        knots = np.tile(np.asarray(f.lower.position)[r.independent_indices], (3, 1))
        path = self.path(realizer=r, knots=knots)
        fast = AnalyticCOMGeometry(r)
        assert_sample_bits(self, path.sample('lift', .23), fast.sample(path, 'lift', .23))
        with self.assertRaisesRegex(ValueError, 'original model identity'):
            AnalyticCOMGeometry(f.realizer).sample(path, 'lift', .23)


if __name__ == '__main__':
    unittest.main()
