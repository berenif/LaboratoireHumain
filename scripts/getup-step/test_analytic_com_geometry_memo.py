"""Pure exact-FK-reuse tests. No trajectory/scalar optimizer or native world."""
import unittest
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np

_helper_path = Path(__file__).resolve().with_name('analytic_com_geometry.py')
_spec = importlib.util.spec_from_file_location('analytic_com_geometry', _helper_path)
geometry = importlib.util.module_from_spec(_spec)
sys.modules['analytic_com_geometry'] = geometry
_spec.loader.exec_module(geometry)
assert Path(geometry.__file__).resolve() == _helper_path
from test_analytic_com_geometry import AnalyticCOMGeometryTests, assert_sample_bits


class FakeKinematics:
    def __init__(self):
        self.calls = 0
    def forward(self, q, root):
        self.calls += 1
        return dict(q=q.copy(), root=root.copy())


class SampleFKMemoTests(unittest.TestCase):
    def assert_pose_bits(self, expected, actual):
        self.assertEqual(tuple(expected), tuple(actual))
        for name in expected:
            self.assertEqual(expected[name].dtype, actual[name].dtype)
            self.assertEqual(expected[name].shape, actual[name].shape)
            self.assertEqual(expected[name].tobytes(), actual[name].tobytes())

    def test_body_misses_are_original_positional_fk_bitwise(self):
        f = AnalyticCOMGeometryTests.fixture
        memo = geometry._SampleFKMemo(f.kin)
        q = np.asarray(f.full.position).copy()
        root = np.eye(4)
        root[:3, 3] = [.13, -.24, .35]
        for index, value in ((None, None), (20, np.nextafter(0., 1.)),
                             (20, -0.), (0, .21), (6, -.81), (12, -.92)):
            if index is not None:
                q[index] = value
            self.assert_pose_bits(f.kin.forward(q, root), memo.forward(q, root))
        root[3, 2] = -0.
        self.assert_pose_bits(f.kin.forward(q, root), memo.forward(q, root))
        self.assertGreater(memo.body_memo.hits, 0)
        self.assertGreater(memo.body_memo.misses, 0)
        self.assertEqual(memo.body_memo.requests,
                         memo.body_memo.hits + memo.body_memo.misses)

    def test_body_byte_keys_reuse_unchanged_subtrees_not_changed_angles(self):
        f = AnalyticCOMGeometryTests.fixture
        memo = geometry._SamplePositionalBodyMemo(f.kin)
        q = np.asarray(f.full.position).copy()
        root = np.eye(4)
        first = memo.forward(q, root)
        misses = memo.misses
        # Even when trig rounds a changed subnormal angle to the same pose,
        # its own body must be recomputed from the requested exact angle bits.
        q[20] = np.nextafter(0., 1.)
        memo.forward(q, root)
        self.assertGreater(memo.misses, misses)
        misses = memo.misses
        q[20] = -0.
        memo.forward(q, root)
        self.assertGreater(memo.misses, misses)
        q[f.realizer.swing_indices] += .017
        changed = memo.forward(q, root)
        self.assert_pose_bits(f.kin.forward(q, root), changed)
        stance = ('left_foot', 'right_foot')[f.realizer.stance_side]
        self.assertIs(changed[stance], first[stance])
        misses = memo.misses
        root[3, 2] = -0.
        self.assert_pose_bits(f.kin.forward(q, root), memo.forward(q, root))
        self.assertGreater(memo.misses, misses)
        with self.assertRaises(ValueError):
            changed[stance][0, 0] = 8.

    def test_body_cache_is_bounded_and_owned_by_one_sample(self):
        f = AnalyticCOMGeometryTests.fixture
        q = np.asarray(f.full.position).copy()
        root = np.eye(4)
        memo = geometry._SampleFKMemo(f.kin, body_capacity=2)
        self.assert_pose_bits(f.kin.forward(q, root), memo.forward(q, root))
        self.assertEqual(len(memo.body_memo.cache), 2)
        self.assertGreater(memo.body_memo.evictions, 0)
        another = geometry._SampleFKMemo(f.kin)
        self.assertIsNot(memo.body_memo, another.body_memo)
        another.forward(q, root)
        self.assertEqual(another.body_memo.hits, 0)
        self.assertEqual(another.body_memo.misses, len(f.kin.parts) - 1)

    def test_full_bytes_include_ulp_signed_zero_and_root(self):
        kin = FakeKinematics()
        memo = geometry._SampleFKMemo(kin)
        q = np.zeros(21)
        root = np.eye(4)
        first = memo.forward(q, root)
        self.assertIs(first, memo.forward(q.copy(), root.copy()))
        q[20] = np.nextafter(0., 1.)
        memo.forward(q, root)
        q[20] = -0.
        memo.forward(q, root)
        root[3, 2] = -0.
        memo.forward(q, root)
        self.assertEqual((memo.requests, memo.hits, memo.misses, kin.calls), (5, 1, 4, 4))

    def test_cache_is_bounded_and_has_no_cross_sample_sharing(self):
        kin = FakeKinematics()
        memo = geometry._SampleFKMemo(kin, capacity=2)
        q = np.zeros(21)
        root = np.eye(4)
        for value in (0., 1., 2.):
            q[0] = value
            memo.forward(q, root)
        self.assertEqual((len(memo.cache), memo.evictions), (2, 1))
        q[0] = 0.
        memo.forward(q, root)
        self.assertEqual((memo.hits, memo.misses, memo.evictions), (0, 4, 2))
        another = geometry._SampleFKMemo(kin)
        another.forward(q, root)
        self.assertEqual((another.requests, another.hits, another.misses), (1, 0, 1))

    def test_original_method_bodies_and_objects_unchanged(self):
        f = AnalyticCOMGeometryTests.fixture
        realizer = f.realizer
        original_kin, original_fk = realizer.kin, realizer.fk
        original_fk_kin = original_fk.kin
        local = geometry._sample_realizer(realizer)
        self.assertIsNot(local, realizer)
        self.assertIsNot(local.fk, realizer.fk)
        self.assertIs(local.kin, local.fk.kin)
        self.assertIs(local.position.__func__, realizer.position.__func__)
        self.assertIs(local.fk.spatial_jacobian.__func__, realizer.fk.spatial_jacobian.__func__)
        self.assertIs(realizer.kin, original_kin)
        self.assertIs(realizer.fk, original_fk)
        self.assertIs(realizer.fk.kin, original_fk_kin)

    def test_real_newton_reuses_duplicates_and_starts_fresh_each_sample(self):
        f = AnalyticCOMGeometryTests.fixture
        test = AnalyticCOMGeometryTests()
        path = test.path()
        fast = geometry.AnalyticCOMGeometry(f.realizer)
        expected = path.sample('lift', .23)
        memo_class = geometry._SampleFKMemo
        memos = []
        def factory(kinematics):
            memo = memo_class(kinematics)
            memos.append(memo)
            return memo
        with patch.object(geometry, '_SampleFKMemo', side_effect=factory):
            first = fast.sample(path, 'lift', .23)
            second = fast.sample(path, 'lift', .23)
        assert_sample_bits(self, expected, first)
        assert_sample_bits(self, expected, second)
        self.assertEqual(len(memos), 2)
        self.assertIsNot(memos[0], memos[1])
        self.assertGreater(memos[0].hits, 0)
        self.assertGreater(memos[0].body_memo.hits, 0)
        self.assertIsNot(memos[0].body_memo, memos[1].body_memo)
        self.assertEqual(memos[0].requests, memos[0].hits + memos[0].misses)
        self.assertEqual((memos[0].requests, memos[0].hits, memos[0].misses),
                         (memos[1].requests, memos[1].hits, memos[1].misses))
        self.assertEqual(memos[0].evictions, 0)
        self.assertEqual(memos[0].body_memo.evictions, 0)
        self.assertEqual((memos[0].body_memo.requests, memos[0].body_memo.hits,
                          memos[0].body_memo.misses),
                         (memos[1].body_memo.requests, memos[1].body_memo.hits,
                          memos[1].body_memo.misses))

    @classmethod
    def setUpClass(cls):
        AnalyticCOMGeometryTests.setUpClass()


if __name__ == '__main__':
    unittest.main()
