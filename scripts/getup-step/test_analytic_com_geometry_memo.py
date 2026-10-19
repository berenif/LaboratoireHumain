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
        self.assertEqual(memos[0].requests, memos[0].hits + memos[0].misses)
        self.assertEqual((memos[0].requests, memos[0].hits, memos[0].misses),
                         (memos[1].requests, memos[1].hits, memos[1].misses))
        self.assertEqual(memos[0].evictions, 0)

    @classmethod
    def setUpClass(cls):
        AnalyticCOMGeometryTests.setUpClass()


if __name__ == '__main__':
    unittest.main()
