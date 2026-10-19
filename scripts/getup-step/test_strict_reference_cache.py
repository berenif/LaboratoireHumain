"""Pure original-owner cache tests; no optimizer/scalar/native execution."""
from dataclasses import fields, is_dataclass
import importlib.util
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import numpy as np

from coupled_posture_reference import Jet
from coupled_posture_kinematics import ModelFK, MatrixJet
from wholebody_cycle_reference import WholeBodyCycle, StanceRealizedIK
from wholebody_math_fixture import make_math_fixture

_path = Path(__file__).resolve().with_name('strict_reference_cache.py')
_spec = importlib.util.spec_from_file_location('strict_reference_cache', _path)
cache_module = importlib.util.module_from_spec(_spec)
sys.modules['strict_reference_cache'] = cache_module
_spec.loader.exec_module(cache_module)


def exact_signature(value):
    """Every nested sample field, distinguishing every float64 bit/type."""
    if isinstance(value, (float, np.floating)):
        return ('float64', struct.pack('>d', float(value)).hex())
    if isinstance(value, np.ndarray):
        return ('array', value.shape, value.dtype.str, value.tobytes().hex())
    if is_dataclass(value):
        return (type(value).__name__, tuple((f.name, exact_signature(getattr(value, f.name))) for f in fields(value)))
    if isinstance(value, dict):
        return ('dict', tuple((key, exact_signature(item)) for key, item in value.items()))
    if isinstance(value, (tuple, list)):
        return (type(value).__name__, tuple(exact_signature(item) for item in value))
    return (type(value).__name__, value)


class StrictReferenceCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f = make_math_fixture()
        cls.knots = np.tile(np.asarray(cls.f.lower.position)[cls.f.realizer.independent_indices], (3, 1))

    def path(self, realizer=None, knots=None):
        return WholeBodyCycle(realizer or self.f.realizer, self.f.context, self.f.limits, self.f.heading,
                              self.knots if knots is None else knots, design_only=True,
                              entry_elapsed=self.f.context.elapsed)

    def test_entire_original_sample_all_fields_bitwise(self):
        path = self.path()
        wrapper = cache_module.StrictReferenceCache()
        for phase, t, options in [('lift', 0., {}), ('lift', .23, {}),
                ('lift', .23, dict(runtime_arms=self.f.context.arms)),
                ('lift', .23, dict(landing_adjustment=(.0001, .0002, -.0003))),
                ('lift_wait', 0., {})]:
            with self.subTest(phase=phase, t=t, options=options):
                a = path.sample(phase, t, **options)
                b = wrapper.sample(path, phase, t, **options)
                self.assertEqual(exact_signature(a), exact_signature(b))
                self.assertEqual(exact_signature(path.validate_sample(a, self.f.support)),
                                 exact_signature(path.validate_sample(b, self.f.support)))

    def test_inherited_original_math_and_immutable_model_fields(self):
        original = self.f.realizer.model
        local = cache_module._SampleBodyModel(original)
        self.assertIs(local.parts, original.parts)
        self.assertIs(local.limits, original.limits)
        self.assertIs(local.coupled.__func__, original.coupled.__func__)
        self.assertIs(local.thigh_jet.__func__, original.thigh_jet.__func__)
        with self.assertRaises(AttributeError):
            local.parts = ()
        joints = self.f.full
        a = original.forward(joints)
        b = local.forward(joints)
        self.assertEqual(exact_signature(a), exact_signature(b))
        c = local.forward(joints)
        self.assertIsNot(b, c)
        self.assertEqual(local.state.requests, 2 * len(original.parts[1:]))
        self.assertEqual(local.state.hits, len(original.parts[1:]))
        self.assertEqual(local.state.misses, len(original.parts[1:]))
        c.pop('head')
        self.assertIn('head', local.forward(joints))
        with self.assertRaises(AttributeError):
            b['head'].position = ()

    def test_signed_zero_and_root_bytes_are_not_normalized(self):
        local = cache_module._SampleBodyModel(self.f.realizer.model)
        q, v, a = self.f.full.arrays()
        local.forward(Jet(q, v, a))
        before = local.state.misses
        v[20] = -0.
        local.forward(Jet(q, v, a))
        self.assertGreater(local.state.misses, before)
        before = local.state.misses
        T, D, H = MatrixJet.rest(np.eye(4)).arrays()
        D[0, 0] = -0.
        local.forward(Jet(q, v, a), MatrixJet(T, D, H))
        self.assertGreater(local.state.misses, before)

    def test_bounded_cache_and_fresh_cache_after_success_and_error(self):
        local = cache_module._SampleBodyModel(self.f.realizer.model, capacity=2)
        local.forward(self.f.full)
        self.assertEqual(len(local.state.cache), 2)
        self.assertGreater(local.state.evictions, 0)
        path = self.path()
        wrapper = cache_module.StrictReferenceCache()
        model_class = cache_module._SampleBodyModel
        models = []
        def factory(original):
            model = model_class(original)
            models.append(model)
            return model
        before_model = path.realizer.model
        before_map = path.realizer.arm_map
        before_context = path.entry.receipt()
        with patch.object(cache_module, '_SampleBodyModel', side_effect=factory):
            first = wrapper.sample(path, 'lift', .23)
            with self.assertRaisesRegex(ValueError, 'causal measured'):
                wrapper.sample(path, 'settle', 0.)
            second = wrapper.sample(path, 'lift', .23)
        self.assertEqual(exact_signature(first), exact_signature(second))
        self.assertEqual(len(models), 3)
        self.assertEqual(models[1].state.requests, 0)
        self.assertIsNot(models[0].state.cache, models[2].state.cache)
        self.assertEqual(models[0].state.misses, models[2].state.misses)
        self.assertIs(path.realizer.model, before_model)
        self.assertIs(path.realizer.arm_map, before_map)
        self.assertEqual(path.entry.receipt(), before_context)
        self.assertFalse(vars(wrapper))

    def test_opposite_stance_and_original_strict_map_rejection(self):
        r = StanceRealizedIK(self.f.kin, 0, MatrixJet.rest(self.f.feet_model[0]), self.f.arm_map, self.f.leg_origin)
        knots = np.tile(np.asarray(self.f.lower.position)[r.independent_indices], (3, 1))
        path = self.path(r, knots)
        wrapper = cache_module.StrictReferenceCache()
        self.assertEqual(exact_signature(path.sample('lift', .2)), exact_signature(wrapper.sample(path, 'lift', .2)))
        class RejectingMap:
            def evaluate(self, delta):
                raise ValueError('original strict map rejection')
        original = path.realizer.arm_map
        path.realizer.arm_map = RejectingMap()
        try:
            for sampler in (path.sample, lambda phase, t: wrapper.sample(path, phase, t)):
                with self.assertRaisesRegex(ValueError, 'original strict map rejection'):
                    sampler('lift', .2)
        finally:
            path.realizer.arm_map = original


if __name__ == '__main__':
    unittest.main()
