"""Complete-field pure parity; synthetic math fixture, no native/optimizers."""
import copy
from dataclasses import fields, is_dataclass
import struct
import unittest
from unittest.mock import patch
import numpy as np
from coupled_posture_kinematics import MatrixJet, ModelFK
from wholebody_cycle_reference import WholeBodyCycle, StanceRealizedIK
from wholebody_math_fixture import make_math_fixture
from strict_reference_cache import StrictReferenceCache
from analytic_native_planning_sample import AnalyticNativePlanningSampler


def exact_signature(value):
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


class AnalyticNativePlanningSampleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f = make_math_fixture()
        cls.knots = np.tile(np.asarray(cls.f.lower.position)[cls.f.realizer.independent_indices], (3, 1))

    def path(self, realizer=None, knots=None):
        return WholeBodyCycle(realizer or self.f.realizer, self.f.context, self.f.limits, self.f.heading,
                              self.knots if knots is None else knots, design_only=True,
                              entry_elapsed=self.f.context.elapsed)

    def compare(self, path, phase, elapsed, **options):
        strict = StrictReferenceCache().sample(path, phase, elapsed, **options)
        fast = AnalyticNativePlanningSampler(path.realizer).sample(path, phase, elapsed, **options)
        self.assertEqual(exact_signature(strict), exact_signature(fast))
        self.assertEqual(exact_signature(path.validate_sample(strict, self.f.support)),
                         exact_signature(path.validate_sample(fast, self.f.support)))
        return fast

    def test_complete_root_allposes_commands_and_slacks_are_bitwise(self):
        path = self.path()
        for phase, elapsed in [('transfer', 0.), ('lift', 0.), ('lift', .23),
                               ('lift', .59), ('lift_wait', 0.)]:
            with self.subTest(phase=phase, elapsed=elapsed):
                self.compare(path, phase, elapsed)

    def test_runtime_arm_and_owned_seek_jets_are_bitwise(self):
        path = self.path()
        self.compare(path, 'lift', .23, runtime_arms=self.f.context.arms)
        self.compare(path, 'lift', .23, landing_adjustment=(.0001, .0002, -.0003))
        self.compare(path, 'lift', .23, runtime_arms=self.f.context.arms,
                     landing_adjustment=(.0001, .0002, -.0003))

    def test_original_newton_fixedseed_jacobian_without_validated_model_forward(self):
        path = self.path();fast = AnalyticNativePlanningSampler(path.realizer)
        expected = StrictReferenceCache().sample(path, 'lift', .23)
        with patch.object(path.realizer, 'position', wraps=path.realizer.position) as position, \
                patch.object(path.realizer.fk, 'spatial_jacobian', wraps=path.realizer.fk.spatial_jacobian) as jacobian, \
                patch.object(ModelFK, 'forward', side_effect=AssertionError('duplicated validated FK forbidden')):
            actual = fast.sample(path, 'lift', .23)
        self.assertEqual(exact_signature(expected), exact_signature(actual))
        self.assertEqual(position.call_count, 1)
        self.assertGreaterEqual(jacobian.call_count, 1)
        self.assertEqual(position.call_args.args[2], path.lower0.position)

    def test_callorder_mutation_and_readonly_static_cache(self):
        path = self.path();fast = AnalyticNativePlanningSampler(path.realizer)
        before = path.entry.receipt();a = fast.sample(path, 'lift', .19)
        fast.sample(path, 'lift', .31);b = fast.sample(path, 'lift', .19)
        self.assertEqual(exact_signature(a), exact_signature(b));self.assertEqual(before, path.entry.receipt())
        with self.assertRaises(ValueError):fast.geometry._parts[1][3][0][0,0] = 1.
        with self.assertRaises(AttributeError):a.geometric.root_model.position = ()

    def test_caller_owned_map_copy_and_original_domain_rejection(self):
        path = self.path();original = path.realizer.arm_map;calls = []
        class Recorder:
            def evaluate(self, delta):calls.append(delta);return original.evaluate(delta)
        copied = copy.copy(path);copied.realizer = copy.copy(path.realizer);copied.realizer.arm_map = Recorder()
        fast = AnalyticNativePlanningSampler(path.realizer)
        expected = StrictReferenceCache().sample(path, 'lift', .23)
        actual = fast.sample(copied, 'lift', .23)
        self.assertEqual(exact_signature(expected), exact_signature(actual));self.assertEqual(len(calls), 1)
        class Reject:
            def evaluate(self, delta):raise ValueError('original reserved map domain')
        copied.realizer.arm_map = Reject()
        for sample in (lambda: StrictReferenceCache().sample(copied, 'lift', .23),
                       lambda: fast.sample(copied, 'lift', .23)):
            with self.assertRaisesRegex(ValueError, 'original reserved map domain'):sample()

    def test_opposite_stance_complete_bitwise_and_wrongmodel_rejection(self):
        realizer = StanceRealizedIK(self.f.kin, 0, MatrixJet.rest(self.f.feet_model[0]), self.f.arm_map, self.f.leg_origin)
        knots = np.tile(np.asarray(self.f.lower.position)[realizer.independent_indices], (3, 1))
        self.compare(self.path(realizer, knots), 'lift', .23)
        with self.assertRaisesRegex(ValueError, 'original model identity'):
            AnalyticNativePlanningSampler(self.f.realizer).sample(self.path(realizer, knots), 'lift', .23)

if __name__ == '__main__':unittest.main(verbosity=2)
