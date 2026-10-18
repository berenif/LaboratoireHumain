"""Immutable reference and native QP-assembly tests; zero integrations."""
import copy
import ctypes as C
from dataclasses import FrozenInstanceError, replace
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

import source_step_probe as source
from source_arm_knee_mobility import native_bytes
from source_reference_context import (SCHEMA, REFERENCE_VERSION, Foot, Jet,
    Orientation, ReferenceContext, canonical, make_context, nominal_com,
    prospective_unload_context)
from source_selected_step import load_preset


PRESET = load_preset()
HELPER = source.ROOT/PRESET['helper']
SHARED = Path('/workspace/shared/cadence-robust-8-ten')
BASE = SHARED/'model-session' if (SHARED/'model-session').is_dir() else source.ROOT/PRESET['sourceSession']
HAS_NATIVE = HELPER.is_file() and (BASE/'slow-continuation/final.bin').is_file()


def native_model_fingerprint(w):
    path = SHARED/'contact-inspector.so'
    if not path.is_file():
        return None
    inspector = C.CDLL(str(path))
    inspector.lh_ro_model_fingerprint.argtypes = [C.c_void_p]
    inspector.lh_ro_model_fingerprint.restype = C.c_uint64
    return inspector.lh_ro_model_fingerprint(w.h)


def sample_context():
    joint = np.linspace(-.1, .1, 21)
    return ReferenceContext(SCHEMA, REFERENCE_VERSION, 80, .1, 1, 'transfer', .1, 0,
        Jet.make([0., 0., .8], [0., 0., 0.], [0., 0., 0.]),
        Jet.make(joint, np.zeros(21), np.zeros(21)),
        Jet.make(np.zeros(6), np.zeros(6), np.zeros(6)), (100., 20., 2.),
        (Orientation.make(1, np.eye(3), np.zeros(3), np.zeros(3)),),
        tuple(Foot(Jet.make([side*.1, 0., .027], np.zeros(3), np.zeros(3)),
                   Orientation.make(7+3*side, np.eye(3), np.zeros(3), np.zeros(3)))
              for side in (0, 1)), (), False,
        canonical(dict(previousTime=.095, previousRate=[.1, -.1])),
        canonical(dict(enabled=True)), canonical(dict(barriers=[])))


def context_controller(w, avoid=True, scale=1.):
    """Baseline references with no support observer or delayed-arm history."""
    c = source.Controller(w, 0, .062, avoid)
    c.trajectory_time_scale = scale
    c._reviewed_step_index, c._reference_epoch = 1, 0
    c.arm_velocity, c.arm_acceleration = np.zeros(6), np.zeros(6)
    c.arm_kp, c.arm_kd, c.arm_weight = 20., 10., 1.
    c.add_capture_constraints = lambda *args: None
    return c


def context_for(c, t, phase):
    return make_context(c, t, phase, nominal_com(c, t, phase),
        causal={}, config=dict(floorTargets=list(c.floorTargets)), seek=None)


def solve_and_capture(c, context, t=.37, phase='transfer'):
    captured = []
    factory = source.clarabel.DefaultSolver
    def solver(P, q, A, b, cones, settings):
        captured.append(dict(P=P.toarray().copy(), linear=np.asarray(q).copy(),
                             A=A.toarray().copy(), b=np.asarray(b).copy(),
                             cones=tuple((type(x).__name__, x.dim) for x in cones)))
        return factory(P, q, A, b, cones, settings)
    with patch.object(source.clarabel, 'DefaultSolver', side_effect=solver):
        result = source.Controller.solve(c, t, phase, reference_context=context)
    if len(captured) != 1:
        raise AssertionError('exactly one QP assembly was expected')
    return result, captured[0]


class ImmutableReference(unittest.TestCase):
    def test_roundtrip_is_exact_and_owns_nested_data(self):
        context = sample_context()
        receipt = json.loads(json.dumps(context.receipt()))
        rebuilt = ReferenceContext.from_receipt(receipt)
        self.assertEqual(context, rebuilt)
        self.assertEqual(context.receipt(), rebuilt.receipt())
        receipt['joints']['position'][6] = 12.
        receipt['causal']['previousRate'][0] = 12.
        self.assertNotEqual(rebuilt.joints.position[6], 12.)
        self.assertEqual(json.loads(rebuilt.causal_json)['previousRate'][0], .1)
        with self.assertRaises(FrozenInstanceError):
            context.epoch = 2
        with self.assertRaises(TypeError):
            context.joints.position[0] = 1.

    def test_input_arrays_and_returned_arrays_do_not_alias(self):
        value = np.array([.1, .2, .3])
        jet = Jet.make(value, value, value)
        value[:] = 10.
        for array in jet.arrays():
            np.testing.assert_array_equal(array, [.1, .2, .3])
            array[:] = 20.
        np.testing.assert_array_equal(jet.arrays()[0], [.1, .2, .3])
        context = sample_context()
        context.feet[0].orientation.arrays()[0][:] = 7.
        np.testing.assert_array_equal(context.feet[0].orientation.arrays()[0], np.eye(3))

    def test_receipt_corruption_is_rejected(self):
        for field in ('joints', 'orientations', 'elapsed', 'source_step'):
            with self.subTest(field=field):
                receipt = json.loads(json.dumps(sample_context().receipt()))
                if field == 'joints': receipt[field]['position'][6] += .01
                elif field == 'orientations': receipt[field][0]['rotation'][0] += .01
                else: receipt[field] += 1
                with self.assertRaisesRegex(ValueError, 'digest mismatch'):
                    ReferenceContext.from_receipt(receipt)

    def test_reference_identity_rejects_every_stale_component(self):
        context = sample_context()
        c = SimpleNamespace(w=SimpleNamespace(h=None, l=SimpleNamespace(
            lh_dim=lambda h, k: 80, lh_time=lambda h: .1)),
            _reviewed_step_index=1, _reference_epoch=0)
        context.validate(c, .1, 'transfer')
        for name, value in dict(source_step=84, source_time=.105, step_index=2,
                                phase='lift', elapsed=.2, epoch=1).items():
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'stale'):
                replace(context, **{name:value}).validate(c, .1, 'transfer')
        with self.assertRaisesRegex(ValueError, 'undeclared'):
            replace(context, hypothetical_changes=('other-task',)).validate(c, .1, 'transfer')

    def test_invalid_jet_values_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            Jet.make([np.nan], [0.], [0.])
        with self.assertRaisesRegex(ValueError, 'inconsistent'):
            Jet.make([0., 1.], [0.], [0.])


@unittest.skipUnless(HAS_NATIVE, 'pinned read-only native assets absent')
class NativeReferenceAssembly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w = source.World(BASE, HELPER)
        cls.w.restore(BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.w)
        cls.model_fingerprint = native_model_fingerprint(cls.w)
        cls.count = cls.w.l.lh_dim(cls.w.h, 6)
        cls.now = cls.w.l.lh_time(cls.w.h)
        cls.held = cls.w.arr(2, (21,)).copy()
        cls.blocked = []
        for name in ('lh_step', 'lh_step_audited', 'lh_set_state'):
            if hasattr(cls.w.l, name):
                cls.blocked.append((name, getattr(cls.w.l, name)))
                setattr(cls.w.l, name, lambda *args: (_ for _ in ()).throw(
                    AssertionError('native stepping/state installation forbidden')))

    @classmethod
    def tearDownClass(cls):
        try:
            assert native_bytes(cls.w) == cls.before
            assert cls.w.l.lh_dim(cls.w.h, 6) == cls.count
            assert cls.w.l.lh_time(cls.w.h) == cls.now
            assert native_model_fingerprint(cls.w) == cls.model_fingerprint
            np.testing.assert_array_equal(cls.w.arr(2, (21,)), cls.held)
        finally:
            for name, function in cls.blocked: setattr(cls.w.l, name, function)
            cls.w.close()

    def tearDown(self):
        self.assertEqual(native_bytes(self.w), self.before)
        self.assertEqual(self.w.l.lh_dim(self.w.h, 6), self.count)
        self.assertEqual(native_model_fingerprint(self.w), self.model_fingerprint)
        np.testing.assert_array_equal(self.w.arr(2, (21,)), self.held)

    def test_default_context_matches_legacy_qp_exactly_across_phases(self):
        for avoid in (False, True):
            for scale in (1., 2.):
                for phase in ('transfer', 'lift', 'translate', 'land', 'settle', 'stand'):
                    with self.subTest(avoid=avoid, scale=scale, phase=phase):
                        c = context_controller(self.w, avoid, scale)
                        (expected, expected_info), old = solve_and_capture(c, None, phase=phase)
                        context = context_for(c, .37, phase)
                        (actual, actual_info), new = solve_and_capture(c, context, phase=phase)
                        np.testing.assert_array_equal(actual, expected)
                        for name in ('P', 'linear', 'A', 'b'):
                            np.testing.assert_array_equal(new[name], old[name])
                        self.assertEqual(new['cones'], old['cones'])
                        for key, value in expected_info.items(): self.assertEqual(actual_info[key], value)

    def test_forecast_context_changes_only_swing_floor_and_unload(self):
        c = context_controller(self.w)
        context = context_for(c, .37, 'transfer')
        forecast = prospective_unload_context(c, context)
        for name in ('com', 'joints', 'arms', 'arm_gains', 'orientations',
                     'capture_constraints', 'causal_json', 'config_json'):
            self.assertEqual(getattr(context, name), getattr(forecast, name))
        self.assertEqual(context.feet[c.stance_side], forecast.feet[c.stance_side])
        self.assertEqual(context.feet[c.swing_side].orientation, forecast.feet[c.swing_side].orientation)
        self.assertTrue(forecast.unload_admitted)
        self.assertEqual(forecast.hypothetical_changes,
                         ('unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm'))
        (_, _), actual = solve_and_capture(c, context)
        (_, _), proposed = solve_and_capture(c, forecast)
        np.testing.assert_array_equal(actual['P'], proposed['P'])
        # The unload row follows dynamics and the eight contact friction cones,
        # before motor/joint/collision inequalities.
        unload_row = 27+8*5
        np.testing.assert_array_equal(actual['A'], np.delete(proposed['A'], unload_row, axis=0))
        np.testing.assert_array_equal(actual['b'], np.delete(proposed['b'], unload_row))
        self.assertEqual(proposed['b'][unload_row], 0.)
        nonzero = np.flatnonzero(proposed['A'][unload_row])
        np.testing.assert_array_equal(nonzero, 50+3*np.arange(4))

    def test_sentinel_joint_orientation_and_foot_jets_reach_both_qps(self):
        c = context_controller(self.w)
        context = context_for(c, .37, 'transfer')
        candidates = {}
        p, v, a = context.joints.arrays(); p[6] += .012; v[6] += .03; a[6] += .04
        candidates['knee'] = replace(context, joints=Jet.make(p, v, a))
        original = context.orientations[0]; R, v, a = original.arrays()
        changed = Orientation.make(1, Rotation.from_rotvec([.004, -.003, 0.]).as_matrix()@R,
                                   v+[.01, -.02, 0.], a+[.02, .01, 0.])
        candidates['orientation'] = replace(context, orientations=(changed,))
        feet = list(context.feet); foot = feet[c.swing_side]
        p, v, a = foot.translation.arrays(); p[0] += .001; v[0] += .002; a[0] += .003
        feet[c.swing_side] = replace(foot, translation=Jet.make(p, v, a))
        candidates['foot'] = replace(context, feet=tuple(feet))
        for kind, changed in candidates.items():
            with self.subTest(kind=kind):
                (_, _), original_actual = solve_and_capture(c, context)
                (_, _), changed_actual = solve_and_capture(c, changed)
                original_forecast = prospective_unload_context(c, context)
                changed_forecast = prospective_unload_context(c, changed)
                (_, _), original_proposed = solve_and_capture(c, original_forecast)
                (_, _), changed_proposed = solve_and_capture(c, changed_forecast)
                actual_delta = changed_actual['linear']-original_actual['linear']
                proposed_delta = changed_proposed['linear']-original_proposed['linear']
                self.assertGreater(float(np.max(np.abs(actual_delta))), 1e-6)
                np.testing.assert_allclose(proposed_delta, actual_delta, atol=1e-10, rtol=0.)
                for old, new in ((original_actual, changed_actual), (original_proposed, changed_proposed)):
                    np.testing.assert_array_equal(old['P'], new['P'])
                    np.testing.assert_array_equal(old['A'], new['A'])
                    np.testing.assert_array_equal(old['b'], new['b'])

    def test_context_consumption_never_calls_mutating_legacy_hooks(self):
        c = context_controller(self.w)
        context = context_for(c, .37, 'transfer')
        def forbidden(*args): raise AssertionError('legacy preparation called during pure solve')
        c.adjust_com_reference = c.landing_adjustment = c.add_capture_constraints = forbidden
        c.previous_time, c.previous_leg_rate = 9., np.array([.1, -.2])
        c._seek_time, c._seek_depth, c._seek_rate = 9., .001, .002
        before = copy.deepcopy({k:getattr(c, k) for k in (
            'previous_time', 'previous_leg_rate', '_seek_time', '_seek_depth', '_seek_rate')})
        first, info = source.Controller.solve(c, .37, 'transfer', reference_context=context)
        second, repeat = source.Controller.solve(c, .37, 'transfer', reference_context=context)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(info, repeat)
        for key, value in before.items(): np.testing.assert_array_equal(getattr(c, key), value)


if __name__ == '__main__': unittest.main()
