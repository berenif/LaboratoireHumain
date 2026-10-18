"""Pure synthetic QP and receipt tests. No native world or integration."""
import copy
from dataclasses import FrozenInstanceError, asdict, replace
import json
import unittest

import clarabel
import numpy as np
from scipy import sparse

from source_soft_load_preference import (ACTIVATION_S, POLICY, SCHEMA, SETTLE_S,
    WEIGHT, AcquiredLoad, SoftLoadPreference, extend_qp, prepare_preference)


def anchor(**changes):
    values = dict(source_step=800, source_time=1., step_index=1, new_side=0,
                  measured_normal_n=30., body_weight_n=400., qualified_landing_samples=80)
    values.update(changes)
    return AcquiredLoad(**values)


def preference(floor=10.):
    return SoftLoadPreference(anchor(), (floor, 0., 0.))


def prepared(elapsed, acquired=None, **kwargs):
    acquired = acquired or anchor()
    return prepare_preference(acquired, 'settle', elapsed,
        acquired.source_step+round(elapsed/.00125), acquired.source_time+elapsed, **kwargs)


def solve(P, q, A, b, equalities=0):
    settings = clarabel.DefaultSettings(); settings.verbose = False
    settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-12
    cones = []
    if equalities:
        cones.append(clarabel.ZeroConeT(equalities))
    if len(b) > equalities:
        cones.append(clarabel.NonnegativeConeT(len(b)-equalities))
    result = clarabel.DefaultSolver(sparse.csc_matrix(P), q, sparse.csc_matrix(A),
                                    b, cones, settings).solve()
    if str(result.status) not in ('Solved', 'AlmostSolved'):
        raise AssertionError(result.status)
    return np.asarray(result.x)


class ImmutablePreference(unittest.TestCase):
    def test_strict_finite_json_receipt_roundtrip(self):
        item = prepared(.05)
        receipt = item.receipt()
        self.assertEqual(set(receipt), {'schema', 'policy', 'anchor', 'floor_jet', 'weight'})
        self.assertEqual(receipt['schema'], SCHEMA)
        self.assertEqual(receipt['policy'], POLICY)
        self.assertEqual(receipt['anchor'], asdict(item.anchor))
        self.assertEqual(AcquiredLoad.from_receipt(receipt['anchor']), item.anchor)
        parsed = json.loads(json.dumps(receipt, allow_nan=False))
        rebuilt = SoftLoadPreference.from_receipt(parsed)
        self.assertEqual(rebuilt, item)
        self.assertEqual(rebuilt.receipt(), parsed)
        self.assertEqual(json.dumps(rebuilt.receipt(), sort_keys=True, allow_nan=False),
                         json.dumps(receipt, sort_keys=True, allow_nan=False))

    def test_constructor_and_receipts_own_all_mutable_data(self):
        jet = np.array([10., 2., 3.])
        item = SoftLoadPreference(anchor(), jet)
        jet[:] = 99.
        self.assertEqual(item.floor_jet, (10., 2., 3.))
        receipt = item.receipt(); rebuilt = SoftLoadPreference.from_receipt(receipt)
        receipt['floor_jet'][0] = 88.; receipt['anchor']['measured_normal_n'] = 77.
        self.assertEqual(rebuilt, item)
        self.assertEqual(rebuilt.anchor.measured_normal_n, 30.)
        with self.assertRaises(FrozenInstanceError):
            item.weight = 1.
        with self.assertRaises(FrozenInstanceError):
            item.anchor.source_step = 1
        with self.assertRaises(TypeError):
            item.floor_jet[0] = 1.

    def test_receipt_shape_schema_policy_and_fields_are_strict(self):
        valid = preference().receipt()
        mutations = [lambda r: r.pop('weight'), lambda r: r.update(extra=1),
            lambda r: r.update(schema=True), lambda r: r.update(schema=2),
            lambda r: r.update(policy='retargeted'), lambda r: r.update(floor_jet=(1., 0., 0.)),
            lambda r: r['anchor'].update(extra=1), lambda r: r['anchor'].pop('step_index'),
            lambda r: r.update(anchor=None), lambda r: r.update(weight=.001),
            lambda r: r.update(floor_jet=[1., 0.]), lambda r: r.update(floor_jet=[-1., 0., 0.]),
            lambda r: r.update(floor_jet=[float('nan'), 0., 0.]),
            lambda r: r.update(floor_jet=[1., float('inf'), 0.]),
            lambda r: r.update(floor_jet=['1', 0., 0.]),
            lambda r: r['anchor'].update(source_step=800.),
            lambda r: r['anchor'].update(new_side=2),
            lambda r: r['anchor'].update(measured_normal_n=2.99),
            lambda r: r['anchor'].update(qualified_landing_samples=79)]
        for i, mutation in enumerate(mutations):
            with self.subTest(mutation=i):
                receipt = copy.deepcopy(valid); mutation(receipt)
                with self.assertRaises(ValueError):
                    SoftLoadPreference.from_receipt(receipt)
        for receipt in (None, [], 'receipt'):
            with self.assertRaises(ValueError):
                SoftLoadPreference.from_receipt(receipt)

    def test_invalid_anchor_scalar_types_values_and_nonfinite_are_rejected(self):
        mutations = dict(source_step=[-1, True, 1.5, '800'], source_time=[-1., np.nan, np.inf, '1'],
            step_index=[-1, False, 1.2], new_side=[-1, 2, True],
            measured_normal_n=[-1., np.nan, np.inf, True], body_weight_n=[0., -1., np.nan],
            qualified_landing_samples=[-1, 80., True])
        for field, values in mutations.items():
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    anchor(**{field: value})

    def test_fixed_weight_and_jet_validation(self):
        for jet in (None, (), (1., 0.), (1., 0., 0., 0.), (-1., 0., 0.),
                    (np.inf, 0., 0.), (1., np.nan, 0.), (True, 0., 0.), ('1', 0., 0.)):
            with self.subTest(jet=jet), self.assertRaises(ValueError):
                SoftLoadPreference(anchor(), jet)
        for weight in (0., .001, np.nan, np.inf, True, '.0005'):
            with self.subTest(weight=weight), self.assertRaises(ValueError):
                SoftLoadPreference(anchor(), (1., 0., 0.), weight)


class CausalActivation(unittest.TestCase):
    def test_no_preference_outside_settle_does_not_require_an_anchor(self):
        for phase in ('transfer', 'lift', 'translate', 'land', 'stand', 'quiet'):
            self.assertIsNone(prepare_preference(None, phase, np.nan, None, None))

    def test_qualified_landing_and_original_anchor_are_required(self):
        for item in (None, anchor(qualified_landing_samples=79), anchor(measured_normal_n=2.99)):
            with self.subTest(anchor=item), self.assertRaises(ValueError):
                prepare_preference(item, 'settle', 0., 800, 1.)
        self.assertIs(prepared(.1).anchor.__class__, AcquiredLoad)
        item = anchor(new_side=1, measured_normal_n=3.)
        self.assertIs(prepared(.1, item).anchor, item)

    def test_native_counter_time_and_elapsed_clocks_must_agree(self):
        invalid = [(.1, 880, 1.2), (.1, 879, 1.1), (.1, 881, 1.1),
                   (0., 799, 1.), (0., 800, .999), (-.00125, 799, .99875),
                   (.100001, 880, 1.100001), (np.nan, 880, 1.1),
                   (.1, 880., 1.1), (.1, True, 1.1), (.1, 880, np.inf)]
        for elapsed, step, time in invalid:
            with self.subTest(elapsed=elapsed, step=step, time=time), self.assertRaises(ValueError):
                prepare_preference(anchor(), 'settle', elapsed, step, time)

    def test_counter_clock_can_not_be_forged_by_changing_both_other_clocks(self):
        with self.assertRaisesRegex(ValueError, 'causal'):
            prepare_preference(anchor(), 'settle', .1, 800, 1.1)
        with self.assertRaisesRegex(ValueError, 'causal'):
            prepare_preference(anchor(source_time=.9), 'settle', .1, 880, 1.1)

    def test_duration_validation_and_custom_config_bound_duration(self):
        for duration in (0., -.1, .099, np.nan, np.inf, True, '1.52'):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                prepared(.05, settle_duration=duration)
        for duration in (.1, 1., 1.52, 3.04):
            self.assertGreater(prepared(.05, settle_duration=duration).floor_jet[0], 0.)
            self.assertEqual(prepared(duration, settle_duration=duration).floor_jet, (0., 0., 0.))

    def test_c2_exact_entry_endpoint_and_extended_wait(self):
        for elapsed in (0., SETTLE_S, SETTLE_S+2.):
            self.assertEqual(prepared(elapsed).floor_jet, (0., 0., 0.))
        # C2 limits at both ends, without relaxing the native clock equality.
        item = anchor(source_step=0, source_time=0.)
        eps = 1e-10
        near_start = prepare_preference(item, 'settle', eps, 0, eps).floor_jet
        near_end = prepare_preference(item, 'settle', SETTLE_S-eps, 1216, SETTLE_S-eps).floor_jet
        self.assertLess(max(abs(x) for x in near_start), 1e-3)
        self.assertLess(max(abs(x) for x in near_end), 1e-3)

    def test_c2_jet_matches_analytic_product_and_native_finite_differences(self):
        def quintic(t, duration):
            if t >= duration:
                return 1., 0., 0.
            u = t/duration
            return (10*u**3-15*u**4+6*u**5,
                    (30*u**2-60*u**3+30*u**4)/duration,
                    (60*u-180*u**2+120*u**3)/duration**2)
        for elapsed in (.025, .05, ACTIVATION_S, .5, 1., 1.5):
            a, da, dda = quintic(elapsed, ACTIVATION_S)
            b, db, ddb = quintic(elapsed, SETTLE_S)
            expected = 30.*np.array([a*(1-b), da*(1-b)-a*db,
                                     dda*(1-b)-2*da*db-a*ddb])
            with self.subTest(elapsed=elapsed):
                np.testing.assert_allclose(prepared(elapsed).floor_jet, expected, atol=1e-10, rtol=1e-12)
        h = .00125; center = prepared(.05).floor_jet
        before, after = (prepared(t).floor_jet for t in (.05-h, .05+h))
        self.assertAlmostEqual((after[0]-before[0])/(2*h), center[1], delta=.5)
        self.assertAlmostEqual((after[1]-before[1])/(2*h), center[2], delta=1.)

    def test_floor_is_acquired_load_capped_at_ten_percent_body_weight(self):
        low = prepared(.05, anchor(measured_normal_n=3.))
        high = prepared(.05, anchor(measured_normal_n=100.))
        capped = prepared(.05, anchor(measured_normal_n=40.))
        np.testing.assert_array_equal(high.floor_jet, capped.floor_jet)
        np.testing.assert_allclose(np.array(low.floor_jet)*40./3., high.floor_jet, atol=1e-11)
        self.assertLess(high.floor_jet[0], 40.)


class OneSlackBorder(unittest.TestCase):
    def setUp(self):
        self.P = np.diag([.1, .001**2, .1**2])
        self.q = np.arange(3.)
        self.A = np.arange(6.).reshape(2, 3)
        self.b = np.array([2., 3.])

    def test_none_preserves_exact_objects_dtypes_and_bytes_without_validation(self):
        for values in ((self.P, self.q, self.A, self.b), (object(), None, [], {'untouched': 1})):
            result = extend_qp(*values, normal_indices=None, preference=None)
            for got, old in zip(result, values):
                self.assertIs(got, old)
        original = tuple(x.tobytes() for x in (self.P, self.q, self.A, self.b))
        extend_qp(self.P, self.q, self.A, self.b, [0], None)
        self.assertEqual(original, tuple(x.tobytes() for x in (self.P, self.q, self.A, self.b)))

    def test_exact_original_blocks_one_new_column_two_rows_and_no_regularizer(self):
        before = tuple(x.copy() for x in (self.P, self.q, self.A, self.b))
        P, q, A, b = extend_qp(self.P, self.q, self.A, self.b, [0, 2], preference())
        self.assertEqual(P.shape, (4, 4)); self.assertEqual(A.shape, (4, 4))
        for old, expected in zip((self.P, self.q, self.A, self.b), before):
            np.testing.assert_array_equal(old, expected)
        for got, old in zip((P[:3, :3], q[:3], A[:-2, :3], b[:-2]), before):
            self.assertEqual(got.tobytes(), old.tobytes())
        self.assertEqual(P[3, 3], WEIGHT**2)
        np.testing.assert_array_equal(P[3, :3], 0.); np.testing.assert_array_equal(P[:3, 3], 0.)
        self.assertEqual(q[3], 0.)
        np.testing.assert_array_equal(A[:-2, 3], 0.)
        np.testing.assert_array_equal(A[-2], [0., 0., 0., -1.])
        np.testing.assert_array_equal(A[-1], [-1., 0., -1., -1.])
        np.testing.assert_array_equal(b[-2:], [0., -10.])
        for got, old in zip((P, q, A, b), (self.P, self.q, self.A, self.b)):
            self.assertFalse(np.shares_memory(got, old))

    def test_zero_normal_force_is_feasible_with_deficit_in_slack(self):
        x = solve(*extend_qp(np.array([[1e-6]]), np.zeros(1), np.ones((1, 1)),
            np.zeros(1), [0], preference()), equalities=1)
        self.assertAlmostEqual(x[0], 0., delta=1e-9)
        self.assertAlmostEqual(x[1], 10., delta=1e-5)

    def test_analytic_genuine_squared_hinge_optimum(self):
        # min .5*k*F²+.5*w²*max(0,10-F)² gives F=2N, zeta=8N.
        x = solve(*extend_qp(np.array([[1e-6]]), np.zeros(1), np.zeros((0, 1)),
            np.zeros(0), [0], preference()))
        self.assertAlmostEqual(x[0], 2., delta=1e-4)
        self.assertAlmostEqual(x[1], 8., delta=1e-4)

    def test_above_floor_has_no_excess_load_penalty(self):
        x = solve(*extend_qp(np.array([[1e-6]]), np.array([-20e-6]), np.zeros((0, 1)),
            np.zeros(0), [0], preference()))
        self.assertAlmostEqual(x[0], 20., delta=1e-5)
        self.assertLess(abs(x[1]), .002)

    def test_aggregate_uses_selected_columns_and_permits_uneven_corner_loads(self):
        # Independent native/base equalities fix two normals 0N/12N and an
        # unrelated column -100N; a 10N aggregate floor is already satisfied.
        x = solve(*extend_qp(np.eye(3)*1e-6, np.zeros(3), np.eye(3),
            np.array([0., -100., 12.]), [0, 2], preference()), equalities=3)
        np.testing.assert_allclose(x[:3], [0., -100., 12.], atol=1e-9)
        self.assertLess(abs(x[3]), .003)

    def test_indices_must_be_nonempty_unique_integer_base_columns(self):
        for indices in (None, [], [0, 0], [-1], [3], [4], [0.], [True], ['0']):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                extend_qp(self.P, self.q, self.A, self.b, indices, preference())
        extend_qp(self.P, self.q, self.A, self.b, np.array([0, 2], dtype=np.int64), preference())

    def test_matrix_dimensions_numerical_types_and_finiteness_are_strict(self):
        valid = [self.P, self.q, self.A, self.b]
        mutations = [(0, np.eye(2)), (1, np.zeros((3, 1))), (2, np.zeros((2, 4))),
            (3, np.zeros((2, 1))), (3, np.zeros(3)), (0, self.P+1j),
            (1, ['0', '1', '2']), (0, np.eye(3, dtype=bool))]
        for slot in range(4):
            for invalid in (np.nan, np.inf):
                item = valid[slot].copy(); item.flat[0] = invalid
                mutations.append((slot, item))
        for slot, value in mutations:
            with self.subTest(slot=slot, value=value), self.assertRaises(ValueError):
                args = valid.copy(); args[slot] = value
                extend_qp(*args, [0], preference())

    def test_unsafe_frozen_object_mutation_is_rechecked_at_consumption(self):
        for field, value in (('weight', .1), ('floor_jet', (np.nan, 0., 0.)),
                             ('floor_jet', (-1., 0., 0.)), ('anchor', None)):
            item = preference(); object.__setattr__(item, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError):
                extend_qp(self.P, self.q, self.A, self.b, [0], item)
        for field, value in (('measured_normal_n', np.nan), ('new_side', 2),
                             ('source_step', -1), ('qualified_landing_samples', 79)):
            item = preference(); object.__setattr__(item.anchor, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError):
                extend_qp(self.P, self.q, self.A, self.b, [0], item)


if __name__ == '__main__':
    unittest.main()
