"""Saved-data corruption tests; no native imports, simulation or integration."""
import copy
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

import audit_source_bounded_settle as audit
import source_bounded_com_settle as producer

SUPPORT = [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
DURATION = 1.52


def fixture(distance=.18192656090102183, delta=None, noisy=False):
    initial = np.array([[.1, -.1, .82], [0., 0., 0.], [0., 0., 0.]])
    final = initial.copy()
    final[0] += [distance, 0., 0.] if delta is None else delta
    if noisy:
        initial[1, 0] = 1e-13
        final[2, 2] = -1e-13
    path, receipt = producer.build_bounded_settle(10., DURATION, initial, final, SUPPORT)
    return path, json.loads(json.dumps(receipt)), initial, final


def run(receipt, initial, final, duration=DURATION, points=SUPPORT):
    return audit.audit_bounded_settle_receipt(receipt, initial, final, duration, points)


class IndependentReconstruction(unittest.TestCase):
    def assert_passes(self, result):
        self.assertTrue(result['passed'], ([k for k, v in result['checks'].items() if not v], result['errors']))

    def test_legacy_quintic_and_seven_piece_profiles_reconstruct(self):
        for distance in (0., .1, .18192656090102183):
            with self.subTest(distance=distance):
                path, receipt, initial, final = fixture(distance)
                result = run(receipt, initial, final)
                self.assert_passes(result)
                times = [9., 10., 10.011, 10.76, 11.519, 11.52, 20.]
                times += [10.+s['offsetS'] for s in result['segments']]
                for t in times:
                    np.testing.assert_allclose(result['expected'](t), path.evaluate(t), atol=2e-13, rtol=0.)

    def test_varying_height_and_all_three_velocity_axes(self):
        for delta in ([.03, .18, .02], [.175, 0., -.025], [-.18, .03, -.01]):
            with self.subTest(delta=delta):
                _, receipt, initial, final = fixture(delta=delta)
                self.assert_passes(run(receipt, initial, final))

    def test_roundoff_endpoint_jets_do_not_erase_genuine_nonzero_jets(self):
        for distance in (.1, .18192656090102183):
            _, receipt, initial, final = fixture(distance, noisy=True)
            self.assert_passes(run(receipt, initial, final))
            initial[1, 0] = 1.01e-12
            self.assertFalse(run(receipt, initial, final)['passed'])

    def test_audit_does_not_call_any_producer_function(self):
        _, receipt, initial, final = fixture()
        with patch.object(producer, 'build_bounded_settle', side_effect=AssertionError('producer planner used')), \
             patch.object(producer, 'from_receipt', side_effect=AssertionError('producer parser used')), \
             patch.object(producer, 'certify_path', side_effect=AssertionError('producer certifier used')), \
             patch.object(producer.BoundedSettlePath, 'evaluate', side_effect=AssertionError('producer evaluator used')), \
             patch.object(producer.QuinticPath, 'evaluate', side_effect=AssertionError('producer evaluator used')):
            self.assert_passes(run(receipt, initial, final))

    def test_module_loads_when_producer_is_unavailable(self):
        path = Path(inspect.getfile(audit))
        spec = importlib.util.spec_from_file_location('independent_bounded_audit', path)
        independent = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'source_bounded_com_settle': None}):
            spec.loader.exec_module(independent)
        self.assertTrue(callable(independent.audit_bounded_settle_receipt))

    def test_bound_event_inputs_are_authoritative(self):
        _, receipt, initial, final = fixture()
        final[0, 0] += .002
        result = run(receipt, initial, final)
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['boundedSettleReceiptJetsBound'])
        self.assertFalse(result['checks']['boundedSettlePathReconstructed'])

    def test_near_transition_seven_piece_profile_is_reconstructed(self):
        # Close to the legacy/alternative transition, endpoint stationary roots
        # can split slightly under independent coefficient reconstruction.
        _, receipt, initial, final = fixture(.16)
        self.assertEqual(receipt['profile'], 'seven-cubic-constant-jerk')
        self.assert_passes(run(receipt, initial, final))


class CorruptedReceipts(unittest.TestCase):
    def setUp(self):
        _, self.receipt, self.initial, self.final = fixture()

    def mutated(self, mutate):
        receipt = copy.deepcopy(self.receipt)
        mutate(receipt)
        return run(receipt, self.initial, self.final)

    def test_schema_version_profile_clock_and_claim_corruptions_fail(self):
        changes = [('schema', 2), ('version', 'unknown'), ('profile', 'legacy-quintic-exact-identity'),
            ('durationUnchanged', False), ('measuredTargetUnchanged', False), ('frozenCapsUnchanged', False),
            ('physicsIntegrations', 1), ('optimizerCalls', 1), ('endpointDerivativesCanonicalized', False),
            ('zeroDerivativeCanonicalizationTolerance', 1e-6)]
        for key, value in changes:
            with self.subTest(key=key):
                self.assertFalse(self.mutated(lambda r: r.__setitem__(key, value))['passed'])
        self.assertFalse(self.mutated(lambda r: r['path'].__setitem__('durationS', 1.53))['passed'])
        self.assertFalse(run(self.receipt, self.initial, self.final, duration=1.53)['passed'])

    def test_every_saved_piece_clock_and_coefficient_is_bound(self):
        for i in range(7):
            for field in ('offsetS', 'durationS'):
                with self.subTest(piece=i, field=field):
                    self.assertFalse(self.mutated(lambda r: r['path']['pieces'][i].__setitem__(
                        field, r['path']['pieces'][i][field]+.001))['passed'])
            for coefficient in range(4):
                with self.subTest(piece=i, coefficient=coefficient):
                    self.assertFalse(self.mutated(lambda r: r['path']['pieces'][i]['coefficients'][coefficient].__setitem__(
                        1, r['path']['pieces'][i]['coefficients'][coefficient][1]+.001))['passed'])

    def test_path_endpoint_and_declared_jets_are_bound(self):
        for field in ('initialPositionM', 'finalPositionM'):
            self.assertFalse(self.mutated(lambda r: r['path'][field].__setitem__(0, r['path'][field][0]+.001))['passed'])
        for field in ('exactInitialJet', 'exactFinalJet'):
            for order in range(3):
                self.assertFalse(self.mutated(lambda r: r[field][order].__setitem__(0, r[field][order][0]+.001))['passed'])

    def test_legacy_coefficients_peaks_and_shape_are_reconstructed(self):
        self.assertFalse(self.mutated(lambda r: r['legacyQuintic']['coefficients'][3].__setitem__(0, 0.))['passed'])
        for field in ('legacyMaximumSpeedMps', 'legacyMaximumAccelerationMps2'):
            self.assertFalse(self.mutated(lambda r: r.__setitem__(field, .001))['passed'])
        for field, value in self.receipt['analyticalShape'].items():
            with self.subTest(field=field):
                self.assertFalse(self.mutated(lambda r: r['analyticalShape'].__setitem__(
                    field, [1.]*len(value) if isinstance(value, list) else value+.001))['passed'])

    def test_every_certificate_top_level_claim_is_checked(self):
        for field, value in self.receipt['certificate'].items():
            if field == 'pieces':
                continue
            bad = (not value if isinstance(value, bool) else value+'.tampered' if isinstance(value, str)
                   else [] if isinstance(value, list) else value+.001)
            with self.subTest(field=field):
                self.assertFalse(self.mutated(lambda r: r['certificate'].__setitem__(field, bad))['passed'])

    def test_frozen_policy_has_no_tolerance_for_changed_caps_or_reserve(self):
        for field in ('reserveM', 'maximumSpeedCapMps', 'maximumAccelerationCapMps2', 'numericalRoundoffTolerance'):
            with self.subTest(field=field):
                self.assertFalse(self.mutated(lambda r: r['certificate'].__setitem__(
                    field, r['certificate'][field]+1e-13))['passed'])

    def test_each_piece_and_face_extrema_claim_is_checked(self):
        mutations = [lambda p: p.__setitem__('maximumSpeedMps', 0.),
            lambda p: p.__setitem__('maximumAccelerationMps2', p['maximumAccelerationMps2']+.001),
            lambda p: p.__setitem__('speedExtremaTimesS', [.1]),
            lambda p: p.__setitem__('accelerationExtremaTimesS', [.1]),
            lambda p: p['heightExtremaM'].__setitem__('min', 0.),
            lambda p: p['heightExtremaM'].__setitem__('conservativeMax', 99.),
            lambda p: p.__setitem__('tauBoundsS', [0., 0.]),
            lambda p: p['faces'][0].__setitem__('positionMinimumMarginM', 99.),
            lambda p: p['faces'][0].__setitem__('captureConservativeMinimumMarginM', 99.),
            lambda p: p['faces'][0]['positionExtrema'].__setitem__('max', 99.),
            lambda p: p['faces'][0]['captureExtremaAtTauBounds'][0].__setitem__('conservativeMax', 99.)]
        for i in range(7):
            for mutation in mutations:
                with self.subTest(piece=i, mutation=mutation):
                    self.assertFalse(self.mutated(lambda r: mutation(r['certificate']['pieces'][i]))['passed'])

    def test_missing_extra_nonfinite_certificate_data_fails_closed(self):
        self.assertFalse(self.mutated(lambda r: r.pop('certificate'))['passed'])
        self.assertFalse(self.mutated(lambda r: r['certificate'].__setitem__('extraProof', True))['passed'])
        self.assertFalse(self.mutated(lambda r: r['certificate'].__setitem__('minimumPositionMarginM', float('nan')))['passed'])
        self.assertFalse(self.mutated(lambda r: r['path']['pieces'][0]['coefficients'][0].__setitem__(0, float('inf')))['passed'])

    def test_current_support_binding_and_whole_path_capture_recomputed(self):
        narrower = [[.0, -1.], [.29, -1.], [.29, 1.], [.0, 1.]]
        result = run(self.receipt, self.initial, self.final, points=narrower)
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['boundedSettleCurrentActualHull'])
        self.assertFalse(result['checks']['boundedSettleWholePathHullAndCapture'])
        self.assertGreater(.29-self.final[0, 0], .003)  # Endpoints alone would pass.

    def test_degenerate_and_nonfinite_bound_inputs_fail_closed(self):
        for support in ([], [[0., 0.], [1., 0.]], [[0., 0.], [1., 0.], [2., 0.]], [[0., 0.], [1., 0.], [0., float('nan')]]):
            with self.subTest(support=support):
                self.assertFalse(run(self.receipt, self.initial, self.final, points=support)['passed'])
        for receipt in (None, {}, {'path': {}}, {'path': {'startSourceTimeS': float('nan')}}):
            self.assertFalse(run(receipt, self.initial, self.final)['passed'])
        final = self.final.copy(); final[0, 0] += .2
        self.assertFalse(run(self.receipt, self.initial, final)['passed'])


if __name__ == '__main__':
    unittest.main()
