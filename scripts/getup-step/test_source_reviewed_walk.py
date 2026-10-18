"""Reviewed-controller command ownership tests. No native integration allowed."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from source_arm_knee_mobility import native_bytes
from source_cadenced_ten_steps import CadencedController, CadencedSteps
from source_opposed_arm_swing import limb_telemetry
import source_reviewed_walk as reviewed
import source_step_probe as core
from source_reference_context import canonical
from source_soft_load_preference import AcquiredLoad
from source_reviewed_walk import ReviewedController, ReviewedSteps, control_sha
from source_reference_context import prospective_unload_context
from source_step_probe import World, Controller, clock
from source_upright_large_steps import ExtendedArmMap
from test_source_reference_context import BASE, HELPER, HAS_NATIVE, SHARED, native_model_fingerprint


class CausalArmPreparation(unittest.TestCase):
    def controller(self):
        state = SimpleNamespace(step=80, time=.1)
        c = ReviewedController.__new__(ReviewedController)
        c.w = SimpleNamespace(h=None, l=SimpleNamespace(lh_dim=lambda h,k:state.step,
                                                       lh_time=lambda h:state.time))
        c.install_reviewed(1, 'baseline-context')
        c.previous_time, c.previous_leg_rate = .095, np.array([.02, -.03])
        return c, state

    def test_once_per_native_command_preserves_nonzero_causal_acceleration(self):
        c, state = self.controller()
        rates = [np.array([.025, -.04]), np.array([.035, -.045])]
        values = [np.array([.1, -.2]), np.array([.11, -.21])]
        expected_legs, expected_rates = values[0].copy(), rates[0].copy()
        with patch.object(reviewed, 'limb_telemetry', side_effect=[
                (values[0], rates[0], np.zeros((2,3))),
                (values[1], rates[1], np.zeros((2,3)))]) as telemetry:
            legs, rate, acc = c.prepared_arm_measurement()
            np.testing.assert_allclose(acc, [1., -2.], atol=1e-12)
            anchor = copy.deepcopy(c._arm_causal)
            legs[:] = rate[:] = acc[:] = 999.
            repeated = c.prepared_arm_measurement()
            np.testing.assert_array_equal(repeated[0], expected_legs)
            np.testing.assert_array_equal(repeated[1], expected_rates)
            np.testing.assert_allclose(repeated[2], [1., -2.], atol=1e-12)
            self.assertEqual(c._arm_causal, anchor)
            self.assertEqual(telemetry.call_count, 1)
            # Reference changes do not constitute a second measured command.
            c._reference_epoch += 1
            np.testing.assert_array_equal(c.prepared_arm_measurement()[2], repeated[2])
            self.assertEqual(telemetry.call_count, 1)
            state.step, state.time = 84, .105
            _, _, next_acc = c.prepared_arm_measurement()
            np.testing.assert_allclose(next_acc, [2., -1.], atol=1e-12)
            self.assertEqual(telemetry.call_count, 2)
            self.assertEqual(c._arm_causal['anchor']['previousTimeS'], .1)
            np.testing.assert_array_equal(c._arm_causal['anchor']['previousLegRateRadps'], expected_rates)

    def test_first_actual_command_has_zero_acceleration_and_no_synthetic_anchor(self):
        c, _ = self.controller()
        c.previous_time = c.previous_leg_rate = None
        with patch.object(reviewed, 'limb_telemetry', return_value=(
                np.array([.1, -.2]), np.array([.2, -.1]), np.zeros((2,3)))):
            _, _, acc = c.prepared_arm_measurement()
        np.testing.assert_array_equal(acc, np.zeros(2))
        self.assertEqual(c._arm_causal['anchor'],
                         dict(previousTimeS=None, previousLegRateRadps=None))

    def test_noncausal_native_time_is_rejected_without_history_commit(self):
        c, state = self.controller()
        state.time = .09
        with patch.object(reviewed, 'limb_telemetry', return_value=(
                np.zeros(2), np.zeros(2), np.zeros((2,3)))), \
             self.assertRaisesRegex(ValueError, 'not causal'):
            c.prepared_arm_measurement()
        self.assertEqual(c.previous_time, .095)
        np.testing.assert_array_equal(c.previous_leg_rate, [.02, -.03])

    def test_descriptive_setup_history_is_discarded_at_first_and_later_steps(self):
        old = ReviewedController.__new__(ReviewedController)
        old.previous_time, old.previous_leg_rate = 13.4, np.array([.2, -.3])
        def descriptive_setup(live):
            c = CadencedController.__new__(CadencedController)
            # Represents the intermediate setup solve performed by ancestors.
            c.previous_time, c.previous_leg_rate = 99., np.array([9., 9.])
            live.controller = c
        for step in (0, 1):
            with self.subTest(step=step), tempfile.TemporaryDirectory(prefix='reviewed-history-') as directory:
                live = ReviewedSteps.__new__(ReviewedSteps)
                live.controller, live.step_index = old, step
                live.reviewed_mode, live.folder = 'baseline-context', Path(directory)
                with patch.object(CadencedSteps, 'begin_step', descriptive_setup):
                    live.begin_step()
                saved = json.loads((live.folder/f'controller-step-{step+1:02d}.json').read_text())
                if step == 0:
                    self.assertIsNone(live.controller.previous_time)
                    self.assertIsNone(live.controller.previous_leg_rate)
                    self.assertIsNone(saved['previous_time'])
                else:
                    self.assertEqual(live.controller.previous_time, old.previous_time)
                    np.testing.assert_array_equal(live.controller.previous_leg_rate, old.previous_leg_rate)
                    self.assertIsNot(live.controller.previous_leg_rate, old.previous_leg_rate)
                self.assertIsNone(live.controller._measurement_cache)
                self.assertEqual(len(live.controller._robust_history), 0)


JOURNAL = SHARED/'run/generation-001'
HAS_REVIEWED_NATIVE = HAS_NATIVE and (JOURNAL/'controller-step-01.json').is_file()


@unittest.skipUnless(HAS_REVIEWED_NATIVE, 'pinned reviewed reference/native assets absent')
class NativeReviewedCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w = World(BASE, HELPER)
        cls.w.restore(BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.w)
        cls.model_fingerprint = native_model_fingerprint(cls.w)
        cls.counter = cls.w.l.lh_dim(cls.w.h, 6)
        cls.now = cls.w.l.lh_time(cls.w.h)
        cls.held = cls.w.arr(2, (21,)).copy()
        cls.record = json.loads((JOURNAL/'controller-step-01.json').read_text())
        arm = json.loads((JOURNAL/'arm-mapping.json').read_text())
        cls.mapping = ExtendedArmMap(cls.w.arr(0, (28,))[7:], cls.w.limits, arm['goalRad'])
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
            assert native_model_fingerprint(cls.w) == cls.model_fingerprint
            np.testing.assert_array_equal(cls.w.arr(2, (21,)), cls.held)
        finally:
            for name, function in cls.blocked: setattr(cls.w.l, name, function)
            cls.w.close()

    def tearDown(self):
        self.assertEqual(native_bytes(self.w), self.before)
        self.assertEqual(self.w.l.lh_dim(self.w.h, 6), self.counter)
        self.assertEqual(self.w.l.lh_time(self.w.h), self.now)
        self.assertEqual(native_model_fingerprint(self.w), self.model_fingerprint)
        np.testing.assert_array_equal(self.w.arr(2, (21,)), self.held)

    def controller(self, cls=ReviewedController):
        c = cls.__new__(cls)
        for key, value in copy.deepcopy(self.record).items():
            if key == 'targets': value = [(np.array(p), np.array(R)) for p,R in value]
            elif isinstance(value, list): value = np.asarray(value, dtype=float)
            setattr(c, key, value)
        c.w, c._mapping = self.w, self.mapping
        c.previous_time = c.previous_leg_rate = None
        if cls is ReviewedController: c.install_reviewed(1, 'baseline-context')
        return c

    def test_baseline_context_command_matches_delayed_arm_legacy(self):
        current, legacy = self.controller(), self.controller(CadencedController)
        with patch.object(reviewed, 'make_context', wraps=reviewed.make_context) as builds:
            actual, info = current.solve(.37, 'transfer')
        self.assertEqual(builds.call_count, 1)
        expected, old = legacy.solve(.37, 'transfer')
        np.testing.assert_array_equal(actual, expected)
        for name in ('comTarget', 'predictedForcesN', 'dynamicsResidual', 'contactSeek',
                     'opposedArmSwing', 'captureBarrier', 'opposedArmRangeBarrier'):
            self.assertEqual(info[name], old[name])
        self.assertFalse(info['robustSupportAdmission']['selected'])
        self.assertEqual(info['actualMotorControlSHA256'], control_sha(actual))
        self.assertEqual(info['reviewedWalk']['mode'], 'baseline-context')

    def test_causal_mode_before_loaded_selection_preserves_baseline_qp(self):
        current, baseline = self.controller(), self.controller()
        current.install_reviewed(1, 'continuous-com-causal')
        baseline.install_reviewed(1, 'continuous-com')
        actual, info = current.solve(.37, 'transfer')
        expected, old = baseline.solve(.37, 'transfer')
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(info['qpAssemblySHA256'], old['qpAssemblySHA256'])
        self.assertFalse(info['robustSupportAdmission']['selected'])
        self.assertIsNone(info['referenceContext']['causal']['causalOverlap'])
        self.assertTrue(info['referenceContext']['config']['causalSupportSelectionPolicy']
                        ['originalMinimumTransferAndMeasuredLiftGatesUnchanged'])

    def test_repeated_command_is_cached_and_returned_values_are_detached(self):
        c = self.controller()
        with patch.object(c, 'prepare_reference_context', wraps=c.prepare_reference_context) as preparation, \
             patch.object(c, '_observe_support', wraps=c._observe_support) as observation, \
             patch.object(reviewed, 'limb_telemetry', wraps=limb_telemetry) as measurement:
            actual, info = c.solve(.37, 'transfer')
            pristine_control, pristine_info = actual.copy(), copy.deepcopy(info)
            history = (c.previous_time, c.previous_leg_rate.copy(), c._seek_time,
                       c._seek_depth, c._seek_rate, c._seek_jet, len(c._robust_history))
            actual[:] = 99.
            info['contactSeek']['depthM'] = 99.
            changed_joint = list(info['referenceContext']['joints']['position'])
            changed_joint[6] = 99.
            info['referenceContext']['joints']['position'] = tuple(changed_joint)
            info['reviewedWalk']['forecasts'].append({'forged':True})
            repeated, repeated_info = c.solve(.37, 'transfer')
            np.testing.assert_array_equal(repeated, pristine_control)
            self.assertEqual(repeated_info, pristine_info)
            self.assertEqual(preparation.call_count, 1)
            self.assertEqual(observation.call_count, 1)
            self.assertEqual(measurement.call_count, 1)
            self.assertEqual(c.previous_time, history[0])
            np.testing.assert_array_equal(c.previous_leg_rate, history[1])
            self.assertEqual((c._seek_time, c._seek_depth, c._seek_rate,
                              c._seek_jet, len(c._robust_history)), history[2:])

    def test_same_native_reference_edits_invalidate_solve_without_derivative_commit(self):
        c = self.controller()
        c.solve(.37, 'transfer')
        anchor = copy.deepcopy(c._arm_causal)
        with patch.object(c, 'prepare_reference_context', wraps=c.prepare_reference_context) as preparation, \
             patch.object(c, '_observe_support', wraps=c._observe_support) as observation, \
             patch.object(reviewed, 'limb_telemetry', wraps=limb_telemetry) as measurement:
            original = c._prepared_reference_context
            c._reference_epoch += 1
            c.solve(.37, 'transfer')
            self.assertNotEqual(c._prepared_reference_context.epoch, original.epoch)
            c.finalCom = c.finalCom.copy(); c.finalCom[0] += .001
            c.solve(.37, 'transfer')
            c.solve(.38, 'transfer')
            self.assertEqual(c._prepared_reference_context.elapsed, .38)
            c.solve(.38, 'settle')
            self.assertEqual(c._prepared_reference_context.phase, 'settle')
            self.assertEqual(preparation.call_count, 4)
            # One new phase is observed; same-native reference edits are pure.
            self.assertEqual(observation.call_count, 1)
            self.assertEqual(measurement.call_count, 0)
            self.assertEqual(c._arm_causal, anchor)

    def test_forecast_reuses_prepared_context_without_arm_seek_or_support_updates(self):
        c = self.controller(); c.solve(.37, 'transfer')
        context_before = c._prepared_reference_context
        causal = copy.deepcopy(c._arm_causal)
        history = copy.deepcopy(list(c._robust_history))
        arm_before = (c.previous_time, c.previous_leg_rate.copy())
        seek_before = (c._seek_time, c._seek_depth, c._seek_rate, c._seek_jet)
        corners, _, clearance = reviewed.actual_stance_patch(self.w, c.stance_side)
        def forbidden(*args, **kwargs): raise AssertionError('forecast invoked mutable preparation')
        with patch.object(c, 'prepared_arm_measurement', side_effect=forbidden), \
             patch.object(c, 'landing_adjustment', side_effect=forbidden), \
             patch.object(c, '_observe_support', side_effect=forbidden), \
             patch.object(c, 'prepare_reference_context', side_effect=forbidden):
            risk, forecast = c._forecast_unload_risk(.37, corners, clearance)
            _, parity = c._forecast_unload_risk(.37, corners, clearance, role='final-context-parity')
        self.assertIs(type(c), ReviewedController)
        self.assertEqual(c._prepared_reference_context, context_before)
        self.assertEqual(c._arm_causal, causal)
        self.assertEqual(list(c._robust_history), history)
        self.assertEqual(c.previous_time, arm_before[0])
        np.testing.assert_array_equal(c.previous_leg_rate, arm_before[1])
        self.assertEqual((c._seek_time, c._seek_depth, c._seek_rate, c._seek_jet), seek_before)
        self.assertEqual(len(c._robust_payloads), 1)
        self.assertEqual(parity['role'], 'final-context-parity')
        self.assertEqual(forecast['role'], 'causal-observer')
        self.assertEqual(forecast['proposedMotorControl'], parity['proposedMotorControl'])
        self.assertEqual(forecast['probeControllerInput']['referenceContext'],
                         prospective_unload_context(c, context_before).receipt())
        self.assertEqual(forecast['proposedMotorControlSHA256'], control_sha(forecast['proposedMotorControl']))
        self.assertFalse(forecast['proposedControlApplied'])
        self.assertEqual(forecast['physicsRolloutIntegrations'], 0)
        self.assertTrue(forecast['nativeBytesAndCounterUnchanged'])

    def test_selection_command_has_distinct_observer_and_final_parity_forecasts(self):
        c = self.controller()
        corners, _, clearance = reviewed.actual_stance_patch(self.w, c.stance_side)
        # Isolate reference lifecycle from the real measured selection policy.
        # Selection adds one harmless marker inequality to the final context.
        # Every QP and motor-response calculation below is nevertheless native.
        def capture(phase, width, J, com, v, rows, bounds):
            if c.robust_support.get('selected'):
                row = np.zeros(width); row[0] = 1.
                rows.append(row); bounds.append(1e6)
        c.add_capture_constraints = capture
        def observe(t, phase):
            _, forecast = c._forecast_unload_risk(t, corners, clearance)
            c.robust_support = dict(selected=True, causalHistoryOnly=True,
                                    selectedCausalUnloadForecast=forecast)
            c._robust_delta = np.zeros(3)
            c._robust_start, c._robust_duration = t, 1.
        with patch.object(c, '_observe_support', side_effect=observe) as observer, \
             patch.object(reviewed, 'limb_telemetry', wraps=limb_telemetry) as measurement, \
             patch.object(reviewed, 'make_context', wraps=reviewed.make_context) as builds:
            _, info = c.solve(.37, 'transfer')
            c.solve(.37, 'transfer')
        self.assertEqual(observer.call_count, 1)
        self.assertEqual(measurement.call_count, 1)
        self.assertEqual(builds.call_count, 2)
        forecasts = info['reviewedWalk']['forecasts']
        self.assertEqual([f['role'] for f in forecasts],
                         ['causal-observer', 'final-context-parity'])
        observer_context, final_context = [f['probeControllerInput']['referenceContext'] for f in forecasts]
        self.assertEqual(len(observer_context['capture_constraints']), 0)
        self.assertEqual(len(final_context['capture_constraints']), 1)
        self.assertEqual(final_context['capture_constraints'], info['referenceContext']['capture_constraints'])
        for key in ('com', 'joints', 'arms', 'orientations'):
            self.assertEqual(observer_context[key], final_context[key])
        self.assertEqual(len(c._robust_payloads), 1)
        self.assertNotEqual(forecasts[0]['qpAssemblySHA256']['A'], forecasts[1]['qpAssemblySHA256']['A'])
        self.assertEqual(info['robustSupportAdmission']['selectionCommandFinalContextForecast']['role'],
                         'final-context-parity')

    def test_draft_reuse_token_binds_active_tasks_and_selected_support(self):
        c = self.controller(); c.solve(.37, 'transfer')
        baseline = c._prepared_inputs_token(.37, 'transfer')
        for name, value in (('_reviewed_mode', 'continuous-com'),
                ('arm_posture', c.arm_posture+.001),
                ('arm_velocity', c.arm_velocity+.001),
                ('arm_acceleration', c.arm_acceleration+.001),
                ('_robust_delta', np.array([.001, 0., 0.])),
                ('_robust_start', .2), ('_robust_duration', 1.1),
                ('_robust_clock', (.1, .2, .3)), ('_robust_weak', 2),
                ('_robust_survivors', [0, 1, 3]), ('_robust_initial_reserve', .002)):
            with self.subTest(name=name):
                existed, previous = hasattr(c, name), getattr(c, name, None)
                setattr(c, name, value)
                self.assertNotEqual(c._prepared_inputs_token(.37, 'transfer'), baseline)
                if existed: setattr(c, name, previous)
                else: delattr(c, name)
                self.assertEqual(c._prepared_inputs_token(.37, 'transfer'), baseline)
        c.robust_support['currentCausalUnloadForecast'] = {'descriptive': True}
        self.assertEqual(c._prepared_inputs_token(.37, 'transfer'), baseline)
        c.robust_support['selected'] = True
        self.assertNotEqual(c._prepared_inputs_token(.37, 'transfer'), baseline)

    def test_soft_load_none_transfer_matches_unchanged_com_context_command(self):
        ordinary, load = self.controller(), self.controller()
        ordinary.install_reviewed(1, 'continuous-com')
        load.install_reviewed(1, 'continuous-com-load')
        expected, a = ordinary.solve(.37, 'transfer')
        actual, b = load.solve(.37, 'transfer')
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(a['qpAssemblySHA256'], b['qpAssemblySHA256'])
        self.assertIsNone(json.loads(load._prepared_reference_context.config_json)['softLoadPreference'])
        self.assertNotIn('softLoadHinge', b)

    def test_soft_load_native_qp_has_only_declared_border_and_excludes_slack_from_forces(self):
        c = self.controller(); c.install_reviewed(1, 'continuous-com-load')
        # Synthetic qualified-anchor metadata isolates matrix consumption. This
        # does not qualify a landing in the cold, zero-integration fixture.
        c._soft_load_anchor = AcquiredLoad(self.counter, self.now, 1, c.swing_side,
                                         31., self.w.mass*9.81, 80)
        measured = self.w.arr(6, (17,3))[1].copy()
        assemblies = []; factory = core.clarabel.DefaultSolver
        def solver(P, q, A, b, cones, settings):
            assemblies.append((P.toarray().copy(), np.array(q), A.toarray().copy(),
                               np.array(b), tuple(x.dim for x in cones)))
            return factory(P, q, A, b, cones, settings)
        with patch.object(c, 'corrected_com_reference', return_value=(measured, np.zeros(3), np.zeros(3))), \
             patch.object(core.clarabel, 'DefaultSolver', side_effect=solver):
            _, info = c.solve(0., 'settle')
            context = c._prepared_reference_context
            config = json.loads(context.config_json); config['softLoadPreference'] = None
            plain = replace(context, config_json=canonical(config))
            Controller.solve(c, 0., 'settle', reference_context=plain)
        P, q, A, b, cones = assemblies[0]; baseP, baseq, baseA, baseb, basecones = assemblies[1]
        n = len(baseq)
        np.testing.assert_array_equal(P[:n,:n], baseP)
        np.testing.assert_array_equal(q[:n], baseq)
        np.testing.assert_array_equal(A[:-2,:n], baseA)
        np.testing.assert_array_equal(b[:-2], baseb)
        np.testing.assert_array_equal(P[:n,n], 0.)
        np.testing.assert_array_equal(A[:-2,n], 0.)
        self.assertEqual(P[n,n], .0005**2)
        self.assertEqual(cones, (basecones[0], basecones[1]+2))
        self.assertEqual(len(info['predictedForcesN']), 8)
        self.assertEqual(info['softLoadHinge']['baseQPWidth'], n)
        self.assertEqual(info['softLoadHinge']['augmentedQPWidth'], n+1)
        self.assertFalse(info['softLoadHinge']['hardForceFloor'])
        with self.assertRaisesRegex(ValueError, 'outside declared settle'):
            Controller.solve(c, 0., 'transfer', reference_context=replace(context, phase='transfer'))

    def test_acquired_load_handoff_is_once_and_bound_to_current_native_row(self):
        c = self.controller(); c.install_reviewed(1, 'continuous-com-load')
        row = dict(sourceStep=self.counter, sourceTimeS=self.now, phase='land', stepIndex=1,
                   feet=[dict(loadN=31.), dict(loadN=369.)])
        with self.assertRaisesRegex(ValueError, 'invalid or replaced'):
            c.prepare_phase_handoff('transfer', 'settle', row, landing_consecutive_samples=80)
        c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=80)
        anchor = c._soft_load_anchor
        self.assertEqual(anchor.source_step, self.counter)
        self.assertEqual(anchor.measured_normal_n, row['feet'][c.swing_side]['loadN'])
        with self.assertRaisesRegex(ValueError, 'invalid or replaced'):
            c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=81)
        self.assertIs(c._soft_load_anchor, anchor)

    def test_nonzero_native_arm_derivative_survives_forecast_and_reference_edit(self):
        c = self.controller()
        legs, rate, _ = limb_telemetry(self.w, 'leg')
        acceleration = np.array([.4, -.3])
        c.previous_time = self.now-.005
        c.previous_leg_rate = rate-.005*acceleration
        mapped, jac, hessian = self.mapping.evaluate(-(legs-c.leg_origin))
        dq = jac*np.repeat(-rate, 3)
        ddq = hessian*np.repeat(rate**2, 3)-jac*np.repeat(acceleration, 3)
        s, ds, dds = clock(.37, c.arm_placement_duration)
        expected = dds*(mapped-c.arm_start)+2*ds*dq+s*ddq
        _, info = c.solve(.37, 'transfer')
        np.testing.assert_allclose(c._arm_causal['measuredLegPitchAccelerationRadps2'],
                                   acceleration, atol=1e-12, rtol=0.)
        np.testing.assert_allclose(info['opposedArmSwing']['targetArmAccelerationRadps2'],
                                   expected, atol=1e-12, rtol=0.)
        corners, _, clearance = reviewed.actual_stance_patch(self.w, c.stance_side)
        _, forecast = c._forecast_unload_risk(.37, corners, clearance)
        np.testing.assert_allclose(forecast['probeControllerInput']['referenceContext']['arms']['acceleration'],
                                   expected, atol=1e-12, rtol=0.)
        c._reference_epoch += 1
        _, repeated = c.solve(.37, 'transfer')
        np.testing.assert_allclose(repeated['opposedArmSwing']['targetArmAccelerationRadps2'],
                                   expected, atol=1e-12, rtol=0.)
        np.testing.assert_allclose(c._arm_causal['measuredLegPitchAccelerationRadps2'],
                                   acceleration, atol=1e-12, rtol=0.)


if __name__ == '__main__': unittest.main()
