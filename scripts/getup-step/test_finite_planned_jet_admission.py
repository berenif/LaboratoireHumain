"""Pure/mock finite planned-jet contracts; no native worlds or solves."""
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from coupled_posture_kinematics import pose_jet_from_source
from coupled_posture_reference import Jet
import finite_planned_jet_admission as admission


CONTACTS = ((1, (0., 0., 0.)), (1, (.1, 0., .001)), (1, (0., .1, -.001)))
ORIGIN = np.array([.02, .03, .001])


def sample(*, arm_velocity=0., root_velocity=0., acceleration=0., joints=None,
           command_height=.8, command_velocity=0., command_acceleration=0.):
    q = np.zeros(21) if joints is None else np.array(joints)
    velocity = np.zeros(21)
    velocity[15] = arm_velocity
    joint_acceleration = np.zeros(21)
    joint_acceleration[15] = acceleration
    root = pose_jet_from_source(Jet([.1, .2, .9], [root_velocity, 0., 0.], [0., 0., 0.]),
                                np.eye(3), np.zeros(3), np.zeros(3))
    geometry = SimpleNamespace(full_joints=Jet(q, velocity, joint_acceleration), root_model=root,
                               com_source=Jet.rest([.1, .2, .9]))
    return SimpleNamespace(geometric=geometry,
        com=Jet([.1, .2, command_height], [command_velocity, 0., 0.],
                [command_acceleration, 0., 0.]),
        diagnostics=dict(initialCompatibilitySlackActive=True))


class Scratch:
    def __init__(self, counter=0):
        self.base = Path('/fixture/original')
        self.xml = self.base / 'model.xml'
        self.helper = self.base / 'helper.so'
        self.limits = np.tile([-1., 1.], (21, 1))
        self.h = object()
        self.mass = 40.
        self.counter = counter
        self.payload = b'complete-native-payload'
        self.installs = []
        self.closed = False
        self.close_count = 0
        self.l = SimpleNamespace(lh_dim=lambda *_: self.counter,
                                 lh_step_audited=Mock(side_effect=AssertionError('unexpected native step')))

    def install_cold(self, value):
        if self.counter != 0:
            raise AssertionError('attempted installation into actual/running world')
        self.installs.append(copy.deepcopy(value))

    def close(self):
        self.closed = True
        self.close_count += 1


def primal_receipt(**changes):
    return dict(dict(success=True, accepted=True, maximumMotorFraction=.5,
        originalStrictMotorCap=.925, maximumNativeEquilibriumResidual=0.,
        maximumFrictionViolation=0., normalForceUpperBound=None, friction=.6,
        motorCommands=[.5] * 21, forcesSource=[[0., 0., 100.]] * 3), **changes)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.actual = Scratch(counter=3200)
        self.worlds = []
        self.provider_calls = []
        self.static_calls = []
        self.native_changes = {}
        self.native_receipt_changes = {}
        self.entry = sample(arm_velocity=.03, acceleration=.04)
        self.moving = sample(root_velocity=.02, arm_velocity=.01, acceleration=.02)
        self.endpoint = sample()
        self.samples = (('entry-with-moving-arms', self.entry), ('moving', self.moving),
                        ('stationary-endpoint', self.endpoint))
        self.patches = [
            patch.object(admission, 'sha', side_effect=self.sha),
            patch.object(admission, 'native_bytes', side_effect=lambda w: w.payload),
            patch.object(admission, 'ContactWrenchCone', side_effect=self.cone),
            patch.object(admission, 'NativePlannedJetProvider', side_effect=self.provider),
            patch.object(admission, 'independent_native_primal_check', return_value=primal_receipt()),
            patch.object(admission, 'strict_contact_lp', side_effect=self.static_lp),
        ]
        self.mocks = [p.start() for p in self.patches]
        self.addCleanup(self.stop_patches)

    def stop_patches(self):
        for p in reversed(self.patches):
            p.stop()

    @staticmethod
    def sha(path):
        path = Path(path)
        # Same original content may live at different base paths.
        if 'wrong-bridge' in str(path) and path.name == 'source_bridge_v2.so':
            return 'different-bridge'
        if 'wrong-engine' in str(path) and path.name == 'libmujoco_nogl.so.2.1.3':
            return 'different-engine'
        return 'sha-' + path.name

    @staticmethod
    def cone(points, origin):
        return SimpleNamespace(points=np.array(points), origin=np.array(origin),
            receipt=dict(pointsSource=np.array(points).tolist(), originSource=np.array(origin).tolist(),
                         normalForceDomain='nonnegative-unbounded', friction=.6))

    def factory(self, actual):
        self.assertIs(actual, self.actual)
        w = Scratch()
        self.worlds.append(w)
        return w

    def provider(self, w, contacts, origin, stance_pose):
        self.assertEqual(contacts, CONTACTS)
        np.testing.assert_array_equal(origin, ORIGIN)
        np.testing.assert_array_equal(stance_pose, np.eye(4))

        def provide(value, *, final=False):
            self.assertTrue(final)
            qpos, velocity, acceleration = admission.generalized_planned_jets(value.geometric)
            w.install_cold(dict(qpos=qpos.tolist(), qvel=velocity.tolist()))
            self.provider_calls.append((w, value, velocity.copy(), acceleration.copy()))
            native = dict(generalizedVelocity=velocity.tolist(),
                generalizedAcceleration=acceleration.tolist(),
                initialCompatibilitySlackActive=True,
                commandGeometricCOMPositionMismatchM=.1, nativeAdmission=False)
            native.update(self.native_receipt_changes)
            data = dict(B=np.zeros((27, 21)), G=np.zeros((27, 9)),
                demand=np.r_[np.zeros(6), acceleration[6:]], stanceJacobian=np.zeros((6, 27)),
                referenceOriginSource=origin.tolist(), forceScaleN=392.4,
                minimumCleanSelfGapM=.03, nativeCOMHeightM=.9, native=native)
            data.update(self.native_changes)
            return data
        return provide

    def static_lp(self, w, contacts):
        self.assertEqual(contacts, CONTACTS)
        self.assertEqual(w.installs[-1]['qvel'], [0.] * 27)
        self.static_calls.append((w, copy.deepcopy(w.installs[-1])))
        return dict(success=True, requiredMotorFraction=.5,
                    maximumEqualityResidual=0., maximumFrictionViolation=0.)

    def run_admission(self, samples=None, factory=None, **kwargs):
        return admission.finite_planned_jet_admission(self.actual,
            self.samples if samples is None else samples, CONTACTS, ORIGIN, np.eye(4),
            factory=self.factory if factory is None else factory, **kwargs)

    def assert_actual_untouched(self):
        self.assertEqual(self.actual.payload, b'complete-native-payload')
        self.assertEqual(self.actual.counter, 3200)
        self.assertEqual(self.actual.installs, [])
        self.assertFalse(self.actual.closed)

    def test_two_independent_worlds_keep_moving_jets_and_separate_static_checks(self):
        result = self.run_admission()
        self.assertTrue(result['accepted'])
        self.assertEqual(result['worldsConstructed'], 2)
        self.assertIsNot(self.worlds[0], self.worlds[1])
        self.assertIsNot(self.worlds[0].h, self.worlds[1].h)
        self.assertEqual(len(self.provider_calls), 6)
        self.assertEqual(len(self.static_calls), 4)
        self.assertEqual([len(w.installs) for w in self.worlds], [5, 5])
        self.assertTrue(all(w.closed and w.close_count == 1 for w in self.worlds))
        self.assert_actual_untouched()
        for row, (_, value, velocity, acceleration) in zip(result['checks'], self.provider_calls):
            np.testing.assert_array_equal(row['plannedNativeInputs']['qvel'], velocity)
            np.testing.assert_array_equal(row['plannedNativeInputs']['qacc'], acceleration)
            for key, jet in (('geometricFullJointJet', value.geometric.full_joints),
                             ('geometricRootModelJet', value.geometric.root_model),
                             ('geometricCOMJet', value.geometric.com_source)):
                np.testing.assert_array_equal(row['plannedNativeInputs'][key], jet.arrays())
            self.assertEqual(row['plannedNativeInputs']['demand'][21], acceleration[21])
            self.assertEqual(row['plannedNative']['commandGeometricCOMPositionMismatchM'], .1)
            static = row['additionalStaticReserve']
            if value is self.moving:
                self.assertIsNone(static)
                self.assertGreater(max(abs(velocity)), 0.)
            else:
                self.assertEqual(static['qpos'], row['plannedNativeInputs']['qpos'])
                self.assertEqual(static['qvel'], [0.] * 27)
                self.assertTrue(static['accepted'])
        entry = result['checks'][0]
        self.assertFalse(entry['stationaryFullGeneralizedJets'])
        self.assertEqual(entry['additionalStaticReserve']['reasons'], ['first-sample'])
        self.assertGreater(entry['plannedNativeInputs']['qvel'][21], 0.)
        self.assertGreater(entry['plannedNativeInputs']['qacc'][21], 0.)
        self.assertEqual(result['plannedJetInstallations'], 6)
        self.assertEqual(result['additionalStaticInstallations'], 4)
        self.assertEqual(result['counterZeroScratchInstallations'], 10)
        self.assertEqual(result['physicsIntegrations'], 0)
        self.assertEqual(result['actualStateWrites'], 0)
        self.assertTrue(result['finiteSamplesOnly'])
        self.assertFalse(result['wholePathOrDynamicQualification'])
        self.assertFalse(result['nativeAdmission'])
        self.assertTrue(result['fullPilotPending'])
        self.assertEqual(result['policy'], admission.POLICY)

    def test_stationarity_uses_full_root_and_arm_velocity_and_acceleration(self):
        cases = ((sample(root_velocity=2e-8), False), (sample(arm_velocity=2e-8), False),
                 (sample(acceleration=2e-8), False), (sample(root_velocity=1e-8), True),
                 (sample(arm_velocity=1e-8, acceleration=1e-8), True))
        for value, expected in cases:
            with self.subTest(expected=expected):
                result = self.run_admission((('entry', self.entry), ('check', value)))
                self.assertEqual(result['checks'][1]['stationaryFullGeneralizedJets'], expected)
                self.assertEqual(result['checks'][1]['additionalStaticReserve'] is not None, expected)

    def test_original_static_reserve_rejects_without_repair_or_second_world(self):
        for changes in (dict(success=False), dict(requiredMotorFraction=.925),
                        dict(maximumEqualityResidual=1e-8), dict(maximumFrictionViolation=1e-8),
                        dict(requiredMotorFraction=float('nan')), dict(requiredMotorFraction=-.1)):
            with self.subTest(changes=changes):
                receipt = dict(success=True, requiredMotorFraction=.5,
                               maximumEqualityResidual=0., maximumFrictionViolation=0.)
                receipt.update(changes)
                self.worlds.clear()
                with patch.object(admission, 'strict_contact_lp', return_value=receipt):
                    result = self.run_admission()
                self.assertFalse(result['accepted'])
                self.assertEqual(len(result['checks']), 1)
                self.assertEqual(len(self.worlds), 1)
                self.assertTrue(self.worlds[0].closed)
                self.assertEqual(len(self.worlds[0].installs), 2)
                self.assert_actual_untouched()

    def test_dynamic_witness_is_original_strict_motor_friction_and_unbounded_normal(self):
        for changes in (dict(success=False), dict(accepted=False), dict(maximumMotorFraction=.925),
                        dict(maximumNativeEquilibriumResidual=1e-8), dict(maximumFrictionViolation=1e-8),
                        dict(maximumMotorFraction=float('nan')), dict(friction=.61),
                        dict(normalForceUpperBound=400.), dict(originalStrictMotorCap=.95)):
            with self.subTest(changes=changes):
                with patch.object(admission, 'independent_native_primal_check',
                                  return_value=primal_receipt(**changes)):
                    result = self.run_admission()
                self.assertFalse(result['accepted'])
                self.assertFalse(result['checks'][0]['plannedJetCheckAccepted'])
                self.assertEqual(len(result['checks']), 1)

    def test_physical_guards_keep_original_strict_gap_and_both_heights(self):
        for changes in (dict(minimumCleanSelfGapM=.025), dict(nativeCOMHeightM=.65 - 1e-9),
                        dict(minimumCleanSelfGapM=float('nan'))):
            with self.subTest(changes=changes):
                self.native_changes = changes
                self.assertFalse(self.run_admission()['accepted'])
        self.native_changes = dict(nativeCOMHeightM=.65)
        self.assertTrue(self.run_admission((('height-boundary', sample(command_height=.65)),))['accepted'])
        self.assertFalse(self.run_admission((('low-command', sample(command_height=.65 - 1e-9)),))['accepted'])

    def test_total_command_com_caps_keep_original_values_and_roundoff(self):
        for attribute, cap in (('command_velocity', .19260552331163003),
                               ('command_acceleration', .3901818436690813)):
            with self.subTest(attribute=attribute):
                self.assertTrue(self.run_admission((('bound', sample(**{attribute: cap})),))['accepted'])
                self.assertFalse(self.run_admission((('exceeded', sample(**{attribute: cap + 2e-10})),))['accepted'])

    def test_lower_and_arm_joint_reserves_are_unchanged(self):
        q = np.zeros(21)
        q[0], q[15] = -1. + .01, 1. - .02
        self.assertTrue(self.run_admission((('boundary', sample(joints=q)),))['accepted'])
        for index in (0, 15):
            shifted = q.copy()
            shifted[index] += -2e-9 if index == 0 else 2e-9
            self.assertFalse(self.run_admission((('outside', sample(joints=shifted)),))['accepted'])

    def test_all_four_scratch_file_bindings_are_checked_before_provider(self):
        for kind in ('model', 'helper', 'bridge', 'engine'):
            with self.subTest(kind=kind):
                w = Scratch()
                if kind == 'model':
                    w.xml = w.base / 'different-model.xml'
                elif kind == 'helper':
                    w.helper = w.base / 'different-helper.so'
                else:
                    w.base = Path('/fixture/wrong-' + kind)
                with self.assertRaisesRegex(ValueError, 'binding differs'):
                    self.run_admission(factory=lambda _: w)
                self.assertTrue(w.closed)
                self.assertEqual(w.installs, [])
                self.assert_actual_untouched()

    def test_different_original_limits_fail_before_any_install(self):
        w = Scratch()
        w.limits[0, 0] += 1e-7
        with self.assertRaisesRegex(ValueError, 'joint limits differ'):
            self.run_admission(factory=lambda _: w)
        self.assertTrue(w.closed)
        self.assertEqual(w.installs, [])

    def test_actual_or_reused_scratch_cannot_supply_two_checks(self):
        with self.assertRaisesRegex(ValueError, 'independent fresh counter-zero'):
            self.run_admission(factory=lambda _: self.actual)
        self.assert_actual_untouched()
        w = Scratch()
        with self.assertRaisesRegex(ValueError, 'independent fresh counter-zero'):
            self.run_admission(factory=lambda _: w)
        self.assertEqual(w.close_count, 1)
        self.assertEqual(len(w.installs), 5)

    def test_nonzero_scratch_counter_is_closed_without_installation(self):
        w = Scratch(counter=1)
        with self.assertRaisesRegex(ValueError, 'counter-zero'):
            self.run_admission(factory=lambda _: w)
        self.assertTrue(w.closed)
        self.assertEqual(w.installs, [])

    def test_shared_actual_native_interface_is_refused_and_new_scratch_closed(self):
        w = Scratch()
        w.l = self.actual.l
        original_step = self.actual.l.lh_step_audited
        with self.assertRaisesRegex(ValueError, 'independent scratch native interface'):
            self.run_admission(factory=lambda _: w)
        self.assertTrue(w.closed)
        self.assertEqual(w.installs, [])
        self.assertIs(self.actual.l.lh_step_audited, original_step)
        self.assert_actual_untouched()

    def test_alias_of_actual_native_handle_is_never_closed_or_installed(self):
        w = Scratch()
        w.h = self.actual.h
        with self.assertRaisesRegex(ValueError, 'independent fresh counter-zero'):
            self.run_admission(factory=lambda _: w)
        self.assertFalse(w.closed)
        self.assertEqual(w.installs, [])
        self.assert_actual_untouched()

    def test_origin_or_generalized_receipt_mismatch_fails_closed(self):
        self.native_changes = dict(referenceOriginSource=[.02, .03, .002])
        with self.assertRaisesRegex(ValueError, 'bound fixed origin'):
            self.run_admission()
        self.assertTrue(self.worlds[-1].closed)
        self.native_changes = {}
        self.native_receipt_changes = dict(generalizedVelocity=[0.] * 27)
        with self.assertRaisesRegex(ValueError, 'declared planned jets'):
            self.run_admission()
        self.assertTrue(self.worlds[-1].closed)

    def test_actual_byte_and_counter_mutation_each_fail_closed(self):
        for kind in ('bytes', 'counter'):
            with self.subTest(kind=kind):
                self.actual = Scratch(counter=3200)
                def mutate(actual):
                    w = self.factory(actual)
                    if kind == 'bytes':
                        actual.payload = b'changed-native-payload'
                    else:
                        actual.counter += 1
                    return w
                with self.assertRaisesRegex(ValueError, 'mutated the actual native world'):
                    self.run_admission(factory=mutate)
                self.assertTrue(self.worlds[-1].closed)
                self.assertFalse(self.actual.closed)

    def test_actual_guard_runs_even_when_scratch_provider_raises(self):
        def mutate_then_raise(*_):
            self.actual.payload = b'changed-native-payload'
            raise RuntimeError('provider failed')
        with patch.object(admission, 'NativePlannedJetProvider', side_effect=mutate_then_raise):
            with self.assertRaisesRegex(ValueError, 'mutated the actual native world'):
                self.run_admission()
        self.assertTrue(self.worlds[-1].closed)

    def test_second_provider_failure_closes_both_worlds(self):
        original = self.provider
        def fail_second(w, *args):
            if len(self.worlds) == 2:
                raise RuntimeError('second provider refused')
            return original(w, *args)
        with patch.object(admission, 'NativePlannedJetProvider', side_effect=fail_second):
            with self.assertRaisesRegex(RuntimeError, 'second provider refused'):
                self.run_admission()
        self.assertEqual(len(self.worlds), 2)
        self.assertTrue(all(w.closed for w in self.worlds))
        self.assert_actual_untouched()

    def test_integration_is_explicitly_forbidden_and_scratch_closed(self):
        def attempt_step(w, *_):
            w.l.lh_step_audited(w.h, 1)
        with patch.object(admission, 'NativePlannedJetProvider', side_effect=attempt_step):
            with self.assertRaisesRegex(AssertionError, 'must never integrate'):
                self.run_admission()
        self.assertTrue(self.worlds[-1].closed)
        self.assert_actual_untouched()

    def test_empty_samples_invalid_contacts_and_nonfinite_jets_never_construct_worlds(self):
        with self.assertRaisesRegex(ValueError, 'nonempty'):
            self.run_admission(())
        for contacts in (CONTACTS[:2], CONTACTS[:2] + (CONTACTS[0],),
                         ((0, CONTACTS[0][1]), *CONTACTS[1:])):
            with self.assertRaisesRegex(ValueError, 'three unique actual contacts'):
                admission.finite_planned_jet_admission(self.actual, self.samples, contacts,
                    ORIGIN, np.eye(4), factory=self.factory)
        self.assertEqual(self.worlds, [])
        bad = sample()
        bad.com = SimpleNamespace(arrays=lambda: (np.array([0., 0., np.nan]), np.zeros(3), np.zeros(3)))
        with self.assertRaisesRegex(ValueError, 'command COM jet'):
            self.run_admission((('bad-com', bad),))
        self.assertTrue(self.worlds[-1].closed)
        self.assert_actual_untouched()

    def test_all_worlds_close_and_actual_is_checked_even_if_one_close_fails(self):
        original = self.factory
        def close_failure(actual):
            w = original(actual)
            if len(self.worlds) == 1:
                close = w.close
                def fail():
                    close()
                    raise RuntimeError('close failed')
                w.close = fail
            return w
        with self.assertRaisesRegex(RuntimeError, 'scratch close failed'):
            self.run_admission(factory=close_failure)
        self.assertTrue(all(w.closed for w in self.worlds))
        self.assert_actual_untouched()


if __name__ == '__main__':
    unittest.main(verbosity=2)
