"""Terminal-only pelvis reference/protocol tests; native tests never integrate."""
import argparse
import copy
from dataclasses import FrozenInstanceError, replace
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

import numpy as np
from scipy.spatial.transform import Rotation

from source_arm_knee_mobility import native_bytes
from source_reference_context import Orientation, canonical, digest
from source_reviewed_walk import ReviewedController, ReviewedSteps
from source_step_probe import ROOT, World
from source_terminal_pelvis import (DURATION_S, MAXIMUM_ANGLE_RAD, MODE, PROFILE,
    TerminalActivation, TerminalPelvisController, TerminalPelvisSteps, argument_parser)
from source_upright_large_steps import ExtendedArmMap
from test_source_reference_context import sample_context


def fake_controller(final=1):
    state = SimpleNamespace(step=1000, time=1.25,
        rotation=Rotation.from_euler('ZYX', [.4, .3, -.1]).as_matrix())
    rotations = lambda *args: np.array([state.rotation.copy() for _ in range(17)])
    world = SimpleNamespace(h=None, mass=40., arr=rotations,
        l=SimpleNamespace(lh_dim=lambda h, k: state.step, lh_time=lambda h: state.time))
    controller = TerminalPelvisController.__new__(TerminalPelvisController)
    controller.w = world
    controller.install_reviewed(1, MODE)
    controller._terminal_final_step_index = final
    return controller, state


def measured_row(state, phase='settle', loads=(100., 300.)):
    return dict(sourceStep=state.step, sourceTimeS=state.time, stepIndex=1,
        phase=phase, feet=[dict(loadN=x) for x in loads],
        swingForwardDisplacementM=.181, requestedForwardM=.18)


def qualified_activation(controller, state):
    landing = measured_row(state, 'land')
    controller.prepare_phase_handoff('land', 'settle', landing, landing_consecutive_samples=80)
    for _ in range(81):
        state.step += 1
        state.time = state.step*.00125
        controller.record_terminal_native_sample(measured_row(state))
    controller.prepare_phase_handoff('settle', 'stand', measured_row(state),
        landing_consecutive_samples=80)
    return controller._terminal_activation


class TerminalProtocol(unittest.TestCase):
    def test_cli_is_one_or_ten_and_bounded_compute_only(self):
        parser = argument_parser()
        args = parser.parse_args(['--output', '/tmp/test', '--inspector', '/tmp/inspector'])
        self.assertEqual(args.count, 1)
        self.assertEqual(args.wall_cap_s, 3600.)
        for count in (1, 10):
            self.assertEqual(parser.parse_args(['--output', '/tmp/t', '--inspector', '/tmp/i',
                '--count', str(count), '--wall-cap-s', '3600']).count, count)
        for value in ('nan', 'inf', '119', '7201'):
            with self.subTest(cap=value), self.assertRaises(SystemExit):
                parser.parse_args(['--output', '/tmp/t', '--inspector', '/tmp/i', '--wall-cap-s', value])
        with self.assertRaises(SystemExit):
            parser.parse_args(['--output', '/tmp/t', '--inspector', '/tmp/i', '--count', '2'])
        c, _ = fake_controller()
        with self.assertRaisesRegex(ValueError, 'unchanged causal'):
            c.install_reviewed(1, 'baseline-context')

    def test_activation_is_once_bound_to_current_landing_and_native_loaded_history(self):
        c, state = fake_controller()
        rotation = state.rotation.copy()
        activation = qualified_activation(c, state)
        receipt = activation.receipt()
        self.assertEqual(receipt['sourceStep'], state.step)
        self.assertEqual(receipt['sourceTimeS'], state.time)
        self.assertEqual(receipt['stepIndex'], receipt['finalStepIndex'])
        self.assertEqual(len(receipt['loadedHistory']['samples']), 81)
        self.assertGreaterEqual(receipt['loadedHistory']['durationS'], .1-1e-10)
        np.testing.assert_array_equal(receipt['initialRotationSource'], rotation)
        self.assertLessEqual(receipt['correctionAngleRad'], MAXIMUM_ANGLE_RAD+1e-14)
        unsigned = copy.deepcopy(receipt)
        self.assertEqual(unsigned.pop('activationSHA256'), digest(unsigned))
        receipt['initialRotationSource'][0][0] = 99.
        self.assertNotEqual(activation.receipt()['initialRotationSource'][0][0], 99.)
        with self.assertRaises(FrozenInstanceError):
            activation.receipt_json = '{}'
        state.rotation = np.eye(3)
        with self.assertRaisesRegex(ValueError, 'replaced terminal stand'):
            c.prepare_phase_handoff('settle', 'stand', measured_row(state), landing_consecutive_samples=80)
        self.assertIs(c._terminal_activation, activation)
        np.testing.assert_array_equal(activation.receipt()['initialRotationSource'], rotation)

    def test_activation_evidence_written_once_and_detached(self):
        c, state = fake_controller()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'terminal-pelvis-activation.json'
            c._terminal_evidence_path = path
            activation = qualified_activation(c, state)
            self.assertEqual(json.loads(path.read_text()), activation.receipt())

    def test_no_early_nonfinal_stale_or_unloaded_activation(self):
        c, state = fake_controller(final=10)
        qualified_activation(c, state)
        self.assertIsNone(c._terminal_activation)
        for defect in ('short', 'low_each', 'low_total', 'gap', 'no_landing', 'stale'):
            with self.subTest(defect=defect):
                c, state = fake_controller()
                if defect != 'no_landing':
                    c.prepare_phase_handoff('land', 'settle', measured_row(state, 'land'), landing_consecutive_samples=80)
                for index in range(80 if defect == 'short' else 81):
                    state.step += 2 if defect == 'gap' and index == 50 else 1
                    state.time = state.step*.00125
                    loads = (2., 398.) if defect == 'low_each' and index == 50 else (
                        (10., 20.) if defect == 'low_total' and index == 50 else (100., 300.))
                    c.record_terminal_native_sample(measured_row(state, loads=loads))
                row = measured_row(state)
                if defect == 'stale': row['sourceStep'] -= 1
                with self.assertRaises(ValueError):
                    c.prepare_phase_handoff('settle', 'stand', row, landing_consecutive_samples=80)
                self.assertIsNone(c._terminal_activation)

    def test_landing_must_be_current_qualified_and_cannot_be_replaced(self):
        c, state = fake_controller()
        row = measured_row(state, 'land')
        for samples, changed in ((79, {}), (80, {'phase':'transfer'}),
                (80, {'sourceStep':state.step-1}), (80, {'swingForwardDisplacementM':.179})):
            with self.subTest(samples=samples, changed=changed), self.assertRaises(ValueError):
                c.prepare_phase_handoff('land', 'settle', dict(row, **changed), landing_consecutive_samples=samples)
        c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=80)
        with self.assertRaisesRegex(ValueError, 'replaced terminal final'):
            c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=80)

    def test_quintic_orientation_and_weight_have_frozen_bounded_endpoint(self):
        c, state = fake_controller()
        activation = qualified_activation(c, state)
        receipt = activation.receipt()
        start = receipt['sourceTimeS']
        axis = np.array(receipt['axisWorldRad'])
        R0 = np.array(receipt['initialRotationSource'])
        for elapsed, scalar in ((0., 0.), (2., .5), (4., 1.), (12., 1.)):
            sample = activation.sample(start+elapsed)
            self.assertAlmostEqual(sample['clock'][0], scalar)
            self.assertAlmostEqual(sample['orientation']['weight'], .5*scalar)
            np.testing.assert_allclose(np.array(sample['orientation']['rotation']).reshape(3,3),
                Rotation.from_rotvec(axis*scalar).as_matrix()@R0, atol=1e-15)
            self.assertEqual(sample['orientation']['body'], 4)
            self.assertEqual(sample['orientation']['kp'], 40.)
            self.assertEqual(sample['orientation']['kd'], 12.)
            if elapsed in (0., 4., 12.):
                np.testing.assert_array_equal(sample['orientation']['velocity'], np.zeros(3))
                np.testing.assert_array_equal(sample['orientation']['acceleration'], np.zeros(3))
        end, held = activation.sample(start+4), activation.sample(start+12)
        self.assertEqual(end['orientation'], held['orientation'])
        with self.assertRaises(ValueError): activation.sample(start-.001)

    def test_small_correction_reaches_same_yaw_upright_without_scaling_up(self):
        c, state = fake_controller()
        state.rotation = Rotation.from_euler('ZYX', [.4, .01, -.02]).as_matrix()
        activation = qualified_activation(c, state)
        receipt = activation.receipt()
        self.assertLess(receipt['correctionAngleRad'], MAXIMUM_ANGLE_RAD)
        np.testing.assert_allclose(receipt['boundedEndpointRotationSource'],
            Rotation.from_euler('Z', .4).as_matrix(), atol=1e-14)

    def test_context_hook_is_exact_preterminal_and_only_appends_pelvis(self):
        c, state = fake_controller()
        context = sample_context()
        for phase in ('transfer', 'lift', 'translate', 'land', 'settle'):
            item = replace(context, phase=phase)
            self.assertIs(c.transform_reviewed_reference_context(item), item)
        with self.assertRaisesRegex(ValueError, 'lacks qualified'):
            c.transform_reviewed_reference_context(replace(context, phase='stand'))
        activation = qualified_activation(c, state)
        stand = replace(context, phase='stand', source_time=state.time+2.)
        changed = c.transform_reviewed_reference_context(stand)
        for name in ('com', 'joints', 'arms', 'arm_gains', 'feet', 'capture_constraints',
                'unload_admitted', 'config_json', 'hypothetical_changes'):
            self.assertIs(getattr(changed, name), getattr(stand, name))
        self.assertIs(changed.orientations[0], stand.orientations[0])
        self.assertEqual(len(changed.orientations), len(stand.orientations)+1)
        self.assertEqual(changed.orientations[-1].body, 4)
        self.assertEqual(json.loads(changed.causal_json)['terminalPelvis']['activation'], activation.receipt())
        c._prepared_reference_context = changed
        with self.assertRaisesRegex(ValueError, 'forecast is forbidden'):
            c._forecast_unload_risk(1., None, None)
        with self.assertRaisesRegex(ValueError, 'replace an existing'):
            c.transform_reviewed_reference_context(changed)

    def test_quiet_resets_through_ramp_and_waits_for_applied_endpoint(self):
        c, state = fake_controller()
        activation = qualified_activation(c, state)
        live = TerminalPelvisSteps.__new__(TerminalPelvisSteps)
        live.controller, live.world, live.quiet_run = c, c.w, 123
        anchor_step, anchor_time = state.step, state.time
        def inherited(_, row):
            live.quiet_run += 1
            row['quietConsecutiveS'] = live.quiet_run*.00125
            return row
        with patch.object(ReviewedSteps, 'decorate', inherited):
            for steps, endpoint, expected in ((1, None, 0), (3199, None, 0),
                    (3200, (anchor_step+3200, anchor_time+4.), 0),
                    (3201, None, 0), (3202, (anchor_step+3201, anchor_time+4.00125), 1)):
                state.step = anchor_step+steps
                state.time = anchor_time+steps*.00125
                c._terminal_endpoint_command = endpoint
                row = live.decorate(measured_row(state, 'stand'))
                self.assertEqual(live.quiet_run, expected)
                self.assertEqual(row['terminalPelvis']['quietCountedSamples'], expected)
                self.assertEqual(row['terminalPelvis']['quietEligible'], bool(expected))


NATIVE_BASE = ROOT/'outputs/ten-step-video-corrections-20261019/cadence-only/model-session'
NATIVE_RECORDS = ROOT/'outputs/ten-step-video-corrections-20261019/cadence-only/run/generation-001'
NATIVE_HELPER = ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
HAS_NATIVE = all(path.is_file() for path in (NATIVE_BASE/'slow-continuation/final.bin',
    NATIVE_RECORDS/'controller-step-01.json', NATIVE_RECORDS/'arm-mapping.json', NATIVE_HELPER))


@unittest.skipUnless(HAS_NATIVE, 'recovered reviewed native fixture absent')
class NativeTerminalCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w = World(NATIVE_BASE, NATIVE_HELPER)
        cls.w.restore(NATIVE_BASE/'slow-continuation/final.bin')
        cls.before = native_bytes(cls.w)
        cls.counter = cls.w.l.lh_dim(cls.w.h, 6)
        cls.now = cls.w.l.lh_time(cls.w.h)
        cls.record = json.loads((NATIVE_RECORDS/'controller-step-01.json').read_text())
        mapping = json.loads((NATIVE_RECORDS/'arm-mapping.json').read_text())
        cls.mapping = ExtendedArmMap(cls.w.arr(0, (28,))[7:], cls.w.limits, mapping['goalRad'])
        cls.blocked = []
        for name in ('lh_step', 'lh_step_audited', 'lh_set_state'):
            if hasattr(cls.w.l, name):
                cls.blocked.append((name, getattr(cls.w.l, name)))
                setattr(cls.w.l, name, lambda *args: (_ for _ in ()).throw(
                    AssertionError('native stepping or state installation forbidden')))

    @classmethod
    def tearDownClass(cls):
        try:
            assert native_bytes(cls.w) == cls.before
        finally:
            for name, function in cls.blocked: setattr(cls.w.l, name, function)
            cls.w.close()

    def tearDown(self):
        self.assertEqual(native_bytes(self.w), self.before)
        self.assertEqual(self.w.l.lh_dim(self.w.h, 6), self.counter)
        self.assertEqual(self.w.l.lh_time(self.w.h), self.now)

    def controller(self, cls):
        c = cls.__new__(cls)
        for key, value in copy.deepcopy(self.record).items():
            if key == 'targets': value = [(np.array(p), np.array(R)) for p, R in value]
            elif isinstance(value, list): value = np.asarray(value, dtype=float)
            setattr(c, key, value)
        c.w, c._mapping = self.w, self.mapping
        c.previous_time = c.previous_leg_rate = None
        c.install_reviewed(1, MODE)
        return c

    def test_preterminal_actual_motor_bytes_and_all_qp_matrices_match(self):
        measured = self.w.arr(6, (17, 3))[1].copy()
        for phase in ('transfer', 'lift', 'translate', 'land', 'settle'):
            with self.subTest(phase=phase):
                ordinary, terminal = self.controller(ReviewedController), self.controller(TerminalPelvisController)
                # Isolate unchanged task assembly on the exact same native
                # state; this cold fixture is not a physically qualified walk.
                jet = (measured.copy(), np.zeros(3), np.zeros(3))
                with patch.object(ordinary, 'corrected_com_reference', return_value=jet), \
                     patch.object(terminal, 'corrected_com_reference', return_value=jet):
                    expected, old = ordinary.solve(.37, phase)
                    actual, info = terminal.solve(.37, phase)
                self.assertEqual(actual.tobytes(), expected.tobytes())
                self.assertEqual(info['qpAssemblySHA256'], old['qpAssemblySHA256'])
                self.assertEqual(info['referenceContext'], old['referenceContext'])
                self.assertEqual(info['reviewedWalk']['featureProfile'], PROFILE)

    def test_unmodified_causal_transfer_matches_without_reference_stub(self):
        ordinary, terminal = self.controller(ReviewedController), self.controller(TerminalPelvisController)
        expected, old = ordinary.solve(.37, 'transfer')
        actual, info = terminal.solve(.37, 'transfer')
        self.assertEqual(actual.tobytes(), expected.tobytes())
        self.assertEqual(info['qpAssemblySHA256'], old['qpAssemblySHA256'])
        self.assertEqual(info['referenceContext'], old['referenceContext'])

    def test_terminal_native_qp_changes_only_soft_orientation_objective(self):
        ordinary, terminal = self.controller(ReviewedController), self.controller(TerminalPelvisController)
        fake, state = fake_controller()
        state.rotation = self.w.arr(5, (17, 3, 3))[4].copy()
        activation = qualified_activation(fake, state)
        receipt = activation.receipt()
        # Synthetic time relocation isolates assembly; never a real admission.
        receipt['sourceTimeS'] = self.now-2.
        receipt.pop('activationSHA256')
        receipt['activationSHA256'] = digest(receipt)
        terminal._terminal_activation = TerminalActivation(canonical(receipt))
        measured = self.w.arr(6, (17, 3))[1].copy()
        jet = (measured, np.zeros(3), np.zeros(3))
        with tempfile.TemporaryDirectory(prefix='terminal-native-witness-') as directory:
            terminal._terminal_native_folder = Path(directory)/'terminal-pelvis-native'
            with patch.object(ordinary, 'corrected_com_reference', return_value=jet), \
                 patch.object(terminal, 'corrected_com_reference', return_value=jet), \
                 patch.object(terminal, '_forecast_unload_risk', side_effect=AssertionError('stand forecast forbidden')):
                _, old = ordinary.solve(.37, 'stand')
                actual, info = terminal.solve(.37, 'stand')
                again, repeated = terminal.solve(.37, 'stand')
            manifest = json.loads((terminal._terminal_native_folder/'manifest.json').read_text())
            self.assertEqual(manifest['profile'], PROFILE)
            self.assertEqual(manifest['physicsIntegrations'], 0)
            self.assertTrue(manifest['actualCommandNotYetApplied'])
            self.assertEqual(len(manifest['samples']), 1)
            witness = manifest['samples'][0]
            self.assertEqual(witness['thresholdS'], .5)
            self.assertEqual(witness['contextSHA256'], info['referenceContext']['contextSHA256'])
            self.assertEqual(witness['actualMotorControlSHA256'], info['actualMotorControlSHA256'])
            self.assertEqual(witness['qpAssemblySHA256'], info['qpAssemblySHA256'])
            payload = zlib.decompress((terminal._terminal_native_folder/witness['filename']).read_bytes())
            self.assertEqual(payload, self.before)
        self.assertEqual(actual.tobytes(), again.tobytes())
        self.assertEqual(info, repeated)
        for key in ('A', 'b'):
            self.assertEqual(info['qpAssemblySHA256'][key], old['qpAssemblySHA256'][key])
        self.assertNotEqual(info['qpAssemblySHA256']['P'], old['qpAssemblySHA256']['P'])
        for key in ('com', 'joints', 'arms', 'arm_gains', 'feet', 'capture_constraints', 'unload_admitted'):
            self.assertEqual(info['referenceContext'][key], old['referenceContext'][key])
        self.assertEqual(info['referenceContext']['orientations'][:-1], old['referenceContext']['orientations'])
        self.assertEqual(info['referenceContext']['orientations'][-1]['body'], 4)


if __name__ == '__main__':
    unittest.main()
