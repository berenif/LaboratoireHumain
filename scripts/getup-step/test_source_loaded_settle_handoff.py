"""Zero-integration loaded-settle thresholds and reference-boundary checks.

No World is created and no native integration or state-install API is called.
The upstream begin_step is mocked so these tests exercise reference carrying
without opening a physical session or changing its counters or slip metrics.
"""
import copy
import inspect
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from source_cadenced_ten_steps import (
    CadencedController, CadencedSteps, DIRECT_PHASE_DURATIONS,
    PHASE_DURATIONS, SimultaneousArmCadencedController)
from source_multistep_probe import loaded_settle_ready
from source_step_probe import Controller, clock
from source_upright_large_steps import LargeStep


class LoadedSettleReadiness(unittest.TestCase):
    def setUp(self):
        self.mass = 40.
        self.controller = SimpleNamespace(finalCom=np.array([0., 0., .8]))
        self.row = dict(
            com=[0., 0., .8], comVelocity=[0., 0., 0.],
            feet=[dict(loadN=200., clearance=0.), dict(loadN=200., clearance=0.)],
            minimumComCaptureMarginM=.02, jointLimitMinimumMarginRad=.02,
            maximumBodyLinearMps=.02, maximumBodyAngularRadps=.1)

    def ready(self, row=None):
        return bool(loaded_settle_ready(self.row if row is None else row,
                                       self.controller, self.mass))

    def test_nominal_loaded_posture_is_ready_without_quiet_fields(self):
        self.assertTrue(self.ready())
        self.assertNotIn('quietConsecutiveS', self.row)
        self.assertNotIn('phase', self.row)

    def test_each_foot_load_includes_three_newtons_and_rejects_less(self):
        for side in (0, 1):
            with self.subTest(side=side):
                row = copy.deepcopy(self.row)
                row['feet'][side]['loadN'] = 3.
                row['feet'][1-side]['loadN'] = 400.
                self.assertTrue(self.ready(row))
                row['feet'][side]['loadN'] = np.nextafter(3., -math.inf)
                self.assertFalse(self.ready(row))

    def test_total_load_includes_eighty_percent_weight_and_rejects_less(self):
        threshold = .8*self.mass*9.81
        row = copy.deepcopy(self.row)
        for foot in row['feet']:
            foot['loadN'] = threshold/2.
        self.assertTrue(self.ready(row))
        for foot in row['feet']:
            foot['loadN'] = (threshold-1e-9)/2.
        self.assertFalse(self.ready(row))

    def test_each_foot_clearance_includes_three_mm_and_rejects_more(self):
        for side in (0, 1):
            with self.subTest(side=side):
                row = copy.deepcopy(self.row)
                row['feet'][side]['clearance'] = .003
                self.assertTrue(self.ready(row))
                row['feet'][side]['clearance'] = np.nextafter(.003, math.inf)
                self.assertFalse(self.ready(row))

    def test_capture_margin_requires_defined_ten_mm_or_more(self):
        for margin, expected in ((None, False), (-.001, False),
                                  (np.nextafter(.01, -math.inf), False),
                                  (.01, True), (.02, True)):
            with self.subTest(margin=margin):
                row = copy.deepcopy(self.row)
                row['minimumComCaptureMarginM'] = margin
                self.assertEqual(self.ready(row), expected)

    def test_com_target_error_is_strictly_less_than_eight_mm(self):
        for error, expected in ((.007999, True), (.008, False), (.008001, False)):
            for axis in (0, 1):
                with self.subTest(error=error, axis=axis):
                    row = copy.deepcopy(self.row)
                    row['com'][axis] = error
                    self.assertEqual(self.ready(row), expected)

    def test_capture_target_error_is_strictly_less_than_ten_mm(self):
        tau = math.sqrt(.8/9.81)
        for error, expected in ((.009999, True), (.01, False), (.010001, False)):
            for axis in (0, 1):
                with self.subTest(error=error, axis=axis):
                    row = copy.deepcopy(self.row)
                    row['comVelocity'][axis] = error/tau
                    self.assertEqual(self.ready(row), expected)

    def test_joint_reserve_includes_five_milliradians(self):
        self.row['jointLimitMinimumMarginRad'] = .005
        self.assertTrue(self.ready())
        self.row['jointLimitMinimumMarginRad'] = np.nextafter(.005, -math.inf)
        self.assertFalse(self.ready())

    def test_motion_thresholds_are_inclusive_and_both_are_required(self):
        for field, limit in (('maximumBodyLinearMps', .05),
                              ('maximumBodyAngularRadps', .2)):
            with self.subTest(field=field):
                row = copy.deepcopy(self.row)
                row[field] = limit
                self.assertTrue(self.ready(row))
                row[field] = np.nextafter(limit, math.inf)
                self.assertFalse(self.ready(row))

    def test_readiness_does_not_modify_row_or_reference(self):
        row_before = copy.deepcopy(self.row)
        reference_before = self.controller.finalCom.copy()
        self.ready()
        self.assertEqual(self.row, row_before)
        np.testing.assert_array_equal(self.controller.finalCom, reference_before)


class LoadedSettleReferenceBoundary(unittest.TestCase):
    def boundary(self, *, settled=True, implicit=False, first=False):
        old = SimpleNamespace(
            com1=np.array([.11, .12, .79]), finalCom=np.array([.21, .22, .8]),
            rootR=Rotation.from_euler('ZYX', [.4, -.02, .03]).as_matrix(),
            lean_axis=np.array([.1, -.02, .01]), q0=np.arange(21)*.001,
            previous_time=7.135, previous_leg_rate=np.array([.02, -.03]))
        new_root = Rotation.from_euler('ZYX', [.41, .015, -.01]).as_matrix()
        new_lean = np.array([-.12, .03, -.01])
        reference_end = Rotation.from_rotvec(new_lean).as_matrix()@new_root
        live = CadencedSteps.__new__(CadencedSteps)
        live.step_index = 0 if first else 1
        live.controller = old
        live.loaded_settle_handoff = settled
        live.continuous_loaded_handoff = not settled
        live.implicit_arms = implicit
        live.step_records = []
        live.slip = np.array([.004, .005])
        live.loaded = np.array([2100, 145])
        live.steps = 7135
        live.total = 7135
        live.max_motor = .93
        native_sentinel = object()
        live.world = native_sentinel
        def parent_begin(instance):
            c = Controller.__new__(Controller)
            c.rootR, c.lean_axis = new_root.copy(), new_lean.copy()
            c.com0, c.com1 = np.array([.101, .199, .791]), np.array([.3, .2, .8])
            c.q0, c.posture_start = np.zeros(21), np.ones(21)
            c.previous_time, c.previous_leg_rate = None, None
            instance.controller = c
            instance.step_records.append({})
        with tempfile.TemporaryDirectory(prefix='loaded-settle-boundary-') as folder:
            live.folder = Path(folder)
            with patch.object(LargeStep, 'begin_step', parent_begin):
                live.begin_step()
            record = json.loads((live.folder/f'controller-step-{live.step_index+1:02d}.json').read_text())
        return live, old, reference_end, native_sentinel, record

    def test_settled_boundary_carries_final_com_and_zero_lean_root(self):
        live, old, expected_end, _, record = self.boundary()
        c = live.controller
        np.testing.assert_array_equal(c.com0, old.finalCom)
        self.assertIsNot(c.com0, old.finalCom)
        np.testing.assert_array_equal(c.rootR, old.rootR)
        self.assertIsNot(c.rootR, old.rootR)
        np.testing.assert_allclose(Rotation.from_rotvec(c.lean_axis).as_matrix()@c.rootR,
                                   expected_end, atol=1e-15)
        np.testing.assert_array_equal(record['com0'], old.finalCom)

    def test_direct_boundary_retains_prior_lean_and_transfer_com_target(self):
        live, old, expected_end, _, _ = self.boundary(settled=False)
        c = live.controller
        np.testing.assert_array_equal(c.com0, old.com1)
        np.testing.assert_allclose(c.rootR,
            Rotation.from_rotvec(old.lean_axis).as_matrix()@old.rootR, atol=1e-15)
        np.testing.assert_allclose(Rotation.from_rotvec(c.lean_axis).as_matrix()@c.rootR,
                                   expected_end, atol=1e-15)

    def test_posture_and_measured_history_carry_by_copy(self):
        live, old, _, _, _ = self.boundary()
        np.testing.assert_array_equal(live.controller.posture_start, old.q0)
        self.assertIsNot(live.controller.posture_start, old.q0)
        self.assertEqual(live.controller.previous_time, old.previous_time)
        np.testing.assert_array_equal(live.controller.previous_leg_rate, old.previous_leg_rate)
        self.assertIsNot(live.controller.previous_leg_rate, old.previous_leg_rate)

    def test_boundary_does_not_reset_global_metrics_or_replace_world(self):
        live, _, _, native_sentinel, _ = self.boundary()
        np.testing.assert_array_equal(live.slip, [.004, .005])
        np.testing.assert_array_equal(live.loaded, [2100, 145])
        self.assertEqual((live.steps, live.total, live.max_motor), (7135, 7135, .93))
        self.assertIs(live.world, native_sentinel)

    def test_loaded_settle_and_direct_durations_remain_separate(self):
        for settled, expected in ((True, PHASE_DURATIONS), (False, DIRECT_PHASE_DURATIONS)):
            with self.subTest(settled=settled):
                live, _, _, _, _ = self.boundary(settled=settled)
                self.assertEqual(live.controller.phase_durations, expected)
                self.assertIsNot(live.controller.phase_durations, expected)
                self.assertEqual(live.controller.arm_placement_duration, expected['transfer'])
                self.assertEqual(live.controller.loaded_settle_capture_barrier, settled)

    def test_first_transfer_still_uses_four_physical_seconds(self):
        for settled in (False, True):
            with self.subTest(settled=settled):
                live, _, _, _, _ = self.boundary(settled=settled, first=True)
                self.assertEqual(live.controller.phase_duration('transfer'), 4.)
                self.assertEqual(live.controller.arm_placement_duration, 4.)

    def test_implicit_arms_remain_an_explicit_separate_class(self):
        signature = inspect.signature(CadencedSteps.__init__)
        self.assertEqual(signature.parameters['handoff'].default, 'loaded-settle')
        self.assertIs(signature.parameters['implicit_arms'].default, False)
        legacy, _, _, _, _ = self.boundary(implicit=False)
        experimental, _, _, _, _ = self.boundary(implicit=True)
        self.assertIs(type(legacy.controller), CadencedController)
        self.assertFalse(hasattr(legacy.controller, 'arm_acceleration_task'))
        self.assertIs(type(experimental.controller), SimultaneousArmCadencedController)
        self.assertTrue(hasattr(experimental.controller, 'arm_acceleration_task'))

    def test_minimum_jerk_endpoint_has_zero_reference_velocity_and_acceleration(self):
        # The previous settle has ended when the next transfer starts, so the
        # carried COM/root endpoints have no reference velocity discontinuity.
        settle = PHASE_DURATIONS['settle']
        transfer = PHASE_DURATIONS['transfer']
        self.assertEqual(tuple(float(x) for x in clock(settle, settle)), (1., 0., 0.))
        self.assertEqual(tuple(float(x) for x in clock(0., transfer)), (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
