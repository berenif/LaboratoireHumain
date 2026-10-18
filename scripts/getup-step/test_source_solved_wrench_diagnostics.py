"""Solved-wrench indexing/stage/purity tests; no native physics integration."""
import copy
import ctypes as C
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from source_arm_knee_mobility import native_bytes
from source_selected_step import load_preset
from source_step_probe import World, ROOT
import source_solved_wrench_diagnostics as diagnostics


class FakeWorld:
    h = None
    def __init__(self):
        self.step, self.now = 80, .1
        self.control = np.linspace(-.2, .2, 21)
        self.wrench = np.array([[3., -4., 10., -1., 2., -3.],
                                [-12., 5., 20., 4., -5., 6.]])
        self.contact = [7., 0., 1., 3., 1., 1., .005, .0001, 0.]
        self.l = SimpleNamespace(lh_dim=lambda h,k:self.step,
            lh_time=lambda h:self.now, lh_dt=lambda h:.00125,
            lh_last_ground_wrench=lambda *args:None)
    def arr(self, key, shape):
        assert key == 2 and shape == (21,)
        return self.control
    def read(self, name, shape):
        assert name == 'lh_last_ground_wrench' and shape == (2,6)
        return self.wrench.copy()
    def friction_parameters(self):
        return dict(effectiveFootGroundContacts=[self.contact.copy()])


def row_for(w):
    return dict(sourceStep=w.step, sourceTimeS=w.now, phase='transfer', stepIndex=2,
        feet=[dict(loadN=10., loadedSlipSpeedMps=.002),
              dict(loadN=20., loadedSlipSpeedMps=.004)])


class SolvedWrenchMath(unittest.TestCase):
    def setUp(self):
        self.w = FakeWorld(); self.row = row_for(self.w)
        self.binding = patch.object(diagnostics, '_bindings', return_value=dict(helperSHA256='fake-test-only'))
        self.binding.start(); self.addCleanup(self.binding.stop)

    def test_left_right_sign_order_and_shear_are_preserved(self):
        before = copy.deepcopy(self.row); control = self.w.control.copy()
        result = diagnostics.reader(self.w, self.row)
        self.assertEqual([f['side'] for f in result['feet']], ['left', 'right'])
        for index in (0,1):
            np.testing.assert_array_equal(result['feet'][index]['forceWorldSourceN'], self.w.wrench[index,:3])
            np.testing.assert_array_equal(result['feet'][index]['momentWorldSourceNmAboutSolvedFootOrigin'], self.w.wrench[index,3:])
        self.assertEqual(result['feet'][0]['netShearForceN'], 5.)
        self.assertEqual(result['feet'][1]['netShearForceN'], 13.)
        self.assertEqual(result['feet'][0]['netShearUtilizationLowerBound'], .5)
        self.assertEqual(result['feet'][1]['netShearUtilizationLowerBound'], .65)
        self.assertEqual(self.row, before)
        np.testing.assert_array_equal(self.w.control, control)
        self.assertEqual((self.w.step,self.w.now), (80,.1))
        self.assertEqual(result['heldMotorControlSHA256'],
                         hashlib.sha256(np.asarray(control,dtype='<f8').tobytes(order='C')).hexdigest())
        json.dumps(result, allow_nan=False)

    def test_held_control_hash_is_little_endian_and_changes_with_held_command(self):
        self.w.control=self.w.control.astype('>f8')
        result=diagnostics.reader(self.w,self.row)
        expected=hashlib.sha256(self.w.control.astype('<f8').tobytes(order='C')).hexdigest()
        self.assertEqual(result['heldMotorControlSHA256'],expected)
        self.w.control[3]+=.001
        self.assertNotEqual(diagnostics.reader(self.w,self.row)['heldMotorControlSHA256'],expected)

    def test_force_and_geometry_times_and_transported_slip_are_separate(self):
        result = diagnostics.reader(self.w, self.row)
        self.assertAlmostEqual(result['solvedStageSourceTimeS'], .09875)
        self.assertEqual(result['solvedStageSourceStepBeforeIntegration'], 79)
        self.assertEqual(result['transportedGeometrySourceTimeS'], .1)
        self.assertAlmostEqual(result['feet'][0]['transportedLoadedSlipIncrementM'], .002*.00125)
        self.assertFalse(result['individualContactNonsaturationEstablished'])
        self.assertIn('lower bound', result['utilizationMeaning'])
        self.assertIn('cancellation', result['utilizationMeaning'])
        self.assertIn('solved-stage foot body origin', result['momentDefinition'])

    def test_utilization_is_undefined_below_three_newtons(self):
        for load in (0., 2.999999):
            self.w.wrench[0,:3] = [1., 0., load]; self.row['feet'][0]['loadN'] = load
            result = diagnostics.reader(self.w, self.row)['feet'][0]
            self.assertIsNone(result['netShearUtilizationLowerBound'])
            self.assertFalse(result['loadedForUtilization'])
        self.w.wrench[0,2] = self.row['feet'][0]['loadN'] = 3.
        self.assertAlmostEqual(diagnostics.reader(self.w,self.row)['feet'][0]['netShearUtilizationLowerBound'], 1./3.)

    def test_zero_step_reports_no_solved_sample(self):
        self.w.step,self.w.now = 0,0.; self.w.wrench[:] = 0.
        self.row = row_for(self.w)
        for foot in self.row['feet']: foot['loadN'] = foot['loadedSlipSpeedMps'] = 0.
        result = diagnostics.reader(self.w,self.row)
        self.assertFalse(result['solvedWrenchAvailable'])
        self.assertIsNone(result['solvedStageSourceTimeS'])
        self.assertIsNone(result['solvedStageSourceStepBeforeIntegration'])
        for foot in result['feet']:
            self.assertIsNone(foot['netShearUtilizationLowerBound'])
            self.assertEqual(foot['transportedLoadedSlipIncrementM'], 0.)

    def test_normal_force_mismatch_and_wrong_side_are_rejected(self):
        self.w.wrench[0,2] += 1.1e-8
        with self.assertRaisesRegex(ValueError,'left.*Fz'): diagnostics.reader(self.w,self.row)
        self.w.wrench = self.w.wrench[::-1].copy()
        with self.assertRaisesRegex(ValueError,'Fz'): diagnostics.reader(self.w,self.row)

    def test_nonfinite_wrench_load_slip_and_time_are_rejected(self):
        for component in range(6):
            w=FakeWorld(); w.wrench[1,component]=np.nan
            with self.subTest(component=component), self.assertRaisesRegex(ValueError,'finite'):
                diagnostics.reader(w,row_for(w))
        for key in ('loadN','loadedSlipSpeedMps'):
            row=copy.deepcopy(self.row);row['feet'][0][key]=float('inf')
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'nonfinite'):
                diagnostics.reader(self.w,row)
        row=copy.deepcopy(self.row);row['sourceTimeS']=float('nan')
        with self.assertRaisesRegex(ValueError,'nonfinite'):diagnostics.reader(self.w,row)

    def test_native_identity_and_changed_friction_are_rejected(self):
        row=copy.deepcopy(self.row);row['sourceStep']+=1
        with self.assertRaisesRegex(ValueError,'current native'):diagnostics.reader(self.w,row)
        self.w.contact[4]=.95
        with self.assertRaisesRegex(ValueError,'mu=1'):diagnostics.reader(self.w,self.row)

    def test_control_mutation_by_an_accessor_is_detected(self):
        original=self.w.read
        def changed(*args):
            value=original(*args);self.w.control[0]+=.1;return value
        self.w.read=changed
        with self.assertRaisesRegex(ValueError,'mutated native'):diagnostics.reader(self.w,self.row)

    def test_no_nonsaturation_claim_even_for_tiny_net_shear(self):
        self.w.wrench[:,:2]=0.
        result=diagnostics.reader(self.w,self.row)
        self.assertEqual(result['feet'][0]['netShearUtilizationLowerBound'],0.)
        self.assertFalse(result['individualContactNonsaturationEstablished'])

    def test_finite_inputs_that_overflow_net_metric_are_rejected(self):
        self.w.wrench[0,:2]=np.finfo(float).max
        with self.assertRaisesRegex(ValueError,'nonfinite.*net shear'):
            diagnostics.reader(self.w,self.row)

    def test_positive_and_negative_force_signs_are_not_folded_into_magnitudes(self):
        self.w.wrench[0,2]=-10.
        with self.assertRaisesRegex(ValueError,'left.*Fz'):
            diagnostics.reader(self.w,self.row)

    def test_wrench_array_shape_is_validated(self):
        self.w.wrench=np.zeros((12,))
        with self.assertRaisesRegex(ValueError,'left/right solved wrench'):
            diagnostics.reader(self.w,self.row)

    def test_native_timestep_mismatch_cannot_mislabel_the_solved_stage(self):
        self.w.l.lh_dt=lambda h:.0025
        with self.assertRaisesRegex(ValueError,'timestep'):
            diagnostics.reader(self.w,self.row)


PRESET=load_preset();HELPER=ROOT/PRESET['helper']
BASE=Path('/workspace/shared/cadence-robust-8-ten/model-session')
if not BASE.is_dir():BASE=ROOT/PRESET['sourceSession']


@unittest.skipUnless(HELPER.is_file()and(BASE/'source-model/humanoid_high_freq.xml').is_file(),
                     'pinned native assets absent')
class NativeSolvedWrenchPurity(unittest.TestCase):
    def test_zero_step_reader_preserves_exact_native_bytes_counter_time_and_control(self):
        w=World(BASE,HELPER)
        try:
            self.assertEqual(w.l.lh_dim(w.h,6),0)
            # A newly opened world has no solved sample. Do not restore/import
            # a cache or install a state just to manufacture this zero-step test.
            row=dict(sourceStep=w.l.lh_dim(w.h,6),sourceTimeS=w.l.lh_time(w.h),
                feet=[dict(loadN=float(w.arr(20,(17,))[w.ids[side+'_foot']]),
                           loadedSlipSpeedMps=w.l.lh_foot_slip_speed(w.h,index))
                      for index,side in enumerate(('left','right'))])
            before=native_bytes(w);held=w.arr(2,(21,)).copy()
            inspector_path=Path('/workspace/shared/cadence-robust-8-ten/contact-inspector.so')
            inspector=None;fingerprint=None
            if inspector_path.is_file():
                inspector=C.CDLL(str(inspector_path))
                inspector.lh_ro_model_fingerprint.argtypes=[C.c_void_p]
                inspector.lh_ro_model_fingerprint.restype=C.c_uint64
                fingerprint=inspector.lh_ro_model_fingerprint(w.h)
            def forbidden(*args):raise AssertionError('native step/state install forbidden')
            for name in ('lh_step','lh_step_audited','lh_set_state','lh_checkpoint_import'):
                if hasattr(w.l,name):setattr(w.l,name,forbidden)
            first=diagnostics.reader(w,row);second=diagnostics.reader(w,row)
            self.assertEqual(first,second)
            self.assertFalse(first['solvedWrenchAvailable'])
            self.assertEqual(first['bindings']['helperSHA256'],PRESET['helperSHA256'])
            self.assertEqual(native_bytes(w),before)
            self.assertEqual(w.l.lh_dim(w.h,6),0)
            self.assertEqual(w.l.lh_time(w.h),row['sourceTimeS'])
            np.testing.assert_array_equal(w.arr(2,(21,)),held)
            if inspector is not None:self.assertEqual(inspector.lh_ro_model_fingerprint(w.h),fingerprint)
            json.dumps(first,allow_nan=False)
        finally:w.close()


if __name__=='__main__':unittest.main()
