"""Pure polynomial/extremum tests. Dense sampling is a test oracle only."""
from fractions import Fraction as F
import json
import math
import unittest

import numpy as np

from source_causal_com_overlap import (MAX_TOTAL_SPEED_MPS, MAX_TOTAL_ACCEL_MPS2,
    _jets, _mul, _stationary_intervals, certify_overlap)


class ExactPiecewiseEnvelope(unittest.TestCase):
    def test_known_nominal_quintic_maxima_and_critical_times(self):
        result = certify_overlap([.1, 0., 0.], 2., [0., 0., 0.], 3., 1.)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['maximumTotalSpeedMps'], 1.875*.1/2., delta=1e-15)
        self.assertAlmostEqual(result['maximumTotalAccelerationMps2'], 10.*math.sqrt(3.)/3.*.1/4., delta=1e-15)
        self.assertIn(1., result['candidateCriticalTimesS']['speed'])
        for expected in (1.-1./math.sqrt(3.), 1.+1./math.sqrt(3.)):
            self.assertTrue(any(abs(t-expected)<1e-14 for t in result['candidateCriticalTimesS']['acceleration']))
        self.assertEqual(result['caps']['totalSpeedMps'], MAX_TOTAL_SPEED_MPS)
        self.assertEqual(result['caps']['totalAccelerationMps2'], MAX_TOTAL_ACCEL_MPS2)

    def test_no_overlap_is_maximum_of_independent_jets_and_wait_is_zero(self):
        result = certify_overlap([.1, 0., 0.], 2., [0., .01, 0.], 3., 1.)
        self.assertAlmostEqual(result['maximumTotalSpeedMps'], .09375, delta=1e-15)
        self.assertAlmostEqual(result['maximumTotalAccelerationMps2'], math.sqrt(3.)/12., delta=1e-15)
        self.assertEqual([(w['startS'],w['endS']) for w in result['windows']], [(0.,2.),(2.,3.),(3.,4.)])
        wait = result['windows'][1]
        self.assertFalse(wait['nominalActive']); self.assertFalse(wait['correctionActive'])
        self.assertEqual(wait['speedUpperBound'], 0.)
        self.assertEqual(wait['accelerationUpperBound'], 0.)

    def test_zero_and_tiny_vectors_are_finite_and_do_not_lose_extrema(self):
        zero = certify_overlap([0.,0.,0.], 2., [0.,0.,0.], 0., 1.)
        self.assertTrue(zero['passed'])
        self.assertEqual(zero['maximumTotalSpeedMps'], 0.)
        self.assertEqual(zero['maximumTotalAccelerationMps2'], 0.)
        tiny = certify_overlap([1e-200,0.,0.], 2., [0.,0.,0.], 0., 1.)
        self.assertGreater(tiny['maximumTotalSpeedMps'], 0.)
        self.assertGreaterEqual(tiny['conservativeMaximumTotalSpeedMps'], tiny['maximumTotalSpeedMps'])
        self.assertAlmostEqual(tiny['maximumTotalSpeedMps']/1e-200, .9375, delta=1e-15)
        self.assertTrue(tiny['passed'])
        json.dumps(tiny, allow_nan=False)

    def test_exact_opposed_simultaneous_jets_cancel(self):
        result = certify_overlap([.01,.002,0.], 1., [-.01,-.002,0.], 0., 1.)
        self.assertEqual(result['maximumTotalSpeedMps'], 0.)
        self.assertEqual(result['maximumTotalAccelerationMps2'], 0.)
        self.assertTrue(result['passed'])

    def test_vector_directions_and_orthogonal_rotation_preserve_norms(self):
        nominal, correction = np.array([.09,.03,-.02]), np.array([.003,-.004,.002])
        R = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        a = certify_overlap(nominal, 2., correction, .25, 1.1)
        b = certify_overlap(R@nominal, 2., R@correction, .25, 1.1)
        self.assertEqual(a['passed'], b['passed'])
        for key in ('maximumTotalSpeedMps', 'maximumTotalAccelerationMps2'):
            self.assertAlmostEqual(a[key], b[key], delta=1e-15)
        orthogonal = certify_overlap([.1,0.,0.], 2., [0.,.01,0.], .5, 1.)
        aligned = certify_overlap([.1,0.,0.], 2., [.01,0.,0.], .5, 1.)
        self.assertLess(orthogonal['maximumTotalSpeedMps'], aligned['maximumTotalSpeedMps'])

    def test_analytic_candidates_dominate_dense_test_only_oracle(self):
        cases = [([.09,.03,0.], 2., [.007,-.003,0.], .4, 1.2),
                 ([.12,0.,.02], 2.3, [0.,.012,0.], .75, 1.),
                 ([.04,-.03,.01], 1.7, [-.006,.004,.002], .2, 1.3)]
        def vectorized_jet(delta, duration, start, times):
            u = np.clip((times-start)/duration, 0., 1.)
            speed = 30*u*u*(1-u)**2/duration
            acceleration = 60*u*(1-u)*(1-2*u)/duration**2
            return np.outer(speed, delta), np.outer(acceleration, delta)
        for args in cases:
            with self.subTest(args=args):
                result = certify_overlap(*args)
                times = np.linspace(0., max(args[1],args[3]+args[4]), 40001)
                nv, na = vectorized_jet(args[0],args[1],0.,times)
                cv, ca = vectorized_jet(args[2],args[4],args[3],times)
                for label, values in (('SpeedMps',nv+cv), ('AccelerationMps2',na+ca)):
                    dense = np.linalg.norm(values,axis=1).max()
                    exact = result['maximumTotal'+label]
                    self.assertGreaterEqual(exact+2e-15, dense)
                    self.assertLess(exact-dense, 2e-8)
                    self.assertGreaterEqual(result['conservativeMaximumTotal'+label]+2e-15,exact)
                self.assertTrue(all(w['speedSquaredDerivativeDegree'] <= 7 and
                    w['accelerationSquaredDerivativeDegree'] <= 5 for w in result['windows']))

    def test_sturm_root_isolation_covers_repeated_midpoint_and_endpoint_roots(self):
        # q' = x*(x-1)*(x-.25)*(x-.5)^2*(x-.75)^2. All interior distinct
        # roots must appear once even when a dyadic root coincides with a split.
        derivative = [F(1)]
        for root in (F(0),F(1),F(1,4),F(1,2),F(1,2),F(3,4),F(3,4)):
            derivative = _mul(derivative,[-root,F(1)])
        polynomial = [F(0)]+[x/F(i+1) for i,x in enumerate(derivative)]
        intervals = _stationary_intervals(polynomial)
        self.assertEqual(len(intervals),3)
        for expected, (lo,hi) in zip((F(1,4),F(1,2),F(3,4)),intervals):
            self.assertLessEqual(lo,expected); self.assertGreaterEqual(hi,expected)

    def test_coincident_intervals_equal_one_summed_quintic(self):
        result = certify_overlap([.1,.02,0.],2.,[-.005,.004,0.],0.,2.)
        distance = math.hypot(.095,.024)
        self.assertAlmostEqual(result['maximumTotalSpeedMps'],1.875*distance/2.,delta=1e-15)
        self.assertAlmostEqual(result['maximumTotalAccelerationMps2'],10.*math.sqrt(3.)/3.*distance/4.,delta=1e-15)
        self.assertEqual(len(result['windows']),1)

    def test_exact_cap_passes_and_one_ulp_over_cap_rejects(self):
        # T=15/8 cancels the exact speed multiplier, giving vmax=delta.
        result = certify_overlap([MAX_TOTAL_SPEED_MPS,0.,0.],1.875,[0.,0.,0.],0.,1.)
        self.assertTrue(result['passed'])
        self.assertEqual(result['conservativeMaximumTotalSpeedMps'],MAX_TOTAL_SPEED_MPS)
        over = certify_overlap([math.nextafter(MAX_TOTAL_SPEED_MPS,math.inf),0.,0.],
                               1.875,[0.,0.,0.],0.,1.)
        self.assertFalse(over['passed']); self.assertIn('totalSpeed',over['rejections'])
        self.assertEqual(over['unresolvedCapBounds'],[])

    def test_sturm_close_roots_and_near_real_complex_pair(self):
        derivative = [F(1)]
        roots = (F(1,2)-F(1,2**80),F(1,2)+F(1,2**80))
        for root in roots:
            derivative = _mul(derivative,[-root,F(1)])
        polynomial = [F(0)]+[x/F(i+1) for i,x in enumerate(derivative)]
        intervals = _stationary_intervals(polynomial)
        self.assertEqual(len(intervals),2)
        for root,(lo,hi) in zip(roots,intervals):
            self.assertLessEqual(lo,root); self.assertGreaterEqual(hi,root)
        # (x-.5)^2+epsilon has a near-real complex pair, not stationary times.
        derivative = _mul([F(-1,4),F(1)],
                          [F(1,4)+F(1,2**160),F(-1),F(1)])
        polynomial = [F(0)]+[x/F(i+1) for i,x in enumerate(derivative)]
        self.assertEqual(_stationary_intervals(polynomial),[(F(1,4),F(1,4))])

    def test_envelope_rejection_is_explicit_without_relaxing_caps(self):
        too_fast = certify_overlap([.3,0.,0.], 2., [0.,0.,0.], 0., 1.)
        self.assertFalse(too_fast['passed']); self.assertIn('totalSpeed',too_fast['rejections'])
        too_far = certify_overlap([0.,0.,0.], 2., [.01201,0.,0.], 0., 1.)
        self.assertFalse(too_far['passed']); self.assertIn('correctionMagnitude',too_far['rejections'])
        too_short = certify_overlap([0.,0.,0.], 2., [.012,0.,0.], 0., .1)
        self.assertFalse(too_short['passed'])
        self.assertTrue({'correctionDuration','correctionSpeed','correctionAcceleration'}.issubset(too_short['rejections']))

    def test_c2_waits_hold_position_and_zero_derivatives(self):
        delta = [.01,-.002,.001]
        for t, position in ((-.1,[0.,0.,0.]),(.25,[0.,0.,0.]),(1.25,delta),(2.,delta)):
            p,v,a = _jets(delta,1.,.25,t)
            np.testing.assert_array_equal(p,position)
            np.testing.assert_array_equal(v,np.zeros(3)); np.testing.assert_array_equal(a,np.zeros(3))
        for t in (.25+1e-10,1.25-1e-10):
            _,v,a = _jets(delta,1.,.25,t)
            self.assertLess(math.hypot(*v),1e-17); self.assertLess(math.hypot(*a),1e-9)
        result = certify_overlap([.1,0.,0.],2.,delta,3.,1.)
        np.testing.assert_array_equal(result['boundaryJets'][-1]['positionM'],np.array([.1,0.,0.])+delta)

    def test_malformed_nonfinite_and_unrepresentable_inputs_are_rejected(self):
        good = [[.1,0.,0.],2.,[.01,0.,0.],.5,1.]
        bad = [(0,None),(0,[1.,2.]),(0,[True,0.,0.]),(0,['.1',0.,0.]),
            (0,[np.nan,0.,0.]),(0,[np.inf,0.,0.]),(1,0.),(1,-1.),(1,np.nan),
            (1,True),(2,[0.,0.,np.inf]),(3,-.1),(3,np.inf),(4,0.),(4,-1.),(4,np.nan)]
        for slot,value in bad:
            with self.subTest(slot=slot,value=value), self.assertRaises(ValueError):
                args = good.copy(); args[slot]=value; certify_overlap(*args)
        with self.assertRaises(ValueError):
            certify_overlap([0.,0.,0.],2.,[0.,0.,0.],1e308,1.)

    def test_certificate_is_owned_finite_json_and_deterministic(self):
        nominal, correction = [.1,0.,0.],[.01,0.,0.]
        result = certify_overlap(nominal,2.,correction,.5,1.)
        self.assertEqual(result,certify_overlap(nominal,2.,correction,.5,1.))
        rebuilt = json.loads(json.dumps(result,allow_nan=False))
        self.assertEqual(rebuilt,result)
        nominal[0]=999.; correction[0]=999.
        self.assertEqual(result['nominal']['deltaM'][0],.1)
        self.assertEqual(result['correction']['deltaM'][0],.01)
        self.assertTrue(result['passed'])


if __name__ == '__main__':
    unittest.main()
