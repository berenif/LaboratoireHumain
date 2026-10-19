"""Pure analytical settle tests. No native/model imports or integrations."""
import copy
import json
import math
from pathlib import Path
import unittest
import numpy as np
import source_bounded_com_settle as bounded

ZERO = np.zeros(3)
BROAD = np.array([[-1.,-1.], [1.,-1.], [1.,1.], [-1.,1.]])
SAVED_ANALYSIS = Path('/workspace/shared/reviewed-walk/bounded-com-settle/posture-one-02-analysis.json')


def build(distance=.18192656090102183, initial=None, final=None, points=BROAD, legacy=None):
    initial = (np.array([0.,0.,.8]), ZERO.copy(), ZERO.copy()) if initial is None else initial
    final = (initial[0]+[distance,0.,0.], ZERO.copy(), ZERO.copy()) if final is None else final
    return bounded.build_bounded_settle(10.,1.52,initial,final,points,legacy=legacy)


class AnalyticalFamily(unittest.TestCase):
    def test_saved_distance_uses_seven_cubics_at_unchanged_caps(self):
        path, receipt = build()
        self.assertIsInstance(path,bounded.BoundedSettlePath)
        self.assertEqual(receipt['profile'],'seven-cubic-constant-jerk')
        self.assertEqual(len(path.pieces),7)
        shape=receipt['analyticalShape']
        self.assertAlmostEqual(shape['rampTimeS'],.08181458277944376,places=14)
        self.assertAlmostEqual(shape['coastTimeS'],.36911052780838527,places=14)
        self.assertEqual(shape['peakSpeedMps'],bounded.MAX_SPEED)
        self.assertEqual(shape['peakAccelerationMps2'],bounded.MAX_ACCELERATION)
        self.assertEqual(path.duration,1.52)
        self.assertEqual(receipt['physicsIntegrations'],0)
        self.assertEqual(receipt['optimizerCalls'],0)

    def test_every_join_has_continuous_position_velocity_acceleration(self):
        path,_=build()
        for left,right in zip(path.pieces,path.pieces[1:]):
            for a,b in zip(bounded._jet(left.coefficients,left.duration),bounded._jet(right.coefficients,0.)):
                np.testing.assert_allclose(a,b,atol=5e-16,rtol=0.)

    def test_exact_endpoint_positions_and_zero_jets_and_waits(self):
        path,_=build(); final=np.array([.18192656090102183,0.,.8])
        for t in (-20.,10.):
            for actual,expected in zip(path.evaluate(t),([0.,0.,.8],ZERO,ZERO)):
                np.testing.assert_array_equal(actual,expected)
        for t in (11.52,12.,200.):
            for actual,expected in zip(path.evaluate(t),(final,ZERO,ZERO)):
                np.testing.assert_array_equal(actual,expected)

    def test_piece_durations_and_jerks_integrate_to_exact_distance(self):
        path,r=build();shape=r['analyticalShape'];J=shape['peakJerkMps3']
        np.testing.assert_array_equal(shape['scalarJerksMps3'],[J,0.,-J,0.,-J,0.,J])
        self.assertAlmostEqual(sum(shape['pieceDurationsS']),path.duration,places=15)
        self.assertAlmostEqual(shape['integratedFinalScalarPositionM'],shape['distanceM'],places=15)
        self.assertLess(abs(shape['integratedFinalScalarVelocityMps']),1e-15)
        self.assertLess(abs(shape['integratedFinalScalarAccelerationMps2']),1e-15)

    def test_all_piece_speed_and_acceleration_extrema_obey_frozen_total_norms(self):
        path,r=build()
        for piece in r['certificate']['pieces']:
            self.assertLessEqual(piece['maximumSpeedMps'],bounded.MAX_SPEED+1e-15)
            self.assertLessEqual(piece['maximumAccelerationMps2'],bounded.MAX_ACCELERATION+1e-15)
            self.assertTrue(piece['speedExtremaTimesS'])
            self.assertTrue(piece['accelerationExtremaTimesS'])
        self.assertAlmostEqual(r['certificate']['certifiedMaximumSpeedMps'],bounded.MAX_SPEED,places=15)
        self.assertAlmostEqual(r['certificate']['certifiedMaximumAccelerationMps2'],bounded.MAX_ACCELERATION,places=15)

    def test_general_3d_direction_uses_total_norm_not_each_axis(self):
        initial=(np.array([.1,-.1,.8]),ZERO,ZERO)
        final=(initial[0]+np.array([.03,.18,.02]),ZERO,ZERO)
        path,r=build(initial=initial,final=final)
        self.assertAlmostEqual(r['analyticalShape']['distanceM'],np.linalg.norm(final[0]-initial[0]),places=15)
        np.testing.assert_array_equal(path.evaluate(11.52)[0],final[0])
        direction=(final[0]-initial[0])/np.linalg.norm(final[0]-initial[0])
        for piece in path.pieces:
            _,v,a=bounded._jet(piece.coefficients,piece.duration/2)
            self.assertLess(np.linalg.norm(np.cross(v,direction)),1e-15)
            self.assertLess(np.linalg.norm(np.cross(a,direction)),1e-15)

    def test_infeasible_distance_fails_without_duration_target_or_cap_change(self):
        with self.assertRaisesRegex(ValueError,'infeasible'):
            build(.25)

    def test_zero_distance_stays_legacy_stationary(self):
        path,r=build(0.)
        self.assertEqual(r['profile'],'legacy-quintic-exact-identity')
        np.testing.assert_array_equal(path.evaluate(10.76),(np.array([0.,0.,.8]),ZERO,ZERO))

    def test_small_legacy_path_keeps_exact_object_and_evaluator(self):
        initial=(np.array([0.,0.,.8]),ZERO,ZERO);final=(np.array([.1,0.,.8]),ZERO,ZERO)
        legacy=bounded.QuinticPath.between(10.,1.52,initial,final)
        path,r=build(initial=initial,final=final,legacy=legacy)
        self.assertIs(path,legacy)
        self.assertFalse(r['endpointDerivativesCanonicalized'])
        for t in (9.,10.,10.13,10.76,11.519,11.52,20.):
            for a,b in zip(path.evaluate(t),legacy.evaluate(t)):np.testing.assert_array_equal(a,b)
        self.assertEqual(r['path'],legacy.receipt())

    def test_legacy_peak_formula_is_independently_reproduced(self):
        _,r=build()
        D=r['analyticalShape']['distanceM'];T=1.52
        self.assertAlmostEqual(r['legacyMaximumSpeedMps'],1.875*D/T,places=13)
        self.assertAlmostEqual(r['legacyMaximumAccelerationMps2'],10/math.sqrt(3)*D/T**2,places=13)
        self.assertGreater(r['legacyMaximumSpeedMps'],bounded.MAX_SPEED)
        self.assertGreater(r['legacyMaximumAccelerationMps2'],bounded.MAX_ACCELERATION)

    def test_sub_tolerance_endpoint_roundoff_may_be_canonicalized(self):
        initial=(np.array([0.,0.,.8]),np.array([1e-13,0.,0.]),ZERO)
        final=(np.array([.1819,0.,.8]),ZERO,np.array([0.,0.,-1e-13]))
        path,r=build(initial=initial,final=final)
        self.assertTrue(r['endpointDerivativesCanonicalized'])
        np.testing.assert_array_equal(path.evaluate(10.)[1:],(ZERO,ZERO))
        np.testing.assert_array_equal(path.evaluate(11.52)[1:],(ZERO,ZERO))

    def test_nonzero_endpoint_derivatives_are_never_erased(self):
        for boundary in ('initial','final'):
            for order in (1,2):
                initial=[np.array([0.,0.,.8]),ZERO.copy(),ZERO.copy()]
                final=[np.array([.1819,0.,.8]),ZERO.copy(),ZERO.copy()]
                (initial if boundary=='initial' else final)[order][0]=1.01e-12
                with self.subTest(boundary=boundary,order=order),self.assertRaises(ValueError):
                    build(initial=initial,final=final)

    def test_nonfinite_and_invalid_clocks_and_inputs_are_rejected(self):
        for start,T in ((float('nan'),1.52),(10.,0.),(10.,-1.),(10.,float('inf'))):
            with self.subTest(start=start,T=T),self.assertRaises(ValueError):
                bounded.build_bounded_settle(start,T,([0.,0.,.8],ZERO,ZERO),([.18,0.,.8],ZERO,ZERO),BROAD)
        for position in ([0.,.8],[float('nan'),0.,.8],[0.,0.,0.]):
            with self.subTest(position=position),self.assertRaises(ValueError):
                build(initial=(np.array(position),ZERO,ZERO))

    def test_nonfinite_bounded_evaluation_time_is_rejected(self):
        path,_=build()
        for t in (float('nan'),float('inf'),-float('inf')):
            with self.subTest(t=t),self.assertRaises(ValueError):path.evaluate(t)


class CurrentActualHullProof(unittest.TestCase):
    def test_support_requires_three_unique_nondegenerate_actual_points(self):
        for points in ([],[[0.,0.],[1.,0.]],[[0.,0.],[1.,0.],[2.,0.]],[[0.,0.],[1.,0.],[float('nan'),1.]]):
            with self.subTest(points=points),self.assertRaises(ValueError):build(points=points)

    def test_duplicate_or_reordered_actual_points_do_not_change_certificate(self):
        _,a=build();points=np.vstack([BROAD[[2,0,3,1]],BROAD[0]])
        _,b=build(points=points)
        self.assertEqual(a['certificate']['normalizedHullEquations'],b['certificate']['normalizedHullEquations'])
        self.assertEqual(a['certificate']['conservativeMinimumCaptureMarginM'],b['certificate']['conservativeMinimumCaptureMarginM'])

    def test_capture_interior_violation_rejects_even_when_endpoints_are_inside(self):
        points=np.array([[-.1,-1.],[.190,-1.],[.190,1.],[-.1,1.]])
        self.assertGreater(.190-.18192656090102183,.003)
        with self.assertRaisesRegex(ValueError,'actual hull inset'):build(points=points)

    def test_origin_or_target_outside_actual_hull_cannot_be_projected(self):
        points=np.array([[.01,-1.],[.3,-1.],[.3,1.],[.01,1.]])
        with self.assertRaises(ValueError):build(points=points)
        points=np.array([[-.1,-1.],[.18,-1.],[.18,1.],[-.1,1.]])
        with self.assertRaises(ValueError):build(points=points)

    def test_varying_height_capture_uses_conservative_both_tau_polynomials(self):
        initial=(np.array([0.,0.,.82]),ZERO,ZERO);final=(np.array([.175,0.,.795]),ZERO,ZERO)
        path,r=build(initial=initial,final=final)
        self.assertIsInstance(path,bounded.BoundedSettlePath)
        for piece,proof in zip(path.pieces,r['certificate']['pieces']):
            low,high=proof['tauBoundsS'];self.assertLessEqual(low,high)
            self.assertGreater(proof['heightExtremaM']['conservativeMin'],0.)
            for t in (0.,piece.duration/2,piece.duration):
                p,v,_=bounded._jet(piece.coefficients,t);tau=math.sqrt(p[2]/9.81)
                self.assertGreaterEqual(tau,low-1e-15);self.assertLessEqual(tau,high+1e-15)
            for face in proof['faces']:
                self.assertEqual(len(face['captureExtremaAtTauBounds']),2)
                for e in face['captureExtremaAtTauBounds']:
                    self.assertGreaterEqual(e['conservativeMax'],e['max']-1e-14)

    def test_extrema_include_stationary_points_and_boundaries(self):
        e=bounded._extrema([0.,1.,-1.],1.)
        self.assertEqual(e['min'],0.);self.assertAlmostEqual(e['max'],.25,places=15)
        self.assertIn(.5,e['timesS']);self.assertIn(0.,e['timesS']);self.assertIn(1.,e['timesS'])

    def test_outward_bernstein_enclosure_is_independent_of_root_roundoff(self):
        e=bounded._enclosure([0.,1.,-1.],1.)
        self.assertLessEqual(e['conservativeMin'],0.)
        self.assertGreaterEqual(e['conservativeMax'],.25)
        self.assertIn('Bernstein',e['enclosureMethod'])

    def test_declared_hull_and_capture_certificate_never_claim_future_support(self):
        _,r=build();c=r['certificate']
        self.assertFalse(c['futureSupportClaim']);self.assertTrue(c['currentActualSupportOnly'])
        self.assertEqual(c['reserveM'],.003)
        for n in np.array(c['normalizedHullEquations'])[:,:2]:self.assertAlmostEqual(np.linalg.norm(n),1.,places=15)
        self.assertTrue(c['activeCaptureFaceIndices'])


class ReconstructibleReceipts(unittest.TestCase):
    def test_seven_piece_json_roundtrip_is_bitwise_same_evaluator(self):
        path,r=build();restored=bounded.from_receipt(json.loads(json.dumps(r)))
        self.assertEqual(path.receipt(),restored.receipt())
        times=[10.,10.01,10.7,11.51,11.52,100.]+[10.+p.offset for p in path.pieces]
        for t in times:
            for a,b in zip(path.evaluate(t),restored.evaluate(t)):np.testing.assert_array_equal(a,b)

    def test_legacy_json_roundtrip_retains_coefficients_and_jets(self):
        path,r=build(.1);restored=bounded.from_receipt(json.loads(json.dumps(r)))
        self.assertEqual(path.receipt(),restored.receipt())
        for t in (10.,10.1,10.76,11.52):
            for a,b in zip(path.evaluate(t),restored.evaluate(t)):np.testing.assert_array_equal(a,b)

    def test_unknown_profile_version_and_bad_clocks_are_rejected(self):
        _,r=build()
        for key,value in (('version','unknown'),('profile','unknown')):
            bad=copy.deepcopy(r);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):bounded.from_receipt(bad)
        bad=copy.deepcopy(r);bad['path']['startSourceTimeS']=float('nan')
        with self.assertRaises(ValueError):bounded.from_receipt(bad)

    def test_tampered_continuity_endpoints_and_support_are_rejected(self):
        _,r=build()
        bads=[]
        bad=copy.deepcopy(r);bad['path']['pieces'][1]['coefficients'][0][0]+=.01;bads.append(bad)
        bad=copy.deepcopy(r);bad['path']['finalPositionM'][0]+=.01;bads.append(bad)
        bad=copy.deepcopy(r);bad['exactFinalJet'][0][0]+=.01;bads.append(bad)
        bad=copy.deepcopy(r);bad['certificate']['actualSupportPointsXY']=[[0.,0.],[1.,0.],[2.,0.]];bads.append(bad)
        for i,bad in enumerate(bads):
            with self.subTest(case=i),self.assertRaises(ValueError):bounded.from_receipt(bad)

    def test_tampered_duration_or_cap_violating_coefficients_are_rejected(self):
        _,r=build();bad=copy.deepcopy(r);bad['path']['pieces'][0]['durationS']+=.01
        with self.assertRaises(ValueError):bounded.from_receipt(bad)
        bad=copy.deepcopy(r);bad['path']['pieces'][0]['coefficients'][3][0]*=2
        with self.assertRaises(ValueError):bounded.from_receipt(bad)

    @unittest.skipUnless(SAVED_ANALYSIS.is_file(), 'optional archived posture fixture is unavailable')
    def test_saved_posture_fixture_is_reconstructible_without_opening_control_or_native(self):
        data=json.loads(SAVED_ANALYSIS.read_text())
        f=data['fixture'];receipt=data['result'];path=bounded.from_receipt(receipt)
        self.assertEqual(path.duration,1.52)
        self.assertAlmostEqual(np.linalg.norm(np.array(f['finalPositionM'])-f['initialPositionM']),.18192656090102183,places=14)
        self.assertGreater(receipt['certificate']['conservativeMinimumCaptureMarginM'],.025)
        self.assertTrue(receipt['durationUnchanged']);self.assertTrue(receipt['measuredTargetUnchanged'])


if __name__=='__main__':unittest.main()
