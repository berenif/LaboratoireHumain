"""Synthetic saved-data checks; no native worlds and no rendering."""
import copy
from fractions import Fraction
import math
import unittest

import numpy as np

from analyze_source_gait_quality import (DT, Margin, analyze_rows, archived_reference,
                                         separate_support_margins)
from render_measured_upright import gate_verdict, select_source_steps


def row(step=1, index=1, phase='land', phase_tick=80, velocity=0., capture=1., slip=.02):
    side = (index-1) % 2
    return dict(sourceStep=step, sourceTimeS=step*DT, elapsedS=step*DT,
                stepIndex=index, phase=phase, phaseElapsedS=phase_tick*DT,
                com=[0., 0., .8], comVelocity=[velocity, 0., 0.],
                supportPointsXY=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
                minimumComCaptureMarginM=capture, swingSide=side,
                loadedSlipDistanceM=[slip, .01], requestedForwardM=.18,
                swingForwardDisplacementM=.18,
                feet=[dict(position=[.18, 0., 0.], loadN=100., clearance=0.) for _ in range(2)],
                uprightMeasured={body: dict(tiltDeg=value) for body, value in
                    [('pelvis', 24.), ('torso', 2.), ('torsoRelativePelvis', 25.)]})


def fixture():
    rows, references, records = [], {}, []
    for index in range(1, 11):
        references[index] = dict(heading=[1., 0., 0.], targets=[[[0., 0., 0.]], [[0., 0., 0.]]],
                                 phase_durations=dict(transfer=.01, land=.05, settle=.005))
        for phase, ticks in [('transfer', 10), ('land', 80), ('settle', 5)]:
            for tick in range(1, ticks+1):
                rows.append(row(len(rows)+1, index, phase, tick))
            if phase == 'land':
                records.append(dict(index=index, landingNativeStep=len(rows), landingTimeS=len(rows)*DT,
                                    completedNativeStep=len(rows)+5, completedTimeS=999.))
    for tick in range(1, 4001):
        rows.append(row(len(rows)+1, 10, 'stand', tick))
    entry = dict(com=[0., 0., .8], sourceStep=0, sourceTimeS=0.)
    return rows, entry, references, records


class GaitQuality(unittest.TestCase):
    def test_nine_measured_loaded_landing_intervals_not_completion_times(self):
        rows, entry, references, records = fixture()
        result = analyze_rows(rows, entry, references, records)
        self.assertTrue(result['allNineLoadedLandingIntervalsAvailable'])
        self.assertTrue(result['landingSummaryMatchesMeasuredRows'])
        np.testing.assert_allclose(result['loadedLandingIntervalsS'], [95*DT]*9)
        self.assertTrue(result['capture']['allFrameDefinedNonnegativePass'])
        self.assertTrue(result['capture']['allFrame3mmQualityPass'])
        self.assertEqual(result['last4000']['nativeRowRange'], [951, 4950])
        self.assertEqual(result['last4000']['durationS'], 5.)
        self.assertTrue(result['last4000']['allStand'])
        self.assertEqual(result['last4000']['tiltDeg']['pelvis']['rms'], 24.)
        self.assertEqual(result['tiltDegByPhase']['land']['torso']['p95'], 2.)
        self.assertEqual(result['maximumTransfer']['samples'], 10)
        self.assertEqual(result['walkingForwardCOMSpeed']['nearZeroFraction'], 1.)
        self.assertAlmostEqual(result['travel']['sumOwnFootAdvancesM'], 1.8)
        self.assertTrue(result['slip']['within20mmPass'])
        self.assertFalse(any('PhysicalPass' in key for key in result))

    def test_undefined_capture_cannot_pass_even_with_positive_remaining_minimum(self):
        margin = Margin()
        margin.add(.01, row())
        margin.add(None, row(step=2))
        report = margin.report()
        self.assertEqual(report['minimumM'], .01)
        self.assertEqual(report['undefinedSamples'], 1)
        self.assertFalse(report['allFrameDefinedNonnegativePass'])
        self.assertFalse(report['allFrame3mmQualityPass'])

    def test_29514mm_is_nonnegative_but_fails_strict_3mm_quality(self):
        margin = Margin()
        margin.add(.002951397547704987, row())
        self.assertTrue(margin.report()['allFrameDefinedNonnegativePass'])
        self.assertFalse(margin.report()['allFrame3mmQualityPass'])
        self.assertEqual(margin.report()['minimumAt']['sourceStep'], 1)
        self.assertEqual(len(margin.report()['minimumAt']['supportPointsXY']), 4)

    def test_degenerate_support_is_explicitly_undefined(self):
        sample = row()
        sample['supportPointsXY'] = [[0., 0.], [1., 0.]]
        self.assertEqual(separate_support_margins(sample), (None, None))

    def test_com_and_capture_minima_are_separate(self):
        com, capture = separate_support_margins(row(velocity=.1))
        self.assertEqual(com, 1.)
        self.assertAlmostEqual(capture, 1.-.1*math.sqrt(.8/9.81))

    def test_near_zero_uses_strict_absolute_threshold_and_excludes_placement(self):
        rows, entry, references, records = fixture()
        rows = rows[:2]
        for sample, speed in zip(rows, [-.005, -.004999]):
            sample['comVelocity'] = [speed, 0., 0.]
            sample['minimumComCaptureMarginM'] = min(separate_support_margins(sample))
        result = analyze_rows(rows, entry, references, walking_start_s=DT)
        self.assertEqual(result['walkingForwardCOMSpeed']['walkingSamples'], 1)
        self.assertEqual(result['walkingForwardCOMSpeed']['nearZeroSamples'], 1)

    def test_loaded_settle_waiting_is_included_in_walking_window(self):
        rows, entry, references, records = fixture()
        result = analyze_rows(rows, entry, references, records)
        self.assertEqual(result['walkingForwardCOMSpeed']['walkingSamples'],
                         sum(sample['phase'] != 'stand' for sample in rows))
        self.assertGreater(sum(sample['phase'] == 'settle' for sample in rows), 0)

    def test_strict_slip_boundary_one_float_above_20mm_fails(self):
        rows, entry, references, _ = fixture()
        rows = rows[:1]
        self.assertTrue(analyze_rows(rows, entry, references)['slip']['within20mmPass'])
        rows[0]['loadedSlipDistanceM'][0] = float(np.nextafter(.020, math.inf))
        result = analyze_rows(rows, entry, references)
        self.assertFalse(result['slip']['within20mmPass'])
        self.assertEqual(result['slip']['firstStrictExceedance']['sourceStep'], 1)

    def test_archived_completions_do_not_invent_loaded_landings(self):
        old = archived_reference(dict(stopOnly=dict(completedTimesS=[9., 17., 30.])), 'summary.json')
        self.assertIsNone(old['loadedLandingIntervalsS'])
        self.assertFalse(old['loadedLandingIntervalsAvailable'])
        self.assertEqual(old['archivedCompletionIntervalsS'], [8., 13.])

    def test_broken_counter_and_summary_landing_are_rejected(self):
        rows, entry, references, records = fixture()
        records[0]['landingNativeStep'] += 1
        self.assertFalse(analyze_rows(rows, entry, references, records)['landingSummaryMatchesMeasuredRows'])
        rows[0]['sourceStep'] = 2
        with self.assertRaisesRegex(ValueError, 'noncontiguous'):
            analyze_rows(rows, entry, references)


class RendererTruthfulness(unittest.TestCase):
    @staticmethod
    def audit():
        return dict(savedDataOnly=True, physicsIntegrations=0,
                    referenceReplay=dict(actualRecordedIntegrations=100),
                    hashes={'frames.ndjson': 'abc'}, reviewedWalkTenStepPhysicalPass=True)

    def test_limited_gate_never_claims_full_physical_pass_without_audit(self):
        result = gate_verdict(.001, .004, 0)
        self.assertTrue(result['walkingJointAndCaptureGatesPass'])
        self.assertIn('LIMITED', result['walkingJointAndCaptureGatesScope'])
        self.assertFalse(result['fullPhysicalPass'])

    def test_undefined_capture_rejects_both_limited_and_full_pass(self):
        for minimum in (.004, None, math.inf, math.nan):
            result = gate_verdict(.001, minimum, 1, self.audit(), 100, 'abc')
            self.assertFalse(result['walkingJointAndCaptureGatesPass'])
            self.assertFalse(result['fullPhysicalPass'])
        self.assertFalse(gate_verdict(None, .004, 0)['walkingJointAndCaptureGatesPass'])

    def test_full_pass_requires_explicit_correct_audit_and_matching_count_hash(self):
        audit = self.audit()
        result = gate_verdict(.001, .00295, 0, audit, 100, 'abc')
        self.assertTrue(result['fullPhysicalPass'])
        self.assertFalse(result['allFrame3mmCaptureQualityPass'])
        self.assertTrue(result['allFrameDefinedNonnegativeCapturePass'])
        for count, sha in [(99, 'abc'), (100, 'wrong')]:
            self.assertFalse(gate_verdict(.001, .004, 0, audit, count, sha)['fullPhysicalPass'])
        for key, value in [('physicsIntegrations', 1), ('savedDataOnly', False),
                           ('reviewedWalkTenStepPhysicalPass', False)]:
            changed = copy.deepcopy(audit)
            changed[key] = value
            self.assertFalse(gate_verdict(.001, .004, 0, changed, 100, 'abc')['fullPhysicalPass'])

    def test_old_motion_or_runner_flags_do_not_qualify_reviewed_physics(self):
        audit = self.audit()
        del audit['reviewedWalkTenStepPhysicalPass']
        audit.update(auditedMotionPass=True, strictCadencedSequencePass=True, passed=True)
        self.assertFalse(gate_verdict(.001, .004, 0, audit, 100, 'abc')['fullPhysicalPass'])

    def test_terminal_physical_field_accepted_but_final_plan_not_inferred(self):
        audit = self.audit()
        del audit['reviewedWalkTenStepPhysicalPass']
        audit['terminalPelvisTenStepPhysicalPass'] = True
        result = gate_verdict(.001, .004, 0, audit, 100, 'abc')
        self.assertTrue(result['fullPhysicalPass'])
        self.assertNotIn('finalPlanPass', result)

    def test_exact_1x_distinct_interval_end_samples_and_final_source_pose(self):
        for native_rows in (4950, 66687, 91234):
            for fps in (5, 10, 20):
                selected, encoded_fps = select_source_steps(native_rows, fps)
                self.assertEqual(selected[-1], native_rows)
                self.assertEqual(len(set(selected)), len(selected))
                self.assertEqual(Fraction(len(selected), 1)/encoded_fps, Fraction(native_rows, 800))
                self.assertTrue(all(abs(Fraction(step, 800)-Fraction(i, 1)/encoded_fps) <= Fraction(1,1600)
                                    for i, step in enumerate(selected, 1)))


if __name__ == '__main__':
    unittest.main()
