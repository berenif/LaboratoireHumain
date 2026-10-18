"""Zero-integration synthetic journal corruption checks for strict cadence audit."""
import copy
import unittest

import numpy as np

from audit_source_cadenced_steps import DT, PHASES, audit_rows


def fixture():
    cadence = dict(requestedSteps=1, requestedDistanceM=.18,
        phaseDurationsS={p:.1 for p in PHASES[:-1]}, initialTransferS=.1,
        finalQuietRequiredS=5., intermediateStableRequiredS=.25,
        globalLoadedSlipLimitM=.01, physicalTimeOnly=True, playbackSpeed=1.)
    q = [0.,0.,.8,1.,0.,0.,0.]+[0.]*21
    entry = dict(qpos=q.copy(), sourceTimeS=0., sourceStep=0,
        com=[0.,0.,.8],
        feet=[dict(position=[0.,.1,.05]),dict(position=[0.,-.1,.05])])
    ref = dict(phase_durations=cadence['phaseDurationsS'].copy(), heading=[1.,0.,0.],
        targets=[[[0.,.1,.05],np.eye(3).tolist()],[[0.,-.1,.05],np.eye(3).tolist()]],
        com1=[0.,0.,.8])
    record = dict(index=1, requestedDistanceM=.18, swingSide='left',startNativeStep=0,startSourceTimeS=0.)
    rows = []
    for phase in PHASES:
        length = 160 if phase == 'transfer' else 4000 if phase == 'stand' else 81
        for sample in range(1,length+1):
            forward = .18 if phase in ('translate','land','settle','stand') else 0.
            single = phase in ('transfer','lift','translate')
            feet = [dict(position=[forward,.1,.05],clearance=.03 if phase in ('lift','translate') else 0.,
                loadN=0. if single else 50.,loadedSlipSpeedMps=0.,
                groundRays=[dict(hit=True,ownBodyExcluded=True) for _ in range(4)]),
                dict(position=[0.,-.1,.05],clearance=0.,loadN=100.,loadedSlipSpeedMps=0.,
                groundRays=[dict(hit=True,ownBodyExcluded=True) for _ in range(4)])]
            n = len(rows)+1
            rows.append(dict(sourceStep=n,sourceTimeS=n*DT,elapsedS=n*DT,
                phaseElapsedS=sample*DT,stepIndex=1,phase=phase,swingSide=0,
                requestedForwardM=.18,swingForwardDisplacementM=forward,
                qpos=q.copy(),qvel=[0.]*27,loadedSlipDistanceM=[0.,0.],
                actualMotorTorquesNm=[0.]*21,warnings=0,nonfootLoadN=0.,
                maximumSelfPenetrationM=0.,jointLimitMinimumMarginRad=.02,
                jointLimitMarginsRad=[.02]*21,minimumComCaptureMarginM=.02,
                supportPointsXY=[[-.1,-.02],[-.1,.02],[.1,-.02],[.1,.02]],
                jointGroundRays=[dict(hit=True,ownBodyExcluded=True) for _ in range(22)],
                feet=feet,com=[0.,0.,.8],comVelocity=[0.,0.,0.],torsoTiltDeg=0.,
                maximumBodyLinearMps=0.,maximumBodyAngularRadps=0.))
        if phase == 'land':
            record.update(landingNativeStep=len(rows),landingTimeS=len(rows)*DT,
                achievedAtLoadAcquisitionM=.18)
    record.update(completedNativeStep=len(rows),completedTimeS=len(rows)*DT,achievedForwardM=.18)
    return rows, entry, [record], {1:ref}, cadence


def sequence_fixture(continuous=True, stable_samples=200):
    rows, entry, records, references, cadence = fixture()
    last_phase = 'land' if continuous else 'stand'
    first = [r for r in rows if (PHASES.index(r['phase']) <= PHASES.index(last_phase)
        and (r['phase'] != 'stand' or r['phaseElapsedS'] <= stable_samples*DT))]
    first_record = copy.deepcopy(records[0])
    first_record.update(completedNativeStep=len(first),completedTimeS=len(first)*DT)
    second = copy.deepcopy(rows)
    offset = len(first)
    for row in second:
        row.update(stepIndex=2,swingSide=1,sourceStep=row['sourceStep']+offset,
            sourceTimeS=row['sourceTimeS']+offset*DT,elapsedS=row['elapsedS']+offset*DT)
        swing,support = row['feet']
        swing['position'][1] = -.1
        support['position'] = [.18,.1,.05]
        row['feet'] = [support,swing]
    second_record = copy.deepcopy(records[0])
    second_record.update(index=2,swingSide='right',startNativeStep=offset,startSourceTimeS=offset*DT)
    for key in ('landingNativeStep','completedNativeStep'):
        second_record[key] += offset
    for key in ('landingTimeS','completedTimeS'):
        second_record[key] += offset*DT
    second_reference = copy.deepcopy(references[1])
    second_reference['targets'][0][0][0] = .18
    cadence.update(requestedSteps=2,intermediateStableRequiredS=0. if continuous else .25)
    if continuous:
        cadence['continuousLoadedHandoff'] = True
    return first+second,entry,[first_record,second_record],{1:references[1],2:second_reference},cadence


class CadencedJournalAudit(unittest.TestCase):
    def setUp(self):
        self.rows,self.entry,self.records,self.references,self.cadence = fixture()

    def result(self):
        return audit_rows(self.rows,self.entry,self.records,self.references,self.cadence,10.,np.ones(21))

    def rejects(self, check):
        result = self.result()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks'][check])

    def test_complete_synthetic_pilot_passes(self):
        result = self.result()
        self.assertTrue(result['passed'], [k for k,v in result['checks'].items() if not v])

    def test_one_short_of_five_seconds_fails(self):
        self.rows.pop()
        self.rejects('finalFiveSecondsQuietOnFinalStep')

    def test_missing_land_phase_fails(self):
        for row in self.rows:
            if row['phase'] == 'land':
                row['phase'] = 'settle'
        self.rejects('exactOrderedConsecutivePhases')

    def test_insufficient_landing_load_run_fails(self):
        landing = [r for r in self.rows if r['phase'] == 'land']
        landing[-2]['feet'][0]['loadN'] = 0.
        self.rejects('everyStepLandingQualified')

    def test_insufficient_clearance_run_fails(self):
        for row in self.rows:
            if row['phase'] == 'lift':
                row['feet'][0]['clearance'] = .024
        self.rejects('everyStepLiftQualified')

    def test_landing_summary_cannot_substitute_for_actual_distance(self):
        for row in self.rows:
            if row['phase'] in ('translate','land','settle','stand'):
                row['feet'][0]['position'][0] = .179
                row['swingForwardDisplacementM'] = .179
        self.rejects('everyStepLandingQualified')

    def test_wrong_measured_side_fails(self):
        self.rows[-1]['swingSide'] = 1
        self.rejects('alternatingMeasuredSides')

    def test_source_time_warp_fails(self):
        self.rows[100]['sourceTimeS'] += .1
        self.rejects('sourceTimeContinuity')

    def test_native_counter_reset_fails(self):
        self.rows[100]['sourceStep'] = 1
        self.rejects('contiguousNativeCounters')

    def test_root_pose_installation_fails(self):
        self.rows[100]['qpos'][0] += .001
        self.rejects('rootPositionContinuity')

    def test_root_rotation_installation_fails(self):
        self.rows[100]['qpos'][3:7] = [np.cos(.01),np.sin(.01),0.,0.]
        self.rejects('rootQuaternionContinuity')

    def test_global_slip_counter_reset_fails(self):
        self.rows[100]['loadedSlipDistanceM'][0] = .001
        self.rejects('loadedSlipAccumulatorContinuity')

    def test_slip_limit_cannot_be_relaxed_in_config(self):
        self.cadence['globalLoadedSlipLimitM'] = .1
        self.rejects('originalQuietAndSlipThresholds')

    def test_missing_final_step_fails(self):
        self.cadence['requestedSteps'] = 2
        self.rejects('allExpectedStepRecords')

    def test_invalid_ground_ray_fails(self):
        self.rows[100]['feet'][0]['groundRays'][0]['ownBodyExcluded'] = False
        self.rejects('allOriginalGroundRays')

    def test_unload_must_be_measured(self):
        for row in self.rows:
            if row['phase'] == 'transfer':
                row['feet'][0]['loadN'] = 1.
        self.rejects('everyStepUnloadQualified')

    def test_fabricated_foot_origin_fails(self):
        self.references[1]['targets'][0][0][0] = -.1
        self.rejects('stepOriginsMatchPrecedingMeasuredFeet')

    def test_landing_record_must_be_at_phase_end(self):
        self.records[0]['landingNativeStep'] -= 1
        self.records[0]['landingTimeS'] -= DT
        self.rejects('landingAndCompletionAreActualPhaseEnds')

    def test_forged_positive_capture_margin_fails(self):
        self.rows[100]['supportPointsXY'] = [[.01,-.02],[.01,.02],[.11,-.02],[.11,.02]]
        self.rows[100]['minimumComCaptureMarginM'] = .02
        result = self.result()
        self.assertFalse(result['checks']['captureMarginTelemetryMatchesReconstruction'])
        self.assertFalse(result['checks']['definedNonnegativeCurrentCapture'])

    def test_undefined_support_hull_fails(self):
        self.rows[100]['supportPointsXY'] = [[0.,0.],[1.,0.]]
        self.rows[100]['minimumComCaptureMarginM'] = None
        self.rejects('definedNonnegativeCurrentCapture')


class ContinuousLoadedHandoffAudit(unittest.TestCase):
    def setUp(self):
        self.rows,self.entry,self.records,self.references,self.cadence = sequence_fixture()

    def result(self):
        return audit_rows(self.rows,self.entry,self.records,self.references,self.cadence,10.,np.ones(21))

    def rejects(self, name):
        result = self.result()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks'][name])

    def test_explicit_continuous_loaded_handoff_passes(self):
        result = self.result()
        self.assertTrue(result['passed'], [k for k,v in result['checks'].items() if not v])
        self.assertEqual(self.records[0]['completedNativeStep'],self.records[0]['landingNativeStep'])

    def test_missing_explicit_flag_cannot_skip_intermediate_stand(self):
        del self.cadence['continuousLoadedHandoff']
        self.rejects('exactOrderedConsecutivePhases')

    def test_handoff_requires_full_loaded_landing_run(self):
        row = self.rows[self.records[0]['landingNativeStep']-2]
        row['feet'][0]['loadN'] = 0.
        self.rejects('everyStepLandingQualified')

    def test_completion_must_equal_landing_sample(self):
        self.records[0]['completedNativeStep'] += 1
        self.records[0]['completedTimeS'] += DT
        self.rejects('everyStepCompletionQualified')

    def test_next_transfer_starts_immediately_after_landing(self):
        self.records[1]['startNativeStep'] += 1
        self.rejects('stepStartCountersMatchJournal')

    def test_nonfinal_settle_or_stand_is_not_continuous_handoff(self):
        self.rows[self.records[0]['landingNativeStep']-1]['phase'] = 'stand'
        self.rejects('exactOrderedConsecutivePhases')

    def test_final_quiet_is_still_five_seconds(self):
        self.rows.pop()
        self.rejects('finalFiveSecondsQuietOnFinalStep')

    def test_default_historical_mode_still_accepts_200_stable_samples(self):
        self.rows,self.entry,self.records,self.references,self.cadence = sequence_fixture(False)
        result = self.result()
        self.assertTrue(result['passed'], [k for k,v in result['checks'].items() if not v])

    def test_default_historical_mode_rejects_199_stable_samples(self):
        self.rows,self.entry,self.records,self.references,self.cadence = sequence_fixture(False,199)
        self.rejects('everyStepCompletionQualified')

    def test_continuous_protocol_requires_explicit_zero_intermediate_stand(self):
        self.cadence['intermediateStableRequiredS'] = .25
        self.rejects('originalQuietAndSlipThresholds')


if __name__ == '__main__':
    unittest.main()
