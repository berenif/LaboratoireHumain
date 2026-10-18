"""Pure causal replay and corruption tests. No native world or integrations."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

import numpy as np
from scipy.spatial import ConvexHull

from audit_source_robust_support import (ACTUAL_PATCH,NOMINAL_PATCH,DT,RESERVE,
    audit_replay,inset_projection,quintic,qualify_robust,support_observation,triangle,
    selection_from_history,forecast_math,qualify_prospective,FORECAST_RISK,
    ExactForecastReplay,ORIGINAL_IDENTITY,UNLOAD_CHECK,REQUIRED_STRICT_CHECKS)


def fixture():
    corners = np.array([[-.1,-.02],[-.1,.02],[.1,-.02],[.1,.02]])
    def foot(offset,load):
        rays = [dict(hit=True,ownBodyExcluded=True,capsuleEndpoint=i,
            originSource=[p[0]+offset,p[1],.227+(.001 if i == 0 else 0.)],
            distanceM=.227+(.001 if i == 0 else 0.),directionSource=[0.,0.,-1.]) for i,p in enumerate(corners)]
        return dict(position=[offset,0.,.027],groundRays=rays,loadN=load,clearance=0.)
    entry = dict(sourceStep=0,sourceTimeS=0.,com=[0.,0.,.8],comVelocity=[0.,0.,0.],
        feet=[foot(.5,0.),foot(0.,100.)],supportPointsXY=corners.tolist(),
        jointLimitMinimumMarginRad=.02,jointLimitMarginsRad=[.02]*21,
        maximumBodyLinearMps=0.,maximumBodyAngularRadps=0.)
    reference = dict(stance_side=1,swing_side=0,radii=[.027,.027],com0=[0.,0.,.8],
        com1=[0.,0.,.8],finalCom=[0.,0.,.8],phase_durations=dict(transfer=.1,lift=.1,
            translate=.1,land=.1,settle=1.52))
    cadence = dict(requestedSteps=1,requestedDistanceM=.18,robustSupportAdmission=dict(
        enabled=True,supportPatchConstruction=ACTUAL_PATCH,causalWeakCornerHistoryS=.1,
        minimumFrequency=.8,actualSurvivorsRequired=True,maximumComCorrectionM=.012,
        measuredAdmissionReserveM=.003,allFrameOriginalCaptureGuardUnchanged=True,
        arbitraryCornerLossRobustnessClaim=False))
    delta = np.r_[.004*np.array([1.,5.])/np.sqrt(26.),0.]
    selection = dict(selected=True,causalHistoryOnly=True,weakCorner=0,survivingCorners=[1,2,3],
        supportPatchConstruction=ACTUAL_PATCH,historySourceStartS=0.,historySourceEndS=.1,
        historySamples=21,shallowestFrequency=1.,selectedAtTransferS=.1,
        allSurvivorsActuallySupportedThroughoutHistory=True,comCorrectionM=delta.tolist(),
        correctionMagnitudeM=.004,maximumCorrectionM=.012,correctionDurationS=1.,
        maximumReferenceSpeedMps=.025,maximumReferenceAccelerationMps2=.15,
        admissionReserveM=.003,referenceReserveM=.004,endpointSupportAssociationToleranceM=.001,
        arbitraryCornerLossRobustnessClaim=False)
    rows,notes = [],[];previous = entry;clock = (0.,0.,0.)
    equations = ConvexHull(corners[[1,2,3]]).equations
    for n in range(1044):
        phase = 'transfer' if n < 960 else 'lift'
        elapsed = n*DT if phase == 'transfer' else (n-960)*DT
        if n%4 == 0:
            if n >= 80:
                telemetry = copy.deepcopy(selection)
                clock = quintic(elapsed-.1,1.) if phase == 'transfer' else (1.,0.,0.)
                telemetry['correctionClock'] = clock
                com = np.array(previous['com'])[:2];gap=.004
                telemetry['barriers'] = [dict(captureMarginM=-float(e[:2]@com+e[2]),
                    comMarginM=-float(e[:2]@com+e[2]),reserveM=-.001+clock[0]*gap,
                    reserveRateMps=clock[1]*gap,reserveAccelerationMps2=clock[2]*gap,
                    actualSurvivingSupportOnly=True,gainPerS=8.) for e in equations]
            else:
                telemetry = dict(selected=False,causalHistoryOnly=True,barriers=[],correctionClock=(0.,0.,0.))
            notes.append(dict(t=n*DT,phase=phase,stepIndex=1,comTarget=(np.array(reference['com1'])+clock[0]*delta).tolist(),
                robustSupportAdmission=telemetry))
        row = copy.deepcopy(entry)
        row.update(sourceStep=n+1,sourceTimeS=(n+1)*DT,stepIndex=1,phase=phase,
            phaseElapsedS=elapsed+DT,com=(np.array(reference['com1'])+clock[0]*delta).tolist())
        rows.append(row);previous=row
    return dict(rows=rows,entry=entry,references={1:reference},cadence=cadence,notes=notes,mass=10.)


def original_fixture(count=1):
    checks = dict.fromkeys(REQUIRED_STRICT_CHECKS,True);checks[ORIGINAL_IDENTITY] = False
    checks[UNLOAD_CHECK] = False
    return dict(requestedSteps=count,requestedDistanceM=.18,strictCadencedSequencePass=False,
        tenStep180mmQualificationPass=False,measured=dict(checks=checks))


class NativeWitnessStub:
    """Only synthetic reducer tests use this; real audit requires exact replay."""
    def __init__(self,valid=True,coverage=True):self.ids=[];self.valid=valid;self.covered=coverage
    def __call__(self,note,row,reference,forecast):
        self.ids.append(row['sourceStep'])
        return dict(passed=self.valid,nativeVerified=self.valid,sampleStep=row['sourceStep'])
    def coverage(self,index,ids):return self.covered and self.ids == ids


def forecast_fixture(disagreements=0):
    data=fixture();data['prospective']=True;data['forecast_replayer']=NativeWitnessStub()
    data['cadence']['robustSupportAdmission'].update(riskSignal=FORECAST_RISK,
        forecastHorizonS=.1,forecastControlApplied=False,forecastRolloutIntegrations=0)
    selected_forecast=None
    for i,note in enumerate(data['notes']):
        n=int(round(note['t']/DT));t=note['robustSupportAdmission']
        if n <= 80:
            row=data['rows'][n-1] if n else data['entry']
            obs=support_observation(row,1);acceleration=[0.,1.,0.,0.] if i < disagreements else [1.,0.,0.,0.]
            predicted=obs['clearances']+.005*np.array(acceleration)
            forecast=dict(virtualUnloadNormalForcesN=[1.,2.,3.,4.],
                nativeUnloadEndpointVerticalAccelerationMps2=acceleration,
                nativeUnloadEndpointVerticalVelocityMps=[0.]*4,localClearanceForecastM=predicted.tolist(),
                currentEndpointClearanceM=obs['clearances'].tolist(),
                currentMatchedSupportCoordinatesSource=obs['matchedCorners'].tolist(),
                proposedMotorControl=[0.]*21,nativeProposedResponse=[0.]*33,
                proposedMotorMaximumFraction=0.,predictionHorizonS=.1,pressureRiskCorner=0,
                clearanceRiskCorner=int(np.argmax(predicted)),proposedControlApplied=False,
                physicsRolloutIntegrations=0,nativeBytesAndCounterUnchanged=True,
                nativePayloadSourceStep=n,nativePayloadSHA256=f'{n:064x}')
            if n < 80:t['currentCausalUnloadForecast']=forecast
            else:selected_forecast=forecast
        if t['selected']:
            t['selectedCausalUnloadForecast']=copy.deepcopy(selected_forecast)
            t['riskSignal']=FORECAST_RISK;t.pop('shallowestFrequency')
            t['riskAgreementFrequency']=(21-disagreements)/21
            t['retainedExactNativeForecastWitnesses']=21;t['forecastHistoryNativeSteps']=list(range(0,81,4))
            t['historyPredictedRiskCorners']=[-1]*disagreements+[0]*(21-disagreements)
    return data


class GeometryAndClock(unittest.TestCase):
    def test_inset_projection_analytic_diagonal(self):
        eq = ConvexHull(np.array([[-.1,.02],[.1,-.02],[.1,.02]])).equations
        expected=.004*np.array([1.,5.])/np.sqrt(26.)
        np.testing.assert_allclose(inset_projection([0.,0.],eq,.004),expected,atol=1e-14)

    def test_inset_projection_preserves_an_inside_point(self):
        eq=ConvexHull(np.array([[0.,0.],[1.,0.],[0.,1.]])).equations
        np.testing.assert_allclose(inset_projection([.2,.2],eq,.01),[.2,.2],atol=1e-14)

    def test_empty_inset_rejected(self):
        eq=ConvexHull(np.array([[0.,0.],[.001,0.],[0.,.001]])).equations
        with self.assertRaises(ValueError):inset_projection([0.,0.],eq,.01)

    def test_quintic_is_c2_at_both_clamped_ends(self):
        self.assertEqual(quintic(-1.,1.),(0.,0.,0.))
        self.assertEqual(quintic(0.,1.),(0.,0.,0.))
        self.assertEqual(quintic(1.,1.),(1.,0.,0.))
        self.assertEqual(quintic(2.,1.),(1.,0.,0.))

    def test_actual_triangle_uses_matched_points(self):
        data=fixture();row=data['entry'];row['supportPointsXY'][1][0] += .0005
        obs=support_observation(row,1)
        actual=triangle(obs,[1,2,3],True);nominal=triangle(obs,[1,2,3],False)
        self.assertGreater(np.max(abs(actual-nominal)),1e-5)

    def test_three_distinct_actual_points_required(self):
        data=fixture();row=data['entry']
        row['feet'][1]['groundRays'][2]['originSource'] = row['feet'][1]['groundRays'][1]['originSource'].copy()
        with self.assertRaises(ValueError):triangle(support_observation(row,1),[1,2,3],True)

    def test_unmatched_weak_corner_does_not_steal_a_valid_support(self):
        row=fixture()['entry'];row['supportPointsXY'].pop(0)
        obs=support_observation(row,1)
        self.assertFalse(obs['present'][0]);self.assertTrue(all(obs['present'][[1,2,3]]))
        self.assertEqual(triangle(obs,[1,2,3],True).shape,(3,3))


class CausalSupportReplay(unittest.TestCase):
    def setUp(self):self.data=fixture()
    def result(self):return audit_replay(**self.data)
    def rejects(self,key):
        result=self.result();self.assertFalse(result['passed']);self.assertFalse(result['checks'][key])
    def selected(self):return [n for n in self.data['notes']if n['robustSupportAdmission']['selected']]

    def test_positive_measured_admission_replay(self):
        result=self.result()
        self.assertTrue(result['passed'],[k for k,v in result['checks'].items()if not v])
        self.assertEqual(result['perStep'][1]['maximumUnloadRun'],80)
        self.assertTrue(result['perStep'][1]['liftAdmitted'])
        self.assertLessEqual(result['maximumRetainedHistorySamples'],22)

    def test_future_history_claim_fails(self):
        self.selected()[0]['robustSupportAdmission']['historySourceEndS'] += .005
        self.rejects('selectionMatchesPrecedingActualHistory')

    def test_forged_weak_corner_fails(self):
        self.selected()[0]['robustSupportAdmission']['weakCorner']=2
        self.rejects('selectionMatchesPrecedingActualHistory')

    def test_short_history_claim_fails(self):
        self.selected()[0]['robustSupportAdmission']['historySamples']=20
        self.rejects('selectionMatchesPrecedingActualHistory')

    def test_previous_history_support_loss_cannot_be_ignored(self):
        self.data['rows'][39]['supportPointsXY'].pop(2)
        self.rejects('selectionAtFirstCausalEligibleSolve')

    def test_previous_history_load_loss_cannot_be_ignored(self):
        self.data['rows'][39]['feet'][1]['loadN']=50.
        self.rejects('selectionAtFirstCausalEligibleSolve')

    def test_selection_cannot_disappear(self):
        self.selected()[10]['robustSupportAdmission']['selected']=False
        self.rejects('selectionNeverDisappearsOrReplacesHistory')

    def test_clock_derivative_forgery_fails(self):
        self.selected()[10]['robustSupportAdmission']['correctionClock']=(.1,99.,0.)
        self.rejects('analyticC2CorrectionClock')

    def test_reference_jump_fails(self):
        self.selected()[10]['comTarget'][0] += .001
        self.rejects('emittedCOMTargetMatchesBoundedCorrection')

    def test_maximum_shift_claim_cannot_relax(self):
        self.selected()[0]['robustSupportAdmission']['maximumCorrectionM']=.02
        self.rejects('selectionScopeAndPhysicalBounds')

    def test_barrier_margin_forgery_fails(self):
        self.selected()[10]['robustSupportAdmission']['barriers'][0]['comMarginM'] += .001
        self.rejects('triangleBarrierMarginsAndC2ReserveReplay')

    def test_79_actual_reserve_samples_do_not_admit_lift(self):
        row=self.data['rows'][880]
        row['com']=(np.array(row['com'])+[0.,-.002,0.]).tolist()
        self.rejects('eachLiftHas80ActualTriangleCorrectedUnloadSamples')
        self.assertTrue(self.result()['checks']['unchangedEveryNativeActualHullCaptureGuard'])

    def test_nominal_triangle_is_diagnostic_only(self):
        self.data['cadence']['robustSupportAdmission']['supportPatchConstruction']=NOMINAL_PATCH
        self.rejects('explicitActualSupportPatchConstruction')
        self.assertTrue(self.result()['nominalGeometryDiagnosticOnly'])

    def test_original_full_hull_guard_is_not_replaced_by_triangle(self):
        self.data['rows'][-1]['com'][0]=.2
        self.rejects('unchangedEveryNativeActualHullCaptureGuard')

    def test_endpoint_fk_disagreement_fails(self):
        self.data['endpoint_validator']=lambda row:.001
        self.rejects('independentFootEndpointFK')

    def test_missing_control_note_fails(self):
        self.data['notes'].pop()
        self.rejects('allActualControlNotesAligned')

    def test_changed_admission_frequency_fails(self):
        self.data['cadence']['robustSupportAdmission']['minimumFrequency']=.7
        self.rejects('robustAdmissionPolicy')

    def test_new_forecast_policy_cannot_inherit_geometric_green(self):
        self.data['cadence']['robustSupportAdmission']['riskSignal']='planned-unload-QP-minimum-normal-pressure-and-native-100ms-clearance-agreement'
        self.rejects('geometricRiskSignalPolicyOnly')

    def test_unselected_reference_jump_fails(self):
        self.data['notes'][2]['comTarget'][0] += .001
        self.rejects('unselectedCOMReferenceRemainsNominal')

    def test_out_of_bounds_candidate_is_recorded_as_rejection(self):
        ref=self.data['references'][1];ref['com1']=[-.1,-.1,.8]
        current=copy.deepcopy(self.data['entry']);current['sourceTimeS']=.1
        history=[(i*.005,0,np.ones(4,dtype=bool),True)for i in range(21)]
        candidate=selection_from_history(history,current,ref,.1,10.,actual=True)
        self.assertGreater(candidate['correctionMagnitudeM'],.012)
        for note in self.data['notes']:
            elapsed=note['t'] if note['phase']=='transfer' else 0.
            s=quintic(elapsed,.1)[0] if note['phase']=='transfer' else 1.
            note['comTarget']=(np.array(ref['com0'])+(np.array(ref['com1'])-ref['com0'])*s).tolist()
            note['robustSupportAdmission']=dict(selected=False,causalHistoryOnly=True,barriers=[],correctionClock=(0.,0.,0.))
            if note['t'] >= .1:
                note['robustSupportAdmission'].update(lastRejectedWeakCorner=0,
                    lastRejectedCorrectionM=candidate['correctionMagnitudeM'],
                    lastAdmissionRejection='bounded correction unavailable; continue observing current support')
        result=self.result()
        self.assertFalse(result['passed'])
        self.assertTrue(result['checks']['boundedRejectionTelemetryMatchesCurrentCausalCandidate'])
        self.assertEqual(result['perStep'][1]['boundedUnavailableSolves'],220)
        self.assertIsNone(result['perStep'][1]['selection'])


class SeparateRobustQualification(unittest.TestCase):
    def result(self,count=1,original=None,replay=True,model=True,controls=True,provenance=True):
        return qualify_robust(original or original_fixture(count),dict(passed=replay),
            dict(passed=model,expectedImpratio=8.),controls,provenance,count)
    def test_pilot_can_never_claim_ten(self):
        result=self.result();self.assertTrue(result['explicitRobustSupportImpratio8PilotPass'])
        self.assertFalse(result['explicitRobustSupportImpratio8TenStepPass'])
        self.assertFalse(result['originalStrictCadencedSequencePass'])
    def test_ten_uses_separate_ten_flag(self):
        result=self.result(10);self.assertFalse(result['explicitRobustSupportImpratio8PilotPass'])
        self.assertTrue(result['explicitRobustSupportImpratio8TenStepPass'])
    def test_missing_final_quiet_is_not_an_allowed_exception(self):
        original=original_fixture();original['measured']['checks']['finalFiveSecondsQuietOnFinalStep']=False
        self.assertFalse(self.result(original=original)['explicitRobustSupportImpratio8PilotPass'])
    def test_slip_failure_is_not_an_allowed_exception(self):
        original=original_fixture();original['measured']['checks']['globalLoadedSlipWithinOriginalLimit']=False
        self.assertFalse(self.result(original=original)['explicitRobustSupportImpratio8PilotPass'])
    def test_ordinary_identity_cannot_be_relabelled_true(self):
        original=original_fixture();original['measured']['checks'][ORIGINAL_IDENTITY]=True
        self.assertFalse(self.result(original=original)['explicitRobustSupportImpratio8PilotPass'])
    def test_replay_failure_prevents_pilot_claim(self):
        self.assertFalse(self.result(replay=False)['explicitRobustSupportImpratio8PilotPass'])
    def test_model_failure_prevents_pilot_claim(self):
        self.assertFalse(self.result(model=False)['explicitRobustSupportImpratio8PilotPass'])
    def test_source_failure_prevents_pilot_claim(self):
        self.assertFalse(self.result(provenance=False)['explicitRobustSupportImpratio8PilotPass'])
    def test_gains_failure_prevents_pilot_claim(self):
        self.assertFalse(self.result(controls=False)['explicitRobustSupportImpratio8PilotPass'])


class ProspectiveRiskReplay(unittest.TestCase):
    def test_distinct_forecast_profile_pass_does_not_relabel_geometric(self):
        result=audit_replay(**forecast_fixture())
        self.assertTrue(result['prospectiveRiskReplayPass'],[k for k,v in result['checks'].items()if not v])
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['geometricRiskSignalPolicyOnly'])
    def test_four_disagreements_are_retained_and_count_against_denominator(self):
        result=audit_replay(**forecast_fixture(4))
        self.assertTrue(result['prospectiveRiskReplayPass'])
        self.assertEqual(result['perStep'][1]['selection']['riskAgreementFrequency'],17/21)
    def test_five_disagreements_block_selection(self):
        result=audit_replay(**forecast_fixture(5))
        self.assertFalse(result['prospectiveRiskReplayPass'])
        self.assertFalse(result['checks']['selectionAtFirstCausalEligibleSolve'])
    def test_missing_native_witness_blocks_forecast_profile(self):
        data=forecast_fixture();data['forecast_replayer']=NativeWitnessStub(coverage=False)
        self.assertFalse(audit_replay(**data)['prospectiveRiskReplayPass'])
    def test_native_or_qp_replay_disagreement_blocks_profile(self):
        data=forecast_fixture();data['forecast_replayer']=NativeWitnessStub(valid=False)
        self.assertFalse(audit_replay(**data)['prospectiveRiskReplayPass'])
    def test_stale_forecast_cannot_count_as_a_fresh_observation(self):
        data=forecast_fixture()
        data['notes'][10]['robustSupportAdmission']['currentCausalUnloadForecast']['nativePayloadSourceStep']=36
        result=audit_replay(**data)
        self.assertFalse(result['prospectiveRiskReplayPass'])
        self.assertFalse(result['checks']['forecastBoundToCurrentNativeSample'])
    def test_future_sample_id_cannot_enter_selected_window(self):
        data=forecast_fixture()
        data['notes'][20]['robustSupportAdmission']['forecastHistoryNativeSteps'][-1]=84
        result=audit_replay(**data)
        self.assertFalse(result['prospectiveRiskReplayPass'])
        self.assertFalse(result['checks']['selectedWindowNativeWitnessCoverage'])
    def test_forecast_arithmetic_forgery_fails(self):
        data=forecast_fixture();forecast=data['notes'][0]['robustSupportAdmission']['currentCausalUnloadForecast']
        forecast['localClearanceForecastM'][0] += .001
        self.assertFalse(forecast_math(forecast,data['entry'],data['references'][1])['passed'])
    def test_proposed_control_application_claim_fails(self):
        data=forecast_fixture();forecast=data['notes'][0]['robustSupportAdmission']['currentCausalUnloadForecast']
        forecast['proposedControlApplied']=True
        self.assertFalse(forecast_math(forecast,data['entry'],data['references'][1])['passed'])
    def test_forecast_points_must_be_current_measured_points(self):
        data=forecast_fixture();forecast=data['notes'][0]['robustSupportAdmission']['currentCausalUnloadForecast']
        forecast['currentMatchedSupportCoordinatesSource'][1][0] += .001
        self.assertFalse(forecast_math(forecast,data['entry'],data['references'][1])['passed'])
    def test_separate_prospective_pilot_flag_requires_native_proof(self):
        original=original_fixture();replay=dict(passed=False,prospectiveRiskReplayPass=True)
        result=qualify_prospective(original,replay,dict(passed=True,expectedImpratio=8.),True,True,1,dict(passed=True))
        self.assertTrue(result['explicitProspectiveUnloadRiskImpratio8PilotPass'])
        self.assertFalse(result['explicitProspectiveUnloadRiskImpratio8TenStepPass'])
        self.assertFalse(result['explicitRobustSupportImpratio8PilotPass'])
        self.assertFalse(result['originalStrictCadencedSequencePass'])
        result=qualify_prospective(original,replay,dict(passed=True,expectedImpratio=8.),True,True,1,dict(passed=False))
        self.assertFalse(result['explicitProspectiveUnloadRiskImpratio8PilotPass'])


class ExactWitnessCorruption(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.source=self.root/'repo/scripts/getup-step';self.source.mkdir(parents=True)
        self.base=self.root/'evidence';self.folder=self.base/'run/generation-001';self.folder.mkdir(parents=True)
        frozen=self.base/'frozen-source';frozen.mkdir()
        names=('source_step_probe.py','source_cadenced_ten_steps.py','source_upright_large_steps.py',
            'source_opposed_arm_swing.py','source_robust_support_steps.py','source_arm_knee_mobility.py')
        self.receipt=dict(codeUnchangedDuringRun=True,codeSHA256AtStart={})
        for name in names:
            data=('inert source '+name).encode();(self.source/name).write_bytes(data);(frozen/name).write_bytes(data)
            self.receipt['codeSHA256AtStart'][name]=hashlib.sha256(data).hexdigest()
        (self.source/'source-step-selected-v1.json').write_text(json.dumps(dict(helper='helper.so')))
        session=self.base/'model-session'
        files=dict(modelSHA256=session/'source-model/humanoid_high_freq.xml',helperSHA256=self.root/'repo/helper.so',
            bridgeSHA256=session/'source_bridge_v2.so',engineSHA256=session/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3')
        bindings={}
        for key,path in files.items():
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(('inert '+key).encode());bindings[key]=hashlib.sha256(path.read_bytes()).hexdigest()
        directory=self.folder/'robust-risk-step-01';directory.mkdir()
        raw=b'x'*1024;packed=zlib.compress(raw);samples=[]
        for counter in range(0,81,4):
            name=f'source-{counter:06d}.native-payload.zlib';(directory/name).write_bytes(packed)
            samples.append(dict(sourceStep=counter,sourceTimeS=counter*DT,payloadBytes=len(raw),
                payloadSHA256=hashlib.sha256(raw).hexdigest(),compressedSHA256=hashlib.sha256(packed).hexdigest(),filename=name))
        self.manifest=dict(schema=1,**bindings,stepIndex=1,phase='transfer',stanceSide=1,selectedAtTransferS=.1,
            payloadEncoding='zlib-compressed exact lh_checkpoint_export payload',originalNativeWorldUnchanged=True,
            physicsIntegrations=0,samples=samples)
        self.manifest_path=directory/'manifest.json';self.write_manifest()
    def write_manifest(self):self.manifest_path.write_text(json.dumps(self.manifest))
    def make(self):
        with patch('importlib.util.find_spec',return_value=SimpleNamespace(origin=str(self.source/'source_step_probe.py'))):
            result=ExactForecastReplay(self.base,self.folder,{1:dict(stance_side=1)},self.receipt,{})
        self.addCleanup(result.close);self.assertIsNone(result.world)
        return result
    def test_metadata_check_never_opens_a_native_world(self):
        r=self.make();self.assertFalse(r.failures);self.assertIsNone(r.world)
        self.assertEqual(r.imports+r.qp_calls+r.response_calls,0)
    def test_missing_required_source_hash_fails(self):
        self.receipt['codeSHA256AtStart'].pop('source_step_probe.py');self.assertTrue(self.make().failures)
    def test_changed_current_source_bytes_fail(self):
        (self.source/'source_step_probe.py').write_bytes(b'changed');self.assertTrue(self.make().failures)
    def test_missing_frozen_source_bytes_fail(self):
        (self.base/'frozen-source/source_step_probe.py').unlink();self.assertTrue(self.make().failures)
    def binding_rejected(self,key):
        self.manifest[key]='0'*64;self.write_manifest();self.assertTrue(self.make().failures)
    def test_wrong_model_binding_fails(self):self.binding_rejected('modelSHA256')
    def test_wrong_helper_binding_fails(self):self.binding_rejected('helperSHA256')
    def test_wrong_bridge_binding_fails(self):self.binding_rejected('bridgeSHA256')
    def test_wrong_engine_binding_fails(self):self.binding_rejected('engineSHA256')
    def test_reversed_manifest_order_fails(self):
        self.manifest['samples'].reverse();self.write_manifest();self.assertTrue(self.make().failures)
    def test_payload_path_escape_fails(self):
        self.manifest['samples'][0]['filename']='../escape';self.write_manifest();self.assertTrue(self.make().failures)
    def test_manifest_rollout_claim_fails(self):
        self.manifest['physicsIntegrations']=1;self.write_manifest();self.assertTrue(self.make().failures)
    def test_exact_selected_coverage_required(self):
        r=self.make();ids=list(range(0,81,4));self.assertFalse(r.coverage(1,ids))
        r.seen[1]=ids.copy();self.assertFalse(r.coverage(1,ids[::-1]));self.assertFalse(r.coverage(1,ids[:-1]))
        self.assertTrue(r.coverage(1,ids));self.assertTrue(r.summary()['passed'])
    def test_payload_corruptions_are_rejected_before_import(self):
        r=self.make();sample,path=r.samples[(1,0)];sample=copy.deepcopy(sample);original=path.read_bytes()
        self.assertEqual(len(r._payload(sample,path)),1024)
        for content in (original+b'extra',original[:-1],zlib.compress(b'x'*10000),zlib.compress(b'y'*1024)):
            with self.subTest(contentLength=len(content)):
                path.write_bytes(content);sample['compressedSHA256']=hashlib.sha256(content).hexdigest()
                with self.assertRaises((ValueError,zlib.error)):r._payload(sample,path)
        self.assertEqual(r.imports+r.qp_calls+r.response_calls,0)


class ExactProbeInputCorruption(unittest.TestCase):
    def test_probe_snapshot_cannot_change_gains_targets_or_extra_fields(self):
        replay=ExactForecastReplay.__new__(ExactForecastReplay)
        replay.maximum_errors={};replay.manifests={1:dict(selectedAtTransferS=.1,samples=[dict(sourceStep=80)])}
        replay.arm_mapping=dict(positionGain=100.,velocityGain=20.,taskWeight=2.,
            verifiedSwingDeltaRangeBySideRad=[[-.6,.8],[-.6,.8]])
        fields={name:0. for name in replay.FIELDS}
        for name,shape in replay.SHAPES.items():fields[name]=np.zeros(shape).tolist()
        fields.update(swing_side=0,stance_side=1,swing_body=10,self_avoidance=True,unload_admitted=True,
            arm_kp=100.,arm_kd=20.,arm_weight=2.,trajectory_time_scale=2.,joint_reserve=.01,shear_weight=.1,
            rootR=np.eye(3).tolist(),flatRotations=[np.eye(3).tolist()]*2,radii=[.027,.027],
            floorTargets=[.0265,.0265],heading=[1.,0.,0.],com0=[0.,0.,.8],com1=[0.,0.,.8],
            finalCom=[.376,0.,.8],landing=[.682,0.,.027],step_distance=.182,
            targets=[[[.5,0.,.027],np.eye(3).tolist()],[[0.,0.,.027],np.eye(3).tolist()]],
            phase_durations=dict(transfer=.1,lift=.8,translate=1.2,land=1.,settle=1.52),_seek_time=-.005)
        B=np.zeros((27,21));order=[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]
        for i,j in enumerate(order):B[6+j,i]=1.
        fields['B']=B.tolist();reference=copy.deepcopy(fields);reference['finalCom']=[9.,9.,.8]
        fields['floorTargets'][0]=.0271
        note=dict(t=0.,phase='transfer',stepIndex=1,opposedArmSwing=dict(targetArmJointRad=[0.]*6,
            targetArmVelocityRadps=[0.]*6,targetArmAccelerationRadps2=[0.]*6,legPitchOriginRad=[0.]*2))
        forecast=dict(probeControllerInput=dict(fields=fields,armMappingRanges=[[-.6,.8],[-.6,.8]],
            elapsedTransferS=0.,phase='transfer',coreEntryPoint='Controller.solve on copied CadencedController; existing arm targets already computed'))
        row=dict(sourceStep=0,sourceTimeS=0.);world=SimpleNamespace(ids={'left_foot':10,'right_foot':7},
            gears=np.ones(21),ground_height=lambda point:0.)
        probe,t=replay._probe(note,row,reference,forecast,world)
        self.assertIs(probe.w,world);self.assertEqual(t,0.)
        mutations=[('gain',lambda f:f['fields'].__setitem__('arm_kp',101.)),
            ('arm velocity',lambda f:f['fields']['arm_velocity'].__setitem__(0,1.)),
            ('arm acceleration',lambda f:f['fields']['arm_acceleration'].__setitem__(0,1.)),
            ('arm target',lambda f:f['fields']['q0'].__setitem__(15,1.)),
            ('leg posture',lambda f:f['fields']['q0'].__setitem__(0,1.)),
            ('extra field',lambda f:f['fields'].__setitem__('__class__','unsafe')),
            ('COM',lambda f:f['fields']['com1'].__setitem__(0,1.)),
            ('clock',lambda f:f.__setitem__('elapsedTransferS',.01)),
            ('phase',lambda f:f.__setitem__('phase','lift')),
            ('floor',lambda f:f['fields']['floorTargets'].__setitem__(0,.0265)),
            ('seek',lambda f:f['fields'].__setitem__('_seek_depth',.001)),
            ('mapping range',lambda f:f['armMappingRanges'][0].__setitem__(0,-9.))]
        for name,mutate in mutations:
            with self.subTest(corruption=name):
                altered=copy.deepcopy(forecast);mutate(altered['probeControllerInput'])
                with self.assertRaises(ValueError):replay._probe(note,row,reference,altered,world)


if __name__=='__main__':unittest.main()
