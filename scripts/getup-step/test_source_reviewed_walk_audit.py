"""Pure independent reviewed-profile corruption tests; no native world or physics."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from audit_source_reviewed_walk import (CONTEXT_FIELDS, OLD_FLAGS, PROFILE, REQUIRED_SOURCES, REQUIRED_ROBUST_REPLAY_CHECKS,
    VERSION, ExactReviewedForecastReplay, audit_context, audit_continuous_com, audit_reference_journal, audit_source_provenance, audit_wrench_sample, audit_wrench_summary, command_sha,
    hermite_coefficients, independent_references, limits_from_bound_xml, jet_matches, qualify_reviewed,
    reviewed_robust_result, segment_jet, value_sha)
from audit_source_contact_impedance import ORIGINAL_IDENTITY, REQUIRED_STRICT_CHECKS
from audit_source_robust_support import UNLOAD_CHECK
from source_reference_context import (Jet, ReferenceContext, canonical, nominal_com,
    nominal_joints, nominal_orientations, nominal_feet)


def rehash(receipt):
    receipt['contextSHA256'] = value_sha({k:v for k,v in receipt.items() if k != 'contextSHA256'})


def fixture():
    q0 = np.zeros(21); q0[[6,12]] = .2
    config = dict(q0=q0, com0=np.array([0.,0.,.8]), com1=np.array([.02,0.,.8]),
        finalCom=np.array([.126,0.,.8]), targets=[(np.array([0.,.1,.027]),np.eye(3)),
        (np.array([0.,-.1,.027]),np.eye(3))], rootR=np.eye(3), flatRotations=[np.eye(3),np.eye(3)],
        radii=[.027,.027],floorTargets=[.0265,.0265],swing_side=0,stance_side=1,swing_body=7,
        self_avoidance=True,B=np.zeros((27,21)),heading=np.array([1.,0.,0.]),
        landing=np.array([.182,.1,.0265]),step_distance=.182,lean_axis=np.array([.1,.02,0.]),
        phase_durations=dict(transfer=4.,lift=.8,translate=1.2,land=1.,settle=1.52),
        joint_reserve=.01,posture_start=np.zeros(21),shear_weight=.1,trajectory_time_scale=2.,
        arm_kp=100.,arm_kd=20.,arm_weight=2.,leg_origin=np.array([0.,0.]),arm_start=np.zeros(6),
        placement=True,arm_placement_duration=4.,loaded_settle_capture_barrier=True,knee_goal=np.array([.2,.2]),
        mode='baseline-context',referenceVersion=VERSION,armMappingRanges=[[-.45,.45],[-.45,.45]])
    c = SimpleNamespace(**config); c.w=SimpleNamespace(ids={'left_foot':7,'right_foot':10}); c.phase_duration=lambda phase: config['phase_durations'].get(phase,0.)
    causal = dict(arm=dict(anchor=dict(previousTimeS=None,previousLegRateRadps=None),
        measuredLegPitchRad=[0.,0.],measuredLegPitchRateRadps=[0.,0.],
        measuredLegPitchAccelerationRadps2=[0.,0.],nativeSourceStep=0,nativeSourceTimeS=0.,
        measuredOncePerNativeCommand=True),contactSeekBefore=dict(timeS=None,depthM=0.,rateMps=0.),
        contactSeekAfter=dict(timeS=0.,depthM=0.,rateMps=0.,jet=[0.,0.,0.]),robustSelectedBefore=False,
        preparedQ0=q0.tolist(),armPosture=[0.]*6,armVelocity=[0.]*6,armAcceleration=[0.]*6,
        armHistoryAfter=dict(previousTimeS=0.,previousLegRateRadps=[0.,0.]),
        persistentLegOrigin=[0.,0.],armMappingNeutralPitch=[0.,0.],
        robustCorrection=dict(selected=False,deltaM=[0.,0.,0.],startTransferS=None,durationS=None,clock=[0.,0.,0.]))
    serial = lambda obj: json.loads(json.dumps(obj,default=lambda x:x.tolist()))
    receipt = ReferenceContext(2,VERSION,0,0.,1,'transfer',0.,0,Jet.make(*nominal_com(c,0.,'transfer')),
        Jet.make(*nominal_joints(c,0.,'transfer')),Jet.make(np.zeros(6),np.zeros(6),np.zeros(6)),
        (100.,20.,2.),nominal_orientations(c,0.,'transfer'),nominal_feet(c,0.,'transfer',(0.,0.,0.)),
        (),False,canonical(causal),canonical(serial(config)),canonical({})).receipt()
    receipt = serial(receipt)
    arm=dict(targetArmJointRad=[0.]*6,targetArmVelocityRadps=[0.]*6,targetArmAccelerationRadps2=[0.]*6,
        legPitchOriginRad=[0.,0.],measuredLegPitchRad=[0.,0.],measuredLegPitchRateRadps=[0.,0.])
    note=dict(t=0.,phase='transfer',stepIndex=1,referenceContext=receipt,opposedArmSwing=arm,
        actualMotorControl=[0.]*21,actualMotorControlSHA256=command_sha([0.]*21),
        qpAssemblySHA256=dict.fromkeys(('P','linear','A','b'),'a'*64),
        reviewedWalk=dict(schema=2,mode='baseline-context',referenceVersion=VERSION,immutablePreparedContext=True),
        robustSupportAdmission=dict(selected=False,causalHistoryOnly=True,barriers=[],correctionClock=[0.,0.,0.]),
        comTarget=receipt['com']['position'])
    entry=dict(sourceStep=0,sourceTimeS=0.,com=[0.,0.,.8],qpos=[0.,0.,.8,1.,0.,0.,0.]+[0.]*21,
        massKg=10.,feet=[dict(position=[0.,.1,.027],loadN=50.,clearance=0.),
            dict(position=[0.,-.1,.027],loadN=50.,clearance=0.)])
    rows=[]
    for n in range(1,5):
        row=copy.deepcopy(entry);row.update(sourceStep=n,sourceTimeS=n*.00125,stepIndex=1,
            phase='transfer',actualMotorTorquesNm=[0.]*21)
        rows.append(row)
    return dict(receipt=receipt,note=note,row=entry,reference=serial(config),rows=rows)


def original_fixture(count):
    checks=dict.fromkeys(REQUIRED_STRICT_CHECKS,True)
    checks[ORIGINAL_IDENTITY]=checks[UNLOAD_CHECK]=False
    return dict(requestedSteps=count,requestedDistanceM=.18,strictCadencedSequencePass=False,
        tenStep180mmQualificationPass=False,measured=dict(checks=checks))


class ReferenceReplay(unittest.TestCase):
    def setUp(self): self.data=fixture()
    def result(self):
        return audit_context(self.data['receipt'],self.data['note'],self.data['row'],self.data['reference'],
            expected_elapsed=0.,expected_q0=self.data['reference']['q0'],expected_floor=self.data['reference']['floorTargets'])
    def rejects(self,key):
        result=self.result();self.assertFalse(result['passed']);self.assertFalse(result['checks'][key])
    def test_independent_baseline_context_passes(self):
        result=self.result();self.assertTrue(result['passed'],result['failedChecks'])
    def test_digest_detects_unrehashable_mutation(self):
        self.data['receipt']['com']['position'][0]+=.001
        self.rejects('contextDigest')
    def test_rehashed_orientation_mutation_rejected(self):
        self.data['receipt']['orientations'][0]['rotation'][0]+=.001;rehash(self.data['receipt'])
        self.rejects('independentOrientationJets')
    def test_rehashed_knee_mutation_rejected(self):
        self.data['receipt']['joints']['position'][6]+=.001;rehash(self.data['receipt'])
        self.rejects('independentJointAndKneeJets')
    def test_rehashed_com_velocity_mutation_rejected(self):
        self.data['receipt']['com']['velocity'][0]+=.001;rehash(self.data['receipt'])
        self.rejects('independentCOMJets')
    def test_rehashed_foot_acceleration_mutation_rejected(self):
        self.data['receipt']['feet'][0]['translation']['acceleration'][0]+=.001;rehash(self.data['receipt'])
        self.rejects('independentFootJets')
    def test_rehashed_clock_mutation_rejected(self):
        self.data['receipt']['elapsed']=.005;rehash(self.data['receipt'])
        self.rejects('contextBoundToPrecedingNativeSample')
    def test_rehashed_source_counter_mutation_rejected(self):
        self.data['receipt']['source_step']=4;rehash(self.data['receipt'])
        self.rejects('contextBoundToPrecedingNativeSample')
    def test_rehashed_reference_config_cannot_change_saved_orientation(self):
        self.data['receipt']['config']['lean_axis'][0]+=.01;rehash(self.data['receipt'])
        self.rejects('staticReferencesBoundToSavedStep')
    def test_rehashed_leg_origin_mutation_rejected(self):
        self.data['receipt']['causal']['persistentLegOrigin'][0]=.1;rehash(self.data['receipt'])
        self.rejects('persistentArmOriginBound')
    def test_causal_history_cannot_start_in_future(self):
        self.data['receipt']['causal']['arm']['anchor']['previousTimeS']=.005;rehash(self.data['receipt'])
        self.rejects('causalDelayedArmHistory')
    def test_actual_context_cannot_smuggle_hypothetical_change(self):
        self.data['receipt']['hypothetical_changes']=['other'];rehash(self.data['receipt'])
        self.rejects('exactDeclaredHypotheticalScope')
    def test_unknown_context_field_rejected_even_after_rehash(self):
        self.data['receipt']['extraReference']=True;rehash(self.data['receipt'])
        self.rejects('exactReferenceSchema')


class SharedProposalBinding(unittest.TestCase):
    def test_rehashed_task_mutations_rejected_before_any_native_access(self):
        for field in ('com','joints','arms','orientations','elapsed','causal'):
            with self.subTest(field=field):
                d=fixture();proposed=copy.deepcopy(d['receipt'])
                proposed['unload_admitted']=True
                proposed['hypothetical_changes']=['unload-normal-force-zero','swing-transfer-floor-plus-0.1mm']
                if field in ('com','joints','arms'):proposed[field]['position'][0]+=.001
                elif field=='orientations':proposed[field][0]['rotation'][0]+=.001
                elif field=='elapsed':proposed[field]+=.005
                else:proposed[field]['arm']['nativeSourceStep']+=4
                rehash(proposed)
                forecast=dict(role='causal-observer',probeControllerInput=dict(fields={},referenceContext=proposed,
                    coreEntryPoint='Controller.solve with immutable reviewed reference context; no preparation',
                    phase='transfer',elapsedTransferS=0.))
                replay=ExactReviewedForecastReplay.__new__(ExactReviewedForecastReplay)
                replay.manifests={1:dict(selectedAtTransferS=.1,samples=[dict(sourceStep=80)])}
                # An object with no native API establishes the failure happens
                # before loading/importing/accessing any native world.
                with self.assertRaises(ValueError):replay._probe(d['note'],d['row'],d['reference'],forecast,object())


class HeldCommandReplay(unittest.TestCase):
    def setUp(self): self.data=fixture()
    def result(self):
        d=self.data
        return audit_reference_journal(d['rows'],d['row'],{1:d['reference']},[d['note']],gears=np.ones(21))
    def test_all_native_rows_bound_to_held_command(self):
        result=self.result();self.assertTrue(result['passed'],result['failedChecks'])
        self.assertEqual(result['actualRecordedIntegrations'],4)
    def test_handoff_final_height_uses_measured_entry_not_carried_desired_com(self):
        d=self.data;d['row']['com'][2]=.799
        d['receipt']['config']['finalCom'][2]=.799;rehash(d['receipt'])
        result=self.result();self.assertTrue(result['passed'],result['failedChecks'])
    def test_handoff_final_height_cannot_be_replaced_by_carried_desired_com(self):
        self.data['row']['com'][2]=.799
        result=self.result();self.assertFalse(result['checks']['floorAndFinalStateBound'])
    def test_rehashed_actual_command_mutation_rejected_by_native_torque(self):
        self.data['note']['actualMotorControl'][0]=.1
        self.data['note']['actualMotorControlSHA256']=command_sha(self.data['note']['actualMotorControl'])
        result=self.result();self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['everyNativeTorqueMatchesHeldActualCommand'])
    def test_actual_command_hash_mutation_rejected(self):
        self.data['note']['actualMotorControlSHA256']='b'*64
        self.assertFalse(self.result()['checks']['actualCommandDigestAndMotorBounds'])
    def test_missing_assembly_digest_rejected(self):
        del self.data['note']['qpAssemblySHA256']['A']
        self.assertFalse(self.result()['checks']['completeActualQPAssemblyDigests'])
    def test_forged_correction_clock_rejected(self):
        self.data['receipt']['causal']['robustCorrection']['clock']=[0.,1.,0.];rehash(self.data['receipt'])
        self.assertFalse(self.result()['checks']['independentCorrectionClock'])
    def test_missing_control_note_rejected(self):
        d=self.data
        result=audit_reference_journal(d['rows'],d['row'],{1:d['reference']},[],gears=np.ones(21))
        self.assertFalse(result['passed'])


def wrench_fixture():
    import source_solved_wrench_diagnostics as producer
    d=fixture(); binding=dict(helperSHA256='a'*64,modelSHA256='b'*64,
        accessor='lh_last_ground_wrench',sideOrder=['left','right'])
    for row in d['rows']:
        for side,speed in enumerate((.002,.004)):row['feet'][side]['loadedSlipSpeedMps']=speed
        row['loadedSlipDistanceM']=(np.array([.002,.004])*row['sourceStep']*.00125).tolist()
        row['jointLimitMarginsRad']=[.1]*21;row['jointLimitMinimumMarginRad']=.1
        w=SimpleNamespace(h=None)
        w.l=SimpleNamespace(lh_dim=lambda h,k,n=row['sourceStep']:n,
            lh_time=lambda h,t=row['sourceTimeS']:t,lh_dt=lambda h:.00125,lh_last_ground_wrench=lambda *args:None)
        w.arr=lambda key,shape:np.zeros(21)
        w.friction_parameters=lambda:dict(effectiveFootGroundContacts=[[7.,0.,1.,3.,1.,1.,.005,.0001,0.]])
        w.read=lambda name,shape:np.array([[3.,-4.,50.,-1.,2.,-3.],[-12.,5.,50.,4.,-5.,6.]])
        with patch.object(producer,'_bindings',return_value=binding):
            row['solvedWrenchDiagnostics']=producer.reader(w,row)
    d['bindings']=binding
    return d


class WrenchSavedDataReplay(unittest.TestCase):
    def setUp(self):self.d=wrench_fixture()
    def result(self):
        row=self.d['rows'][0]
        return audit_wrench_sample(row['solvedWrenchDiagnostics'],row,self.d['bindings'],command_sha(np.zeros(21)),[0.,0.])
    def rejects(self,key):
        r=self.result();self.assertFalse(r['passed']);self.assertFalse(r['checks'][key])
    def test_live_reader_record_has_independent_arithmetic_and_identity(self):
        r=self.result();self.assertTrue(r['passed'],r['checks'])
        self.assertEqual(r['values']['netShearForceN'],[5.,13.])
    def test_wrench_cannot_bind_another_held_command(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['heldMotorControlSHA256']='c'*64
        self.rejects('wrenchActualHeldCommandBinding')
    def test_same_load_wrong_helper_or_model_rejected(self):
        for field in ('helperSHA256','modelSHA256'):
            with self.subTest(field=field):
                d=copy.deepcopy(self.d['rows'][0]['solvedWrenchDiagnostics']);d['bindings'][field]='c'*64
                self.assertFalse(audit_wrench_sample(d,self.d['rows'][0],self.d['bindings'],command_sha(np.zeros(21)),[0.,0.])['passed'])
    def test_solved_and_transported_geometry_stages_cannot_be_conflated(self):
        d=self.d['rows'][0]['solvedWrenchDiagnostics'];d['solvedStageSourceTimeS']=d['sourceTimeS']
        self.rejects('wrenchSampleNativeIdentity')
    def test_force_normal_disagreement_rejected(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['feet'][0]['forceWorldSourceN'][2]+=.0001
        self.rejects('wrenchNormalForceRowLoadAgreement:left')
    def test_net_shear_forgery_rejected(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['feet'][0]['netShearForceN']=4.
        self.rejects('wrenchNetShearArithmetic:left')
    def test_net_ratio_forgery_rejected(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['feet'][0]['netShearUtilizationLowerBound']=.01
        self.rejects('wrenchNetUtilizationLowerBoundArithmetic:left')
    def test_finite_forces_cannot_hide_nan_moment(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['feet'][1]['momentWorldSourceNmAboutSolvedFootOrigin'][0]=float('nan')
        self.rejects('completeFiniteWrenchDiagnostic')
    def test_global_slip_reset_rejected_even_when_diagnostic_increment_looks_valid(self):
        self.d['rows'][0]['loadedSlipDistanceM']=[0.,0.]
        self.rejects('wrenchTransportedAndGlobalSlipIncrement:left')
    def test_small_net_ratio_never_establishes_per_contact_nonsaturation(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['individualContactNonsaturationEstablished']=True
        self.rejects('wrenchExactStageAndLowerBoundScope')
    def test_undeclared_extra_safety_claim_rejected(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['allContactsUnsaturated']=True
        self.rejects('wrenchExactReaderSchema')
    def test_rehashed_command_and_wrench_tag_still_cannot_change_native_torque(self):
        d=self.d;d['note']['actualMotorControl'][0]=.1
        digest=command_sha(d['note']['actualMotorControl']);d['note']['actualMotorControlSHA256']=digest
        for row in d['rows']:row['solvedWrenchDiagnostics']['heldMotorControlSHA256']=digest
        self.assertFalse(self.journal()['checks']['everyNativeTorqueMatchesHeldActualCommand'])
    def test_joint_margin_summary_is_bound_to_executed_xml_limits(self):
        d=self.d
        r=audit_reference_journal(d['rows'],d['row'],{1:d['reference']},[d['note']],gears=np.ones(21),
            limits=np.repeat([[-1.,1.]],21,axis=0),wrench_enabled=True,wrench_bindings=d['bindings'])
        self.assertFalse(r['checks']['wrenchSummaryJointMarginsMatchExecutedXML'])
    def test_native_timestep_assertion_required(self):
        self.d['rows'][0]['solvedWrenchDiagnostics']['nativeTimeStepSVerified']=False
        self.rejects('wrenchSampleNativeIdentity')
    def test_wrong_side_order_rejected(self):
        d=self.d['rows'][0]['solvedWrenchDiagnostics'];d['feet'].reverse()
        self.rejects('wrenchFootIdentity:left')
    def test_below_threshold_utilization_is_undefined(self):
        row=self.d['rows'][0];d=row['solvedWrenchDiagnostics'];row['feet'][0]['loadN']=2.
        d['feet'][0].update(forceWorldSourceN=[3.,-4.,2.],normalForceN=2.,rowLoadN=2.,normalForceRowErrorN=0.,
            loadedForUtilization=False,netShearUtilizationLowerBound=None,utilizationUndefinedReason='below-3N-load')
        self.assertTrue(self.result()['passed'])
    def test_missing_enabled_row_fails_complete_coverage(self):
        del self.d['rows'][1]['solvedWrenchDiagnostics']
        r=self.journal();self.assertFalse(r['passed']);self.assertFalse(r['checks']['completeEnabledWrenchDiagnosticCoverage'])
    def journal(self,enabled=True):
        d=self.d
        return audit_reference_journal(d['rows'],d['row'],{1:d['reference']},[d['note']],
            gears=np.ones(21),wrench_enabled=enabled,wrench_bindings=d['bindings'])
    def test_complete_enabled_native_coverage_passes(self):
        r=self.journal();self.assertTrue(r['passed'],r['failedChecks'])
        self.assertTrue(r['wrenchDiagnostics']['coveragePass']);self.assertEqual(r['wrenchDiagnostics']['verifiedNativeSamples'],4)
        self.assertFalse(r['wrenchDiagnostics']['individualContactNonsaturationEstablished'])
    def test_old_absent_field_pilot_has_explicitly_unavailable_coverage(self):
        for row in self.d['rows']:del row['solvedWrenchDiagnostics']
        r=self.journal(enabled=False);self.assertTrue(r['passed'],r['failedChecks'])
        self.assertFalse(r['wrenchDiagnostics']['available']);self.assertIsNone(r['wrenchDiagnostics']['coveragePass'])
    def summary(self):
        r=self.journal();phase=copy.deepcopy(r['wrenchDiagnostics']['perStepPhase']['1:transfer'])
        phase['maximumNetShearUtilizationLowerBound']=phase.pop('maximumNetUtilizationLowerBound')
        summary=dict(actualIntegrations=4,stopped='done',globalLoadedSlipM=self.d['rows'][-1]['loadedSlipDistanceM'])
        saved=dict(schema=1,nativeSamples=4,actualIntegrations=4,sourceStepLast=4,allNativeSamplesCovered=True,
            physicsStepS=.00125,stopped='done',footSideOrder=['left','right'],
            jointOrder='source qpos21; actuator-order permutation removed with B',
            thresholds=dict(nearMotorFraction=.925,nearJointMarginRad=.02,nearNetShearUtilizationLowerBound=.9),
            individualContactNonsaturationEstablished=False,
            forceStage='actual cached mj_step2 solve before integration; current geometry and material-point slip remain separately identified',
            perStepPhase={'1:transfer':phase},globalLoadedSlipM=summary['globalLoadedSlipM'])
        return saved,r,summary
    def test_phase_summary_independently_rebuilt(self):
        saved,r,summary=self.summary();proof=audit_wrench_summary(saved,r,summary)
        self.assertTrue(proof['passed'],proof['failedChecks'])
    def test_near_motor_or_joint_limit_summary_forgery_rejected(self):
        for field in ('nearMotorFractionS','nearJointLimitS','netShearImpulseNs','normalImpulseNs','loadedSlipM'):
            with self.subTest(field=field):
                saved,r,summary=self.summary();saved['perStepPhase']['1:transfer'][field][0]+=.01
                self.assertFalse(audit_wrench_summary(saved,r,summary)['passed'])
    def test_summary_cannot_reinterpret_lower_bound_as_nonsaturation(self):
        saved,r,summary=self.summary();saved['individualContactNonsaturationEstablished']=True
        self.assertFalse(audit_wrench_summary(saved,r,summary)['passed'])
    def test_summary_thresholds_cannot_be_relaxed(self):
        saved,r,summary=self.summary();saved['thresholds']['nearMotorFraction']=.95
        self.assertFalse(audit_wrench_summary(saved,r,summary)['passed'])


class ProfileQualification(unittest.TestCase):
    def result(self,count,**kwargs):
        good=dict(passed=True)
        return qualify_reviewed(original_fixture(count),good,good,dict(passed=True,expectedImpratio=8.),
            True,good,good,count,**kwargs)
    def test_one_step_is_only_pilot(self):
        r=self.result(1);self.assertTrue(r['reviewedWalkPilotPhysicalPass'])
        self.assertFalse(r['reviewedWalkTenStepPhysicalPass']);self.assertFalse(r['reviewedWalkFinalPlanPass'])
    def test_two_and_seven_are_diagnostic_only(self):
        for count in (2,7):
            r=self.result(count);self.assertTrue(r['reviewedWalkDiagnosticPass'])
            for key in ('reviewedWalkPilotPhysicalPass','reviewedWalkTenStepPhysicalPass','reviewedWalkFinalPlanPass',*OLD_FLAGS):
                self.assertIs(r[key],False,key)
    def test_ten_steps_cannot_establish_visual_improvement(self):
        r=self.result(10);self.assertTrue(r['reviewedWalkTenStepPhysicalPass']);self.assertFalse(r['reviewedWalkFinalPlanPass'])
    def test_baseline_context_never_final_even_with_claimed_improvement(self):
        improvement=dict.fromkeys(('independentlyAudited','sameContactCausalComparisonPass','deliveredVideoComparisonPass',
            'physicallyFasterPass','visuallyImprovedPass','allIntervalsAndMatchedPhasesPublished','physicalTime1xVideoVerified'),True)
        self.assertFalse(self.result(10,improvements=improvement)['reviewedWalkFinalPlanPass'])
    def test_unsupported_counts_and_bool_rejected(self):
        for count in (0,3,6,11,True):
            r=self.result(count);self.assertFalse(any(r[key] for key in
                ('reviewedWalkDiagnosticPass','reviewedWalkPilotPhysicalPass','reviewedWalkTenStepPhysicalPass','reviewedWalkFinalPlanPass')))
    def test_missing_physical_or_arm_gate_fails_closed(self):
        good=dict(passed=True);original=original_fixture(1)
        del original['measured']['checks']['independentLimbFKAndExactOpposition']
        r=qualify_reviewed(original,good,good,dict(passed=True,expectedImpratio=8.),True,good,good,1)
        self.assertFalse(r['reviewedWalkPilotPhysicalPass'])
    def test_every_original_unreplaced_failure_remains_failure(self):
        good=dict(passed=True);original=original_fixture(1)
        original['measured']['checks']['originalMotorLimit']=False
        r=qualify_reviewed(original,good,good,dict(passed=True,expectedImpratio=8.),True,good,good,1)
        self.assertFalse(r['reviewedWalkPilotPhysicalPass'])
    def test_incomplete_support_replay_cannot_pass(self):
        self.assertFalse(reviewed_robust_result(dict(checks={'nonemptyReplay':True}),2,dict(passed=True))['passed'])
    def test_legacy_support_output_retained_unmodified(self):
        legacy=dict(passed=False,checks=dict(geometricRiskSignalPolicyOnly=False,explicitPilotOrTenRequest=False,
            emittedCOMTargetMatchesBoundedCorrection=False,unchangedEveryNativeActualHullCaptureGuard=True))
        legacy['checks'].update(dict.fromkeys(REQUIRED_ROBUST_REPLAY_CHECKS,True))
        before=copy.deepcopy(legacy);r=reviewed_robust_result(legacy,2,dict(passed=True))
        self.assertEqual(before,legacy);self.assertTrue(r['passed']);self.assertFalse(r['oldProfileQualificationClaim'])
        legacy['checks']['unchangedEveryNativeActualHullCaptureGuard']=False
        self.assertFalse(reviewed_robust_result(legacy,2,dict(passed=True))['passed'])


class ContinuousSupportReplay(unittest.TestCase):
    def setUp(self):
        from scipy.spatial import ConvexHull
        import source_continuous_com as producer
        from audit_source_robust_support import support_observation, triangle
        from test_source_robust_support_audit import fixture as support_fixture
        data=support_fixture();self.row=data['entry'];self.row['sourceTimeS']=.5
        self.reference=fixture()['reference'];self.reference.update(com1=[0.,0.,.8],heading=[1.,0.,0.])
        observation=support_observation(self.row,1)
        equations=np.vstack([ConvexHull(np.asarray(self.row['supportPointsXY'])).equations,triangle(observation,[1,2,3],actual=True)])
        delta=np.r_[.004*np.array([1.,5.])/np.sqrt(26.),0.]
        c=SimpleNamespace(com1=np.array([0.,0.,.8]),_robust_delta=delta,heading=np.array([1.,0.,0.]),
            phase_duration=lambda p:dict(translate=1.2,land=1.)[p],w=SimpleNamespace(h=None,l=SimpleNamespace(lh_time=lambda h:.5)))
        with patch.object(producer,'actual_equations',return_value=equations):
            path,plan=producer.plan_forward(c,0.)
            offset=path.evaluate(.5);total=(c.com1+delta+offset[0],offset[1],offset[2])
            diagnostic=producer.validate_reference(c,'translate',total)
        self.receipt=fixture()['receipt'];r=self.receipt
        r.update(phase='translate',elapsed=.5,source_time=.5)
        r['config'].update(mode='continuous-com',com1=c.com1.tolist(),heading=c.heading.tolist())
        r['causal']['robustCorrection'].update(selected=True,deltaM=delta.tolist(),clock=[1.,0.,0.])
        r['causal']['continuousCom']=dict(path=path.receipt(),settle=None,plan=plan,diagnostic=diagnostic)
        r['com']=dict(zip(('position','velocity','acceleration'),[x.tolist() for x in total]))
        self.state=dict(continuousPath=None,continuousSettle=None,phaseStarts={'translate':0.},support=dict(survivingCorners=[1,2,3]))
    def result(self):return audit_continuous_com(self.receipt,self.row,self.reference,self.state,.5)
    def test_independent_path_selection_support_and_total_envelope_pass(self):
        r=self.result();self.assertTrue(r['passed'],r['checks'])
    def test_polynomial_mutation_fails_even_with_rehashed_context(self):
        self.receipt['causal']['continuousCom']['path']['coefficients'][3][0]+=.001;rehash(self.receipt)
        self.assertFalse(self.result()['passed'])
    def test_plan_clock_cannot_be_changed(self):
        self.receipt['causal']['continuousCom']['path']['startSourceTimeS']+=.01;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['continuousPathClockFromMeasuredPhase'])
    def test_frozen_total_speed_envelope_cannot_be_widened(self):
        self.receipt['causal']['continuousCom']['diagnostic']['maximumTotalSpeedMps']=1.;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['continuousTotalJetEnvelopeNeverWidened'])
    def test_changed_current_support_rejects_existing_path(self):
        self.row['supportPointsXY']=[[x+.1,y]for x,y in self.row['supportPointsXY']]
        self.assertFalse(self.result()['passed'])


class SourceBinding(unittest.TestCase):
    def test_source_start_end_and_actual_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);hashes={}
            for name in REQUIRED_SOURCES:
                (root/name).write_text('# frozen '+name+'\n')
                import hashlib
                hashes[name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
            receipt=dict(codeSHA256AtStart=hashes,codeSHA256AtEnd=hashes.copy(),codeUnchangedDuringRun=True)
            self.assertTrue(audit_source_provenance(receipt,root)['passed'])
            receipt['codeSHA256AtEnd']['source_reference_context.py']='a'*64
            self.assertFalse(audit_source_provenance(receipt,root)['passed'])
            receipt['codeSHA256AtEnd']=hashes.copy();(root/'source_step_probe.py').write_text('# changed')
            self.assertFalse(audit_source_provenance(receipt,root)['passed'])
    def test_enabled_wrench_reader_source_cannot_be_omitted(self):
        proof=audit_source_provenance(dict(codeSHA256AtStart={},codeSHA256AtEnd={},codeUnchangedDuringRun=True,
            wrenchDiagnosticsEnabled=True),Path('.'))
        self.assertFalse(proof['checks']['requiredSolvedWrenchReaderSourcePresent'])
    def test_missing_core_module_cannot_be_omitted_from_hashes(self):
        self.assertFalse(audit_source_provenance(dict(codeSHA256AtStart={},codeSHA256AtEnd={},
            codeUnchangedDuringRun=True),Path('.'))['passed'])


class SelectedNativeRangeBinding(unittest.TestCase):
    def test_executed_xml_ranges_override_older_presentation_limits(self):
        model=dict(segments=[dict(reference=dict(hinge=dict(name='joint'+str(i),qposIndex=i))) for i in range(21)])
        xml='<mujoco><worldbody><body>'+''.join('<joint name="joint'+str(i)+'" range="-35 15"/>' for i in range(21))+'</body></worldbody></mujoco>'
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'selected.xml';path.write_text(xml)
            limits=limits_from_bound_xml(path,model)
            np.testing.assert_allclose(limits,np.repeat(np.deg2rad([[-35.,15.]]),21,axis=0))
    def test_absent_initial_history_is_equal_without_coercing_it_to_nan(self):
        self.assertTrue(ExactReviewedForecastReplay._equal({'previous_time':None,'previous_leg_rate':None},
            {'previous_time':None,'previous_leg_rate':None}))
        self.assertFalse(ExactReviewedForecastReplay._equal({'previous_time':None},{'previous_time':0.}))


class PolynomialReconstruction(unittest.TestCase):
    def test_hermite_endpoints_and_derivatives(self):
        initial=(np.array([.1,.2,.8]),np.array([.01,-.02,0.]),np.array([.1,.02,0.]))
        final=(np.array([.2,.1,.8]),np.array([-.01,.02,0.]),np.array([-.1,-.02,0.]))
        coefficients=hermite_coefficients(initial,final,1.52)
        segment=dict(startSourceTimeS=10.,durationS=1.52,coefficients=coefficients.tolist())
        for a,b in zip(segment_jet(segment,10.),initial):np.testing.assert_allclose(a,b,atol=1e-14)
        for a,b in zip(segment_jet(segment,11.52),final):np.testing.assert_allclose(a,b,atol=1e-14)
    def test_bad_polynomial_shape_and_clock_rejected(self):
        for segment in (dict(startSourceTimeS=0.,durationS=0.,coefficients=np.zeros((6,3)).tolist()),
            dict(startSourceTimeS=0.,durationS=1.,coefficients=np.zeros((5,3)).tolist())):
            with self.assertRaises(ValueError):segment_jet(segment,.5)


if __name__=='__main__':unittest.main()
