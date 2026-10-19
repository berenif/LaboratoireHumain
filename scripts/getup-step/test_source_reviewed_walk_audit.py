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
    audit_soft_load_context, independent_soft_load_border,
    hermite_coefficients, independent_references, independent_endpoint_projection, projected_capture_extrema,
    limits_from_bound_xml, jet_matches, qualify_reviewed,
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
    def test_soft_load_cannot_qualify_without_active_settle_native_witnesses(self):
        r=self.result(1,mode='continuous-com-load')
        self.assertFalse(r['reviewedWalkPilotPhysicalPass'])
        self.assertFalse(r['checks']['activeSoftLoadNativeAssemblyAndCommandReplay'])
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


class SoftLoadReplay(unittest.TestCase):
    def fixture(self,elapsed=.5):
        from source_soft_load_preference import AcquiredLoad, prepare_preference
        d=fixture();r=d['receipt'];anchor=AcquiredLoad(10,.0125,1,0,31.,100.,80)
        r.update(phase='settle',elapsed=elapsed,source_step=10+round(elapsed/.00125),source_time=.0125+elapsed)
        r['config'].update(mode='continuous-com-load',softLoadPreference=prepare_preference(
            anchor,'settle',elapsed,r['source_step'],r['source_time']).receipt())
        r['causal']['softLoadAcquisition']=anchor.receipt()
        note=d['note'];note['predictedForcesN']=[[0.,0.,20.]]*8
        note['softLoadHinge']=dict(policy='retained-acquired-load-hinge-v1',baseQPWidth=72,augmentedQPWidth=73,
            slackN=0.,targetNormalN=r['config']['softLoadPreference']['floor_jet'][0],predictedNewFootNormalN=80.,
            weightPerNewton=.0005,hardForceFloor=False)
        return r,note,anchor.receipt()
    def test_causal_floor_jets_at_activation_fade_and_exact_endpoints(self):
        for elapsed in (0.,.05,.1,.5,1.52,2.):
            r,n,a=self.fixture(elapsed);proof=audit_soft_load_context(r,n,a)
            self.assertTrue(proof['passed'],proof['checks'])
    def test_replaced_measured_anchor_rejected(self):
        r,n,a=self.fixture();r['causal']['softLoadAcquisition']['measured_normal_n']+=1.
        self.assertFalse(audit_soft_load_context(r,n,a)['checks']['softLoadAnchorIndependentlyMeasuredAndNeverReplaced'])
    def test_missing_external_handoff_binding_fails_closed(self):
        r,n,a=self.fixture();self.assertFalse(audit_soft_load_context(r,n)['passed'])
    def test_counter_clock_or_stage_clock_mutation_rejected(self):
        for key in ('source_step','source_time'):
            r,n,a=self.fixture();r[key]+=.00125 if key=='source_time' else 1
            self.assertFalse(audit_soft_load_context(r,n,a)['checks']['softLoadMeasuredHandoffClock'])
    def test_floor_weight_and_boolean_floor_mutations_rejected(self):
        for key,value in (('weight',.001),('floor_jet',[True,0.,0.]),('floor_jet',[1.,0.,0.])):
            r,n,a=self.fixture();r['config']['softLoadPreference'][key]=value
            self.assertFalse(audit_soft_load_context(r,n,a)['checks']['softLoadIndependentC2FloorAndWeight'])
    def test_predicted_force_slice_and_slack_constraints_verified(self):
        for key,value in (('predictedNewFootNormalN',79.),('slackN',-1.),('baseQPWidth',73),('hardForceFloor',True)):
            r,n,a=self.fixture();n['softLoadHinge'][key]=value
            self.assertFalse(audit_soft_load_context(r,n,a)['checks']['softLoadOneSlackMetadataAndPredictedForce'])
    def test_pref_cannot_exist_before_landing_or_in_other_modes(self):
        r,n,a=self.fixture();r['phase']='transfer'
        self.assertFalse(audit_soft_load_context(r,n,None)['passed'])
        r,n,a=self.fixture();r['config']['mode']='continuous-com'
        self.assertFalse(audit_soft_load_context(r,n,a)['passed'])
    def test_retained_anchor_but_no_preference_in_stand(self):
        r,n,a=self.fixture();r['phase']='stand';r['config']['softLoadPreference']=None;n.pop('softLoadHinge')
        self.assertTrue(audit_soft_load_context(r,n,a)['passed'])
    def test_independent_qp_border_preserves_every_original_cost_and_row(self):
        import source_soft_load_preference as producer
        r,n,a=self.fixture();preference=producer.SoftLoadPreference.from_receipt(r['config']['softLoadPreference'])
        rng=np.random.default_rng(14);raw=rng.normal(size=(72,72));P=raw.T@raw;q=rng.normal(size=72)
        A=rng.normal(size=(13,72));b=rng.normal(size=13);indices=(50,53,56,59)
        expected=independent_soft_load_border(P,q,A,b,indices,preference.floor_jet[0])
        actual=producer.extend_qp(P,q,A,b,indices,preference)
        for x,y in zip(expected,actual):np.testing.assert_array_equal(x,y)
        np.testing.assert_array_equal(actual[0][:72,:72],P);np.testing.assert_array_equal(actual[2][:-2,:72],A)
        for original,unchanged in zip((P,q,A,b),producer.extend_qp(P,q,A,b,indices,None)):
            self.assertIs(original,unchanged)
    def journal_transition(self,prefix=0,land_samples=801):
        d=fixture();rows=[];notes=[];captured=[];handoff=prefix+land_samples
        for source in range(1,handoff+5):
            row=copy.deepcopy(d['row']);phase='translate' if source<=prefix else 'land' if source<=handoff else 'settle'
            start=0 if phase=='translate' else prefix if phase=='land' else handoff
            row.update(sourceStep=source,sourceTimeS=source*.00125,stepIndex=1,phase=phase,
                phaseElapsedS=(source-start)*.00125,actualMotorTorquesNm=[0.]*21)
            row['feet'][0].update(position=[.18,.1,.027],loadN=50. if source-prefix>=722 else 0.)
            rows.append(row)
            n=source-1
            if n%4==0:
                note=copy.deepcopy(d['note']);note.update(t=n*.00125,phase=phase)
                note['reviewedWalk']['mode']='continuous-com-load';r=note['referenceContext']
                r.update(phase=phase,source_step=n,source_time=n*.00125,elapsed=(n-start)*.00125)
                r['config']['mode']='continuous-com-load'
                before=None if n==0 else (n-4)*.00125
                r['causal']['contactSeekBefore']['timeS']=before;r['causal']['contactSeekAfter']['timeS']=n*.00125
                notes.append(note)
        def capture(receipt,note,row,reference,**kwargs):
            if receipt['phase']=='settle':captured.append(kwargs['expected_load_anchor'])
            return dict(checks={})
        with patch('audit_source_reviewed_walk.audit_context',side_effect=capture),patch(
                'audit_source_reviewed_walk.audit_continuous_com',return_value=dict(checks={})):
            result=audit_reference_journal(rows,d['row'],{1:d['reference']},notes,mode='continuous-com-load',gears=np.ones(21))
        return result,captured
    def test_streamed_handoff_anchor_precedes_first_command_at_all_offsets(self):
        for prefix in (0,1,2,7):
            result,anchors=self.journal_transition(prefix)
            self.assertTrue(result['checks']['softLoadOriginalQualifiedLandingHandoff'])
            self.assertTrue(result['checks']['completeReferenceCommandSchedule'])
            self.assertTrue(anchors)
            self.assertEqual(anchors[0],dict(source_step=prefix+801,source_time=(prefix+801)*.00125,
                step_index=1,new_side=0,measured_normal_n=50.,body_weight_n=98.10000000000001,qualified_landing_samples=80))
    def test_source_float_phase_comparison_is_replayed_without_early_tolerance(self):
        # At prefix 3, 803*DT-3*DT is just below 1.0 in source arithmetic.
        self.assertLess(803*.00125-3*.00125,1.)
        result,anchors=self.journal_transition(3,802)
        self.assertTrue(result['checks']['softLoadOriginalQualifiedLandingHandoff'])
        self.assertEqual(anchors[0]['source_step'],805)
        self.assertEqual(anchors[0]['qualified_landing_samples'],81)
    def test_streamed_eligible_landing_cannot_be_delayed_and_reanchored(self):
        result,anchors=self.journal_transition(0,883)
        self.assertFalse(result['checks']['softLoadOriginalQualifiedLandingHandoff'])
        self.assertFalse(result['checks']['softLoadFirstEligibleLandingHandoffNeverDelayed'])


class IndependentProjectionMath(unittest.TestCase):
    def project(self, desired, A, b, radius):
        return independent_endpoint_projection(desired,np.asarray(A).reshape(-1,2),b,radius)
    def test_closed_form_capture_and_frozen_radius_constants(self):
        critical,maximum=projected_capture_extrema(.8,2.2)
        self.assertAlmostEqual(critical,.8037714552054018,14)
        self.assertAlmostEqual(maximum,1.0418076078276193,14)
    def test_interior_and_zero_goal(self):
        np.testing.assert_allclose(self.project([.2,.3],[],[],1.),[.2,.3])
        np.testing.assert_allclose(self.project([0.,0.],[],[],1.),[0.,0.])
    def test_single_line_projection(self):
        np.testing.assert_allclose(self.project([2.,.4],[[1.,0.]],[.5],2.),[.5,.4])
    def test_two_line_vertex(self):
        np.testing.assert_allclose(self.project([2.,2.],[[1.,0.],[0.,1.]],[.5,.25],5.),[.5,.25])
    def test_circle_only_radial_projection(self):
        np.testing.assert_allclose(self.project([2.,0.],[],[],1.),[1.,0.])
    def test_line_circle_intersection(self):
        np.testing.assert_allclose(self.project([2.,2.],[[1.,0.]],[.5],1.),[.5,np.sqrt(.75)])
    def test_duplicate_scaled_and_near_parallel_constraints(self):
        np.testing.assert_allclose(self.project([2.,2.],[[1.,0.],[1000.,0.],[0.,1.]],[.5,500.,.25],5.),[.5,.25])
        np.testing.assert_allclose(self.project([2.,.4],[[1e-14,0.]],[5e-15],5.),[.5,.4])
        np.testing.assert_allclose(self.project([2.,2.],[[1.,0.],[1.,1e-10]],[.5,.5],5.),[.5-2e-10,2.],atol=1e-9)
    def test_tangent_line(self):
        np.testing.assert_allclose(self.project([2.,2.],[[0.,-1.]],[-1.],1.),[0.,1.],atol=1e-12)
    def test_infeasible_zero_normal_and_nonfinite_inputs_rejected(self):
        for A,b in (([[1.,0.],[-1.,0.]],[-2.,-2.]),([[0.,0.]],[-1.])):
            with self.assertRaises(ValueError):self.project([0.,0.],A,b,1.)
        with self.assertRaises(ValueError):self.project([np.nan,0.],[],[],1.)
        with self.assertRaises(ValueError):projected_capture_extrema(0.,2.2)


class ProjectedSupportReplay(ContinuousSupportReplay):
    def setUp(self):
        super().setUp()
        import source_continuous_com as producer
        c0=self.receipt['config'];delta=np.asarray(self.receipt['causal']['robustCorrection']['deltaM'])
        equations=np.asarray(self.receipt['causal']['continuousCom']['plan']['actualSupportEquations'])
        c=SimpleNamespace(com1=np.asarray(c0['com1']),_robust_delta=delta,heading=np.asarray(c0['heading']),
            landing=np.asarray(c0['landing']),targets=c0['targets'],stance_side=c0['stance_side'],
            phase_duration=lambda p:c0['phase_durations'][p],w=SimpleNamespace(h=None,l=SimpleNamespace(lh_time=lambda h:.5)))
        with patch.object(producer,'actual_equations',return_value=equations):
            path,plan=producer.plan_projected(c,0.);offset=path.evaluate(.5)
            total=(c.com1+delta+offset[0],offset[1],offset[2])
            diagnostic=producer.validate_reference(c,'translate',total)
        c0['mode']='continuous-com-projected'
        self.receipt['causal']['arm']['nativeSourceTimeS']=.5
        self.receipt['causal']['continuousCom']=dict(path=path.receipt(),settle=None,plan=plan,diagnostic=diagnostic)
        self.receipt['com']=dict(zip(('position','velocity','acceleration'),[x.tolist() for x in total]))
    def plan(self):return self.receipt['causal']['continuousCom']['plan']
    def test_projected_plan_replayed_without_importing_producer_projector(self):
        import source_continuous_com as producer
        with patch.object(producer,'project_endpoint',side_effect=AssertionError('producer projector called')):
            result=self.result();self.assertTrue(result['passed'],result['checks'])
    def test_rehashed_goal_mutation_rejected(self):
        self.plan()['plannedGoalSourceM'][0]+=.01;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedGoalUsesPlannedFeetAndCorrectedOrigin'])
    def test_adaptive_face_reserve_mutation_rejected(self):
        self.plan()['planningFaceReservesM'][0]+=.001;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedOriginAndAdaptiveFaceReserves'])
    def test_lateral_constraint_widening_rejected(self):
        self.plan()['projectionConstraintsB'][-1]=.02;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedHalfPlanesIndependentlyRebuilt'])
    def test_capture_overshoot_cannot_be_replaced_by_endpoint_only_factor(self):
        factors=self.plan()['wholePolynomialFactors'];factors[np.argmax(factors)]=1.;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedExactWholePolynomialFactors'])
    def test_missing_true_capture_extremum_rejected(self):
        xs=self.plan()['wholePolynomialExtremaU'];critical,_=projected_capture_extrema(.8,2.2)
        xs[min(range(len(xs)),key=lambda index:abs(xs[index]-critical))]=.5;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedExactWholePolynomialExtrema'])
    def test_feasible_but_suboptimal_endpoint_rejected(self):
        self.plan()['projectionLocalEndpointM']=[0.,0.];self.plan()['forwardM']=self.plan()['lateralM']=0.;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedClosestFeasibleEndpoint'])
    def test_displacement_envelope_radius_cannot_be_widened(self):
        self.plan()['maximumDisplacementM']+=.01;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['projectedClosestFeasibleEndpoint'])
    def test_selected_triangle_faces_cannot_be_omitted(self):
        self.plan()['actualSupportEquations']=self.plan()['actualSupportEquations'][:-3];rehash(self.receipt)
        self.assertFalse(self.result()['checks']['continuousPlanUsesCurrentActualSupport'])
    def test_plan_is_immutable_after_first_command(self):
        self.assertTrue(self.result()['passed'])
        self.plan()['plannedAtSourceTimeS']+=.01;rehash(self.receipt)
        self.assertFalse(self.result()['checks']['continuousPlanNeverReplaced'])
    def test_future_measured_settle_target_is_not_used_as_projected_goal(self):
        self.receipt['config']['finalCom']=[-10.,20.,.9]
        result=self.result();self.assertTrue(result['passed'],result['checks'])
    def test_projected_mode_independently_rebuilds_total_com_jets(self):
        expected=independent_references(self.receipt['config'],self.receipt['causal'],.5,'translate')
        self.assertTrue(jet_matches(self.receipt['com'],expected['com'],3))


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


from fractions import Fraction as F
import math
import unittest

import audit_source_reviewed_walk as overlap_audit


def zero_certificate():
    checks = {k: True for k in overlap_audit.CHECK_ORDER}
    return {
        'schema': 1, 'policy': overlap_audit.POLICY, 'method': overlap_audit.PRODUCER_METHOD,
        'nominal': {'deltaM': [0., 0., 0.], 'startS': 0., 'durationS': 1.},
        'correction': {'deltaM': [0., 0., 0.], 'startS': 0., 'durationS': 1.,
                       'magnitudeM': 0., 'maximumSpeedMps': 0.,
                       'maximumAccelerationMps2': 0.},
        'caps': dict(overlap_audit.CAPS),
        'maximumTotalSpeedMps': 0., 'maximumTotalAccelerationMps2': 0.,
        'conservativeMaximumTotalSpeedMps': 0.,
        'conservativeMaximumTotalAccelerationMps2': 0.,
        'maximumSpeedAtS': 1., 'maximumAccelerationAtS': 1.,
        'candidateCriticalTimesS': {'speed': [0., 1.], 'acceleration': [0., 1.]},
        'windows': [{'startS': 0., 'endS': 1., 'nominalActive': True,
                     'correctionActive': True,
                     'speedStationaryTimesS': [], 'speedStationaryRootCount': 0,
                     'speedSquaredDerivativeDegree': 0, 'speedUpperBound': 0.,
                     'accelerationStationaryTimesS': [], 'accelerationStationaryRootCount': 0,
                     'accelerationSquaredDerivativeDegree': 0, 'accelerationUpperBound': 0.}],
        'boundaryJets': [{'timeS': t, 'positionM': [0., 0., 0.],
                          'velocityMps': [0., 0., 0.], 'accelerationMps2': [0., 0., 0.]}
                         for t in (0., 1.)],
        'checks': checks, 'passed': True, 'rejections': [], 'unresolvedCapBounds': []}


def analytic_rejected_certificate():
    """Handwritten one-axis .1m/1s quintic fixture, no producer call."""
    cert = zero_certificate()
    proof = overlap_audit.independent_overlap_envelope((.1,0.,0.), 1., (0.,0.,0.), 0., 1.)
    cert['nominal']['deltaM'][0] = .1
    cert['boundaryJets'][1]['positionM'][0] = .1
    low, high = .5-math.sqrt(3)/6, .5+math.sqrt(3)/6
    cert.update(maximumTotalSpeedMps=.1875,
                maximumTotalAccelerationMps2=.1*10/math.sqrt(3),
                maximumSpeedAtS=.5, maximumAccelerationAtS=high,
                conservativeMaximumTotalSpeedMps=proof['maxima']['speed']['upper'],
                conservativeMaximumTotalAccelerationMps2=proof['maxima']['acceleration']['upper'])
    window = cert['windows'][0]
    window.update(speedStationaryTimesS=[.5], speedStationaryRootCount=1,
                  speedSquaredDerivativeDegree=7, speedUpperBound=proof['maxima']['speed']['upper'],
                  accelerationStationaryTimesS=[low,.5,high], accelerationStationaryRootCount=3,
                  accelerationSquaredDerivativeDegree=5,
                  accelerationUpperBound=proof['maxima']['acceleration']['upper'])
    cert['candidateCriticalTimesS'] = {'speed': [0.,.5,1.], 'acceleration': [0.,low,.5,high,1.]}
    cert['checks']['totalAcceleration'] = False
    cert['passed'] = False
    cert['rejections'] = ['totalAcceleration']
    return cert


class IndependentOverlapAuditTests(unittest.TestCase):
    def test_zero_overlap_and_handcrafted_schema(self):
        inputs = ((0., 0., 0.), 1., (0., 0., 0.), 0., 1.)
        proof = overlap_audit.independent_overlap_envelope(*inputs)
        self.assertTrue(proof['passed'])
        result = overlap_audit.audit_overlap_certificate(zero_certificate(), *inputs)
        self.assertTrue(result['certificateValid'], result['errors'])
        self.assertTrue(result['envelopePassed'])

    def test_analytic_single_quintic_and_honest_rejection(self):
        good = overlap_audit.independent_overlap_envelope((.05, 0., 0.), 1., (0., 0., 0.), 0., 1.)
        self.assertTrue(good['passed'])
        self.assertAlmostEqual(good['maxima']['speed']['lower'], .05*15/8, places=15)
        self.assertAlmostEqual(good['maxima']['acceleration']['upper'], .05*10/math.sqrt(3), places=15)
        self.assertEqual(good['windows'][0]['speed']['stationaryRootCount'], 1)
        # Acceleration^2 has two maxima and an interior zero, all stationary.
        self.assertEqual(good['windows'][0]['acceleration']['stationaryRootCount'], 3)
        bad = overlap_audit.independent_overlap_envelope((.1, 0., 0.), 1., (0., 0., 0.), 0., 1.)
        self.assertFalse(bad['passed'])
        self.assertTrue(bad['resolved'])
        self.assertEqual(bad['rejections'], ['totalAcceleration'])
        result = overlap_audit.audit_overlap_certificate(analytic_rejected_certificate(),
                    (.1,0.,0.), 1., (0.,0.,0.), 0., 1.)
        self.assertTrue(result['certificateValid'], result['errors'])
        self.assertFalse(result['envelopePassed'])
        self.assertFalse(result['passed'])

    def test_expanded_overlap_cancellation_and_windows(self):
        proof = overlap_audit.independent_overlap_envelope((.01, .002, 0.), 1., (-.01, -.002, 0.), 0., 1.)
        self.assertTrue(proof['passed'])
        self.assertEqual(proof['maxima']['speed']['upper'], 0.)
        self.assertEqual(proof['maxima']['acceleration']['upper'], 0.)
        staggered = overlap_audit.independent_overlap_envelope((.05, 0., 0.), 1., (0., .01, 0.), .25, 1.)
        self.assertTrue(staggered['passed'])
        self.assertEqual([(w['startS'], w['endS']) for w in staggered['windows']],
                         [(0., .25), (.25, 1.), (1., 1.25)])

    def test_root_multiplicity_and_nearby_roots(self):
        roots, _ = overlap_audit._isolate_stationary([F(1,16), F(-1,2), F(3,2), F(-2), F(1)])
        self.assertEqual(roots, [(F(1,2), F(1,2))])
        # q' = (u-.5)^2 - 2^-100; two roots 2^-49 apart.
        roots, _ = overlap_audit._isolate_stationary([F(0), F(1,4)-F(1,2**100), F(-1,2), F(1,3)])
        self.assertEqual(len(roots), 2)
        for interval, expected in zip(roots, (F(1,2)-F(1,2**50), F(1,2)+F(1,2**50))):
            self.assertLessEqual(interval[0], expected)
            self.assertGreaterEqual(interval[1], expected)
        roots, _ = overlap_audit._isolate_stationary([F(0), F(1), F(0), F(1)])
        self.assertEqual(roots, [])

    def test_exact_correction_gates_and_no_cap_slack(self):
        at_gate = overlap_audit.independent_overlap_envelope((0., 0., 0.), 1., (.012, 0., 0.), 0., 1.)
        self.assertTrue(at_gate['passed'])
        beyond_gate = overlap_audit.independent_overlap_envelope((0., 0., 0.), 1.,
                    (math.nextafter(.012, math.inf), 0., 0.), 0., 1.)
        self.assertFalse(beyond_gate['checks']['correctionMagnitude'])
        short = overlap_audit.independent_overlap_envelope((0., 0., 0.), 1., (0., 0., 0.), 0.,
                                                   math.nextafter(1., 0.))
        self.assertFalse(short['checks']['correctionDuration'])

    def test_metadata_tampering_and_fail_closed_limits(self):
        inputs = ((0., 0., 0.), 1., (0., 0., 0.), 0., 1.)
        cases = []
        cert = zero_certificate(); cert['caps']['totalSpeedMps'] = math.nextafter(overlap_audit.CAPS['totalSpeedMps'], math.inf); cases.append(cert)
        cert = zero_certificate(); cert['boundaryJets'][0]['velocityMps'][0] = .001; cases.append(cert)
        cert = zero_certificate(); cert['windows'][0]['speedStationaryRootCount'] = 1; cases.append(cert)
        cert = zero_certificate(); cert['maximumSpeedAtS'] = .25; cases.append(cert)
        cert = zero_certificate(); cert['checks']['totalSpeed'] = False; cases.append(cert)
        for cert in cases:
            result = overlap_audit.audit_overlap_certificate(cert, *inputs)
            self.assertFalse(result['certificateValid'], result)
        unresolved = overlap_audit.independent_overlap_envelope((.05, 0., 0.), 1., (0., 0., 0.), 0., 1., max_depth=0)
        self.assertFalse(unresolved['passed'])
        self.assertFalse(unresolved['resolved'])
        self.assertIn('proofError', unresolved)

    def test_invalid_inputs(self):
        for inputs in [((True,0,0),1,(0,0,0),0,1), ((math.nan,0,0),1,(0,0,0),0,1),
                       ((0,0,0),0,(0,0,0),0,1), ((0,0,0),1,(0,0,0),-1,1)]:
            with self.assertRaises(ValueError):
                overlap_audit.independent_overlap_envelope(*inputs)




class StateOnlyCausalReplay(unittest.TestCase):
    def setUp(self):
        import audit_source_reviewed_walk as audit
        from test_source_robust_support_audit import forecast_fixture
        from source_causal_com_overlap import certify_overlap
        self.audit = audit
        self.data = forecast_fixture()
        self.data['endpoint_validator'] = lambda row: 0.
        self.data['cadence']['reviewedReferenceProfile'] = dict(mode='continuous-com-causal')
        ref = self.data['references'][1]
        ref['phase_durations']['transfer'] = .5
        delta = next(n for n in self.data['notes'] if n['robustSupportAdmission']['selected'])['robustSupportAdmission']['comCorrectionM']
        cert = certify_overlap([0.,0.,0.], .5, delta, .1, 1.)
        for note in self.data['notes']:
            note['referenceContext'] = dict(causal=dict(causalOverlap=copy.deepcopy(cert)
                if note['robustSupportAdmission']['selected'] else None))

    def result(self):
        return self.audit.audit_causal_support_replay(**self.data)

    def test_first_actual_state_window_selects_before_nominal_transfer(self):
        from audit_source_robust_support import selection_from_history, support_observation
        result = self.result()
        wrapped = reviewed_robust_result(result, 1, dict(passed=True, causalOverlapEnvelopePass=True), mode='continuous-com-causal')
        self.assertTrue(wrapped['passed'], [k for k,v in result['checks'].items() if not v])
        self.assertTrue(wrapped['causalModeReplayVerified'])
        self.assertEqual(result['perStep'][1]['selection']['selectedAtTransferS'], .1)
        obs = support_observation(self.data['entry'], 1)
        history = [(i*.005, 0, obs['present'], True, 4*i) for i in range(21)]
        row = copy.deepcopy(self.data['entry']); row['sourceTimeS'] = .1
        self.assertIsNone(selection_from_history(history, row, self.data['references'][1], .1, 10., actual=True, prospective=True))
        self.assertIsNotNone(self.audit.state_only_selection_from_history(history, row, self.data['references'][1], .1, 10., actual=True, prospective=True))

    def test_actual_load_loss_resets_history_before_selection(self):
        self.data['rows'][39]['feet'][1]['loadN'] = 50.
        self.assertFalse(self.result()['checks']['selectionAtFirstCausalEligibleSolve'])

    def test_risk_disagreement_cannot_be_omitted(self):
        from test_source_robust_support_audit import forecast_fixture
        altered = forecast_fixture(disagreements=5)
        self.data['notes'][0:21] = altered['notes'][0:21]
        self.assertFalse(self.result()['checks']['selectionAtFirstCausalEligibleSolve'])

    def test_missing_actual_survivor_blocks_early_selection(self):
        self.data['rows'][39]['supportPointsXY'].pop(2)
        self.assertFalse(self.result()['checks']['selectionAtFirstCausalEligibleSolve'])

    def test_envelope_rejection_precedes_any_selection_commit(self):
        cert = self.data['notes'][20]['referenceContext']['causal']['causalOverlap']
        cert['passed'] = False
        with patch.object(self.audit, 'audit_overlap_certificate', return_value=dict(valid=True,
                independent=dict(passed=False))):
            result = self.result()
        self.assertFalse(result['checks']['summedEnvelopeRejectionRemainsUnselected'])
        self.assertFalse(result['checks']['selectionAtFirstCausalEligibleSolve'])

    def test_mutated_overlap_cannot_pass_causal_coverage(self):
        self.data['notes'][20]['referenceContext']['causal']['causalOverlap']['correction']['startS'] += .005
        self.assertFalse(self.result()['checks']['everyCausalCandidateOverlapCertificateValid'])

    def test_original_minimum_transfer_and_eighty_samples_still_bind(self):
        self.data['references'][1]['phase_durations']['transfer'] = 1.2
        result = self.result()
        self.assertFalse(result['checks']['eachLiftHas80ActualTriangleCorrectedUnloadSamples'])

    def test_causal_mode_cannot_qualify_through_old_profile(self):
        legacy = dict(checks=dict.fromkeys(REQUIRED_ROBUST_REPLAY_CHECKS, True))
        r = reviewed_robust_result(legacy, 1, dict(passed=True), mode='continuous-com-causal')
        self.assertFalse(r['passed']); self.assertFalse(r['causalModeReplayVerified'])
        good = dict(passed=True)
        q = qualify_reviewed(original_fixture(1), good, good, dict(passed=True, expectedImpratio=8.), True, good, good, 1, mode='continuous-com-causal')
        self.assertFalse(q['reviewedWalkPilotPhysicalPass'])

    def test_policy_literal_and_helper_source_are_required(self):
        from source_reviewed_walk import causal_selection_policy
        self.assertEqual(self.audit.causal_selection_policy_audit(), causal_selection_policy())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in REQUIRED_SOURCES: (root/name).write_text(name)
            hashes = {name:self.audit.file_sha(root/name) for name in REQUIRED_SOURCES}
            receipt = dict(codeSHA256AtStart=hashes, codeSHA256AtEnd=hashes,
                causalSupportSelectionEnabled=True)
            self.assertFalse(audit_source_provenance(receipt, root)['passed'])

    def test_independent_verifier_does_not_call_producer(self):
        cert = self.data['notes'][20]['referenceContext']['causal']['causalOverlap']
        with patch('source_causal_com_overlap.certify_overlap', side_effect=AssertionError('producer called')):
            result = self.audit.audit_overlap_certificate(cert, [0.,0.,0.], .5,
                cert['correction']['deltaM'], .1, 1.)
        self.assertTrue(result['passed'], result['errors'])





class ExplicitPostureScopeTests(unittest.TestCase):
    def test_default_context_rejects_posture_even_with_valid_digest(self):
        f=fixture();f['receipt']['config']['coupledPosture']={'profile':'reviewed-coupled-posture-pilot-v3'}
        rehash(f['receipt'])
        proof=audit_context(f['receipt'],f['note'],f['row'],f['reference'])
        self.assertFalse(proof['passed']);self.assertFalse(proof['checks']['explicitReferenceFeatureScope'])

    def test_default_context_rejects_bounded_settle_even_with_valid_digest(self):
        f=fixture();f['receipt']['config']['boundedSettle']={'profile':'seven-cubic-constant-jerk'}
        rehash(f['receipt'])
        proof=audit_context(f['receipt'],f['note'],f['row'],f['reference'])
        self.assertFalse(proof['passed']);self.assertFalse(proof['checks']['explicitReferenceFeatureScope'])

    def test_default_context_rejects_hidden_feature_declaration(self):
        f=fixture();f['note']['reviewedWalk']['featureProfile']='reviewed-coupled-posture-pilot-v3'
        self.assertFalse(audit_context(f['receipt'],f['note'],f['row'],f['reference'])['passed'])

    def test_callback_changes_only_explicit_reference_seam(self):
        f=fixture();calls=[]
        class Policy:
            def references(self,*args):
                calls.append(args)
                return {'sentinel':'explicit policy'}
        self.assertEqual(independent_references(f['receipt']['config'],f['receipt']['causal'],0.,'transfer',reference_policy=Policy()),{'sentinel':'explicit policy'})
        self.assertEqual(len(calls),1)
        self.assertIn('feet',independent_references(f['receipt']['config'],f['receipt']['causal'],0.,'transfer'))


if __name__=='__main__':unittest.main()
