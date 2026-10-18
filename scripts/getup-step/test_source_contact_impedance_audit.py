"""Pure corruption tests; no native library, world or integration is used."""
import copy
import struct
import unittest

from audit_source_contact_impedance import (ORIGINAL_OPTION, EXPERIMENT_OPTION,
    ORIGINAL_IDENTITY, REQUIRED_STRICT_CHECKS, SELECTED_PRESET_SHA256,
    audit_model_proof, audit_trial_controls, cache_proof, qualify_experiment, sha, state_sha)


def cache(model, bridge, engine, counter=0):
    header = (0x4c485343,213,28,27,21,17,16,1,16,32)
    payload = struct.pack('<10i',*header)+struct.pack('<i',counter)+bytes(12+16+16+8)
    return b'LHSC002!'+b''.join(bytes.fromhex(h) for h in (model,bridge,engine))+bytes.fromhex(sha(payload))+payload


def model_fixture():
    old = b'<mujoco>'+ORIGINAL_OPTION+b'<geom friction=".8 .1 .1"/></mujoco>\n'
    new = old.replace(ORIGINAL_OPTION,EXPERIMENT_OPTION)
    old_hash,new_hash,helper,bridge,engine = map(sha,(old,new,b'helper',b'bridge',b'engine'))
    entry = dict(qpos=[0.,0.,.8,1.,0.,0.,0.]+[0.]*21,qvel=[0.]*27,sourceStep=0)
    state = state_sha(entry)
    options = [213.,.00125,1.,0.]+[0.]*18
    changed = options.copy();changed[2] = 2.
    cold,baseline = cache(new_hash,bridge,engine),cache(old_hash,bridge,engine)
    preset = dict(selectedModelSHA256=old_hash,helperSHA256=helper,initialQposQvelSHA256=state)
    prep = dict(schema=1,experiment='contact-impratio-only',baselineImpratio=1.,candidateImpratio=2.,
        baselineModelSHA256=old_hash,modelSHA256=new_hash,helperSHA256=helper,
        initialQposQvelSHA256=state,coldCacheSHA256=sha(cold),initialNativeCounter=0,physicsIntegrations=0,
        onlyXMLDifference='option impratio: absent(default1) -> 2',
        originalMuGeometryMassMotorJointLimitsPreserved=True,
        originalSolverSolrefSolimpConeAndClockPreserved=True,selectedPresetChanged=False,
        originalModelQualificationClaim=False,originalFullCacheImportedIntoCandidate=False,
        modelBoundColdCacheRebuilt=True,originalAndCandidateOptions=[options,changed])
    selection = dict(variant='hip10-knee27_5-impratio2',modelSHA256=new_hash,baselineModelSHA256=old_hash,
        helperSHA256=helper,initialQposQvelSHA256=state,contactImpedanceChanged=True,contactImpratio=2.,
        originalModelQualificationClaim=False,modelSpecificColdCache=True,physicsIntegrations=0)
    assets = dict.fromkeys(('materials.xml','visual.xml','skybox.xml'),b'<asset/>')
    return dict(baseline_xml=old,candidate_xml=new,baseline_assets=assets.copy(),candidate_assets=assets.copy(),
        preset=preset,preparation=prep,selection=selection,helper_hash=helper,cold_cache=cold,
        baseline_cache=baseline,bridge_hash=bridge,engine_hash=engine,
        initial=dict(modelSHA256=new_hash,helperSHA256=helper,entry=entry,
            originalWarmStartCacheRestored=False,explicitColdInitialPoseAndVelocity=True))


def controls_fixture():
    config = dict(requestedSteps=10,requestedDistanceM=.18,loadedSettleHandoff=True,
        continuousLoadedHandoff=False,intermediateStableRequiredS=0.,initialTransferS=4.,
        phaseDurationsS=dict(transfer=2.,lift=.8,translate=1.2,land=1.,settle=1.52),
        originalContactJointMotorCollisionCaptureGates=True,simultaneousArmCoordination=dict(enabled=False),
        measuredSupportCaptureBarrier=dict(reserveM=.003,gainPerS=8.),
        opposedArmRangeBarrier=dict(verifiedRangeReserveRad=.02,predictionHorizonS=.2,clippingEnabled=False),
        contactImpedanceExperiment=dict(impratio=2.,baselineImpratio=1.,muChanged=False,
            originalModelQualificationClaim=False,separatelyModelBoundColdCache=True))
    refs = {1:dict(arm_kp=100.,arm_kd=20.,arm_weight=2.)}
    mapping = dict(angularGain=-1.,sideOrder=['right','left'])
    note = dict(t=0.,phase='lift',opposedArmSwing=dict(amplitudeGain=-1.,mappingClipped=False,
        feedbackUsesMeasuredLeg=True),captureBarrier=[dict(reserveM=.003,gainPerS=8.,measuredSupportOnly=True)],
        opposedArmRangeBarrier=[dict(side=s,verifiedArmRangeReserveRad=.02,predictionHorizonS=.2,
            clippingEnabled=False) for s in ('right','left')])
    return dict(cadence=config,references=refs,mapping=mapping,notes=[note],integrations=4)


def profile8_fixture():
    data = model_fixture()
    data['candidate_xml'] = data['candidate_xml'].replace(b'impratio="2"',b'impratio="8"')
    new_hash = sha(data['candidate_xml'])
    data['cold_cache'] = cache(new_hash,data['bridge_hash'],data['engine_hash'])
    data['preparation'].update(candidateImpratio=8.,modelSHA256=new_hash,
        coldCacheSHA256=sha(data['cold_cache']),onlyXMLDifference='option impratio: absent(default1) -> 8')
    data['preparation']['originalAndCandidateOptions'][1][2] = 8.
    data['selection'].update(variant='hip10-knee27_5-impratio8',contactImpratio=8.,modelSHA256=new_hash)
    data['initial']['modelSHA256'] = new_hash
    data['expected_ratio'] = 8.
    return data


def profile8_controls():
    data = controls_fixture()
    data['cadence']['contactImpedanceExperiment']['impratio'] = 8.
    data['expected_ratio'] = 8.
    return data


def original_fixture():
    checks = dict.fromkeys(REQUIRED_STRICT_CHECKS,True)
    checks[ORIGINAL_IDENTITY] = False
    return dict(requestedSteps=10,requestedDistanceM=.18,strictCadencedSequencePass=False,
        tenStep180mmQualificationPass=False,failedChecks=[ORIGINAL_IDENTITY],measured=dict(checks=checks))


class ContactImpedanceModelProof(unittest.TestCase):
    def setUp(self):
        self.data = model_fixture()

    def result(self):
        return audit_model_proof(**self.data)

    def rejects(self,name):
        result = self.result()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks'][name])

    def test_valid_explicit_model_proof(self):
        self.assertTrue(self.result()['passed'])
        self.assertFalse(self.result()['originalSelectedModelIdentity'])

    def test_extra_mu_edit_is_not_whitelisted_even_if_rehashed(self):
        self.data['candidate_xml'] = self.data['candidate_xml'].replace(b'.8 .1 .1',b'.9 .1 .1')
        h = sha(self.data['candidate_xml'])
        self.data['preparation']['modelSHA256'] = self.data['selection']['modelSHA256'] = h
        self.rejects('onlyWhitelistedXMLByteInsertion')

    def test_ratio_three_fails(self):
        self.data['candidate_xml'] = self.data['candidate_xml'].replace(b'impratio="2"',b'impratio="3"')
        self.rejects('onlyWhitelistedXMLByteInsertion')

    def test_second_option_insertion_fails(self):
        self.data['candidate_xml'] += EXPERIMENT_OPTION
        self.rejects('onlyWhitelistedXMLByteInsertion')

    def test_extra_included_xml_edit_fails(self):
        self.data['candidate_assets']['materials.xml'] = b'altered'
        self.rejects('allOtherModelAssetsByteIdentical')

    def test_changed_selected_preset_fails(self):
        self.data['preset_hash'] = sha(b'changed preset')
        self.rejects('selectedPresetBytesUnchanged')

    def test_baseline_not_selected_preset_fails(self):
        self.data['preset']['selectedModelSHA256'] = '0'*64
        self.rejects('baselineMatchesSelectedPreset')

    def test_candidate_summary_must_bind_new_model(self):
        self.data['initial']['modelSHA256'] = sha(self.data['baseline_xml'])
        self.rejects('initialSummaryModelHelperAndMeasuredStateIdentity')

    def test_changed_initial_measured_state_fails(self):
        self.data['initial']['entry']['qpos'][0] += .001
        self.rejects('initialSummaryModelHelperAndMeasuredStateIdentity')

    def test_initial_summary_cannot_declare_original_warm_import(self):
        self.data['initial']['originalWarmStartCacheRestored'] = True
        self.rejects('initialSummaryModelHelperAndMeasuredStateIdentity')

    def test_initial_summary_requires_explicit_cold_initialization(self):
        self.data['initial']['explicitColdInitialPoseAndVelocity'] = False
        self.rejects('initialSummaryModelHelperAndMeasuredStateIdentity')

    def test_changed_helper_fails(self):
        self.data['helper_hash'] = sha(b'wrong helper')
        self.rejects('helperIdentityPreserved')

    def test_original_cache_cannot_be_relabelled_candidate(self):
        self.data['cold_cache'] = self.data['baseline_cache']
        self.data['preparation']['coldCacheSHA256'] = sha(self.data['cold_cache'])
        self.rejects('candidateColdCacheModelBindingAndPayload')
        self.assertFalse(self.result()['checks']['distinctModelBoundCaches'])

    def test_wrong_bridge_binding_fails(self):
        self.data['bridge_hash'] = sha(b'wrong bridge')
        self.rejects('candidateColdCacheModelBindingAndPayload')

    def test_corrupt_payload_fails_even_if_receipt_filehash_updated(self):
        self.data['cold_cache'] = self.data['cold_cache'][:-1]+b'!'
        self.data['preparation']['coldCacheSHA256'] = sha(self.data['cold_cache'])
        self.rejects('candidateColdCacheModelBindingAndPayload')

    def test_nonzero_counter_in_self_consistent_cache_fails(self):
        self.data['cold_cache'] = cache(sha(self.data['candidate_xml']),self.data['bridge_hash'],
            self.data['engine_hash'],counter=1)
        self.data['preparation']['coldCacheSHA256'] = sha(self.data['cold_cache'])
        self.rejects('candidateColdCacheModelBindingAndPayload')

    def test_false_original_cache_import_declaration_fails(self):
        self.data['preparation']['originalFullCacheImportedIntoCandidate'] = True
        self.rejects('separateColdCacheDeclarations')

    def test_solver_option_change_fails(self):
        self.data['preparation']['originalAndCandidateOptions'][1][4] = 1.
        self.rejects('optionVectorOnlyImpratioChanged')

    def test_wrong_recorded_ratio_fails(self):
        self.data['preparation']['originalAndCandidateOptions'][1][2] = 1.
        self.rejects('optionVectorOnlyImpratioChanged')

    def test_false_original_identity_claim_fails(self):
        self.data['preparation']['originalModelQualificationClaim'] = True
        self.rejects('explicitPreparationScope')

    def test_false_original_identity_selection_claim_fails(self):
        self.data['selection']['originalModelQualificationClaim'] = True
        self.rejects('explicitAlteredSelection')


class ContactImpedanceControls(unittest.TestCase):
    def setUp(self):
        self.data = controls_fixture()

    def rejects(self,name):
        result = audit_trial_controls(**self.data)
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks'][name])

    def test_original_controls_pass(self):
        self.assertTrue(audit_trial_controls(**self.data)['passed'])

    def test_changed_arm_gain_fails(self):
        self.data['references'][1]['arm_kp'] = 200.
        self.rejects('unchangedArmGainAndMapping')

    def test_changed_capture_gain_fails(self):
        self.data['notes'][0]['captureBarrier'][0]['gainPerS'] = 9.
        self.rejects('everyActualControlOriginalBarriers')

    def test_missing_capture_barrier_in_swing_fails(self):
        self.data['notes'][0]['captureBarrier'] = []
        self.rejects('everyActualControlOriginalBarriers')

    def test_implicit_arms_would_be_a_second_experiment(self):
        self.data['cadence']['simultaneousArmCoordination']['enabled'] = True
        self.rejects('unchangedCadenceControls')

    def test_changed_duration_fails(self):
        self.data['cadence']['phaseDurationsS']['settle'] = 1.5
        self.rejects('unchangedCadenceControls')

    def test_clipping_fails(self):
        self.data['notes'][0]['opposedArmSwing']['mappingClipped'] = True
        self.rejects('everyActualControlOriginalMapping')

    def test_false_trial_original_identity_claim_fails(self):
        self.data['cadence']['contactImpedanceExperiment']['originalModelQualificationClaim'] = True
        self.rejects('explicitContactImpedanceExperiment')

    def test_missing_actual_control_note_fails(self):
        self.data['notes'] = []
        self.rejects('original200HzControlSchedule')


class ExplicitExperimentalQualification(unittest.TestCase):
    def setUp(self):
        self.original = original_fixture()

    def result(self,proof=True,controls=True,provenance=True):
        return qualify_experiment(self.original,dict(passed=proof),dict(passed=controls),provenance)

    def test_exactly_one_identity_failure_permits_separate_result(self):
        before = copy.deepcopy(self.original)
        result = self.result()
        self.assertTrue(result['explicitImpratio2TenStepPass'])
        self.assertFalse(result['originalStrictCadencedSequencePass'])
        self.assertFalse(result['originalTenStep180mmQualificationPass'])
        self.assertEqual(before,self.original)

    def test_forged_original_identity_true_fails(self):
        self.original['measured']['checks'][ORIGINAL_IDENTITY] = True
        self.original['strictCadencedSequencePass'] = True
        self.original['tenStep180mmQualificationPass'] = True
        self.original['failedChecks'] = []
        self.assertFalse(self.result()['explicitImpratio2TenStepPass'])

    def test_any_original_physical_failure_is_still_fatal(self):
        self.original['measured']['checks']['globalLoadedSlipWithinOriginalLimit'] = False
        self.original['failedChecks'].append('globalLoadedSlipWithinOriginalLimit')
        self.assertFalse(self.result()['explicitImpratio2TenStepPass'])

    def test_missing_original_arm_gate_is_fatal(self):
        del self.original['measured']['checks']['independentLimbFKAndExactOpposition']
        self.assertFalse(self.result()['explicitImpratio2TenStepPass'])

    def test_model_proof_failure_is_fatal(self):
        self.assertFalse(self.result(proof=False)['explicitImpratio2TenStepPass'])

    def test_changed_gains_are_fatal(self):
        self.assertFalse(self.result(controls=False)['explicitImpratio2TenStepPass'])

    def test_missing_source_provenance_is_fatal(self):
        self.assertFalse(self.result(provenance=False)['explicitImpratio2TenStepPass'])

    def test_pilot_count_cannot_claim_ten(self):
        self.original['requestedSteps'] = 2
        self.assertFalse(self.result()['explicitImpratio2TenStepPass'])

    def test_missing_cache_magic_rejected_without_native_import(self):
        x = model_fixture()
        self.assertFalse(cache_proof(x['cold_cache'][1:],sha(x['candidate_xml']),
            x['bridge_hash'],x['engine_hash'],True)['passed'])


class ExplicitImpratio8Profile(unittest.TestCase):
    def test_explicit_expected8_model_proof_passes(self):
        result = audit_model_proof(**profile8_fixture())
        self.assertTrue(result['passed'])
        self.assertEqual(result['expectedImpratio'],8.)
        self.assertFalse(result['originalSelectedModelIdentity'])

    def test_default_expected2_rejects8(self):
        data = profile8_fixture();del data['expected_ratio']
        result = audit_model_proof(**data)
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['onlyWhitelistedXMLByteInsertion'])
        self.assertFalse(result['checks']['explicitPreparationScope'])

    def test_explicit_expected8_rejects2(self):
        data = model_fixture();data['expected_ratio'] = 8.
        self.assertFalse(audit_model_proof(**data)['passed'])

    def test_extra_mu_edit_still_fails8(self):
        data = profile8_fixture()
        data['candidate_xml'] = data['candidate_xml'].replace(b'.8 .1 .1',b'.9 .1 .1')
        result = audit_model_proof(**data)
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['onlyWhitelistedXMLByteInsertion'])

    def test_wrong_profile_receipt_flag_fails8(self):
        data = profile8_fixture();data['preparation']['candidateImpratio'] = 2.
        self.assertFalse(audit_model_proof(**data)['checks']['explicitPreparationScope'])

    def test_wrong_selection_profile_fails8(self):
        data = profile8_fixture();data['selection']['variant'] = 'hip10-knee27_5-impratio2'
        self.assertFalse(audit_model_proof(**data)['checks']['explicitAlteredSelection'])

    def test_profile2_cache_cannot_enter8(self):
        data = profile8_fixture();data['cold_cache'] = model_fixture()['cold_cache']
        data['preparation']['coldCacheSHA256'] = sha(data['cold_cache'])
        self.assertFalse(audit_model_proof(**data)['checks']['candidateColdCacheModelBindingAndPayload'])

    def test_wrong_recorded_option_ratio_fails8(self):
        data = profile8_fixture();data['preparation']['originalAndCandidateOptions'][1][2] = 2.
        self.assertFalse(audit_model_proof(**data)['checks']['optionVectorOnlyImpratioChanged'])

    def test_explicit_expected8_controls_pass(self):
        self.assertTrue(audit_trial_controls(**profile8_controls())['passed'])

    def test_default_controls_reject8(self):
        data = profile8_controls();del data['expected_ratio']
        self.assertFalse(audit_trial_controls(**data)['checks']['explicitContactImpedanceExperiment'])

    def test_only_profile8_result_can_pass_for8(self):
        result = qualify_experiment(original_fixture(),audit_model_proof(**profile8_fixture()),
            audit_trial_controls(**profile8_controls()),expected_ratio=8.)
        self.assertTrue(result['explicitImpratio8TenStepPass'])
        self.assertFalse(result['explicitImpratio2TenStepPass'])
        self.assertFalse(result['originalStrictCadencedSequencePass'])
        self.assertFalse(result['originalTenStep180mmQualificationPass'])

    def test_profile2_proofs_cannot_qualify8(self):
        result = qualify_experiment(original_fixture(),audit_model_proof(**model_fixture()),
            audit_trial_controls(**controls_fixture()),expected_ratio=8.)
        self.assertFalse(result['explicitImpratio8TenStepPass'])
        self.assertFalse(result['explicitImpratio2TenStepPass'])
        self.assertFalse(result['checks']['explicitProfileBinding'])

    def test_only_profile2_result_can_pass_for2(self):
        result = qualify_experiment(original_fixture(),audit_model_proof(**model_fixture()),
            audit_trial_controls(**controls_fixture()))
        self.assertTrue(result['explicitImpratio2TenStepPass'])
        self.assertFalse(result['explicitImpratio8TenStepPass'])

    def test_unapproved_profile5_is_never_supported(self):
        data = model_fixture();data['expected_ratio'] = 5.
        with self.assertRaises(ValueError):
            audit_model_proof(**data)


if __name__ == '__main__':
    unittest.main()
