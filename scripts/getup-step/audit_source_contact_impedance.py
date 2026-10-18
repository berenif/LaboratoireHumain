#!/usr/bin/env python3
"""Saved-data audit of explicitly altered impratio=2 or 8 comparisons.

The unchanged cadence auditor is called as-is. Its original selected-model
identity must FAIL. Only the separately named experimental result can pass.
No native world is opened and no physics is integrated here.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

from audit_source_cadenced_steps import audit as audit_cadence, json_array_items

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_IDENTITY = 'selectedModelHelperAndInitialStateIdentity'
SELECTED_PRESET_SHA256 = 'a73270f51cb94a83545ee530f1edf620f287b1f7407f08a0c9afb174a61c658d'
ORIGINAL_OPTION = b'<option timestep=".00125"/>'
EXPERIMENT_OPTION = b'<option timestep=".00125" impratio="2"/>'
REQUIRED_STRICT_CHECKS = frozenset('''explicitHandoffProtocol
originalLoadedSettleReadinessThresholds boundedRequestedCount allExpectedStepRecords
originalQuietAndSlipThresholds physicalTimeConfiguration contiguousNativeCounters
elapsedMatchesActualIntegrations alternatingMeasuredSides requestedDistancesAndSides
controllerDurationsMatchConfiguration stepOriginsMatchPrecedingMeasuredFeet
stepStartCountersMatchJournal stepHeadingMatchesNativeRoot phaseElapsedMatchesNativeSamples
finiteRecordedState unitRootQuaternions monotonicGlobalLoadedSlip
globalLoadedSlipWithinOriginalLimit originalGlobalPhysicalGates postEntryJointLimits
captureMarginTelemetryMatchesReconstruction definedNonnegativeCurrentCapture
allOriginalGroundRays forwardMatchesMeasuredFootPosition landingSummaryMatchesMeasuredTransition
loadedSettleTargetReconstructedFromActualLanding loadedSettleCompletionSummaryMatchesReadiness
completionSummaryMatchesMeasuredTransition loadedSettleHandoffBeginsOnNextSample
nonemptyJournal exactOrderedConsecutivePhases allMeasuredStepDistances
everyStepLiftQualified everyStepLandingQualified everyStepUnloadQualified
everyStepCompletionQualified landingAndCompletionAreActualPhaseEnds
everyMotionPhaseHonorsConfiguredDuration allLoadedSettleTargetsReconstructed
everyIntermediateLoadedSettleQualified finalFiveSecondsQuietOnFinalStep
sourceTimeContinuity rootPositionContinuity hingePositionContinuity rootQuaternionContinuity
loadedSlipAccumulatorContinuity originalMotorLimit consistentGlobalIntegrationAccounting
completedRunner declaredNoResetOrRootAssistance originalMotorFrictionAndClock
codeUnchangedDuringRun selectedModelHelperAndInitialStateIdentity
retainedModelAndHelperBytesMatchIdentity summaryGlobalSlipMatchesJournal
existingMotionArmAndBodyFKQualification independentLimbFKAndExactOpposition'''.split())


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def state_sha(entry):
    return sha(np.r_[entry['qpos'], entry['qvel']].astype('<f8').tobytes())


def ratio_value(expected_ratio):
    if expected_ratio not in (2.,8.):
        raise ValueError('Only the explicit impratio2 and impratio8 profiles are supported')
    return float(expected_ratio)


def cache_proof(blob, model_hash, bridge_hash, engine_hash, cold=False):
    """Pure LHSC002! binding, payload digest and fixed native ABI checks."""
    checks = dict(headerMagic=len(blob) >= 180 and blob[:8] == b'LHSC002!',
        modelBridgeEngineBinding=False, payloadSHA256=False, nativeABI=False,
        zeroNativeCounter=not cold)
    payload = blob[136:]
    native_counter = None
    try:
        binding = b''.join(bytes.fromhex(s) for s in (model_hash, bridge_hash, engine_hash))
        checks['modelBridgeEngineBinding'] = len(binding) == 96 and blob[8:104] == binding
        checks['payloadSHA256'] = len(blob) >= 180 and sha(payload) == blob[104:136].hex()
        header = struct.unpack('<10i', payload[:40])
        # This retained ABI has two leading pointers before LH.steps. The
        # checkpoint stores the remaining LH tail, then mjData and its buffers.
        checks['nativeABI'] = (header[:6] == (0x4c485343,213,28,27,21,17)
            and all(v > 0 for v in header[6:]) and header[9] >= 20
            and len(payload) == 40+header[9]-16+header[8]+header[6]+8*header[7])
        native_counter = struct.unpack('<i', payload[40:44])[0]
        if cold:
            checks['zeroNativeCounter'] = native_counter == 0
    except (ValueError, struct.error, TypeError):
        pass
    return dict(passed=all(checks.values()), checks=checks,
        cacheSHA256=sha(blob), payloadSHA256=sha(payload), bytes=len(blob),nativeCounter=native_counter)


def audit_model_proof(baseline_xml, candidate_xml, baseline_assets, candidate_assets,
                      preset, preparation, selection, helper_hash, cold_cache,
                      baseline_cache, bridge_hash, engine_hash, initial=None,
                      preset_hash=SELECTED_PRESET_SHA256, expected_ratio=2.):
    """Pure, separately scoped model/cache proof; never substitutes model identity."""
    ratio = ratio_value(expected_ratio)
    option = f'<option timestep=".00125" impratio="{ratio:g}"/>'.encode()
    old_hash, new_hash = sha(baseline_xml), sha(candidate_xml)
    checks = dict(selectedPresetBytesUnchanged=preset_hash == SELECTED_PRESET_SHA256,
        baselineMatchesSelectedPreset=(old_hash == preset.get('selectedModelSHA256')
            == preparation.get('baselineModelSHA256') == selection.get('baselineModelSHA256')),
        onlyWhitelistedXMLByteInsertion=(baseline_xml.count(ORIGINAL_OPTION) == 1
            and candidate_xml == baseline_xml.replace(ORIGINAL_OPTION, option, 1)
            and candidate_xml.count(option) == 1 and old_hash != new_hash),
        allOtherModelAssetsByteIdentical=(baseline_assets == candidate_assets
            and set(baseline_assets) >= {'materials.xml','skybox.xml','visual.xml'}),
        candidateModelIdentity=(new_hash == preparation.get('modelSHA256') == selection.get('modelSHA256')),
        helperIdentityPreserved=(helper_hash == preparation.get('helperSHA256')
            == selection.get('helperSHA256') == preset.get('helperSHA256')),
        initialStateIdentityDeclared=(preparation.get('initialQposQvelSHA256')
            == selection.get('initialQposQvelSHA256') == preset.get('initialQposQvelSHA256')),
        explicitPreparationScope=(preparation.get('schema') == 1
            and preparation.get('experiment') == 'contact-impratio-only'
            and preparation.get('baselineImpratio') == 1. and preparation.get('candidateImpratio') == ratio
            and preparation.get('initialNativeCounter') == 0 and preparation.get('physicsIntegrations') == 0
            and preparation.get('onlyXMLDifference') == f'option impratio: absent(default1) -> {ratio:g}'
            and preparation.get('originalMuGeometryMassMotorJointLimitsPreserved') is True
            and preparation.get('originalSolverSolrefSolimpConeAndClockPreserved') is True
            and preparation.get('selectedPresetChanged') is False
            and preparation.get('originalModelQualificationClaim') is False),
        separateColdCacheDeclarations=(preparation.get('originalFullCacheImportedIntoCandidate') is False
            and preparation.get('modelBoundColdCacheRebuilt') is True
            and selection.get('modelSpecificColdCache') is True and selection.get('physicsIntegrations') == 0),
        explicitAlteredSelection=(selection.get('variant') == f'hip10-knee27_5-impratio{ratio:g}'
            and selection.get('contactImpedanceChanged') is True and selection.get('contactImpratio') == ratio
            and selection.get('originalModelQualificationClaim') is False),
        optionVectorOnlyImpratioChanged=False)
    try:
        options = np.array(preparation['originalAndCandidateOptions'], dtype=float)
        checks['optionVectorOnlyImpratioChanged'] = bool(options.shape == (2,22)
            and np.isfinite(options).all() and np.array_equal(np.delete(options[0],2),np.delete(options[1],2))
            and options[0,2] == 1. and options[1,2] == ratio
            and options[0,0] == 213. and options[0,1] == .00125 and options[0,3] == 0.)
    except (KeyError, TypeError, ValueError):
        pass
    cache = cache_proof(cold_cache,new_hash,bridge_hash,engine_hash,cold=True)
    old_cache = cache_proof(baseline_cache,old_hash,bridge_hash,engine_hash,cold=True)
    checks['candidateColdCacheModelBindingAndPayload'] = cache['passed']
    checks['baselineColdCacheModelBindingAndPayload'] = old_cache['passed']
    checks['coldCacheHashMatchesPreparation'] = cache['cacheSHA256'] == preparation.get('coldCacheSHA256')
    checks['distinctModelBoundCaches'] = cold_cache != baseline_cache and cold_cache[8:40] != baseline_cache[8:40]
    if initial is not None:
        checks['initialSummaryModelHelperAndMeasuredStateIdentity'] = (
            initial.get('modelSHA256') == new_hash and initial.get('helperSHA256') == helper_hash
            and state_sha(initial['entry']) == preset.get('initialQposQvelSHA256')
            and initial['entry']['sourceStep'] == 0
            and initial.get('originalWarmStartCacheRestored') is False
            and initial.get('explicitColdInitialPoseAndVelocity') is True)
    return dict(passed=all(checks.values()), expectedImpratio=ratio, checks=checks, baselineModelSHA256=old_hash,
        alteredModelSHA256=new_hash, originalSelectedModelIdentity=new_hash == preset.get('selectedModelSHA256'),
        candidateColdCache=cache, baselineColdCache=old_cache)


def audit_trial_controls(cadence, references, mapping, notes, integrations, expected_ratio=2.):
    """Bound this isolated comparison to the unchanged loaded-settle controls."""
    ratio = ratio_value(expected_ratio)
    checks = dict(explicitContactImpedanceExperiment=cadence.get('contactImpedanceExperiment') == dict(
        impratio=ratio,baselineImpratio=1.,muChanged=False,originalModelQualificationClaim=False,
        separatelyModelBoundColdCache=True),
        unchangedCadenceControls=(cadence.get('requestedSteps') == 10 and cadence.get('requestedDistanceM') == .18
            and cadence.get('loadedSettleHandoff') is True and cadence.get('continuousLoadedHandoff') is False
            and cadence.get('intermediateStableRequiredS') == 0. and cadence.get('initialTransferS') == 4.
            and cadence.get('phaseDurationsS') == dict(transfer=2.,lift=.8,translate=1.2,land=1.,settle=1.52)
            and cadence.get('originalContactJointMotorCollisionCaptureGates') is True
            and cadence.get('simultaneousArmCoordination',{}).get('enabled') is False),
        unchangedBarrierConfiguration=(cadence.get('measuredSupportCaptureBarrier') == dict(reserveM=.003,gainPerS=8.)
            and cadence.get('opposedArmRangeBarrier') == dict(verifiedRangeReserveRad=.02,predictionHorizonS=.2,clippingEnabled=False)),
        unchangedArmGainAndMapping=(mapping.get('angularGain') == -1. and mapping.get('sideOrder') == ['right','left']
            and bool(references) and all(r.get('arm_kp') == 100. and r.get('arm_kd') == 20.
                and r.get('arm_weight') == 2. for r in references.values())),
        everyActualControlOriginalMapping=True, everyActualControlOriginalBarriers=True,
        original200HzControlSchedule=True)
    count = 0
    for i,note in enumerate(notes):
        count += 1
        swing = note.get('opposedArmSwing',{})
        checks['everyActualControlOriginalMapping'] &= (swing.get('amplitudeGain') == -1.
            and swing.get('mappingClipped') is False and swing.get('feedbackUsesMeasuredLeg') is True
            and 'simultaneousArmCoordination' not in note)
        barriers = note.get('opposedArmRangeBarrier',[])
        capture = note.get('captureBarrier')
        checks['everyActualControlOriginalBarriers'] &= (len(barriers) == 2
            and [b.get('side') for b in barriers] == ['right','left']
            and all(b.get('verifiedArmRangeReserveRad') == .02 and b.get('predictionHorizonS') == .2
                and b.get('clippingEnabled') is False for b in barriers)
            and isinstance(capture,list)
            and (bool(capture) or note.get('phase') not in ('lift','translate','land','settle'))
            and all(b.get('reserveM') == .003 and b.get('gainPerS') == 8.
                and b.get('measuredSupportOnly') is True for b in (capture or [])))
        t = note.get('t')
        checks['original200HzControlSchedule'] &= (isinstance(t,(float,int)) and math.isfinite(t)
            and abs(t-i*.005) < 1e-8)
    checks['original200HzControlSchedule'] &= count > 0 and count == (integrations+3)//4
    return dict(passed=all(checks.values()),expectedImpratio=ratio,checks=checks,actualControlNotes=count)


def qualify_experiment(original, model_proof, controls, provenance_pass=True, expected_ratio=2.):
    """Permit exactly one failed check, and keep all original flags false."""
    ratio = ratio_value(expected_ratio)
    checks = original.get('measured',{}).get('checks',{})
    failed = sorted(k for k,v in checks.items() if v is not True)
    verdict = dict(originalAuditRetainsAlteredModelIdentityFailure=(checks.get(ORIGINAL_IDENTITY) is False
        and original.get('strictCadencedSequencePass') is False
        and original.get('tenStep180mmQualificationPass') is False),
        originalFullCheckCoverage=REQUIRED_STRICT_CHECKS <= set(checks),
        everyOriginalCheckExceptSelectedModelIdentity=(failed == [ORIGINAL_IDENTITY]
            and sorted(original.get('failedChecks',[])) == failed),
        requestedTen180mm=(original.get('requestedSteps') == 10 and original.get('requestedDistanceM') == .18),
        explicitModelProof=model_proof.get('passed') is True,
        unchangedTrialControls=controls.get('passed') is True,
        sourceAndLibraryProvenance=provenance_pass is True,
        explicitProfileBinding=(model_proof.get('expectedImpratio',2.) == ratio
            and controls.get('expectedImpratio',2.) == ratio))
    passed = all(verdict.values())
    return dict(explicitImpratio2TenStepPass=passed and ratio == 2.,
        explicitImpratio8TenStepPass=passed and ratio == 8.,expectedImpratio=ratio,checks=verdict,
        failedChecks=[k for k,v in verdict.items() if not v],
        originalStrictCadencedSequencePass=original.get('strictCadencedSequencePass'),
        originalTenStep180mmQualificationPass=original.get('tenStep180mmQualificationPass'))


def audit_preparation(base, initial=None, expected_ratio=2.):
    """Read-only zero-integration preparation proof, usable before the run."""
    base = Path(base)
    read = lambda p: json.loads(p.read_text())
    preset_path = Path(__file__).with_name('source-step-selected-v1.json')
    preset = read(preset_path)
    preparation = read(base/'contact-impedance-model.json')
    old, new = base/'baseline-model-session',base/'model-session'
    assets = lambda p: {x.name:x.read_bytes() for x in (p/'source-model').iterdir()
        if x.is_file() and x.name != 'humanoid_high_freq.xml'}
    bridge = file_sha(new/'source_bridge_v2.so')
    engine = file_sha(new/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3')
    proof = audit_model_proof((old/'source-model/humanoid_high_freq.xml').read_bytes(),
        (new/'source-model/humanoid_high_freq.xml').read_bytes(),assets(old),assets(new),preset,
        preparation,read(base/'selection.json'),file_sha(ROOT/preset['helper']),
        (new/'slow-continuation/final.bin').read_bytes(),(old/'slow-continuation/final.bin').read_bytes(),
        bridge,engine,initial,preset_hash=file_sha(preset_path),expected_ratio=expected_ratio)
    proof['checks']['sameBaselineBridgeAndEngine'] = (bridge == file_sha(old/'source_bridge_v2.so')
        and engine == file_sha(old/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    inspector = base/'contact-inspector.so'
    proof['checks']['retainedInspectorBytesMatchPreparation'] = (inspector.is_file()
        and file_sha(inspector) == preparation.get('inspectorSHA256'))
    runner = base/'frozen-source/source_contact_impedance_probe.py'
    if not runner.is_file():
        runner = Path(__file__).with_name('source_contact_impedance_probe.py')
    proof['checks']['launcherBytesMatchPreparation'] = (runner.is_file()
        and file_sha(runner) == preparation.get('runnerSHA256'))
    proof['retainedInspectorPath'] = str(inspector)
    proof['retainedInspectorPathNotAssumedToBeOriginalLoadedPath'] = True
    proof['launcherPathChecked'] = str(runner)
    proof['passed'] = all(proof['checks'].values())
    return proof


def audit(folder, expected_ratio=2.):
    folder = Path(folder)
    base = folder.parents[1]
    read = lambda p: json.loads(p.read_text())
    initial = read(folder/'summary.json')
    proof = audit_preparation(base,initial,expected_ratio=expected_ratio)
    # Intentionally no patching, monkeypatching or identity rewriting here.
    original = audit_cadence(folder)
    summary = read(folder/'multistep-summary.json')
    final_cache = cache_proof((folder/'final.bin').read_bytes(),proof['alteredModelSHA256'],
        file_sha(base/'model-session/source_bridge_v2.so'),
        file_sha(base/'model-session/dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    proof['finalCheckpoint'] = final_cache
    proof['checks']['finalCheckpointBindingPayloadAndNativeCount'] = (
        final_cache['passed'] and final_cache['nativeCounter'] == summary['actualIntegrations'])
    proof['passed'] = all(proof['checks'].values())
    cadence = read(folder/'cadenced-step-config.json')
    references = {s['index']:read(folder/f"controller-step-{s['index']:02d}.json") for s in summary['steps']}
    controls = audit_trial_controls(cadence,references,read(folder/'arm-mapping.json'),
        json_array_items(folder/'control.json'),original['measured']['actualRecordedIntegrations'],
        expected_ratio=expected_ratio)
    receipt = read(folder/'opposed-arm-run.json')
    prep = read(base/'contact-impedance-model.json')
    source_checks = {}
    for name,expected in receipt.get('codeSHA256AtStart',{}).items():
        path = base/'frozen-source'/name
        if not path.is_file():
            path = Path(__file__).with_name(name)
        source_checks[name] = path.is_file() and file_sha(path) == expected
    runner = 'source_contact_impedance_probe.py'
    source_checks['experimentLauncherBoundToPreparation'] = (
        receipt.get('codeSHA256AtStart',{}).get(runner) == prep.get('runnerSHA256')
        and source_checks.get(runner) is True)
    source_checks['sourceReceiptUnchanged'] = receipt.get('codeUnchangedDuringRun') is True
    result = qualify_experiment(original,proof,controls,bool(source_checks) and all(source_checks.values()),
        expected_ratio=expected_ratio)
    result.update(schema=1,savedDataOnly=True,physicsIntegrations=0,alteredModelExperiment=True,
        originalModelQualificationClaim=False,modelProof=proof,trialControls=controls,
        originalSelectedModelIdentity=original['measured']['checks'].get(ORIGINAL_IDENTITY),
        sourceProvenance=source_checks,originalStrictAudit=original,
        auditorSHA256=file_sha(Path(__file__)),
        originalStrictAuditorSHA256=file_sha(Path(__file__).with_name('audit_source_cadenced_steps.py')),
        evidenceLimit='Cache bindings, payload hashes, initial-state identity and hashed launcher declarations are verified. Saved files cannot prove the absence of every unrecorded earlier import or external action.',
        hashes={str(p.relative_to(base)):file_sha(p) for p in (
            base/'contact-impedance-model.json',base/'selection.json',
            base/'model-session/source-model/humanoid_high_freq.xml',
            base/'baseline-model-session/source-model/humanoid_high_freq.xml',
            base/'model-session/slow-continuation/final.bin',
            base/'baseline-model-session/slow-continuation/final.bin')})
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true',help='folder is the experiment base; never audit live journals')
    p.add_argument('--impratio',type=float,choices=(2.,8.),default=2.,help='Explicit expected profile; never inferred from a receipt')
    args = p.parse_args()
    result = audit_preparation(args.folder,expected_ratio=args.impratio) if args.prepare_only else audit(args.folder,expected_ratio=args.impratio)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    passed = result['passed'] if args.prepare_only else result[f'explicitImpratio{args.impratio:g}TenStepPass']
    print(json.dumps(dict(passed=passed,failedChecks=result.get('failedChecks',
        [k for k,v in result.get('checks',{}).items() if not v])),indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
