"""Pure/mock wrapper contracts. No native World, optimizer or physics is opened."""
from collections import deque
from dataclasses import asdict, replace, dataclass
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from numpy.testing import assert_allclose

import source_reviewed_wholebody as w
from source_reference_context import (ReferenceContext, Jet as CJ, Orientation,
    Foot, canonical, prospective_unload_context)
from coupled_posture_reference import Jet, OrientationJet
from coupled_posture_kinematics import MatrixJet


def fixture_context(phase='transfer', elapsed=4., now=4., step=None):
    foot = Foot(CJ.make([0., 0., .03], [0.]*3, [0.]*3),
        Orientation.make(2, np.eye(3), np.zeros(3), np.zeros(3), kp=45., kd=14., weight=7.))
    joints = CJ.make(np.r_[np.zeros(15), np.arange(6)/100.], np.arange(21)/1000., np.arange(21)/10000.)
    arms = CJ.make(np.arange(6)/100., np.arange(6)/1000., np.arange(6)/10000.)
    return ReferenceContext(2, 'reviewed-reference-v1', round(now/.00125) if step is None else step,
        now, 1, phase, elapsed, 0, CJ.make([0., 0., .8], [0.]*3, [0.]*3),
        joints, arms, (100., 20., 2.),
        (Orientation.make(1, np.eye(3), [0.]*3, [0.]*3, kp=40., kd=12., weight=4.),),
        (foot, replace(foot, orientation=replace(foot.orientation, body=3))),
        (((1., 2., 3.), 4.),), False,
        canonical(dict(contactSeekAfter=dict(jet=[0., 0., 0.]))),
        canonical(dict(floorTargets=[.03, .03], finalCom=[.02, 0., .8], stance_side=1)),
        canonical(dict(captureBarrier=[], opposedArmRangeBarrier=[], robustSupportBarriers=[])))


def sample_for(context, offset=.002):
    pose = MatrixJet.rest(np.eye(4))
    full = Jet.rest(np.r_[np.full(15, .01), np.arange(6)/100.])
    geometric = SimpleNamespace(full_joints=full,
        future_arm_delta_right_left=Jet.rest([0., 0.]))
    return SimpleNamespace(lower_joints=Jet.rest(np.full(15, .01)),
        com=Jet.rest(np.array(context.com.position)+[offset, 0., 0.]),
        thorax=OrientationJet.rest(1, np.eye(3)), pelvis=OrientationJet.rest(4, np.eye(3)),
        feet=(pose, pose), geometric=geometric, diagnostics=dict(mockPureSample=True))


def controller():
    state = SimpleNamespace(step=3200, now=4.)
    c = w.WholeBodyController.__new__(w.WholeBodyController)
    c.install_reviewed(1, 'continuous-com-causal')
    def arr(k, shape):
        x = np.zeros(shape)
        if k == 20: x[:] = 100.
        return x
    c.w = SimpleNamespace(h=None, mass=10., arr=arr,
        feet=lambda:[dict(body=2), dict(body=3)],
        l=SimpleNamespace(lh_dim=lambda h, k:state.step, lh_time=lambda h:state.now))
    c.phase_duration = lambda phase:dict(transfer=4., lift=.8, translate=1.2, land=1., settle=1.52)[phase]
    c.swing_side = 0; c.stance_side = 1
    c._seek_jet = (0., 0., 0.)
    c.arm_posture = np.zeros(6); c.arm_velocity = np.zeros(6); c.arm_acceleration = np.zeros(6)
    c._robust_clock = (1., 0., 0.)
    c._wholebody_survivors = (1, 2, 3)
    return c, state


class WholeBodyOwnership(unittest.TestCase):
    def test_disabled_transform_is_exact_object_without_plan_world_or_history(self):
        c, _ = controller(); context = fixture_context()
        with patch.object(c, '_activate_wholebody', side_effect=AssertionError('activation')):
            self.assertIs(c.transform_reviewed_reference_context(context), context)
        self.assertFalse(c._wholebody_attempted)
        self.assertEqual(len(c._wholebody_history), 0)

    def test_disabled_corrected_com_and_observer_and_readiness_delegate_exactly(self):
        c, _ = controller(); result = object()
        with patch.object(w.ReviewedController, 'corrected_com_reference', return_value=result) as old:
            self.assertIs(c.corrected_com_reference(2., 'transfer'), result)
            old.assert_called_once_with(2., 'transfer')
        with patch.object(w.ReviewedController, '_observe_support', return_value=result) as old:
            self.assertIs(c._observe_support(2., 'transfer'), result)
            old.assert_called_once_with(2., 'transfer')
        with patch.object(w.ReviewedController, 'robust_support_ready', return_value=True) as old:
            self.assertTrue(c.robust_support_ready({'native':'row'}))
            old.assert_called_once_with({'native':'row'})

    def test_single_sample_owns_lower_com_bodies_feet_preserving_all_headers(self):
        context = fixture_context(); sample = sample_for(context)
        changed = w.owned_context(context, sample, dict(accepted=True), pelvis_weight=0.)
        for name in ('schema', 'version', 'source_step', 'source_time', 'step_index',
                     'phase', 'elapsed', 'epoch', 'capture_constraints', 'unload_admitted',
                     'hypothetical_changes', 'causal_json', 'config_json', 'arm_gains'):
            self.assertEqual(getattr(changed, name), getattr(context, name), name)
        self.assertIs(changed.arms, context.arms)
        for x, y in zip(changed.joints.arrays(), context.joints.arrays()):
            assert_allclose(x[15:], y[15:], atol=0.)
        assert_allclose(changed.joints.position[:15], sample.lower_joints.position, atol=0.)
        assert_allclose(changed.com.position, sample.com.position, atol=0.)
        self.assertEqual([(o.body, o.kp, o.kd, o.weight) for o in changed.orientations],
            [(1, 40., 12., 4.), (4, 40., 12., 0.)])
        self.assertEqual(changed.feet[0].orientation.kp, 45.)
        self.assertEqual(changed.feet[0].orientation.weight, 7.)

    def test_actual_forecast_share_effective_reference_without_history_mutation(self):
        context = fixture_context(); changed = w.owned_context(context, sample_for(context), {}, pelvis_weight=0.)
        c, _ = controller()
        c.floorTargets = [.03, .03]; c.targets = [(np.zeros(3), np.eye(3))]*2
        c.radii = [.03, .03]; c.w.ground_height = lambda position:0.
        c._wholebody_history.append(('old',)); before = tuple(c._wholebody_history)
        forecast = prospective_unload_context(c, changed)
        self.assertIs(forecast.arms, changed.arms)
        self.assertIs(forecast.joints, changed.joints)
        self.assertIs(forecast.com, changed.com)
        self.assertIs(forecast.orientations, changed.orientations)
        self.assertEqual(forecast.capture_constraints, changed.capture_constraints)
        self.assertTrue(forecast.unload_admitted)
        self.assertEqual(forecast.hypothetical_changes,
            ('unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm'))
        self.assertEqual(tuple(c._wholebody_history), before)
        self.assertFalse(changed.unload_admitted)

    def test_completed_correction_is_not_added_again_and_pure_sample_is_reused(self):
        c, _ = controller(); c._wholebody_enabled = True
        sample = sample_for(fixture_context())
        c._wholebody_cycle = SimpleNamespace(sample=Mock(return_value=sample))
        c._robust_delta = np.array([100., 100., 100.])
        before = tuple(c._wholebody_history)
        for _ in range(2):
            result = c.corrected_com_reference(4.2, 'transfer')
            assert_allclose(result, sample.com.arrays(), atol=0.)
        self.assertEqual(c._wholebody_cycle.sample.call_count, 1)
        self.assertEqual(c._robust_clock, (1., 0., 0.))
        self.assertEqual(tuple(c._wholebody_history), before)
        self.assertEqual(c._wholebody_cycle.sample.call_args.kwargs['landing_adjustment'], (0., 0., 0.))

    def test_new_cycle_readiness_cannot_inherit_old_admission(self):
        c, _ = controller(); c._wholebody_enabled = True
        with patch.object(w.ReviewedController, 'robust_support_ready', return_value=True) as old:
            self.assertFalse(c.robust_support_ready({})); old.assert_not_called()
            c._wholebody_cycle = object()
            self.assertFalse(c.robust_support_ready({})); old.assert_not_called()
            c._wholebody_admitted = True
            self.assertTrue(c.robust_support_ready({})); old.assert_called_once()

    def test_frozen_cycle_rejects_new_baseline_correction(self):
        c, _ = controller(); c._wholebody_enabled = True; c._wholebody_cycle = object()
        with self.assertRaisesRegex(ValueError, 'another correction'):
            c.admit_robust_reference(4., np.zeros(3), 1.)

    def test_phase_handoff_captures_actual_row_once_before_controller_midpoint_update(self):
        c, state = controller(); c._wholebody_enabled = True
        c._wholebody_cycle = object(); c._wholebody_admitted = True
        row = dict(phase='land', stepIndex=1, sourceStep=state.step,
            sourceTimeS=state.now, feet=[dict(loadN=3.), dict(loadN=5.)])
        c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=80)
        self.assertEqual(c._wholebody_phase_boundary, row)
        self.assertIsNot(c._wholebody_phase_boundary, row)
        with self.assertRaisesRegex(ValueError, 'replaced'):
            c.prepare_phase_handoff('land', 'settle', row, landing_consecutive_samples=80)

    def test_missing_settle_path_never_falls_back_to_stand_q0(self):
        c, _ = controller(); c._wholebody_enabled = True; c._wholebody_cycle = object()
        with self.assertRaisesRegex(ValueError, 'missing admitted'):
            c._sample_current(0., 'stand')

    def test_settle_seek_is_embedded_once_and_later_changes_fail_closed(self):
        c, _ = controller(); c._wholebody_enabled = True
        sample = sample_for(fixture_context('settle', .01))
        c._wholebody_settle = SimpleNamespace(key='s'*64, sample=Mock(return_value=sample))
        c._wholebody_settle_seek_jet = (.001, 0., 0.)
        c._seek_jet = (.001, 0., 0.)
        self.assertIs(c._sample_current(.01, 'settle'), sample)
        self.assertEqual(c._wholebody_settle.sample.call_args.kwargs.keys(), {'runtime_arms'})
        c._seek_jet = (.00101, 0., 0.)
        with self.assertRaisesRegex(ValueError, 'lost frozen'):
            c._sample_current(.02, 'settle')
        c._seek_jet = (.001, .0001, 0.)
        with self.assertRaisesRegex(ValueError, 'lost frozen'):
            c._sample_current(.02, 'settle')


class FreshRiskAdmission(unittest.TestCase):
    def run_history(self, risk=0, missing_survivor=False):
        c, state = controller(); c._wholebody_enabled = True
        c._wholebody_cycle = SimpleNamespace(key='c'*64)
        c._robust_history.append(('old-baseline-window',))
        present = np.ones(4, dtype=bool)
        if missing_survivor: present[1] = False
        directory = tempfile.TemporaryDirectory(); self.addCleanup(directory.cleanup)
        c._wholebody_evidence_path = Path(directory.name)/'admission.json'
        with patch.object(w, 'actual_stance_patch', return_value=(np.zeros((4, 3)), present, np.zeros(4))), \
             patch.object(c, '_forecast_unload_risk', return_value=(risk, dict(mock=True))), \
             patch.object(w, 'native_bytes', side_effect=lambda world:str(state.step).encode()), \
             patch.object(c, '_native_source_bindings', return_value=dict(mock=True)):
            for i in range(21):
                state.step = 3200+4*i; state.now = 4.+.005*i
                c._observe_support(4.+.005*i, 'transfer')
                if i < 20: self.assertFalse(c._wholebody_admitted)
        return c, Path(directory.name)

    def test_21_new_commands_have21_unique_exact_native_witnesses(self):
        c, directory = self.run_history()
        self.assertTrue(c._wholebody_admitted)
        self.assertEqual(tuple(c._robust_history), (('old-baseline-window',),))
        manifest = json.loads((directory/'wholebody-risk-step-01/manifest.json').read_text())
        self.assertEqual(manifest['historySamples'], 21)
        self.assertFalse(manifest['originalAdmissionInherited'])
        self.assertEqual(len({x['sourceStep'] for x in manifest['samples']}), 21)
        self.assertAlmostEqual(manifest['historySourceEndS']-manifest['historySourceStartS'], .1)
        self.assertEqual(manifest['survivingCorners'], [1, 2, 3])
        import zlib
        for row in manifest['samples']:
            data = zlib.decompress((directory/'wholebody-risk-step-01'/row['filename']).read_bytes())
            self.assertEqual(hashlib.sha256(data).hexdigest(), row['payloadSHA256'])
            self.assertEqual(data, str(row['sourceStep']).encode())

    def test_pressure_clearance_disagreement_cannot_admit(self):
        c, directory = self.run_history(risk=-1)
        self.assertFalse(c._wholebody_admitted)
        self.assertFalse((directory/'wholebody-risk-step-01').exists())

    def test_changed_triangle_cannot_admit(self):
        c, directory = self.run_history(risk=1)
        self.assertFalse(c._wholebody_admitted)

    def test_missing_actual_survivor_cannot_admit(self):
        c, directory = self.run_history(missing_survivor=True)
        self.assertFalse(c._wholebody_admitted)


class NumericalRestorationBinding(unittest.TestCase):
    """Synthetic protocol receipts only; these never admit a native proposal."""
    def setUp(self):
        directory = tempfile.TemporaryDirectory(); self.addCleanup(directory.cleanup)
        self.root = Path(directory.name); self.path = self.root/'proposal.json'
        q = np.zeros((3, 9)).tolist(); key = w.digest(q)
        self.input = dict(explicitMockOnly=True, entryLiftElapsedS=.00125,
            support=dict(equations=[[1., 0., -1.], [-1., 0., -1.], [0., 1., -1.]]))
        recovery = dict(inputBinding=self.input, optimizerSuccess=False, optimizerCalls=1,
            terminalState='compute-cap', progressRecords=[dict(iteration=33,
                candidateSHA256=key, unadmittedIndependentKnotsRad=q)])
        rp = self.root/'recovery.json'; rp.write_text(json.dumps(recovery))
        diagnostic = dict(candidateSHA256=key, recoveryResultSHA256=w.sha(rp),
            realOptimizerCalls=0, sourceInputBinding=self.input)
        dp = self.root/'diagnostic.json'; dp.write_text(json.dumps(diagnostic))
        stage = dict(schema=1, sourceRecoverySHA256=w.sha(rp), sourceDiagnosticSHA256=w.sha(dp),
            sourceCandidateMajorIteration=33, sourceCandidateSHA256=key,
            sourceCandidateIndependentKnotsRad=q, sourceRecoveryOptimizerCalls=1,
            sourceRecoveryOptimizerSuccess=False, sourceRecoveryTermination='compute-cap',
            polishOptimizerCalls=1, numericalFeasibilityOnly=True, originalHardLimitsUnchanged=True,
            originalFootPathAndClocksUnchanged=True, analyticConstraintJets=True,
            goalOrWeightScan=False, nativeWorldConstructions=0, physicsIntegrations=0,
            maximumCorrectionRad=0., finalOriginalStrictMapRestored=True,
            declared123SampleCertificate=True)
        self.result = dict(version=w.POLISH_VERSION, inputBinding=self.input,
            twoStageFeasibilityRestoration=stage, totalProducerOptimizerCalls=2,
            independentKnotsRad=q, analyticSamples=[dict(phase=p, fraction=f, accepted=True)
                for p in w.PHASES for f in w.ANALYTIC_FRACTIONS])
        self.binding = dict(feasibilityStageFiles={role:dict(filename=p.name, SHA256=w.sha(p))
            for role, p in [('recovery', rp), ('diagnostic', dp)]})

    def check(self):
        return w.verify_feasibility_stages(self.result, self.binding, self.path)

    def test_complete_explicit_two_stage_protocol_is_verified_read_only(self):
        before = json.dumps(self.result, sort_keys=True)
        self.assertIsNone(self.check()); self.assertEqual(json.dumps(self.result, sort_keys=True), before)

    def test_failed_parent_may_not_be_replaced_or_relabelled(self):
        p = self.root/'recovery.json'; r = json.loads(p.read_text()); r['optimizerSuccess'] = True
        p.write_text(json.dumps(r))
        with self.assertRaisesRegex(ValueError, 'stage differs'):
            self.check()

    def test_exact_final_major_iterate_is_mandatory(self):
        self.result['twoStageFeasibilityRestoration']['sourceCandidateMajorIteration'] = 32
        with self.assertRaisesRegex(ValueError, 'semantics'):
            self.check()

    def test_new_seed_and_changed_problem_input_are_rejected(self):
        self.result['inputBinding'] = dict(otherInput=True)
        with self.assertRaisesRegex(ValueError, 'problem input'):
            self.check()
        self.result['inputBinding'] = self.input
        self.result['twoStageFeasibilityRestoration']['sourceCandidateIndependentKnotsRad'][0][0] = .001
        with self.assertRaisesRegex(ValueError, 'exact final'):
            self.check()

    def test_nonfinite_or_wrong_correction_is_rejected(self):
        for value in (float('nan'), float('inf'), .001, None):
            self.result['twoStageFeasibilityRestoration']['maximumCorrectionRad'] = value
            with self.assertRaisesRegex(ValueError, 'correction'):
                self.check()

    def test_missing_strict_full_grid_or_changed_map_is_rejected(self):
        self.result['analyticSamples'].pop()
        with self.assertRaisesRegex(ValueError, '123-sample'):
            self.check()
        self.result['twoStageFeasibilityRestoration']['finalOriginalStrictMapRestored'] = False
        with self.assertRaisesRegex(ValueError, 'semantics'):
            self.check()

    def test_stage_paths_cannot_escape_the_proposal_directory(self):
        self.binding['feasibilityStageFiles']['recovery']['filename'] = '../recovery.json'
        with self.assertRaisesRegex(ValueError, 'basename'):
            self.check()

    def test_polish_profile_cannot_omit_failed_parent_and_stage_semantics(self):
        self.result.pop('twoStageFeasibilityRestoration')
        with self.assertRaisesRegex(ValueError, 'explicit two-stage'):
            self.check()
        self.result.pop('version')
        with self.assertRaisesRegex(ValueError, 'explicit two-stage'):
            self.check()
        self.result.pop('totalProducerOptimizerCalls')
        self.binding['frozenSourceHashes'] = {'wholebody_feasibility_polish.py':'a'*64}
        with self.assertRaisesRegex(ValueError, 'explicit two-stage'):
            self.check()

    def make_refined(self):
        durations = [.8-self.input['entryLiftElapsedS'], 1.2, 1.]
        failures = [dict(phase='lift', fraction=.8, commandAccelerationMps2=.404),
            dict(phase='translate', fraction=.625, supportCaptureMarginM=.00289),
            dict(phase='land', fraction=.725, supportCaptureMarginM=.0029999)]
        parent = dict(version=w.POLISH_VERSION, optimizerSuccess=True, accepted=False,
            optimizerCalls=1, inputBinding=self.input, analyticFailures=failures,
            twoStageFeasibilityRestoration=self.result['twoStageFeasibilityRestoration'],
            path=dict(phaseDurationsS=durations), analyticSamples=self.result['analyticSamples'],
            unadmittedFinalIndependentKnotsRad=self.result['independentKnotsRad'],
            finalCandidateSHA256=w.digest(self.result['independentKnotsRad']))
        p = self.root/'failed-polish.json'; p.write_text(json.dumps(parent))
        self.binding['feasibilityStageFiles']['failedPolish'] = dict(filename=p.name, SHA256=w.sha(p))
        brackets = [dict(phase=r['phase'], worstSampleFraction=r['fraction'],
            bracketFractions=[r['fraction']-.025, r['fraction'], r['fraction']+.025]) for r in failures]
        answers = [dict(phase=r['phase'], success=True, fraction=r['fraction']+.001,
            minimumSlack=-.001, timeS=(r['fraction']+.001)*duration, evaluations=10, iterations=10)
            for r, duration in zip(failures, durations)]
        selection = dict(sourceFailedPolishSHA256=w.sha(p),
            selectionInputCandidateSHA256=parent['finalCandidateSHA256'], realTrajectoryOptimizerCalls=0,
            scalarSearchCalls=3, sourceBrackets=brackets, bracketExtremaResults=answers,
            passed=True, originalStrictMapUsed=True,
            strictSourceEvaluationRows=[dict(phase=a['phase'], fraction=a['fraction'], minimumSlack=a['minimumSlack'],
                maximumPoseResidual=1e-14, originalStrictMapUsed=True) for a in answers for _ in range(a['evaluations'])])
        sp = self.root/'source-selection.json'; sp.write_text(json.dumps(selection))
        self.binding['feasibilityStageFiles']['sourceSelection'] = dict(filename=sp.name, SHA256=w.sha(sp))
        self.result.update(version=w.REFINED_POLISH_VERSION, totalProducerOptimizerCalls=3,
            collocationRefinement=dict(schema=1, **selection, selectionReceiptSHA256=w.sha(sp),
                addedConstraintNodes=[dict(phase=r['phase'], fraction=r['fraction']) for r in answers],
                selectionPolicy='worst-failing-bracket-extremum-per-phase-v1', anchorUnchanged=True,
                onlyOneRefinement=True, intermediateFailedPolishOptimizerCalls=1, refinementOptimizerCalls=1))
        rows = [dict(phase=phase, fraction=float(f), accepted=True)
            for phase, duration in zip(w.PHASES, durations)
            for f in np.linspace(0., 1., int(np.ceil(duration/.005))+1)]
        metrics = ['COMSpeed', 'COMAcceleration', *['COMFace'+str(i) for i in range(3)],
            *['captureFace'+str(i) for i in range(3)]]
        for row in rows: row['timeExtremaScalarSlacks'] = {m:.01 for m in metrics}
        self.result.update(path=dict(phaseDurationsS=durations), denseCommandSamples=rows,
            denseCommandGridCertificate=dict(passed=True, originalStrictMapUsed=True,
                maximumStepS=.005, sampleCount=len(rows)),
            extremaVerification=dict(passed=True, wholePathProven=False, originalStrictMapUsed=True,
                metrics=metrics, searchCount=0, evaluatedSampleCount=0,
                phaseMetricCoverage=[dict(phase=p, metric=m, allDenseValuesAvailable=True,
                    minimumDenseSlack=.01, bracketCount=0) for p in w.PHASES for m in metrics]),
            timeExtremaSearches=[], extremaCommandSamples=[])

    def test_one_refinement_requires_causal_parent_and_complete_dense_grid(self):
        self.make_refined(); self.assertIsNone(self.check())
        self.result['denseCommandSamples'].pop(1)
        self.result['denseCommandGridCertificate']['sampleCount'] -= 1
        with self.assertRaisesRegex(ValueError, 'gap'):
            self.check()

    def test_refinement_cannot_change_nodes_or_downgrade_profile(self):
        self.make_refined()
        self.result['collocationRefinement']['addedConstraintNodes'][0]['fraction'] = .75
        with self.assertRaisesRegex(ValueError, 'causal bracket'):
            self.check()
        self.result['version'] = w.POLISH_VERSION
        with self.assertRaisesRegex(ValueError, 'downgraded'):
            self.check()

    def test_refinement_requires_numerical_extrema_and_retains_finite_claim(self):
        self.make_refined(); self.result['extremaVerification']['wholePathProven'] = True
        with self.assertRaisesRegex(ValueError, 'overclaimed'):
            self.check()
        self.result['extremaVerification'] = dict(passed=False, wholePathProven=False)
        with self.assertRaisesRegex(ValueError, 'absent'):
            self.check()

    def test_dense_grid_cannot_shorten_original_clocks_to_hide_missing_commands(self):
        self.make_refined(); self.result['path']['phaseDurationsS'] = [.001]*3
        with self.assertRaisesRegex(ValueError, 'original phase clocks'):
            self.check()

    def test_extrema_pass_flags_cannot_replace_numeric_coverage(self):
        self.make_refined(); self.result['extremaVerification']['phaseMetricCoverage'] = []
        with self.assertRaisesRegex(ValueError, 'coverage/count'):
            self.check()

    def make_exchanged(self):
        self.make_refined(); parent = json.loads(json.dumps(self.result))
        candidate = parent['independentKnotsRad']; key = w.digest(candidate)
        parent.update(accepted=False, optimizerSuccess=True, optimizerCalls=1,
            finalCandidateSHA256=key, unadmittedFinalIndependentKnotsRad=candidate)
        p = self.root/'rejected-refinement.json'; p.write_text(json.dumps(parent))
        self.result.update(exchangeHistory=[dict(filename=p.name, SHA256=w.sha(p),
            finalCandidateSHA256=key, optimizerCalls=1,
            objectiveAnchorSHA256=self.result['twoStageFeasibilityRestoration']['sourceCandidateSHA256'],
            initialContinuationCandidateSHA256=key, hardLimitsUnchanged=True)],
            initialContinuationIndependentKnotsRad=candidate,
            initialContinuationCandidateSHA256=key, totalProducerOptimizerCalls=4)
        intervals = [[0., .25], [.25, .5], [.5, .75], [.75, 1.]]
        eq = self.input['support']['equations']; original = 13*(50+2*len(eq))
        entries = [dict(pointKind=kind, faceIndex=face, intervalFractions=interval)
            for kind in ('COM', 'capture') for face in range(len(eq)) for interval in intervals]
        for i, entry in enumerate(entries): entry['rowIndex'] = original+2+i
        self.result.update(globalTranslateSupportEnvelopeCoverage=dict(schema=1, phase='translate',
            sourceSupportEquationsSHA256=w.digest(eq), sourceSupportEquations=eq,
            faceCount=len(eq), pointKinds=['COM', 'capture'], originalSupportReserveM=.003,
            intervalsFractions=intervals, constraints=entries, closedIntervalEndpointsCompared=True,
            wholePathProven=False), activeConstraintRowCounts=dict(original=original,
                retainedLiftLandEnvelopes=2, globalTranslateEnvelopes=len(entries), total=original+2+len(entries)))

    def test_exchange_reuses_profile_and_requires_every_whole_phase_face(self):
        self.make_exchanged(); self.assertIsNone(self.check())
        self.result['globalTranslateSupportEnvelopeCoverage']['constraints'].pop()
        with self.assertRaisesRegex(ValueError, 'every original translate'):
            self.check()

    def test_exchange_cannot_select_a_different_continuation_seed(self):
        self.make_exchanged()
        self.result['initialContinuationIndependentKnotsRad'] = np.full((3, 9), .001).tolist()
        with self.assertRaisesRegex(ValueError, 'exact linked final'):
            self.check()


class PreflightAndFailClosed(unittest.TestCase):
    def test_missing_or_failed_proposal_rejects_before_world_preparation(self):
        with patch.object(w, 'prepare_impedance', side_effect=AssertionError('native prepare called')):
            with self.assertRaisesRegex(ValueError, 'absent'):
                w.main(['--output', '/tmp/no-native', '--inspector', '/tmp/no-inspector',
                    '--enable-wholebody', '--proposal', '/tmp/no-wholebody-proposal.json'])
            with tempfile.TemporaryDirectory() as directory:
                p = Path(directory)/'failed.json'; p.write_text(json.dumps(dict(optimizerSuccess=False)))
                with self.assertRaisesRegex(ValueError, 'optimizerSuccess'):
                    w.main(['--output', '/tmp/no-native', '--inspector', '/tmp/no-inspector',
                        '--enable-wholebody', '--proposal', str(p)])

    def test_count2_and10_reject_before_preflight_or_native(self):
        with patch.object(w, 'prepare_impedance', side_effect=AssertionError('native prepare')), \
             patch.object(w, 'frozen_proposal', side_effect=AssertionError('proposal')):
            for count in (2, 10):
                with self.assertRaises(SystemExit):
                    w.main(['--output', '/tmp/no-native', '--inspector', '/tmp/no-inspector',
                        '--enable-wholebody', '--count', str(count), '--proposal', '/tmp/absent'])

    def test_no_retarget_after_a_failed_activation(self):
        c, _ = controller(); c._wholebody_attempted = True
        with self.assertRaisesRegex(ValueError, 'no automatic replanning'):
            c._activate_wholebody(fixture_context())

    def test_missing_phase_boundary_rejects_before_endpoint_solve(self):
        c, _ = controller(); context = fixture_context('settle', 0.)
        with patch.object(w, 'solve_samefoot_endpoint', side_effect=AssertionError('optimizer')):
            with self.assertRaisesRegex(ValueError, 'phase boundary is absent'):
                c._activate_settle(context, context, None)
        self.assertTrue(c._wholebody_settle_attempted)

    def test_noncurrent_handoff_rejects_before_endpoint_solve(self):
        c, _ = controller(); context = fixture_context('settle', .005, now=4.005)
        c._wholebody_phase_boundary = dict(sourceStep=3200, sourceTimeS=4.)
        with patch.object(w, 'solve_samefoot_endpoint', side_effect=AssertionError('optimizer')):
            with self.assertRaisesRegex(ValueError, 'zero-to-three'):
                c._activate_settle(context, context, None)


class MockCurrentActivation(unittest.TestCase):
    def activate(self, accepted):
        @dataclass(frozen=True)
        class Map:
            ranges: tuple = ((-1., 1.), (-1., 1.))
        @dataclass(frozen=True)
        class MockSupport:
            explicitMockOnly: bool = True
        mapping = Map(); c, state = controller(); c._wholebody_enabled = True
        c.w.xml = Path('/mock/xml'); c.w.limits = np.tile([-1., 1.], (21, 1))
        c.leg_origin = np.zeros(2); c.heading = np.array([1., 0., 0.]); c._mapping = object()
        c._robust_survivors = [1, 2, 3]
        c._wholebody_proposal_json = canonical(dict(binding=dict(sourceXMLSHA256='a'*64,
            armMapSemanticSHA256=w.digest(asdict(mapping)), originalSourceLimitsRad=c.w.limits.tolist(),
            legOriginRightLeftRad=[0., 0.], stanceSide=1, swingSide=0),
            independentKnotsRad=np.zeros((3, 9)).tolist(), proposalSHA256='p'*64))
        context = fixture_context(); sample = sample_for(context)
        cycle = SimpleNamespace(key='c'*64, durations=(.8, 1.2, 1.),
            sample=Mock(return_value=sample), validate_sample=Mock(return_value=dict(accepted=True)),
            receipt=lambda:dict(pathSHA256='c'*64))
        directory = tempfile.TemporaryDirectory(); self.addCleanup(directory.cleanup)
        c._wholebody_evidence_path = Path(directory.name)/'current.json'
        patches = (patch.object(w, 'live_arm_map', return_value=mapping),
            patch.object(w, 'sha', return_value='a'*64), patch.object(w, 'native_bytes', return_value=b'exact current'),
            patch.object(w, 'current_feet_model', return_value=(np.eye(4), np.eye(4))),
            patch.object(w, 'current_support', return_value=MockSupport()),
            patch.object(w, 'actual_contacts', return_value=((1, (0., 0., 0.)),)),
            patch.object(w, 'StanceRealizedIK', return_value=object()),
            patch.object(w, 'load_kinematics', return_value=object()),
            patch.object(w, 'WholeBodyCycle', return_value=cycle),
            patch.object(w, 'finite_static_admission', return_value=dict(accepted=accepted)),
            patch.object(c, '_native_source_bindings', return_value=dict(mock=True)))
        from contextlib import ExitStack
        with ExitStack() as stack:
            entered = [stack.enter_context(p) for p in patches]
            if accepted:
                c._activate_wholebody(context)
            else:
                with self.assertRaisesRegex(ValueError, 'static admission failed'):
                    c._activate_wholebody(context)
            self.assertEqual(entered[8].call_count, 1)
            self.assertEqual(entered[9].call_count, 1)
        return c, cycle, context

    def test_static_rejection_cannot_install_reference_or_retry(self):
        c, _, context = self.activate(False)
        self.assertIsNone(c._wholebody_cycle)
        self.assertTrue(c._wholebody_attempted)
        self.assertFalse(c._wholebody_receipt['accepted'])
        self.assertFalse(c._wholebody_receipt['finiteStatic']['accepted'])
        retained = c._wholebody_receipt['retainedActivationInput']
        receipt = json.loads(c._wholebody_evidence_path.with_name(retained['filename']).read_text())
        self.assertEqual(receipt['referenceContext'], json.loads(json.dumps(context.receipt())))
        import zlib
        self.assertEqual(zlib.decompress(c._wholebody_evidence_path.with_name(receipt['retainedPayload']).read_bytes()),
            b'exact current')
        with self.assertRaisesRegex(ValueError, 'no automatic replanning'):
            c._activate_wholebody(context)

    def test_current_activation_retains_exact_payload_and_starts_empty_new_history(self):
        c, cycle, context = self.activate(True)
        self.assertIs(c._wholebody_cycle, cycle)
        self.assertFalse(c._wholebody_admitted)
        self.assertEqual(len(c._wholebody_history), 0)
        self.assertEqual(len(c._wholebody_payloads), 0)
        self.assertEqual(c._wholebody_receipt['actualPayloadSHA256'],
            hashlib.sha256(b'exact current').hexdigest())
        self.assertEqual(c._wholebody_receipt['activationContextSHA256'], context.receipt()['contextSHA256'])
        self.assertEqual(len(c._wholebody_receipt['analyticChecks']), 123)
        self.assertFalse(c._wholebody_receipt['originalAdmissionInherited'])
        self.assertEqual(c._wholebody_receipt['optimizerCalls'], 0)


class ProposalBinding(unittest.TestCase):
    def proposal(self, directory):
        result = dict(optimizerSuccess=True, finiteSampledKinematicPass=True,
            desiredEarlyKneeCorrectionPresent=True, optimizerCalls=1, optimizerException=None,
            independentKnotsRad=np.zeros((3, 9)).tolist(), analyticFailures=[])
        path = Path(directory)/'proposal.json'; path.write_text(json.dumps(result))
        binding = dict(schema=1, proposalSHA256=w.sha(path),
            modelSHA256=w.sha(w.ROOT/'rust/crates/model/data/getup-v1.json'), sourceXMLSHA256='a'*64,
            armMapSemanticSHA256='b'*64, originalSourceLimitsRad=np.tile([-1., 1.], (21, 1)).tolist(),
            legOriginRightLeftRad=[0., 0.], stanceSide=1, swingSide=0,
            frozenSourceHashes={name:w.sha(w.source_path(name)) for name in w.CRITICAL_PROPOSAL_SOURCES})
        annotation = Path(directory)/'binding.json'; annotation.write_text(json.dumps(binding))
        return path, annotation, binding

    def test_preflight_uses_immutable_knots_and_separate_exact_annotation(self):
        with tempfile.TemporaryDirectory() as directory:
            path, annotation, binding = self.proposal(directory)
            original = path.read_bytes()
            frozen = json.loads(w.frozen_proposal(path, annotation))
            self.assertEqual(frozen['binding'], binding)
            self.assertEqual(frozen['flightOptimizerCallsAtRuntime'], 0)
            self.assertEqual(path.read_bytes(), original)

    def test_source_mutation_annotation_mismatch_or_incomplete_binding_reject(self):
        with tempfile.TemporaryDirectory() as directory:
            path, annotation, binding = self.proposal(directory)
            binding['frozenSourceHashes']['wholebody_cycle_reference.py'] = '0'*64
            annotation.write_text(json.dumps(binding))
            with self.assertRaisesRegex(ValueError, 'proposal source differs'):
                w.frozen_proposal(path, annotation)
            binding['proposalSHA256'] = '0'*64
            annotation.write_text(json.dumps(binding))
            with self.assertRaisesRegex(ValueError, 'retained proposal bytes'):
                w.frozen_proposal(path, annotation)
            path.write_text(json.dumps(dict(optimizerSuccess=True, finiteSampledKinematicPass=True,
                desiredEarlyKneeCorrectionPresent=True, optimizerCalls=1,
                independentKnotsRad=np.zeros((3, 9)).tolist(), analyticFailures=[])))
            with self.assertRaisesRegex(ValueError, 'runtime proposal binding'):
                w.frozen_proposal(path)


if __name__ == '__main__':
    unittest.main()
