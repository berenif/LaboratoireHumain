#!/usr/bin/env python3
"""Opt-in, unqualified ONE-step stance-realized whole-body native pilot.

No optimizer is invoked for flight. One immutable offline 27-coordinate proposal
must pass preflight BEFORE world preparation and must be reconstructed and
independently admitted against the current prepared transfer command. The causal
same-foot settle endpoint has exactly one runtime solve, after actual landing.
No live pose/root installation, inherited profile qualification or cross-step
ownership is provided. Without --enable-wholebody, use the exact causal baseline.
"""
import os
# Set before NumPy/SciPy import, including imports through the baseline owner.
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
from collections import deque
from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import time
import zlib

import numpy as np
from source_step_probe import ROOT, clock, sha
from source_reviewed_walk import ReviewedController, ReviewedSteps
from source_contact_impedance_probe import prepare_impedance
from source_arm_knee_mobility import native_bytes
from source_robust_support_steps import actual_stance_patch
from source_reference_context import ReferenceContext, Jet as ContextJet, Orientation, Foot, canonical, digest
from coupled_posture_reference import Jet, CurrentSupport, chart, rotation
from coupled_posture_kinematics import ArmMap, MatrixJet, P, pose_jet_to_source
from phase_leg_reference import AdvanceWindow, MeasuredAdvanceSample
from wholebody_cycle_reference import WholeBodyCycle, StanceRealizedIK, PHASES, FEET
from wholebody_settle_reference import SameFootSettlePlan, solve_samefoot_endpoint
from wholebody_static_admission import finite_static_admission

PROFILE = 'reviewed-stance-realized-wholebody-one-step-v1'
PROCESS_CAP_S = 1200.
ANALYTIC_FRACTIONS = tuple(float(x) for x in np.linspace(0., 1., 41))
STATIC_FRACTIONS = (0., .25, .5, .75, 1.)
CRITICAL_PROPOSAL_SOURCES = ('wholebody_cycle_reference.py', 'wholebody_horizon_solver.py',
    'coupled_posture_kinematics.py', 'coupled_posture_reference.py',
    'phase_leg_reference.py', 'source_reference_context.py', 'getup-step-reference.py')
RUNTIME = ('source_reviewed_wholebody.py', 'wholebody_cycle_reference.py',
    'wholebody_horizon_solver.py', 'wholebody_settle_reference.py',
    'wholebody_static_admission.py', 'coupled_posture_kinematics.py',
    'coupled_posture_reference.py', 'phase_leg_reference.py', 'source_reference_context.py')
POLISH_VERSION = 'free-com-flight-feasibility-polish-v1'
REFINED_POLISH_VERSION = 'free-com-flight-collocation-refinement-v1'


def source_path(name):
    """Only explicit repository/shared sibling source locations are eligible."""
    for path in (Path(__file__).with_name(name), ROOT/'scripts/getup-step'/name,
                 ROOT/'scripts'/name):
        if path.is_file():
            return path.resolve()
    raise ValueError('required source is missing: '+name)


def verify_feasibility_stages(result, binding, proposal_path):
    """Verify the explicit numerical repair, never relabel its failed parent."""
    if result.get('activationRetargeting') is not None:
        verify_activation_retargeting(result, binding, proposal_path)
        return
    stage = result.get('twoStageFeasibilityRestoration')
    if stage is None:
        count = result.get('totalProducerOptimizerCalls')
        if (result.get('version') not in (None, 'free-com-flight-horizon-once-v1')
                or (count is not None and (type(count) is not int or count != 1))
                or 'wholebody_feasibility_polish.py' in binding.get('frozenSourceHashes', {})):
            raise ValueError('polish proposal lacks explicit two-stage restoration')
        return
    refined = result.get('version') == REFINED_POLISH_VERSION
    if result.get('version') not in (POLISH_VERSION, REFINED_POLISH_VERSION) or stage.get('schema') != 1:
        raise ValueError('unsupported two-stage feasibility profile')
    if (result.get('collocationRefinement') is not None) != refined:
        raise ValueError('collocation refinement profile was downgraded')
    expected = dict(sourceCandidateMajorIteration=33,
        sourceRecoveryOptimizerCalls=1, sourceRecoveryOptimizerSuccess=False,
        sourceRecoveryTermination='compute-cap', polishOptimizerCalls=1,
        numericalFeasibilityOnly=True, originalHardLimitsUnchanged=True,
        originalFootPathAndClocksUnchanged=True, analyticConstraintJets=True,
        goalOrWeightScan=False, nativeWorldConstructions=0, physicsIntegrations=0,
        finalOriginalStrictMapRestored=True, declared123SampleCertificate=True)
    if any(type(stage.get(k)) is not type(v) or stage.get(k) != v
           for k, v in expected.items()):
        raise ValueError('two-stage feasibility semantics differ')
    history = result.get('exchangeHistory', [])
    if not isinstance(history, list) or (history and not refined):
        raise ValueError('exchange history requires the existing refinement profile')
    if type(result.get('totalProducerOptimizerCalls')) is not int or result['totalProducerOptimizerCalls'] != (3+len(history) if refined else 2):
        raise ValueError('two-stage optimizer counts differ')
    files = binding.get('feasibilityStageFiles', {})
    retained = {}
    for role, key in (('recovery', 'sourceRecoverySHA256'),
                      ('diagnostic', 'sourceDiagnosticSHA256')):
        item = files.get(role, {})
        name = item.get('filename')
        if not isinstance(name, str) or Path(name).name != name:
            raise ValueError('bound feasibility stage basename required')
        path = Path(proposal_path).parent/name
        if not path.is_file() or sha(path) != stage.get(key) or sha(path) != item.get('SHA256'):
            raise ValueError('retained feasibility stage differs: '+role)
        retained[role] = json.loads(path.read_text())
    recovery = retained['recovery']; diagnostic = retained['diagnostic']
    progress = recovery.get('progressRecords', [])
    if (recovery.get('optimizerSuccess') is not False or recovery.get('optimizerCalls') != 1
            or recovery.get('terminalState') != 'compute-cap' or not progress):
        raise ValueError('failed recovery parent was relabeled')
    if (result.get('inputBinding') != recovery.get('inputBinding')
            or diagnostic.get('sourceInputBinding') != recovery.get('inputBinding')):
        raise ValueError('feasibility stages changed the original problem input')
    final = progress[-1]
    source = np.asarray(stage.get('sourceCandidateIndependentKnotsRad'), dtype=float)
    if (source.shape != (3, 9) or not np.isfinite(source).all()
            or final.get('iteration') != 33 or final.get('candidateSHA256') != digest(source.tolist())
            or stage.get('sourceCandidateSHA256') != final.get('candidateSHA256')
            or not np.array_equal(source, np.asarray(final.get('unadmittedIndependentKnotsRad')))
            or diagnostic.get('candidateSHA256') != final.get('candidateSHA256')
            or diagnostic.get('recoveryResultSHA256') != stage['sourceRecoverySHA256']
            or diagnostic.get('realOptimizerCalls') != 0):
        raise ValueError('feasibility polish did not use exact final major iterate')
    final_knots = np.asarray(result.get('independentKnotsRad'), dtype=float)
    correction = float(np.max(abs(final_knots-source)))
    declared_correction = stage.get('maximumCorrectionRad')
    if (type(declared_correction) not in (int, float) or not math.isfinite(declared_correction)
            or not math.isfinite(correction) or abs(correction-declared_correction) > 1e-14):
        raise ValueError('feasibility correction receipt differs')
    samples = result.get('analyticSamples', [])
    expected_nodes = [(phase, f) for phase in PHASES for f in ANALYTIC_FRACTIONS]
    if (len(samples) != len(expected_nodes) or any(row.get('accepted') is not True
            or row.get('phase') != phase or abs(row.get('fraction', math.inf)-f) > 1e-14
            for row, (phase, f) in zip(samples, expected_nodes))):
        raise ValueError('complete original strict 123-sample certificate required')
    if refined:
        verify_collocation_refinement(result, binding, proposal_path, stage)


def verify_collocation_refinement(result, binding, proposal_path, original_stage):
    """One causal exchange, with failed sparse-grid proof and dense recheck."""
    refinement = result['collocationRefinement']
    expected = dict(schema=1, selectionPolicy='worst-failing-bracket-extremum-per-phase-v1',
        anchorUnchanged=True, onlyOneRefinement=True,
        intermediateFailedPolishOptimizerCalls=1, refinementOptimizerCalls=1)
    if any(type(refinement.get(k)) is not type(v) or refinement.get(k) != v for k, v in expected.items()):
        raise ValueError('collocation refinement semantics differ')
    verify_exchange_history(result, proposal_path, original_stage)
    item = binding.get('feasibilityStageFiles', {}).get('failedPolish', {})
    name = item.get('filename')
    if not isinstance(name, str) or Path(name).name != name:
        raise ValueError('retained failed-polish basename required')
    path = Path(proposal_path).parent/name
    if (not path.is_file() or sha(path) != refinement.get('sourceFailedPolishSHA256')
            or sha(path) != item.get('SHA256')):
        raise ValueError('retained failed polish differs')
    parent = json.loads(path.read_text())
    if (parent.get('version') != POLISH_VERSION or parent.get('optimizerSuccess') is not True
            or parent.get('accepted') is not False or parent.get('optimizerCalls') != 1
            or parent.get('inputBinding') != result.get('inputBinding')
            or parent.get('twoStageFeasibilityRestoration', {}).get('sourceCandidateSHA256')
               != original_stage['sourceCandidateSHA256']):
        raise ValueError('collocation failed parent or anchor differs')
    failures = parent.get('analyticFailures', [])
    added = refinement.get('addedConstraintNodes', [])
    brackets = refinement.get('sourceBrackets', [])
    extrema_results = refinement.get('bracketExtremaResults', [])
    if len(added) != 3 or len(brackets) != 3 or len(extrema_results) != 3:
        raise ValueError('exactly three causal constraint nodes required')
    candidate = parent.get('unadmittedFinalIndependentKnotsRad')
    if (refinement.get('selectionInputCandidateSHA256') != digest(candidate)
            or parent.get('finalCandidateSHA256') != digest(candidate)):
        raise ValueError('source-bracket selection changed failed final candidate')
    for phase, node, bracket, answer in zip(PHASES, added, brackets, extrema_results):
        rows = [r for r in failures if r.get('phase') == phase]
        if not rows:
            raise ValueError('causal failed phase is absent')
        worst = (max(rows, key=lambda r:r['commandAccelerationMps2']) if phase == 'lift'
                 else min(rows, key=lambda r:r['supportCaptureMarginM']))
        source_rows = [r for r in parent.get('analyticSamples', []) if r.get('phase') == phase]
        index = next((i for i, row in enumerate(source_rows) if abs(row['fraction']-worst['fraction']) < 1e-14), -1)
        if not 0 < index < len(source_rows)-1:
            raise ValueError('source worst sample has no retained neighboring bracket')
        expected_bracket = [source_rows[i]['fraction'] for i in (index-1, index, index+1)]
        given = bracket.get('bracketFractions', [])
        fraction = answer.get('fraction', math.nan)
        minimum = answer.get('minimumSlack', math.nan)
        source_durations = parent.get('path', {}).get('phaseDurationsS', [])
        duration = source_durations[PHASES.index(phase)] if len(source_durations) == 3 else math.nan
        if (bracket.get('phase') != phase or len(given) != 3
                or max(abs(np.asarray(given)-expected_bracket)) > 1e-14
                or abs(bracket.get('worstSampleFraction', math.inf)-worst['fraction']) > 1e-14
                or answer.get('phase') != phase or answer.get('success') is not True
                or not math.isfinite(fraction) or not expected_bracket[0] <= fraction <= expected_bracket[2]
                or not math.isfinite(minimum) or minimum >= 0.
                or type(answer.get('evaluations')) is not int or answer['evaluations'] < 1
                or not math.isfinite(duration) or not math.isfinite(answer.get('timeS', math.nan))
                or abs(answer['timeS']-fraction*duration) > 1e-10
                or node.get('phase') != phase or abs(node.get('fraction', math.inf)-fraction) > 1e-14):
            raise ValueError('constraint node is not the retained causal bracket extremum')
    selection_file = binding.get('feasibilityStageFiles', {}).get('sourceSelection', {})
    selection_name = selection_file.get('filename')
    if not isinstance(selection_name, str) or Path(selection_name).name != selection_name:
        raise ValueError('bound source-selection basename required')
    selection_path = Path(proposal_path).parent/selection_name
    if (not selection_path.is_file() or sha(selection_path) != refinement.get('selectionReceiptSHA256')
            or sha(selection_path) != selection_file.get('SHA256')):
        raise ValueError('source-selection receipt differs')
    selection = json.loads(selection_path.read_text())
    if (selection.get('realTrajectoryOptimizerCalls') != 0 or selection.get('scalarSearchCalls') != 3
            or selection.get('passed') is not True or selection.get('originalStrictMapUsed') is not True
            or any(selection.get(k) != refinement.get(k) for k in ('sourceFailedPolishSHA256',
                'selectionInputCandidateSHA256', 'sourceBrackets', 'bracketExtremaResults'))):
        raise ValueError('source-selection provenance differs')
    selection_rows = selection.get('strictSourceEvaluationRows', [])
    if (sum(a['evaluations'] for a in extrema_results) != len(selection_rows)
            or any(r.get('originalStrictMapUsed') is not True
                or not math.isfinite(r.get('minimumSlack', math.nan))
                or not 0. <= r.get('maximumPoseResidual', math.inf) < 2e-11 for r in selection_rows)):
        raise ValueError('source-selection numeric evaluation evidence differs')
    for answer in extrema_results:
        if not any(r.get('phase') == answer['phase']
                and abs(r.get('fraction', math.inf)-answer['fraction']) < 1e-14
                and abs(r['minimumSlack']-answer['minimumSlack']) < 1e-14 for r in selection_rows):
            raise ValueError('selected source extremum has no retained strict evaluation')
    verify_finite_path_certificate(result, parent_durations=parent.get('path', {}).get('phaseDurationsS', []))


def verify_finite_path_certificate(result, *, parent_durations=None):
    """Reuse the original strict command-grid/numerical-extrema contract."""
    grid = result.get('denseCommandGridCertificate', {})
    if (grid.get('passed') is not True or grid.get('originalStrictMapUsed') is not True
            or not 0. < grid.get('maximumStepS', math.inf) <= .005+1e-14):
        raise ValueError('strict 200Hz command-grid certificate required')
    rows = result.get('denseCommandSamples', [])
    durations = result.get('path', {}).get('phaseDurationsS', [])
    entry_elapsed = result.get('inputBinding', {}).get('entryLiftElapsedS')
    if (type(entry_elapsed) not in (float, int) or not math.isfinite(entry_elapsed)
            or not 0. <= entry_elapsed < .8 or len(durations) != 3
            or (parent_durations is not None and not np.array_equal(np.asarray(durations), np.asarray(parent_durations)))
            or max(abs(np.asarray(durations)-[.8-entry_elapsed, 1.2, 1.])) > 1e-14):
        raise ValueError('dense command-grid changed original phase clocks')
    if grid.get('sampleCount') != len(rows) or any(r.get('accepted') is not True for r in rows):
        raise ValueError('dense command-grid samples differ')
    for phase, duration in zip(PHASES, durations):
        fractions = [r.get('fraction', math.nan) for r in rows if r.get('phase') == phase]
        if (not math.isfinite(duration) or duration <= 0. or len(fractions) < 2
                or fractions[0] != 0. or fractions[-1] != 1.
                or any(not math.isfinite(f) for f in fractions)
                or any(not 0. < (b-a)*duration <= .005+1e-12 for a, b in zip(fractions, fractions[1:]))):
            raise ValueError('dense command-grid phase has a gap or wrong endpoints')
    extrema = result.get('extremaVerification', {})
    if (extrema.get('passed') is not True or extrema.get('wholePathProven') is not False
            or extrema.get('originalStrictMapUsed') is not True):
        raise ValueError('bounded numerical extrema check is absent or overclaimed')
    face_count = len(result['inputBinding'].get('support', {}).get('equations', []))
    metrics = ['COMSpeed', 'COMAcceleration', *['COMFace'+str(i) for i in range(face_count)],
        *['captureFace'+str(i) for i in range(face_count)]]
    coverage = extrema.get('phaseMetricCoverage', [])
    searches = result.get('timeExtremaSearches', [])
    evaluated = result.get('extremaCommandSamples', [])
    if (face_count < 3 or extrema.get('metrics') != metrics
            or len(coverage) != len(PHASES)*len(metrics)
            or extrema.get('searchCount') != len(searches)
            or extrema.get('evaluatedSampleCount') != len(evaluated)):
        raise ValueError('numeric extrema coverage/count evidence is incomplete')
    expected_searches = []
    for item, (phase, metric) in zip(coverage, [(p, m) for p in PHASES for m in metrics]):
        phase_rows = [r for r in rows if r['phase'] == phase]
        values = [r.get('timeExtremaScalarSlacks', {}).get(metric, math.nan) for r in phase_rows]
        brackets = [j for j in range(1, len(values)-1) if values[j] < values[j-1] and values[j] <= values[j+1]]
        if (item.get('phase') != phase or item.get('metric') != metric
                or item.get('allDenseValuesAvailable') is not True
                or not all(math.isfinite(v) and v >= -1e-10 for v in values)
                or not math.isfinite(item.get('minimumDenseSlack', math.nan))
                or abs(item['minimumDenseSlack']-min(values)) > 1e-14
                or item.get('bracketCount') != len(brackets)):
            raise ValueError('numeric extrema dense minima or brackets differ')
        duration = durations[PHASES.index(phase)]
        expected_searches.extend((phase, metric,
            [phase_rows[k]['fraction']*duration for k in (j-1, j, j+1)]) for j in brackets)
    if len(searches) != len(expected_searches):
        raise ValueError('numeric extrema searches omit a dense minimum bracket')
    for search, (phase, metric, bracket) in zip(searches, expected_searches):
        given = search.get('bracketS', [])
        minimum = search.get('minimumSlack', math.nan)
        if (search.get('phase') != phase or search.get('metric') != metric or search.get('success') is not True
                or len(given) != 3 or max(abs(np.asarray(given)-bracket)) > 1e-12
                or not math.isfinite(minimum) or minimum < -1e-10
                or not bracket[0] <= search.get('timeS', math.inf) <= bracket[2]
                or type(search.get('evaluations')) is not int or search['evaluations'] < 1):
            raise ValueError('bounded numerical extrema search failed or differs')
    if (sum(s['evaluations'] for s in searches) != len(evaluated)
            or any(r.get('accepted') is not True or r.get('originalStrictMapEvaluationPassed') is not True
                or any(not math.isfinite(r.get('timeExtremaScalarSlacks', {}).get(m, math.nan))
                    or r['timeExtremaScalarSlacks'][m] < -1e-10 for m in metrics) for r in evaluated)):
        raise ValueError('strict numeric extrema evaluation evidence differs')


def verify_exchange_history(result, proposal_path, original_stage):
    """Reuse one profile for deterministic, source-bound constraint exchange."""
    history = result.get('exchangeHistory', [])
    if not history:
        if result.get('globalTranslateSupportEnvelopeCoverage') is not None:
            raise ValueError('global face coverage lacks its deterministic exchange parent')
        return
    last_knots = None
    for index, entry in enumerate(history):
        name = entry.get('filename')
        if not isinstance(name, str) or Path(name).name != name:
            raise ValueError('bound exchange-parent basename required')
        path = Path(proposal_path).parent/name
        if not path.is_file() or sha(path) != entry.get('SHA256'):
            raise ValueError('exchange parent receipt differs')
        parent = json.loads(path.read_text())
        last_knots = np.asarray(parent.get('unadmittedFinalIndependentKnotsRad'), dtype=float)
        if (last_knots.shape != (3, 9) or not np.isfinite(last_knots).all()
                or parent.get('version') != REFINED_POLISH_VERSION or parent.get('accepted') is not False
                or parent.get('optimizerSuccess') is not True or parent.get('optimizerCalls') != 1
                or parent.get('totalProducerOptimizerCalls') != 3+index
                or parent.get('exchangeHistory', []) != history[:index]
                or parent.get('inputBinding') != result.get('inputBinding')
                or parent.get('finalCandidateSHA256') != digest(last_knots.tolist())
                or entry.get('finalCandidateSHA256') != digest(last_knots.tolist())
                or entry.get('initialContinuationCandidateSHA256') != digest(last_knots.tolist())
                or entry.get('objectiveAnchorSHA256') != original_stage['sourceCandidateSHA256']
                or entry.get('optimizerCalls') != 1 or entry.get('hardLimitsUnchanged') is not True):
            raise ValueError('exchange changed a failed parent, continuation or objective')
    continuation = np.asarray(result.get('initialContinuationIndependentKnotsRad'), dtype=float)
    if (not np.array_equal(continuation, last_knots)
            or result.get('initialContinuationCandidateSHA256') != digest(last_knots.tolist())):
        raise ValueError('exchange did not continue the exact linked final candidate')
    coverage = result.get('globalTranslateSupportEnvelopeCoverage', {})
    equations = result['inputBinding'].get('support', {}).get('equations', [])
    count = len(equations); intervals = [[0., .25], [.25, .5], [.5, .75], [.75, 1.]]
    expected = {(kind, face, lo, hi) for kind in ('COM', 'capture')
        for face in range(count) for lo, hi in intervals}
    records = coverage.get('constraints', [])
    actual = {(r.get('pointKind'), r.get('faceIndex'), *r.get('intervalFractions', [])) for r in records}
    original_rows = 13*(50+2*count)
    if (coverage.get('schema') != 1 or coverage.get('phase') != 'translate'
            or coverage.get('sourceSupportEquations') != equations
            or coverage.get('sourceSupportEquationsSHA256') != digest(equations)
            or coverage.get('faceCount') != count or coverage.get('pointKinds') != ['COM', 'capture']
            or coverage.get('originalSupportReserveM') != .003
            or coverage.get('intervalsFractions') != intervals
            or coverage.get('closedIntervalEndpointsCompared') is not True
            or coverage.get('wholePathProven') is not False
            or len(records) != len(expected) or actual != expected
            or {r.get('rowIndex') for r in records} != set(range(original_rows+2, original_rows+2+len(expected)))
            or result.get('activeConstraintRowCounts') != dict(original=original_rows,
                retainedLiftLandEnvelopes=2, globalTranslateEnvelopes=len(expected), total=original_rows+2+len(expected))):
        raise ValueError('exchange does not cover every original translate face/interval')


def retained_json(directory, name, expected, role):
    if not isinstance(name, str) or Path(name).name != name:
        raise ValueError('bound '+role+' basename required')
    path = Path(directory)/name
    if not path.is_file() or sha(path) != expected:
        raise ValueError('retained '+role+' bytes differ')
    return json.loads(path.read_text()), path


def verify_global_phase_support_coverage(result, *, required_phases):
    coverage = result.get('globalPhaseSupportEnvelopeCoverage', {})
    equations = result['inputBinding'].get('support', {}).get('equations', [])
    count = len(equations); intervals = [[0., .25], [.25, .5], [.5, .75], [.75, 1.]]
    expected = {(phase, kind, face, lo, hi) for phase in required_phases
        for kind in ('COM', 'capture') for face in range(count) for lo, hi in intervals}
    records = coverage.get('constraints', [])
    actual = {(r.get('phase'), r.get('pointKind'), r.get('faceIndex'),
        *r.get('intervalFractions', [])) for r in records}
    original = 13*(50+2*count)
    if (count < 3 or coverage.get('schema') != 1
            or coverage.get('phases') != list(required_phases)
            or coverage.get('sourceSupportEquations') != equations
            or coverage.get('sourceSupportEquationsSHA256') != digest(equations)
            or coverage.get('faceCount') != count or coverage.get('pointKinds') != ['COM', 'capture']
            or coverage.get('originalSupportReserveM') != .003
            or coverage.get('intervalsFractions') != intervals
            or coverage.get('closedIntervalEndpointsCompared') is not True
            or coverage.get('wholePathProven') is not False
            or len(records) != len(expected) or actual != expected
            or {r.get('rowIndex') for r in records} != set(range(original+2, original+2+len(expected)))
            or result.get('activeConstraintRowCounts') != dict(original=original,
                retainedLiftLandEnvelopes=2, globalPhaseSupportEnvelopes=len(expected),
                total=original+2+len(expected))):
        raise ValueError('retarget does not cover every original support face across all flight phases')


def verify_current_input_exchange(result, directory, seed_knots, anchor_key):
    """Continue only exact failed major iterates from this same causal input."""
    history = result.get('exchangeHistory', [])
    if not isinstance(history, list):
        raise ValueError('same-input exchange history must be a list')
    initial = seed_knots
    for index, entry in enumerate(history):
        parent, _ = retained_json(directory, entry.get('filename'), entry.get('SHA256'),
            'same-input exchange parent')
        candidate = parent.get('unadmittedFinalIndependentKnotsRad')
        candidate_key = digest(candidate)
        array = np.asarray(candidate, dtype=float)
        progress = parent.get('progressRecords', [])
        expected = dict(inputBindingSHA256=digest(result['inputBinding']),
            activationRetargetingSHA256=digest(result['activationRetargeting']),
            parentResultFilename=entry.get('filename'), parentResultSHA256=entry.get('SHA256'),
            sourceCandidateSHA256=candidate_key,
            finalCandidateSHA256=candidate_key, initialContinuationCandidateSHA256=candidate_key,
            objectiveAnchorSHA256=anchor_key, optimizerCalls=1,
            optimizerSuccess=parent.get('optimizerSuccess'), terminalState=parent.get('terminalState'),
            finalCandidateOrigin=parent.get('finalCandidateOrigin'),
            completedMajorIteration=parent.get('completedCallbackIterations'), hardLimitsUnchanged=True)
        if (array.shape != (3, 9) or not np.isfinite(array).all()
                or type(parent.get('optimizerSuccess')) is not bool
                or type(expected['completedMajorIteration']) is not int or expected['completedMajorIteration'] < 1
                or parent.get('version') != REFINED_POLISH_VERSION or parent.get('accepted') is not False
                or parent.get('inputBinding') != result['inputBinding']
                or parent.get('activationRetargeting') != result['activationRetargeting']
                or parent.get('exchangeHistory', []) != history[:index]
                or parent.get('totalProducerOptimizerCalls') != 5+index
                or parent.get('optimizerCalls') != 1
                or parent.get('finalCandidateSHA256') != candidate_key
                or parent.get('initialContinuationIndependentKnotsRad') != initial
                or any(parent.get('twoStageFeasibilityRestoration', {}).get(k) != v
                       for k, v in result['twoStageFeasibilityRestoration'].items()
                       if k != 'maximumCorrectionRad')
                or not progress or progress[-1].get('iteration') != expected['completedMajorIteration']
                or progress[-1].get('candidateSHA256') != candidate_key
                or progress[-1].get('unadmittedIndependentKnotsRad') != candidate
                or parent.get('finalCandidateOrigin') not in
                   ('last-completed-major-iteration', 'optimizer-returned-final')
                or any(type(entry.get(k)) is not type(v) or entry.get(k) != v for k, v in expected.items())):
            raise ValueError('same-input exchange changed a failed major iterate or causal problem')
        initial = candidate
    if (result.get('initialContinuationIndependentKnotsRad') != initial
            or result.get('initialContinuationCandidateSHA256') != digest(initial)):
        raise ValueError('same-input exchange did not continue its exact linked final iterate')
    return len(history)


def verify_activation_retargeting(result, binding, proposal_path):
    """An explicit change of causal input, never a relabeled old certificate."""
    transition = result['activationRetargeting']; directory = Path(proposal_path).parent
    expected = dict(schema=1, originalHardLimitsUnchanged=True,
        referenceFamilyUnchanged=True, physicalStateInstalled=False)
    if (result.get('version') != REFINED_POLISH_VERSION or result.get('accepted') is not True
            or any(type(transition.get(k)) is not type(v) or transition.get(k) != v
                   for k, v in expected.items())):
        raise ValueError('actual activation transition semantics differ')
    seed, seed_path = retained_json(directory, transition.get('sourceProposalFilename'),
        transition.get('sourceProposalSHA256'), 'source proposal')
    seed_binding, _ = retained_json(directory, transition.get('sourceProposalBindingFilename'),
        transition.get('sourceProposalBindingSHA256'), 'source proposal binding')
    if (seed.get('activationRetargeting') is not None or seed.get('accepted') is not True
            or seed_binding.get('proposalSHA256') != sha(seed_path)):
        raise ValueError('activation source is not the retained accepted offline proposal')
    verify_feasibility_stages(seed, seed_binding, seed_path)
    for key in ('modelSHA256', 'sourceXMLSHA256', 'armMapSemanticSHA256',
                'originalSourceLimitsRad', 'legOriginRightLeftRad', 'stanceSide', 'swingSide'):
        if binding.get(key) != seed_binding.get(key):
            raise ValueError('activation retarget changed original model/map/limits/sides')
    retained, _ = retained_json(directory, transition.get('retainedActivationInputFilename'),
        transition.get('retainedActivationInputSHA256'), 'actual activation input')
    context = ReferenceContext.from_receipt(retained['referenceContext'])
    support = CurrentSupport(**retained['currentSupport']); support.validate_identity(context)
    support.validate_com(context.com)
    if context.phase != 'transfer':
        raise ValueError('actual stationary transfer input required')
    captured = retained.get('proposal', {})
    seed_knots = seed.get('independentKnotsRad'); seed_key = digest(seed_knots)
    if (captured.get('proposalSHA256') != sha(seed_path)
            or captured.get('binding') != seed_binding
            or captured.get('independentKnotsRad') != seed_knots
            or captured.get('offlineReceiptSHA256') != digest(seed)
            or transition.get('sourceProposalCandidateSHA256') != seed_key
            or transition.get('initialContinuationCandidateSHA256') != seed_key):
        raise ValueError('activation continuation differs from the exact rejected current proposal')
    native_name = retained.get('retainedPayload')
    if not isinstance(native_name, str) or Path(native_name).name != native_name:
        raise ValueError('bound actual native payload basename required')
    native_path = directory/native_name
    if not native_path.is_file() or sha(native_path) != retained.get('compressedPayloadSHA256'):
        raise ValueError('retained activation native payload differs')
    payload = zlib.decompress(native_path.read_bytes())
    if (len(payload) != retained.get('actualPayloadBytes')
            or hashlib.sha256(payload).hexdigest() != retained.get('actualPayloadSHA256')):
        raise ValueError('actual activation payload identity differs')
    identity = dict(activationContextSHA256=context.receipt()['contextSHA256'],
        actualPayloadSHA256=retained['actualPayloadSHA256'],
        compressedPayloadSHA256=retained['compressedPayloadSHA256'],
        activationSourceStep=context.source_step, activationSourceTimeS=context.source_time)
    if any(transition.get(k) != v for k, v in identity.items()):
        raise ValueError('activation transition identifies a different current state')
    current = result.get('inputBinding', {})
    if (current.get('inputSHA256') != transition['retainedActivationInputSHA256']
            or current.get('contextSHA256') != identity['activationContextSHA256']
            or current.get('currentSupportSHA256') != support.key
            or current.get('actualFeetModelSHA256') != digest(retained['currentFeetModel'])
            or current.get('sourceNativeStep') != context.source_step
            or current.get('sourceTimeS') != context.source_time
            or current.get('entryLiftElapsedS') != 0.
            or current.get('heading') != json.loads(context.config_json).get('heading')
            or any(current.get(k) != binding.get(k) for k in
                   ('modelSHA256', 'sourceXMLSHA256', 'armMapSemanticSHA256'))
            or current.get('support', {}).get('equations') != support.equations().tolist()):
        raise ValueError('new proposal input does not bind the retained current geometry')
    old_stage = seed['twoStageFeasibilityRestoration']; stage = result.get('twoStageFeasibilityRestoration', {})
    if any(stage.get(k) != v for k, v in old_stage.items() if k != 'maximumCorrectionRad'):
        raise ValueError('activation retarget changed the original objective anchor or caps')
    anchor = np.asarray(old_stage['sourceCandidateIndependentKnotsRad'])
    exchange_count = verify_current_input_exchange(result, directory, seed_knots, old_stage['sourceCandidateSHA256'])
    knots = np.asarray(result.get('independentKnotsRad'))
    if (transition.get('objectiveAnchorSHA256') != old_stage['sourceCandidateSHA256']
            or knots.shape != (3, 9) or not np.isfinite(knots).all()
            or abs(stage.get('maximumCorrectionRad', math.inf)-float(np.max(abs(knots-anchor)))) > 1e-14
            or result.get('totalProducerOptimizerCalls') != seed['totalProducerOptimizerCalls']+1+exchange_count):
        raise ValueError('activation objective/correction/optimizer count differs')
    samples = result.get('analyticSamples', [])
    nodes = [(p, f) for p in PHASES for f in ANALYTIC_FRACTIONS]
    if (len(samples) != len(nodes) or any(r.get('accepted') is not True
            or r.get('phase') != p or r.get('fraction') != f for r, (p, f) in zip(samples, nodes))):
        raise ValueError('complete original strict 123-sample certificate required')
    verify_finite_path_certificate(result)
    verify_global_phase_support_coverage(result, required_phases=PHASES)


def frozen_proposal(proposal_path, binding_path=None):
    """Read-only preflight, deliberately independent of any native World.

    A separate binding annotation may bind the unchanged final offline receipt;
    it never supplies replacement knots, goals, a preset or a second solve.
    """
    path = Path(proposal_path)
    if not path.is_file():
        raise ValueError('accepted offline whole-body proposal is absent')
    result = json.loads(path.read_text())
    for name in ('optimizerSuccess', 'finiteSampledKinematicPass',
                 'desiredEarlyKneeCorrectionPresent'):
        if result.get(name) is not True:
            raise ValueError('offline whole-body proposal failed '+name)
    if result.get('optimizerCalls') != 1 or result.get('optimizerException') is not None:
        raise ValueError('exactly one completed offline horizon attempt required')
    knots = np.array(result.get('independentKnotsRad'), dtype=float)
    if knots.shape != (3, 9) or not np.isfinite(knots).all():
        raise ValueError('one finite immutable 3x9 proposal required')
    if result.get('analyticFailures') != []:
        raise ValueError('offline finite analytic admission has failures or is absent')
    if binding_path is not None:
        binding = json.loads(Path(binding_path).read_text())
        if binding.get('proposalSHA256') != sha(path):
            raise ValueError('proposal annotation differs from retained proposal bytes')
    else:
        binding = result.get('runtimeProposalBinding')
    if not isinstance(binding, dict) or binding.get('schema') != 1:
        raise ValueError('explicit runtime proposal binding required')
    verify_feasibility_stages(result, binding, path)
    for key in ('modelSHA256', 'sourceXMLSHA256', 'armMapSemanticSHA256'):
        value = binding.get(key)
        if not isinstance(value, str) or len(value) != 64:
            raise ValueError('missing runtime binding '+key)
    limits = np.array(binding.get('originalSourceLimitsRad'), dtype=float)
    origins = np.array(binding.get('legOriginRightLeftRad'), dtype=float)
    if (limits.shape != (21, 2) or origins.shape != (2,) or
            not np.isfinite(limits).all() or not np.isfinite(origins).all() or
            np.any(limits[:, 0] >= limits[:, 1])):
        raise ValueError('bound original source limits/leg origins required')
    if (type(binding.get('stanceSide')) is not int or binding['stanceSide'] not in (0, 1)
            or binding.get('swingSide') != 1-binding['stanceSide']):
        raise ValueError('bound original left/right stance and swing required')
    sources = binding.get('frozenSourceHashes', {})
    if not isinstance(sources, dict) or any(n not in sources for n in CRITICAL_PROPOSAL_SOURCES):
        raise ValueError('complete proposal-producing source binding required')
    if result.get('twoStageFeasibilityRestoration') is not None and 'wholebody_feasibility_polish.py' not in sources:
        raise ValueError('feasibility producer source binding required')
    if result.get('plannerFastGeometryUsed') is True:
        name = result.get('plannerFastGeometrySourceName')
        if (not isinstance(name, str) or Path(name).name != name
                or sources.get(name) != result.get('plannerFastGeometrySHA256')):
            raise ValueError('fast analytic planner source binding required')
    if result.get('strictCertificateCacheUsed') is True:
        name = result.get('strictCertificateCacheSourceName')
        if (name != 'strict_reference_cache.py'
                or sources.get(name) != result.get('strictCertificateCacheSHA256')):
            raise ValueError('original strict certificate cache source binding required')
    for name, expected in sources.items():
        if Path(name).name != name or sha(source_path(name)) != expected:
            raise ValueError('proposal source differs: '+name)
    if sha(ROOT/'rust/crates/model/data/getup-v1.json') != binding['modelSHA256']:
        raise ValueError('proposal FK source model differs')
    # Canonical JSON owns all retained values, with no caller aliases or edits.
    value = dict(proposalSHA256=sha(path), proposalFilename=path.name,
        binding=binding, independentKnotsRad=knots.tolist(),
        offlineReceiptSHA256=digest(result), flightOptimizerCallsAtRuntime=0)
    if result.get('activationRetargeting') is not None:
        value.update(activationRetargeting=result['activationRetargeting'],
            activationInputBinding=result['inputBinding'])
    return canonical(value)


def live_arm_map(mapping):
    """Copy the existing original spline; never fit, measure, clip or rebuild it."""
    return ArmMap.from_archived_table(dict(originalReceipt=mapping.receipt(),
        gridsBySideRad=[curve.x.tolist() for curve in mapping.curves],
        splineCoefficientsBySide=[curve.c.tolist() for curve in mapping.curves]))


def load_kinematics():
    path = ROOT/'scripts/getup-step-reference.py'
    spec = importlib.util.spec_from_file_location('wholebody_existing_source_fk', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Kinematics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()))


def current_feet_model(world):
    """Read current native body poses once, converting source Z-up to model axes."""
    positions = world.arr(3, (17, 3)).copy()
    rotations = world.arr(5, (17, 3, 3)).copy()
    feet = []
    for name in FEET:
        body = world.ids[name]; pose = np.eye(4)
        pose[:3, :3] = P.T@rotations[body]@P
        pose[:3, 3] = P.T@positions[body]
        feet.append(pose)
    return tuple(feet)


def current_support(controller, context, *, triangle=True):
    world = controller.w
    corners, present, _ = actual_stance_patch(world, controller.stance_side)
    count = world.l.lh_support_count(world.h)
    ids = tuple(int(x) for x in controller._wholebody_survivors)
    support = CurrentSupport(context.source_step, context.source_time,
        context.step_index, context.phase,
        tuple(map(tuple, world.arr(21, (128,))[:2*count].reshape(-1, 2))),
        tuple(map(tuple, corners)), tuple(bool(x) for x in present), ids, bool(triangle))
    support.validate_identity(context)
    return support


def verify_current_activation_identity(proposal, context, support, feet, payload):
    """Retargeted geometry is valid only for its exact retained causal state."""
    transition = proposal.get('activationRetargeting')
    if transition is None:
        return
    bound = proposal.get('activationInputBinding', {})
    if (context.source_step != transition.get('activationSourceStep')
            or context.source_time != transition.get('activationSourceTimeS')
            or context.receipt()['contextSHA256'] != transition.get('activationContextSHA256')
            or hashlib.sha256(payload).hexdigest() != transition.get('actualPayloadSHA256')
            or support.key != bound.get('currentSupportSHA256')
            or digest([np.asarray(p).tolist() for p in feet]) != bound.get('actualFeetModelSHA256')):
        raise ValueError('current activation differs from explicitly retargeted causal input')


def actual_contacts(controller, support, *, both_feet=False):
    """Static LP gets currently matched native points, never future contacts."""
    if not both_feet:
        return tuple((controller.stance_side, support.stance_points_xyz[i])
            for i in support.surviving_corners)
    contacts = []
    for side in (0, 1):
        corners, present, _ = actual_stance_patch(controller.w, side)
        contacts.extend((side, tuple(p)) for p, ok in zip(corners, present) if ok)
    if len(contacts) < 3:
        raise ValueError('current actual static contact points are absent')
    return tuple(contacts)


def validate_settle_sample(plan, sample, support):
    """Complete command check with original bounds; no .06 flight reserve."""
    q = np.asarray(sample.geometric.full_joints.position)
    margins = np.minimum(q-plan.limits[:, 0], plan.limits[:, 1]-q)
    delta = np.asarray(sample.geometric.future_arm_delta_right_left.position)
    ranges = np.asarray(plan.realizer.arm_map.ranges)
    checks = dict(lowerInterior=bool(min(margins[:15]) >= .01-1e-9),
        armReserve=bool(min(margins[15:]) >= .02-1e-9),
        mapReserve=bool(min(np.minimum(delta-ranges[:, 0], ranges[:, 1]-delta)) >= .02-1e-9),
        lumbar=bool(max(abs(q[:3]-np.asarray(plan.lower0.position)[:3])) <= math.radians(5.)+1e-9),
        noHyperextension=bool(max(q[[6, 12]]) <= 1e-10))
    # MatrixJet construction already checks every full foot pose/derivative.
    for foot in sample.feet:
        if not all(np.isfinite(x).all() for x in foot.arrays()):
            checks['finiteFeet'] = False
    support_check = support.validate_com(sample.com)
    if not all(checks.values()):
        raise ValueError('same-foot settle command failed original joint/map/foot admission')
    return dict(accepted=True, checks=checks, **support_check)


def owned_context(context, sample, check, *, pelvis_weight):
    """One pure task replacement. All arms/history/safety header owners survive."""
    lower = sample.lower_joints.arrays(); old = context.joints.arrays()
    joints = ContextJet.make(*(np.r_[a, b[15:]] for a, b in zip(lower, old)))
    rows = {1:sample.thorax, 4:sample.pelvis}
    orientations = []
    for row in context.orientations:
        if row.body not in rows:
            orientations.append(row); continue
        weight = pelvis_weight if row.body == 4 else row.weight
        orientations.append(Orientation.make(row.body, *rows[row.body].arrays(),
            kp=row.kp, kd=row.kd, weight=weight))
    if 4 not in [row.body for row in context.orientations]:
        orientations.append(Orientation.make(4, *sample.pelvis.arrays(),
            kp=40., kd=12., weight=pelvis_weight))
    feet = []
    for old, pose in zip(context.feet, sample.feet):
        value = pose_jet_to_source(pose)
        feet.append(Foot(ContextJet.make(*value.translation.arrays()), Orientation.make(
            old.orientation.body, np.asarray(value.rotation).reshape(3, 3),
            value.angular_velocity_world, value.angular_acceleration_world,
            kp=old.orientation.kp, kd=old.orientation.kd, weight=old.orientation.weight)))
    if len(feet) != 2:
        raise ValueError('two owned whole-body command feet required')
    diagnostics = json.loads(context.diagnostics_json)
    diagnostics['wholeBodyCycle'] = dict(profile=PROFILE, sampleAdmission=check,
        **sample.diagnostics, runtimeArmsObjectUnchanged=True, lower15Only=True,
        completedCorrectionEmbeddedOnce=True, contactSeekAppliedOnce=True)
    return replace(context, com=ContextJet.make(*sample.com.arrays()), joints=joints,
        orientations=tuple(orientations), feet=tuple(feet), diagnostics_json=canonical(diagnostics))


class WholeBodyController(ReviewedController):
    def install_reviewed(self, step_index, mode):
        super().install_reviewed(step_index, mode)
        self._wholebody_enabled = False
        self._wholebody_cycle = None; self._wholebody_settle = None
        self._wholebody_attempted = False; self._wholebody_settle_attempted = False
        self._wholebody_admitted = False; self._wholebody_survivors = ()
        self._wholebody_history = deque(maxlen=21); self._wholebody_payloads = deque(maxlen=21)
        self._wholebody_advance_samples = deque(maxlen=80)
        self._wholebody_sample_cache = None; self._wholebody_phase_boundary = None
        self._wholebody_receipt = None; self._wholebody_settle_receipt = None
        self._wholebody_settle_seek_jet = None

    def _command_identity(self, t, phase):
        base = super()._command_identity(t, phase)
        if not self._wholebody_enabled:
            return base
        return base+(None if self._wholebody_cycle is None else self._wholebody_cycle.key,
            None if self._wholebody_settle is None else self._wholebody_settle.key)

    def _save_wholebody_receipt(self):
        if not hasattr(self, '_wholebody_evidence_path'):
            return
        value = dict(schema=1, profile=PROFILE, flight=self._wholebody_receipt,
            settle=self._wholebody_settle_receipt, freshFlightAdmission=self._wholebody_admitted,
            originalAdmissionInherited=False, finalPlanQualificationClaim=False,
            oldProfileQualificationClaim=False, visualQualificationClaim=False,
            crossStepOwnershipReviewed=False, requestedSteps=1)
        self._wholebody_evidence_path.write_text(json.dumps(value, indent=2)+'\n')

    def _sample_current(self, t, phase, *, prepared_arms=None):
        arms = (prepared_arms if prepared_arms is not None else
            ContextJet.make(self.arm_posture, self.arm_velocity, self.arm_acceleration))
        seek = tuple(float(x) for x in self._seek_jet)
        key = (self.w.l.lh_dim(self.w.h, 6), self.w.l.lh_time(self.w.h),
            phase, float(t), arms, seek,
            None if self._wholebody_settle is None else self._wholebody_settle.key)
        if self._wholebody_sample_cache is not None and self._wholebody_sample_cache[0] == key:
            return self._wholebody_sample_cache[1]
        if phase in ('settle', 'stand'):
            if self._wholebody_settle is None:
                if phase != 'settle':
                    raise ValueError('missing admitted same-foot settle path before stand')
                sample = self._wholebody_cycle.sample('settle_handoff', 0.,
                    runtime_arms=arms, landing_adjustment=seek)
            else:
                frozen_seek = self._wholebody_settle_seek_jet
                if (frozen_seek is None or abs(seek[0]-frozen_seek[0]) > 1e-12
                        or abs(seek[1]) > 1e-12 or abs(seek[2]) > 1e-12):
                    raise ValueError('loaded same-foot return lost frozen contact-seek admission')
                elapsed = t if phase == 'settle' else self.phase_duration('settle')
                sample = self._wholebody_settle.sample(elapsed, runtime_arms=arms)
        else:
            sample = self._wholebody_cycle.sample(phase, t,
                runtime_arms=arms, landing_adjustment=seek)
        self._wholebody_sample_cache = (key, sample)
        return sample

    def corrected_com_reference(self, t, phase):
        if not self._wholebody_enabled or self._wholebody_cycle is None:
            return super().corrected_com_reference(t, phase)
        # Complete FK/initial-slack command already contains the old correction.
        self._robust_clock = (1., 0., 0.)
        return self._sample_current(t, phase).com.arrays()

    def admit_robust_reference(self, t, delta, duration):
        if self._wholebody_enabled and self._wholebody_cycle is not None:
            raise ValueError('frozen whole-body proposal cannot acquire another correction')
        return super().admit_robust_reference(t, delta, duration)

    def _activate_wholebody(self, context):
        if self._wholebody_attempted:
            raise ValueError('single whole-body proposal rejected; no automatic replanning')
        self._wholebody_attempted = True
        proposal = json.loads(self._wholebody_proposal_json); binding = proposal['binding']
        mapping = live_arm_map(self._mapping)
        if (sha(self.w.xml) != binding['sourceXMLSHA256'] or
                digest(asdict(mapping)) != binding['armMapSemanticSHA256'] or
                not np.array_equal(self.w.limits, np.asarray(binding['originalSourceLimitsRad'])) or
                not np.array_equal(self.leg_origin, np.asarray(binding['legOriginRightLeftRad'])) or
                self.stance_side != binding['stanceSide'] or self.swing_side != binding['swingSide']):
            raise ValueError('current source/XML/map/limits/origins/sides differ from frozen proposal')
        self._wholebody_survivors = tuple(int(x) for x in self._robust_survivors)
        support = current_support(self, context)
        before = native_bytes(self.w)
        feet = current_feet_model(self.w)
        # Preserve the causal input even if the first constructor/check rejects.
        payload_path = self._wholebody_evidence_path.with_name('wholebody-activation.native-payload.zlib')
        payload_path.write_bytes(zlib.compress(before, 1))
        input_path = self._wholebody_evidence_path.with_name('wholebody-activation-input.json')
        input_path.write_text(json.dumps(dict(referenceContext=context.receipt(),
            currentSupport=asdict(support), currentFeetModel=[p.tolist() for p in feet],
            actualPayloadSHA256=hashlib.sha256(before).hexdigest(), actualPayloadBytes=len(before),
            retainedPayload=payload_path.name, compressedPayloadSHA256=sha(payload_path),
            proposal=proposal), indent=2)+'\n')
        retained_input = dict(filename=input_path.name, SHA256=sha(input_path))
        realizer = StanceRealizedIK(load_kinematics(), self.stance_side,
            MatrixJet.rest(feet[self.stance_side]), mapping, self.leg_origin)
        checks = []; static_samples = []; static = None
        try:
            verify_current_activation_identity(proposal, context, support, feet, before)
            cycle = WholeBodyCycle(realizer, context, self.w.limits, self.heading,
                proposal['independentKnotsRad'])
            for phase, duration in zip(PHASES, cycle.durations):
                for fraction in ANALYTIC_FRACTIONS:
                    sample = cycle.sample(phase, fraction*duration)
                    check = cycle.validate_sample(sample, support, require_clearance=phase == 'translate')
                    checks.append(dict(phase=phase, fraction=fraction, **check))
                    if check.get('accepted') is not True:
                        raise ValueError('current whole-body finite analytic path admission failed')
                    if fraction in STATIC_FRACTIONS:
                        static_samples.append((phase+':'+str(fraction), sample))
            initial = cycle.sample('transfer', context.elapsed, runtime_arms=context.arms,
                landing_adjustment=json.loads(context.causal_json)['contactSeekAfter']['jet'])
            initial_check = cycle.validate_sample(initial, support)
            if initial_check.get('accepted') is not True:
                raise ValueError('current arm-conditioned activation admission failed')
            static_samples.insert(0, ('actual-arm-conditioned-activation', initial))
            static = finite_static_admission(self.w, static_samples, actual_contacts(self, support))
            if static.get('accepted') is not True:
                raise ValueError('current whole-body finite static admission failed')
        except Exception as error:
            self._wholebody_receipt = dict(accepted=False, rejection=str(error),
                proposal=proposal, retainedActivationInput=retained_input,
                analyticChecks=checks, finiteStatic=static, optimizerCalls=0)
            self._save_wholebody_receipt()
            raise
        if native_bytes(self.w) != before:
            raise ValueError('whole-body activation admission mutated actual native payload')
        self._wholebody_receipt = dict(accepted=True, proposal=proposal,
            retainedActivationInput=retained_input,
            activationSourceStep=context.source_step, activationSourceTimeS=context.source_time,
            activationContextSHA256=context.receipt()['contextSHA256'],
            actualPayloadSHA256=hashlib.sha256(before).hexdigest(), actualPayloadBytes=len(before),
            retainedPayload=payload_path.name, compressedPayloadSHA256=sha(payload_path),
            sourceBindings=self._native_source_bindings(), path=cycle.receipt(),
            provisionalSurvivors=list(self._wholebody_survivors), analyticChecks=checks,
            finiteStatic=static, optimizerCalls=0, originalAdmissionInherited=False,
            wholeNonlinearPathProven=False, everyCommandRecheckRequired=True)
        if self._wholebody_history or self._wholebody_admitted or self._wholebody_payloads:
            raise ValueError('new proposal inherited old support-risk history')
        self._wholebody_cycle = cycle
        self._wholebody_sample_cache = None
        self._save_wholebody_receipt()

    def _native_source_bindings(self):
        return dict(modelSHA256=sha(ROOT/'rust/crates/model/data/getup-v1.json'),
            sourceXMLSHA256=sha(self.w.xml), helperSHA256=sha(self.w.helper),
            bridgeSHA256=sha(self.w.base/'source_bridge_v2.so'),
            engineSHA256=sha(self.w.base/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'),
            runtimeSHA256={name:sha(source_path(name)) for name in RUNTIME})

    def prepare_phase_handoff(self, old_phase, new_phase, row, *, landing_consecutive_samples):
        super().prepare_phase_handoff(old_phase, new_phase, row,
            landing_consecutive_samples=landing_consecutive_samples)
        if not self._wholebody_enabled or new_phase != 'settle':
            return
        if (old_phase != 'land' or self._wholebody_cycle is None or not self._wholebody_admitted
                or self._wholebody_phase_boundary is not None
                or row.get('phase') != 'land' or row.get('stepIndex') != self._reviewed_step_index
                or type(landing_consecutive_samples) is not int or landing_consecutive_samples < 80
                or any(foot['loadN'] < 3. for foot in row['feet'])
                or row['sourceStep'] != self.w.l.lh_dim(self.w.h, 6)
                or abs(row['sourceTimeS']-self.w.l.lh_time(self.w.h)) > 1e-10):
            raise ValueError('invalid or replaced actual whole-body landing boundary')
        # The runner retains ownership of its measured midpoint+35mm finalCom
        # update immediately AFTER this callback. Capture, never precompute it.
        self._wholebody_phase_boundary = json.loads(canonical(row))

    def _activate_settle(self, context, outgoing, support):
        if self._wholebody_settle_attempted:
            raise ValueError('same-foot endpoint/path rejected; no retry or hybrid return')
        self._wholebody_settle_attempted = True
        if self._wholebody_phase_boundary is None:
            raise ValueError('actual landing phase boundary is absent')
        boundary = self._wholebody_phase_boundary
        lag = context.source_step-boundary['sourceStep']
        if (lag not in range(4) or abs(context.elapsed-lag*.00125) > 1e-10 or
                abs(context.source_time-boundary['sourceTimeS']-context.elapsed) > 1e-9):
            raise ValueError('first settle command is not current boundary plus zero-to-three samples')
        feet = current_feet_model(self.w)
        prior = self._wholebody_cycle.realizer
        realizer = StanceRealizedIK(prior.kin, self.stance_side,
            MatrixJet.rest(feet[self.stance_side]), prior.arm_map, self.leg_origin)
        before = native_bytes(self.w)
        seek = tuple(float(x) for x in self._seek_jet)
        if (abs(seek[1]) > 1e-12 or abs(seek[2]) > 1e-12
                or any(outgoing.joints.velocity[:15]) or any(outgoing.joints.acceleration[:15])):
            raise ValueError('same-foot family requires rest incoming lower/seek jets; no zeroing')
        identity = dict(sourceStep=context.source_step, sourceTimeS=context.source_time,
            actualStateSHA256=hashlib.sha256(before).hexdigest(),
            modelSHA256=sha(ROOT/'rust/crates/model/data/getup-v1.json'), sourceXMLSHA256=sha(self.w.xml))
        heading = chart(next(row for row in outgoing.orientations if row.body == 1).arrays()[0])[0]
        endpoint = solve_samefoot_endpoint(realizer.model, self.w.limits, realizer.arm_map,
            self.leg_origin, stance_side=self.stance_side, current_feet_model=feet,
            measured_com_target=self.finalCom.copy(), initial_lower=outgoing.joints.position[:15],
            lumbar_anchor=outgoing.joints.position[:3],
            thorax_preference_source=rotation([heading, math.radians(3.), 0.]),
            pelvis_preference_source=rotation([heading, 0., 0.]),
            actual_hull_equations=support.equations(), source_identity=identity, heading_rad=heading)
        self._wholebody_settle_receipt = dict(accepted=False, endpoint=endpoint.receipt(),
            phaseBoundary=boundary, firstCommandIdentity=identity,
            incomingReferenceContext=context.receipt(), outgoingReferenceContext=outgoing.receipt(),
            originalDurationS=1.52, remainingDurationS=1.52-context.elapsed,
            contactSeekOwnershipTransition='current prepared seek embedded once in outgoing slacks; coupled same-actual-foot return then owns feet',
            initialOwnedSeekJet=seek, laterSeekMustRemainRestAtSameDepth=True)
        self._save_wholebody_receipt()
        if not endpoint.accepted:
            raise ValueError('single current same-foot endpoint rejected')
        plan = SameFootSettlePlan(realizer, outgoing, endpoint, feet[self.swing_side],
            self.w.limits, current_identity=identity, phase_boundary=boundary,
            duration=self.phase_duration('settle'))
        admission = plan.prepare(support, samples=41)
        if admission.get('accepted') is not True:
            self._wholebody_settle_receipt['finiteAdmission'] = admission
            self._save_wholebody_receipt()
            raise ValueError('current same-foot finite path rejected')
        samples = []; analytic_checks = []
        for fraction in ANALYTIC_FRACTIONS:
            elapsed = context.elapsed+fraction*plan.duration
            sample = plan.sample(elapsed)
            check = validate_settle_sample(plan, sample, support)
            analytic_checks.append(dict(elapsedS=elapsed, **check))
            if fraction in STATIC_FRACTIONS:
                samples.append(('settle:'+str(fraction), sample))
        first = plan.sample(context.elapsed, runtime_arms=context.arms)
        validate_settle_sample(plan, first, support)
        samples.insert(0, ('actual-arm-conditioned-settle-activation', first))
        static = finite_static_admission(self.w, samples, actual_contacts(self, support, both_feet=True))
        if static.get('accepted') is not True:
            self._wholebody_settle_receipt['finiteStatic'] = static
            self._save_wholebody_receipt()
            raise ValueError('current same-foot finite static admission failed')
        if native_bytes(self.w) != before:
            raise ValueError('same-foot endpoint/path admission mutated actual native payload')
        payload = self._wholebody_evidence_path.with_name('wholebody-settle-activation.native-payload.zlib')
        payload.write_bytes(zlib.compress(before, 1))
        self._wholebody_settle_receipt.update(accepted=True, pathSHA256=plan.key,
            finiteAdmission=admission, completeAnalyticChecks=analytic_checks,
            finiteStatic=static, sourceBindings=self._native_source_bindings(),
            retainedPayload=payload.name, payloadSHA256=identity['actualStateSHA256'],
            compressedPayloadSHA256=sha(payload), actualNativeBytesUnchanged=True,
            outgoingContextSHA256=outgoing.receipt()['contextSHA256'],
            armMeasurementReused=True, staleContextRelabeled=False,
            measuredFinalCOMUnchanged=True, originalLoadedSettleAndQuietGatesUnchanged=True)
        self._wholebody_settle = plan
        self._wholebody_settle_seek_jet = seek
        self._wholebody_sample_cache = None
        self._save_wholebody_receipt()

    def transform_reviewed_reference_context(self, context):
        if not self._wholebody_enabled:
            return context
        if self._wholebody_cycle is None:
            if (context.phase != 'transfer' or context.elapsed < self.phase_duration('transfer')-1e-10
                    or not self.robust_support.get('selected') or self._robust_clock != (1., 0., 0.)):
                return context
            self._activate_wholebody(context)
        support = current_support(self, context, triangle=context.phase not in ('settle', 'stand'))
        sample = self._sample_current(context.elapsed, context.phase, prepared_arms=context.arms)
        if context.phase == 'settle' and self._wholebody_settle is None:
            # Assemble outgoing analytic tasks at THIS current command identity,
            # using its already-prepared arms/seek. No old context is relabeled.
            outgoing = owned_context(context, sample, dict(outgoingBoundary=True), pelvis_weight=4.)
            self._activate_settle(context, outgoing, support)
            sample = self._sample_current(context.elapsed, context.phase, prepared_arms=context.arms)
        if context.phase in ('settle', 'stand'):
            if self._wholebody_settle is None or not self._wholebody_settle_receipt['accepted']:
                raise ValueError('missing endpoint/path/current settle admission')
            check = validate_settle_sample(self._wholebody_settle, sample, support)
        else:
            if context.phase == 'translate':
                AdvanceWindow(tuple(self._wholebody_advance_samples), self.w.mass*9.81,
                    context.source_step).validate()
            check = self._wholebody_cycle.validate_sample(sample, support,
                require_clearance=context.phase == 'translate')
            if check.get('accepted') is not True:
                raise ValueError('current whole-body command admission failed')
        if context.phase == 'transfer':
            pelvis_weight = 0.
        elif context.phase == 'lift':
            pelvis_weight = 4.*float(clock(context.elapsed, self.phase_duration('lift'))[0])
        else:
            pelvis_weight = 4.
        result = owned_context(context, sample, check, pelvis_weight=pelvis_weight)
        config = json.loads(result.config_json)
        config['wholeBodyCycle'] = dict(profile=PROFILE, pathSHA256=self._wholebody_cycle.key,
            settlePathSHA256=None if self._wholebody_settle is None else self._wholebody_settle.key,
            pelvisWeight=pelvis_weight, originalCorrectionEmbeddedOnce=True,
            provisionalSurvivors=list(self._wholebody_survivors), originalAdmissionInherited=False,
            finalPlanQualificationClaim=False, oldProfileQualificationClaim=False,
            visualQualificationClaim=False, crossStepOwnershipReviewed=False)
        return replace(result, config_json=canonical(config))

    def _observe_support(self, t, phase):
        if not self._wholebody_enabled or self._wholebody_cycle is None:
            return super()._observe_support(t, phase)
        if phase != 'transfer' or self._wholebody_admitted:
            return
        now = self.w.l.lh_time(self.w.h)
        source_step = self.w.l.lh_dim(self.w.h, 6)
        corners, present, clearance = actual_stance_patch(self.w, self.stance_side)
        body = self.w.feet()[self.stance_side]['body']
        loaded = bool(self.w.arr(20, (17,))[body] >= .8*self.w.mass*9.81)
        history = self._wholebody_history
        if not loaded:
            history.clear(); self._wholebody_payloads.clear(); return
        risk, forecast = self._forecast_unload_risk(t, corners, clearance)
        self.robust_support['currentWholeBodyUnloadForecast'] = forecast
        payload = native_bytes(self.w)
        witness = dict(sourceStep=source_step, sourceTimeS=now,
            payloadSHA256=hashlib.sha256(payload).hexdigest(), payloadBytes=len(payload),
            compressed=zlib.compress(payload, 1))
        if history and source_step == history[-1][0]:
            raise ValueError('duplicate whole-body risk command')
        if history and (source_step-history[-1][0] != 4 or abs(now-history[-1][1]-.005) > 1e-9):
            history.clear(); self._wholebody_payloads.clear()
        history.append((source_step, now, risk, tuple(bool(x) for x in present), loaded))
        self._wholebody_payloads.append(witness)
        while len(history) > 1 and history[1][1] < now-.1-1e-10:
            history.popleft()
        if len(history) < 21 or now-history[0][1] < .1-1e-9:
            return
        votes = np.bincount([h[2] for h in history if h[2] >= 0], minlength=4)
        weak = int(np.argmax(votes)); survivors = tuple(i for i in range(4) if i != weak)
        if survivors != self._wholebody_survivors or votes[weak]/len(history) < .8:
            return
        if not all(h[4] and all(h[3][i] for i in survivors) for h in history):
            return
        witnesses = [w for w in self._wholebody_payloads if history[0][0] <= w['sourceStep'] <= source_step]
        if (len(history) != 21 or len(witnesses) != 21 or
                [w['sourceStep'] for w in witnesses] != [h[0] for h in history] or
                len({w['sourceStep'] for w in witnesses}) != 21):
            raise ValueError('fresh whole-body risk admission requires21 exact native command witnesses')
        folder = self._wholebody_evidence_path.parent/'wholebody-risk-step-01'
        folder.mkdir()
        saved = []
        for witness in witnesses:
            path = folder/f"source-{witness['sourceStep']:06d}.native-payload.zlib"
            path.write_bytes(witness['compressed'])
            saved.append({k:v for k,v in witness.items() if k != 'compressed'} | dict(
                filename=path.name, compressedSHA256=sha(path)))
        admission = dict(schema=1, profile=PROFILE, pathSHA256=self._wholebody_cycle.key,
            historySourceStartS=history[0][1], historySourceEndS=now, historySamples=21,
            historyPredictedRiskCorners=[h[2] for h in history],
            riskAgreementFrequency=float(votes[weak]/len(history)), survivingCorners=list(survivors),
            allSurvivorsActuallySupportedThroughoutHistory=True, samples=saved,
            sourceBindings=self._native_source_bindings(), originalAdmissionInherited=False,
            originalCorrectionCompleteBeforeActivation=True, physicsIntegrations=0,
            selectedAtTransferS=t, payloadEncoding='zlib-compressed exact lh_checkpoint_export payload')
        (folder/'manifest.json').write_text(json.dumps(admission, indent=2)+'\n')
        self.robust_support['wholeBodyFreshAdmission'] = admission
        self._wholebody_admitted = True
        self._save_wholebody_receipt()

    def robust_support_ready(self, row):
        if not self._wholebody_enabled:
            return super().robust_support_ready(row)
        return bool(self._wholebody_cycle is not None and self._wholebody_admitted and
            super().robust_support_ready(row))

    def solve(self, t, phase='transfer'):
        u, info = super().solve(t, phase)
        if self._wholebody_enabled:
            info['reviewedWalk']['featureProfile'] = PROFILE
            info['wholeBodyCycle'] = dict(enabled=self._wholebody_cycle is not None,
                pathSHA256=None if self._wholebody_cycle is None else self._wholebody_cycle.key,
                freshReferenceAdmission=self._wholebody_admitted,
                sameFootSettlePathSHA256=None if self._wholebody_settle is None else self._wholebody_settle.key,
                originalAdmissionInherited=False, finalPlanQualificationClaim=False,
                oldProfileQualificationClaim=False, visualQualificationClaim=False)
        return u, info


class WholeBodySteps(ReviewedSteps):
    reviewed_controller_class = WholeBodyController

    def __init__(self, base, helper, output, count=1, *, proposal_json):
        if count != 1:
            raise ValueError('whole-body pilot count1 only; C2 cross-step ownership requires explicit review')
        self._wholebody_proposal_json = proposal_json
        super().__init__(base, helper, output, 1, 'continuous-com-causal')
        self._wholebody_source_paths = {name:source_path(name) for name in RUNTIME}
        self._wholebody_code_start = {name:sha(path) for name, path in self._wholebody_source_paths.items()}

    def begin_step(self):
        super().begin_step()
        self.controller._wholebody_enabled = True
        self.controller._wholebody_proposal_json = self._wholebody_proposal_json
        self.controller._wholebody_evidence_path = self.folder/'wholebody-current-admission.json'

    def decorate(self, row):
        result = super().decorate(row)
        controller = getattr(self, 'controller', None)
        if controller is not None and getattr(controller, '_wholebody_enabled', False):
            if row.get('phase') in ('lift', 'translate'):
                swing = row['feet'][controller.swing_side]
                stance = row['feet'][controller.stance_side]
                controller._wholebody_advance_samples.append(MeasuredAdvanceSample(
                    row['sourceStep'], swing['clearance'], swing['loadN'], stance['loadN']))
        return result

    def save(self, reason):
        super().save(reason)
        if not hasattr(self, 'folder'):
            return
        source_paths = getattr(self, '_wholebody_source_paths', {})
        end = {name:sha(path) for name, path in source_paths.items()}
        start = getattr(self, '_wholebody_code_start', {})
        for filename in ('reviewed-walk-run.json', 'reviewed-walk-config.json'):
            path = self.folder/filename
            if not path.exists():
                continue
            value = json.loads(path.read_text())
            value.update(profile=PROFILE, wholeBodyCycleEnabled=True,
                requestedSteps=1, crossStepOwnershipReviewed=False,
                finalPlanQualificationClaim=False, oldProfileQualificationClaim=False,
                originalModelQualificationClaim=False, visualQualificationClaim=False,
                inheritedPhysicalGatesUnchanged=True, nativePhysicsStepS=.00125,
                originalQuietConsecutiveSamples=4000, originalActualJointReadinessRad=.005,
                flightOptimizerCallsAtRuntime=0, maximumSameFootEndpointCalls=1,
                proposal=json.loads(self._wholebody_proposal_json),
                wholeBodyCodeSHA256AtStart=start, wholeBodyCodeSHA256AtEnd=end,
                wholeBodyCodeUnchangedDuringRun=bool(start and start == end))
            path.write_text(json.dumps(value, indent=2)+'\n')
        path = self.folder/'cadenced-step-config.json'
        if path.exists():
            value = json.loads(path.read_text())
            value['reviewedReferenceProfile']['featureProfile'] = PROFILE
            value['reviewedReferenceProfile']['oldProfileQualificationClaim'] = False
            value['reviewedReferenceProfile']['finalPlanQualificationClaim'] = False
            path.write_text(json.dumps(value, indent=2)+'\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inspector', type=Path, required=True)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--enable-wholebody', action='store_true')
    parser.add_argument('--proposal', type=Path)
    parser.add_argument('--proposal-binding', type=Path)
    args = parser.parse_args(argv)
    proposal = None
    if args.enable_wholebody:
        if args.count != 1:
            parser.error('whole-body count1 only; counts2/10 await explicit C2 cross-step ownership review')
        if args.proposal is None:
            parser.error('--enable-wholebody requires one admitted --proposal')
        # All failures here precede prepare_impedance and all native construction.
        proposal = frozen_proposal(args.proposal, args.proposal_binding)
    elif args.proposal is not None or args.proposal_binding is not None:
        parser.error('proposal supplied without explicit --enable-wholebody')
    elif args.count not in (1, 2, 7, 10):
        parser.error('unsupported baseline count')
    session, helper, preparation = prepare_impedance(args.output, args.inspector, 8.)
    retained = args.output/'contact-inspector.so'
    shutil.copyfile(args.inspector, retained)
    if sha(retained) != preparation['inspectorSHA256']:
        raise ValueError('retained original read-only inspector differs')
    live = (WholeBodySteps(session, helper, args.output/'run', 1, proposal_json=proposal)
        if args.enable_wholebody else
        ReviewedSteps(session, helper, args.output/'run', args.count, 'continuous-com-causal'))
    live.paused = False; started = time.monotonic()
    try:
        while not live.finished and not live.failure:
            if time.monotonic()-started >= PROCESS_CAP_S:
                raise RuntimeError('1200s process cap')
            live.advance(4)
            if live.steps % 800 == 0:
                print('progress', live.steps*.00125, 'step', live.step_index+1,
                    live.phase, 'slip', live.slip, 'quiet', live.quiet_run*.00125, flush=True)
        print(live.failure or 'wholebody-one-step-physical-pilot-plus5squiet', flush=True)
    except Exception as error:
        live.failure = repr(error)
        raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__ == '__main__':
    main()
