#!/usr/bin/env python3
"""Independent saved-data profile for the five-reviewed, opt-in walk.

Legacy audits and flags remain unchanged. Two/seven-step runs are diagnostics,
one-step runs are pilots, and ten physical steps alone do not establish the
plan's required visual/cadence improvement. Native witness replay imports an
isolated exact checkpoint and evaluates QPs, but NEVER integrates a world.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace
from fractions import Fraction

import numpy as np
from scipy.spatial.transform import Rotation

from audit_source_cadenced_steps import audit as audit_cadence, json_array_items, DT
from audit_source_contact_impedance import (ORIGINAL_IDENTITY, REQUIRED_STRICT_CHECKS,
    audit_preparation, audit_trial_controls, cache_proof, file_sha)
from audit_source_robust_support import (audit_replay, ExactForecastReplay, FootEndpointFK,
    FORECAST_RISK, UNLOAD_CHECK, close)

PROFILE = 'five-reviewed-walk'
VERSION = 'reviewed-reference-v1'
COUNTS = (1, 2, 7, 10)
MODES = ('baseline-context', 'continuous-com', 'continuous-com-projected', 'continuous-com-load', 'continuous-com-causal')
REQUIRED_SOURCES = frozenset(('source_reviewed_walk.py', 'source_reference_context.py',
    'source_step_probe.py', 'source_multistep_probe.py', 'source_cadenced_ten_steps.py',
    'source_selected_step.py', 'source_opposed_arm_swing.py', 'source_upright_large_steps.py',
    'source_robust_support_steps.py', 'source_contact_impedance_probe.py',
    'source_arm_knee_mobility.py', 'source_implicit_arm_coordination.py', 'source_continuous_com.py'))
REQUIRED_ROBUST_REPLAY_CHECKS = frozenset('''explicitProspectiveRiskPolicy
explicitActualSupportPatchConstruction robustAdmissionPolicy originalFootRadiiPreserved
phaseClockReconstructedFromNativeCounters allActualControlNotesAligned
robustTelemetryOnEveryActualControl independentFootEndpointFK
finiteForecastDimensions forecastUsesCurrentActualMatchedPoints
forecastUsesCurrentPhysicalEndpointClearances forecastArithmeticAndRiskIndexes
forecastIsPureBoundedUnappliedProposal forecastBoundToCurrentNativeSample
allRetainedNativeForecastWitnessesReplay selectionAtFirstCausalEligibleSolve
selectedWindowNativeWitnessCoverage selectionNeverDisappearsOrReplacesHistory
selectionMatchesPrecedingActualHistory selectionScopeAndPhysicalBounds
selectionDeclaresActualPointConstruction boundedC2Correction analyticC2CorrectionClock
barriersNeverUseMissingOrUnselectedSupport triangleBarrierMarginsAndC2ReserveReplay
unchangedEveryNativeActualHullCaptureGuard eachLiftHas80ActualTriangleCorrectedUnloadSamples
actualControlScheduleComplete allRequestedStepsHaveMeasuredCausalSelection
allRequestedLiftsHaveMeasuredActualTriangleAdmission nonemptyReplay'''.split())
CONTEXT_FIELDS = frozenset(('schema', 'version', 'source_step', 'source_time', 'step_index',
    'phase', 'elapsed', 'epoch', 'com', 'joints', 'arms', 'arm_gains', 'orientations',
    'feet', 'capture_constraints', 'unload_admitted', 'causal', 'config', 'diagnostics',
    'hypothetical_changes', 'contextSHA256'))
STATIC_FIELDS = ('com0', 'com1', 'targets', 'rootR', 'flatRotations', 'radii', 'swing_side',
    'stance_side', 'swing_body', 'self_avoidance', 'B', 'heading', 'landing', 'step_distance',
    'lean_axis', 'phase_durations', 'joint_reserve', 'posture_start', 'shear_weight',
    'trajectory_time_scale', 'arm_kp', 'arm_kd', 'arm_weight', 'leg_origin', 'arm_start',
    'placement', 'arm_placement_duration', 'loaded_settle_capture_barrier', 'knee_goal')
OLD_FLAGS = ('strictCadencedSequencePass', 'tenStep180mmQualificationPass',
    'explicitRobustSupportImpratio8PilotPass', 'explicitRobustSupportImpratio8TenStepPass',
    'explicitProspectiveUnloadRiskImpratio8PilotPass', 'explicitProspectiveUnloadRiskImpratio8TenStepPass')
MOTOR_JOINT_ORDER = (1, 0, 2, 3, 4, 5, 6, 8, 7, 9, 10, 11, 12, 14, 13, 15, 16, 17, 18, 19, 20)


def causal_selection_policy_audit():
    """Independent literal policy, never read back from the runtime producer."""
    return dict(schema=1, policy='actual-loaded-history-no-nominal-wait-v1',
        stateOnlySupportSelection=True,
        nominalRiskHistoryTimerRequired=False, nominalSelectionTimerRequired=False,
        minimumLoadedHistoryS=.1, minimumHistorySamples=21,
        minimumActualStanceWeightFraction=.8, minimumRiskAgreementFrequency=.8,
        actualSurvivorsRequired=True, minimumCorrectionDurationS=1.,
        maximumCorrectionM=.012, maximumCorrectionSpeedMps=.025,
        maximumCorrectionAccelerationMps2=.15,
        maximumTotalSpeedMps=.19260552331163003,
        maximumTotalAccelerationMps2=.3901818436690813,
        originalMinimumTransferAndMeasuredLiftGatesUnchanged=True,
        transferReferenceValidation='every command after state-qualified selection; unchanged legacy transfer before selection')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def value_sha(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def command_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value, dtype='<f8').tobytes()).hexdigest()


def is_sha(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def equal(a, b, tolerance=1e-10):
    if isinstance(a, np.ndarray): a = a.tolist()
    if isinstance(b, np.ndarray): b = b.tolist()
    if isinstance(a, dict) or isinstance(b, dict):
        return isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys() and all(
            equal(a[k], b[k], tolerance) for k in a)
    if isinstance(a, (tuple, list)) or isinstance(b, (tuple, list)):
        return isinstance(a, (tuple, list)) and isinstance(b, (tuple, list)) and len(a) == len(b) and all(
            equal(x, y, tolerance) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None or isinstance(a, str) or isinstance(b, str):
        return type(a) is type(b) and a == b
    try:
        return math.isfinite(float(a)) and math.isfinite(float(b)) and abs(float(a)-float(b)) <= tolerance
    except (TypeError, ValueError):
        return False


def array(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError('invalid finite array shape '+str(shape))
    return result


def clock(t, duration):
    """Analytic audit implementation; intentionally does not call the producer."""
    if not math.isfinite(t) or not math.isfinite(duration) or duration <= 0.:
        raise ValueError('invalid reference clock')
    x = min(1., max(0., t/duration))
    return np.array((10*x**3-15*x**4+6*x**5,
        (30*x**2-60*x**3+30*x**4)/duration,
        (60*x-180*x**2+120*x**3)/duration**2))


def jet(value, width):
    if not isinstance(value, dict) or set(value) != {'position', 'velocity', 'acceleration'}:
        raise ValueError('invalid reference jet keys')
    return tuple(array(value[k], (width,)) for k in ('position', 'velocity', 'acceleration'))


def jet_matches(value, expected, width):
    return all(close(a, b, 1e-10) for a, b in zip(jet(value, width), expected))


def segment_jet(segment, now):
    if not isinstance(segment, dict) or set(segment) != {'startSourceTimeS', 'durationS', 'coefficients'}:
        raise ValueError('invalid continuous COM segment')
    start, duration = float(segment['startSourceTimeS']), float(segment['durationS'])
    if not math.isfinite(start) or not math.isfinite(duration) or duration <= 0.:
        raise ValueError('invalid continuous COM clock')
    c = array(segment['coefficients'], (6, 3)); x = min(1., max(0., (now-start)/duration))
    return (sum(c[i]*x**i for i in range(6)),
        sum(i*c[i]*x**(i-1) for i in range(1, 6))/duration,
        sum(i*(i-1)*c[i]*x**(i-2) for i in range(2, 6))/duration**2)


def hermite_coefficients(initial, final, duration):
    p, v, a = [np.asarray(x) for x in initial]; q, w, b = [np.asarray(x) for x in final]
    c0, c1, c2 = p, duration*v, .5*duration**2*a
    r0, r1, r2 = q-c0-c1-c2, duration*w-c1-2*c2, duration**2*b-2*c2
    return np.array((c0, c1, c2, 10*r0-4*r1+.5*r2, -15*r0+7*r1-r2, 6*r0-3*r1+.5*r2))


def independent_references(config, causal, elapsed, phase, hypothetical=False, reference_policy=None):
    """Rebuild all planned task jets without importing the reference producer."""
    if reference_policy is not None:
        return reference_policy.references(config, causal, elapsed, phase, hypothetical)
    c = config
    com0, com1, final = [array(c[k], (3,)) for k in ('com0', 'com1', 'finalCom')]
    durations = c['phase_durations']
    z3 = np.zeros(3)
    s, ds, dds = clock(elapsed, durations['transfer'])
    com = (com0+(com1-com0)*s, (com1-com0)*ds, (com1-com0)*dds) if phase == 'transfer' else (com1.copy(), z3.copy(), z3.copy())
    if phase in ('settle', 'stand'):
        s, ds, dds = clock(elapsed, durations['settle']) if phase == 'settle' else (1., 0., 0.)
        com = (com1+(final-com1)*s, (final-com1)*ds, (final-com1)*dds)
    correction = causal['robustCorrection']
    correction_clock = np.zeros(3)
    if correction['selected']:
        if phase == 'transfer':
            correction_clock = clock(elapsed-float(correction['startTransferS']), float(correction['durationS']))
        elif phase == 'settle':
            a, b, d = clock(elapsed, durations['settle']); correction_clock = np.array((1.-a, -b, -d))
        elif phase != 'stand':
            correction_clock = np.array((1., 0., 0.))
        delta = array(correction['deltaM'], (3,))
        com = tuple(p+k*delta for p, k in zip(com, correction_clock))
    if c.get('mode') in ('continuous-com', 'continuous-com-projected', 'continuous-com-load', 'continuous-com-causal'):
        continuous = causal['continuousCom']; now = float(causal['arm']['nativeSourceTimeS'])
        if phase in ('translate', 'land'):
            path = segment_jet(continuous['path'], now)
            com = tuple(a+b for a, b in zip(com, path))
        elif phase == 'settle' and continuous.get('path') is not None:
            com = segment_jet(continuous['settle'], now)
    q0 = array(c['q0'], (21,)); z21 = np.zeros(21)
    if phase == 'transfer' and 'posture_start' in c:
        s, ds, dds = clock(elapsed, durations['transfer'])
        start = array(c['posture_start'], (21,)); delta = q0-start
        joints = (start+delta*s, delta*ds, delta*dds)
    else:
        joints = (q0.copy(), z21.copy(), z21.copy())
    R = array(c['rootR'], (3, 3)); rv, ra = z3.copy(), z3.copy()
    if c['self_avoidance']:
        s, ds, dds = clock(elapsed, durations['transfer']) if phase == 'transfer' else (1., 0., 0.)
        if phase == 'settle':
            a, b, d = clock(elapsed, durations['settle']); s, ds, dds = 1.-a, -b, -d
        elif phase == 'stand':
            s, ds, dds = 0., 0., 0.
        axis = array(c['lean_axis'], (3,)); R = Rotation.from_rotvec(axis*s).as_matrix()@R
        rv, ra = axis*ds, axis*dds
    orientation = dict(body=1, rotation=R.reshape(-1).tolist(), velocity=rv.tolist(), acceleration=ra.tolist(), kp=40., kd=12., weight=4.)
    floor = array(c['floorTargets'], (2,)).copy()
    if hypothetical:
        # All retained worlds have the unchanged flat ground. The original
        # saved target has ground+radius-0.5mm, hence exactly +0.6mm here.
        side = c['swing_side']; floor[side] = c['targets'][side][0][2]
        # The absolute floor is supplied by the independently bound baseline
        # floor, rather than by a possibly already admitted current floor.
        baseline_floor = causal.get('initialFloorTargets', c.get('initialFloorTargets'))
        if baseline_floor is None:
            baseline_floor = c['floorTargets']
            if causal.get('actualUnloadAdmitted', False):
                baseline_floor = list(baseline_floor); baseline_floor[side] -= .0006
        floor[side] = float(baseline_floor[side])+.0006
    seek = array(causal['contactSeekAfter']['jet'], (3,))
    feet = []
    for side in (0, 1):
        pos0 = array(c['targets'][side][0], (3,)); initial = array(c['targets'][side][1], (3, 3))
        flat = array(c['flatRotations'][side], (3, 3)); axis = Rotation.from_matrix(flat@initial.T).as_rotvec()
        pt, pv, pa = pos0.copy(), z3.copy(), z3.copy(); R, rv, ra = flat.copy(), z3.copy(), z3.copy()
        if phase == 'transfer':
            s, ds, dds = clock(elapsed, durations['transfer']); R = Rotation.from_rotvec(axis*s).as_matrix()@initial
            rv, ra = axis*ds, axis*dds; dz = floor[side]-pos0[2]
            pt[2] += dz*s; pv[2], pa[2] = dz*ds, dz*dds
        else:
            pt[2] = floor[side]
        if side == c['swing_side'] and phase != 'transfer':
            s, ds, dds = clock(elapsed, durations[phase]) if phase in ('lift', 'translate', 'land') else (1., 0., 0.)
            if phase == 'lift':
                pt[2] += .04*s; pv[2], pa[2] = .04*ds, .04*dds
            elif phase == 'translate':
                heading = array(c['heading'], (3,)); pt[2] += .04; pt += c['step_distance']*heading*s
                pv, pa = c['step_distance']*heading*ds, c['step_distance']*heading*dds
            elif phase == 'land':
                pt = array(c['landing'], (3,)).copy(); pt[2] += .04*(1.-s)-.0005*s
                pv[2], pa[2] = -.0405*ds, -.0405*dds
            else:
                pt = array(c['landing'], (3,)).copy(); pt[2] -= .0005
            pt[2] -= seek[0]; pv[2] -= seek[1]; pa[2] -= seek[2]
        feet.append((pt, pv, pa, R, rv, ra))
    return dict(com=com, joints=joints, orientations=[orientation], feet=feet, correctionClock=correction_clock)


_UNBOUND_ANCHOR = object()
SOFT_LOAD_POLICY = 'retained-acquired-load-hinge-v1'
SOFT_LOAD_ANCHOR_FIELDS = frozenset(('source_step', 'source_time', 'step_index', 'new_side',
    'measured_normal_n', 'body_weight_n', 'qualified_landing_samples'))


def independent_soft_load_border(P, linear, A, b, normal_indices, floor):
    """One-slack border constructed without the producer helper."""
    n = len(linear); rows = len(b)
    P = array(P, (n, n)); linear = array(linear, (n,)); A = array(A, (rows, n)); b = array(b, (rows,))
    indices = tuple(normal_indices)
    if (not indices or len(set(indices)) != len(indices)
            or any(type(index) is not int or not 0 <= index < n for index in indices)
            or type(floor) not in (int, float) or not math.isfinite(floor) or floor < 0.): raise ValueError('invalid soft-load border')
    cost = np.pad(P, ((0, 1), (0, 1))); cost[-1, -1] = .0005**2
    q = np.r_[linear, 0.]; constraints = np.pad(A, ((0, 2), (0, 1)))
    constraints[-2, -1] = -1.; constraints[-1, -1] = -1.; constraints[-1, list(indices)] = -1.
    return cost, q, constraints, np.r_[b, 0., -floor]


def audit_soft_load_context(receipt, note, expected_anchor=_UNBOUND_ANCHOR):
    """Bind the optional C2 load preference to an independently measured handoff."""
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    try:
        c, causal, phase = receipt['config'], receipt['causal'], receipt['phase']
        anchor, preference, hinge = causal.get('softLoadAcquisition'), c.get('softLoadPreference'), note.get('softLoadHinge')
        if c.get('mode') != 'continuous-com-load':
            check('noSoftLoadPreferenceInOtherModes', anchor is None and preference is None and hinge is None)
        else:
            check('softLoadExplicitPreparedFields', 'softLoadAcquisition' in causal and 'softLoadPreference' in c)
            before = phase in ('transfer', 'lift', 'translate', 'land')
            check('softLoadAnchorOnlyAfterQualifiedLanding', (before and anchor is None)
                or (not before and isinstance(anchor, dict) and set(anchor) == SOFT_LOAD_ANCHOR_FIELDS))
            if before:
                check('noSoftLoadBeforeAcquisition', preference is None and hinge is None)
            else:
                check('softLoadAnchorIndependentlyMeasuredAndNeverReplaced', expected_anchor is not _UNBOUND_ANCHOR
                    and expected_anchor is not None and equal(anchor, expected_anchor))
                if not isinstance(anchor, dict): raise ValueError('missing acquired load')
                check('softLoadAnchorStrictIdentityAndQualification', set(anchor) == SOFT_LOAD_ANCHOR_FIELDS
                    and all(type(anchor[key]) is int and anchor[key] >= 0 for key in
                        ('source_step', 'step_index', 'new_side', 'qualified_landing_samples'))
                    and all(type(anchor[key]) in (int, float) and math.isfinite(anchor[key]) for key in
                        ('source_time', 'measured_normal_n', 'body_weight_n')) and anchor['source_time'] >= 0.
                    and anchor['new_side'] == c['swing_side'] and anchor['step_index'] == receipt['step_index']
                    and anchor['qualified_landing_samples'] >= 80 and float(anchor['measured_normal_n']) >= 3.
                    and math.isfinite(float(anchor['body_weight_n'])) and float(anchor['body_weight_n']) > 0.)
                if phase == 'settle':
                    elapsed, duration = float(receipt['elapsed']), float(c['phase_durations']['settle'])
                    check('softLoadMeasuredHandoffClock', elapsed >= 0.
                        and close(DT*(receipt['source_step']-anchor['source_step']), elapsed, 1e-9)
                        and close(receipt['source_time']-anchor['source_time'], elapsed, 1e-9))
                    a, da, dda = clock(elapsed, .1); fade, reversed_rate, ddfade = clock(duration-elapsed, duration)
                    dfade = -reversed_rate; acquired = min(anchor['measured_normal_n'], .1*anchor['body_weight_n'])
                    floor = [acquired*a*fade, acquired*(da*fade+a*dfade), acquired*(dda*fade+2*da*dfade+a*ddfade)]
                    if elapsed == 0. or elapsed >= duration: floor = [0., 0., 0.]
                    check('softLoadIndependentC2FloorAndWeight', isinstance(preference, dict)
                        and set(preference) == {'schema', 'policy', 'anchor', 'floor_jet', 'weight'}
                        and type(preference['schema']) is int and preference['schema'] == 1
                        and preference['policy'] == SOFT_LOAD_POLICY and equal(preference['anchor'], anchor)
                        and type(preference['floor_jet']) is list and len(preference['floor_jet']) == 3
                        and all(type(value) in (int, float) and math.isfinite(value) for value in preference['floor_jet'])
                        and close(preference['floor_jet'], floor, 1e-9) and preference['weight'] == .0005)
                    forces = array(note['predictedForcesN'], (8, 3)); side = anchor['new_side']
                    predicted = float(np.sum(forces[4*side:4*side+4, 2]))
                    check('softLoadOneSlackMetadataAndPredictedForce', isinstance(hinge, dict)
                        and set(hinge) == {'policy', 'baseQPWidth', 'augmentedQPWidth', 'slackN', 'targetNormalN',
                            'predictedNewFootNormalN', 'weightPerNewton', 'hardForceFloor'}
                        and hinge['policy'] == SOFT_LOAD_POLICY and type(hinge['baseQPWidth']) is int and hinge['baseQPWidth'] == 72
                        and type(hinge['augmentedQPWidth']) is int and hinge['augmentedQPWidth'] == 73
                        and all(type(hinge[key]) in (int, float) and math.isfinite(hinge[key]) for key in
                            ('slackN', 'targetNormalN', 'predictedNewFootNormalN', 'weightPerNewton'))
                        and math.isfinite(hinge['slackN']) and hinge['slackN'] >= -1e-7
                        and close(hinge['targetNormalN'], floor[0], 1e-9)
                        and close(hinge['predictedNewFootNormalN'], predicted, 1e-9)
                        and hinge['slackN']+predicted >= floor[0]-1e-7
                        and hinge['weightPerNewton'] == .0005 and hinge['hardForceFloor'] is False)
                else:
                    check('noSoftLoadOutsideSettle', preference is None and hinge is None)
    except (KeyError, ValueError, TypeError, IndexError, ArithmeticError):
        checks['completeSoftLoadReferenceEvidence'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks)


def audit_context(receipt, note, row, reference, *, expected_elapsed=None,
                  expected_q0=None, expected_floor=None, expected_final=None,
                  previous_arm=None, expected_mode='baseline-context', hypothetical=False,
                  expected_load_anchor=_UNBOUND_ANCHOR, reference_policy=None):
    """Pure independent context arithmetic and causal identity check.

    `row` is the PRECEDING measured native sample, never the result of applying
    this command. Optional reconstructed states come from the streaming reducer.
    """
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    try:
        check('exactReferenceSchema', isinstance(receipt, dict) and set(receipt) == CONTEXT_FIELDS
            and receipt['schema'] == 2 and receipt['version'] == VERSION)
        d = copy.deepcopy(receipt); claimed = d.pop('contextSHA256')
        check('contextDigest', is_sha(claimed) and value_sha(d) == claimed)
        c, causal = receipt['config'], receipt['causal']; t = float(receipt['elapsed']); phase = note['phase']
        check('explicitReferenceFeatureScope', reference_policy is not None or (c.get('coupledPosture') is None
            and c.get('boundedSettle') is None and note.get('reviewedWalk', {}).get('featureProfile') is None))
        bound_reference = reference_policy.bound_reference(reference, receipt) if reference_policy else reference
        if reference_policy is not None:
            for key, value in reference_policy.context_checks(receipt, note, row, reference).items(): check(key, value)
        for key, value in audit_soft_load_context(receipt, note, expected_load_anchor)['checks'].items(): check(key, value)
        if expected_mode != 'continuous-com-causal':
            check('noStateOnlySelectionPolicyInOtherModes', c.get('causalSupportSelectionPolicy') is None and causal.get('causalOverlap') is None)
        check('contextBoundToPrecedingNativeSample', type(receipt['source_step']) is int
            and receipt['source_step'] == row['sourceStep'] and close(receipt['source_time'], row['sourceTimeS'], 1e-12)
            and close(note['t'], row['sourceStep']*DT, 1e-10) and receipt['step_index'] == note['stepIndex']
            and receipt['phase'] == phase and type(receipt['epoch']) is int and receipt['epoch'] >= 0
            and (expected_elapsed is None or close(t, expected_elapsed, 1e-10)))
        check('explicitReviewedReferenceMode', c.get('mode') == expected_mode and expected_mode in MODES
            and c.get('referenceVersion') == VERSION)
        check('staticReferencesBoundToSavedStep', all(equal(c.get(k), bound_reference[k]) for k in STATIC_FIELDS if k in bound_reference))
        arm = note.get('opposedArmSwing', {})
        check('preparedArmTasksBoundToActualDelayedTargets', all(equal(receipt['arms'][field], arm.get(key))
            for field, key in (('position', 'targetArmJointRad'), ('velocity', 'targetArmVelocityRadps'), ('acceleration', 'targetArmAccelerationRadps2')))
            and equal(receipt['arm_gains'], [100., 20., 2.])
            and all(equal(causal.get(name), receipt['arms'][field]) for name, field in
                (('armPosture', 'position'), ('armVelocity', 'velocity'), ('armAcceleration', 'acceleration'))))
        check('preparedJointStateBound', equal(causal.get('preparedQ0'), c['q0'])
            and equal(c['q0'][15:], receipt['arms']['position'])
            and (expected_q0 is None or equal(c['q0'][:15], np.asarray(expected_q0)[:15].tolist())))
        check('floorAndFinalStateBound', (expected_floor is None or equal(c['floorTargets'], expected_floor))
            and (expected_final is None or equal(c['finalCom'], expected_final)))
        check('persistentArmOriginBound', equal(causal.get('persistentLegOrigin'), reference['leg_origin'])
            and equal(arm.get('legPitchOriginRad'), reference['leg_origin']))
        measure = causal['arm']; rate = array(measure['measuredLegPitchRateRadps'], (2,))
        previous_time = None if previous_arm is None else previous_arm['time']
        previous_rate = None if previous_arm is None else previous_arm['rate']
        dt = None if previous_time is None else float(row['sourceTimeS'])-previous_time
        acceleration = np.zeros(2) if dt is None or dt <= 1e-8 else (rate-previous_rate)/dt
        check('causalDelayedArmHistory', measure.get('measuredOncePerNativeCommand') is True
            and measure.get('nativeSourceStep') == row['sourceStep'] and close(measure.get('nativeSourceTimeS'), row['sourceTimeS'])
            and equal(measure.get('anchor'), dict(previousTimeS=previous_time,
                previousLegRateRadps=None if previous_rate is None else np.asarray(previous_rate).tolist()))
            and equal(measure.get('measuredLegPitchRad'), arm.get('measuredLegPitchRad'))
            and equal(rate.tolist(), arm.get('measuredLegPitchRateRadps'))
            and close(measure.get('measuredLegPitchAccelerationRadps2'), acceleration, 1e-9)
            and equal(causal.get('armHistoryAfter'), dict(previousTimeS=row['sourceTimeS'], previousLegRateRadps=rate.tolist())))
        audit_config = copy.deepcopy(c); audit_config['initialFloorTargets'] = reference['floorTargets']
        expected = independent_references(audit_config, causal, t, phase, hypothetical, reference_policy)
        check('independentCOMJets', jet_matches(receipt['com'], expected['com'], 3))
        check('independentJointAndKneeJets', jet_matches(receipt['joints'], expected['joints'], 21))
        check('independentOrientationJets', equal(receipt['orientations'], expected['orientations']))
        foot_pass = len(receipt['feet']) == 2
        for value, planned in zip(receipt['feet'], expected['feet']):
            orientation = value['orientation']
            foot_pass = foot_pass and jet_matches(value['translation'], planned[:3], 3)
            foot_pass = foot_pass and close(np.asarray(orientation['rotation']).reshape(3, 3), planned[3])
            foot_pass = foot_pass and close(orientation['velocity'], planned[4]) and close(orientation['acceleration'], planned[5])
        check('independentFootJets', foot_pass)
        check('independentCorrectionClock', equal(causal['robustCorrection'].get('clock'), expected['correctionClock'].tolist()))
        changes = receipt['hypothetical_changes']
        check('exactDeclaredHypotheticalScope', changes == (['unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm'] if hypothetical else [])
            and (not hypothetical or receipt['unload_admitted'] is True))
        for orientation in receipt['orientations']+[f['orientation'] for f in receipt['feet']]:
            R = array(orientation['rotation'], (9,)).reshape(3, 3)
            if not (close(R.T@R, np.eye(3), 1e-10) and abs(np.linalg.det(R)-1.) < 1e-10):
                raise ValueError('nonrotation reference')
        jet(receipt['arms'], 6)
        for constraint, bound in receipt['capture_constraints']:
            a = np.asarray(constraint, dtype=float)
            if a.ndim != 1 or len(a) not in (60, 72) or not np.isfinite(a).all() or not math.isfinite(bound):
                raise ValueError('invalid capture constraint')
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        checks['completeFiniteReferenceContext'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v])


def audit_source_provenance(receipt, source_dir, frozen_dir=None):
    source_dir = Path(source_dir); frozen_dir = Path(frozen_dir) if frozen_dir is not None else None
    start, end = receipt.get('codeSHA256AtStart'), receipt.get('codeSHA256AtEnd')
    checks = dict(requiredReviewedAndCoreSourcesPresent=isinstance(start, dict) and REQUIRED_SOURCES <= set(start),
        exactStartEndSourceBindings=isinstance(start, dict) and isinstance(end, dict) and bool(start) and start == end,
        codeUnchangedDuringRun=receipt.get('codeUnchangedDuringRun') is True)
    if receipt.get('wrenchDiagnosticsEnabled') is True:
        checks['requiredSolvedWrenchReaderSourcePresent'] = isinstance(start, dict) and 'source_solved_wrench_diagnostics.py' in start
    if receipt.get('softLoadPreferenceEnabled') is True:
        checks['requiredSoftLoadPreferenceSourcePresent'] = isinstance(start, dict) and 'source_soft_load_preference.py' in start
    if receipt.get('causalSupportSelectionEnabled') is True:
        checks['requiredCausalOverlapCertificateSourcePresent'] = isinstance(start, dict) and 'source_causal_com_overlap.py' in start
    if isinstance(start, dict):
        for name, expected in start.items():
            safe = isinstance(name, str) and name == Path(name).name and name.endswith('.py') and is_sha(expected)
            p = source_dir/name if safe else None
            good = bool(safe and p.is_file() and not p.is_symlink() and file_sha(p) == expected)
            if frozen_dir is not None:
                p = frozen_dir/name if safe else None
                good = bool(good and p.is_file() and not p.is_symlink() and file_sha(p) == expected)
            checks['source:'+str(name)] = good
    return dict(passed=all(checks.values()), checks=checks)


def qualify_reviewed(original, robust, references, model, controls, provenance, native,
                     count, *, mode='baseline-context', improvements=None):
    """No old profile flag is re-used, even when every physical gate passes."""
    old = original.get('measured', {}).get('checks', {})
    required = REQUIRED_STRICT_CHECKS-({'loadedSettleCompletionSummaryMatchesReadiness',
        'loadedSettleHandoffBeginsOnNextSample'} if count == 1 else set())
    allowed = {ORIGINAL_IDENTITY, UNLOAD_CHECK}
    checks = dict(explicitReviewedCountAndMode=type(count) is int and count in COUNTS and mode in MODES,
        originalModelFlagsRemainFalse=original.get('strictCadencedSequencePass') is False
            and original.get('tenStep180mmQualificationPass') is False and old.get(ORIGINAL_IDENTITY) is False,
        originalPhysicalAndArmCoverage=required <= set(old),
        everyUnreplacedPhysicalAndArmGate=all(v is True for k, v in old.items() if k not in allowed),
        exactRequestedDistanceAndCount=original.get('requestedSteps') == count and original.get('requestedDistanceM') == .18,
        independentCausalActualTriangleAdmission=robust.get('passed') is True,
        independentReferenceAndCommandReplay=references.get('passed') is True,
        explicitUnchangedImpratio8Model=model.get('passed') is True and model.get('expectedImpratio') == 8.,
        unchangedContactAndArmControlGates=controls is True,
        startEndSourceProvenance=provenance.get('passed') is True,
        exactNativeReferenceAndQPWitnessReplay=native.get('passed') is True)
    if mode == 'continuous-com-load':
        load_native = native.get('activeSoftLoadWitnessReplay', {})
        checks['activeSoftLoadNativeAssemblyAndCommandReplay'] = (load_native.get('passed') is True
            and load_native.get('expectedWitnesses') == 2*count
            and load_native.get('verifiedWitnesses') == 2*count and load_native.get('physicsIntegrations') == 0)
    if mode == 'continuous-com-causal':
        checks['independentStateOnlyCausalAndOverlapReplay'] = robust.get('causalModeReplayVerified') is True
    physical = all(checks.values())
    # Baseline-context is an unchanged-motion parity profile. Continuous-com
    # still requires independent B evidence; a boolean in a runner is insufficient.
    improved = isinstance(improvements, dict) and improvements.get('independentlyAudited') is True and (
        improvements.get('sameContactCausalComparisonPass') is True
        and improvements.get('deliveredVideoComparisonPass') is True
        and improvements.get('physicallyFasterPass') is True and improvements.get('visuallyImprovedPass') is True
        and improvements.get('allIntervalsAndMatchedPhasesPublished') is True
        and improvements.get('physicalTime1xVideoVerified') is True)
    result = dict(profile=PROFILE, reviewedWalkDiagnosticPass=physical and count in (2, 7),
        reviewedWalkPilotPhysicalPass=physical and count == 1,
        reviewedWalkTenStepPhysicalPass=physical and count == 10,
        reviewedWalkFinalPlanPass=bool(physical and count == 10 and mode != 'baseline-context' and improved),
        diagnosticOnly=count in (2, 7), checks=checks, failedChecks=[k for k, v in checks.items() if not v],
        originalModelQualificationClaim=False, originalStrictCadencedSequencePass=original.get('strictCadencedSequencePass'),
        originalTenStep180mmQualificationPass=original.get('tenStep180mmQualificationPass'),
        replacedOriginalChecks=sorted(allowed), improvements=improvements)
    result.update({k: False for k in OLD_FLAGS})
    return result


def projected_capture_extrema(height, duration):
    """Closed-form global extrema, independent of producer polynomial roots."""
    if not math.isfinite(height) or not math.isfinite(duration) or height <= 0. or duration <= 0.:
        raise ValueError('invalid projected capture clock/height')
    k = math.sqrt(height/9.81)/duration
    critical = .5*(1.+1./(math.hypot(1., 4.*k)+4.*k))
    s = 10.*critical**3-15.*critical**4+6.*critical**5
    maximum = s+k*30.*critical**2*(1.-critical)**2
    return critical, maximum


def independent_endpoint_projection(desired, A, b, radius):
    """Minimize distance on each entire feasible boundary-line interval.

    This does not copy the producer's line-pair/line-circle active-set code.
    An optimum is desired, circle-radial, or on at least one boundary line.
    Clipping that line by every half-plane and the disk covers all its vertices.
    """
    desired = array(desired, (2,)); A = array(A, (len(A), 2)); b = array(b, (len(A),))
    if not math.isfinite(radius) or radius <= 0.: raise ValueError('invalid projection radius')
    normals, bounds = [], []
    for normal, bound in zip(A, b):
        length = float(np.hypot(*normal))
        if length == 0.:
            if bound < 0.: raise ValueError('infeasible zero-normal half-plane')
            continue
        normals.append(normal/length); bounds.append(float(bound/length))
    normals = np.asarray(normals).reshape(-1, 2); bounds = np.asarray(bounds)
    candidates = [desired.copy(), np.zeros(2)]
    norm = float(np.linalg.norm(desired))
    if norm > 0.: candidates.append(radius*desired/norm)
    for normal, bound in zip(normals, bounds):
        remaining = radius**2-bound**2
        if remaining < -1e-14*max(1., radius**2): continue
        centre = bound*normal; tangent = np.array([-normal[1], normal[0]])
        extent = math.sqrt(max(0., remaining)); lower, upper = -extent, extent
        for other, limit in zip(normals, bounds):
            coefficient = float(other@tangent); rhs = float(limit-other@centre)
            low_value, high_value = sorted((coefficient*lower, coefficient*upper))
            if low_value > rhs+1e-12:
                lower, upper = 1., 0.; break
            if high_value <= rhs+1e-12: continue
            if coefficient > 0.: upper = min(upper, rhs/coefficient)
            elif coefficient < 0.: lower = max(lower, rhs/coefficient)
            elif rhs < -1e-12:
                lower, upper = 1., 0.; break
        if lower <= upper+1e-12:
            candidates.append(centre+float(np.clip(tangent@desired, lower, upper))*tangent)
    feasible = [point for point in candidates if np.isfinite(point).all()
        and np.linalg.norm(point) <= radius+1e-12 and np.all(normals@point <= bounds+1e-12)]
    if not feasible: raise ValueError('empty projected endpoint intersection')
    return min(feasible, key=lambda point: float(np.sum((point-desired)**2))).copy()


def audit_projected_plan(plan, config, origin, equations, path):
    """Independently rebuild goal, face reserves, extrema, constraints and optimum."""
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    try:
        c = config; origin = array(origin, (3,)); equations = array(equations, (len(equations), 3))
        heading = array(c['heading'], (3,))[:2]
        if abs(float(np.linalg.norm(heading))-1.) > 1e-10: raise ValueError('nonunit heading')
        basis = np.column_stack((heading, [-heading[1], heading[0]]))
        duration = float(c['phase_durations']['translate']+c['phase_durations']['land'])
        critical, maximum = projected_capture_extrema(float(origin[2]), duration)
        margins = -(equations[:, :2]@origin[:2]+equations[:, 2]); reserves = np.minimum(.004, margins)
        check('projectedOriginAndAdaptiveFaceReserves', min(margins) >= .003-1e-10
            and close(plan.get('planningOriginMarginsM'), margins)
            and close(plan.get('planningFaceReservesM'), reserves)
            and plan.get('planningReserveRule') == 'min(4mm,current-origin-margin) per actual support face; reject origin below3mm')
        goal = .5*(array(c['landing'], (3,))+array(c['targets'][c['stance_side']][0], (3,)))
        goal[:2] += .035*heading; goal[2] = origin[2]
        desired = basis.T@(goal[:2]-origin[:2])
        check('projectedGoalUsesPlannedFeetAndCorrectedOrigin', close(plan.get('plannedGoalSourceM'), goal)
            and close(plan.get('plannedDeltaLocalM'), desired))
        xs = array(plan['wholePolynomialExtremaU'], (len(plan['wholePolynomialExtremaU']),))
        roots = (0., 1., critical)
        check('projectedExactWholePolynomialExtrema', len(xs) >= 3 and close(xs[:2], [0., 1.])
            and all(min(abs(float(x)-root) for root in roots) <= 1e-9 for x in xs)
            and all(min(abs(float(x)-root) for x in xs) <= 1e-9 for root in roots))
        k = math.sqrt(origin[2]/9.81)/duration
        smooth = lambda x: 10.*x**3-15.*x**4+6.*x**5
        factors = [float(smooth(x)) for x in xs]+[float(smooth(x)+k*30.*x**2*(1.-x)**2) for x in xs]
        check('projectedExactWholePolynomialFactors', close(plan.get('wholePolynomialFactors'), factors)
            and abs(max(factors)-maximum) <= 1e-10 and abs(min(factors)) <= 1e-10)
        # Receipt row ordering is checked separately. The optimization uses
        # independently derived exact extrema, never the receipt's factors/A/b.
        def constraints(values):
            A, b = [], []
            for face, margin, reserve in zip(equations, margins, reserves):
                for factor in values:
                    A.append(factor*(face[:2]@basis)); b.append(float(margin-reserve))
            return np.asarray(A+[[0., 1.], [0., -1.]]), np.asarray(b+[.012, .012])
        reported_A, reported_b = constraints(factors)
        check('projectedHalfPlanesIndependentlyRebuilt', close(plan.get('projectionConstraintsA'), reported_A)
            and close(plan.get('projectionConstraintsB'), reported_b))
        A, b = constraints((0., 1., maximum))
        radius = min(.19260552331163003*duration/(15./8.), .3901818436690813*duration**2/(10./math.sqrt(3.)))
        local = independent_endpoint_projection(desired, A, b, radius)
        delta = np.r_[basis@local, 0.]
        check('projectedClosestFeasibleEndpoint', close(plan.get('projectionLocalEndpointM'), local, 1e-10)
            and close(plan.get('forwardM'), local[0]) and close(plan.get('lateralM'), local[1])
            and close(plan.get('maximumDisplacementM'), radius))
        coeff = hermite_coefficients((np.zeros(3),)*3, (delta, np.zeros(3), np.zeros(3)), duration)
        check('projectedZeroJetQuinticAndExactEnvelope', close(path['coefficients'], coeff)
            and np.linalg.norm(delta)*(15./8.)/duration <= .19260552331163003+1e-10
            and np.linalg.norm(delta)*(10./math.sqrt(3.))/duration**2 <= .3901818436690813+1e-10)
        check('projectedPolicyAndFrozenBounds', plan.get('endpointPolicy') == 'closest-feasible-planned-midpoint-v1'
            and plan.get('reserveM') == .003 and plan.get('designReserveCapM') == .004
            and plan.get('frozenMaximumTotalSpeedMps') == .19260552331163003
            and plan.get('frozenMaximumTotalAccelerationMps2') == .3901818436690813
            and plan.get('correctionBoundsUnchanged') is True
            and plan.get('referenceCaptureProxy') == 'constant planned COM height sqrt(z/g)'
            and close(plan.get('phaseStartSourceTimeS'), path['startSourceTimeS']))
    except (KeyError, ValueError, TypeError, IndexError, ArithmeticError):
        checks['completeProjectedCOMPlanReplay'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks)


def audit_legacy_continuous_endpoint(plan, c, start, equations, path):
    """The unchanged original 10mm/lateral-scalar planning audit."""
    heading = array(c['heading'], (3,))[:2]; left = np.array([-heading[1], heading[0]])
    T = float(path['durationS']); tau = math.sqrt(start[2]/9.81)
    f = np.polynomial.Polynomial([0., 0., 0., 10., -15., 6.])
    capture = f+tau/T*f.deriv(); xs = [0., 1.]
    xs.extend(float(x.real) for x in capture.deriv().roots() if abs(x.imag) < 1e-10 and 0. < x.real < 1.)
    factors = [float(p(x)) for p in (f, capture) for x in xs]
    lower, upper = -.012, .012
    for nx, ny, offset in equations:
        normal = np.array([nx, ny]); base = float(normal@start[:2]+offset+.003)
        for factor in factors:
            a = factor*float(normal@left); b = -base-factor*.010*float(normal@heading)
            if abs(a) < 1e-14:
                if b < -1e-12: raise ValueError('infeasible continuous reference')
            elif a > 0.: upper = min(upper, b/a)
            else: lower = max(lower, b/a)
    if lower > upper+1e-12: raise ValueError('empty continuous lateral interval')
    preferred = min((.0025, -.0025), key=lambda x: abs(x-float(np.clip(x, lower, upper))))
    lateral = float(np.clip(preferred, lower, upper)); delta = np.r_[.010*heading+lateral*left, 0.]
    coeff = hermite_coefficients((np.zeros(3),)*3, (delta, np.zeros(3), np.zeros(3)), T)
    checks = dict(continuousPathAndLateralChoiceReconstructed=close(path['coefficients'], coeff)
        and close(plan['lateralM'], lateral) and close(plan['lateralIntervalM'], [lower, upper])
        and plan.get('forwardM') == .010 and plan.get('reserveM') == .003
        and close(plan.get('phaseStartSourceTimeS'), path['startSourceTimeS'])
        and plan.get('frozenMaximumTotalSpeedMps') == .19260552331163003
        and plan.get('frozenMaximumTotalAccelerationMps2') == .3901818436690813
        and plan.get('correctionBoundsUnchanged') is True)
    return dict(passed=all(checks.values()), checks=checks)


def audit_continuous_com(receipt, row, reference, state, elapsed, reference_policy=None):
    """Reconstruct frozen polynomial endpoints, current support and total jets."""
    from scipy.spatial import ConvexHull
    from audit_source_robust_support import support_observation, triangle
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    try:
        phase = receipt['phase']; continuous = receipt['causal']['continuousCom']
        c = receipt['config']; correction = receipt['causal']['robustCorrection']
        path, settle, plan = [continuous.get(k) for k in ('path', 'settle', 'plan')]
        if path is not None:
            check('continuousPathNeverReplaced', state['continuousPath'] is None or equal(state['continuousPath'], path))
            check('continuousPlanNeverReplaced', state.get('continuousPlan') is None or equal(state['continuousPlan'], plan))
            state['continuousPlan'] = copy.deepcopy(plan)
            first = state['continuousPath'] is None
            state['continuousPath'] = copy.deepcopy(path)
            check('continuousPathClockFromMeasuredPhase', close(path['startSourceTimeS'], state['phaseStarts'].get('translate'))
                and close(path['durationS'], c['phase_durations']['translate']+c['phase_durations']['land']))
            start = np.asarray(c['com1'])+np.asarray(correction['deltaM'])
            equations = array(plan['actualSupportEquations'], (len(plan['actualSupportEquations']), 3))
            if first:
                actual = [ConvexHull(np.unique(np.asarray(row['supportPointsXY']), axis=0)).equations]
                # The selected IDs are independently checked by the causal
                # admission reducer and passed through the per-command state.
                ids = state['support'].get('survivingCorners')
                actual.append(triangle(support_observation(row, c['stance_side']), ids, actual=True))
                check('continuousPlanUsesCurrentActualSupport', close(equations, np.vstack(actual))
                    and close(plan['plannedAtSourceTimeS'], row['sourceTimeS']))
            if c.get('mode') == 'continuous-com-projected':
                projected = audit_projected_plan(plan, c, start, equations, path)
                for key, value in projected['checks'].items(): check(key, value)
            else:
                legacy = audit_legacy_continuous_endpoint(plan, c, start, equations, path)
                for key, value in legacy['checks'].items(): check(key, value)
        if phase in ('translate', 'land'): check('continuousPathPresentDuringAdvance', path is not None)
        if phase == 'settle' and reference_policy is not None and c.get('coupledPosture') is not None:
            for key, value in reference_policy.settle_checks(receipt, row, state).items(): check(key, value)
        elif phase == 'settle':
            check('continuousSettlePresent', path is not None and settle is not None)
            if path is not None and settle is not None:
                check('continuousSettleNeverReplaced', state['continuousSettle'] is None or equal(state['continuousSettle'], settle))
                state['continuousSettle'] = copy.deepcopy(settle)
                start_time = state['phaseStarts']['settle']; pp, pv, pa = segment_jet(path, start_time)
                initial = (np.asarray(c['com1'])+np.asarray(correction['deltaM'])+pp, pv, pa)
                final = (np.asarray(c['finalCom']), np.zeros(3), np.zeros(3))
                coeff = hermite_coefficients(initial, final, c['phase_durations']['settle'])
                check('continuousSettleC2EndpointsAndMeasuredTarget', close(settle['startSourceTimeS'], start_time)
                    and close(settle['durationS'], c['phase_durations']['settle']) and close(settle['coefficients'], coeff))
        if phase in ('translate', 'land', 'settle', 'stand') or (phase == 'transfer'
                and c.get('mode') == 'continuous-com-causal' and correction.get('selected')):
            points = np.unique(np.asarray(row['supportPointsXY']), axis=0)
            eq = [ConvexHull(points).equations]
            if phase in ('translate', 'land'):
                eq.append(triangle(support_observation(row, c['stance_side']), state['support']['survivingCorners'], actual=True))
            eq = np.vstack(eq); p, v, a = jet(receipt['com'], 3)
            capture = p[:2]+math.sqrt(p[2]/9.81)*v[:2]
            margin = float(min(np.min(-(eq[:, :2]@p[:2]+eq[:, 2])), np.min(-(eq[:, :2]@capture+eq[:, 2]))))
            speed, acceleration = float(np.linalg.norm(v)), float(np.linalg.norm(a))
            diagnostic = (reference_policy.reference_diagnostic(receipt) if reference_policy is not None
                else continuous['diagnostic'])
            check('continuousTotalJetsInsideCurrentActualSupportInset', margin >= .003-1e-10
                and close(diagnostic.get('referenceSupportMarginM'), margin) and diagnostic.get('actualSupportOnly') is True)
            check('continuousTotalJetEnvelopeNeverWidened', speed <= .19260552331163003+1e-10
                and acceleration <= .3901818436690813+1e-10
                and close(diagnostic.get('totalSpeedMps'), speed) and close(diagnostic.get('totalAccelerationMps2'), acceleration)
                and diagnostic.get('maximumTotalSpeedMps') == .19260552331163003
                and diagnostic.get('maximumTotalAccelerationMps2') == .3901818436690813)
    except (KeyError, ValueError, TypeError, IndexError, ArithmeticError):
        checks['completeContinuousCOMReplay'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks)


WRENCH_MODULE = 'source_solved_wrench_diagnostics.py'
WRENCH_FORCE_STAGE = 'cached mj_step2 solve before integration; pre-step1 geometry'
WRENCH_MOMENT_DEFINITION = 'sum(contact lever cross world force) about solved-stage foot body origin; intrinsic contact couples excluded'
WRENCH_SLIP_STAGE = 'current mj_step1 material-point velocity weighted by just-solved normal forces'
WRENCH_UTILIZATION_MEANING = 'lower bound on maximum individual-contact shear utilization; cancellation can hide saturation'
WRENCH_CACHE_REQUIREMENT = 'live synchronous audited sample; not restored from checkpoint and not superseded by another world'
WRENCH_FIELDS = frozenset('''schema sourceStep sourceTimeS phase stepIndex physicsStepS
nativeTimeStepSVerified solvedWrenchAvailable solvedStageSourceStepBeforeIntegration
solvedStageSourceTimeS transportedGeometrySourceTimeS forceMomentStage momentDefinition
slipStage coordinateFrame frictionCoefficient currentEffectiveFootGroundContactsMu1Verified
loadedThresholdN normalLoadAgreementToleranceN utilizationMeaning
individualContactNonsaturationEstablished cacheRequirement nativeCounterTimeHeldControlUnchanged
heldMotorControlSHA256 bindings feet'''.split())
WRENCH_FOOT_FIELDS = frozenset('''side sideIndex forceWorldSourceN
momentWorldSourceNmAboutSolvedFootOrigin normalForceN rowLoadN normalForceRowErrorN
netShearForceN loadedForUtilization netShearUtilizationLowerBound
utilizationUndefinedReason transportedLoadedSlipSpeedMps transportedLoadedSlipIncrementM'''.split())


def audit_wrench_sample(diagnostic, row, expected_bindings, held_control_sha, previous_slip):
    """Validate recorded live-cache metadata/arithmetic, not a restored cache.

    The pinned helper's last-solve cache is process-global and absent from the
    checkpoint. This reducer never pretends importing a checkpoint recreates it.
    Fx/Fy/moments are source-bound logged measurements; only Fz is cross-checked
    against independent row loads. Aggregate utilization cannot prove that any
    individual contact is below its friction limit.
    """
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    values = None
    try:
        d = diagnostic if isinstance(diagnostic, dict) else {}
        step = row['sourceStep']; now = float(row['sourceTimeS'])
        available = step > 0
        check('wrenchExactReaderSchema', set(d) == WRENCH_FIELDS)
        check('wrenchSampleNativeIdentity', isinstance(diagnostic, dict) and type(step) is int and step >= 0
            and type(row.get('stepIndex')) is int and d.get('schema') == 1
            and type(d.get('sourceStep')) is int and d['sourceStep'] == step
            and close(d.get('sourceTimeS'), now, 1e-10) and d.get('phase') == row['phase']
            and type(d.get('stepIndex')) is int and d['stepIndex'] == row['stepIndex']
            and d.get('physicsStepS') == DT and d.get('nativeTimeStepSVerified') is True and d.get('solvedWrenchAvailable') is available
            and d.get('solvedStageSourceStepBeforeIntegration') == (step-1 if available else None)
            and (close(d.get('solvedStageSourceTimeS'), now-DT, 1e-10) if available else d.get('solvedStageSourceTimeS') is None)
            and close(d.get('transportedGeometrySourceTimeS'), now, 1e-10))
        check('wrenchExactStageAndLowerBoundScope', d.get('forceMomentStage') == WRENCH_FORCE_STAGE
            and d.get('momentDefinition') == WRENCH_MOMENT_DEFINITION
            and d.get('slipStage') == WRENCH_SLIP_STAGE and d.get('coordinateFrame') == 'source world x-forward z-up'
            and d.get('utilizationMeaning') == WRENCH_UTILIZATION_MEANING
            and d.get('individualContactNonsaturationEstablished') is False
            and d.get('cacheRequirement') == WRENCH_CACHE_REQUIREMENT)
        check('wrenchPinnedHelperAndExecutedModelBinding', isinstance(expected_bindings, dict)
            and d.get('bindings') == expected_bindings
            and all(is_sha(expected_bindings.get(k)) for k in ('helperSHA256', 'modelSHA256'))
            and expected_bindings.get('accessor') == 'lh_last_ground_wrench'
            and expected_bindings.get('sideOrder') == ['left', 'right'])
        check('wrenchActualHeldCommandBinding', is_sha(d.get('heldMotorControlSHA256'))
            and d['heldMotorControlSHA256'] == held_control_sha
            and d.get('nativeCounterTimeHeldControlUnchanged') is True)
        check('wrenchFixedFrictionAndLoadPolicy', d.get('frictionCoefficient') == 1.
            and d.get('currentEffectiveFootGroundContactsMu1Verified') is True
            and d.get('loadedThresholdN') == 3. and d.get('normalLoadAgreementToleranceN') == 1e-8)
        feet = d['feet']; check('wrenchExactlyTwoOrderedFeet', isinstance(feet, list) and len(feet) == 2)
        normals, shears, ratios, increments = [], [], [], []
        prior = array(previous_slip, (2,)); current = array(row['loadedSlipDistanceM'], (2,))
        for index, side in enumerate(('left', 'right')):
            foot = feet[index]; measured = row['feet'][index]
            force = array(foot['forceWorldSourceN'], (3,)); moment = array(foot['momentWorldSourceNmAboutSolvedFootOrigin'], (3,))
            load = float(measured['loadN']); speed = float(measured['loadedSlipSpeedMps'])
            if not math.isfinite(load) or not math.isfinite(speed) or load < 0. or speed < 0.: raise ValueError('invalid physical load/slip')
            normal = float(force[2]); shear = float(np.hypot(force[0], force[1])); loaded = bool(available and load >= 3.)
            check('wrenchFootIdentity:'+side, set(foot) == WRENCH_FOOT_FIELDS and foot.get('side') == side and type(foot.get('sideIndex')) is int and foot['sideIndex'] == index)
            check('wrenchNormalForceRowLoadAgreement:'+side, normal >= -1e-8 and abs(normal-load) <= 1e-8
                and close(foot.get('normalForceN'), normal, 1e-10) and close(foot.get('rowLoadN'), load, 1e-10)
                and close(foot.get('normalForceRowErrorN'), normal-load, 1e-10))
            check('wrenchNetShearArithmetic:'+side, close(foot.get('netShearForceN'), shear, 1e-9))
            ratio = shear/normal if loaded else None
            check('wrenchNetUtilizationLowerBoundArithmetic:'+side, foot.get('loadedForUtilization') is loaded
                and (close(foot.get('netShearUtilizationLowerBound'), ratio, 1e-10) if loaded else foot.get('netShearUtilizationLowerBound') is None)
                and foot.get('utilizationUndefinedReason') == (None if loaded else ('no-prior-audited-sample' if not available else 'below-3N-load')))
            increment = speed*DT if available else 0.
            check('wrenchTransportedAndGlobalSlipIncrement:'+side, close(foot.get('transportedLoadedSlipSpeedMps'), speed, 1e-12)
                and close(foot.get('transportedLoadedSlipIncrementM'), increment, 1e-12)
                and close(current[index]-prior[index], increment, 1e-10))
            check('wrenchAbsentSolveIsZero:'+side, available or (np.all(force == 0.) and np.all(moment == 0.)))
            normals.append(normal); shears.append(shear); ratios.append(ratio); increments.append(increment)
        values = dict(normalForceN=normals, netShearForceN=shears,
            netShearUtilizationLowerBound=ratios, loadedSlipIncrementM=increments)
    except (KeyError, ValueError, TypeError, IndexError, ArithmeticError, OverflowError):
        checks['completeFiniteWrenchDiagnostic'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks, values=values)


def audit_reference_journal(rows, entry, references, notes, *, mode='baseline-context',
                            gears=None, limits=None, arm_mapping=None, wrench_enabled=False, wrench_bindings=None, reference_policy=None):
    """Bounded-memory native-clock, history, task and held-command replay."""
    checks = {}
    def check(name, condition): checks[name] = checks.get(name, True) and bool(condition)
    previous = entry; stream = iter(notes); next_note = next(stream, None)
    states = {}; previous_arm = None; count = commands = 0; command = None; overlap_proofs = {}
    max_torque_error = 0.
    max_epoch = {}; epoch_bindings = {}
    wrench_samples = 0; wrench_phase = {}; previous_slip = np.zeros(2)
    for row in rows:
        n = count; count += 1; index, phase = row['stepIndex'], row['phase']
        reference = references[index]
        state = states.setdefault(index, dict(phase=None, phaseStart=n, phaseStarts={}, continuousPath=None, continuousSettle=None, seek=None,
            q0=np.array(reference['q0'], dtype=float), unload=False,
            floor=np.array(reference['floorTargets'], dtype=float), final=None,
            entryCom=array(previous['com'], (3,)).copy(), loadAnchor=None, landRun=0, firstEligibleLandingStep=None))
        if state['phase'] != phase:
            old_phase = state['phase']
            state['phaseStart'] = n; state['phaseStarts'][phase] = n*DT
            if phase in ('settle', 'stand'):
                q = np.asarray(previous['qpos'], dtype=float)[7:]
                state['q0'] = np.clip(q, limits[:, 0]+.002, limits[:, 1]-.002) if limits is not None else q.copy()
            state['phase'] = phase
            if phase == 'settle':
                if mode == 'continuous-com-load':
                    check('softLoadOriginalQualifiedLandingHandoff', old_phase == 'land' and state['loadAnchor'] is None
                        and state['landRun'] >= 80
                        and (previous['sourceStep']-1)*DT-state['phaseStarts']['land'] >= reference['phase_durations']['land']
                        and state['firstEligibleLandingStep'] == previous['sourceStep'])
                    state['loadAnchor'] = dict(source_step=previous['sourceStep'], source_time=previous['sourceTimeS'],
                        step_index=index, new_side=reference['swing_side'],
                        measured_normal_n=previous['feet'][reference['swing_side']]['loadN'],
                        body_weight_n=entry['massKg']*9.81, qualified_landing_samples=state['landRun'])
                # Controller construction preserves measured entry height.
                # Cadenced loaded handoff replaces com0 with the previous
                # desired final target, but does not replace finalCom's z.
                final = state['entryCom'].copy()
                final[:2] = np.mean(np.asarray([f['position'] for f in previous['feet']]), axis=0)[:2]+.035*np.asarray(reference['heading'])[:2]
                state['final'] = final
        elapsed = (n-state['phaseStart'])*DT
        if n % 4 == 0:
            note = next_note; next_note = next(stream, None); commands += 1
            check('everyActualCommandAligned', isinstance(note, dict) and note.get('stepIndex') == index
                and note.get('phase') == phase and close(note.get('t'), n*DT, 1e-10))
            if not isinstance(note, dict): break
            receipt = note.get('referenceContext', {})
            # q0 may be reset only at the inherited measured settle/stand
            # transition. The unchanged knee policy and already-prepared arms
            # then replace exactly their own coordinates.
            state['q0'][[6, 12]] = np.asarray(reference['knee_goal'])
            arm = note.get('opposedArmSwing', {})
            try:
                state['q0'][15:] = array(arm['targetArmJointRad'], (6,))
                original_final = state['entryCom'].copy()
                original_final[:2] = (np.asarray(reference['landing'])[:2]+np.asarray(reference['targets'][reference['stance_side']][0])[:2])/2.+.035*np.asarray(reference['heading'])[:2]
                final = state['final'] if phase in ('settle', 'stand') and state['final'] is not None else original_final
                result = audit_context(receipt, note, previous, reference, expected_elapsed=elapsed,
                    expected_q0=state['q0'], expected_floor=state['floor'], expected_final=final,
                    previous_arm=previous_arm, expected_mode=mode, expected_load_anchor=state['loadAnchor'], reference_policy=reference_policy)
                for key, value in result['checks'].items(): check(key, value)
                previous_arm = dict(time=previous['sourceTimeS'], rate=array(receipt['causal']['arm']['measuredLegPitchRateRadps'], (2,)))
                epoch = receipt['epoch']; key = (index, receipt['source_step'], phase, epoch)
                check('referenceEpochMonotonicWithinStep', epoch >= max_epoch.get(index, 0))
                max_epoch[index] = max(epoch, max_epoch.get(index, 0))
                old = epoch_bindings.get(key)
                check('sameCommandReferenceIdentityIdempotent', old is None or old == receipt['contextSHA256'])
                epoch_bindings[key] = receipt['contextSHA256']
                # One entry per command suffices; retain no growing receipt map.
                epoch_bindings = {key: receipt['contextSHA256']}
                seek = receipt['causal']['contactSeekAfter']
                before = receipt['causal']['contactSeekBefore']
                previous_seek = state['seek']
                expected_before = dict(timeS=None, depthM=0., rateMps=0.) if previous_seek is None else {
                    k: previous_seek[k] for k in ('timeS', 'depthM', 'rateMps')}
                initial_zero = previous_seek is None and equal(before, dict(timeS=state['phaseStarts']['transfer'], depthM=0., rateMps=0.))
                check('contactSeekHistoryCausalAndContinuous', equal(before, expected_before) or initial_zero)
                dt = 0. if before['timeS'] is None else previous['sourceTimeS']-before['timeS']
                depth, old_rate = float(before['depthM']), float(before['rateMps'])
                remaining = max(0., .003-depth); half = .5*.03*dt
                safe = max(0., math.sqrt(half**2+2*.03*max(0., remaining-.5*old_rate*dt))-half)
                desired = 0.; side = reference['swing_side']; stance = reference['stance_side']
                if (phase == 'land' and elapsed >= reference['phase_durations']['land']
                    and previous['feet'][side]['clearance'] <= .003
                    and previous['feet'][stance]['loadN'] >= .8*entry.get('massKg', 0.)*9.81
                    and previous['feet'][side]['loadN'] < 12.):
                    desired = min(.003, safe)
                new_rate = old_rate+float(np.clip(desired-old_rate, -.03*dt, .03*dt))
                new_depth = depth+min(remaining, .5*(old_rate+new_rate)*dt)
                acceleration = (new_rate-old_rate)/dt if dt > 0. else 0.
                check('contactSeekAnalyticUpdate', dt >= 0. and equal(seek.get('jet'), [new_depth, new_rate, acceleration])
                    and close(seek.get('timeS'), previous['sourceTimeS'])
                    and close(seek.get('depthM'), new_depth) and close(seek.get('rateMps'), new_rate))
                state['seek'] = copy.deepcopy(seek)
                if arm_mapping is not None:
                    check('persistentMappingNeutralAndRanges', equal(receipt['causal'].get('armMappingNeutralPitch'), arm_mapping.get('neutralArmPitchRad'))
                        and equal(receipt['config'].get('armMappingRanges'), arm_mapping.get('verifiedSwingDeltaRangeBySideRad',
                            [arm_mapping.get('verifiedSwingDeltaRangeRad')]*2)))
                command = array(note['actualMotorControl'], (21,))
                check('actualCommandDigestAndMotorBounds', is_sha(note.get('actualMotorControlSHA256'))
                    and command_sha(command) == note['actualMotorControlSHA256'] and np.max(np.abs(command)) <= .95000001)
                assembly = note.get('qpAssemblySHA256', {})
                check('completeActualQPAssemblyDigests', isinstance(assembly, dict) and set(assembly) == {'P', 'linear', 'A', 'b'}
                    and all(is_sha(value) for value in assembly.values()))
                check('reviewedEveryCommandExplicitlyDeclared', note.get('reviewedWalk', {}).get('schema') == 2
                    and note['reviewedWalk'].get('mode') == mode
                    and note['reviewedWalk'].get('referenceVersion') == VERSION
                    and note['reviewedWalk'].get('immutablePreparedContext') is True)
                support = note.get('robustSupportAdmission', {}); state['support'] = support
                if mode == 'continuous-com-causal':
                    check('explicitCausalStateOnlyPolicyEveryCommand', receipt['config'].get('causalSupportSelectionPolicy') == causal_selection_policy_audit()
                        and 'causalOverlap' in receipt['causal'])
                    overlap = receipt['causal'].get('causalOverlap'); correction = receipt['causal']['robustCorrection']
                    if not correction['selected']:
                        check('noOverlapCertificateBeforeSelection', overlap is None)
                    else:
                        check('selectedOverlapCertificateNeverReplaced', state.get('causalOverlap') is None or equal(state['causalOverlap'], overlap))
                        if state.get('causalOverlap') is None:
                            proof = audit_overlap_certificate(overlap, np.asarray(reference['com1'])-np.asarray(reference['com0']),
                                reference['phase_durations']['transfer'], correction['deltaM'], correction['startTransferS'], correction['durationS'])
                            check('independentSelectedWholePolynomialOverlap', proof.get('valid') is True and proof['independent']['passed'] is True
                                and isinstance(overlap, dict) and overlap.get('passed') is True and overlap.get('unresolvedCapBounds') == [])
                            overlap_proofs[index] = dict(passed=proof.get('valid') is True and proof['independent']['passed'] is True,
                                checks=proof['checks'], errors=proof['errors'],
                                independentEnvelope=proof['independent'], metadataTolerance=proof['metadataTolerance'],
                                certificateSHA256=value_sha(overlap), method='independent exact-rational Bernstein full-interval cap proof')
                            state['causalOverlap'] = copy.deepcopy(overlap)
                if mode in ('continuous-com', 'continuous-com-projected', 'continuous-com-load', 'continuous-com-causal'):
                    continuous_proof = audit_continuous_com(receipt, previous, reference, state, elapsed, reference_policy)
                    for key, value in continuous_proof['checks'].items(): check(key, value)
                correction = receipt['causal']['robustCorrection']
                check('contextCorrectionBoundToCausalAdmission', correction['selected'] is support.get('selected')
                    and (not correction['selected'] or (equal(correction['deltaM'], support.get('comCorrectionM'))
                        and close(correction['startTransferS'], support.get('selectedAtTransferS'))
                        and close(correction['durationS'], support.get('correctionDurationS')))))
                check('actualContextUsedByQP', equal(note.get('comTarget'), receipt['com']['position']))
            except (KeyError, ValueError, TypeError, IndexError):
                check('completeActualCommandReferenceTelemetry', False)
                command = None
        if command is not None and gears is not None:
            predicted = np.zeros(21)
            for motor, joint in enumerate(MOTOR_JOINT_ORDER): predicted[joint] = gears[joint]*command[motor]
            try:
                error = float(np.max(np.abs(array(row['actualMotorTorquesNm'], (21,))-predicted)))
                max_torque_error = max(max_torque_error, error)
                check('everyNativeTorqueMatchesHeldActualCommand', error <= 1e-8)
            except (ValueError, TypeError): check('everyNativeTorqueMatchesHeldActualCommand', False)
        if wrench_enabled:
            sample = audit_wrench_sample(row.get('solvedWrenchDiagnostics'), row, wrench_bindings,
                None if command is None else command_sha(command), previous_slip)
            for key, value in sample['checks'].items(): check(key, value)
            if sample['passed']: wrench_samples += 1
            if sample['values'] is not None:
                key = str(index)+':'+phase
                stats = wrench_phase.setdefault(key, dict(nativeSamples=0, normalForceMaximumN=[0., 0.],
                    netShearImpulseNs=[0., 0.], normalImpulseNs=[0., 0.], nearNetShearLowerBoundS=[0., 0.],
                    nearMotorFractionS=[0.]*21, nearJointLimitS=[0.]*21,
                    maximumActualMotorFractionByJoint=[0.]*21, minimumJointMarginRad=[1e300]*21,
                    netShearMaximumN=[0., 0.], maximumNetUtilizationLowerBound=[None, None],
                    loadedSamples=[0, 0], netUtilizationAtOrAbove95PercentSamples=[0, 0], loadedSlipM=[0., 0.]))
                stats['nativeSamples'] += 1
                for side in (0, 1):
                    value = sample['values']; ratio = value['netShearUtilizationLowerBound'][side]
                    stats['normalForceMaximumN'][side] = max(stats['normalForceMaximumN'][side], value['normalForceN'][side])
                    stats['netShearMaximumN'][side] = max(stats['netShearMaximumN'][side], value['netShearForceN'][side])
                    stats['loadedSlipM'][side] += value['loadedSlipIncrementM'][side]
                    stats['netShearImpulseNs'][side] += value['netShearForceN'][side]*DT
                    stats['normalImpulseNs'][side] += value['normalForceN'][side]*DT
                    if ratio is not None:
                        old = stats['maximumNetUtilizationLowerBound'][side]
                        stats['maximumNetUtilizationLowerBound'][side] = ratio if old is None else max(old, ratio)
                        stats['loadedSamples'][side] += 1
                        stats['netUtilizationAtOrAbove95PercentSamples'][side] += int(ratio >= .95)
                        if ratio >= .9: stats['nearNetShearLowerBoundS'][side] += DT
                try:
                    fractions = np.abs(array(row['actualMotorTorquesNm'], (21,)))/array(gears, (21,))
                    margins = array(row['jointLimitMarginsRad'], (21,))
                    if limits is not None:
                        q = array(row['qpos'], (28,))[7:]
                        expected_margins = np.minimum(q-limits[:, 0], limits[:, 1]-q)
                        check('wrenchSummaryJointMarginsMatchExecutedXML', close(margins, expected_margins, 1e-10)
                            and close(row.get('jointLimitMinimumMarginRad'), np.min(margins), 1e-10))
                    for joint in range(21):
                        stats['maximumActualMotorFractionByJoint'][joint] = max(stats['maximumActualMotorFractionByJoint'][joint], float(fractions[joint]))
                        stats['minimumJointMarginRad'][joint] = min(stats['minimumJointMarginRad'][joint], float(margins[joint]))
                        if fractions[joint] >= .925: stats['nearMotorFractionS'][joint] += DT
                        if margins[joint] <= .02: stats['nearJointLimitS'][joint] += DT
                except (KeyError, ValueError, TypeError, ArithmeticError):
                    check('completeWrenchSummaryMotorAndJointTelemetry', False)
        if 'loadedSlipDistanceM' in row: previous_slip = np.asarray(row['loadedSlipDistanceM'], dtype=float)
        if mode == 'continuous-com-load' and phase == 'land':
            side = reference['swing_side']; forward = float((np.asarray(row['feet'][side]['position'])
                -np.asarray(reference['targets'][side][0]))@np.asarray(reference['heading']))
            loaded = (row['feet'][side]['loadN'] >= 3. and row['feet'][1-side]['loadN'] >= 3.
                and row['feet'][side]['clearance'] <= .003 and forward >= .18)
            state['landRun'] = state['landRun']+1 if loaded else 0
            if state['landRun'] >= 80 and n*DT-state['phaseStarts']['land'] >= reference['phase_durations']['land']:
                if state['firstEligibleLandingStep'] is None: state['firstEligibleLandingStep'] = row['sourceStep']
            if state['firstEligibleLandingStep'] is not None:
                check('softLoadFirstEligibleLandingHandoffNeverDelayed', state['firstEligibleLandingStep'] == row['sourceStep'])
        # The inherited measured posture/admission test is evaluated after
        # every integration and can alter only the next command's unload floor.
        if phase == 'transfer' and elapsed >= reference['phase_durations']['transfer']:
            try:
                telemetry = note.get('robustSupportAdmission', {}) if n % 4 == 0 else last_support
                if telemetry.get('selected') and telemetry.get('correctionClock', [0.])[0] >= 1.:
                    from audit_source_robust_support import support_observation, triangle, measured_margin
                    obs = support_observation(row, reference['stance_side'])
                    margin = measured_margin(row, triangle(obs, telemetry['survivingCorners'], actual=True))
                    com = np.asarray(row['com']); cp = com[:2]+math.sqrt(com[2]/9.81)*np.asarray(row['comVelocity'])[:2]
                    goal = (reference_policy.goal(note, reference) if reference_policy else
                        np.asarray(reference['com1'])+np.asarray(telemetry['comCorrectionM']))
                    ready = (margin >= .003 and np.linalg.norm((com-goal)[:2]) < .008
                        and np.linalg.norm(cp-goal[:2]) < .01 and row['jointLimitMinimumMarginRad'] >= .005
                        and min(row['jointLimitMarginsRad'][i] for i in (7, 13)) >= reference.get('minimum_ankle_ready_rad', 0.)
                        and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .2)
                    if reference_policy is not None: ready = ready and reference_policy.ready(note, row)
                    if ready:
                        state['unload'] = True; state['floor'][reference['swing_side']] = reference['floorTargets'][reference['swing_side']]+.0006
            except (KeyError, ValueError, TypeError, IndexError):
                check('reconstructedActualUnloadFloor', False)
        if n % 4 == 0: last_support = note.get('robustSupportAdmission', {})
        previous = row
    check('completeReferenceCommandSchedule', count > 0 and commands == (count+3)//4 and next_note is None)
    check('allExpectedStepReferencesObserved', set(states) == set(references))
    if wrench_enabled: check('completeEnabledWrenchDiagnosticCoverage', wrench_samples == count and count > 0)
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v], actualRecordedIntegrations=count,
        actualControlNotes=commands, maximumHeldCommandTorqueErrorNm=max_torque_error,
        independentReferenceArithmetic=True, physicsIntegrations=0,
        causalOverlapEnvelopePass=(bool(overlap_proofs) and set(overlap_proofs) == set(references)
            and all(proof['passed'] for proof in overlap_proofs.values())) if mode == 'continuous-com-causal' else None,
        causalOverlapProofs=overlap_proofs,
        wrenchDiagnostics=dict(enabled=wrench_enabled, available=wrench_enabled and wrench_samples == count and count > 0,
            coveragePass=(wrench_samples == count and count > 0) if wrench_enabled else None,
            verifiedNativeSamples=wrench_samples, expectedNativeSamples=count if wrench_enabled else 0,
            perStepPhase=wrench_phase, individualContactNonsaturationEstablished=False,
            evidenceLimit='Logged live-cache measurements are bound to helper/model/source/control and checked for arithmetic/load/slip consistency. The process-global solve cache cannot be independently recovered from checkpoints; aggregate net utilization cannot establish individual-contact nonsaturation.'))


def independent_native_constraints(w, receipt, row, reference, support):
    """Rebuild immutable arm/capture inequalities from native current geometry."""
    from scipy.spatial import ConvexHull
    from audit_source_robust_support import support_observation, triangle
    c = receipt['config']; phase = receipt['phase']; velocity = w.arr(1, (27,)).copy()
    single = phase in ('lift', 'translate') or (phase == 'land' and w.arr(20, (17,))[c['swing_body']] < 3.)
    width = 60 if single else 72; result = []
    positions = w.arr(3, (17, 3)); R = w.arr(5, (17, 3, 3))[1]
    Jr = w.read('lh_body_jac', (6, 27), 1)[3:]; omega = R.T@(Jr@velocity)
    br = w.read('lh_jdot', (6,), 1)[3:]
    for index, side in enumerate(('right', 'left')):
        ia, ib = [w.l.lh_id(w.h, 1, name.encode()) for name in (side+'_thigh', side+'_shin')]
        Jdelta = w.read('lh_body_jac', (6, 27), ib)[:3]-w.read('lh_body_jac', (6, 27), ia)[:3]
        bdelta = w.read('lh_jdot', (6,), ib)[:3]-w.read('lh_jdot', (6,), ia)[:3]
        d = R.T@(positions[ib]-positions[ia])
        skew = np.array([[0., -d[2], d[1]], [d[2], 0., -d[0]], [-d[1], d[0], 0.]])
        D = R.T@Jdelta+skew@(R.T@Jr); rate_vector = D@velocity
        bias = R.T@bdelta-2.*np.cross(omega, R.T@(Jdelta@velocity))+np.cross(d, R.T@br)+np.cross(omega, np.cross(omega, d))
        x, z = d[0], d[2]; r2 = x*x+z*z; gradient = np.array([-z, 0., x])/r2
        hessian = np.array([[2*x*z, 0., z*z-x*x], [0., 0., 0.], [z*z-x*x, 0., -2*x*z]])/r2**2
        J = gradient@D; angle = math.atan2(x, -z); rate = float(J@velocity)
        jet_bias = float(gradient@bias+rate_vector@hessian@rate_vector)
        lower = c['leg_origin'][index]-c['armMappingRanges'][index][1]+.02
        upper = c['leg_origin'][index]-c['armMappingRanges'][index][0]-.02
        a = np.zeros(width); a[:27] = J
        result.extend(((a, 2*(upper-angle-.2*rate)/.2**2-jet_bias),
            (-a, 2*(angle-lower+.2*rate)/.2**2+jet_bias)))
    J = w.read('lh_com_jac', (3, 27)); com = w.arr(6, (17, 3))[1].copy(); vel = J@velocity
    tau = math.sqrt(com[2]/9.81); tau_rate = vel[2]/(2*9.81*tau); capture = com[:2]+tau*vel[:2]
    bias = w.read('lh_jdot', (3,), -1)
    def faces(equations, reserves):
        for face, (reserve, dr, ddr) in zip(equations, reserves):
            normal, offset = face[:2], face[2]
            cp_margin, com_margin = -float(normal@capture+offset), -float(normal@com[:2]+offset)
            a = np.zeros(width); a[:27] = tau*normal@J[:2]
            result.append((a, 8*(cp_margin-reserve)-dr-normal@((1+tau_rate)*vel[:2]+tau*bias[:2])))
            a = np.zeros(width); a[:27] = normal@J[:2]
            result.append((a, 64*(com_margin-reserve)-16*normal@vel[:2]-normal@bias[:2]-16*dr-ddr))
    if phase in ('lift', 'translate', 'land') or (phase == 'settle' and c.get('loaded_settle_capture_barrier', False)):
        count = w.l.lh_support_count(w.h); points = np.unique(w.arr(21, (128,))[:2*count].reshape(-1, 2), axis=0)
        eq = ConvexHull(points).equations; faces(eq, [(.003, 0., 0.)]*len(eq))
    selected = receipt['causal']['robustCorrection']['selected']
    if selected and phase in ('transfer', 'lift', 'translate', 'land'):
        obs = support_observation(row, c['stance_side']); ids = support['survivingCorners']
        if all(obs['present'][ids]):
            eq = triangle(obs, ids, actual=True); diagnostics = receipt['diagnostics']['robustSupportBarriers']
            if not equal(diagnostics, support.get('barriers')):
                raise ValueError('immutable triangle bounds differ from independently replayed telemetry')
            faces(eq, [(b['reserveM'], b['reserveRateMps'], b['reserveAccelerationMps2']) for b in diagnostics])
    return result


def audit_wrench_summary(saved, replay, summary):
    """Independently reduce per-phase slip/impulses/utilization/limit exposure."""
    checks = {}
    def check(name, condition): checks[name] = bool(condition)
    try:
        coverage = replay['wrenchDiagnostics']; count = replay['actualRecordedIntegrations']
        check('wrenchSummaryEnabledNativeCoverage', coverage['enabled'] is True and coverage['coveragePass'] is True
            and saved.get('schema') == 1 and saved.get('nativeSamples') == count
            and saved.get('actualIntegrations') == summary['actualIntegrations'] == count
            and saved.get('sourceStepLast') == count and saved.get('allNativeSamplesCovered') is True
            and saved.get('physicsStepS') == DT and saved.get('stopped') == summary['stopped'])
        check('wrenchSummaryExactScopeAndThresholds', saved.get('footSideOrder') == ['left', 'right']
            and saved.get('jointOrder') == 'source qpos21; actuator-order permutation removed with B'
            and saved.get('thresholds') == dict(nearMotorFraction=.925, nearJointMarginRad=.02, nearNetShearUtilizationLowerBound=.9)
            and saved.get('individualContactNonsaturationEstablished') is False
            and saved.get('forceStage') == 'actual cached mj_step2 solve before integration; current geometry and material-point slip remain separately identified')
        actual = coverage['perStepPhase']; recorded = saved['perStepPhase']
        check('wrenchSummaryAllActualPhases', set(actual) == set(recorded))
        fields = ('nativeSamples', 'loadedSlipM', 'netShearImpulseNs', 'normalImpulseNs',
            'nearNetShearLowerBoundS', 'nearMotorFractionS', 'nearJointLimitS',
            'maximumActualMotorFractionByJoint', 'minimumJointMarginRad')
        for key, phase in actual.items():
            saved_phase = recorded[key]
            check('wrenchSummaryPhaseArithmetic:'+key, all(equal(saved_phase.get(k), phase[k], 1e-9) for k in fields)
                and equal(saved_phase.get('maximumNetShearUtilizationLowerBound'), phase['maximumNetUtilizationLowerBound'], 1e-10))
        total_slip = np.sum([phase['loadedSlipM'] for phase in actual.values()], axis=0)
        check('wrenchSummaryGlobalSlipSum', close(total_slip, summary['globalLoadedSlipM'], 1e-10)
            and close(saved.get('globalLoadedSlipM'), summary['globalLoadedSlipM'], 1e-10))
    except (KeyError, ValueError, TypeError, ArithmeticError):
        checks['completeWrenchSummary'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v], physicsIntegrations=0,
        individualContactNonsaturationEstablished=False)


class ExactReviewedForecastReplay(ExactForecastReplay):
    """Existing bounded exact-payload replay, with the reviewed context ABI.

    Reconstructing the controller never calls its observer or arm differentiator.
    The inherited native verifier forbids step entry points and checks all bytes.
    """
    def __init__(self, *args, reference_policy=None, **kwargs):
        self.reference_policy = reference_policy
        super().__init__(*args, **kwargs)

    @staticmethod
    def _equal(a, b, tolerance=1e-10):
        # Schema2 explicitly records the absent initial derivative history.
        return equal(a, b, tolerance)

    def _check_sources(self):
        super()._check_sources()
        hashes = self.receipt.get('codeSHA256AtStart', {})
        self._require(REQUIRED_SOURCES <= set(hashes), 'missing reviewed/core replay source hashes')
        self._require(self.receipt.get('codeSHA256AtEnd') == hashes, 'source start/end mismatch')

    def _probe(self, note, row, reference, forecast, w):
        from source_reference_context import ReferenceContext
        from source_step_probe import Controller
        envelope = forecast.get('probeControllerInput', {})
        self._require(envelope.get('coreEntryPoint') == 'Controller.solve with immutable reviewed reference context; no preparation', 'wrong reviewed probe entry point')
        self._require(envelope.get('phase') == 'transfer' and note.get('phase') == 'transfer', 'wrong reviewed probe phase')
        fields = envelope.get('fields', {}); receipt = envelope.get('referenceContext', {})
        self._require(isinstance(fields, dict) and isinstance(receipt, dict), 'missing reviewed probe fields/context')
        context = ReferenceContext.from_receipt(receipt)
        self._require(context.source_step == row['sourceStep'] and context.source_time == row['sourceTimeS']
            and context.step_index == note['stepIndex'] and context.phase == 'transfer', 'stale reviewed probe context')
        self._require(context.hypothetical_changes == ('unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm')
            and context.unload_admitted is True, 'incorrect reviewed hypothetical scope')
        t = float(envelope['elapsedTransferS'])
        manifest = self.manifests[note['stepIndex']]
        self._require(abs(t-(float(manifest['selectedAtTransferS'])-(manifest['samples'][-1]['sourceStep']-row['sourceStep'])*DT)) < 1e-9,
            'reviewed probe transfer clock mismatch')
        self._require(context.elapsed == t, 'context/probe elapsed clock mismatch')
        config = receipt['config']
        self._require(self.reference_policy is not None or (config.get('coupledPosture') is None
            and config.get('boundedSettle') is None and note.get('reviewedWalk', {}).get('featureProfile') is None),
            'undeclared reference feature in default profile')
        actual_receipt = note.get('referenceContext', {})
        # The observer may vote from the preselection draft. The newly selected
        # command also retains a distinct final-context parity proposal.
        same_draft = not (forecast.get('role') == 'causal-observer'
            and actual_receipt.get('causal', {}).get('robustSelectedBefore') is False
            and actual_receipt.get('causal', {}).get('robustCorrection', {}).get('selected') is True)
        if same_draft:
            for name in ('schema', 'version', 'source_step', 'source_time', 'step_index', 'phase', 'elapsed', 'epoch',
                    'com', 'joints', 'arms', 'arm_gains', 'orientations', 'capture_constraints', 'causal', 'config', 'diagnostics'):
                self._require(equal(receipt.get(name), actual_receipt.get(name)), 'actual/hypothetical shared reference mismatch: '+name)
        else:
            parity = [item for item in note.get('reviewedWalk', {}).get('forecasts', []) if item.get('role') == 'final-context-parity']
            self._require(len(parity) == 1, 'selection command missing unique final-context parity forecast')
            final_receipt = parity[0].get('probeControllerInput', {}).get('referenceContext', {})
            for name in ('com', 'joints', 'arms', 'arm_gains', 'orientations', 'capture_constraints', 'causal', 'config', 'diagnostics'):
                self._require(equal(final_receipt.get(name), actual_receipt.get(name)), 'final parity shared reference mismatch: '+name)
        bound_reference = self.reference_policy.bound_reference(reference, receipt) if self.reference_policy else reference
        self._require(all(equal(config.get(k), bound_reference[k]) for k in STATIC_FIELDS if k in bound_reference), 'forecast static step reference mismatch')
        self._require(equal(config['q0'][:15], reference['q0'][:15]), 'forecast leg/knee posture mismatch')
        self._require(audit_soft_load_context(receipt, note, None)['passed'], 'invalid forecast soft-load scope')
        allowed = set(config)-{'mode', 'referenceVersion', 'armMappingRanges', 'softLoadPreference', 'causalSupportSelectionPolicy', 'coupledPosture', 'boundedSettle'}
        self._require(set(fields) == allowed|{'arm_posture', 'arm_velocity', 'arm_acceleration', 'unload_admitted'}, 'unexpected reviewed probe fields')
        for name in allowed: self._require(equal(fields[name], config[name]), 'probe field/context mismatch: '+name)
        for name, field in (('arm_posture', 'position'), ('arm_velocity', 'velocity'), ('arm_acceleration', 'acceleration')):
            self._require(equal(fields[name], receipt['arms'][field]), 'probe prepared arm mismatch: '+name)
        self._require(equal(envelope.get('armMappingRanges'), config.get('armMappingRanges')), 'probe mapping range mismatch')
        self._require(fields['unload_admitted'] is True, 'probe unload force bound missing')
        # Independently reconstruct the draft/final tasks before the native QP.
        audit_config = copy.deepcopy(config); audit_config['initialFloorTargets'] = reference['floorTargets']
        planned = independent_references(audit_config, receipt['causal'], t, 'transfer', hypothetical=True, reference_policy=self.reference_policy)
        self._require(jet_matches(receipt['com'], planned['com'], 3), 'probe COM reference mismatch')
        self._require(jet_matches(receipt['joints'], planned['joints'], 21), 'probe joint/knee reference mismatch')
        self._require(equal(receipt['orientations'], planned['orientations']), 'probe orientation reference mismatch')
        for actual, expected in zip(receipt['feet'], planned['feet']):
            self._require(jet_matches(actual['translation'], expected[:3], 3)
                and close(np.asarray(actual['orientation']['rotation']).reshape(3, 3), expected[3])
                and close(actual['orientation']['velocity'], expected[4]) and close(actual['orientation']['acceleration'], expected[5]), 'probe foot reference mismatch')
        p = Controller.__new__(Controller); p.w = w
        for name, value in fields.items():
            if name == 'targets': value = [(np.asarray(x[0], dtype=float), np.asarray(x[1], dtype=float)) for x in value]
            elif name == 'flatRotations': value = [np.asarray(x, dtype=float) for x in value]
            elif isinstance(value, list): value = np.asarray(value, dtype=float)
            elif isinstance(value, dict): value = copy.deepcopy(value)
            setattr(p, name, value)
        p._reviewed_step_index = context.step_index; p._reference_epoch = context.epoch
        p._prepared_reference_context = context
        constraints = independent_native_constraints(w, receipt, row, reference, note.get('robustSupportAdmission', {}))
        self._require(len(constraints) == len(receipt['capture_constraints']) and all(close(a, b, 1e-8) and close(x, y, 1e-7)
            for (a, x), (b, y) in zip(constraints, receipt['capture_constraints'])), 'independent native immutable capture/arm constraint mismatch')
        self._last_probe = p
        self._require(is_sha(forecast.get('proposedMotorControlSHA256'))
            and command_sha(forecast.get('proposedMotorControl')) == forecast['proposedMotorControlSHA256'], 'proposed command digest mismatch')
        return p, t

    def __call__(self, note, row, reference, forecast):
        result = super().__call__(note, row, reference, forecast)
        if not result.get('passed') or not result.get('nativeVerified'): return result
        try:
            from source_reference_context import ReferenceContext
            from source_step_probe import Controller
            from source_arm_knee_mobility import native_bytes
            w = self.world; before = native_bytes(w); counter = w.l.lh_dim(w.h, 6)
            # Re-evaluate the exact proposal to bind the assembly digests too.
            probe = self._last_probe
            proposed, proposal_info = Controller.solve(probe, probe._prepared_reference_context.elapsed, 'transfer')
            self.qp_calls += 1
            self._require(proposal_info.get('qpAssemblySHA256') == forecast.get('qpAssemblySHA256'), 'proposed QP assembly digest mismatch')
            actual_context = ReferenceContext.from_receipt(note['referenceContext'])
            probe._reviewed_step_index = actual_context.step_index; probe._reference_epoch = actual_context.epoch
            probe._prepared_reference_context = actual_context
            expected_constraints = independent_native_constraints(w, note['referenceContext'], row, reference, note.get('robustSupportAdmission', {}))
            self._require(len(expected_constraints) == len(actual_context.capture_constraints) and all(close(a, b, 1e-8) and close(x, y, 1e-7)
                for (a, x), (b, y) in zip(expected_constraints, actual_context.capture_constraints)), 'actual immutable capture/arm constraints differ')
            control, actual_info = Controller.solve(probe, actual_context.elapsed, actual_context.phase)
            self.qp_calls += 1
            self._compare('actual reviewed command', control, note.get('actualMotorControl'), (21,), 1e-9)
            self._require(command_sha(note['actualMotorControl']) == note.get('actualMotorControlSHA256'), 'actual command digest mismatch')
            self._require(actual_info.get('qpAssemblySHA256') == note.get('qpAssemblySHA256'), 'actual QP assembly digest mismatch')
            self._require(native_bytes(w) == before and w.l.lh_dim(w.h, 6) == counter, 'reviewed replay changed native state')
            result['actualReferenceAndCommandVerified'] = True
        except Exception as error:
            result['passed'] = result['nativeVerified'] = False
            self.failures.append('reviewed actual command: '+str(error))
        return result


# Independent whole-interval polynomial proof, distinct from producer Sturm code.
from numbers import Real

F = Fraction
POLICY = 'causal-quintic-overlap-v1'
PRODUCER_METHOD = 'exact-rational-polynomial-sturm-interval-bounds'
CAPS = {
    'totalSpeedMps': .19260552331163003,
    'totalAccelerationMps2': .3901818436690813,
    'correctionMagnitudeM': .012,
    'minimumCorrectionDurationS': 1.,
    'correctionSpeedMps': .025,
    'correctionAccelerationMps2': .15,
}
ATOL = 2e-11
RTOL = 2e-10
ROOT_WIDTH = F(1, 2**64)
CHECK_ORDER = ('correctionDuration', 'correctionMagnitude', 'correctionSpeed',
               'correctionAcceleration', 'totalSpeed', 'totalAcceleration')


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('invalid ' + label)
    try:
        value = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError('invalid ' + label) from exc
    if not math.isfinite(value):
        raise ValueError('nonfinite ' + label)
    return value


def _vector(value, label):
    try:
        out = tuple(_number(x, label) for x in value)
    except TypeError as exc:
        raise ValueError('invalid ' + label) from exc
    if len(out) != 3:
        raise ValueError('invalid ' + label + ' dimension')
    return out


def _inputs(nominal_delta3, nominal_duration, correction_delta3,
            correction_start, correction_duration):
    n = _vector(nominal_delta3, 'nominal delta')
    c = _vector(correction_delta3, 'correction delta')
    T = _number(nominal_duration, 'nominal duration')
    S = _number(correction_start, 'correction start')
    D = _number(correction_duration, 'correction duration')
    if T <= 0 or D <= 0 or S < 0:
        raise ValueError('invalid positive duration or causal start')
    if not math.isfinite(S + D) or S + D <= S:
        raise ValueError('unrepresentable correction interval')
    return tuple(map(F, n)), F(T), tuple(map(F, c)), F(S), F(D)


def _trim(p):
    p = list(p)
    while len(p) > 1 and p[-1] == 0:
        p.pop()
    return p or [F(0)]


def _sum_poly(a, b):
    out = [F(0)] * max(len(a), len(b))
    for i, value in enumerate(a):
        out[i] += value
    for i, value in enumerate(b):
        out[i] += value
    return _trim(out)


def _derivative(p):
    return _trim([F(i) * c for i, c in enumerate(p) if i])


def _evaluate(p, u):
    total = F(0)
    for coefficient in reversed(p):
        total = total * u + coefficient
    return total


def _compose_affine(p, offset, scale):
    """Binomial expansion p(offset + scale*u), ascending power coefficients."""
    out = [F(0)] * len(p)
    for power, coefficient in enumerate(p):
        for j in range(power + 1):
            out[j] += coefficient * math.comb(power, j) * offset**(power-j) * scale**j
    return _trim(out)


def _divide(a, b):
    a, b = _trim(a), _trim(b)
    if b == [0]:
        raise ValueError('polynomial division by zero')
    quotient = [F(0)] * max(1, len(a) - len(b) + 1)
    while a != [0] and len(a) >= len(b):
        j = len(a) - len(b)
        scale = a[-1] / b[-1]
        quotient[j] += scale
        for i in range(len(b)):
            a[i+j] -= scale * b[i]
        a = _trim(a)
    return _trim(quotient), a


def _monic(p):
    return [x / p[-1] for x in p] if p != [0] else p


def _squarefree_interior(p):
    """Remove multiplicity and roots at 0/1, which are boundary candidates."""
    p = _trim(p)
    if len(p) <= 1:
        return [F(1)]
    a, b = p, _derivative(p)
    while b != [0]:
        _, remainder = _divide(a, b)
        a, b = b, _monic(remainder)
    p, remainder = _divide(p, _monic(a))
    if remainder != [0]:
        raise ValueError('square-free division did not close')
    while len(p) > 1 and p[0] == 0:
        p = p[1:]
    while len(p) > 1 and _evaluate(p, F(1)) == 0:
        p, remainder = _divide(p, [F(-1), F(1)])
        if remainder != [0]:
            raise ValueError('endpoint division did not close')
    return _monic(p)


def _bernstein(p):
    degree = len(p) - 1
    return [sum((p[i] * F(math.comb(k, i), math.comb(degree, i))
                 for i in range(k + 1)), F(0)) for k in range(degree + 1)]


def _bisect_controls(controls):
    row = list(controls)
    left, right = [row[0]], [row[-1]]
    while len(row) > 1:
        row = [(a+b)/2 for a, b in zip(row, row[1:])]
        left.append(row[0])
        right.append(row[-1])
    return left, list(reversed(right))


def _variations(controls):
    signs = [(x > 0) - (x < 0) for x in controls if x]
    return sum(a != b for a, b in zip(signs, signs[1:]))


def _isolate_stationary(poly, *, max_depth=256, max_nodes=20000):
    """Descartes in the Bernstein basis: 0 variations => 0 roots; 1 => 1.

    Square-free reduction ensures subdivision eventually separates real roots.
    Exact roots on subdivision boundaries are recorded once and excluded from
    the two open children by ignoring zero endpoint controls.
    """
    derivative = _derivative(poly)
    p = _squarefree_interior(derivative)
    if len(p) <= 1:
        return [], 0
    pending = [(F(0), F(1), _bernstein(p), 0)]
    roots, visited = [], 0
    while pending:
        left, right, controls, depth = pending.pop()
        visited += 1
        if visited > max_nodes:
            raise ValueError('Bernstein root-node limit exhausted')
        variations = _variations(controls)
        if variations == 0:
            continue
        if variations == 1 and right - left <= ROOT_WIDTH:
            roots.append((left, right))
            continue
        if depth >= max_depth:
            raise ValueError('Bernstein root-depth limit exhausted')
        middle = (left + right)/2
        a, b = _bisect_controls(controls)
        if a[-1] == 0:
            roots.append((middle, middle))
        pending.append((middle, right, b, depth+1))
        pending.append((left, middle, a, depth+1))
    roots.sort()
    if any(a[1] >= b[0] for a, b in zip(roots, roots[1:])):
        raise ValueError('Bernstein stationary intervals overlap')
    return roots, visited


def _position_poly(delta, duration, start, left, right):
    midpoint = (left + right)/2
    if midpoint <= start:
        return [[F(0)] for _ in delta]
    if midpoint >= start + duration:
        return [[d] for d in delta]
    # Expanded position is 10u^3 - 15u^4 + 6u^5. No factored jets.
    base = _compose_affine([F(0), F(0), F(0), F(10), F(-15), F(6)],
                           (left-start)/duration, (right-left)/duration)
    return [[d*x for x in base] for d in delta]


def _square_norm(vectors):
    out = [F(0)]
    for p in vectors:
        square = [F(0)] * (2*len(p)-1)
        for i, a in enumerate(p):
            for j, b in enumerate(p):
                square[i+j] += a*b
        out = _sum_poly(out, square)
    return out


def _exact_jets(delta, duration, start, t):
    if t <= start:
        return ([F(0)]*3, [F(0)]*3, [F(0)]*3)
    if t >= start + duration:
        return (list(delta), [F(0)]*3, [F(0)]*3)
    u = (t-start)/duration
    p = [F(0), F(0), F(0), F(10), F(-15), F(6)]
    return tuple([d*_evaluate(jet, u)/duration**order for d in delta]
                 for order, jet in enumerate((p, _derivative(p),
                                              _derivative(_derivative(p)))))


def _total_jets(n, T, c, S, D, t):
    first, second = _exact_jets(n, T, F(0), t), _exact_jets(c, D, S, t)
    return tuple([x+y for x, y in zip(a, b)] for a, b in zip(first, second))


def _sqrt_float(value, upward=True):
    if value < 0:
        raise ValueError('negative squared norm')
    if not value:
        return 0.
    exponent = value.numerator.bit_length() - value.denominator.bit_length()
    exponent -= exponent % 2
    try:
        result = math.ldexp(math.sqrt(float(value/F(2)**exponent)), exponent//2)
    except (OverflowError, ValueError) as exc:
        raise ValueError('nonfinite independent norm') from exc
    if not math.isfinite(result):
        raise ValueError('nonfinite independent norm')
    if upward:
        while F(result)**2 < value:
            result = math.nextafter(result, math.inf)
            if not math.isfinite(result):
                raise ValueError('nonfinite independent norm')
    else:
        while F(result)**2 > value:
            result = math.nextafter(result, 0.)
    return result


def _fraction_pair(value):
    return [value.numerator, value.denominator]


def _maximum_enclosure(poly, roots):
    samples = [(F(0), _evaluate(poly, F(0))), (F(1), _evaluate(poly, F(1)))]
    uppers = [value for _, value in samples]
    root_data = []
    for left, right in roots:
        midpoint = (left+right)/2
        samples.append((midpoint, _evaluate(poly, midpoint)))
        local = _compose_affine(poly, left, right-left)
        controls = _bernstein(local)
        upper = max(controls)
        uppers.append(upper)
        root_data.append({'unitIntervalExact': [_fraction_pair(left), _fraction_pair(right)],
                          'squaredValueUpperExact': _fraction_pair(upper)})
    witness, lower = max(samples, key=lambda x: (x[1], x[0]))
    upper = max(F(0), *uppers)
    if lower < 0 or upper < lower:
        raise ValueError('invalid independent norm enclosure')
    return lower, upper, witness, root_data


def _build(nominal_delta3, nominal_duration, correction_delta3, correction_start,
           correction_duration, *, max_depth=256, max_nodes=20000):
    n, T, c, S, D = _inputs(nominal_delta3, nominal_duration, correction_delta3,
                           correction_start, correction_duration)
    boundaries = sorted({F(0), T, S, S+D})
    windows = []
    for left, right in zip(boundaries, boundaries[1:]):
        np = _position_poly(n, T, F(0), left, right)
        cp = _position_poly(c, D, S, left, right)
        positions = [_sum_poly(a, b) for a, b in zip(np, cp)]
        velocity = [[a/(right-left) for a in _derivative(p)] for p in positions]
        acceleration = [[a/(right-left) for a in _derivative(p)] for p in velocity]
        polys = {'speed': _square_norm(velocity),
                 'acceleration': _square_norm(acceleration)}
        window = {'left': left, 'right': right,
                  'nominalActive': F(0) < (left+right)/2 < T,
                  'correctionActive': S < (left+right)/2 < S+D,
                  'polynomials': polys, 'metrics': {}}
        for name, p in polys.items():
            roots, nodes = _isolate_stationary(p, max_depth=max_depth, max_nodes=max_nodes)
            lower, upper, witness, root_data = _maximum_enclosure(p, roots)
            window['metrics'][name] = {'roots': roots, 'lower': lower, 'upper': upper,
                                       'witness': witness, 'rootData': root_data,
                                       'subdivisionNodes': nodes}
        windows.append(window)
    return (n, T, c, S, D), boundaries, windows


def independent_overlap_envelope(nominal_delta3, nominal_duration, correction_delta3,
                                 correction_start, correction_duration, *,
                                 max_depth=256, max_nodes=20000):
    """JSON-safe exact proof summary; algebra exhaustion returns passed=False.

    Invalid/nonfinite inputs raise ValueError, as the producer API does. Exact
    numerators/denominators accompany displayed bounds. All comparisons to caps
    are exact, with no ATOL/RTOL. Outside the reported boundary range both jets
    are constant and have zero speed/acceleration.
    """
    parsed = _inputs(nominal_delta3, nominal_duration, correction_delta3,
                     correction_start, correction_duration)
    n, T, c, S, D = parsed
    norm2 = sum((x*x for x in c), F(0))
    corr_speed2 = F(225, 64)*norm2/D**2
    corr_accel2 = F(100, 3)*norm2/D**4
    checks = {
        'correctionDuration': D >= F(CAPS['minimumCorrectionDurationS']),
        'correctionMagnitude': norm2 <= F(CAPS['correctionMagnitudeM'])**2,
        'correctionSpeed': corr_speed2 <= F(CAPS['correctionSpeedMps'])**2,
        'correctionAcceleration': corr_accel2 <= F(CAPS['correctionAccelerationMps2'])**2,
    }
    result = {'method': 'independent-exact-bernstein-descartes', 'resolved': False,
              'caps': dict(CAPS), 'checks': checks, 'passed': False,
              'unresolvedCapBounds': [], 'windows': [], 'maxima': {},
              'correctionSquared': {name: _fraction_pair(value) for name, value in
                (('magnitude', norm2), ('speed', corr_speed2), ('acceleration', corr_accel2))}}
    try:
        _, boundaries, windows = _build(*parsed, max_depth=max_depth, max_nodes=max_nodes)
        for name, cap_name, check_name in (
                ('speed', 'totalSpeedMps', 'totalSpeed'),
                ('acceleration', 'totalAccelerationMps2', 'totalAcceleration')):
            lower = max(w['metrics'][name]['lower'] for w in windows)
            upper = max(w['metrics'][name]['upper'] for w in windows)
            cap2 = F(CAPS[cap_name])**2
            checks[check_name] = upper <= cap2
            if lower <= cap2 < upper:
                result['unresolvedCapBounds'].append(name)
            result['maxima'][name] = {
                'lower': _sqrt_float(lower, False), 'upper': _sqrt_float(upper, True),
                'lowerSquaredExact': _fraction_pair(lower),
                'upperSquaredExact': _fraction_pair(upper)}
        for w in windows:
            public = {'startS': float(w['left']), 'endS': float(w['right']),
                      'nominalActive': w['nominalActive'],
                      'correctionActive': w['correctionActive']}
            for name, p in w['polynomials'].items():
                m = w['metrics'][name]
                public[name] = {
                    'squaredPolynomialExact': [_fraction_pair(x) for x in p],
                    'squaredDerivativeDegree': len(_derivative(p))-1,
                    'stationaryRootCount': len(m['roots']),
                    'stationaryTimesS': [float(w['left']+(w['right']-w['left'])*(a+b)/2)
                                         for a, b in m['roots']],
                    'stationaryRoots': m['rootData'],
                    'lower': _sqrt_float(m['lower'], False),
                    'upper': _sqrt_float(m['upper'], True),
                    'lowerSquaredExact': _fraction_pair(m['lower']),
                    'upperSquaredExact': _fraction_pair(m['upper']),
                    'subdivisionNodes': m['subdivisionNodes']}
            result['windows'].append(public)
        result['boundaryJets'] = []
        for t in boundaries:
            p, v, a = _total_jets(*parsed, t)
            result['boundaryJets'].append({'timeS': float(t),
                'positionM': list(map(float, p)), 'velocityMps': list(map(float, v)),
                'accelerationMps2': list(map(float, a))})
        result['resolved'] = not result['unresolvedCapBounds']
        result['passed'] = result['resolved'] and all(checks.values())
    except (ValueError, OverflowError, ZeroDivisionError) as exc:
        checks.update(totalSpeed=False, totalAcceleration=False)
        result['proofError'] = str(exc)
    result['rejections'] = [name for name in CHECK_ORDER if not checks.get(name, False)]
    return result


def audit_overlap_certificate(cert, nominal_delta3, nominal_duration, correction_delta3,
                              correction_start, correction_duration, *,
                              atol=ATOL, rtol=RTOL, max_depth=256, max_nodes=20000):
    """Audit producer metadata and independently prove its admission decision.

    certificateValid/valid distinguish metadata validity from envelopePassed.
    Thus an honest cap rejection can be a valid certificate. A producer's
    conservative unresolved rejection may disagree with an independently proved
    admission; that is exposed separately rather than silently accepted as an
    identical decision. passed requires validity AND independent admission.
    """
    if not (0 <= atol <= 1e-9 and 0 <= rtol <= 1e-9):
        raise ValueError('metadata tolerances must be in [0, 1e-9]')
    independent = independent_overlap_envelope(nominal_delta3, nominal_duration,
        correction_delta3, correction_start, correction_duration,
        max_depth=max_depth, max_nodes=max_nodes)
    errors, detail = [], {}

    def require(ok, label):
        detail[label] = bool(ok)
        if not ok:
            errors.append(label)

    def numeric(value):
        return _number(value, 'certificate numeric metadata')

    def near(actual, expected):
        actual, expected = numeric(actual), numeric(expected)
        return abs(actual-expected) <= atol + rtol*max(abs(actual), abs(expected))

    def exact(actual, expected):
        return F(numeric(actual)) == F(expected)

    def vector_near(actual, expected):
        return isinstance(actual, (list, tuple)) and len(actual) == 3 and all(
            near(a, b) for a, b in zip(actual, expected))

    def norm_matches(value, lower, upper):
        value = numeric(value)
        if value < 0:
            return False
        tolerance = F(atol + rtol*abs(value))
        return max(F(0), F(value)-tolerance)**2 <= lower and upper <= (F(value)+tolerance)**2

    try:
        require(isinstance(cert, dict), 'schema.object')
        if not isinstance(cert, dict):
            raise ValueError('certificate is not an object')
        require(type(cert['schema']) is int and cert['schema'] == 1, 'schema.version')
        require(cert['policy'] == POLICY, 'schema.policy')
        require(cert['method'] == PRODUCER_METHOD, 'schema.method')
        parsed, boundaries, windows = _build(nominal_delta3, nominal_duration,
            correction_delta3, correction_start, correction_duration,
            max_depth=max_depth, max_nodes=max_nodes)
        n, T, c, S, D = parsed
        require(independent['resolved'], 'independent.proofResolved')
        require(set(cert['caps']) == set(CAPS) and all(exact(cert['caps'][k], v)
                for k, v in CAPS.items()), 'inputs.frozenCaps')
        for name, expected_delta, expected_start, expected_duration in (
                ('nominal', n, F(0), T), ('correction', c, S, D)):
            item = cert[name]
            require(isinstance(item['deltaM'], (list, tuple)) and len(item['deltaM']) == 3
                    and all(exact(a, b) for a, b in zip(item['deltaM'], expected_delta)),
                    'inputs.'+name+'.delta')
            require(exact(item['startS'], expected_start), 'inputs.'+name+'.start')
            require(exact(item['durationS'], expected_duration), 'inputs.'+name+'.duration')
        norm2 = sum((x*x for x in c), F(0))
        for field, squared in (('magnitudeM', norm2),
                ('maximumSpeedMps', F(225, 64)*norm2/D**2),
                ('maximumAccelerationMps2', F(100, 3)*norm2/D**4)):
            value = numeric(cert['correction'][field])
            require(value >= 0 and F(value)**2 >= squared and norm_matches(value, squared, squared),
                    'correction.'+field)
        supplied_windows = cert['windows']
        require(isinstance(supplied_windows, list) and len(supplied_windows) == len(windows),
                'windows.count')
        candidate_times = {'speed': [float(t) for t in boundaries],
                           'acceleration': [float(t) for t in boundaries]}
        for index, (w, item) in enumerate(zip(windows, supplied_windows)):
            prefix = 'windows.'+str(index)+'.'
            require(exact(item['startS'], float(w['left'])) and
                    exact(item['endS'], float(w['right'])), prefix+'bounds')
            for key in ('nominalActive', 'correctionActive'):
                require(type(item[key]) is bool and item[key] == w[key], prefix+key)
            for name, poly in w['polynomials'].items():
                m = w['metrics'][name]
                times = item[name+'StationaryTimesS']
                require(isinstance(times, list) and len(times) == len(m['roots']),
                        prefix+name+'.stationaryCount')
                require(type(item[name+'StationaryRootCount']) is int and
                        item[name+'StationaryRootCount'] == len(m['roots']),
                        prefix+name+'.rootCount')
                require(type(item[name+'SquaredDerivativeDegree']) is int and
                        item[name+'SquaredDerivativeDegree'] == len(_derivative(poly))-1,
                        prefix+name+'.degree')
                expected_times = [float(w['left']+(w['right']-w['left'])*(a+b)/2)
                                  for a, b in m['roots']]
                require(all(near(a, b) for a, b in zip(times, expected_times)) and
                        all(numeric(a) <= numeric(b) for a, b in zip(times, times[1:])),
                        prefix+name+'.stationaryTimes')
                candidate_times[name].extend(numeric(t) for t in times)
                require(norm_matches(item[name+'UpperBound'], m['lower'], m['upper']),
                        prefix+name+'.upperBound')
        supplied_boundary = cert['boundaryJets']
        require(isinstance(supplied_boundary, list) and len(supplied_boundary) == len(boundaries),
                'boundaryJets.count')
        for index, (t, item) in enumerate(zip(boundaries, supplied_boundary)):
            require(exact(item['timeS'], float(t)), 'boundaryJets.'+str(index)+'.time')
            expected = _total_jets(*parsed, t)
            for name, vector in zip(('positionM', 'velocityMps', 'accelerationMps2'), expected):
                require(vector_near(item[name], vector), 'boundaryJets.'+str(index)+'.'+name)
        for name, maximum_key, conservative_key, time_key, jet_index in (
                ('speed', 'maximumTotalSpeedMps', 'conservativeMaximumTotalSpeedMps', 'maximumSpeedAtS', 1),
                ('acceleration', 'maximumTotalAccelerationMps2',
                 'conservativeMaximumTotalAccelerationMps2', 'maximumAccelerationAtS', 2)):
            lower = max(w['metrics'][name]['lower'] for w in windows)
            upper = max(w['metrics'][name]['upper'] for w in windows)
            require(norm_matches(cert[maximum_key], lower, upper), name+'.maximum')
            require(norm_matches(cert[conservative_key], lower, upper), name+'.conservativeMaximum')
            require(numeric(cert[conservative_key]) + atol + rtol*abs(numeric(cert[conservative_key]))
                    >= numeric(cert[maximum_key]), name+'.conservativeOrdering')
            supplied = cert['candidateCriticalTimesS'][name]
            expected = sorted(set(candidate_times[name]))
            # Float conversion can merge distinct exact times. Compare the
            # unique float representations, while still checking monotonicity.
            require(isinstance(supplied, list) and
                    all(numeric(a) <= numeric(b) for a, b in zip(supplied, supplied[1:])) and
                    len(set(map(numeric, supplied))) == len(expected) and
                    all(near(a, b) for a, b in zip(sorted(set(map(numeric, supplied))), expected)),
                    name+'.candidateTimes')
            maximum_time = numeric(cert[time_key])
            require(any(near(maximum_time, t) for t in expected), name+'.maximizerCandidate')
            jets = _total_jets(*parsed, F(maximum_time))[jet_index]
            squared_at_time = sum((x*x for x in jets), F(0))
            require(norm_matches(cert[maximum_key], squared_at_time, squared_at_time),
                    name+'.maximizerValue')
            candidate_max = max(sum((x*x for x in _total_jets(*parsed, F(t))[jet_index]), F(0))
                                for t in supplied)
            require(norm_matches(cert[maximum_key], candidate_max, candidate_max),
                    name+'.candidateMaximum')
        producer_checks = cert['checks']
        require(set(producer_checks) == set(CHECK_ORDER) and
                all(type(producer_checks[k]) is bool for k in CHECK_ORDER), 'decision.checkSchema')
        unresolved = cert['unresolvedCapBounds']
        require(isinstance(unresolved, list) and len(unresolved) == len(set(unresolved)) and
                all(name in ('speed', 'acceleration') for name in unresolved), 'decision.unresolvedSchema')
        conservative_mismatch = []
        for check in CHECK_ORDER:
            expected = independent['checks'][check]
            actual = producer_checks[check]
            metric = {'totalSpeed': 'speed', 'totalAcceleration': 'acceleration'}.get(check)
            allowed_conservative = (expected and not actual and metric in unresolved)
            require(actual == expected or allowed_conservative, 'decision.'+check)
            if allowed_conservative:
                conservative_mismatch.append(metric)
        for metric in unresolved:
            check, key, maximum_key, cap_key = (
                ('totalSpeed', 'conservativeMaximumTotalSpeedMps', 'maximumTotalSpeedMps', 'totalSpeedMps')
                if metric == 'speed' else
                ('totalAcceleration', 'conservativeMaximumTotalAccelerationMps2',
                 'maximumTotalAccelerationMps2', 'totalAccelerationMps2'))
            cap = CAPS[cap_key]
            require(not producer_checks[check] and numeric(cert[key]) >= cap and
                    numeric(cert[maximum_key]) <= cap + atol + rtol*cap,
                    'decision.unresolved.'+metric)
        require(type(cert['passed']) is bool and cert['passed'] == all(producer_checks.values()),
                'decision.passed')
        require(cert['rejections'] == [k for k in CHECK_ORDER if not producer_checks[k]],
                'decision.rejections')
        cap_agreement = all(producer_checks[k] == independent['checks'][k] for k in CHECK_ORDER)
    except (KeyError, TypeError, ValueError, IndexError, OverflowError, ZeroDivisionError) as exc:
        errors.append('malformedOrUnresolved: '+str(exc))
        cap_agreement, conservative_mismatch = False, []
    valid = not errors
    return {'passed': valid and independent['passed'], 'valid': valid,
            'certificateValid': valid, 'envelopePassed': independent['passed'],
            'capDecisionAgreement': cap_agreement,
            'conservativeUnresolvedMismatch': conservative_mismatch,
            'checks': detail, 'errors': errors, 'independent': independent,
            'metadataTolerance': {'absolute': atol, 'relative': rtol}}


def state_only_selection_from_history(history, row, reference, elapsed, mass, actual=False, prospective=False):
    """Pure causal selection from the retained 200 Hz preceding-state window."""
    from audit_source_robust_support import support_observation, triangle, inset_projection, measured_margin, RESERVE, ASSOCIATION
    if elapsed < 0. or not history:
        return None
    now = row['sourceTimeS']
    if now-history[0][0] < .1-1e-9:
        return None
    if prospective and len(history) < 21:return None
    votes = np.bincount([h[1] for h in history if not prospective or h[1] >= 0],minlength=4)
    weak = int(np.argmax(votes));survivors = [i for i in range(4) if i != weak]
    frequency = float(votes[weak]/len(history))
    if frequency < .8 or not all(h[3] and all(h[2][survivors]) for h in history):
        return None
    obs = support_observation(row,reference['stance_side'])
    eq = triangle(obs,survivors,actual=actual)
    goal = inset_projection(np.array(reference['com1'])[:2],eq,RESERVE+ASSOCIATION)
    delta = np.r_[goal-np.array(reference['com1'])[:2],0.]
    distance = float(np.linalg.norm(delta))
    duration = max(1.,1.875*distance/.025,math.sqrt(5.78*distance/.15))
    return dict(weakCorner=weak,survivingCorners=survivors,
        historySourceStartS=history[0][0],historySourceEndS=now,historySamples=len(history),
        shallowestFrequency=frequency,selectedAtTransferS=elapsed,comCorrectionM=delta.tolist(),
        correctionMagnitudeM=distance,correctionDurationS=duration,
        initialReserveM=min(measured_margin(row,eq)-.001,RESERVE))


def audit_causal_support_replay(rows,entry,references,cadence,notes,mass,endpoint_validator=None,
                 prospective=False,forecast_replayer=None,reference_policy=None):
    """Separate state-only profile; legacy timer-gated auditor is never changed."""
    from collections import deque
    from scipy.spatial import QhullError
    from audit_source_robust_support import (ACTUAL_PATCH, NOMINAL_PATCH, MAX_SHIFT, RESERVE,
        ASSOCIATION, support_observation, forecast_math, triangle, measured_margin, quintic)
    from audit_source_cadenced_steps import capture_margin
    checks = {}
    def check(name,condition):
        checks[name] = checks.get(name,True) and bool(condition)
    check('explicitStateOnlyCausalReplayProfile', cadence.get('reviewedReferenceProfile', {}).get('mode') == 'continuous-com-causal')
    policy = cadence.get('robustSupportAdmission',{})
    mode = policy.get('supportPatchConstruction',NOMINAL_PATCH)
    check('geometricRiskSignalPolicyOnly',policy.get('riskSignal','current-shallowest-endpoint-height') == 'current-shallowest-endpoint-height')
    if prospective:
        check('explicitProspectiveRiskPolicy',policy.get('riskSignal') == FORECAST_RISK
            and policy.get('forecastHorizonS') == .1 and policy.get('forecastControlApplied') is False
            and policy.get('forecastRolloutIntegrations') == 0 and forecast_replayer is not None)
    check('explicitActualSupportPatchConstruction',mode == ACTUAL_PATCH)
    check('robustAdmissionPolicy',policy.get('enabled') is True
        and policy.get('causalWeakCornerHistoryS') == .1 and policy.get('minimumFrequency') == .8
        and policy.get('actualSurvivorsRequired') is True and policy.get('maximumComCorrectionM') == MAX_SHIFT
        and policy.get('measuredAdmissionReserveM') == RESERVE
        and policy.get('allFrameOriginalCaptureGuardUnchanged') is True
        and policy.get('arbitraryCornerLossRobustnessClaim') is False)
    check('explicitPilotOrTenRequest',type(cadence.get('requestedSteps')) is int
        and cadence['requestedSteps'] in (1,10) and cadence.get('requestedDistanceM') == .18)
    stream = iter(notes);next_note = next(stream,None)
    previous = entry;count = control_count = 0;states = {};last_step = None
    metadata_keys = ('weakCorner','survivingCorners','historySourceStartS','historySourceEndS',
        'historySamples','riskAgreementFrequency' if prospective else 'shallowestFrequency','selectedAtTransferS','comCorrectionM',
        'correctionMagnitudeM','correctionDurationS')
    endpoint_error = 0.;maximum_history = 0
    for row in rows:
        index,phase = row['stepIndex'],row['phase'];count += 1
        reference = references[index]
        state = states.setdefault(index,dict(history=deque(),selection=None,unload=0,maximumUnloadRun=0,
            liftAdmitted=False,lastPhase=None,clock=(0.,0.,0.),minimumLiftActualMarginM=None,
            liftActualReserveSamples=0,maximumLiftActualReserveRun=0,selectionNativeStep=None,
            boundedUnavailableSolves=0,lastBoundedUnavailableCandidate=None))
        check('originalFootRadiiPreserved',reference.get('radii') == [.027,.027])
        if state['lastPhase'] != phase:
            state['phaseStartNativeStep'] = count-1
        elapsed = (count-1-state['phaseStartNativeStep'])*DT
        check('phaseClockReconstructedFromNativeCounters',abs(row['phaseElapsedS']-elapsed-DT) < 1e-9)
        if (count-1)%4 == 0:
            note = next_note;next_note = next(stream,None);control_count += 1
            check('allActualControlNotesAligned',note is not None and note.get('stepIndex') == index
                and note.get('phase') == phase and abs(note.get('t',-1.)-(count-1)*DT) < 1e-9)
            if note is None:
                break
            telemetry = note.get('robustSupportAdmission',{})
            check('robustTelemetryOnEveryActualControl',isinstance(telemetry,dict)
                and telemetry.get('causalHistoryOnly') is True and type(telemetry.get('selected')) is bool)
            if endpoint_validator is not None:
                error = endpoint_validator(previous)
                endpoint_error = max(endpoint_error,error)
                check('independentFootEndpointFK',error < 1e-9)
            if reference_policy is not None:
                for key,value in reference_policy.observe(note, previous, reference, elapsed, forecast_replayer).items(): check(key,value)
            eligible = None
            if phase == 'transfer' and state['selection'] is None:
                obs = support_observation(previous,reference['stance_side'])
                now = previous['sourceTimeS']
                loaded = previous['feet'][reference['stance_side']]['loadN'] >= .8*mass*9.81
                append = True;risk = int(np.argmax(obs['clearances']))
                if prospective:
                    if not loaded:state['history'].clear()
                    append = loaded
                    if append:
                        forecast = telemetry.get('currentCausalUnloadForecast',telemetry.get('selectedCausalUnloadForecast',{}))
                        calculation = forecast_math(forecast,previous,reference)
                        for key,value in calculation['checks'].items():check(key,value)
                        risk = calculation['risk']
                        if forecast_replayer is not None:
                            witness = forecast_replayer(note,previous,reference,forecast)
                            check('allRetainedNativeForecastWitnessesReplay',witness.get('passed') is True)
                        if state['history'] and now-state['history'][-1][0] > .0050001:state['history'].clear()
                if append:state['history'].append((now,risk,obs['present'],loaded,previous['sourceStep']))
                while len(state['history']) > 1 and state['history'][1][0] < now-.1-1e-10:
                    state['history'].popleft()
                maximum_history = max(maximum_history,len(state['history']))
                try:
                    eligible = state_only_selection_from_history(state['history'],previous,reference,elapsed,mass,
                        actual=mode == ACTUAL_PATCH,prospective=prospective) if append else None
                    if prospective and eligible is not None:
                        eligible['riskAgreementFrequency'] = eligible.pop('shallowestFrequency')
                    if eligible is not None and eligible['correctionMagnitudeM'] > MAX_SHIFT:
                        state['boundedUnavailableSolves'] += 1
                        state['lastBoundedUnavailableCandidate'] = {k:eligible[k] for k in (
                            'weakCorner','correctionMagnitudeM','selectedAtTransferS','historySourceStartS','historySourceEndS')}
                        check('unavailableCorrectionRemainsUnselected',telemetry.get('selected') is False)
                        if mode == ACTUAL_PATCH:
                            check('boundedRejectionTelemetryMatchesCurrentCausalCandidate',
                                telemetry.get('lastRejectedWeakCorner') == eligible['weakCorner']
                                and close(telemetry.get('lastRejectedCorrectionM'),eligible['correctionMagnitudeM'])
                                and telemetry.get('lastAdmissionRejection') == 'bounded correction unavailable; continue observing current support')
                        eligible = None
                    if eligible is not None:
                        certificate = (note.get('referenceContext', {}).get('causal', {}).get('causalOverlap')
                            if telemetry.get('selected') else telemetry.get('lastRejectedCausalOverlap'))
                        # The geometric replay is independent and compared at
                        # the unchanged selection tolerance. Prove the exact
                        # binary jet that the producer actually certified:
                        # equivalent projection arithmetic can differ by ULPs.
                        proposed = certificate['correction'] if isinstance(certificate, dict) else {}
                        check('causalOverlapInputsBoundToIndependentCandidate',
                            close(proposed.get('deltaM'), eligible['comCorrectionM'])
                            and close(proposed.get('startS'), elapsed)
                            and close(proposed.get('durationS'), eligible['correctionDurationS']))
                        inputs = (np.asarray(reference['com1'])-np.asarray(reference['com0']),
                            reference['phase_durations']['transfer'], proposed.get('deltaM'),
                            proposed.get('startS'), proposed.get('durationS'))
                        proof = audit_overlap_certificate(certificate, *inputs)
                        envelope = proof['independent']
                        check('everyCausalCandidateOverlapCertificateValid', proof.get('valid') is True)
                        state['overlapCandidates'] = state.get('overlapCandidates', 0)+1
                        if not envelope['passed']:
                            state['overlapRejectedCandidates'] = state.get('overlapRejectedCandidates', 0)+1
                            check('summedEnvelopeRejectionRemainsUnselected', telemetry.get('selected') is False
                                and telemetry.get('lastAdmissionRejection') == 'summed causal COM jets exceed frozen envelope')
                            eligible = None
                        else:
                            check('firstStateEligibleCertificatePassed', certificate is not None
                                and certificate.get('passed') is True and certificate.get('unresolvedCapBounds') == [])
                except (ValueError,QhullError):
                    check('validCausalSelectionGeometry',False)
            if state['selection'] is None:
                check('selectionAtFirstCausalEligibleSolve',bool(eligible) == (telemetry.get('selected') is True))
                if eligible is not None and telemetry.get('selected') is True:
                    if prospective:
                        ids = [h[4] for h in state['history']]
                        check('selectedWindowNativeWitnessCoverage',forecast_replayer is not None
                            and forecast_replayer.coverage(index,ids)
                            and telemetry.get('forecastHistoryNativeSteps') == ids
                            and telemetry.get('historyPredictedRiskCorners') == [h[1] for h in state['history']]
                            and telemetry.get('retainedExactNativeForecastWitnesses') == len(ids)
                            and telemetry.get('riskSignal') == FORECAST_RISK)
                    state['selection'] = eligible
                    state['selectionNativeStep'] = count-1
                    state['history'].clear()
            selected = state['selection']
            check('selectionNeverDisappearsOrReplacesHistory',telemetry.get('selected') is (selected is not None))
            if selected is not None:
                check('selectionMatchesPrecedingActualHistory',all(close(telemetry.get(k),selected[k]) for k in metadata_keys))
                check('selectionScopeAndPhysicalBounds',telemetry.get('allSurvivorsActuallySupportedThroughoutHistory') is True
                    and telemetry.get('maximumCorrectionM') == MAX_SHIFT
                    and telemetry.get('maximumReferenceSpeedMps') == .025
                    and telemetry.get('maximumReferenceAccelerationMps2') == .15
                    and telemetry.get('admissionReserveM') == RESERVE
                    and telemetry.get('referenceReserveM') == RESERVE+ASSOCIATION
                    and telemetry.get('endpointSupportAssociationToleranceM') == ASSOCIATION
                    and telemetry.get('arbitraryCornerLossRobustnessClaim') is False)
                if mode == ACTUAL_PATCH:
                    check('selectionDeclaresActualPointConstruction',telemetry.get('supportPatchConstruction') == ACTUAL_PATCH)
                delta = np.array(selected['comCorrectionM']);duration = selected['correctionDurationS']
                check('boundedC2Correction',selected['correctionMagnitudeM'] <= MAX_SHIFT+1e-12
                    and duration >= 1. and 1.875*np.linalg.norm(delta)/duration <= .025+1e-12
                    and (10/math.sqrt(3))*np.linalg.norm(delta)/duration**2 <= .15+1e-12)
                if phase == 'transfer':
                    state['clock'] = quintic(elapsed-selected['selectedAtTransferS'],duration)
                elif phase == 'settle':
                    a,b,c = quintic(elapsed,reference['phase_durations']['settle']);state['clock'] = (1.-a,-b,-c)
                else:
                    state['clock'] = (0.,0.,0.) if phase == 'stand' else (1.,0.,0.)
                if reference_policy is not None:
                    state['clock'] = reference_policy.correction_clock(note, state['clock'])
                check('analyticC2CorrectionClock',close(telemetry.get('correctionClock'),state['clock']))
                base = np.array(reference['com1'])
                if phase == 'transfer':
                    s,_,_ = quintic(elapsed,reference['phase_durations']['transfer'])
                    base = np.array(reference['com0'])+(base-np.array(reference['com0']))*s
                elif phase in ('settle','stand'):
                    s = quintic(elapsed,reference['phase_durations']['settle'])[0] if phase == 'settle' else 1.
                    base = base+(np.array(reference['finalCom'])-base)*s
                check('emittedCOMTargetMatchesBoundedCorrection',close(note.get('comTarget'),
                    reference_policy.expected_com(note, reference) if reference_policy else base+state['clock'][0]*delta))
                obs = support_observation(previous,reference['stance_side'])
                present = all(obs['present'][selected['survivingCorners']])
                barriers = telemetry.get('barriers',[])
                expected_barriers = phase in ('transfer','lift','translate','land') and present
                check('barriersNeverUseMissingOrUnselectedSupport',len(barriers) == (3 if expected_barriers else 0))
                if expected_barriers:
                    try:
                        eq = triangle(obs,selected['survivingCorners'],actual=mode == ACTUAL_PATCH)
                        com = np.array(previous['com']);cp = com[:2]+math.sqrt(com[2]/9.81)*np.array(previous['comVelocity'])[:2]
                        s,ds,dds = state['clock'] if phase == 'transfer' else (1.,0.,0.)
                        gap = RESERVE-selected['initialReserveM'];reserve = selected['initialReserveM']+s*gap
                        check('triangleBarrierMarginsAndC2ReserveReplay',all(
                            close(b.get('captureMarginM'),-float(e[:2]@cp+e[2]))
                            and close(b.get('comMarginM'),-float(e[:2]@com[:2]+e[2]))
                            and close(b.get('reserveM'),reserve) and close(b.get('reserveRateMps'),ds*gap)
                            and close(b.get('reserveAccelerationMps2'),dds*gap)
                            and b.get('actualSurvivingSupportOnly') is True and b.get('gainPerS') == 8.
                            for b,e in zip(barriers,eq)))
                    except (ValueError,QhullError):
                        check('triangleBarrierMarginsAndC2ReserveReplay',False)
            else:
                check('unselectedControllerAddsNoTriangleBarrier',telemetry.get('barriers') == [])
                base = np.array(reference['com1'])
                if phase == 'transfer':
                    s,_,_ = quintic(elapsed,reference['phase_durations']['transfer'])
                    base = np.array(reference['com0'])+(base-np.array(reference['com0']))*s
                check('unselectedCOMReferenceRemainsNominal',close(note.get('comTarget'),base))
        actual_margin = None
        selected = state['selection']
        if selected is not None:
            try:
                obs = support_observation(row,reference['stance_side'])
                actual_margin = measured_margin(row,triangle(obs,selected['survivingCorners'],actual=True))
            except (ValueError,QhullError):
                pass
        full_margin = capture_margin(row)
        check('unchangedEveryNativeActualHullCaptureGuard',full_margin is not None and full_margin >= 0.)
        ready = False
        if phase == 'transfer' and selected is not None:
            com = np.array(row['com']);cp = com[:2]+math.sqrt(com[2]/9.81)*np.array(row['comVelocity'])[:2]
            goal = reference_policy.goal(note, reference) if reference_policy else np.array(reference['com1'])+selected['comCorrectionM']
            ready = (elapsed >= reference['phase_durations']['transfer']-1e-10 and state['clock'][0] >= 1.
                and actual_margin is not None and actual_margin >= RESERVE
                and np.linalg.norm((com-goal)[:2]) < .008 and np.linalg.norm(cp-goal[:2]) < .01
                and row['jointLimitMinimumMarginRad'] >= .005
                and min(row['jointLimitMarginsRad'][i] for i in (7,13)) >= reference.get('minimum_ankle_ready_rad',0.)
                and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .2
                and row['feet'][reference['swing_side']]['loadN'] < .5
                and row['feet'][reference['stance_side']]['loadN'] >= .8*mass*9.81)
            if reference_policy is not None: ready = ready and reference_policy.ready(note, row)
            state['unload'] = state['unload']+1 if ready else 0
            state['maximumUnloadRun'] = max(state['maximumUnloadRun'],state['unload'])
            if endpoint_validator is not None and ready:
                error = endpoint_validator(row);endpoint_error = max(endpoint_error,error)
                check('independentFootEndpointFK',error < 1e-9)
        if phase == 'lift' and state['lastPhase'] == 'transfer':
            state['liftAdmitted'] = state['unload'] >= 80
            check('eachLiftHas80ActualTriangleCorrectedUnloadSamples',state['liftAdmitted'])
            state['liftAdmissionNativeStep'] = count-1
        if phase == 'lift':
            if actual_margin is not None:
                state['minimumLiftActualMarginM'] = actual_margin if state['minimumLiftActualMarginM'] is None else min(state['minimumLiftActualMarginM'],actual_margin)
            state['liftActualReserveSamples'] = state['liftActualReserveSamples']+1 if actual_margin is not None and actual_margin >= RESERVE else 0
            state['maximumLiftActualReserveRun'] = max(state['maximumLiftActualReserveRun'],state['liftActualReserveSamples'])
        state['lastPhase'] = phase;previous = row;last_step = index
    check('actualControlScheduleComplete',control_count == (count+3)//4 and next_note is None)
    check('allRequestedStepsHaveMeasuredCausalSelection',len(states) == cadence['requestedSteps']
        and all(s['selection'] is not None for s in states.values()))
    check('allRequestedLiftsHaveMeasuredActualTriangleAdmission',len(states) == cadence['requestedSteps']
        and all(s['liftAdmitted'] for s in states.values()))
    check('nonemptyReplay',count > 0)
    results = {i:{k:v for k,v in s.items() if k != 'history'} for i,s in states.items()}
    prospective_pass = (prospective and all(v for k,v in checks.items() if k != 'geometricRiskSignalPolicyOnly'))
    return dict(passed=all(checks.values()),prospectiveRiskReplayPass=prospective_pass,
        checks=checks,actualRecordedIntegrations=count,
        actualControlNotes=control_count,supportPatchConstruction=mode,
        nominalGeometryDiagnosticOnly=mode != ACTUAL_PATCH,maximumRetainedHistorySamples=maximum_history,
        independentFootEndpointMaximumErrorM=endpoint_error,perStep=results,
        replayProfile='continuous-com-causal-state-only-v1',
        interpretation='Actual-triangle admission is measured before lift. Minimum margins during lift are reported separately; arbitrary corner-loss robustness is not claimed.')


def reviewed_robust_result(legacy, count, reference_proof, *, mode="baseline-context"):
    """Retain old output verbatim; replace only explicitly named policy checks."""
    old = legacy.get('checks', {})
    exclusions = {'geometricRiskSignalPolicyOnly', 'explicitPilotOrTenRequest',
        'emittedCOMTargetMatchesBoundedCorrection', 'unselectedCOMReferenceRemainsNominal'}
    checks = dict(fullLegacyCausalAndNativeCoverage=REQUIRED_ROBUST_REPLAY_CHECKS <= set(old) and all(v is True for k, v in old.items() if k not in exclusions),
        diagnosticOrPilotOrTenCount=type(count) is int and count in COUNTS,
        independentReviewedReferenceReplay=reference_proof.get('passed') is True)
    if mode == 'continuous-com-causal':
        required = ('explicitStateOnlyCausalReplayProfile',
            'causalOverlapInputsBoundToIndependentCandidate',
            'everyCausalCandidateOverlapCertificateValid', 'firstStateEligibleCertificatePassed')
        checks['independentStateOnlySelectionAndEnvelopeAdmission'] = (
            legacy.get('replayProfile') == 'continuous-com-causal-state-only-v1'
            and all(old.get(k) is True for k in required)
            and reference_proof.get('causalOverlapEnvelopePass') is True)
    return dict(passed=all(checks.values()), checks=checks,
        failedChecks=[k for k, v in checks.items() if not v], unchangedLegacyReplay=legacy,
        causalModeReplayVerified=mode == 'continuous-com-causal' and all(checks.values()),
        replacedLegacyPolicyChecks=sorted(exclusions), oldProfileQualificationClaim=False)


def limits_from_bound_xml(path, model):
    """Read selected MuJoCo ranges, not the older presentation model ranges."""
    from xml.etree import ElementTree
    root = ElementTree.parse(path).getroot(); compiler = root.find('compiler')
    factor = 1. if compiler is not None and compiler.get('angle', 'degree') == 'radian' else math.pi/180.
    joints = {j.get('name'): j for j in root.findall('.//worldbody//joint') if j.get('name')}
    limits = np.full((21, 2), np.nan)
    for part in model['segments']:
        h = part['reference'].get('hinge')
        if h:
            span = [float(x)*factor for x in joints[h['name']].get('range', '').split()]
            if len(span) != 2 or not span[0] < span[1]: raise ValueError('missing selected native joint limits')
            limits[h['qposIndex']] = span
    if not np.isfinite(limits).all(): raise ValueError('incomplete selected native joint ranges')
    return limits


def audit(folder, *, reference_policy=None):
    folder = Path(folder); base = folder.parents[1]
    read = lambda p: json.loads(Path(p).read_text())
    config = read(folder/'reviewed-walk-config.json'); receipt = read(folder/'reviewed-walk-run.json')
    cadence = read(folder/'cadenced-step-config.json'); summary = read(folder/'multistep-summary.json'); initial = read(folder/'summary.json')
    references = {s['index']: read(folder/f"controller-step-{s['index']:02d}.json") for s in summary['steps']}
    mapping = read(folder/'arm-mapping.json'); model_data = read(Path(__file__).resolve().parents[2]/'rust/crates/model/data/getup-v1.json')
    gears = np.zeros(21); limits = np.zeros((21, 2))
    for segment in model_data['segments']:
        hinge = segment['reference'].get('hinge')
        if hinge:
            index = hinge['qposIndex']; gears[index] = hinge['gear']
            # Native limits are independently checked in exact witness replay;
            # model JSON stores lower/upper under limitRad in retained versions.
            span = hinge.get('limitRad', hinge.get('limitsRad'))
            if span is None:
                axes = segment.get('jointProfile', {}).get('axes', [])
                if len(axes) == 1: span = [axes[0]['minRadians'], axes[0]['maxRadians']]
            if span is not None: limits[index] = span
    limits = limits_from_bound_xml(base/'model-session/source-model/humanoid_high_freq.xml', model_data)
    wrench_enabled = config.get('wrenchDiagnosticsEnabled', False) is True
    preset = read(Path(__file__).with_name('source-step-selected-v1.json'))
    wrench_bindings = dict(helperSHA256=preset['helperSHA256'], modelSHA256=initial['modelSHA256'],
        accessor='lh_last_ground_wrench', sideOrder=['left', 'right'])
    entry = dict(initial['entry'], massKg=model_data['totalMassKg'])
    with (folder/'frames.ndjson').open() as source:
        reference_proof = audit_reference_journal((json.loads(line) for line in source), entry, references,
            json_array_items(folder/'control.json'), mode=config['mode'], gears=gears, limits=limits, arm_mapping=mapping,
            wrench_enabled=wrench_enabled, wrench_bindings=wrench_bindings, reference_policy=reference_policy)
    wrench_summary_proof = None
    if wrench_enabled:
        saved_wrench_path = folder/'reviewed-wrench-summary.json'
        saved_wrench = read(saved_wrench_path) if saved_wrench_path.is_file() else {}
        wrench_summary_proof = audit_wrench_summary(saved_wrench, reference_proof, summary)
        reference_proof['checks']['independentWrenchPhaseSummaryReplay'] = wrench_summary_proof['passed']
        reference_proof['passed'] = all(reference_proof['checks'].values())
        reference_proof['failedChecks'] = [k for k, v in reference_proof['checks'].items() if not v]
    native = ExactReviewedForecastReplay(base, folder, references, receipt, mapping, reference_policy=reference_policy)
    if reference_policy is not None: reference_policy.install_native_windows(native)
    try:
        with (folder/'frames.ndjson').open() as source:
            replay = audit_causal_support_replay if config['mode'] == 'continuous-com-causal' else audit_replay
            legacy = replay((json.loads(line) for line in source), initial['entry'], references, cadence,
                json_array_items(folder/'control.json'), model_data['totalMassKg'], FootEndpointFK(),
                prospective=True, forecast_replayer=native, **(dict(reference_policy=reference_policy) if reference_policy else {}))
        native_proof = native.summary()
    finally:
        native.close()
    robust = reviewed_robust_result(legacy, config['requestedSteps'], reference_proof, mode=config['mode'])
    original = audit_cadence(folder)
    proof = audit_preparation(base, initial, expected_ratio=8.)
    final = cache_proof((folder/'final.bin').read_bytes(), proof['alteredModelSHA256'],
        file_sha(base/'model-session/source_bridge_v2.so'), file_sha(base/'model-session/dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    proof['finalCheckpoint'] = final
    proof['checks']['finalCheckpointBindingPayloadAndNativeCount'] = final['passed'] and final['nativeCounter'] == summary['actualIntegrations']
    proof['passed'] = all(proof['checks'].values())
    controls = audit_trial_controls(cadence, references, mapping, json_array_items(folder/'control.json'), summary['actualIntegrations'], expected_ratio=8.)
    policy = (type(config.get('requestedSteps')) is int and config['requestedSteps'] in COUNTS
        and cadence.get('requestedSteps') == config['requestedSteps'] and cadence.get('requestedDistanceM') == .18
        and cadence.get('loadedSettleHandoff') is True and cadence.get('continuousLoadedHandoff') is False
        and cadence.get('intermediateStableRequiredS') == 0. and cadence.get('initialTransferS') == 4.
        and cadence.get('phaseDurationsS') == dict(transfer=2., lift=.8, translate=1.2, land=1., settle=1.52)
        and cadence.get('originalContactJointMotorCollisionCaptureGates') is True
        and cadence.get('simultaneousArmCoordination', {}).get('enabled') is False)
    controls_pass = policy and all(v is True for k, v in controls['checks'].items() if k != 'unchangedCadenceControls')
    source_dir = Path(__file__).resolve().parent
    provenance = audit_source_provenance(receipt, source_dir, base/'frozen-source' if (base/'frozen-source').is_dir() else None)
    declarations = dict(explicitWrenchDiagnosticsDeclaration=type(config.get('wrenchDiagnosticsEnabled', False)) is bool
        and type(receipt.get('wrenchDiagnosticsEnabled', False)) is bool
        and config.get('wrenchDiagnosticsEnabled', False) == receipt.get('wrenchDiagnosticsEnabled', False),
        exactReviewedSchemaAndProfile=config.get('schema') == receipt.get('schema') == 2
        and config.get('profile') == receipt.get('profile') == (reference_policy.profile if reference_policy else PROFILE)
        and config.get('mode') == receipt.get('mode') and config.get('referenceVersion') == receipt.get('referenceVersion') == VERSION,
        explicitNoOriginalOrPrematureFinalClaim=config.get('originalModelQualificationClaim') is False
        and config.get('finalPlanQualificationClaim') is False and receipt.get('originalModelQualificationClaim') is False,
        sameRequestAndNativeAccounting=config.get('requestedSteps') == receipt.get('requestedSteps') == cadence.get('requestedSteps')
        and receipt.get('actualIntegrations') == summary['actualIntegrations']
        and receipt.get('physicsStepS') == DT and receipt.get('torqueUpdateS') == .005,
        unchangedPhysicalGatesAndLoadedSettle=config.get('loadedSettleProtocolUnchanged') is True
        and config.get('originalPhysicalGatesUnchanged') is True and receipt.get('implicitArms') is False
        and receipt.get('loadedSlipResetBetweenSteps') is False and receipt.get('rootAssistance') is False
        and receipt.get('physicalStateInstallationsAfterInitialization') == 0)
    expected_load = dict(enabled=True, policy=SOFT_LOAD_POLICY, weightPerNewton=.0005,
        maximumAcquiredWeightFraction=.1, activationS=.1, fadeDurationS=1.52,
        acquiredFrom='actual qualified land-to-settle native sample', noHardForceFloor=True,
        originalCornerNormalAndShearCostsUnchanged=True)
    declarations['explicitSoftLoadPreferencePolicy'] = (receipt.get('softLoadPreferenceEnabled') is True
        and config.get('softLoadPreference') == expected_load) if config['mode'] == 'continuous-com-load' else (
        receipt.get('softLoadPreferenceEnabled', False) is False and config.get('softLoadPreference') is None)
    declarations['explicitCausalSupportSelectionPolicy'] = (
        receipt.get('causalSupportSelectionEnabled') is True
        and config.get('causalSupportSelectionPolicy') == causal_selection_policy_audit()
        and cadence.get('reviewedReferenceProfile', {}).get('mode') == 'continuous-com-causal'
        ) if config['mode'] == 'continuous-com-causal' else (
        receipt.get('causalSupportSelectionEnabled', False) is False
        and config.get('causalSupportSelectionPolicy') is None)
    declarations['explicitFeatureProfileScope'] = (reference_policy.declarations(config, receipt, cadence) if reference_policy else
        config.get('coupledPostureEnabled', False) is False and receipt.get('coupledPostureEnabled', False) is False
        and cadence.get('reviewedReferenceProfile', {}).get('featureProfile') is None)
    provenance['checks'].update(declarations); provenance['passed'] = all(provenance['checks'].values())
    # Final B evidence is deliberately not inferred from names or runner booleans.
    result = qualify_reviewed(original, robust, reference_proof, proof, controls_pass, provenance,
        native_proof, config['requestedSteps'], mode=config['mode'])
    result.update(schema=2, savedDataOnly=True, physicsIntegrations=0, requestedSteps=config['requestedSteps'],
        mode=config['mode'], referenceReplay=reference_proof, robustReplay=robust,
        exactNativeForecastReplay=native_proof, explicitModelProof=proof,
        contactControlAudit=controls, explicitReviewedControlPolicyPass=policy,
        sourceProvenance=provenance, originalStrictAudit=original,
        wrenchDiagnostics=reference_proof['wrenchDiagnostics'], wrenchSummaryReplay=wrench_summary_proof,
        auditorSHA256=file_sha(Path(__file__)),
        legacyAuditorSHA256={name: file_sha(source_dir/name) for name in
            ('audit_source_cadenced_steps.py', 'audit_source_robust_support.py', 'audit_source_contact_impedance.py')},
        hashes={name: file_sha(folder/name) for name in ('reviewed-walk-config.json', 'reviewed-walk-run.json',
            'frames.ndjson', 'control.json', 'multistep-summary.json', 'summary.json', 'final.bin')},
        evidenceLimit='Every saved native sample is checked; exact retained native witnesses bind references and QP commands locally. Hashed saved data cannot exclude every unrecorded external intervention. Physical qualification alone does not establish visual/cadence improvement.')
    if reference_policy is not None:
        result = reference_policy.finish(result)
    if wrench_enabled and (folder/'reviewed-wrench-summary.json').is_file():
        result['hashes']['reviewed-wrench-summary.json'] = file_sha(folder/'reviewed-wrench-summary.json')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); result = audit(args.folder)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    keys = ('reviewedWalkDiagnosticPass', 'reviewedWalkPilotPhysicalPass', 'reviewedWalkTenStepPhysicalPass', 'reviewedWalkFinalPlanPass')
    print(json.dumps({key: result[key] for key in keys}|dict(failedChecks=result['failedChecks']), indent=2))
    raise SystemExit(0 if any(result[key] for key in keys) else 1)


if __name__ == '__main__': main()
