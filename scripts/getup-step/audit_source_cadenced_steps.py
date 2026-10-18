#!/usr/bin/env python3
"""Strict saved-data-only audit of one uninterrupted cadenced step sequence.

Pilot sequences can qualify for their configured count. Only a qualified
ten-step, 180 mm request sets tenStep180mmQualificationPass. No native world is
opened and no physics is integrated by this reducer.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

DT = .00125
PHASES = ('transfer', 'lift', 'translate', 'land', 'settle', 'stand')
ROOT = Path(__file__).resolve().parents[2]
IMPLICIT_ARM_MODE = 'same-QP-implicit-measured-thigh-acceleration'
IMPLICIT_ACCELERATION_MODEL = 'same-QP implicit measured thigh acceleration'
LOADED_SETTLE_READINESS = dict(consecutiveS=.1, minimumEachFootLoadN=3.,
    minimumTotalWeightFraction=.8, maximumFootClearanceM=.003,
    minimumCaptureMarginM=.01, minimumJointMarginRad=.005,
    maximumComTargetErrorM=.008, maximumCaptureTargetErrorM=.01,
    maximumBodyLinearMps=.05, maximumBodyAngularRadps=.2,
    forwardMidpointOffsetM=.035)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def json_array_items(path):
    """Stream the saved control array without retaining every command note."""
    decoder = json.JSONDecoder()
    buffer, position, eof, started = '', 0, False, False
    with Path(path).open() as source:
        while True:
            if not eof and (position >= len(buffer) or len(buffer)-position < 65536):
                buffer = buffer[position:]+source.read(1048576)
                position = 0
                eof = len(buffer) == 0 or source.tell() == Path(path).stat().st_size
            while position < len(buffer) and buffer[position].isspace():
                position += 1
            if not started:
                if position >= len(buffer) or buffer[position] != '[':
                    raise ValueError('control journal must be one JSON array')
                position += 1
                started = True
                continue
            if position < len(buffer) and buffer[position] == ',':
                position += 1
                continue
            if position < len(buffer) and buffer[position] == ']':
                if buffer[position+1:].strip() or source.read().strip():
                    raise ValueError('trailing data after control journal')
                return
            try:
                item, end = decoder.raw_decode(buffer, position)
            except json.JSONDecodeError:
                if eof:
                    raise ValueError('incomplete control journal')
                buffer = buffer[position:]+source.read(1048576)
                position = 0
                eof = source.tell() == Path(path).stat().st_size
                continue
            position = end
            yield item


def audit_implicit_provenance(cadence, receipt, notes, actual_integrations,
                              expected_module_sha256=None):
    """Pure opt-in provenance checks, independent of journal physical gates."""
    if 'simultaneousArmCoordination' not in cadence:
        return dict(declared=False, passed=True, checks={}, actualControlNotes=0)
    config = cadence['simultaneousArmCoordination']
    config = config if isinstance(config, dict) else {}
    if config.get('enabled') is False:
        return dict(declared=True, enabled=False, passed=True, checks={}, actualControlNotes=0)
    checks = dict(implicitConfigurationExact=(config.get('enabled') is True
        and config.get('mode', IMPLICIT_ARM_MODE) == IMPLICIT_ARM_MODE
        and config.get('finiteDifferenceAccelerationUsed') is False
        and config.get('positionGain') == 100. and config.get('velocityGain') == 20.
        and config.get('taskWeight') == 2. and config.get('angularGain') == -1.
        and config.get('targetClippingEnabled') is False))
    module_hash = receipt.get('codeSHA256AtStart', {}).get('source_implicit_arm_coordination.py', '')
    checks['implicitModuleSourceHash'] = bool(isinstance(module_hash, str)
        and re.fullmatch(r'[0-9a-fA-F]{64}', module_hash)
        and (expected_module_sha256 is None or module_hash.lower() == expected_module_sha256.lower()))
    checks.update(implicitEveryControlMode=True, implicitEveryControlGains=True,
        implicitEveryControlOpposedAcceleration=True, implicit200HzControlSchedule=True)
    count = 0
    for index, note in enumerate(notes):
        count += 1
        if not isinstance(note, dict):
            checks['implicitEveryControlMode'] = False
            checks['implicit200HzControlSchedule'] = False
            continue
        coordination = note.get('simultaneousArmCoordination', {})
        opposed = note.get('opposedArmSwing', {})
        coordination = coordination if isinstance(coordination, dict) else {}
        opposed = opposed if isinstance(opposed, dict) else {}
        checks['implicitEveryControlMode'] &= bool(coordination.get('mode') == IMPLICIT_ARM_MODE
            and coordination.get('sideOrder') == ['right','left']
            and coordination.get('finiteDifferenceAccelerationUsed') is False
            and coordination.get('targetClippingEnabled') is False)
        checks['implicitEveryControlGains'] &= bool(coordination.get('positionGain') == 100.
            and coordination.get('velocityGain') == 20. and coordination.get('taskWeight') == 2.)
        checks['implicitEveryControlOpposedAcceleration'] &= bool(
            opposed.get('accelerationTargetModel') == IMPLICIT_ACCELERATION_MODEL
            and opposed.get('finiteDifferenceLegAccelerationUsed') is False
            and 'targetArmAccelerationRadps2' in opposed
            and opposed['targetArmAccelerationRadps2'] is None)
        t = note.get('t')
        checks['implicit200HzControlSchedule'] &= bool(isinstance(t, (int,float))
            and math.isfinite(t) and abs(t-index*.005) < 1e-8)
    checks['implicit200HzControlSchedule'] &= count == (actual_integrations+3)//4
    checks['implicitControlNotesPresent'] = count > 0
    return dict(declared=True, enabled=True, passed=all(checks.values()), checks=checks,
        actualControlNotes=count, expectedControlNotes=(actual_integrations+3)//4,
        sourceModuleSHA256=module_hash, expectedMode=IMPLICIT_ARM_MODE)


def capture_margin(row):
    """Same LIP proxy, independently rebuilt from recorded current supports."""
    try:
        com = np.array(row['com'])
        capture = com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
        points = np.unique(np.array(row['supportPointsXY']).reshape(-1,2),axis=0)
        equations = ConvexHull(points).equations
        return float(min(np.min(-(equations[:,:2]@com[:2]+equations[:,2])),
                         np.min(-(equations[:,:2]@capture+equations[:,2]))))
    except (QhullError, ValueError):
        return None


def quiet(row, mass, drift, margin):
    return (row['phase'] == 'stand' and all(f['loadN'] >= 3 for f in row['feet'])
            and sum(f['loadN'] for f in row['feet']) >= .8*mass*9.81
            and row['torsoTiltDeg'] <= 5 and row['maximumBodyLinearMps'] <= .22
            and row['maximumBodyAngularRadps'] <= .65
            and margin is not None and margin >= .003
            and .780625 <= row['com'][2] <= .829976 and drift <= .01
            and row['maximumSelfPenetrationM'] <= .005)


def loaded_settle_ready(row, target, mass, margin):
    """Readiness from measured rows and an independently rebuilt target."""
    if target is None:
        return False
    com = np.array(row['com'])
    capture = com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
    return bool(all(f['loadN'] >= 3. and f['clearance'] <= .003 for f in row['feet'])
        and sum(f['loadN'] for f in row['feet']) >= .8*mass*9.81
        and margin is not None and margin >= .01 and row['jointLimitMinimumMarginRad'] >= .005
        and np.linalg.norm(com[:2]-np.array(target)[:2]) < .008
        and np.linalg.norm(capture-np.array(target)[:2]) < .01
        and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .2)


def audit_rows(rows, entry, records, references, cadence, mass, gears):
    """Pure streaming qualification, also used by synthetic corruption tests."""
    checks = {}
    def check(name, condition):
        checks[name] = checks.get(name, True) and bool(condition)

    expected = cadence['requestedSteps']
    requested = cadence['requestedDistanceM']
    continuous = cadence.get('continuousLoadedHandoff', False) is True
    loaded_settle = cadence.get('loadedSettleHandoff', False) is True
    def expected_phases(index):
        if index < expected:
            return PHASES[:4] if continuous else PHASES[:5] if loaded_settle else PHASES
        return PHASES
    def completion_phase(index):
        return ('land' if continuous else 'settle' if loaded_settle else 'stand') if index < expected else 'stand'
    check('explicitHandoffProtocol', type(cadence.get('continuousLoadedHandoff', False)) is bool
          and type(cadence.get('loadedSettleHandoff', False)) is bool and not (continuous and loaded_settle))
    if loaded_settle:
        policy = cadence.get('loadedSettleReadiness', {})
        check('originalLoadedSettleReadinessThresholds', isinstance(policy, dict)
              and all(policy.get(k) == v for k,v in LOADED_SETTLE_READINESS.items()))
    check('boundedRequestedCount', type(expected) is int and 1 <= expected <= 10)
    check('allExpectedStepRecords', [s['index'] for s in records] == list(range(1, expected+1)))
    check('originalQuietAndSlipThresholds', cadence['finalQuietRequiredS'] == 5.
          and cadence['intermediateStableRequiredS'] == (0. if continuous or loaded_settle else .25)
          and cadence['globalLoadedSlipLimitM'] == .01)
    check('physicalTimeConfiguration', cadence['physicalTimeOnly'] is True and cadence['playbackSpeed'] == 1.)
    states = {}
    previous_q = np.array(entry['qpos'])
    previous_time = entry['sourceTimeS']
    previous_step = entry['sourceStep']
    previous_feet = np.array([f['position'] for f in entry['feet']])
    initial_feet = previous_feet.copy()
    initial_com = np.array(entry['com'])
    initial_heading = Rotation.from_quat(previous_q[[4,5,6,3]]).as_matrix()[:,0]
    initial_heading[2] = 0.
    initial_heading /= np.linalg.norm(initial_heading)
    final_com = initial_com.copy()
    previous_slip = np.zeros(2)
    final_quiet = deque(maxlen=4000)
    transitions = []
    count = 0
    position_error = hinge_error = quaternion_error = time_error = slip_error = 0.
    motor = 0.
    minimum_capture = math.inf
    record_by_step = {s['index']: s for s in records}
    for row in rows:
        count += 1
        index, phase, side = row['stepIndex'], row['phase'], row['swingSide']
        check('contiguousNativeCounters', row['sourceStep'] == previous_step+1 and row['sourceStep'] == entry['sourceStep']+count)
        time_error = max(time_error, abs(row['sourceTimeS']-previous_time-DT),
                         abs(row['sourceTimeS']-entry['sourceTimeS']-count*DT))
        check('elapsedMatchesActualIntegrations', abs(row['elapsedS']-count*DT) < 1e-10)
        check('alternatingMeasuredSides', index in range(1, expected+1) and side == (index-1) % 2)
        if index not in references or index not in record_by_step:
            check('allExpectedReferences', False)
            continue
        reference, record = references[index], record_by_step[index]
        check('requestedDistancesAndSides', abs(row['requestedForwardM']-requested) < 1e-12
              and abs(record['requestedDistanceM']-requested) < 1e-12
              and record['swingSide'] == ('left' if side == 0 else 'right'))
        durations = reference.get('phase_durations', {})
        for key in PHASES[:-1]:
            target = cadence['initialTransferS'] if key == 'transfer' and index == 1 else cadence['phaseDurationsS'][key]
            check('controllerDurationsMatchConfiguration', durations.get(key) == target and target > 0.)
        first_sample = index not in states
        state = states.setdefault(index, dict(phases=[], clear=0, maximumClearRun=0, clearAtTransition=False,
            landing=0, landingAtRecordedTransition=False, maximumLandingRun=0,
            unload=0, unloadAtTransition=False, ready=0, anchor=None,
            finalForwardM=0., completedAtRecordedTransition=False, phaseLastNativeSteps={}, phaseEndElapsedS={},
            measuredSettleTargetSource=None, loadedSettleReadySamples=0, loadedSettleAtTransition=False))
        if first_sample:
            state['entryComSource'] = final_com.tolist()
            check('stepOriginsMatchPrecedingMeasuredFeet', np.max(abs(np.array([t[0] for t in reference['targets']])-previous_feet)) < 1e-10)
            check('stepStartCountersMatchJournal', record['startNativeStep'] == row['sourceStep']-1
                  and abs(record['startSourceTimeS']-previous_time) < 1e-8)
            if continuous and index > 1:
                prior = record_by_step.get(index-1, {})
                check('continuousHandoffBeginsOnNextSample', phase == 'transfer'
                      and previous_step == prior.get('landingNativeStep') == prior.get('completedNativeStep'))
            if loaded_settle and index > 1:
                prior = record_by_step.get(index-1, {})
                check('loadedSettleHandoffBeginsOnNextSample', phase == 'transfer'
                      and previous_step == prior.get('completedNativeStep')
                      and states.get(index-1, {}).get('phaseLastNativeSteps', {}).get('settle') == previous_step)
            heading = Rotation.from_quat(previous_q[[4,5,6,3]]).as_matrix()[:,0]
            heading[2] = 0.
            heading /= np.linalg.norm(heading)
            check('stepHeadingMatchesNativeRoot', np.max(abs(heading-reference['heading'])) < 1e-10)
        if not transitions or transitions[-1] != (index, phase):
            transitions.append((index, phase))
            state['phases'].append(phase)
            state['phaseStartNativeStep'] = count-1
        check('phaseElapsedMatchesNativeSamples', abs(row['phaseElapsedS']-(count-state['phaseStartNativeStep'])*DT) < 1e-9)
        state['phaseLastNativeSteps'][phase] = row['sourceStep']
        state['phaseEndElapsedS'][phase] = row['phaseElapsedS']
        q, v = np.array(row['qpos']), np.array(row['qvel'])
        check('finiteRecordedState', np.isfinite(q).all() and np.isfinite(v).all())
        position_error = max(position_error, float(np.max(abs(q[:3]-previous_q[:3]-DT*v[:3]))))
        hinge_error = max(hinge_error, float(np.max(abs(q[7:]-previous_q[7:]-DT*v[6:]))))
        delta = (Rotation.from_quat(previous_q[[4,5,6,3]]).inv()
                 * Rotation.from_quat(q[[4,5,6,3]])).as_rotvec()
        quaternion_error = max(quaternion_error, float(np.linalg.norm(delta-DT*v[3:6])))
        check('unitRootQuaternions', abs(np.linalg.norm(q[3:7])-1.) < 1e-6)
        slip = np.array(row['loadedSlipDistanceM'])
        increment = DT*np.array([f['loadedSlipSpeedMps'] for f in row['feet']])
        slip_error = max(slip_error, float(np.max(abs(slip-previous_slip-increment))))
        check('monotonicGlobalLoadedSlip', np.all(increment >= 0.) and np.all(slip >= previous_slip-1e-12))
        check('globalLoadedSlipWithinOriginalLimit', np.all(slip <= .01+1e-12))
        motor = max(motor, float(np.max(abs(np.array(row['actualMotorTorquesNm'])/gears))))
        check('originalGlobalPhysicalGates', row['warnings'] == 0 and row['nonfootLoadN'] < 3
              and not all(f['loadN'] < 3 for f in row['feet'])
              and row['maximumSelfPenetrationM'] <= .005 and row['com'][2] >= .65)
        check('postEntryJointLimits', row['elapsedS'] <= .06375 or row['jointLimitMinimumMarginRad'] >= 0.)
        margin = capture_margin(row)
        recorded_margin = row['minimumComCaptureMarginM']
        check('captureMarginTelemetryMatchesReconstruction',
              (margin is None and recorded_margin is None)
              or (margin is not None and recorded_margin is not None and abs(margin-recorded_margin) < 1e-10))
        check('definedNonnegativeCurrentCapture', margin is not None and margin >= 0.)
        if margin is not None:
            minimum_capture = min(minimum_capture, margin)
        rays = row['jointGroundRays'] + [r for foot in row['feet'] for r in foot['groundRays']]
        check('allOriginalGroundRays', len(row['jointGroundRays']) == 22 and len(rays) == 30
              and all(r['hit'] and r['ownBodyExcluded'] for r in rays))
        heading = np.array(reference['heading'])
        swing, support = row['feet'][side], row['feet'][1-side]
        observed = float((np.array(swing['position'])-np.array(reference['targets'][side][0]))@heading)
        check('forwardMatchesMeasuredFootPosition', abs(observed-row['swingForwardDisplacementM']) < 1e-10)
        state['finalForwardM'] = observed
        state['finalSupportRelativeForwardM'] = float((np.array(swing['position'])-np.array(support['position']))@heading)
        clear = phase == 'lift' and swing['clearance'] >= .025 and swing['loadN'] < 3 and support['loadN'] >= .8*mass*9.81
        state['clear'] = state['clear']+1 if clear else 0
        state['maximumClearRun'] = max(state['maximumClearRun'], state['clear'])
        if phase == 'lift':
            state['clearAtTransition'] = state['clear'] >= 80
        landed = phase == 'land' and all(f['loadN'] >= 3 for f in row['feet']) and swing['clearance'] <= .003 and observed >= requested
        state['landing'] = state['landing']+1 if landed else 0
        state['maximumLandingRun'] = max(state['maximumLandingRun'], state['landing'])
        if row['sourceStep'] == record.get('landingNativeStep'):
            state['landingAtRecordedTransition'] = (state['landing'] >= 80
                and row['phaseElapsedS']-DT >= durations['land']-1e-10)
            state['landingTimeS'] = row['elapsedS']
            state['landingForwardM'] = observed
            state['landingSupportRelativeForwardM'] = state['finalSupportRelativeForwardM']
            check('landingSummaryMatchesMeasuredTransition', abs(record['landingTimeS']-row['elapsedS']) < 1e-10
                  and abs(record['achievedAtLoadAcquisitionM']-observed) < 1e-10)
            if loaded_settle:
                positions = np.array([f['position'] for f in row['feet']])
                target = np.array(state['entryComSource'])
                target[:2] = np.mean(positions,axis=0)[:2]+.035*heading[:2]
                state['measuredSettleTargetSource'] = target.tolist()
                saved_feet = np.asarray(record.get('settleMeasuredFeet', []))
                saved_target = np.asarray(record.get('settleComTargetSource', []))
                reference_target = np.asarray(reference.get('finalCom', []))
                check('loadedSettleTargetReconstructedFromActualLanding',
                    saved_feet.shape == (2,3) and saved_target.shape == (3,) and reference_target.shape == (3,)
                    and np.max(abs(saved_feet-positions)) < 1e-10
                    and np.max(abs(saved_target-target)) < 1e-10
                    and np.max(abs(reference_target-target)) < 1e-10)
        com = np.array(row['com'])
        final_com = com.copy()
        capture = com[:2]+np.array(row['comVelocity'])[:2]*math.sqrt(com[2]/9.81)
        target = np.array(reference['com1'])
        ready_posture = (np.linalg.norm((com-target)[:2]) < .008 and np.linalg.norm(capture-target[:2]) < .01
            and row['jointLimitMinimumMarginRad'] >= .005
            and min(row['jointLimitMarginsRad'][i] for i in (7,13)) >= reference.get('minimum_ankle_ready_rad', 0.)
            and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .2)
        unloaded = (phase == 'transfer' and row['phaseElapsedS']-DT >= durations['transfer']-1e-10
                    and ready_posture and swing['loadN'] < .5 and support['loadN'] >= .8*mass*9.81)
        state['unload'] = state['unload']+1 if unloaded else 0
        if phase == 'transfer':
            state['unloadAtTransition'] = state['unload'] >= 80
        settle_target = state['measuredSettleTargetSource']
        settle_ready = loaded_settle and phase == 'settle' and loaded_settle_ready(row, settle_target, mass, margin)
        state['loadedSettleReadySamples'] = state['loadedSettleReadySamples']+1 if settle_ready else 0
        if phase == 'settle':
            state['loadedSettleAtTransition'] = state['loadedSettleReadySamples'] >= 80
        drift = math.inf
        if phase == 'stand':
            positions = np.array([f['position'] for f in row['feet']])
            if state['anchor'] is None:
                state['anchor'] = positions.copy()
            drift = float(np.max(np.linalg.norm(positions-state['anchor'], axis=1)))
        isquiet = quiet(row, mass, drift, margin)
        stable = (isquiet and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .15
                  and margin is not None and margin >= .01)
        state['ready'] = state['ready']+1 if stable else 0
        final_quiet.append((index, isquiet))
        if row['sourceStep'] == record.get('completedNativeStep'):
            if continuous and index < expected:
                state['completedAtRecordedTransition'] = (phase == 'land' and observed >= requested
                    and row['sourceStep'] == record.get('landingNativeStep')
                    and state['landingAtRecordedTransition'])
            elif loaded_settle and index < expected:
                state['completedAtRecordedTransition'] = (phase == 'settle' and observed >= requested
                    and row['phaseElapsedS']-DT >= durations['settle']-1e-10
                    and state['loadedSettleAtTransition'])
                check('loadedSettleCompletionSummaryMatchesReadiness',
                    record.get('completedAt') == 'measured-loaded-settle'
                    and record.get('loadedSettleReadyConsecutiveS', -1.) >= .1
                    and abs(record.get('loadedSettleReadyConsecutiveS', -1.)-state['loadedSettleReadySamples']*DT) < 1e-10)
            else:
                state['completedAtRecordedTransition'] = (phase == 'stand' and observed >= requested
                    and (index == expected or state['ready'] >= 200))
            check('completionSummaryMatchesMeasuredTransition', abs(record['completedTimeS']-row['elapsedS']) < 1e-10
                  and abs(record['achievedForwardM']-observed) < 1e-10)
        previous_q, previous_time, previous_step, previous_slip = q, row['sourceTimeS'], row['sourceStep'], slip
        previous_feet = np.array([f['position'] for f in row['feet']])
    check('nonemptyJournal', count > 0)
    check('exactOrderedConsecutivePhases', transitions == [(i,p) for i in range(1,expected+1) for p in expected_phases(i)])
    check('allMeasuredStepDistances', len(states) == expected and all(s['finalForwardM'] >= requested for s in states.values()))
    check('everyStepLiftQualified', len(states) == expected and all(s['clearAtTransition'] for s in states.values()))
    check('everyStepLandingQualified', len(states) == expected and all(s['landingAtRecordedTransition'] for s in states.values()))
    check('everyStepUnloadQualified', len(states) == expected and all(s['unloadAtTransition'] for s in states.values()))
    check('everyStepCompletionQualified', len(states) == expected and all(s['completedAtRecordedTransition'] for s in states.values()))
    check('landingAndCompletionAreActualPhaseEnds', all(
        s['phaseLastNativeSteps'].get('land') == record_by_step[i].get('landingNativeStep', -1)
        and s['phaseLastNativeSteps'].get(completion_phase(i)) == record_by_step[i].get('completedNativeStep', -1)
        for i,s in states.items()))
    check('everyMotionPhaseHonorsConfiguredDuration', all(
        all(s['phaseEndElapsedS'].get(p, -1.)-DT >= references[i]['phase_durations'][p]-1e-10 for p in expected_phases(i) if p != 'stand')
        for i,s in states.items()))
    if loaded_settle:
        check('allLoadedSettleTargetsReconstructed', len(states) == expected
              and all(s['measuredSettleTargetSource'] is not None for s in states.values()))
        check('everyIntermediateLoadedSettleQualified', len(states) == expected
              and all(s['loadedSettleAtTransition'] for i,s in states.items() if i < expected))
    check('finalFiveSecondsQuietOnFinalStep', len(final_quiet) == 4000 and all(i == expected and q for i,q in final_quiet))
    check('sourceTimeContinuity', time_error < 1e-8)
    check('rootPositionContinuity', position_error < 1e-10)
    check('hingePositionContinuity', hinge_error < 1e-10)
    check('rootQuaternionContinuity', quaternion_error < 1e-6)
    check('loadedSlipAccumulatorContinuity', slip_error < 1e-10)
    check('originalMotorLimit', motor <= .95000001)
    return dict(checks=checks, passed=all(checks.values()), actualRecordedIntegrations=count,
        continuousLoadedHandoff=continuous,
        loadedSettleHandoff=loaded_settle,
        physicsSimSeconds=count*DT, sourceTimeMaximumErrorS=time_error,
        positionVelocityContinuityMaxErrorM=position_error,
        hingePositionVelocityContinuityMaxErrorRad=hinge_error,
        quaternionVelocityContinuityMaxErrorRad=quaternion_error,
        loadedSlipAccumulatorMaximumErrorM=slip_error, globalLoadedSlipM=previous_slip.tolist(),
        maximumMotorFraction=motor, minimumComCaptureMarginM=minimum_capture if math.isfinite(minimum_capture) else None,
        netCOMForwardM=float((final_com-initial_com)@initial_heading),
        netFootMidpointForwardM=float((np.mean(previous_feet,axis=0)-np.mean(initial_feet,axis=0))@initial_heading),
        distanceDefinition='Each requested distance is swing-foot displacement from its own measured step-entry position, not landing separation beyond the other foot.',
        measuredPerStep={i:{k:v for k,v in s.items() if k != 'anchor'} for i,s in states.items()})


def audit(folder):
    folder = Path(folder)
    read = lambda name: json.loads((folder/name).read_text())
    cadence, summary, initial, receipt = [read(n) for n in (
        'cadenced-step-config.json', 'multistep-summary.json', 'summary.json', 'opposed-arm-run.json')]
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    gears = np.zeros(21)
    for part in model['segments']:
        hinge = part['reference'].get('hinge')
        if hinge:
            gears[hinge['qposIndex']] = hinge['gear']
    references = {s['index']:read(f"controller-step-{s['index']:02d}.json") for s in summary['steps']}
    with (folder/'frames.ndjson').open() as frames:
        measured = audit_rows((json.loads(line) for line in frames), initial['entry'],
            summary['steps'], references, cadence, model['totalMassKg'], gears)
    checks = measured['checks']
    count = measured['actualRecordedIntegrations']
    checks['consistentGlobalIntegrationAccounting'] = (initial['entry']['sourceStep'] == 0
        and count == summary['actualIntegrations'] == summary['totalActualIntegrations']
        == initial['actualIntegrations'] == initial['liveIntegrationsAcrossResets'] == receipt['actualIntegrations']
        and abs(summary['physicsSimSeconds']-count*DT) < 1e-10
        and count <= cadence['integrationBudget'])
    checks['completedRunner'] = summary['passed'] is True and receipt['finished'] is True
    checks['declaredNoResetOrRootAssistance'] = (initial['rootAssistance'] is False
        and initial['physicalStateWritesAfterInitialization'] == 0
        and summary['physicalStateInstallationsBetweenSteps'] == 0
        and receipt['rootAssistance'] is False and receipt['physicalStateInstallationsAfterInitialization'] == 0
        and cadence['noInterstepStateInstallation'] is True)
    checks['originalMotorFrictionAndClock'] = (receipt['originalMotorStrength'] is True
        and receipt['originalFriction'] is True and receipt['physicsStepS'] == DT
        and receipt['torqueUpdateS'] == .005 and receipt['selectedPresetChanged'] is False)
    checks['codeUnchangedDuringRun'] = receipt['codeUnchangedDuringRun'] is True
    frozen_implicit = folder.parents[1]/'frozen-source/source_implicit_arm_coordination.py'
    implicit_path = frozen_implicit if frozen_implicit.is_file() else Path(__file__).with_name('source_implicit_arm_coordination.py')
    implicit = audit_implicit_provenance(cadence, receipt,
        json_array_items(folder/'control.json') if 'simultaneousArmCoordination' in cadence else (),
        count, digest(implicit_path) if implicit_path.is_file() else '')
    implicit['sourceModulePathChecked'] = str(implicit_path)
    checks.update(implicit['checks'])
    selection = json.loads((folder.parents[1]/'selection.json').read_text())
    preset = json.loads((Path(__file__).with_name('source-step-selected-v1.json')).read_text())
    state_hash = hashlib.sha256(np.r_[initial['entry']['qpos'], initial['entry']['qvel']].astype('<f8').tobytes()).hexdigest()
    checks['selectedModelHelperAndInitialStateIdentity'] = (
        initial['modelSHA256'] == selection['modelSHA256'] == preset['selectedModelSHA256']
        and initial['helperSHA256'] == selection['helperSHA256'] == preset['helperSHA256']
        and state_hash == selection['initialQposQvelSHA256'] == preset['initialQposQvelSHA256'])
    model_path = folder.parents[1]/'model-session/source-model/humanoid_high_freq.xml'
    helper_path = ROOT/preset['helper']
    checks['retainedModelAndHelperBytesMatchIdentity'] = (model_path.is_file() and helper_path.is_file()
        and digest(model_path) == initial['modelSHA256'] and digest(helper_path) == initial['helperSHA256'])
    checks['summaryGlobalSlipMatchesJournal'] = bool(np.max(abs(np.array(summary['globalLoadedSlipM'])-measured['globalLoadedSlipM'])) < 1e-10)
    from audit_source_upright_large_steps import audit as audit_motion
    physical = audit_motion(folder)
    checks['existingMotionArmAndBodyFKQualification'] = physical['auditedOpposedStepPass']
    # Limb FK and exact target identity supplement the body-orientation FK in
    # the existing large-step audit. They do not open a native engine world.
    from audit_source_opposed_arm_swing import audit as audit_opposition
    opposition = audit_opposition(folder) if summary['passed'] else None
    checks['independentLimbFKAndExactOpposition'] = bool(opposition and opposition['physicalSwingQualificationPass'])
    measured['passed'] = all(checks.values())
    return dict(schema=1, savedDataOnly=True, physicsIntegrations=0,
        requestedSteps=cadence['requestedSteps'], requestedDistanceM=cadence['requestedDistanceM'],
        strictCadencedSequencePass=measured['passed'],
        tenStep180mmQualificationPass=bool(measured['passed'] and cadence['requestedSteps'] == 10
            and abs(cadence['requestedDistanceM']-.18) < 1e-12),
        failedChecks=[name for name,value in checks.items() if not value], measured=measured,
        originalMotionAudit=physical, opposedSwingAudit=opposition, implicitArmProvenance=implicit,
        evidenceLimit='No-reset/root-assistance declarations are checked alongside saved-state continuity and source receipts; logs alone cannot prove all possible unrecorded external forces absent.',
        hashes={n:digest(folder/n) for n in ('frames.ndjson','control.json','multistep-summary.json','summary.json','final.bin','cadenced-step-config.json','opposed-arm-run.json')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.folder)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('strictCadencedSequencePass','tenStep180mmQualificationPass','failedChecks')}, indent=2))
    raise SystemExit(0 if result['strictCadencedSequencePass'] else 1)


if __name__ == '__main__':
    main()
