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

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

DT = .00125
PHASES = ('transfer', 'lift', 'translate', 'land', 'settle', 'stand')
ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


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


def audit_rows(rows, entry, records, references, cadence, mass, gears):
    """Pure streaming qualification, also used by synthetic corruption tests."""
    checks = {}
    def check(name, condition):
        checks[name] = checks.get(name, True) and bool(condition)

    expected = cadence['requestedSteps']
    requested = cadence['requestedDistanceM']
    continuous = cadence.get('continuousLoadedHandoff', False) is True
    def expected_phases(index):
        return PHASES[:4] if continuous and index < expected else PHASES
    check('explicitHandoffProtocol', type(cadence.get('continuousLoadedHandoff', False)) is bool)
    check('boundedRequestedCount', type(expected) is int and 1 <= expected <= 10)
    check('allExpectedStepRecords', [s['index'] for s in records] == list(range(1, expected+1)))
    check('originalQuietAndSlipThresholds', cadence['finalQuietRequiredS'] == 5.
          and cadence['intermediateStableRequiredS'] == (0. if continuous else .25)
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
            finalForwardM=0., completedAtRecordedTransition=False, phaseLastNativeSteps={}, phaseEndElapsedS={}))
        if first_sample:
            check('stepOriginsMatchPrecedingMeasuredFeet', np.max(abs(np.array([t[0] for t in reference['targets']])-previous_feet)) < 1e-10)
            check('stepStartCountersMatchJournal', record['startNativeStep'] == row['sourceStep']-1
                  and abs(record['startSourceTimeS']-previous_time) < 1e-8)
            if continuous and index > 1:
                prior = record_by_step.get(index-1, {})
                check('continuousHandoffBeginsOnNextSample', phase == 'transfer'
                      and previous_step == prior.get('landingNativeStep') == prior.get('completedNativeStep'))
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
        and s['phaseLastNativeSteps'].get('land' if continuous and i < expected else 'stand') == record_by_step[i].get('completedNativeStep', -1)
        for i,s in states.items()))
    check('everyMotionPhaseHonorsConfiguredDuration', all(
        all(s['phaseEndElapsedS'].get(p, -1.)-DT >= references[i]['phase_durations'][p]-1e-10 for p in expected_phases(i) if p != 'stand')
        for i,s in states.items()))
    check('finalFiveSecondsQuietOnFinalStep', len(final_quiet) == 4000 and all(i == expected and q for i,q in final_quiet))
    check('sourceTimeContinuity', time_error < 1e-8)
    check('rootPositionContinuity', position_error < 1e-10)
    check('hingePositionContinuity', hinge_error < 1e-10)
    check('rootQuaternionContinuity', quaternion_error < 1e-6)
    check('loadedSlipAccumulatorContinuity', slip_error < 1e-10)
    check('originalMotorLimit', motor <= .95000001)
    return dict(checks=checks, passed=all(checks.values()), actualRecordedIntegrations=count,
        continuousLoadedHandoff=continuous,
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
        originalMotionAudit=physical, opposedSwingAudit=opposition,
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
