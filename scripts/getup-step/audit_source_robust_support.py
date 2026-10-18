#!/usr/bin/env python3
"""Saved-data replay for the explicit impratio8 causal-support experiment.

Original cadence results are retained verbatim. Only its original-model
identity and uncorrected-com1 unload test may be replaced, by separately
identified model and measured-triangle proofs. A pilot is never a ten-step pass.
"""
import argparse
import ast
from collections import deque
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

from audit_source_cadenced_steps import audit as audit_cadence, json_array_items, DT, capture_margin
from audit_source_contact_impedance import (audit_preparation, audit_trial_controls,
    cache_proof, file_sha, ORIGINAL_IDENTITY, REQUIRED_STRICT_CHECKS)

ROOT = Path(__file__).resolve().parents[2]
ACTUAL_PATCH = 'three-unique-nearest-actual-support-xy'
NOMINAL_PATCH = 'nominal-associated-corners'
FORECAST_RISK = 'planned-unload-QP-minimum-normal-pressure-and-native-100ms-clearance-agreement'
UNLOAD_CHECK = 'everyStepUnloadQualified'
RESERVE, ASSOCIATION, MAX_SHIFT = .003, .001, .012


def quintic(t, duration):
    x = float(np.clip(t/duration,0.,1.))
    return (10*x**3-15*x**4+6*x**5,
        (30*x*x-60*x**3+30*x**4)/duration,
        (60*x-180*x*x+120*x**3)/duration**2)


def inset_projection(point, equations, reserve):
    """Independent polygon clipping and nearest-edge projection in 2D."""
    point = np.asarray(point,dtype=float)
    polygon = [point+[-2.,-2.],point+[2.,-2.],point+[2.,2.],point+[-2.,2.]]
    for a,b,c in equations:
        normal = np.array([a,b]);limit = -c-reserve
        clipped = []
        for start,end in zip(polygon,polygon[1:]+polygon[:1]):
            ds,de = float(normal@start-limit),float(normal@end-limit)
            if ds <= 1e-13:
                clipped.append(start)
            if (ds < 0.) != (de < 0.):
                clipped.append(start+(end-start)*ds/(ds-de))
        polygon = clipped
        if not polygon:
            raise ValueError('empty support inset')
    if np.max(equations[:,:2]@point+equations[:,2]+reserve) <= 1e-12:
        return point.copy()
    candidates = []
    for start,end in zip(polygon,polygon[1:]+polygon[:1]):
        delta = end-start;length = float(delta@delta)
        candidates.append(start+delta*np.clip((point-start)@delta/length,0.,1.) if length else start)
    return min(candidates,key=lambda p:float(np.sum((p-point)**2))).copy()


def support_observation(row, side, radius=.027):
    """Reconstruct current physical capsule ends and ground gaps from saved rays."""
    rays = row['feet'][side]['groundRays']
    if len(rays) != 4 or not all(r['hit'] and r['ownBodyExcluded']
            and r.get('capsuleEndpoint',i) == i
            and np.max(abs(np.asarray(r.get('directionSource',[0.,0.,-1.]))-[0.,0.,-1.])) < 1e-12
            for i,r in enumerate(rays)):
        raise ValueError('invalid ordered foot endpoint rays')
    corners = np.array([r['originSource'] for r in rays],dtype=float)
    corners[:,2] -= .2+radius
    clearances = np.array([r['distanceM']-.2-radius for r in rays])
    supports = np.asarray(row['supportPointsXY'],dtype=float).reshape(-1,2)
    if not np.isfinite(corners).all() or not np.isfinite(clearances).all() or not np.isfinite(supports).all():
        raise ValueError('nonfinite support geometry')
    distances = np.linalg.norm(corners[:,:2,None]-supports.T[None,:,:],axis=1) if len(supports) else np.empty((4,0))
    nearest = np.argmin(distances,axis=1) if len(supports) else np.full(4,-1)
    present = np.array([len(supports) > 0 and distances[i,nearest[i]] <= ASSOCIATION for i in range(4)])
    matched = corners.copy()
    for i,index in enumerate(nearest):
        if present[i]:matched[i,:2] = supports[index]
    originally_present = present.copy()
    for i,index in enumerate(nearest):
        if originally_present[i] and np.count_nonzero((nearest == index)&originally_present) > 1:
            present[i] = False
    return dict(corners=corners,matchedCorners=matched,clearances=clearances,supports=supports,nearest=nearest,present=present)


def triangle(observation, survivors, actual=True):
    if not all(observation['present'][survivors]):
        raise ValueError('a surviving corner has no current actual support')
    indices = observation['nearest'][survivors]
    if len(set(indices.tolist())) != 3:
        raise ValueError('surviving corners do not have three unique actual supports')
    points = observation['supports'][indices] if actual else observation['corners'][survivors,:2]
    if np.unique(points,axis=0).shape != (3,2):
        raise ValueError('degenerate actual support triangle')
    return ConvexHull(points).equations


def measured_margin(row, equations):
    com = np.array(row['com'])
    cp = com[:2]+math.sqrt(com[2]/9.81)*np.array(row['comVelocity'])[:2]
    return float(min(np.min(-(equations[:,:2]@com[:2]+equations[:,2])),
        np.min(-(equations[:,:2]@cp+equations[:,2]))))


def selection_from_history(history, row, reference, elapsed, mass, actual=False, prospective=False):
    """Pure causal selection from the retained 200 Hz preceding-state window."""
    if elapsed < reference['phase_durations']['transfer'] or not history:
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


def close(a,b,tolerance=1e-9):
    try:
        a,b = np.asarray(a,dtype=float),np.asarray(b,dtype=float)
        return bool(a.shape == b.shape and np.isfinite(a).all() and np.isfinite(b).all()
            and (a.size == 0 or np.max(abs(a-b)) <= tolerance))
    except (ValueError,TypeError):
        return False


def forecast_math(forecast,row,reference):
    """Saved current geometry and pressure/clearance arithmetic, no dynamics."""
    checks = {};risk = -1
    try:
        arrays = {key:np.asarray(forecast[key],dtype=float) for key in (
            'virtualUnloadNormalForcesN','nativeUnloadEndpointVerticalAccelerationMps2',
            'nativeUnloadEndpointVerticalVelocityMps','localClearanceForecastM',
            'currentEndpointClearanceM','currentMatchedSupportCoordinatesSource',
            'proposedMotorControl','nativeProposedResponse')}
        shapes = [(4,)]*5+[(4,3),(21,),(33,)]
        checks['finiteForecastDimensions'] = all(a.shape == shape and np.isfinite(a).all()
            for a,shape in zip(arrays.values(),shapes))
        if not checks['finiteForecastDimensions']:raise ValueError('forecast dimensions')
        obs = support_observation(row,reference['stance_side'])
        checks['forecastUsesCurrentActualMatchedPoints'] = close(arrays['currentMatchedSupportCoordinatesSource'],obs['matchedCorners'])
        checks['forecastUsesCurrentPhysicalEndpointClearances'] = close(arrays['currentEndpointClearanceM'],obs['clearances'])
        predicted = (arrays['currentEndpointClearanceM']+.1*arrays['nativeUnloadEndpointVerticalVelocityMps']
            +.005*arrays['nativeUnloadEndpointVerticalAccelerationMps2'])
        p = int(np.argmin(arrays['virtualUnloadNormalForcesN']));c = int(np.argmax(predicted))
        checks['forecastArithmeticAndRiskIndexes'] = (close(predicted,arrays['localClearanceForecastM'])
            and forecast.get('pressureRiskCorner') == p and forecast.get('clearanceRiskCorner') == c)
        risk = p if p == c else -1
        checks['forecastIsPureBoundedUnappliedProposal'] = (forecast.get('predictionHorizonS') == .1
            and forecast.get('proposedControlApplied') is False and forecast.get('physicsRolloutIntegrations') == 0
            and forecast.get('nativeBytesAndCounterUnchanged') is True
            and max(abs(arrays['proposedMotorControl'])) <= .95000001
            and close(forecast.get('proposedMotorMaximumFraction'),max(abs(arrays['proposedMotorControl']))))
        checks['forecastBoundToCurrentNativeSample'] = (forecast.get('nativePayloadSourceStep') == row['sourceStep']
            and isinstance(forecast.get('nativePayloadSHA256'),str)
            and re.fullmatch('[0-9a-f]{64}',forecast['nativePayloadSHA256']) is not None)
    except (KeyError,ValueError,TypeError,IndexError):
        checks['validForecastTelemetry'] = False
    return dict(passed=all(checks.values()),checks=checks,risk=risk)


def audit_replay(rows,entry,references,cadence,notes,mass,endpoint_validator=None,
                 prospective=False,forecast_replayer=None):
    """Bounded-memory replay: one row, one note and <=22 history observations."""
    checks = {}
    def check(name,condition):
        checks[name] = checks.get(name,True) and bool(condition)
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
            eligible = None
            if phase == 'transfer' and state['selection'] is None:
                obs = support_observation(previous,reference['stance_side'])
                now = previous['sourceTimeS']
                loaded = previous['feet'][reference['stance_side']]['loadN'] >= .8*mass*9.81
                append = True;risk = int(np.argmax(obs['clearances']))
                if prospective:
                    if not loaded:state['history'].clear()
                    append = loaded and elapsed >= reference['phase_durations']['transfer']-.1-1e-10
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
                    eligible = selection_from_history(state['history'],previous,reference,elapsed,mass,
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
                check('analyticC2CorrectionClock',close(telemetry.get('correctionClock'),state['clock']))
                base = np.array(reference['com1'])
                if phase == 'transfer':
                    s,_,_ = quintic(elapsed,reference['phase_durations']['transfer'])
                    base = np.array(reference['com0'])+(base-np.array(reference['com0']))*s
                elif phase in ('settle','stand'):
                    s = quintic(elapsed,reference['phase_durations']['settle'])[0] if phase == 'settle' else 1.
                    base = base+(np.array(reference['finalCom'])-base)*s
                check('emittedCOMTargetMatchesBoundedCorrection',close(note.get('comTarget'),base+state['clock'][0]*delta))
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
            goal = np.array(reference['com1'])+selected['comCorrectionM']
            ready = (elapsed >= reference['phase_durations']['transfer']-1e-10 and state['clock'][0] >= 1.
                and actual_margin is not None and actual_margin >= RESERVE
                and np.linalg.norm((com-goal)[:2]) < .008 and np.linalg.norm(cp-goal[:2]) < .01
                and row['jointLimitMinimumMarginRad'] >= .005
                and min(row['jointLimitMarginsRad'][i] for i in (7,13)) >= reference.get('minimum_ankle_ready_rad',0.)
                and row['maximumBodyLinearMps'] <= .05 and row['maximumBodyAngularRadps'] <= .2
                and row['feet'][reference['swing_side']]['loadN'] < .5
                and row['feet'][reference['stance_side']]['loadN'] >= .8*mass*9.81)
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
        interpretation='Actual-triangle admission is measured before lift. Minimum margins during lift are reported separately; arbitrary corner-loss robustness is not claimed.')


class FootEndpointFK:
    def __init__(self):
        from source_step_preview import fk,P
        self.P = P
        model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
        self.kin = fk.Kinematics(model)
        self.primitives = {side:next(p['reference']['primitives'] for p in model['segments'] if p['id'] == side+'_foot') for side in ('left','right')}

    def __call__(self,row):
        q = np.array(row['qpos']);P = self.P;root = np.eye(4)
        root[:3,:3] = P.T@Rotation.from_quat(q[[4,5,6,3]]).as_matrix()@P;root[:3,3] = P.T@q[:3]
        poses = self.kin.forward(q[7:],root);error = 0.
        for index,side in enumerate(('left','right')):
            points = [P@(poses[side+'_foot']@np.r_[[p[k][a] for a in 'xyz'],1.])[:3]
                # Retained MuJoCo capsule local +z points toward fromto.a;
                # World.feet emits center-axis*half, then center+axis*half.
                for p in self.primitives[side] for k in ('b','a')]
            measured = [np.array(r['originSource'])-[0.,0.,.2] for r in row['feet'][index]['groundRays']]
            error = max(error,float(np.max(abs(np.array(points)-measured))))
        return error


def qualify_robust(original,replay,model_proof,control_checks,provenance,count):
    checks = original.get('measured',{}).get('checks',{})
    required = REQUIRED_STRICT_CHECKS-({'loadedSettleCompletionSummaryMatchesReadiness','loadedSettleHandoffBeginsOnNextSample'} if count == 1 else set())
    allowed = {ORIGINAL_IDENTITY,UNLOAD_CHECK}
    verdict = dict(originalFlagsRemainFalse=original.get('strictCadencedSequencePass') is False
        and original.get('tenStep180mmQualificationPass') is False and checks.get(ORIGINAL_IDENTITY) is False,
        everyOtherOriginalCheckPasses=all(v is True for k,v in checks.items() if k not in allowed),
        originalFullProtocolCoverage=required <= set(checks),
        requestedPilotOrTen=count in (1,10) and original.get('requestedSteps') == count
            and original.get('requestedDistanceM') == .18,
        independentActualTriangleReplay=replay.get('passed') is True,
        explicitImpratio8ModelProof=model_proof.get('passed') is True and model_proof.get('expectedImpratio') == 8.,
        unchangedOtherTrialControls=control_checks is True,strictSourceProvenance=provenance is True)
    passed = all(verdict.values())
    return dict(explicitRobustSupportImpratio8PilotPass=passed and count == 1,
        explicitRobustSupportImpratio8TenStepPass=passed and count == 10,
        checks=verdict,failedChecks=[k for k,v in verdict.items() if not v],
        originalStrictCadencedSequencePass=original.get('strictCadencedSequencePass'),
        originalTenStep180mmQualificationPass=original.get('tenStep180mmQualificationPass'),
        replacedOriginalChecks=sorted(allowed))


def qualify_prospective(original,replay,model_proof,control_checks,provenance,count,native_proof):
    """A distinct risk policy never inherits a geometric-profile green flag."""
    geometric = qualify_robust(original,replay,model_proof,control_checks,provenance,count)
    prospective = qualify_robust(original,dict(passed=replay.get('prospectiveRiskReplayPass') is True),
        model_proof,control_checks,provenance,count)
    native_pass = native_proof.get('passed') is True
    geometric.update(explicitProspectiveUnloadRiskImpratio8PilotPass=(
        prospective['explicitRobustSupportImpratio8PilotPass'] and native_pass),
        explicitProspectiveUnloadRiskImpratio8TenStepPass=(
        prospective['explicitRobustSupportImpratio8TenStepPass'] and native_pass),
        prospectiveQualificationChecks={**prospective['checks'], 'exactNativeAndProbeQPWitnessReplay':native_pass},
        prospectiveFailedChecks=prospective['failedChecks']+([] if native_pass else ['exactNativeAndProbeQPWitnessReplay']))
    geometric['geometricQualificationChecks'] = geometric['checks']
    geometric['geometricFailedChecks'] = geometric['failedChecks']
    geometric['checks'] = geometric['prospectiveQualificationChecks']
    geometric['failedChecks'] = geometric['prospectiveFailedChecks']
    return geometric



class ExactForecastReplay:
    """Exact selected-witness replay. Never advances the native world.

    Constructor performs metadata/source checks only; the one isolated World is
    created lazily for the first selected sample. No raw payload is retained.
    """
    MAX_RAW_BYTES = 32 * 1024 * 1024
    MAX_COMPRESSED_BYTES = 4 * 1024 * 1024
    MAX_MANIFEST_BYTES = 256 * 1024
    MAX_SAMPLES = 22
    FIELDS = frozenset(('q0','com0','com1','targets','rootR','flatRotations','radii',
        'floorTargets','swing_side','stance_side','swing_body','self_avoidance','B',
        'heading','landing','step_distance','finalCom','lean_axis','phase_durations',
        'joint_reserve','posture_start','shear_weight','trajectory_time_scale',
        'arm_posture','arm_velocity','arm_acceleration','arm_kp','arm_kd','arm_weight',
        'leg_origin','unload_admitted','_seek_time','_seek_depth','_seek_rate','_seek_jet'))
    SHAPES = dict(q0=(21,),com0=(3,),com1=(3,),rootR=(3,3),radii=(2,),
        floorTargets=(2,),B=(27,21),heading=(3,),landing=(3,),finalCom=(3,),
        lean_axis=(3,),posture_start=(21,),arm_posture=(6,),arm_velocity=(6,),
        arm_acceleration=(6,),leg_origin=(2,),_seek_jet=(3,))
    STATIC = ('com0','com1','targets','rootR','flatRotations','radii','swing_side',
        'stance_side','swing_body','self_avoidance','B','heading','landing','step_distance',
        'lean_axis','phase_durations','joint_reserve','posture_start','trajectory_time_scale',
        'arm_kp','arm_kd','arm_weight','leg_origin')

    def __init__(self, base, folder, references, receipt, arm_mapping):
        import importlib.util
        import json
        from pathlib import Path
        self.base, self.folder = Path(base), Path(folder)
        self.session = self.base/'model-session' if (self.base/'model-session').is_dir() else self.base
        self.references, self.receipt, self.arm_mapping = references, receipt, arm_mapping
        self.world = None
        self.manifests, self.expected, self.samples, self.seen = {}, {}, {}, {}
        self.covered, self.failures, self.source_checks = set(), [], {}
        self.results, self.maximum_errors = [], {}
        self.calls = self.unselected_calls = self.imports = self.qp_calls = self.response_calls = 0
        self._binding_checks = {}
        try:
            spec = importlib.util.find_spec('source_step_probe')
            self._require(spec is not None and spec.origin is not None, 'source_step_probe unavailable')
            self.source_dir = Path(spec.origin).resolve().parent
            preset = self._json(self.source_dir/'source-step-selected-v1.json')
            self.helper = self.source_dir.parents[1]/preset['helper']
            self.xml = self.session/'source-model/humanoid_high_freq.xml'
            self._check_sources()
            self.bindings = dict(modelSHA256=self._sha(self.xml),helperSHA256=self._sha(self.helper),
                bridgeSHA256=self._sha(self.session/'source_bridge_v2.so'),
                engineSHA256=self._sha(self.session/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
            proof_path = self.base/'contact-impedance-model.json'
            if proof_path.is_file():
                proof = self._json(proof_path)
                for name in ('modelSHA256','helperSHA256'):
                    self._require(proof.get(name)==self.bindings[name], 'prepared '+name+' mismatch')
            paths = sorted(self.folder.glob('robust-risk-step-*/manifest.json'))
            self._require(0 < len(paths) <= 10, 'missing or excessive witness manifests')
            for path in paths:
                self._require(path.stat().st_size <= self.MAX_MANIFEST_BYTES, 'manifest too large')
                m = self._json(path); step = m['stepIndex']
                self._require(type(step) is int and step in self.references and step not in self.manifests,
                    'invalid or duplicate manifest step')
                self._require(path.parent.name==f'robust-risk-step-{step:02d}', 'manifest folder mismatch')
                ref = self.references[step]
                self._require(m.get('schema')==1 and m.get('phase')=='transfer'
                    and m.get('stanceSide')==ref['stance_side']
                    and m.get('originalNativeWorldUnchanged') is True and m.get('physicsIntegrations')==0
                    and m.get('payloadEncoding')=='zlib-compressed exact lh_checkpoint_export payload',
                    'manifest protocol mismatch')
                for key, digest in self.bindings.items():
                    self._require(m.get(key)==digest, 'manifest '+key+' mismatch')
                samples = m.get('samples',[])
                self._require(isinstance(samples,list) and 21 <= len(samples) <= self.MAX_SAMPLES,
                    'manifest sample count outside bounded selection history')
                ids = [s['sourceStep'] for s in samples]
                self._require(all(type(i) is int and i>=0 for i in ids)
                    and all(b-a==4 for a,b in zip(ids,ids[1:])), 'manifest IDs not ordered 4-frame cadence')
                times = [float(s['sourceTimeS']) for s in samples]
                self._require(all(abs((b-a)-.005)<1e-9 for a,b in zip(times,times[1:]))
                    and .1-1e-9 <= times[-1]-times[0] <= .105+1e-9, 'manifest time window mismatch')
                for sample in samples:
                    sid=sample['sourceStep'];filename=sample.get('filename')
                    self._require(filename==f'source-{sid:06d}.native-payload.zlib', 'noncanonical payload filename')
                    self._require(type(sample.get('payloadBytes')) is int
                        and 0 < sample['payloadBytes'] <= self.MAX_RAW_BYTES, 'invalid payload size')
                    for key in ('payloadSHA256','compressedSHA256'):
                        self._require(self._digest(sample.get(key)), 'invalid '+key)
                    payload_path=path.parent/filename
                    self._require(payload_path.is_file() and not payload_path.is_symlink()
                        and 0 < payload_path.stat().st_size <= self.MAX_COMPRESSED_BYTES,
                        'missing or oversized compressed witness')
                    self.samples[(step,sid)] = (sample,payload_path)
                actual_files={p.name for p in path.parent.glob('*.native-payload.zlib')}
                self._require(actual_files=={s['filename'] for s in samples}, 'unexpected witness payload files')
                self.manifests[step], self.expected[step], self.seen[step] = m, ids, []
        except Exception as error:
            self.failures.append('initialization: '+str(error))

    @staticmethod
    def _require(condition, message):
        if not condition:
            raise ValueError(message)

    @staticmethod
    def _digest(value):
        return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)

    @staticmethod
    def _sha(path):
        import hashlib
        with open(path,'rb') as stream:
            return hashlib.file_digest(stream,'sha256').hexdigest()

    @staticmethod
    def _json(path):
        import json
        with open(path) as stream:
            return json.load(stream)

    @staticmethod
    def _array(value, shape):
        import numpy as np
        a=np.asarray(value,dtype=float)
        if a.shape != shape or not np.isfinite(a).all():
            raise ValueError('invalid finite array shape '+str(shape))
        return a.copy()

    @staticmethod
    def _equal(a, b, tolerance=1e-10):
        import numpy as np
        if isinstance(a,dict) or isinstance(b,dict):
            return isinstance(a,dict) and isinstance(b,dict) and a.keys()==b.keys() and all(
                ExactForecastReplay._equal(a[k],b[k],tolerance) for k in a)
        if isinstance(a,(list,tuple)) and len(a)==2 and isinstance(a[0],(list,tuple)) and len(a[0])==2:
            try:
                return len(b)==2 and all(ExactForecastReplay._equal(x,y,tolerance) for x,y in zip(a,b))
            except (TypeError,ValueError):
                return False
        try:
            x,y=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
            return x.shape==y.shape and bool(np.isfinite(x).all() and np.isfinite(y).all()
                and np.max(np.abs(x-y),initial=0.)<=tolerance)
        except (TypeError,ValueError):
            return a==b

    def _compare(self, name, actual, expected, shape, tolerance):
        import numpy as np
        a,b=self._array(actual,shape),self._array(expected,shape)
        error=float(np.max(np.abs(a-b),initial=0.))
        self.maximum_errors[name]=max(self.maximum_errors.get(name,0.),error)
        self._require(error<=tolerance,name+' differs by '+str(error))
        return a

    def _check_sources(self):
        hashes=self.receipt.get('codeSHA256AtStart',{})
        required={'source_step_probe.py','source_cadenced_ten_steps.py','source_upright_large_steps.py',
            'source_opposed_arm_swing.py','source_robust_support_steps.py','source_arm_knee_mobility.py'}
        self._require(isinstance(hashes,dict) and required<=hashes.keys(), 'missing frozen replay source hashes')
        self._require(self.receipt.get('codeUnchangedDuringRun') is True, 'run source stability not certified')
        for name,digest in hashes.items():
            self._require(name==__import__('pathlib').Path(name).name and self._digest(digest), 'unsafe source binding')
            current=self.source_dir/name
            good=current.is_file() and self._sha(current)==digest
            frozen=self.base/'frozen-source'/name
            if (self.base/'frozen-source').is_dir():
                good=good and frozen.is_file() and self._sha(frozen)==digest
            self.source_checks[name]=bool(good)
            self._require(good,'current replay source differs from frozen receipt: '+name)

    def _ensure_world(self):
        import ctypes as C
        from source_step_probe import World, PTR
        from source_cadenced_ten_steps import CadencedController
        if self.world is None:
            self._check_sources()
            self.world=World(self.session,self.helper,xml=self.xml)
            self.world.l.lh_motor_response.argtypes=[C.c_void_p,PTR,PTR]
            self.world.l.lh_motor_response.restype=None
            self.world.l.lh_self_separation.argtypes=[C.c_void_p,PTR]
            self.world.l.lh_self_separation.restype=None
            for symbol in ('lh_step','lh_step_audited'):
                if hasattr(self.world.l,symbol):
                    setattr(self.world.l,symbol,self._forbidden_step)
        return self.world

    @staticmethod
    def _forbidden_step(*args, **kwargs):
        raise RuntimeError('native integrations forbidden during forecast replay')

    def _payload(self, sample, path):
        import hashlib
        import zlib
        self._require(path.stat().st_size <= self.MAX_COMPRESSED_BYTES, 'compressed witness grew')
        compressed=path.read_bytes()
        self._require(hashlib.sha256(compressed).hexdigest()==sample['compressedSHA256'], 'compressed digest mismatch')
        if 'compressedBytes' in sample:
            self._require(len(compressed)==sample['compressedBytes'],'compressed length mismatch')
        decoder=zlib.decompressobj()
        raw=decoder.decompress(compressed,sample['payloadBytes']+1)
        self._require(len(raw)==sample['payloadBytes'] and decoder.eof
            and not decoder.unused_data and not decoder.unconsumed_tail, 'bounded raw payload size/stream mismatch')
        self._require(hashlib.sha256(raw).hexdigest()==sample['payloadSHA256'], 'raw payload digest mismatch')
        return raw

    def _probe(self, note, row, reference, forecast, w):
        import math
        import types
        import numpy as np
        from source_cadenced_ten_steps import CadencedController
        envelope=forecast.get('probeControllerInput',{})
        fields=envelope.get('fields',{})
        self._require(isinstance(fields,dict) and set(fields)<=self.FIELDS, 'unwhitelisted probe controller field')
        required=self.FIELDS-{'shear_weight'}
        self._require(required<=fields.keys(), 'missing required probe controller fields')
        self._require(envelope.get('phase')=='transfer' and note.get('phase')=='transfer'
            and envelope.get('coreEntryPoint')=='Controller.solve on copied CadencedController; existing arm targets already computed',
            'unexpected copied probe entry point')
        t=float(envelope['elapsedTransferS'])
        self._require(math.isfinite(t) and 0<=t<=160., 'invalid transfer probe time')
        self._require(abs(float(note['t'])-float(row['sourceStep'])*.00125)<1e-9,
            'probe is not aligned with preceding saved native row')
        # Transfer starts are bounded by the per-step reference and are checked
        # by the outer history replay. The selected instant binds this local t.
        step=note['stepIndex'];m=self.manifests[step]
        self._require(abs(t-(float(m['selectedAtTransferS'])
            -(m['samples'][-1]['sourceStep']-row['sourceStep'])*.00125))<1e-9, 'probe transfer clock mismatch')
        for name in self.STATIC:
            self._require(name in reference and self._equal(fields.get(name),reference[name]),
                'probe static reference mismatch: '+name)
        if 'shear_weight' in fields or 'shear_weight' in reference:
            self._require(self._equal(fields.get('shear_weight',.1),reference.get('shear_weight',.1)),
                'probe shear weight mismatch')
        arm=note.get('opposedArmSwing',{})
        for field,key in (('arm_posture','targetArmJointRad'),('arm_velocity','targetArmVelocityRadps'),
                ('arm_acceleration','targetArmAccelerationRadps2')):
            self._compare('bound '+field,fields.get(field),arm.get(key),(6,),1e-12)
        q0=self._array(fields['q0'],(21,));rq=self._array(reference['q0'],(21,))
        self._require(self._equal(q0[:15],rq[:15]) and self._equal(q0[15:],arm.get('targetArmJointRad')),
            'probe posture is not static legs plus actual arm target')
        self._require(self._equal(fields['leg_origin'],arm.get('legPitchOriginRad')), 'arm leg origin mismatch')
        ranges=self.arm_mapping.get('verifiedSwingDeltaRangeBySideRad')
        if ranges is None:
            ranges=np.repeat(self._array(self.arm_mapping.get('verifiedSwingDeltaRangeRad'),(2,))[None,:],2,axis=0)
        ranges=self._array(ranges,(2,2))
        self._require(self._equal(envelope.get('armMappingRanges'),ranges), 'probe arm mapping ranges mismatch')
        for field,key in (('arm_kp','positionGain'),('arm_kd','velocityGain'),('arm_weight','taskWeight')):
            self._require(self._equal(fields[field],self.arm_mapping.get(key)), 'probe '+field+' mapping mismatch')
        self._require(fields['unload_admitted'] is True and fields['self_avoidance'] is True,
            'probe is not the self-avoiding proposed unload')
        side=fields['swing_side'];stance=fields['stance_side']
        self._require(type(side) is int and side in (0,1) and stance==1-side
            and fields['swing_body']==w.ids['left_foot' if side==0 else 'right_foot'], 'probe side/body mismatch')
        floor=list(reference['floorTargets'])
        floor[side]=w.ground_height(np.asarray(reference['targets'][side][0]))+fields['radii'][side]+.0001
        self._require(self._equal(fields['floorTargets'],floor), 'probe prospective unload target mismatch')
        # finalCom is rewritten at measured loaded-settle handoff in the saved
        # per-step reference. Its historical transfer value is reconstructible.
        final=np.asarray(reference['finalCom'],dtype=float).copy()
        final[:2]=(np.asarray(reference['landing'])[:2]+np.asarray(reference['targets'][stance][0])[:2])/2.+.035*np.asarray(reference['heading'])[:2]
        self._require(self._equal(fields['finalCom'],final), 'probe original transfer finalCom mismatch')
        self._require(self._equal(fields['_seek_depth'],0.) and self._equal(fields['_seek_rate'],0.)
            and self._equal(fields['_seek_jet'],[0.,0.,0.])
            and abs(float(fields['_seek_time'])-(float(row['sourceTimeS'])-.005))<1e-8,
            'transfer contact seek history mismatch')
        expected_B=np.zeros((27,21));order=[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]
        for motor,joint in enumerate(order):expected_B[6+joint,motor]=w.gears[motor]
        self._require(self._equal(fields['B'],expected_B), 'probe motor map differs from native gears')
        p=CadencedController.__new__(CadencedController);p.w=w
        for name,value in fields.items():
            if name in self.SHAPES:
                value=self._array(value,self.SHAPES[name])
            elif name=='targets':
                self._require(isinstance(value,list) and len(value)==2,'invalid targets')
                value=[(self._array(v[0],(3,)),self._array(v[1],(3,3)))for v in value]
            elif name=='flatRotations':
                self._require(isinstance(value,list) and len(value)==2,'invalid flat rotations')
                value=[self._array(v,(3,3))for v in value]
            elif name=='phase_durations':
                self._require(isinstance(value,dict) and set(value)=={'transfer','lift','translate','land','settle'}
                    and all(type(v) in (int,float) and math.isfinite(v) and 0<v<=10 for v in value.values()),
                    'invalid phase durations')
                value=dict(value)
            elif type(value) not in (int,float,bool):
                raise ValueError('invalid scalar probe field '+name)
            elif not math.isfinite(float(value)):
                raise ValueError('nonfinite scalar probe field '+name)
            setattr(p,name,value)
        p.floorTargets=list(p.floorTargets)
        p._mapping=types.SimpleNamespace(ranges=ranges)
        return p,t

    def __call__(self, note, row, reference, forecast):
        import ctypes as C
        import hashlib
        import numpy as np
        self.calls+=1
        step=note.get('stepIndex');sid=row.get('sourceStep')
        result=dict(passed=False,nativeVerified=False,sampleStep=sid,stepIndex=step)
        if (step,sid) not in self.samples:
            self.unselected_calls+=1
            result['passed']=True
            return result
        try:
            from source_step_probe import Controller, ptr
            from source_arm_knee_mobility import native_bytes
            self._require(not self.failures,'replay has prior failures')
            self._check_sources()
            self._require(self._equal(reference,self.references[step]), 'reference object binding mismatch')
            self._require(sid==forecast.get('nativePayloadSourceStep'), 'forecast/native witness ID mismatch')
            expected=self.expected[step];seen=self.seen[step]
            self._require(len(seen)<len(expected) and sid==expected[len(seen)], 'selected witness replay out of order or duplicated')
            sample,path=self.samples[(step,sid)]
            self._require(sample['payloadSHA256']==forecast.get('nativePayloadSHA256'), 'forecast payload digest mismatch')
            w=self._ensure_world()
            payload=self._payload(sample,path)
            self._require(w.l.lh_checkpoint_size(w.h)==len(payload), 'native checkpoint size mismatch')
            buf=C.create_string_buffer(payload)
            self._require(w.l.lh_checkpoint_import(w.h,buf,len(payload))==len(payload), 'native import rejected exact payload')
            del buf
            self.imports+=1
            self._require(native_bytes(w)==payload, 'native import/export not byte exact')
            self._compare('qpos',w.arr(0,(28,)),row.get('qpos'),(28,),0.)
            self._compare('qvel',w.arr(1,(27,)),row.get('qvel'),(27,),0.)
            counter=w.l.lh_dim(w.h,6);now=w.l.lh_time(w.h)
            self._require(counter==sid and abs(now-float(row['sourceTimeS']))<1e-12
                and abs(now-float(sample['sourceTimeS']))<1e-12, 'imported native counter/time mismatch')
            probe,t=self._probe(note,row,reference,forecast,w)
            self._require(native_bytes(w)==payload and w.l.lh_dim(w.h,6)==counter and w.l.lh_time(w.h)==now,
                'probe construction changed native bytes/counter/time')
            proposed,info=Controller.solve(probe,t,'transfer');self.qp_calls+=1
            self._require(native_bytes(w)==payload and w.l.lh_dim(w.h,6)==counter and w.l.lh_time(w.h)==now,
                'pure copied-probe QP changed native bytes/counter/time')
            self._compare('proposed motor control',proposed,forecast.get('proposedMotorControl'),(21,),1e-9)
            pressures=np.asarray(info['predictedForcesN']).reshape(2,4,3)[probe.stance_side,:,2]
            self._compare('proposed normal pressure',pressures,forecast.get('virtualUnloadNormalForcesN'),(4,),1e-7)
            response=w.read('lh_motor_response',(33,),ptr(np.ascontiguousarray(proposed)));self.response_calls+=1
            self._require(native_bytes(w)==payload and w.l.lh_dim(w.h,6)==counter and w.l.lh_time(w.h)==now,
                'pure native proposed-response call changed native bytes/counter/time')
            self._compare('native proposed response',response,forecast.get('nativeProposedResponse'),(33,),1e-8)
            foot=w.feet()[probe.stance_side];body=foot['body'];corners=np.asarray(foot['points']).copy();corners[:,2]-=foot['radius']
            clearance=np.array([p[2]-w.ground_height(p)for p in corners])
            count=w.l.lh_support_count(w.h);supports=w.arr(21,(128,))[:2*count].reshape(-1,2)
            for p in corners:
                if len(supports):
                    j=int(np.argmin(np.linalg.norm(supports-p[:2],axis=1)))
                    if np.linalg.norm(supports[j]-p[:2])<=.001:p[:2]=supports[j]
            self._compare('current matched support',corners,forecast.get('currentMatchedSupportCoordinatesSource'),(4,3),1e-10)
            self._compare('current endpoint clearance',clearance,forecast.get('currentEndpointClearanceM'),(4,),1e-10)
            v=w.arr(1,(27,));Jbody=w.read('lh_body_jac',(6,27),body);jet=w.read('lh_jdot',(6,),body)
            omega=Jbody[3:]@v;origin=w.arr(3,(17,3))[body];rates=[];accelerations=[]
            for point in corners:
                J=w.read('lh_point_jac',(3,27),body,ptr(np.ascontiguousarray(point)))
                r=point-origin;bias=jet[:3]+np.cross(jet[3:],r)+np.cross(omega,np.cross(omega,r))
                rates.append(float(J[2]@v));accelerations.append(float(J[2]@response[:27]+bias[2]))
            self._compare('endpoint vertical velocity',rates,forecast.get('nativeUnloadEndpointVerticalVelocityMps'),(4,),1e-10)
            self._compare('endpoint vertical acceleration',accelerations,forecast.get('nativeUnloadEndpointVerticalAccelerationMps2'),(4,),1e-8)
            predicted=clearance+.1*np.asarray(rates)+.005*np.asarray(accelerations)
            self._compare('local clearance forecast',predicted,forecast.get('localClearanceForecastM'),(4,),1e-10)
            self._require(forecast.get('predictionHorizonS')==.1
                and forecast.get('pressureRiskCorner')==int(np.argmin(pressures))
                and forecast.get('clearanceRiskCorner')==int(np.argmax(predicted)), 'independent forecast risk mismatch')
            self._require(forecast.get('proposedControlApplied') is False and forecast.get('physicsRolloutIntegrations')==0
                and forecast.get('nativeBytesAndCounterUnchanged') is True
                and abs(float(forecast['proposedMotorMaximumFraction'])-float(np.max(np.abs(proposed))))<=1e-10,
                'forecast pure-call protocol mismatch')
            self._require(native_bytes(w)==payload and w.l.lh_dim(w.h,6)==counter and w.l.lh_time(w.h)==now,
                'complete replay changed exact native bytes/counter/time')
            del payload
            seen.append(sid)
            result.update(passed=True,nativeVerified=True,nativeBytesCounterTimeUnchanged=True,
                proposedQPSolved=True,pureMotorResponseVerified=True,physicsIntegrations=0)
        except Exception as error:
            message='step '+str(step)+' source '+str(sid)+': '+str(error)
            self.failures.append(message);result['error']=message
        self.results.append(result)
        return result

    def coverage(self, step, ids):
        try:
            good=(not self.failures and type(step) is int and step in self.expected
                and list(ids)==self.expected[step] and self.seen[step]==self.expected[step])
            if good:self.covered.add(step)
            return bool(good)
        except (TypeError,ValueError):
            return False

    def summary(self):
        try:
            self._check_sources()
        except Exception as error:
            message='final source binding: '+str(error)
            if message not in self.failures:self.failures.append(message)
        expected=sum(len(v)for v in self.expected.values());verified=sum(len(v)for v in self.seen.values())
        complete=bool(self.manifests and set(self.manifests)==self.covered and expected==verified)
        return dict(passed=not self.failures and complete,physicsIntegrations=0,
            exactNativeWitnessReplay=True,oneIsolatedScratchWorld=True,retainedRawPayloads=0,
            selectedManifestCount=len(self.manifests),expectedSelectedWitnesses=expected,
            nativeVerifiedWitnesses=verified,checkpointImports=self.imports,proposedQPSolves=self.qp_calls,
            pureMotorResponseCalls=self.response_calls,forecastCalls=self.calls,nonselectedForecastCalls=self.unselected_calls,
            selectedCoverageComplete=complete,coveredSteps=sorted(self.covered),
            manifestNativeSteps={str(k):v for k,v in self.expected.items()},
            verifiedNativeSteps={str(k):v for k,v in self.seen.items()},sourceHashesMatchFrozenReceipt=self.source_checks,
            maximumAbsoluteErrors=self.maximum_errors,failures=self.failures,results=self.results)

    def close(self):
        if self.world is not None:
            self.world.close();self.world=None


def audit(folder):
    folder = Path(folder);base = folder.parents[1]
    read = lambda path:json.loads(path.read_text())
    cadence,initial,summary,receipt = [read(folder/n) for n in ('cadenced-step-config.json','summary.json','multistep-summary.json','opposed-arm-run.json')]
    references = {s['index']:read(folder/f"controller-step-{s['index']:02d}.json") for s in summary['steps']}
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    prospective = cadence.get('robustSupportAdmission',{}).get('riskSignal') == FORECAST_RISK
    native = ExactForecastReplay(base,folder,references,receipt,read(folder/'arm-mapping.json')) if prospective else None
    try:
        with (folder/'frames.ndjson').open() as source:
            replay = audit_replay((json.loads(line) for line in source),initial['entry'],references,cadence,
                json_array_items(folder/'control.json'),model['totalMassKg'],FootEndpointFK(),
                prospective=prospective,forecast_replayer=native)
        native_proof = native.summary() if native is not None else None
    finally:
        if native is not None:native.close()
    original = audit_cadence(folder)
    proof = audit_preparation(base,initial,expected_ratio=8.)
    final = cache_proof((folder/'final.bin').read_bytes(),proof['alteredModelSHA256'],
        file_sha(base/'model-session/source_bridge_v2.so'),file_sha(base/'model-session/dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))
    proof['finalCheckpoint'] = final
    proof['checks']['finalCheckpointBindingPayloadAndNativeCount'] = final['passed'] and final['nativeCounter'] == summary['actualIntegrations']
    proof['passed'] = all(proof['checks'].values())
    controls = audit_trial_controls(cadence,references,read(folder/'arm-mapping.json'),
        json_array_items(folder/'control.json'),summary['actualIntegrations'],expected_ratio=8.)
    # The contact comparison intentionally hard-codes ten. Keep its result
    # unchanged and replace only that count-policy check for an explicit pilot.
    pilot_controls = (cadence['requestedSteps'] in (1,10) and cadence['requestedDistanceM'] == .18
        and cadence.get('loadedSettleHandoff') is True and cadence.get('continuousLoadedHandoff') is False
        and cadence.get('intermediateStableRequiredS') == 0. and cadence.get('initialTransferS') == 4.
        and cadence.get('phaseDurationsS') == dict(transfer=2.,lift=.8,translate=1.2,land=1.,settle=1.52)
        and cadence.get('originalContactJointMotorCollisionCaptureGates') is True
        and cadence.get('simultaneousArmCoordination',{}).get('enabled') is False)
    controls_pass = pilot_controls and all(v for k,v in controls['checks'].items() if k != 'unchangedCadenceControls')
    required_sources = {'source_robust_support_steps.py','source_step_probe.py','source_multistep_probe.py',
        'source_contact_impedance_probe.py','source_cadenced_ten_steps.py','source_selected_step.py','source_opposed_arm_swing.py'}
    hashes = receipt.get('codeSHA256AtStart',{});source_checks = {};paths = {}
    for name,expected in hashes.items():
        path = base/'frozen-source'/name
        if not path.is_file():path = Path(__file__).with_name(name)
        source_checks[name] = path.is_file() and file_sha(path) == expected;paths[name] = path
    source_checks['requiredRobustAndCoreSourcesPresent'] = required_sources <= set(hashes)
    for name,attributes in {'source_step_probe.py':{'adjust_com_reference'},
            'source_multistep_probe.py':{'transfer_ready_goal','robust_support_ready'},
            'source_robust_support_steps.py':{'adjust_com_reference','robust_support_ready'}}.items():
        path = paths.get(name)
        tree = ast.parse(path.read_text()) if path and path.is_file() else ast.Module(body=[],type_ignores=[])
        found = {node.attr for node in ast.walk(tree) if isinstance(node,ast.Attribute)}|{node.name for node in ast.walk(tree) if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))}
        source_checks[name+'RequiredOptInHooks'] = attributes <= found
    source_checks['codeUnchangedDuringRun'] = receipt.get('codeUnchangedDuringRun') is True
    result = (qualify_prospective(original,replay,proof,controls_pass,all(source_checks.values()),cadence['requestedSteps'],native_proof)
        if prospective else qualify_robust(original,replay,proof,controls_pass,all(source_checks.values()),cadence['requestedSteps']))
    result.update(schema=1,savedDataOnly=True,physicsIntegrations=0,requestedSteps=cadence['requestedSteps'],
        originalModelQualificationClaim=False,robustReplay=replay,explicitModelProof=proof,
        unchangedContactControlAudit=controls,explicitPilotControlPolicyPass=pilot_controls,
        sourceProvenance=source_checks,originalStrictAudit=original,auditorSHA256=file_sha(Path(__file__)),
        prospectiveUnloadRiskProfile=prospective,exactNativeForecastReplay=native_proof,
        hashes=original['hashes'],evidenceLimit='Saved-data continuity and hashed source receipts cannot prove absence of every possible unrecorded external intervention.')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path,required=True);args = p.parse_args()
    result = audit(args.folder);args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    passed = any(result.get(key,False) for key in ('explicitRobustSupportImpratio8PilotPass',
        'explicitRobustSupportImpratio8TenStepPass','explicitProspectiveUnloadRiskImpratio8PilotPass',
        'explicitProspectiveUnloadRiskImpratio8TenStepPass'))
    print(json.dumps(dict(passed=passed,failedChecks=result.get('prospectiveFailedChecks',result['failedChecks'])),indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':main()
