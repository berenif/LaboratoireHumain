#!/usr/bin/env python3
"""Read-only contact chronology and numeric admission for a future hybrid recovery."""
import argparse
import hashlib
import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO / 'evidence/recovery-trajectory-preparation-2026-10-11/reference/source-contact-intervals.json'
READY = REPO / 'evidence/recovery-coordinated-retarget-2026-10-11/reference-ready.json'
READY_GEOMETRY = REPO / 'evidence/recovery-coordinated-retarget-2026-10-11/ready-geometry.json'
HAND = REPO / 'evidence/hand-support-2026-10-11/corridor-evaluated.json'
HAND_RESULT = REPO / 'evidence/hand-support-2026-10-11/native-ab/analysis.json'
HAND_STATIC = REPO / 'evidence/hand-support-2026-10-11/terminal-statics.json'
MODEL = REPO / 'rust/crates/model/data/canonical-v1.json'
FIXTURES = REPO / 'rust/crates/sim/data/recovery-v1.json'
CONTRACT = REPO / 'rust/crates/sim/data/recovery-contract-v1.json'
EXPECTED_PHASES = ['prepared-descent', 'loaded-brace', 'plant-lead', 'plant-trailing',
                   'release-hands', 'rise-and-stand', 'original-handoff-and-hold']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_plan(plan, model, contract):
    segments = {s['id'] for s in model['segments']}
    assert plan['schema'] == 1
    assert plan['sourceFixture'] == 'landed-crouch-left'
    assert plan['clock'] == 'single continuous original native clock at 240 Hz'
    assert plan['maximumRecoveryTimeS'] == contract['acceptance']['recoveryLimitS'] == 25
    assert plan['finalObservationTimeS'] == 26
    phases = plan['phases']
    assert [s['id'] for s in phases] == EXPECTED_PHASES
    assert [s['latestS'] for s in phases] == sorted(s['latestS'] for s in phases)
    assert phases[-1]['latestS'] == 25
    assert all(set(s['intent']) <= segments and s['intent'] for s in phases)
    assert all(set(s.get('release', [])) <= segments for s in phases)
    assert set(phases[2]['release']) == {'leftShin', 'leftFoot', 'leftForefoot'}
    assert set(phases[3]['release']) == {'rightShin', 'rightFoot', 'rightForefoot'}
    assert set(phases[4]['release']) == {'leftHand', 'rightHand'}
    assert all(s['requiredForExit'] for s in phases)
    return phases


def check_keys(program, model):
    ids = [s['id'] for s in model['segments']]
    if program.get('segmentIds') != ids or len(program.get('keyframes', [])) < 2:
        return ['wrong topology or fewer than two numeric keys']
    failures = []
    previous = -1
    previous_frame = None
    for frame in program['keyframes']:
        step = frame['substep']
        if not isinstance(step, int) or step <= previous or step > 25*240:
            failures.append('nonmonotone or out-of-window numeric key')
        for field in ['positions', 'velocities']:
            if len(frame[field]) != 25 or any(len(row) != 3 for row in frame[field]):
                return ['numeric coordinate shape differs from 25 × 3']
        for i, part in enumerate(model['segments']):
            limits = {a['coordinate']:a for a in (part.get('jointProfile') or {}).get('axes', [])}
            for k, axis in enumerate('xyz'):
                q = frame['positions'][i][k]
                v = frame['velocities'][i][k]
                if not (math.isfinite(q) and math.isfinite(v)) or abs(v) > 1:
                    failures.append(f'nonfinite/excess target speed at {step}: {part["id"]}.{axis}')
                elif axis in limits and not limits[axis]['minRadians'] <= q <= limits[axis]['maxRadians']:
                    failures.append(f'anatomical target outside native range at {step}: {part["id"]}.{axis}')
                elif axis not in limits and (q != 0 or v != 0):
                    failures.append(f'unmodeled/root command at {step}: {part["id"]}.{axis}')
                if previous_frame is not None and step > previous:
                    dt = (step - previous) / 240
                    if abs(q - previous_frame['positions'][i][k]) > dt:
                        failures.append(f'key target slew exceeds 1 rad/s at {step}: {part["id"]}.{axis}')
        previous_frame = frame
        previous = step
    if previous < 8*240:
        failures.append('numeric program ends before native minimum duration')
    if any(v != 0 for row in program['keyframes'][0]['velocities'] for v in row):
        failures.append('initial native velocity target is not at rest')
    if any(v != 0 for row in program['keyframes'][-1]['velocities'] for v in row):
        failures.append('terminal native velocity target is not at rest')
    return sorted(set(failures))


def inspect(plan):
    model = json.loads(MODEL.read_text())
    contract = json.loads(CONTRACT.read_text())
    phases = verify_plan(plan, model, contract)
    source = json.loads(SOURCE.read_text())
    ready = json.loads(READY.read_text())
    ready_geometry = json.loads(READY_GEOMETRY.read_text())
    corridor = json.loads(HAND.read_text())
    native = json.loads(HAND_RESULT.read_text())['cases']['b-prepared-arms']
    static = json.loads(HAND_STATIC.read_text())
    fixture = next(x for x in json.loads(FIXTURES.read_text())['cases'] if x['id'] == plan['sourceFixture'])
    first_source = {side: min(e['solveStartTimeS'] for e in source['events']
                              if e['sourceBody'] == f'{side}_hand') for side in ['left', 'right']}
    first_intent = {side: min(c['startSubstep']/240 for c in ready['contacts']
                              if c['segment'] == f'{side}Hand') for side in ['left', 'right']}
    terminal = corridor['rows'][-1]['coordinates']
    modeled = [(i,k) for i,s in enumerate(model['segments']) for k,c in enumerate('xyz')
               if any(a['coordinate'] == c for a in (s.get('jointProfile') or {}).get('axes', []))]
    distances = [(max(abs(terminal[i][k] - key['positions'][i][k]) for i,k in modeled),
                  key['substep']/240) for key in ready['keyframes']]
    gap, closest_time = min(distances)
    early_gap, early_time = min((d,t) for d,t in distances if 2 <= t <= 5)
    closest_geometry = min(ready_geometry['frames'], key=lambda f:abs(f['substep']/240-early_time))
    a = corridor['rows'][-1]['poses'][0]
    b = closest_geometry['poses'][0]
    root_gap = math.sqrt(sum((a['position'][k]-b['position'][k])**2 for k in 'xyz'))
    dot = abs(sum(a['rotation'][k]*b['rotation'][k] for k in 'xyzw'))
    root_angle = 2*math.acos(min(1,dot))
    numerical = plan.get('motorProgram')
    failures = []
    if numerical is None:
        failures.append('no complete 25-body numeric motor trajectory from the actual crouch')
    else:
        path = REPO/numerical
        if not path.is_file():
            failures.append('declared motor program file missing')
        else:
            failures += check_keys(json.loads(path.read_text()), model)
    if plan.get('geometryFrames') is None:
        failures.append('no full-path native 0°/60° geometry frames for this phase sequence')
    failures.append('no prospective pre-impact velocity/angular-momentum and support proof')
    failures.append('no static force/couple/friction certificate for every transfer boundary')
    return {
        'scope': 'read-only documentary/numeric prescreen; no native world stepped',
        'physicalIntegrations': 0, 'staticWorldInspections': 0,
        'inputsSha256': {str(p.relative_to(REPO)):sha(p) for p in
                         [SOURCE,READY,READY_GEOMETRY,HAND,HAND_RESULT,HAND_STATIC,MODEL,FIXTURES,CONTRACT]},
        'sourceInitialContacts': {'handFirstSolvedS':first_source,
                                  'rightThighLoadedAtStart':any(e['sourceBody']=='right_thigh' and e['solveStartTimeS']==0
                                                                  for e in source['events'])},
        'nativeReference': {'firstHandIntentS':first_intent,
                            'declaredShinOrThighIntentCount':sum(c['segment'].endswith(('Shin','Thigh'))
                                                                 for c in ready['contacts']),
                            'actualPreparedHandsBilateralS':native['firstBothHands']['timeS'],
                            'actualPreparedHeadContactS':native['events']['head']['firstInstantaneous3NTimeS'],
                            'intermediateSupportPassed':native['intermediateSupportPassed']},
        'archivedDescentEndpointStatics': {'success':static['success'],
                                            'aggregateCapNm':static['aggregateCapNm'],
                                            'aggregateTorqueNm':static['aggregateTorqueNm'],
                                            'peakAnatomicalUtilization':static['peakAnatomicalUtilization'],
                                            'scope':'one old terminal pose only; no dynamic or full-path certificate'},
        'initialRootMismatch': {'nativeCrouchPelvisHeightM':fixture['poses'][0]['position']['y'],
                                'sourcePelvisHeightM':0.0993,
                                'sourceValueOrigin':'documented source export; approximate'},
        'naiveStitch': {'handCorridorEndS':corridor['rows'][-1]['timeS'],
                        'minimumMaximumModeledJointGapRad':gap,
                        'closestReadyReferenceTimeS':closest_time,
                        'minimumTransitionTimeAtUnchangedOneRadPerS':gap,
                        'earlyReferenceWindowS':[2,5],
                        'earlyWindowJointGapRad':early_gap,
                        'earlyWindowClosestReferenceTimeS':early_time,
                        'earlyWindowMinimumTransitionSAtOneRadPerS':early_gap,
                        'closestGeometryTimeS':closest_geometry['substep']/240,
                        'pelvisPositionGapM':root_gap,
                        'pelvisOrientationGapRad':root_angle,
                        'meaning':'lower bound only; no root/contact/geometry or dynamic feasibility'},
        'chosenPhases':[s['id'] for s in phases],
        'admissionFailures':failures,
        'completeNumericTrajectoryReady':False,
        'newNativeHistoryAllowed':False,
    }


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--plan',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True)
    args = a.parse_args()
    assert not args.output.exists(), 'Refuse overwriting a prep receipt'
    report = inspect(json.loads(args.plan.read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['physicalIntegrations','nativeReference','naiveStitch',
                                             'admissionFailures','completeNumericTrajectoryReady']},indent=2))


if __name__ == '__main__':
    main()
