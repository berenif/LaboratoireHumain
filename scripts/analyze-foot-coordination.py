#!/usr/bin/env python3
"""Summarize archived B/C rows and solver records; never advance a world."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import statistics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, required=True)
args = parser.parse_args()
root = args.root
names = ['000-witness-B', '001-coordinated-C']
segments = json.loads((Path(__file__).resolve().parents[1]/'rust/crates/model/data/canonical-v1.json').read_text())['segments']
ids = [s['id'] for s in segments]
motor_ids = {'leftThigh', 'leftShin', 'leftAnkle', 'leftForefoot',
             'rightThigh', 'rightShin', 'rightAnkle', 'rightForefoot'}


def summary(path):
    report = json.loads((path/'report.json').read_text())
    rows = [json.loads(line) for line in (path/'substeps.jsonl').open()]
    trace = defaultdict(list)
    for line in (path/'solver-boundaries.jsonl').open():
        record = json.loads(line)
        trace[record['native_substep']].append(record)
    assert max(trace) == report['physicalSubsteps']
    assert all(r['substep'] in trace for r in rows)
    by_id = defaultdict(lambda: {'positiveSteps': [], 'persistentThreeNewtonSteps': [],
                                 'posteriorSteps': [], 'posteriorThreeNewtonSteps': [],
                                 'posteriorImpulseNms': 0.0, 'positiveImpulseNms': 0.0,
                                 'minFriction': 1000.0, 'maxFriction': 0.0,
                                 'maxTangentSpeedAfterMps': 0.0,
                                 'maxCoulombUsage': 0.0})
    for substep, records in trace.items():
        current = defaultdict(lambda: {'all': 0.0, 'posterior': 0.0})
        for record in records:
            dt = record['dt_s']
            for floor in record['floors']:
                index = floor['collider']['index']-1
                assert 0 <= index < len(ids)
                name = ids[index]
                for point in floor['points']:
                    impulse = max(0.0, point['normal_impulse_ns'])
                    if impulse <= 0:
                        continue
                    current[name]['all'] += impulse
                    posterior = name.endswith('Foot') and point['body_local_point'][2] < -0.02
                    if posterior:
                        current[name]['posterior'] += impulse
                        by_id[name]['posteriorImpulseNms'] += impulse
                    by_id[name]['positiveImpulseNms'] += impulse
                    by_id[name]['minFriction'] = min(by_id[name]['minFriction'], point['friction'])
                    by_id[name]['maxFriction'] = max(by_id[name]['maxFriction'], point['friction'])
                    by_id[name]['maxTangentSpeedAfterMps'] = max(
                        by_id[name]['maxTangentSpeedAfterMps'], point['tangent_speed_after_solve_mps'])
                    tangent = sum(x*x for x in point['tangent_impulse_world_ns'])**.5
                    if point['friction']*impulse > 0:
                        by_id[name]['maxCoulombUsage'] = max(
                            by_id[name]['maxCoulombUsage'], tangent/(point['friction']*impulse))
        for name, values in current.items():
            if values['all'] > 0:
                by_id[name]['positiveSteps'].append(substep)
            if values['all'] >= 3/240:
                by_id[name]['persistentThreeNewtonSteps'].append(substep)
            if values['posterior'] > 0:
                by_id[name]['posteriorSteps'].append(substep)
            if values['posterior'] >= 3/240:
                by_id[name]['posteriorThreeNewtonSteps'].append(substep)
    def intervals(steps):
        if not steps:
            return []
        steps = sorted(set(steps))
        result = []
        start = end = steps[0]
        for value in steps[1:]:
            if value == end+1:
                end = value
            else:
                result.append([round(start/240, 4), round(end/240, 4), end-start+1])
                start = end = value
        result.append([round(start/240, 4), round(end/240, 4), end-start+1])
        return result
    floor = {}
    for name in ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot', 'leftHand', 'rightHand']:
        v = by_id[name]
        floor[name] = {key: {'firstS': round(value[0]/240, 4) if value else None,
                             'lastS': round(value[-1]/240, 4) if value else None,
                             'stepCount': len(value),
                             'longestContinuousSteps': max((i[2] for i in intervals(value)), default=0),
                             'intervals': intervals(value)[:12]}
                       for key, value in v.items() if key.endswith('Steps')}
        floor[name].update({key: value for key, value in v.items() if not key.endswith('Steps')})
    motor = defaultdict(lambda: {'requested': [], 'deliveredAbs': [], 'saturation': [], 'caps': []})
    for r in rows:
        requested = {(a['id'], a['coordinate']): a for a in r['command'].get('ownerMotorRequests', [])}
        for part in r['nativeMotorDelivery']:
            if part['id'] not in motor_ids:
                continue
            for a in part['axes']:
                if a['coordinate'] != 'x':
                    continue
                key = part['id']+' x'
                m = motor[key]
                request = requested.get((part['id'], 'x'))
                if request:
                    m['requested'].append(request['ownerSpringRequestNm'])
                m['deliveredAbs'].append(a['absoluteImpulseNms']*240)
                m['saturation'].append(a['nativeScalarCapRatio'])
                m['caps'].append(a['installedCapNm'])
    motor = {k: {'maxRequestNm': max(map(abs, v['requested']), default=None),
                 'maxDeliveredAbsoluteNm': max(v['deliveredAbs'], default=None),
                 'maxNativeScalarCapRatio': max(v['saturation'], default=None),
                 'stepsAt95PercentNativeScalarCap': sum(x >= .95 for x in v['saturation']),
                 'maxInstalledCapNm': max(v['caps'], default=None)} for k, v in motor.items()}
    windows = {}
    for start, end in [(0, .1), (.1, 1), (1, 3), (3, 5), (5, report['physicalSeconds'])]:
        samples = [r for r in rows if start <= r['timeS'] < end]
        if not samples:
            continue
        windows[f'{start:g}-{end:.3f}s'] = {
            'n': len(samples),
            'meanForwardAccelerationMps2': statistics.fmean(r['acceleration']['z'] for r in samples),
            'firstComZM': samples[0]['com']['z'], 'lastComZM': samples[-1]['com']['z'],
            'firstComForwardVelocityMps': samples[0]['velocity']['z'],
            'lastComForwardVelocityMps': samples[-1]['velocity']['z'],
            'meanRearFootLoadN': statistics.fmean(r['loadsN'][18]+r['loadsN'][23] for r in samples),
            'meanForefootLoadN': statistics.fmean(r['loadsN'][19]+r['loadsN'][24] for r in samples),
        }
    milestones = {}
    for t in [.05, .1, .5, 1, 2, 3, 5, report['physicalSeconds']]:
        if t > report['physicalSeconds']:
            continue
        r = min(rows, key=lambda x: abs(x['timeS']-t))
        foot = {f['id']: {'posteriorHeightM': f['posteriorMinimumHeightM'],
                          'pitchRad': f['worldPitchRad']} for f in r['footGeometry']}
        milestones[f'{t:.4f}'] = {'recordedAtS': r['timeS'], 'pelvisHeightM': r['heightM'],
                                  'comZ': r['com']['z'], 'comVZ': r['velocity']['z'],
                                  'comAZ': r['acceleration']['z'],
                                  'rearFootLoadN': r['loadsN'][18]+r['loadsN'][23],
                                  'forefootLoadN': r['loadsN'][19]+r['loadsN'][24],
                                  'heel': foot, 'phase': (r['terminalStatus'] or {}).get('phase')}
    return {'physicalSubsteps': report['physicalSubsteps'], 'validRows': len(rows),
            'physicalSeconds': report['physicalSeconds'], 'firstRefusal': report['firstRefusal'],
            'fullRecoverySuccess': report['fullRecoverySuccess'],
            'originalRecoveries': report['originalRecoveries'],
            'maximumActualAnatomicalRatio': report['maximumActualAnatomicalRatio'],
            'maximumComAccelerationMps2': report['maximumComAccelerationMps2'],
            'floor': floor, 'motors': motor, 'windows': windows, 'milestones': milestones}


result = {name: summary(root/name) for name in names}
a = [json.loads(l) for l in (root/names[0]/'substeps.jsonl').open()]
b = [json.loads(l) for l in (root/names[1]/'substeps.jsonl').open()]
result['comparison'] = {
    'firstPhysicalHashDivergenceS': next((x['timeS'] for x, y in zip(a,b)
                                          if any(x[k] != y[k] for k in ['bodiesHash','motorsHash','contactsHash'])), None),
    'bothFullRecovery': [result[n]['fullRecoverySuccess'] for n in names],
    'nativeHistoriesExecuted': 2, 'additionalPhysicsIntegrationsForAnalysis': 0,
}
out = root/'analysis.json'
out.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({'comparison': result['comparison'], 'terminal': {n:result[n]['firstRefusal'] for n in names},
                  'rearFootPosterior': {n:{side:result[n]['floor'][side+'Foot']['posteriorThreeNewtonSteps']
                                            for side in ['left','right']} for n in names}}, indent=2))
