#!/usr/bin/env python3
"""Analyze existing CCD journals without integrating any physics.

Never declares P1 or physical recovery passed. It checks observation neutrality
and quantifies how last-interval contact reads omit earlier applied impulses.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path


def rows(path):
    with path.open() as file:
        return [json.loads(line) for line in file]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(enabled, disabled, historical=None):
    numeric = rows(enabled / 'substeps.jsonl')
    off = rows(disabled / 'substeps.jsonl')
    journal = rows(enabled / 'solver-boundaries.jsonl')
    grouped = defaultdict(list)
    for row in journal:
        grouped[row['native_substep']].append(row)
    report = json.loads((enabled / 'report.json').read_text())
    off_report = json.loads((disabled / 'report.json').read_text())
    neutrality = {'sameNumericRows': numeric == off,
                  'sameInitial': report['initial'] == off_report['initial'],
                  'sameFinal': report['final'] == off_report['final'],
                  'sameRefusal': report['firstRefusal'] == off_report['firstRefusal'],
                  'comparedValidNativeSteps': len(numeric)}
    if historical:
        old = rows(historical / 'substeps.jsonl')
        old_report = json.loads((historical / 'report.json').read_text())
        neutrality.update(sameHistoricalNumericRows=numeric == old,
                          sameHistoricalInitial=report['initial'] == old_report['initial'],
                          sameHistoricalFinal=report['final'] == old_report['final'],
                          sameHistoricalRefusal=report['firstRefusal'] == old_report['firstRefusal'])
    assert set(grouped) == set(range(1, report['physicalSubsteps']+1)), 'Missing integrated boundary'
    for intervals in grouped.values():
        assert [r['ccd_interval'] for r in intervals] == list(range(len(intervals))), 'Duplicate or reordered CCD interval'
        assert all(r['dt_s'] > 0 for r in intervals), 'Invalid interval duration'
        assert all(a['momentum_after_ns'] == b['momentum_before_ns'] for a, b in zip(intervals, intervals[1:])), 'Momentum changed outside recorded solve'
    final_hashes_present = 'finalWorldHashes' in report and 'finalWorldHashes' in off_report
    if final_hashes_present:
        neutrality['sameFinalWorldHashesIncludingRefusal'] = report['finalWorldHashes'] == off_report['finalWorldHashes']
    aggregate = []
    for native in numeric:
        intervals = grouped[native['substep']]
        dt = sum(row['dt_s'] for row in intervals)
        impulse = [sum(row['floor_impulse_ns'][k] for row in intervals) for k in range(3)]
        residual = [sum(row['linear_residual_ns'][k] for row in intervals) for k in range(3)]
        required = [(intervals[-1]['momentum_after_ns'][k]-intervals[0]['momentum_before_ns'][k]
                     -sum(row['gravity_and_user_impulse_ns'][k] for row in intervals))/dt
                    for k in range(3)]
        aggregate.append({'nativeSubstep': native['substep'], 'timeS': native['timeS'],
                          'ccdIntervals': len(intervals), 'dtS': dt,
                          'requiredFloorN': required, 'allIntervalsFloorN': [v/dt for v in impulse],
                          'legacyReportedVerticalN': sum(native['loadsN']),
                          'sumLinearResidualNs': residual})
    worst = max(aggregate, key=lambda r: abs(r['requiredFloorN'][1]-r['legacyReportedVerticalN']))
    per_axis = [max(abs(row['linear_residual_ns'][k]) for row in journal) for k in range(3)]
    summary = {'scope': 'Read-only CCD impulse boundary diagnosis; no planner or recovery qualification',
               'p1Passed': False, 'roundoffBoundCertified': False, 'angularBalanceVerified': False,
               'neutrality': neutrality, 'finalWorldHashesCompared': final_hashes_present, 'nativeStepsIncludingRefusal': report['physicalSubsteps'],
               'solverIntervalsIncludingRefusal': len(journal),
               'multipleIntervalNativeStepsIncludingRefusal': sum(len(v)>1 for v in grouped.values()),
               'maxIntervalsPerNativeStep': max(map(len, grouped.values())),
               'observationErrorCount': sum(row['observation_error'] is not None for row in journal),
               'maximumIntervalLinearResidualNsByAxis': per_axis,
               'maximumSummedNativeLinearResidualNsByAxis': [max(abs(r['sumLinearResidualNs'][k]) for r in aggregate) for k in range(3)],
               'worstLegacyVerticalDiscrepancy': worst,
               'fullRecoverySuccess': report['fullRecoverySuccess'], 'firstRefusal': report['firstRefusal'],
               'hashes': {str(path): digest(path) for path in [enabled/'solver-boundaries.jsonl', enabled/'substeps.jsonl', disabled/'substeps.jsonl']}}
    assert all(v for k, v in neutrality.items() if k != 'comparedValidNativeSteps'), 'Observation changed physics'
    assert summary['observationErrorCount'] == 0, 'Invalid contact observation'
    return summary, aggregate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', required=True, type=Path)
    parser.add_argument('--disabled', required=True, type=Path)
    parser.add_argument('--historical', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--require-final-hashes', action='store_true')
    args = parser.parse_args()
    summary, aggregate = analyze(args.enabled, args.disabled, args.historical)
    if args.require_final_hashes:
        assert summary['finalWorldHashesCompared'], 'Final integrated state hashes missing'
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (args.out/'aggregates.jsonl').open('w') as file:
        for row in aggregate:
            file.write(json.dumps(row)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
