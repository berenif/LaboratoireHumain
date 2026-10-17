#!/usr/bin/env python3
"""Exact IIS certificates/witnesses from frozen coefficients; no model/FK/physics."""
import argparse
from fractions import Fraction as F
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
EPS = F.from_float(1e-8)


def fraction(value):
    return F.from_float(float(value))


def encode(value):
    return {'numerator': str(value.numerator), 'denominator': str(value.denominator),
            'approximate': float(value)}


def decode(value):
    return F(int(value['numerator']), int(value['denominator']))


def linear_solve(matrix, rhs):
    """Small exact Gaussian elimination, not dynamics or an optimization solver."""
    n = len(rhs)
    assert len(matrix) == n and all(len(row) == n for row in matrix)
    work = [row[:] + [value] for row, value in zip(matrix, rhs)]
    for col in range(n):
        pivot = next((row for row in range(col, n) if work[row][col]), None)
        if pivot is None:
            raise ValueError('Removal system singular: minimality not proved')
        work[col], work[pivot] = work[pivot], work[col]
        divisor = work[col][col]
        work[col] = [v/divisor for v in work[col]]
        for row in range(n):
            if row != col:
                factor = work[row][col]
                work[row] = [a-factor*b for a, b in zip(work[row], work[col])]
    return [row[-1] for row in work]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def build_iis(case, side):
    p = case['problem']
    lower, upper = map(lambda xs: list(map(fraction, xs)), (p['lower'], p['upper']))
    assert len(lower) == len(upper) == 46
    assert all(-1 <= lo <= hi <= 1 for lo, hi in zip(lower, upper))
    tx, ty, tz = p['hard'][side*3:side*3+3]
    x, y, z = [list(map(fraction, task['row'])) for task in (tx, ty, tz)]
    # Cancel hip-z and knee-x. Cancelling two hip columns instead yields
    # coefficients ~1e-19 and unnecessarily huge removal witnesses (~1e18).
    i, j = 34+7*side, 35+7*side
    t = (y[i]*x[j]-y[j]*x[i])/(z[i]*x[j]-z[j]*x[i])
    u = (t*z[j]-y[j])/x[j]
    assert t > 0 and u > 0
    combined = [a-t*b+u*c for a, b, c in zip(y, z, x)]
    assert combined[i] == combined[j] == 0
    # All constraints are represented as A*v <= b, expanded by the original
    # feasibility tolerance. Negative dual sum proves even this expansion empty.
    names = [ty['name']+'.lower', tz['name']+'.upper', tx['name']+'.lower']
    rows = [[-v for v in y], z, [-v for v in x]]
    bounds = [-fraction(ty['lower'])+EPS, fraction(tz['upper'])+EPS,
              -fraction(tx['lower'])+EPS]
    weights = [F(1), t, u]
    for k, value in enumerate(combined):
        if value:
            sign = 1 if value > 0 else -1
            names.append(f'axis-{k}.{"upper" if sign > 0 else "lower"}')
            rows.append([F(sign) if axis == k else F(0) for axis in range(46)])
            bounds.append((upper[k] if sign > 0 else -lower[k])+EPS)
            weights.append(abs(value))
    axes = sorted({k for row in rows for k, value in enumerate(row) if value})
    matrix = [[row[k] for k in axes] for row in rows]
    assert len(names) == 8 and len(axes) == 7
    witnesses = []
    for removed in range(len(names)):
        keep = [k for k in range(len(names)) if k != removed]
        witness = linear_solve([matrix[k] for k in keep], [bounds[k] for k in keep])
        assert all(dot(matrix[k], witness) == bounds[k] for k in keep)
        violation = dot(matrix[removed], witness)-bounds[removed]
        assert violation > 0
        witnesses.append({'removedConstraint': names[removed],
                          'coordinates': [encode(v) for v in witness],
                          'removedConstraintViolation': encode(violation)})
    result = {'feedbackScale': case['feedbackScale'], 'patch': side,
              'axisIndices': axes, 'scalarConstraintCount': len(names),
              'constraintIds': names, 'rows': [[encode(v) for v in row] for row in matrix],
              'expandedUpperBounds': list(map(encode, bounds)),
              'positiveDualWeights': list(map(encode, weights)),
              'contradictionMargin': encode(-dot(weights, bounds)),
              'removalWitnesses': witnesses, 'minimalByInclusion': True,
              'minimumCardinalityClaimed': False,
              'witnessScope': 'IIS minus named scalar inequality only; other original QP rows are not asserted'}
    verify_iis(result)
    return result


def verify_iis(result):
    names = result['constraintIds']
    A = [[decode(v) for v in row] for row in result['rows']]
    b = list(map(decode, result['expandedUpperBounds']))
    w = list(map(decode, result['positiveDualWeights']))
    assert len(A) == len(b) == len(w) == len(names) == 8
    assert len(set(names)) == 8 and all(len(row) == 7 for row in A)
    assert all(weight > 0 for weight in w)
    assert all(sum(weight*row[k] for weight, row in zip(w, A)) == 0 for k in range(7))
    assert dot(w, b) < 0
    assert decode(result['contradictionMargin']) == -dot(w, b)
    assert len(result['removalWitnesses']) == 8
    assert {v['removedConstraint'] for v in result['removalWitnesses']} == set(names)
    for item in result['removalWitnesses']:
        removed = names.index(item['removedConstraint'])
        v = list(map(decode, item['coordinates']))
        assert len(v) == 7
        assert all(dot(row, v) <= limit for k, (row, limit) in enumerate(zip(A, b)) if k != removed)
        violation = dot(A[removed], v)-b[removed]
        assert violation > 0 and violation == decode(item['removedConstraintViolation'])
    return True


def extract(index_path, canonical_path):
    index = json.loads(index_path.read_text())
    entry = next(f for f in index['files'] if f['rawPath'] == 'native-run/002-B-planning-000.json')
    parts = sorted(entry['parts'], key=lambda p: p['order'])
    raw = b''.join(gzip.decompress((index_path.parent/p['gzipPath']).read_bytes()) for p in parts)
    assert len(raw) == entry['rawBytes']
    assert hashlib.sha256(raw).hexdigest() == entry['rawSha256']
    data = json.loads(raw)
    canonical_bytes = canonical_path.read_bytes()
    canonical = json.loads(canonical_bytes)
    axes = []
    for segment, body in enumerate(canonical['segments']):
        for axis in (body.get('jointProfile') or {}).get('axes', []):
            axes.append({'index': len(axes), 'segment': segment,
                         'id': body['id'], 'coordinate': axis['coordinate']})
    assert len(axes) == 46
    cases = []
    for case in data['cases']:
        attempt = case['terminalOwner']['numericAttempt']
        assert attempt['nativeSubstep'] == 16
        p = attempt['problem']
        cases.append({'feedbackScale': case['feedbackScale'],
                      'nativeSubstep': 16,
                      'problem': {k: p[k] for k in ('lower', 'upper', 'hard')},
                      'previousQ': attempt['compilation']['previousQ'],
                      'patches': case['terminalOwner']['patches']['active']})
    return {'schema': 1, 'sourceArchiveIndex': str(index_path.relative_to(ROOT)),
            'sourceRawSha256': entry['rawSha256'],
            'canonicalJsonSha256': hashlib.sha256(canonical_bytes).hexdigest(),
            'extraction': 'JSON fields only; no Model/Simulation/FK', 'axes': axes, 'cases': cases}


def write_new(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, indent=2)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-index', type=Path, default=ROOT/'docs/evidence/coordination-local-pair-2026-10-13/archive-index.json')
    parser.add_argument('--canonical', type=Path, default=ROOT/'rust/crates/model/data/canonical-v1.json')
    parser.add_argument('--fixtures', type=Path)
    parser.add_argument('--output-dir', type=Path, default=HERE)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    fixtures = json.loads(args.fixtures.read_text()) if args.fixtures else extract(args.archive_index, args.canonical)
    results = [build_iis(case, side) for case in fixtures['cases'] for side in range(2)]
    report = {'schema': 1, 'scope': 'pure algebra on archived coefficients',
              'method': 'exact rational dual certificate and exact Gaussian removal witnesses',
              'scalarIisCount': len(results), 'exactRemovalWitnessCount': sum(len(r['removalWitnesses']) for r in results),
              'originalFeasibilityTolerance': encode(EPS),
              'minimumCardinalityClaimed': False, 'exhaustiveEnumerationClaimed': False,
              'nativeSteps': 0, 'fkEvaluations': 0, 'modelConstructions': 0,
              'simulationConstructions': 0, 'lpSolverCalls': 0,
              'exactLinearSystemsSolved': 64,
              'sourceRawSha256': fixtures['sourceRawSha256'],
              'scriptSha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'systems': results}
    if not args.fixtures:
        write_new(args.output_dir/'archived-support-problems.json', fixtures)
    write_new(args.output_dir/'minimal-support-conflicts.json', report)
    print(json.dumps({k: report[k] for k in ('scalarIisCount', 'exactRemovalWitnessCount', 'nativeSteps', 'fkEvaluations', 'lpSolverCalls')}))


if __name__ == '__main__':
    main()
