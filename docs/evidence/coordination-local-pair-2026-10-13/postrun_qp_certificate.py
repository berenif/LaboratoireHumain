#!/usr/bin/env python3
"""Exact arithmetic on archived QP coefficients only: no solver, FK or physics."""
import argparse
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path


def rational(value):
    # The recorded decimal round-trips to the native f64 coefficient; preserve
    # that exact binary rational for every subsequent arithmetic operation.
    return Fraction.from_float(float(value))


def evidence(value):
    return {'approximate': float(value), 'numerator': str(value.numerator),
            'denominator': str(value.denominator)}


def certify(data):
    eps = rational(1e-8)
    results = []
    assert [c['feedbackScale'] for c in data['cases']] == [0, 0.5, 1, 1.5]
    for case in data['cases']:
        attempt = case['terminalOwner']['numericAttempt']
        assert attempt['nativeSubstep'] == 16
        problem = attempt['problem']
        lower = list(map(rational, problem['lower']))
        upper = list(map(rational, problem['upper']))
        assert len(lower) == len(upper) == 46
        assert all(-1 <= lo <= hi <= 1 for lo, hi in zip(lower, upper))
        for side in range(2):
            tx, ty, tz = problem['hard'][3*side:3*side+3]
            assert [row['name'] for row in (tx, ty, tz)] == [
                f'material-patch-{side}-world-{axis}' for axis in range(3)]
            x, y, z = [list(map(rational, row['row'])) for row in (tx, ty, tz)]
            i, j = 32 + 7*side, 34 + 7*side
            t = (y[i]*x[j] - y[j]*x[i]) / (z[i]*x[j] - z[j]*x[i])
            u = (t*z[j] - y[j]) / x[j]
            assert t > 0 and u > 0
            row = [a - t*b + u*c for a, b, c in zip(y, z, x)]
            assert row[i] == row[j] == 0
            required = rational(ty['lower']) - t*rational(tz['upper']) + u*rational(tx['lower'])
            maximum = sum(a*(upper[k] if a >= 0 else lower[k]) for k, a in enumerate(row))
            gap = required - maximum
            # Allow every hard inequality AND every coordinate bound its entire
            # original feasibility tolerance, making the contradiction weaker.
            allowance = eps * (1 + t + u + sum(map(abs, row)))
            assert gap > allowance
            results.append({
                'feedbackScale': case['feedbackScale'], 'patch': side,
                'hardRows': [tx['name'], ty['name'], tz['name']],
                'cancelledAxisIndices': [i, j], 'lambdaZ': evidence(t),
                'lambdaX': evidence(u), 'requiredMinimum': evidence(required),
                'boxMaximum': evidence(maximum), 'gap': evidence(gap),
                'originalToleranceAllowance': evidence(allowance),
                'gapAfterAllOriginalTolerances': evidence(gap-allowance),
                'exactContradiction': True,
            })
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stored = args.input.read_bytes()
    raw = gzip.decompress(stored) if args.input.suffix == '.gz' else stored
    results = certify(json.loads(raw))
    report = {'schema': 1, 'scope': 'archived-instantaneous-linear-constraints-only',
              'method': 'exact binary-rational linear combination and interval bound; no optimization',
              'sourcePath': str(args.input), 'sourceUncompressedSha256': hashlib.sha256(raw).hexdigest(),
              'scriptSha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'nativeSteps': 0, 'fkEvaluations': 0, 'solverInvocations': 0,
              'exactContradictions': len(results), 'allPassed': True, 'certificates': results}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'exactContradictions': len(results), 'nativeSteps': 0,
                      'fkEvaluations': 0, 'solverInvocations': 0, 'output': str(args.output)}))


if __name__ == '__main__':
    main()
