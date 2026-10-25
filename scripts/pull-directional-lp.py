#!/usr/bin/env python3
"""One fixed-state, fixed-branch directional LP. No dynamics or controller writes."""
import argparse
import gzip
import hashlib
import json
import math
import subprocess
import time
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'web/proofs/pull-gravity-frame-001'
EPS = 2.220446049250313e-16
TOL = 1e-10  # Existing normalized numerical certificate, not a physical relaxation.
EXPECTED = {
    'last-qp.json.gz': 'bc1331c19e539356457eb4b57748bcab50cc84722863354c24108af84a1dc44c',
    'last-solve.json.gz': 'c30dd00a97cd627e48aae06c02388f0afafcbd5ae8615105cdec1c67d3a4e827',
    'initial-candidate-native.json.gz': 'f6157d64c1f5598647f221802b3631c56ca696056628f9e7e5f308b1951cc9da',
    'final.json.gz': '3bf174a791649512792b8dea38acbacb54094e509d75b526d26ea43dc1311304',
}


def dot(a, b):
    return math.fsum(x*y for x, y in zip(a, b, strict=True))


def exactdot(a, b):
    return sum((Fraction(x)*Fraction(y) for x, y in zip(a, b, strict=True)), Fraction())


def original_residuals(G, h, u):
    residual = [dot(g, u)-b for g, b in zip(G, h, strict=True)]
    normalized = [r/(math.hypot(*g) or 1.) for r, g in zip(residual, G)]
    return dict(residuals=residual, maximumNormalized=max(0., *normalized),
                passed=max(0., *normalized) <= TOL)


def certificate(c, c0, G, h, u, lam):
    """Independent original-units certificate, exact rational dual bound.

    Cap rows imply |u_j|<=1. Therefore for ANY nonnegative lambda:
      min(c0+c.u) >= c0-h.lambda-||c+G.T lambda||_1.
    All inputs to this bound are treated as their exact stored binary64 values.
    A tiny primal violation is reported, never silently called strict feasibility.
    """
    primal = original_residuals(G, h, u)
    r = [math.fsum([c[j], *[row[j]*v for row, v in zip(G, lam)]])
         for j in range(len(c))]
    sign = min(0., *lam)
    den = [math.fsum([abs(c[j]), *[abs(row[j]*v) for row, v in zip(G, lam)]])
           for j in range(len(c))]
    backward = max((abs(v)/d if d else (0. if v == 0 else math.inf))
                   for v, d in zip(r, den))
    complement = max(0., *[abs(v*s) for v, s in zip(lam, primal['residuals'])])
    upper_exact = Fraction(c0)+exactdot(c, u)
    exact_primal = [exactdot(row, u)-Fraction(b) for row, b in zip(G, h)]
    strict_feasible = all(v <= 0 for v in exact_primal)
    lower = lower_exact = lower_float = allowance = None
    if sign == 0:
        exact_r = [Fraction(c[j])+exactdot([g[j] for g in G], lam)
                   for j in range(len(c))]
        L = Fraction(c0)-exactdot(h, lam)-sum(map(abs, exact_r))
        lower_exact = [str(L.numerator), str(L.denominator)]
        lower = math.nextafter(float(L), -math.inf)
        lower_float = c0-dot(h, lam)-math.fsum(map(abs, r))
        allowance = max(0., lower_float-lower)
        assert Fraction(lower) <= L
    U = math.nextafter(float(upper_exact), math.inf)
    assert Fraction(U) >= upper_exact
    passed = (primal['passed'] and sign == 0 and backward <= TOL
              and complement <= 1e-8 and all(math.isfinite(v) for v in [U, *r, *lam]))
    return dict(passed=passed, primal=primal, multipliers=lam, dualSignMinimum=sign,
                stationarity=r, componentwiseBackwardError=backward,
                complementarity=complement, dualLower=lower,
                dualLowerExact=lower_exact, dualFloatBeforeAllowance=lower_float,
                dualRoundingAllowance=allowance, primalObjective=U,
                primalUpperStrictlyCertified=strict_feasible,
                exactPrimalMaximum=float(max(exact_primal)),
                numericalGap=None if lower is None else U-lower,
                note='Dual bound exact for stored rounded affine LP. Primal upper is strict only when exactPrimalMaximum<=0; otherwise numerical witness only.')


def pure_tests():
    # No linprog calls in tests. Hand-known optima/multipliers and counterexamples.
    for c, u, lam in [([1.], [-1.], [0., 1.]), ([-1.], [1.], [1., 0.])]:
        x = certificate(c, 0., [[1.], [-1.]], [1., 1.], u, lam)
        assert x['passed'] and x['primalUpperStrictlyCertified']
        assert x['dualLower'] <= -1. <= x['primalObjective']
    assert not certificate([1.], 0., [[1.], [-1.]], [1., 1.], [-1.], [0., -1.])['passed']
    assert not original_residuals([[1.], [-1.]], [1., 1.], [1.001])['passed']
    # Active edge: jar<=0. A negative native force predictor must be rejected.
    assert original_residuals([[2.]], [1.], [.25])['passed']
    assert not original_residuals([[2.]], [1.], [.6])['passed']
    # World +Y is free-side; c.u+c0 reproduces J(a0+A.u)+bias.
    a0, A, j, b, u = [1., 2.], [[3., 4.], [-2., 1.]], [2., -1.], .7, [.2, -.3]
    c, c0 = [dot(j, col) for col in A], dot(j, a0)+b
    a = [v+math.fsum(col[i]*u[k] for k, col in enumerate(A)) for i, v in enumerate(a0)]
    assert abs(dot(c, u)+c0-dot(j, a)-b) < 1e-15
    # Gear/DOF mapping must not create a root actuator.
    col = [0.]*6+[20., 0.]
    assert next(i for i, v in enumerate(col) if v) >= 6 and dot(col, [0.]*6+[.5, 0.]) == 10.
    return dict(passed=True, checks=8, LPcalls=0, nativeCalls=0)


def growth():
    return int(subprocess.check_output(['du', '-sx', '-B1', '/workspace']).split()[0])-419647488+13123543


def main(out):
    import scipy
    from scipy.optimize import linprog
    assert not out.exists(), 'Do not duplicate a started diagnostic'
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == '2cde6bb99e6f6537175e2207312aa4a266b02314'
    assert growth()+100000 < 1530000000
    sources = {}
    for name, expected in EXPECTED.items():
        data = (SOURCE/name).read_bytes()
        assert hashlib.sha256(data).hexdigest() == expected
        sources[name] = json.loads(gzip.decompress(data))
    q, old, raw, cp = [sources[k] for k in EXPECTED]
    assert len(q['rows']) == 156 and len(q['J']) == 24 and q['n'] == 21
    assert all(q['active']) and q['states'] == [1]*24 and q['types'] == [6]*24
    assert len(cp['state']) == 233 and cp['physics']['steps'] == 8727
    assert cp['experimental']['reference']['recoveryDone']
    tests = pure_tests()
    G = [r['a'][:] for r in q['rows']]
    h = [r['bound'] for r in q['rows']]
    labels = [r['label'] for r in q['rows']]
    branch = []
    for i, row in enumerate(q['J']):
        g, b = [dot(row, col) for col in q['A']], q['aref'][i]-dot(row, q['a0'])
        G.append(g); h.append(b); labels.append(f'native-active-edge-{i}')
        branch.append(dict(label=labels[-1], a=g, bound=b))
    j, bias = q['gravity']['worldCOM']['jp'][1], q['gravity']['worldCOM']['bp'][1]
    c, c0 = [dot(j, col) for col in q['A']], dot(j, q['a0'])+bias
    witness = old['solve']['x']
    witness_check = original_residuals(G, h, witness)
    assert witness_check['passed'] and raw['controls'] == witness
    witness_objective = dot(c, witness)+c0
    assert abs(witness_objective-dot(j, raw['qacc'])-bias) < 1e-10
    assert len(q['A']) == 21 and all(next(k for k, v in enumerate(col) if v) >= 6 and sum(v != 0 for v in col) == 1 for col in q['B'][:21])
    scale = [max(math.hypot(*g), abs(b)) or 1. for g, b in zip(G, h)]
    scaledG = [[v/s for v in row] for row, s in zip(G, scale)]
    scaledh = [b/s for b, s in zip(h, scale)]
    out.mkdir()
    ledger = dict(sourceHEAD='2cde6bb99e6f6537175e2207312aa4a266b02314',
                  inputs=EXPECTED, LPcalls=0, pendingLP=False, forwards=0,
                  pendingForward=False, ticks=0, forecasts=0, confirmedTicks=229565,
                  unknownHistoryPreserved=True, growthBefore=growth(), completed=False,
                  options=dict(method='highs-ds', maxiter=64, time_limit=10., presolve=True,
                               primal_feasibility_tolerance=TOL, dual_feasibility_tolerance=TOL))
    def save(name, data):
        (out/name).write_bytes(gzip.compress(json.dumps(data, allow_nan=False, separators=(',', ':')).encode(), mtime=0))
    def persist():
        (out/'ledger.json').write_text(json.dumps(ledger, separators=(',', ':')))
    save('problem.json.gz', dict(objective=c, constant=c0, branchRows=branch,
         labels=labels, rowScales=scale, originalRowsFrom=EXPECTED['last-qp.json.gz'],
         originalConstraintCount=156, branchConstraintCount=24,
         nativeLaw='lambda=max(0,-D*(J*a-aref)); all current edges active',
         tests=tests, savedWitness=witness, witnessCheck=witness_check,
         witnessObjective=witness_objective, scipy=scipy.__version__))
    ledger.update(LPcalls=1, pendingLP=True); persist()
    start = time.perf_counter()
    # Explicit unbounded variables: all actuator bounds already appear in G.
    fit = linprog(c, A_ub=scaledG, b_ub=scaledh, bounds=[(None, None)]*21,
                  method='highs-ds', options={k: v for k, v in ledger['options'].items() if k != 'method'})
    elapsed = time.perf_counter()-start
    ledger.update(pendingLP=False, LPseconds=elapsed, iterations=int(fit.nit), LPstatus=int(fit.status))
    result = dict(success=bool(fit.success), status=int(fit.status), message=str(fit.message),
                  iterations=int(fit.nit), elapsedSeconds=elapsed,
                  controls=None if fit.x is None else fit.x.tolist(),
                  scipyMarginals=None if fit.ineqlin.marginals is None else fit.ineqlin.marginals.tolist())
    # Persist solver output BEFORE any acceptance gate.
    save('lp.json.gz', result); persist()
    assert fit.success and fit.nit <= 64 and elapsed <= 10., 'Single LP inconclusive; stop without retry'
    # HiGHS marginals are d optimal cost/d bound, opposite to lambda for G*u<=h.
    lam = [-float(m)/s for m, s in zip(fit.ineqlin.marginals, scale)]
    cert = certificate(c, c0, G, h, result['controls'], lam)
    u = result['controls']
    pred = [v+math.fsum(col[i]*u[k] for k, col in enumerate(q['A'])) for i, v in enumerate(q['a0'])]
    cert.update(labels=labels, predictedAcceleration=pred,
                predictedWorldY=dot(j, pred)+bias,
                predictedEdges=[-D*(dot(row, pred)-ar) for row, D, ar in zip(q['J'], q['D'], q['aref'])])
    save('certificate.json.gz', cert)
    ledger['LPcertificatePassed'] = cert['passed']; persist()
    assert cert['passed'], 'Independent LP certificate inconclusive; no native candidate'
    print(json.dumps(dict(status='LP qualified numerically', iterations=fit.nit,
        seconds=elapsed, lower=cert['dualLower'], candidateObjective=cert['primalObjective'],
        strictPrimal=cert['primalUpperStrictlyCertified'],
        primalViolation=cert['primal']['maximumNormalized'], backwardError=cert['componentwiseBackwardError'])))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', nargs='?', type=Path)
    parser.add_argument('--pure', action='store_true')
    args = parser.parse_args()
    if args.pure:
        print(json.dumps(pure_tests()))
    else:
        assert args.output is not None
        main(args.output)
