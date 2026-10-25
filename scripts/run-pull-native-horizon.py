#!/usr/bin/env python3
"""Exactly one bounded SLSQP call; persistent native worker owns all forecasts."""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('PYTHONDONTWRITEBYTECODE', '1')
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
MAX_EVALUATIONS = 128
MAX_SECONDS = 900


class StopCampaign(RuntimeError):
    pass


def save(path, obj):
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(obj, separators=(',', ':'), allow_nan=False))
    tmp.replace(path)


def key(x):
    return np.asarray(x, dtype=np.float64).tobytes()


def tests():
    # Verify callback caching, signed normalized constraints and finite-difference
    # accounting WITHOUT invoking an optimizer, worker, or native dynamics.
    cache, calls = {}, []
    def ev(x):
        k = key(x)
        if k not in cache:
            calls.append(k)
            cache[k] = {'objective': sum(v*v for v in x), 'constraints': [1-x[0]]}
        return cache[k]
    for x in ([0.], [0.], [1e-8], [1e-8]):
        ev(x)['objective']; ev(x)['constraints']
    assert len(calls) == 2
    assert ev([1.1])['constraints'][0] < 0
    assert key([0.]) != key([-0.])
    assert MAX_EVALUATIONS*400 == 51200
    assert math.isfinite(np.sqrt(np.finfo(float).eps))
    return {'passed': True, 'checks': 5, 'optimizerCalls': 0,
            'nativeSteps': 0, 'cacheCountsSyntheticOnly': True}


def run(out, setup_repair_of=None):
    assert not out.exists(), 'Refuse existing campaign; no duplicate/restart'
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == 'e066a3b8557f82ab2ed81f9b9380b0a4f86dab50'
    additions = ['scripts/pull-native-horizon.mjs', 'scripts/evaluate-pull-native-horizon.mjs', 'scripts/run-pull-native-horizon.py']
    before = int(subprocess.check_output(['du', '-sx', '-B1', '/workspace']).split()[0])-419647488+13123543
    assert before+500000 < 1530000000
    if setup_repair_of:
        previous = json.loads((setup_repair_of/'search-ledger.json').read_text())
        native_previous = json.loads((setup_repair_of/'worker-ledger.json').read_text())
        assert previous['optimizerCalls'] == 0 and previous['predictiveSteps'] == 0
        assert native_previous['evaluations'] == 0 and native_previous['predictiveSteps'] == 0
        assert native_previous['pendingPredictiveSteps'] == 0
    checks = tests()
    out.mkdir()
    protocol = dict(kind='one native dynamic shooting search', horizonSeconds=.5,
        dt=.00125, substepsPerEvaluation=400, maxEvaluations=128,
        maxPredictiveSteps=51200, maxSeconds=900, realTicks=0,
        oneOptimizerCall=True, maxIterations=64, variables=42,
        interpolation='convex C1 cubic smoothstep at0/.25/.5seconds; u0 fixed',
        initializer='repeat recorded actual command at both free nodes',
        objective='mean squared normalized torque400x21, no posture/force weight tuning',
        constraints='native actual path ROM/caps/20deg/25mm/support1N25ms/nonplantar/fall, terminal5mm world AND relative, velocities<=0, tilt<=entry, omega<=.2',
        predictor120ms='preserved historical diagnostic, not a proof or hard constraint for this changing command sequence',
        infeasibleForecasts='complete400 steps to measure all violations, never applied live; engine inconsistency stops campaign',
        maxCodeAndProofBytes=500000, maximumGrowth=1530000000,
        confirmedBefore=229565, unknownHistoryPreserved=True, Rapier=45735,
        caching='exact binary64 vector keys; objective/constraints share ONE native evaluation; finite differences counted',
        validation='only evaluated vectors may be qualified; no automatic extra validation or retry',
        numericalSettings={'SLSQPftol': 1e-9, 'finiteDifferenceEps': float(np.sqrt(np.finfo(float).eps)), 'physicalToleranceRelaxation': False},
        setupRepairOf=str(setup_repair_of) if setup_repair_of else None,
        pure=checks, sourceFiles={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in additions})
    save(out/'protocol.json', protocol)
    ledger = dict(optimizerCalls=0, nativeEvaluations=0, realTicks=0,
        predictiveSteps=0, pendingEvaluation=False, completed=False, growthBefore=before)
    save(out/'search-ledger.json', ledger)
    stderr = (out/'worker-stderr.txt').open('w')
    worker = subprocess.Popen(['node', 'scripts/evaluate-pull-native-horizon.mjs', str(out)],
        cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr,
        text=True, bufsize=1, env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
    started = time.monotonic()
    epoch_deadline = int((time.time()+900)*1000)
    cache, labels = {}, None
    ready = None
    worker_fatal = False
    def receive():
        nonlocal worker_fatal
        remaining = MAX_SECONDS-(time.monotonic()-started)
        if remaining <= 0 or not select.select([worker.stdout], [], [], remaining)[0]:
            raise StopCampaign('15minute campaign wall budget')
        line = worker.stdout.readline()
        if not line:
            raise StopCampaign('Native worker closed; inspect stderr/worker-ledger before any recovery')
        obj = json.loads(line)
        if obj.get('fatal'):
            worker_fatal = True
            raise StopCampaign('Native engine/copy inconsistency: '+obj['error'])
        return obj
    def send(obj):
        worker.stdin.write(json.dumps(obj, separators=(',', ':'))+'\n')
        worker.stdin.flush()
    def evaluate(x):
        nonlocal labels
        k = key(x)
        if k in cache:
            return cache[k]
        if len(cache) >= MAX_EVALUATIONS:
            raise StopCampaign('128evaluations budget, including finite differences')
        if time.monotonic()-started >= MAX_SECONDS:
            raise StopCampaign('15minute campaign wall budget')
        if not np.isfinite(x).all() or np.max(np.abs(x)) > 1:
            raise StopCampaign('Optimizer requested nonfinite/out-of-cap vector; no clipping')
        ledger['pendingEvaluation'] = True
        save(out/'search-ledger.json', ledger)
        send({'action': 'evaluate', 'x': np.asarray(x).tolist(), 'deadlineEpochMs': epoch_deadline})
        obj = receive()
        assert obj['evaluated'] and obj['summary']['steps'] == 400
        if labels is None:
            labels = obj['labels']
        assert labels == obj['labels'], 'Fixed constraint order/dimension required'
        assert len(obj['constraints']) == len(labels) and np.isfinite(obj['constraints']).all()
        cache[k] = obj
        ledger.update(pendingEvaluation=False, nativeEvaluations=len(cache),
            predictiveSteps=obj['summary']['predictiveSteps'])
        save(out/'search-ledger.json', ledger)
        if len(cache) == 1 or len(cache) % 20 == 0:
            print(json.dumps({'stage': 'native horizon progress', 'evaluations': len(cache),
                'predictiveSteps': ledger['predictiveSteps'], 'objective': obj['summary']['objective'],
                'violation': obj['summary']['maxNormalizedViolation'],
                'worldGain': obj['summary']['terminal']['worldGain'],
                'relativeGain': obj['summary']['terminal']['relativeGain']}), flush=True)
        return obj
    result, reason, optimizer_x_key = None, 'not started', None
    try:
        ready = receive()
        assert ready['ready'] and ready['pure']['passed']
        save(out/'ready.json', {'pure': ready['pure'], 'time': ready['declaration']['time'], 'sourceSHA256': ready['declaration']['sourceSHA256']})
        x0 = np.tile(ready['declaration']['u0'], 2)
        ledger['optimizerCalls'] = 1
        save(out/'search-ledger.json', ledger)
        print(json.dumps({'stage': 'SINGLE SLSQP starts', 'time': ready['declaration']['time'], 'restored233': True,
            'maxEvaluations': 128, 'maxPredictiveSteps': 51200, 'pure': ready['pure']}), flush=True)
        result = minimize(lambda x: evaluate(x)['summary']['objective'], x0,
            method='SLSQP', bounds=[(-1., 1.)]*42,
            constraints=[{'type': 'ineq', 'fun': lambda x: np.array(evaluate(x)['constraints'])}],
            options={'maxiter': 64, 'ftol': 1e-9, 'eps': np.sqrt(np.finfo(float).eps), 'disp': False})
        optimizer_x_key = key(result.x)
        reason = 'SLSQP terminated: '+str(result.message)
    except Exception as exc:
        reason = type(exc).__name__+': '+str(exc)
    finally:
        # Do not materialize an unevaluated result as a successful trajectory.
        evaluated_result = cache.get(optimizer_x_key)
        snapshot = dict(reason=reason, optimizerCalled=ledger['optimizerCalls'],
            optimizerSuccess=bool(result.success) if result is not None else False,
            optimizerIterations=int(result.nit) if result is not None else None,
            optimizerXActuallyEvaluated=evaluated_result is not None,
            optimizerResultFeasible=evaluated_result['summary']['feasible'] if evaluated_result else None,
            nativeEvaluations=len(cache), predictiveSteps=ledger['predictiveSteps'],
            realTicks=0, elapsedSeconds=time.monotonic()-started,
            interpretation='Only actual evaluated native paths are evidence; search nonconvergence is not physical impossibility.')
        if result is not None:
            snapshot['optimizerX'] = result.x.tolist()
        save(out/'search-result.json', snapshot)
        if worker.poll() is None and not worker_fatal:
            try:
                send({'action': 'finish', 'reason': reason})
                final = receive()
                save(out/'final-result.json', {**snapshot, **final})
                ledger['workerFinished'] = bool(final.get('finished'))
            except Exception as exc:
                ledger['finishError'] = str(exc)
        if worker_fatal and worker.stdin:
            worker.stdin.close()
        if worker.poll() is None:
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                worker.terminate(); worker.wait(timeout=5)
                ledger['workerTerminated'] = True
        stderr.close()
        ledger.update(completed=True, reason=reason, elapsedSeconds=time.monotonic()-started)
        wl = out/'worker-ledger.json'
        if wl.exists():
            native = json.loads(wl.read_text())
            ledger['predictiveSteps'] = native['predictiveSteps']
            ledger['pendingPredictiveSteps'] = native['pendingPredictiveSteps']
            ledger['nativeEvaluationAttempts'] = native['evaluations']
            ledger['unknownOutcome'] = bool(native['pendingPredictiveSteps'])
            ledger['pendingEvaluation'] = False
        ledger['confirmedIncludingNewForecasts'] = 229565+ledger['predictiveSteps']
        ledger['unknownHistoryPreserved'] = True
        ledger['growthAfter'] = int(subprocess.check_output(['du', '-sx', '-B1', '/workspace']).split()[0])-419647488+13123543
        ledger['codeAndProofBytes'] = sum((ROOT/p).stat().st_size for p in additions)+sum(p.stat().st_size for p in out.iterdir())
        save(out/'search-ledger.json', ledger)
        assert ledger['predictiveSteps'] <= 51200 and ledger['codeAndProofBytes'] < 500000
        assert ledger['growthAfter'] < 1530000000
        print(json.dumps({'stage': 'campaign stopped', **ledger}), flush=True)
    return 0 if (out/'final-result.json').exists() else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser();p.add_argument('out', nargs='?');p.add_argument('--check', action='store_true');p.add_argument('--setup-repair-of', type=Path);args = p.parse_args()
    if args.check:
        print(json.dumps(tests()));sys.exit(0)
    assert args.out
    sys.exit(run(Path(args.out), args.setup_repair_of))
