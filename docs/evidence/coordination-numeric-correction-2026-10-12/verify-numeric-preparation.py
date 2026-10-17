#!/usr/bin/env python3
"""Run only the reviewed algebra/static test modules; never a native worker."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
LEDGER = ROOT / 'docs/evidence/coordination-prototype-2026-10-12/budget/ledger.json'
CLOSED = '2afa69b3ae9c8ab8343f7dabdf46b955c8ac7b88aa259404958e25d49a72ef07'
MODULES = {'whole_body_tasks': 11, 'coordinated_runtime': 7,
           'geometric_tasks': 6, 'whole_body_controller': 6}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cargo', default='cargo')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    if digest(LEDGER) != CLOSED:
        raise SystemExit('Closed physical ledger differs; no verification started')
    started = time.monotonic()
    intent = {'startedUtc': datetime.now(timezone.utc).isoformat(),
              'scope': 'reviewed pure/static modules only, no Rapier substeps',
              'nativeSubstepsAuthorized': 0, 'closedLedgerSha256': CLOSED,
              'moduleExpectedTests': MODULES, 'releaseWorkerBuiltOrRun': False}
    (out / 'intent.json').write_text(json.dumps(intent, indent=2) + '\n')
    env = dict(os.environ)
    for var, name in [('LH_NUMERIC_REPLAY_EXPORT', 'reconstructed-problems.json'),
                      ('LH_NUMERIC_REFUSAL_EXPORT', 'reproduced-refusals.json'),
                      ('LH_NUMERIC_CORRECTION_EXPORT', 'certified-commands.json')]:
        env[var] = str(out / name)
    base = [args.cargo, '+1.89.0']
    common = ['--manifest-path', str(ROOT / 'rust/Cargo.toml'), '-p', 'lh-sim', '--lib', '--locked']
    results = []
    try:
        for module, expected in MODULES.items():
            cmd = base + ['test'] + common + ['--features', 'experimental-solver-step-observation',
                                            f'recovery::planner::{module}::tests']
            path = out / (module + '.log')
            with path.open('w') as log:
                run = subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=600)
            summaries = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed;', path.read_text())
            ok = run.returncode == 0 and summaries == [(str(expected), '0')]
            results.append({'module': module, 'command': cmd, 'returncode': run.returncode,
                            'expectedTests': expected, 'passed': ok, 'logSha256': digest(path)})
            print(module, 'PASS' if ok else 'FAIL', flush=True)
            if not ok:
                raise RuntimeError(f'Fixed test set failed: {module}')
        cmd = base + ['check'] + common
        path = out / 'production-check.log'
        with path.open('w') as log:
            run = subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=600)
        results.append({'module': 'production-check', 'command': cmd, 'returncode': run.returncode,
                        'passed': run.returncode == 0, 'logSha256': digest(path)})
        if run.returncode:
            raise RuntimeError('Production check failed')
    finally:
        unchanged = digest(LEDGER) == CLOSED
        receipt = dict(intent, endedUtc=datetime.now(timezone.utc).isoformat(),
                       elapsedSeconds=time.monotonic()-started, results=results,
                       ledgerUnchanged=unchanged, nativeSubstepsPerformed=0,
                       statementBasis='reviewed fixed call paths, not runtime instrumentation')
        (out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        if not unchanged:
            raise RuntimeError('Closed physical ledger changed')
    print('30 targeted tests and production check PASS; no native integration', flush=True)

if __name__ == '__main__':
    main()
