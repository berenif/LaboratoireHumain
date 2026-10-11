#!/usr/bin/env python3
"""Explicit P1 histories with irreversible reservations; never launches a search.

Interrupted launches retain their full reservation. All integration in this
milestone must use this coordinator (one native process, one thread).
"""
import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

REPO = Path(__file__).resolve().parents[1]
MAX_HISTORIES = 32
MAX_NATIVE_STEPS = 26 * 240
CONTRACT_WALL_S = 1800
GLOBAL_WALL_S = 21600
GLOBAL_STEPS = 1509120


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    tmp = path.with_suffix('.tmp')
    with tmp.open('w') as file:
        json.dump(value, file, indent=2)
        file.write('\n')
        file.flush()
        os.fsync(file.fileno())
    os.replace(tmp, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--kernel', type=Path, required=True)
    parser.add_argument('--program', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--trace', action='store_true')
    parser.add_argument('--controller-control', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9-]+', args.label):
        parser.error('label must contain lowercase letters, digits and hyphens')
    root = args.root.resolve()
    kernel, program = args.kernel.resolve(strict=True), args.program.resolve(strict=True)
    root.mkdir(parents=True, exist_ok=True)
    with (root / 'coordinator.lock').open('a') as lock:
        # Keep exclusive ownership while the single history runs.
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger_path = root / 'ledger.json'
        if ledger_path.exists():
            ledger = json.loads(ledger_path.read_text())
        else:
            now = utc()
            ledger = {'schema': 1, 'campaign': 'native-hybrid-v1', 'createdUtc': now,
                      'contractStartedUtc': now, 'status': 'P1-observation-in-progress',
                      'p1Passed': False, 'searchStarted': 0, 'validationStarted': 0,
                      'limits': {'contractHistories': MAX_HISTORIES,
                                 'nativeStepsPerContractHistory': MAX_NATIVE_STEPS,
                                 'contractWallS': CONTRACT_WALL_S, 'globalWallS': GLOBAL_WALL_S,
                                 'globalNativeSteps': GLOBAL_STEPS},
                      'historiesStarted': 0, 'reservedNativeSteps': 0,
                      'actualNativeStepsKnown': 0, 'entries': [],
                      'previousCampaignLedgerSha256': sha(REPO / 'evidence/recovery-coordinated-retarget-2026-10-11/native-campaign/ledger.json')}
            save(ledger_path, ledger)
        assert all('endedUtc' in entry for entry in ledger['entries']), 'Uncertain previous launch: reconcile before resuming'
        assert sum(p.stat().st_size for p in root.rglob('*') if p.is_file()) + 512*1024**2 <= 2*1024**3, 'Evidence storage exhausted'
        assert ledger['campaign'] == 'native-hybrid-v1'
        assert ledger['limits'] == {'contractHistories': MAX_HISTORIES,
                                    'nativeStepsPerContractHistory': MAX_NATIVE_STEPS,
                                    'contractWallS': CONTRACT_WALL_S, 'globalWallS': GLOBAL_WALL_S,
                                    'globalNativeSteps': GLOBAL_STEPS}, 'Budget contract changed'
        assert ledger['previousCampaignLedgerSha256'] == sha(REPO / 'evidence/recovery-coordinated-retarget-2026-10-11/native-campaign/ledger.json') == 'b61f17b180afece7db1e3a7d541edeaf24094f358edd2bb7e1362797d8b600e4', 'Closed campaign changed'
        assert ledger['status'] == 'P1-observation-in-progress', 'P1 budget closed'
        elapsed = (dt.datetime.now(dt.timezone.utc) - dt.datetime.fromisoformat(ledger['contractStartedUtc'])).total_seconds()
        remaining = CONTRACT_WALL_S - elapsed
        assert remaining > 0 and elapsed < GLOBAL_WALL_S, 'Wall budget exhausted'
        assert ledger['historiesStarted'] < MAX_HISTORIES, 'Contract histories exhausted'
        assert ledger['reservedNativeSteps'] + MAX_NATIVE_STEPS <= GLOBAL_STEPS
        index = ledger['historiesStarted']
        output = root / f'{index:03d}-{args.label}'
        assert not output.exists(), 'Never overwrite a history'
        entry = {'index': index, 'label': args.label, 'startedUtc': utc(),
                 'reservedNativeSteps': MAX_NATIVE_STEPS, 'status': 'reserved-before-launch',
                 'kernelSha256': sha(kernel), 'programSha256': sha(program),
                 'coordinatorSha256': sha(Path(__file__)), 'traceEnabled': args.trace,
                 'controllerControl': args.controller_control, 'output': str(output),
                 'gitHead': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()}
        ledger['entries'].append(entry)
        ledger['historiesStarted'] += 1
        ledger['reservedNativeSteps'] += MAX_NATIVE_STEPS
        save(ledger_path, ledger)
        env = dict(os.environ, LH_WHOLE_PROGRAM=str(program), LH_WHOLE_OUT=str(output),
                   LH_WHOLE_SOLVER_TRACE='1' if args.trace else '0',
                   LH_WHOLE_CONTROLLER_CONTROL='1' if args.controller_control else '0',
                   LH_WHOLE_WALL_S=str(remaining), LH_WHOLE_IDLE_VALIDATION='0',
                   LH_WHOLE_HEADING_DEG='0', LH_WHOLE_PERTURB_DEG='0',
                   OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        command = [str(kernel), '--exact',
                   'recovery::support_probe::whole_history::run_whole_reference_native_history',
                   '--ignored', '--nocapture', '--test-threads=1']
        began = time.monotonic()
        with (root / f'{index:03d}-{args.label}.log').open('w') as log:
            try:
                result = subprocess.run(command, cwd=REPO, env=env, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=remaining, check=False)
                entry['processExitCode'] = result.returncode
                entry['status'] = 'process-completed' if result.returncode == 0 else 'process-failed'
            except subprocess.TimeoutExpired:
                entry['status'] = 'timeout-reservation-retained'
        entry.update(endedUtc=utc(), coordinatorWallS=time.monotonic()-began)
        report_path = output / 'report.json'
        if report_path.exists():
            report = json.loads(report_path.read_text())
            count = report['physicalSubsteps']
            assert 0 <= count <= MAX_NATIVE_STEPS, 'Native step reservation exceeded'
            entry.update(actualNativeSteps=count, reportSha256=sha(report_path),
                         firstRefusal=report['firstRefusal'], fullRecoverySuccess=report['fullRecoverySuccess'])
            ledger['actualNativeStepsKnown'] += count
        save(ledger_path, ledger)
        print(json.dumps(entry, indent=2))
        if entry['status'] != 'process-completed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
