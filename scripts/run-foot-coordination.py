#!/usr/bin/env python3
"""Frozen archived B witness then coordinated C. Exactly two histories; no tuning."""
import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

REPO = Path(__file__).resolve().parents[1]
LIMITS = {'histories': 2, 'nativeStepsPerHistory': 6240,
          'totalReservedNativeSteps': 12480, 'simulationWallSeconds': 300,
          'processes': 1, 'threads': 1}


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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


def admit(ledger, manifest):
    assert ledger['campaign'] == 'foot-coordination-2026-10-12', 'Wrong campaign'
    assert dt.datetime.now(dt.timezone.utc) < dt.datetime.fromisoformat(ledger['preparationDeadlineUtc']), 'Preparation deadline expired'
    assert ledger['limits'] == LIMITS, 'Budget contract changed'
    assert ledger['status'] == 'frozen-ready', 'A/B budget closed or not frozen'
    assert ledger['historiesStarted'] == len(ledger['entries']), 'Inconsistent count'
    assert all(e.get('status') == 'process-completed' and 'actualNativeSteps' in e
               for e in ledger['entries']), 'Uncertain previous launch'
    assert ledger['historiesStarted'] < 2, 'History cap exhausted'
    assert ledger['reservedNativeSteps'] == ledger['historiesStarted'] * 6240
    assert ledger['reservedNativeSteps'] + 6240 <= 12480, 'Native cap exhausted'
    assert ledger['manifestSha256'] == manifest['sha256'], 'Manifest changed'
    for path, digest in manifest['value']['files'].items():
        assert sha(path) == digest, 'Frozen input changed: ' + path
    for path, digest in ledger['previousLedgers'].items():
        assert sha(REPO / path) == digest, 'Closed previous ledger changed'
    index = ledger['historiesStarted']
    if index == 1:
        assert ledger['entries'][0].get('witnessEquivalence', {}).get('equal'), 'Witness neutrality missing'
    elapsed = 0.0 if 'simulationStartedUtc' not in ledger else (
        dt.datetime.now(dt.timezone.utc) - dt.datetime.fromisoformat(ledger['simulationStartedUtc'])).total_seconds()
    assert elapsed < 300, 'Wall budget exhausted'
    return index, 300-elapsed


def witness_equivalence(actual, archived):
    keys = ['substep','timeS','heightM','com','velocity','acceleration','loadsN','capSumNm',
            'deliveredAbsoluteNm','standing','routeStanding','quietHoldSubsteps','recoveries',
            'motion','bodiesHash','motorsHash','contactsHash']
    a = [json.loads(line) for line in (actual/'substeps.jsonl').open()]
    b = [json.loads(line) for line in (Path(archived)/'substeps.jsonl').open()]
    if len(a) != len(b):
        return {'equal':False,'reason':'different observed step count','actual':len(a),'archived':len(b)}
    for index,(x,y) in enumerate(zip(a,b)):
        for key in keys:
            if x[key] != y[key]:
                return {'equal':False,'firstDifferentSubstep':index+1,'key':key}
    ar = json.loads((actual/'report.json').read_text())
    br = json.loads((Path(archived)/'report.json').read_text())
    for key in ['physicalSubsteps','firstRefusal','fullRecoverySuccess','finalWorldHashes']:
        if ar[key] != br[key]:
            return {'equal':False,'reportKey':key}
    return {'equal':True,'observedSteps':len(a),'physicalSteps':ar['physicalSubsteps'],
            'fullNativeWorldHashesEqual':True,'comparedKeys':keys}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    with (root / 'coordinator.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger_path = root / 'ledger.json'
        ledger = json.loads(ledger_path.read_text())
        manifest_path = root / 'frozen-manifest.json'
        manifest = {'sha256': sha(manifest_path), 'value': json.loads(manifest_path.read_text())}
        index, remaining = admit(ledger, manifest)
        assert sum(p.stat().st_size for p in root.rglob('*') if p.is_file()) + 256*1024**2 < 2*1024**3, 'Storage cap'
        mode = 'witness' if index == 0 else 'coordination'
        output = root / ['000-witness-B', '001-coordinated-C'][index]
        assert not output.exists(), 'Never overwrite a history'
        value = manifest['value']
        if 'simulationStartedUtc' not in ledger:
            ledger['simulationStartedUtc'] = utc()
            ledger['simulationDeadlineUtc'] = (dt.datetime.fromisoformat(ledger['simulationStartedUtc'])+dt.timedelta(seconds=300)).isoformat()
        entry = {'index': index, 'mode': mode, 'startedUtc': utc(), 'status': 'reserved-before-launch',
                 'reservedNativeSteps': 6240, 'manifestSha256': manifest['sha256'], 'output': str(output)}
        ledger['entries'].append(entry)
        ledger['historiesStarted'] += 1
        ledger['reservedNativeSteps'] += 6240
        save(ledger_path, ledger)
        remaining = (dt.datetime.fromisoformat(ledger['simulationDeadlineUtc'])-dt.datetime.now(dt.timezone.utc)).total_seconds()
        assert remaining > 0, 'Wall budget expired after irreversible reservation'
        env = dict(os.environ, LH_GETUP_AB_MODE='B', LH_FOOT_COORD_MODE=mode, LH_WHOLE_PROGRAM=value['program'],
                   LH_WHOLE_OUT=str(output), LH_WHOLE_SOLVER_TRACE='1',
                   LH_WHOLE_CONTROLLER_CONTROL='0', LH_WHOLE_IDLE_VALIDATION='0',
                   LH_WHOLE_HEADING_DEG='0', LH_WHOLE_PERTURB_DEG='0',
                   LH_WHOLE_WALL_S=str(remaining), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        command = [value['kernel'], '--exact',
                   'recovery::support_probe::whole_history::run_whole_reference_native_history',
                   '--ignored', '--nocapture', '--test-threads=1']
        began = time.monotonic()
        with (root / f'{index:03d}.log').open('w') as log:
            try:
                result = subprocess.run(command, cwd=REPO, env=env, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=remaining, check=False)
                entry.update(status='process-completed' if result.returncode == 0 else 'process-failed',
                             processExitCode=result.returncode)
            except subprocess.TimeoutExpired:
                entry['status'] = 'timeout-reservation-retained'
        entry.update(endedUtc=utc(), wallSeconds=time.monotonic()-began)
        report_path = output / 'report.json'
        if report_path.exists():
            report = json.loads(report_path.read_text())
            count = report['physicalSubsteps']
            assert 0 <= count <= 6240, 'History native reservation exceeded'
            entry.update(actualNativeSteps=count, reportSha256=sha(report_path),
                         fullRecoverySuccess=report['fullRecoverySuccess'], firstRefusal=report['firstRefusal'])
            ledger['actualNativeStepsKnown'] += count
        if entry['status'] != 'process-completed' or 'actualNativeSteps' not in entry:
            ledger['status'] = 'closed-uncertain-or-failed-process'
        elif index == 0:
            entry['witnessEquivalence'] = witness_equivalence(output, value['archivedWitness'])
            if not entry['witnessEquivalence']['equal']:
                ledger['status'] = 'closed-witness-not-neutral'
        elif index == 1:
            ledger['status'] = 'closed-complete-candidate' if entry['fullRecoverySuccess'] else 'closed-no-complete-recovery'
            ledger['closedUtc'] = utc()
        save(ledger_path, ledger)
        print(json.dumps(entry, indent=2))
        if ledger['status'] == 'closed-witness-not-neutral':
            raise SystemExit(1)
        if entry['status'] != 'process-completed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
