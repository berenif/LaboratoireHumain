#!/usr/bin/env python3
"""Freeze the one parent-authorized contact diagnostic; never launch a process."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASE = 'a5c8ff36af4dfbe784277aebc91021040b947a52'
WORKER = ROOT / 'rust/crates/sim/src/recovery/planner/transfer_worker.rs'
WORKER_SHA = '811df09bae544476e03026a9d832f7ea306ff774fe85e42929f974efd2c3f89c'
OLD = ROOT / 'docs/evidence/coordination-transfer-native-7100-2026-10-15'
spec = importlib.util.spec_from_file_location('diagnostic_supervisor_freeze', HERE / 'run_diagnostic.py')
supervisor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supervisor)


def read(path):
    return json.loads(path.read_text())


def main():
    destinations = [HERE / name for name in
                    ('manifest.json', 'authorization.json', 'freeze-receipt.json')]
    assert not any(path.exists() for path in destinations), 'Do not overwrite a freeze'
    assert not (HERE / 'authorization-claim.json').exists()
    assert not (HERE / 'native-run-once').exists()
    assert supervisor.sha(WORKER) == WORKER_SHA
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
    reservation = read(HERE / 'budget-reservation.json')
    assert reservation['baseCommit'] == BASE and reservation['singleUse'] is True
    assert reservation['nativeBudget']['newSingleReservationSteps'] == 568
    assert reservation['wallBudget']['newProvisionSeconds'] == 120
    assert reservation['nativeBudget']['chargedAfterReservation'] == 139508
    assert datetime.now(timezone.utc) < datetime.fromisoformat(
        reservation['preparation']['activeWindowDeadlineUtc'])
    release = read(HERE / 'release-admitted-receipt.json')
    assert release['exitCode'] == 0 and release['workerExecuted'] is False
    assert release['candidateSourceStable'] and release['historicalReleasePreserved']
    assert release['workerSourceBefore'] == release['workerSourceAfter'] == WORKER_SHA
    executable, = release['executables']
    binary = Path(executable['path']).resolve(strict=True)
    assert supervisor.sha(binary) == executable['sha256']
    tests = read(HERE / 'worker-pure-tests-receipt.json')
    assert tests['allPassed'] and tests['testCount'] == 12 and tests['nativeSteps'] == 0
    assert tests['sourceSha256'] == WORKER_SHA
    assert 'Ran 54 tests' in (HERE / 'supervisor-tests-final.log').read_text()
    assert (HERE / 'supervisor-tests-final.log').read_text().rstrip().endswith('OK')
    records = {}

    def add(path, role, expected=None):
        path = path.resolve(strict=True)
        actual = supervisor.sha(path)
        if expected is not None:
            assert actual == expected, str(path)
        if str(path) in records:
            assert records[str(path)]['sha256'] == actual
            return
        records[str(path)] = {'path': str(path), 'sha256': actual, 'role': role}

    old = read(OLD / 'manifest.json')
    for row in old['files']:
        path = Path(row['path'])
        role = 'validation' if row['role'] == 'binary' else row['role']
        if role == 'code':
            relative = path.relative_to(ROOT)
            baseline = subprocess.check_output(['git', 'cat-file', 'blob', f'{BASE}:{relative}'], cwd=ROOT)
            if path == WORKER:
                assert supervisor.sha(path) == WORKER_SHA
            else:
                assert hashlib.sha256(baseline).hexdigest() == supervisor.sha(path), str(path)
            add(path, role)
        else:
            add(path, role, row['sha256'])
    for path_string in supervisor.required_code_paths():
        path = Path(path_string)
        if path.is_relative_to(ROOT / 'rust') and path != WORKER:
            relative = path.relative_to(ROOT)
            baseline = subprocess.check_output(['git', 'cat-file', 'blob', f'{BASE}:{relative}'], cwd=ROOT)
            assert hashlib.sha256(baseline).hexdigest() == supervisor.sha(path), str(path)
        add(path, 'code')
    audit = read(ROOT / 'docs/evidence/coordination-contact-continuity-2026-10-15/final-budget-and-integrity.json')
    for row in audit['immutableEvidence']['checks']:
        add(ROOT / row['path'], 'validation', row['expectedSha256'])
    assert len(audit['immutableEvidence']['checks']) == 42
    add(binary, 'binary', executable['sha256'])
    for path in HERE.glob('*.py'):
        add(path, 'code')
    for name in ('protocol.md', 'supervisor-contract.json', 'supervisor-readme.md',
                 'parent-authorization-provenance.json'):
        add(HERE / name, 'protocol')
    add(HERE / 'budget-reservation.json', 'preparation')
    for name in ('release-admitted-receipt.json', 'release-admitted.jsonl', 'release-admitted.stderr.log',
                 'pure-worker-test-admitted-receipt.json', 'worker-pure-tests.log',
                 'worker-pure-tests-receipt.json', 'supervisor-validation.json',
                 'supervisor-tests-final.log', 'prelaunch-budget-review.json',
                 'prelaunch-budget-review.md', 'wall-source-reconciliation.json',
                 'readiness-review.json', 'readiness-review.md', 'diagnostic-worker-review.json',
                 'diagnostic-worker-review.md'):
        add(HERE / name, 'validation')
    previous = ROOT / 'docs/evidence/coordination-contact-continuity-2026-10-15'
    for name in ('implementation-review.json', 'implementation-review.md', 'publication-check.json',
                 'continuity-archive-36-78.json', 'first-pure-tests/receipt.json',
                 'fixture-fix-pure-tests/receipt.json'):
        add(previous / name, 'validation')
    manifest = {'schema': supervisor.MANIFEST_SCHEMA, 'protocolId': supervisor.PROTOCOL,
                'authorizationId': reservation['authorizationId'], 'binaryPath': str(binary),
                'outputPath': str(HERE / 'native-run-once'), 'inputs': old['inputs'],
                'files': sorted(records.values(), key=lambda row: (row['role'], row['path'])),
                'prerequisites': {name: True for name in supervisor.PREREQUISITES}}
    supervisor.write_new(destinations[0], manifest)
    authorization = {'schema': supervisor.AUTHORIZATION_SCHEMA, 'protocolId': supervisor.PROTOCOL,
                     'authorizationId': reservation['authorizationId'], 'approved': True,
                     'approvalEvidence': 'Verbatim explicit parent authorization from source thread '
                        '01a12799-cefe-7764-8aa3-db5db4c1d0f3, SHA256=' +
                        supervisor.sha(HERE / 'parent-authorization-provenance.json') +
                        '; one global568 reservation SHA256=' + supervisor.sha(HERE / 'budget-reservation.json'),
                     'approvedAtUtc': datetime.now(timezone.utc).isoformat(),
                     'manifestSha256': supervisor.sha(destinations[0]), 'singleUse': True,
                     'reopenHistoricalFamily': False, 'limits': supervisor.LIMITS}
    supervisor.write_new(destinations[1], authorization)
    prepared = supervisor.preflight(destinations[0], destinations[1])
    receipt = {'schema': 1, 'status': 'frozen-authorized-awaiting-independent-admission',
               'createdUtc': datetime.now(timezone.utc).isoformat(), 'baseCorrectionCommit': BASE,
               'workerSourceSha256': WORKER_SHA, 'manifestSha256': supervisor.sha(destinations[0]),
               'authorizationSha256': supervisor.sha(destinations[1]), 'binarySha256': supervisor.sha(binary),
               'frozenFilesVerified': len(prepared['records']),
               'roles': dict(Counter(row['role'] for row in records.values())),
               'immutableHistoricalPiecesVerified': 42, 'parentReservedNativeSteps': 568,
               'maximumWallSecondsAllInclusive': 120, 'nativeSteps': 0, 'workerStarted': False,
               'claimCreated': False, 'automaticRenewalAllowed': False,
               'budgetRelationship': '52+516 runtime allocations subdivide the existing single568 reservation'}
    supervisor.write_new(destinations[2], receipt)
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
