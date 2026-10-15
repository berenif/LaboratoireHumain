#!/usr/bin/env python3
"""Freeze this prepared candidate once. Reads a binary, never executes it.

This creates an explicitly UNAPPROVED authorization template. The source of
any future approval must be the parent's distinct decision, never this script.
Run from the repository root after the dedicated --no-run release build.
"""
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
RESUME = REPO / 'docs/evidence/coordination-transfer-resume-2026-10-15'
CANDIDATE = REPO / 'docs/evidence/coordination-transfer-candidate-2026-10-13'
PROTOTYPE = REPO / 'docs/evidence/coordination-prototype-2026-10-12'
spec = importlib.util.spec_from_file_location('transfer_freeze_supervisor', HERE / 'run_transfer.py')
supervisor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supervisor)


def main():
    outputs = [HERE / name for name in
               ('manifest.json', 'authorization-template.json', 'freeze-receipt.json')]
    if any(path.exists() for path in outputs):
        raise SystemExit('A freeze output already exists; do not silently replace a frozen candidate')
    binary_receipt = json.loads((RESUME / 'release-binary.json').read_text())
    binary = Path(binary_receipt['binaryPath'])
    if (not binary_receipt['buildOnly'] or binary_receipt['executions'] != 0
            or supervisor.sha(binary) != binary_receipt['binarySha256']):
        raise SystemExit('Dedicated build-only binary receipt mismatch')
    records = {}

    def add(path, role):
        path = path.resolve(strict=True)
        key = str(path)
        if key in records:
            if records[key]['role'] != role:
                raise ValueError('Conflicting frozen file roles: ' + key)
            return
        records[key] = {'path': key, 'sha256': supervisor.sha(path), 'role': role}

    for path in supervisor.required_code_paths():
        add(Path(path), 'code')
    for path in HERE.glob('*.py'):
        add(path, 'code')
    add(binary, 'binary')
    inputs = {
        'programPath': PROTOTYPE / 'continuation-candidate-program.json',
        'tasksPath': PROTOTYPE / 'continuation-tasks.json',
        'entryPath': CANDIDATE / 'observed-entry-step36.json',
        'prefixFramesPath': HERE / 'worker-prefix-frames.json',
    }
    for key, path in inputs.items():
        add(path, supervisor.INPUT_ROLES[key])
    add(CANDIDATE / 'observed-entry-task-state.json', 'entry')
    for path in (REPO / 'rust/crates/model/data').glob('*.json'):
        add(path, 'model')
    add(REPO / 'rust/crates/sim/data/recovery-profile-v24.json', 'profile')
    add(REPO / 'rust/crates/sim/data/recovery-v1.json', 'criteria')
    for path in (CANDIDATE / 'preparation-reservation.json', RESUME / 'preparation-reservation.json'):
        add(path, 'preparation')
    for name in ('future-native-protocol.json', 'future-native-protocol.md', 'supervisor-readme.md'):
        add(HERE / name, 'protocol')
    for path in HERE.glob('*.log'):
        add(path, 'validation')
    for name in ('supervisor-validation.json', 'worker-prefix-receipt.json', 'prerequisite-evidence.json'):
        add(HERE / name, 'validation')
    for path in RESUME.glob('*.log'):
        add(path, 'validation')
    for name in ('review.md', 'review-evidence.json', 'release-binary.json', 'integrity-audit.json'):
        add(RESUME / name, 'validation')
    for path in CANDIDATE.rglob('*.log'):
        add(path, 'validation')
    for name in ('archived-a36-command-result.json', 'support-threshold-provenance.md',
                 'observed-entry-review.md', 'transfer-adapter-pure-validation.json',
                 'transfer-technical-pure-validation.json'):
        add(CANDIDATE / name, 'validation')

    identifier = 'transfer-a36-distinct-2026-10-15-single-history'
    manifest = {
        'schema': supervisor.MANIFEST_SCHEMA, 'protocolId': supervisor.PROTOCOL,
        'authorizationId': identifier, 'binaryPath': str(binary),
        'outputPath': str(HERE / 'native-run-once'),
        'inputs': {key: str(path) for key, path in inputs.items()},
        'files': sorted(records.values(), key=lambda item: (item['role'], item['path'])),
        'prerequisites': {key: True for key in supervisor.PREREQUISITES},
    }
    supervisor.write_new(outputs[0], manifest)
    checked = supervisor.preflight(outputs[0], require_authorization=False)
    template = {
        'schema': supervisor.AUTHORIZATION_SCHEMA, 'protocolId': supervisor.PROTOCOL,
        'authorizationId': identifier, 'approved': False,
        'approvalEvidence': '', 'approvedAtUtc': None,
        'manifestSha256': supervisor.sha(outputs[0]), 'singleUse': True,
        'reopenHistoricalFamily': False, 'limits': supervisor.LIMITS,
    }
    supervisor.write_new(outputs[1], template)
    receipt = {
        'schema': 1, 'status': 'frozen-awaiting-distinct-native-authorization',
        'createdUtc': datetime.now(timezone.utc).isoformat(),
        'manifestPath': str(outputs[0]), 'manifestSha256': supervisor.sha(outputs[0]),
        'authorizationTemplatePath': str(outputs[1]),
        'authorizationTemplateSha256': supervisor.sha(outputs[1]),
        'authorizationApproved': False, 'inspectionPassed': True,
        'inspectionGrantsNativePermission': False, 'workerExecuted': False,
        'newWorlds': 0, 'fkEvaluations': 0, 'nativeSteps': 0,
        'filesVerified': len(checked['records']),
        'roles': dict(sorted(Counter(item['role'] for item in records.values()).items())),
        'historicalProtectedFilesVerified': len(checked['protected']),
        'binarySha256': supervisor.sha(binary),
        'preparedLimitNativeSteps': supervisor.TOTAL,
        'preparedLimitWallSeconds': supervisor.WALL_SECONDS,
    }
    supervisor.write_new(outputs[2], receipt)
    print(json.dumps(receipt, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
