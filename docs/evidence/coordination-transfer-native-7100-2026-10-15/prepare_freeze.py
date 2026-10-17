#!/usr/bin/env python3
"""Materialize the explicitly parent-authorized7100-second freeze, without launch."""
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
OLD = REPO / 'docs/evidence/coordination-transfer-native-proposal-2026-10-13'
spec = importlib.util.spec_from_file_location('authorized_transfer_supervisor', HERE / 'run_transfer.py')
supervisor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supervisor)


def main():
    destinations = [HERE / name for name in ('manifest.json', 'authorization.json', 'freeze-receipt.json')]
    assert not any(path.exists() for path in destinations), 'Freeze already exists; do not overwrite'
    assert not (OLD / 'authorization-claim.json').exists()
    assert not (OLD / 'native-run-once').exists()
    assert not (HERE / 'authorization-claim.json').exists()
    assert not (HERE / 'native-run-once').exists()
    assert supervisor.sha(OLD / 'manifest.json') == '4f3708fd50cd4dabf473b7a6bff528ee11ad798cf25dfbeaf0f6141c156818f7'
    old = json.loads((OLD / 'manifest.json').read_text())
    assert supervisor.sha(Path(old['binaryPath'])) == '63e0e035d38d02f762c69b9dacb65d61e179c72ced50f5facd0bc29c64b9e8b5'
    authority = json.loads((HERE / 'parent-authorization-provenance.json').read_text())
    assert authority['maxWallSeconds'] == 7100 and authority['maximumHistories'] == 1
    assert authority['maxNativeSteps'] == 131096 and authority['noAdditionalConfirmationRequired'] is True
    records = {item['path']: item for item in old['files']}
    for item in records.values():
        assert supervisor.sha(Path(item['path'])) == item['sha256'], item['path']

    def add(path, role):
        path = path.resolve(strict=True)
        assert str(path) not in records
        records[str(path)] = {'path': str(path), 'sha256': supervisor.sha(path), 'role': role}

    for name in ('run_transfer.py', 'test_run_transfer.py', 'prepare_freeze.py'):
        add(HERE / name, 'code')
    for name in ('protocol.json', 'protocol.md', 'supervisor-readme.md', 'parent-authorization-provenance.json'):
        add(HERE / name, 'protocol')
    for name in ('deadline-validation.json', 'deadline-validation.log', 'budget-admission-review.json', 'budget-admission-review.md'):
        add(HERE / name, 'validation')
    add(HERE / 'preparation-reservation.json', 'preparation')
    add(OLD / 'manifest.json', 'validation')
    identifier = 'transfer-a36-distinct-2026-10-15-7100-single-history'
    manifest = {**old, 'protocolId': supervisor.PROTOCOL, 'authorizationId': identifier,
                'outputPath': str(HERE / 'native-run-once'),
                'files': sorted(records.values(), key=lambda item: (item['role'], item['path']))}
    supervisor.write_new(destinations[0], manifest)
    authorization = {
        'schema': supervisor.AUTHORIZATION_SCHEMA, 'protocolId': supervisor.PROTOCOL,
        'authorizationId': identifier, 'approved': True,
        'approvalEvidence': 'Explicit parent delegation recorded in parent-authorization-provenance.json; SHA256=' + supervisor.sha(HERE / 'parent-authorization-provenance.json') + '; source thread01a12799-cefe-7764-8aa3-db5db4c1d0f3 authorizes reduction, new freeze and single launch without another round trip.',
        'approvedAtUtc': datetime.now(timezone.utc).isoformat(),
        'manifestSha256': supervisor.sha(destinations[0]),
        'singleUse': True, 'reopenHistoricalFamily': False, 'limits': supervisor.LIMITS,
    }
    supervisor.write_new(destinations[1], authorization)
    prepared = supervisor.preflight(destinations[0], destinations[1])
    receipt = {
        'schema': 1, 'status': 'frozen-parent-authorized-not-yet-executed',
        'createdUtc': datetime.now(timezone.utc).isoformat(),
        'manifestSha256': supervisor.sha(destinations[0]),
        'authorizationSha256': supervisor.sha(destinations[1]),
        'supervisorSha256': supervisor.sha(HERE / 'run_transfer.py'),
        'binarySha256': supervisor.sha(prepared['binary']),
        'predecessorManifestSha256': supervisor.sha(OLD / 'manifest.json'),
        'frozenFilesVerified': len(prepared['records']),
        'roles': dict(sorted(Counter(item['role'] for item in records.values()).items())),
        'historicalProtectedFilesVerified': len(prepared['protected']),
        'staticPreflightPassed': True, 'authorizationVerified': True,
        'workerStarted': False, 'nativeSteps': 0, 'physicalClaimCreated': False,
        'maximumNativeSteps': supervisor.TOTAL, 'maximumWallSeconds': supervisor.WALL_SECONDS,
        'newCandidateCount': 1, 'automaticRenewalAllowed': False,
    }
    supervisor.write_new(destinations[2], receipt)
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
