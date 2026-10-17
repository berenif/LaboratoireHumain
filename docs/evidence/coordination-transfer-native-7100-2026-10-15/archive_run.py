#!/usr/bin/env python3
"""Losslessly archive existing closed-run bytes. No physics or state reconstruction."""
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
RUN = HERE / 'native-run-once'


def digest(stream):
    result = hashlib.sha256()
    while block := stream.read(1024 * 1024):
        result.update(block)
    return result.hexdigest()


def main():
    ledger = json.loads((RUN / 'physical-ledger.json').read_text())['payload']
    assert ledger['status'] in ('closed-refused', 'closed-success')
    assert ledger['actualNativeStepsKnown'] == 130
    receipts = []
    for name in ('native-result.json', 'native-trace.jsonl'):
        source = RUN / name
        target = RUN / (name + '.gz')
        with source.open('rb') as stream:
            before = digest(stream)
        if name == 'native-result.json':
            assert before == ledger['evidence']['sha256']
        with source.open('rb') as src, target.open('xb') as dst:
            with gzip.GzipFile(filename='', mode='wb', fileobj=dst, compresslevel=9, mtime=0) as compressed:
                shutil.copyfileobj(src, compressed, 1024 * 1024)
        with gzip.open(target, 'rb') as expanded:
            assert digest(expanded) == before
        with source.open('rb') as unchanged:
            assert digest(unchanged) == before
        with target.open('rb') as stream:
            compressed_digest = digest(stream)
        receipts.append({'originalPath': str(source), 'originalBytes': source.stat().st_size,
                         'originalSha256': before, 'archivePath': str(target),
                         'archiveBytes': target.stat().st_size, 'archiveSha256': compressed_digest,
                         'roundTripBytesVerified': True, 'originalRetainedLocally': True})
    receipt = {'schema': 1, 'createdUtc': datetime.now(timezone.utc).isoformat(),
               'scope': 'Lossless compression of existing closed-run bytes only',
               'nativeStepsDuringArchiving': 0, 'newWorlds': 0, 'fkEvaluations': 0,
               'files': receipts}
    with (HERE / 'archive-index.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
        stream.write('\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
