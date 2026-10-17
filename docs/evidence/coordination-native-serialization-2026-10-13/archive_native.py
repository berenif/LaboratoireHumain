#!/usr/bin/env python3
"""Losslessly archive already closed native evidence; no worker or simulation."""
from pathlib import Path
import gzip
import hashlib
import json
import time

ROOT = Path(__file__).resolve().parent

def digest(path, compressed=False):
    value = hashlib.sha256()
    size = 0
    opener = gzip.open if compressed else open
    with opener(path, 'rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(part)
            size += len(part)
    return value.hexdigest(), size

def main():
    started = time.monotonic()
    directory = ROOT / 'native-run'
    ledger = json.loads((directory / 'physical-ledger.json').read_text())['payload']
    if ledger['status'] not in ('closed-success', 'closed-failure'):
        raise SystemExit('Archive requires the already closed, distinct native ledger')
    rows = []
    for name, key in [('native-result.json', 'evidence'), ('native-trace.jsonl', 'trace')]:
        raw = directory / name
        if not raw.exists():
            continue
        raw_hash, raw_size = digest(raw)
        receipt = ledger.get(key)
        if receipt is not None and raw_hash != receipt['sha256']:
            raise SystemExit('Raw artifact differs from the closed native receipt: ' + name)
        packed = raw.with_name(raw.name + '.gz')
        with raw.open('rb') as source, packed.open('xb') as destination:
            with gzip.GzipFile(filename='', fileobj=destination, mode='wb', mtime=0, compresslevel=6) as stream:
                for part in iter(lambda: source.read(1024 * 1024), b''):
                    stream.write(part)
        if digest(packed, compressed=True) != (raw_hash, raw_size):
            raise SystemExit('Lossless validation failed: ' + name)
        rows.append({'rawPath': str(raw), 'rawBytes': raw_size, 'rawSha256': raw_hash,
                     'gzipPath': str(packed), 'gzipBytes': packed.stat().st_size,
                     'gzipSha256': digest(packed)[0], 'decompressedBytesAndHashVerified': True,
                     'closedReceiptHashAvailable': receipt is not None})
    index = {'schema': 1, 'scope': 'post-run lossless compression only', 'nativeIntegrations': 0,
             'closedLedgerSha256': digest(directory / 'physical-ledger.json')[0],
             'elapsedSeconds': time.monotonic() - started, 'files': rows}
    with (ROOT / 'archive-index.json').open('x') as out:
        json.dump(index, out, indent=2)
        out.write('\n')
    print(json.dumps(index))

if __name__ == '__main__':
    main()
