#!/usr/bin/env python3
"""Archive closed native evidence losslessly; standard library only, no physics.

Raw files remain untouched. Files larger than 32 MiB are split into independent
32 MiB raw chunks (including every file >64 MiB), keeping each gzip below 50 MiB.
Restore by concatenating decompressed chunks in index order, without adding any
bytes at chunk boundaries. Never rerun over existing output or an incomplete lot.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import stat
import time

HERE = Path(__file__).resolve().parent
CHUNK_BYTES = 32 * 1024 * 1024
MAX_GZIP_BYTES = 50 * 1024 * 1024
IO_BYTES = 1024 * 1024
PATTERNS = ('*.states.jsonl', '*.json.accounting.jsonl',
            '000-native-verification.json', '*-planning-*.json')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def identity(path):
    value = path.lstat()
    require(stat.S_ISREG(value.st_mode), 'Expected regular, non-symlink file: ' + str(path))
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)


def digest(path):
    result, size = hashlib.sha256(), 0
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(IO_BYTES), b''):
            result.update(data)
            size += len(data)
    return result.hexdigest(), size


def closed_receipts(run_dir, ledger_path):
    result_path = run_dir / 'run-result.json'
    protected = {}
    values = []
    for path in (ledger_path, result_path):
        before = identity(path)
        raw = path.read_bytes()
        require(identity(path) == before, 'Receipt changed while reading: ' + str(path))
        protected[path] = (hashlib.sha256(raw).hexdigest(), len(raw), before)
        values.append(json.loads(raw))
    envelope, result = values
    state = envelope['payload']
    require(hashlib.sha256(canonical(state)).hexdigest() == envelope['sha256'], 'Ledger digest mismatch')
    require(not (ledger_path.parent / '.ledger.pending').exists(), 'Ledger write is incomplete')
    require(str(state['status']).startswith('closed'), 'Ledger is not closed')
    require(result.get('status') in ('protocol-completed', 'failed-closed'), 'Run result is not terminal')
    budget = result['budget']
    require(budget['id'] == state['id'] and budget['status'] == state['status'], 'Run and ledger identity/status disagree')
    require(budget.get('authorizesIntegration') is False, 'Run receipt still authorizes integration')
    require(budget['lastEvent'] == state['events'][-1], 'Run result does not describe final ledger event')
    expected = {}
    for item in state.get('reservations', []) + result.get('histories', []):
        record = item.get('outcome', item).get('evidence')
        if record:
            path = Path(record['path']).resolve()
            require(path not in expected or expected[path] == record['sha256'], 'Conflicting closed receipt hashes')
            expected[path] = record['sha256']
    return protected, expected, state['id'], state['status']


def archive(run_dir, ledger_path, index_path):
    started = time.monotonic()
    run_dir = Path(run_dir).resolve(strict=True)
    ledger_path, index_path = Path(ledger_path).absolute(), Path(index_path).absolute()
    require(not os.path.lexists(index_path), 'Index already exists: ' + str(index_path))
    protected, expected, budget_id, status = closed_receipts(run_dir, ledger_path)
    files = sorted({path for pattern in PATTERNS for path in run_dir.glob(pattern)})
    require(files, 'No matching native evidence files exist')
    plan = []
    for raw in files:
        before = identity(raw)
        size = before[2]
        count = max(1, (size + CHUNK_BYTES - 1) // CHUNK_BYTES)
        outputs = [raw.with_name(raw.name + ('.gz' if count == 1 else f'.part{n:05d}.gz'))
                   for n in range(count)]
        for packed in outputs:
            require(not os.path.lexists(packed), 'Archive output already exists: ' + str(packed))
        plan.append((raw, before, outputs))
    rows = []
    for raw, before, outputs in plan:
        raw_hash, raw_size = digest(raw)  # Independent raw pass before compression.
        require(raw_size == before[2] and identity(raw) == before, 'Raw artifact changed: ' + str(raw))
        receipt_hash = expected.get(raw.resolve())
        require(receipt_hash is None or receipt_hash == raw_hash, 'Raw artifact differs from closed receipt: ' + str(raw))
        compressed_input, restored_file = hashlib.sha256(), hashlib.sha256()
        parts, offset = [], 0
        with raw.open('rb') as source:
            for number, packed in enumerate(outputs):
                size = min(CHUNK_BYTES, raw_size - offset)
                chunk_hash, remaining = hashlib.sha256(), size
                with packed.open('xb') as destination:
                    with gzip.GzipFile(filename='', fileobj=destination, mode='wb',
                                       mtime=0, compresslevel=3) as compressed:
                        while remaining:
                            data = source.read(min(IO_BYTES, remaining))
                            require(data, 'Raw artifact truncated during compression: ' + str(raw))
                            compressed.write(data)
                            chunk_hash.update(data)
                            compressed_input.update(data)
                            remaining -= len(data)
                    destination.flush()
                    os.fsync(destination.fileno())
                packed_hash, packed_size = digest(packed)
                require(packed_size <= MAX_GZIP_BYTES, 'Gzip exceeds 50 MiB: ' + str(packed))
                restored_chunk, restored_size = hashlib.sha256(), 0
                with gzip.open(packed, 'rb') as restored:
                    for data in iter(lambda: restored.read(IO_BYTES), b''):
                        restored_chunk.update(data)
                        restored_file.update(data)
                        restored_size += len(data)
                require((restored_chunk.hexdigest(), restored_size) == (chunk_hash.hexdigest(), size),
                        'Gzip round trip differs from raw chunk: ' + str(packed))
                parts.append({'order': number, 'rawOffset': offset, 'rawBytes': size,
                              'rawSha256': chunk_hash.hexdigest(),
                              'gzipPath': os.path.relpath(packed, index_path.parent),
                              'gzipBytes': packed_size, 'gzipSha256': packed_hash,
                              'decompressedBytesAndHashVerified': True})
                offset += size
            require(not source.read(1), 'Raw artifact grew during compression: ' + str(raw))
        require(offset == raw_size and compressed_input.hexdigest() == raw_hash and
                restored_file.hexdigest() == raw_hash and identity(raw) == before,
                'Whole-file byte/hash/stability verification failed: ' + str(raw))
        rows.append({'rawPath': os.path.relpath(raw, index_path.parent), 'rawBytes': raw_size,
                     'rawSha256': raw_hash, 'closedReceiptHashAvailable': receipt_hash is not None,
                     'parts': parts, 'concatenatedDecompressionVerified': True})
    for path, (expected_hash, expected_size, before) in protected.items():
        require(digest(path) == (expected_hash, expected_size) and identity(path) == before,
                'Closed receipt changed during archival: ' + str(path))
    for raw, before, _ in plan:
        require(identity(raw) == before, 'Raw artifact changed before index publication: ' + str(raw))
    require(not (ledger_path.parent / '.ledger.pending').exists(), 'Ledger became incomplete during archival')
    index = {'schema': 1, 'scope': 'post-closure lossless compression; no worker, FK or physics',
             'nativeIntegrations': 0, 'budgetId': budget_id, 'closedBudgetStatus': status,
             'closedReceipts': [{'path': os.path.relpath(path, index_path.parent), 'sha256': value[0],
                                 'bytes': value[1]} for path, value in protected.items()],
             'compressionLevel': 3, 'maximumRawChunkBytes': CHUNK_BYTES,
             'maximumGzipBytes': MAX_GZIP_BYTES, 'rawFilesPreserved': True,
             'restore': 'Concatenate gzip decompressions in parts.order, without separators; verify rawBytes/rawSha256.',
             'elapsedSeconds': time.monotonic() - started, 'files': rows}
    with index_path.open('x') as stream:
        json.dump(index, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    directory = os.open(index_path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
    return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, default=HERE / 'native-run')
    parser.add_argument('--ledger', type=Path, default=HERE / 'budget' / 'ledger.json')
    parser.add_argument('--index', type=Path, default=HERE / 'archive-index.json')
    args = parser.parse_args()
    try:
        result = archive(args.run_dir, args.ledger, args.index)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, 'REFUSED: ' + str(exc) + '\n')
    print(json.dumps({'index': str(args.index), 'files': len(result['files']),
                      'gzipParts': sum(len(row['parts']) for row in result['files']),
                      'nativeIntegrations': 0, 'elapsedSeconds': result['elapsedSeconds']}))


if __name__ == '__main__':
    main()
