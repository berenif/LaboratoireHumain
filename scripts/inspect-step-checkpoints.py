#!/usr/bin/env python3
"""Read checkpoint envelopes only; never load or advance a Rapier world.
A matching envelope is merely a candidate, never an admission certificate.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct

LIMIT = 64 * 1024 * 1024
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('evidence', type=Path)
args = parser.parse_args()
model = (Path(__file__).resolve().parents[1] / 'rust/crates/model/data/getup-v1.json').read_bytes()
rows = []
for path in sorted(args.evidence.rglob('checkpoint*.bin.gz')):
    row = {'path': str(path), 'compressed_bytes': path.stat().st_size}
    try:
        with gzip.open(path, 'rb') as stream:
            raw = stream.read(LIMIT + 1)
        if len(raw) > LIMIT:
            raise ValueError('checkpoint exceeds 64 MiB')
        schema = struct.unpack_from('<I', raw)[0]
        offset = 4
        values = []
        for _ in range(3):
            length = struct.unpack_from('<Q', raw, offset)[0]
            offset += 8
            if length > len(raw) - offset:
                raise ValueError('invalid envelope length')
            values.append(raw[offset:offset + length])
            offset += length
        if offset != len(raw):
            raise ValueError('trailing bytes')
        auxiliary = json.loads(values[2])
        inner_schema = schema
        for _ in range(2):
            if inner_schema not in (3, 4):
                break
            inner_schema = auxiliary['inner_schema']
            auxiliary = json.loads(auxiliary['auxiliary'])
        if inner_schema == 2:
            auxiliary = json.loads(auxiliary['runtime'])
        elif inner_schema != 1:
            raise ValueError('unsupported inner schema')
        sim = auxiliary['simulation']
        profile = sim['profile']
        selector = profile.get('getup_native_locomotion')
        row.update(schema=schema, raw_sha256=hashlib.sha256(raw).hexdigest(),
                   model_matches=values[0] == model, profile=profile.get('id'),
                   native_selector=selector, native_substeps=sim.get('substeps_completed'),
                   native_state_present=sim.get('native_locomotion') is not None,
                   verdict='not-admitted-envelope-only')
        if selector != 'native-joint-owner-v1' or sim.get('native_locomotion') is None:
            row['reason'] = 'missing native owner selector/state; cannot continue earned Step'
        elif values[0] != model:
            row['reason'] = 'canonical model differs'
        else:
            row['reason'] = 'requires exact Rust restore and readiness validation without integration'
    except (ValueError, KeyError, struct.error, OSError) as error:
        row.update(verdict='invalid-envelope', reason=str(error))
    rows.append(row)
print(json.dumps({'scope': 'read-only envelope inspection; no Rapier deserialization or integration',
                  'count': len(rows), 'checkpoints': rows}, indent=2))
