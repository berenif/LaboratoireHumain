#!/usr/bin/env python3
"""Extract 36 complete archived rows; no model, FK, world or integration."""
from pathlib import Path
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT / 'docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-extract.py'


def write_once(path, data):
    if path.exists():
        if path.read_bytes() != data:
            raise RuntimeError(f'Different prior extraction: {path}')
    else:
        path.write_bytes(data)


def main():
    spec = importlib.util.spec_from_file_location('archived_entry_extraction', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows, source = module.local_rows('native-run/001-A.states.jsonl')
    frames = rows[:36]
    assert [r['nativeSubstep'] for r in frames] == list(range(1, 37))
    entry_path = ROOT / 'docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json'
    entry_bytes = entry_path.read_bytes()
    assert frames[-1] == json.loads(entry_bytes)['archivedNativeRow']
    value = {'schema': 1, 'source': source, 'prefixFrames': frames,
             'entrySha256': hashlib.sha256(entry_bytes).hexdigest(),
             'exclusions': ['kind label only', 'new owner.transfer must be absent or null'],
             'nativeSteps': 0, 'fkCalls': 0, 'worldsConstructed': 0}
    data = (json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n').encode()
    path = OUT / 'worker-prefix-frames.json'
    write_once(path, data)
    receipt = {'schema': 1, 'path': str(path.relative_to(ROOT)), 'bytes': len(data),
               'sha256': hashlib.sha256(data).hexdigest(), 'rows': 36,
               'source': source, 'extractorSha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               'sourceExtractorSha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
               'nativeSteps': 0, 'fkCalls': 0, 'worldsConstructed': 0}
    write_once(OUT / 'worker-prefix-receipt.json', (json.dumps(receipt, indent=2) + '\n').encode())
    print(json.dumps({k: receipt[k] for k in ('path', 'bytes', 'sha256', 'rows')}))


if __name__ == '__main__':
    main()
