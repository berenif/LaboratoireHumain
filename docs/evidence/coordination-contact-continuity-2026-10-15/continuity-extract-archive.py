#!/usr/bin/env python3
"""Extract archived supports only. No worker/model/world/FK/native calls."""
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT / 'docs/evidence/coordination-transfer-native-7100-2026-10-15/native-run-once/native-trace.jsonl.gz'
raw = SOURCE.read_bytes()
rows = [json.loads(line) for line in gzip.decompress(raw).splitlines()]
entry = next(row for row in rows if row.get('nativeSubstep') == 36 and 'support' in row)
end = next(row for row in rows if row.get('nativeSubstep') == 78 and row.get('kind') == 'preview-transfer-end')
patch = next(p for p in end['owner']['transfer']['decision']['tracker']['active'] if p['segment'] == 19)
contacts = entry['support']['segments'][19]['contacts']
index = next(i for i, c in enumerate(contacts) if c['local_point_m'] == patch['local_point_m'])
fixture = {'schema': 1, 'source': {'path': str(SOURCE.relative_to(ROOT)), 'sha256': hashlib.sha256(raw).hexdigest()},
           'scope': 'Immutable archived A36 and failed forecast78 support observations; no replay',
           'acquisition': entry['support'], 'failed': end['support'],
           'segment': 19, 'acquiredContactIndex': index,
           'archivedPatchBeforeRefusedObservation': patch,
           'nativeSteps': 0, 'worlds': 0, 'fkPoses': 0}
data = (json.dumps(fixture, ensure_ascii=False, sort_keys=True, indent=2) + '\n').encode()
path = OUT / 'continuity-archive-36-78.json'
if path.exists():
    assert path.read_bytes() == data, 'Refuse to overwrite a different extracted archive'
else:
    with path.open('xb') as file:
        file.write(data)
print(json.dumps({'path': str(path.relative_to(ROOT)), 'sha256': hashlib.sha256(data).hexdigest(),
                  'bytes': len(data), 'nativeSteps': 0, 'worlds': 0, 'fkPoses': 0}))
