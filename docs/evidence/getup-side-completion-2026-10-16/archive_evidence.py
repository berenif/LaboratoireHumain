#!/usr/bin/env python3
"""Archive experiment outputs byte-for-byte; this never loads a physics engine."""
from pathlib import Path
import gzip,hashlib,json
repo=Path(__file__).resolve().parents[3];source=repo/'outputs/getup-side-completion-2026-10-16';destination=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
records=[]
for p in sorted(source.rglob('*')):
    if not p.is_file() or p.suffix=='.mp4':continue
    relative=p.relative_to(source);raw=p.read_bytes();compressed=p.suffix in ['.bin','.ndjson','.log'] or p.name=='reference-report.json'
    target=destination/relative
    if compressed:target=target.with_suffix(target.suffix+'.gz')
    target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(gzip.compress(raw,mtime=0) if compressed else raw)
    restored=gzip.decompress(target.read_bytes()) if compressed else target.read_bytes();assert restored==raw
    records.append(dict(source=str(p.relative_to(repo)),archive=str(target.relative_to(repo)),
        rawBytes=len(raw),rawSha256=sha(raw),archivedBytes=target.stat().st_size,archivedSha256=sha(target.read_bytes()),gzip=compressed))
(destination/'archives.json').write_text(json.dumps(dict(physicsIntegrations=0,files=records),indent=2)+'\n')
print(json.dumps(dict(files=len(records),rawBytes=sum(r['rawBytes'] for r in records),archivedBytes=sum(r['archivedBytes'] for r in records),verified=True,physicsIntegrations=0)))
