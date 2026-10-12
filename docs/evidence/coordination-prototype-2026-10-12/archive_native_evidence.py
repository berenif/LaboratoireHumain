#!/usr/bin/env python3
"""Lossless archival only; never imports the simulator or changes original evidence."""
import gzip,hashlib,json,pathlib,shutil
root=pathlib.Path(__file__).resolve().parent/'native-run'
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
items=[]
for source in sorted(root.iterdir()):
 if source.suffix not in ('.json','.jsonl') or source.stat().st_size<1_000_000:continue
 target=source.with_name(source.name+'.gz')
 with source.open('rb') as s,target.open('xb') as t:
  with gzip.GzipFile(filename='',mode='wb',fileobj=t,compresslevel=6,mtime=0) as z:shutil.copyfileobj(s,z)
 h=hashlib.sha256()
 with gzip.open(target,'rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 original=digest(source)
 if h.hexdigest()!=original:raise RuntimeError('Archive failed roundtrip: '+str(source))
 items.append({'original':source.name,'originalBytes':source.stat().st_size,'originalSha256':original,'archive':target.name,'archiveBytes':target.stat().st_size,'archiveSha256':digest(target),'roundtripVerified':True})
(root/'archive-index.json').write_text(json.dumps({'scope':'lossless compression of already archived native evidence; zero integrations','files':items,'restore':'gzip -dk FILE.gz; verify originalSha256. Original bytes remain in this workspace.'},indent=2)+'\n')
(root/'.gitignore').write_text('# Raw originals are retained locally; versioned gzip archives restore identical bytes.\n'+''.join('/'+x['original']+'\n' for x in items))
print(json.dumps({'archives':len(items),'originalBytes':sum(x['originalBytes']for x in items),'archiveBytes':sum(x['archiveBytes']for x in items)}))
