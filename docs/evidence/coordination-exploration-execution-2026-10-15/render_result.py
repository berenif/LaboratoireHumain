#!/usr/bin/env python3
"""Reuse archived-pose renderer; no Simulation, FK or physics execution."""
import csv,gzip,hashlib,importlib.util,json
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[2]
p=ROOT/'docs/evidence/coordination-transfer-native-7100-2026-10-15/render_actual.py'
spec=importlib.util.spec_from_file_location('existing_render',p); render=importlib.util.module_from_spec(spec);spec.loader.exec_module(render)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
closed=json.loads((HERE/'closed.json').read_text());assert closed['status']=='closed'
frames={}; rows=[]; failures=[]
with (HERE/'native-trace.jsonl').open() as stream:
 for line in stream:
  r=json.loads(line); kind=r.get('kind'); step=r.get('nativeSubstep')
  if kind=='actual-history':
   r['com']=r['support']['center_of_mass_m']
   r['pelvisHeightM']=r['support']['bodies'][0]['position_m'][1]-r['support']['support_height_m']
   r['segmentContacts']=r['support']['segments']
  if kind in ('actual-history','exploration-step'):
   rows.append(r)
  if kind=='exploration-failure':failures.append(r)
  if kind in ('actual-history','exploration-step','exploration-failure') and r.get('snapshot') is not None:
   frames[step]={'step':step,'snapshot':r['snapshot'],'source':kind,'com':r.get('com'),'diagnostic':r.get('localDiagnostic')}
track={'name':'main-history','label':'Exploration réelle A36 — séquence complète, aucune qualification','frames':[frames[k] for k in sorted(frames)],'refusals':failures}
out=HERE/'rendered';out.mkdir(exist_ok=False)
model=json.loads((ROOT/'rust/crates/model/data/canonical-v1.json').read_text())
assert model['segments'][0]['id']=='pelvis'
recipe=render.render([track],model['segments'],out,3,True)
fields=['native_substep','time_s','pelvis_height_m','left_heel_reported_n','right_heel_reported_n','support_fraction','com_inside','capture_inside','standing_pose_diagnostic','loaded_segments']
with (out/'measurements.csv').open('x') as f:
 writer=csv.writer(f);writer.writerow(fields)
 for r in rows:
  d=r['localDiagnostic'];loaded=[s['id'] for s in r['segmentContacts'] if sum(c['current_upward_load_n'] for c in s['contacts'])>=3]
  writer.writerow([r['nativeSubstep'],r['nativeSubstep']/240,r['pelvisHeightM'],*d['heel_loads_n'],d['support_fraction'],d['com_inside'],d['capture_inside'],r.get('standingPoseDiagnostic'),';'.join(loaded)])
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
t=[r['nativeSubstep']/240 for r in rows]
axes[0].plot(t,[r['pelvisHeightM'] for r in rows]);axes[0].set_ylabel('Hauteur bassin (m)')
for i,label in enumerate(('Talon gauche','Talon droit')):axes[1].plot(t,[r['localDiagnostic']['heel_loads_n'][i] for r in rows],label=label)
axes[1].legend();axes[1].set_ylabel('Charge rapportée (N)')
axes[2].plot(t,[int(r['localDiagnostic']['capture_inside']) for r in rows],label='Point de capture dans appuis');axes[2].plot(t,[r['localDiagnostic']['support_fraction'] for r in rows],label='Fraction de soutien')
axes[2].legend();axes[2].set_xlabel('Temps natif (s)')
fig.suptitle('Mesures réelles — charges publiées par le solveur, pas forces continues');fig.tight_layout();fig.savefig(out/'measurements.png');plt.close(fig)
summary={'nativeSteps':closed['workerResult'].get('actualNativeSteps'),'durationS':closed['workerResult'].get('simulatedSeconds'),
 'pelvisHeightStartM':rows[0]['pelvisHeightM'],'pelvisHeightA36M':next(r['pelvisHeightM'] for r in rows if r['nativeSubstep']==36),
 'pelvisHeightLastObservedM':rows[-1]['pelvisHeightM'],'pelvisHeightMaxM':max(r['pelvisHeightM'] for r in rows),
 'firstCaptureOutsideAfterA36':next((r['nativeSubstep'] for r in rows if r['nativeSubstep']>36 and not r['localDiagnostic']['capture_inside']),None),
 'firstStandingPoseDiagnostic':next((r['nativeSubstep'] for r in rows if r.get('standingPoseDiagnostic')),None),
 'heelLoadMaximumN':[max(r['localDiagnostic']['heel_loads_n'][i] for r in rows) for i in (0,1)],
 'firstPhysicalFailure':failures[0].get('failure') if failures else None,'recipe':recipe,'additionalPhysicsSteps':0,'qualified':False,
 'traceSha256':sha(HERE/'native-trace.jsonl'),'outputsSha256':{p.name:sha(p) for p in out.iterdir() if p.is_file()}}
(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
with (HERE/'native-trace.jsonl').open('rb') as source, (HERE/'native-trace.jsonl.gz').open('xb') as target:
 with gzip.GzipFile(filename='',mode='wb',fileobj=target,mtime=0) as gz:
  import shutil;shutil.copyfileobj(source,gz)
print(json.dumps({k:v for k,v in summary.items() if k not in ('recipe','outputsSha256')},ensure_ascii=False))
