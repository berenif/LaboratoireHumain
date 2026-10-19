import json,pathlib,sys,copy,hashlib
D=pathlib.Path('outputs/getup-terminal-balance-20261019');B=pathlib.Path('outputs/getup-recovery-root-20261019')
results=[]
for kind in sys.argv[1:] or ['back','belly','side','side-supported']:
 c=json.loads((D/kind/'result.json').read_text());report=json.loads((D/kind/'reference-report.json').read_text()); old=json.loads((B/kind/'reference-report.json').read_text());binding=c['terminalBalance'];cut=binding['activated_native_substep'] if binding else c['realNativeIntegrations']
 def clean(row):
  row=copy.deepcopy(row);d=row['snapshot']['diagnostics'];d.pop('controller_ms');d.pop('update_ms')
  # Before triggering the test fixture, the experimental Ready diagnostic may
  # differ without any altered body/control/actor state. This is checked separately.
  if row['triggerNative'] is None or row['fixture']['nativeSubsteps']==row['triggerNative']:
   d['getup'].pop('controller');d['getup']['policy'].pop('phase')
  return row
 base=[json.loads(l) for l in (B/kind/'trace.ndjson').open()];candidate=[json.loads(l) for l in (D/kind/'trace.ndjson').open()]; base={(r['fixture']['nativeSubsteps'],r['triggerNative']):r for r in base};compared=0
 for r in candidate:
  n=r['fixture']['nativeSubsteps']
  if n>cut:break
  key=(n,r['triggerNative']);a=clean(base[key]);b=clean(r)
  # Binding happens after this native integration, so the authoritative ID may
  # already expose a transfer whose first changed command is the next step.
  if n==cut and binding:
   a['snapshot']['diagnostics']['getup'].pop('controller',None);b['snapshot']['diagnostics']['getup'].pop('controller',None)
  assert a==b, f'{kind} full normalized emitted packet differs at {n}'
  compared+=1
 decisions=report['student_decisions'];assert decisions==old['student_decisions'][:len(decisions)],kind+' SLOW archive prefix differs'
 # WEAK physical snapshots carry only the explicitly recorded pre-trigger Ready
 # label/phase difference; actions, observations, targets and body before-poses must agree.
 assert len(report['teacher_frames'])==len(old['teacher_frames'])
 for a,b in zip(report['teacher_frames'],old['teacher_frames']):
  for key in ['time_s','observation','action','target','before']:
   assert a[key]==b[key],f'{kind} WEAK {key} differs'
 results.append({'kind':kind,'activationNative':None if not binding else cut,'exactFullNormalizedEmittedPacketsBeforeFirstChangedCommand':compared,'exactStudentDecisionPrefix':len(decisions),'exactTeacherActionsObservationsTargetsAndBeforePoses':len(report['teacher_frames']),'binding':binding})
print(json.dumps({'comparison':'Frozen baseline versus opt-in candidate; only timing fields removed throughout, and pre-first-recovery-integration Ready label/phase plus activation-boundary controller ID removed at their explicitly described boundaries','cases':results},indent=2))
