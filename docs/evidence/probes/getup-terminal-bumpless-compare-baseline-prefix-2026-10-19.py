import json,pathlib,hashlib
base=pathlib.Path('outputs/getup-root-translation-carry-20261019/fresh-3plus1')
cand=pathlib.Path('outputs/getup-terminal-bumpless-20261019/fresh-3plus1')
reports=[]
for kind in ['side','back','belly','side-supported']:
 b=[json.loads(s) for s in (base/kind/'trace.ndjson').read_text().splitlines()]
 c=[json.loads(s) for s in (cand/kind/'trace.ndjson').read_text().splitlines()]
 final=json.loads((cand/kind/'reference-report.json').read_text())
 activation=(final.get('terminal_balance') or {}).get('activated_native_substep')
 baseline_report=json.loads((base/kind/'reference-report.json').read_text())
 baseline_activation=(baseline_report.get('terminal_balance') or {}).get('activated_native_substep')
 limit=min(n for n in [activation,baseline_activation,10**30] if n is not None)
 bm={(round(r['snapshot']['simulation_time_s']*800),r['triggerNative']):r for r in b}
 samples=[];mismatches=[]
 for row in c:
  n=round(row['snapshot']['simulation_time_s']*800)
  if n>limit: continue
  old=bm.get((n,row['triggerNative']))
  if old is None: mismatches.append({'native':n,'cause':'baseline sample absent'});continue
  # Complete actual pose/velocity slots and native joint/control diagnostics are
  # physical receipts. New variant labels, wall clocks and admission bookkeeping
  # are intentionally not included in this pre-transfer physical comparison.
  for key in ['segments']:
   if row['snapshot'][key]!=old['snapshot'][key]:mismatches.append({'native':n,'field':key})
  for key in ['joint_angles_rad','joint_velocities_radps','motor_controls']:
   if row['snapshot']['diagnostics']['getup'][key]!=old['snapshot']['diagnostics']['getup'][key]:mismatches.append({'native':n,'field':key})
  if row['fixture']!=old['fixture']:mismatches.append({'native':n,'field':'fixture'})
  samples.append(n)
 br=json.loads((base/kind/'reference-report.json').read_text())
 teacher_equal=final['teacher_frames']==br['teacher_frames']
 prefix=[d for d in final['student_decisions'] if d['native_substep']<limit]
 baseline_prefix=[d for d in br['student_decisions'] if d['native_substep']<limit]
 decisions_equal=prefix==baseline_prefix
 reports.append({'kind':kind,'candidateActivationNative':activation,'compared40HzNativeSamples':samples,'physicalMismatches':mismatches,'exactTeacherArchive':teacher_equal,'exactSlowDecisionArchivePrefix':decisions_equal,'candidateSlowArchiveLength':len(prefix),'baselineSlowArchiveLength':len(br['student_decisions']),'comparedSlowPrefixLength':len(prefix),'comparisonScopeEndsAtEarlierBranchTransferNative':limit,'initialComparisonCorrection':'Native stamp alone conflated duplicate pre/posttrigger advance0 rows; now key(native,triggerNative). SLOW comparison is restricted to before first transfer in either branch, rather than comparing newly continuing SLOW decisions to a stopped baseline. Physical thresholds and trajectories unchanged; initial output preserved.','scope':'Exact pre-transfer physical samples/joint controls/fixture and source actor archives; labels/wall costs and separate admission state excluded. No claim of unseen800Hz prefix parity.'})
path=cand/'matched-baseline-prefix-comparison.json';path.write_text(json.dumps({'cases':reports},indent=2)+'\n');print(json.dumps(reports,indent=2))
