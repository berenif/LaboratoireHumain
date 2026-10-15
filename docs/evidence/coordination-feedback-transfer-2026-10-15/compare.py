"""Compare archived executed histories; never advances a world."""
import json,csv,math
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
model=json.loads((ROOT/'rust/crates/model/data/canonical-v1.json').read_text())['segments'];mass=sum(s['massKg'] for s in model)
def load(path):
 rows=[]; samples=[]; first={}; heelmax=[0.,0.]; heelpersist=[None,None]; standing=longest=0; lastcommand=None
 for line in path.open():
  r=json.loads(line)
  if r.get('kind') not in ('actual-history','exploration-step'):continue
  step=r['nativeSubstep'];support=r.get('support',{});d=r['localDiagnostic'];snap=r.get('snapshot');contacts=r.get('segmentContacts',support.get('segments'))
  h=r.get('pelvisHeightM',support.get('bodies',[{'position_m':[0,None,0]}])[0]['position_m'][1]);com=r.get('com',support.get('center_of_mass_m'))
  if step<=36:continue
  loaded=[s['id'] for s in contacts if sum(c['current_upward_load_n'] for c in s['contacts'])>=3]
  if not d['capture_inside']:first.setdefault('captureOutsideInsetS',step/240)
  if not d['com_inside']:first.setdefault('comOutsideInsetS',step/240)
  if any(x in loaded for x in ('head','torso','pelvis')):first.setdefault('headTorsoPelvisLoadedS',step/240)
  if any(x not in ('leftFoot','leftForefoot','rightFoot','rightForefoot') for x in loaded):first.setdefault('nonfootLoadedS',step/240)
  for i in (0,1):
   heelmax[i]=max(heelmax[i],d['heel_loads_n'][i])
   if heelpersist[i] is None and d['heel_loaded_substeps'][i]>=12:heelpersist[i]=step/240
  upright=r.get('standingPoseDiagnostic',False) and h>.93 and d['maximum_linear_mps']<=.22 and d['maximum_angular_radps']<=.65
  standing=standing+1 if upright else 0;longest=max(longest,standing)
  if r.get('commands'):lastcommand=r['commands']
  rows.append({'step':step,'t':step/240,'pelvisH':h,'com':com,'loaded':loaded,'heelLoads':d['heel_loads_n'],'standing':upright})
  if snap:
   poses=snap['segments'];v=r.get('comVelocityMps') or [sum(s['massKg']*p['linearVelocity'][a] for s,p in zip(model,poses))/mass for a in 'xyz'];pv=r.get('pelvisVelocityMps') or [poses[0]['linearVelocity'][a] for a in 'xyz']
   samples.append({'t':step/240,'step':step,'pelvisH':h,'comH':com[1],'comZ':com[2],'comVx':v[0],'comVy':v[1],'comVz':v[2],'comSpeed':math.sqrt(sum(x*x for x in v)),'pelvisVx':pv[0],'pelvisVy':pv[1],'pelvisVz':pv[2],'pelvisSpeed':math.sqrt(sum(x*x for x in pv))})
 summary={'firstEvents':first,'lastNativeSubstep':rows[-1]['step'] if rows else 36,'pelvisLastM':rows[-1]['pelvisH'] if rows else None,'pelvisMinimumM':min((r['pelvisH'] for r in rows),default=None),'pelvisMaximumM':max((r['pelvisH'] for r in rows),default=None),'comLastM':rows[-1]['com'] if rows else None,'heelMaximumReportedN':heelmax,'firstHeel12LoadedSubstepsS':heelpersist,'longestStandingWithSpeedCriteriaS':longest/240,'lastCommand':{k:lastcommand.get(k) for k in ('commandSubstep','sourceSubstep','feedback')} if lastcommand else None,'lastVelocities':samples[-1] if samples else None,'lastLoadedSegments':rows[-1]['loaded'] if rows else None}
 return rows,samples,summary
baseline=ROOT/'docs/evidence/coordination-exploration-execution-2026-10-15/native-trace.jsonl'
brows,bs,b=load(baseline);crows,cs,c=load(HERE/'native-trace.jsonl')
common={r['step']:r for r in bs};matched=[{'reference':common[r['step']],'feedback':r} for r in cs if r['step'] in common]
out={'commonFinalSnapshot':matched[-1] if matched else None,'reference':b,'feedback':c,'nativeOutcome':json.loads((HERE/'closed.json').read_text()),'scope':'Same A36 prefix; reference tracking versus one coherent feedback candidate, not isolated parameter attribution. No impact/24-case qualification.','velocityMethod':'Baseline mass-weighted archived native body velocities at snapshot boundaries; feedback explicit native CoM velocity; pelvis native body velocity.','standingMethod':'standing_support diagnostic plus pelvis>0.93m and original speeds<=0.22m/s,0.65rad/s; no recovery counter certification','additionalNativeSteps':0}
(HERE/'comparison.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
fields=['case','t','step','pelvisH','comH','comZ','comVx','comVy','comVz','comSpeed','pelvisVx','pelvisVy','pelvisVz','pelvisSpeed']
with (HERE/'comparison.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
 for name,rows in [('reference',bs),('feedback',cs)]:
  for r in rows:w.writerow(dict(case=name,**r))
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(11,7))
for label,rows in [('Référence',bs),('Retour d’état',cs)]:
 t=[r['t'] for r in rows]
 for ax,key,title in zip(axes.flat,['pelvisH','comH','pelvisSpeed','comSpeed'],['Hauteur bassin (m)','Hauteur CoM (m)','Norme vitesse bassin (m/s)','Norme vitesse CoM (m/s)']):
  ax.plot(t,[r[key] for r in rows],label=label);ax.set_title(title);ax.set_xlabel('Temps natif (s)');ax.set_xlim(0,min(bs[-1]['t'],cs[-1]['t']) if bs and cs else 4);ax.legend()
fig.suptitle('Deux histoires réellement exécutées — les variations ne qualifient pas un relevage');fig.tight_layout();fig.savefig(HERE/'comparison.png');plt.close(fig)
print(json.dumps({'reference':b,'feedback':c},ensure_ascii=False))
