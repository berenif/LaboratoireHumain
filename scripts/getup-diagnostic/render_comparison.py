#!/usr/bin/env python3
"""Plot and render archived states only; never imports the physics engine."""
import os,json,pathlib
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-mujoco-plot');os.environ.setdefault('XDG_CACHE_HOME','/tmp/lh-mujoco-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'outputs/getup-mujoco-comparison-2026-10-16';DEST=pathlib.Path('/workspace/lh-deliverables')
ref=json.loads((ROOT/'outputs/getup-policy-native-clock-2026-10-16/reference-report.json').read_text());analysis=json.loads((OUT/'comparison.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());C=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
cases=[('matched800-closed','MuJoCo 800 Hz · WEAK'),('source200-closed','MuJoCo source 200 Hz · WEAK'),('matched800-replay','MuJoCo 800 Hz · commandes Rapier')]
histories={name:[json.loads(x) for x in (OUT/name/'states.ndjson').read_text().splitlines()] for name,_ in cases}
colors=['#2b8cbe','#44a65b','#c48422'];nativeTime=np.array([0.]+[f['time_s'] for f in ref['teacher_frames']]);nativeHead=np.array([ref['teacher_frames'][0]['observation'][21]]+[next(p['position']['y'] for p in f['snapshot']['segments'] if p['id']=='head') for f in ref['teacher_frames']])
fig,axs=plt.subplots(2,2,figsize=(12,8));axs[0,0].plot(nativeTime,nativeHead,'o-',color='#8b4b91',label='Rapier archivé',markersize=3)
for (name,label),color,case in zip(cases,colors,analysis['cases']):
 rows=histories[name];t=[r['timeS'] for r in rows];axs[0,0].plot(t,[r['headHeightM'] for r in rows],label=label,color=color)
 axs[0,1].plot(t,[1000*r['maximumSelfPenetrationM'] for r in rows],label=label,color=color)
 axs[1,0].plot([r['timeS'] for r in case['comparison']],[r['maximumJointAngleDifferenceRad'] for r in case['comparison']],label=label,color=color)
 cmd=json.loads((OUT/name/'commands.json').read_text());axs[1,1].plot([r['timeS'] for r in cmd],[r['maximumActionDifferenceFromRapier'] for r in cmd],label=label,color=color)
axs[0,1].axhline(5,color='#b53436',linestyle='--',label='Seuil qualification 5 mm')
axs[0,0].set_ylabel('Hauteur tête (m)');axs[0,1].set_ylabel('Pénétration propre maximale (mm)');axs[1,0].set_ylabel('Écart articulaire maximal / Rapier (rad)');axs[1,1].set_ylabel('Écart maximal de commande / Rapier')
for ax in axs.flat:ax.set_xlabel('Temps depuis l’état archivé (s)');ax.grid(alpha=.25)
axs[0,0].legend(fontsize=7);axs[0,1].legend(fontsize=7);fig.suptitle('État généralisé apparié · MuJoCo 2.1.3 / Rapier',fontsize=15);fig.text(.5,.012,'Archives uniquement · aucun calcul physique de rendu · le source poursuit 0,30 s pour diagnostic, sans qualification',ha='center',fontsize=9);fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(DEST/'LaboratoireHumain-GetUp-MuJoCo-Rapier-comparaison-2026-10-16.png',dpi=150);plt.close(fig)
# Slow-motion comparison of real archived poses, sampled without interpolation.
meshes=[(p,np.array(p['geometry']['vertices']).reshape(-1,3),np.array(p['geometry']['indices']).reshape(-1,3)) for p in model['segments'] if p['geometry']['vertices']]
seed=ref['teacher_initial'];native=[(0.,seed)]+[(f['time_s'],f['snapshot']) for f in ref['teacher_frames']]
fig=plt.figure(figsize=(14,5.5),facecolor='#fafbfd');axes=[fig.add_subplot(1,3,i+1,projection='3d') for i in range(3)]
for path in [DEST/'LaboratoireHumain-GetUp-MuJoCo-Rapier-2026-10-16.mp4']:assert not path.exists()
writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-color_range','tv','-movflags','+faststart'])
with writer.saving(fig,str(DEST/'LaboratoireHumain-GetUp-MuJoCo-Rapier-2026-10-16.mp4'),100):
 for frame in range(61):
  t=min(.30,frame*.005)
  for ax,kind,label in zip(axes,['rapier','source200-closed','matched800-replay'],['Rapier archivé · arrêt à 0,290 s','MuJoCo source 200 Hz · WEAK','MuJoCo 800 Hz · commandes Rapier']):
   ax.clear()
   if kind=='rapier':
    time,snap=max((r for r in native if r[0]<=t+1e-8),key=lambda r:r[0]);poses={p['id']:p for p in snap['segments']}
    transform=lambda p:(Rotation.from_quat([poses[p['id']]['rotation'][k] for k in 'xyzw']),np.array([poses[p['id']]['position'][k] for k in 'xyz']))
   else:
    row=max((r for r in histories[kind] if r['timeS']<=t+1e-8),key=lambda r:r['timeS']);time=row['timeS'];poses=row['bodyPosesSource']
    def transform(p):
     raw=poses[p['id']];R=Rotation.from_quat(np.array(raw['quaternionWxyz'])[[1,2,3,0]]).as_matrix();return Rotation.from_matrix(C@R@C.T),C@np.array(raw['position'])
   for p,vertices,indices in meshes:
    R,pos=transform(p);world=(R.apply(vertices)+pos)[:,[0,2,1]];ax.add_collection3d(Poly3DCollection(world[indices],facecolor='#d56a40' if p['id'] in ['left_thigh','left_foot'] else '#6d879d',edgecolor='none',alpha=.95))
   ax.add_collection3d(Poly3DCollection([[[-.65,-.1,0],[.65,-.1,0],[.65,1.9,0],[-.65,1.9,0]]],facecolor='#dce4ea',alpha=.3));ax.set(xlim=(-.65,.65),ylim=(-.1,1.9),zlim=(-.05,.8));ax.set_box_aspect((1.3,2,.85));ax.view_init(elev=18,azim=-55);ax.tick_params(labelsize=6);ax.set_title(label+f'\nÉtat enregistré : {time:.3f} s',fontsize=9)
  fig.suptitle('Comparaison diagnostique depuis le même état · aucun relevage qualifié',fontsize=14,y=.97)
  if frame==0:fig.text(.5,.025,'Ralenti ×10 · états archivés sans interpolation · aucune intégration physique pendant le rendu',ha='center',fontsize=10)
  writer.grab_frame()
plt.close(fig)
print('Rendered comparison figure and 61-frame video from archives; zero physics.')
