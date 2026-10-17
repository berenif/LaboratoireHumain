#!/usr/bin/env python3
"""Render the successful single-case archive only; no physics or interpolation."""
import json
import os
from pathlib import Path
import sys
import time
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-complete-recovery-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/lh-complete-recovery-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter

began=time.monotonic(); root=Path(__file__).resolve().parents[2]
out=root/'outputs/getup-complete-replay-2026-10-16'
destination=Path(sys.argv[1]).resolve(); assert not destination.exists()
rows=[json.loads(x) for x in (out/'trace.ndjson').read_text().splitlines()]
audit=json.loads((out/'postrun-audit.json').read_text())
model=json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
assert audit['standingStableContinuousFinalS']>=3 and rows[-1]['simulation_time_s']==9.8
az,el=np.deg2rad(-55),np.deg2rad(14)
basis=np.array([[-np.sin(az),0,np.cos(az)],
                [-np.sin(el)*np.cos(az),np.cos(el),-np.sin(el)*np.sin(az)],
                [np.cos(el)*np.cos(az),np.sin(el),np.cos(el)*np.sin(az)]]).T
meshes=[(p['id'],np.array(p['geometry']['vertices']).reshape(-1,3),
         np.array(p['geometry']['indices']).reshape(-1,3))
        for p in model['segments'] if p['geometry']['vertices']]

def project(row):
    poses={p['id']:p for p in row['segments']};polys=[];colors=[]
    for name,vertices,indices in meshes:
        p=poses[name];R=Rotation.from_quat([p['rotation'][k] for k in 'xyzw'])
        tri=(R.apply(vertices)+[p['position'][k] for k in 'xyz'])[indices]
        normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-12)
        shade=.55+.45*np.maximum(0,normal@np.array([-.3,.81,-.5]))
        base=np.array([.73,.39,.24]) if name.endswith('_foot') else np.array([.29,.57,.57])
        polys.append(tri@basis);colors.append(np.clip(shade[:,None]*base,0,1))
    polys=np.concatenate(polys);colors=np.concatenate(colors);order=np.argsort(polys[:,:,2].mean(axis=1))
    return polys[order,:,:2],colors[order]

head=lambda row:next(p['position']['y'] for p in row['segments'] if p['id']=='head')
times=np.array([s['simulation_time_s'] for s in rows]);heads=np.array([head(s) for s in rows])
stable=np.array([s['diagnostics']['getup']['policy']['stable_standing_s'] for s in rows])
fig=plt.figure(figsize=(12.8,7.2),facecolor='#f7f8f5');ax=fig.add_axes([.025,.18,.59,.65])
ax.set_aspect('equal');ax.set_xlim(-1.1,1.6);ax.set_ylim(-.5,1.8);ax.axis('off')
for value in np.arange(-1.5,1.51,.25):
    for ends in [np.array([[-1.5,0,value],[1.5,0,value]]),np.array([[value,0,-1.5],[value,0,1.5]])]:
        p=ends@basis;ax.plot(p[:,0],p[:,1],color='#d5ddda',lw=.6,zorder=0)
collection=PolyCollection([],closed=True,edgecolors='none',zorder=3);ax.add_collection(collection)
heightplot=fig.add_axes([.68,.55,.28,.24]);heightplot.set(xlim=(0,9.8),ylim=(0,1.7),xlabel='Temps physique (s)',ylabel='Tête (m)')
heightplot.set_title('Histoire réelle · un impact',fontsize=11)
heightplot.axvline(audit['impactAtPhysicalS'],color='#b76640',ls=':',label='Impact')
heightplot.axvline(3.60125,color='#7b638e',ls=':',label='Début SLOW')
heightplot.axhline(1.4,color='#79848b',ls='--',lw=.7)
heightline,=heightplot.plot([],[],color='#218980',lw=2);heightplot.grid(alpha=.2);heightplot.legend(fontsize=7,loc='lower right')
stableplot=fig.add_axes([.68,.23,.28,.21]);stableplot.set(xlim=(0,9.8),ylim=(0,3.3),xlabel='Temps physique (s)',ylabel='Stabilité continue (s)')
stableplot.set_title('Critère physique natif · appuis et vitesses',fontsize=11)
stableplot.axhline(1,color='#79848b',ls='--',lw=.8,label='1 s requise');stableplot.legend(fontsize=7,loc='upper left')
stableline,=stableplot.plot([],[],color='#218980',lw=2);stableplot.grid(alpha=.2)
fig.text(.5,.95,'Rapier · relevage prototype et maintien debout',ha='center',fontsize=18,color='#274745')
title=fig.text(.32,.86,'',ha='center',fontsize=12,color='#365958');status=fig.text(.32,.15,'',ha='center',fontsize=11,color='#365958')
fig.text(.5,.085,'Preuve 1 s à 5,53 s après impact · stabilité finale 3,009 s · SLOW observé pendant 6,199 s',ha='center',fontsize=11,color='#218980')
fig.text(.5,.04,'Un seul cas · récupération publique désactivée · qualification des 24 cas non effectuée',ha='center',fontsize=10,color='#66716d')
fig.text(.5,.012,'Poses archivées sans interpolation · dernière pose maintenue 1 s · aucun calcul physique de rendu',ha='center',fontsize=9,color='#66716d')
presentation=[i/20 for i in range(196)]+[9.8]*20
writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-color_range','tv','-movflags','+faststart'])
with writer.saving(fig,str(destination),100):
    for t in presentation:
        index=max(i for i,x in enumerate(times) if x<=t+1e-9);row=rows[index];p=row['diagnostics']['getup']['policy']
        polygons,colors=project(row);collection.set_verts(polygons);collection.set_facecolors(colors)
        heightline.set_data(times[:index+1],heads[:index+1]);stableline.set_data(times[:index+1],stable[:index+1])
        title.set_text(f"État physique archivé : {row['simulation_time_s']:.3f} s · tête {head(row):.3f} m")
        if p['stable_standing_s']>=1: label=f"Debout stable : {p['stable_standing_s']:.3f} s"
        elif p['policy_decisions']: label='SLOW réel · relevage et stabilisation'
        else: label='État initial, impact et chute mesurés'
        status.set_text(label);writer.grab_frame()
    fig.savefig(out/'last-rendered-frame.png',dpi=100)
plt.close(fig)
print(json.dumps(dict(file=str(destination),frames=len(presentation),fps=20,physicalDurationS=9.8,
                     presentationDurationS=len(presentation)/20,archivedStatesOnly=True,
                     additionalPhysicsIntegrations=0,renderWallSeconds=time.monotonic()-began)))
