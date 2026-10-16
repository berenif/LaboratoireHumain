#!/usr/bin/env python3
"""Compare three independent archived histories; no physics or pose interpolation."""
import os,json,time,sys
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-three-poses-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/lh-three-poses-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter

began=time.monotonic();root=Path(__file__).resolve().parents[2];out=root/'outputs/getup-three-poses-2026-10-16'
destination=Path(sys.argv[1]).resolve();assert not destination.exists()
audit=json.loads((out/'postrun-audit.json').read_text());model=json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
histories={kind:[json.loads(x) for x in (out/kind/'trace.ndjson').read_text().splitlines()] for kind in ['back','belly','side']}
labels=['Dos','Ventre','Côté : fixture non admise'];az,el=np.deg2rad(-55),np.deg2rad(14)
B=np.array([[-np.sin(az),0,np.cos(az)],[-np.sin(el)*np.cos(az),np.cos(el),-np.sin(el)*np.sin(az)],
            [np.cos(el)*np.cos(az),np.sin(el),np.cos(el)*np.sin(az)]]).T
meshes=[(p['id'],np.array(p['geometry']['vertices']).reshape(-1,3),np.array(p['geometry']['indices']).reshape(-1,3)) for p in model['segments'] if p['geometry']['vertices']]

def triangles(row):
    poses={p['id']:p for p in row['segments']};polys=[];colors=[]
    for name,vertices,index in meshes:
        p=poses[name];R=Rotation.from_quat([p['rotation'][k] for k in 'xyzw']);tri=(R.apply(vertices)+[p['position'][k] for k in 'xyz'])[index]
        normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-12)
        shade=.55+.45*np.maximum(0,normal@np.array([-.3,.81,-.5]));base=np.array([.73,.39,.24]) if name.endswith('_foot') else np.array([.29,.57,.57])
        polys.append(tri@B);colors.append(np.clip(shade[:,None]*base,0,1))
    polys=np.concatenate(polys);colors=np.concatenate(colors);order=np.argsort(polys[:,:,2].mean(axis=1));return polys[order,:,:2],colors[order]

fig=plt.figure(figsize=(16,9),facecolor='#f7f8f5');panels=[]
for i,(kind,label,case) in enumerate(zip(['back','belly','side'],labels,audit['cases'])):
    x=.025+i*.33;rows=histories[kind];ax=fig.add_axes([x,.30,.30,.50]);ax.set_aspect('equal');ax.axis('off')
    centers=np.array([[p['position'][k] for k in 'xyz'] for row in rows for p in row['segments']])@B
    mid=(centers[:,:2].min(axis=0)+centers[:,:2].max(axis=0))/2
    ax.set_xlim(mid[0]-1.4,mid[0]+1.4);ax.set_ylim(-.35,1.75)
    for value in np.arange(-1.5,1.51,.3):
        for ends in [np.array([[-1.5,0,value],[1.5,0,value]]),np.array([[value,0,-1.5],[value,0,1.5]])]:
            p=ends@B;ax.plot(p[:,0],p[:,1],color='#d5ddda',lw=.6,zorder=0)
    collection=PolyCollection([],closed=True,edgecolors='none',zorder=3);ax.add_collection(collection)
    fig.text(x+.15,.855,label,ha='center',fontsize=17,color='#274745')
    clock=fig.text(x+.15,.815,'',ha='center',fontsize=11,color='#365958')
    status=fig.text(x+.15,.295,'',ha='center',fontsize=10,color='#a25436')
    plot=fig.add_axes([x+.025,.135,.25,.115]);plot.set(xlim=(0,14.6),ylim=(0,1.15),xlabel='Temps physique du cas (s)',ylabel='Stabilité (s)')
    plot.axhline(1,color='#79848b',ls='--',lw=.8);plot.grid(alpha=.2);plot.tick_params(labelsize=8)
    line,=plot.plot([],[],color='#218980',lw=2);star,=plot.plot([],[],'*',color='#b76640',markersize=10)
    panels.append((rows,case,collection,clock,status,line,star))
fig.text(.5,.96,'Trois poses initiales · même modèle, mêmes poids et paramètres',ha='center',fontsize=20,color='#274745')
fig.text(.5,.915,'Histoires indépendantes, un corps par cas · 124,722 s de calcul physique réel / 180 s maximum',ha='center',fontsize=12,color='#66716d')
fig.text(.5,.065,'Dos et ventre : 1 s native franchie, puis stabilité courante perdue · Côté : politique non évaluée',ha='center',fontsize=12,color='#a25436')
fig.text(.5,.025,'Étoiles : seuil natif confirmé entre échantillons · image figée après la fin de chaque cas · aucun calcul physique de rendu',ha='center',fontsize=10,color='#66716d')
times=[i/20 for i in range(292)]+[14.6]*20
writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-color_range','tv','-movflags','+faststart'])
with writer.saving(fig,str(destination),100):
    for t in times:
        for rows,case,collection,clock,status,line,star in panels:
            index=max(i for i,row in enumerate(rows) if row['simulation_time_s']<=t+1e-9);row=rows[index];p=row['diagnostics']['getup']['policy']
            poly,color=triangles(row);collection.set_verts(poly);collection.set_facecolors(color)
            finished=t>=rows[-1]['simulation_time_s'];clock.set_text(f"État archivé : {row['simulation_time_s']:.3f} s"+(' · fin du cas' if finished else ''))
            if case['fixtureValid'] and p['prototype_standing_passed']:text='1 s franchie · compteur courant : 0 s'
            elif not case['fixtureValid'] and finished:text='Tassée oblique vers le ventre · non admise'
            elif row['casePhase']=='preparation':text='Tassement physique · moteurs à zéro'
            else:text=f"SLOW réel · stabilité courante : {p['stable_standing_s']:.3f} s"
            status.set_text(text)
            line.set_data([r['simulation_time_s'] for r in rows[:index+1]],
                          [r['diagnostics']['getup']['policy']['stable_standing_s'] for r in rows[:index+1]])
            if case.get('oneSecondProofPhysicalS') is not None and t>=case['oneSecondProofPhysicalS']:
                star.set_data([case['oneSecondProofPhysicalS']],[1])
        writer.grab_frame()
    fig.savefig(out/'last-rendered-frame.png',dpi=100)
plt.close(fig)
print(json.dumps(dict(file=str(destination),frames=len(times),fps=20,presentationDurationS=len(times)/20,
    physicalDurationByCaseS={k:v[-1]['simulation_time_s'] for k,v in histories.items()},
    archivedStatesOnly=True,additionalPhysicsIntegrations=0,renderWallSeconds=time.monotonic()-began)))
