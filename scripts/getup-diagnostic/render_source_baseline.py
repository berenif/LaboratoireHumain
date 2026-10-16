#!/usr/bin/env python3
"""Render completed SLOW archives only; orthographic mesh projection, zero physics."""
import os,pathlib,json
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-source-baseline-mpl');os.environ.setdefault('XDG_CACHE_HOME','/tmp/lh-source-baseline-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'outputs/getup-source-baseline-2026-10-16';DEST=pathlib.Path('/workspace/lh-deliverables')
rows=[json.loads(s) for s in (OUT/'student-continuation-states.ndjson').read_text().splitlines()];initial=json.loads((OUT/'student-initial.json').read_text());frames=[initial]+[json.loads(s) for s in (OUT/'frames.ndjson').read_text().splitlines() if json.loads(s)['phase']=='student-continuation'];result=json.loads((OUT/'result.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());C=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
t=np.array([r['timeS'] for r in rows]);fig,axes=plt.subplots(2,2,figsize=(11,7.5))
axes[0,0].plot(t,[r['headHeightM'] for r in rows],color='#218980');axes[0,0].axhline(1.4,color='#79848b',linestyle='--');axes[0,0].set_ylabel('Hauteur tête (m)')
axes[0,1].plot(t,[1000*r['maximumSelfPenetrationM'] for r in rows],color='#b76640');axes[0,1].axhline(5,color='#a82f43',linestyle='--',label='Seuil app 5 mm');axes[0,1].set_ylabel('Auto-pénétration maximale (mm)');axes[0,1].legend(fontsize=8)
axes[1,0].plot(t,[sum(r['standing']['footLoadsN']) for r in rows],label='Pieds',color='#218980');axes[1,0].plot(t,[r['standing']['maximumOtherFloorLoadN'] for r in rows],label='Autres appuis (max)',color='#b76640');axes[1,0].set_ylabel('Charge normale résolue (N)');axes[1,0].legend(fontsize=8)
axes[1,1].plot(t,[r['standing']['stableSeconds'] for r in rows],color='#218980');axes[1,1].axhline(1,color='#79848b',linestyle='--');axes[1,1].set_ylabel('Station stable continue (s)')
for ax in axes.flat:ax.set_xlabel('Temps SLOW source (s)');ax.grid(alpha=.25)
fig.suptitle('Témoin source complet · seed 0 · speed 0,5 · MuJoCo 2.1.3',fontsize=14);fig.text(.5,.01,'216 décisions / 5,4 s · stabilité réelle finale 2,9225 s · seuil app dépassé pendant le mouvement · aucune qualification de l’app',ha='center',fontsize=8);fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(DEST/'LaboratoireHumain-GetUp-source-positif-mesures-2026-10-16.png',dpi=150);plt.close(fig)
# Fixed orthographic camera, real source (+Z up) mesh poses, depth-sorted triangles.
az=np.deg2rad(-55);el=np.deg2rad(14);right=np.array([-np.sin(az),np.cos(az),0]);up=np.array([-np.sin(el)*np.cos(az),-np.sin(el)*np.sin(az),np.cos(el)]);depth=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]);B=np.array([right,up,depth]).T
meshes=[]
for p in model['segments']:
 if not p['geometry']['vertices']:continue
 meshes.append((p['id'],np.array(p['geometry']['vertices']).reshape(-1,3)@C,np.array(p['geometry']['indices']).reshape(-1,3)))
def triangles(frame):
 polys=[];colors=[]
 for name,vertices,index in meshes:
  p=frame['bodyPosesSource'][name];R=Rotation.from_quat(np.array(p['quaternionWxyz'])[[1,2,3,0]]);world=R.apply(vertices)+np.array(p['position']);tri=world[index];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-12);shade=.55+.45*np.maximum(0,normal@np.array([-.3,-.5,.81]));base=np.array([.72,.39,.24]) if name in ['left_foot','right_foot'] else np.array([.29,.57,.57]);polys.append(tri@B);colors.append(np.clip(shade[:,None]*base,0,1))
 polys=np.concatenate(polys);colors=np.concatenate(colors);order=np.argsort(polys[:,:,2].mean(axis=1));return polys[order,:,:2],colors[order]
allpos=np.array([p['position'] for f in frames for p in f['bodyPosesSource'].values()])@B;center=(allpos[:,:2].min(axis=0)+allpos[:,:2].max(axis=0))/2
fig=plt.figure(figsize=(12.8,7.2),facecolor='#f7f8f5');ax=fig.add_axes([.08,.13,.84,.70]);ax.set_aspect('equal');ax.set_xlim(center[0]-.95,center[0]+.95);ax.set_ylim(center[1]-1.,center[1]+1.);ax.axis('off')
for value in np.arange(-1.5,1.51,.25):
 for endpoints in [np.array([[-1.5,value,0],[1.5,value,0]]),np.array([[value,-1.5,0],[value,1.5,0]])]:
  projected=endpoints@B;ax.plot(projected[:,0],projected[:,1],color='#d5ddda',lw=.6,zorder=0)
collection=PolyCollection([],closed=True,edgecolors='none',zorder=3);ax.add_collection(collection)
fig.text(.5,.94,'GetUp SLOW · témoin positif dans les conditions source',ha='center',fontsize=17,color='#274745');title=fig.text(.5,.88,'',ha='center',fontsize=12,color='#365958');status=fig.text(.5,.065,'',ha='center',fontsize=11,color='#365958');fig.text(.5,.025,'MuJoCo 2.1.3 · seed 0 · speed 0,5 · poses archivées · aucun calcul physique de rendu · app non qualifiée',ha='center',fontsize=9,color='#66716d')
path=DEST/'LaboratoireHumain-GetUp-source-positif-2026-10-16.mp4';assert not path.exists();writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-color_range','tv','-movflags','+faststart'])
times=list(np.arange(0,5.40001,.05))+[5.4]*15
with writer.saving(fig,str(path),100):
 for time in times:
  frame=max((f for f in frames if f['timeS']<=time+1e-8),key=lambda f:f['timeS']);polys,colors=triangles(frame);collection.set_verts(polys);collection.set_facecolors(colors);title.set_text(f"Temps physique source : {frame['timeS']:.2f} s · tête : {frame['headHeightM']:.3f} m")
  stable=frame.get('standing',{}).get('stableSeconds',0);status.set_text(f'Station debout stable continue : {stable:.2f} s' if stable>0 else 'Relevage source en cours · contrôles physiques archivés');writer.grab_frame()
plt.close(fig);print(json.dumps({'file':str(path),'frames':len(times),'fps':20,'sourceDurationSeconds':5.4,'additionalPhysicsIntegrations':0}))
