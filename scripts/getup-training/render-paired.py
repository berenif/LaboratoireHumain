#!/usr/bin/env python3
"""Render all four frozen evaluation pairs; consumes recorded poses, no physics."""
import json, os, sys
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-paired-mpl')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter
root=Path(__file__).resolve().parents[2];out=Path(sys.argv[1])
model=json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
meshes=[(p['id'],np.array(p['geometry']['vertices']).reshape(-1,3),np.array(p['geometry']['indices']).reshape(-1,3))
    for p in model['segments'] if p['geometry']['vertices']]
az,el=np.deg2rad(-55),np.deg2rad(14)
basis=np.array([[-np.sin(az),0,np.cos(az)],[-np.sin(el)*np.cos(az),np.cos(el),-np.sin(el)*np.sin(az)],
    [np.cos(el)*np.cos(az),np.sin(el),np.cos(el)*np.sin(az)]]).T
labels=['forward','backward','left','right'];names=['+X','−X','+Z','−Z']
rows=[[json.loads((out/f'{kind}-{name}.json').read_text()) for name in labels] for kind in ['baseline','candidate']]
fig,axes=plt.subplots(2,4,figsize=(16,8),facecolor='#f5f7f4')
collections=[];texts=[]
for r in range(2):
 for c in range(4):
  ax=axes[r,c];ax.set_aspect('equal');ax.axis('off');ax.set_xlim(-.05,1.55);ax.set_ylim(-.3,1.65)
  for x in np.arange(-1,2.1,.25):
   p=np.array([[x,0,-1],[x,0,1]])@basis;ax.plot(p[:,0],p[:,1],color='#d4dfda',lw=.4)
  coll=PolyCollection([],edgecolors='none');ax.add_collection(coll);collections.append(coll)
  title=('Initial SLOW' if r==0 else 'Après le pilote SAC')+' · poussée '+names[c]
  ax.set_title(title,fontsize=11,color='#274745');texts.append(ax.text(.5,.01,'',transform=ax.transAxes,ha='center',fontsize=9))
fig.suptitle('Comparaison sur les 4 poussées réservées · mêmes états et forces',fontsize=17,color='#274745')
clock=fig.text(.5,.935,'',ha='center',fontsize=11)
fig.text(.5,.02,'Politiques expérimentales · pas de nouvelle physique pour la vidéo · tous les cas montrés',ha='center',fontsize=10,color='#66716d')
def polygons(snapshot,row):
 poses={p['id']:p for p in snapshot['segments']};polys=[];colors=[]
 for name,v,idx in meshes:
  p=poses[name];q=p['rotation'];t=p['position']
  tri=(Rotation.from_quat([q[k] for k in 'xyzw']).apply(v)+[t[k] for k in 'xyz'])[idx]
  normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-12)
  shade=.55+.45*np.maximum(0,normal@np.array([-.3,.81,-.5]))
  color=np.array([.28,.57,.57]) if row==0 else np.array([.73,.43,.25])
  polys.append(tri@basis);colors.append(shade[:,None]*color)
 polys=np.concatenate(polys);colors=np.concatenate(colors);order=np.argsort(polys[:,:,2].mean(axis=1))
 return polys[order,:,:2],colors[order]
writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
with writer.saving(fig,str(out/'paired-heldout.mp4'),80):
 for frame in range(80):
  t=frame*.05+.025;clock.set_text(f'{t:.3f} s · '+('Poussée 20 N' if .5<=t<.7 else 'Retour au calme'))
  for r in range(2):
   for c in range(4):
    idx=r*4+c;data=rows[r][c]
    if frame<len(data['frames']):
     verts,colors=polygons(data['frames'][frame],r);collections[idx].set_verts(verts);collections[idx].set_facecolors(colors)
     texts[idx].set_text('')
    else:collections[idx].set_verts([]);texts[idx].set_text('Arrêt physique du cas')
  writer.grab_frame()
 fig.savefig(out/'paired-last-frame.png',dpi=100)
plt.close(fig)
(out/'render-report.json').write_text(json.dumps({'video':'paired-heldout.mp4','frames':80,'fps':20,
 'sampledPoseDurationSeconds':3.975,'physicsIntegrationsForRendering':0,'allFourPredeclaredCasesShown':True,
 'terminalFramesRepeatedAsHold':False},indent=2)+'\n')
