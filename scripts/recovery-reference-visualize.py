#!/usr/bin/env python3
"""Faithful source-state diagrams; not engine screenshots or native validation."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

root=Path('evidence/recovery-trajectory-preparation-2026-10-11/reference')
meta=json.loads((root/'tao-slow-seed0.json').read_text())
z=np.load(root/'tao-slow-seed0.npz',allow_pickle=False)
t=z['time_s']-z['time_s'][0]; names=meta['bodyNames'];pos=z['body_pos'];parent=z['body_parent_id']
loads=z['floor_load_n'];mass=z['body_mass_kg'];center=z['com_pos'];
indices=[int(np.argmin(abs(t-s))) for s in [0,0.25,0.5,0.8,1.1,1.65,3.65]]
limits=[(pos[indices,1:,k].min()-.15,pos[indices,1:,k].max()+.15) for k in range(3)]

def draw(ax,index):
    for i in range(1,len(names)):
        if parent[i]>0:
            points=pos[index,[parent[i],i]]
            ax.plot(points[:,0],points[:,1],points[:,2],color='#415a77',lw=4)
    p=pos[index,1:]
    active=loads[index,1:]>=3 if index>0 else np.zeros(len(names)-1,dtype=bool)
    ax.scatter(p[:,0],p[:,1],p[:,2],c=np.where(active,'#c34a36','#415a77'),s=18)
    ax.scatter(*center[index],color='#dba629',s=26)
    ax.plot([limits[0][0],limits[0][1]],[0,0],[0,0],color='#999999',lw=1)
    ax.set(xlim=limits[0],ylim=limits[1],zlim=(0,1.75))
    ax.set_box_aspect([1,1,1.4]);ax.view_init(elev=12,azim=-60)
    ax.set_xlabel('X source (m)',fontsize=7);ax.set_ylabel('Y source (m)',fontsize=7)
    ax.set_zlabel('Z haut (m)',fontsize=7);ax.tick_params(labelsize=6)
    ax.set_title(f't = {t[index]:.2f} s · tête {z["head_height_m"][index]:.2f} m',fontsize=9)

fig=plt.figure(figsize=(15,4.6))
for k,index in enumerate(indices):draw(fig.add_subplot(1,len(indices),k+1,projection='3d'),index)
fig.suptitle('Référence SLOW extraite — poses mesurées dans MuJoCo, pas de relevage Rapier',fontsize=13)
fig.text(.5,.02,'Schéma des origines des corps · rouge : contact de sol résolu ≥ 3 N · jaune : centre de masse · t=0 après reset source, sans preuve d’appui',ha='center',fontsize=9)
fig.subplots_adjust(left=.01,right=.99,bottom=.08,top=.87,wspace=.03)
fig.savefig(root/'source-milestones.png',dpi=140);plt.close(fig)

fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
axes[0].plot(t,z['head_height_m'],label='Tête');axes[0].plot(t,center[:,2],label='Centre de masse');axes[0].set_ylabel('Hauteur source (m)');axes[0].legend()
for name in ['left_hand','right_hand','left_foot','right_foot','left_thigh','right_thigh']:
 axes[1].plot(t[1:],loads[1:,names.index(name)],label=name,lw=.8)
axes[1].set_yscale('symlog',linthresh=3);axes[1].set_ylabel('Appui résolu (N)');axes[1].legend(ncol=3,fontsize=8)
axes[2].plot(t,np.linalg.norm(z['com_vel'],axis=1),label='Vitesse CoM (m/s)')
axes[2].plot(t,np.max(np.abs(z['qfrc_actuator'][:,6:]),axis=1),label='Couple généralisé max (Nm)')
axes[2].legend();axes[2].set_xlabel('Temps physique source (s)')
fig.suptitle('Référence source : contacts et dynamique mesurés — anatomie 40,84 kg, 17 corps dont le monde')
fig.tight_layout();fig.savefig(root/'source-support.png',dpi=140);plt.close(fig)

fig=plt.figure(figsize=(7,6));ax=fig.add_subplot(111,projection='3d')
writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p'])
frames=np.unique(np.append(np.searchsorted(t,np.arange(0,t[-1],.05)),len(t)-1))
with writer.saving(fig,str(root/'source-recorded-poses.mp4'),100):
 for index in frames:
  ax.clear();draw(ax,int(index))
  fig.suptitle('SLOW / MuJoCo : schéma de poses extraites\nAucune réussite Rapier démontrée',fontsize=11)
  writer.grab_frame()
(root/'visuals.json').write_text(json.dumps({'diagramNotEngineScreenshot':True,'videoFrames':len(frames),
 'videoFps':20,'lastPhysicsTimeS':float(t[-1]),'usesRecordedSourceBodiesOnly':True,
 'nativePhysicsSuccess':False},indent=2)+'\n')
