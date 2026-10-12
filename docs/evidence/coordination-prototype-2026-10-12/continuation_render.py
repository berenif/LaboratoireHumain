#!/usr/bin/env python3
"""Render archived native geometry poses only; no FK/query/integration is run."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import numpy as np
os.environ.setdefault("MPLCONFIGDIR", "/tmp/lh-continuation-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MODEL=ROOT/"rust/crates/model/data/canonical-v1.json"
TRACE=HERE/"continuation-native-geometry.jsonl.gz"
FRAMES=HERE/"continuation-route-frames.jsonl"


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def hull(points):
    points=sorted(set(map(tuple,points)))
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    result=[]
    for part in (points,list(reversed(points))):
        side=[]
        for p in part:
            while len(side)>1 and cross(side[-2],side[-1],p)<=0: side.pop()
            side.append(p)
        result.extend(side[:-1])
    return result


def rotation(q):
    x,y,z,w=(q[k] for k in 'xyzw')
    scale=2/(x*x+y*y+z*z+w*w)
    return np.array([[1-scale*(y*y+z*z),scale*(x*y-z*w),scale*(x*z+y*w)],
                     [scale*(x*y+z*w),1-scale*(x*x+z*z),scale*(y*z-x*w)],
                     [scale*(x*z-y*w),scale*(y*z+x*w),1-scale*(x*x+y*y)]])


def draw(ax, archived, diagnostic, detailed=False):
    bodies=[]
    for segment,pose in zip(segments,archived['result']['poses']):
        local=np.asarray(segment['geometry']['vertices']).reshape(-1,3)
        position=np.array([pose['position'][k] for k in 'xyz'])
        world=local@rotation(pose['rotation']).T+position
        bodies.append((position[0],segment,world))
    for depth,segment,world in sorted(bodies,key=lambda b:b[0]):
        role=segment['role']
        color='#316b86' if role in ('hindfoot','forefoot') else '#b78858' if role=='hand' else '#71969b' if role in ('shin','thigh','ankle') else '#243d53'
        ax.add_patch(Polygon(hull(world[:,[2,1]]),closed=True,facecolor=color,
                             edgecolor='#eef2f3',linewidth=.4,alpha=.88))
    com=diagnostic['inspection']['centerOfMassM']
    ax.plot([com[2],com[2]],[0,com[1]],color='#c18300',linewidth=1.05,linestyle='--',zorder=20)
    ax.scatter(com[2],com[1],c='#efa900',s=45,zorder=21,edgecolors='white',linewidths=.5)
    toe=seed['initial']['inspection']['feet']['leftForefoot']['lowestWorldPointM']
    heel=diagnostic['inspection']['feet']['leftFoot']['posteriorWorldPointM']
    ax.scatter(toe[2],toe[1],c='#d97912',marker='o',s=36,zorder=30)
    ax.scatter(heel[2],heel[1],c='#be426c',marker='^',s=45,zorder=30)
    if diagnostic['substep']>=168:
        ax.plot([heel[2],toe[2]],[0,0],color='#4b9b74',linewidth=4,zorder=10)
    ax.axhline(0,color='#99a5ad',linewidth=1)
    ax.set_xlim(-.42,.32);ax.set_ylim(-.07,1.86);ax.set_aspect('equal')
    ax.set_facecolor('#f7f8fa');ax.spines[['top','right']].set_visible(False)
    ax.spines[['left','bottom']].set_color('#bac3c9')
    ax.tick_params(colors='#697781',labelsize=8)
    ax.set_xlabel('z sagittal (m)',fontsize=8,color='#526371')
    if detailed:
        ax.set_ylabel('hauteur (m)',fontsize=9,color='#526371')
        ax.text(.01,.98,'Vue sagittale des maillages\naux poses natives archivées',transform=ax.transAxes,
                va='top',fontsize=9,color='#526371')
    margin=diagnostic['virtualFootSupportMarginM']
    caption='Support prévu : ligne' if margin is None else f'Marge CoM virtuelle : {1000*margin:.1f} mm'
    ax.text(.5,.01,caption,transform=ax.transAxes,ha='center',fontsize=8,color='#316d50')


if __name__=='__main__':
    segments=json.loads(MODEL.read_text())['segments']
    seed=json.loads((HERE/'continuation-seed.json').read_text())
    diagrams=[json.loads(line) for line in FRAMES.read_text().splitlines()]
    steps=set(range(0,1921,24))|{0,168,432,1680,1920}
    archived={}
    with gzip.open(TRACE,'rt') as stream:
        for line in stream:
            value=json.loads(line)
            if value['substep'] in steps: archived[value['substep']]=value
    plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':'#ffffff'})
    fig,axes=plt.subplots(1,5,figsize=(15.5,6.2),sharey=True)
    for ax,step,label in zip(axes,[0,168,432,1680,1920],['Fixture','Talons prévus','Recentrage tenu','Objectif debout','Posture tenue 1 s']):
        draw(ax,archived[step],diagrams[step]);ax.set_title(f'{step/240:g} s · {label}',fontsize=10,pad=14,color='#243d53')
    axes[0].set_ylabel('hauteur (m)',fontsize=9,color='#526371')
    fig.suptitle('Cibles géométriques — aucune simulation physique',fontsize=17,color='#243d53',y=.96)
    fig.text(.5,.025,'Orange : pivot matériel avant-pied   •   Rose : point du talon   •   Jaune : CoM virtuel\n'
             'Maillages aux poses du service statique natif. Contacts chargés, efforts et récupération non démontrés.',
             ha='center',fontsize=10,color='#576a78')
    fig.subplots_adjust(left=.06,right=.99,top=.86,bottom=.15,wspace=.13)
    png=HERE/'continuation-geometric-milestones.png';fig.savefig(png,dpi=160);plt.close(fig)

    movie=HERE/'continuation-geometric-route.mp4'
    fig=plt.figure(figsize=(12.8,7.2),dpi=100)
    writer=FFMpegWriter(fps=10,codec='libx264',bitrate=1700,metadata={'title':'Cibles géométriques sans simulation physique'})
    video_steps=list(range(0,1921,24))
    with writer.saving(fig,str(movie),100):
        for step in video_steps:
            fig.clear();ax=fig.add_axes([.06,.11,.42,.77]);draw(ax,archived[step],diagrams[step],True)
            fig.text(.51,.86,'Cibles géométriques',fontsize=25,color='#243d53',weight='bold')
            fig.text(.51,.81,'Aucune simulation physique',fontsize=15,color='#be426c')
            fig.text(.51,.71,f't = {step/240:.2f} s   •   sous-pas {step}/1920',fontsize=15,color='#243d53')
            phase='Pivot vers les talons' if step<168 else 'Recentrage' if step<300 else 'Maintien prévu 0,55 s' if step<432 else 'Extension des jambes' if step<1680 else 'Posture finale tenue 1 s'
            fig.text(.51,.64,phase,fontsize=18,color='#316b86')
            margin=diagrams[step]['virtualFootSupportMarginM']
            text='Support initial presque linéaire' if margin is None else f'Marge CoM virtuelle : {1000*margin:.1f} mm'
            fig.text(.51,.54,text,fontsize=13,color='#316d50')
            fig.text(.51,.42,'Points matériels conservés dans la référence.\nLa racine est un état inspecté, jamais une action.\n\n'
                     'La géométrie ne prouve ni charge de contact,\nni effort moteur, ni récupération physique.',fontsize=12,color='#576a78',linespacing=1.6)
            timeline=fig.add_axes([.52,.14,.40,.06]);timeline.plot([0,8],[0,0],c='#d6dfe4',lw=5)
            timeline.plot([0,step/240],[0,0],c='#316b86',lw=5);timeline.scatter([step/240],[0],c='#efa900',s=60,zorder=3)
            timeline.set_xlim(-.1,8.1);timeline.set_ylim(-1,1);timeline.set_yticks([]);timeline.set_xticks([0,.7,1.8,7,8]);timeline.set_xticklabels(['0','0,7','1,8','7','8 s']);timeline.spines[:].set_visible(False)
            fig.text(.51,.065,'Source : poses natives archivées · rendu de maillage uniquement',fontsize=9,color='#7a8790')
            writer.grab_frame()
    plt.close(fig)
    receipt={'sourceTraceSha256':digest(TRACE),'sourceFramesSha256':digest(FRAMES),'modelSha256':digest(MODEL),
             'pngSha256':digest(png),'mp4Sha256':digest(movie),'pngMilestoneSubsteps':[0,168,432,1680,1920],
             'videoArchivedPoseFrames':len(video_steps),'videoFps':10,'newFkInspections':0,'nativeGeometryQueries':0,
             'physicalIntegrations':0,'scope':'rigid rendering of stored body poses; geometry targets, no physical trajectory'}
    (HERE/'continuation-render-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt))
