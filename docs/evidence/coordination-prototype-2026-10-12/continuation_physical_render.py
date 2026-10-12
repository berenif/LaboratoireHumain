#!/usr/bin/env python3
"""Draw archived actual/preview states. No dynamics, FK or geometry query.

The compact extract preserves selected body poses and already measured values.
If present it is reused, so replay does not require the large native raw traces.
"""
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-physical-archive-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
RUN=HERE/'native-run'
MODEL=ROOT/'rust/crates/model/data/canonical-v1.json'
COMPACT=HERE/'continuation-physical-render-states.json.gz'


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda:stream.read(1024*1024),b''):h.update(data)
    return h.hexdigest()


def compact_row(row):
    support=row['support']
    return {'nativeSubstep':row['nativeSubstep'],'kind':row['kind'],
            'bodies':[[b['segment'],b['position_m'],b['rotation_xyzw']] for b in support['bodies']],
            'comPositionM':support['center_of_mass_m'],'diagnostic':row['localDiagnostic'],
            'contacts':[[s['segment'],c['world_point_m'],c['current_upward_load_n']]
                        for s in support['segments'] for c in s['contacts'] if c['current_upward_load_n']>=3.],
            'motion':row['snapshot']['motion'],'recoveries':row['snapshot']['counters']['recoveries']}


def extract():
    if COMPACT.exists():
        with gzip.open(COMPACT,'rt') as stream:return json.load(stream)
    a_report=json.loads((RUN/'001-A.json').read_text())
    b_report=json.loads((RUN/'002-B.json').read_text())
    initial=a_report['initial']
    names=[p['id'] for p in initial['segments']]
    origin={'nativeSubstep':0,'kind':'actual-initial-snapshot',
            'bodies':[[i,[p['position'][k] for k in 'xyz'],[p['rotation'][k] for k in 'xyzw']]
                      for i,p in enumerate(initial['segments'])],
            'comPositionM':None,'diagnostic':None,'contacts':[],
            'motion':initial['motion'],'recoveries':initial['counters']['recoveries']}
    frames=[origin];counts={};max_stable=0
    with (RUN/'001-A.states.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['kind']!='actual-history':raise RuntimeError('A contains non-actual history')
            max_stable=max(max_stable,row['localDiagnostic']['consecutive_stable_substeps'])
            if row['nativeSubstep']%12==0:frames.append(compact_row(row))
    last_b=None
    with (RUN/'002-B.states.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);kind=row['kind'];counts[kind]=counts.get(kind,0)+1
            if not kind.startswith('preview-'):raise RuntimeError('Unexpected B actual state')
            last_b=compact_row(row)
    if a_report['actualNativeSteps']!=960 or b_report['actualNativeSteps']!=0:
        raise RuntimeError('Archived result differs from requested A/B scope')
    result={'schemaVersion':1,'segmentIds':names,'nativeHz':240,'A':frames,'BLastPreview':last_b,
            'AOutcome':a_report['outcome'],'BOutcome':b_report['outcome'],
            'BRefusal':b_report['refusal'],'BPreviewGroups':counts,'AActualNativeSteps':960,
            'BActualNativeSteps':0,'BPredictedNativeSteps':b_report['predictionNativeSteps'],
            'AMaximumConsecutiveStableSubsteps':max_stable,
            'sourceSha256':{str(p.relative_to(ROOT)):sha(p) for p in
                [MODEL,RUN/'001-A.json',RUN/'001-A.states.jsonl',RUN/'002-B.json',RUN/'002-B.states.jsonl']}}
    with gzip.open(COMPACT,'wt',compresslevel=9) as stream:json.dump(result,stream,separators=(',',':'),allow_nan=False)
    return result


def hull(points):
    points=sorted(set(map(tuple,points)))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    result=[]
    for part in (points,list(reversed(points))):
        side=[]
        for p in part:
            while len(side)>1 and cross(side[-2],side[-1],p)<=0:side.pop()
            side.append(p)
        result.extend(side[:-1])
    return result


def rotation(values):
    x,y,z,w=values;s=2/(x*x+y*y+z*z+w*w)
    return np.array([[1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w)],
                     [s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w)],
                     [s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)]])


def draw(ax,frame):
    body_polygons=[]
    for i,position,q in frame['bodies']:
        s=segments[i];local=np.asarray(s['geometry']['vertices']).reshape(-1,3)
        # Rigid rendering at STORED body pose; no joint chain/FK is evaluated.
        world=local@rotation(q).T+np.asarray(position)
        body_polygons.append((position[0],s,hull(world[:,[2,1]])))
    for depth,s,polygon in sorted(body_polygons,key=lambda p:p[0]):
        role=s['role'];color='#316b86' if role in ('hindfoot','forefoot') else '#b78858' if role=='hand' else '#71969b' if role in ('shin','thigh','ankle') else '#243d53'
        ax.add_patch(Polygon(polygon,closed=True,facecolor=color,edgecolor='#f1f4f5',linewidth=.4,alpha=.90))
    com=frame['comPositionM']
    if com is not None:ax.scatter(com[2],com[1],c='#efa900',s=32,zorder=20,edgecolors='white',linewidths=.5)
    for _,point,load in frame['contacts']:
        ax.scatter(point[2],point[1],c='#438563',s=min(38,5+math.sqrt(load)),zorder=21,alpha=.85)
    ax.axhline(0,c='#87979f',lw=1)
    ax.set_xlim(-.48,2.06);ax.set_ylim(-.075,1.87);ax.set_aspect('equal')
    ax.set_facecolor('#f5f7f9');ax.spines[['top','right']].set_visible(False)
    ax.spines[['left','bottom']].set_color('#bdc7cc');ax.tick_params(labelsize=8,colors='#647680')
    ax.set_xlabel('z sagittal (m)',fontsize=8,color='#647680')


if __name__=='__main__':
    segments=json.loads(MODEL.read_text())['segments'];data=extract()
    assert sha(MODEL)==data['sourceSha256'][str(MODEL.relative_to(ROOT))], 'Canonical mesh changed since extraction'
    assert data['segmentIds']==[s['id'] for s in segments]
    a={r['nativeSubstep']:r for r in data['A']};last_b=data['BLastPreview']
    assert list(a)==list(range(0,961,12))
    plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':'white'})
    fig,axes=plt.subplots(2,3,figsize=(15,8.3))
    for ax,step in zip(axes.flat[:5],[0,240,480,720,960]):
        draw(ax,a[step]);ax.set_title(f'A — simulation réelle · {step/240:g} s',fontsize=12,color='#243d53')
        if step>0:ax.text(.98,.96,f'Bassin {a[step]["bodies"][0][1][1]:.3f} m',transform=ax.transAxes,ha='right',va='top',fontsize=9,color='#647680')
    ax=axes.flat[5];draw(ax,last_b)
    ax.set_title(f'B — prévision refusée · {last_b["nativeSubstep"]/240:.2f} s',fontsize=12,color='#a43c68')
    ax.text(.98,.96,'Aucun pas réel de B',transform=ax.transAxes,ha='right',va='top',fontsize=10,color='#a43c68')
    fig.suptitle('Protocole clos : A exécuté, B refusé avant action',fontsize=19,color='#243d53',y=.975)
    fig.text(.5,.026,'Poses natives archivées uniquement · Jaune : CoM mesuré · Vert : points de contact courants ≥3 N\n'
             'A : diagnostic local non acquis, 0 récupération. B : dernière copie prédite, aucune action réelle.',
             ha='center',fontsize=10,color='#5b6c77')
    fig.subplots_adjust(top=.91,bottom=.16,left=.05,right=.99,wspace=.14,hspace=.30)
    png=HERE/'continuation-physical-results.png';fig.savefig(png,dpi=155);plt.close(fig)

    movie=HERE/'continuation-A-actual-history.mp4'
    fig=plt.figure(figsize=(12.8,7.2),dpi=100)
    writer=FFMpegWriter(fps=20,codec='libx264',bitrate=1500,metadata={'title':'A simulation réelle — états archivés, diagnostic local non acquis'})
    with writer.saving(fig,str(movie),100):
        for frame in data['A']:
            step=frame['nativeSubstep'];diag=frame['diagnostic'];fig.clear()
            ax=fig.add_axes([.05,.18,.57,.69]);draw(ax,frame);ax.set_ylabel('hauteur (m)',fontsize=9,color='#647680')
            fig.text(.66,.85,'A — simulation réelle',fontsize=20,color='#243d53',weight='bold')
            fig.text(.66,.80,'Rendu des états archivés',fontsize=12,color='#647680')
            fig.text(.66,.69,f't = {step/240:.2f} s  ·  {step}/960 pas',fontsize=15,color='#243d53')
            fig.text(.66,.61,f'Bassin : {frame["bodies"][0][1][1]:.3f} m',fontsize=13,color='#316b86')
            if diag is not None:
                heels=diag['heel_loads_n'];lines=(f'Talons G/D : {heels[0]:.1f} / {heels[1]:.1f} N\n'
                        f'Support admis : {100*diag["support_fraction"]:.1f} % du poids\n'
                        f'Stabilité : {diag["consecutive_stable_substeps"]} / 132 pas\n'
                        f'Appuis hors pieds : {diag["nonfoot_current_load_n"]:.1f} N')
            else:lines='État initial archivé\nAucune charge encore mesurée'
            fig.text(.66,.50,lines,fontsize=11,color='#586e7b',linespacing=1.7,va='top')
            fig.text(.66,.25,'Diagnostic local non acquis\n0 récupération observée',fontsize=13,color='#a43c68',linespacing=1.5)
            timeline=fig.add_axes([.07,.09,.53,.045]);timeline.plot([0,4],[0,0],c='#d7e0e4',lw=5)
            timeline.plot([0,step/240],[0,0],c='#316b86',lw=5);timeline.scatter(step/240,0,c='#efa900',s=40,zorder=3)
            timeline.set(xlim=(-.04,4.04),ylim=(-1,1),yticks=[],xticks=[0,1,2,3,4]);timeline.spines[:].set_visible(False)
            fig.text(.07,.025,'Aucun calcul physique, FK ou nouvelle inspection pendant ce rendu.',fontsize=9,color='#647680')
            writer.grab_frame()
    plt.close(fig)
    receipt={'scope':'rendering of archived actual A and rejected B preview states only',
             'sourceSha256':data['sourceSha256'],'compactStatesSha256':sha(COMPACT),
             'pngSha256':sha(png),'mp4Sha256':sha(movie),'scriptSha256':sha(Path(__file__)),
             'AActualHistorySubsteps':960,'ARenderedArchivedFrames':81,'AImageSubsteps':[0,240,480,720,960],
             'BImagePreviewKind':last_b['kind'],'BImagePreviewSubstep':last_b['nativeSubstep'],
             'BActualHistorySubsteps':0,'BPreviewGroups':data['BPreviewGroups'],
             'AOutcome':data['AOutcome'],'BRefusal':data['BRefusal'],
             'videoFps':20,'newNativeSubsteps':0,'newFkInspections':0,'newGeometryQueries':0}
    (HERE/'continuation-physical-render-receipt.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in receipt.items() if k!='sourceSha256'}))
