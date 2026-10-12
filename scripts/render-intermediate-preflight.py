#!/usr/bin/env python3
"""Plot recorded source/native contact chronology and static stitching gaps only."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/lh-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/lh-render-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--receipt',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
r=json.loads(a.receipt.read_text())
assert r['physicalIntegrations']==0 and not r['completeNumericTrajectoryReady']
fig,axes=plt.subplots(2,2,figsize=(11,6))
src=r['sourceInitialContacts']['handFirstSolvedS']
for i,(side,t) in enumerate(src.items()):
    axes[0,0].scatter([t],[i],s=90)
axes[0,0].set(xlim=(0,.07),yticks=[0,1],yticklabels=['main gauche','main droite'],
              xlabel='Temps source (s)',title='Source déjà couchée : vrais contacts')
native=r['nativeReference']
for i,side in enumerate(['left','right']):
    axes[0,1].scatter(native['firstHandIntentS'][side],i,s=90)
axes[0,1].axvline(native['actualPreparedHandsBilateralS'],color='#e38b21',linestyle='--',label='deux mains réelles, autre essai')
axes[0,1].axvline(native['actualPreparedHeadContactS'],color='#ba3f3f',linestyle=':',label='tête réelle, autre essai')
axes[0,1].set(xlim=(0,8.3),yticks=[0,1],yticklabels=['main gauche','main droite'],
              xlabel='Temps natif depuis crouch (s)',title='Programme natif : intentions tardives')
axes[0,1].legend(fontsize=7,loc='upper center')
gap=r['naiveStitch']
axes[1,0].barh(['écart articulaire max (rad)','écart position bassin (m)','écart rotation bassin (rad)'],
               [gap['earlyWindowJointGapRad'],gap['pelvisPositionGapM'],gap['pelvisOrientationGapRad']],
               color=['#ca5c37','#6d8ca2','#9675aa'])
axes[1,0].set(xlim=(0,2),title='Raccord à la référence native vers 3,2 s')
axes[1,1].axis('off')
axes[1,1].set(xlim=(0,1),ylim=(0,1))
axes[1,1].text(0,.92,'Résultat du préscreen',fontsize=12,weight='bold')
axes[1,1].text(0,.66,'Programme numérique complet : absent\nPont racine/contact : non construit\nSimulation nouvelle : zéro',fontsize=10)
axes[1,1].text(0,.18,'Les horloges ci-dessus partent de poses différentes.\nLe graphe ne prouve aucun relevage natif.',fontsize=9)
fig.suptitle('Transfert intermédiaire : écarts mesurés avant tout nouvel essai',fontsize=14)
fig.tight_layout(rect=(0,.02,1,.95))
a.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(a.output,dpi=140)
plt.close(fig)
