#!/usr/bin/env python3
"""Render measured impulse comparisons from archived rows; zero world steps."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--analysis', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
rows = [json.loads(line) for line in (args.analysis/'aggregates.jsonl').read_text().splitlines()]
summary = json.loads((args.analysis/'summary.json').read_text())
time = np.array([r['timeS'] for r in rows])
required = np.array([r['requiredFloorN'][1] for r in rows])
all_ccd = np.array([r['allIntervalsFloorN'][1] for r in rows])
legacy = np.array([r['legacyReportedVerticalN'] for r in rows])
dt = np.array([r['dtS'] for r in rows])
residual = np.array([r['sumLinearResidualNs'] for r in rows])
worst = summary['worstLegacyVerticalDiscrepancy']
fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
fig.suptitle('Impacts natifs : observer chaque résolution CCD\nTrajectoire échouée, physique inchangée ; P1 non certifiée', fontsize=14)
window = np.abs(time-worst['timeS']) <= .12
ax = axes[0]
ax.plot(time[window], required[window], color='#222222', label='Bilan des vitesses + gravité')
ax.plot(time[window], all_ccd[window], '--', color='#007f73', label='Somme des résolutions CCD')
ax.plot(time[window], legacy[window], color='#c25229', label='Lecture historique en fin de pas')
ax.set(ylabel='Réaction verticale moyenne (N)', title='Impact présentant le plus grand écart historique')
ax.legend(loc='upper right', fontsize=9)
ax = axes[1]
ax.plot(time, (required-legacy)*dt, color='#c25229', label='Écart historique')
ax.plot(time, residual[:,1], color='#007f73', label='Résidu avec toutes les résolutions')
ax.set(ylabel='Résidu vertical (N·s)', title='Même histoire physique ; changement de frontière de lecture')
ax.legend(fontsize=9)
ax = axes[2]
for k, label in enumerate(['X', 'Y', 'Z']):
    ax.plot(time, residual[:,k], label=label, linewidth=.9)
ax.set(xlabel='Temps natif (s)', ylabel='Résidu linéaire (N·s)', title='Résidus restants ; aucune tolérance de certification ajoutée')
ax.legend(ncol=3)
for ax in axes:
    ax.grid(alpha=.2)
args.out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.out, dpi=160)
