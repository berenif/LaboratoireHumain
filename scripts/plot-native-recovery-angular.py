#!/usr/bin/env python3
"""Static angular diagnosis from saved native rows; never integrates a world."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trace', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
rows = [json.loads(line) for line in args.trace.read_text().splitlines()]
x = np.array([r['native_substep']/240 for r in rows])
fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
fig.suptitle('Diagnostic angulaire natif — P1 non certifiée\nPhysique inchangée ; trajectoire échouée', fontsize=14)
for ax, key, title in zip(axes, ['unresolved_angular_ns', 'inverse_damping_estimate_ns', 'geometry_transport_ns'],
                          ['Résidu incomplet : lignes articulaires et opérations natives manquantes',
                           'Correction inverse d’amortissement estimée ; livraison non journalisée',
                           'Transport mesuré après poses, clamp CCD et propriétés de masse']):
    values = np.array([r[key] for r in rows])
    for k, label in enumerate(['X', 'Y', 'Z']):
        ax.plot(x, values[:, k], label=label, linewidth=.9)
    ax.set(title=title, ylabel='N·m·s')
    ax.grid(alpha=.2)
    ax.legend(ncol=3)
axes[-1].set_xlabel('Horloge du banc (s) ; points des résolutions CCD')
args.out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.out, dpi=150)
