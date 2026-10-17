#!/usr/bin/env python3
"""Render actual archived recovery and an explicitly separate forecast summary."""
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-bounded-recovery-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/lh-bounded-recovery-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter

began = time.monotonic()
root = Path(__file__).resolve().parents[2]
out = Path(sys.argv[1]).resolve(strict=True)
destination = Path(sys.argv[2]).resolve()
assert not destination.exists()
rows = [json.loads(line) for line in (out/'trace.ndjson').read_text().splitlines()]
reference = json.loads((out/'reference-report.json').read_text())
audit = json.loads((out/'postrun-audit.json').read_text())
assert len(rows) == 161 and rows[-1]['simulation_time_s'] == 4
assert audit['slowDecisions'] == 16 and audit['forecastAdmitted']
model = json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
az, el = np.deg2rad(-55), np.deg2rad(14)
right = [-np.sin(az), 0, np.cos(az)]
up = [-np.sin(el)*np.cos(az), np.cos(el), -np.sin(el)*np.sin(az)]
depth = [np.cos(el)*np.cos(az), np.sin(el), np.cos(el)*np.sin(az)]
basis = np.array([right, up, depth]).T
meshes = [(p['id'], np.array(p['geometry']['vertices']).reshape(-1, 3),
           np.array(p['geometry']['indices']).reshape(-1, 3))
          for p in model['segments'] if p['geometry']['vertices']]


def project(row):
    poses = {p['id']:p for p in row['segments']}
    polys, colors = [], []
    for name, vertices, indices in meshes:
        p = poses[name]
        rotation = Rotation.from_quat([p['rotation'][k] for k in 'xyzw'])
        triangles = (rotation.apply(vertices) + [p['position'][k] for k in 'xyz'])[indices]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
        shade = .55 + .45*np.maximum(0, normals @ np.array([-.3, .81, -.5]))
        base = np.array([.73, .39, .24]) if name.endswith('_foot') else np.array([.29, .57, .57])
        polys.append(triangles @ basis)
        colors.append(np.clip(shade[:, None]*base, 0, 1))
    polys, colors = np.concatenate(polys), np.concatenate(colors)
    order = np.argsort(polys[:, :, 2].mean(axis=1))
    return polys[order, :, :2], colors[order]


head = lambda row:next(p['position']['y'] for p in row['segments'] if p['id']=='head')
times = np.array([r['simulation_time_s'] for r in rows])
heads = np.array([head(r) for r in rows])
fig = plt.figure(figsize=(12.8, 7.2), facecolor='#f7f8f5')
ax = fig.add_axes([.03, .18, .59, .65])
ax.set_aspect('equal')
ax.set_xlim(-1.25, 1.25)
ax.set_ylim(-.5, 1.8)
ax.axis('off')
for value in np.arange(-1.5, 1.51, .25):
    for endpoints in [np.array([[-1.5, 0, value], [1.5, 0, value]]),
                      np.array([[value, 0, -1.5], [value, 0, 1.5]])]:
        projected = endpoints @ basis
        ax.plot(projected[:, 0], projected[:, 1], color='#d5ddda', lw=.6, zorder=0)
collection = PolyCollection([], closed=True, edgecolors='none', zorder=3)
ax.add_collection(collection)
real_plot = fig.add_axes([.68, .55, .28, .24])
real_plot.set(xlim=(0, 4), ylim=(0, 1.7), xlabel='Temps réel (s)', ylabel='Tête (m)')
real_plot.set_title('Histoire réelle', fontsize=11)
real_plot.axvline(4-reference['status']['since_impact_s'], color='#b76640', ls=':', label='Impact')
real_plot.axvline(audit['slowStartS'], color='#7b638e', ls=':', label='Début SLOW')
line, = real_plot.plot([], [], color='#218980', lw=2)
real_plot.legend(fontsize=7, loc='lower left')
real_plot.grid(alpha=.2)
forecast_plot = fig.add_axes([.68, .23, .28, .21])
forecast_plot.plot([0]+[f['time_s'] for f in reference['teacher_frames']],
                   [head(reference['teacher_initial'])]+[head(f['snapshot']) for f in reference['teacher_frames']],
                   color='#b76640', lw=2)
forecast_plot.axhline(1.4, color='#79848b', ls='--', lw=.8)
forecast_plot.set(xlim=(0, .7), ylim=(0, 1.7), xlabel='Temps local prédit (s)', ylabel='Tête (m)')
forecast_plot.set_title('Résumé WEAK · copie du monde', fontsize=11)
forecast_plot.grid(alpha=.2)
fig.text(.5, .95, 'Rapier · prévision admise, relevage non démontré', ha='center', fontsize=18, color='#274745')
title = fig.text(.32, .86, '', ha='center', fontsize=12, color='#365958')
phase = fig.text(.32, .15, '', ha='center', fontsize=11, color='#365958')
fig.text(.5, .085, 'WEAK : 540 sous-pas · SLOW réel : 16 décisions / 0,399 s · stabilité debout : 0 s',
         ha='center', fontsize=11, color='#a25436')
fig.text(.5, .035, '4 s physiques maximum · dernière pose maintenue 1 s · poses archivées sans interpolation · aucun calcul physique de rendu',
         ha='center', fontsize=9, color='#66716d')
writer = FFMpegWriter(fps=20, codec='libx264', extra_args=[
    '-pix_fmt', 'yuv420p', '-color_range', 'tv', '-movflags', '+faststart'])
presentation = [i/20 for i in range(80)]+[4.0]*20
with writer.saving(fig, str(destination), 100):
    for t in presentation:
        index = max(i for i, x in enumerate(times) if x <= t+1e-9)
        row = rows[index]
        polygons, colors = project(row)
        collection.set_verts(polygons)
        collection.set_facecolors(colors)
        line.set_data(times[:index+1], heads[:index+1])
        title.set_text(f"Monde réel enregistré · {row['simulation_time_s']:.3f} s · tête {head(row):.3f} m")
        label = 'Fin bornée à 4 s · corps encore couché' if t == 4 else (
            'SLOW appliqué au corps réel' if row['diagnostics']['getup']['policy']['policy_decisions'] else 'Avant déclenchement de SLOW')
        phase.set_text(label)
        writer.grab_frame()
    fig.savefig(out/'last-rendered-frame.png', dpi=100)
plt.close(fig)
print(json.dumps({'file':str(destination), 'frames':100, 'fps':20,
                  'physicalDurationS':4, 'presentationDurationS':5,
                  'archivedStatesOnly':True, 'additionalPhysicsIntegrations':0,
                  'renderWallSeconds':time.monotonic()-began}))
