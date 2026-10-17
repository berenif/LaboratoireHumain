#!/usr/bin/env python3
"""Render retained poses only. No physics import, integration, or interpolation."""
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-bounded-trial-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/lh-bounded-trial-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter

root = Path(__file__).resolve().parents[2]
out = Path(sys.argv[1]).resolve(strict=True)
destination = Path(sys.argv[2]).resolve()
assert not destination.exists()
rows = [json.loads(line) for line in (out/'trace.ndjson').read_text().splitlines()]
audit = json.loads((out/'postrun-audit.json').read_text())
assert len(rows) == 2 and audit['actualConsumedNativeSteps'] == 20
assert audit['actualForecastNativeSteps'] == 0 and not audit['impactOccurred']
model = json.loads((root/'rust/crates/model/data/getup-v1.json').read_text())
az, el = np.deg2rad(-55), np.deg2rad(12)
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


fig = plt.figure(figsize=(12.8, 7.2), facecolor='#f7f8f5')
ax = fig.add_axes([.08, .16, .84, .66])
ax.set_aspect('equal')
ax.set_xlim(-.9, .9)
ax.set_ylim(-.12, 1.62)
ax.axis('off')
for value in np.arange(-1, 1.01, .25):
    for endpoints in [np.array([[-1, 0, value], [1, 0, value]]),
                      np.array([[value, 0, -1], [value, 0, 1]])]:
        projected = endpoints @ basis
        ax.plot(projected[:, 0], projected[:, 1], color='#d5ddda', lw=.6, zorder=0)
collection = PolyCollection([], closed=True, edgecolors='none', zorder=3)
ax.add_collection(collection)
fig.text(.5, .94, 'Rapier · essai interrompu avant impact', ha='center', fontsize=18, color='#274745')
title = fig.text(.5, .88, '', ha='center', fontsize=12, color='#365958')
fig.text(.5, .095, '20 sous-pas réels · 0 prévision · 0 décision SLOW · relevage non évalué',
         ha='center', fontsize=12, color='#a25436')
fig.text(.5, .045, 'Deux états archivés maintenus chacun 1 s pour lecture · durée physique totale : 0,025 s',
         ha='center', fontsize=10, color='#66716d')
fig.text(.5, .015, 'Défaut du compteur du lanceur · aucun échec numérique observé · aucun calcul physique de rendu',
         ha='center', fontsize=9, color='#66716d')
writer = FFMpegWriter(fps=20, codec='libx264', extra_args=[
    '-pix_fmt', 'yuv420p', '-color_range', 'tv', '-movflags', '+faststart'])
with writer.saving(fig, str(destination), 100):
    for index in range(40):
        row = rows[index//20]
        polygons, colors = project(row)
        collection.set_verts(polygons)
        collection.set_facecolors(colors)
        title.set_text(f"Temps physique enregistré : {row['simulation_time_s']:.3f} s · tick {row['stamp']['tick']}")
        writer.grab_frame()
    fig.savefig(out/'last-rendered-frame.png', dpi=100)
plt.close(fig)
print(json.dumps({'file':str(destination), 'frames':40, 'fps':20,
                  'physicalDurationS':.025, 'presentationDurationS':2,
                  'archivedStatesOnly':True, 'additionalPhysicsIntegrations':0}))
