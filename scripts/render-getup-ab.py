#!/usr/bin/env python3
"""Render archived native A/B poses only; zero physics integrations."""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/lh-render-cache')
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
repo = Path(__file__).resolve().parents[1]
parts = json.loads((repo/'rust/crates/model/data/canonical-v1.json').read_text())['segments']
meshes = [(np.array(s['geometry']['vertices']).reshape(-1, 3), np.array(s['geometry']['indices']).reshape(-1, 3)) for s in parts]
histories = []
for folder, label in [('000-witness-A', 'A · témoin sans correction'), ('001-feedback-B', 'B · correction coordonnée')]:
    path = args.root/folder
    report = json.loads((path/'report.json').read_text())
    frames = [json.loads(l)['physical'] for l in (path/'frames.jsonl').open()]
    frames.insert(0, {'timeS': 0.0, 'snapshot': report['initial'], 'resolvedContacts': []})
    frames.append({'timeS': report['physicalSeconds'], 'snapshot': report['final'], 'resolvedContacts': [], 'final': True})
    rows = [json.loads(l) for l in (path/'substeps.jsonl').open()]
    histories.append((label, report, frames, rows))


def draw(ax, history, time):
    label, report, frames, _ = history
    f = max((f for f in frames if f['timeS'] <= time+1e-8), key=lambda f: f['timeS'])
    loaded = {c['id'] for c in f.get('resolvedContacts', []) if c['persistent'] and c['normalLoadN'] >= 3}
    for part, (v, indices), pose in zip(parts, meshes, f['snapshot']['segments']):
        rot = Rotation.from_quat([pose['rotation'][k] for k in 'xyzw'])
        vertices = (rot.apply(v)+np.array([pose['position'][k] for k in 'xyz']))[:, [0, 2, 1]]
        ax.add_collection3d(Poly3DCollection(vertices[indices], facecolor='#bd4a36' if part['id'] in loaded else '#6f88a4', edgecolor='none', alpha=.9))
    ax.add_collection3d(Poly3DCollection([[[-1.05,-.4,0],[.85,-.4,0],[.85,2.1,0],[-1.05,2.1,0]]], facecolor='#d8dee5', alpha=.4))
    if 'com' in f:
        ax.scatter(f['com']['x'], f['com']['z'], f['com']['y'], color='#daa520', s=16)
    ax.set(xlim=(-1.05,.85), ylim=(-.4,2.1), zlim=(-.05,1.95))
    ax.set_box_aspect([1.9,2.5,2.0])
    ax.view_init(elev=13, azim=-60)
    ax.tick_params(labelsize=6)
    ended = time >= report['physicalSeconds']
    refusal = report['firstRefusal']
    if report['fullRecoverySuccess']:
        verdict = 'relevage physique complet'
    elif refusal and 'stagnation' in refusal['offending']:
        verdict = 'arrêt : stagnation physique (3 s)'
    elif refusal and 'recovery observer refused' in refusal['offending']:
        verdict = 'terminal debout refusé'
    else:
        verdict = 'aucun relevage'
    ax.set_title(label+'\n'+f"état enregistré à {f['timeS']:.3f} s"+('\n'+verdict if ended else ''), fontsize=9)


last = max(h[1]['physicalSeconds'] for h in histories)
fig = plt.figure(figsize=(12,6))
axes = [fig.add_subplot(1,2,i+1,projection='3d') for i in range(2)]
fig.text(.5,.025,'Lecture environ 4× · temps physique affiché · états arrêtés fixes · rouge : contacts persistants chargés',ha='center',fontsize=8)
writer = FFMpegWriter(fps=20, codec='libx264', extra_args=['-pix_fmt','yuv420p'])
times = np.append(np.arange(0,last,.2),last)
with writer.saving(fig, str(args.output/'getup-ab-recorded.mp4'), 110):
    for t in times:
        for ax, history in zip(axes, histories):
            ax.clear(); draw(ax, history, float(t))
        fig.suptitle('Comparaison Rapier A/B · poses physiques archivées', fontsize=13)
        writer.grab_frame()
plt.close(fig)

fig = plt.figure(figsize=(14,8))
for i, history in enumerate(histories):
    for j,t in enumerate([.05, min(1.5,history[1]['physicalSeconds']), history[1]['physicalSeconds']]):
        draw(fig.add_subplot(2,3,i*3+j+1,projection='3d'), history, t)
fig.suptitle('États Rapier archivés · rouge : surfaces chargées et persistantes', fontsize=13)
fig.text(.5,.02,'Les états arrêtés restent fixes ; aucune trajectoire ultérieure extrapolée. Aucun relevage qualifié.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.05,1,.94)); fig.savefig(args.output/'getup-ab-milestones.png',dpi=140); plt.close(fig)

fig, axes = plt.subplots(4,1,figsize=(10,9),sharex=True)
for label, report, _, rows in histories:
    t = [r['timeS'] for r in rows]
    axes[0].plot(t,[r['heightM'] for r in rows],label=label)
    axes[1].plot(t,[r['command'].get('maximumNominalTrackingErrorRad',np.nan) for r in rows])
    axes[2].plot(t,[r['acceleration']['z'] for r in rows])
    axes[3].plot(t,[r['deliveredAbsoluteNm'] for r in rows])
axes[0].set_ylabel('Bassin (m)'); axes[0].legend(fontsize=8)
axes[1].set_ylabel('Suivi nominal max\n(rad)'); axes[2].set_ylabel('Accélération CoM Z\n(m/s²)')
axes[3].set_ylabel('Couples Σ absolue\n(Nm)'); axes[3].set_xlabel('Temps physique (s)')
fig.suptitle('Résultats natifs A/B · diagnostic de suivi distinct du terminal physique')
fig.tight_layout();fig.savefig(args.output/'getup-ab-comparison.png',dpi=140);plt.close(fig)
(args.output/'visuals.json').write_text(json.dumps({'usesRecordedNativePoses':True,'additionalPhysicsIntegrations':0,
    'engineScreenshot':False,'videoFrames':len(times),'videoFps':20,'approximatePlaybackSpeed':4,
    'afterRefusal':'last archived state held with explicit stop label; no extrapolation',
    'outcomes':{h[0]:{'physicalSeconds':h[1]['physicalSeconds'],'completeRecovery':h[1]['fullRecoverySuccess']} for h in histories}},indent=2)+'\n')
