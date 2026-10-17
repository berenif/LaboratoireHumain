#!/usr/bin/env python3
"""Render original lateral recovery plus the real checkpoint continuation.

No physics is integrated, and the final frame is not repeated as extra hold.
"""
import json
import os
import sys
from pathlib import Path

os.environ["MPLCONFIGDIR"] = "/tmp/lh-standing-mpl"
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.animation import FFMpegWriter

root = Path(sys.argv[1])
out = root / "outputs/getup-standing/baseline-final-5s"
original = [json.loads(x) for x in (root / "outputs/getup-lateral/trial/trace.ndjson").read_text().splitlines()]
continued = [json.loads(x) for x in (out / "trace.ndjson").read_text().splitlines()]
rows = original + continued
times = np.array([r["snapshot"]["simulation_time_s"] for r in rows])
assert np.all(np.diff(times) > 0)
assert abs(times[-1] - times[len(original) - 1] - 5.0) < 1e-8
model = json.loads((root / "rust/crates/model/data/getup-v1.json").read_text())
az, el = np.deg2rad(-55), np.deg2rad(14)
basis = np.array([[-np.sin(az), 0, np.cos(az)], [-np.sin(el) * np.cos(az), np.cos(el), -np.sin(el) * np.sin(az)], [np.cos(el) * np.cos(az), np.sin(el), np.cos(el) * np.sin(az)]]).T
meshes = [(p["id"], np.array(p["geometry"]["vertices"]).reshape(-1, 3), np.array(p["geometry"]["indices"]).reshape(-1, 3)) for p in model["segments"] if p["geometry"]["vertices"]]


def triangles(row):
    poses = {p["id"]: p for p in row["snapshot"]["segments"]}
    polys, colors = [], []
    for name, vertices, index in meshes:
        p = poses[name]
        rotation = Rotation.from_quat([p["rotation"][k] for k in "xyzw"])
        tri = (rotation.apply(vertices) + [p["position"][k] for k in "xyz"])[index]
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
        shade = .55 + .45 * np.maximum(0, normal @ np.array([-.3, .81, -.5]))
        base = np.array([.73, .39, .24]) if name.endswith("_foot") else np.array([.29, .57, .57])
        polys.append(tri @ basis)
        colors.append(np.clip(shade[:, None] * base, 0, 1))
    polys, colors = np.concatenate(polys), np.concatenate(colors)
    order = np.argsort(polys[:, :, 2].mean(axis=1))
    return polys[order, :, :2], colors[order]


fig = plt.figure(figsize=(12.8, 7.2), facecolor="#f7f8f5")
ax = fig.add_axes([.05, .16, .9, .65])
ax.set_aspect("equal")
ax.axis("off")
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-.4, 1.8)
for value in np.arange(-2, 2.01, .25):
    for ends in [np.array([[-2, 0, value], [2, 0, value]]), np.array([[value, 0, -2], [value, 0, 2]])]:
        p = ends @ basis
        ax.plot(p[:, 0], p[:, 1], color="#d5ddda", lw=.6, zorder=0)
collection = PolyCollection([], closed=True, edgecolors="none", zorder=3)
ax.add_collection(collection)
fig.text(.5, .94, "Départ latéral · maintien prolongé réellement simulé", ha="center", fontsize=19, color="#274745")
clock = fig.text(.5, .88, "", ha="center", fontsize=12)
status = fig.text(.5, .10, "", ha="center", fontsize=12, color="#a25436")
fig.text(.5, .04, "Contrôleur inchangé · aucun blocage artificiel · aucune image finale prolongée", ha="center", fontsize=10, color="#66716d")
frame_times = list(np.arange(times[0], times[-1], .05)) + [times[-1]]
writer = FFMpegWriter(fps=20, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
with writer.saving(fig, str(out / "extended-lateral.mp4"), 100):
    for t in frame_times:
        row = rows[max(0, min(len(rows) - 1, np.searchsorted(times, t, side="right") - 1))]
        poly, color = triangles(row)
        collection.set_verts(poly)
        collection.set_facecolors(color)
        snapshot = row["snapshot"]
        policy = snapshot["diagnostics"]["getup"]["policy"]
        when = snapshot["simulation_time_s"]
        clock.set_text(f"Temps physique : {when:.3f} s · entrée GetUp à 0.250 s")
        label = "Contact latéral, moteurs à zéro" if when < .25 else f"Récupération libre · stabilité continue {policy['stable_standing_s']:.3f} s"
        if when > times[len(original) - 1]:
            label = f"Maintien prolongé calculé · stabilité continue {policy['stable_standing_s']:.3f} s"
        status.set_text(label)
        writer.grab_frame()
    fig.savefig(out / "last-frame.png", dpi=100)
plt.close(fig)
report = {"file": str(out / "extended-lateral.mp4"), "frames": len(frame_times), "fps": 20, "physicalDurationS": float(times[-1]), "continuedPhysicsSeconds": 5.0, "additionalPhysicsIntegrationsForRendering": 0, "repeatedFinalTailFrames": 0}
(out / "render-report.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report))
