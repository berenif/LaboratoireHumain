#!/usr/bin/env python3
"""Render archived source WEAK qpos via pure FK; never imports a simulator.

At 20 fps, the 8.25 s movie contains 165 distinct interval-end samples
(0.050, ..., 8.250 s). The contact sheet additionally shows the true reset
at 0.000 s. There is no interpolation or duplicated terminal hold.
"""
import sys

sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SESSION = ROOT / "outputs/getup-source-engine/session-20261017T233404Z"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path, default=DEFAULT_SESSION / "corrected-attempt/positive-control-frames.ndjson")
    parser.add_argument("--summary", type=Path, default=DEFAULT_SESSION / "corrected-attempt/positive-control-summary.json")
    parser.add_argument("--output", type=Path, default=DEFAULT_SESSION / "render")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(out / ".mpl-cache"))
    os.environ.setdefault("XDG_CACHE_HOME", str(out / ".cache"))

    import numpy as np
    from scipy.spatial.transform import Rotation
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from matplotlib.collections import PolyCollection
    from source_math import P

    fk_path = ROOT / "scripts/getup-step-reference.py"
    spec = importlib.util.spec_from_file_location("source_render_pure_fk", fk_path)
    fk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fk)  # Definitions only; its simulator-using main is not called.
    model_path = ROOT / "rust/crates/model/data/getup-v1.json"
    journal_bytes = args.journal.read_bytes()
    summary_bytes = args.summary.read_bytes()
    model_bytes = model_path.read_bytes()
    rows = [json.loads(line) for line in journal_bytes.splitlines() if line.strip()]
    summary = json.loads(summary_bytes)
    model = json.loads(model_bytes)
    kin = fk.Kinematics(model)
    fps, source_hz = 20, 200
    duration = 8.25
    times = np.array([row["timeS"] for row in rows])
    steps = np.array([row["sourceStep"] for row in rows])
    assert len(rows) == 1651 and np.array_equal(steps, np.arange(1651))
    assert np.allclose(times, steps / source_hz, atol=2e-10, rtol=0)
    assert abs(summary["sourceSimSeconds"] - duration) < 2e-10
    assert summary["sourceMuJoCoSteps"] == 1650
    assert summary["settlingNativeSteps"] == 400 and summary["teacherNativeSteps"] == 1250
    assert all(len(row["qpos"]) == 28 for row in rows)
    video_indices = np.arange(10, 1651, 10, dtype=int)
    sheet_indices = [0, 825, 1650]
    assert len(video_indices) / fps == duration
    assert len(set(video_indices)) == len(video_indices)

    az, el = np.deg2rad([-55, 14])
    basis = np.array([
        [-np.sin(az), 0, np.cos(az)],
        [-np.sin(el) * np.cos(az), np.cos(el), -np.sin(el) * np.sin(az)],
        [np.cos(el) * np.cos(az), np.sin(el), np.cos(el) * np.sin(az)],
    ]).T
    meshes = [(part["id"], np.array(part["geometry"]["vertices"]).reshape(-1, 3),
               np.array(part["geometry"]["indices"]).reshape(-1, 3))
              for part in model["segments"] if part["geometry"]["vertices"]]
    light = np.array([-.3, .81, -.5])
    feet_colors = {"left_foot": np.array([.82, .37, .17]), "right_foot": np.array([.22, .45, .70])}

    def poses_for(row):
        qpos = np.asarray(row["qpos"], dtype=float)
        assert np.isfinite(qpos).all() and abs(np.linalg.norm(qpos[3:7]) - 1) < 1e-10
        root = np.eye(4)
        # qpos source free root is +Z-up, wxyz. Conjugate both local and world bases.
        root[:3, :3] = P.T @ Rotation.from_quat(qpos[[4, 5, 6, 3]]).as_matrix() @ P
        root[:3, 3] = P.T @ qpos[:3]
        return kin.forward(qpos[7:], root)

    needed = sorted(set(video_indices.tolist() + sheet_indices))
    poses_cache = {}
    projected = []
    head_errors, com_errors = [], []
    for index in needed:
        row = rows[index]
        poses = poses_for(row)
        poses_cache[index] = poses
        for name, vertices, _ in meshes:
            pose = poses[name]
            projected.append((vertices @ pose[:3, :3].T + pose[:3, 3]) @ basis)
        head_errors.append(abs(poses["head"][1, 3] - row["physicalObservation"][21]))
        com = sum(part["massKg"] * (poses[part["id"]] @ np.r_[fk.xyz(part["goldenMass"]["centerOfMass"]), 1])[:3]
                  for part in model["segments"]) / model["totalMassKg"]
        com_errors.append(float(np.max(np.abs(P @ com - np.array(row["comSource"])))))
    assert max(head_errors) < 1e-10 and max(com_errors) < 1e-10
    projected = np.concatenate(projected)
    xmin, ymin = projected[:, :2].min(axis=0) - [.15, .13]
    xmax, ymax = projected[:, :2].max(axis=0) + [.15, .13]
    center = (xmin + xmax) / 2
    width = max(xmax - xmin, 1.85)
    xlim, ylim = [center - width / 2, center + width / 2], [ymin, ymax]

    def polygons(index):
        poses = poses_cache[index]
        all_polys, all_colors = [], []
        for name, vertices, triangles in meshes:
            pose = poses[name]
            world = vertices @ pose[:3, :3].T + pose[:3, 3]
            tri = world[triangles]
            normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
            shade = .55 + .45 * np.maximum(0, normal @ light)
            color = feet_colors.get(name, np.array([.29, .57, .57]))
            all_polys.append(tri @ basis)
            all_colors.append(shade[:, None] * color)
        polys, colors = np.concatenate(all_polys), np.concatenate(all_colors)
        order = np.argsort(polys[:, :, 2].mean(axis=1))
        return polys[order, :, :2], colors[order]

    def prepare_axes(ax):
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.axis("off")
        for value in np.arange(-3, 3.01, .25):
            for endpoints in [np.array([[-3, 0, value], [3, 0, value]]),
                              np.array([[value, 0, -3], [value, 0, 3]])]:
                p = endpoints @ basis
                ax.plot(p[:, 0], p[:, 1], color="#d5ddda", lw=.55, zorder=0)

    def phase(row):
        return "Préparation · commandes nulles" if row["sourceStep"] <= 400 else "WEAK figé · relevage source enregistré"

    fig = plt.figure(figsize=(12.8, 7.2), facecolor="#f7f8f5")
    ax = fig.add_axes([.06, .17, .88, .62])
    prepare_axes(ax)
    collection = PolyCollection([], edgecolors="none", zorder=3)
    ax.add_collection(collection)
    fig.text(.5, .952, "MuJoCo WEAK original · témoin positif de relevage", ha="center", fontsize=18, color="#274745")
    clock = fig.text(.5, .899, "", ha="center", fontsize=12, color="#365958")
    status = fig.text(.5, .845, "", ha="center", fontsize=12, color="#725441")
    measurements = fig.text(.5, .117, "", ha="center", fontsize=10, color="#365958")
    fig.text(.5, .066, "2,00 s de préparation + 6,25 s de WEAK · poses source 200 Hz · rendu 20 Hz · aucune physique de rendu",
             ha="center", fontsize=9, color="#66716d")
    fig.text(.5, .027, "Témoin de relevage uniquement · pas complet et station calme non qualifiés", ha="center", fontsize=10, color="#a25436")
    video = out / "source-weak-positive-control.mp4"
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=2600,
                         extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    with writer.saving(fig, str(video), 100):
        for n, index in enumerate(video_indices):
            row = rows[index]
            verts, colors = polygons(int(index))
            collection.set_verts(verts)
            collection.set_facecolors(colors)
            clock.set_text(f"Temps physique source : {row['timeS']:.3f} s · hauteur tête : {row['physicalObservation'][21]:.3f} m")
            status.set_text(phase(row))
            left, right = row["loadsN"]
            measurements.set_text(f"Charges résolues des pieds : G {left:.0f} N / D {right:.0f} N · appui hors pieds : {'oui' if row['nonfootSupport'] else 'non'}")
            writer.grab_frame()
            if n in [0, len(video_indices) // 2, len(video_indices) - 1]:
                print(f"Rendu source : {n + 1}/{len(video_indices)}", flush=True)
    plt.close(fig)
    print(json.dumps({"video": str(video), "frames": len(video_indices), "fps": fps, "durationS": duration}), flush=True)

    fig = plt.figure(figsize=(16, 7.5), facecolor="#f7f8f5")
    fig.text(.5, .951, "Relevage WEAK original · trois poses réellement enregistrées", ha="center", fontsize=19, color="#274745")
    names = ["Départ source", "Milieu enregistré", "Dernière pose réelle"]
    for column, (name, index) in enumerate(zip(names, sheet_indices)):
        row = rows[index]
        x = .025 + column * .33
        ax = fig.add_axes([x, .22, .29, .59])
        prepare_axes(ax)
        verts, colors = polygons(index)
        ax.add_collection(PolyCollection(verts, facecolors=colors, edgecolors="none", zorder=3))
        mid = x + .145
        fig.text(mid, .87, f"{name} · {row['timeS']:.3f} s", ha="center", fontsize=12, color="#274745")
        fig.text(mid, .18, phase(row), ha="center", fontsize=10, color="#725441")
        fig.text(mid, .14, f"Tête : {row['physicalObservation'][21]:.3f} m · pieds G/D : {row['loadsN'][0]:.0f}/{row['loadsN'][1]:.0f} N", ha="center", fontsize=10, color="#365958")
    fig.text(.5, .075, "Même caméra et échelle · instant initial exact et fin réelle inclus · qpos archivés, FK seul, aucune interpolation",
             ha="center", fontsize=10, color="#66716d")
    fig.text(.5, .03, "MuJoCo 2.1.3 · témoin positif de relevage uniquement · aucun pas ni maintien calme qualifié", ha="center", fontsize=11, color="#a25436")
    sheet = out / "source-weak-first-mid-final.png"
    fig.savefig(sheet, dpi=100)
    plt.close(fig)

    hashes = {str(args.journal.resolve()): hashlib.sha256(journal_bytes).hexdigest(),
              str(args.summary.resolve()): hashlib.sha256(summary_bytes).hexdigest(),
              str(model_path): hashlib.sha256(model_bytes).hexdigest(),
              str(fk_path): hashlib.sha256(fk_path.read_bytes()).hexdigest(),
              str(Path(__file__).resolve()): hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    report = {
        "video": str(video), "contactSheet": str(sheet), "frames": len(video_indices), "fps": fps,
        "encodedDurationS": duration, "sourceRealDurationS": float(times[-1]), "sourceHz": source_hz,
        "sourceRecordedRows": len(rows), "videoSourceSteps": video_indices.tolist(),
        "videoSampleConvention": "Distinct interval-end samples at 0.050, ..., 8.250 s; one recorded source row per 50 ms display interval. The true reset at 0.000 s is included in the contact sheet.",
        "contactSheetSourceSteps": sheet_indices, "contactSheetSourceTimesS": [rows[i]["timeS"] for i in sheet_indices],
        "additionalPhysicsIntegrations": 0, "simulatorImportedOrRun": False, "plannedPathUsed": False,
        "poseInterpolation": False, "duplicatedVideoSourceSteps": 0, "terminalTailFramesAdded": 0,
        "headHeightFKParityMaxErrorM": float(max(head_errors)), "bodyCOMFKParityMaxErrorM": float(max(com_errors)),
        "sourceToNativeBasisP": P.tolist(), "sourceRootQuaternionOrder": "wxyz; normalized by scipy Rotation",
        "nativeRootReconstruction": "R_native=P.T@R_source@P; p_native=P.T@p_source; hinge q=qpos[7:]",
        "sameFixedCameraAndWorldScale": True, "cameraAzimuthElevationDeg": [-55, 14],
        "scope": "Original source MuJoCo WEAK get-up positive control only; no step or quiet-standing qualification.",
        "inputSHA256": hashes,
    }
    (out / "source-control-render-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
