#!/usr/bin/env python3
"""Render CLOSED A/B archives only; never import or start a simulation.

Usage after the native lot has closed:
  python render_archived.py --output-dir /absolute/new/render-directory

Optional: --run-dir DIR --stride 12 --no-video. Requires numpy, matplotlib and
ffmpeg (libx264) for video. Initial/final poses come from the history report;
intermediate poses/measurements come only from top-level actual-history rows.
Preview rows are excluded. Missing measurements stay empty, never become zero.

Videos use stored poses held between sampled native clocks, without pose
interpolation. Each hold lasts an integer number of 1/240-second frames, so the
movie duration is exactly the covered interval. The last endpoint has no added
hold; its exact archived pose appears in the milestone PNG and CSV. No physics,
FK, collision query, CoM recomputation or invented continuation is performed.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
HZ = 240
MAX_STEPS = 960


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def decode(text):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, 'Duplicate JSON key: ' + key)
            result[key] = value
        return result
    def nonfinite(value):
        raise ValueError('Non-finite JSON number: ' + value)
    def finite_float(value):
        parsed = float(value)
        require(math.isfinite(parsed), 'Non-finite JSON float')
        return parsed
    return json.loads(text, object_pairs_hook=unique, parse_constant=nonfinite, parse_float=finite_float)


def open_text(path):
    return gzip.open(path, 'rt', encoding='utf-8') if path.suffix == '.gz' else path.open(encoding='utf-8')


def existing(path):
    if path.is_file():
        return path
    compressed = Path(str(path) + '.gz')
    return compressed if compressed.is_file() else None


class Inputs:
    def __init__(self):
        self.hashes = {}

    def record(self, path):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        require(str(path) not in self.hashes or self.hashes[str(path)] == digest, 'Source changed')
        self.hashes[str(path)] = digest
        return path

    def read(self, path):
        path = self.record(path)
        with open_text(path) as source:
            return decode(source.read())

    def verify(self):
        for path, digest in self.hashes.items():
            require(sha(path) == digest, 'Source changed during rendering: ' + path)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'Invalid archived scalar')
    return value


def validate_snapshot(snapshot, step, ids):
    require(isinstance(snapshot, dict), 'Missing archived snapshot')
    poses = snapshot.get('segments', [])
    require([pose.get('id') for pose in poses] == ids and len(poses) == 25, 'Expected the same 25 body IDs')
    require(abs(number(snapshot['simulation_time_s']) - step / HZ) <= 1e-5, 'Snapshot/native clock mismatch')
    for pose in poses:
        for key, axes in [('position', 'xyz'), ('rotation', 'xyzw')]:
            for axis in axes:
                number(pose[key][axis])
        require(sum(pose['rotation'][axis] ** 2 for axis in 'xyzw') > 0, 'Zero archived quaternion')


def load_history(inputs, run, name, index, ids):
    report_path = existing(run / f'{index:03d}-{name}.json')
    trace_path = existing(run / f'{index:03d}-{name}.states.jsonl')
    report = inputs.read(report_path) if report_path else None
    frames, counts = {}, {}
    actual = None
    if report is not None:
        require(report.get('id') == name, 'History report ID mismatch')
        actual = report['actualNativeSteps']
        require(actual is None or type(actual) is int and 0 <= actual <= MAX_STEPS, 'Invalid actual history count')
        endpoints = [(0, 'initial')]
        if actual is not None:
            endpoints.append((actual, 'final'))
        # An unknown receipt count must stay unknown. In that case retain only
        # its initial pose and independently clocked actual-history rows;
        # do not infer a final native count from an unqualified final snapshot.
        for step, key in endpoints:
            snapshot = report[key]
            validate_snapshot(snapshot, step, ids)
            if step == 0 and 0 in frames:
                require(snapshot['segments'] == frames[0]['snapshot']['segments'], 'Zero-step final pose differs')
                continue
            frames[step] = {'step': step, 'snapshot': snapshot, 'diagnostic': None,
                            'com': None, 'source': 'report-' + key}
    seen_steps = []
    if trace_path:
        inputs.record(trace_path)
        with open_text(trace_path) as source:
            for line_index, line in enumerate(source, 1):
                require(bool(line.strip()), f'Empty trace row {line_index}')
                row = decode(line)
                kind = row.get('kind')
                require(isinstance(kind, str), 'Trace kind missing')
                counts[kind] = counts.get(kind, 0) + 1
                if kind != 'actual-history':
                    continue  # In particular, no preview state enters video or CSV.
                step = row['nativeSubstep']
                require(type(step) is int and 1 <= step <= MAX_STEPS, 'Invalid actual substep')
                require(step == (seen_steps[-1] + 1 if seen_steps else 1), 'Gap or duplicate in actual trace')
                require(actual is None or step <= actual, 'Trace exceeds final actual count')
                validate_snapshot(row['snapshot'], step, ids)
                diagnostic = row['localDiagnostic']
                require(type(diagnostic['passed']) is bool, 'Missing local passed flag')
                require(len(diagnostic['heel_loads_n']) == 2, 'Missing heel loads')
                for value in diagnostic['heel_loads_n']:
                    number(value)
                com = row['support']['center_of_mass_m']
                require(isinstance(com, list) and len(com) == 3, 'Missing measured CoM')
                for value in com:
                    number(value)
                if step in frames:
                    require(row['snapshot']['segments'] == frames[step]['snapshot']['segments'], 'Final trace/report body poses differ')
                frames[step] = {'step': step, 'snapshot': row['snapshot'], 'diagnostic': diagnostic,
                                'com': com, 'source': 'actual-history'}
                seen_steps.append(step)
    ordered = [frames[step] for step in sorted(frames)]
    # A terminal integrity refusal can produce one final snapshot without a
    # diagnostic row. Larger gaps must never become a held/interpolated history.
    require(not ordered or [f['step'] for f in ordered] == list(range(ordered[0]['step'], ordered[-1]['step'] + 1)),
            'Missing archived poses: refuse to invent an interval')
    return {'name': name, 'actualNativeStepsReported': actual, 'frames': ordered,
            'traceKindCounts': counts, 'actualRows': len(seen_steps),
            'lastArchivedSubstepUsed': ordered[-1]['step'] if ordered else None,
            'finalSnapshotOmittedForUnknownCount': report is not None and actual is None,
            'refusal': report.get('refusal') if report else 'History report unavailable',
            'outcome': report.get('outcome') if report else None}


def csv_rows(history, path):
    fields = ['history', 'native_substep', 'time_s', 'pose_source', 'left_heel_load_n',
              'right_heel_load_n', 'com_x_m', 'com_y_m', 'com_z_m', 'local_passed',
              'consecutive_stable_substeps', 'support_fraction']
    with path.open('x', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for frame in history['frames']:
            row = dict(history=history['name'], native_substep=frame['step'], time_s=frame['step'] / HZ,
                       pose_source=frame['source'])
            if frame['com'] is not None:
                row.update(zip(['com_x_m', 'com_y_m', 'com_z_m'], frame['com']))
            if frame['diagnostic'] is not None:
                d = frame['diagnostic']
                row.update(left_heel_load_n=d['heel_loads_n'][0], right_heel_load_n=d['heel_loads_n'][1],
                           local_passed=d['passed'], consecutive_stable_substeps=d['consecutive_stable_substeps'],
                           support_fraction=d['support_fraction'])
            writer.writerow(row)


def render(histories, segments, output, stride, video):
    # Only rendering libraries are imported; the simulation is never imported.
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-local-pair-archive-mpl')
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    meshes = [(np.asarray(s['geometry']['vertices']).reshape(-1, 3),
               np.asarray(s['geometry']['indices']).reshape(-1, 3)) for s in segments]
    positions = [[pose['position'][axis] for axis in 'xzy']
                 for h in histories for f in h['frames'] for pose in f['snapshot']['segments']]
    radius = max(float(np.linalg.norm(v, axis=1).max()) for v, _ in meshes)
    cloud = np.asarray(positions or [[0, 0, 1]])
    low, high = cloud.min(axis=0) - radius, cloud.max(axis=0) + radius
    low[2] = min(low[2], 0)
    high[2] = max(high[2], 1.8)

    def draw(ax, frame, title):
        for (vertices, triangles), pose in zip(meshes, frame['snapshot']['segments']):
            x, y, z, w = [pose['rotation'][axis] for axis in 'xyzw']
            factor = 2 / (x*x + y*y + z*z + w*w)
            rotation = np.array([[1-factor*(y*y+z*z), factor*(x*y-z*w), factor*(x*z+y*w)],
                                 [factor*(x*y+z*w), 1-factor*(x*x+z*z), factor*(y*z-x*w)],
                                 [factor*(x*z-y*w), factor*(y*z+x*w), 1-factor*(x*x+y*y)]])
            translated = vertices @ rotation.T + np.array([pose['position'][axis] for axis in 'xyz'])
            ax.add_collection3d(Poly3DCollection(translated[:, [0, 2, 1]][triangles],
                                                facecolor='#537d99', edgecolor='none', alpha=.95))
        if frame['com'] is not None:
            x, y, z = frame['com']
            ax.scatter(x, z, y, color='#edb335', s=30)
        ax.add_collection3d(Poly3DCollection([[[low[0], low[1], 0], [high[0], low[1], 0],
                                               [high[0], high[1], 0], [low[0], high[1], 0]]],
                                            facecolor='#d8dee5', alpha=.3))
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
        ax.set_box_aspect(high-low)
        ax.view_init(elev=12, azim=-52)
        ax.set_title(title, fontsize=10)
        ax.tick_params(labelsize=6)

    fig = plt.figure(figsize=(12, 8), dpi=120)
    milestones = {}
    for index, history in enumerate(histories):
        frames = history['frames']
        chosen = [frames[i] for i in sorted({0, len(frames)//2, len(frames)-1})] if frames else []
        milestones[history['name']] = [f['step'] for f in chosen]
        for col in range(3):
            ax = fig.add_subplot(2, 3, index*3+col+1, projection='3d')
            if col < len(chosen):
                frame = chosen[col]
                draw(ax, frame, f"{history['name']} · pose archivée {frame['step']}/{HZ} s\n{frame['source']}")
            else:
                ax.set_axis_off()
                if col == len(chosen):
                    text = 'Aucune histoire réelle archivée' if len(frames) <= 1 else ''
                    count = history['actualNativeStepsReported']
                    ax.text2D(.05, .6, f"{history['name']} — {text}\nCompte rapporté : {count if count is not None else 'inconnu'}\nPrévisions exclues", transform=ax.transAxes)
    fig.suptitle('États réels archivés — aucune continuation inventée')
    fig.text(.5, .025, '25 poses mesurées · jaune : CoM archivé · diagnostic local ≠ relevage qualifié', ha='center')
    fig.tight_layout(rect=(0, .05, 1, .94))
    fig.savefig(output / 'milestones.png')
    plt.close(fig)

    videos = {}
    for history in histories:
        frames = history['frames']
        if not video or len(frames) < 2:
            videos[history['name']] = {'created': False, 'reason': 'disabled' if not video else 'No real history interval archived'}
            continue
        selected = frames[::stride]
        if selected[-1]['step'] != frames[-1]['step']:
            selected.append(frames[-1])
        path = output / f"{history['name']}-actual-history.mp4"
        fig = plt.figure(figsize=(9.6, 6), dpi=100)
        command = [shutil.which('ffmpeg'), '-nostdin', '-v', 'error', '-n', '-f', 'rawvideo',
                   '-pixel_format', 'rgba', '-video_size', '960x600', '-framerate', str(HZ),
                   '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '25',
                   '-pix_fmt', 'yuv420p', '-video_track_timescale', '240000', str(path)]
        log = output / f"{history['name']}-ffmpeg.stderr.log"
        with log.open('xb') as stderr:
            encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=stderr)
            try:
                for frame, following in zip(selected, selected[1:]):
                    fig.clear()
                    ax = fig.add_subplot(projection='3d')
                    draw(ax, frame, f"{history['name']} — état réel archivé · t = {frame['step']/HZ:.6f} s\n"
                         f"Échantillonnage ≤ {stride}/{HZ} s ; poses tenues, sans interpolation")
                    fig.text(.5, .03, 'Aucune prévision représentée · dernier état archivé dans milestones.png', ha='center', fontsize=9)
                    fig.canvas.draw()
                    pixels = fig.canvas.buffer_rgba().tobytes()
                    for _ in range(following['step'] - frame['step']):
                        encoder.stdin.write(pixels)
                encoder.stdin.close()
                require(encoder.wait(timeout=60) == 0, 'ffmpeg failed; inspect its stderr archive')
            finally:
                if encoder.poll() is None:
                    encoder.kill()
                    encoder.wait(timeout=10)
                plt.close(fig)
        videos[history['name']] = {'created': True, 'path': str(path), 'command': command,
                                  'sampledSubsteps': [f['step'] for f in selected], 'encodedFps': HZ,
                                  'startSubstep': frames[0]['step'], 'endSubstep': frames[-1]['step'],
                                  'encodedFrames': frames[-1]['step']-frames[0]['step'],
                                  'durationSeconds': (frames[-1]['step']-frames[0]['step'])/HZ,
                                  'archivedEndpointNotHeldPastEnd': True}
    return {'milestoneSubsteps': milestones, 'videos': videos,
            'numpyVersion': np.__version__, 'matplotlibVersion': matplotlib.__version__}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', type=Path, default=HERE/'native-run')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=12)
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    require(1 <= args.stride <= HZ, 'Stride must be 1..240 native steps')
    inputs = Inputs()
    run = args.run_dir.resolve(strict=True)
    result = inputs.read(run/'run-result.json')
    require(result.get('status') in ('protocol-completed', 'failed-closed'), 'Run has not closed')
    require(str(result.get('budget', {}).get('status', '')).startswith('closed'), 'Budget has not closed')
    preflight = inputs.read(run/'static-preflight.json')
    manifest_path = Path(preflight['manifestPath'])
    manifest = inputs.read(manifest_path)
    require(sha(manifest_path) == preflight['manifestSha256'], 'Frozen manifest changed')
    model_path = ROOT/'rust/crates/model/data/canonical-v1.json'
    models = manifest['files']['model']
    model_entry = next((v for v in models if Path(v['path']).resolve() == model_path), None)
    require(model_entry is not None and sha(model_path) == model_entry['sha256'], 'Canonical mesh not frozen unchanged')
    segments = inputs.read(model_path)['segments']
    ids = [s['id'] for s in segments]
    histories = [load_history(inputs, run, name, index, ids) for index, name in enumerate(['A', 'B'], 1)]
    if not args.no_video:
        require(shutil.which('ffmpeg') is not None, 'ffmpeg required; use --no-video for PNG/CSV only')
        inputs.record(Path(shutil.which('ffmpeg')))
    output = args.output_dir.absolute()
    output.mkdir(parents=False, exist_ok=False)
    inputs.record(Path(__file__))
    for history in histories:
        csv_rows(history, output/f"{history['name']}-measurements.csv")
    recipe = render(histories, segments, output, args.stride, not args.no_video)
    inputs.verify()
    receipt = {'schema': 1, 'scope': 'archived actual A/B states only', 'sourcesSha256': inputs.hashes,
               'histories': [{k: v for k, v in h.items() if k != 'frames'} for h in histories],
               'recipe': recipe, 'recipeSha256': hashlib.sha256(json.dumps(recipe, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
               'outputsSha256': {p.name: sha(p) for p in sorted(output.iterdir()) if p.is_file()},
               'additionalNativeSubsteps': 0, 'newFkInspections': 0, 'collisionQueries': 0,
               'measurementPolicy': 'CSV keeps all archived actual rows; report-only poses have blank unmeasured values',
               'videoPolicy': 'Stored poses held on exact native clock intervals; no interpolation, no preview and no post-end hold',
               'fullRecoveryQualified': False}
    with (output/'render-receipt.json').open('x', encoding='utf-8') as target:
        json.dump(receipt, target, indent=2, allow_nan=False)
        target.write('\n')
    print(json.dumps({'output': str(output), 'nativeSubsteps': 0, 'receipt': str(output/'render-receipt.json')}))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print('Archived rendering refused: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
