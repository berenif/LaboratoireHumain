#!/usr/bin/env python3
"""Render a CLOSED transfer run from archived body poses, without simulation/FK.

Technical prefixes/counterparts and the main history are separate tracks. Copied
parity branches and forecasts never enter actual-motion images, video or CSV.
Missing diagnostics stay missing. No pose interpolation or continuation occurs.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OLD_RENDERER = ROOT / 'docs/evidence/coordination-local-pair-2026-10-13/render_archived.py'
spec = importlib.util.spec_from_file_location('archived_render_helpers', OLD_RENDERER)
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
require, sha, decode, open_text = helpers.require, helpers.sha, helpers.decode, helpers.open_text
HZ = 240


def load_tracks(inputs, trace_path, ids):
    """Read only native rows; stream large forecasts without retaining them."""
    tracks, counts, phase, pair, last_kind = {}, {}, None, 0, None
    excluded = {'copyRows': 0, 'forecastRows': 0, 'forecastEndpointRows': 0}

    def insert(name, label, row, failure=False):
        step = row['nativeSubstep']
        require(type(step) is int and 0 <= step <= 6244, 'Invalid native clock')
        helpers.validate_snapshot(row['snapshot'], step, ids)
        track = tracks.setdefault(name, {'name': name, 'label': label, 'frames': {}, 'refusals': []})
        frame = {'step': step, 'snapshot': row['snapshot'], 'source': row['kind'],
                 'diagnostic': row.get('localDiagnostic'),
                 'com': row.get('support', {}).get('center_of_mass_m'),
                 'owner': row.get('owner')}
        if step in track['frames']:
            require(track['frames'][step]['snapshot']['segments'] == row['snapshot']['segments'],
                    'Conflicting actual body poses at the same native clock')
            if not failure:
                raise ValueError('Duplicate actual row')
        else:
            require(not track['frames'] or step == max(track['frames']) + 1,
                    'Gap in actual poses; refuse to invent motion')
            track['frames'][step] = frame
        if failure:
            track['refusals'].append({'nativeSubstep': step,
                                     'error': row.get('firstFailure', row.get('error'))})

    inputs.record(trace_path)
    with open_text(trace_path) as source:
        for line_index, line in enumerate(source, 1):
            require(bool(line.strip()), f'Empty native trace row {line_index}')
            row = decode(line)
            kind, event = row.get('kind'), row.get('event')
            key = kind or ('event:' + str(event))
            counts[key] = counts.get(key, 0) + 1
            if event == 'prefix-start':
                require(type(row.get('technical')) is bool, 'Missing prefix provenance')
                phase = 'technical' if row['technical'] else 'main'
            elif kind == 'actual-history':
                require(phase in ('technical', 'main'), 'Ambiguous prefix provenance')
                insert('technical-prefix' if phase == 'technical' else 'main-history',
                       'Préfixe technique rejoué — candidat pas encore appliqué' if phase == 'technical'
                       else 'Histoire principale réellement exécutée', row)
            elif kind == 'transfer-verify-copy':
                if last_kind != kind:
                    pair += 1
                excluded['copyRows'] += 1
            elif kind == 'transfer-verify-actual':
                require(pair in (1, 2), 'Technical counterpart lacks its copied pair')
                insert(f'technical-actual-{pair}', f'Contrepartie technique réelle {pair} — aucun relevage qualifié', row)
            elif kind == 'transfer-actual-history':
                require(phase == 'main', 'Candidate row outside the main history')
                insert('main-history', 'Histoire principale réellement exécutée', row)
            elif kind == 'last-failed-state':
                origin = row['source']
                if origin == 'transfer-verify-actual':
                    require(pair in (1, 2), 'Unbound technical failure')
                    insert(f'technical-actual-{pair}', f'Contrepartie technique réelle {pair} — refus', row, True)
                elif origin == 'transfer-actual-history':
                    insert('main-history', 'Histoire principale réellement exécutée — refus', row, True)
                else:
                    raise ValueError('Unknown failed-state provenance: ' + str(origin))
            elif kind == 'prefix-refusal':
                require(phase in ('technical', 'main'), 'Unbound prefix failure')
                insert('technical-prefix' if phase == 'technical' else 'main-history',
                       'Préfixe technique rejoué — refus' if phase == 'technical'
                       else 'Préfixe principal rejoué — refus', row, True)
            elif kind == 'transfer-preview-step':
                excluded['forecastRows'] += 1
            elif kind == 'preview-transfer-end':
                excluded['forecastEndpointRows'] += 1
            elif event != 'forecast-end':
                raise ValueError('Unknown native trace record: ' + str(key))
            last_kind = kind
    for track in tracks.values():
        track['frames'] = [track['frames'][step] for step in sorted(track['frames'])]
    return list(tracks.values()), counts, excluded


def write_metrics(tracks, output):
    fields = ['track', 'provenance', 'native_substep', 'time_s', 'pose_source',
              'left_heel_load_n', 'right_heel_load_n', 'support_fraction',
              'local_passed', 'stable_substeps', 'com_inside', 'capture_inside',
              'maximum_linear_mps', 'maximum_angular_radps', 'com_x_m', 'com_y_m', 'com_z_m']
    with (output / 'actual-measurements.csv').open('x', newline='', encoding='utf-8') as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()
        for track in tracks:
            for frame in track['frames']:
                row = dict(track=track['name'], provenance=track['label'], native_substep=frame['step'],
                           time_s=frame['step']/HZ, pose_source=frame['source'])
                if frame['com'] is not None:
                    row.update(zip(('com_x_m', 'com_y_m', 'com_z_m'), frame['com']))
                d = frame['diagnostic']
                if d is not None:
                    row.update(left_heel_load_n=d['heel_loads_n'][0], right_heel_load_n=d['heel_loads_n'][1],
                               support_fraction=d['support_fraction'], local_passed=d['passed'],
                               stable_substeps=d['consecutive_stable_substeps'], com_inside=d['com_inside'],
                               capture_inside=d['capture_inside'], maximum_linear_mps=d['maximum_linear_mps'],
                               maximum_angular_radps=d['maximum_angular_radps'])
                writer.writerow(row)


def render(tracks, segments, output, stride, video):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-transfer-archive-mpl')
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    meshes = [(np.asarray(s['geometry']['vertices']).reshape(-1, 3),
               np.asarray(s['geometry']['indices']).reshape(-1, 3)) for s in segments]
    cloud = np.asarray([[p['position'][a] for a in 'xzy'] for t in tracks
                        for f in t['frames'] for p in f['snapshot']['segments']])
    radius = max(float(np.linalg.norm(v, axis=1).max()) for v, _ in meshes)
    low, high = cloud.min(axis=0)-radius, cloud.max(axis=0)+radius
    low[2], high[2] = min(0, low[2]), max(1.8, high[2])

    def draw(ax, frame, title):
        for (vertices, triangles), pose in zip(meshes, frame['snapshot']['segments']):
            x, y, z, w = [pose['rotation'][a] for a in 'xyzw']
            k = 2/(x*x+y*y+z*z+w*w)
            rotation = np.array([[1-k*(y*y+z*z), k*(x*y-z*w), k*(x*z+y*w)],
                                 [k*(x*y+z*w), 1-k*(x*x+z*z), k*(y*z-x*w)],
                                 [k*(x*z-y*w), k*(y*z+x*w), 1-k*(x*x+y*y)]])
            placed = vertices @ rotation.T + np.array([pose['position'][a] for a in 'xyz'])
            ax.add_collection3d(Poly3DCollection(placed[:, [0, 2, 1]][triangles],
                                                facecolor='#537d99', edgecolor='none'))
        if frame['com'] is not None:
            x, y, z = frame['com']
            ax.scatter(x, z, y, color='#edb335', s=30)
        ax.add_collection3d(Poly3DCollection([[[low[0], low[1], 0], [high[0], low[1], 0],
                                               [high[0], high[1], 0], [low[0], high[1], 0]]],
                                            facecolor='#d8dee5', alpha=.3))
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
        ax.set_box_aspect(high-low)
        ax.view_init(elev=12, azim=-52)
        ax.set_title(title, fontsize=9)
        ax.tick_params(labelsize=6)

    fig = plt.figure(figsize=(12, 3.8*len(tracks)), dpi=120)
    milestones, videos = {}, {}
    for ti, track in enumerate(tracks):
        frames = track['frames']
        chosen = [frames[i] for i in sorted({0, len(frames)//2, len(frames)-1})]
        milestones[track['name']] = [f['step'] for f in chosen]
        for column, frame in enumerate(chosen):
            ax = fig.add_subplot(len(tracks), 3, ti*3+column+1, projection='3d')
            draw(ax, frame, f"{track['label']}\n{frame['source']} · t={frame['step']/HZ:.6f} s")
    fig.suptitle('États natifs réellement archivés — prévisions exclues')
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(output/'actual-milestones.png')
    plt.close(fig)

    for track in tracks:
        frames = track['frames']
        if not video or len(frames) < 2:
            videos[track['name']] = {'created': False, 'reason': 'disabled' if not video else 'One isolated archived pose; no interval invented'}
            continue
        selected = frames[::stride]
        if selected[-1]['step'] != frames[-1]['step']:
            selected.append(frames[-1])
        path = output/(track['name']+'.mp4')
        command = [shutil.which('ffmpeg'), '-nostdin', '-v', 'error', '-n', '-f', 'rawvideo',
                   '-pixel_format', 'rgba', '-video_size', '960x600', '-framerate', str(HZ),
                   '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '25',
                   '-pix_fmt', 'yuv420p', '-video_track_timescale', '240000', str(path)]
        fig = plt.figure(figsize=(9.6, 6), dpi=100)
        with (output/(track['name']+'-ffmpeg.stderr.log')).open('xb') as log:
            encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
            try:
                for frame, next_frame in zip(selected, selected[1:]):
                    fig.clear()
                    ax = fig.add_subplot(projection='3d')
                    draw(ax, frame, f"{track['label']}\nÉtat archivé {frame['step']}/240 s — poses tenues, sans interpolation")
                    fig.text(.5, .03, 'Prévisions exclues · aucun mouvement extrapolé · dernier état dans la capture',
                             ha='center', fontsize=9)
                    fig.canvas.draw()
                    pixels = fig.canvas.buffer_rgba().tobytes()
                    for _ in range(next_frame['step']-frame['step']):
                        encoder.stdin.write(pixels)
                encoder.stdin.close()
                require(encoder.wait(timeout=60) == 0, 'ffmpeg failed')
            finally:
                if encoder.poll() is None:
                    encoder.kill()
                    encoder.wait(timeout=10)
                plt.close(fig)
        videos[track['name']] = {'created': True, 'sampledSubsteps': [f['step'] for f in selected],
            'encodedFrames': frames[-1]['step']-frames[0]['step'], 'encodedFps': HZ,
            'durationSeconds': (frames[-1]['step']-frames[0]['step'])/HZ,
            'startSubstep': frames[0]['step'], 'endSubstep': frames[-1]['step'],
            'endpointHeldPastEnd': False, 'command': command}
    return {'milestones': milestones, 'videos': videos,
            'numpyVersion': np.__version__, 'matplotlibVersion': matplotlib.__version__}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, default=HERE/'manifest.json')
    parser.add_argument('--output-dir', type=Path, default=HERE/'rendered')
    parser.add_argument('--stride', type=int, default=4)
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    require(1 <= args.stride <= HZ, 'Stride must be 1..240')
    inputs = helpers.Inputs()
    inputs.record(Path(__file__))
    inputs.record(OLD_RENDERER)
    run = args.run_dir.resolve(strict=True)
    closed = inputs.read(run/'run-result.json')
    require(str(closed.get('status', '')).startswith('closed'), 'Run must already be closed')
    budget = closed['budget']
    require(str(budget.get('status', '')).startswith('closed'), 'Budget must already be closed')
    manifest = inputs.read(args.manifest)
    require(sha(args.manifest) == budget['manifestSha256'], 'Manifest differs from the executed one')
    require(Path(manifest['outputPath']) == run, 'Run path differs from executed manifest')
    model = ROOT/'rust/crates/model/data/canonical-v1.json'
    frozen_model = next((r for r in manifest['files'] if Path(r['path']) == model and r['role'] == 'model'), None)
    require(frozen_model is not None and frozen_model['sha256'] == sha(model), 'Canonical display mesh changed')
    segments = inputs.read(model)['segments']
    native_result = inputs.read(run/'native-result.json') if (run/'native-result.json').is_file() else None
    tracks, counts, excluded = load_tracks(inputs, run/'native-trace.jsonl', [s['id'] for s in segments])
    if not args.no_video:
        require(shutil.which('ffmpeg') is not None, 'ffmpeg unavailable; use --no-video')
        inputs.record(Path(shutil.which('ffmpeg')))
    output = args.output_dir.absolute()
    require(output == HERE/'rendered', 'Only the assigned new rendered/ output is allowed')
    output.mkdir(parents=False, exist_ok=False)
    write_metrics(tracks, output)
    recipe = render(tracks, segments, output, args.stride, not args.no_video) if tracks else {'reason': 'No actual archived poses'}
    inputs.verify()
    receipt = {'schema': 1, 'scope': 'Closed native run; actual tracks separately labelled',
        'sourcesSha256': inputs.hashes, 'traceKindCounts': counts, 'excludedPredictionRows': excluded,
        'tracks': [{'name': t['name'], 'label': t['label'], 'actualArchivedPoses': len(t['frames']),
                    'firstNativeSubstep': t['frames'][0]['step'], 'lastNativeSubstep': t['frames'][-1]['step'],
                    'sourceKinds': sorted({f['source'] for f in t['frames']}), 'refusals': t['refusals']}
                   for t in tracks],
        'nativeOutcome': native_result.get('outcome') if native_result else None,
        'nativeRefusal': native_result.get('refusal') if native_result else None,
        'supervisorStatus': closed['status'], 'supervisorError': closed.get('error'),
        'mainHistoryPresent': any(t['name'] == 'main-history' for t in tracks),
        'mainCandidateRows': counts.get('transfer-actual-history', 0),
        'recipe': recipe, 'outputsSha256': {p.name: sha(p) for p in sorted(output.iterdir()) if p.is_file()},
        'additionalNativeSubsteps': 0, 'newFkInspections': 0, 'collisionQueries': 0,
        'measurementPolicy': 'Only archived actual diagnostics; unmeasured values remain blank',
        'videoPolicy': 'Exact stored body poses held for native intervals, no interpolation or post-end hold',
        'fullRecoveryQualified': False}
    with (output/'render-receipt.json').open('x', encoding='utf-8') as target:
        json.dump(receipt, target, indent=2, ensure_ascii=False, allow_nan=False)
        target.write('\n')
    print(json.dumps({'output': str(output), 'additionalNativeSubsteps': 0, 'tracks': len(tracks)}))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print('Archived rendering refused: '+str(error), file=sys.stderr)
        raise SystemExit(1)
