#!/usr/bin/env python3
"""Plot only archived local contacts at the refused forecast endpoint; no FK."""
import gzip
import hashlib
import json
import math
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = HERE / 'native-run-once'
OUT = HERE / 'rendered'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    result_path = RUN/'run-result.json'
    result = json.loads(result_path.read_text())
    assert result['status'] == 'closed-refused'
    assert result['budget']['status'] == 'closed-refused'
    manifest_path = HERE/'manifest.json'
    assert sha(manifest_path) == result['budget']['manifestSha256']
    trace_path = RUN/'native-trace.jsonl'
    if not trace_path.is_file():
        trace_path = RUN/'native-trace.jsonl.gz'
    sources = {str(p): sha(p) for p in (Path(__file__).resolve(), result_path, manifest_path, trace_path)}
    opener = gzip.open if trace_path.suffix == '.gz' else open
    endpoint = None
    endpoint_line = None
    with opener(trace_path, 'rt', encoding='utf-8') as source:
        for index, line in enumerate(source, 1):
            row = json.loads(line)
            if row.get('kind') == 'preview-transfer-end':
                assert endpoint is None, 'This figure is specific to the sole refused forecast'
                endpoint, endpoint_line = row, index
    assert endpoint is not None and endpoint['nativeSubstep'] == 78
    decision = endpoint['owner']['transfer']['decision']
    previous = decision['policy']['latest']
    assert decision['lastObserved'] == previous['native_substep'] == 77
    initial = next(a for a in decision['policy']['initial_material_obligations'] if a['segment'] == 19)
    point = initial['local_point_m']
    old = next(s for s in previous['segments'] if s['segment'] == 19)
    new = next(s for s in endpoint['support']['segments'] if s['segment'] == 19)
    assert old['id'] == new['id'] == 'leftForefoot'
    assert len(old['contacts']) == 4 and len(new['contacts']) == 3
    tolerance = .012
    panels = []
    for clock, segment in ((77, old), (78, new)):
        contacts = [{**c, 'distanceToRetainedPointM': math.dist(point, c['local_point_m'])}
                    for c in segment['contacts']]
        panels.append({'nativeSubstep': clock, 'contacts': contacts,
                       'totalCurrentUpwardLoadN': sum(c['current_upward_load_n'] for c in contacts),
                       'nearestRetainedPointDistanceM': min(c['distanceToRetainedPointM'] for c in contacts),
                       'retainedPointPresentWithin12mm': any(c['distanceToRetainedPointM'] <= tolerance for c in contacts)})
    assert panels[0]['retainedPointPresentWithin12mm'] and not panels[1]['retainedPointPresentWithin12mm']

    os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-transfer-contact-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.lines import Line2D

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 6.7), dpi=140)
    for ax, panel in zip(axes, panels):
        ax.set_facecolor('#f8fafc')
        ax.add_patch(Circle((1000*point[0], 1000*point[2]), 12, fill=False, color='#b42335',
                            linestyle='--', linewidth=1.5))
        for contact in panel['contacts']:
            x, _, z = [1000*v for v in contact['local_point_m']]
            matching = contact['distanceToRetainedPointM'] <= tolerance
            load = contact['current_upward_load_n']
            ax.scatter(x, z, s=45+4*load, color='#1f8d66' if matching else '#386e9c',
                       edgecolors='white', linewidths=1, zorder=3)
            offset = (12, -20) if x < 0 else (-12, -20)
            ax.annotate(f'{load:.2f} N', (x, z), xytext=offset, textcoords='offset points',
                        ha='left' if x < 0 else 'right', fontsize=10, color='#182638')
        ax.scatter(1000*point[0], 1000*point[2], marker='+', s=180, color='#b42335', linewidths=1.7, zorder=4)
        present = panel['retainedPointPresentWithin12mm']
        status = 'point retenu présent' if present else 'point retenu absent'
        ax.set_title(f"Pas prédit {panel['nativeSubstep']} — {status}\n"
                     f"{len(panel['contacts'])} contacts · charge totale de l’avant-pied {panel['totalCurrentUpwardLoadN']:.2f} N",
                     fontsize=12, pad=12)
        if not present:
            ax.annotate('Point retenu absent', (1000*point[0], 1000*point[2]), xytext=(-5, 88),
                        ha='center', color='#b42335', arrowprops={'arrowstyle': '->', 'color': '#b42335'})
            ax.annotate('', xy=(45, 60), xytext=(-45, 60),
                        arrowprops={'arrowstyle': '<->', 'color': '#806b5c', 'linewidth': 1})
            ax.text(0, 65, '90 mm vers le contact restant le plus proche', ha='center', fontsize=8, color='#806b5c')
        ax.set(xlim=(-82, 82), ylim=(-92, 105), xlabel='x local (mm)', ylabel='z local (mm)')
        ax.set_aspect('equal')
        ax.grid(alpha=.18)
    handles = [Line2D([], [], marker='o', linestyle='', color='#386e9c', label='Contact natif archivé, charge indiquée'),
               Line2D([], [], marker='+', linestyle='', color='#b42335', label='Point matériel retenu depuis A36'),
               Line2D([], [], linestyle='--', color='#b42335', label='Tolérance 12 mm (projection x-z)')]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .08), ncol=3, fontsize=9, frameon=False)
    fig.suptitle('PRÉVISION REFUSÉE — perte d’un point de contact, pas de tout l’avant-pied gauche', fontsize=14, y=.97)
    fig.text(.5, .035, 'Coordonnées locales et charges archivées · distances calculées en 3D · aucun état reconstruit / aucune physique ajoutée',
             ha='center', fontsize=9)
    fig.subplots_adjust(top=.83, bottom=.20, wspace=.24)
    png, svg = OUT/'contact-point-loss.png', OUT/'contact-point-loss.svg'
    assert not png.exists() and not svg.exists(), 'Do not overwrite archived plots'
    fig.savefig(png)
    fig.savefig(svg)
    plt.close(fig)
    for name, digest in sources.items():
        assert sha(Path(name)) == digest, 'Source changed during rendering'
    receipt = {'schema': 1, 'scope': 'Refused copied prediction only; no main candidate execution',
               'sourcesSha256': sources, 'endpointTraceLine': endpoint_line,
               'previousObservationJsonPath': 'owner.transfer.decision.policy.latest',
               'previousObservationNativeSubstep': 77, 'endpointNativeSubstep': 78,
               'retainedPointJsonPath': 'owner.transfer.decision.policy.initial_material_obligations[segment=19]',
               'retainedPointLocalM': point, 'segment': 19, 'matchingToleranceM': tolerance,
               'panels': panels, 'outputsSha256': {p.name: sha(p) for p in (png, svg)},
               'interpretation': 'The retained left forefoot contact disappears; three other contacts remain loaded. This is not proof of whole-foot support loss or drift over 10 mm.',
               'additionalNativeSubsteps': 0, 'newFkInspections': 0, 'collisionQueries': 0}
    with (OUT/'contact-point-loss-receipt.json').open('x', encoding='utf-8') as target:
        json.dump(receipt, target, indent=2, ensure_ascii=False, allow_nan=False)
        target.write('\n')
    print(json.dumps({'output': str(png), 'additionalNativeSubsteps': 0,
                      'previousLoadN': panels[0]['totalCurrentUpwardLoadN'],
                      'endpointLoadN': panels[1]['totalCurrentUpwardLoadN']}))


if __name__ == '__main__':
    main()
