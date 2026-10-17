#!/usr/bin/env python3
"""Illustrate a tested synthetic certificate and the unchanged archived refusal.

Only local XZ contact coordinates are plotted. No body pose, model, FK, world,
native worker or simulation is evaluated.
"""
import hashlib
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    fixture_path = HERE/'continuity-archive-36-78.json'
    forensics_path = HERE/'forensics.json'
    test_source = ROOT/'rust/crates/sim/src/recovery/planner/material_continuity_tests.rs'
    test_log = HERE/'first-pure-tests/support_state-material_continuity-tests.log'
    inputs = {str(p): sha(p) for p in (Path(__file__).resolve(), fixture_path, forensics_path, test_source, test_log)}
    fixture = json.loads(fixture_path.read_text())
    forensics = json.loads(forensics_path.read_text())
    patch = fixture['archivedPatchBeforeRefusedObservation']
    assert fixture['segment'] == patch['segment'] == 19
    assert fixture['failed']['native_substep'] == 78
    contacts = next(s['contacts'] for s in fixture['failed']['segments'] if s['segment'] == 19)
    coverage = next(p for p in forensics['refusal_patch_coverage'] if p['segment'] == 19)
    endpoint = next(o for o in coverage['observations'] if o['native_substep'] == 78)
    assert contacts == endpoint['contacts']
    assert patch['local_point_m'] == coverage['immutable_local_point_m']
    region = endpoint['current_loaded_region_local_xz']
    assert not region['inside'] and len(contacts) == 3
    assert sum(c['current_upward_load_n'] >= 3 for c in contacts) == 2
    assert 'original_interior_point_has_loaded_surface_certificate_without_reanchoring ... ok' in test_log.read_text()

    # Exact constructed observation used by the named passing pure Rust test.
    synthetic = [(-.03, -.03), (.03, -.03), (0., .04)]
    source = test_source.read_text()
    assert all(line in source for line in ('contact([-0.03, 0.0, -0.03])',
        'contact([0.03, 0.0, -0.03])', 'contact([0.0, 0.0, 0.04])', 'current_upward_load_n: 5.0'))

    os.environ.setdefault('MPLCONFIGDIR', '/tmp/lh-continuity-proof-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(13, 7.7), dpi=150)
    for ax in axes:
        ax.set_facecolor('#f7f9fb')
        ax.set(xlim=(-82, 82), ylim=(-85, 95), xlabel='x local (mm)', ylabel='z local (mm)')
        ax.set_aspect('equal')
        ax.grid(alpha=.18)

    ax = axes[0]
    points = [(x*1000, z*1000) for x, z in synthetic]
    ax.add_patch(Polygon(points, closed=True, facecolor='#d6eee3', edgecolor='#22865a', linewidth=1.8))
    for x, z in points:
        ax.scatter(x, z, s=130, color='#22865a', edgecolors='white', zorder=3)
        ax.annotate('5 N', (x, z), xytext=(0, -23 if z < 0 else 12),
                    textcoords='offset points', ha='center', fontsize=11)
    ax.scatter(0, 0, marker='+', s=220, color='#bb2638', linewidths=2, zorder=4)
    ax.annotate('Point matériel initial\ncontact direct retiré', (0, 0), xytext=(-70, 67),
                arrowprops={'arrowstyle': '->', 'color': '#bb2638'}, color='#8f2231', fontsize=10)
    ax.text(0, -64, 'ACCEPTÉ — certificat surfacique', ha='center', color='#17633e', weight='bold')
    ax.set_title('SYNTHÉTIQUE — test pur réussi\n3 témoins coplanaires à 5 N entourent le point', fontsize=12, pad=12)

    ax = axes[1]
    hull = [(x*1000, z*1000) for x, z in region['hull']]
    ax.add_patch(Polygon(hull, closed=True, facecolor='#f9e8d9', alpha=.8,
                         edgecolor='#aa784a', linestyle='--', linewidth=1.6))
    for c in contacts:
        x, _, z = [1000*v for v in c['local_point_m']]
        load = c['current_upward_load_n']
        weak = load < 3
        ax.scatter(x, z, s=100 if weak else 150, marker='o',
                   facecolors='white' if weak else '#386e9c', edgecolors='#bf7f33' if weak else 'white',
                   linewidths=2 if weak else 1, zorder=3)
        label = f'{load:.4f} N < 3 N' if weak else f'{load:.2f} N'
        ax.annotate(label, (x, z), xytext=(0, 13 if weak else -23),
                    textcoords='offset points', ha='center', color='#98601f' if weak else '#213e59', fontsize=10)
    anchor = (1000*patch['local_point_m'][0], 1000*patch['local_point_m'][2])
    nearest = tuple(1000*v for v in region['nearest_boundary']['nearest_point'])
    ax.scatter(*anchor, marker='+', s=220, color='#bb2638', linewidths=2, zorder=4)
    ax.annotate('Point retenu\nimmuable', anchor, xytext=(-74, 84), fontsize=10, color='#8f2231',
                arrowprops={'arrowstyle': '->', 'color': '#bb2638'})
    ax.annotate('', xy=nearest, xytext=anchor,
                arrowprops={'arrowstyle': '<->', 'color': '#bb2638', 'linewidth': 1.4})
    ax.text(-70, 16, f"{region['distance_m']*1000:.3f} mm\nhors du triangle", color='#bb2638', fontsize=10)
    ax.text(0, -76, 'REFUS MAINTENU', ha='center', color='#a52334', weight='bold')
    ax.set_title('ARCHIVE — prévision au pas 78, avant-pied G19\n2 témoins seulement atteignent 3 N', fontsize=12, pad=12)

    fig.suptitle('Continuité matérielle : préserver le point initial, prouver son soutien', fontsize=16, y=.97)
    fig.text(.5, .09,
        'À gauche : même plan et même normale, mode Hold, dérive nulle ; aucune force inventée au point retiré.\n'
        'À droite : même en incluant le point à 0,9899 N, le triangle reste trop loin ; il n’est pas un certificat admissible.',
        ha='center', fontsize=10, linespacing=1.7)
    fig.text(.5, .025, 'Ancres inchangées · illustration statique, aucun relevage validé · zéro physique / monde / FK ajouté',
             ha='center', fontsize=9, color='#425466')
    fig.legend(handles=[Line2D([], [], color='#bb2638', marker='+', linestyle='', markersize=10, label='Point matériel conservé'),
                        Line2D([], [], color='#aa784a', linestyle='--', label='Triangle des 3 contacts observés (dont 1 insuffisant)')],
               loc='lower center', bbox_to_anchor=(.5, .15), ncol=2, frameon=False, fontsize=9)
    fig.subplots_adjust(top=.83, bottom=.25, wspace=.25)
    output = HERE/'continuity-proof.png'
    assert not output.exists(), 'Do not overwrite an existing figure'
    fig.savefig(output)
    plt.close(fig)
    assert all(sha(Path(p)) == digest for p, digest in inputs.items()), 'Input changed during rendering'
    receipt = {'schema': 1, 'sourcesSha256': inputs,
        'synthetic': {'localXZPointsM': synthetic, 'loadsN': [5, 5, 5], 'immutablePointLocalM': [0, 0, 0],
                      'claim': 'EquivalentLoadedSurface in the named passing pure test; no physical outcome',
                      'test': 'original_interior_point_has_loaded_surface_certificate_without_reanchoring'},
        'archive': {'nativeSubstep': 78, 'segment': 19, 'contacts': contacts,
                    'immutablePointLocalM': patch['local_point_m'], 'immutableWorldAnchorM': patch['world_anchor_m'],
                    'allObservedContactsHullLocalXZ': region['hull'], 'outsideHullDistanceM': region['distance_m'],
                    'eligibleWitnessCountAtLeast3N': 2, 'result': 'Refused'},
        'outputsSha256': {output.name: sha(output)}, 'additionalNativeSubsteps': 0, 'newWorlds': 0, 'newFkInspections': 0,
        'coordinatePolicy': 'Local XZ positions only; no body pose reconstruction or anchor relocation'}
    with (HERE/'continuity-proof-receipt.json').open('x', encoding='utf-8') as target:
        json.dump(receipt, target, indent=2, ensure_ascii=False, allow_nan=False)
        target.write('\n')
    print(json.dumps({'output': str(output), 'nativeSteps': 0, 'fkPoses': 0}))


if __name__ == '__main__':
    main()
