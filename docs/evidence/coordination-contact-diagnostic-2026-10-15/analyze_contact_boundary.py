#!/usr/bin/env python3
"""Analyze a CLOSED contact-diagnostic archive; never starts a native worker.

Requires exact input hashes and a pinned source commit. Missing inputs or an
unconfirmed closure produce no analysis artifacts. Standard-library arithmetic
on stored poses/contacts only; git cat-file is the only subprocess invocation.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MATCH_M = 0.012
SOURCE_PATHS = [
    'rust/crates/sim/src/recovery/planner/native_contact_diagnostics.rs',
    'rust/crates/sim/src/recovery/planner/native_observation.rs',
    'rust/crates/sim/src/recovery/planner/support_state.rs',
    'rust/crates/sim/src/recovery/planner/transfer_adapter.rs',
    'rust/crates/sim/src/recovery/planner/transfer_worker.rs',
    'rust/vendor/rapier3d/src/geometry/contact_pair.rs',
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_text(path):
    return gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def material_motion(body, local):
    q = body['rotation_xyzw']
    uv = cross(q[:3], local)
    uuv = cross(q[:3], uv)
    rotated = [local[i]+2*(q[3]*uv[i]+uuv[i]) for i in range(3)]
    angular = cross(body['angular_velocity_radps'], rotated)
    return {'world_point_m': [body['position_m'][i]+rotated[i] for i in range(3)],
            'world_velocity_mps': [body['linear_velocity_mps'][i]+angular[i] for i in range(3)]}


def point_segment_distance(p, a, b):
    d = [b[i]-a[i] for i in range(2)]
    norm2 = sum(v*v for v in d)
    t = min(1., max(0., sum(d[i]*(p[i]-a[i]) for i in range(2))/norm2)) if norm2 else 0.
    return math.dist(p, [a[i]+t*d[i] for i in range(2)])


def hull_coverage(local, contacts):
    points = sorted(set((c['local_point_m'][0], c['local_point_m'][2]) for c in contacts))
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    if len(points) >= 3:
        halves = []
        for sequence in (points, reversed(points)):
            chain = []
            for p in sequence:
                while len(chain) > 1 and orient(chain[-2], chain[-1], p) <= 0:
                    chain.pop()
                chain.append(p)
            halves.append(chain)
        points = halves[0][:-1]+halves[1][:-1]
    p = (local[0], local[2])
    edges = [(points[i], points[(i+1) % len(points)]) for i in range(len(points))]
    inside = len(points) >= 3 and all(orient(a, b, p) >= -1e-15 for a, b in edges)
    return {'hull_local_xz_m': points, 'inside_geometric_hull': inside,
            'outside_distance_m': 0. if inside else min((point_segment_distance(p, a, b) for a, b in edges), default=None),
            'meaning': 'Archive-only geometric diagnostic; not a replacement acceptance rule or proof of a support wrench.'}


def all_samples(rows):
    branch = None
    occurrences = Counter()
    observations = {}
    for line, row in enumerate(rows, 1):
        if row.get('event') == 'prefix-start':
            branch = 'technical-prefix' if row['technical'] else 'main-prefix'
        kind = row.get('kind')
        if kind in ('transfer-verify-copy', 'transfer-verify-actual'):
            if row['nativeSubstep'] == 37:
                occurrences[kind] += 1
            branch = f"technical-pair-{occurrences[kind]-1}-{'copy' if kind.endswith('copy') else 'actual'}"
        elif kind in ('transfer-preview-step', 'preview-transfer-end'):
            branch = f"forecast-{row.get('forecast', 1)}"
        observation = row.get('support')
        if observation is not None:
            key = (branch, observation['native_substep'])
            previous = observations.get(key)
            observations[key] = {'branch': branch, 'native_substep': observation['native_substep'],
                'trace_lines_1_based': (previous['trace_lines_1_based'] if previous else [])+[line],
                'observation': observation, 'kind': kind}
    return list(observations.values())


def retained_patches(rows):
    # Prefer the endpoint's immutable obligations, never substitute a new point.
    for row in reversed(rows):
        owner = row.get('owner') or {}
        decision = ((owner.get('transfer') or {}).get('decision') or {})
        patches = (decision.get('tracker') or {}).get('active')
        if patches:
            return patches, 'owner.transfer.decision.tracker.active'
    for row in rows:
        patches = (row.get('transferProposal') or {}).get('material_obligations')
        if patches:
            return patches, 'transferProposal.material_obligations'
    return [], 'unavailable: no transfer obligations were archived'


def reconcile_admitted(pair, support_segment):
    selected = []
    for collection in ('raw_manifolds', 'solver_clusters'):
        for manifold in pair[collection]:
            if not manifold['is_current_solver_collection']:
                continue
            for c in manifold['selected_contacts']:
                if c['admitted_as_current_contact']:
                    selected.append((tuple(c['segment_local_point_m']), tuple(c['segment_world_point_m']),
                                     c['upward_normal_on_segment'], c['current_upward_load_n']))
    expected = [(tuple(c['local_point_m']), tuple(c['world_point_m']), c['upward_normal'], c['current_upward_load_n']) for c in support_segment['contacts']]
    return {'matches_exactly': sorted(selected) == sorted(expected), 'telemetry_count': len(selected), 'support_count': len(expected)}


def inspect_patch(observation, pair, patch):
    local = patch['local_point_m']
    material = material_motion(observation['bodies'][patch['segment']], local)
    segment = observation['segments'][patch['segment']]
    selected_candidates, raw_candidates = [], []
    for collection in ('raw_manifolds', 'solver_clusters'):
        for manifold in pair[collection]:
            context = {'collection': collection, 'manifold_index_at_this_boundary': manifold['manifold_index'],
                'current_solver_collection': manifold['is_current_solver_collection'], 'subshape1': manifold['subshape1'],
                'subshape2': manifold['subshape2'], 'world_normal': manifold['world_normal'],
                'upward_normal_on_segment': manifold['upward_normal_on_segment'], 'computes_impulses': manifold['computes_impulses']}
            point_by_index = {p['point_index']: p for p in manifold['points']}
            for contact in manifold['selected_contacts']:
                raw = point_by_index.get(contact['manifold_point_index'])
                selected_candidates.append({'scope': context, 'distance_to_immutable_material_local_m': math.dist(local, contact['segment_local_point_m']),
                    'contact': contact, 'linked_raw_point': raw})
            for point in manifold['points']:
                anatomical_world = point['current_world_p1_m'] if pair['segment_is_collider1'] else point['current_world_p2_m']
                raw_candidates.append({'scope': context, 'point': point,
                    'distance_raw_anatomical_world_to_material_world_m': math.dist(anatomical_world, material['world_point_m']) if anatomical_world is not None else None,
                    'geometry_caveat': 'Stored narrow-phase geometric point transformed through archived current collider/subshape pose; not a fresh closest-point query.'})
    current = [c for c in selected_candidates if c['scope']['current_solver_collection']]
    matching = [c for c in current if c['distance_to_immutable_material_local_m'] <= MATCH_M]
    positive = [c for c in matching if c['contact']['admitted_as_current_contact']]
    raw_near = [c for c in raw_candidates if c['scope']['current_solver_collection'] and c['distance_raw_anatomical_world_to_material_world_m'] is not None and c['distance_raw_anatomical_world_to_material_world_m'] <= MATCH_M]
    if not pair['pair_present']:
        classification = 'floor_pair_absent'
    elif positive:
        classification = 'direct_material_point_loaded'
    elif matching:
        reasons = sorted(set(reason for c in matching for reason in c['contact']['filtered_reasons']))
        classification = 'matching_selected_contact_excluded:' + ','.join(reasons or ['measurement_error_or_missing_admission'])
    elif any(not c['point']['selected_contact_indices'] for c in raw_near):
        classification = 'nearby_raw_geometry_present_but_unselected;check_geometry_scope'
    else:
        classification = 'no_matching_selected_material_point;raw_geometry_not_equivalent_or_unavailable'
    delta = [material['world_point_m'][i]-patch['world_anchor_m'][i] for i in range(3)]
    return {'patch_id': patch['id'], 'segment': patch['segment'], 'immutable_local_point_m': local,
        'immutable_world_anchor_m': patch['world_anchor_m'], 'material_motion': material,
        'tangential_anchor_drift_m': math.hypot(delta[0], delta[2]), 'classification': classification,
        'pair_identity': {k: pair[k] for k in ('segment_collider', 'pair_present', 'pair_colliders', 'segment_is_collider1', 'solver_uses_clusters')},
        'selected_candidates': selected_candidates, 'raw_candidates': raw_candidates,
        'direct_matching_selected_count': len(matching), 'direct_loaded_matching_count': len(positive),
        'filtered_current_contacts': segment['contacts'], 'loaded_substeps': segment['loaded_substeps'],
        'integrated_support_load_n': segment['persistent_load_n'],
        'current_selected_upward_load_n': sum(c['current_upward_load_n'] for c in segment['contacts']),
        'material_point_coverage': hull_coverage(local, segment['contacts'])}


def parse():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--expected-trace-sha256', required=True)
    parser.add_argument('--expected-result-sha256', required=True)
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--closed-confirmed', action='store_true', required=True,
                        help='Operator confirms the sole diagnostic is closed; this flag cannot authorize or run physics.')
    return parser.parse_args()


def main():
    args = parse()
    if not re.fullmatch('[0-9a-f]{40}', args.source_commit):
        raise SystemExit('Require an exact 40-hex frozen source commit')
    paths = (args.trace.resolve(), args.result.resolve())
    expected = (args.expected_trace_sha256, args.expected_result_sha256)
    if any(not p.is_file() for p in paths):
        raise SystemExit('Closed diagnostic input is missing; no result generated')
    for p, value in zip(paths, expected):
        if not re.fullmatch('[0-9a-f]{64}', value) or sha(p) != value:
            raise SystemExit(f'Input hash mismatch: {p.name}; no result generated')
    with read_text(paths[0]) as stream:
        rows = [json.loads(line) for line in stream]
    with read_text(paths[1]) as stream:
        result = json.load(stream)
    if result.get('event') != 'result' or result.get('op') != 'contact-diagnostic':
        raise SystemExit('Require a completed contact-diagnostic result; no substitution of an old transfer trial')
    source_hashes = {}
    for source in SOURCE_PATHS:
        blob = subprocess.run(['git', 'cat-file', 'blob', f'{args.source_commit}:{source}'], cwd=ROOT,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout
        source_hashes[source] = hashlib.sha256(blob).hexdigest()
    patches, patch_source = retained_patches(rows)
    observations, issues, analyses = all_samples(rows), [], []
    for sample in observations:
        obs = sample['observation']
        boundary = obs.get('native_contact_boundary')
        if boundary is None:
            issues.append({'branch': sample['branch'], 'at': sample['native_substep'], 'issue': 'missing contact boundary'})
            continue
        if boundary['schema_version'] != 1 or boundary['native_substep'] != obs['native_substep'] or len(boundary['pairs']) != 25:
            issues.append({'branch': sample['branch'], 'at': sample['native_substep'], 'issue': 'schema, clock or pair count mismatch'})
            continue
        pairs = {pair['segment']: pair for pair in boundary['pairs']}
        if set(pairs) != set(range(25)):
            issues.append({'branch': sample['branch'], 'at': sample['native_substep'], 'issue': 'noncanonical segment identities'})
            continue
        reconciliation = {str(i): reconcile_admitted(pairs[i], obs['segments'][i]) for i in range(25)}
        if not all(value['matches_exactly'] for value in reconciliation.values()):
            issues.append({'branch': sample['branch'], 'at': sample['native_substep'], 'issue': 'telemetry admission differs from support observation'})
        analyses.append({'branch': sample['branch'], 'native_substep': sample['native_substep'], 'trace_lines_1_based': sample['trace_lines_1_based'],
            'floor_collider': boundary['floor_collider'], 'enclosing_native_dt_s': boundary['enclosing_native_dt_s'],
            'admission_reconciliation': reconciliation, 'patches': [inspect_patch(obs, pairs[p['segment']], p) for p in patches],
            'foot_loads_n': {str(i): obs['segments'][i]['persistent_load_n'] for i in (18, 19, 23, 24)}})
    if not patches:
        issues.append({'issue': 'No immutable transfer obligations archived; causal point comparison unavailable'})
    focal = [s for s in analyses if s['native_substep'] in (27, 28, 29, 76, 77, 78)]
    available = {(s['branch'], s['native_substep']) for s in analyses}
    expected_focal = [('main-prefix', i) for i in (27, 28, 29)]+[('forecast-1', i) for i in (76, 77, 78)]
    missing_focal = [{'branch': branch, 'native_substep': at} for branch, at in expected_focal if (branch, at) not in available]
    report = {'schema': 'laboratoirehumain.closed-contact-boundary-analysis.v1', 'sourceCodeCommit': args.source_commit,
        'source_sha256': source_hashes, 'script_sha256': sha(Path(__file__)),
        'inputs': [{'path': str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p), 'sha256': value} for p, value in zip(paths, expected)],
        'scope': 'Closed archive inspection only; no native executable, simulation, FK, new world, collision query, Rust tests or network.',
        'result': {key: result.get(key) for key in ('operationId', 'actualNativeSteps', 'technicalActualNativeSteps', 'historyActualNativeSteps', 'outcome', 'refusal', 'countsReconciled')},
        'audit': {key: value for key, value in (result.get('audit') or {}).items() if key != 'technicalPairs'},
        'immutable_material_source': patch_source, 'immutable_material_points': patches,
        'diagnostic_issues': issues, 'missing_requested_focal_boundaries': missing_focal,
        'all_available_boundary_analyses': analyses, 'focal_boundaries': focal,
        'limits': ['Manifold indices, point indices, feature IDs and new-contact bits are diagnostic scope only, never durable material identities.',
            'Stored geometric points and stored normals can persist through contact recycling; transforming them at current poses is not a new narrow-phase query.',
            'Selected solver anchors, actual impulse and explicit filter reasons are the primary evidence for the observed exclusion.',
            'Unselected raw impulses can be stale and must not contribute to current support.',
            'A contact diagnostic does not certify a transfer window or a complete stand. Even a demonstrable observational defect does not authorize another physical run.',
            'The script classifies observable predicates; independent causal interpretation must consider current geometry, selection and scope together.']}
    if any(sha(p) != value for p, value in zip(paths, expected)):
        raise SystemExit('Closed inputs changed while reading; no result generated')
    destination = paths[0].parent
    if destination != (HERE/'native-run-once').resolve():
        raise SystemExit('Analysis output must belong to this protocol native-run-once directory')
    json_path = destination/'contact-boundary-analysis.json'
    md_path = destination/'contact-boundary-analysis.md'
    lines = ['# Analyse indépendante de la frontière de contact', '',
             f"Sources figées au commit `{args.source_commit}`. Archives fermées vérifiées par SHA256 ; aucune simulation ou FK exécutée par ce script.", '',
             f"Sous-pas natifs rapportés par l’essai : **{result.get('actualNativeSteps')}**. Refus rapporté : `{result.get('refusal')}`.", '',
             f"Points matériels retrouvés : {len(patches)}. Problèmes de télémétrie : {len(issues)}. Frontières focales manquantes : {len(missing_focal)}.", '',
             '| Branche | Pas | Segment | Classification observable | Charge du segment (N) | Dérive (mm) | Distance hors enveloppe (mm) |',
             '|---|---:|---:|---|---:|---:|---:|']
    for sample in focal:
        for patch in sample['patches']:
            distance = patch['material_point_coverage']['outside_distance_m']
            distance_text = 'indisponible' if distance is None else f'{distance*1000:.6f}'
            lines.append(f"|{sample['branch']}|{sample['native_substep']}|{patch['segment']}|{patch['classification']}|{patch['current_selected_upward_load_n']:.6f}|{patch['tangential_anchor_drift_m']*1000:.6f}|{distance_text}|")
    lines += ['', 'Les classifications ci-dessus décrivent les prédicats réellement archivés. Les indices et caractéristiques géométriques sont comparés dans leur portée locale ; ils ne deviennent pas des identités matérielles durables. Les données détaillées conservent tous les points bruts et contacts sélectionnés des segments retenus, y compris les impulsions nulles et chaque exclusion.', '',
              'Le diagnostic seul ne qualifie aucune continuation complète. Une décision causale finale doit distinguer erreur d’observation, redistribution de charge et perte géométrique avant toute correction statique ; aucune relance n’est autorisée par ce rapport.', '']
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    md_path.write_text('\n'.join(lines))
    print(json.dumps({'json_sha256': sha(json_path), 'md_sha256': sha(md_path), 'boundary_count': len(analyses),
                      'diagnostic_issues': len(issues), 'missing_focal_boundaries': missing_focal}))


if __name__ == '__main__':
    main()
