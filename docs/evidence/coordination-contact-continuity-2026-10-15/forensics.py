#!/usr/bin/env python3
"""Archive-only contact forensics. No native binary, FK, imports from the app, or network.

Reads the closed native-7100 archives and pinned Git source blobs; writes only
forensics.json and forensics.md beside this script. Python standard library and
read-only git cat-file calls only.
"""
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BATCH = ROOT / 'docs/evidence/coordination-transfer-native-7100-2026-10-15/native-run-once'
SOURCE_CODE_COMMIT = 'da4575170c6ae9b0858c0851a6f1ed15fe1ce858'
EXPECTED_INPUT_SHA256 = {
    'native-trace.jsonl.gz': '5daf337c1cf4e07938b1996e2febf752a706415873f2efd5e55454ec4a0a76fc',
    'native-result.json.gz': '89c4f37c7182f1c600dbfdae8ad4b0d0414d703205fbc2d91061764d69d6faa0',
    'forecast-refusal-review.json': 'cfa7b05dd663327eb408bf7e7703ff52ed98b4b1f235e87d164f15785cdeb8e0',
    'forecast-refusal-review.md': 'b4c2448cf5105981c2d38fc4c4f09d91dbaf268d6d65bb95ec5841c693e007c2',
}
PARRY_SOURCE_SHA256 = '51712acb9dbc352189a00b69e6fa72e5a8f4dd1cbfb6be026e23f5880dd68df0'
FEET = (18, 19, 23, 24)
MATCH = 0.012
DT = struct.unpack('f', struct.pack('f', 1 / 240))[0]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pinned_repo_source_digest(path):
    relative = path.relative_to(ROOT).as_posix()
    result = subprocess.run(
        ['git', 'cat-file', 'blob', f'{SOURCE_CODE_COMMIT}:{relative}'],
        cwd=ROOT, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    return hashlib.sha256(result.stdout).hexdigest()


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def rotate(q, local):
    uv = cross(q[:3], local)
    uuv = cross(q[:3], uv)
    return [local[i] + 2*(q[3]*uv[i]+uuv[i]) for i in range(3)]


def point_motion(body, local):
    offset = rotate(body['rotation_xyzw'], local)
    angular = cross(body['angular_velocity_radps'], offset)
    return {'world_point_m': [body['position_m'][i]+offset[i] for i in range(3)],
            'world_velocity_mps': [body['linear_velocity_mps'][i]+angular[i] for i in range(3)]}


def segment_distance(p, a, b):
    v = [b[i]-a[i] for i in range(len(p))]
    den = sum(x*x for x in v)
    t = max(0., min(1., sum(v[i]*(p[i]-a[i]) for i in range(len(p)))/den)) if den else 0.
    q = [a[i]+t*v[i] for i in range(len(p))]
    return {'distance_m': math.dist(p, q), 'fraction': t, 'nearest_point': q}


def hull(points):
    points = sorted(set(tuple(x) for x in points))
    def orient(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    if len(points) < 3:
        return points
    low, high = [], []
    for chain, seq in [(low, points), (high, reversed(points))]:
        for p in seq:
            while len(chain) > 1 and orient(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    return low[:-1]+high[:-1]


def coverage(point, points):
    h = hull(points)
    if not h:
        return {'hull': [], 'inside': False, 'distance_m': None, 'area_m2': 0.}
    edges = [(h[i], h[(i+1) % len(h)]) for i in range(len(h))]
    inside = len(h) >= 3 and all((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]) >= -1e-15 for a, b in edges)
    nearest = min((segment_distance(point, a, b) for a, b in edges), key=lambda d: d['distance_m'])
    return {'hull': h, 'inside': inside, 'distance_m': 0. if inside else nearest['distance_m'],
            'nearest_boundary': nearest, 'area_m2': abs(sum(a[0]*b[1]-a[1]*b[0] for a, b in edges))/2}


def contact_record(c):
    return dict(c, derived_normal_impulse_ns=c['current_upward_load_n']*DT/c['upward_normal'])


def foot_record(s, body):
    contacts = s['contacts']
    total = sum(c['current_upward_load_n'] for c in contacts)
    cop = [sum(c['world_point_m'][i]*c['current_upward_load_n'] for c in contacts)/total for i in range(3)] if total else None
    return {'segment': s['segment'], 'id': s['id'], 'persistent_load_n': s['persistent_load_n'],
            'loaded_substeps': s['loaded_substeps'], 'selected_current_load_n': total,
            'body_pose_and_velocity': body, 'contacts': [contact_record(c) for c in contacts],
            'derived_normal_load_centroid_world_m': cop,
            'maximum_local_y_residual_from_minus_45mm_m': max((abs(c['local_point_m'][1]+.045) for c in contacts), default=None)}


def lost_gained(old, new):
    def unmatched(a, b):
        return [c for c in a if not any(math.dist(c['local_point_m'], d['local_point_m']) <= MATCH for d in b)]
    return {'disappeared_loaded_local_points': unmatched(old, new),
            'appeared_loaded_local_points': unmatched(new, old),
            'meaning': '12mm geometric correspondence over filtered loaded contacts; not raw manifold identity.'}


def main():
    trace_path, result_path = BATCH/'native-trace.jsonl.gz', BATCH/'native-result.json.gz'
    protected = [trace_path, result_path, BATCH/'forecast-refusal-review.json', BATCH/'forecast-refusal-review.md']
    before = {str(p.relative_to(ROOT)): digest(p) for p in protected}
    for path in protected:
        if before[str(path.relative_to(ROOT))] != EXPECTED_INPUT_SHA256[path.name]:
            raise RuntimeError(f'Closed archive hash mismatch: {path.name}')
    rows = [json.loads(line) for line in gzip.open(trace_path, 'rt')]
    result = json.load(gzip.open(result_path, 'rt'))
    endpoint = next(r for r in rows if r.get('kind') == 'preview-transfer-end')
    decision = endpoint['owner']['transfer']['decision']
    records, full_observations, compact = [], {}, []
    technical_occurrence = Counter()
    branch = None
    previous_by_branch = {}
    for line_number, row in enumerate(rows, 1):
        kind = row.get('kind')
        if row.get('event') == 'prefix-start':
            branch = 'technical-prefix' if row['technical'] else 'main-prefix'
            continue
        if kind in ('transfer-verify-copy', 'transfer-verify-actual'):
            if row['nativeSubstep'] == 37:
                technical_occurrence[kind] += 1
            branch = f"technical-pair-{technical_occurrence[kind]-1}-{'copy' if kind.endswith('copy') else 'actual'}"
        elif kind in ('transfer-preview-step', 'preview-transfer-end'):
            branch = 'forecast-1'
        if not kind:
            continue
        at = row['nativeSubstep']
        record = {'branch': branch, 'kind': kind, 'native_substep': at, 'trace_line_1_based': line_number}
        if 'support' in row:
            observation = row['support']
            full_observations[(branch, at)] = observation
            record['support'] = {'clock': observation['native_substep'], 'feet': [foot_record(observation['segments'][i], observation['bodies'][i]) for i in FEET],
                                 'nonfoot_contacts': [s for s in observation['segments'] if s['segment'] not in FEET and s['contacts']],
                                 'ccd_impulses': observation['ccd_impulses'], 'center_of_mass_m': observation['center_of_mass_m'],
                                 'center_of_mass_velocity_mps': observation['center_of_mass_velocity_mps']}
            record['snapshot_clocks'] = {k: row['snapshot'][k] for k in ('simulation_time_s', 'contact_time_s', 'stamp')}
            previous = previous_by_branch.get(branch)
            if previous is not None:
                record['changes_from_previous_full_observation'] = {'previous_substep': previous['native_substep'], 'consecutive': at == previous['native_substep']+1,
                    'feet': {str(i): lost_gained(previous['segments'][i]['contacts'], observation['segments'][i]['contacts']) for i in FEET}}
            previous_by_branch[branch] = observation
        else:
            p = row['transferProposal']
            record['available_compact'] = {'post_step_local_diagnostic': row['localDiagnostic'], 'post_step_center_of_mass_m': row['centerOfMassM'],
                'proposal_observation_clock': p['native_substep'], 'material_obligations': p['material_obligations'],
                'support_points_xz_at_proposal_clock': p['target_support_points_xz_m'], 'support_segments_at_proposal_clock': p['target_support_segments'],
                'heel_observations_at_proposal_clock': p['heel_observations'], 'heel_intents_at_proposal_clock': p['heel_intents'],
                'installed_command_substep': row['installedCommand']['commandSubstep'],
                'unknown': ['full body poses', 'individual current contact local coordinates, load and normal', 'raw manifold IDs and selection/zero-impulse status']}
            compact.append(record)
        records.append(record)
    # The rejected observe is atomic. Its preserved decision adds a full77 sample.
    full_observations[('forecast-1', 77)] = decision['lastObservation']
    preserved77 = {'source': 'preview-transfer-end.owner.transfer.decision.lastObservation', 'native_substep': 77,
                   'feet': [foot_record(decision['lastObservation']['segments'][i], decision['lastObservation']['bodies'][i]) for i in FEET]}

    raw = []
    for pair in result['audit']['technicalPairs']:
        for mode in ('predicted', 'actual'):
            audit = pair[mode]
            slices = [('initial', audit['initial'])]+[(f'frame-{i}', f) for i, f in enumerate(audit['frames'])]+[('post_tick', audit['post_tick'])]
            for label, frame in slices:
                world = json.loads(bytes(frame['world']))
                state = json.loads(bytes(frame['state']))
                memory = state['nativeMemory']
                floor = tuple(memory['floor'])
                foot_handles = {tuple(memory['colliders'][i]): i for i in FEET}
                pairs = []
                for edge_index, edge in enumerate(world['narrow_phase']['contact_graph']['graph']['edges']):
                    pair_data = edge['weight']
                    handles = [tuple(pair_data[key][k] for k in ('index', 'generation')) for key in ('collider1', 'collider2')]
                    if floor not in handles:
                        continue
                    other = handles[1] if handles[0] == floor else handles[0]
                    if other not in foot_handles:
                        continue
                    anatomical_side = 1 if handles[0] == other else 2
                    manifolds = pair_data['solver_clusters'] or pair_data['manifolds']
                    extracted = []
                    for manifold_index, manifold in enumerate(manifolds):
                        selected = [c['contact_id'][0] & ~(1 << 31) for c in manifold['data']['solver_contacts']]
                        points = [dict(p, manifold_point_index=i, selected_now=i in selected,
                                   anatomical_tracked_local_point_m=p[f'local_p{anatomical_side}']) for i, p in enumerate(manifold['points'])]
                        extracted.append({'manifold_index_this_frame_only': manifold_index, 'subshape1': manifold['subshape1'], 'subshape2': manifold['subshape2'],
                            'normal': manifold['data']['normal'], 'local_n1': manifold['local_n1'], 'local_n2': manifold['local_n2'],
                            'solver_flags': manifold['data']['solver_flags'], 'selected_point_indices': selected,
                            'solver_contacts': manifold['data']['solver_contacts'], 'tracked_points': points,
                            'selected_normal_impulse_ns': sum(manifold['points'][i]['data']['impulse'] for i in selected),
                            'unselected_cached_normal_impulse_ns': sum(p['data']['impulse'] for i, p in enumerate(manifold['points']) if i not in selected)})
                    pairs.append({'segment': foot_handles[other], 'graph_edge_index_this_frame_only': edge_index,
                        'collider1': pair_data['collider1'], 'collider2': pair_data['collider2'],
                        'uses_solver_clusters': bool(pair_data['solver_clusters']), 'manifolds': extracted})
                raw.append({'pair': pair['pair'], 'mode': mode, 'slice': label, 'native_substep': frame['native_substep'],
                            'world_bytes_sha256': hashlib.sha256(bytes(frame['world'])).hexdigest(), 'floor_pairs': pairs})

    patch_analysis = []
    for p in decision['tracker']['active']:
        per_observation = []
        for at in (77, 78):
            observation = full_observations[('forecast-1', at)]
            segment, body = observation['segments'][p['segment']], observation['bodies'][p['segment']]
            point = point_motion(body, p['local_point_m'])
            contacts = segment['contacts']
            local = [c['local_point_m'] for c in contacts]
            region = coverage([p['local_point_m'][i] for i in (0, 2)], [[v[i] for i in (0, 2)] for v in local])
            world_region = coverage([p['world_anchor_m'][i] for i in (0, 2)], [[c['world_point_m'][i] for i in (0, 2)] for c in contacts])
            nearest3 = min((segment_distance(p['local_point_m'], a, b) for i, a in enumerate(local) for b in local[i+1:]), key=lambda x: x['distance_m'])
            load = sum(c['current_upward_load_n'] for c in contacts)
            per_observation.append({'native_substep': at, 'material_point_motion_from_archived_body': point,
                'contacts': contacts, 'nearest_loaded_local_point_m': min(math.dist(p['local_point_m'], c) for c in local),
                'current_loaded_region_local_xz': region, 'original_world_anchor_vs_current_loaded_region_xz': world_region,
                'nearest_contact_edge_3d': nearest3,
                'maximum_local_plane_y_residual_m': max(abs(c[1]-p['local_point_m'][1]) for c in local),
                'selected_current_load_n': load,
                'normal_load_centroid_local_m': [sum(c['local_point_m'][i]*c['current_upward_load_n'] for c in contacts)/load for i in range(3)]})
        patch_analysis.append({'patch_id': p['id'], 'segment': p['segment'], 'immutable_local_point_m': p['local_point_m'],
                               'immutable_world_anchor_m': p['world_anchor_m'], 'observations': per_observation})

    prefix_patch_presence = []
    left = decision['tracker']['active'][0]
    for prefix_branch in ('technical-prefix', 'main-prefix'):
        for at in range(1, 37):
            observation = full_observations[(prefix_branch, at)]
            contacts = observation['segments'][left['segment']]['contacts']
            matching = [c for c in contacts if math.dist(c['local_point_m'], left['local_point_m']) <= MATCH]
            prefix_patch_presence.append({'branch': prefix_branch, 'native_substep': at,
                'retrospective_A36_material_point_loaded_presence': bool(matching),
                'matching_point_load_n': sum(c['current_upward_load_n'] for c in matching),
                'same_segment_selected_current_load_n': sum(c['current_upward_load_n'] for c in contacts),
                'meaning': 'Retrospective identification of the same local material corner; the A36 retained obligation was not active during this earlier prefix.'})

    normals = [c['upward_normal'] for observation in full_observations.values() for s in observation['segments'] for c in s['contacts']]
    plane_residuals = [abs(c['local_point_m'][1]+.045) for observation in full_observations.values() for s in observation['segments'] if s['segment'] in FEET for c in s['contacts']]
    branch_events = defaultdict(list)
    for record in records:
        changes = record.get('changes_from_previous_full_observation', {})
        for segment, change in changes.get('feet', {}).items():
            if change['disappeared_loaded_local_points'] or change['appeared_loaded_local_points']:
                branch_events[record['branch']].append({'at': record['native_substep'], 'previous': changes['previous_substep'], 'segment': int(segment),
                    'lost': len(change['disappeared_loaded_local_points']), 'gained': len(change['appeared_loaded_local_points']), 'consecutive': changes['consecutive']})
    raw_primary = [r for r in raw if r['pair'] == 0 and r['mode'] == 'actual' and r['slice'] != 'post_tick']
    raw_identity_summary = []
    for segment in FEET:
        samples = []
        for row in raw_primary:
            pair_data = next(p for p in row['floor_pairs'] if p['segment'] == segment)
            samples.append({'at': row['native_substep'], 'manifold_count': len(pair_data['manifolds']),
                'selected_indices': [m['selected_point_indices'] for m in pair_data['manifolds']],
                'tracked_identity_and_geometry': [[{k: p[k] for k in ('manifold_point_index', 'fid1', 'fid2', 'anatomical_tracked_local_point_m')} for p in m['tracked_points']] for m in pair_data['manifolds']],
                'unselected_cached_normal_impulse_ns': sum(m['unselected_cached_normal_impulse_ns'] for m in pair_data['manifolds'])})
        raw_identity_summary.append({'segment': segment, 'samples': samples,
            'unchanged_index_feature_geometry_36_through_40': all(s['tracked_identity_and_geometry'] == samples[0]['tracked_identity_and_geometry'] and s['selected_indices'] == samples[0]['selected_indices'] for s in samples)})

    source_paths = [ROOT/p for p in ['rust/crates/sim/src/recovery/planner/native_observation.rs', 'rust/crates/sim/src/recovery/planner/support_state.rs',
        'rust/crates/sim/src/recovery/planner/transfer_adapter.rs', 'rust/crates/sim/src/recovery/planner/transfer_worker.rs', 'rust/crates/sim/src/measurement.rs',
        'rust/vendor/rapier3d/src/geometry/contact_pair.rs', 'rust/vendor/rapier3d/src/geometry/narrow_phase/pair_update.rs']]
    parry = Path('/workspace/lh-cargo/registry/src/index.crates.io-1949cf8c6b5b557f/parry3d-0.30.2/src/query/contact_manifolds/contact_manifold.rs')
    if digest(parry) != PARRY_SOURCE_SHA256:
        raise RuntimeError('Local Parry reference source hash mismatch')
    source_digests = {str(p.relative_to(ROOT)): pinned_repo_source_digest(p) for p in source_paths}
    source_digests[str(parry)] = PARRY_SOURCE_SHA256
    report = {'schema': 'laboratoirehumain.contact-continuity-forensics.v1',
        'script_sha256': digest(Path(__file__)),
        'sourceCodeCommit': SOURCE_CODE_COMMIT,
        'source_retrieval': {'repository_sources': 'git cat-file blob <sourceCodeCommit>:<path>, including vendored Rapier; no worktree source hashes',
                             'external_parry_source': 'Absolute local reference file, required to match the pinned SHA256',
                             'archives': 'Four exact closed input hashes checked before reading and again unchanged after analysis'},
        'scope': 'Read-only arithmetic over closed archives and inspection of local source. No physics, FK, new world, Rust tests, third-party execution or network.',
        'input_sha256': before, 'source_sha256': source_digests,
        'trace_accounting': {'rows_including_events_and_endpoint': len(rows), 'kinds': dict(Counter(r.get('kind', r.get('event')) for r in rows)),
            'native_executed': 130, 'technical_prefix': 36, 'technical_pairs': 16, 'main_prefix': 36, 'forecast': 42, 'main_transfer_after_A36': 0},
        'coverage': {'full_filtered_support_trace_records': sum('support' in r for r in records), 'full_filtered_observations_including_preserved77': len(full_observations),
            'raw_world_slices': len(raw), 'raw_world_clock_range': [36, 40], 'raw_world_branches': 'two technical pairs only, predicted and actual, initial/frame/post_tick',
            'forecast_full_filtered_clocks': [77, 78], 'forecast_compact_post_step_clocks': [37, 78],
            'forecast_compact_proposal_clocks': [36, 77], 'raw_world_or_point_selection_at_78_available': False,
            'nativeRecords_warning': 'identityBytes is only the byte length, not serialized identity data; it cannot recover the missing world at78.'},
        'normal_and_plane_statistics': {'contact_count_in_full_observations_including_preserved77': len(normals), 'upward_normal_range': [min(normals), max(normals)],
            'max_local_y_residual_from_minus45mm_m': max(plane_residuals),
            'limits': 'Filtered positive-impulse contacts only. Full normal vectors and raw separation are absent outside technical raw-world slices. No claim for missing/unselected/zero-impulse points or unavailable intermediate forecast contacts.'},
        'per_trace_record': records, 'preserved_forecast77': preserved77, 'filtered_contact_transition_events': dict(branch_events),
        'forecast77_to78_filtered_contact_transition': {str(i): lost_gained(decision['lastObservation']['segments'][i]['contacts'], endpoint['support']['segments'][i]['contacts']) for i in FEET},
        'retrospective_left_material_corner_prefix_presence': prefix_patch_presence,
        'technical_raw_manifold_slices': raw, 'technical_identity_summary': raw_identity_summary,
        'refusal_patch_coverage': patch_analysis,
        'observations': ['Technical36..40 shows unchanged local geometry, features and selected manifold indices on all four foot segments; raw identity churn at78 is not observable.',
            'At78 the retained left point is outside the loaded same-segment convex hull by approximately69.993mm. This is not an equivalent covering triangle renewal.',
            'The left point geometric position remains below the support floor; its archived-pose instantaneous vertical velocity is about+0.098154m/s. Geometry, normal impulse and material-point identity are separate observables.',
            'In each earlier prefix the same local left corner is loaded at27, missing from loaded contacts at28, and loaded again at29 while the segment remains supported. This proves an earlier transient absence, not that the refused forecast would recover at79.',
            'Raw unselected forefoot points retain positive impulses in technical36..40. They must not be counted as current load or used to fill a support hull.',
            'The single generic refusal can be narrowed to lost loaded correspondence. Whether the underlying point was removed, deselected or had zero impulse remains unresolved.'],
        'proposed_rule_not_implemented': {'material': 'Keep the original material local coordinate and world anchor immutable, and preserve the10mm tangential drift guard.',
            'direct_support': 'An observed selected loaded contact can witness direct support when normal, separation, load and same-body/surface identity are valid.',
            'regional_support': 'For an explicitly regional task, optionally accept absence of an exact solver sample only when a nondegenerate same-segment, same-surface loaded triangle covers that unchanged material point, with constrained normal/plane/separation and admissible aggregate support wrench. Do not infer pointwise force from interpolation.',
            'point_pivot': 'A task requiring a loaded point pivot is stronger than regional coverage; do not silently reinterpret it.',
            'negative_or_unknown': 'Outside hull, separated, contradictory or insufficient evidence must refuse; missing raw evidence must not be relabeled benign renewal.',
            'this_trace': 'The conservative regional rule still refuses78; no reanchoring, threshold relaxation or point substitution is justified.'},
        'future_observation_requirements': ['At every substep preserve body/surface handles with generations, subshape and manifold scope, feature IDs and point indices with explicit lifetime semantics.',
            'Record all geometric candidates, selected solver indices/newness flags, zero and positive impulses, current solver-anchor world positions, live normal separation and exclusion reason separately.',
            'Keep per-CCD-interval impulses separate from latest-interval selected contacts and the enclosing240Hz total. Preserve normal/tangent components and current normal.',
            'Log direct-point versus regional support acceptance, barycentric weights, plane residual, hull distance, unchanged anchor drift, load centroid and global support/moment checks.'],
        'derived_quantity_definitions': {'normal_impulse_ns': 'current_upward_load_n * exact f32 native dt / upward_normal; reconstructs the selected latest-interval scalar, not an enclosing-step total when CCD has multiple intervals.',
            'load_centroid': 'Weighted mean of recorded selected contact positions by upward loads; excludes friction moments and is not a complete resultant wrench.',
            'coverage': 'Convex hull of current loaded contact local XZ coordinates on the same anatomical segment. Plane residual is reported separately. No body/model reconstruction.',
            'point_motion': 'Stored translation/quaternion and linear/angular velocity only: p=t+R*local; v_point=v+omega cross(R*local). No kinematic tree or integration.'}}
    after = {str(p.relative_to(ROOT)): digest(p) for p in protected}
    if before != after:
        raise RuntimeError('Protected source changed during forensic read')
    report['protected_inputs_unchanged'] = True
    (HERE/'forensics.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    left78 = patch_analysis[0]['observations'][1]
    md = f'''# Continuité des contacts : examen statique de l’essai clos

Le garde a refusé une **perte de soutien chargé au point matériel retenu gauche**. Les archives ne prouvent pas si le point brut a disparu, a été désélectionné, ou a reçu une impulsion nulle. Elles excluent cependant un simple remplacement par un triangle chargé équivalent : le point matériel immuable reste **{left78['current_loaded_region_local_xz']['distance_m']*1000:.6f} mm hors de l’enveloppe chargée** de son avant-pied à 78. Une règle prudente de continuité régionale refuserait donc encore ce cas.

## Ce qui est conservé, et ce qui manque

| Branche | Sous-pas | Archives disponibles |
|---|---|---|
| Préfixe technique | 1–36 | Poses, vitesses, contacts locaux/monde chargés, normale verticale, forces équivalentes, journaux CCD |
| Deux paires techniques, copies et contreparties | 37–40 pour chaque branche | Mêmes observations ; mondes bruts aux horloges 36–40 et checkpoint final40 |
| Préfixe principal | 1–36 | Mêmes observations complètes ; histoire séparée |
| Première prévision | 37–78 | CoM et diagnostic après chaque pas ; proposition et points XZ de support issus de l’observation précédente |
| Prévision, état préservé | 77 | Observation complète dans la décision conservée après le rejet atomique |
| Prévision, état refusé | 78 | Observation complète et géométrie actuelle ; **pas de monde brut/manifold/points non sélectionnés** |

Le résultat contient24 tranches de mondes bruts, toutes limitées aux paires techniques. Les `identityBytes` de la mémoire de prévision ne sont que des longueurs : on ne peut en extraire les données manquantes à78. La prévision compacte ne fournit pas les normales, impulsions individuelles et points locaux de chaque pas intermédiaire. Le JSON compagnon conserve toutes les lignes avec leur branche et leurs horloges distinctes, sans remplir ces lacunes par interpolation.

Le total reste130 sous-pas déjà exécutés :36 préfixe technique +16 vérifications +36 préfixe principal +42 prévision. **Zéro commande de transfert a été exécutée aprèsA36 dans l’histoire principale.** Aucun sous-pas, monde, FK ou test n’a été ajouté par cet examen.

## Identité du point, identité du manifold et charge

Dans les mondes techniques36–40, les quatre segments de pied conservent leurs indices sélectionnés, coordonnées locales brutes et couples d’identifiants géométriques. Les avant-pieds utilisent les indices `[0,4,3,1]`, les arrière-pieds `[4,3]`. Aucun renouvellement n’est constaté sur cette courte fenêtre. Cela ne renseigne pas la transition77→78, dont le monde brut n’a pas été conservé.

Les préfixes complets montrent une absence transitoire antérieure du même coin matériel gauche. Il porte85,06281N au27, disparaît des contacts chargés au28 alors que l’avant-pied porte toujours212,06144N, puis réapparaît au29 avec21,10173N. Les deux préfixes présentent ce même motif. L’identification rétrospective emploie le point local qui sera retenu àA36 ; le garde de ce candidat n’était pas encore actif aux pas27–29. Cette observation interdit de confondre toute absence d’un point chargé avec une impossibilité de soutien. Elle ne prouve toutefois ni un renouvellement de manifold, ni un retour hypothétique au79, ni l’innocuité d’une tolérance temporelle nouvelle.

Le code local de Rapier définit `contact_id` comme un index dans le manifold courant, avec un bit de nouveauté ; ce n’est pas un identifiant matériel universel. Les points peuvent être réduits à un sous-ensemble et triés ; Parry rapproche des données par identifiants de caractéristiques ou positions. Les ancres du solveur, les points géométriques bruts, les coordonnées liées au CoM et les coordonnées locales corporelles n’ont pas la même définition. L’observateur actuel transforme les **ancres courantes du solveur** dans le repère du corps. Une comparaison d’indices ou de coordonnées appartenant à des représentations différentes ne constitue pas une preuve de glissement.

Un exemple concret interdit de sommer aveuglément le cache : le point non sélectionné d’indice2 garde une impulsion normale de0,8868289N·s à gauche et0,88085604N·s à droite pendant36–40. Ces valeurs ne sont pas des charges courantes. Le chemin d’observation existant les exclut correctement ; la continuité ne doit pas les réintroduire pour remplir un polygone d’appui.

## Transition décisive77→78

Le point gauche conservé est localement environ `[-0.045,-0.045,+0.060]`m. À77 il porte65,341455N ; à78, le segment reste chargé à85,552933N via trois autres points. Le plus proche est à90,000026mm. Le point n’est pas couvert par leur triangle : distance locale3D à l’arête la plus proche **69,992785mm** ; distance XZ de l’ancre monde initiale à l’enveloppe courante **69,816090mm**. Ce dernier calcul et celui du point matériel courant sont volontairement distincts.

La dérive tangente du point gauche reste0,186918mm, sous le garde de10mm. Sa hauteur dérivée des poses est−3,929819mm, et sa vitesse verticale instantanée passe de−0,012204m/s à77 à+0,098154m/s à78. Ces données sont compatibles avec une décharge et un mouvement ascendant, mais ne prouvent pas pourquoi le solveur/filtre n’expose plus ce point. Elles n’autorisent pas à qualifier l’absence de simple renouvellement normal, ni de décollement du pied entier.

Les trois contacts gauches à78 ont une normale verticale1.0 et leurs coordonnées localesY diffèrent du point retenu d’au plus7,45nanomètres. L’exclusion de l’enveloppe est donc latérale et importante, pas un résidu de plan de quelques nanomètres. Le point droit garde son contact chargé direct ; il n’a pas besoin d’une exception régionale.

Dans les{len(normals)} contacts des{len(full_observations)} observations complètes, état préservé77 compris et branches répétées incluses, les normales verticales sont comprises entre{min(normals):.10f} et{max(normals):.1f}. Le résidu maximal à la face localeY=−45mm est{max(plane_residuals)*1e9:.3f}nanomètres. Ce constat concerne les points **sélectionnés et chargés conservés**, pas ceux absents ni les contacts intermédiaires non archivés. Le JSON détaille les changements de population sur tous les sous-pas disposant d’observations complètes, et signale les intervalles non consécutifs.

## Règle proposée, sans implémentation

1. Conserver une identité matérielle indépendante du solveur : point local et ancre monde immuables, dérive tangente mesurée, garde10mm inchangé.
2. Distinguer géométrie et charge : normale/séparation et contact géométrique d’une part ; impulsions sélectionnées réellement actives, charge résultante et centre de charge d’autre part. Une impulsion nulle n’efface pas l’identité matérielle ; elle ne prouve pas non plus que le point soutient le corps.
3. Pour une tâche explicitement **régionale**, accepter éventuellement l’absence d’un échantillon exact si un triangle non dégénéré de contacts chargés du même segment et de la même surface couvre toujours le point inchangé, avec normale, plan, séparation et bilan global des efforts compatibles. La couverture ne fabrique pas une force ponctuelle interpolée.
4. Pour un **pivot ponctuel chargé**, conserver l’exigence plus forte ou effectuer une transition d’appui explicite. Ne pas convertir silencieusement cette tâche en région et ne pas déplacer son ancre.
5. Hors enveloppe, séparation, données contradictoires ou couverture insuffisante : refuser. Le cas78 reste refusé avec cette règle.

Pour départager les causes lors d’un futur protocole autorisé, l’archive devra distinguer points géométriques, points sélectionnés, impulsions nulles/positives, motifs de filtrage, identifiants de corps/surfaces et de sous-formes, ancres courantes, séparation normale et intervalles CCD. Aucun nouveau test physique n’est proposé comme déjà autorisé ici.

## Reproduction et limites

Exécuter `python3 docs/evidence/coordination-contact-continuity-2026-10-15/forensics.py` ne lit que les archives et sources locales et réécrit ses deux livrables. Le script emploie la bibliothèque standard Python et Git en lecture seule ; il ne charge ni le moteur natif ni le modèle. Les transformées rigides utilisent les poses/vitesses mesurées déjà archivées, sans arbre cinématique ni intégration. Les SHA256 des archives et sources consultées figurent dans le JSON ; les quatre preuves d’entrée contrôlées restent inchangées.

La provenance du code est fixée explicitement à `sourceCodeCommit={SOURCE_CODE_COMMIT}`. Toutes les sources du dépôt, y compris Rapier inclus dans le dépôt, sont lues par `git cat-file blob` à ce commit ; les empreintes ne proviennent pas du worktree en cours de modification. La référence Parry externe conserve son chemin absolu et son SHA256 imposé. Les quatre archives d’entrée doivent correspondre à leurs empreintes historiques exactes avant toute lecture analytique ; aucun fichier manquant ou différent n’est remplacé silencieusement. Cette correction de provenance ne modifie aucune colonne calculée à partir des archives.

Les forces de contact sont des impulsions du dernier intervalle sélectionné divisées par le pas natif ; elles ne sont pas des forces instantanées ni nécessairement le total d’un pas comportant plusieurs intervalles CCD. Les centres de charge calculés n’incluent pas les moments de friction et ne remplacent pas une validation complète du torseur de support. Aucun relevage, impossibilité anatomique ou nécessité universelle d’appuis supplémentaires ne découle de cet essai.
'''
    # Typographic spacing only; never rewrite machine-readable numerical data.
    md = re.sub(r'\b(aux|les|des|au|à|de|par|pendant|en|avec|porte|toujours|contient|techniques|avant-pieds|arrière-pieds)(?=\d)', r'\1 ', md)
    md = re.sub(r'(?<=\d)(?=(?:mm|nanomètres|N·s|N\b|m/s))', ' ', md)
    md = md.replace('aprèsA36', 'après A36').replace('àA36', 'à A36').replace('de10mm', 'de 10 mm').replace('de0,', 'de 0,')
    (HERE/'forensics.md').write_text(md)
    print(json.dumps({'json_sha256': digest(HERE/'forensics.json'), 'md_sha256': digest(HERE/'forensics.md'),
        'protected_inputs_unchanged': True, 'full_observations': len(full_observations), 'raw_world_slices': len(raw),
        'left78_hull_distance_m': left78['current_loaded_region_local_xz']['distance_m']}))


if __name__ == '__main__':
    main()
