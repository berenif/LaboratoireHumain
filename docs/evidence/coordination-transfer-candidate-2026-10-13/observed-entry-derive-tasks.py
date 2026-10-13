#!/usr/bin/env python3
"""Pure algebra of archived poses and native mass properties, never FK."""
from pathlib import Path
import hashlib
import json
import math
import struct

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent


def read(rel):
    path = ROOT / rel
    raw = path.read_bytes()
    return json.loads(raw), {'path': rel, 'sha256': hashlib.sha256(raw).hexdigest()}


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def bits(values):
    return b''.join(struct.pack('<f', x) for x in values)


def add(a, b):
    return f32(a + b)


def mul(a, b):
    return f32(a * b)


def dot(a, b):
    return add(add(mul(a[0], b[0]), mul(a[1], b[1])), mul(a[2], b[2]))


def rotate(q, v):
    # q v conjugate(q), without normalization or new skeletal geometry.
    # Every primitive op rounds to f32; no claim of native SIMD cache identity.
    b, w = q[:3], q[3]
    c = [add(mul(b[1], v[2]), -mul(b[2], v[1])),
         add(mul(b[2], v[0]), -mul(b[0], v[2])),
         add(mul(b[0], v[1]), -mul(b[1], v[0]))]
    scalar = add(mul(w, w), -dot(b, b))
    proj = mul(dot(v, b), 2.)
    cross_scale = mul(w, 2.)
    return [add(add(mul(v[i], scalar), mul(b[i], proj)), mul(c[i], cross_scale))
            for i in range(3)]


def center(position, q, local):
    return [add(p, r) for p, r in zip(position, rotate(q, local))]


def pose_values(p):
    return ([p['position'][k] for k in 'xyz'] +
            [p['rotation'][k] for k in 'xyzw'] +
            [p['linearVelocity'][k] for k in 'xyz'] +
            [p['angularVelocity'][k] for k in 'xyz'])


def world_values(b):
    return (b['pos']['position']['translation'] + b['pos']['position']['rotation'] +
            b['vels']['linvel'] + b['vels']['angvel'])


def write(name, data):
    path = OUT / name
    raw = (json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n').encode()
    if path.exists() and path.read_bytes() != raw:
        raise RuntimeError(f'Refusing to replace different output {path}')
    if not path.exists():
        path.write_bytes(raw)
    return {'path': str(path.relative_to(ROOT)), 'bytes': len(raw),
            'sha256': hashlib.sha256(raw).hexdigest()}


def main():
    entry, entry_src = read('docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json')
    world, world_src = read('docs/evidence/coordination-native-serialization-2026-10-13/archived-initial-world.json')
    origin, origin_src = read('docs/evidence/coordination-native-serialization-2026-10-13/extraction.json')
    report, report_src = read('docs/evidence/coordination-local-pair-2026-10-13/native-run/001-A.json')
    assert world_src['sha256'] == origin['outputSha256']
    arena = [(i, x['Occupied']['value']) for i, x in enumerate(world['bodies']['bodies']['items'])
             if 'Occupied' in x]
    assert len(arena) == 25
    initial = report['initial']['segments']
    observed = entry['archivedNativeRow']['support']['bodies']
    snapshots = entry['archivedNativeRow']['snapshot']['segments']
    old_mass = entry['relatedMassPropertyArchive']['localMassProperties']
    assert len(initial) == len(observed) == len(snapshots) == len(old_mass) == 25
    bodies, mapping = [], []
    used = set()
    initial_cache_errors = []
    for segment, (old, body, snapshot, old_props) in enumerate(zip(initial, observed, snapshots, old_mass)):
        assert body['segment'] == segment
        assert old['id'] == snapshot['id'] == old_props['id']
        matching = [(i, b) for i, b in arena if bits(world_values(b)) == bits(pose_values(old))]
        assert len(matching) == 1, (segment, old['id'], [i for i, _ in matching])
        index, native = matching[0]
        assert index not in used and old_props['bodyArenaIndex'] == index
        used.add(index)
        mp = native['mprops']['local_mprops']
        assert bits(mp['local_com']) == bits(old_props['localMassProperties']['local_com'])
        assert bits([mp['inv_mass']]) == bits([old_props['localMassProperties']['inv_mass']])
        mass = f32(1. / mp['inv_mass'])
        assert mass > 0 and math.isfinite(mass)
        assert bits(pose_values(snapshot)) == bits(body['position_m'] + body['rotation_xyzw'] +
                                                  body['linear_velocity_mps'] + body['angular_velocity_radps'])
        derived = center(body['position_m'], body['rotation_xyzw'], mp['local_com'])
        predicted_initial = center(native['pos']['position']['translation'],
                                   native['pos']['position']['rotation'], mp['local_com'])
        initial_cache_errors.append(max(abs(a-b) for a, b in zip(predicted_initial, native['mprops']['world_com'])))
        bodies.append({k: body[k] for k in ['position_m', 'rotation_xyzw',
                                          'linear_velocity_mps', 'angular_velocity_radps']})
        bodies[-1].update(world_center_of_mass_m=derived, mass_kg=mass)
        mapping.append({'segment': segment, 'id': old['id'], 'bodyArenaIndex': index,
                        'nativeInvMass': mp['inv_mass'], 'nativeLocalComM': mp['local_com'],
                        'derivedMassKg': mass, 'initialPoseAndVelocityF32BitsMatch': True,
                        'priorNativeMassArchiveF32BitsMatch': True})
    total_mass, weighted = 0., [0., 0., 0.]
    for b in bodies:
        total_mass = add(total_mass, b['mass_kg'])
        weighted = [add(x, mul(c, b['mass_kg'])) for x, c in zip(weighted, b['world_center_of_mass_m'])]
    com = [f32(v/total_mass) for v in weighted]
    measured = entry['archivedNativeRow']['support']['center_of_mass_m']
    weight = entry['archivedNativeRow']['support']['weight_n']
    assert abs(total_mass - 72.2) < 1e-4
    result = {
        'schema': 1, 'scope': 'Offline task observation with algebraically derived mass and world CoM fields',
        'observed_task_state': {'native_substep': 36, 'bodies': bodies},
        'sources': [entry_src, world_src, origin_src, report_src],
        'mapping': mapping,
        'derivation': {
            'sourceMassProperties': 'Native local_mprops from archived initial world; no canonical model construction or recomputation',
            'massKg': 'f32(1 / archived_native_inv_mass)',
            'worldCom': 'f32 position + quaternion rotation of archived native local_com, primitive f32 operations, no quaternion renormalization',
            'worldComIsDirectlyArchivedAtStep36': False,
            'bodyWorldComCacheIdentityAtStep36Certified': False,
            'poseAndVelocityCopiedWithoutTransformation': True,
            'all25BodyIdentitiesMatchedByIdsAndInitialPoseVelocityBits': True,
            'all25NativeMassPropertiesMatchIndependentPriorArchiveF32Bits': True,
            'totalMassKgF32CanonicalOrder': total_mass,
            'archivedSupportWeightN': weight,
            'derivedTotalMassTimes981N': total_mass * 9.81,
            'derivedAggregateComM': com,
            'archivedAggregateComM': measured,
            'aggregateComDifferenceM': [a-b for a,b in zip(com, measured)],
            'maximumInitialWorldComCacheDifferenceM': max(initial_cache_errors),
            'exactPhysicalReplayStillRequired': True,
        },
        'newNativeSteps': 0, 'newWorlds': 0, 'newFkEvaluations': 0,
    }
    info = write('observed-entry-task-state.json', result)
    print(json.dumps({'output': info, 'checks': result['derivation']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
