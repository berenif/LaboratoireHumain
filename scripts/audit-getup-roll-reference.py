#!/usr/bin/env python3
"""Pure FK audit of the single frozen roll proposal; integrates no dynamics.

The input is a retained actual-WASM observation, never a new fixture endpoint.
This checks anatomy/limits/reference construction, not physical executability.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('roll_kinematics', ROOT/'scripts/getup-step-reference.py')
kinematics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kinematics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    context = json.loads(args.context.read_text())
    source = (ROOT/'rust/crates/sim/src/getup/physical_roll.rs').read_text()
    model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    kin = kinematics.Kinematics(model)
    root_pose = context['cachedSnapshotSegments'][0]
    root = kinematics.transform(root_pose['position'], root_pose['rotation'])
    q0 = np.asarray(context['actualFixtureReport']['jointQ'])
    poses = kin.forward(q0, root)
    parity = max(np.max(np.abs(poses[p['id']] - kinematics.transform(p['position'], p['rotation'])))
                 for p in context['cachedSnapshotSegments'])
    assert parity < 2e-5, parity
    rows = []
    for name in ['TUCK', 'UNCOIL']:
        block = re.search(r'pub const '+name+r': \[f32; 21\] = \[(.*?)\];', source, re.S).group(1)
        q = np.asarray([float(v) for v in block.split(',') if v.strip()])
        assert q.shape == (21,) and np.all(q > kin.limits[:, 0]) and np.all(q < kin.limits[:, 1])
        proposed = kin.forward(q, root)
        # Positions are virtual FK proposals with the initial root HELD ONLY
        # inside this algebraic calculation. No physical root is ever held.
        rows.append({'phaseEndpoint': name, 'q': q.tolist(), 'dq': [0.0]*21,
                     'virtualRootHeldForFkOnly': True,
                     'selectedOriginPositionsM': {b: proposed[b][:3, 3].tolist() for b in
                         ['torso', 'pelvis', 'left_hand', 'right_hand', 'left_shin', 'right_shin']},
                     'sourceStopMarginMinRad': float(np.min(np.minimum(q-kin.limits[:, 0], kin.limits[:, 1]-q)))})
    report = {'schema': 1, 'physicsIntegrations': 0, 'referenceType': 'fixed-joint-quintic-Hermite-proposal',
              'route': 'right-flank-tuck-uncoil-v1', 'plannedContactSequence': ['supine', 'right-lateral', 'prone'],
              'contactReactions': 'Native outcomes only; none supplied by this kinematic audit.',
              'contextSha256': hashlib.sha256(args.context.read_bytes()).hexdigest(),
              'rollSourceSha256': hashlib.sha256(source.encode()).hexdigest(),
              'initialNative': context['nativeSubsteps'], 'entryFkPoseParityMaxAbs': float(parity),
              'referenceSecondsPerActivePhase': 1.0, 'totalPreparationCapSeconds': 5.0,
              'normalizedMotorCap': 0.85, 'originalGearAggregateNm': float(kin.gears.sum()),
              'proposalMaximumAggregateTorqueNm': float(0.85*kin.gears.sum()), 'endpoints': rows,
              'physicalExecutabilityEstablished': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'physicsIntegrations': 0, 'entryFkPoseParityMaxAbs': float(parity),
                      'endpointCount': len(rows), 'physicalExecutabilityEstablished': False}))


if __name__ == '__main__':
    main()
