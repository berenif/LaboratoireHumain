#!/usr/bin/env python3
"""Export pinned plain tensors; compare with the audited upstream Actor, no physics.

Requires CPU torch >=2.6, numpy and scipy. Never falls back to unrestricted pickle.
Source files are data until the exact hashed Actor/functions are selected by AST.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import struct
from types import SimpleNamespace

import numpy as np
from scipy import interpolate
import torch
import torch.nn as nn
import torch.nn.functional as F

PIN = '5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e'
HASHES = {
    'student.pt': '8b6be27ad1395cee3ddcd953907dad151cdfe1bd75b98a6bf02bfed30623fa0d',
    'teacher.pt': '9935efee1b1d46e796bb241dc1240f1905ad40e0d20284909cc063575606b244',
    'SAC.py': '64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420',
    'env.py': '62af08523865cef28f71df70d3852da3cbda8ff06f4296fafc7f759ce756bcc5',
}
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = root/'rust/crates/sim/data/getup-policy'
output.mkdir(exist_ok=True)
assert tuple(int(n) for n in torch.__version__.split('+')[0].split('.')[:2]) >= (2, 6)
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
for name, expected in HASHES.items():
    assert hashlib.sha256((args.source/name).read_bytes()).hexdigest() == expected, name
# Do not import the upstream module, SAC trainer, environment or arbitrary helpers.
tree = ast.parse((args.source/'SAC.py').read_text())
names = {'Actor', 'squash', 'gaussian_logprob'}
selected = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names]
assert {n.name for n in selected} == names
scope = dict(torch=torch, nn=nn, F=F, np=np)
exec(compile(ast.Module(body=selected, type_ignores=[]), 'pinned-SAC-Actor', 'exec'), scope)
env_tree = ast.parse((args.source/'env.py').read_text())
env_class = next(n for n in env_tree.body if isinstance(n, ast.ClassDef) and n.name == 'HumanoidStandupVelocityEnv')
pd_method = next(n for n in env_class.body if isinstance(n, ast.FunctionDef) and n.name == 'substep')
pd_scope = {'np': np}
exec(compile(ast.Module(body=[pd_method], type_ignores=[]), 'pinned-SLOW-PD', 'exec'), pd_scope)
metadata = {'sourceCommit': PIN, 'sourceSha256': HASHES, 'torchVersion': torch.__version__,
            'weightsOnly': True, 'extraSafeGlobals': [], 'physicsIntegrations': 0,
            'deterministicAction': 'tanh(mu) * observation[-1]', 'actors': {}}
fixtures = {'sourceCommit': PIN, 'actionTolerance': 5e-5, 'rawTolerance': 2e-4, 'actors': [], 'splines': [], 'pd': []}
trace = root/'outputs/getup-body-replacement-2026-10-16/trace.ndjson'
observations = [json.loads(line)['diagnostics']['getup']['observation'] for line in trace.read_text().splitlines()]
for name, width in [('teacher', 68), ('student', 118)]:
    state = torch.load(args.source/f'{name}.pt', map_location='cpu', weights_only=True)
    dims = [width, 1024, 1024, 42]
    keys = [f'trunk.{i}.{kind}' for i in [0, 2, 4] for kind in ['weight', 'bias']]
    assert set(state) == set(keys)
    blob = bytearray(b'LHGPW001' + struct.pack('<4I', *dims))
    for layer, index in enumerate([0, 2, 4]):
        for kind, shape in [('weight', (dims[layer+1], dims[layer])), ('bias', (dims[layer+1],))]:
            tensor = state[f'trunk.{index}.{kind}']
            assert type(tensor) is torch.Tensor and tensor.dtype == torch.float32
            assert tuple(tensor.shape) == shape and tensor.is_contiguous() and torch.isfinite(tensor).all()
            blob.extend(tensor.numpy().astype('<f4').tobytes())
    (output/f'{name}.bin').write_bytes(blob)
    metadata['actors'][name] = {'dimensions': dims, 'bytes': len(blob), 'sha256': hashlib.sha256(blob).hexdigest()}
    actor = scope['Actor'](width, 21, -5, 2)
    actor.load_state_dict(state, strict=True)
    actor.eval()
    inputs = []
    for k in range(12):
        p = np.array(observations[round(k*(len(observations)-1)/11)], dtype=np.float32)
        if name == 'teacher':
            x = np.r_[p, np.float32(.663420431)]
        else:
            now = np.r_[p[:21], np.float32(.8), p[34:37]]
            ahead = now.copy(); ahead[:21] += np.float32(.05)
            x = np.r_[p, now, ahead, np.float32(1)]
        inputs.append(x.astype(np.float32))
    rng = np.random.default_rng(521)
    for power in [0., .2, .663420431, 1.]:
        x = rng.uniform(-2, 2, width).astype(np.float32); x[-1] = power; inputs.append(x)
    for x in inputs:
        with torch.inference_mode():
            t = torch.from_numpy(x[None, :])
            raw = actor.trunk(t).numpy()[0]
            action = actor(t, compute_pi=False, compute_log_pi=False)[0].numpy()[0]
        fixtures['actors'].append({'actor': name, 'input': x.tolist(), 'raw': raw.tolist(), 'action': action.tolist()})
        if name == 'student':
            for holding in [False, True]:
                controls = []; calls = []
                physics = SimpleNamespace(data=SimpleNamespace(qpos=np.r_[np.zeros(7), x[:21]].astype(np.float64),
                    qvel=np.r_[np.zeros(6), x[46:67]].astype(np.float64)), model=SimpleNamespace(actuator_gear=np.ones((21,6))),
                    step=lambda: calls.append('count-only-no-physics'))
                task = SimpleNamespace(before_step=lambda u,p: controls.append(np.clip(u,-1,1)), after_step=lambda p: None)
                owner = SimpleNamespace(env=SimpleNamespace(_n_sub_steps=5,_physics=physics,_task=task),physics=physics,
                    _step_num=0, teacher_episode_length=0 if holding else 1,
                    standing_pose=x[67:88].astype(np.float64), interpolated_trajectory={'qpos':np.r_[np.zeros(7),x[67:88]][None,:]},
                    qpos_to_ctrl_index=np.array([1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]))
                pd_scope['substep'](owner,action)
                assert len(calls)==len(controls)==20 and all(np.array_equal(c,controls[0]) for c in controls)
                fixtures['pd'].append({'holding':holding,'q':x[:21].tolist(),'dq':x[46:67].tolist(),
                    'reference':x[67:88].tolist(),'residual':action.tolist(),'controls':controls[0].tolist()})
for count in [4, 5, 11, 31]:
    y = np.random.default_rng(count).uniform(-2, 2, (count, 25)).astype(np.float32)
    for speed in [.2, .5, .8]:
        times = np.linspace(0, count, count)
        target = np.linspace(0, count, math.ceil(count/speed))
        expected = np.stack([interpolate.splev(target, interpolate.splrep(times, y[:, i])) for i in range(25)], axis=1)
        fixtures['splines'].append({'knots': y.tolist(), 'speed': speed, 'expected': expected.tolist()})
(output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
(output/'parity.json').write_text(json.dumps(fixtures, separators=(',', ':'))+'\n')
print(json.dumps(metadata, indent=2))
print(json.dumps({'actorFixtures': len(fixtures['actors']), 'splineFixtures': len(fixtures['splines']), 'physicsIntegrations': 0}))
