"""Numeric preparation only. Reuse source seed sampler and retiming; no physics."""
import ast,json,sys
from pathlib import Path
import numpy as np
from reference import source_retime,measured_teacher_frames
mode=sys.argv[1];data=json.load(sys.stdin)
if mode=='seed':
    tree=ast.parse(Path(__file__).with_name('source_positive_control.py').read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='sample_pose')
    scope={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),'<source sample_pose>','exec'),scope)
    rng=np.random.RandomState(10)
    print(json.dumps([scope['sample_pose'](data['qpos0'],data['joints'],rng).tolist() for _ in range(100)]))
elif mode=='reference':
    frames=measured_teacher_frames(data)
    trajectory={k:np.array([r[k] for r in frames]) for k in ['qpos','qvel','com','com_ori']}
    retimed=source_retime(trajectory,.5)
    print(json.dumps({'measuredFrames':len(frames),'speed':.5,'data':{k:v.tolist() for k,v in retimed.items()}}))
else:raise ValueError(mode)
