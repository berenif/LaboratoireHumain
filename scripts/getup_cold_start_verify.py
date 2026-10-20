"""Independent exact tensor and executed-row mapping verification."""
import math
import hashlib
from getup_learning_loop import ContractError


def assert_state_equal(left,right):
    import numpy as np
    import torch
    if type(left)!=type(right):raise ContractError('Restored learning state type differs')
    if isinstance(left,torch.Tensor):
        equal=torch.equal(left,right)
    elif isinstance(left,np.ndarray):
        equal=np.array_equal(left,right)
    elif isinstance(left,dict):
        if left.keys()!=right.keys():raise ContractError('Restored learning keys differ')
        for key in left:assert_state_equal(left[key],right[key])
        return
    elif isinstance(left,(list,tuple)):
        if len(left)!=len(right):raise ContractError('Restored learning sequence length differs')
        for a,b in zip(left,right):assert_state_equal(a,b)
        return
    else:equal=left==right
    if not equal:raise ContractError('Restored optimizer/model/gradient/RNG state differs')


def verify_replay_mapping(arrays,metadata,mapping,lines,gamma):
    import numpy as np
    n=len(metadata)
    if (arrays['features'].shape!=(n,1024) or arrays['next_features'].shape!=(n,1024)
            or arrays['correction'].shape!=(n,21)
            or sorted(row['replay_index'] for row in mapping)!=list(range(n))):
        raise ContractError('Restored replay dimensions/exact mapping coverage differ')
    if any(not np.isfinite(arrays[key]).all() for key in arrays.files):
        raise ContractError('Nonfinite restored replay')
    covered=set()
    for item in mapping:
        i=item['replay_index'];row=metadata[i]
        start,end=item['transition_line_start'],item['transition_line_end']
        if not 0<=start<=end<len(lines):raise ContractError('Restored raw row range differs')
        span=set(range(start,end+1))
        if covered.intersection(span):raise ContractError('Replay raw ranges overlap')
        covered.update(span);segment=lines[start:end+1];first,last=segment[0],segment[-1]
        if any(x['episodeId']!=row['episode'] or x['behaviorPolicyId']!=row['behavior'] for x in segment):
            raise ContractError('Replay episode/behavior raw mapping differs')
        if (first['preNative']!=row['pre_native'] or last['postNative']!=row['post_native']
                or any(b['preNative']!=a['postNative'] for a,b in zip(segment,segment[1:]))):
            raise ContractError('Replay native chronology differs')
        if any(x['impactEpoch']!=row['impact_native'] for x in segment):
            raise ContractError('Replay impact clock differs')
        reward=sum(gamma**j*x['reward'] for j,x in enumerate(segment))
        if not math.isclose(reward,row['reward'],rel_tol=0.,abs_tol=1e-10):
            raise ContractError('Folded source-tail reward differs')
        if (bool(last['terminal'])!=row['terminal'] or row['discount']!=(
                0. if last['terminal'] else gamma**len(segment))):
            raise ContractError('Replay terminal/bootstrap distinction differs')
        if not np.array_equal(arrays['correction'][i],np.asarray(first['correction'],np.float32)):
            raise ContractError('Replay action differs from executed correction')
        for key,raw in [('features',first['sourceFeaturesSha256']),
                        ('next_features',last['nextFeaturesSha256'])]:
            if hashlib.sha256(arrays[key][i].astype('<f4').tobytes()).hexdigest()!=raw:
                raise ContractError('Replay frozen features differ from actual runtime observation')
        if any(x['preObservation'] is not None for x in segment[1:]):
            raise ContractError('Folded tail swallows another source decision')
    return {'passed':True,'rows':n,'mappedRawLines':len(covered),
            'unmappedRawLines':sorted(set(range(len(lines)))-covered)}
