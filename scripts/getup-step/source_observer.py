"""Hash-bound immutable source geometry and process-local validated samples.

No engine/optimizer imports or calls. A sample cannot be reconstructed from a
serialized dictionary: its process-local validation token and model identity are
checked before metric reuse. Raw packet hashes/IDs are rechecked at consumption.
"""
import copy,hashlib,json
from dataclasses import dataclass
from collections import OrderedDict
import numpy as np
from audit_metrics import fk

class FrozenDict(dict):
    def _readonly(self,*args,**kwargs):raise TypeError('immutable observer mapping')
    __setitem__=__delitem__=clear=pop=popitem=setdefault=update=__ior__=_readonly
    def __deepcopy__(self,memo):return self

def freeze(value):
    if isinstance(value,dict):return FrozenDict({k:freeze(v)for k,v in value.items()})
    if isinstance(value,(list,tuple)):return tuple(freeze(v)for v in value)
    if isinstance(value,np.ndarray):
        if value.dtype.hasobject:raise ValueError('observer arrays must be numeric')
        # An immutable bytes owner prevents setflags(write=True), unlike a
        # merely read-only owned ndarray whose flag can later be switched back.
        return np.frombuffer(value.tobytes(order='C'),dtype=value.dtype).reshape(value.shape)
    return value

def packet_hash(row):
    return hashlib.sha256(json.dumps(row,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()

@dataclass(frozen=True)
class ModelContext:
    model:object
    sha256:str
    kin:object
    parts:object
    mass:float
    gear:object
    def __deepcopy__(self,memo):return self

_CACHE=OrderedDict()
def model_context(model):
    digest=packet_hash(model)
    if digest in _CACHE:
        _CACHE.move_to_end(digest);return _CACHE[digest]
    owned=freeze(model);kin=fk.Kinematics(owned)
    kin.parent_frames=freeze(kin.parent_frames);kin.child_inverse=freeze(kin.child_inverse)
    for key in ['nominal','limits','gears']:setattr(kin,key,freeze(getattr(kin,key)))
    parts=FrozenDict({p['id']:p for p in owned['segments']});gear=np.zeros(21)
    for p in parts.values():
        h=p['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    result=ModelContext(owned,digest,kin,parts,sum(p['massKg']for p in parts.values()),freeze(gear))
    _CACHE[digest]=result
    while len(_CACHE)>8:_CACHE.popitem(last=False)
    return result

_TOKEN=object()
class ValidatedSample:
    __slots__=('metrics','packet_sha256','native_id','_context','_token','_sealed')
    def __init__(self,metrics,digest,context,token):
        if token is not _TOKEN:raise ValueError('only decoded source packets can become validated samples')
        object.__setattr__(self,'metrics',freeze(metrics));object.__setattr__(self,'packet_sha256',digest);object.__setattr__(self,'native_id',metrics['nativeId']);object.__setattr__(self,'_context',context);object.__setattr__(self,'_token',token);object.__setattr__(self,'_sealed',True)
    def __setattr__(self,*_):raise TypeError('immutable validated source sample')
    def __deepcopy__(self,memo):return self

def validated_sample(metrics,digest,context):return ValidatedSample(metrics,digest,context,_TOKEN)

def checked_metrics(sample,row,context):
    if type(sample)is not ValidatedSample or sample._token is not _TOKEN or sample._context is not context or type(row.get('sourceStep'))is not int or sample.native_id!=row['sourceStep']or sample.packet_sha256!=packet_hash(row):raise ValueError('validated sample token/model/packet/ID mismatch')
    return sample.metrics
