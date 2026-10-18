"""Counted online-stage optimizer/storage boundary and paired rolling saves."""
import copy,json,os,shutil
import numpy as np
from pathlib import Path
import torch
from native_env import atomic_json,sha256
from online_learning import state

class Guard:
    def __init__(self,budget,stage):self.budget=budget;self.stage=stage;self.phase='benchmark'
    def storage(self,extra=0,admission=False):
        used=sum(p.stat().st_size for p in self.stage.rglob('*')if p.is_file());free=shutil.disk_usage(self.stage).free
        if used+extra>8*1024**3 or free<extra+64*1024**2 or admission and free<12*1024**3:raise OSError('stage storage admission/cap exhausted')
        return used
    def check(self):
        self.budget.allowance(self.phase);self.storage()
        if self.phase!='validation'and self.budget.remaining_wall()<=300:raise TimeoutError('protect300wallseconds')
    def charge(self,kind):
        self.check();d=self.budget.data
        category='benchmark'if self.phase=='benchmark'else'candidate'
        limit=18 if category=='benchmark'else 2988
        key=category+'OptimizerCalls'
        if d.get('pendingOptimizer')or d.get('proposalInProgress')or d.get(key,0)>=limit or d['optimizerCalls']>=3006:raise ValueError('optimizer pending/cap')
        allowed={'benchmark-critic':16,'benchmark-head':2,'candidate-warmup':1000,'candidate-critic':994,'candidate-head':994}
        if kind not in allowed or d.get(kind,0)>=allowed[kind]:raise ValueError('optimizer type/cap')
        d[kind]=d.get(kind,0)+1;d[key]=d.get(key,0)+1;d['optimizerCalls']+=1;d['pendingOptimizer']=kind
        if kind.endswith('-head'):d['proposalInProgress']=kind
        atomic_json(self.budget.path,d)
    def complete(self,kind):
        if self.budget.data.get('pendingOptimizer')!=kind:raise ValueError('optimizer completion mismatch')
        self.budget.data['pendingOptimizer']=None;atomic_json(self.budget.path,self.budget.data);self.check()
    def commit_proposal(self,kind):
        if self.budget.data.get('proposalInProgress')!=kind or self.budget.data.get('pendingOptimizer'):raise ValueError('proposal transaction not complete')
        self.budget.data['proposalInProgress']=None;atomic_json(self.budget.path,self.budget.data);self.check()
    def torch_save(self,path,value):
        self.check();self.storage(128*1024**2);temp=path.with_suffix(path.suffix+'.tmp')
        with temp.open('wb')as stream:torch.save(value,stream);stream.flush();os.fsync(stream.fileno())
        os.replace(temp,path);self.check()
    def bytes_save(self,path,value):
        self.check();self.storage(len(value));temp=path.with_suffix(path.suffix+'.tmp')
        with temp.open('wb')as stream:stream.write(value);stream.flush();os.fsync(stream.fileno())
        os.replace(temp,path);self.check()

def rolling_checkpoint(out,guard,env,agent,replay,roles,label,*,immutable=None,padding_bytes=0,timing_max_rows=0):
    """Alternate slots; old manifest/files remain valid until new commit."""
    if guard.budget.data.get('pendingOptimizer')or guard.budget.data.get('requestOutstanding')or guard.budget.data.get('proposalInProgress'):raise ValueError('cannot save unresolved optimizer/proposal/native transaction')
    guard.check();manifest=out/'latest-checkpoint.json';old=json.loads(manifest.read_text())if manifest.exists()else{'slot':1,'generation':0};slot=1-old['slot'];generation=old['generation']+1
    blob,side=env.checkpoint();native=out/f'rolling-{slot}.bin';learning=out/f'rolling-{slot}.pt';controller=out/f'rolling-{slot}.json'
    guard.bytes_save(native,blob);payload=dict(schema=1,label=label,learning=state(agent,replay,roles))
    if padding_bytes:payload['timingOnlyNotReplayPadding']=torch.zeros(padding_bytes//4,dtype=torch.float32)
    if timing_max_rows:
        if timing_max_rows!=5000 or not replay.items:raise ValueError('timing-only maximum replay shape')
        # Add a separately tagged maximum-shaped payload, NEVER admitted into
        # learner replay. This overprices the actual64-row benchmark state.
        items=[replay.items[i%len(replay.items)]for i in range(timing_max_rows)]
        metadata=[copy.deepcopy(replay.metadata[i%len(replay.metadata)])for i in range(timing_max_rows)]
        columns=[torch.tensor(np.array([row[k]for row in items]),dtype=torch.float32)for k in range(5)]
        payload['TIMING_ONLY_NEVER_TRAINING_maximumReplay']=copy.deepcopy(dict(metadata=metadata,columns=columns,global_ids=list(range(1,5001))))
    guard.torch_save(learning,payload);record=dict(schema=1,label=label,source=side,nativeSHA256=sha256(native),learnerSHA256=sha256(learning),generation=generation)
    guard.check();atomic_json(controller,record);guard.check();entry=dict(slot=slot,generation=generation,native=str(native),learner=str(learning),controller=str(controller),nativeSHA256=record['nativeSHA256'],learnerSHA256=record['learnerSHA256'],controllerSHA256=sha256(controller));atomic_json(manifest,entry)
    if immutable is not None:
        for suffix,p in [('bin',native),('pt',learning),('json',controller)]:
            target=out/f'milestone-{immutable}.{suffix}'
            if target.exists():raise ValueError('immutable milestone already exists')
            guard.check();guard.storage(p.stat().st_size);os.link(p,target)
        fixed=dict(entry,native=str(out/f'milestone-{immutable}.bin'),learner=str(out/f'milestone-{immutable}.pt'),controller=str(out/f'milestone-{immutable}.json'));atomic_json(out/f'milestone-{immutable}-manifest.json',fixed)
    guard.check();return entry
