"""Source-SLOW residual replay and exact CPU learner checkpoint helpers.

No engine or training loop on import. Actions are residual radians; actual motor
controls are separate environment evidence. All optimizer calls need a charge hook.
"""
import copy,random
import numpy as np
from slow_task import initialize_slow_agent,CONFIG_PATH
from task import load_config
FAILURE_CODES=set(load_config(CONFIG_PATH)[0]["termination"]["failureCodes"])

class SlowReplay:
    KEYS=['state','action','next_state','reward','not_done']
    def __init__(self,capacity=1500,seed=0):
        if type(capacity)is not int or not 1<=capacity<=1500:raise ValueError('invalid reviewed replay capacity')
        self.capacity=capacity;self.rng=np.random.default_rng(seed);self.items=[];self.global_ids=[];self.metadata=[]
    def add(self,global_id,transition):
        if type(global_id)is not int or global_id<1 or self.global_ids and global_id<=self.global_ids[-1]:raise ValueError('replay integration accounting IDs must increase')
        if len(self.items)>=self.capacity:raise ValueError('reviewed replay ceiling reached')
        values=[]
        for key,width in zip(self.KEYS,[118,21,118,1,1]):
            value=np.asarray(transition[key],dtype=np.float32).reshape(-1)
            if value.shape!=(width,)or not np.isfinite(value).all():raise ValueError('invalid source residual replay tensor')
            values.append(value.copy())
        if max(abs(values[1]))>1 or values[0][-1]!=1 or values[2][-1]!=1 or values[4][0]not in[0,1]:raise ValueError('source action/power/mask mismatch')
        count=transition['info']['nativeSampleCount']
        if transition['info']['physicalFailure']is not None and transition['info']['physicalFailure']not in FAILURE_CODES:raise ValueError('invalid replay physical failure code')
        if type(count)is not int or not 1<=count<=20 or count<20 and not transition['info']['physicalFailure']:raise ValueError('only verified finite physical terminals may be partial')
        success=transition['info'].get('success',False)
        if type(success)is not bool or values[4][0]!=(0 if transition['info']['physicalFailure']is not None or success else 1):raise ValueError('physical failure/absorbing goal must not bootstrap')
        self.items.append(values);self.global_ids.append(global_id);self.metadata.append(dict(nativeSampleCount=count,physicalFailure=transition['info']['physicalFailure'],success=success))
    def sample(self,n):
        import torch
        if type(n)is not int or n<1 or not self.items:raise ValueError('invalid source replay sample')
        ids=self.rng.integers(0,len(self.items),n)
        return tuple(torch.tensor(np.array([self.items[i][k]for i in ids]),dtype=torch.float32)for k in range(5))
    def state(self):
        import torch
        return dict(capacity=self.capacity,metadata=copy.deepcopy(self.metadata),rng=copy.deepcopy(self.rng.bit_generator.state),global_ids=self.global_ids.copy(),columns=[torch.tensor(np.array([row[k]for row in self.items]),dtype=torch.float32).reshape(len(self.items),[118,21,118,1,1][k])for k in range(5)])
    @classmethod
    def restore(cls,state):
        other=cls(state['capacity']);columns=state['columns'];ids=state['global_ids']
        if len(columns)!=5 or len(ids)>other.capacity or len(set(ids))!=len(ids)or any(type(v)is not int or v<1 for v in ids)or ids!=sorted(ids):raise ValueError('invalid replay checkpoint counters')
        expected=[118,21,118,1,1]
        for column,width in zip(columns,expected):
            if tuple(column.shape)!=(len(ids),width)or not bool(column.isfinite().all()):raise ValueError('invalid replay checkpoint tensors')
        metadata=state['metadata']
        if len(metadata)!=len(ids):raise ValueError('replay duration metadata missing')
        for i,meta in enumerate(metadata):
            count=meta['nativeSampleCount']
            if type(count)is not int or not 1<=count<=20 or meta['physicalFailure']is not None and meta['physicalFailure']not in FAILURE_CODES or type(meta['success'])is not bool or columns[4][i,0]!=(0 if meta['physicalFailure']is not None or meta['success']else 1)or count<20 and not meta['physicalFailure']:raise ValueError('invalid replay terminal duration')
        other.metadata=copy.deepcopy(metadata);other.rng.bit_generator.state=copy.deepcopy(state['rng']);other.global_ids=ids.copy();other.items=[[columns[k][i].numpy().copy()for k in range(5)]for i in range(len(ids))]
        if other.items:
            if any(row[0][-1]!=1 or row[2][-1]!=1 or max(abs(row[1]))>1 or row[4][0]not in[0,1]for row in other.items):raise ValueError('invalid restored replay units/masks')
        return other

def learner_state(agent,replay):
    import torch
    numpy=np.random.get_state()
    return copy.deepcopy(dict(schema=1,actor=agent.actor.state_dict(),critic=agent.critic.state_dict(),critic_target=agent.critic_target.state_dict(),actor_optimizer=agent.actor_optimizer.state_dict(),critic_optimizer=agent.critic_optimizer.state_dict(),alpha_optimizer=agent.log_alpha_optimizer.state_dict(),log_alpha=agent.log_alpha.detach().clone(),total_it=agent.total_it,critic_warmup_it=getattr(agent,'critic_warmup_it',0),loss_dict=agent.loss_dict,replay=replay.state(),torch_rng=torch.get_rng_state(),numpy_global=[numpy[0],numpy[1].tolist(),numpy[2],numpy[3],numpy[4]],python_rng=random.getstate()))

def restore_learner(agent,state):
    import torch
    if state['schema']!=1 or type(state['total_it'])is not int or not 0<=state['total_it']<=25 or type(state['critic_warmup_it'])is not int or not 0<=state['critic_warmup_it']<=100 or state['log_alpha'].shape!=torch.Size([])or not bool(state['log_alpha'].isfinite()):raise ValueError('invalid source learner state')
    # Check into temporary replay/RNG objects before touching learner tensors.
    replay=SlowReplay.restore(state['replay']);npcheck=np.random.RandomState();n=state['numpy_global'];npcheck.set_state((n[0],np.array(n[1],dtype=np.uint32),n[2],n[3],n[4]));random.Random().setstate(state['python_rng'])
    generator=torch.Generator(device='cpu');generator.set_state(state['torch_rng'])
    for key,module in [('actor',agent.actor),('critic',agent.critic),('critic_target',agent.critic_target)]:
        if set(state[key])!=set(module.state_dict())or any(v.shape!=module.state_dict()[k].shape or not bool(v.isfinite().all())for k,v in state[key].items()):raise ValueError('invalid source learner tensors')
    for key,module in [('actor',agent.actor),('critic',agent.critic),('critic_target',agent.critic_target)]:module.load_state_dict(state[key],strict=True)
    for key,opt in [('actor_optimizer',agent.actor_optimizer),('critic_optimizer',agent.critic_optimizer),('alpha_optimizer',agent.log_alpha_optimizer)]:opt.load_state_dict(copy.deepcopy(state[key]))
    with torch.no_grad():agent.log_alpha.copy_(state['log_alpha'])
    agent.total_it=state['total_it'];agent.critic_warmup_it=state['critic_warmup_it'];agent.loss_dict=copy.deepcopy(state['loss_dict']);torch.set_rng_state(state['torch_rng']);np.random.set_state((n[0],np.array(n[1],dtype=np.uint32),n[2],n[3],n[4]));random.setstate(state['python_rng']);return replay

def charged_source_update(agent,replay,batch_size,*,before_optimizer_step,after_optimizer_step=lambda kind:None):
    """Unchanged source SAC.train equations, each real optimizer call precharged."""
    import torch
    originals=[];calls=[]
    def wrapper(kind,opt,original):
        def step(*args,**kwargs):
            parameters=[p for group in opt.param_groups for p in group['params']]
            if any(not bool(p.isfinite().all())or p.grad is not None and not bool(p.grad.isfinite().all())for p in parameters):raise ValueError('nonfinite source optimizer tensor')
            before_optimizer_step(kind);result=original(*args,**kwargs);calls.append(kind)
            if any(not bool(p.isfinite().all())for p in parameters):raise ValueError('source optimizer returned nonfinite parameters')
            after_optimizer_step(kind)
            return result
        return step
    for kind,opt in [('critic',agent.critic_optimizer),('actor',agent.actor_optimizer),('alpha',agent.log_alpha_optimizer)]:
        original=opt.step;originals.append((opt,original));opt.step=wrapper(kind,opt,original)
    try:agent.train(replay,batch_size)
    finally:
        for opt,original in originals:opt.step=original
    if calls!=['critic','actor','alpha']:raise ValueError('unexpected source optimizer call schedule')
    return dict(criticLoss=agent.loss_dict['critic'][-1],actorLoss=agent.loss_dict['actor'][-1],temperatureLoss=agent.loss_dict['temperature'][-1],alpha=float(agent.alpha.detach()),optimizerCalls=calls)
