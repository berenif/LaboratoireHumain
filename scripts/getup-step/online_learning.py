"""Online SLOW output-head learning primitives; no engine/CLI on import.

The original trunk/temperature are fixed. Both mean AND std output rows learn.
Collection, critic-target, proposal and replay randomness have separate owners.
"""
import copy,hashlib,json
from contextlib import contextmanager
from pathlib import Path
import numpy as np
import torch
from slow_task import initialize_slow_agent,original_slow_actor
from slow_learning import SlowReplay,learner_state
from slow_trust import gaussian,ALPHA
from training import source_module

PROTOCOL=Path(__file__).with_name('slow-online-protocol-v1.json')
PROTOCOL_SHA='556d119fae46eb4ff63a13bbd20d2dc6f50850dc6345afdbadf324ebb5e81ec3'
def tensor_sha(x):return hashlib.sha256(x.detach().cpu().contiguous().numpy().tobytes()).hexdigest()
def seed(role,attempt=0):return int.from_bytes(hashlib.sha256(f'{PROTOCOL_SHA}:0:{role}:{attempt}'.encode()).digest()[:8],'little')%(2**63-1)
def trunk_sha(actor):return hashlib.sha256(b''.join(v.detach().numpy().tobytes()for k,v in actor.state_dict().items()if not k.startswith('trunk.4'))).hexdigest()

def initialize():
    agent,_=initialize_slow_agent(0);original=original_slow_actor();agent.actor.load_state_dict(original.state_dict())
    for p in agent.actor.parameters():p.requires_grad_(False)
    for p in agent.actor.trunk[4].parameters():p.requires_grad_(True)
    agent.actor_optimizer=torch.optim.Adam(agent.actor.trunk[4].parameters(),lr=1e-5,betas=(.9,.999))
    with torch.no_grad():agent.log_alpha.fill_(np.log(ALPHA))
    agent.log_alpha.requires_grad_(False)
    return agent,original

class Roles:
    def __init__(self,namespace='candidate'):
        if namespace not in ['candidate','benchmark','fixture']:raise ValueError('unknown RNG namespace')
        self.namespace=namespace;self.collection=torch.Generator().manual_seed(seed(namespace+'/collection'));self.critic=torch.Generator().manual_seed(seed(namespace+'/critic_target'));self.replay=np.random.default_rng(seed(namespace+'/replay'));self.attempt_id=0
    def proposal(self,n):
        if type(n)is not int or n!=self.attempt_id+1 or n>994:raise ValueError('fresh monotonic proposal substream required')
        self.attempt_id=n;return torch.Generator().manual_seed(seed(self.namespace+'/actor_proposal',n))
    @contextmanager
    def critic_scope(self):
        before=torch.get_rng_state();torch.set_rng_state(self.critic.get_state())
        try:yield
        finally:self.critic.set_state(torch.get_rng_state());torch.set_rng_state(before)
    def state(self):return copy.deepcopy(dict(schema=1,protocol=PROTOCOL_SHA,namespace=self.namespace,collection=self.collection.get_state(),critic=self.critic.get_state(),replay=self.replay.bit_generator.state,attempt_id=self.attempt_id))
    @classmethod
    def restore(cls,s):
        if s['schema']!=1 or s['protocol']!=PROTOCOL_SHA or type(s['attempt_id'])is not int or not 0<=s['attempt_id']<=994:raise ValueError('invalid online RNG role schema')
        x=cls(s['namespace']);x.collection.set_state(s['collection']);x.critic.set_state(s['critic']);x.replay.bit_generator.state=copy.deepcopy(s['replay']);x.attempt_id=s['attempt_id'];return x

def sample_action(actor,state,roles,policy_version):
    if not isinstance(state,torch.Tensor)or tuple(state.shape)!=(1,118)or state.dtype!=torch.float32 or state.device.type!='cpu'or not state.isfinite().all()or state[0,-1]!=1 or type(policy_version)is not int or not 0<=policy_version<=994:raise ValueError('one finite source118/power1 row and policy version required')
    source=source_module()
    with torch.no_grad():
        mu,ls=gaussian(actor,state);noise=torch.randn(mu.shape,generator=roles.collection);latent=mu+noise*ls.exp();lp=source.gaussian_logprob(noise,ls);_,a,lp=source.squash(mu,latent,lp,state[:,-1])
    record=dict(policyVersion=policy_version,rawMean=mu[0].tolist(),logstd=ls[0].tolist(),noise=noise[0].tolist(),sourceEpsilonLogProbability=float(lp[0,0]))
    return a[0].numpy(),record

def validate_behaviour(b,action):
    if type(b.get('policyVersion'))is not int or not 0<=b['policyVersion']<=994:raise ValueError('invalid collection policy version')
    values=[]
    for key in ['rawMean','logstd','noise']:
        v=torch.tensor(b[key],dtype=torch.float32).reshape(-1)
        if tuple(v.shape)!=(21,)or not v.isfinite().all():raise ValueError('invalid collection latent witness')
        values.append(v[None,:])
    mu,ls,noise=values
    if torch.any(ls< -5)or torch.any(ls>2):raise ValueError('source std bounds')
    source=source_module();lp=source.gaussian_logprob(noise,ls);_,a,lp=source.squash(mu,mu+noise*ls.exp(),lp,torch.ones(1))
    value=b['sourceEpsilonLogProbability']
    if type(value)not in[int,float]or not np.isfinite(value)or abs(float(lp)-value)>1e-5 or not np.array_equal(a[0].numpy(),np.asarray(action,dtype=np.float32)):raise ValueError('collection action/logprob witness mismatch')

class OnlineReplay(SlowReplay):
    def __init__(self,capacity=5000,seed=0):
        if capacity!=5000:raise ValueError('reviewed online replay capacity is5000')
        super().__init__(1500,seed);self.capacity=5000
    def add(self,global_id,t):
        validate_behaviour(t['behaviour'],t['action']);super().add(global_id,t);self.metadata[-1]['behaviour']=copy.deepcopy(t['behaviour'])
    @classmethod
    def restore(cls,state):
        other=super().restore(state)
        for row,meta in zip(other.items,other.metadata):validate_behaviour(meta['behaviour'],row[1])
        return other

class Features:
    """Owned tensor features for ONE exact batch and frozen trunk identity."""
    def __init__(self,actor,states,expected_trunk):
        if trunk_sha(actor)!=expected_trunk or states.ndim!=2 or states.shape[1]!=118 or not states.isfinite().all()or not torch.all(states[:,-1]==1):raise ValueError('feature tensor/trunk mismatch')
        self.states=states.detach().clone();self.input_sha=tensor_sha(self.states);self.trunk_sha=expected_trunk
        with torch.no_grad():self.hidden=actor.trunk[:4](self.states).detach()
        self.versions=(self.states._version,self.hidden._version)
        self.checked_trunks={}
    def raw(self,actor):
        if self.versions!=(self.states._version,self.hidden._version):raise ValueError('feature tensor mutated')
        versions=tuple(p._version for p in actor.trunk[:4].parameters())
        if self.checked_trunks.get(actor)!=versions:
            if trunk_sha(actor)!=self.trunk_sha:raise ValueError('feature cache belongs to another frozen trunk')
            self.checked_trunks[actor]=versions
        return actor.trunk[4](self.hidden)
    def distribution(self,actor):
        mu,raw=self.raw(actor).chunk(2,-1);return mu,-5+3.5*(torch.tanh(raw)+1)

def loss(agent,features,noise):
    source=source_module();mu,ls=features.distribution(agent.actor);lp=source.gaussian_logprob(noise,ls);_,a,lp=source.squash(mu,mu+noise*ls.exp(),lp,features.states[:,-1]);q1,q2=agent.critic(features.states,a);value=(agent.alpha.detach()*lp-torch.minimum(q1,q2)).mean()
    if not value.isfinite():raise ValueError('nonfinite source actor loss')
    return value

def divergence(mu0,ls0,mu,ls):
    a,b,c,d=[x.double()for x in [mu0,ls0,mu,ls]]
    result=torch.sum(d-b+.5*(torch.exp(2*(b-d))+((a-c)*torch.exp(-d))**2-1),-1)
    if not result.isfinite().all()or float(result.min())< -1e-8:raise ValueError('invalid Gaussian KL')
    return result.clamp_min(0)

def _propose(agent,original,batch,witness,generator,*,charge,complete,check,minimum_decrease=1e-6,kl_limit=.05,mean_limit=.05,enforce_original_mean_distance=True):
    if type(enforce_original_mean_distance)is not bool:raise ValueError('explicit original-mean rejection mode required')
    check();frozen=trunk_sha(original)
    if trunk_sha(agent.actor)!=frozen:raise ValueError('original trunk changed')
    bf=Features(agent.actor,batch,frozen);wf=Features(agent.actor,witness,frozen)
    with torch.no_grad():old_mu,old_ls=wf.distribution(agent.actor);anchor_mu,anchor_ls=wf.distribution(original)
    old_mu,old_ls=old_mu.clone(),old_ls.clone();head=agent.actor.trunk[4];before=copy.deepcopy(head.state_dict());opt=copy.deepcopy(agent.actor_optimizer.state_dict());noise=torch.randn((len(batch),21),generator=generator);rng_before=generator.get_state().clone();base=float(loss(agent,bf,noise).detach())
    agent.actor_optimizer.zero_grad(set_to_none=True);value=loss(agent,bf,noise);value.backward()
    if any(p.grad is None or not p.grad.isfinite().all()for p in head.parameters()):raise ValueError('invalid output-head gradients')
    gradients=dict(mean=float(torch.sqrt(sum((p.grad[:21]**2).sum()for p in head.parameters()))),std=float(torch.sqrt(sum((p.grad[21:]**2).sum()for p in head.parameters()))))
    charge();agent.actor_optimizer.step();complete();after=copy.deepcopy(head.state_dict());proposed_adam=copy.deepcopy(agent.actor_optimizer.state_dict());std_delta=max(float((after[k][21:]-before[k][21:]).abs().max())for k in before);trials=[];accepted=None
    for k in range(11):
        check();scale=2.**(-k)
        with torch.no_grad():
            for name,p in head.state_dict().items():p.copy_(before[name]+scale*(after[name]-before[name]))
            mu,ls=wf.distribution(agent.actor);kl=divergence(old_mu,old_ls,mu,ls);distance=.5*torch.sum(((mu-anchor_mu)/anchor_ls.exp())**2,-1);candidate=float(loss(agent,bf,noise));record=dict(scale=scale,actorLoss=candidate,lossDecrease=base-candidate,maximumStepKL=float(kl.max()),maximumOriginalMeanDistance=float(distance.max()))
        trials.append(record)
        if record['lossDecrease']>=minimum_decrease and record['maximumStepKL']<=kl_limit and(not enforce_original_mean_distance or record['maximumOriginalMeanDistance']<=mean_limit):accepted=scale;break
    if accepted is None:head.load_state_dict(before);agent.actor_optimizer.load_state_dict(copy.deepcopy(opt))
    if trunk_sha(agent.actor)!=frozen or abs(float(agent.alpha)-ALPHA)>1e-12:raise ValueError('frozen trunk/temperature changed')
    return dict(accepted=accepted is not None,scale=accepted,baselineActorLoss=base,trials=trials,headGradientNorms=gradients,proposedStdParameterMaximumChange=std_delta,noiseSHA256=tensor_sha(noise),proposalGeneratorAfterNoiseSHA256=tensor_sha(rng_before),batchSHA256=bf.input_sha,witnessSHA256=wf.input_sha,trunkSHA256=frozen,_evidence=dict(beforeHead=before,proposedHead=after,beforeAdam=opt,proposedAdam=proposed_adam,retainedAdam=copy.deepcopy(agent.actor_optimizer.state_dict()),noise=noise,proposalGeneratorAfterNoise=rng_before))

def propose(agent,original,batch,witness,generator,**kwargs):
    """Rollback actor-side state on any interrupted/uncommitted proposal.

    Critic, replay, all charges and the retired proposal substream stay advanced.
    The runner additionally holds its durable proposal marker until evidence and
    global veto checks complete; failed transactions cannot replace rolling state.
    """
    head=agent.actor.trunk[4];before=copy.deepcopy(head.state_dict());adam=copy.deepcopy(agent.actor_optimizer.state_dict())
    try:return _propose(agent,original,batch,witness,generator,**kwargs)
    except BaseException:
        head.load_state_dict(before);agent.actor_optimizer.load_state_dict(adam);raise

def state(agent,replay,roles):
    if replay.rng is not roles.replay:raise ValueError('replay RNG role ownership mismatch')
    return dict(schema=1,protocol=PROTOCOL_SHA,learner=learner_state(agent,replay),roles=roles.state())

def restore(s):
    if s['schema']!=1 or s['protocol']!=PROTOCOL_SHA:raise ValueError('wrong online learner protocol')
    agent,original=initialize();x=s['learner'];replay=OnlineReplay.restore(x['replay']);roles=Roles.restore(s['roles'])
    if replay.rng.bit_generator.state!=roles.replay.bit_generator.state:raise ValueError('duplicated replay RNG states differ')
    replay.rng=roles.replay
    if type(x['critic_warmup_it'])is not int or not 0<=x['critic_warmup_it']<=1994 or type(x['total_it'])is not int or not 0<=x['total_it']<=994 or abs(float(x['log_alpha'].exp())-ALPHA)>1e-12:raise ValueError('invalid online learner counts/alpha')
    for key,module in [('actor',agent.actor),('critic',agent.critic),('critic_target',agent.critic_target)]:
        if set(x[key])!=set(module.state_dict())or any(v.shape!=module.state_dict()[k].shape or not v.isfinite().all()for k,v in x[key].items()):raise ValueError('invalid online tensor checkpoint')
        module.load_state_dict(x[key])
    if trunk_sha(agent.actor)!=trunk_sha(original):raise ValueError('checkpoint changed frozen source trunk')
    for key,opt in [('actor_optimizer',agent.actor_optimizer),('critic_optimizer',agent.critic_optimizer),('alpha_optimizer',agent.log_alpha_optimizer)]:opt.load_state_dict(copy.deepcopy(x[key]))
    if agent.log_alpha_optimizer.state_dict()['state']:raise ValueError('temperature optimizer must be inert')
    agent.total_it=x['total_it'];agent.critic_warmup_it=x['critic_warmup_it'];agent.loss_dict=copy.deepcopy(x['loss_dict'])
    import random
    torch.set_rng_state(x['torch_rng']);n=x['numpy_global'];np.random.set_state((n[0],np.array(n[1],dtype=np.uint32),n[2],n[3],n[4]));random.setstate(x['python_rng'])
    return agent,original,replay,roles

def episode_returns(records,gamma=.97):
    """Complete BEHAVIOUR trajectory returns, not current-policy certificates."""
    if not records or not records[-1]['info']['terminated']or records[-1]['info']['truncated']:return None
    raw=[0.]*len(records);soft=[0.]*len(records)
    for i in range(len(records)-1,-1,-1):
        r=float(records[i]['reward'])
        raw[i]=r+(gamma*raw[i+1]if i+1<len(records)else 0.)
        soft[i]=r+(gamma*(soft[i+1]-ALPHA*records[i+1]['behaviour']['sourceEpsilonLogProbability'])if i+1<len(records)else 0.)
    if not np.isfinite([raw,soft]).all():raise ValueError('nonfinite complete episode returns')
    return dict(taskOnly=raw,sourceSoft=soft,scope='complete off-policy behaviour trajectory; source-epsilon recorded logprob, not atanh reconstruction')
