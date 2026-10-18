"""Offline-only projected mean-head SAC proposals. No engine or CLI on import.

Original hidden layers, state-dependent logstd and temperature remain fixed.
An optimizer callback is mandatory. Backtracking does only forward arithmetic.
"""
import copy,hashlib
import numpy as np
import torch
from slow_task import initialize_slow_agent,original_slow_actor

ALPHA=0.000612579835321518

def initialize(seed=0):
    agent,_=initialize_slow_agent(seed)
    original=original_slow_actor();agent.actor.load_state_dict(original.state_dict())
    with torch.no_grad():agent.log_alpha.fill_(np.log(ALPHA))
    agent.log_alpha.requires_grad_(False)  # inherited alpha optimizer is inert
    for p in agent.actor.parameters():p.requires_grad_(False)
    head=agent.actor.trunk[4]
    for p in [head.weight,head.bias]:p.requires_grad_(True)
    # Adam has no weight decay; zero std gradients + zero initial moments leave
    # its frozen rows exactly unchanged. Assert the entire frozen state below.
    def mask(g):
        g=g.clone();g[21:]=0;return g
    head.weight.register_hook(mask);head.bias.register_hook(mask)
    agent.actor_optimizer=torch.optim.Adam([head.weight,head.bias],lr=1e-5,betas=(.9,.999))
    return agent,original

def gaussian(actor,x):
    raw=actor.trunk(x);mu,logstd=raw.chunk(2,-1)
    return mu,(-5+3.5*(torch.tanh(logstd)+1))

def fixed_noise_loss(agent,state,noise):
    from training import source_module
    source=source_module();mu,logstd=gaussian(agent.actor,state)
    latent=mu+noise*logstd.exp()
    logp=source.gaussian_logprob(noise,logstd)
    _,action,logp=source.squash(mu,latent,logp,state[:,-1])
    q1,q2=agent.critic(state,action)
    loss=(agent.alpha.detach()*logp-torch.minimum(q1,q2)).mean()
    if not torch.isfinite(loss):raise ValueError('nonfinite fixed-noise actor loss')
    return loss

def frozen_equal(actor,anchor):
    for k,v in actor.state_dict().items():
        expected=anchor[k]
        if k.startswith('trunk.4'):v,expected=v[21:],expected[21:]
        if not torch.equal(v,expected):raise ValueError('frozen original trunk/std changed')

def trust_kl(actor,witness,anchor_mu,anchor_std):
    mu,logstd=gaussian(actor,witness)
    if not torch.equal(logstd.exp(),anchor_std):raise ValueError('original per-state std changed')
    kl=.5*torch.sum(((mu-anchor_mu)/anchor_std)**2,-1)
    if not torch.isfinite(kl).all():raise ValueError('nonfinite KL witness')
    return kl

def proposal(agent,state,witness,anchor_mu,anchor_std,anchor_tensors,*,charge,complete,check_wall,kl_limit=.05,minimum_decrease=1e-6):
    """One charged Adam attempt; only actor-side postcritic state rolls back.

    The caller already sampled replay and updated critic/target. Those states and
    their RNG/charges are intentionally NOT restored on actor rejection.
    """
    frozen_equal(agent.actor,anchor_tensors);check_wall()
    before=copy.deepcopy(agent.actor.state_dict());opt=copy.deepcopy(agent.actor_optimizer.state_dict());rng=torch.get_rng_state().clone()
    noise=torch.randn((len(state),21));baseline=float(fixed_noise_loss(agent,state,noise).detach())
    agent.actor_optimizer.zero_grad(set_to_none=True);loss=fixed_noise_loss(agent,state,noise);loss.backward()
    params=agent.actor_optimizer.param_groups[0]['params']
    if any(p.grad is None or not torch.isfinite(p.grad).all()for p in params):raise ValueError('invalid mean-head gradient')
    charge();agent.actor_optimizer.step();complete()
    frozen_equal(agent.actor,anchor_tensors);after=copy.deepcopy(agent.actor.state_dict());trials=[];accepted=None
    for k in range(11):
        check_wall();scale=2.**(-k)
        with torch.no_grad():
            for key,value in agent.actor.state_dict().items():
                if key.startswith('trunk.4'):value.copy_(before[key]+scale*(after[key]-before[key]))
            frozen_equal(agent.actor,anchor_tensors)
            candidate=float(fixed_noise_loss(agent,state,noise));kl=trust_kl(agent.actor,witness,anchor_mu,anchor_std)
            record=dict(scale=scale,actorLoss=candidate,maximumKL=float(kl.max()),meanKL=float(kl.mean()),lossDecrease=baseline-candidate)
        trials.append(record)
        if record['lossDecrease']>=minimum_decrease and record['maximumKL']<=kl_limit:accepted=scale;break
    if accepted is None:
        agent.actor.load_state_dict(before);agent.actor_optimizer.load_state_dict(opt);torch.set_rng_state(rng)
    frozen_equal(agent.actor,anchor_tensors)
    return dict(accepted=accepted is not None,scale=accepted,baselineActorLoss=baseline,trials=trials,optimizerProposalsCharged=1,actorNoiseRngRestored=accepted is None,actorNoiseSHA256=hashlib.sha256(noise.numpy().tobytes()).hexdigest(),actorInputBatchSHA256=hashlib.sha256(state.detach().numpy().tobytes()).hexdigest(),preActorTorchRngSHA256=hashlib.sha256(rng.numpy().tobytes()).hexdigest(),resultTorchRngSHA256=hashlib.sha256(torch.get_rng_state().numpy().tobytes()).hexdigest())

def drift(actor,original,states):
    with torch.no_grad():
        delta=actor(states,False,False)[0]-original(states,False,False)[0]
        return dict(rms=float(torch.sqrt(torch.mean(delta**2))),maximum=float(delta.abs().max()))

def restore(agent,state):
    from slow_learning import restore_learner
    state=copy.deepcopy(state);count=state['critic_warmup_it']
    if type(count)is not int or not 0<=count<=105:raise ValueError('invalid trust critic count')
    state['critic_warmup_it']=min(count,100)
    replay=restore_learner(agent,state);agent.critic_warmup_it=count
    if abs(float(agent.alpha.detach())-ALPHA)>1e-10:raise ValueError('fixed learned temperature changed')
    return replay
