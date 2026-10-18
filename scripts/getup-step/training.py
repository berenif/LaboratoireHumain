"""Source SAC construction/admission helpers; no collection or optimizer CLI.

The exact task config keeps learning disabled. Synthetic parity tests can build
throwaway networks, but an experiment must separately satisfy every reviewed
native/reference/ledger gate. No derived weights are exported by this module.
"""
import hashlib,importlib.util,json
from pathlib import Path
import numpy as np
from actors import DenseActor
from task import load_config
ROOT=Path(__file__).resolve().parents[2]
PINNED_SAC_SHA256='64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420'


def source_module():
    path=ROOT/'scripts/getup-training/SAC.py'
    if hashlib.sha256(path.read_bytes()).hexdigest()!=PINNED_SAC_SHA256:raise ValueError('pinned source SAC changed')
    spec=importlib.util.spec_from_file_location('getup_step_source_SAC',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # This pilot is local CPU, independent of whether a machine also has a GPU.
    module.device=module.torch.device('cpu')
    return module


def teacher_for_synthetic_parity(config):
    """Build original equations + exact reference-zero initialization, no update."""
    source=source_module();pilot=config['pilot']
    agent=source.SAC(118,21,discount=pilot['teacherDiscount'],actor_lr=pilot['actorLearningRate'],critic_lr=pilot['criticLearningRate'],alpha_lr=pilot['temperatureLearningRate'],init_temperature=.1,policy_freq=1,tau=.005)
    dense=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin').conditioned_teacher()
    with source.torch.no_grad():
        for index,(weight,bias)in zip([0,2,4],dense.layers):
            agent.actor.trunk[index].weight.copy_(source.torch.from_numpy(weight));agent.actor.trunk[index].bias.copy_(source.torch.from_numpy(bias))
    return agent


def admit_task_learning(config_path,*,expected_config_sha256,admission):
    """Fail closed; current reviewed config cannot collect or optimize."""
    config,digest=load_config(config_path,expected_sha256=expected_config_sha256)
    # Enabling the flags needs a separately reviewed config revision. load_config
    # deliberately refuses that in this pilot scaffold, so no hidden bypass.
    if not config['pilot']['collectionEnabled']or not config['pilot']['learningEnabled']:
        raise ValueError('task collection/learning has no admission')
    required=['referenceRuntimeQualified','nativeInterfaceQualified','hybridContinuationQualified','frozenBaselinePreserved','rewardReviewed','ledgerAdmitted']
    if any(admission.get(k)is not True for k in required):raise ValueError('missing task learning prerequisite')
    return config,digest


def learning_update_due(actual_transitions,updates,config,*,critic_only_updates=0):
    if type(actual_transitions)is not int or type(updates)is not int or min(actual_transitions,updates)<0:raise ValueError('invalid learning counters')
    p=config['pilot']
    if p.get('firstUpdateTransition')is None:
        if type(critic_only_updates)is not int or critic_only_updates<0:raise ValueError('invalid critic-only counter')
        return bool(critic_only_updates>=p['criticOnlyWarmupUpdates']and updates<p['maximumUpdates']and actual_transitions<=p['maximumActualTransitions']and actual_transitions>=p['warmupActualTransitions']+p['newTransitionsPerUpdate']and(actual_transitions-p['warmupActualTransitions'])%p['newTransitionsPerUpdate']==0)
    return bool(updates<p['maximumUpdates']and actual_transitions<=p['maximumActualTransitions']and actual_transitions>=p['firstUpdateTransition']and(actual_transitions-p['warmupActualTransitions'])%p['newTransitionsPerUpdate']==0)


def warmup_controls(deterministic,power,rng,config):
    deterministic=np.asarray(deterministic,dtype=float)
    if deterministic.shape!=(21,)or not np.isfinite(deterministic).all()or not 0<power<=1 or not np.isfinite(power):raise ValueError('invalid teacher controls/power')
    # Explicit pilot override; after warmup the unchanged stochastic SAC actor is
    # used instead. Power is never also applied to the original gear.
    return np.clip(deterministic+rng.normal(0.,.02,21),-power,power)


def critic_only_update(agent,replay_buffer,batch_size,*,before_optimizer_step):
    """Declared warmup adaptation: source critic loss + target Polyak EVERY step.

    Actor/alpha and their optimizers are untouched. Setting policy_freq high in
    upstream train() would incorrectly freeze the targets; do not use that hack.
    This helper has no collection loop and requires a charge/admission callback.
    """
    import torch
    if type(batch_size)is not int or batch_size<1 or not callable(before_optimizer_step):raise ValueError('critic-only admission callback required')
    state,action,next_state,reward,not_done=replay_buffer.sample(batch_size)
    for value,shape in [(state,(batch_size,118)),(action,(batch_size,21)),(next_state,(batch_size,118)),(reward,(batch_size,1)),(not_done,(batch_size,1))]:
        if tuple(value.shape)!=shape or not torch.isfinite(value).all():raise ValueError('invalid critic-only batch')
    if torch.max(action.abs())>1 or torch.any((not_done!=0)&(not_done!=1)):raise ValueError('invalid controls/bootstrap masks')
    with torch.no_grad():
        _,next_action,logp,_=agent.actor(next_state);q1,q2=agent.critic_target(next_state,next_action)
        target=reward+not_done*agent.discount*(torch.minimum(q1,q2)-agent.alpha.detach()*logp)
    q1,q2=agent.critic(state,action);loss=torch.nn.functional.mse_loss(q1,target)+torch.nn.functional.mse_loss(q2,target)
    if not torch.isfinite(loss):raise ValueError('nonfinite critic-only loss')
    agent.critic_optimizer.zero_grad(set_to_none=True);loss.backward()
    if any(p.grad is not None and not torch.isfinite(p.grad).all()for p in agent.critic.parameters()):raise ValueError('nonfinite critic-only gradient')
    before_optimizer_step();agent.critic_optimizer.step()
    with torch.no_grad():
        for p,t in zip(agent.critic.parameters(),agent.critic_target.parameters()):t.copy_(agent.tau*p+(1-agent.tau)*t)
    agent.critic_warmup_it=getattr(agent,'critic_warmup_it',0)+1
    return dict(criticLoss=float(loss.detach()),criticOnlyUpdates=agent.critic_warmup_it,sourceActorUpdates=0,sourceAlphaUpdates=0,targetUpdated=True)
