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
