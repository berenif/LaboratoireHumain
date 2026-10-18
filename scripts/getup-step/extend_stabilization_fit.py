#!/usr/bin/env python3
"""Explicit counted reconstruction100 + continuation400, fixed500 endpoint.

The original100 failure is preserved. Reconstruction calls are NOT free:
100 original +100 reconstruction +400 new =600 actual optimizer calls.
No source physics, SAC, best-checkpoint selection or initializer fallback.
"""
import argparse,datetime as dt,fcntl,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,sha256,atomic_json
from task import load_config
from source_task import CONFIG_PATH
from stabilization_init import initialize_torque_actor,control_error
CONFIG_SHA='ac3f550c829c446b1c8643b059c605fd55d96a60645c00778afb7eeece6572bb'


def save_full(path,actor,optimizer,rng,trajectory_updates,actual_calls,dataset_sha):
    import torch
    payload=dict(schema=1,kind='local-supervised-torque-fit-fullstate',actor=actor.state_dict(),optimizer=optimizer.state_dict(),minibatch_rng=rng.bit_generator.state,torch_rng=torch.get_rng_state(),trajectoryUpdates=trajectory_updates,actualOptimizerCalls=actual_calls,configSHA256=CONFIG_SHA,datasetSHA256=dataset_sha)
    temporary=path.with_suffix('.tmp');torch.save(payload,temporary);temporary.replace(path)


def restore_full(path,actor,optimizer,rng,*,dataset_sha):
    import torch
    saved=torch.load(path,map_location='cpu',weights_only=True)
    if saved['schema']!=1 or saved['configSHA256']!=CONFIG_SHA or saved['datasetSHA256']!=dataset_sha:raise ValueError('fit checkpoint configuration/data mismatch')
    actor.load_state_dict(saved['actor'],strict=True);optimizer.load_state_dict(saved['optimizer']);rng.bit_generator.state=saved['minibatch_rng'];torch.set_rng_state(saved['torch_rng']);return saved


def inputs(stage):
    paths=[Path(__file__),CONFIG_PATH,ROOT/'scripts/getup-step/stabilization_init.py',ROOT/'scripts/getup-step/fit_stabilization.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/source_task.py',ROOT/'scripts/getup-step/training.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin',stage/'causal-torque-initialization.npz',stage/'causal-torque-initialization.json',stage/'unfitted-torque-base-local.pt',stage/'fitted-torque-initializer-local.pt',stage/'fit-summary.json']
    return{str(p.resolve()):sha256(p)for p in paths}


def prepare(stage):
    config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA);old=json.loads((stage/'fit-summary.json').read_text());ledger=json.loads((stage/'ledger.json').read_text())
    if old['updates']!=100 or old['admitted']or ledger['supervisedUpdates']!=100 or ledger['physicsSeconds']!=0:raise ValueError('not the preserved failed100 trajectory')
    atomic_json(stage/'fit-extension-admission.json',dict(schema=1,status='parent-approved-awaiting-R4-runner-review',originalUpdates=100,reconstructionOptimizerCalls=100,newTrajectoryUpdates=400,fixedTrajectoryEndpoint=500,maximumActualOptimizerCalls=600,configurationAndDatasetUnchanged=True,bitExact100WeightCheckRequired=True,maximumRMS=.02,maximumAbsolute=.05,wallDeadlineUnchanged=ledger['deadlineUtc'],protectedWallSeconds=300,fullAdamMinibatchAndTorchRNGSavedEveryStep=True,physicsAllowanceAdded=0,sourceSACAllowanceAdded=0,inputHashes=inputs(stage)))


def run(stage):
    import torch
    torch.set_num_threads(1)
    lock=(stage/'stage.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);admission=json.loads((stage/'fit-extension-admission.json').read_text());ledger=json.loads((stage/'ledger.json').read_text())
    if ledger['supervisedUpdates']!=100 or ledger.get('pendingSupervisedUpdate')is not None or ledger['physicsSeconds']!=0:raise ValueError('extension not fresh')
    for path,digest in admission['inputHashes'].items():
        if sha256(path)!=digest:raise ValueError('extension input changed')
    (stage/'fit-extension-started-once').open('x').close();config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA);data=dict(np.load(stage/'causal-torque-initialization.npz',allow_pickle=False));actor=initialize_torque_actor(config)
    original=torch.load(stage/'fitted-torque-initializer-local.pt',map_location='cpu',weights_only=True);optimizer=torch.optim.Adam(actor.parameters(),lr=1e-5,betas=(.9,.999));rng=np.random.default_rng(0)
    x=torch.as_tensor(data['observation'][:120],dtype=torch.float32);y=torch.as_tensor(data['raw_mean_target'][:120],dtype=torch.float32);last=actor.trunk[4];fixed_w=last.weight[21:].detach().clone();fixed_b=last.bias[21:].detach().clone();dataset_sha=sha256(stage/'causal-torque-initialization.npz')
    deadline=dt.datetime.fromisoformat(ledger['deadlineUtc'].replace('Z','+00:00'));losses=[];started=time.monotonic();bit_exact=False
    ledger.update(status='admitted-fit-reconstruction-and-extension',supervisedUpdateLimit=600,originalSupervisedUpdates=100,reconstructionSupervisedUpdates=0,newExtensionSupervisedUpdates=0,parameterTrajectoryUpdates=0,parameterTrajectoryUpdateLimit=500,extensionAdmissionSHA256=sha256(stage/'fit-extension-admission.json'));atomic_json(stage/'ledger.json',ledger)
    save_full(stage/'fit-extension-fullstate-last-local.pt',actor,optimizer,rng,0,100,dataset_sha)
    try:
        for update in range(500):
            if (deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()<=300:raise TimeoutError('fit extension must preserve validation wall')
            if ledger['supervisedUpdates']>=600:raise ValueError('actual optimizer600 cap')
            index=torch.as_tensor(rng.integers(0,120,128),dtype=torch.long);raw=actor.trunk(x[index]);loss=torch.mean((raw[:,:21]-y[index])**2)
            if not torch.isfinite(loss):raise ValueError('nonfinite extension loss')
            optimizer.zero_grad(set_to_none=True);loss.backward()
            if any(p.grad is not None and not torch.isfinite(p.grad).all()for p in actor.parameters())or torch.count_nonzero(last.weight.grad[21:])or torch.count_nonzero(last.bias.grad[21:]):raise ValueError('invalid extension gradient')
            ledger['supervisedUpdates']+=1;ledger['pendingSupervisedUpdate']=update+1
            role='reconstructionSupervisedUpdates'if update<100 else'newExtensionSupervisedUpdates';ledger[role]+=1;atomic_json(stage/'ledger.json',ledger)
            optimizer.step()
            if any(not torch.isfinite(p).all()for p in actor.parameters())or not torch.equal(last.weight[21:],fixed_w)or not torch.equal(last.bias[21:],fixed_b):raise ValueError('invalid parameters/frozen std changed')
            losses.append(float(loss.detach()));save_full(stage/'fit-extension-fullstate-last-local.pt',actor,optimizer,rng,update+1,ledger['supervisedUpdates'],dataset_sha)
            ledger.update(parameterTrajectoryUpdates=update+1,pendingSupervisedUpdate=None);atomic_json(stage/'ledger.json',ledger)
            if update==99:
                bit_exact=all(torch.equal(value,original[key])for key,value in actor.state_dict().items())
                atomic_json(stage/'fit-100-reconstruction-check.json',dict(bitExactActor=bit_exact,actualOptimizerCalls=ledger['supervisedUpdates'],trajectoryUpdates=100,allNamedTensorsCompared=list(original),firstLoss=losses[0],finalLoss=losses[-1],scope='deterministic reconstruction; original optimizer was not saved; all100 reconstruction calls charged'))
                if not bit_exact:raise ValueError('reconstructed100 actor not bit exact: extension prohibited')
        report=dict(trajectoryUpdates=500,actualOptimizerCalls=ledger['supervisedUpdates'],originalOptimizerCalls=100,reconstructionCalls=100,newExtensionCalls=400,reconstructionBitExact=bit_exact,fit=control_error(actor,data['observation'][:120],data['control'][:120]),heldout=control_error(actor,data['observation'][120:],data['control'][120:]),wallSeconds=time.monotonic()-started,fixedEndpointOnly=True,fullLosses=losses,sourceSACUpdates=0,physicsIntegrations=0,original100FailurePreserved=True)
        report['admitted']=bool(report['heldout']['rms']<=.02 and report['heldout']['maximumAbsolute']<=.05 and report['heldout']['maximumStdError']<1e-7)
        torch.save(actor.state_dict(),stage/'fit-500-torque-initializer-local.pt');report['actorSHA256']=sha256(stage/'fit-500-torque-initializer-local.pt');atomic_json(stage/'fit-500-summary.json',report)
        ledger.update(status='fit500-complete-awaiting-benchmark'if report['admitted']else'fit500-admission-failed-stop',fit500Admitted=report['admitted']);print(json.dumps({k:v for k,v in report.items()if k not in['fullLosses','fit','heldout']},indent=2));print(json.dumps(report['heldout'],indent=2))
    except BaseException as error:
        ledger.update(status='fit-extension-aborted-no-fallback',failure=f'{type(error).__name__}: {error}');atomic_json(stage/'fit-extension-abort.json',dict(failure=ledger['failure'],actualOptimizerCalls=ledger['supervisedUpdates'],trajectoryUpdates=ledger['parameterTrajectoryUpdates'],original100FailurePreserved=True));raise
    finally:atomic_json(stage/'ledger.json',ledger);fcntl.flock(lock,fcntl.LOCK_UN);lock.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);a=p.parse_args()
    if a.mode=='prepare':prepare(a.stage)
    else:run(a.stage)
if __name__=='__main__':main()
