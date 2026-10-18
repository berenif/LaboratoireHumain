#!/usr/bin/env python3
"""One admitted, hash-locked local supervised torque initialization; no physics.

Not a SAC run, teacher replay, or transferable stepping policy. All derived actor
weights stay local. Requires the independently reviewed causal dataset/helpers.
"""
import argparse,datetime as dt,fcntl,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,sha256,atomic_json
from source_task import CONFIG_PATH
from task import load_config
from stabilization_init import build_causal_dataset,initialize_torque_actor,control_error,fit_mean_only

CONFIG_SHA='ac3f550c829c446b1c8643b059c605fd55d96a60645c00778afb7eeece6572bb'
SOURCE_JOURNAL_SHA='a880c0f00ba698da9c06267f17ea677410cf0147d70f105942f4892da7bef47a'


def inputs(source):
    files=[Path(__file__),CONFIG_PATH,ROOT/'scripts/getup-step/stabilization_init.py',ROOT/'scripts/getup-step/source_task.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/training.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-step/task.py',ROOT/'scripts/getup-step/audit_metrics.py',ROOT/'scripts/getup-step/reference.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'scripts/getup-step-reference.py',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin',source/'slow-continuation/frames.ndjson']
    return{str(p.resolve()):sha256(p)for p in files}


def prepare(stage,source):
    config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA)
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());dataset,receipt=build_causal_dataset(source/'slow-continuation/frames.ndjson',model)
    if receipt['sourceJournalSHA256']!=SOURCE_JOURNAL_SHA:raise ValueError('not the reviewed actual SLOW journal')
    actor=initialize_torque_actor(config);error=control_error(actor,dataset['observation'],dataset['control'])
    if error['maximumStdError']>1e-7:raise ValueError('source logstd inverse map mismatch')
    np.savez(stage/'causal-torque-initialization.npz',**dataset);atomic_json(stage/'causal-torque-initialization.json',receipt)
    atomic_json(stage/'fit-preflight.json',dict(passed=True,optimizerUpdates=0,physicsIntegrations=0,configSHA256=CONFIG_SHA,datasetSHA256=sha256(stage/'causal-torque-initialization.npz'),causalReceipt=receipt,initialActionError=error,maximumSupervisedUpdates=100,maximumFitLearningRate=1e-5,sourceSACUpdates=0,inputHashes=inputs(source)))


def run(stage,source):
    import torch
    lock=(stage/'stage.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    pre=json.loads((stage/'fit-preflight.json').read_text());ledger=json.loads((stage/'ledger.json').read_text())
    if not pre['passed']or ledger['supervisedUpdates']or ledger.get('requestOutstanding'):raise ValueError('fit stage is not fresh/admitted')
    for path,digest in pre['inputHashes'].items():
        if sha256(path)!=digest:raise ValueError('fit input changed after preflight')
    if sha256(stage/'causal-torque-initialization.npz')!=pre['datasetSHA256']:raise ValueError('fit dataset changed')
    config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA);data=dict(np.load(stage/'causal-torque-initialization.npz',allow_pickle=False));actor=initialize_torque_actor(config)
    (stage/'fit-started-once').open('x').close();torch.save(actor.state_dict(),stage/'unfitted-torque-base-local.pt')
    deadline=dt.datetime.fromisoformat(ledger['deadlineUtc'].replace('Z','+00:00'));started=time.monotonic()
    def charge(update):
        if (deadline-dt.datetime.now(dt.timezone.utc)).total_seconds()<=ledger['validationWallReservedSeconds']:raise TimeoutError('fit must protect validation wall reserve')
        if update>=ledger['supervisedUpdateLimit']:raise ValueError('supervised optimizer cap')
        ledger.update(status='supervised-torque-fit',supervisedUpdates=update+1,pendingSupervisedUpdate=update+1);atomic_json(stage/'ledger.json',ledger)
    try:
        report=fit_mean_only(actor,data,config,before_optimizer_step=charge)
        report.update(wallSeconds=time.monotonic()-started,sourceSACUpdates=0,physicsIntegrations=0,datasetSHA256=pre['datasetSHA256'],inputHashes=pre['inputHashes'],actualTeacherBenchmarkQualified=False,derivedWeightsRedistributed=False)
        torch.save(actor.state_dict(),stage/'fitted-torque-initializer-local.pt');report['actorSHA256']=sha256(stage/'fitted-torque-initializer-local.pt')
        ledger.update(status='fit-complete-awaiting-held-control-benchmark'if report['admitted']else'fit-admission-failed-stop',pendingSupervisedUpdate=None,fitAdmitted=report['admitted']);atomic_json(stage/'fit-summary.json',report);print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','fit','heldout']},indent=2));print(json.dumps(report['heldout'],indent=2))
    except BaseException as error:
        ledger.update(status='fit-aborted-no-fallback',failure=f'{type(error).__name__}: {error}');torch.save(actor.state_dict(),stage/'aborted-torque-fit-local.pt');atomic_json(stage/'fit-abort-summary.json',dict(failure=ledger['failure'],supervisedUpdates=ledger['supervisedUpdates'],physicsIntegrations=0,sourceSACUpdates=0));raise
    finally:atomic_json(stage/'ledger.json',ledger);fcntl.flock(lock,fcntl.LOCK_UN);lock.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);a=p.parse_args()
    if a.mode=='prepare':prepare(a.stage,a.source)
    else:run(a.stage,a.source)
if __name__=='__main__':main()
