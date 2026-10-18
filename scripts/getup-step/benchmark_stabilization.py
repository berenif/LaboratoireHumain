#!/usr/bin/env python3
"""Admitted-stage benchmark ONLY: fixed fitted torque teacher,64interval cap.

One first-command full-cache repeat is charged, then continue the repeated
branch. Branch-local source IDs may repeat; global charged integration IDs do
not. Original SLOW800Hz caches are never copied into this200Hz model. No actor
optimizer or task-learner replay is used. Optional16 critic-only benchmark steps
are throwaway, with all state discarded after reporting.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,sha256,atomic_json
from source_task import SourceStabilization,CONFIG_PATH
from source_task_env import SourceTaskWorld
from stabilization_init import initialize_torque_actor
from stabilization_ledger import StageBudget
from task import load_config
CONFIG_SHA='ac3f550c829c446b1c8643b059c605fd55d96a60645c00778afb7eeece6572bb'


def fingerprints(stage,source):
    paths=[Path(__file__),CONFIG_PATH,ROOT/'scripts/getup-step/stabilization_ledger.py',ROOT/'scripts/getup-step/source_task.py',ROOT/'scripts/getup-step/source_task_env.py',ROOT/'scripts/getup-step/source_slow_control.py',ROOT/'scripts/getup-step/source_engine.py',ROOT/'scripts/getup-step/stabilization_init.py',ROOT/'scripts/getup-step/training.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/reference.py',ROOT/'scripts/getup-step/task.py',ROOT/'scripts/getup-step/audit_metrics.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'scripts/getup-step-reference.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/teacher.bin',source/'source_bridge_v2.so',source/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',source/'source-model/humanoid_static.xml',stage/'source200-entry-preflight.json',stage/'source200-entry-cache-local.bin',stage/'fit-500-summary.json',stage/'fit-500-torque-initializer-local.pt']
    return{str(p.resolve()):sha256(p)for p in paths}


def actor_for(stage,config):
    import torch
    torch.set_num_threads(1);actor=initialize_torque_actor(config);actor.load_state_dict(torch.load(stage/'fit-500-torque-initializer-local.pt',map_location='cpu',weights_only=True),strict=True);actor.eval();return actor


def prepare(stage,source):
    import torch
    fit=json.loads((stage/'fit-500-summary.json').read_text());entry=json.loads((stage/'source200-entry-preflight.json').read_text());config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA)
    if not fit['admitted']or not fit['reconstructionBitExact']or not entry['passed']:raise ValueError('benchmark prerequisites failed')
    with StageBudget(stage/'ledger.json')as budget:local_deadline=budget.start_benchmark_clock();budget.allowance('benchmark')
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());w=SourceTaskWorld(source)
    try:
        w.restore((stage/'source200-entry-cache-local.bin').read_bytes());packet=w.task_packet()
        if packet!=entry['entryPacket']:raise ValueError('source200 exact entry/cache mismatch')
        core=SourceStabilization(packet,model);actor=actor_for(stage,config);obs=core.observation(w.observation())
        with torch.inference_mode():u,_,_,logstd=actor(torch.tensor(obs)[None,:],compute_pi=False,compute_log_pi=False)
        if not torch.isfinite(u).all()or torch.max(abs(u))>1:raise ValueError('invalid fitted controls')
        atomic_json(stage/'benchmark-preflight.json',dict(passed=True,benchmarkDeadlineUtc=local_deadline,physicsIntegrations=0,optimizerUpdates=0,maximumPolicyIntervals=64,maximumIntegrations=320,maximumSimSecondsAllBranches=1.6,maximumPhysicsSeconds=1.,firstCommandRepeatCharged=True,firstAction=u[0].numpy().tolist(),sourceInitialCounter=w.dim(6),sourceInitialTimeS=w.lib.lh_time(w.h),sourceDtS=.005,initializationStrictQuietQualified=False,fullCacheZeroStepEqual=True,continuedDynamicsParityQualified=False,maximumThrowawayCriticUpdates=16,sourceActorUpdates=0,configSHA256=CONFIG_SHA,inputHashes=fingerprints(stage,source)))
    finally:w.close()


def run(stage,source):
    import torch
    pre=json.loads((stage/'benchmark-preflight.json').read_text())
    for p,h in pre['inputHashes'].items():
        if sha256(p)!=h:raise ValueError('benchmark input changed')
    config,_=load_config(CONFIG_PATH,expected_sha256=CONFIG_SHA);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());actor=actor_for(stage,config)
    (stage/'benchmark-started-once').open('x').close();out=stage/'benchmark';out.mkdir();world=SourceTaskWorld(source);core=None;rows=[];transitions=[];pair_first=None;pair_equal=False;stop=None;started=time.monotonic();rng=np.random.default_rng(0)
    with StageBudget(stage/'ledger.json')as budget:
        try:
            world.restore((stage/'source200-entry-cache-local.bin').read_bytes());initial=world.task_packet();core=SourceStabilization(initial,model);cache=world.checkpoint();bindings={'benchmarkInputsSHA256':sha256(stage/'benchmark-preflight.json')};side=core.checkpoint(cache,bindings,rng.bit_generator.state)
            for command in range(64):
                branch='repeatA'if command==0 else'repeatB-continuation'
                if command==1:
                    world.restore(cache);core=SourceStabilization(initial,model);rng.bit_generator.state=core.restore(side,cache,world.task_packet(),expected_bindings=bindings)
                obs=core.observation(world.observation())
                with torch.inference_mode():mean,_,_,_=actor(torch.as_tensor(obs)[None,:],compute_pi=False,compute_log_pi=False)
                u=mean[0].numpy().copy();budget.begin_interval('benchmark');world.arr(2,(21,))[:]=u;packet_rows=[]
                for hold in range(5):
                    budget.integrate('benchmark',lambda maximum:world.lib.lh_step_audited(world.h,maximum),lambda:world.dim(6))
                    row=world.task_packet();packet_rows.append(row);rows.append(dict(globalIntegration=budget.data['sourceIntegrations'],command=command,hold=hold,branch=branch,packet=row))
                    if core.terminal_in_prefix(packet_rows):break
                result=core.consume(packet_rows,expected_teacher_controls=u)
                transitions.append(dict(command=command,branch=branch,state=obs.tolist(),action=u.tolist(),next_state=result['actualFinalObservation118'],reward=result['reward'],not_done=result['not_done'],info=result))
                if command==0:pair_first=copy.deepcopy(packet_rows)
                if command==1:
                    pair_equal=packet_rows==pair_first
                    if not pair_equal:stop='full-cache-continued-physics-parity-failed';break
                if result['terminated']or result['truncated']:stop='actual-terminal-'+str(result['physicalFailure']);break
            stop=stop or'completed64-interval-held-control-benchmark'
            # No model/optimizer state from this throughput benchmark is retained
            # as a learner; only scalar loss/rate evidence is recorded.
            if stop=='completed64-interval-held-control-benchmark'and pair_equal:
                from training import source_module,critic_only_update
                module=source_module();torch.manual_seed(0);agent=module.SAC(118,21,discount=.97,actor_lr=1e-5,critic_lr=1e-4,alpha_lr=1e-4,policy_freq=1,tau=.005);agent.actor.load_state_dict(actor.state_dict());fixed={k:v.detach().clone()for k,v in agent.actor.state_dict().items()};alpha=agent.log_alpha.detach().clone();sample_rng=np.random.default_rng(0)
                class Replay:
                    def sample(self,n):
                        batch=[transitions[i]for i in sample_rng.integers(0,len(transitions),n)]
                        return tuple(torch.tensor(np.array([t[k]for t in batch]),dtype=torch.float32).reshape(n,-1)for k in ['state','action','next_state','reward','not_done'])
                losses=[]
                for update in range(16):
                    budget.allowance('benchmark')
                    def charge():
                        budget.data['benchmarkThrowawayCriticUpdates']=update+1;atomic_json(stage/'ledger.json',budget.data)
                    losses.append(critic_only_update(agent,Replay(),128,before_optimizer_step=charge)['criticLoss'])
                if any(not torch.equal(v,fixed[k])for k,v in agent.actor.state_dict().items())or not torch.equal(agent.log_alpha,alpha):raise ValueError('benchmark changed frozen actor/alpha')
                atomic_json(out/'throwaway-critic-throughput.json',dict(updates=16,losses=losses,actorUnchanged=True,alphaUnchanged=True,allNetworkOptimizerTargetAndRNGStateDiscarded=True))
        except BaseException as error:stop=f'{type(error).__name__}: {error}'
        finally:
            if core is not None:
                blob=world.checkpoint();(out/'final-source-cache-local.bin').write_bytes(blob)
                try:
                    if world.dim(6)!=core.native_id or core.packet_hash(world.task_packet())!=core.last_packet_hash:raise ValueError('native/controller boundary differs; not resumable')
                    atomic_json(out/'final-controller.json',core.checkpoint(blob,bindings,rng.bit_generator.state))
                except BaseException as e:atomic_json(out/'final-controller-invalid.json',dict(error=str(e),reason='last native call may not have formed a valid complete transition'))
            atomic_json(out/'transitions.json',transitions)
            with(out/'frames.ndjson').open('w')as stream:
                for row in rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
            report=dict(stopped=stop,sourcePolicyIntervals=budget.data['stagePolicyIntervals']['benchmark'],allChargedIntegrations=budget.data['sourceIntegrations'],sourcePhysicsSeconds=budget.data['stagePhysicsSeconds']['benchmark'],processWallSeconds=time.monotonic()-started,fullCacheContinuedDynamicsBitExact=pair_equal,final=None if not transitions else transitions[-1]['info'],sourceActorUpdates=0,sourceAlphaUpdates=0,benchmarkCriticUpdates=budget.data.get('benchmarkThrowawayCriticUpdates',0),taskLearnerDataCollected=False,actualTeacherInitializationAdmitted=stop=='completed64-interval-held-control-benchmark'and pair_equal,inputHashes=pre['inputHashes']);atomic_json(out/'summary.json',report);budget.data['status']='benchmark-complete'if report['actualTeacherInitializationAdmitted']else'benchmark-failed-stop';budget.data['teacherInitializationBenchmarkPassed']=report['actualTeacherInitializationAdmitted'];atomic_json(stage/'ledger.json',budget.data);world.close();print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','final']},indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);a=p.parse_args()
    if a.mode=='prepare':prepare(a.stage,a.source)
    else:run(a.stage,a.source)
if __name__=='__main__':main()
