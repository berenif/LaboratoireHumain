#!/usr/bin/env python3
"""Bounded original800Hz SLOW residual benchmark. No candidate SAC release.

Every integration, including exact first-command repeat, is globally charged.
Task reward is measured, never a replacement for actual source motor equations.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,atomic_json,sha256
from slow_task import SourceSLOWWorld,SourceSLOWEnv,CONFIG_PATH,CONFIG_SHA,initialize_slow_agent
from stabilization_ledger import StageBudget
from source_math import ORDER

def fingerprints(stage,source):
    local=['slow_benchmark.py','slow_task.py','source_task.py','source_observer.py','source_task_env.py','source_slow_control.py','source_engine.py','source_positive_control.py','source_math.py','actors.py','reference.py','audit_metrics.py','native_env.py','task.py','training.py','stabilization_init.py','stabilization_ledger.py','source_bridge.c']
    paths=[Path(__file__).with_name(n)for n in local]+[CONFIG_PATH,ROOT/'scripts/getup-step-reference.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',source/'source_bridge_v2.so',source/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',*sorted((source/'source-model').glob('*.xml')),source/'slow-continuation/final.bin',source/'slow-continuation/final.controller.json',source/'slow-continuation/frames.ndjson']
    return {str(p.resolve()):sha256(p)for p in paths}

def prepare(stage,source):
    import torch
    world=SourceSLOWWorld(source);clone=SourceSLOWWorld(source)
    try:
        blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(blob);clone.restore(blob);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());packet=world.task_packet();last=json.loads((source/'slow-continuation/frames.ndjson').read_text().splitlines()[-1]);env=SourceSLOWEnv(world,model,bindings={'source':'zero-step'},rng=np.random.default_rng(0));agent,original=initialize_slow_agent();obs=env.core.observation(world.observation());x=torch.tensor(obs)[None,:]
        if world.checkpoint()!=blob or clone.checkpoint()!=blob or clone.task_packet()!=packet:raise ValueError('full800Hz source cache zero-step equality failed')
        if any(packet[k]!=last[k]for k in ['qpos','qvel','physicalObservation','comSource','qfrcActuator']):raise ValueError('actual original SLOW endpoint mismatch')
        with torch.inference_mode():a=agent.actor(x,False,False);b=original(x,False,False);raw=agent.actor.trunk(x)[:,:21];raw0=original.trunk(x)[:,:21]
        if not torch.equal(a[0],b[0])or not torch.equal(raw,raw0)or torch.max(abs(a[3].exp()-.02))>1e-8:raise ValueError('original SLOW mean/std initialization failed')
        atomic_json(stage/'benchmark-preflight.json',dict(passed=True,sourceHz=800,policyHz=40,sourceInitialCounter=world.dim(6),sourceInitialTimeS=packet['timeS'],sourceInitialPacket=packet,sourceCacheCopiedAcrossModels=False,originalSourceBatch1MeanBitExact=True,originalSourceRawMeanBitExact=True,initialLatentStd=.02,fullCacheZeroStepEqual=True,continuedPhysicalEvolutionQualified=False,maximumAllBranchPolicyIntervals=64,maximumAllBranchSourceIntegrations=1280,maximumAllBranchSimSeconds=1.6,maximumBenchmarkPhysicsSeconds=1.,maximumThrowawayCriticUpdates=16,sourceActorUpdates=0,taskCollectionEnabled=False,initialStrictQuietQualified=False,configSHA256=CONFIG_SHA,inputHashes=fingerprints(stage,source)))
        print('SLOW800Hz zero-step cache/mean/raw/std admission passed;0 integrations/updates')
    finally:world.close();clone.close()

def run(stage,source):
    import torch
    pre=json.loads((stage/'benchmark-preflight.json').read_text())
    for name,digest in pre['inputHashes'].items():
        if sha256(name)!=digest:raise ValueError('reviewed benchmark fingerprint changed')
    out=stage/'benchmark';out.mkdir(exist_ok=False);world=None;env=None;rows=[];transitions=[];pair_first=None;pair=False;stop=None;throwaway=[];started=time.monotonic()
    with StageBudget(stage/'ledger.json')as budget:
        if budget.profile!='slow-stabilization'or budget.data['sourceIntegrations']or budget.data['totalPolicyIntervals']:raise ValueError('fresh SLOW stage benchmark required')
        (stage/'benchmark-started-once').open('x').close();budget.start_benchmark_clock();budget.allowance('benchmark')
        try:
            agent,original=initialize_slow_agent();world=SourceSLOWWorld(source);blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(blob);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());initial=world.task_packet();bindings={'benchmarkPreflightSHA256':sha256(stage/'benchmark-preflight.json')};rng=np.random.default_rng(0);env=SourceSLOWEnv(world,model,bindings=bindings,rng=rng);cache,side=env.checkpoint()
            if initial!=pre['sourceInitialPacket']:raise ValueError('actual800Hz source initial packet changed')
            for command in range(64):
                if command==1:env.restore(cache,side)
                obs=env.core.observation(world.observation())
                with torch.inference_mode():mean,_,_,_=agent.actor(torch.tensor(obs)[None,:],False,False)
                budget.begin_interval('benchmark');result=env.measured_interval(mean[0].numpy(),lambda w:budget.integrate('benchmark',lambda maximum:w.lib.lh_step_audited(w.h,maximum),lambda:w.dim(6)))
                transitions.append({k:v for k,v in result.items()if k not in ['actualFrames','perNativeControls']})
                count=len(result['actualFrames']);end=budget.data['sourceIntegrations']
                for i,(packet,control)in enumerate(zip(result['actualFrames'],result['perNativeControls'])):rows.append(dict(globalIntegration=end-count+i+1,command=command,branch='repeatA'if command==0 else'repeatB-continuation',packet=packet,sourceControlAudit=control))
                if command==0:pair_first=copy.deepcopy(result)
                if command==1:
                    pair=result==pair_first
                    if not pair:stop='full800Hz-cache-continuation-mismatch';break
                if result['info']['terminated']or result['info']['truncated']:stop='physical-terminal-'+str(result['info']['physicalFailure']);break
            stop=stop or'completed64-SLOW-residual-benchmark'
            if stop=='completed64-SLOW-residual-benchmark'and pair:
                from training import critic_only_update
                sample_rng=np.random.default_rng(0)
                class Replay:
                    def sample(self,n):
                        batch=[transitions[i]for i in sample_rng.integers(0,len(transitions),n)]
                        return tuple(torch.tensor(np.array([t[k]for t in batch]),dtype=torch.float32).reshape(n,-1)for k in ['state','action','next_state','reward','not_done'])
                fixed={k:v.detach().clone()for k,v in agent.actor.state_dict().items()};alpha=agent.log_alpha.detach().clone()
                for i in range(16):
                    budget.allowance('benchmark')
                    def charge():budget.data.update(benchmarkThrowawayUpdates=i+1);atomic_json(stage/'ledger.json',budget.data)
                    throwaway.append(critic_only_update(agent,Replay(),128,before_optimizer_step=charge)['criticLoss'])
                if any(not torch.equal(v,fixed[k])for k,v in agent.actor.state_dict().items())or not torch.equal(agent.log_alpha,alpha):raise ValueError('throwaway critic phase changed actor/alpha')
        except BaseException as e:stop=type(e).__name__+': '+str(e)
        finally:
            if world is not None:
                (out/'final-source-cache-local.bin').write_bytes(world.checkpoint())
                try:
                    blob,side=env.checkpoint();atomic_json(out/'final-controller.json',side)
                except BaseException as e:atomic_json(out/'final-controller-invalid.json',dict(error=str(e)))
            with(out/'frames.ndjson').open('w')as f:
                for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
            atomic_json(out/'transitions.json',transitions);elapsed=time.monotonic()-started
            report=dict(stopped=stop,passed=stop=='completed64-SLOW-residual-benchmark'and pair and len(throwaway)==16,allBranchNativeIntegrations=budget.data['sourceIntegrations'],allBranchPolicyIntervals=budget.data['totalPolicyIntervals'],fullCache20SampleContinuationBitExact=pair,physicsSeconds=budget.data['stagePhysicsSeconds']['benchmark'],processWallSeconds=elapsed,endToEndPolicyIntervalsPerSecond=budget.data['totalPolicyIntervals']/elapsed,throwawayCriticUpdates=len(throwaway),throwawayCriticLosses=throwaway,allBenchmarkLearningStateDiscarded=True,actorUpdates=0,sourceAlphaUpdates=0,taskLearnerDataCollected=False,maximumPerNativeTorqueErrorNm=max([t['info']['maximumPerNativeTorqueErrorNm']for t in transitions]or[0]),final=None if not transitions else transitions[-1]['info'],inputHashes=pre['inputHashes']);atomic_json(out/'summary.json',report);budget.data.update(status='SLOW-benchmark-passed'if report['passed']else'SLOW-benchmark-failed-stop',slowBenchmarkPassed=report['passed']);atomic_json(stage/'ledger.json',budget.data)
            if world is not None:world.close()
            print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','final','throwawayCriticLosses']},indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);a=p.parse_args();(prepare if a.mode=='prepare'else run)(a.stage,a.source)
if __name__=='__main__':main()
