#!/usr/bin/env python3
"""Separate admitted zero-correction analytic-actor diagnostic. No learning.

Uses a fresh1s/10min ledger, not the closed failed-fit allowance. Source SLOW
reference semantics remain inside frozen base, and200Hz engine holds torque for
25ms. First-command full-cache repeat is charged and explicitly branch-tagged.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from analytic_actor import AnalyticSLOWHoldingActor
from native_env import ROOT,sha256,atomic_json
from source_task import SourceStabilization,CONFIG_PATH
from source_task_env import SourceTaskWorld
from stabilization_ledger import StageBudget
DIAGNOSTIC_CONFIG=Path(__file__).with_name('analytic-diagnostic-config-v1.json')


def fingerprints(stage,source,entry):
    paths=[Path(__file__),DIAGNOSTIC_CONFIG,CONFIG_PATH,ROOT/'scripts/getup-step/analytic_actor.py',ROOT/'scripts/getup-step/stabilization_ledger.py',ROOT/'scripts/getup-step/source_task.py',ROOT/'scripts/getup-step/source_task_env.py',ROOT/'scripts/getup-step/source_slow_control.py',ROOT/'scripts/getup-step/source_positive_control.py',ROOT/'scripts/getup-step/source_engine.py',ROOT/'scripts/getup-step/stabilization_init.py',ROOT/'scripts/getup-step/training.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-step/source_math.py',ROOT/'scripts/getup-step/actors.py',ROOT/'scripts/getup-step/reference.py',ROOT/'scripts/getup-step/task.py',ROOT/'scripts/getup-step/audit_metrics.py',ROOT/'scripts/getup-step/native_env.py',ROOT/'scripts/getup-step-reference.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',source/'source_bridge_v2.so',source/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',source/'source-model/humanoid_static.xml',entry/'source200-entry-preflight.json',entry/'source200-entry-cache-local.bin']
    return{str(p.resolve()):sha256(p)for p in paths}


def prepare(stage,source,entry):
    import torch
    ledger=json.loads((stage/'ledger.json').read_text());cfg=json.loads(DIAGNOSTIC_CONFIG.read_text());prior=json.loads((entry/'source200-entry-preflight.json').read_text())
    if ledger['profile']!='analytic-diagnostic'or ledger['physicsSeconds']or ledger['sourceIntegrations']or not prior['passed']:raise ValueError('not fresh diagnostic admission')
    model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());w=SourceTaskWorld(source)
    try:
        w.restore((entry/'source200-entry-cache-local.bin').read_bytes());row=w.task_packet()
        if row!=prior['entryPacket']:raise ValueError('analytic diagnostic entry state mismatch')
        core=SourceStabilization(row,model);actor=AnalyticSLOWHoldingActor();obs=torch.tensor(core.observation(w.observation()))[None,:]
        with torch.inference_mode():mu,_,_,std=actor(obs,compute_pi=False,compute_log_pi=False);base=actor.base_torque(obs)
        report=dict(passed=True,physicsIntegrations=0,optimizerUpdates=0,stageDeadlineUtc=ledger['deadlineUtc'],maximumAllBranchCommands=64,maximumAllBranchSourceSteps=320,maximumAllBranchSimSeconds=1.6,retainedBranchSimSeconds=1.575,maximumPhysicsSeconds=1.,maximumTotalStageWallSeconds=600,maximumExecutionWallSeconds=120,sourceDtS=.005,firstCommandRepeatCharged=True,initialMeanVsBaseMaximumError=float(torch.max(abs(mu-base))),latentStdMaximumError=float(torch.max(abs(std.exp()-.02))),firstAction=mu[0].numpy().tolist(),failedFitWeightsLoaded=False,sourceCacheCopiedAcrossTimesteps=False,initialStrictQuietQualified=False,source200EntryZeroStepEqual=True,configSHA256=sha256(DIAGNOSTIC_CONFIG),inputHashes=fingerprints(stage,source,entry))
        atomic_json(stage/'preflight.json',report);print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','firstAction']},indent=2))
    finally:w.close()


def run(stage,source,entry):
    import torch
    pre=json.loads((stage/'preflight.json').read_text())
    for p,h in pre['inputHashes'].items():
        if sha256(p)!=h:raise ValueError('analytic diagnostic fingerprint changed')
    if not pre['passed']or pre['failedFitWeightsLoaded']:raise ValueError('diagnostic preflight failed')
    started=time.monotonic();world=None;core=None;records=[];commands=[];pair_first=None;pair_equal=False;stop=None;max_base_error=0.;saturation_count=0;rng=np.random.default_rng(0)
    with StageBudget(stage/'ledger.json')as budget:
        if budget.profile!='analytic-diagnostic'or budget.data['sourceIntegrations']or budget.data.get('optimizerUpdates',0):raise ValueError('not fresh analytic-only ledger')
        (stage/'started-once').open('x').close();budget.start_benchmark_clock();budget.allowance('benchmark')
        try:
            # Local120s execution clock includes actor/world setup and all C calls.
            actor=AnalyticSLOWHoldingActor();world=SourceTaskWorld(source);world.restore((entry/'source200-entry-cache-local.bin').read_bytes());initial=world.task_packet();model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());core=SourceStabilization(initial,model);cache=world.checkpoint();bindings={'preflightSHA256':sha256(stage/'preflight.json')};side=core.checkpoint(cache,bindings,rng.bit_generator.state)
            for command in range(64):
                branch='repeatA'if command==0 else'repeatB-continuation'
                if command==1:
                    world.restore(cache);core=SourceStabilization(initial,model);rng.bit_generator.state=core.restore(side,cache,world.task_packet(),expected_bindings=bindings)
                obs=core.observation(world.observation());tensor=torch.tensor(obs)[None,:]
                with torch.inference_mode():mean,_,_,_=actor(tensor,compute_pi=False,compute_log_pi=False);base=actor.base_torque(tensor)
                u=mean[0].numpy().copy();max_base_error=max(max_base_error,float(torch.max(abs(mean-base))));saturation_count+=int((abs(base)>=1-1e-6).sum());budget.begin_interval('benchmark');world.arr(2,(21,))[:]=u;packet_rows=[]
                for hold in range(5):
                    budget.integrate('benchmark',lambda maximum:world.lib.lh_step_audited(world.h,maximum),lambda:world.dim(6))
                    row=world.task_packet();packet_rows.append(row);records.append(dict(globalIntegration=budget.data['sourceIntegrations'],command=command,hold=hold,branch=branch,packet=row))
                    if core.terminal_in_prefix(packet_rows):break
                result=core.consume(packet_rows,expected_teacher_controls=u);commands.append(dict(command=command,branch=branch,state118=obs.tolist(),normalizedActuatorTorque=u.tolist(),info=result))
                if command==0:pair_first=copy.deepcopy(packet_rows)
                if command==1:
                    pair_equal=packet_rows==pair_first
                    if not pair_equal:stop='full-cache-continued-physical-parity-failed';break
                if result['terminated']or result['truncated']:stop='actual-terminal-'+str(result['physicalFailure']);break
            stop=stop or'completed64-interval-analytic-diagnostic'
        except BaseException as error:stop=f'{type(error).__name__}: {error}'
        finally:
            if world is not None:
                blob=world.checkpoint();(stage/'final-source-cache-local.bin').write_bytes(blob)
                try:
                    if core is None or world.dim(6)!=core.native_id or core.packet_hash(world.task_packet())!=core.last_packet_hash:raise ValueError('native/controller boundary mismatch; engine cache is not a resumable hybrid checkpoint')
                    atomic_json(stage/'final-controller.json',core.checkpoint(blob,bindings,rng.bit_generator.state))
                except BaseException as error:atomic_json(stage/'final-controller-invalid.json',dict(error=str(error)))
            atomic_json(stage/'commands.json',commands)
            with(stage/'frames.ndjson').open('w')as stream:
                for row in records:stream.write(json.dumps(row,allow_nan=False)+'\n')
            report=dict(stopped=stop,policyIntervalsCharged=budget.data['totalPolicyIntervals'],allBranchSourceIntegrations=budget.data['sourceIntegrations'],aggregatePhysicsSeconds=budget.data['physicsSeconds'],processWallSeconds=time.monotonic()-started,fullCacheContinuedPhysicalBitExact=pair_equal,maximumInitialMeanVsAnalyticBaseError=max_base_error,baseSaturationCount=saturation_count,optimizerUpdates=0,taskCollectionEnabled=False,sourceSACEnabled=False,failedFitWeightsLoaded=False,diagnosticCompleted=stop=='completed64-interval-analytic-diagnostic'and pair_equal,final=None if not commands else commands[-1]['info'],fiveSecondQuietQualified=False,forwardStepQualified=False,inputHashes=pre['inputHashes']);atomic_json(stage/'summary.json',report);budget.data.update(status='closed-analytic-diagnostic',diagnosticCompleted=report['diagnosticCompleted'],unusedBudgetNotTransferred=True);atomic_json(stage/'ledger.json',budget.data)
            if world is not None:world.close()
            print(json.dumps({k:v for k,v in report.items()if k not in['inputHashes','final']},indent=2))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--entry',type=Path,required=True);a=p.parse_args()
    if a.mode=='prepare':prepare(a.stage,a.source,a.entry)
    else:run(a.stage,a.source,a.entry)
if __name__=='__main__':main()
