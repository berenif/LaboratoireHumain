#!/usr/bin/env python3
"""Separately admitted offline mean-head trust experiment; no default execution."""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,atomic_json,sha256
from stabilization_ledger import StageBudget
from slow_benchmark import fingerprints as prior_inputs
from slow_learning import SlowReplay,learner_state
from slow_task import SourceSLOWWorld,SourceSLOWEnv
from slow_trust import initialize,gaussian,proposal,drift,frozen_equal,ALPHA
from training import critic_only_update

PLAN_SHA='dcbd484f57aa51457a3145ec5c39eb10b7cd32326168da37c99d27c0be48b3e7'
OLD_LEDGER_SHA='9486c51f47997a870097c4344bab2a973572bc41a7e840a26f87c94202c0af2d'
ALPHA_SHA='839dd6a95e3a17db160eb06611ba22254e3d9d17eb34f326ac0260e49c189465'

def fingerprints(stage,previous,source):
    files=prior_inputs(previous,source)
    for name in ['slow_trust.py','slow_trust_run.py','test_slow_trust.py','slow_learning.py','test_stabilization_ledger.py']:
        p=Path(__file__).with_name(name);files[str(p.resolve())]=sha256(p)
    paths=[ROOT/'docs/experiments/getup-python-step-plan.md',previous/'learner/checkpoint-002.pt',previous/'learner/pinned-student-log-alpha-local.pt',previous/'benchmark/summary.json',previous/'benchmark-preflight.json',stage/'fixtures.json']
    for p in paths:files[str(p.resolve())]=sha256(p)
    return files

def probes(source,core):
    import torch
    rows=[json.loads(x)for x in(source/'slow-continuation/frames.ndjson').read_text().splitlines()]
    return tuple(torch.tensor(np.array([core.observation(rows[i]['physicalObservation'])for i in ids]),dtype=torch.float32)for ids in [range(2400,6401,20),range(2410,6400,20)])

def prepare(stage,previous,source):
    import torch
    if sha256(ROOT/'docs/experiments/getup-python-step-plan.md')!=PLAN_SHA:raise ValueError('reviewed plan changed')
    for receipt in [json.loads((previous/'benchmark/summary.json').read_text()),json.loads((previous/'benchmark-preflight.json').read_text())]:
        for path,digest in receipt['inputHashes'].items():
            if Path(path).name=='stabilization_ledger.py':
                if digest!=OLD_LEDGER_SHA:raise ValueError('unexpected old resource helper')
            elif sha256(path)!=digest:raise ValueError('original20-sample interface proof is stale')
    bench=json.loads((previous/'benchmark/summary.json').read_text())
    if not bench['passed']or not bench['fullCache20SampleContinuationBitExact']:raise ValueError('prior exact physical proof absent')
    tests=json.loads((stage/'fixtures.json').read_text())
    if not tests['passed']or sha256(stage/'fixtures.log')!=tests['logSHA256']:raise ValueError('fixtures not bound')
    alpha_file=previous/'learner/pinned-student-log-alpha-local.pt'
    if sha256(alpha_file)!=ALPHA_SHA or abs(float(torch.load(alpha_file,weights_only=True,map_location='cpu').exp())-ALPHA)>1e-16:raise ValueError('pinned original temperature mismatch')
    cp=torch.load(previous/'learner/checkpoint-002.pt',weights_only=True,map_location='cpu');replay=SlowReplay.restore(cp['replay'])
    if len(replay.items)!=256 or sum(x['physicalFailure']is not None for x in replay.metadata)!=2:raise ValueError('wrong fixed replay')
    with StageBudget(stage/'ledger.json')as b:
        if b.profile!='slow-offline-trust'or b.data['planSHA256']!=PLAN_SHA or b.data['optimizerCalls']or b.data['sourceIntegrations']:raise ValueError('fresh trust stage required')
        if b.remaining_wall()<=300:raise TimeoutError('offline validation reserve exhausted')
    world=SourceSLOWWorld(source)
    try:
        blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(blob)
        core=SourceSLOWEnv(world,json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()),bindings={'preflight':'trust-zero-step'},rng=np.random.default_rng(0)).core
        a,o=initialize();x=torch.tensor(core.observation(world.observation()))[None,:]
        if world.checkpoint()!=blob or not torch.equal(a.actor.trunk(x),o.trunk(x)):raise ValueError('original full distribution/cache zero-step mismatch')
    finally:world.close()
    atomic_json(stage/'admission.json',dict(schema=1,planSHA256=PLAN_SHA,stageStartedUtc=json.loads((stage/'ledger.json').read_text())['startedUtc'],stageDeadlineUtc=json.loads((stage/'ledger.json').read_text())['deadlineUtc'],originalLearnedStdFrozen=True,originalAlphaFixed=ALPHA,replayCount=256,maximumOptimizerCalls=110,maximumProposals=5,maximumPairedIntervals=640,maximumNativeIntegrations=12800,validationWallReserved=300,deterministicPhysicalPair=True,old20SampleProofRecheck='hash-only; new resource profile tested separately',measuredRate=bench['endToEndPolicyIntervalsPerSecond'],physicsPerStep=bench['physicsSeconds']/bench['allBranchNativeIntegrations'],inputHashes=fingerprints(stage,previous,source)))
    print('admission SHA256',sha256(stage/'admission.json'))

def run(stage,previous,source,expected):
    import torch
    if not expected or sha256(stage/'admission.json')!=expected:raise ValueError('external literal admission hash required')
    admit=json.loads((stage/'admission.json').read_text())
    for path,digest in admit['inputHashes'].items():
        if sha256(path)!=digest:raise ValueError('admitted input changed')
    out=stage/'result';out.mkdir(exist_ok=False);started=time.monotonic();history=[];validation=[];stop=None;world=None;candidate=None
    with StageBudget(stage/'ledger.json')as budget:
        if budget.profile!='slow-offline-trust'or budget.data['optimizerCalls']or budget.data.get('pendingOptimizer')or budget.data['sourceIntegrations']or budget.data['deadlineUtc']!=admit['stageDeadlineUtc']or budget.data['startedUtc']!=admit['stageStartedUtc']or budget.data['planSHA256']!=admit['planSHA256']:raise ValueError('fresh stage-only admission required')
        (stage/'started-once').open('x').close()
        def wall():
            if budget.remaining_wall()<=300 or budget.data.get('requestOutstanding'):raise TimeoutError('offline work must protect300s validation wall')
        def charge(kind):
            wall()
            if budget.data.get('pendingOptimizer')or budget.data['optimizerCalls']>=110:raise ValueError('optimizer pending or limit')
            budget.data['optimizerCalls']+=1;budget.data['pendingOptimizer']=kind;atomic_json(stage/'ledger.json',budget.data)
        def complete(kind):
            if budget.data['pendingOptimizer']!=kind:raise ValueError('optimizer completion mismatch')
            budget.data['pendingOptimizer']=None;atomic_json(stage/'ledger.json',budget.data);wall()
        try:
            wall();agent,original=initialize();cp=torch.load(previous/'learner/checkpoint-002.pt',weights_only=True,map_location='cpu');replay=SlowReplay.restore(cp['replay'])
            # New fixed replay sampling stream, not the old candidate's advanced
            # critic optimizer/RNG. All original transition tensors remain intact.
            replay.rng=np.random.default_rng(0)
            witness=torch.cat([cp['replay']['columns'][0],cp['replay']['columns'][2]])
            with torch.no_grad():anchor_mu,anchor_ls=gaussian(original,witness);anchor_std=anchor_ls.exp()
            anchor=copy.deepcopy(original.state_dict());world=SourceSLOWWorld(source);blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(blob);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());temp=SourceSLOWEnv(world,model,bindings={'runtimeAdmission':expected},rng=np.random.default_rng(0));guard,midpoints=probes(source,temp.core)
            torch.save(dict(states=witness,anchorMu=anchor_mu,anchorStd=anchor_std,globalGuard=guard,midpointAudit=midpoints),out/'witness-local.pt')
            def save(label):
                wall();p=out/f"checkpoint-{budget.data['proposalAttempts']:03d}-{label}.pt";torch.save(dict(schema=1,label=label,learner=learner_state(agent,replay),originalActor=anchor,witnessSHA256=sha256(out/'witness-local.pt'),admissionSHA256=expected,proposalAttempts=budget.data['proposalAttempts'],acceptedSteps=budget.data['acceptedSteps']),p);atomic_json(out/'latest-checkpoint.json',dict(path=str(p),sha256=sha256(p)));wall()
            for _ in range(100):
                wall();row=critic_only_update(agent,replay,128,before_optimizer_step=lambda:charge('warmup-critic'));complete('warmup-critic');history.append(dict(phase='warmup',**row))
            frozen_equal(agent.actor,anchor);save('warmup-complete')
            for attempt in range(5):
                wall();batch=replay.sample(128)
                class Batch:
                    def sample(self,n):return batch
                loss=critic_only_update(agent,Batch(),128,before_optimizer_step=lambda:charge('proposal-critic'));complete('proposal-critic')
                budget.data['proposalAttempts']+=1;atomic_json(stage/'ledger.json',budget.data)
                pre=out/f'attempt-{attempt+1:03d}-postcritic-preactor.pt';wall();torch.save(dict(learner=learner_state(agent,replay),batch=batch,witnessSHA256=sha256(out/'witness-local.pt'),admissionSHA256=expected),pre);wall()
                result=proposal(agent,batch[0],witness,anchor_mu,anchor_std,anchor,charge=lambda:charge('meanhead-proposal'),complete=lambda:complete('meanhead-proposal'),check_wall=wall)
                result.update(attempt=attempt+1,criticLoss=loss['criticLoss'],preActorCheckpointSHA256=sha256(pre))
                if result['accepted']:
                    budget.data['acceptedSteps']+=1;atomic_json(stage/'ledger.json',budget.data);result['globalGuard']=drift(agent.actor,original,guard)
                history.append(dict(phase='proposal',**result));save('proposal-boundary')
                if not result['accepted']:break
                if result['globalGuard']['rms']>.02 or result['globalGuard']['maximum']>.05:stop='unchanged-global-drift-veto';break
            if stop is None:
                midpoint=drift(agent.actor,original,midpoints);atomic_json(out/'midpoint-audit.json',midpoint)
                if midpoint['rms']>.02 or midpoint['maximum']>.05:stop='midpoint-drift-veto'
                elif not budget.data['acceptedSteps']:stop='no-accepted-proposal'
            if stop is None:
                # Re-admit the WHOLE pair live; no repeat physics is added.
                need_wall=640/admit['measuredRate']*1.5;need_physics=12800*admit['physicsPerStep']*1.5
                if budget.remaining_wall()<max(300,need_wall)+.1 or 3-budget.data['physicsSeconds']<need_physics or 640-budget.data['totalPolicyIntervals']<640 or 12800-budget.data['sourceIntegrations']<12800:raise TimeoutError('whole deterministic pair cannot fit')
                for label,policy in [('frozen-original',original),('candidate',agent.actor)]:
                    world.restore(blob);env=SourceSLOWEnv(world,model,bindings={'runtimeAdmission':expected},rng=np.random.default_rng(0));records=[]
                    for command in range(320):
                        with torch.no_grad():action=policy(torch.tensor(env.core.observation(world.observation()))[None,:],False,False)[0][0].numpy()
                        budget.begin_interval('validation');t=env.measured_interval(action,lambda w:budget.integrate('validation',lambda maximum:w.lib.lh_step_audited(w.h,maximum),lambda:w.dim(6)))
                        records.append({k:v for k,v in t.items()if k not in['actualFrames','perNativeControls']})
                        with(out/(label+'-frames.ndjson')).open('a')as stream:
                            end=budget.data['sourceIntegrations']
                            for i,row in enumerate(t['actualFrames']):stream.write(json.dumps(dict(globalIntegration=end-len(t['actualFrames'])+i+1,command=command,packet=row),allow_nan=False)+'\n')
                        if t['info']['terminated']or t['info']['truncated']:break
                    cache,side=env.checkpoint();(out/(label+'-final.bin')).write_bytes(cache);atomic_json(out/(label+'-controller.json'),side);atomic_json(out/(label+'-commands.json'),records)
                    validation.append(dict(branch=label,intervals=len(records),final=records[-1]['info'],qualified=bool(records[-1]['info']['success']and records[-1]['info']['quietConsecutiveSamples']>=4000)))
                stop='completed-fixed-pair'
        except BaseException as e:stop=type(e).__name__+': '+str(e)
        finally:
            if world is not None:world.close()
            atomic_json(out/'optimization.json',history);atomic_json(out/'validation.json',validation)
            qualified=bool(len(validation)==2 and validation[-1]['qualified']);report=dict(stopped=stop,optimizerCalls=budget.data['optimizerCalls'],proposalAttempts=budget.data['proposalAttempts'],acceptedSteps=budget.data['acceptedSteps'],physicsSeconds=budget.data['physicsSeconds'],actualPolicyIntervals=budget.data['totalPolicyIntervals'],actualNativeIntegrations=budget.data['sourceIntegrations'],runWallSeconds=time.monotonic()-started,quietStabilizationQualified=qualified,forwardStepQualified=False,selectedPolicy='candidate'if qualified else'immutable-source-SLOW-baseline',fixedOriginalAlpha=ALPHA,originalStateDependentStdFrozen=True,deterministicPair=True,admissionSHA256=expected)
            atomic_json(out/'summary.json',report);budget.data.update(status='closed-offline-trust',unusedAllowanceNotTransferred=True);atomic_json(stage/'ledger.json',budget.data);print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--previous',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--expected-admission-sha',default='');a=p.parse_args();prepare(a.stage,a.previous,a.source)if a.mode=='prepare'else run(a.stage,a.previous,a.source,a.expected_admission_sha)
