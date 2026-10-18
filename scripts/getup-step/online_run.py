#!/usr/bin/env python3
"""Explicitly admitted online source-SLOW experiment; no default execution.

Source residual radians and native800Hz PD are preserved. All engine/optimizer
calls are counted, including rejected proposals, resets and repeated branches.
"""
import argparse,copy,hashlib,json,os,time
from pathlib import Path
import numpy as np
import torch
from native_env import ROOT,atomic_json,sha256
from slow_task import SourceSLOWWorld,SourceSLOWEnv,CONFIG_SHA
from stabilization_ledger import StageBudget
from online_io import Guard,rolling_checkpoint
from online_learning import (PROTOCOL,PROTOCOL_SHA,ALPHA,initialize,Roles,OnlineReplay,
 sample_action,propose,Features,divergence,episode_returns,trunk_sha,tensor_sha)
from training import critic_only_update
from online_cadence import CADENCE,CADENCE_SHA,compatible,io_timings,save_complete,periodic_checkpoint_due,candidate_identity

PLAN=ROOT/'docs/experiments/getup-python-step-plan.md'
PLAN_SHA='47324409e7b21e2c01ca941f2e0871f19e0afe09a41008d1de608760a67b1554'

def inputs(stage,source):
    files=list(Path(__file__).parent.glob('*.py'))+[PROTOCOL,CADENCE,PLAN,ROOT/'scripts/getup-step-reference.py',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'scripts/getup-step/source_bridge.c',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',source/'source_bridge_v2.so',source/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',*sorted((source/'source-model').glob('*.xml')),source/'slow-continuation/final.bin',source/'slow-continuation/final.controller.json',source/'slow-continuation/frames.ndjson',stage/'fixtures.json']
    for name in ['cadence-fixtures.json','cadence-benchmark-migration.json','cadence-io-timing.json','fresh-candidate-window.json']:
        if (stage/name).exists():files.append(stage/name)
    return {str(p.resolve()):sha256(p)for p in files}

def verify(receipt):
    for p,digest in receipt['inputHashes'].items():
        if sha256(p)!=digest:raise ValueError('stale runtime input: '+p)

def identity(budget,receipt):
    d=budget.data
    if budget.profile!='slow-online'or d['startedUtc']!=receipt['stageStartedUtc']or d['deadlineUtc']!=receipt['stageDeadlineUtc']or d['planSHA256']!=PLAN_SHA or d['protocolSHA256']!=PROTOCOL_SHA:raise ValueError('fresh stage identity mismatch')
    if sha256(PLAN)!=PLAN_SHA or sha256(PROTOCOL)!=PROTOCOL_SHA or d.get('pendingOptimizer')or d.get('requestOutstanding')or d.get('proposalInProgress'):raise ValueError('plan/protocol or pending request mismatch')

def setup(source,bindings):
    world=SourceSLOWWorld(source);blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(blob);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());env=SourceSLOWEnv(world,model,bindings=bindings,rng=np.random.default_rng(0));return world,env,blob,model

def probes(source,core,mid=False):
    rows=[json.loads(x)for x in(source/'slow-continuation/frames.ndjson').read_text().splitlines()]
    ids=range(2410,6400,20)if mid else range(2400,6401,20)
    return torch.tensor(np.array([core.observation(rows[i]['physicalObservation'])for i in ids]),dtype=torch.float32)

def audit(policy,original,x):
    with torch.no_grad():
        a=policy(x,False,False)[0];b=original(x,False,False)[0];delta=a-b
        f=Features(policy,x,trunk_sha(original));mu,ls=f.distribution(policy);m0,l0=f.distribution(original)
        return dict(rms=float(torch.sqrt(torch.mean(delta**2))),maximum=float(delta.abs().max()),maximumCumulativeFullKL=float(divergence(m0,l0,mu,ls).max()),latentStd=ls.exp().tolist())

def audit_ok(a):return a['rms']<=.02 and a['maximum']<=.05

def transition_record(t,**extra):return dict({k:v for k,v in t.items()if k not in['actualFrames','perNativeControls']},**extra)

def write_rows(stream,t,budget,**extra):
    count=len(t['actualFrames']);end=budget.data['sourceIntegrations']
    for i,(packet,control)in enumerate(zip(t['actualFrames'],t['perNativeControls'])):stream.write(json.dumps(dict(globalIntegration=end-count+i+1,globalPolicyInterval=budget.data['totalPolicyIntervals'],packet=packet,sourceControlAudit=control,**extra),allow_nan=False)+'\n')
    if t['info']['terminated']or t['info']['truncated']:stream.flush();os.fsync(stream.fileno())

def interval(env,action,budget,phase):
    budget.begin_interval(phase)
    return env.measured_interval(action,lambda w:budget.integrate(phase,lambda maximum:w.lib.lh_step_audited(w.h,maximum),lambda:w.dim(6)))

def valid_physical_terminal(t):
    info=t['info']
    if info['maximumPerNativeTorqueErrorNm']>1e-5:raise ValueError('motor mapping failure')
    if any(row['sourceWarnings']for row in t['actualFrames']):raise ValueError('source engine warning')
    # Only finite, decoded penetration-only native_integrity may be reset.
    if info['physicalFailure']=='native_integrity'and not any(row['maximumSelfPenetrationM']>.005 for row in t['actualFrames']):raise ValueError('unclassified native integrity failure')

def prepare_benchmark(stage,source):
    receipt=json.loads((stage/'fixtures.json').read_text())
    if not receipt['passed']or sha256(stage/'fixtures.log')!=receipt['logSHA256']:raise ValueError('fixture receipt missing/stale')
    world=None
    with StageBudget(stage/'ledger.json')as budget:
        guard=Guard(budget,stage);guard.storage(admission=True)
        if budget.data['optimizerCalls']or budget.data['sourceIntegrations']or budget.data['totalPolicyIntervals']:raise ValueError('fresh benchmark required')
        try:
            agent,original=initialize();world,env,blob,_=setup(source,{'stage':'zero-step'});packet=world.task_packet();last=json.loads((source/'slow-continuation/frames.ndjson').read_text().splitlines()[-1]);x=torch.tensor(env.core.observation(world.observation()))[None,:]
            with torch.no_grad():raw=agent.actor.trunk(x);raw0=original.trunk(x)
            if not torch.equal(raw,raw0)or world.checkpoint()!=blob or any(packet[k]!=last[k]for k in['qpos','qvel','physicalObservation','comSource','qfrcActuator']):raise ValueError('original full-distribution/cache zero-step parity failed')
            pre=dict(passed=True,kind='original full SLOW distribution and same800Hz cache',stageStartedUtc=budget.data['startedUtc'],stageDeadlineUtc=budget.data['deadlineUtc'],planSHA256=PLAN_SHA,protocolSHA256=PROTOCOL_SHA,sourceInitialPacket=packet,sourceInitialCounter=world.dim(6),originalFull42Batch1BitExact=True,sourceAlpha=ALPHA,maximumBenchmarkIntervals=64,maximumBenchmarkNativeSteps=1280,maximumBenchmarkOptimizerCalls=18,fixtureReceiptSHA256=sha256(stage/'fixtures.json'),inputHashes=inputs(stage,source))
            atomic_json(stage/'benchmark-preflight.json',pre);print('benchmark preflight SHA256',sha256(stage/'benchmark-preflight.json'))
        finally:
            if world:world.close()

def benchmark(stage,source,expected):
    if sha256(stage/'benchmark-preflight.json')!=expected:raise ValueError('literal external benchmark preflight SHA required')
    pre=json.loads((stage/'benchmark-preflight.json').read_text());verify(pre);out=stage/'benchmark';out.mkdir(exist_ok=False);world=None;stop=None;timings={};pair=False;results=[];started=time.monotonic()
    with StageBudget(stage/'ledger.json')as budget:
        identity(budget,pre)
        if budget.data['sourceIntegrations']or budget.data['totalPolicyIntervals']or budget.data['optimizerCalls']:raise ValueError('benchmark requires untouched fresh ledger')
        (stage/'benchmark-started-once').open('x').close();budget.start_benchmark_clock();guard=Guard(budget,stage);guard.check();guard.storage(admission=True)
        try:
            agent,original=initialize();roles=Roles('benchmark');replay=OnlineReplay();replay.rng=roles.replay;world,env,blob,model=setup(source,{'benchmarkPreflightSHA256':expected});x=probes(source,env.core);initial_blob,initial_side=env.checkpoint();first=None
            if world.task_packet()!=pre['sourceInitialPacket']:raise ValueError('benchmark entry changed')
            begin=time.monotonic()
            with(out/'frames.ndjson').open('x')as stream:
                for command in range(64):
                    if command==1:env.restore(initial_blob,initial_side)
                    obs=torch.tensor(env.core.observation(world.observation()))[None,:]
                    with torch.no_grad():action=agent.actor(obs,False,False)[0][0].numpy()
                    t=interval(env,action,budget,'benchmark');write_rows(stream,t,budget,command=command,branch='repeatA'if command==0 else'retainedB');results.append(transition_record(t));valid_physical_terminal(t)
                    if command==0:first=copy.deepcopy(t)
                    if command==1:
                        pair=t==first
                        if not pair:raise ValueError('20packet full transition repeat mismatch')
                    # Benchmark replay is disposable; label deterministic actions
                    # with a genuine source latent/noise0 witness for validation.
                    from slow_trust import gaussian
                    from training import source_module
                    with torch.no_grad():mu,ls=gaussian(agent.actor,obs);noise=torch.zeros_like(mu);src=source_module();lp=src.gaussian_logprob(noise,ls);_,a,lp=src.squash(mu,mu,lp,obs[:,-1])
                    t['behaviour']=dict(policyVersion=0,rawMean=mu[0].tolist(),logstd=ls[0].tolist(),noise=noise[0].tolist(),sourceEpsilonLogProbability=float(lp[0,0]));replay.add(budget.data['totalPolicyIntervals'],t)
                    if t['info']['terminated']or t['info']['truncated']:raise ValueError('benchmark physical terminal: '+str(t['info']['physicalFailure']))
                stream.flush();os.fsync(stream.fileno())
            timings['actual64IntervalLoopWallSeconds']=time.monotonic()-begin
            critic_times=[]
            for _ in range(16):
                begin=time.monotonic()
                with roles.critic_scope():critic_only_update(agent,replay,128,before_optimizer_step=lambda:guard.charge('benchmark-critic'))
                guard.complete('benchmark-critic');critic_times.append(time.monotonic()-begin)
            timings['worstCriticUpdateWallSeconds']=max(critic_times)
            states=torch.tensor(np.array([v for row in replay.items for v in[row[0],row[2]]]),dtype=torch.float32);witness=states.repeat((2560+len(states)-1)//len(states),1)[:2560];proposal_times=[];receipts=[]
            for attempt in[1,2]:
                batch=replay.sample(128);begin=time.monotonic();p=propose(agent,original,batch[0],witness,roles.proposal(attempt),charge=lambda:guard.charge('benchmark-head'),complete=lambda:guard.complete('benchmark-head'),check=guard.check,minimum_decrease=1e6);p.pop('_evidence');a=audit(agent.actor,original,x);receipts.append(p)
                if p['accepted']or len(p['trials'])!=11 or not audit_ok(a):raise ValueError('timing proposal must cover all11scales and reject')
                guard.commit_proposal('benchmark-head');proposal_times.append(time.monotonic()-begin)
            timings['worstFull2560Witness11ScaleProposalWallSeconds']=max(proposal_times)
            begin=time.monotonic();rolling_checkpoint(out,guard,env,agent,replay,roles,'TIMING-ONLY-DISCARDED',timing_max_rows=5000);timings['worstFullCheckpointWallSeconds']=time.monotonic()-begin
            begin=time.monotonic();world.restore(blob);env=SourceSLOWEnv(world,model,bindings={'benchmarkPreflightSHA256':expected},rng=np.random.default_rng(0));timings['actualCacheAndControllerResetWallSeconds']=time.monotonic()-begin;guard.check()
            atomic_json(out/'proposal-timing-receipts.json',receipts);stop='completed64-and18-discarded-calls'
        except BaseException as error:stop=type(error).__name__+': '+str(error)
        finally:
            boundary_error=None
            try:
                if world is not None:
                    # Read-only terminal proof; no integration or cache repair.
                    guard.check();atomic_json(out/'last-native-packet.json',world.task_packet());guard.bytes_save(out/'last-native-cache-local.bin',world.checkpoint())
                    try:
                        _,side=env.checkpoint();atomic_json(out/'last-controller.json',side)
                    except BaseException as error:atomic_json(out/'last-controller-invalid.json',dict(error=str(error)))
            except BaseException as error:boundary_error=str(error)
            finally:
                if world is not None:world.close()
            passed=stop=='completed64-and18-discarded-calls'and pair and boundary_error is None
            report=dict(passed=passed,stopped=stop,lastBoundarySaveError=boundary_error,fullCache20SampleContinuationBitExact=pair,timings=timings,maximumPerNativeTorqueErrorNm=max([t['info']['maximumPerNativeTorqueErrorNm']for t in results]or[0]),allBranchNativeIntegrations=budget.data['sourceIntegrations'],allBranchPolicyIntervals=budget.data['totalPolicyIntervals'],physicsSeconds=budget.data['stagePhysicsSeconds']['benchmark'],processWallSeconds=time.monotonic()-started,allBenchmarkLearningAndRNGDiscarded=True,benchmarkOptimizerCalls=budget.data['benchmarkOptimizerCalls'],inputHashes=pre['inputHashes']);atomic_json(out/'summary.json',report);atomic_json(out/'commands.json',results);budget.data.update(status='benchmark-passed'if passed else'benchmark-failed',onlineBenchmarkPassed=passed);atomic_json(budget.path,budget.data)
            print(json.dumps({k:v for k,v in report.items()if k!='inputHashes'},indent=2))

def projection(bench,intervals,proposals,warmups,*,pair=True):
    t=bench['timings'];loop=t['actual64IntervalLoopWallSeconds']/64;reset=t['actualCacheAndControllerResetWallSeconds'];save=t['worstFullCheckpointWallSeconds'];critic=t['worstCriticUpdateWallSeconds'];proposal=t['worstFull2560Witness11ScaleProposalWallSeconds']
    # Every interval may terminate: count its actual reset/full snapshot cost.
    # Proposals additionally save full state and compact receipts (priced by
    # another full-save bound). Raw journals/fsync are inside measured loop.
    io=bench.get('cadenceIOTimings',dict(worstThreeJournalTerminalFsyncWallSeconds=save,worstCompactReceiptWallSeconds=save));full_saves=(intervals+63)//64+9
    training=1.5*(intervals*(loop+reset+io['worstThreeJournalTerminalFsyncWallSeconds'])+warmups*critic+proposals*(critic+proposal+io['worstCompactReceiptWallSeconds'])+full_saves*save)
    validation=1.5*(640*loop+2*(reset+save))if pair else 0.
    physics_per=bench['physicsSeconds']/bench['allBranchNativeIntegrations']
    return dict(trainingWall=training,validationWall=validation,trainingPhysics=1.5*intervals*20*physics_per,validationPhysics=1.5*640*20*physics_per)

def capacity(budget,guard,bench,intervals,proposals,warmups):
    guard.check();guard.storage(admission=True);p=projection(bench,intervals,proposals,warmups);d=budget.data
    if budget.remaining_wall()<300+p['trainingWall']or p['validationWall']>300 or p['trainingPhysics']>15-d['stagePhysicsSeconds']['training']or p['validationPhysics']>4-d['stagePhysicsSeconds']['validation']or 5704-d['totalPolicyIntervals']<intervals+640 or 114080-d['sourceIntegrations']<(intervals+640)*20:raise TimeoutError('full online path+whole pair cannot fit live reserves: '+str(p))
    return p

def prepare_candidate(stage,source):
    pre=json.loads((stage/'benchmark-preflight.json').read_text());bench=json.loads((stage/'benchmark/summary.json').read_text());compatible(stage,[pre,bench],Path(__file__))
    fixture=json.loads((stage/'cadence-fixtures.json').read_text())
    if not fixture['passed']or sha256(stage/'cadence-fixtures.log')!=fixture['logSHA256']:raise ValueError('cadence fixtures missing/stale')
    if not bench['passed']or not bench['fullCache20SampleContinuationBitExact']or bench['maximumPerNativeTorqueErrorNm']>1e-5 or bench['benchmarkOptimizerCalls']!=18:raise ValueError('benchmark gate failed')
    with StageBudget(stage/'ledger.json')as budget:
        candidate_identity(budget,pre,stage);guard=Guard(budget,stage);guard.phase='training'
        if budget.data['stagePolicyIntervals']['training']or budget.data['candidateOptimizerCalls']:raise ValueError('candidate must start fresh')
        bench['cadenceIOTimings']=io_timings(stage,guard,json.loads((stage/'benchmark/commands.json').read_text())[-1]);p=capacity(budget,guard,bench,5000,994,1000);admission=dict(schema=1,kind='parent admitted stage-only online SLOW over disabled equation preset',stageStartedUtc=budget.data['startedUtc'],stageDeadlineUtc=budget.data['deadlineUtc'],planSHA256=PLAN_SHA,protocolSHA256=PROTOCOL_SHA,cadenceOverlaySHA256=CADENCE_SHA,cadenceIOTimings=bench['cadenceIOTimings'],projection=p,runtimeActualCollectionEnabled=True,maximumActualNewTransitions=5000,maximumCandidateOptimizerCalls=2988,benchmarkSummarySHA256=sha256(stage/'benchmark/summary.json'),inputHashes=inputs(stage,source));atomic_json(stage/'runtime-admission.json',admission);print('runtime admission SHA256',sha256(stage/'runtime-admission.json'));print(json.dumps(p,indent=2))

class FixedBatch:
    def __init__(self,batch):self.batch=batch
    def sample(self,n):
        if n!=len(self.batch[0]):raise ValueError('fixed batch size')
        return self.batch

def candidate(stage,source,expected):
    if sha256(stage/'runtime-admission.json')!=expected:raise ValueError('literal external runtime admission SHA required')
    admission=json.loads((stage/'runtime-admission.json').read_text());verify(admission);bench=json.loads((stage/'benchmark/summary.json').read_text())
    if sha256(stage/'benchmark/summary.json')!=admission['benchmarkSummarySHA256']or sha256(CADENCE)!=admission['cadenceOverlaySHA256']:raise ValueError('benchmark/cadence changed')
    bench['cadenceIOTimings']=admission['cadenceIOTimings']
    out=stage/'learner';out.mkdir(exist_ok=False);world=None;env=None;stop=None;records=[];episode_rows=[];episode_reports=[];updates=[];validation=[];accepted=0;rejections=0;started=time.monotonic();milestones=set()
    with StageBudget(stage/'ledger.json')as budget:
        candidate_identity(budget,admission,stage);guard=Guard(budget,stage);guard.phase='training';capacity(budget,guard,bench,5000,994,1000)
        if budget.data['candidateOptimizerCalls']or budget.data['stagePolicyIntervals']['training']:raise ValueError('fresh candidate only')
        (stage/'candidate-started-once').open('x').close();budget.data.update(taskCollectionEnabled=True,status='online-collection');atomic_json(budget.path,budget.data)
        try:
            agent,original=initialize();roles=Roles();replay=OnlineReplay();replay.rng=roles.replay;world,env,entry_blob,model=setup(source,{'runtimeAdmissionSHA256':expected});entry_sha=hashlib.sha256(entry_blob).hexdigest();x=probes(source,env.core);mid=probes(source,env.core,True);episode=0;initial_witness=None
            save_complete(out,guard,env,agent,replay,roles,'initial')
            with(out/'frames.ndjson').open('x')as frames,(out/'transitions.ndjson').open('x')as commands,(out/'updates.ndjson').open('x')as update_stream,(out/'terminal-proofs.ndjson').open('x')as terminal_stream:
                while len(replay.items)<5000:
                    guard.check()
                    if env.core.terminal:
                        begin=time.monotonic();world.restore(entry_blob);env=SourceSLOWEnv(world,model,bindings={'runtimeAdmissionSHA256':expected},rng=np.random.default_rng(0));episode+=1;episode_rows=[]
                    obs=torch.tensor(env.core.observation(world.observation()),dtype=torch.float32)[None,:];action,behaviour=sample_action(agent.actor,obs,roles,accepted);t=interval(env,action,budget,'training');write_rows(frames,t,budget,episode=episode);valid_physical_terminal(t);t['behaviour']=behaviour;replay.add(budget.data['totalPolicyIntervals'],t)
                    record=transition_record(t,episode=episode,globalPolicyInterval=budget.data['totalPolicyIntervals'],actualTaskDataCollected=True,runtimeAdmissionSHA256=expected);record['info']=copy.deepcopy(record['info']);record['info'].update(measurementOnly=False,replayEligible=True,runtimeActualCollectionEnabled=True);records.append(record);episode_rows.append(record);commands.write(json.dumps(record,allow_nan=False)+'\n')
                    n=len(replay.items);terminal=bool(t['info']['terminated']or t['info']['truncated'])
                    if terminal:
                        commands.flush();os.fsync(commands.fileno());terminal_stream.write(json.dumps(dict(episode=episode,nextResetOrdinal=episode+1,actualResetCacheSHA256=entry_sha,globalPolicyInterval=budget.data['totalPolicyIntervals'],globalSourceIntegration=budget.data['sourceIntegrations'],executedInterval=record),allow_nan=False)+'\n');terminal_stream.flush();os.fsync(terminal_stream.fileno());returns=episode_returns(episode_rows);episode_reports.append(dict(episode=episode,firstReplayIndex=n-len(episode_rows),actualNativeSamples=sum(r['info']['nativeSampleCount']for r in episode_rows),shorterThan100ms=sum(r['info']['nativeSampleCount']for r in episode_rows)<80,final=t['info'],returns=returns))
                    if n==1024:
                        initial_witness=torch.tensor(np.array([v for row in replay.items for v in[row[0],row[2]]]),dtype=torch.float32);fixed=copy.deepcopy(agent.actor.state_dict())
                        for k in range(1000):
                            with roles.critic_scope():loss=critic_only_update(agent,replay,128,before_optimizer_step=lambda:guard.charge('candidate-warmup'))
                            guard.complete('candidate-warmup')
                            if k%100==99:update_stream.write(json.dumps(dict(phase='critic-warmup',update=k+1,**loss))+'\n');update_stream.flush()
                        if any(not torch.equal(v,fixed[k])for k,v in agent.actor.state_dict().items()):raise ValueError('warmup changed frozen entire actor')
                        capacity(budget,guard,bench,3976,994,0)
                    if n>1024 and(n-1024)%4==0:
                        batch=replay.sample(128)
                        with roles.critic_scope():cl=critic_only_update(agent,FixedBatch(batch),128,before_optimizer_step=lambda:guard.charge('candidate-critic'))
                        guard.complete('candidate-critic');attempt=roles.attempt_id+1;generator=roles.proposal(attempt);budget.data['proposalAttempts']=attempt;atomic_json(budget.path,budget.data);recent=torch.tensor(np.array([v for row in replay.items[-256:]for v in[row[0],row[2]]]),dtype=torch.float32);witness=torch.cat([initial_witness,recent]);p=propose(agent,original,batch[0],witness,generator,charge=lambda:guard.charge('candidate-head'),complete=lambda:guard.complete('candidate-head'),check=guard.check);agent.total_it=attempt;evidence=p.pop('_evidence');a=audit(agent.actor,original,x)
                        if p['accepted']:accepted+=1;rejections=0
                        else:rejections+=1
                        budget.data['acceptedSteps']=accepted;atomic_json(budget.path,budget.data);p.update(attempt=attempt,newActualTransitions=n,acceptedSteps=accepted,critic=cl,historicalAudit=a,proposalSeedRole='candidate/actor_proposal',actorSideRNGAfterNoiseSHA256=p['proposalGeneratorAfterNoiseSHA256']);path=out/f'attempt-{attempt:04d}-local.pt';guard.torch_save(path,dict(schema=1,receipt=p,evidence=evidence,roles=roles.state()));p['immutableEvidenceSHA256']=sha256(path);updates.append(p);update_stream.write(json.dumps(p,allow_nan=False)+'\n');update_stream.flush();os.fsync(update_stream.fileno())
                        if not audit_ok(a):raise ValueError('global residual drift veto')
                        guard.commit_proposal('candidate-head')
                        if rejections>=128:stop='normal128-consecutive-rejections';break
                    if n in[1024,2048,3072,4096,5000]and n not in milestones:
                        save_complete(out,guard,env,agent,replay,roles,'milestone',immutable=str(n));milestones.add(n)
                        # Diagnostic only: fresh critic versus recorded complete
                        # behaviour returns, not a current-policy certificate.
                        complete=[e for e in episode_reports if e['returns']];calibration=[]
                        for e in complete[-128:]:
                            row=replay.items[e['firstReplayIndex']]
                            with torch.no_grad():q1,q2=agent.critic(torch.tensor(row[0])[None,:],torch.tensor(row[1])[None,:])
                            calibration.append(dict(episode=e['episode'],qMinimum=float(torch.minimum(q1,q2)),taskOnlyBehaviourReturn=e['returns']['taskOnly'][0],sourceSoftBehaviourReturn=e['returns']['sourceSoft'][0]))
                        atomic_json(out/f'diagnostic-{n}.json',dict(actualTransitions=n,completeEpisodes=len(complete),criticCalibrationReportOnly=calibration,scope='complete off-policy behaviour return; no administrative truncation as zero and no current-policy value certificate',criticWarmupUpdates=agent.critic_warmup_it,acceptedSteps=accepted))
                    elif periodic_checkpoint_due(n):save_complete(out,guard,env,agent,replay,roles,'periodic64-completed-intervals')
                commands.flush();os.fsync(commands.fileno());frames.flush();os.fsync(frames.fileno())
            stop=stop or'completed5000';final_audit=audit(agent.actor,original,x);mid_audit=audit(agent.actor,original,mid);atomic_json(out/'final-audits.json',dict(historical=final_audit,midpoint=mid_audit));save_complete(out,guard,env,agent,replay,roles,'training-final',immutable='final')
            if not audit_ok(final_audit)or not audit_ok(mid_audit):raise ValueError('final drift audit veto')
            if accepted:
                guard.phase='validation';guard.check();p=projection(bench,0,0,0);d=budget.data
                if budget.remaining_wall()<p['validationWall']+.1 or 4-d['stagePhysicsSeconds']['validation']<p['validationPhysics']or 5704-d['totalPolicyIntervals']<640 or 114080-d['sourceIntegrations']<12800:raise TimeoutError('whole final pair not admissible live')
                for label,policy in[('frozen',original),('candidate',agent.actor)]:
                    world.restore(entry_blob);ve=SourceSLOWEnv(world,model,bindings={'runtimeAdmissionSHA256':expected},rng=np.random.default_rng(0));vcommands=[]
                    with(out/(label+'-frames.ndjson')).open('x')as vf:
                        for command in range(320):
                            with torch.no_grad():act=policy(torch.tensor(ve.core.observation(world.observation()),dtype=torch.float32)[None,:],False,False)[0][0].numpy()
                            t=interval(ve,act,budget,'validation');write_rows(vf,t,budget,command=command,branch=label);vcommands.append(transition_record(t));valid_physical_terminal(t)
                            if t['info']['terminated']or t['info']['truncated']:break
                        vf.flush();os.fsync(vf.fileno())
                    vb,vs=ve.checkpoint();guard.bytes_save(out/(label+'-final.bin'),vb);atomic_json(out/(label+'-final-controller.json'),vs);atomic_json(out/(label+'-commands.json'),vcommands);final=vcommands[-1]['info'];validation.append(dict(branch=label,final=final,policyIntervals=len(vcommands),final4000QuietQualified=bool(final['success']and final['quietConsecutiveSamples']>=4000)))
            else:stop+='-zeroaccepted-baseline-retained'
        except BaseException as error:stop='ABORT '+type(error).__name__+': '+str(error)
        finally:
            # Previous rolling manifest stays authoritative on an incomplete or
            # validation-world boundary. Never fabricate a paired sidecar.
            try:
                if env is not None and world.dim(6)==env.core.native_id and env.core.packet_hash(world.task_packet())==env.core.last_packet_hash:
                    if budget.remaining_wall()>300 and guard.phase!='validation':save_complete(out,guard,env,agent,replay,roles,'last-complete-boundary')
                else:atomic_json(out/'current-boundary-not-training.json',dict(reason='validation or incomplete native request; previous rolling checkpoint retained'))
            except BaseException as error:atomic_json(out/'final-save-invalid.json',dict(error=str(error)))
            finally:
                if world:world.close()
            latest=json.loads((out/'latest-checkpoint.json').read_text())if(out/'latest-checkpoint.json').exists()else{}
            report=dict(checkpointRecovery=latest,chargedWorkAfterLastCheckpointIsNotRefunded=True,stopped=stop,newActualTransitions=len(records),acceptedSteps=accepted,proposalAttempts=budget.data.get('proposalAttempts',0),candidateOptimizerCalls=budget.data['candidateOptimizerCalls'],allOptimizerCalls=budget.data['optimizerCalls'],allSourceIntegrations=budget.data['sourceIntegrations'],allPolicyIntervals=budget.data['totalPolicyIntervals'],physicsSeconds=budget.data['physicsSeconds'],runWallSeconds=time.monotonic()-started,validation=validation,selectedPolicy='immutable-source-SLOW-baseline',quietStabilizationQualified=bool(len(validation)==2 and validation[-1]['final4000QuietQualified']),forwardStepQualified=False,runtimeAdmissionSHA256=expected)
            if report['quietStabilizationQualified']:report['selectedPolicy']='candidate-source-SLOW-stabilization-only'
            atomic_json(out/'summary.json',report);atomic_json(out/'episodes.json',episode_reports);budget.data.update(status='closed-online-stage',taskCollectionEnabled=False,unusedAllowanceNotTransferred=True);atomic_json(budget.path,budget.data)
            print(json.dumps(report,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare-benchmark','benchmark','prepare-candidate','candidate']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--expected-admission-sha',default='');a=p.parse_args();{'prepare-benchmark':lambda:prepare_benchmark(a.stage,a.source),'benchmark':lambda:benchmark(a.stage,a.source,a.expected_admission_sha),'prepare-candidate':lambda:prepare_candidate(a.stage,a.source),'candidate':lambda:candidate(a.stage,a.source,a.expected_admission_sha)}[a.mode]()
if __name__=='__main__':main()
