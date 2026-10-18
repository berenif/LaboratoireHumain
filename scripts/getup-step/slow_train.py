#!/usr/bin/env python3
"""Separately hash-admitted source SLOW stabilization viability. No default run.

The immutable design config remains disabled. This CLI requires an externally
pinned runtime admission binding passed benchmark/tests/code and a live fresh
stage ledger. It never changes the original source reference, PD or actor units.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from native_env import ROOT,atomic_json,sha256
from slow_task import SourceSLOWWorld,SourceSLOWEnv,CONFIG_PATH,CONFIG_SHA,initialize_slow_agent
from slow_learning import SlowReplay,learner_state,charged_source_update
from slow_benchmark import fingerprints as benchmark_inputs
from stabilization_ledger import StageBudget
from training import critic_only_update

def inputs(stage,source):
    files=benchmark_inputs(stage,source)
    for name in ['slow_train.py','slow_learning.py','test_slow_learning.py','test_slow_task.py']:
        p=Path(__file__).with_name(name);files[str(p.resolve())]=sha256(p)
    for p in [stage/'benchmark-preflight.json',stage/'benchmark/summary.json',stage/'learner-fixtures.json']:
        files[str(p.resolve())]=sha256(p)
    return files

def prepare(stage,source):
    bench=json.loads((stage/'benchmark/summary.json').read_text());tests=json.loads((stage/'learner-fixtures.json').read_text());ledger=json.loads((stage/'ledger.json').read_text())
    for receipt in [bench,json.loads((stage/'benchmark-preflight.json').read_text())]:
        for path,digest in receipt['inputHashes'].items():
            if sha256(path)!=digest:raise ValueError('benchmark evidence is stale for current interface')
    if not bench['passed']or not bench['fullCache20SampleContinuationBitExact']or bench['maximumPerNativeTorqueErrorNm']>1e-5 or not tests['passed']:raise ValueError('learner interface/fixture admission failed')
    if ledger['profile']!='slow-stabilization'or ledger['stagePolicyIntervals']['training']or ledger.get('actorUpdates',0)or ledger.get('requestOutstanding'):raise ValueError('fresh candidate collection required')
    rate=bench['endToEndPolicyIntervalsPerSecond'];physics_per_frame=bench['physicsSeconds']/bench['allBranchNativeIntegrations']
    with StageBudget(stage/'ledger.json')as budget:
        need_validation=640/rate*1.5;need_training=356/rate*1.5+60
        if rate<=0 or budget.remaining_wall()<300+need_training or need_validation>300 or physics_per_frame*640*20*1.5>3 or physics_per_frame*356*20*1.5>6:raise ValueError('measured rate cannot admit training plus protected whole validation')
    admission=dict(schema=1,kind='explicit stage-only runtime authorization over disabled immutable equation preset',configSHA256=CONFIG_SHA,runtimeCollectionEnabled=True,runtimeSourceSACEnabledAfterCriticWarmup=True,designConfigDefaultFlagsRemainFalse=True,stageStartedUtc=ledger['startedUtc'],stageDeadlineUtc=ledger['deadlineUtc'],maximumTrainingIntervals=356,maximumActorUpdates=25,maximumCriticOnlyUpdates=100,maximumWarmupTransitions=256,maximumBatchSize=128,maximumPerPairValidationIntervals=640,minimumValidationPhysicsReserve=3,minimumValidationWallReserve=300,maximumSourceIntegrationsAllStages=30000,benchmarkComplete=True,tensorCheckpointFixturesPassed=True,measuredPolicyIntervalsPerSecond=rate,predictedTrainingWallSeconds=need_training,predictedWholePairedValidationWallSeconds=need_validation,measuredPhysicsSecondsPerNativeIntegration=physics_per_frame,predictedTrainingPhysicsSeconds=physics_per_frame*356*20*1.5,predictedValidationPhysicsSeconds=physics_per_frame*640*20*1.5,inputHashes=inputs(stage,source))
    atomic_json(stage/'runtime-admission.json',admission);print(json.dumps({k:v for k,v in admission.items()if k!='inputHashes'},indent=2));print('Admission SHA256',sha256(stage/'runtime-admission.json'))

def real_probes(source,core):
    rows=[json.loads(x)for x in(source/'slow-continuation/frames.ndjson').read_text().splitlines()]
    return np.array([core.observation(rows[i]['physicalObservation'])for i in range(2400,6401,20)],dtype=np.float32)

def run(stage,source,expected_sha):
    import torch
    if not expected_sha or sha256(stage/'runtime-admission.json')!=expected_sha:raise ValueError('external runtime admission SHA required')
    admission=json.loads((stage/'runtime-admission.json').read_text())
    if not admission['runtimeCollectionEnabled']or not admission['runtimeSourceSACEnabledAfterCriticWarmup']or admission['configSHA256']!=CONFIG_SHA:raise ValueError('runtime not admitted')
    for path,digest in admission['inputHashes'].items():
        if sha256(path)!=digest:raise ValueError('runtime admission input changed')
    out=stage/'learner';out.mkdir(exist_ok=False);started=time.monotonic();world=None;env=None;records=[];episodes=[];updates=[];validations=[];stop=None;generation=0
    with StageBudget(stage/'ledger.json')as budget:
        if budget.profile!='slow-stabilization'or not budget.data.get('slowBenchmarkPassed')or budget.data.get('pendingOptimizer')or budget.data.get('actorUpdates',0)or budget.data['stagePolicyIntervals']['training']:raise ValueError('fresh admitted candidate required')
        if budget.data['startedUtc']!=admission['stageStartedUtc']or budget.data['deadlineUtc']!=admission['stageDeadlineUtc']:raise ValueError('stage clock cannot restart')
        def check_live_capacity(planned_intervals,planned_wall):
            d=budget.data;validation_left=max(0,640-d['stagePolicyIntervals']['validation']);need_steps=(planned_intervals+validation_left)*20;need_training_physics=admission['measuredPhysicsSecondsPerNativeIntegration']*planned_intervals*20*1.5
            if budget.remaining_wall()<300+planned_wall or 1500-d['totalPolicyIntervals']<planned_intervals+validation_left or 796-d['stagePolicyIntervals']['training']<planned_intervals or 30000-d['sourceIntegrations']<need_steps or 10-d['physicsSeconds']<3+need_training_physics or 6-d['stagePhysicsSeconds']['training']<need_training_physics:raise TimeoutError('live training admission must preserve whole paired validation capacity')
        check_live_capacity(356,admission['predictedTrainingWallSeconds']);(stage/'learner-started-once').open('x').close();budget.allowance('training')
        budget.data.update(taskCollectionEnabled=True,sourceSACEnabled=False,status='SLOW-candidate-collection');atomic_json(stage/'ledger.json',budget.data)
        try:
            agent,_=initialize_slow_agent();frozen=copy.deepcopy(agent.actor);replay=SlowReplay();world=SourceSLOWWorld(source);entry_blob=(source/'slow-continuation/final.bin').read_bytes();world.restore(entry_blob);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());bindings={'runtimeAdmissionSHA256':expected_sha};rng=np.random.default_rng(0);env=SourceSLOWEnv(world,model,bindings=bindings,rng=rng);probes=torch.tensor(real_probes(source,env.core));episode=0
            with torch.inference_mode():baseline_actions=frozen(probes,False,False)[0].clone()
            def save_checkpoint(label):
                nonlocal generation
                generation+=1;prefix=out/f'checkpoint-{generation:03d}';blob,side=env.checkpoint();native=prefix.with_suffix('.bin');native.write_bytes(blob);learning=prefix.with_suffix('.pt');torch.save(learner_state(agent,replay),learning);sidepath=prefix.with_suffix('.json');atomic_json(sidepath,dict(schema=1,label=label,source=side,learnerSHA256=sha256(learning),nativeSHA256=sha256(native),runtimeAdmissionSHA256=expected_sha,candidateActorUpdates=budget.data.get('actorUpdates',0),criticOnlyUpdates=budget.data.get('criticOnlyUpdates',0)));atomic_json(out/'latest-checkpoint.json',dict(native=str(native),learner=str(learning),controller=str(sidepath),nativeSHA256=sha256(native),learnerSHA256=sha256(learning),controllerSHA256=sha256(sidepath)))
            def before(kind):
                budget.allowance('training')
                if budget.data.get('pendingOptimizer'):raise ValueError('unresolved optimizer attempt')
                if kind=='critic-only':
                    if budget.data.get('criticOnlyUpdates',0)>=100:raise ValueError('critic warmup limit')
                    budget.data['criticOnlyUpdates']=budget.data.get('criticOnlyUpdates',0)+1
                elif kind=='actor':
                    if budget.data.get('criticOnlyUpdates',0)!=100 or budget.data.get('actorUpdates',0)>=25:raise ValueError('actor release/count gate')
                    budget.data['actorUpdates']=budget.data.get('actorUpdates',0)+1
                budget.data['optimizerUpdates']=budget.data.get('optimizerUpdates',0)+1;budget.data['pendingOptimizer']=kind;atomic_json(stage/'ledger.json',budget.data)
            def after(kind):
                if budget.data.get('pendingOptimizer')!=kind:raise ValueError('optimizer charge mismatch')
                budget.data['pendingOptimizer']=None;atomic_json(stage/'ledger.json',budget.data)
            save_checkpoint('candidate-initial-source-mean')
            while len(replay.items)<356:
                budget.allowance('training')
                if env.core.terminal:
                    episodes.append(dict(episode=episode,terminal=records[-1]['info'],reason='actual finite task terminal; new training-only episode from same actual cache'))
                    if records[-1]['info']['physicalFailure']=='native_integrity':raise ValueError('native integrity stop; no episode retry')
                    world.restore(entry_blob);env=SourceSLOWEnv(world,model,bindings=bindings,rng=rng);episode+=1
                obs=env.core.observation(world.observation())
                with torch.inference_mode():_,action,_,_=agent.actor(torch.tensor(obs)[None,:],compute_log_pi=False)
                budget.begin_interval('training');t=env.measured_interval(action[0].numpy(),lambda w:budget.integrate('training',lambda maximum:w.lib.lh_step_audited(w.h,maximum),lambda:w.dim(6)))
                replay.add(budget.data['totalPolicyIntervals'],t)
                record={k:v for k,v in t.items()if k not in['actualFrames','perNativeControls']};record.update(episode=episode,globalPolicyInterval=budget.data['totalPolicyIntervals'],actualTaskDataCollected=True);record['info']=copy.deepcopy(record['info']);record['info'].update(measurementOnly=False,replayEligible=True,taskDataCollectionEnabled=True,optimizerEnabled=bool(budget.data['sourceSACEnabled']),runtimeAdmissionSHA256=expected_sha,designPresetDefaultFlagsRemainFalse=True);records.append(record)
                with(out/'frames.ndjson').open('a')as stream:
                    end=budget.data['sourceIntegrations'];count=len(t['actualFrames'])
                    for i,row in enumerate(t['actualFrames']):stream.write(json.dumps(dict(globalIntegration=end-count+i+1,globalPolicyInterval=budget.data['totalPolicyIntervals'],episode=episode,packet=row),allow_nan=False)+'\n')
                n=len(replay.items)
                if n==256:
                    fixed={k:v.detach().clone()for k,v in agent.actor.state_dict().items()};alpha=agent.log_alpha.detach().clone();actor_opt=copy.deepcopy(agent.actor_optimizer.state_dict());alpha_opt=copy.deepcopy(agent.log_alpha_optimizer.state_dict())
                    for _ in range(100):
                        loss=critic_only_update(agent,replay,128,before_optimizer_step=lambda:before('critic-only'));after('critic-only');updates.append(dict(phase='critic-only',**loss))
                    if any(not torch.equal(v,fixed[k])for k,v in agent.actor.state_dict().items())or not torch.equal(agent.log_alpha,alpha)or agent.actor_optimizer.state_dict()!=actor_opt or agent.log_alpha_optimizer.state_dict()!=alpha_opt:raise ValueError('critic warmup altered frozen actor/alpha')
                    check_live_capacity(100,100/admission['measuredPolicyIntervalsPerSecond']*1.5+60);budget.data['sourceSACEnabled']=True;atomic_json(stage/'ledger.json',budget.data);save_checkpoint('critic-warmup-complete')
                if n>256 and(n-256)%4==0:
                    if budget.data.get('criticOnlyUpdates',0)!=100 or budget.data.get('actorUpdates',0)>=25:raise ValueError('source update cadence gate')
                    loss=charged_source_update(agent,replay,128,before_optimizer_step=before,after_optimizer_step=after)
                    with torch.inference_mode():mean,_,_,logstd=agent.actor(probes,False,False);delta=mean-baseline_actions;drift=dict(rms=float(torch.sqrt(torch.mean(delta**2))),maximum=float(torch.max(abs(delta))),latentStdMaximum=float(torch.max(logstd.exp())))
                    updates.append(dict(phase='source-SAC',newActualTransitions=n,**loss,drift=drift,perProbeLatentStd=logstd.exp().numpy().tolist()));save_checkpoint('source-SAC-update')
                    if drift['rms']>.02 or drift['maximum']>.05:stop='deterministic-residual-drift-stop-rollback';break
            if stop is None:
                if budget.data.get('actorUpdates',0)!=25:raise ValueError('unexpected candidate actor update endpoint')
                # Admit a whole paired final5s opportunity, never shorten it to fit.
                need=640/admission['measuredPolicyIntervalsPerSecond']*1.5
                if budget.remaining_wall()<need+.1 or budget.data['aggregatePhysicsLimitSeconds']-budget.data['physicsSeconds']<3:raise TimeoutError('whole paired validation no longer admissible')
                for label,policy in [('frozen-source-initializer',frozen),('candidate',agent.actor)]:
                    world.restore(entry_blob);ve=SourceSLOWEnv(world,model,bindings=bindings,rng=np.random.default_rng(0));vrows=[]
                    for command in range(320):
                        obs=ve.core.observation(world.observation())
                        with torch.inference_mode():a=policy(torch.tensor(obs)[None,:],False,False)[0][0].numpy()
                        budget.begin_interval('validation');t=ve.measured_interval(a,lambda w:budget.integrate('validation',lambda maximum:w.lib.lh_step_audited(w.h,maximum),lambda:w.dim(6)));vrows.append({k:v for k,v in t.items()if k not in['actualFrames','perNativeControls']})
                        with(out/(label+'-frames.ndjson')).open('a')as stream:
                            end=budget.data['sourceIntegrations'];count=len(t['actualFrames'])
                            for i,row in enumerate(t['actualFrames']):stream.write(json.dumps(dict(globalIntegration=end-count+i+1,command=command,packet=row),allow_nan=False)+'\n')
                        if t['info']['terminated']or t['info']['truncated']:break
                    vblob,vside=ve.checkpoint();(out/(label+'-final.bin')).write_bytes(vblob);torch.save(policy.state_dict(),out/(label+'-actor-local.pt'));atomic_json(out/(label+'-final-controller.json'),dict(source=vside,actorSHA256=sha256(out/(label+'-actor-local.pt')),runtimeAdmissionSHA256=expected_sha))
                    validations.append(dict(branch=label,policyIntervals=len(vrows),final=vrows[-1]['info'],final4000QuietQualified=bool(vrows[-1]['info']['success']and vrows[-1]['info']['quietConsecutiveSamples']>=4000)));atomic_json(out/(label+'-commands.json'),vrows)
                stop='completed-bounded-SLOW-viability'
        except BaseException as error:stop=type(error).__name__+': '+str(error)
        finally:
            try:
                if env is not None and world is not None and world.dim(6)==env.core.native_id and env.core.packet_hash(world.task_packet())==env.core.last_packet_hash:save_checkpoint('final-complete-training-boundary')
                elif world is not None:atomic_json(out/'final-hybrid-invalid.json',dict(reason='current engine is a validation or incomplete interval, not paired with latest training controller; retained previous complete training checkpoint'))
            except BaseException as e:atomic_json(out/'final-hybrid-invalid.json',dict(error=str(e)))
            atomic_json(out/'transitions.json',records);atomic_json(out/'episodes.json',episodes);atomic_json(out/'updates.json',updates);atomic_json(out/'validation-summary.json',validations)
            candidate_quiet=bool(validations and validations[-1]['branch']=='candidate'and validations[-1]['final4000QuietQualified']);regression=False
            if len(validations)==2:
                base_final=validations[0]['final'];candidate_final=validations[1]['final'];severe={'native_integrity','flight','nonfoot_support','interfoot_overlap'}
                regression=bool(candidate_final['physicalFailure']in severe and base_final['physicalFailure']not in severe or candidate_final['physicalFailure']is not None and candidate_final['referenceClock']['tau']<base_final['referenceClock']['tau']-.025)
            report=dict(stopped=stop,candidatePromoted=candidate_quiet and not regression,selectedPolicy='candidate'if candidate_quiet and not regression else'immutable-source-SLOW-baseline',rollbackToFrozen=not(candidate_quiet and not regression),pairedPhysicalSafetyRegression=regression,candidateActualTransitions=len(records),candidateActorUpdates=budget.data.get('actorUpdates',0),criticOnlyUpdates=budget.data.get('criticOnlyUpdates',0),candidateOptimizerCallsCharged=budget.data.get('optimizerUpdates',0),allStageNativeIntegrations=budget.data['sourceIntegrations'],allStagePolicyIntervals=budget.data['totalPolicyIntervals'],allStagePhysicsSeconds=budget.data['physicsSeconds'],candidateRunWallSeconds=time.monotonic()-started,validation=validations,quietStabilizationQualified=bool(validations and validations[-1]['branch']=='candidate'and validations[-1]['final4000QuietQualified']),forwardStepQualified=False,sourceHoldingReferencePreserved=True,sourceInnerPDHz=800,actionUnits='qpos-order residual radians',runtimeAdmissionSHA256=expected_sha,sourceModelAndWeightsRedistributed=False)
            atomic_json(out/'summary.json',report);budget.data.update(status='closed-SLOW-viability',taskCollectionEnabled=False,sourceSACEnabled=False,unusedAllowanceNotTransferred=True);atomic_json(stage/'ledger.json',budget.data)
            if world is not None:world.close()
            print(json.dumps({k:v for k,v in report.items()if k!='validation'},indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--expected-admission-sha',default='');a=p.parse_args();prepare(a.stage,a.source)if a.mode=='prepare'else run(a.stage,a.source,a.expected_admission_sha)
if __name__=='__main__':main()
