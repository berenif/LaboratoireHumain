#!/usr/bin/env python3
"""Prospectively reviewed exact-training continuation; no default execution."""
import argparse,copy,hashlib,json,time,os
from pathlib import Path
import numpy as np
import torch
from native_env import ROOT,atomic_json,sha256
from continuation import CONFIG,CONFIG_SHA,PLAN_SHA,config,retained,native_resume,ContinuationGuard,restore_receipt
from stabilization_ledger import StageBudget
from online_learning import propose,episode_returns,sample_action,PROTOCOL,PROTOCOL_SHA
from online_run import probes,audit,transition_record,write_rows,interval,valid_physical_terminal,projection,FixedBatch
from online_cadence import CADENCE,CADENCE_SHA,save_complete,periodic_checkpoint_due
from slow_task import SourceSLOWEnv,CONFIG_PATH as TASK_CONFIG,CONFIG_SHA as TASK_CONFIG_SHA
from training import critic_only_update

def verify(receipt):
    for p,d in receipt['inputHashes'].items():
        if sha256(p)!=d:raise ValueError('continuation input changed: '+p)
def stage_identity(budget,receipt):
    d=budget.data
    if budget.profile!='slow-online-continuation'or d['startedUtc']!=receipt['stageStartedUtc']or d['deadlineUtc']!=receipt['stageDeadlineUtc']or d['protocolSHA256']!=CONFIG_SHA or d['planSHA256']!=PLAN_SHA or d.get('pendingOptimizer')or d.get('proposalInProgress')or d.get('requestOutstanding')or d['status'].startswith('closed'):raise ValueError('fresh exact continuation stage identity/pending mismatch')
def capacity(budget,guard,bench,intervals,proposals):
    guard.check();guard.storage(admission=True);p=projection(bench,intervals,proposals,0);d=budget.data
    if budget.remaining_wall()<300+p['trainingWall']or p['validationWall']>300 or p['trainingPhysics']>11-d['stagePhysicsSeconds']['training']or p['validationPhysics']>4-d['stagePhysicsSeconds']['validation']or 4060-d['totalPolicyIntervals']<intervals+640 or 81200-d['sourceIntegrations']<(intervals+640)*20:raise TimeoutError('whole continuation and pair no longer admissible: '+str(p))
    return p
def save_continuation(out,guard,env,agent,replay,roles,label,**kwargs):
    entry=save_complete(out,guard,env,agent,replay,roles,label,**kwargs);entry.update(stageNewActualTransitions=len(replay.items)-1580,priorTrainingIntegrations=31110,priorActualTransitions=1580,priorProposalAttempt=139,runtimeAdmissionSHA256=guard.budget.data.get('runtimeAdmissionSHA256'),continuationConfigSHA256=CONFIG_SHA);guard.check();atomic_json(out/'latest-checkpoint.json',entry);return entry
def inputs(stage,source):
    c=config();m=json.loads(Path(c['resume']['manifest']).read_text());paths=list(Path(__file__).parent.glob('*.py'))+[CONFIG,CADENCE,PROTOCOL,TASK_CONFIG,ROOT/'docs/experiments/getup-python-step-plan.md',ROOT/'scripts/getup-training/SAC.py',ROOT/'scripts/getup-diagnostic/compare_mujoco.py',ROOT/'scripts/getup-diagnostic/mujoco_bridge.c',ROOT/'scripts/getup-step-reference.py',ROOT/'scripts/getup-step/source_bridge.c',ROOT/'rust/crates/model/data/getup-v1.json',ROOT/'rust/crates/sim/data/getup-policy/student.bin',source/'source_bridge_v2.so',source/'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3',*sorted((source/'source-model').glob('*.xml')),source/'slow-continuation/final.bin',source/'slow-continuation/frames.ndjson',Path(c['resume']['manifest']),*[Path(m[k])for k in ['native','learner','controller']],Path(c['resume']['closedPriorLedger']),Path(c['resume']['manifest']).parents[1]/'runtime-admission.json',Path(c['resume']['manifest']).parents[1]/'benchmark/summary.json',Path(c['resume']['manifest']).parents[1]/'cadence-io-timing.json',Path(c['resume']['manifest']).parent/'transitions.ndjson',Path(c['resume']['manifest']).parent/'episodes.json',stage/'fixtures.json',stage/'restore-receipt.json']
    return {str(p.resolve()):sha256(p)for p in paths}
def prepare(stage,source):
    c=config()
    if sha256(PROTOCOL)!=PROTOCOL_SHA or sha256(TASK_CONFIG)!=TASK_CONFIG_SHA:raise ValueError('retained source protocol/task equation preset changed')
    fixture=json.loads((stage/'fixtures.json').read_text())
    if not fixture['passed']or sha256(stage/'fixtures.log')!=fixture['logSHA256']:raise ValueError('continuation fixture receipt stale')
    receipt=restore_receipt(source);atomic_json(stage/'restore-receipt.json',receipt);old=Path(c['resume']['manifest']).parents[1];bench=json.loads((old/'benchmark/summary.json').read_text());bench['cadenceIOTimings']=json.loads((old/'cadence-io-timing.json').read_text())
    # Native physical equations, model and assets must still match the ACTUALLY
    # executed parent admission. Intentional learning/ledger/plan changes are
    # reviewed in this new manifest, never passed off as prior executed inputs.
    
    if sha256(old/'runtime-admission.json')!=c['resume']['priorRuntimeAdmissionSHA256']:raise ValueError('retained original runtime admission changed')
    prior=json.loads((old/'runtime-admission.json').read_text());required=['slow_task.py','source_task.py','source_observer.py','source_task_env.py','source_slow_control.py','source_engine.py','source_positive_control.py','source_math.py','actors.py','reference.py','audit_metrics.py','native_env.py','task.py','training.py','stabilization_init.py','SAC.py','compare_mujoco.py','mujoco_bridge.c','source_bridge.c','getup-v1.json','student.bin','source_bridge_v2.so','libmujoco_nogl.so.2.1.3']
    for p,d in prior['inputHashes'].items():
        if Path(p).name in required or p.endswith('.xml'):
            if sha256(p)!=d:raise ValueError('source physical/action equation changed')
    with StageBudget(stage/'ledger.json')as budget:
        fresh=dict(stageStartedUtc=budget.data['startedUtc'],stageDeadlineUtc=budget.data['deadlineUtc']);stage_identity(budget,fresh);guard=ContinuationGuard(budget,stage);guard.phase='training';p=capacity(budget,guard,bench,3420,855)
        if budget.data['sourceIntegrations']or budget.data['optimizerCalls']or budget.data['totalPolicyIntervals']:raise ValueError('fresh continuation only')
        admission=dict(schema=1,**fresh,configSHA256=CONFIG_SHA,planSHA256=PLAN_SHA,cadenceOverlaySHA256=CADENCE_SHA,benchmarkSummary=str((old/'benchmark/summary.json').resolve()),benchmarkSummarySHA256=sha256(old/'benchmark/summary.json'),cadenceIOTimings=bench['cadenceIOTimings'],projection=p,parentChargesPreserved=True,restoreReceiptSHA256=sha256(stage/'restore-receipt.json'),inputHashes=inputs(stage,source));atomic_json(stage/'runtime-admission.json',admission);print('runtime admission SHA256',sha256(stage/'runtime-admission.json'));print(json.dumps(p,indent=2))

def run(stage,source,expected):
    if sha256(stage/'runtime-admission.json')!=expected:raise ValueError('literal external runtime admission SHA required')
    admission=json.loads((stage/'runtime-admission.json').read_text());verify(admission);bench=json.loads(Path(admission['benchmarkSummary']).read_text())
    if sha256(admission['benchmarkSummary'])!=admission['benchmarkSummarySHA256']or sha256(CADENCE)!=admission['cadenceOverlaySHA256']:raise ValueError('benchmark/cadence changed')
    bench['cadenceIOTimings']=admission['cadenceIOTimings']
    out=stage/'learner';out.mkdir(exist_ok=False);world=None;env=None;stop=None;records=[];episode_rows=[];episode_reports=[];updates=[];validation=[];accepted=11;new_accepted=0;rejections=0;started=time.monotonic();milestones=set()
    with StageBudget(stage/'ledger.json')as budget:
        stage_identity(budget,admission);guard=ContinuationGuard(budget,stage);guard.phase='training';capacity(budget,guard,bench,3420,855)
        if budget.data['candidateOptimizerCalls']or budget.data['stagePolicyIntervals']['training']:raise ValueError('fresh candidate only')
        (stage/'candidate-started-once').open('x').close();budget.data.update(taskCollectionEnabled=True,status='online-collection',runtimeAdmissionSHA256=expected);atomic_json(budget.path,budget.data)
        try:
            c,m,agent,original,replay,roles,resume_blob,resume_side,episode_rows,episode_reports=retained();world,env,model=native_resume(source,resume_blob,resume_side);entry_blob=(source/'slow-continuation/final.bin').read_bytes();entry_sha=hashlib.sha256(entry_blob).hexdigest();x=probes(source,env.core);mid=probes(source,env.core,True);episode=48;initial_witness=torch.tensor(np.array([v for row in replay.items[:1024]for v in[row[0],row[2]]]),dtype=torch.float32)
            save_continuation(out,guard,env,agent,replay,roles,'initial')
            with(out/'frames.ndjson').open('x')as frames,(out/'transitions.ndjson').open('x')as commands,(out/'updates.ndjson').open('x')as update_stream,(out/'terminal-proofs.ndjson').open('x')as terminal_stream:
                while len(replay.items)<5000:
                    guard.check()
                    if env.core.terminal:
                        begin=time.monotonic();world.restore(entry_blob);env=SourceSLOWEnv(world,model,bindings=resume_side['core']['bindings'],rng=np.random.default_rng(0));episode+=1;episode_rows=[]
                    obs=torch.tensor(env.core.observation(world.observation()),dtype=torch.float32)[None,:];action,behaviour=sample_action(agent.actor,obs,roles,accepted);t=interval(env,action,budget,'training');write_rows(frames,t,budget,episode=episode,priorTrainingIntegrations=31110,cumulativeTrainingTransition=len(replay.items)+1);valid_physical_terminal(t);t['behaviour']=behaviour;replay.add(len(replay.items)+1,t)
                    record=transition_record(t,episode=episode,globalPolicyInterval=budget.data['totalPolicyIntervals'],cumulativeTrainingTransition=len(replay.items),actualTaskDataCollected=True,runtimeAdmissionSHA256=expected);record['info']=copy.deepcopy(record['info']);record['info'].update(measurementOnly=False,replayEligible=True,runtimeActualCollectionEnabled=True);records.append(record);episode_rows.append(record);commands.write(json.dumps(record,allow_nan=False)+'\n')
                    n=len(replay.items);terminal=bool(t['info']['terminated']or t['info']['truncated'])
                    if terminal:
                        commands.flush();os.fsync(commands.fileno());terminal_stream.write(json.dumps(dict(episode=episode,nextResetOrdinal=episode+1,actualResetCacheSHA256=entry_sha,globalPolicyInterval=budget.data['totalPolicyIntervals'],globalSourceIntegration=budget.data['sourceIntegrations'],executedInterval=record),allow_nan=False)+'\n');terminal_stream.flush();os.fsync(terminal_stream.fileno());returns=episode_returns(episode_rows);episode_reports.append(dict(episode=episode,firstReplayIndex=n-len(episode_rows),actualNativeSamples=sum(r['info']['nativeSampleCount']for r in episode_rows),shorterThan100ms=sum(r['info']['nativeSampleCount']for r in episode_rows)<80,final=t['info'],returns=returns))
                    if n>1024 and(n-1024)%4==0:
                        if not budget.data['proposalAttempts']:capacity(budget,guard,bench,5000-n,855)
                        batch=replay.sample(128)
                        with roles.critic_scope():cl=critic_only_update(agent,FixedBatch(batch),128,before_optimizer_step=lambda:guard.charge('continuation-critic'))
                        guard.complete('continuation-critic');attempt=roles.attempt_id+1;generator=roles.proposal(attempt);budget.data['proposalAttempts']=attempt-139;budget.data['cumulativeProposalAttempt']=attempt;atomic_json(budget.path,budget.data);recent=torch.tensor(np.array([v for row in replay.items[-256:]for v in[row[0],row[2]]]),dtype=torch.float32);witness=torch.cat([initial_witness,recent]);p=propose(agent,original,batch[0],witness,generator,charge=lambda:guard.charge('continuation-head'),complete=lambda:guard.complete('continuation-head'),check=guard.check,enforce_original_mean_distance=False);agent.total_it=attempt;evidence=p.pop('_evidence');a=audit(agent.actor,original,x)
                        if p['accepted']:accepted+=1;new_accepted+=1;rejections=0
                        else:rejections+=1
                        budget.data['acceptedSteps']=new_accepted;budget.data['cumulativeAcceptedSteps']=accepted;budget.data['newConsecutiveLossKLRejections']=rejections;atomic_json(budget.path,budget.data);p.update(attempt=attempt,newActualTransitions=n-1580,cumulativeActualTransitions=n,acceptedSteps=accepted,critic=cl,historicalAudit=a,proposalSeedRole='candidate/actor_proposal',actorSideRNGAfterNoiseSHA256=p['proposalGeneratorAfterNoiseSHA256']);path=out/f'attempt-{attempt:04d}-local.pt';guard.torch_save(path,dict(schema=1,receipt=p,evidence=evidence,roles=roles.state()));p['immutableEvidenceSHA256']=sha256(path);updates.append(p);update_stream.write(json.dumps(p,allow_nan=False)+'\n');update_stream.flush();os.fsync(update_stream.fileno())
                        # Original-policy distance and historical drift are REPORT ONLY.
                        guard.commit_proposal('continuation-head')
                        if rejections>=128:stop='normal128-consecutive-rejections';break
                    if n in[2048,3072,4096,5000]and n not in milestones:
                        save_continuation(out,guard,env,agent,replay,roles,'milestone',immutable=str(n));milestones.add(n)
                        # Diagnostic only: fresh critic versus recorded complete
                        # behaviour returns, not a current-policy certificate.
                        complete=[e for e in episode_reports if e['returns']];calibration=[]
                        for e in complete[-128:]:
                            row=replay.items[e['firstReplayIndex']]
                            with torch.no_grad():q1,q2=agent.critic(torch.tensor(row[0])[None,:],torch.tensor(row[1])[None,:])
                            calibration.append(dict(episode=e['episode'],qMinimum=float(torch.minimum(q1,q2)),taskOnlyBehaviourReturn=e['returns']['taskOnly'][0],sourceSoftBehaviourReturn=e['returns']['sourceSoft'][0]))
                        atomic_json(out/f'diagnostic-{n}.json',dict(actualTransitions=n,completeEpisodes=len(complete),criticCalibrationReportOnly=calibration,scope='complete off-policy behaviour return; no administrative truncation as zero and no current-policy value certificate',criticWarmupUpdates=agent.critic_warmup_it,acceptedSteps=accepted))
                    elif periodic_checkpoint_due(n):save_continuation(out,guard,env,agent,replay,roles,'periodic64-completed-intervals')
                commands.flush();os.fsync(commands.fileno());frames.flush();os.fsync(frames.fileno())
            stop=stop or'completed-cumulative5000';final_audit=audit(agent.actor,original,x);mid_audit=audit(agent.actor,original,mid);atomic_json(out/'final-audits.json',dict(historical=final_audit,midpoint=mid_audit));save_continuation(out,guard,env,agent,replay,roles,'training-final',immutable='final')
            # Finite audit reports never veto the prospective physical pair.
            if new_accepted:
                guard.phase='validation';guard.check();p=projection(bench,0,0,0);d=budget.data
                if budget.remaining_wall()<p['validationWall']+.1 or 4-d['stagePhysicsSeconds']['validation']<p['validationPhysics']or 4060-d['totalPolicyIntervals']<640 or 81200-d['sourceIntegrations']<12800:raise TimeoutError('whole final pair not admissible live')
                for label,policy in[('frozen',original),('candidate',agent.actor)]:
                    world.restore(entry_blob);ve=SourceSLOWEnv(world,model,bindings=resume_side['core']['bindings'],rng=np.random.default_rng(0));vcommands=[]
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
                    if budget.remaining_wall()>300 and guard.phase!='validation':save_continuation(out,guard,env,agent,replay,roles,'last-complete-boundary')
                else:atomic_json(out/'current-boundary-not-training.json',dict(reason='validation or incomplete native request; previous rolling checkpoint retained'))
            except BaseException as error:atomic_json(out/'final-save-invalid.json',dict(error=str(error)))
            finally:
                if world:world.close()
            latest=json.loads((out/'latest-checkpoint.json').read_text())if(out/'latest-checkpoint.json').exists()else{}
            report=dict(checkpointRecovery=latest,chargedWorkAfterLastCheckpointIsNotRefunded=True,stopped=stop,newActualTransitions=len(records),cumulativeActualTransitions=1580+len(records),newAcceptedSteps=new_accepted,cumulativeAcceptedSteps=accepted,newProposalAttempts=budget.data.get('proposalAttempts',0),cumulativeProposalAttempt=139+budget.data.get('proposalAttempts',0),candidateOptimizerCalls=budget.data['candidateOptimizerCalls'],allOptimizerCalls=budget.data['optimizerCalls'],allSourceIntegrations=budget.data['sourceIntegrations'],allPolicyIntervals=budget.data['totalPolicyIntervals'],physicsSeconds=budget.data['physicsSeconds'],runWallSeconds=time.monotonic()-started,validation=validation,selectedPolicy='immutable-source-SLOW-baseline',quietStabilizationQualified=bool(len(validation)==2 and validation[-1]['final4000QuietQualified']),forwardStepQualified=False,runtimeAdmissionSHA256=expected)
            if report['quietStabilizationQualified']:report['selectedPolicy']='candidate-source-SLOW-stabilization-only'
            atomic_json(out/'summary.json',report);atomic_json(out/'episodes.json',episode_reports);budget.data.update(status='closed-online-continuation',taskCollectionEnabled=False,unusedAllowanceNotTransferred=True);atomic_json(budget.path,budget.data)
            print(json.dumps(report,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--expected-admission-sha',default='');a=p.parse_args();prepare(a.stage,a.source)if a.mode=='prepare'else run(a.stage,a.source,a.expected_admission_sha)
if __name__=='__main__':main()
