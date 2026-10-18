#!/usr/bin/env python3
"""Algebraic closed-continuation evidence, without engine/optimizer construction."""
import argparse,hashlib,json
from collections import Counter
from pathlib import Path
import numpy as np
ORDER=np.array([1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20])
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb')as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())
def lines(p):
    with Path(p).open()as f:
        for s in f:yield json.loads(s)
def last(p):
    result=None
    for result in lines(p):pass
    if result is None:raise ValueError('empty trace')
    return result

def reduce(stage,source,root):
    out=stage/'learner';summary=read(out/'summary.json');ledger=read(stage/'ledger.json');config=read(root/'scripts/getup-step/slow-online-continuation-v1.json');old=Path(config['resume']['manifest']).parent
    assert ledger['status']=='closed-online-continuation'and not ledger['requestOutstanding']and not ledger.get('pendingOptimizer')and not ledger.get('proposalInProgress')
    original=last(source/'slow-continuation/frames.ndjson');retained=last(old/'frames.ndjson')['packet'];model=read(root/'rust/crates/model/data/getup-v1.json');qref=np.array(model['referenceMetadata']['standingPose']);gear=np.zeros(21)
    for part in model['segments']:
        h=part['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    commands=list(lines(out/'transitions.ndjson'));updates=list(lines(out/'updates.ndjson'));proofs=list(lines(out/'terminal-proofs.ndjson'));global_next=1;branches=[]
    def frames(path,lookup,label):
        nonlocal global_next
        previous=retained if label=='training'else original;episode=48 if label=='training'else None;n=0;torque_error=control_error=0.;warnings=flight=nonfoot=0;slip=np.array([.0020284666548212992,.0015043418958506877])if label=='training'else np.zeros(2);crossings=[]
        for row in lines(path):
            assert row['globalIntegration']==global_next;global_next+=1;n+=1
            if label=='training':
                if row['episode']!=episode:previous=original;episode=row['episode'];slip=np.zeros(2)
                t=lookup[row['globalPolicyInterval']-1]
            else:t=lookup[row['command']]
            packet=row['packet'];u=np.clip(qref+np.array(t['action'])-np.array(previous['qpos'])[7:]-.2*np.array(previous['qvel'])[6:],-1,1)
            torque_error=max(torque_error,float(np.max(abs(np.array(packet['qfrcActuator'])[6:]-gear*u))));control_error=max(control_error,float(np.max(abs(np.array(row['sourceControlAudit']['expectedControls'])-u[ORDER]))))
            before=max(slip);slip+=np.array(packet['loadedSlipSpeedMps'])*.00125
            if before<=.01<max(slip):
                assert t['info']['physicalFailure']=='loaded_slip'and packet['sourceStep']==t['info']['referenceClock']['last_native_substep']
                crossings.append(dict(localSample=n,globalIntegration=row['globalIntegration'],episode=episode,nativeId=packet['sourceStep']))
            warnings+=packet['sourceWarnings']!=0;flight+=max(packet['loadsN'])<3;nonfoot+=packet['nonfootSupport'];previous=packet
        assert torque_error==0 and control_error==0
        result=dict(branch=label,actualNativeSamples=n,torqueEquationMaximumErrorNm=torque_error,controlEquationMaximumError=control_error,warningSamples=warnings,bothFeetUnloadedSamples=flight,nonfootSupportSamples=nonfoot,loadedSlipFirstCrossings=crossings)
        if label!='training':
            final=lookup[-1]['info'];failure=final['physicalFailure'];conditions=[r for t in lookup for r in t['info']['nativeConditions']];anchor=next((i for i,c in enumerate(conditions,1)if c['anchorDriftM']>.01),None)
            assert len(conditions)==n and np.allclose(slip,final['loadedSlipM'],rtol=0,atol=1e-16)
            if failure=='loaded_slip':assert crossings[0]['localSample']==n
            if failure=='fixed_anchor_drift':assert anchor==n
            result.update(elapsedSeconds=n/800,physicalFailure=failure,loadedSlipM=slip.tolist(),firstAnchorCrossingSample=anchor,terminalAnchorDriftM=conditions[-1]['anchorDriftM'],terminalNativeSamples=final['nativeSampleCount'],terminalMask=lookup[-1]['not_done'],final4000QuietQualified=bool(final['success']and final['quietConsecutiveSamples']>=4000))
        return result
    branches.append(frames(out/'frames.ndjson',commands,'training'))
    paired={label:read(out/(label+'-commands.json'))for label in ['frozen','candidate']}
    for label in ['frozen','candidate']:branches.append(frames(out/(label+'-frames.ndjson'),paired[label],label))
    assert np.array_equal(paired['frozen'][0]['state'],paired['candidate'][0]['state'])
    delta=np.array(paired['candidate'][0]['action'])-paired['frozen'][0]['action'];first_action=dict(initialObservation118BitExact=True,residualRMSDifferenceRad=float(np.sqrt(np.mean(delta**2))),residualMaximumDifferenceRad=float(np.max(abs(delta))))
    failures=Counter();partials=Counter();full_terminals=0
    for i,t in enumerate(commands,1):
        assert t['globalPolicyInterval']==i and t['cumulativeTrainingTransition']==1580+i;f=t['info']['physicalFailure'];count=t['info']['nativeSampleCount']
        if f is not None:failures[f]+=1;assert t['not_done']==0 and t['info']['terminated']and not t['info']['truncated'];full_terminals+=count==20
        if count<20:partials[f]+=1;assert f is not None and t['not_done']==0
        conditions=t['info']['nativeConditions'];assert len(conditions)==count
        if f=='fixed_anchor_drift':assert all(v['anchorDriftM']<=.01 for v in conditions[:-1])and conditions[-1]['anchorDriftM']>.01
    assert len(commands)==3420 and sum(t['info']['nativeSampleCount']for t in commands)==64096 and len(proofs)==sum(failures.values())
    for proof,t in zip(proofs,[t for t in commands if t['info']['physicalFailure']is not None]):assert proof['executedInterval']==t and proof['actualResetCacheSHA256']==sha(source/'slow-continuation/final.bin')
    assert [x['attempt']for x in updates]==list(range(140,995))and all(x['accepted']for x in updates)
    all_trials=[v for u in updates for v in u['trials']];assert all(u['trials'][-1]['maximumStepKL']<=.05 and u['trials'][-1]['lossDecrease']>=1e-6 for u in updates)
    assert ledger['candidateOptimizerCalls']==1710==2*len(updates)and global_next-1==66445==ledger['sourceIntegrations']
    paths=[stage/'runtime-admission.json',stage/'ledger.json',out/'summary.json',out/'latest-checkpoint.json',out/'transitions.ndjson',out/'updates.ndjson',out/'terminal-proofs.ndjson',out/'frames.ndjson',out/'frozen-frames.ndjson',out/'candidate-frames.ndjson',out/'final-audits.json',out/'episodes.json']
    return dict(schema=1,kind='closed saved-journal algebraic reduction; zero engine/optimizer',newActualTransitions=3420,cumulativeActualTransitions=5000,priorTrainingNativeSamples=31110,newTrainingNativeSamples=64096,cumulativeTrainingNativeSamples=95206,trainingTerminalCounts=dict(failures),partialTerminalCounts=dict(partials),full20SamplePhysicalTerminals=full_terminals,terminalProofCount=len(proofs),newProposalAttempts=855,newAcceptedProposals=855,cumulativeProposalAttempt=994,cumulativeAcceptedProposals=866,newOptimizerCalls=1710,cumulativeCandidateOptimizerCalls=2988,acceptedScaleCounts=dict(Counter(str(u['scale'])for u in updates)),maximumIncrementalKL=max(u['trials'][-1]['maximumStepKL']for u in updates),minimumAcceptedLossDecrease=min(u['trials'][-1]['lossDecrease']for u in updates),branches=branches,deterministicFirstCommand=first_action,allNewStageNativeSamples=global_next-1,candidateMinusFrozenFailureTimeS=branches[-1]['elapsedSeconds']-branches[-2]['elapsedSeconds'],quietQualified=False,forwardStepQualified=False,selectedPolicy=summary['selectedPolicy'],allInputSHA256={str(p.resolve()):sha(p)for p in paths})
def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=reduce(a.stage,a.source,Path(__file__).resolve().parents[2]);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items()if k not in['allInputSHA256','branches']},indent=2))
if __name__=='__main__':main()
