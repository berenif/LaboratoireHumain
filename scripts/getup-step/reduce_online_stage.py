#!/usr/bin/env python3
"""Read saved online journals; zero engine initialization or optimizer calls."""
import argparse,hashlib,json
from collections import Counter
from pathlib import Path
import numpy as np
ORDER=np.array([1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20])
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def lines(p):
    with Path(p).open()as stream:
        for line in stream:yield json.loads(line)
def reduce(stage,source,root):
    out=stage/'learner';model=read(root/'rust/crates/model/data/getup-v1.json');qref=np.array(model['referenceMetadata']['standingPose']);gear=np.zeros(21)
    for part in model['segments']:
        h=part['reference'].get('hinge')
        if h:gear[h['qposIndex']]=h['gear']
    initial=None
    for initial in lines(source/'slow-continuation/frames.ndjson'):pass
    summary=read(out/'summary.json');ledger=read(stage/'ledger.json');commands=list(lines(out/'transitions.ndjson'));updates=[x for x in lines(out/'updates.ndjson')if 'attempt'in x];proofs=list(lines(out/'terminal-proofs.ndjson'));global_next=1;branch_summaries=[]
    def frames(path,command_lookup,label):
        nonlocal global_next
        previous=initial;episode=None;n=0;torque_error=0.;control_error=0.;warnings=flight=nonfoot=0;slip=np.zeros(2);first_cross=None
        for row in lines(path):
            assert row['globalIntegration']==global_next;global_next+=1;n+=1
            if label=='training':
                if row['episode']!=episode:previous=initial;episode=row['episode']
                command=command_lookup[row['globalPolicyInterval']-1]
            else:command=command_lookup[row['command']]
            packet=row['packet'];a=np.asarray(command['action']);u=np.clip(qref+a-np.asarray(previous['qpos'])[7:]-.2*np.asarray(previous['qvel'])[6:],-1,1)
            torque_error=max(torque_error,float(np.max(abs(np.asarray(packet['qfrcActuator'])[6:]-gear*u))));control_error=max(control_error,float(np.max(abs(np.asarray(row['sourceControlAudit']['expectedControls'])-u[ORDER]))))
            warnings+=packet['sourceWarnings']!=0;flight+=max(packet['loadsN'])<3;nonfoot+=packet['nonfootSupport']
            if label!='training':
                slip+=np.asarray(packet['loadedSlipSpeedMps'])*.00125
                if max(slip)>.01 and first_cross is None:first_cross=n
            previous=packet
        assert torque_error==0 and control_error==0
        result=dict(branch=label,actualNativeSamples=n,torqueEquationMaximumErrorNm=torque_error,controlEquationMaximumError=control_error,warningSamples=warnings,bothFeetUnloadedSamples=flight,nonfootSupportSamples=nonfoot)
        if label!='training':
            result.update(elapsedSeconds=n/800,loadedSlipM=slip.tolist(),firstLoadedSlipCrossingSample=first_cross);assert first_cross==n;assert np.allclose(slip,command_lookup[-1]['info']['loadedSlipM'],rtol=0,atol=1e-16)
        return result
    branch_summaries.append(frames(out/'frames.ndjson',commands,'training'))
    for label in ['frozen','candidate']:branch_summaries.append(frames(out/(label+'-frames.ndjson'),read(out/(label+'-commands.json')),label))
    failures=Counter();partials=Counter();version=Counter()
    for i,t in enumerate(commands,1):
        assert t['globalPolicyInterval']==i;f=t['info']['physicalFailure'];version[t['behaviour']['policyVersion']]+=1
        if f is not None:failures[f]+=1;assert t['not_done']==0
        if t['info']['nativeSampleCount']<20:partials[f]+=1;assert f is not None and t['not_done']==0
    assert sum(t['info']['nativeSampleCount']for t in commands)==branch_summaries[0]['actualNativeSamples'];assert global_next-1==ledger['sourceIntegrations'];assert ledger['candidateOptimizerCalls']==1000+2*len(updates)
    accepted=[u for u in updates if u['accepted']];last_accept=max(u['attempt']for u in accepted);assert len(updates)-last_accept==128
    paths=[stage/'runtime-admission.json',stage/'fresh-candidate-window.json',stage/'ledger.json',out/'summary.json',out/'transitions.ndjson',out/'updates.ndjson',out/'terminal-proofs.ndjson',out/'frames.ndjson',out/'frozen-frames.ndjson',out/'candidate-frames.ndjson',out/'final-audits.json']
    return dict(schema=1,kind='saved-journal algebraic reduction; zero engine/optimizer',newActualTransitions=len(commands),trainingTerminalCounts=dict(failures),partialTerminalCounts=dict(partials),behaviourPolicyVersions=dict(version),terminalProofCount=len(proofs),proposalAttempts=len(updates),acceptedProposals=len(accepted),lastAcceptedAttempt=last_accept,finalConsecutiveRejections=len(updates)-last_accept,acceptedScales=[u['scale']for u in accepted],candidateOptimizerCalls=ledger['candidateOptimizerCalls'],branches=branch_summaries,allGlobalSamples=global_next-1,candidateMinusFrozenFailureTimeS=branch_summaries[-1]['elapsedSeconds']-branch_summaries[-2]['elapsedSeconds'],quietQualified=summary['quietStabilizationQualified'],forwardStepQualified=False,selectedPolicy=summary['selectedPolicy'],allInputSHA256={str(p.resolve()):sha(p)for p in paths})
def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=Path(__file__).resolve().parents[2];result=reduce(a.stage,a.source,root);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items()if k not in['allInputSHA256','behaviourPolicyVersions']},indent=2))
if __name__=='__main__':main()
