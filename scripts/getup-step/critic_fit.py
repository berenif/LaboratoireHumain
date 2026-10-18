#!/usr/bin/env python3
"""Two fixed-target replay regression arms; no environment construction/physics."""
import argparse,copy,datetime as dt,fcntl,hashlib,json,os,shutil,time
from pathlib import Path
import numpy as np
import torch
from native_env import ROOT,atomic_json,sha256
from online_learning import restore,tensor_sha
from slow_trust import gaussian,ALPHA
from training import source_module
CONFIG=Path(__file__).with_name('critic-fit-protocol-v1.json')

def tensors_sha(values):return hashlib.sha256(b''.join(k.encode()+v.detach().cpu().contiguous().numpy().tobytes()for k,v in sorted(values.items()))).hexdigest()
def nested_sha(value):
    h=hashlib.sha256()
    def visit(x):
        if isinstance(x,torch.Tensor):h.update(str((x.dtype,tuple(x.shape))).encode());h.update(x.detach().cpu().contiguous().numpy().tobytes())
        elif isinstance(x,dict):
            for k in sorted(x,key=str):h.update(str(k).encode());visit(x[k])
        elif isinstance(x,(list,tuple)):
            for v in x:visit(v)
        else:h.update(repr(x).encode())
    visit(value);return h.hexdigest()

def load():
    c=json.loads(CONFIG.read_text());path=ROOT/c['sourceCheckpoint']
    if sha256(path)!=c['sourceCheckpointSHA256']:raise ValueError('fixed5000 checkpoint changed')
    payload=torch.load(path,map_location='cpu',weights_only=True);a,_,r,_=restore(payload['learning'])
    for module in[a.actor,a.critic_target]:
        for p in module.parameters():p.requires_grad_(False)
    a.log_alpha.requires_grad_(False)
    if len(r.items)!=5000 or a.total_it!=994 or a.critic_warmup_it!=1994:raise ValueError('wrong closed endpoint')
    columns=[torch.tensor(np.array([row[k]for row in r.items]),dtype=torch.float32)for k in range(5)]
    base=ROOT/'outputs/getup-source-continuation/session-20261018T032214Z/learner';old=ROOT/'outputs/getup-source-online/session-20261018T030112Z/learner';records=[]
    for p in[old/'transitions.ndjson',base/'transitions.ndjson']:records.extend(json.loads(s)for s in p.read_text().splitlines())
    if len(records)!=5000:raise ValueError('episode replay alignment')
    for k,key in enumerate(['state','action','next_state','reward','not_done']):
        if not torch.equal(columns[k],torch.tensor(np.array([np.asarray(row[key]).reshape(-1)for row in records]),dtype=torch.float32)):raise ValueError('replay/record mismatch')
    return c,a,columns,records

def episode_split(records):
    completed={r['episode']for r in records if r['info']['terminated']or r['info']['truncated']};train=[i for i,r in enumerate(records)if r['episode']in completed and r['episode']%5!=0];holdout=[i for i,r in enumerate(records)if r['episode']in completed and r['episode']%5==0]
    if not train or not holdout or set(train)&set(holdout):raise ValueError('invalid disjoint episode split')
    return dict(train=train,holdout=holdout,excluded=[i for i in range(len(records))if i not in set(train+holdout)],trainEpisodes=sorted({records[i]['episode']for i in train}),holdoutEpisodes=sorted({records[i]['episode']for i in holdout}))

def fixed_targets(a,columns,seed):
    _,_,next_state,reward,mask=columns
    if mask.shape!=reward.shape or mask.shape!=(len(next_state),1)or not all(x.isfinite().all()for x in columns)or torch.any((mask!=0)&(mask!=1)):raise ValueError('finite source replay masks required')
    source=source_module();g=torch.Generator().manual_seed(seed)
    with torch.no_grad():
        mu,ls=gaussian(a.actor,next_state);noise=torch.randn(mu.shape,generator=g);logp=source.gaussian_logprob(noise,ls);_,action,logp=source.squash(mu,mu+noise*ls.exp(),logp,next_state[:,-1]);q1,q2=a.critic_target(next_state,action);y=reward+mask*a.discount*(torch.minimum(q1,q2)-a.alpha.detach()*logp)
    if not y.isfinite().all()or not torch.equal(y[mask==0],reward[mask==0]):raise ValueError('terminal target not exact reward')
    return y,noise

def batch_indices(arm,split,mask,rng):
    if arm=='natural':return rng.choice(split['train'],128,replace=True)
    if arm!='balanced':raise ValueError('fixed arm required')
    terminal=[i for i in split['train']if mask[i,0]==0];nonterminal=[i for i in split['train']if mask[i,0]==1]
    if not terminal or not nonterminal:raise ValueError('nonempty fixed sampling strata required')
    return np.concatenate([rng.choice(terminal,64,replace=True),rng.choice(nonterminal,64,replace=True)])

def fit_step(critic,optimizer,state,action,target,*,charge,complete):
    if state.ndim!=2 or state.shape[1]!=118 or action.shape!=(len(state),21)or target.shape!=(len(state),1)or not all(x.isfinite().all()for x in[state,action,target]):raise ValueError('invalid fixed-target batch')
    q1,q2=critic(state,action);loss=torch.nn.functional.mse_loss(q1,target)+torch.nn.functional.mse_loss(q2,target)
    if not loss.isfinite():raise ValueError('nonfinite source critic MSE')
    optimizer.zero_grad(set_to_none=True);loss.backward()
    if any(p.grad is None or not p.grad.isfinite().all()for p in critic.parameters()):raise ValueError('invalid critic gradients')
    charge();optimizer.step();complete()
    if any(not p.isfinite().all()for p in critic.parameters()):raise ValueError('nonfinite updated critic')
    return float(loss.detach())

def evaluate(critic,columns,y,split):
    with torch.no_grad():q=critic(columns[0],columns[1])
    result={}
    for group,ids in split.items():
        if group not in['train','holdout']:continue
        for kind,flag in[('terminal',0),('nonterminal',1)]:
            ix=[i for i in ids if columns[4][i,0]==flag];values=[]
            if not ix:raise ValueError('empty endpoint evaluation slice')
            for twin in q:
                error=(twin[ix]-y[ix]).double();values.append(dict(count=len(ix),rmse=float(torch.sqrt(torch.mean(error**2))),bias=float(error.mean())))
            result[group+'_'+kind]=values
    return result

def qualifies(metric,baseline):
    return all(metric['holdout_terminal'][i]['rmse']<=1 and abs(metric['holdout_terminal'][i]['bias'])<=.5 and metric['train_terminal'][i]['rmse']<=.5 and metric['holdout_nonterminal'][i]['rmse']<=1.1*baseline['holdout_nonterminal'][i]['rmse']for i in[0,1])

class Budget:
    def __init__(self,stage):
        self.stage=stage;self.path=stage/'ledger.json';self.lock=(stage/'stage.lock').open('a');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB);self.d=json.loads(self.path.read_text())
        if self.d.get('pendingOptimizer')or self.d['status'].startswith('closed'):self.close();raise ValueError('closed/unresolved diagnostic')
        if (dt.datetime.fromisoformat(self.d['deadlineUtc'])-dt.datetime.fromisoformat(self.d['startedUtc'])).total_seconds()!=900 or self.d['optimizerLimit']!=400 or any(self.d[k]!=0 for k in ['physicsSeconds','sourceIntegrations','actorUpdates','targetCriticUpdates']):self.close();raise ValueError('wrong exact diagnostic scope')
    def close(self):
        if self.lock:fcntl.flock(self.lock,fcntl.LOCK_UN);self.lock.close();self.lock=None
    def check(self):
        remaining=(dt.datetime.fromisoformat(self.d['deadlineUtc'])-dt.datetime.now(dt.timezone.utc)).total_seconds()
        if remaining<=120:raise TimeoutError('fixed diagnostic/report reserve')
        if sum(p.stat().st_size for p in self.stage.rglob('*')if p.is_file())>512*1024**2 or shutil.disk_usage(self.stage).free<2*1024**3:raise OSError('diagnostic storage cap')
    def charge(self,arm):
        self.check()
        if arm not in['natural','balanced']or self.d['optimizerCalls']>=400 or self.d['armOptimizerCalls'][arm]>=200 or self.d.get('pendingOptimizer'):raise ValueError('critic call cap/pending')
        self.d['optimizerCalls']+=1;self.d['armOptimizerCalls'][arm]+=1;self.d['pendingOptimizer']=arm;atomic_json(self.path,self.d)
    def complete(self,arm):
        if self.d.get('pendingOptimizer')!=arm:raise ValueError('critic completion mismatch')
        self.d['pendingOptimizer']=None;atomic_json(self.path,self.d);self.check()

def input_hashes(stage):
    c=json.loads(CONFIG.read_text());paths=list(Path(__file__).parent.glob('*.py'))+[CONFIG,ROOT/c['sourceCheckpoint'],ROOT/'scripts/getup-training/SAC.py',ROOT/'rust/crates/sim/data/getup-policy/student.bin',ROOT/'scripts/getup-step/task-config-mujoco-slow-stabilization-v1.json',ROOT/'scripts/getup-step/slow-online-protocol-v1.json',ROOT/'outputs/getup-source-online/session-20261018T030112Z/learner/transitions.ndjson',ROOT/'outputs/getup-source-continuation/session-20261018T032214Z/learner/transitions.ndjson',stage/'fixed-targets.pt',stage/'split.json',stage/'fixtures.json']
    return {str(p.resolve()):sha256(p)for p in paths}

def prepare(stage):
    b=Budget(stage)
    try:
        b.check();fixture=json.loads((stage/'fixtures.json').read_text())
        if not fixture['passed']or sha256(stage/'fixtures.log')!=fixture['logSHA256']:raise ValueError('target/mask fixtures stale')
        c,a,columns,records=load();split=episode_split(records);y,noise=fixed_targets(a,columns,c['targetNoiseSeed']);atomic_json(stage/'split.json',split);torch.save(dict(targets=y,noise=noise,columnsSHA256=[tensor_sha(x)for x in columns]),stage/'fixed-targets.pt');baseline=evaluate(a.critic,columns,y,split)
        admission=dict(schema=1,startedUtc=b.d['startedUtc'],deadlineUtc=b.d['deadlineUtc'],maximumCalls=400,configSHA256=sha256(CONFIG),targetSHA256=tensor_sha(y),noiseSHA256=tensor_sha(noise),terminalTargetsExactlyReward=True,terminalCount=int((columns[4]==0).sum()),trainCount=len(split['train']),holdoutCount=len(split['holdout']),excludedIncomplete=len(split['excluded']),actorSHA256=tensors_sha(a.actor.state_dict()),targetCriticSHA256=tensors_sha(a.critic_target.state_dict()),initialCriticSHA256=tensors_sha(a.critic.state_dict()),initialCriticAdamPopulated=bool(a.critic_optimizer.state_dict()['state']),initialCriticAdamSHA256=nested_sha(a.critic_optimizer.state_dict()),baseline=baseline,inputHashes=input_hashes(stage));atomic_json(stage/'admission.json',admission);print('admissionSHA256',sha256(stage/'admission.json'))
    finally:b.close()

def run(stage,expected):
    if sha256(stage/'admission.json')!=expected:raise ValueError('literal diagnostic admission required')
    receipt=json.loads((stage/'admission.json').read_text())
    for p,h in receipt['inputHashes'].items():
        if sha256(p)!=h:raise ValueError('diagnostic input changed '+p)
    b=Budget(stage);reports=[];stop='incomplete'
    try:
        b.check()
        if b.d['optimizerCalls']or b.d['startedUtc']!=receipt['startedUtc']or b.d['deadlineUtc']!=receipt['deadlineUtc']:raise ValueError('fresh fixed stage identity')
        (stage/'started-once').open('x').close();b.d['status']='running-fixed-critic-only';atomic_json(b.path,b.d);targets=torch.load(stage/'fixed-targets.pt',map_location='cpu',weights_only=True);y=targets['targets']
        if tensor_sha(y)!=receipt['targetSHA256']or tensor_sha(targets['noise'])!=receipt['noiseSHA256']:raise ValueError('fixed target/noise digest mismatch')
        split=json.loads((stage/'split.json').read_text())
        for arm in['natural','balanced']:
            c,a,columns,_=load();rng=np.random.default_rng(c['samplerSeed']);frozen=(tensors_sha(a.actor.state_dict()),tensors_sha(a.critic_target.state_dict()),float(a.alpha));base=evaluate(a.critic,columns,y,split);history=[dict(update=0,metrics=base)];terminal=[i for i in split['train']if columns[4][i,0]==0];nonterminal=[i for i in split['train']if columns[4][i,0]==1];batch_ids=[]
            if frozen[0]!=receipt['actorSHA256']or frozen[1]!=receipt['targetCriticSHA256']or tensors_sha(a.critic.state_dict())!=receipt['initialCriticSHA256']or nested_sha(a.critic_optimizer.state_dict())!=receipt['initialCriticAdamSHA256']or [tensor_sha(v)for v in columns]!=targets['columnsSHA256']:raise ValueError('arms do not share exact starting tensors')
            for n in range(1,201):
                ids=batch_indices(arm,split,columns[4],rng);batch_ids.append(ids.tolist());loss=fit_step(a.critic,a.critic_optimizer,columns[0][ids],columns[1][ids],y[ids],charge=lambda:b.charge(arm),complete=lambda:b.complete(arm))
                if n in[50,100,200]:
                    b.check()
                    if frozen!=(tensors_sha(a.actor.state_dict()),tensors_sha(a.critic_target.state_dict()),float(a.alpha)):raise ValueError('frozen actor/alpha/target changed')
                    history.append(dict(update=n,loss=loss,metrics=evaluate(a.critic,columns,y,split)));atomic_json(stage/(arm+'-progress.json'),history)
            endpoint=history[-1]['metrics'];report=dict(arm=arm,updates=200,endpointQualifies=qualifies(endpoint,base),history=history,batchIDsSHA256=hashlib.sha256(np.array(batch_ids,dtype=np.int64).tobytes()).hexdigest(),frozenActorAlphaTargetUnchanged=True);torch.save(dict(critic=a.critic.state_dict(),critic_optimizer=a.critic_optimizer.state_dict(),batch_ids=batch_ids,samplerState=rng.bit_generator.state),stage/(arm+'-endpoint-local.pt'));reports.append(report);atomic_json(stage/(arm+'-report.json'),report)
        stop='completed-fixed200-per-arm'
    except BaseException as error:stop=type(error).__name__+': '+str(error)
    finally:
        result=dict(stopped=stop,arms=reports,optimizerCalls=b.d['optimizerCalls'],physicsIntegrations=0,actorUpdates=0,targetCriticUpdates=0,policyReleased=False,sourceCheckpointUnmodified=True,admissionSHA256=expected)
        try:
            atomic_json(stage/'summary.json',result);b.d.update(status='closed-fixed-critic-diagnostic',result=stop);atomic_json(b.path,b.d)
        finally:b.close()
        print(json.dumps(result,indent=2))
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('--stage',type=Path,required=True);p.add_argument('--expected-sha',default='');a=p.parse_args();prepare(a.stage)if a.mode=='prepare'else run(a.stage,a.expected_sha)
if __name__=='__main__':main()
