#!/usr/bin/env python3
"""One local bounded frozen-feature SAC pilot in the actual WASM recovery world."""
import argparse,copy,hashlib,importlib.util,json,math,os,random,select,subprocess,time
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'scripts/getup-training/SAC.py'
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()=='64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420'
spec=importlib.util.spec_from_file_location('source_sac',SOURCE);upstream=importlib.util.module_from_spec(spec);spec.loader.exec_module(upstream)
SEED=20261020
class CorrectionActor(nn.Module):
    def __init__(self):
        super().__init__();self.mean=nn.Linear(1024,21);self.log_std=nn.Parameter(torch.full((21,),-3.))
        nn.init.zeros_(self.mean.weight);nn.init.zeros_(self.mean.bias)
    def forward(self,x,sample=True):
        mean=self.mean(x);ls=self.log_std.clamp(-5.,-2.).expand_as(mean);noise=torch.randn_like(mean) if sample else torch.zeros_like(mean)
        pre=mean+noise*ls.exp();c=pre.tanh();logp=upstream.gaussian_logprob(noise,ls)-torch.log(1-c.square()+1e-6).sum(-1,keepdim=True)
        return mean.tanh(),c,logp
    def exported(self):
        return {'weights':(self.mean.weight.detach().numpy()/32.).astype(np.float32).ravel().tolist(),'bias':self.mean.bias.detach().numpy().tolist()}
class Critic(nn.Module):
    def __init__(self):
        super().__init__();make=lambda:nn.Sequential(nn.Linear(1045,256),nn.ReLU(),nn.Linear(256,256),nn.ReLU(),nn.Linear(256,1))
        self.q1=make();self.q2=make()
    def forward(self,s,c):
        x=torch.cat((s,c),-1);return self.q1(x),self.q2(x)
class Replay:
    def __init__(self,cap,rng):self.rows=[];self.cap=cap;self.rng=rng;self.success=[]
    def add(self,s,c,n,r,terminal,success=False):
        assert len(self.rows)<self.cap
        row=(np.asarray(s,np.float32)/32.,np.asarray(c,np.float32),np.asarray(n,np.float32)/32.,[r],[not terminal])
        self.rows.append(row)
        if success:self.success.append(len(self.rows)-1)
    def sample(self,n):
        assert self.success and n%4==0
        ids=list(self.rng.choice(self.success,n//4))+list(self.rng.integers(len(self.rows),size=3*n//4))
        return tuple(torch.tensor(np.asarray([self.rows[i][j] for i in ids]),dtype=torch.float32) for j in range(5))
    def save(self,path):np.savez_compressed(path,**{str(i):np.asarray([r[i] for r in self.rows],dtype=np.float32) for i in range(5)},success=np.asarray(self.success))
class SourceTransition:
    """Close the final source action across unchanged standing-controller tail."""
    def __init__(self,gamma):self.gamma=gamma;self.pending=None
    def push(self,state,c,nxt):
        pre=state.get('preview');post=nxt['state'].get('preview');row=None
        if self.pending:
            assert pre is None and post is None,'Source preview unexpectedly reappeared during standing tail'
            self.pending[3]+=self.gamma**self.pending[5]*nxt['reward'];self.pending[5]+=1
            if nxt['terminal']:
                row=tuple(self.pending[:3])+ (self.pending[3],True);self.pending=None
        elif pre:
            if post or nxt['terminal']:
                row=(pre['features'],c,post['features'] if post else np.zeros(1024,np.float32),nxt['reward'],nxt['terminal'])
            else:self.pending=[pre['features'],c,np.zeros(1024,np.float32),nxt['reward'],False,1]
        return row
class Wasm:
    def __init__(self,out,worker):
        self.out=out;self.proc=subprocess.Popen(['node',str(ROOT/'scripts/getup-wasm-adaptation-protocol.mjs'),str(out),str(worker)],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'protocol-stderr.log').open('w'),text=True,bufsize=1)
    def call(self,q):
        self.proc.stdin.write(json.dumps(q)+'\n');self.proc.stdin.flush()
        if not select.select([self.proc.stdout],[],[],120)[0]:raise TimeoutError('WASM response timeout; no request retry')
        line=self.proc.stdout.readline();assert line,'WASM bridge closed'
        v=json.loads(line)
        if not v['ok']:raise RuntimeError(json.dumps(v))
        return v['result']
    def close(self):self.proc.stdin.close();self.proc.wait(timeout=10)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--worker',type=Path,required=True);p.add_argument('--qualify-only',action='store_true');args=p.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True);cfg=json.loads((out/'frozen-config.json').read_text())
    prior=json.loads((out/'budget-ledger.json').read_text()) if (out/'budget-ledger.json').exists() else {}
    if prior.get('trainingDecisions',0)>0 or (out/'candidate-training.pt').exists():
        raise RuntimeError('This bounded pilot has already collected training data; no experiment restart or second candidate is supported.')
    random.seed(SEED);np.random.seed(SEED);torch.manual_seed(SEED);torch.set_num_threads(2);rng=np.random.default_rng(SEED)
    actor=CorrectionActor();critic=Critic();target=copy.deepcopy(critic);ao=torch.optim.Adam(actor.parameters(),lr=1e-4);co=torch.optim.Adam(critic.parameters(),lr=1e-4)
    logalpha=torch.tensor(math.log(.01),requires_grad=True);alphaopt=torch.optim.Adam([logalpha],lr=1e-4)
    replay=Replay(cfg['replayCapacity'],rng);report={'schema':1,'seed':SEED,'sourceSha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'configSha256':hashlib.sha256((out/'frozen-config.json').read_bytes()).hexdigest(),'trainingDecisions':0,'criticOnlyUpdates':0,'jointUpdates':0,'releaseAccepted':False,'candidateEnabledByDefault':False}
    wasm=Wasm(out,args.worker.resolve());started=time.monotonic()
    def save():
        report['elapsedWallSeconds']=time.monotonic()-started;(out/'pilot-report.json').write_text(json.dumps(report,indent=2)+'\n')
    def evaluate(case,label,head=None,preserve=False):
        state=wasm.call({'op':'reset','case':case,'label':label,'head':head,'prepared':case=='belly' and (out/'belly-seed.bin').exists()})
        pending=[];decisions=0;builder=SourceTransition(cfg["gamma"])
        while True:
            nxt=wasm.call({'op':'step'});decisions+=1
            if nxt['failure'] and nxt['failure'].get('check')!='original25s-deadline':raise RuntimeError('Physical integrity failure during '+label+': '+json.dumps(nxt['failure']))
            if head and state.get('preview'):
                with torch.no_grad():expected=actor(torch.tensor(state['preview']['features'],dtype=torch.float32).reshape(1,-1)/32.,sample=False)[0][0].numpy()
                receipt=nxt['state']['lastCorrection'];assert receipt['native_substep']==state['native']
                error=float(np.max(np.abs(np.asarray(receipt['correction'],np.float32)-expected)))
                assert error<=2e-6,'WASM correction-head export parity failed: '+str(error)
                report['exportParityMaxCorrectionError']=max(report.get('exportParityMaxCorrectionError',0.),error)
            transition=builder.push(state,[0.]*21,nxt)
            if transition:pending.append(transition)
            state=nxt['state']
            if nxt['terminal']:break
            assert decisions<=1100,'Original physical deadline must terminate every case'
        result={'case':case,'decisions':decisions,'success':nxt['success'],'failure':nxt['failure'],'longestMeasuredInterval':nxt['longestMeasuredInterval'],'native':nxt['native']}
        if case=='belly':result['thirtySecondHold']=wasm.call({'op':'hold'})
        if preserve:
            assert result['success'],'Frozen belly baseline regression/invalid configuration stops pilot'
            for row in pending:replay.add(*row,success=True)
        wasm.call({'op':'checkpoint','name':label+'-final.bin'});return result
    try:
        report['identity']=wasm.call({'op':'identity'})
        if (out/'adapter-parity.json').exists():report['adapterParity']=json.loads((out/'adapter-parity.json').read_text())
        else:report['adapterParity']=wasm.call({'op':'qualify'})
        save()
        # Retained one-shot heldout is never regenerated after an interrupted harness.
        if (out/'heldout-admission.json').exists():report['heldoutAdmission']=json.loads((out/'heldout-admission.json').read_text())
        else:report['heldoutAdmission']=wasm.call({'op':'heldout'})
        save()
        if args.qualify_only:report['stopped']='qualification-only';return
        report['baseline']=[evaluate('original','baseline-original'),evaluate('heldout','baseline-heldout'),evaluate('belly','baseline-belly',preserve=True)];save()
        ledger=wasm.call({'op':'ledger'});remaining=ledger['deadlineUnixMs']/1000-time.time()
        global_elapsed=time.time()-__import__('datetime').datetime.fromisoformat(ledger['startedAt'].replace('Z','+00:00')).timestamp()
        rate=(ledger['realIntegrations']+ledger['forecastIntegrations'])/max(.001,global_elapsed)
        # Full final eval reserve includes two25s BACKs, belly25s worstcase,3*5000forecasts and initialization.
        reserve=102000;collection=cfg['trainingDecisionCap']*20+9*5000
        report['capacityCheck']={'measuredNativePerWallSecond':rate,'remainingWallSeconds':remaining,'reservedEvaluationIntegrations':reserve,'maximumCollectionIntegrations':collection,'remainingIntegrations':ledger['limit']-ledger['nativeIntegrations']}
        assert ledger['nativeIntegrations']+reserve+collection<=ledger['limit'],'Insufficient integration capacity before updates'
        assert (reserve+collection)/rate+180<remaining,'Insufficient measured wall capacity before updates'
        report['evaluationReserve']=wasm.call({'op':'reserve','integrations':reserve,'wallSeconds':reserve/rate+180})
        state=None;builder=SourceTransition(cfg["gamma"])
        while report['trainingDecisions']<cfg['trainingDecisionCap'] or builder.pending:
            if state is None:state=wasm.call({'op':'reset','case':'original','label':'training-original','external':True})
            features=state.get('preview')
            if features:
                with torch.no_grad():c=actor(torch.tensor(features['features'],dtype=torch.float32).reshape(1,-1)/32.)[1][0].numpy().tolist()
            else:c=None
            nxt=wasm.call({'op':'step','c':c,'training':bool(features)})
            if features:report['trainingDecisions']+=1
            transition=builder.push(state,c,nxt)
            if transition:replay.add(*transition)
            state=None if nxt['terminal'] else nxt['state']
            if nxt['failure'] and nxt['failure'].get('check')!='original25s-deadline':raise RuntimeError('Integrity failure during collection: '+json.dumps(nxt['failure']))
            if report['trainingDecisions']%256==0:save()
        # Administrative collection cutoff is not a physical terminal; rows retain bootstrap.
        report['collectionTruncatedAdministratively']=state is not None;report['replayEntries']=len(replay.rows);report['preservationEntries']=len(replay.success);replay.save(out/'replay.npz');save()
        metrics=[]
        for update in range(cfg['criticOnlyUpdates']+cfg['jointUpdates']):
            assert time.time()+reserve/rate+180<ledger['deadlineUnixMs']/1000,'Evaluation wall reserve exhausted'
            s,c,n,r,nd=replay.sample(cfg['batchSize'])
            with torch.no_grad():
                _,nc,nlogp=actor(n);q1,q2=target(n,nc);y=r+nd*cfg['gamma']*(torch.minimum(q1,q2)-logalpha.exp().detach()*nlogp)
            q1,q2=critic(s,c);cl=F.mse_loss(q1,y)+F.mse_loss(q2,y);co.zero_grad();cl.backward();torch.nn.utils.clip_grad_norm_(critic.parameters(),10.);co.step()
            row={'update':update,'critic':float(cl)}
            if update>=cfg['criticOnlyUpdates']:
                mu,pc,lp=actor(s);a,b=critic(s,pc);preservation=mu[:cfg['batchSize']//4].square().mean()
                al=(logalpha.exp().detach()*lp-torch.minimum(a,b)).mean()+10.*preservation
                ao.zero_grad();al.backward();torch.nn.utils.clip_grad_norm_(actor.parameters(),1.);ao.step()
                tl=(logalpha.exp()*(-lp-(-21)).detach()).mean();alphaopt.zero_grad();tl.backward();alphaopt.step()
                report['jointUpdates']+=1;row.update(actor=float(al),preservation=float(preservation),alpha=float(logalpha.exp()))
            else:report['criticOnlyUpdates']+=1
            with torch.no_grad():
                for a,b in zip(critic.parameters(),target.parameters()):b.lerp_(a,.005)
            assert all(math.isfinite(v) for v in row.values()) and all(torch.isfinite(v).all() for v in actor.parameters()),'Nonfinite update'
            metrics.append(row)
        head=actor.exported();(out/'candidate-head.json').write_text(json.dumps(head)+'\n');torch.save({'actor':actor.state_dict(),'critic':critic.state_dict(),'target':target.state_dict(),'actorOptimizer':ao.state_dict(),'criticOptimizer':co.state_dict(),'logAlpha':logalpha.detach(),'alphaOptimizer':alphaopt.state_dict(),'torchRng':torch.get_rng_state(),'numpyRng':rng.bit_generator.state,'config':cfg,'report':report},out/'candidate-training.pt')
        (out/'updates.json').write_text(json.dumps(metrics,indent=2)+'\n');save()
        parity=[]
        for source_row in replay.rows[:32]:
            features=(source_row[0]*32.).astype(np.float32)
            with torch.no_grad():expected=actor(torch.from_numpy(source_row[0]).reshape(1,-1),sample=False)[0][0].numpy()
            actual=wasm.call({'op':'headparity','head':head,'features':features.tolist()})
            error=float(np.max(np.abs(expected-np.asarray(actual['correction'],np.float32))))
            assert actual['physicsIntegrations']==0 and error<=2e-6,'Pure WASM nonzero head export parity failed'
            parity.append({'featuresSha256':hashlib.sha256(features.tobytes()).hexdigest(),'expected':expected.tolist(),'actual':actual['correction'],'maxError':error})
        report['pureExportParityMaxError']=max(row['maxError'] for row in parity)
        (out/'head-export-parity.json').write_text(json.dumps(parity,indent=2)+'\n')
        wasm.call({'op':'reserve','integrations':0,'wallSeconds':0})
        report['final']=[evaluate('original','candidate-original',head),evaluate('heldout','candidate-heldout',head),evaluate('belly','candidate-belly',head)]
        report['firstSuccessCensoredPhysicalSignal']=all(report['final'][i]['longestMeasuredInterval']>=report['baseline'][i]['longestMeasuredInterval']+80 for i in (0,1)) and report['final'][2]['success'] and report['final'][2]['failure'] is None
        report['stopped']='single-checkpoint-finished-fixed25-audit-required'
        report['physicalSignal']=None
    except Exception as e:
        report['stopped']='blocked-or-integrity-stop';report['error']=repr(e);raise
    finally:
        try:report['finalLedger']=wasm.call({'op':'ledger'})
        except Exception as e:report['ledgerError']=repr(e)
        save();wasm.close()
if __name__=='__main__':main()
