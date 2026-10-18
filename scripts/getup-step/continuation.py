"""Exact retained-training restore and fresh continuation resource boundary."""
import copy,hashlib,json
from pathlib import Path
import numpy as np
import torch
from native_env import ROOT,atomic_json,sha256
from online_learning import restore,Roles,sample_action,tensor_sha
from online_io import Guard
from slow_task import SourceSLOWWorld,SourceSLOWEnv
CONFIG=Path(__file__).with_name('slow-online-continuation-v1.json')
CONFIG_SHA='3886425eb10f962c6f1fa06838d89ed4a1b9422963ca4d57456789ae95bcfda0'
PLAN_SHA='2b28d8eb568dd081ced100f8fca42da6863509f9ccf7abd1aa7903c0916086e8'
def config():
    if sha256(CONFIG)!=CONFIG_SHA or sha256(ROOT/'docs/experiments/getup-python-step-plan.md')!=PLAN_SHA:raise ValueError('reviewed continuation config/plan changed')
    return json.loads(CONFIG.read_text())
def retained():
    c=config();r=c['resume'];p=Path(r['manifest']);m=json.loads(p.read_text())
    for key in ['native','learner','controller']:
        expected=r[key+'SHA256']
        if m[key+'SHA256']!=expected or sha256(m[key])!=expected:raise ValueError('retained TRAINING checkpoint hash mismatch')
    ledger=Path(r['closedPriorLedger']);d=json.loads(ledger.read_text())
    if sha256(ledger)!=r['closedPriorLedgerSHA256']or not d['status'].startswith('closed')or d['candidateOptimizerCalls']!=1278 or d['sourceIntegrations']!=35654:raise ValueError('old charges not preserved/closed')
    payload=torch.load(m['learner'],map_location='cpu',weights_only=True);a,o,replay,roles=restore(payload['learning']);outer=json.loads(Path(m['controller']).read_text());side=outer['source'];native=Path(m['native']).read_bytes()
    if outer['nativeSHA256']!=r['nativeSHA256']or outer['learnerSHA256']!=r['learnerSHA256']or len(replay.items)!=1580 or a.total_it!=139 or roles.attempt_id!=139 or a.critic_warmup_it!=1139:raise ValueError('retained cumulative learner counters mismatch')
    old_commands=[json.loads(s)for s in(p.parent/'transitions.ndjson').read_text().splitlines()];episode=old_commands[-1]['episode'];prefix=[row for row in old_commands if row['episode']==episode]
    if episode!=48 or len(prefix)!=7 or side['core']['state']['terminal']or side['core']['state']['native_id']!=6540:raise ValueError('unfinished retained episode mismatch')
    return c,m,a,o,replay,roles,native,side,prefix,json.loads((p.parent/'episodes.json').read_text())
def native_resume(source,native,side):
    w=SourceSLOWWorld(source)
    try:
        w.restore(native);model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());e=SourceSLOWEnv(w,model,bindings=copy.deepcopy(side['core']['bindings']),rng=np.random.default_rng(0),original_entry_packet=side['core']['entryPacket']);e.restore(native,side);return w,e,model
    except BaseException:w.close();raise
class ContinuationGuard(Guard):
    def charge(self,kind):
        self.check();d=self.budget.data
        if kind not in ['continuation-critic','continuation-head']or d.get(kind,0)>=855 or d.get('pendingOptimizer')or d.get('proposalInProgress')or d['optimizerCalls']>=1710:raise ValueError('new continuation optimizer cap/pending')
        d[kind]=d.get(kind,0)+1;d['candidateOptimizerCalls']+=1;d['optimizerCalls']+=1;d['pendingOptimizer']=kind
        if kind.endswith('-head'):d['proposalInProgress']=kind
        atomic_json(self.budget.path,d)
def restore_receipt(source):
    c,m,a,o,replay,roles,native,side,prefix,episodes=retained();world=None
    try:
        world,env,_=native_resume(source,native,side);counter=world.dim(6);packet=world.task_packet();blob,roundtrip=env.checkpoint();x=torch.tensor(env.core.observation(world.observation()),dtype=torch.float32)[None,:]
        if blob!=native or roundtrip!=side or not np.array_equal(x[0].numpy(),replay.items[-1][2]):raise ValueError('native/controller/replay zero-step mismatch')
        states=copy.deepcopy(roles.state());action,b=sample_action(a.actor,x,roles,11);indices=roles.replay.integers(0,1584,128);noise=torch.randn((128,21),generator=roles.proposal(140));_,_,aa,oo,rr,gg,_,_,_,_=retained();act2,b2=sample_action(aa.actor,x,gg,11);ids2=gg.replay.integers(0,1584,128);noise2=torch.randn((128,21),generator=gg.proposal(140))
        if not np.array_equal(action,act2)or b!=b2 or not np.array_equal(indices,ids2)or not torch.equal(noise,noise2)or world.dim(6)!=counter:raise ValueError('next action/batch-index/attempt140 tape restore mismatch')
        return dict(passed=True,newIntegrations=0,newOptimizerCalls=0,retainedNativeCounter=counter,nativeBytesRoundTripExact=True,controllerEnvelopeRoundTripExact=True,nextObservation118MatchesReplay=True,nextGaussianAction=action.tolist(),nextActionNoiseSHA256=hashlib.sha256(np.array(b['noise'],dtype=np.float32).tobytes()).hexdigest(),nextReplayIndicesAtFutureSize1584SHA256=hashlib.sha256(indices.tobytes()).hexdigest(),nextProposal140NoiseSHA256=tensor_sha(noise),oldUnfinishedEpisode=48,oldEpisodeRowsCarried=7,oldClock=side['core']['state']['clock'],oldSlip=side['core']['state']['slip'],oldBindingsPreserved=side['core']['bindings'],newRejectionStreak=0,oldAnchorRejectionsPreserved=128)
    finally:
        if world:world.close()
