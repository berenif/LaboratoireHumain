import copy,datetime,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from critic_fit import fixed_targets,episode_split,batch_indices,fit_step,Budget
from training import source_module
class StopBeforeStep(Exception):pass
class Tests(unittest.TestCase):
    def agent(self):
        s=source_module();torch.manual_seed(41);return SimpleNamespace(actor=s.Actor(118,21,-5,2),critic_target=s.Critic(118,21),discount=.97,alpha=torch.tensor(.000612579835321518))
    def columns(self):
        torch.manual_seed(9);x=torch.randn(8,118)*.1;x[:,-1]=1;return[x,torch.randn(8,21)*.1,x.clone()+torch.cat([torch.zeros(8,117),torch.zeros(8,1)],-1),torch.randn(8,1),torch.tensor([[0.],[1.],[0.],[1.],[0.],[1.],[0.],[1.]])]
    def test_fixed_source_targets_and_exact_terminal_no_bootstrap(self):
        a=self.agent();v=self.columns();y,n=fixed_targets(a,v,347);torch.manual_seed(347)
        with torch.no_grad():
            _,action,lp,_=a.actor(v[2]);q1,q2=a.critic_target(v[2],action);expected=v[3]+v[4]*.97*(torch.minimum(q1,q2)-a.alpha*lp)
        self.assertTrue(torch.equal(y,expected));self.assertTrue(torch.equal(y[v[4]==0],v[3][v[4]==0]));v[2]=v[2]+100;v[2][:,-1]=1;y2,_=fixed_targets(a,v,939);self.assertTrue(torch.equal(y2[v[4]==0],v[3][v[4]==0]));v[4][0]=.5
        with self.assertRaises(ValueError):fixed_targets(a,v,9)
    def test_episode_separation_and_balanced_sampling(self):
        rows=[dict(episode=e,info=dict(terminated=i==2,truncated=False))for e in range(10)for i in range(3)]+[dict(episode=10,info=dict(terminated=False,truncated=False))];s=episode_split(rows);self.assertEqual(s['excluded'],[30]);self.assertEqual(s['holdoutEpisodes'],[0,5]);mask=torch.tensor([[i%2]for i in range(31)])
        ids=batch_indices('balanced',s,mask,np.random.default_rng(1));self.assertEqual(int((mask[ids]==0).sum()),64);self.assertEqual(int((mask[ids]==1).sum()),64);self.assertTrue(set(ids)<=set(s['train']));self.assertEqual(batch_indices('natural',s,mask,np.random.default_rng(2)).shape,(128,))
    def test_source_critic_mse_gradients_before_any_optimizer(self):
        s=source_module();torch.manual_seed(54);a=s.Critic(118,21);b=copy.deepcopy(a);v=self.columns();target=torch.arange(8,dtype=torch.float32)[:,None];q1,q2=b(v[0],v[1]);loss=torch.nn.functional.mse_loss(q1,target)+torch.nn.functional.mse_loss(q2,target);loss.backward()
        class Opt:
            def zero_grad(self,**_):
                for p in a.parameters():p.grad=None
            def step(self):raise AssertionError('must not step')
        def charge():raise StopBeforeStep()
        with self.assertRaises(StopBeforeStep):fit_step(a,Opt(),v[0],v[1],target,charge=charge,complete=lambda:None)
        self.assertTrue(all(torch.equal(p.grad,q.grad)for p,q in zip(a.parameters(),b.parameters())))
    def test_durable_caps_pending_and_exact_scope(self):
        with tempfile.TemporaryDirectory()as tmp:
            p=Path(tmp);now=datetime.datetime.now(datetime.timezone.utc);d=dict(startedUtc=now.isoformat(),deadlineUtc=(now+datetime.timedelta(seconds=900)).isoformat(),optimizerLimit=400,optimizerCalls=0,armOptimizerCalls=dict(natural=0,balanced=0),pendingOptimizer=None,status='setup',physicsSeconds=0,sourceIntegrations=0,actorUpdates=0,targetCriticUpdates=0);(p/'ledger.json').write_text(json.dumps(d));b=Budget(p);b.charge('natural');self.assertEqual(json.loads((p/'ledger.json').read_text())['pendingOptimizer'],'natural')
            with self.assertRaises(ValueError):b.charge('balanced')
            b.complete('natural');b.d['armOptimizerCalls']['natural']=200
            with self.assertRaises(ValueError):b.charge('natural')
            b.close();d['pendingOptimizer']='balanced';(p/'ledger.json').write_text(json.dumps(d))
            with self.assertRaises(ValueError):Budget(p)
if __name__=='__main__':unittest.main()
