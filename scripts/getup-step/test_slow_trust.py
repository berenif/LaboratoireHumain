import copy,unittest
import numpy as np
import torch
from slow_trust import initialize,gaussian,proposal,drift,ALPHA,trust_kl,restore,fixed_noise_loss
from slow_learning import SlowReplay,learner_state

class TrustTests(unittest.TestCase):
    def setup(self):
        agent,original=initialize();x=torch.zeros(8,118);x[:,-1]=1
        with torch.no_grad():mu,ls=gaussian(original,x)
        return agent,original,x,mu,ls.exp(),copy.deepcopy(original.state_dict())
    def step(self,a,x,mu,std,anchor,**kw):
        calls=[]
        r=proposal(a,x,x,mu,std,anchor,charge=lambda:calls.append('charge'),complete=lambda:calls.append('done'),check_wall=lambda:None,**kw)
        self.assertEqual(calls,['charge','done']);return r
    def test_original_distribution_and_kl(self):
        a,o,x,mu,std,anchor=self.setup()
        self.assertTrue(torch.equal(a.actor(x,False,False)[0],o(x,False,False)[0]))
        self.assertTrue(torch.equal(a.actor(x,False,False)[3],o(x,False,False)[3]))
        self.assertAlmostEqual(float(a.alpha),ALPHA,places=10)
        with torch.no_grad():a.actor.trunk[4].bias[:21]+=.001
        actual=trust_kl(a.actor,x,mu,std)
        expected=.5*torch.sum((.001/std)**2,-1)
        self.assertTrue(torch.allclose(actual,expected,rtol=1e-4,atol=1e-7))
    def test_rejection_populated_adam_and_rng_rollback(self):
        a,o,x,mu,std,anchor=self.setup();self.step(a,x,mu,std,anchor,kl_limit=10,minimum_decrease=-100)
        before=copy.deepcopy(a.actor.state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict());rng=torch.get_rng_state().clone()
        r=self.step(a,x,mu,std,anchor,minimum_decrease=1e6)
        self.assertFalse(r['accepted']);self.assertEqual(len(r['trials']),11)
        self.assertTrue(all(torch.equal(v,before[k])for k,v in a.actor.state_dict().items()));self.assertTrue(torch.equal(torch.get_rng_state(),rng))
        for key,item in opt['state'].items():
            for k,v in item.items():self.assertTrue(torch.equal(v,a.actor_optimizer.state_dict()['state'][key][k]))
    def test_full_state_next_proposal_continuation(self):
        a,o,x,mu,std,anchor=self.setup();self.step(a,x,mu,std,anchor,kl_limit=10,minimum_decrease=-100)
        replay=SlowReplay();state=learner_state(a,replay)
        one=self.step(a,x,mu,std,anchor,kl_limit=10,minimum_decrease=-100);after=copy.deepcopy(a.actor.state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict())
        restored,_=initialize();restore(restored,state)
        two=self.step(restored,x,mu,std,anchor,kl_limit=10,minimum_decrease=-100)
        self.assertEqual(one,two);self.assertTrue(all(torch.equal(v,after[k])for k,v in restored.actor.state_dict().items()))
        for key,item in opt['state'].items():
            for k,v in item.items():self.assertTrue(torch.equal(v,restored.actor_optimizer.state_dict()['state'][key][k]))
    def test_scaled_acceptance_keeps_full_adam_but_scaled_parameters(self):
        a,o,x,mu,std,anchor=self.setup();state=learner_state(a,SlowReplay());before=copy.deepcopy(a.actor.state_dict())
        full=self.step(a,x,mu,std,anchor,kl_limit=10,minimum_decrease=-100);after=copy.deepcopy(a.actor.state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict());rng=torch.get_rng_state().clone()
        restore(a,state);scaled=self.step(a,x,mu,std,anchor,kl_limit=full['trials'][0]['maximumKL']*.3,minimum_decrease=-100)
        self.assertEqual(scaled['scale'],.5);self.assertTrue(torch.equal(torch.get_rng_state(),rng))
        for k,v in a.actor.state_dict().items():self.assertTrue(torch.equal(v,before[k]+.5*(after[k]-before[k])))
        for key,item in opt['state'].items():
            for k,v in item.items():self.assertTrue(torch.equal(v,a.actor_optimizer.state_dict()['state'][key][k]))
    def test_fixed_noise_loss_matches_unchanged_source_actor(self):
        a,o,x,mu,std,anchor=self.setup();rng=torch.get_rng_state().clone()
        _,action,logp,_=a.actor(x);q1,q2=a.critic(x,action);expected=(a.alpha.detach()*logp-torch.minimum(q1,q2)).mean()
        torch.set_rng_state(rng);noise=torch.randn((len(x),21));actual=fixed_noise_loss(a,x,noise)
        self.assertTrue(torch.equal(actual,expected));self.assertFalse(a.log_alpha.requires_grad)

if __name__=='__main__':unittest.main()
