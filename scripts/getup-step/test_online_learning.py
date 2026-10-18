import copy,unittest
import numpy as np
import torch
from online_learning import initialize,Roles,sample_action,OnlineReplay,Features,trunk_sha,loss,divergence,propose,state,restore,episode_returns,ALPHA
from training import critic_only_update

def equal(a,b):
    if isinstance(a,torch.Tensor):return torch.equal(a,b)
    if isinstance(a,np.ndarray):return np.array_equal(a,b)
    if isinstance(a,dict):return a.keys()==b.keys()and all(equal(a[k],b[k])for k in a)
    if isinstance(a,(list,tuple)):return len(a)==len(b)and all(equal(x,y)for x,y in zip(a,b))
    return a==b

class OnlineTests(unittest.TestCase):
    def setup(self):
        a,o=initialize();roles=Roles('fixture');x=torch.zeros(16,118);x[:,-1]=1;return a,o,roles,x
    def proposal(self,a,o,roles,x,**kw):
        calls=[];r=propose(a,o,x[:8],x,roles.proposal(roles.attempt_id+1),charge=lambda:calls.append('charge'),complete=lambda:calls.append('done'),check=lambda:None,**kw);self.assertEqual(calls,['charge','done']);return r
    def replay(self,a,roles,x):
        r=OnlineReplay();r.rng=roles.replay
        for i in range(len(x)):
            action,b=sample_action(a.actor,x[i:i+1],roles,0);r.add(i+1,dict(state=x[i].numpy(),action=action,next_state=x[(i+1)%len(x)].numpy(),reward=1.,not_done=1.,behaviour=b,info=dict(nativeSampleCount=20,physicalFailure=None,success=False)))
        return r
    def test_feature_forward_loss_and_both_head_gradients(self):
        a,o,roles,x=self.setup();f=Features(a.actor,x,trunk_sha(o));self.assertTrue(torch.equal(f.raw(a.actor),a.actor.trunk(x)))
        rng=torch.get_rng_state().clone();_,pi,lp,_=a.actor(x);q1,q2=a.critic(x,pi);expected=(a.alpha.detach()*lp-torch.minimum(q1,q2)).mean();eg=torch.autograd.grad(expected,list(a.actor.trunk[4].parameters()))
        torch.set_rng_state(rng);noise=torch.randn((len(x),21));actual=loss(a,f,noise);ag=torch.autograd.grad(actual,list(a.actor.trunk[4].parameters()))
        self.assertTrue(torch.equal(expected,actual));self.assertTrue(all(torch.equal(v,w)for v,w in zip(eg,ag)));self.assertGreater(float(ag[0][21:].abs().max()),0)
        f.hidden.add_(0)
        with self.assertRaises(ValueError):f.raw(a.actor)
    def test_full_kl_and_std_update(self):
        a,o,roles,x=self.setup();before=copy.deepcopy(a.actor.state_dict());r=self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10)
        self.assertTrue(r['accepted']);self.assertGreater(r['proposedStdParameterMaximumChange'],0);self.assertFalse(torch.equal(a.actor.trunk[4].weight[21:],before['trunk.4.weight'][21:]));self.assertEqual(trunk_sha(a.actor),trunk_sha(o))
        z=torch.zeros(2,21);ls=z+np.log(.2);v=divergence(z,ls,z+.01,z+np.log(.18));expected=21*(np.log(.18/.2)+(.2**2+.01**2)/(2*.18**2)-.5);self.assertTrue(np.allclose(v.numpy(),expected,rtol=1e-5))
    def test_rejection_does_not_condition_collection_or_critic_streams(self):
        a,o,roles,x=self.setup();replay=self.replay(a,roles,x)
        with roles.critic_scope():critic_only_update(a,replay,8,before_optimizer_step=lambda:None)
        self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10)
        expected=Roles.restore(roles.state());head=copy.deepcopy(a.actor.state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict());result=self.proposal(a,o,roles,x,minimum_decrease=1e6)
        self.assertFalse(result['accepted']);self.assertEqual(len(result['trials']),11);self.assertTrue(all(torch.equal(v,head[k])for k,v in a.actor.state_dict().items()))
        aa,bb=sample_action(a.actor,x[:1],roles,0);cc,dd=sample_action(a.actor,x[:1],expected,0);np.testing.assert_array_equal(aa,cc);self.assertEqual(bb,dd);self.assertTrue(torch.equal(roles.critic.get_state(),expected.critic.get_state()))
        self.assertTrue(equal(opt,a.actor_optimizer.state_dict()));self.assertTrue(equal(result['_evidence']['retainedAdam'],opt));self.assertFalse(equal(result['_evidence']['proposedAdam'],opt));self.assertNotEqual(roles.proposal(3).initial_seed(),expected.proposal(2).initial_seed())
    def test_populated_full_stochastic_next_collection_critic_proposal_restore(self):
        a,o,roles,x=self.setup();replay=self.replay(a,roles,x)
        with roles.critic_scope():critic_only_update(a,replay,8,before_optimizer_step=lambda:None)
        self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10);saved=state(a,replay,roles)
        def nextstep(a,o,replay,roles):
            action,b=sample_action(a.actor,x[:1],roles,1)
            with roles.critic_scope():critic_only_update(a,replay,8,before_optimizer_step=lambda:None)
            result=self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10)
            return action,b,result,state(a,replay,roles)
        one=nextstep(a,o,replay,roles);aa,oo,rr,gg=restore(saved);two=nextstep(aa,oo,rr,gg);self.assertTrue(equal(one,two))
    def test_failure_masks_and_logged_soft_returns(self):
        a,o,roles,x=self.setup();replay=OnlineReplay();replay.rng=roles.replay;action,b=sample_action(a.actor,x[:1],roles,0)
        t=dict(state=x[0].numpy(),action=action,next_state=x[0].numpy(),reward=-10.,not_done=1.,behaviour=b,info=dict(nativeSampleCount=20,physicalFailure='loaded_slip',success=False))
        with self.assertRaises(ValueError):replay.add(1,t)
        self.assertFalse(replay.items);t['not_done']=0.;replay.add(1,t);self.assertEqual(len(OnlineReplay.restore(replay.state()).items),1)
        rows=[dict(reward=1.,behaviour=b,info=dict(terminated=False,truncated=False)),dict(reward=-10.,behaviour=b,info=dict(terminated=True,truncated=False))];r=episode_returns(rows);self.assertAlmostEqual(r['taskOnly'][0],1-.97*10);self.assertAlmostEqual(r['sourceSoft'][0],1+.97*(-10-ALPHA*b['sourceEpsilonLogProbability']))
        rows[-1]['info']['truncated']=True;self.assertIsNone(episode_returns(rows))
    def test_scaled_both_head_acceptance_keeps_full_adam(self):
        a,o,roles,x=self.setup();replay=self.replay(a,roles,x);saved=state(a,replay,roles);before=copy.deepcopy(a.actor.trunk[4].state_dict())
        full=self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10);after=copy.deepcopy(a.actor.trunk[4].state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict())
        a,o,replay,roles=restore(saved);half=self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=full['trials'][0]['maximumStepKL']*.3,mean_limit=10)
        self.assertEqual(half['scale'],.5);self.assertTrue(equal(opt,a.actor_optimizer.state_dict()))
        for k,v in a.actor.trunk[4].state_dict().items():self.assertTrue(torch.equal(v,before[k]+.5*(after[k]-before[k])))
    def test_interrupted_line_search_restores_populated_both_head_state(self):
        a,o,roles,x=self.setup();self.proposal(a,o,roles,x,minimum_decrease=-100,kl_limit=10,mean_limit=10);head=copy.deepcopy(a.actor.trunk[4].state_dict());adam=copy.deepcopy(a.actor_optimizer.state_dict());calls=[];checks=[]
        def fail():
            checks.append(1)
            if len(checks)==2:raise KeyboardInterrupt('projection interrupted')
        with self.assertRaises(KeyboardInterrupt):propose(a,o,x[:8],x,roles.proposal(2),charge=lambda:calls.append('charged'),complete=lambda:calls.append('optimizer-finished'),check=fail)
        self.assertEqual(calls,['charged','optimizer-finished']);self.assertTrue(equal(head,a.actor.trunk[4].state_dict()));self.assertTrue(equal(adam,a.actor_optimizer.state_dict()));self.assertEqual(roles.attempt_id,2)
    def test_bad_collection_input_before_noise_and_foreign_feature_trunk(self):
        a,o,roles,x=self.setup();before=roles.collection.get_state().clone()
        for row,version in [(x[:2],0),(x[:1].double(),0),(x[:1],True),(x[:1],995),(torch.full((1,118),float('nan')),0)]:
            with self.assertRaises(ValueError):sample_action(a.actor,row,roles,version)
            self.assertTrue(torch.equal(before,roles.collection.get_state()))
        f=Features(a.actor,x,trunk_sha(a.actor))
        with torch.no_grad():o.trunk[0].bias.add_(.001)
        with self.assertRaises(ValueError):f.raw(o)

if __name__=='__main__':unittest.main()
