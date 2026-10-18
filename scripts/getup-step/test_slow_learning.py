import copy,unittest,numpy as np
from slow_learning import SlowReplay,learner_state,restore_learner,charged_source_update
class SlowLearningTests(unittest.TestCase):
    def test_empty_replay_roundtrip(self):
        r=SlowReplay();other=SlowReplay.restore(r.state());self.assertEqual(other.items,[])
    def test_residual_replay_and_partial_masks(self):
        r=SlowReplay();t=dict(state=np.r_[np.zeros(117),1],action=np.linspace(-1,1,21),next_state=np.r_[np.ones(117)*.1,1],reward=-3.,not_done=0.,info=dict(nativeSampleCount=3,physicalFailure='flight'));r.add(1,t);self.assertEqual(r.sample(2)[1].shape,(2,21));t['not_done']=1
        with self.assertRaises(ValueError):r.add(2,t)
        t['not_done']=0;t['action']=np.ones(21)*2
        with self.assertRaises(ValueError):r.add(2,t)
        self.assertEqual(len(r.items),1)
    def test_full_interval_physical_failure_and_goal_never_bootstrap(self):
        r=SlowReplay();t=dict(state=np.r_[np.zeros(117),1],action=np.zeros(21),next_state=np.r_[np.zeros(117),1],reward=-10.,not_done=1.,info=dict(nativeSampleCount=20,physicalFailure="flight",success=False))
        with self.assertRaises(ValueError):r.add(1,t)
        self.assertEqual(r.items,[]);t["not_done"]=0.;r.add(1,t);state=r.state();state["columns"][4][0,0]=1.
        with self.assertRaises(ValueError):SlowReplay.restore(state)
        t["info"].update(physicalFailure=None,success=True);t["not_done"]=1.
        with self.assertRaises(ValueError):r.add(2,t)
        t["not_done"]=0.;r.add(2,t)
    def test_full_stochastic_next_action_optimizer_replay_restore(self):
        try:import torch
        except ImportError:self.skipTest('optional existing Torch')
        from slow_task import initialize_slow_agent
        agent,_=initialize_slow_agent();r=SlowReplay();rng=np.random.default_rng(7)
        for i in range(8):
            x=rng.normal(0,.01,118).astype(np.float32);x[-1]=1;t=dict(state=x,action=rng.uniform(-.1,.1,21),next_state=x.copy(),reward=.2,not_done=1.,info=dict(nativeSampleCount=20,physicalFailure=None));r.add(i+1,t)
        from training import critic_only_update
        critic_only_update(agent,r,4,before_optimizer_step=lambda:None)
        charged_source_update(agent,r,4,before_optimizer_step=lambda _:None)
        self.assertTrue(agent.actor_optimizer.state_dict()['state']);self.assertTrue(agent.critic_optimizer.state_dict()['state']);self.assertTrue(agent.log_alpha_optimizer.state_dict()['state'])
        checkpoint=learner_state(agent,r);x=torch.tensor(r.items[0][0])[None,:]
        with torch.no_grad():action1=agent.actor(x)[1].clone()
        calls=[];out1=charged_source_update(agent,r,4,before_optimizer_step=calls.append);wanted={k:v.detach().clone()for k,v in agent.actor.state_dict().items()};critic={k:v.detach().clone()for k,v in agent.critic.state_dict().items()};alpha=agent.log_alpha.detach().clone();wanted_full=learner_state(agent,r)
        r2=restore_learner(agent,checkpoint)
        with torch.no_grad():action2=agent.actor(x)[1].clone()
        out2=charged_source_update(agent,r2,4,before_optimizer_step=lambda _:None)
        self.assertTrue(torch.equal(action1,action2));self.assertEqual(out1,out2);self.assertTrue(all(torch.equal(v,wanted[k])for k,v in agent.actor.state_dict().items()));self.assertTrue(all(torch.equal(v,critic[k])for k,v in agent.critic.state_dict().items()));self.assertTrue(torch.equal(agent.log_alpha,alpha));self.assertEqual(calls,['critic','actor','alpha'])
        def compare(a,b):
            if isinstance(a,torch.Tensor):self.assertTrue(torch.equal(a,b));return
            if isinstance(a,dict):
                self.assertEqual(set(a),set(b))
                for key in a:compare(a[key],b[key])
            elif isinstance(a,(list,tuple)):
                self.assertEqual(len(a),len(b))
                for av,bv in zip(a,b):compare(av,bv)
            else:self.assertEqual(a,b)
        compare(wanted_full,learner_state(agent,r2))
if __name__=='__main__':unittest.main()
