import copy,datetime,json,tempfile,unittest
from pathlib import Path
import numpy as np
import torch
from continuation import retained,ContinuationGuard
from online_learning import initialize,Roles,propose,sample_action,state,restore
from stabilization_ledger import StageBudget
from test_online_learning import equal
class Tests(unittest.TestCase):
    def test_only_original_policy_heuristics_become_report_only(self):
        a,o=initialize();x=torch.zeros(8,118);x[:,-1]=1
        with torch.no_grad():a.actor.trunk[4].bias[:21].add_(.2)
        saved=copy.deepcopy(a.actor.state_dict());r=Roles('fixture');old=propose(a,o,x,x,r.proposal(1),charge=lambda:None,complete=lambda:None,check=lambda:None)
        self.assertFalse(old['accepted']);self.assertTrue(all(t['maximumOriginalMeanDistance']>.05 for t in old['trials']))
        a.actor.load_state_dict(saved);r=Roles('fixture');new=propose(a,o,x,x,r.proposal(1),charge=lambda:None,complete=lambda:None,check=lambda:None,enforce_original_mean_distance=False)
        self.assertTrue(new['accepted']);self.assertGreater(new['trials'][0]['maximumOriginalMeanDistance'],.05);self.assertLessEqual(new['trials'][0]['maximumStepKL'],.05);self.assertGreaterEqual(new['trials'][0]['lossDecrease'],1e-6)
    def test_hard_kl_loss_nonfinite_and_rollback_remain(self):
        for kwargs in[dict(kl_limit=0.),dict(minimum_decrease=1e6)]:
            a,o=initialize();x=torch.zeros(8,118);x[:,-1]=1;r=Roles('fixture');before=copy.deepcopy(a.actor.state_dict());opt=copy.deepcopy(a.actor_optimizer.state_dict());p=propose(a,o,x,x,r.proposal(1),charge=lambda:None,complete=lambda:None,check=lambda:None,enforce_original_mean_distance=False,**kwargs)
            self.assertFalse(p['accepted']);self.assertTrue(equal(before,a.actor.state_dict()));self.assertTrue(equal(opt,a.actor_optimizer.state_dict()))
        x[0,0]=float('nan')
        with self.assertRaises(ValueError):propose(a,o,x,x,r.proposal(2),charge=lambda:None,complete=lambda:None,check=lambda:None,enforce_original_mean_distance=False)
    def test_populated_real_retained_next_action_batch_and_attempt140(self):
        c,m,a,o,replay,r,native,side,prefix,episodes=retained();x=torch.tensor(replay.items[-1][2])[None,:];opt=copy.deepcopy(a.actor_optimizer.state_dict());action,b=sample_action(a.actor,x,r,11);indices=r.replay.integers(0,1584,128);noise=torch.randn((128,21),generator=r.proposal(140));_,_,aa,oo,rr,gg,_,_,pp,ee=retained();action2,b2=sample_action(aa.actor,x,gg,11);ids2=gg.replay.integers(0,1584,128);noise2=torch.randn((128,21),generator=gg.proposal(140))
        np.testing.assert_array_equal(action,action2);self.assertEqual(b,b2);np.testing.assert_array_equal(indices,ids2);self.assertTrue(torch.equal(noise,noise2));self.assertTrue(equal(opt,aa.actor_optimizer.state_dict()));self.assertTrue(opt['state']);self.assertEqual((len(prefix),prefix[0]['episode'],side['core']['state']['native_id']),(7,48,6540));self.assertEqual(len(replay.items),1580)
    def test_fresh_caps_do_not_refund_old_learner_steps(self):
        with tempfile.TemporaryDirectory()as d:
            path=Path(d)/'ledger.json';data=dict(profile='slow-online-continuation',deadlineUtc=(datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(minutes=45)).isoformat(),aggregatePhysicsLimitSeconds=15.,validationWallReservedSeconds=300,allIntervalLimit=4060,maximumSourceSteps=81200,physicsSeconds=0.,sourceIntegrations=0,totalPolicyIntervals=0,stagePhysicsSeconds=dict(benchmark=0.,training=0.,validation=0.),stagePolicyIntervals=dict(benchmark=0,training=0,validation=0),optimizerCalls=0,candidateOptimizerCalls=0,pendingOptimizer=None,requestOutstanding=False);path.write_text(json.dumps(data))
            with StageBudget(path)as b:
                with self.assertRaises((TimeoutError,ValueError)):b.begin_interval('benchmark')
                g=ContinuationGuard(b,Path(d));g.phase='training';g.charge('continuation-critic');g.complete('continuation-critic');g.charge('continuation-head');g.complete('continuation-head')
                with self.assertRaises(ValueError):g.charge('continuation-critic')
                g.commit_proposal('continuation-head');b.data['continuation-critic']=855
                with self.assertRaises(ValueError):g.charge('continuation-critic')
                b.data['stagePolicyIntervals']['training']=3420;b.data['totalPolicyIntervals']=3420
                with self.assertRaises(TimeoutError):b.begin_interval('training')
                b.begin_interval('validation');b.data['sourceIntegrations']=81200
                with self.assertRaises(TimeoutError):b.integrate('validation',lambda _:None,lambda:0)
if __name__=='__main__':unittest.main()
