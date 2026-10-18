"""Zero-physics synthetic one-update parity against unchanged pinned SAC.py.

This uses random throwaway networks/replay only. No task actor is optimized or
saved, no actual transition is collected, and no learning admission is implied.
"""
import copy,hashlib,importlib.util,unittest
from pathlib import Path
import numpy as np
try:
    import torch
except ImportError:
    torch=None
ROOT=Path(__file__).resolve().parents[2];PATH=ROOT/'scripts/getup-training/SAC.py'
if torch is not None:
    s=importlib.util.spec_from_file_location('pinned_getup_sac',PATH);source=importlib.util.module_from_spec(s);s.loader.exec_module(source)

@unittest.skipIf(torch is None,'optional Torch2.9.1 CPU required for synthetic SAC parity')
class SACMathTests(unittest.TestCase):
    def test_conditioned_torque_teacher_Torch_initializer_full42_parity(self):
        from task import load_config
        from training import teacher_for_synthetic_parity
        from actors import DenseActor
        torch.set_num_threads(1);torch.manual_seed(17)
        agent=teacher_for_synthetic_parity(load_config()[0]);original=DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/teacher.bin');rng=np.random.default_rng(17)
        for power in [.4,.663420431,1.]:
            physical=rng.normal(0,.2,67).astype(np.float32);old=np.r_[physical,np.float32(power)].astype(np.float32);new=np.r_[physical,rng.normal(0,.2,50),np.float32(power)].astype(np.float32)
            expected=original.infer(old)
            with torch.no_grad():raw=agent.actor.trunk(torch.from_numpy(new).reshape(1,118)).numpy()[0]
            np.testing.assert_allclose(raw,expected['raw'],atol=2e-4,rtol=0.)
            np.testing.assert_allclose(np.tanh(raw[:21])*np.float32(power),expected['action'],atol=5e-5,rtol=0.)
            self.assertEqual(torch.count_nonzero(agent.actor.trunk[0].weight[:,67:117]).item(),0)
    def test_critic_only_warmup_polyak_and_untouched_actor_alpha(self):
        from training import critic_only_update
        torch.set_num_threads(1);torch.manual_seed(5)
        a=source.SAC(118,21,discount=.97,policy_freq=1,tau=.005)
        old_actor=copy.deepcopy(a.actor.state_dict());old_alpha=a.log_alpha.detach().clone();actor_optimizer=copy.deepcopy(a.actor_optimizer.state_dict());alpha_optimizer=copy.deepcopy(a.log_alpha_optimizer.state_dict());old_target=[p.detach().clone()for p in a.critic_target.parameters()]
        x=torch.zeros(4,118);x[:,-1]=1.;u=torch.zeros(4,21);r=torch.tensor([[1.],[-10.],[0.],[.5]]);mask=torch.tensor([[1.],[0.],[1.],[0.]])
        class Replay:
            def sample(self,n):return x,u,x,r,mask
        charged=[];out=critic_only_update(a,Replay(),4,before_optimizer_step=lambda:charged.append(1))
        self.assertEqual(charged,[1]);self.assertEqual(out['criticOnlyUpdates'],1);self.assertEqual(a.total_it,0)
        for key,value in a.actor.state_dict().items():torch.testing.assert_close(value,old_actor[key],atol=0,rtol=0)
        torch.testing.assert_close(a.log_alpha,old_alpha,atol=0,rtol=0);self.assertEqual(a.actor_optimizer.state_dict(),actor_optimizer);self.assertEqual(a.log_alpha_optimizer.state_dict(),alpha_optimizer)
        for current,p,old in zip(a.critic_target.parameters(),a.critic.parameters(),old_target):torch.testing.assert_close(current,.005*p+.995*old,atol=0,rtol=0)
    def test_pinned_source_synthetic_one_update_matches_explicit_equations(self):
        self.assertEqual(hashlib.sha256(PATH.read_bytes()).hexdigest(),'64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420')
        torch.set_num_threads(1);torch.manual_seed(20261017)
        a=source.SAC(118,21,discount=.97,actor_lr=1e-5,critic_lr=1e-4,alpha_lr=1e-4,policy_freq=1,tau=.005);b=copy.deepcopy(a)
        state=torch.randn(4,118)*.1;state[:,-1]=1.;next_state=state+.01;next_state[:,-1]=1.;action=torch.randn(4,21)*.1;reward=torch.tensor([[1.],[-10.],[0.],[3.]]);not_done=torch.tensor([[1.],[0.],[1.],[0.]])
        class Replay:
            def sample(self,batch):return state,action,next_state,reward,not_done
        torch.manual_seed(1234);a.train(Replay(),batch_size=4)
        torch.manual_seed(1234)
        with torch.no_grad():
            _,next_action,logp,_=b.actor(next_state);q1,q2=b.critic_target(next_state,next_action);y=reward+not_done*.97*(torch.minimum(q1,q2)-b.alpha.detach()*logp)
            torch.testing.assert_close(y[1],reward[1]);torch.testing.assert_close(y[3],reward[3])
        q1,q2=b.critic(state,action);critic_loss=((q1-y)**2).mean()+((q2-y)**2).mean();b.critic_optimizer.zero_grad();critic_loss.backward();b.critic_optimizer.step()
        _,pi,logp,_=b.actor(state);q1,q2=b.critic(state,pi);actor_loss=(b.alpha.detach()*logp-torch.minimum(q1,q2)).mean();b.actor_optimizer.zero_grad();actor_loss.backward();b.actor_optimizer.step()
        alpha_loss=(b.alpha*(-logp-b.target_entropy).detach()).mean();b.log_alpha_optimizer.zero_grad();alpha_loss.backward();b.log_alpha_optimizer.step()
        for p,t in zip(b.critic.parameters(),b.critic_target.parameters()):t.data.copy_(.005*p.data+.995*t.data)
        for first,second in [(a.actor,b.actor),(a.critic,b.critic),(a.critic_target,b.critic_target)]:
            for x,y in zip(first.parameters(),second.parameters()):torch.testing.assert_close(x,y,atol=2e-7,rtol=0.)
        torch.testing.assert_close(a.log_alpha,b.log_alpha,atol=1e-12,rtol=0.)
        self.assertEqual(a.total_it,1);self.assertAlmostEqual(a.loss_dict['critic'][0],critic_loss.item(),places=6)
if __name__=='__main__':unittest.main()
