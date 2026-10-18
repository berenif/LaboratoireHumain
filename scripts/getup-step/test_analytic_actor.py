import json,unittest
from pathlib import Path
import numpy as np
try:import torch
except ImportError:torch=None
ROOT=Path(__file__).resolve().parents[2]
@unittest.skipIf(torch is None,'optional existing CPU Torch')
class AnalyticActorTests(unittest.TestCase):
    def actor_and_state(self):
        from analytic_actor import AnalyticSLOWHoldingActor
        from source_math import conditioned_teacher_observation
        row=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame'];p=np.array(row['physicalObservation']);t=np.r_[p[:21],row['comSource'][2],p[34:37]]
        obs=conditioned_teacher_observation(p,t,t,1.)
        return AnalyticSLOWHoldingActor(),torch.tensor(np.tile(obs,(4,1)))
    def test_zero_correction_source_hold_pd_mean_std_and_target_separation(self):
        from source_math import student_observation,slow_controls
        from actors import DenseActor
        from source_engine import torch_actor
        a,x=self.actor_and_state();infer=torch_actor(DenseActor.load(ROOT/'rust/crates/sim/data/getup-policy/student.bin'));p=x[0,:67].numpy();target=a.source_hold_target.numpy();_,residual=infer(student_observation(p,target,target));expected=slow_controls(a.source_standing.numpy(),residual,p[:21],p[46:67],holding=True)
        mu,_,_,ls=a(x,compute_pi=False,compute_log_pi=False);np.testing.assert_allclose(mu[0].detach().numpy(),expected,atol=2e-6,rtol=0.);torch.testing.assert_close(mu,a.base_torque(x),atol=3e-8,rtol=0.);np.testing.assert_allclose(ls.exp().detach(),.02,atol=1e-8,rtol=0.)
        different=x.clone();different[:,67:117]+=1.23;changed,_,_,_=a(different,compute_pi=False,compute_log_pi=False);torch.testing.assert_close(mu,changed,atol=0,rtol=0);self.assertEqual(a.source_hold_target[21].item(),torch.tensor(1.4).item())
    def test_state_dependent_base_has_standard_shifted_gaussian_tanh_density(self):
        a,x=self.actor_and_state();torch.manual_seed(11);_,action,logp,logstd=a(x);mean,_,_,_=a.latent_components(x);latent=torch.atanh(action);normal=torch.distributions.Normal(mean,logstd.exp());expected=(normal.log_prob(latent)-torch.log(torch.relu(1-action**2)+1e-6)).sum(-1,keepdim=True);torch.testing.assert_close(logp,expected,atol=1e-4,rtol=0.)
        loss=(.1*logp-action.sum(-1,keepdim=True)).mean();loss.backward();self.assertTrue(all(p.grad is None for p in a.frozen_slow.parameters()));self.assertTrue(torch.isfinite(a.correction.trunk[4].weight.grad).all());self.assertGreater(torch.count_nonzero(a.correction.trunk[4].weight.grad[:21]),0)
    def test_saturated_large_latent_source_epsilon_density_and_gradients(self):
        a,x=self.actor_and_state();x[:,:21]=100.
        with torch.no_grad():a.correction.trunk[4].bias[:21].fill_(-10.)
        mean,ls,_,_=a.latent_components(x);torch.manual_seed(13);noise=torch.randn_like(mean);torch.manual_seed(13);_,action,logp,_=a(x)
        expected=a.source.gaussian_logprob(noise,ls)-torch.log(torch.relu(1-action**2)+1e-6).sum(-1,keepdim=True)
        torch.testing.assert_close(logp,expected,atol=0,rtol=0);self.assertTrue(torch.isfinite(logp).all());self.assertTrue((action==-1).all())
        (.1*logp-action.sum(-1,keepdim=True)).mean().backward();self.assertTrue(all(p.grad is None for p in a.frozen_slow.parameters()));self.assertTrue(all(p.grad is None or torch.isfinite(p.grad).all()for p in a.correction.parameters()))
        self.assertEqual({id(p)for p in a.trainable_parameters},{id(p)for p in a.correction.parameters()});self.assertFalse(a.single_mlp_binary_export_supported)
    def test_saturation_epsilon_and_power_restriction(self):
        a,x=self.actor_and_state();x[:,:21]=100.;mu,_,_,_=a(x,compute_pi=False,compute_log_pi=False);base=a.base_torque(x);self.assertLessEqual(float(torch.max(abs(mu-base)).detach()),1.1e-6);self.assertTrue(a.outputs['baseSaturation'].all())
        x[:,-1]=.663420431
        with self.assertRaises(ValueError):a(x)
if __name__=='__main__':unittest.main()
