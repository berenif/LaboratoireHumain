import unittest,numpy as np
from stabilization_init import causal_pair,logstd_raw_bias
class StabilizationInitTests(unittest.TestCase):
    def test_source_inverse_logstd_map(self):
        raw=logstd_raw_bias();actual=-1.5+3.5*np.tanh(raw);self.assertAlmostEqual(np.exp(actual),.02,places=14)
    def test_label_uses_prephysical_and_actuator_order(self):
        from source_math import ORDER
        p=dict(sourceStep=4,timeS=.005,physicalObservation=np.arange(67,dtype=float).tolist());u=np.arange(21)*.01;gear=np.arange(21)+1.
        c=dict(sourceStep=5,timeS=.00625,physicalObservation=np.ones(67)*999,controlsActuatorOrder=u.tolist(),qfrcActuator=np.r_[np.zeros(6),gear*u[np.argsort(ORDER)]].tolist())
        target=dict(q=np.zeros(21),com=np.array([0,0,.8]),com_ori=np.array([0,0,1.]))
        x,actual,mu=causal_pair(p,c,target,gear);np.testing.assert_array_equal(x[:67],np.arange(67));np.testing.assert_allclose(actual,u,atol=1e-8);np.testing.assert_allclose(np.tanh(mu),u,atol=1e-8)
        c['timeS']=float('nan')
        with self.assertRaises(ValueError):causal_pair(p,c,target,gear)
        c['timeS']=.00625;c['sourceStep']=6
        with self.assertRaises(ValueError):causal_pair(p,c,target,gear)

class SourceActorInitializationTests(unittest.TestCase):
    def test_torch_source_std_is_point02_without_any_optimizer(self):
        try:import torch
        except ImportError:self.skipTest('optional existing Torch runtime')
        from stabilization_init import initialize_torque_actor
        from task import load_config
        from source_task import CONFIG_PATH
        actor=initialize_torque_actor(load_config(CONFIG_PATH)[0]);x=torch.zeros(5,118);x[:,-1]=1
        with torch.no_grad():_,_,_,logstd=actor(x,compute_pi=False,compute_log_pi=False)
        np.testing.assert_allclose(logstd.exp().numpy(),.02,atol=1e-8,rtol=0.)
        loss=(actor.trunk(x)[:,:21]**2).mean();actor.zero_grad();loss.backward()
        self.assertEqual(torch.count_nonzero(actor.trunk[4].weight.grad[21:]),0);self.assertEqual(torch.count_nonzero(actor.trunk[4].bias.grad[21:]),0)

if __name__=='__main__':unittest.main()
