import tempfile,unittest
from pathlib import Path
import numpy as np
try:import torch
except ImportError:torch=None
@unittest.skipIf(torch is None,'optional Torch')
class FitCheckpointTests(unittest.TestCase):
    def test_full_adam_rng_next_update_exact(self):
        from extend_stabilization_fit import save_full,restore_full
        torch.manual_seed(9);a=torch.nn.Linear(3,1);opt=torch.optim.Adam(a.parameters(),lr=1e-5);rng=np.random.default_rng(9)
        def step(model,optimizer,random):
            x=torch.tensor(random.normal(size=(7,3)),dtype=torch.float32);y=torch.tensor(random.normal(size=(7,1)),dtype=torch.float32);loss=((model(x)-y)**2).mean();optimizer.zero_grad();loss.backward();optimizer.step()
        step(a,opt,rng)
        with tempfile.TemporaryDirectory()as folder:
            p=Path(folder)/'checkpoint.pt';save_full(p,a,opt,rng,1,1,'test-data');expected_torch=torch.randn(11);step(a,opt,rng)
            b=torch.nn.Linear(3,1);other=torch.optim.Adam(b.parameters(),lr=1e-5);random=np.random.default_rng(0);saved=restore_full(p,b,other,random,dataset_sha='test-data');self.assertTrue(torch.equal(torch.randn(11),expected_torch));step(b,other,random)
            self.assertEqual(saved['trajectoryUpdates'],1)
            for x,y in zip(a.parameters(),b.parameters()):self.assertTrue(torch.equal(x,y))
if __name__=='__main__':unittest.main()
