#!/usr/bin/env python3
import importlib.util, pathlib, unittest
import numpy as np
import torch
p=pathlib.Path(__file__).with_name('train-getup-wasm-adaptation.py');spec=importlib.util.spec_from_file_location('pilot',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def state(preview=True):return {'preview':{'features':np.ones(1024,dtype=np.float32)} if preview else None}
class Contract(unittest.TestCase):
 def test_zero_mean_and_bounded_stochastic_c(self):
  actor=m.CorrectionActor();x=torch.randn(128,1024);mu,c,logp=actor(x)
  self.assertTrue(torch.equal(mu,torch.zeros_like(mu)));self.assertTrue(torch.all(c.abs()<=1));self.assertTrue(torch.isfinite(logp).all())
 def test_source_tail_keeps_discounted_terminal_reward(self):
  b=m.SourceTransition(.9);self.assertIsNone(b.push(state(),[0.]*21,{'state':state(False),'reward':1.,'terminal':False}))
  self.assertIsNone(b.push(state(False),None,{'state':state(False),'reward':2.,'terminal':False}))
  r=b.push(state(False),None,{'state':state(False),'reward':100.,'terminal':True})
  self.assertAlmostEqual(r[3],1+.9*2+.9**2*100);self.assertTrue(r[4]);self.assertTrue(np.all(r[2]==0));self.assertIsNone(b.pending)
 def test_terminal_failure_keeps_source_action(self):
  b=m.SourceTransition(.99);r=b.push(state(),[.1]*21,{'state':state(False),'reward':-100.,'terminal':True})
  self.assertEqual(r[3],-100.);self.assertTrue(r[4]);self.assertEqual(r[1],[.1]*21)
 def test_tail_reentry_rejected(self):
  b=m.SourceTransition(.9);b.push(state(),[0.]*21,{'state':state(False),'reward':0.,'terminal':False})
  with self.assertRaises(AssertionError):b.push(state(False),None,{'state':state(),'reward':0.,'terminal':False})
 def test_replay_has_exact_successful_quarter(self):
  r=m.Replay(20,np.random.default_rng(1))
  for i in range(5):r.add(np.ones(1024)*i,[0.]*21,np.ones(1024),0,False,success=i==0)
  batch=r.sample(128);self.assertEqual(batch[0].shape,(128,1024));self.assertTrue(torch.all(batch[0][:32]==0))
if __name__=='__main__':unittest.main()
