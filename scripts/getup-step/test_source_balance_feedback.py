"""Pure counterfactual feedback checks; no native integrations."""
import copy,unittest
from types import SimpleNamespace
import numpy as np
from source_step_probe import World,Controller,ROOT
from source_multistep_probe import unloaded_support_ready
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z';HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
@unittest.skipUnless(BASE.exists()and HELPER.exists(),'local source assets absent')
class BalanceFeedback(unittest.TestCase):
 def setUp(self):self.w=World(BASE,HELPER);self.w.restore(BASE/'slow-continuation/final.bin');self.initial_counter=self.w.l.lh_dim(self.w.h,6)
 def tearDown(self):self.w.close()
 def test_com_sensor_counterfactual_changes_actual_qp_commands(self):
  w=self.w;c=Controller(w);before,_=c.solve(.5);original=w.arr
  def changed(k,shape):
   value=original(k,shape)
   if k==6:value=value.copy();value[1,0]+=.005
   return value
  w.arr=changed;after,_=c.solve(.5);self.assertGreater(max(abs(after-before)),1e-5);self.assertLessEqual(max(abs(after)),.95000001);self.assertEqual(w.l.lh_dim(w.h,6),self.initial_counter)
 def test_actual_velocity_counterfactual_changes_commands_without_steps(self):
  initial=self.w.packet();w=World(BASE,HELPER)
  try:
   w.install_cold(initial);c=Controller(w);before,_=c.solve(.5);shifted=copy.deepcopy(initial);shifted['qvel'][0]+=.02;w.install_cold(shifted);after,_=c.solve(.5);self.assertGreater(max(abs(after-before)),1e-5);self.assertEqual(w.l.lh_dim(w.h,6),0)
  finally:w.close()
 def test_measured_landing_load_changes_contact_solve(self):
  w=self.w;c=Controller(w);original=w.arr
  def observed(load):
   def sensor(k,shape):
    value=original(k,shape)
    if k==20:value=value.copy();value[c.swing_body]=load
    return value
   return sensor
  w.arr=observed(0.);before,a=c.solve(.5,'land');w.arr=observed(40.);after,b=c.solve(.5,'land');self.assertEqual(len(a['predictedForcesN']),4);self.assertEqual(len(b['predictedForcesN']),8);self.assertGreater(max(abs(after-before)),1e-5);self.assertEqual(w.l.lh_dim(w.h,6),self.initial_counter)
class AdmissionFeedback(unittest.TestCase):
 def test_transfer_requires_actual_capture_load_and_joint_reserve(self):
  c=SimpleNamespace(com1=np.array([0.,0.,.8]),swing_side=0,stance_side=1);row=dict(com=[0.,0.,.8],comVelocity=[0.,0.,0.],jointLimitMinimumMarginRad=.01,jointLimitMarginsRad=[.01]*21,maximumBodyLinearMps=.01,maximumBodyAngularRadps=.02,feet=[dict(loadN=0.),dict(loadN=400.)]);self.assertTrue(unloaded_support_ready(row,c,40.))
  changes=[('com',[.02,0.,.8]),('comVelocity',[.05,0.,0.]),('jointLimitMinimumMarginRad',.002),('maximumBodyAngularRadps',.3),('feet',[dict(loadN=10.),dict(loadN=390.)]),('feet',[dict(loadN=0.),dict(loadN=200.)])]
  for key,value in changes:
   changed=copy.deepcopy(row);changed[key]=value
   with self.subTest(input=key,value=value):self.assertFalse(unloaded_support_ready(changed,c,40.))
if __name__=='__main__':unittest.main()
