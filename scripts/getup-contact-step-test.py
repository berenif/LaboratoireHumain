#!/usr/bin/env python3
"""Pure step mode gates; no Native physics."""
import importlib.util,unittest
from pathlib import Path
import numpy as np
s=importlib.util.spec_from_file_location('step',Path(__file__).with_name('getup-contact-step.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class Kin:
 model={'totalMassKg':40.84402122}
 def bottom(self,body,pose):return pose[1,3]-.03
class StepTests(unittest.TestCase):
 def setUp(self):
  self.step=m.RecoveryStep([0,.81,0],[0,.81,.14],7.3667,Kin());pose=np.eye(4);pose[:3,3]=[.1,.03,-.2];self.model={'geometry':({'left_foot':pose},None,None,None)};self.state={'metrics':{'foot_loads_n':[0.,400.],'foot_clearance_m':[0.,0.]}};self.diag={'capture':dict(x=0,y=.81,z=.14)};self.last=[dict(loads_n=[0.,400.],clearance_m=[0.,0.])]*20
 def update(self,t):return self.step.update(t,self.state,self.model,self.diag,self.last,0.)
 def ready(self):
  for i in range(4):self.update(8.+i*.025)
 def test_no_lift_before_100ms_actual_unload(self):
  for i in range(3):
   _,task,_=self.update(8.+i*.025);self.assertEqual(self.step.phase,'transfer');self.assertAlmostEqual(task['position'][1],.03)
  self.update(8.075);self.assertEqual(self.step.phase,'lift')
 def test_actual_unload_waits_for_independent_capacity_readiness(self):
  for i in range(8):
   _,task,_=self.step.update(8.+i*.025,self.state,self.model,self.diag,self.last,0.,lift_ready=False)
   self.assertEqual(self.step.phase,'transfer');np.testing.assert_allclose(task['position'],self.model['geometry'][0]['left_foot'][:3,3])
  self.step.update(8.2,self.state,self.model,self.diag,self.last,0.,lift_ready=True);self.assertEqual(self.step.phase,'lift')
 def test_waiting_for_capacity_never_bypasses_bounded_timeout(self):
  self.step.unloaded=100;self.step.capture_run=.2
  self.step.update(9.5,self.state,self.model,self.diag,self.last,0.,lift_ready=False);self.assertEqual(self.step.phase,'stop');self.assertEqual(self.step.failed,'no-qualified-unload')
 def test_horizontal_progress_requires_actual_whole_clearance(self):
  self.ready();self.update(8.5);self.assertEqual(self.step.phase,'lift')
  self.last=[dict(loads_n=[0.,400.],clearance_m=[.02,0.])]*20;self.state['metrics']['foot_clearance_m'][0]=.02;self.update(8.525);self.assertEqual(self.step.phase,'translate')
 def test_clearance_loss_stops_horizontal_target_and_lands(self):
  self.ready();self.step.phase='translate';self.step.phase_start=8.1;self.update(8.2);self.assertEqual(self.step.phase,'land');self.assertEqual(self.step.failed,'clearance-lost-during-placement')
 def test_explicit_new_protocol_timeout_waits_but_never_returns_unloaded(self):
  self.step.transfer_timeout_s=10.725;self.step.capture_run=0.;self.last=[];self.step.update(9.5,self.state,self.model,self.diag,self.last,.02);self.assertEqual(self.step.phase,'transfer');self.step.update(10.725,self.state,self.model,self.diag,self.last,.02);self.assertEqual(self.step.phase,'stop')
 def test_unqualified_timeout_stops_instead_of_returning_over_unloaded_foot(self):
  self.step.capture_run=0.;self.last=[];self.step.update(9.5,self.state,self.model,self.diag,self.last,.02);self.assertEqual(self.step.phase,'stop');self.assertIsNone(self.step.return_start)
 def test_com_cannot_return_until_persistent_landing(self):
  self.ready();self.step.phase='land';self.step.phase_start=9.;self.step.lower_origin=self.step.high+self.step.delta
  for i in range(3):
   com,_,_=self.update(9.5+i*.025);self.assertEqual(self.step.phase,'land');np.testing.assert_allclose(com,self.step.stance_com)
  self.last=[dict(loads_n=[40.,360.],clearance_m=[0.,0.])]*20
  for i in range(3):self.update(9.6+i*.025);self.assertEqual(self.step.phase,'land')
  self.update(9.675);self.assertEqual(self.step.phase,'return')
class TerminalUnloadTests(unittest.TestCase):
 def test_soft_unload_waits_for_readiness_then_matches_the_ceiling_ramp(self):
  goal=m.TerminalUnloadGoal();self.assertEqual(goal.soft_weight,0.);goal.update(False,45.);self.assertEqual(goal.soft_weight,0.);goal.update(True,45.);self.assertEqual(goal.soft_weight,0.)
  weights=[]
  for _ in range(20):
   ceiling=goal.update(True,999.);weights.append(goal.soft_weight);self.assertAlmostEqual(ceiling/goal.anchor+goal.soft_weight,1.)
  self.assertEqual(weights[-1],1.);self.assertTrue(all(a<=b for a,b in zip(weights,weights[1:])));goal.update(False,0.);self.assertEqual(goal.soft_weight,0.)
 def test_ceiling_starts_from_measured_load_and_is_monotone(self):
  goal=m.TerminalUnloadGoal();self.assertIsNone(goal.update(False,45.));limits=[goal.update(True,45.)]
  for i in range(20):limits.append(goal.update(True,999.))
  self.assertEqual(limits[0],45.);self.assertAlmostEqual(limits[-1],0.);self.assertTrue(all(a>=b>=0 for a,b in zip(limits,limits[1:])));self.assertEqual(goal.anchor,45.)
 def test_lost_support_aborts_without_recapture_or_force_goal(self):
  goal=m.TerminalUnloadGoal();goal.update(True,30.);self.assertIsNone(goal.update(False,20.));self.assertTrue(goal.failed);self.assertIsNone(goal.update(True,10.));self.assertEqual(goal.anchor,30.)
 def test_shorter_than_half_second_is_rejected(self):
  with self.assertRaises(AssertionError):m.TerminalUnloadGoal(.2)
if __name__=='__main__':unittest.main()
