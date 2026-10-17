#!/usr/bin/env python3
"""Conic adapter sign/scaling/bounds tests; no Native physics."""
import importlib.util,json,unittest
from pathlib import Path
import numpy as np
sp=importlib.util.spec_from_file_location('d',Path(__file__).with_name('getup-contact-dynamics.py'));d=importlib.util.module_from_spec(sp);sp.loader.exec_module(d)
class ConicQPTests(unittest.TestCase):
 def test_half_quadratic_convention_equality_inequality_and_boxes(self):
  r=d.solve_clarabel_qp(2*np.eye(2),np.array([-4.,-2.]),np.array([[1.,1.]]),np.array([2.]),np.array([[1.,0.]]),np.array([1.2]),np.zeros(2),np.ones(2)*2)
  self.assertTrue(r.success);np.testing.assert_allclose(r.x,[1.2,.8],atol=1e-6)
 def test_exact_inactive_force_bounds(self):
  r=d.solve_clarabel_qp(np.eye(2),np.array([-10.,-1.]),np.zeros((0,2)),np.zeros(0),np.zeros((0,2)),np.zeros(0),np.array([0.,-2.]),np.array([0.,2.]));self.assertTrue(r.success);self.assertLess(abs(r.x[0]),1e-8);self.assertAlmostEqual(r.x[1],1.,places=6)
 def test_measured_elbow_future_buffer_reserves_margin_under_original_caps(self):
  root=Path(__file__).resolve().parents[1];x=json.loads((root/'docs/fixtures/contact-landing-mvp/elbow-braking-state.json').read_text());ref=x['reference'];values=[]
  for margin in [0.,1e-4]:
   c=d.ContactDynamics(json.loads((root/'rust/crates/model/data/getup-v1.json').read_text()),contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=margin);c.flat_targets={k:np.array(v) for k,v in ref['flatTargets'].items()};st=c.state(x['state']['observation'],x['snapshot'],x['diagnostic']);swing={k:np.array(v) if k!='body' else v for k,v in ref['swing'].items()}
   _,info=c.solve(st,np.array(ref['targetCOM']),np.array(ref['rootRotation']),np.array(ref['posture']),swing=swing,_inactive=[]);self.assertTrue(info['success']);self.assertLess(info['equalityError'],1e-8);self.assertLessEqual(info['maximumMotorFraction'],.900001);self.assertGreaterEqual(info['minimumPredictedJointStoppingMarginRad'],margin-1e-8);values.append(info['qdd'][23])
  self.assertGreater(values[1],values[0]+.25)
 def test_inconsistent_bounds_are_rejected(self):
  with self.assertRaises(ValueError):d.solve_clarabel_qp(np.eye(1),np.zeros(1),np.zeros((0,1)),np.zeros(0),np.zeros((0,1)),np.zeros(0),np.ones(1),np.zeros(1))
if __name__=='__main__':unittest.main()
