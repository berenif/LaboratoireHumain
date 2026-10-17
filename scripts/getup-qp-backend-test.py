#!/usr/bin/env python3
"""Conic adapter sign/scaling/bounds tests; no Native physics."""
import importlib.util,unittest
from pathlib import Path
import numpy as np
sp=importlib.util.spec_from_file_location('d',Path(__file__).with_name('getup-contact-dynamics.py'));d=importlib.util.module_from_spec(sp);sp.loader.exec_module(d)
class ConicQPTests(unittest.TestCase):
 def test_half_quadratic_convention_equality_inequality_and_boxes(self):
  r=d.solve_clarabel_qp(2*np.eye(2),np.array([-4.,-2.]),np.array([[1.,1.]]),np.array([2.]),np.array([[1.,0.]]),np.array([1.2]),np.zeros(2),np.ones(2)*2)
  self.assertTrue(r.success);np.testing.assert_allclose(r.x,[1.2,.8],atol=1e-6)
 def test_exact_inactive_force_bounds(self):
  r=d.solve_clarabel_qp(np.eye(2),np.array([-10.,-1.]),np.zeros((0,2)),np.zeros(0),np.zeros((0,2)),np.zeros(0),np.array([0.,-2.]),np.array([0.,2.]));self.assertTrue(r.success);self.assertLess(abs(r.x[0]),1e-8);self.assertAlmostEqual(r.x[1],1.,places=6)
 def test_inconsistent_bounds_are_rejected(self):
  with self.assertRaises(ValueError):d.solve_clarabel_qp(np.eye(1),np.zeros(1),np.zeros((0,1)),np.zeros(0),np.zeros((0,1)),np.zeros(0),np.ones(1),np.zeros(1))
if __name__=='__main__':unittest.main()
