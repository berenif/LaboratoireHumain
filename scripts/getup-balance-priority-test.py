#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
def module(name,file):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/file);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
d=module('dynamics','getup-contact-dynamics.py');p=d.balance_priority

class BalancePriorityTests(unittest.TestCase):
    def test_coupled_slack_before_optional_objective(self):
        H=np.eye(2);g=np.array([-100.,0.]);E=np.array([[1.,1.]]);b=np.array([1.]);C=np.zeros((0,2));upper=np.zeros(0)
        result,info=p.solve_tracking_hierarchy(d.solve_clarabel_qp,H,g,E,b,C,upper,np.full(2,-2.),np.full(2,2.),[0,1],np.zeros(2),np.zeros(2))
        self.assertTrue(result.success);np.testing.assert_allclose(result.x,[.5,.5],atol=3e-6);self.assertAlmostEqual(info['minimumSquaredSlackCost'],.25,places=7)
    def test_physics_not_relaxed(self):
        result,info=p.solve_tracking_hierarchy(d.solve_clarabel_qp,np.eye(1),np.zeros(1),np.array([[1.]]),np.array([2.]),np.zeros((0,1)),np.zeros(0),np.array([-1.]),np.array([1.]),[0],[-.1],[.1])
        self.assertFalse(result.success);self.assertFalse(info['firstStageSuccess'])
    def test_physical_stop_can_require_tracking_slack(self):
        result,info=p.solve_tracking_hierarchy(d.solve_clarabel_qp,np.eye(1),np.zeros(1),np.array([[1.]]),np.array([.5]),np.zeros((0,1)),np.zeros(0),np.array([.5]),np.array([.5]),[0],[-.1],[.1])
        self.assertTrue(result.success);self.assertAlmostEqual(info['minimumSlackRadps2'][0],.4,places=6);self.assertAlmostEqual(info['minimumSquaredSlackCost'],.08,places=8);self.assertEqual(info['analyticallyEliminatedFixedTrackingCoordinates'],[0])
    def test_capture_derivative_with_height(self):
        com=np.array([.01,.8,-.02]);velocity=np.array([.02,.015,-.01]);acc=np.array([-.08,-.03,.04]);omega=np.sqrt(9.81/com[1]);capture=(com+velocity/omega)[[0,2]]
        state=dict(com=com,comVelocity=velocity,Jcom=np.eye(3),comBias=np.zeros(3));points=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        rows,upper,info=p.capture_rows(state,points,capture)
        from scipy.spatial import ConvexHull
        hull=ConvexHull(np.unique(points,axis=0));normal=hull.equations[:,:2];h=-normal@capture-hull.equations[:,2]-.003
        epsilon=1e-7;c2=com+epsilon*velocity+.5*epsilon**2*acc;v2=velocity+epsilon*acc;cap2=(c2+v2/np.sqrt(9.81/c2[1]))[[0,2]]
        hdot=-normal@(cap2-capture)/epsilon
        np.testing.assert_allclose(upper-rows@acc,hdot+4*h,atol=2e-8);self.assertTrue(info['initiallyInsideInset'])
    def test_outside_capture_is_reported_recovery(self):
        state=dict(com=np.array([0.,.8,0.]),comVelocity=np.zeros(3),Jcom=np.eye(3),comBias=np.zeros(3))
        _,_,info=p.capture_rows(state,[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]],[.099,0.]);self.assertFalse(info['initiallyInsideInset'])
    def test_segment_requests_zero_inset_recovery(self):
        state=dict(com=np.array([0.,.8,0.]),comVelocity=np.zeros(3),Jcom=np.eye(3),comBias=np.zeros(3))
        rows,upper,info=p.capture_rows(state,[[-.1,0.],[.1,0.]],[0.,.005],allow_segment=True)
        self.assertEqual(info['supportDimension'],1);self.assertEqual(info['insetM'],0.);self.assertFalse(info['initiallyInsideInset'])
        acceleration=np.array([0.,0.,-.02*np.sqrt(9.81/.8)]);np.testing.assert_allclose((upper-rows@acceleration)[:2],0.,atol=1e-12)
    def test_invalid_or_unqualified_hull_fails_cleanly(self):
        state=dict(com=np.array([0.,.8,0.]),comVelocity=np.zeros(3),Jcom=np.eye(3),comBias=np.zeros(3))
        for points,allow in [([],True),([[0.,0.]],True),([[-.1,0.],[.1,0.]],False)]:
            with self.assertRaises(ValueError):p.capture_rows(state,points,[0.,0.],allow_segment=allow)

if __name__=='__main__':unittest.main()
