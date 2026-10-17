#!/usr/bin/env python3
"""Pure FK/force-allocation checks, never native integration."""
import importlib.util,json,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('planner',ROOT/'scripts/plan-getup-stance-target.py');p=importlib.util.module_from_spec(s);s.loader.exec_module(p)
class StanceReferenceTests(unittest.TestCase):
 def setUp(self):
  self.plan=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/feasible-stance-reference.json').read_text());self.c=p.d.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));self.refs=p.FeasibleReferences(self.plan,self.c);self.contacts=[dict(body=c['body'],point=np.array(c['point']),basis=np.array(c['basis']))for c in self.plan['plannedContactPointsWorld']]
 def test_same_runtime_pchip_holds_both_feet_to_declared_tolerance(self):
  q,R,com=self.refs.reference(0.);root=np.array(self.plan['pathKnots'][0]['root']);initial=self.c.kin.forward(q,root);maximum=0.
  for w in np.linspace(0.,1.,101):
   q,_,_=self.refs.reference(w);root=self.refs.right@np.linalg.inv(self.c.kin.forward(q,np.eye(4))['right_foot']);poses=self.c.kin.forward(q,root);np.testing.assert_allclose(poses['right_foot'],self.refs.right,atol=1e-10);maximum=max(maximum,float(max(abs(poses['left_foot'][:3,3]-initial['left_foot'][:3,3]))))
  self.assertLess(maximum,.0001)
 def test_terminal_pose_has_real_zero_left_static_capacity(self):
  q,_,_=self.refs.reference(1.);root=self.refs.right@np.linalg.inv(self.c.kin.forward(q,np.eye(4))['right_foot']);info=p.static_capacity(self.c,q,root,self.contacts,left_share=0);self.assertTrue(info['feasible']);self.assertLess(info['minimumRequiredMotorFraction'],.81);self.assertLess(info['rootAndJointBalanceResidual'],1e-7)
 def test_fast_joint_cannot_receive_a_near_stop_reaction(self):
  q,_,_=self.refs.reference(1.);root=self.refs.right@np.linalg.inv(self.c.kin.forward(q,np.eye(4))['right_foot']);v=np.zeros(27);v[9]=.1;info=p.static_capacity(self.c,q,root,self.contacts,left_share=0,velocity=v);self.assertNotIn(3,[s['qpos']for s in info['limitReactions']])
 def test_no_stop_capacity_does_not_credit_any_limit_reaction(self):
  q,_,_=self.refs.reference(1.);root=self.refs.right@np.linalg.inv(self.c.kin.forward(q,np.eye(4))['right_foot']);free=p.static_capacity(self.c,q,root,self.contacts,left_share=0,allow_limit_reactions=False);old=p.static_capacity(self.c,q,root,self.contacts,left_share=0);self.assertTrue(free['feasible']);self.assertEqual(free['limitReactions'],[]);self.assertGreater(free['minimumRequiredMotorFraction'],old['minimumRequiredMotorFraction']);self.assertLess(free['rootAndJointBalanceResidual'],1e-7)
 def test_selected_interior_reference_has_no_stop_certificate_and_real_joint_margin(self):
  plan=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/no-stop-stance-reference.json').read_text());row=plan['pathKnots'][-1];q=np.array(row['q']);root=np.array(row['root']);contacts=[dict(body=v['body'],point=np.array(v['forcePoint']),basis=np.array(v['basis']))for v in plan['plannedContactPointsWorld']];info=p.static_capacity(self.c,q,root,contacts,left_share=0,allow_limit_reactions=False);self.assertTrue(info['feasible']);self.assertEqual(info['limitReactions'],[]);self.assertLessEqual(info['minimumRequiredMotorFraction'],.85000001);self.assertGreaterEqual(min(np.min(q[:15]-self.c.kin.limits[:15,0]),np.min(self.c.kin.limits[:15,1]-q[:15])),.01-1e-10);self.assertLess(plan['maximumRuntimePchipLeftFootErrorM'],.0001);self.assertGreaterEqual(plan['minimumHullMarginM'],.01-1e-10)
if __name__=='__main__':unittest.main()
