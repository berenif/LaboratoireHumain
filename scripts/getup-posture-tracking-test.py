#!/usr/bin/env python3
"""Pure reference derivatives/phase scope; never invokes native physics."""
import importlib.util,json,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
t=module('tracking',ROOT/'scripts/getup-posture-tracking.py');p=module('planner',ROOT/'scripts/plan-getup-stance-target.py')
class TrackingTests(unittest.TestCase):
 def setUp(self):
  self.plan=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/no-stop-stance-reference.json').read_text());self.c=p.d.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));self.refs=p.FeasibleReferences(self.plan,self.c);self.track=t.TrackingReference(self.refs,self.c.kin.limits);self.ramp=self.plan['rampS'];self.initial=self.refs.curve(0.)
 def state(self,time,phase='transfer',phase_elapsed=0.,landed=None):return self.track.reference(time,self.ramp,phase,phase_elapsed,self.initial,landed)
 def test_exact_chain_rule_matches_finite_differences(self):
  for time in [2.123,5.4,8.2]:
   eps=1e-4;x=self.state(time);before=self.state(time-eps);after=self.state(time+eps)
   np.testing.assert_allclose(x['velocity'],(after['position']-before['position'])/(2*eps),atol=1e-8)
   np.testing.assert_allclose(x['acceleration'],(after['velocity']-before['velocity'])/(2*eps),atol=1e-7)
 def test_clamped_clock_has_zero_hold_derivatives(self):
  for time in [-1.,0.,1.,1.+self.ramp,20.]:
   x=self.state(time);np.testing.assert_array_equal(x['velocity'],np.zeros(21));np.testing.assert_array_equal(x['acceleration'],np.zeros(21))
 def test_fixed_radius_covers_interpolated_forward_limit_margin(self):
  self.assertTrue(np.all(self.track.radius[:15]<=.03));self.assertGreaterEqual(min(self.track.radius[:15]),.005-1e-12)
  for q in self.refs.curve(np.linspace(0,1,101)):
   self.assertTrue(np.all(q[:15]-self.track.radius[:15]>=self.c.kin.limits[:15,0]-1e-12));self.assertTrue(np.all(q[:15]+self.track.radius[:15]<=self.c.kin.limits[:15,1]+1e-12))
 def test_only_swing_chain_is_exempt_and_landed_posture_is_not_reset(self):
  self.assertEqual(self.state(0.)['indices'],list(range(15)));landed=np.arange(6)*.1
  for phase in ['lift','translate','land','return','stand']:
   x=self.state(10.,phase,.8,landed);self.assertEqual(x['indices'],list(range(9)))
   if phase in ['return','stand']:np.testing.assert_array_equal(x['position'][9:15],landed);np.testing.assert_array_equal(x['velocity'][9:15],np.zeros(6))
 def test_opt_in_arm_references_are_tracked_and_retained_after_landing(self):
  import copy
  plan=copy.deepcopy(self.plan);q0=np.array(plan['pathKnots'][0]['q']);rest=q0.copy();rest[15:]=np.clip(rest[15:],self.c.kin.limits[15:,0]+.1,self.c.kin.limits[15:,1]-.1)
  for row in plan['pathKnots']:row['q'][15:]=(q0[15:]+row['weight']*(rest[15:]-q0[15:])).tolist()
  tracker=t.TrackingReference(p.FeasibleReferences(plan,self.c),self.c.kin.limits,include_arms=True);start=tracker.reference(0.,self.ramp,'transfer',0.,q0);self.assertEqual(start['indices'],list(range(21)));np.testing.assert_array_equal(start['position'][15:],q0[15:]);np.testing.assert_allclose(tracker.radius[15:],.03)
  for phase in ['lift','translate','land','return','stand']:
   x=tracker.reference(12.,self.ramp,phase,2.,q0,np.arange(6)*.1);self.assertEqual(x['indices'],list(range(9))+list(range(15,21)));np.testing.assert_allclose(x['position'][15:],rest[15:]);np.testing.assert_array_equal(x['velocity'][15:],np.zeros(6));np.testing.assert_array_equal(x['acceleration'][15:],np.zeros(6))
  with self.assertRaisesRegex(AssertionError,'Interior terminal'):t.TrackingReference(self.refs,self.c.kin.limits,include_arms=True)
 def test_initial_admissibility_and_velocity_violation_are_visible(self):
  x=self.state(0.);good=t.admissibility(self.initial,np.zeros(21),x);self.assertGreater(min(good.values()),0.);bad=t.admissibility(self.initial,np.full(21,1.),x);self.assertLess(bad['minimumFirstOrderBarrierMarginRadps'],0.)
if __name__=='__main__':unittest.main()
