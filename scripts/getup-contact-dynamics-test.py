#!/usr/bin/env python3
"""Pure inverse-dynamics/observer tests; no Native process or physics."""
import copy,importlib.util,json,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('contact',ROOT/'scripts/getup-contact-dynamics.py');d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
class ContactDynamicsTests(unittest.TestCase):
 def setUp(self):
  self.x=json.loads((ROOT/'docs/evidence/getup-recovery-session-2026-10-17/quiet-transfer-offline/plan.json').read_text());self.c=d.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));self.state=self.c.state(self.x['initial']['observation'],self.x['snapshot'],self.x['diagnostic']);self.R=d.normalized_rotation(self.state['root'][:3,:3])
 def test_actual_generalized_velocity_not_snapshot_cache(self):
  snapshot=copy.deepcopy(self.x['snapshot'])
  for p in snapshot['segments']:p['linearVelocity']={'x':999,'y':999,'z':999};p['angularVelocity']={'x':999,'y':999,'z':999}
  state=self.c.state(self.x['initial']['observation'],snapshot,self.x['diagnostic']);np.testing.assert_array_equal(state['v'],self.state['v'])
  self.assertLess(state['comParityErrorM'],1e-5);self.assertLess(state['comVelocityParityErrorMps'],2e-5);self.assertLess(state['contactVelocityParityErrorMps'],2e-5)
 def test_mass_is_symmetric_positive_definite(self):
  np.testing.assert_allclose(self.state['M'],self.state['M'].T,atol=1e-12);self.assertGreater(min(np.linalg.eigvalsh(self.state['M'])),.01)
 def test_sticking_zero_qp_and_torque_interface(self):
  a,info=self.c.solve(self.state,self.state['com'],self.R,self.state['q']);self.assertTrue(info['success']);self.assertTrue(info['constraintFeasible']);self.assertLess(info['boundsError'],1e-6);self.assertLess(info['equalityError'],1e-4);self.assertLess(info['maximumTorqueRepresentationErrorNm'],1e-10);self.assertLessEqual(max(abs(a)),1.)
  self.assertTrue(all(r['reactionNm']>=0 for r in info['activeStops']))
 def test_small_shift_predicted_acceleration_has_right_sign(self):
  delta=np.array([-.003,0.,.004]);_,info=self.c.solve(self.state,self.state['com']+delta,self.R,self.state['q']);self.assertTrue(info['success']);self.assertGreater(np.array(info['achievedComAcceleration'])@delta,0.)
 def test_contact_segment_projection_uses_no_hidden_margin(self):
  point,distance=d.support_projection([[0.,0.],[1.,0.],[.5,0.]],[.4,.007]);np.testing.assert_allclose(point,[.4,0.]);self.assertAlmostEqual(distance,.007)
 def test_support_polygon_contains_point_exactly(self):
  point,distance=d.support_projection([[0.,0.],[1.,0.],[0.,1.]],[.2,.2]);self.assertEqual(distance,0.)
 def test_reloaded_foot_task_keeps_legitimate_ankle_stop(self):
  pose=self.state['geometry'][0]['left_foot'];swing=dict(body='left_foot',position=pose[:3,3],rotation=d.normalized_rotation(pose[:3,:3]))
  _,info=self.c.solve(self.state,self.state['com'],self.R,self.state['q'],swing=swing);self.assertTrue(info['success']);self.assertIn(13,[r['qpos'] for r in info['activeStops']])
 def test_separating_ankle_cannot_get_stop_reaction(self):
  state=copy.deepcopy(self.state);state['v'][13]=.02
  _,info=self.c.solve(state,state['com'],self.R,state['q']);self.assertNotIn(7,[r['qpos'] for r in info['activeStops']])
if __name__=='__main__':unittest.main()
