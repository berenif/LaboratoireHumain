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
 def test_flat_targets_use_true_sole_normal_and_preserve_heading(self):
  c=d.ContactDynamics(self.c.kin.model,flat_soles=True);c.set_flat_targets(self.state)
  for foot,R in c.flat_targets.items():
   np.testing.assert_allclose(R[:,1],[0.,1.,0.],atol=1e-12);np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(R),1.);heading=self.state['geometry'][0][foot][:3,2].copy();heading[1]=0.;heading/=np.linalg.norm(heading);np.testing.assert_allclose(R[:,2],heading,atol=1e-12)
 def test_native_paired_gap_and_impulse_units(self):
  fixtures=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())
  for x in [fixtures['quiet'],fixtures['failed']]:
   cm=x['diagnostic']['contactMotion'];self.assertAlmostEqual(cm['expectedHumanFloorSolverSubstepS'],cm['nativeFrameDtS']/(cm['nativeBaseSolverIterations']+cm['humanMaximumAdditionalSolverIterations']),places=10)
   for foot in cm['feet']:
    self.assertAlmostEqual(sum(c['normalImpulseNs'] for c in foot['contacts'])/cm['nativeFrameDtS'],foot['loadN'],places=3)
    for c in foot['contacts']:self.assertAlmostEqual(np.dot(d.reference.xyz(c['point'])-d.reference.xyz(c['floorPoint']),d.reference.xyz(c['normal'])),c['signedGapM'],places=9)
  opening=fixtures['failed']['diagnostic']['contactMotion']['feet'][1]['contacts'][1];self.assertGreater(abs(opening['signedGapM']-opening['point']['y']),5e-5)
 def test_unilateral_quiet_mode_preserves_support(self):
  x=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())['quiet'];c=d.ContactDynamics(self.c.kin.model,contact_mode='unilateral-soft');state=c.state(x['observation'],x['snapshot'],x['diagnostic'])
  _,info=c.solve(state,state['com'],d.normalized_rotation(state['root'][:3,:3]),state['q']);self.assertTrue(info['success']);self.assertEqual(info['inactiveContacts'],[]);self.assertTrue(all(g<=1e-6 for g in info['predictedContactGapsM']))
 def test_unilateral_rolling_mode_has_no_inactive_force(self):
  fixtures=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text());x=fixtures['failed'];c=d.ContactDynamics(self.c.kin.model,contact_mode='unilateral-soft');state=c.state(x['observation'],x['snapshot'],x['diagnostic']);quiet=c.state(fixtures['quiet']['observation'],fixtures['quiet']['snapshot'],fixtures['quiet']['diagnostic']);target=quiet['com'].copy();target[[0,2]]=[.9854289905196197,-.07612006698020413];pose=state['geometry'][0]['left_foot'];swing=dict(body='left_foot',position=pose[:3,3],rotation=d.normalized_rotation(pose[:3,:3]))
  _,info=c.solve(state,target,d.normalized_rotation(quiet['root'][:3,:3]),quiet['q'],swing=swing);self.assertTrue(info['success']);self.assertIn(1,info['inactiveContacts'])
  for i,(gap,force) in enumerate(zip(info['predictedContactGapsM'],info['plannedContactForcesWorldN'])):
   if i in info['inactiveContacts']:self.assertGreaterEqual(gap,-1e-6);self.assertLess(max(abs(v) for v in force),1e-6)
   else:self.assertLessEqual(gap,1e-6);self.assertGreaterEqual(gap,min(state['contacts'][i]['gap'],0.)-1e-6)
 def test_unilateral_mode_requires_fresh_paired_gap(self):
  c=d.ContactDynamics(self.c.kin.model,contact_mode='unilateral-soft')
  with self.assertRaisesRegex(AssertionError,'Fresh paired gaps'):c.solve(self.state,self.state['com'],self.R,self.state['q'])
 def test_full_solve_viability_keeps_original_equality_rhs_vector(self):
  self.c.joint_viability=True;_,info=self.c.solve(self.state,self.state['com'],self.R,self.state['q']);self.assertTrue(info['success']);self.assertLess(info['equalityError'],1e-4);self.assertGreaterEqual(info['minimumPredictedJointStoppingMarginRad'],-1e-8)
 def test_terminal_load_ceiling_is_feasible_and_does_not_change_hard_caps(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/terminal-unload-state.json').read_text());c=d.ContactDynamics(self.c.kin.model,flat_soles=True);state=c.state(x['observation'],x['snapshot'],x['diagnostic']);ref=x['reference'];c.flat_targets={k:np.array(v) for k,v in ref['flatTargets'].items()}
  _,base=c.solve(state,np.array(ref['targetCOM']),np.array(ref['rootRotation']),np.array(ref['posture']),unload_left=1.);self.assertTrue(base['success']);self.assertGreater(base['plannedLeftNormalForceN'],20.)
  for ceiling in [25.,12.5,0.]:
   _,info=c.solve(state,np.array(ref['targetCOM']),np.array(ref['rootRotation']),np.array(ref['posture']),unload_left=1.,left_force_ceiling_n=ceiling)
   self.assertTrue(info['success']);self.assertLessEqual(info['plannedLeftNormalForceN'],ceiling+1e-4);self.assertLess(info['leftNormalForceCeilingViolationN'],1e-4);self.assertLessEqual(info['maximumMotorFraction'],.900001);self.assertLess(max(abs(np.array(info['qdd']))),10.)
  np.testing.assert_allclose(info['plannedContactForcesWorldN'][0],[0.,0.,0.],atol=1e-5)
class BrakingBoundsTests(unittest.TestCase):
 def test_future_buffer_tightens_both_sides_without_changing_current_bounds(self):
  for q,v in [(.9,1.),(-.9,-1.)]:
   oldlo,oldhi=d.braking_acceleration_bounds(q,v,-1.,1.);lo,hi=d.braking_acceleration_bounds(q,v,-1.,1.,reserve=1e-4)
   self.assertGreaterEqual(lo,oldlo);self.assertLessEqual(hi,oldhi)
   for a in [lo,hi]:
    p=q+.025*v+.5*.025**2*a;w=v+.025*a;self.assertLessEqual(p+max(w,0.)**2/100.,1.-1e-4+1e-9);self.assertGreaterEqual(p-min(w,0.)**2/100.,-1.+1e-4-1e-9)
 def test_current_state_inside_buffer_can_move_away_without_retroactive_rejection(self):
  lo,hi=d.braking_acceleration_bounds(-1.+1e-6,.02,-1.,1.,reserve=1e-4);self.assertLessEqual(lo,hi)
  for reserve in [0.,1e-4]:
   with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(-1.+3.0235e-5,-.06391884,-1.,1.,reserve=reserve)
 def test_buffer_retains_mirrored_signs_and_handles_zero_distance(self):
  lo,hi=d.braking_acceleration_bounds(.99,.2,-1.,1.,reserve=1e-4);a,b=d.braking_acceleration_bounds(-.99,-.2,-1.,1.,reserve=1e-4);self.assertAlmostEqual(lo,-b);self.assertAlmostEqual(hi,-a)
  lo,_=d.braking_acceleration_bounds(-1.,0.,-1.,1.,reserve=1e-4);self.assertGreater(lo,0.)
 def test_measured_knee_case_reserves_braking_distance(self):
  lo,hi=d.braking_acceleration_bounds(-.07425740361213684,2.7617790699005127,-2.792526803190927,.03490658503988659);self.assertAlmostEqual(hi,-23.3655,places=3);self.assertEqual(lo,-50.)
 def test_mirrored_signs_have_mirrored_bounds(self):
  lo,hi=d.braking_acceleration_bounds(.8,2.,-1.,1.);a,b=d.braking_acceleration_bounds(-.8,-2.,-1.,1.);self.assertAlmostEqual(lo,-b);self.assertAlmostEqual(hi,-a)
 def test_at_bound_zero_velocity_cannot_accelerate_outward(self):
  _,hi=d.braking_acceleration_bounds(1.,0.,-1.,1.);lo,_=d.braking_acceleration_bounds(-1.,0.,-1.,1.);self.assertAlmostEqual(hi,0.);self.assertAlmostEqual(lo,0.)
 def test_already_unviable_is_rejected_not_clamped(self):
  for q,v in [(1.-1e-5,.5),(-1.+1e-5,-.5),(.0098747611,3.97580433)]:
   with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(q,v,-1. if abs(q)>.1 else -2.7925268,1. if abs(q)>.1 else .034906585)
 def test_endpoint_and_stopping_reserve_hold_through_returned_interval(self):
  for q,v in [(.9,1.),(-.9,-1.),(.2,0.),(.99,-.5)]:
   lo,hi=d.braking_acceleration_bounds(q,v,-1.,1.)
   for a in np.linspace(lo,hi,9):
    p=q+.025*v+.5*.025**2*a;w=v+.025*a;self.assertLessEqual(p+max(w,0.)**2/100.,1.+1e-9);self.assertGreaterEqual(p-min(w,0.)**2/100.,-1.-1e-9)
if __name__=='__main__':unittest.main()
