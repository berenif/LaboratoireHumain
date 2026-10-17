#!/usr/bin/env python3
"""Pure inverse-dynamics/observer tests; no Native process or physics."""
import copy,importlib.util,json,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('contact',ROOT/'scripts/getup-contact-dynamics.py');d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
class ContactDynamicsTests(unittest.TestCase):
 def setUp(self):
  self.x=json.loads((ROOT/'docs/evidence/getup-recovery-session-2026-10-17/quiet-transfer-offline/plan.json').read_text());self.c=d.ContactDynamics(json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text()));self.state=self.c.state(self.x['initial']['observation'],self.x['snapshot'],self.x['diagnostic']);self.R=d.normalized_rotation(self.state['root'][:3,:3])
 def test_capsule_clearance_uses_all_endpoints_and_angular_velocity(self):
  st=copy.deepcopy(self.state);root=st['root'].copy();root[1,3]+=.08;st['v']=np.zeros(27);st['v'][3]=1.;root2=root.copy();root2[:3,:3]=d.Rotation.from_rotvec([1e-5,0.,0.]).as_matrix()@root[:3,:3];st['geometry']=self.c.geometry(st['q'],root);st['geometryNext']=self.c.geometry(st['q'],root2)
  rows=self.c.swing_clearance_points(st);self.assertEqual(len(rows),4);self.assertEqual(len(set(r['name']for r in rows)),4);self.assertAlmostEqual(min(r['height']for r in rows),self.c.kin.bottom('left_foot',st['geometry'][0]['left_foot']));self.assertGreater(max(r['velocity']for r in rows)-min(r['velocity']for r in rows),.05)
 def test_capsule_clearance_prevents_interior_parabolic_dip(self):
  floor=.015;height=floor+55e-6;velocity=-.01;acc=d.capsule_clearance_acceleration_lower(height,velocity);self.assertGreater(acc,.8)
  times=np.linspace(0.,.025,1001);self.assertGreaterEqual(min(height+velocity*times+.5*acc*times**2),floor-1e-10)
  with self.assertRaises(ValueError):d.capsule_clearance_acceleration_lower(floor-1e-6,.01)
 def test_capsule_future_buffer_does_not_relax_current_physical_clearance(self):
  for height,velocity in [(.0164,-.05),(.01534,-.031),(.01505,.001)]:
   current=height-max(-velocity,0.)**2/4.-.015;self.assertGreater(current,0.);a=d.capsule_clearance_acceleration_lower(height,velocity,future_reserve=.00025);v=velocity+.025*a;h=height+.025*velocity+.5*.025**2*a
   self.assertGreaterEqual(h-max(-v,0.)**2/4.-.015,min(.00025,current)-1e-10);times=np.linspace(0.,.025,1001);self.assertGreaterEqual(min(height+velocity*times+.5*a*times**2),.015-1e-10)
   self.assertGreaterEqual(a,d.capsule_clearance_acceleration_lower(height,velocity)-1e-10)
  with self.assertRaises(ValueError):d.capsule_clearance_acceleration_lower(.01499,.1,future_reserve=.00025)
  with self.assertRaises(ValueError):d.capsule_clearance_acceleration_lower(.01501,-.01,future_reserve=.00025)
 def test_impact_speed_recovery_preserves_height_and_acceleration_bounds(self):
  for sign in [-1.,1.]:
   v=sign*.08474382;old=d.vertical_speed_acceleration_bounds(-.3,.3,v);self.assertGreater(old[0],old[1]);lo,hi=d.vertical_speed_acceleration_bounds(-.3,.3,v,True);self.assertLessEqual(lo,hi);self.assertGreaterEqual(lo,-.3);self.assertLessEqual(hi,.3)
   for a in [lo,hi]:self.assertLess(abs(v+.025*a),abs(v))
  self.assertEqual(d.vertical_speed_acceleration_bounds(-.3,.3,.01),d.vertical_speed_acceleration_bounds(-.3,.3,.01,True))
 def test_frozen_rotating_toe_clearance_guard_preserves_physical_caps(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/whole-capsule-clearance-state.json').read_text());c=d.ContactDynamics(self.c.kin.model,contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4,force_jacobian='native-frozen',joint_stop_mode='native-bound-release',joint_reserve_recovery=True,friction_mode='active-sticking',tracking_priority='balance-first');c.flat_targets={k:np.array(v)for k,v in x['flatTargets'].items()};state=c.state(x['observation'],x['snapshot'],x['diagnostic']);_,info=c.solve(state,np.array(x['targetCOM']),np.array(x['targetRotation']),np.array(x['posture']),swing={k:np.array(v)if isinstance(v,list)else v for k,v in x['swing'].items()},tracking=x['tracking'],balance=x['balance']);self.assertTrue(info['success']);self.assertLess(info['equalityError'],1e-4);self.assertLessEqual(info['maximumMotorFraction'],.900001)
  for point in c.swing_clearance_points(state):
   acceleration=point['J']@np.array(info['qdd'])+point['bias'];self.assertGreaterEqual(acceleration,point['lowerAcceleration']-1e-6);v=point['velocity']+.025*acceleration;h=point['height']+.025*point['velocity']+.5*.025**2*acceleration;self.assertGreaterEqual(h-min(v,0.)**2/4.,.015-1e-8)
 def test_actual_generalized_velocity_not_snapshot_cache(self):
  snapshot=copy.deepcopy(self.x['snapshot'])
  for p in snapshot['segments']:p['linearVelocity']={'x':999,'y':999,'z':999};p['angularVelocity']={'x':999,'y':999,'z':999}
  state=self.c.state(self.x['initial']['observation'],snapshot,self.x['diagnostic']);np.testing.assert_array_equal(state['v'],self.state['v'])
  self.assertLess(state['comParityErrorM'],1e-5);self.assertLess(state['comVelocityParityErrorMps'],2e-5);self.assertLess(state['contactVelocityParityErrorMps'],2e-5)
 def test_native_force_lever_map_does_not_change_material_point_kinematics(self):
  x=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())['quiet'];states=[]
  for mode in ['live-geometric','native-frozen']:
   c=d.ContactDynamics(self.c.kin.model,force_jacobian=mode);st=c.state(x['observation'],x['snapshot'],x['diagnostic']);states.append(st)
   for con in st['contacts']:
    if mode=='native-frozen':
     pose=st['geometry'][0][con['body']];com=pose[:3,3]+d.normalized_rotation(pose[:3,:3])@d.reference.xyz(c.by_id[con['body']]['goldenMass']['centerOfMass']);jcom,jw=c.point_jacobian(st['geometry'],con['body'],com);np.testing.assert_allclose(con['Jforce'],jcom-d.skew(con['forcePoint']-com)@jw,atol=1e-12)
  np.testing.assert_array_equal(states[0]['M'],states[1]['M']);np.testing.assert_array_equal(states[0]['bias'],states[1]['bias'])
  self.assertTrue(any(np.max(abs(a['Jforce']-b['Jforce']))>1e-5 for a,b in zip(states[0]['contacts'],states[1]['contacts'])))
  for a,b in zip(states[0]['contacts'],states[1]['contacts']):np.testing.assert_array_equal(a['J'],b['J']);np.testing.assert_array_equal(a['velocity'],b['velocity']);self.assertEqual(a['gap'],b['gap'])
 def test_native_force_bias_differentiates_world_fixed_solver_lever(self):
  x=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())['failed'];c=d.ContactDynamics(self.c.kin.model,force_jacobian='native-frozen');st=c.state(x['observation'],x['snapshot'],x['diagnostic'])
  for con in st['contacts']:
   body=con['body'];pose=st['geometry'][0][body];pose2=st['geometryNext'][0][body];local=d.reference.xyz(c.by_id[body]['goldenMass']['centerOfMass']);com=pose[:3,3]+d.normalized_rotation(pose[:3,:3])@local;com2=pose2[:3,3]+d.normalized_rotation(pose2[:3,:3])@local;rho=con['forcePoint']-com;future=c.point_jacobian(st['geometryNext'],body,com2+rho)[0];np.testing.assert_allclose(con['forceBias'],(future-con['Jforce'])@st['v']/st['directionalEpsilon'],atol=1e-11);np.testing.assert_allclose(con['forceVelocity'],con['Jforce']@st['v'],atol=1e-12)
 def test_restricted_sticking_endpoint_velocity_is_zero_at_active_contacts(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/joint-stop-tracking-state.json').read_text());c=d.ContactDynamics(self.c.kin.model,contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4,force_jacobian='native-frozen',joint_stop_mode='native-bound-release',joint_reserve_recovery=True,friction_mode='active-sticking');c.flat_targets={k:np.array(v)for k,v in x['flatTargets'].items()};st=c.state(x['observation'],x['snapshot'],x['diagnostic']);_,info=c.solve(st,np.array(x['targetCOM']),np.array(x['targetRotation']),np.array(x['posture']),tracking=x['tracking']);self.assertTrue(info['success']);qdd=np.array(info['qdd'])
  for i,con in enumerate(st['contacts']):
   if i not in info['inactiveContacts']:np.testing.assert_allclose(con['basis'][:,:2].T@(con['forceVelocity']+.025*(con['Jforce']@qdd+con['forceBias'])),0.,atol=1e-8)
  self.assertLess(info['equalityError'],1e-4);self.assertLessEqual(info['maximumMotorFraction'],.900001)
 def test_all_joint_limits_match_native_f32_and_json_round_trip(self):
  expected=np.zeros((21,2))
  for part in self.c.parts:
   h=part['reference'].get('hinge')
   if h:
    axis=part['jointProfile']['axes'][0];expected[h['qposIndex']]=np.array([axis['minRadians'],axis['maxRadians']],dtype=np.float32).astype(float)
  np.testing.assert_array_equal(self.c.kin.limits,expected);np.testing.assert_array_equal(json.loads(json.dumps(expected.tolist())),expected)
 def test_mass_is_symmetric_positive_definite(self):
  np.testing.assert_allclose(self.state['M'],self.state['M'].T,atol=1e-12);self.assertGreater(min(np.linalg.eigvalsh(self.state['M'])),.01)
 def test_tracking_band_intersects_acceleration_without_changing_motor_caps(self):
  track=dict(indices=[0],position=self.state['q'].copy(),velocity=self.state['v'][6:].copy(),acceleration=np.zeros(21),radius=np.full(21,.01));_,info=self.c.solve(self.state,self.state['com'],self.R,self.state['q'],tracking=track);self.assertTrue(info['success']);self.assertLessEqual(abs(info['qdd'][6]),.160001);self.assertLessEqual(info['maximumMotorFraction'],.900001);self.assertEqual(info['trackingConstraint']['indices'],[0]);self.assertLess(info['boundsError'],1e-6)
 def test_opt_in_arm_tracking_uses_original_joint_caps(self):
  self.c.tracking_priority='physical-first';position=self.state['q'].copy();position[17]+=.01;tracking=dict(indices=[17,20],position=position,velocity=np.zeros(21),acceleration=np.zeros(21),radius=np.full(21,.03));_,info=self.c.solve(self.state,self.state['com'],self.R,self.state['q'],tracking=tracking);self.assertTrue(info['success']);self.assertEqual(info['trackingConstraint']['indices'],[17,20]);self.assertLessEqual(info['maximumMotorFraction'],.900001);self.assertLess(info['equalityError'],1e-4)
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
class JointStopModeTests(unittest.TestCase):
 def setUp(self):
  self.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());self.c=d.ContactDynamics(self.model,joint_stop_mode='native-bound-release',joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4);self.state=dict(q=self.c.kin.limits.mean(axis=1),v=np.zeros(27),contacts=[])
 def test_reactions_require_actual_native_bound_and_nonseparating_velocity(self):
  for i,sign in [(7,1.),(3,-1.)]:
   low,high=self.c.kin.limits[i];bound=low if sign>0 else high
   for offset,velocity,eligible in [(75e-6*sign,-.002*sign,False),(0.,-.002*sign,True),(0.,.002*sign,False),(-50e-6*sign,-.002*sign,True),(-150e-6*sign,-.002*sign,False),(0.,-.06*sign,False)]:
    with self.subTest(joint=i,offset=offset,velocity=velocity):
     state=copy.deepcopy(self.state);state['q'][i]=bound+offset;state['v'][6+i]=velocity;candidates=self.c.bound_stop_candidates(state);self.assertEqual(any(row[0]==i for row in candidates),eligible)
     if eligible:self.assertIn((i,sign,bound),candidates)
 def test_airborne_ankle_limit_is_an_internal_constraint(self):
  state=copy.deepcopy(self.state);state['q'][13]=self.c.kin.limits[13,0];swing=dict(body='left_foot');self.assertIn(13,[r[0]for r in self.c.bound_stop_candidates(state,swing)])
 def test_free_branch_is_preferred_and_active_fallback_is_explicit(self):
  class SelectionProbe(d.ContactDynamics):
   def solve(self,state,*args,_released_stops=None,**kwargs):
    # Recursion passes its mode positionally, like the production wrapper.
    released=args[8] if len(args)>8 else _released_stops
    if released is None:return super().solve(state,*args,**kwargs)
    self.previous=np.ones(1);return np.zeros(21),dict(success=self.accept_free or not released,activeStops=[] if released else [dict(qpos=7,reactionNm=1.)])
  for accept_free,expected in [(True,[7]),(False,[])]:
   c=SelectionProbe(self.model,joint_stop_mode='native-bound-release',joint_viability=True,joint_reserve_margin=1e-4);c.accept_free=accept_free;state=copy.deepcopy(self.state);state['q'][7]=c.kin.limits[7,0];_,info=c.solve(state,None,None,None);self.assertTrue(info['success']);self.assertEqual(info['releasedJointStops'],expected)
 def test_reached_bound_zero_hold_retains_only_legitimate_signed_reactions(self):
  x=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())['quiet'];state=self.c.state(x['observation'],x['snapshot'],x['diagnostic']);_,info=self.c.solve(state,state['com'],d.normalized_rotation(state['root'][:3,:3]),state['q']);self.assertTrue(info['success']);candidates={r[0]for r in self.c.bound_stop_candidates(state)};self.assertTrue(all(r['qpos']in candidates and r['reactionNm']>=0. for r in info['activeStops']));self.assertNotIn(7,[r['qpos']for r in info['activeStops']]);self.assertLess(info['equalityError'],1e-4)
 def test_stopped_tracking_state_keeps_strict_future_buffer_rejection(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/joint-stop-tracking-state.json').read_text());c=d.ContactDynamics(self.model,contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4,force_jacobian='native-frozen',joint_stop_mode='native-bound-release');c.flat_targets={k:np.array(v)for k,v in x['flatTargets'].items()};state=c.state(x['observation'],x['snapshot'],x['diagnostic']);self.assertNotIn(7,[r[0]for r in c.bound_stop_candidates(state)]);_,info=c.solve(state,np.array(x['targetCOM']),np.array(x['targetRotation']),np.array(x['posture']),tracking=x['tracking']);self.assertFalse(info['success']);self.assertEqual(info['error'],'No admissible joint-stop/contact mode')

 def test_positive_buffer_recovery_and_actual_bound_release_solve_stopped_state(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/joint-stop-tracking-state.json').read_text());c=d.ContactDynamics(self.model,contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4,force_jacobian='native-frozen',joint_stop_mode='native-bound-release',joint_reserve_recovery=True);c.flat_targets={k:np.array(v)for k,v in x['flatTargets'].items()};state=c.state(x['observation'],x['snapshot'],x['diagnostic']);self.assertNotIn(7,[r[0]for r in c.bound_stop_candidates(state)]);_,info=c.solve(state,np.array(x['targetCOM']),np.array(x['targetRotation']),np.array(x['posture']),tracking=x['tracking']);self.assertTrue(info['success']);self.assertNotIn(7,[r['qpos']for r in info['activeStops']]);a=info['qdd'][13];q=state['q'][7];v=state['v'][13];low=c.kin.limits[7,0];current=q-min(v,0.)**2/100.-low;future=q+.025*v+.5*.025**2*a-min(v+.025*a,0.)**2/100.-low;self.assertGreaterEqual(future,current-1e-9);self.assertLess(info['equalityError'],1e-4)

class TrackingPriorityTests(unittest.TestCase):
 def test_scalar_minimum_slack_preserves_physics_and_mirrors_signs(self):
  np.testing.assert_allclose(d.intersect_tracking_acceleration(1.3,50.,.26,.42,True),(1.3,1.3,.88));np.testing.assert_allclose(d.intersect_tracking_acceleration(-50.,-1.3,-.42,-.26,True),(-1.3,-1.3,.88));self.assertEqual(d.intersect_tracking_acceleration(-2.,3.,-1.,2.,True),(-1.,2.,0.));self.assertEqual(d.intersect_tracking_acceleration(-2.,3.,-4.,4.,True),(-2.,3.,0.));self.assertEqual(d.intersect_tracking_acceleration(1.3,50.,.26,.42),(1.3,.42,0.))
  with self.assertRaisesRegex(ValueError,'Empty physical'):d.intersect_tracking_acceleration(2.,1.,0.,3.,True)
 def test_exact_braking_conflict_relaxes_only_necessary_tracking_joint(self):
  x=json.loads((ROOT/'docs/fixtures/contact-landing-mvp/tracking-braking-conflict-state.json').read_text());model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
  for priority,expected in [('hard-band',False),('physical-first',True)]:
   c=d.ContactDynamics(model,contact_mode='unilateral-soft',flat_soles=True,joint_viability=True,qp_solver='clarabel',joint_reserve_margin=1e-4,force_jacobian='native-frozen',joint_stop_mode='native-bound-release',joint_reserve_recovery=True,friction_mode='active-sticking',tracking_priority=priority);c.flat_targets={k:np.array(v)for k,v in x['flatTargets'].items()};st=c.state(x['observation'],x['snapshot'],x['diagnostic']);_,info=c.solve(st,np.array(x['targetCOM']),np.array(x['targetRotation']),np.array(x['posture']),tracking=x['tracking']);self.assertEqual(info['success'],expected)
   if expected:self.assertEqual([r['qpos']for r in info['trackingConstraint']['physicalPriorityRelaxations']],[7]);self.assertAlmostEqual(info['qdd'][13],1.299539752,places=7);self.assertLess(info['equalityError'],1e-4);self.assertLessEqual(info['maximumMotorFraction'],.900001)
 def test_legitimate_active_stop_equality_also_outranks_tracking(self):
  model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());x=json.loads((ROOT/'docs/fixtures/getup-contact-modes.json').read_text())['quiet'];c=d.ContactDynamics(model,joint_stop_mode='native-bound-release',joint_viability=True,joint_reserve_margin=1e-4,tracking_priority='physical-first',qp_solver='clarabel');st=c.state(x['observation'],x['snapshot'],x['diagnostic']);position=st['q'].copy();position[13]+=.02;tracking=dict(indices=[13],position=position,velocity=np.zeros(21),acceleration=np.zeros(21),radius=np.full(21,.005));_,info=c.solve(st,st['com'],d.normalized_rotation(st['root'][:3,:3]),st['q'],tracking=tracking);self.assertTrue(info['success']);self.assertIn(13,[r['qpos']for r in info['activeStops']]);self.assertEqual([r['qpos']for r in info['trackingConstraint']['physicalPriorityRelaxations']],[13]);self.assertLess(info['equalityError'],1e-4)
 def test_physical_first_does_not_waive_negative_current_reserve(self):
  with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(-1.+30.235e-6,-.06391884,-1.,1.,reserve=1e-4,reserve_recovery=True)

class BrakingBoundsTests(unittest.TestCase):
 def test_recovery_buffer_is_mirrored_nondecreasing_and_keeps_negative_reserve_rejection(self):
  for q,v in [(-1.+75.519e-6,-.00256488),(1.-75.519e-6,.00256488)]:
   lo,hi=d.braking_acceleration_bounds(q,v,-1.,1.,reserve=1e-4,reserve_recovery=True)
   current_lower=q-min(v,0.)**2/100.+1.;current_upper=1.-q-max(v,0.)**2/100.
   for a in np.linspace(lo,hi,7):
    p=q+.025*v+.5*.025**2*a;w=v+.025*a;self.assertGreaterEqual(p-min(w,0.)**2/100.+1.,min(1e-4,current_lower)-1e-9);self.assertGreaterEqual(1.-p-max(w,0.)**2/100.,min(1e-4,current_upper)-1e-9)
  a,b=d.braking_acceleration_bounds(-1.+75.519e-6,-.00256488,-1.,1.,reserve=1e-4,reserve_recovery=True);lo,hi=d.braking_acceleration_bounds(1.-75.519e-6,.00256488,-1.,1.,reserve=1e-4,reserve_recovery=True);self.assertAlmostEqual(a,-hi);self.assertAlmostEqual(b,-lo)
  for q,v in [(-1.+30.235e-6,-.06391884),(1.-30.235e-6,.06391884)]:
   with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(q,v,-1.,1.,reserve=1e-4,reserve_recovery=True)
 def test_recovery_does_not_change_outside_buffer_or_neighbor_future_guard(self):
  for q,v in [(-.9,-.1),(.9,.1),(-1.+.001,0.),(1.-.001,0.)]:self.assertEqual(d.braking_acceleration_bounds(q,v,-1.,1.,reserve=1e-4),d.braking_acceleration_bounds(q,v,-1.,1.,reserve=1e-4,reserve_recovery=True))
  low=float(np.float32(-.34906585));high=float(np.float32(.78539816));q=float(np.nextafter(np.float32(low),np.float32(-np.inf)));self.assertEqual(d.braking_acceleration_bounds(q,.01,low,high,reserve=1e-4,native_float32_neighbors=True),d.braking_acceleration_bounds(q,.01,low,high,reserve=1e-4,native_float32_neighbors=True,reserve_recovery=True))
 def test_native_neighbor_release_is_one_ulp_separating_only_and_future_safe(self):
  low=float(np.float32(-.3490658503988659));high=float(np.float32(.7853981633974483));below=float(np.nextafter(np.float32(low),np.float32(-np.inf)));above=float(np.nextafter(np.float32(high),np.float32(np.inf)))
  for q,v in [(below,.01084318),(above,-.01084318)]:
   self.assertTrue(d.separating_float32_neighbor(q,v,low,high));a,b=d.braking_acceleration_bounds(q,v,low,high,reserve=1e-4,native_float32_neighbors=True)
   for dd in [a,b]:
    p=q+.025*v+.5*.025**2*dd;w=v+.025*dd;self.assertGreaterEqual(p-min(w,0.)**2/100.,low+1e-4-1e-9);self.assertLessEqual(p+max(w,0.)**2/100.,high-1e-4+1e-9)
   for velocity in [0.,-v]:
    self.assertFalse(d.separating_float32_neighbor(q,velocity,low,high))
    with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(q,velocity,low,high,native_float32_neighbors=True)
  for q,v,direction in [(below,.01,-np.inf),(above,-.01,np.inf)]:
   farther=float(np.nextafter(np.float32(q),np.float32(direction)));self.assertFalse(d.separating_float32_neighbor(farther,v,low,high))
   with self.assertRaisesRegex(ValueError,'Already outside'):d.braking_acceleration_bounds(farther,v,low,high,native_float32_neighbors=True)
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
