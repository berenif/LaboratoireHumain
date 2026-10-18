"""Zero-integration source ground-ray and geometry checks; no policy learning."""
import ctypes as C
import json
import re
import shutil
import tempfile
import unittest
from pathlib import Path
import numpy as np
from source_step_probe import World,Controller,ROOT
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z'
HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v5.so'
@unittest.skipUnless(HELPER.exists()and BASE.exists(),'local original source dependencies absent')
class GroundRays(unittest.TestCase):
 def setUp(self):
  self.world=World(BASE,HELPER);self.world.restore(BASE/'slow-continuation/final.bin')
 def tearDown(self):self.world.close()
 def test_actual_compiled_source_friction(self):
  contacts=self.world.friction_parameters()['effectiveFootGroundContacts'];self.assertGreater(len(contacts),0);self.assertTrue(all(row[4]==1. and row[5]==1. for row in contacts))
 def test_every_joint_has_actual_ground_ray(self):
  w=self.world;before=w.l.lh_dim(w.h,6);rays=w.joint_ground_rays()
  self.assertEqual(len(rays),22);self.assertEqual(len({r['joint']for r in rays}),22)
  self.assertTrue(all(r['hit']and r['geomId']==0 and r['ownBodyExcluded']for r in rays))
  for ray in rays:
   self.assertAlmostEqual(ray['pointSource'][2],0.,places=12)
   self.assertAlmostEqual(ray['distanceM'],ray['originSource'][2],places=12)
   self.assertFalse(ray['anchorGroundDistanceIsColliderClearance'])
  json.dumps(w.packet(),allow_nan=False);self.assertEqual(w.l.lh_dim(w.h,6),before)
 def test_self_geometries_never_win_ray(self):
  w=self.world
  for side in ['left_foot','right_foot','torso']:
   point=w.arr(3,(17,3))[w.ids[side]].copy();ray=w.ground_ray(point)
   self.assertTrue(ray['hit']);self.assertEqual(ray['geomId'],0);self.assertAlmostEqual(ray['distanceM'],point[2],places=12)
 def test_heel_forefoot_clearance_from_ground(self):
  for foot in self.world.feet():
   self.assertEqual(len(foot['groundRays']),4)
   expected=min(point[2]for point in foot['points'])-foot['radius']
   self.assertAlmostEqual(foot['clearance'],expected,places=12)
 def test_no_hit_is_explicit_and_blocks_target(self):
  ray=self.world.ground_ray([0.,0.,1.],[0.,0.,1.]);self.assertFalse(ray['hit']);self.assertIsNone(ray['distanceM']);self.assertIsNone(ray['pointSource'])
  with self.assertRaisesRegex(ValueError,'no-ground-ray-hit'):self.world.ground_height([0.,0.,-2.])
 def test_joint_ground_distance_causally_changes_motor_command(self):
  w=self.world;c=Controller(w);baseline,info=c.solve(0.);self.assertEqual(info['activeJointGroundBarriers'],0)
  original=w.joint_ground_rays;measured=original();target=next(row for row in info['jointGroundAdjustments']if row['joint']=='root');changed=json.loads(json.dumps(measured))
  changed[0]['distanceM']-=target['adjacentColliderClearanceM']-.005;changed[0]['pointSource'][2]=changed[0]['originSource'][2]-changed[0]['distanceM'];w.joint_ground_rays=lambda:changed
  adjusted,after=c.solve(0.);self.assertGreater(after['activeJointGroundBarriers'],0);self.assertGreater(float(np.max(abs(adjusted-baseline))),1e-6)
  w.joint_ground_rays=original
 def test_far_joint_ground_does_not_bias_posture(self):
  w=self.world;c=Controller(w);baseline,info=c.solve(0.);original=w.joint_ground_rays;far=original()
  for ray in far:ray['distanceM']+=1.;ray['pointSource'][2]-=1.
  w.joint_ground_rays=lambda:far;actual,after=c.solve(0.);self.assertEqual(after['activeJointGroundBarriers'],0);np.testing.assert_allclose(actual,baseline,atol=1e-12,rtol=0);w.joint_ground_rays=original
 def test_joint_no_hit_stops_before_command(self):
  w=self.world;c=Controller(w);rays=w.joint_ground_rays();rays[0].update(hit=False,distanceM=None,pointSource=None);w.joint_ground_rays=lambda:rays
  with self.assertRaisesRegex(ValueError,'no-ground-ray-hit-at-joint-root'):c.solve(0.)
 def test_floor_height_comes_from_installed_geometry(self):
  source=BASE/'source-model/humanoid_high_freq.xml';text=source.read_text();text=text.replace('<geom name="floor"','<geom pos="0 0 .17" name="floor"',1)
  with tempfile.TemporaryDirectory()as folder:
   path=Path(folder)/'shifted-floor.xml';path.write_text(text)
   for dependency in source.parent.iterdir():
    if dependency.is_file()and dependency.suffix=='.xml'and dependency.name!=path.name:shutil.copy(dependency,Path(folder)/dependency.name)
   w=World(BASE,HELPER,xml=path)
   try:
    with self.assertRaisesRegex(AssertionError,'cross-model full-cache restore'):w.restore(BASE/'slow-continuation/final.bin')
    forward=w.l.lh_forward;forward.argtypes=[C.c_void_p];forward.restype=None;forward(w.h)
    ray=w.ground_ray([.5,.2,1.3]);self.assertTrue(ray['hit']);self.assertAlmostEqual(ray['distanceM'],1.13,places=12);self.assertAlmostEqual(ray['pointSource'][2],.17,places=12);self.assertEqual(w.l.lh_dim(w.h,6),0)
   finally:w.close()
if __name__=='__main__':unittest.main()
