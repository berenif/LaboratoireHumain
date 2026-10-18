"""Zero-integration tests for the optional support-reference planner."""
import ctypes as C,unittest
import numpy as np
from source_step_probe import World,Controller,ROOT
from plan_source_stance import plan
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z';HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
@unittest.skipUnless(BASE.exists()and HELPER.exists(),'local original source engine absent')
class StanceReference(unittest.TestCase):
 def test_static_reference_is_pure_contact_qualified_and_motor_bounded(self):
  w=World(BASE,HELPER)
  try:
   w.restore(BASE/'slow-continuation/final.bin');size=w.l.lh_checkpoint_size(w.h);before=C.create_string_buffer(size);w.l.lh_checkpoint_export(w.h,before,size);count=w.l.lh_dim(w.h,6);r=plan(w,1,True,.9);other=plan(w,0,True,.9);self.assertGreater(np.linalg.norm((np.array(r['comReferenceSource'])-other['comReferenceSource'])[:2]),.12);self.assertTrue(other['staticMotorContactFeasible']);after=C.create_string_buffer(size);w.l.lh_checkpoint_export(w.h,after,size);self.assertEqual(before.raw,after.raw);self.assertEqual(w.l.lh_dim(w.h,6),count);self.assertTrue(r['staticOnly']);self.assertEqual(r['physicsIntegrations'],0);self.assertTrue(r['staticMotorContactFeasible']);self.assertLess(r['minimumRequiredMotorFraction'],.925);self.assertGreaterEqual(r['minimumJointReserveRad'],.01999999);self.assertTrue(r['stanceContactPoseFeasible']);self.assertGreater(r['minimumSelfSeparationM'],.01)
  finally:w.close()
 def test_time_scale_preserves_geometric_target_and_changes_joint_commands(self):
  w=World(BASE,HELPER)
  try:
   w.restore(BASE/'slow-continuation/final.bin');c=Controller(w);u,a=c.solve(.5);c.trajectory_time_scale=2.;v,b=c.solve(1.);np.testing.assert_array_equal(a['comTarget'],b['comTarget']);self.assertGreater(max(abs(u-v)),1e-6);self.assertTrue(np.isfinite(v).all());self.assertLessEqual(max(abs(v)),.95000001);self.assertEqual(w.l.lh_dim(w.h,6),6400)
  finally:w.close()
if __name__=='__main__':unittest.main()
