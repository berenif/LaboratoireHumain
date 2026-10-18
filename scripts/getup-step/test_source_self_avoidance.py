"""Zero-integration fixtures for measured arm/thigh separation feedback."""
import ctypes as C,json,unittest
import numpy as np
from source_step_probe import World,Controller,ROOT,PTR,ptr
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z';HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v6.so';FIXTURE=ROOT/'docs/fixtures/getup-step/arm-thigh-contact-witness.json'
@unittest.skipUnless(BASE.exists()and HELPER.exists(),'local original source engine absent')
class SelfAvoidance(unittest.TestCase):
 def test_capsule_separation_matches_measured_native_contact(self):
  fixture=json.loads(FIXTURE.read_text());w=World(BASE,HELPER)
  try:
   w.install_cold(fixture);c=Controller(w,1,.062,True);raw=w.read('lh_self_separation',(897,));pairs=raw[1:1+14*int(raw[0])].reshape(-1,14);pair=next(p for p in pairs if list(map(int,p[:2]))==fixture['expectedPairBodyIds']);self.assertAlmostEqual(pair[4],fixture['expectedSignedSeparationM'],places=12);self.assertEqual(w.l.lh_dim(w.h,6),0)
  finally:w.close()
 def test_active_self_geometry_changes_joint_torques(self):
  fixture=json.loads(FIXTURE.read_text());w=World(BASE,HELPER)
  try:
   w.install_cold(fixture);original,baseline=Controller(w,1,.062).solve(0.,'lift');adjusted,info=Controller(w,1,.062,True).solve(0.,'lift');self.assertEqual(baseline['activeSelfAvoidanceBarriers'],0);self.assertGreater(info['activeSelfAvoidanceBarriers'],0);self.assertGreater(float(max(abs(adjusted-original))),1e-6);self.assertLessEqual(max(abs(adjusted)),.95000001);self.assertEqual(w.l.lh_dim(w.h,6),0)
  finally:w.close()
if __name__=='__main__':unittest.main()
