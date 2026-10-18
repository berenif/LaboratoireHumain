"""Source contact diagnostics: scratch purity and explicitly counted parity."""
import ctypes as C,unittest
import numpy as np
from source_step_probe import World,Controller,ROOT,PTR,ptr
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z';OLD=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v6.so';NEW=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
@unittest.skipUnless(BASE.exists()and OLD.exists()and NEW.exists(),'retained local source assets absent')
class ContactResponse(unittest.TestCase):
 def test_scratch_response_is_pure_and_predicts_one_counted_native_solve(self):
  w=World(BASE,NEW)
  try:
   w.restore(BASE/'slow-continuation/final.bin');c=Controller(w);u,_=c.solve(0.);size=w.l.lh_checkpoint_size(w.h);before=C.create_string_buffer(size);w.l.lh_checkpoint_export(w.h,before,size);steps=w.l.lh_dim(w.h,6);w.l.lh_motor_response.argtypes=[C.c_void_p,PTR,PTR];prediction=w.read('lh_motor_response',(33,),ptr(u));after=C.create_string_buffer(size);w.l.lh_checkpoint_export(w.h,after,size);self.assertEqual(before.raw,after.raw);self.assertEqual(w.l.lh_dim(w.h,6),steps);w.arr(2,(21,))[:]=u;self.assertEqual(w.l.lh_step_audited(w.h,steps+1),steps+1);np.testing.assert_array_equal(prediction[:27],w.arr(15,(27,)));np.testing.assert_allclose(prediction[29::3],[f['loadN']for f in w.packet()['feet']],atol=1e-12,rtol=0)
  finally:w.close()
 def test_readonly_wrench_hook_preserves_exact_physical_trajectory(self):
  a=World(BASE,OLD);b=World(BASE,NEW)
  try:
   for w in [a,b]:w.restore(BASE/'slow-continuation/final.bin')
   ca,cb=Controller(a,0,.032,True),Controller(b,0,.032,True);start=a.l.lh_dim(a.h,6)
   for i in range(80):
    if i%4==0:ua,_=ca.solve(i*.00125);ub,_=cb.solve(i*.00125);np.testing.assert_array_equal(ua,ub)
    for w,u in [(a,ua),(b,ub)]:w.arr(2,(21,))[:]=u;self.assertEqual(w.l.lh_step_audited(w.h,start+80),start+i+1)
    np.testing.assert_array_equal(a.arr(0,(28,)),b.arr(0,(28,)));np.testing.assert_array_equal(a.arr(1,(27,)),b.arr(1,(27,)))
   b.l.lh_last_ground_wrench.argtypes=[C.c_void_p,PTR];forces=b.read('lh_last_ground_wrench',(2,6));np.testing.assert_allclose(forces[:,2],[f['loadN']for f in b.packet()['feet']],atol=1e-12,rtol=0)
  finally:a.close();b.close()
if __name__=='__main__':unittest.main()
