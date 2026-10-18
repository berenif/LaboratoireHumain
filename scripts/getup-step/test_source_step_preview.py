"""Live adapter checks; integrations are explicit, bounded and retained locally."""
import json,threading,time,unittest,urllib.request,urllib.error
from pathlib import Path
import numpy as np
from source_step_preview import LiveStep,serve,ROOT
BASE=ROOT/'outputs/getup-source-engine/session-20261017T233404Z';HELPER=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v5.so'
OUT=ROOT/'outputs/getup-source-step/live-preview-tests'
@unittest.skipUnless(BASE.exists()and HELPER.exists(),'local source assets absent')
class Preview(unittest.TestCase):
 def setUp(self):
  folder=OUT/self._testMethodName;folder.mkdir(parents=True,exist_ok=False);self.live=LiveStep(BASE,HELPER,folder);self.rid=0
 def tearDown(self):self.live.close()
 def send(self,kind,**kw):
  self.rid+=1;return self.live.handle(dict(type=kind,requestId=self.rid,**kw))
 def command(self,action):
  return self.send('commands',commands=[dict(stamp=dict(schema=4,generation=self.live.generation,sequence=self.live.command_sequence+1,tick=self.live.steps//20),action=action)])
 def test_pause_stops_physics(self):
  self.send('advance',ticks=4);self.assertEqual(self.live.total,0);self.command('Resume');self.send('advance',ticks=2);self.assertEqual(self.live.total,40);self.command('Pause');before=self.live.world.arr(0,(28,)).copy();self.send('advance',ticks=4);time.sleep(.05);np.testing.assert_array_equal(self.live.world.arr(0,(28,)),before);self.assertEqual(self.live.total,40)
 def test_reset_determinism_and_monotonic_charges(self):
  self.command('Resume');self.send('advance',ticks=2);q=self.live.world.arr(0,(28,)).copy();v=self.live.world.arr(1,(27,)).copy();self.command('Reset');self.assertEqual(self.live.steps,0);self.assertEqual(self.live.total,40);self.assertTrue(self.live.paused);self.command('Resume');self.send('advance',ticks=2);np.testing.assert_array_equal(self.live.world.arr(0,(28,)),q);np.testing.assert_array_equal(self.live.world.arr(1,(27,)),v);self.assertEqual(self.live.total,80)
 def test_stale_requests_never_integrate(self):
  self.send('initialize');before=self.live.total
  with self.assertRaisesRegex(AssertionError,'stale request'):self.live.handle(dict(type='advance',ticks=4,requestId=self.rid))
  self.assertEqual(self.live.total,before)
 def test_existing_renderer_packet_identity(self):
  reply=self.send('initialize');snapshot=reply['snapshot'];self.assertEqual(len(snapshot['segments']),25);self.assertEqual(len(reply['sourceStepLab']['jointGroundRaysWorld']),22)
  for part,pose in zip(self.live.model['segments'],snapshot['segments']):
   self.assertEqual(part['id'],pose['id']);values=[pose[key][axis]for key in ['position','rotation','linearVelocity','angularVelocity']for axis in('xyzw'if key=='rotation'else'xyz')];self.assertEqual(len(values),13);np.testing.assert_array_equal(np.array(values,dtype=np.float32).astype(float),values)
  left=snapshot['segments'][16]['position'];expected=np.array(self.live.row['feet'][0]['position'])[[1,2,0]];np.testing.assert_allclose(list(left.values()),expected,atol=3e-8)
 def test_loopback_origin_guard(self):
  server=serve(self.live,ROOT/'rust/dist/app',0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();url=f'http://127.0.0.1:{server.server_port}/__source_step/action'
  try:
   body=json.dumps(dict(type='advance',ticks=4,requestId=1)).encode();bad=urllib.request.Request(url,body,headers={'Content-Type':'application/json','Origin':'https://example.com'})
   with self.assertRaises(urllib.error.HTTPError)as caught:urllib.request.urlopen(bad)
   self.assertEqual(caught.exception.code,403);self.assertEqual(self.live.total,0)
   good=urllib.request.Request(url,body,headers={'Content-Type':'application/json','Origin':url.split('/__source_step')[0]});reply=json.load(urllib.request.urlopen(good));self.assertEqual(reply['sourceStepLab']['nativeSteps'],0)
  finally:server.shutdown();server.server_close();thread.join()
 def test_full_live_step_is_measured(self):
  self.command('Resume')
  for _ in range(161):
   self.send('advance',ticks=4)
   if self.live.finished or self.live.failure:break
  self.assertIsNone(self.live.failure);self.assertTrue(self.live.finished);self.assertEqual(self.live.steps,9605);self.assertEqual(self.live.quiet_run,4000);self.assertGreaterEqual(self.live.row['forwardFootDisplacementM'],.06);self.assertLess(max(self.live.slip),.01);self.assertTrue(self.live.paused)
if __name__=='__main__':unittest.main()
