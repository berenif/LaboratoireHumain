import copy,json,unittest
from pathlib import Path
import numpy as np
from source_task_env import SourceTaskEnv
from source_task import CONFIG_PATH
from task import load_config
ROOT=Path(__file__).resolve().parents[2]
class FakeWorld:
    def __init__(self,row):self.row=copy.deepcopy(row);self.writes=0
    def task_packet(self):return copy.deepcopy(self.row)
    def checkpoint(self):return json.dumps(self.row,sort_keys=True,allow_nan=False).encode()
    def restore(self,blob):self.row=json.loads(blob)
    def arr(self,*_):self.writes+=1;raise AssertionError('disabled environment wrote controls')
class SourceTaskEnvTests(unittest.TestCase):
    def setUp(self):
        self.row=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame'];self.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text());_,self.sha=load_config(CONFIG_PATH)
    def env(self):
        return SourceTaskEnv(FakeWorld(self.row),self.model,config_path=CONFIG_PATH,expected_config_sha256=self.sha,bindings={'source':'test'},rng=np.random.default_rng(3))
    def test_disabled_config_stops_before_any_control_or_integration(self):
        e=self.env()
        with self.assertRaises(ValueError):e.step(np.zeros(21),lambda w:self.fail('engine called'))
        self.assertEqual(e.world.writes,0)
    def test_resume_retains_original_entry_instead_of_reanchoring(self):
        e=self.env();rows=[]
        for i in range(6401,6406):
            r=copy.deepcopy(self.row);r['sourceStep']=i;r['timeS']=8+(i-6400)*.005;r['loadedSlipSpeedMps']=[0,0];rows.append(r)
        e.core.consume(rows);e.world.row=rows[-1];blob,side=e.checkpoint()
        clone=SourceTaskEnv(FakeWorld(rows[-1]),self.model,config_path=CONFIG_PATH,expected_config_sha256=self.sha,bindings={'source':'test'},rng=np.random.default_rng(0),original_entry_packet=side['entryPacket'])
        self.assertTrue(clone.resume_required);clone.restore(blob,side);self.assertFalse(clone.resume_required)
        np.testing.assert_array_equal(clone.core.anchors,e.core.anchors);self.assertEqual(clone.core.start_id,6400)
    def test_full_sidecar_rng_restore_and_failure_rolls_back_native(self):
        e=self.env();blob,side=e.checkpoint();wanted=e.rng.random(7);e.restore(blob,side);np.testing.assert_array_equal(e.rng.random(7),wanted)
        before=e.world.checkpoint();bad=copy.deepcopy(side);bad['state']['quiet_run']=1000
        with self.assertRaises(ValueError):e.restore(blob,bad)
        self.assertEqual(e.world.checkpoint(),before);self.assertEqual(e.core.quiet_run,0)
if __name__=='__main__':unittest.main()
