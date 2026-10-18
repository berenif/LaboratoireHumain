import copy,json,unittest
from pathlib import Path
import numpy as np
from slow_task import SourceSLOWCore,SourceSLOWEnv,CONFIG_PATH,CONFIG_SHA,initialize_slow_agent
from source_math import slow_controls,ORDER
from task import load_config,TaskHistory
ROOT=Path(__file__).resolve().parents[2]
class FakeSLOWWorld:
    def __init__(self,row,model):
        self.row=copy.deepcopy(row);self.controls=np.zeros(21);self.gear=np.zeros(21);self.writes=0;self.delta_velocity=0.
        for p in model['segments']:
            h=p['reference'].get('hinge')
            if h:self.gear[h['qposIndex']]=h['gear']
    def task_packet(self):return copy.deepcopy(self.row)
    def observation(self):return np.array(self.row['physicalObservation'])
    def checkpoint(self):return json.dumps(self.row,sort_keys=True).encode()
    def restore(self,b):self.row=json.loads(b);self.controls=np.array(self.row['qfrcActuator'])[6:][ORDER]/self.gear[ORDER]
    def dim(self,i):assert i==6;return self.row['sourceStep']
    def arr(self,i,shape):
        if i==0:return np.array(self.row['qpos'])
        if i==1:return np.array(self.row['qvel'])
        assert i==2;return self.controls
    def advance(self):
        self.row['sourceStep']+=1;self.row['timeS']+=.00125;self.row['loadedSlipSpeedMps']=[0.,0.];self.row['qfrcActuator']=np.r_[np.zeros(6),self.gear*self.controls[np.argsort(ORDER)]].tolist();self.row['qvel'][6]+=self.delta_velocity;self.row['physicalObservation'][46]=self.row['qvel'][6]
class SlowTaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.row=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame'];cls.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    def make(self):return SourceSLOWEnv(FakeSLOWWorld(self.row,self.model),self.model,bindings={'model':'fake'},rng=np.random.default_rng(7))
    def test_original_actor_target_and800hz_partial_scaling(self):
        c=SourceSLOWCore(self.row,self.model);o=c.observation(self.row['physicalObservation']);self.assertEqual(o[88],np.float32(1.4));np.testing.assert_array_equal(o[67:88],np.array(self.model['referenceMetadata']['standingPose'],dtype=np.float32));self.assertEqual(c.clock.sample_seconds,.00125)
        cfg,sha=load_config(CONFIG_PATH);self.assertEqual(sha,CONFIG_SHA);h=TaskHistory(0);errors={k:np.zeros(n)for k,n in cfg['reward']['errorDimensions'].items()};r=h.evaluate([1,2,3],errors,np.zeros(21),np.zeros(21),0,physical_failure='flight',config=cfg);self.assertAlmostEqual(r['reward'],-9.85);self.assertEqual(r['not_done'],0);self.assertEqual(r['durationSeconds'],.00375)
    def test_residualheld20_pd_eachsample_and_nextcommand_restore(self):
        e=self.make();a=np.linspace(-.02,.02,21);r=e.measured_interval(a,lambda w:w.advance());self.assertEqual(len(r['actualFrames']),20);self.assertEqual(r['action'],a.tolist());self.assertEqual(r['info']['durationSeconds'],.025);self.assertEqual(r['info']['maximumPerNativeTorqueErrorNm'],0)
        want=slow_controls(e.core.source_standing,a,self.row['qpos'][7:],self.row['qvel'][6:],True);np.testing.assert_array_equal(e.world.controls,want)
        blob,side=e.checkpoint();wanted_rng=e.rng.random(3);r1=e.measured_interval(-a,lambda w:w.advance());e.restore(blob,side);np.testing.assert_array_equal(e.rng.random(3),wanted_rng);r2=e.measured_interval(-a,lambda w:w.advance());self.assertEqual(r1,r2)
    def test_pd_refreshes_actual_velocity_inside_held_residual(self):
        e=self.make();e.world.delta_velocity=.01;r=e.measured_interval(np.zeros(21),lambda w:w.advance())
        first=np.array(r['perNativeControls'][0]['expectedControls']);last=np.array(r['perNativeControls'][-1]['expectedControls'])
        self.assertAlmostEqual(last[list(ORDER).index(0)]-first[list(ORDER).index(0)],-.2*.01*19)
        self.assertEqual(r['action'],[0.]*21);self.assertEqual(r['info']['maximumPerNativeTorqueErrorNm'],0.)
    def test_badresidual_before_control_and_wrongclock_rejects(self):
        e=self.make();before=e.world.checkpoint()
        for a in [np.ones(21)*1.1,np.ones(20),np.ones(21)*np.nan]:
            with self.assertRaises(ValueError):e.measured_interval(a,lambda _:self.fail('called'))
            self.assertEqual(e.world.checkpoint(),before)
        def wrong(w):w.advance();w.row['timeS']+=.00125
        with self.assertRaises(ValueError):e.measured_interval(np.zeros(21),wrong)
        self.assertEqual(e.core.native_id,6400)
    def test_original_mean_batch1_unchanged_std_reinitialized(self):
        try:import torch
        except ImportError:self.skipTest('optional existing Torch')
        agent,original=initialize_slow_agent();e=self.make();x=torch.tensor(e.core.observation(self.row['physicalObservation']))[None,:]
        with torch.no_grad():a=agent.actor(x,False,False);b=original(x,False,False)
        self.assertTrue(torch.equal(a[0],b[0]));self.assertTrue(torch.equal(agent.actor.trunk[4].weight[:21],original.trunk[4].weight[:21]));self.assertTrue(torch.equal(agent.actor.trunk[2].weight,original.trunk[2].weight));self.assertLess(float(max(abs(a[3].exp()[0]-.02))),1e-8)
        self.assertEqual(agent.actor_optimizer.state_dict()['state'],{});self.assertTrue(all(torch.equal(p,t)for p,t in zip(agent.critic.parameters(),agent.critic_target.parameters())))
if __name__=='__main__':unittest.main()
