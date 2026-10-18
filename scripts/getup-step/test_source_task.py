import copy,json,unittest
from pathlib import Path
import numpy as np
from source_task import SourceStabilization,decode_source_frame,CONFIG_PATH
from task import load_config,TaskHistory
from reference import PhaseClock
ROOT=Path(__file__).resolve().parents[2]

class SourceTaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.row=json.loads((ROOT/'docs/fixtures/getup-step/source-SLOW-final-frame.json').read_text())['frame'];cls.model=json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
    def row_at(self,i):
        # Synthetic frozen geometry test packet, not an integrated teacher rollout.
        x=copy.deepcopy(self.row);x['sourceStep']=i;x['timeS']=8.+(i-6400)*.005;x['loadedSlipSpeedMps']=[0.,0.];return x
    def test_actual_source_decoder_and_action_units(self):
        x=decode_source_frame(self.row,self.model);self.assertEqual(x['combinedDimension'],2);self.assertGreater(x['combinedCaptureMargin'],.05)
        self.assertLess(np.max(abs(x['controls'])),1.);self.assertAlmostEqual(x['mass'],40.844021221621325)
        s=SourceStabilization(self.row,self.model);o=s.observation(x['physical'],.663420431);self.assertEqual(o.shape,(118,));self.assertAlmostEqual(o[-1],.663420431,places=7)
        np.testing.assert_array_equal(o[67:92],o[92:117])
    def test_200hz_clock_5sample_and_partial_failure_masks(self):
        cfg,_=load_config(CONFIG_PATH);errors={k:np.zeros(n)for k,n in cfg['reward']['errorDimensions'].items()}
        h=TaskHistory(0);r=h.evaluate(range(1,6),errors,np.zeros(21),np.zeros(21),0.,config=cfg);self.assertEqual(r['durationSeconds'],.025)
        r=h.evaluate([6,7],errors,np.zeros(21),np.zeros(21),0.,physical_failure='flight',config=cfg);self.assertAlmostEqual(r['reward'],-9.6);self.assertEqual(r['not_done'],0.)
        c=PhaseClock(0.,1.,0,sample_seconds=.005);c.advance(5,True,8.);self.assertEqual(c.tau,.025);self.assertEqual(PhaseClock(**c.state()).advance(10,True,8.),c.advance(10,True,8.))
    def test_false_native_packets_reject_without_mutation(self):
        env=SourceStabilization(self.row,self.model);old=copy.deepcopy(env.__dict__)
        for kind in ['floatid','nanforce','mismatchq','badpoints']:
            r=self.row_at(6401)
            if kind=='floatid':r['sourceStep']=6401.
            if kind=='nanforce':r['loadsN'][0]=float('nan')
            if kind=='mismatchq':r['qvel'][6]+=.1
            if kind=='badpoints':r['sourceQuiet']['solvedPositiveForcePointsCurrentXY']=[[float('nan'),0]]
            with self.assertRaises(ValueError):env.consume([r])
            self.assertEqual(env.native_id,old['native_id']);self.assertEqual(env.history.last_native_id,6400)
    def test_partial_physical_terminal_and_no_incomplete_nonterminal(self):
        env=SourceStabilization(self.row,self.model);r=self.row_at(6401)
        with self.assertRaises(ValueError):env.consume([r])
        self.assertEqual(env.native_id,6400)
        r['loadedSlipSpeedMps']=[3.,0.];x=env.consume([r]);self.assertEqual(x['physicalFailure'],'loaded_slip');self.assertEqual(x['not_done'],0.);self.assertEqual(x['durationSeconds'],.005)
        with self.assertRaises(ValueError):env.consume([self.row_at(6402)])
    def test_true_time_event_and_prospective_checkpoint_resume(self):
        a=SourceStabilization(self.row,self.model);rows=[self.row_at(i)for i in range(6401,6406)];first=a.consume(rows);self.assertFalse(first['success']);self.assertEqual(first['referenceClock']['tau'],.025)
        native=b'test-native-cache';rng=np.random.default_rng(7).bit_generator.state;bindings={'model':'pinned-test','code':'pinned-test'};side=a.checkpoint(native,bindings,rng)
        b=SourceStabilization(self.row,self.model);self.assertEqual(b.restore(side,native,rows[-1],expected_bindings=bindings),rng)
        nextrows=[self.row_at(i)for i in range(6406,6411)];self.assertEqual(a.consume(nextrows),b.consume(nextrows))
        bad=copy.deepcopy(side);bad['state']['quiet_run']=1000
        with self.assertRaises(ValueError):b.restore(bad,native,rows[-1],expected_bindings=bindings)
        with self.assertRaises(ValueError):b.restore(side,b'wrong',rows[-1],expected_bindings=bindings)
    def test_final_five_seconds_not_longest_or_clock_goal(self):
        env=SourceStabilization(self.row,self.model)
        for k in range(204):r=env.consume([self.row_at(i)for i in range(6401+5*k,6406+5*k)])
        self.assertTrue(r['success']);self.assertEqual(r['not_done'],0.);self.assertGreaterEqual(r['quietConsecutiveSamples'],1000);self.assertEqual(r['eventBonus'],5.)
        self.assertFalse(r['taskDataCollectionEnabled']);self.assertFalse(r['optimizerEnabled'])
    def test_source_actor_updates_require_critic_warmup(self):
        from training import learning_update_due
        cfg,_=load_config(CONFIG_PATH)
        self.assertFalse(learning_update_due(260,0,cfg,critic_only_updates=99))
        self.assertTrue(learning_update_due(260,0,cfg,critic_only_updates=100))
        self.assertFalse(learning_update_due(260,25,cfg,critic_only_updates=100))
    def test_wrong_source_timestep_and_residual_as_torque_reject(self):
        env=SourceStabilization(self.row,self.model);rows=[self.row_at(i)for i in range(6401,6406)]
        rows[-1]['timeS']+=.00125
        with self.assertRaises(ValueError):env.consume(rows)
        rows[-1]=self.row_at(6405)
        with self.assertRaises(ValueError):env.consume(rows,expected_teacher_controls=np.ones(21)*.9)
        self.assertEqual(env.native_id,6400)
    def test_semantically_invalid_rehashed_resume_rejects(self):
        base=SourceStabilization(self.row,self.model);rng=np.random.default_rng(7).bit_generator.state;bindings={'model':'pinned'}
        for bad in ['clock','quiet','feet','anchor','target','slip']:
            a=copy.deepcopy(base)
            if bad=='clock':a.clock.sample_seconds=.00125
            if bad=='quiet':a.quiet_run=1000
            if bad=='feet':a.feet_run=[True,0]
            if bad=='anchor':a.anchors[0,0]+=.01
            if bad=='target':a.target['q'][0]+=.1
            if bad=='slip':a.slip[0]=-1
            side=a.checkpoint(b'cache',bindings,rng)
            with self.assertRaises(ValueError):base.restore(side,b'cache',self.row,expected_bindings=bindings)
            self.assertEqual(base.native_id,6400)
        with self.assertRaises(ValueError):base.checkpoint(b'cache',bindings,{'state':7})
    def test_success_at_exact_phase_deadline_precedes_timeout(self):
        env=SourceStabilization(self.row,self.model)
        for k in range(320):
            rows=[self.row_at(i)for i in range(6401+5*k,6406+5*k)]
            if k<120:
                for r in rows:r['sourceQuiet']['maximumBodyLinearMps']=1.
            result=env.consume(rows)
        self.assertTrue(result['success']);self.assertIsNone(result['physicalFailure']);self.assertEqual(result['quietConsecutiveSamples'],1000)
    def test_extrinsic_cutoff_preserves_actual_terminal_observation(self):
        env=SourceStabilization(self.row,self.model);r=env.consume([self.row_at(i)for i in range(6401,6406)],extrinsic_cutoff=True)
        self.assertTrue(r['truncated']);self.assertFalse(r['terminated']);self.assertEqual(r['not_done'],1.);self.assertEqual(r['actualFinalObservation118'][:21],np.array(self.row['physicalObservation'][:21],dtype=np.float32).astype(float).tolist())
if __name__=='__main__':unittest.main()
