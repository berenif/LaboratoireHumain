import copy,json,unittest
import numpy as np
from task import load_config,TaskHistory

class TaskTests(unittest.TestCase):
    def setUp(self):
        self.config,self.sha=load_config();self.errors={k:np.zeros(n)for k,n in self.config['reward']['errorDimensions'].items()}
    def run_transition(self,h,events=(),**kw):
        return h.evaluate(range(h.last_native_id+1,h.last_native_id+21),self.errors,np.zeros(21),np.zeros(21),0.,confirmed_events=events,config=self.config,**kw)
    def test_locked_draft_no_training_and_reward_scales(self):
        self.assertEqual(len(self.sha),64);self.assertFalse(self.config['pilot']['learningEnabled'])
        h=TaskHistory(400);r=self.run_transition(h);self.assertAlmostEqual(r['reward'],1.)
        for name in self.errors:self.errors[name]=np.ones_like(self.errors[name])*self.config['reward']['denseTerms'][name]['scale']
        r=self.run_transition(h);self.assertAlmostEqual(r['reward'],np.exp(-.5))
    def test_once_events_restore_and_order(self):
        h=TaskHistory(0);a=self.run_transition(h,['unloaded']);self.assertEqual(a['eventBonus'],1.)
        h=TaskHistory(**json.loads(json.dumps(h.__dict__)));b=self.run_transition(h,['unloaded']);self.assertEqual(b['eventBonus'],0.)
        old=copy.deepcopy(h.__dict__)
        with self.assertRaises(ValueError):self.run_transition(h,['touchdown'])
        self.assertEqual(h.__dict__,old)
        for event in self.config['reward']['eventOrder'][1:]:last=self.run_transition(h,[event])
        self.assertTrue(last['success']);self.assertTrue(last['terminated']);self.assertEqual(last['not_done'],0.)
    def test_physical_failure_vs_timeout_mask_and_no_bonus(self):
        h=TaskHistory(0);r=self.run_transition(h,['unloaded'],physical_failure='flight',truncated=True)
        self.assertEqual(r['not_done'],0);self.assertEqual(r['reward'],-9.);self.assertFalse(r['truncated']);self.assertEqual(h.awarded_events,[])
        r=self.run_transition(h,truncated=True);self.assertEqual(r['not_done'],1);self.assertTrue(r['truncated'])
    def test_integrity_partial_duplicate_missing_and_backward_excluded(self):
        for ids in [range(1,20),[1]*20,range(2,22),range(-19,1),[float(i)for i in range(1,21)],[True]+list(range(2,21))]:
            h=TaskHistory(0)
            with self.assertRaises(ValueError):h.evaluate(ids,self.errors,np.zeros(21),np.zeros(21),0.)
            self.assertEqual(h.last_native_id,0)
    def test_verified_partial_physical_terminal_retained_without_bootstrap(self):
        h=TaskHistory(0)
        r=h.evaluate(range(1,8),self.errors,np.zeros(21),np.zeros(21),0.,physical_failure='flight')
        self.assertAlmostEqual(r['reward'],7/20-10);self.assertEqual(r['not_done'],0.)
        self.assertEqual(r['durationSeconds'],7/800);self.assertEqual(h.last_native_id,7)
    def test_nonfinite_wrong_units_and_failed_input_not_mutated(self):
        for value in [np.nan,np.inf]:
            h=TaskHistory(0);self.errors['comHeightErrorM']=value
            with self.assertRaises(ValueError):self.run_transition(h)
            self.assertEqual(h.last_native_id,0)
        h=TaskHistory(0);self.errors['comHeightErrorM']=0
        with self.assertRaises(ValueError):h.evaluate(range(1,21),self.errors,np.ones(21)*2,np.zeros(21),0.)
    def test_exact_error_shapes_and_stable_failure_codes(self):
        self.errors['jointPositionErrorRad']=0.
        with self.assertRaises(ValueError):self.run_transition(TaskHistory(0))
        self.errors['jointPositionErrorRad']=np.zeros(21)
        for bad in ['', 'made_up', 'nonfinite']:
            with self.assertRaises(ValueError):self.run_transition(TaskHistory(0),physical_failure=bad)
        r=self.run_transition(TaskHistory(0),physical_failure='phase_timeout')
        self.assertEqual(r['not_done'],0.);self.assertEqual(r['reward'],-9.)
    def test_hash_pin_and_invalid_optional_scale_reject_before_mutation(self):
        with self.assertRaises(ValueError):load_config(expected_sha256='0'*64)
        for bad in [0.,np.nan,-.1]:
            config=copy.deepcopy(self.config);config['reward']['denseTerms']['comHeightErrorM']['scale']=bad;h=TaskHistory(0)
            with self.assertRaises(ValueError):h.evaluate(range(1,21),self.errors,np.zeros(21),np.zeros(21),0.,config=config)
            self.assertEqual(h.last_native_id,0)
    def test_computed_reward_overflow_cannot_mutate_history(self):
        h=TaskHistory(0)
        with self.assertRaises(ValueError):h.evaluate(range(1,21),self.errors,np.zeros(21),np.zeros(21),1e308)
        self.assertEqual(h.last_native_id,0)
    def test_frozen_config_reward_no_stationary_or_unload_penalty(self):
        reward=self.config['reward'];self.assertFalse(reward['stationaryWorldAnchorPenalty']);self.assertFalse(reward['genericUnloadedFootPenalty'])
        self.assertEqual(self.config['phaseGates']['finalQuietConsecutiveNativeSamples'],4000)
if __name__=='__main__':unittest.main()
