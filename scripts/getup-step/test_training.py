import unittest,numpy as np
from task import load_config
from training import admit_task_learning,learning_update_due,warmup_controls
class TrainingGuardTests(unittest.TestCase):
    def test_no_collection_or_optimizer_from_reviewed_disabled_config(self):
        from task import CONFIG_PATH
        c,sha=load_config()
        with self.assertRaises(ValueError):admit_task_learning(CONFIG_PATH,expected_config_sha256=sha,admission={})
    def test_exact_pilot_update_clock(self):
        c,_=load_config();due=[n for n in range(2001)if learning_update_due(n,0,c)]
        self.assertEqual(due[:3],[260,264,268]);self.assertFalse(learning_update_due(1056,200,c))
        self.assertEqual(256+4*200,1056);self.assertFalse(learning_update_due(2001,0,c))
    def test_noise_applies_once_in_normalized_units_and_repeats_with_rng(self):
        c,_=load_config();a=warmup_controls(np.ones(21)*.49,.5,np.random.default_rng(7),c);b=warmup_controls(np.ones(21)*.49,.5,np.random.default_rng(7),c)
        np.testing.assert_array_equal(a,b);self.assertLessEqual(max(abs(a)),.5);self.assertGreater(min(a),.4)
if __name__=='__main__':unittest.main()
