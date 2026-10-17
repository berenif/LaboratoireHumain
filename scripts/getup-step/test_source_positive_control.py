import unittest
import numpy as np
from source_positive_control import MAXIMUM_STEPS,SETTLE_DECISIONS,TEACHER_DECISIONS,rng_json,sample_pose,validate_source_frame

class SourcePositiveControlTest(unittest.TestCase):
    def joints(self):
        return [dict(type=0,qposAddress=0,limited=False,range=[0,0])]+[dict(type=3,qposAddress=i+7,limited=True,range=[-.5,.7])for i in range(21)]
    def test_original_counts_include_settling(self):
        self.assertEqual(SETTLE_DECISIONS,80);self.assertEqual(TEACHER_DECISIONS,250);self.assertEqual(MAXIMUM_STEPS,1650);self.assertEqual(MAXIMUM_STEPS*.005,8.25)
    def test_positive_uniform_quaternion_and_model_order(self):
        q0=np.r_[[.2,-.4,1.5],[1.,0,0,0],np.zeros(21)];rng=np.random.RandomState(10)
        q=sample_pose(q0,self.joints(),rng);reference=np.random.RandomState(10);quat=reference.rand(4);quat/=np.linalg.norm(quat)
        np.testing.assert_array_equal(q[:3],q0[:3]);np.testing.assert_array_equal(q[3:7],quat);self.assertTrue(np.all(q[3:7]>0));self.assertAlmostEqual(np.linalg.norm(q[3:7]),1)
        np.testing.assert_array_equal(q[7:],reference.uniform(-.5,.7,21))
    def test_rejected_samples_advance_same_rng(self):
        q0=np.r_[[0,0,1.5],[1.,0,0,0],np.zeros(21)];rng=np.random.RandomState(10)
        first=sample_pose(q0,self.joints(),rng);saved=rng_json(rng);second=sample_pose(q0,self.joints(),rng);restored=np.random.RandomState();restored.set_state((saved['name'],np.array(saved['keys'],dtype=np.uint32),saved['position'],saved['hasGaussian'],saved['cachedGaussian']))
        np.testing.assert_array_equal(second,sample_pose(q0,self.joints(),restored));self.assertFalse(np.array_equal(first,second))
    def test_json_frame_velocity_list_accepts_and_rejects(self):
        validate_source_frame(dict(qvel=[0.]*27,sourceWarnings=0),0.)
        for row,torque in [(dict(qvel=[10001.]*27,sourceWarnings=0),0.),(dict(qvel=[float('nan')]*27,sourceWarnings=0),0.),(dict(qvel=[0.]*27,sourceWarnings=1),0.),(dict(qvel=[0.]*27,sourceWarnings=0),float('nan'))]:
            with self.assertRaises(ValueError):validate_source_frame(row,torque)
    def test_bad_qpos_rejects(self):
        for q in [np.zeros(27),np.full(28,np.nan)]:
            with self.assertRaises(ValueError):sample_pose(q,self.joints(),np.random.RandomState(10))

if __name__=='__main__':unittest.main()
