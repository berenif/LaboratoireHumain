import json
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from reference import PhaseClock,UniformReference,source_resample_columns,source_retime,native_pose_to_source_qpos
ROOT=Path(__file__).resolve().parents[2]

class ReferenceTests(unittest.TestCase):
    def test_all_twelve_source_cubic_parity_fixtures(self):
        rows=json.loads((ROOT/'rust/crates/sim/data/getup-policy/parity.json').read_text())['splines']
        worst=0.
        for row in rows:worst=max(worst,float(np.max(abs(source_resample_columns(row['knots'],row['speed'])-row['expected']))))
        self.assertLess(worst,1e-10);print('Source cubic max error',worst)
    def test_source_qvel_is_independently_scaled(self):
        x=np.arange(12).reshape(4,3);tr={k:x.copy()for k in ['qpos','qvel','com','com_ori']}
        result=source_retime(tr,.5);np.testing.assert_allclose(result['qvel'],result['qpos']*.5)
    def test_terminal_padding_and_five_policy_step_lookahead(self):
        q=np.zeros((8,28));q[:,3]=1;q[:,7]=np.arange(8)
        tr=dict(qpos=q,qvel=np.zeros((8,27)),com=np.tile([0,0,.8],(8,1)),com_ori=np.tile([0,0,1.],(8,1)))
        terminal=np.r_[np.ones(21)*.4,.81,0,0,1.];r=UniformReference(tr,terminal,kind='planned-unqualified')
        now,ahead=r.now_ahead(2);self.assertEqual(now[0],2);self.assertEqual(ahead[0],7)
        np.testing.assert_array_equal(r.now_ahead(8)[0],terminal);np.testing.assert_array_equal(r.now_ahead(208)[1],terminal)
    def test_source_zero_quaternion_target_sentinel_cannot_be_reset_data(self):
        tr=dict(qpos=np.zeros((4,28)),qvel=np.zeros((4,27)),com=np.zeros((4,3)),com_ori=np.zeros((4,3)))
        with self.assertRaises(ValueError):UniformReference(tr,np.zeros(25),kind='planned-unqualified')
    def test_bridge_to_governor_continuity_and_clock_restore(self):
        a=PhaseClock(tau=.5,rate=1.,last_native_substep=400)
        self.assertEqual(a.advance(400,True,3.),(.5,1.,0.));self.assertEqual(a.ahead(3.),.625)
        state=a.state();b=PhaseClock(**state)
        self.assertEqual(a.advance(420,False,3.),b.advance(420,False,3.));self.assertAlmostEqual(a.tau,.525)
        self.assertEqual(a.rate,1.);self.assertEqual(a.target_rate,0.)
        a.advance(440,False,3.);self.assertLess(a.rate,1.);self.assertGreater(a.tau,.525)
    def test_phase_clock_causal_hold_and_ahead_clamp(self):
        c=PhaseClock(0.,0.,0,target_rate=1.);c.advance(20,False,1.)
        self.assertAlmostEqual(c.rate,1-np.exp(-.25));self.assertAlmostEqual(c.tau,.025-.1*(1-np.exp(-.25)))
        c.tau=.999;self.assertEqual(c.ahead(1.),1.)
        with self.assertRaises(ValueError):c.advance(10,True,1.)
    def test_invalid_restored_clock_cannot_exceed_declared_rate(self):
        for kw in [dict(target_rate=2.),dict(rate=np.nan),dict(tau=np.inf),dict(response_seconds=0.),dict(response_seconds=.01),dict(last_native_substep=1.5)]:
            values=dict(tau=.5,rate=1.,last_native_substep=400);values.update(kw)
            with self.assertRaises(ValueError):PhaseClock(**values)
        c=PhaseClock(.5,1.,400)
        for end in [np.nan,np.inf,.4]:
            with self.assertRaises(ValueError):c.advance(420,True,end)
        with self.assertRaises(ValueError):c.advance(420.5,True,3.)
    def test_pose_basis_conjugation_and_wxyz_not_component_guess(self):
        pose=Rotation.from_euler('xyz',[.3,-.4,.7]).as_matrix();q=native_pose_to_source_qpos([1,2,3],pose,np.zeros(21))
        np.testing.assert_array_equal(q[:3],[3,1,2]);P=np.array([[0,0,1],[1,0,0],[0,1,0]])
        np.testing.assert_allclose(Rotation.from_quat(q[[4,5,6,3]]).as_matrix(),P@pose@P.T,atol=1e-15)
if __name__=='__main__':unittest.main()
