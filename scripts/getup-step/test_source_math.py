import json
from pathlib import Path
import unittest
import numpy as np
import source_math as m
from actors import DenseActor
ROOT = Path(__file__).resolve().parents[2]

class SourceMathTests(unittest.TestCase):
    def test_source_observation_and_power_coordinates(self):
        physical=np.arange(67);now=np.arange(25)+100;ahead=now+30
        x=m.student_observation(physical,now,ahead)
        np.testing.assert_array_equal(x[:67],physical);np.testing.assert_array_equal(x[67:92],now)
        np.testing.assert_array_equal(x[92:117],ahead);self.assertEqual(x[117],1)
        t=m.conditioned_teacher_observation(physical,now,ahead,.66342)
        np.testing.assert_array_equal(t[:-1],x[:-1]);self.assertAlmostEqual(t[-1],.66342,places=7)
        self.assertEqual(m.AHEAD_DECISIONS*m.POLICY_DT,.125)
    def test_mapping_and_both_damping_branches(self):
        q=np.linspace(-.2,.2,21);dq=np.linspace(-.7,.8,21);ref=q+.05;res=np.linspace(-.1,.1,21)
        for hold,kd in [(False,.1),(True,.2)]:
            np.testing.assert_allclose(m.slow_controls(ref,res,q,dq,hold),(ref+res-q-kd*dq)[m.ORDER],atol=1e-15)
        self.assertEqual(m.ORDER.tolist(),[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20])
    def test_clipping_happens_before_gear_and_no_second_power(self):
        u=m.slow_controls(np.ones(21)*20,np.zeros(21),np.zeros(21),np.zeros(21))
        np.testing.assert_array_equal(u,np.ones(21));gear=np.arange(21)+20
        np.testing.assert_array_equal(u*gear,gear)
        a=np.linspace(-.4,.4,21);np.testing.assert_array_equal(m.teacher_controls(a,.4),a)
        with self.assertRaises(ValueError):m.teacher_controls(a,.3)
    def test_archived_pd_parity(self):
        rows=json.loads((ROOT/'rust/crates/sim/data/getup-policy/parity.json').read_text())['pd']
        for row in rows:
            np.testing.assert_allclose(m.slow_controls(row['reference'],row['residual'],row['q'],row['dq'],row['holding']),row['controls'],atol=2e-7)
    def test_tolerance_margin_values(self):
        for name,value in [('gaussian',.1),('long_tail',.2),('linear',0.),('quadratic',0.)]:
            self.assertAlmostEqual(float(m.tolerance(1.,margin=1.,sigmoid=name,value_at_margin=value)),value)
            self.assertEqual(float(m.tolerance(0.,margin=1.,sigmoid=name,value_at_margin=value)),1.)
    def test_source_teacher_reward_and_unused_effort(self):
        args=dict(com_height=.80326,torso_up=1.,head_height=1.6,horizontal_velocity=[0.,0.],foot_separation=.3)
        self.assertEqual(m.teacher_reward(**args),1.)
        self.assertLess(m.teacher_reward(**dict(args,foot_separation=1.1)),1.)
        self.assertEqual(m.teacher_reward(**dict(args,com_height=.4,torso_up=-1)),1.)
    def test_moving_reward_uses_only_two_hip_velocities(self):
        qv=np.zeros(27);ref=m.target(np.zeros(21),.8,[0,0,1]);kw=dict(qvel27=qv,com_height=.8,torso_vertical=[0,0,1],reference_qvel27=np.zeros(27),reference_target=ref)
        self.assertEqual(m.slow_moving_reward(**kw),1.)
        qv[12]=30;self.assertEqual(m.slow_moving_reward(**kw),1.)
        qv[11]=3;self.assertLess(m.slow_moving_reward(**kw),1.)
    def test_hold_height_is_distinct_from_terminal_observation(self):
        args=dict(q=np.zeros(21),com_height=.80326,torso_up=1.,horizontal_velocity=[0,0],standing_q=np.zeros(21))
        self.assertEqual(m.slow_hold_reward(**args),1.)
        self.assertLess(m.slow_hold_reward(**dict(args,com_height=1.4)),.1)
        np.testing.assert_array_equal(m.target(np.zeros(21),1.4,[0,0,.9])[-3:],[0,0,.9])
    def test_last_moving_action_poststep_holding_reward(self):
        self.assertEqual(m.source_boundary(9,10),dict(commandHolding=False,rewardHolding=True,nowIndex=9,aheadIndex=10))
        self.assertTrue(m.source_boundary(10,10)['commandHolding'])
        self.assertEqual(m.source_terminal(160,10,.8,.8),dict(terminated=False,truncated=True))
        self.assertTrue(m.source_terminal(11,10,.4,.8)['terminated'])
    def test_nonfinite_and_float32_overflow_rejected(self):
        for huge in [np.nan, np.inf, 1e300]:
            with self.assertRaises(ValueError):m.student_observation([huge]*67,np.zeros(25),np.zeros(25))
        with self.assertRaises(ValueError):m.source_terminal(0,10,np.nan,.8)
        with self.assertRaises(ValueError):m.slow_hold_reward(np.zeros(21),np.nan,1.,[0,0],np.zeros(21))
        with self.assertRaises(ValueError):m.teacher_reward(com_height=.8,torso_up=1.,head_height=1.6,horizontal_velocity=[0,0],foot_separation=-.01)
    def test_invalid_input_has_no_side_effect_path(self):
        for bad in [[0]*20,[np.nan]*21]:
            with self.assertRaises(ValueError):m.slow_controls(bad,np.zeros(21),np.zeros(21),np.zeros(21))

class ActorParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.actors={name:DenseActor.load(ROOT/f'rust/crates/sim/data/getup-policy/{name}.bin')for name in ['teacher','student']}
    def test_archived_all_raw_and_action_outputs(self):
        rows=json.loads((ROOT/'rust/crates/sim/data/getup-policy/parity.json').read_text())['actors']
        raw_error=action_error=0.
        for row in rows:
            x=self.actors[row['actor']].infer(row['input'])
            raw_error=max(raw_error,float(max(abs(x['raw']-row['raw']))));action_error=max(action_error,float(max(abs(x['action']-row['action']))))
        self.assertLessEqual(raw_error,2e-4);self.assertLessEqual(action_error,5e-5)
        print(f'Actor parity: raw max {raw_error:.9g}, action max {action_error:.9g}; no physics')
    def test_conditioned_teacher_full42_zero_column_parity(self):
        old=self.actors['teacher'];new=old.conditioned_teacher();rng=np.random.default_rng(20261017)
        for power in [0.,.4,.663420431,1.]:
            physical=rng.normal(0,.2,67);now=rng.normal(0,2,25);ahead=rng.normal(0,2,25)
            a=old.infer(m.teacher_observation(physical,power));b=new.infer(m.conditioned_teacher_observation(physical,now,ahead,power))
            np.testing.assert_array_equal(a['raw'],b['raw']);np.testing.assert_array_equal(a['action'],b['action'])
        np.testing.assert_array_equal(new.layers[0][0][:,67:117],0.)
    def test_layer_overflow_rejected_like_native(self):
        x=np.ones(118,dtype=np.float32)*np.finfo(np.float32).max;x[-1]=1.
        with self.assertRaises(ValueError):self.actors['student'].infer(x)
    def test_bad_binary_rejected_without_pickle(self):
        with self.assertRaises(ValueError):DenseActor.from_bytes(b'bad')

if __name__=='__main__':unittest.main()
