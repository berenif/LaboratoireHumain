"""Pure mock/LP safety contracts. No native world or physics is constructed."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from coupled_posture_kinematics import MatrixJet
from coupled_posture_reference import Jet
import wholebody_static_admission as admission


class LPWorld:
    mass=400./9.81
    gears=np.ones(21)
    def __init__(self, horizontal=0., joint_bias=0.):
        self.bias=np.zeros(27);self.bias[0]=horizontal;self.bias[2]=400.;self.bias[6]=joint_bias
    def feet(self):return [dict(body=2),dict(body=3)]
    def read(self,name,shape,*args):
        if name=='lh_bias':return self.bias.copy()
        if name=='lh_point_jac':return np.c_[np.eye(3),np.zeros((3,24))]
        raise AssertionError(name)


class LPTests(unittest.TestCase):
    def test_static_force_balance_and_original_motor_limits(self):
        r=admission.strict_contact_lp(LPWorld(),[(0,[0.,0.,0.])])
        self.assertTrue(r['success']);self.assertEqual(r['requiredMotorFraction'],0.)
        self.assertLess(r['maximumEqualityResidual'],1e-12)
        self.assertEqual(r['originalMotorBounds'],[-.95,.95])
        self.assertEqual(r['conservativeStaticFriction'],.6)
    def test_unachievable_original_motor_bias_rejects(self):
        self.assertFalse(admission.strict_contact_lp(LPWorld(joint_bias=2.),[(1,[0.,0.,0.])])['success'])
    def test_conservative_friction_not_relaxed(self):
        self.assertFalse(admission.strict_contact_lp(LPWorld(horizontal=250.),[(1,[0.,0.,0.])])['success'])
    def test_multiple_feet_keep_explicit_body_ids(self):
        r=admission.strict_contact_lp(LPWorld(),[(0,[0.,0.,0.]),(1,[.1,0.,0.])])
        self.assertTrue(r['success']);self.assertEqual([x['footSide']for x in r['explicitContacts']],[0,1])
    def test_empty_or_wrong_side_points_reject(self):
        for points in ([],[(2,[0.,0.,0.])]):
            with self.assertRaises(ValueError):admission.strict_contact_lp(LPWorld(),points)


class Scratch:
    base=Path('/unused');xml=Path('/unused/model');helper=Path('/unused/helper')
    limits=np.tile([-1.,1.],(21,1))
    h=object()
    def __init__(self,counter=0):
        self.counter=counter;self.installs=0;self.closed=False
        def placeholder(*args):raise AssertionError('unexpected native call')
        self.l=SimpleNamespace(lh_dim=lambda h,k:self.counter,lh_step_audited=placeholder,
            lh_self_separation=placeholder)
    def install_cold(self,value):
        assert self.counter==0;self.installs+=1
    def close(self):self.closed=True
    def read(self,name,shape):
        assert name=='lh_self_separation';raw=np.zeros(897);raw[0]=1;raw[5]=.03;return raw
    def packet(self):return dict(com=[0.,0.,1.])


class ScratchTests(unittest.TestCase):
    def setUp(self):
        self.actual=Scratch(counter=3200);self.worlds=[]
        result=SimpleNamespace(full_joints=Jet.rest(np.zeros(21)),root_model=MatrixJet.rest(np.eye(4)),com_source=Jet.rest([0.,0.,1.]))
        self.samples=[('fixed-pose',SimpleNamespace(geometric=result))]
        self.patches=[patch.object(admission,'sha',return_value='same-source'),
            patch.object(admission,'native_bytes',side_effect=lambda w:bytes(str((w.counter,w.installs)), 'utf8')),
            patch.object(admission,'strict_contact_lp',return_value=dict(success=True,requiredMotorFraction=.5,
                maximumEqualityResidual=0.,maximumFrictionViolation=0.))]
        for p in self.patches:p.start()
    def tearDown(self):
        for p in reversed(self.patches):p.stop()
    def factory(self,actual):
        w=Scratch();self.worlds.append(w);return w
    def test_two_counterzero_rechecks_without_actual_mutation(self):
        r=admission.finite_static_admission(self.actual,self.samples,[(0,[0.,0.,0.])],factory=self.factory)
        self.assertTrue(r['accepted']);self.assertEqual(r['worldsConstructed'],2)
        self.assertEqual([w.installs for w in self.worlds],[1,1]);self.assertTrue(all(w.closed for w in self.worlds))
        self.assertEqual(self.actual.installs,0);self.assertEqual(self.actual.counter,3200)
        self.assertFalse(self.actual.closed);self.assertEqual(r['physicsIntegrations'],0)
    def test_actual_cannot_be_scratch_or_closed(self):
        with self.assertRaisesRegex(ValueError,'counter-zero'):
            admission.finite_static_admission(self.actual,self.samples,[(0,[0.,0.,0.])],factory=lambda w:w)
        self.assertFalse(self.actual.closed);self.assertEqual(self.actual.installs,0)
    def test_bad_counter_scratch_is_closed(self):
        bad=Scratch(counter=1)
        with self.assertRaisesRegex(ValueError,'counter-zero'):
            admission.finite_static_admission(self.actual,self.samples,[(0,[0.,0.,0.])],factory=lambda w:bad)
        self.assertTrue(bad.closed);self.assertEqual(bad.installs,0)
    def test_rejected_static_margin_does_not_trigger_fallback(self):
        with patch.object(admission,'strict_contact_lp',return_value=dict(success=False,requiredMotorFraction=None)):
            r=admission.finite_static_admission(self.actual,self.samples,[(0,[0.,0.,0.])],factory=self.factory)
        self.assertFalse(r['accepted']);self.assertEqual(len(self.worlds),1)
        self.assertEqual(self.actual.installs,0);self.assertTrue(self.worlds[0].closed)


if __name__=='__main__':unittest.main()
