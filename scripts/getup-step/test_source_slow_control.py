import unittest,json
from types import SimpleNamespace
import numpy as np
from source_slow_control import SourceQuiet,HOLD_DECISIONS,DT

class FakeWorld:
    h=None
    ids={'left_foot':9,'right_foot':6}
    lib=SimpleNamespace(lh_support_count=lambda _:4,lh_body_velocity=lambda *_:None)
    def arr(self,key,shape):
        if key==10:return np.r_[0.,np.full(16,40.8/16)]
        if key==21:return np.r_[[-.1,-.1,.1,-.1,.1,.1,-.1,.1],np.zeros(120)]
        if key==3:
            p=np.zeros((17,3));p[9,0]=-.05;p[6,0]=.05;return p
        raise ValueError('unexpected fake array')

def row():
    obs=np.zeros(67);obs[21]=1.5;obs[36]=1.;loads=np.zeros(17);loads[9]=loads[6]=200.
    return dict(physicalObservation=obs.tolist(),allBodyFloorLoadsN=loads.tolist(),comSource=[0,0,.8],maximumSelfPenetrationM=0.)

class SourceSlowControlTest(unittest.TestCase):
    def test_real_time_counts(self):
        self.assertEqual(DT,.00125);self.assertEqual(HOLD_DECISIONS,240);self.assertEqual(80*20+240*20,6400);self.assertEqual(4000*DT,5.)
    def test_motion_does_not_count_as_final_hold(self):
        quiet=SourceQuiet();w=FakeWorld()
        for _ in range(4050):result=quiet.observe(w,row(),False)
        self.assertEqual(result['quietConsecutiveSamples'],0);self.assertIsNone(quiet.anchors);json.dumps(result,allow_nan=False)
    def test_declared_settle_window_cannot_freeze_hold_anchor(self):
        quiet=SourceQuiet();w=FakeWorld()
        for _ in range(800):result=quiet.observe(w,row(),True,False)
        self.assertEqual(result['quietConsecutiveSamples'],0);self.assertIsNone(quiet.anchors);json.dumps(result,allow_nan=False)
    def test_final4000_requires_all_predicates(self):
        quiet=SourceQuiet();w=FakeWorld()
        for _ in range(4039):result=quiet.observe(w,row(),True)
        self.assertEqual(result['quietConsecutiveSamples'],4000);self.assertEqual(result['quietSeconds'],5.);json.dumps(result,allow_nan=False)
        bad=row();bad['physicalObservation'][36]=np.cos(np.deg2rad(6));result=quiet.observe(w,bad,True)
        self.assertEqual(result['quietConsecutiveSamples'],0);self.assertFalse(result['conditions']['upright5deg']);json.dumps(result,allow_nan=False)
    def test_drift_anchor_does_not_reset_on_failure(self):
        quiet=SourceQuiet();w=FakeWorld()
        for _ in range(40):quiet.observe(w,row(),True)
        anchor=quiet.anchors.copy();bad=row();bad['physicalObservation'][21]=1.;quiet.observe(w,bad,True)
        np.testing.assert_array_equal(anchor,quiet.anchors)

if __name__=='__main__':unittest.main()
