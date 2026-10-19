"""Exact cumulative per-foot walking-slip boundary and historical replay tests."""
import io
import unittest
from types import SimpleNamespace

import numpy as np

from source_walk_limits import GLOBAL_LOADED_SLIP_LIMIT_M,loaded_slip_exceeded,validate_saved_slip_limit
from source_multistep_probe import MultiStep
from source_cadenced_ten_steps import CadencedSteps
from audit_source_cadenced_steps import audit_rows,DT
from test_source_cadenced_audit import fixture


class WalkingSlipTests(unittest.TestCase):
    def test_exact_twenty_millimetres_is_allowed(self):
        self.assertEqual(GLOBAL_LOADED_SLIP_LIMIT_M,.02)
        self.assertFalse(loaded_slip_exceeded([.02,.02]))
        self.assertTrue(loaded_slip_exceeded([np.nextafter(.02,np.inf),0.]))
        self.assertFalse(loaded_slip_exceeded([.015,.015]))
        self.assertEqual(CadencedSteps.loaded_slip_limit_m,.02)

    def test_historical_limits_are_explicit_not_silently_rewritten(self):
        self.assertEqual(validate_saved_slip_limit(.01),.01)
        self.assertEqual(validate_saved_slip_limit(.02),.02)
        for value in (.1,float('nan'),True):
            with self.assertRaises(ValueError):validate_saved_slip_limit(value)

    def live(self,slip):
        live=MultiStep.__new__(MultiStep)
        live.steps=live.total=0;live.maximum=100;live.paused=False;live.finished=False;live.failure=None
        live.step_index=0;live.distances=(.18,);live.phase='transfer';live.phase_start=0.
        live.slip=np.array(slip);live.loaded=np.zeros(2,dtype=int);live.notes=[]
        live.max_motor=0.;live.max_clear=-1.;live.journal=io.StringIO()
        live.controller=SimpleNamespace(swing_side=0,solve=lambda *a:(np.zeros(21),{}),phase_duration=lambda p:999.)
        class FakeWorld:
            mass=10.;h=None
            def __init__(self):self.counter=0;self.l=SimpleNamespace(lh_step_audited=self.step)
            def arr(self,*args):return np.zeros(21)
            def step(self,*args):self.counter+=1;return self.counter
            def packet(self):
                return dict(feet=[dict(loadN=50.,loadedSlipSpeedMps=0.,clearance=0.),dict(loadN=50.,loadedSlipSpeedMps=0.,clearance=0.)],
                    warnings=0,nonfootLoadN=0.,maximumSelfPenetrationM=0.,com=[0.,0.,.8],swingForwardDisplacementM=0.)
        live.world=FakeWorld();live.decorate=lambda row:row
        return live

    def test_real_advance_keeps_stop_active_at_new_boundary(self):
        live=self.live([.02,.02]);live.advance(1)
        self.assertIsNone(live.failure);self.assertEqual(live.steps,20)
        live=self.live([np.nextafter(.02,np.inf),0.]);live.advance(1)
        self.assertEqual(live.failure,'global-loaded-slip-20mm');self.assertTrue(live.paused)
        self.assertEqual(live.steps,1)

    def test_validator_accepts_twenty_and_retains_historical_ten(self):
        rows,entry,records,references,cadence=fixture()
        step=.016/len(rows)
        for i,row in enumerate(rows,1):
            row['loadedSlipDistanceM']=[0.,step*i]
            row['feet'][1]['loadedSlipSpeedMps']=step/DT
        cadence['globalLoadedSlipLimitM']=.02
        current=audit_rows(iter(rows),entry,records,references,cadence,10.,np.ones(21))
        self.assertTrue(current['passed'],[k for k,v in current['checks'].items()if not v])
        self.assertTrue(current['checks']['globalLoadedSlipWithinConfiguredLimit'])
        self.assertTrue(current['checks']['originalQuietAndSlipThresholds'])
        cadence['globalLoadedSlipLimitM']=.01
        historical=audit_rows(iter(rows),entry,records,references,cadence,10.,np.ones(21))
        self.assertFalse(historical['checks']['globalLoadedSlipWithinConfiguredLimit'])


if __name__=='__main__':unittest.main()
