#!/usr/bin/env python3
"""Pure guarded-model schedule regressions; no native process or physics."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
S=importlib.util.spec_from_file_location('equilibrium',Path(__file__).with_name('probe-getup-equilibrium.py'))
m=importlib.util.module_from_spec(S);S.loader.exec_module(m)

class GuardedScheduleTests(unittest.TestCase):
    def setUp(self):
        self.rows=[dict(com=[0,0,0],normalForcesN=[200,100,100]),dict(com=[.1,0,0],normalForcesN=[0,200,200])]
        self.guard=m.GuardedTransfer(np.array([0.,1.]),np.array([[0]*21,[.1]*21]),self.rows)
        self.diag={'com':dict(x=.1,y=0,z=0),'capture':dict(x=.1,y=0,z=0)}
        self.metrics={'foot_loads_n':[200,200]}
    def test_unsafe_clock_pauses_then_returns(self):
        for i in range(41):self.guard.schedule(1+i*.025,self.diag,self.metrics,[])
        self.assertEqual(self.guard.clock,0.)
        self.assertIsNotNone(self.guard.return_start)
        w,b=self.guard.schedule(6.1,self.diag,self.metrics,[])
        self.assertEqual((w,b),(0.,0.))
    def test_native_flight_during_return_releases_once(self):
        self.guard.return_start=2.;self.guard.return_progress=.3
        w,b=self.guard.schedule(2.5,self.diag,self.metrics,[])
        self.guard.inspect_native(2.5,w,b,[{'loads_n':[0,0]},{'loads_n':[200,200]}])
        old=self.guard.abort
        self.guard.inspect_native(2.525,w,b,[{'loads_n':[0,0]}])
        self.assertEqual(self.guard.abort,old)
        self.assertEqual(self.guard.schedule(2.55,self.diag,self.metrics,[])[1],0.)
    def test_initial_blend_release_does_not_jump_to_one(self):
        self.guard.inspect_native(.5,0.,.1,[{'loads_n':[0,0]}])
        self.assertEqual(self.guard.schedule(.5,self.diag,self.metrics,[])[1],.1)
        self.assertAlmostEqual(self.guard.schedule(.525,self.diag,self.metrics,[])[1],.05)
    def test_raw_abort_stays_exact_source(self):
        self.guard.abort=(1.,.2,0.)
        for t in [1.,1.025,2.,10.]:self.assertEqual(self.guard.schedule(t,self.diag,self.metrics,[])[1],0.)
    def test_path_progress_keeps_full_model_after_handoff(self):
        w,b=m.transfer_schedule(2.,np.array([0.,1.]),np.array([[0]*21,[.1]*21]))
        self.assertGreater(w,0.);self.assertEqual(b,1.)

if __name__=='__main__':unittest.main()
