#!/usr/bin/env python3
"""Focused diagnostics tests; native parity check makes zero integrations."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from source_step_probe import Controller,ROOT,World
from source_arm_knee_mobility import native_bytes
from source_unconstrained_steps import UnconstrainedSteps
from source_unconstrained_scheduled import RelaxedController
from source_robust_support_steps import RobustSupportSteps


class ReportOnlyTests(unittest.TestCase):
    def observer(self):
        live=UnconstrainedSteps.__new__(UnconstrainedSteps)
        live.quality_events={};live.quality_counts={};live.slip=np.array([.02,.03])
        live.minimum_com_height=10.;live.minimum_joint=1.;live.minimum_capture=1.
        live.maximum_penetration=live.maximum_tilt=live.maximum_nonfoot_load=0.
        live.failure=None
        return live

    def test_quality_thresholds_are_recorded_without_stopping(self):
        live=self.observer()
        row=dict(elapsedS=8.,sourceStep=6400,stepIndex=1,phase='settle',warnings=0,
            nonfootLoadN=500.,feet=[dict(loadN=0.),dict(loadN=0.)],
            maximumSelfPenetrationM=.02,com=[0.,0.,.3],jointLimitMinimumMarginRad=-.1,
            torsoTiltDeg=95.,minimumComCaptureMarginM=-.2)
        live.observe_quality(row)
        self.assertIsNone(live.failure)
        self.assertEqual(live.quality_counts['global-loaded-slip-20mm'],1)
        self.assertEqual(live.minimum_com_height,.3)
        self.assertEqual(live.maximum_nonfoot_load,500.)

    def test_first_violation_is_stable_and_counts_continue(self):
        live=self.observer()
        row=dict(elapsedS=1.,sourceStep=800,stepIndex=1,phase='transfer')
        live.record_quality('example',row)
        row['elapsedS']=2.;live.record_quality('example',row)
        self.assertEqual(live.quality_events['example']['timeS'],1.)
        self.assertEqual(live.quality_counts['example'],2)

    def test_metadata_never_promotes_diagnostic_or_hides_unrecorded_terminal(self):
        live=self.observer()
        live.finished=False;live.scheduled=False;live.distances=(.18,)
        live.steps=4000;live.recorded_integrations=3999;live.quiet_run=0
        live.step_records=[]
        with tempfile.TemporaryDirectory()as d:
            live.folder=Path(d)
            def fake_parent_save(obj,reason):
                for name in ('summary.json','multistep-summary.json','cadenced-step-config.json','large-step-config.json'):
                    (obj.folder/name).write_text(json.dumps(dict(passed=True,runtimeMeasuredCaptureAndJointGuards=True)))
            with patch.object(RobustSupportSteps,'save',fake_parent_save):live.save('synthetic-finite-terminal')
            summary=json.loads((live.folder/'summary.json').read_text())
            multi=json.loads((live.folder/'multistep-summary.json').read_text())
            config=json.loads((live.folder/'large-step-config.json').read_text())
            self.assertEqual(summary['recordedIntegrations'],3999)
            self.assertEqual(summary['unjournaledTerminalIntegrations'],1)
            self.assertFalse(multi['passed'])
            self.assertFalse(multi['sequenceAndQuietCompleted'])
            self.assertFalse(config['runtimeMeasuredCaptureAndJointGuards'])
            self.assertTrue(config['physicalModelGravityCollisionsJointLimitsAndActuatorsPreserved'])


class NativeZeroStepTests(unittest.TestCase):
    def test_relaxed_core_does_not_apply_forces_or_change_native_bytes(self):
        base=ROOT/'outputs/unconstrained-walk-20261019/report-only-stops/model-session'
        helper=ROOT/'outputs/getup-source-step/first-liftoff/source_dynamics-v9.so'
        if not (base/'slow-continuation/final.bin').exists():self.skipTest('local native diagnostic assets absent')
        w=World(base,helper)
        try:
            w.restore(base/'slow-continuation/final.bin')
            c=Controller(w,0,.182,True,maximum_distance=.302)
            c.__class__=RelaxedController
            # A deliberately impossible injected planner inequality is ignored
            # by the isolated relaxed optimizer, but native physics is untouched.
            def impossible(phase,width,Jcom,com,v,ineq,bounds):
                ineq.append(np.zeros(width));bounds.append(-1.)
            c.add_capture_constraints=impossible
            before=native_bytes(w);counter=w.l.lh_dim(w.h,6)
            u,info=c.relaxed_core(0.,'transfer')
            self.assertTrue(np.isfinite(u).all())
            self.assertLessEqual(float(max(abs(u))),1.+1e-8)
            self.assertLess(info['dynamicsResidual'],1e-6)
            self.assertEqual(native_bytes(w),before)
            self.assertEqual(w.l.lh_dim(w.h,6),counter)
        finally:w.close()


if __name__=='__main__':unittest.main()
