"""Timing/config checks only; qualified ten-step journals remain required."""
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from scipy.spatial.transform import Rotation
from unittest.mock import patch

import numpy as np
import source_step_probe as source
from source_cadenced_ten_steps import CadencedSteps, CadencedController, PHASE_DURATIONS, thigh_pitch_jet
from source_selected_step import load_preset
from source_arm_knee_mobility import native_bytes

PRESET=load_preset()
BASE=source.ROOT/PRESET['sourceSession']
HELPER=source.ROOT/PRESET['helper']


class DurationContract(unittest.TestCase):
    def test_historical_scale_fallback_is_unchanged(self):
        c=source.Controller.__new__(source.Controller)
        for scale in (1.,2.):
            c.trajectory_time_scale=scale
            for phase,base in dict(transfer=2.,lift=1.,translate=1.,land=1.,settle=2.).items():
                self.assertEqual(c.phase_duration(phase),base*scale)

    def test_phase_durations_are_independent_physical_seconds(self):
        c=source.Controller.__new__(source.Controller)
        c.trajectory_time_scale=2.
        c.phase_durations=PHASE_DURATIONS.copy()
        for phase,duration in PHASE_DURATIONS.items():
            self.assertEqual(c.phase_duration(phase),duration)
        self.assertGreaterEqual(PHASE_DURATIONS['translate'],PHASE_DURATIONS['lift'])

    def test_ten_step_budget_and_preset_are_explicit(self):
        source_text=Path(__file__).with_name('source_cadenced_ten_steps.py').read_text()
        self.assertIn('self.sequence_integration_budget = 160000',source_text)
        self.assertEqual(PRESET['selectedVariant'],'hip10-knee27_5')
        self.assertNotIn('install_cold(',source_text)
        self.assertNotIn('.restore(',source_text)
        self.assertNotIn('lh_set_state',source_text)


class LoadedHandoffReferenceContinuity(unittest.TestCase):
    def test_com_root_posture_and_measured_arm_derivative_history_are_carried(self):
        from source_upright_large_steps import LargeStep
        old=source.Controller.__new__(source.Controller)
        old.com1=np.array([.1,.2,.79]);old.rootR=Rotation.from_euler('z',.4).as_matrix()
        old.lean_axis=np.array([.1,-.02,.01]);old.q0=np.arange(21)*.001
        old.previous_time=7.135;old.previous_leg_rate=np.array([.02,-.03])
        start_R=Rotation.from_rotvec(old.lean_axis).as_matrix()@old.rootR
        new_R=Rotation.from_euler('z',.41).as_matrix();new_axis=np.array([-.12,.03,-.01])
        expected_end=Rotation.from_rotvec(new_axis).as_matrix()@new_R
        live=CadencedSteps.__new__(CadencedSteps);live.step_index=1;live.controller=old;live.step_records=[]
        def begin(instance):
            c=source.Controller.__new__(source.Controller);c.rootR=new_R.copy();c.lean_axis=new_axis.copy()
            c.com0=np.array([.101,.199,.791]);c.com1=np.array([.2,.3,.78]);c.q0=np.zeros(21)
            c.previous_time=None;c.previous_leg_rate=None;instance.controller=c;instance.step_records.append({})
        with tempfile.TemporaryDirectory(prefix='cadenced-handoff-')as folder:
            live.folder=Path(folder)
            with patch.object(LargeStep,'begin_step',begin):live.begin_step()
        np.testing.assert_array_equal(live.controller.com0,old.com1)
        np.testing.assert_allclose(live.controller.rootR,start_R,atol=1e-15)
        np.testing.assert_allclose(Rotation.from_rotvec(live.controller.lean_axis).as_matrix()@live.controller.rootR,expected_end,atol=1e-15)
        np.testing.assert_array_equal(live.controller.posture_start,old.q0)
        self.assertEqual(live.controller.previous_time,old.previous_time)
        np.testing.assert_array_equal(live.controller.previous_leg_rate,old.previous_leg_rate)
        self.assertIsNot(live.controller.previous_leg_rate,old.previous_leg_rate)


class MeasuredCaptureBarrier(unittest.TestCase):
    def controller(self):
        class MeasuredSupport:
            h=None
            l=SimpleNamespace(lh_support_count=lambda h:4)
            def arr(self,key,shape):
                assert key==21
                out=np.zeros(128);out[:8]=[-.1,-.05,.1,-.05,.1,.05,-.1,.05]
                return out
            def read(self,name,shape,body):
                assert name=='lh_jdot'and body==-1
                return np.array([.01,-.02,.03])
        c=CadencedController.__new__(CadencedController);c.w=MeasuredSupport();c.leg_origin=np.zeros(2);c._mapping=SimpleNamespace(ranges=np.array([[-1.,1.],[-1.,1.]]))
        return c

    @patch('source_cadenced_ten_steps.thigh_pitch_jet',return_value=(0.,np.zeros(27),0.))
    def test_capture_and_com_face_inequalities_match_exact_jets(self,unused):
        c=self.controller();J=np.eye(3,27);com=np.array([.01,-.005,.8]);v=np.zeros(27);v[:3]=[.03,-.02,.01]
        rows=[];bounds=[];c.add_capture_constraints('translate',72,J,com,v,rows,bounds)
        self.assertEqual(len(rows),12)
        tau=np.sqrt(com[2]/9.81);tau_rate=v[2]/(2*9.81*tau)
        acceleration=np.linspace(-.2,.3,27);bias=c.w.read('lh_jdot',(3,),-1)
        for i,face in enumerate(c.capture_barrier):
            normal=rows[4+2*i][:2]/tau
            cp_dot=normal@((1+tau_rate)*v[:2]+tau*(J[:2]@acceleration+bias[:2]))
            cp_rate_bound=8*(face['captureMarginM']-.003)
            self.assertAlmostEqual(rows[4+2*i][:27]@acceleration-bounds[4+2*i],cp_dot-cp_rate_bound)
            com_rate=normal@(J[:2]@acceleration+bias[:2])+16*normal@v[:2]
            self.assertAlmostEqual(rows[5+2*i][:27]@acceleration-bounds[5+2*i],com_rate-64*(face['comMarginM']-.003))
            np.testing.assert_array_equal(rows[4+2*i][27:],np.zeros(45))

    @patch('source_cadenced_ten_steps.thigh_pitch_jet',return_value=(0.,np.zeros(27),0.))
    def test_only_single_support_motion_phases_add_barriers(self,unused):
        c=self.controller()
        for phase in ('transfer','settle','stand'):
            rows=[];bounds=[];c.add_capture_constraints(phase,72,np.eye(3,27),np.array([0.,0.,.8]),np.zeros(27),rows,bounds)
            self.assertEqual(len(rows),4);self.assertEqual(len(bounds),4)

    @patch('source_cadenced_ten_steps.thigh_pitch_jet',return_value=(0.,np.zeros(27),0.))
    def test_undefined_support_does_not_fabricate_virtual_support(self,unused):
        c=self.controller();c.w.l=SimpleNamespace(lh_support_count=lambda h:0)
        rows=[];bounds=[];c.add_capture_constraints('lift',72,np.eye(3,27),np.array([0.,0.,.8]),np.zeros(27),rows,bounds)
        self.assertEqual(len(rows),4)
        self.assertEqual(c.capture_barrier,[])


@unittest.skipUnless(BASE.exists()and HELPER.exists(),'local pinned source assets absent')
class NativeTimingParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if subprocess.run(['git','cat-file','-e','0ffb1bb^{commit}'],cwd=source.ROOT,
                          stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            raise unittest.SkipTest('historical 0ffb1bb parity source is not retained in this checkout')
        cls.w=source.World(BASE,HELPER)
        cls.w.restore(BASE/'slow-continuation/final.bin')
        cls.before=native_bytes(cls.w)
        cls.directory=tempfile.TemporaryDirectory(prefix='cadenced-frozen-')
        path=Path(cls.directory.name)/'frozen.py'
        path.write_text(subprocess.check_output(['git','show','0ffb1bb:scripts/getup-step/source_step_probe.py'],cwd=source.ROOT,text=True))
        spec=importlib.util.spec_from_file_location('cadenced_frozen',path)
        cls.frozen=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.frozen)

    @classmethod
    def tearDownClass(cls):
        assert native_bytes(cls.w)==cls.before
        cls.w.close();cls.directory.cleanup()

    def test_default_motor_commands_exactly_match_previous_code(self):
        for avoid in (False,True):
            for scale in (1.,2.):
                for phase in ('transfer','lift','translate','land','settle','stand'):
                    new=source.Controller(self.w,0,.062,avoid)
                    old=self.frozen.Controller(self.w,0,.062,avoid)
                    new.trajectory_time_scale=old.trajectory_time_scale=scale
                    u,info=new.solve(.37,phase)
                    previous,old_info=old.solve(.37,phase)
                    np.testing.assert_array_equal(u,previous)
                    self.assertEqual(info,old_info)

    def test_thigh_pitch_jacobian_and_bias_match_independent_directional_fk(self):
        from source_opposed_arm_swing import limb_telemetry
        rng=np.random.default_rng(731)
        for _ in range(3):
            w=source.World(BASE,HELPER);probe=source.World(BASE,HELPER)
            try:
                w.restore(BASE/'slow-continuation/final.bin');q=w.arr(0,(28,)).copy();v=rng.normal(size=27)*.05
                w.install_cold(dict(qpos=q.tolist(),qvel=v.tolist()))
                before=native_bytes(w)
                for index,side in enumerate(('right','left')):
                    angle,J,bias=thigh_pitch_jet(w,side)
                    values,rates,_=limb_telemetry(w,'leg')
                    self.assertAlmostEqual(angle,values[index],places=12)
                    self.assertAlmostEqual(J@v,rates[index],places=12)
                    sampled=[];eps=2e-5
                    for dt in (-eps,eps):
                        trial=q.copy();trial[:3]+=dt*v[:3];trial[7:]+=dt*v[6:]
                        quat=(Rotation.from_quat(q[[4,5,6,3]])*Rotation.from_rotvec(dt*v[3:6])).as_quat()
                        trial[3:7]=quat[[3,0,1,2]]
                        probe.install_cold(dict(qpos=trial.tolist(),qvel=v.tolist()))
                        sampled.append(thigh_pitch_jet(probe,side)[1]@v)
                    self.assertAlmostEqual((sampled[1]-sampled[0])/(2*eps),bias,delta=2e-5)
                self.assertEqual(native_bytes(w),before)
            finally:
                w.close();probe.close()

    def test_override_clocks_use_exact_per_phase_duration(self):
        for phase in ('transfer','lift','translate','land','settle'):
            c=source.Controller(self.w,0,.062,True)
            c.phase_durations=PHASE_DURATIONS.copy()
            with patch.object(source,'clock',wraps=source.clock)as clock:
                c.solve(.2,phase)
            times=[call.args[1]for call in clock.call_args_list]
            # The COM setup computes the transfer jet before selecting phase.
            self.assertIn(PHASE_DURATIONS['transfer'],times)
            self.assertIn(PHASE_DURATIONS[phase],times)
            self.assertEqual(native_bytes(self.w),self.before)


if __name__=='__main__':unittest.main()
