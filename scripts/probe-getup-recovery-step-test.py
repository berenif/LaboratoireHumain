#!/usr/bin/env python3
"""No-physics tests of contact admission, handoff and evidence accounting."""
import importlib.util
import json
from pathlib import Path
import datetime
import sys
import tempfile
import unittest
from unittest import mock
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('probe',ROOT/'scripts/probe-getup-recovery-step.py')
probe = importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


class FakeNative:
    def __init__(self,model):
        self.model = model;self.inside = False
    def command(self,request):
        if request['op'] == 'diagnostic':
            return {'predicates':{'captureInSupport':self.inside},'capture':{'x':.3,'y':.8,'z':0},
                    'contactMotion':{'feet':[{'comVelocityMps':{'y':.1}} for _ in range(2)]}}
        if request['op'] == 'snapshot':
            return {'segments':[dict(id=k,**v) for k,v in self.model['referenceMetadata']['standingBodyPoses'].items()]}
        raise AssertionError('No physical command permitted in pure tests')


class Tests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
        self.kin = probe.reference.Kinematics(self.model);self.native = FakeNative(self.model)
        self.state = {'observation':self.kin.nominal.tolist()+[0.]*99,
                      'metrics':{'foot_loads_n':[0.,400.],'foot_clearance_m':[0.,0.]}}
        self.controller = probe.OneStep(self.kin)
    def call(self,tick=0,last=None):
        return self.controller.action(self.native,self.state,np.zeros(21),tick,last or [])
    def test_loaded_or_insufficient_stance_never_enters(self):
        for loads in [[3.,400.],[0.,200.],[400.,400.]]:
            self.state['metrics']['foot_loads_n'] = loads
            action,_ = self.call();self.assertEqual(self.controller.phase,'waiting')
            np.testing.assert_array_equal(action,np.zeros(21))
    def test_capture_inside_does_not_request_step(self):
        self.native.inside = True
        self.call();self.assertEqual(self.controller.phase,'waiting')
    def test_only_unloaded_swing_joints_change(self):
        action,note = self.call()
        self.assertEqual(self.controller.phase,'lift');self.assertEqual(note['otherJointMaxDifference'],0.)
        np.testing.assert_array_equal(action[:9],np.zeros(9))
        np.testing.assert_array_equal(action[15:],np.zeros(6))
    def test_horizontal_motion_waits_for_clearance(self):
        _,note = self.call()
        np.testing.assert_allclose(np.array(note['target'])[[0,2]],self.controller.origin[[0,2]])
        self.assertFalse(self.controller.clearance_seen)
    def test_native_recontact_triggers_smooth_release(self):
        self.call()
        self.call(1,[{'loads_n':[40.,360.]}])
        self.assertEqual(self.controller.phase,'release')
        self.assertEqual(self.controller.log[-1]['event'],'native-early-recontact')
        self.call(2);self.call(3);self.assertEqual(self.controller.phase,'done')
        self.assertEqual(self.controller.log[-1]['event'],'source-handoff')
    def test_native_stance_loss_is_not_hidden_by_final_state(self):
        self.call()
        self.call(1,[{'loads_n':[0.,0.]}])
        self.assertEqual(self.controller.phase,'release')
        self.assertEqual(self.controller.log[-1]['event'],'native-stance-support-lost')
    def test_clearance_gate_is_measured(self):
        self.call();self.state['metrics']['foot_clearance_m'][0] = .02
        self.call(1);self.assertTrue(self.controller.clearance_seen)
        self.assertEqual(self.controller.phase,'translate')
    def test_blend_does_not_restart_when_swing_phase_changes(self):
        self.call();self.state['metrics']['foot_clearance_m'][0] = .02
        _,at_gate = self.call(1)
        _,after_gate = self.call(2)
        self.assertEqual(at_gate['blend'],1.)
        self.assertEqual(after_gate['blend'],1.)
        self.assertEqual(after_gate['maximumRepresentationErrorRad'],0.)
    def test_native_recontact_is_checked_across_phase_reset(self):
        self.call();self.state['metrics']['foot_clearance_m'][0] = .02
        self.call(1);self.assertEqual(self.controller.elapsed,0.)
        self.call(2,[{'loads_n':[10.,390.]}])
        self.assertEqual(self.controller.phase,'release')
        self.assertEqual(self.controller.log[-1]['event'],'native-early-recontact')
    def test_target_representation_clipping_is_visible(self):
        goal = self.kin.nominal.copy();goal[9:15] += 2.
        with mock.patch.object(self.kin,'leg_ik',return_value=(goal,np.eye(4),np.zeros(6))):
            self.call();_,note = self.call(1)
        self.assertEqual(note['maximumRepresentationErrorRad'],1.)
        np.testing.assert_allclose(note['representationErrorsRad'],[-1.]*6)
    def test_rising_strategy_rejects_short_unload_and_descending_foot(self):
        self.controller = probe.OneStep(self.kin,'rising-sagittal')
        self.call(last=[{'loads_n':[0.,400.]}]*9)
        self.assertEqual(self.controller.phase,'waiting')
        diag = self.native.command({'op':'diagnostic'});diag['contactMotion']['feet'][0]['comVelocityMps']['y'] = -.1
        self.controller.action(self.native,self.state,np.zeros(21),1,[{'loads_n':[0.,400.]}]*10,diag)
        self.assertEqual(self.controller.phase,'waiting')
    def test_rising_strategy_preserves_eighteen_source_commands(self):
        self.controller = probe.OneStep(self.kin,'rising-sagittal')
        action,note = self.call(last=[{'loads_n':[0.,400.]}]*10)
        self.assertEqual(note['overriddenQposIndices'],[11,12,13])
        self.assertEqual(note['otherJointMaxDifference'],0.)
        np.testing.assert_array_equal(action[[i for i in range(21) if i not in [11,12,13]]],np.zeros(18))
    def test_additive_composition_preserves_source_bias(self):
        self.controller = probe.OneStep(self.kin,'rising-additive')
        goal = self.kin.nominal.copy();goal[[11,12,13]] += [-.1,-.2,.05]
        baseline = np.ones(21)*.2
        with mock.patch.object(self.kin,'leg_ik',return_value=(goal,np.eye(4),np.zeros(6))):
            action,note = self.controller.action(self.native,self.state,baseline,0,[{'loads_n':[0.,400.]}]*10)
        np.testing.assert_allclose(action[[11,12,13]],[.15,.1,.225])
        self.assertEqual(note['otherJointMaxDifference'],0.)
    def test_execution_exception_cannot_be_labelled_zero_physics_preflight(self):
        state = self.state;native_diag = {'nativeSubsteps':0}
        class InterruptedNative:
            def __init__(self,*args):
                self.wall = 0.;self.native = 0;self.uncertain = []
            def command(self,request):
                if request['op'] in ['observe','reset']:return state
                if request['op'] in ['initializedAction','referenceAction']:return {'action':[0.]*21}
                if request['op'] == 'diagnostic':return native_diag
                if request['op'] == 'step':
                    self.wall = .03;self.native = 7
                    raise RuntimeError('Injected synthetic interruption')
                raise AssertionError(request)
            def close(self):pass
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'probe'
            source = str(ROOT/'rust/crates/model/data/getup-v1.json')
            deadline = (datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(seconds=60)).isoformat()
            argv = ['probe','--binary',source,'--checkpoint',source,'--output',str(output),
                    '--physics-budget','1','--deadline',deadline,'--case','baseline','--execute']
            with mock.patch.object(probe,'Native',InterruptedNative),mock.patch.object(sys,'argv',argv):
                with self.assertRaisesRegex(RuntimeError,'synthetic interruption'):probe.main()
            summary = json.loads((output/'summary.json').read_text())
            self.assertEqual(summary['status'],'failed-or-incomplete-probe')
            self.assertEqual(summary['nativeSubsteps'],7)
            self.assertEqual(summary['physicsWallSeconds'],.03)
    def test_snapshot_fk_reconstructs_canonical_poses(self):
        snapshot = self.native.command({'op':'snapshot'})
        root = probe.reference.transform(snapshot['segments'][0]['position'],snapshot['segments'][0]['rotation'])
        # Source nominal elbows are just beyond the physical limit; model
        # construction clamps the initial coordinates, without changing it.
        q = np.clip(self.kin.nominal,self.kin.limits[:,0],self.kin.limits[:,1])
        poses = self.kin.forward(q,root)
        for p in snapshot['segments']:
            np.testing.assert_allclose(poses[p['id']],probe.reference.transform(p['position'],p['rotation']),atol=1e-10)


if __name__ == '__main__':
    unittest.main()
