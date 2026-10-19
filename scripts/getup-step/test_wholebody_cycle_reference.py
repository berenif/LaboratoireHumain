"""Pure reference/analytic tests. Never runs an optimizer or opens a world."""
import json,math,unittest,os
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from wholebody_math_fixture import make_math_fixture
from wholebody_cycle_reference import (WholeBodyCycle,Jet,PathRejected,pose_jet_to_source,
    FEET,StanceRealizedIK,MAX_TOTAL_COM_SPEED,MAX_TOTAL_COM_ACCELERATION)
from coupled_posture_reference import CurrentSupport

class CycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture=make_math_fixture()
        cls.inputs,cls.context,cls.realizer,cls.limits,cls.support,cls.binding=(fixture.inputs,fixture.context,
            fixture.realizer,fixture.limits,fixture.support,fixture.binding)
        cls.q=np.array(cls.context.joints.position)[:15]
        cls.knots=np.tile(cls.q[cls.realizer.independent_indices],(3,1))
        cls.path=WholeBodyCycle(cls.realizer,cls.context,cls.limits,cls.inputs['controller']['heading'],cls.knots,
            design_only=True,entry_elapsed=cls.context.elapsed)

    def test_entry_all_command_jets_preserved(self):
        sample=self.path.sample('lift',0.,runtime_arms=self.context.arms)
        for a,b in zip(sample.com.arrays(),self.context.com.arrays()):np.testing.assert_array_equal(a,b)
        for a,b in zip(sample.lower_joints.arrays(),self.context.joints.arrays()):np.testing.assert_array_equal(a,b[:15])
        original=next(o for o in self.context.orientations if o.body==1)
        for a,b in zip(sample.thorax.arrays(),original.arrays()):np.testing.assert_allclose(a,b,atol=5e-16)
        for pose,old in zip(sample.feet,self.context.feet):
            s=pose_jet_to_source(pose)
            for a,b in zip(s.translation.arrays(),old.translation.arrays()):np.testing.assert_allclose(a,b,atol=1e-15)
            np.testing.assert_allclose(np.array(s.rotation).reshape(3,3),old.orientation.arrays()[0],atol=1e-15)

    def test_analytic_full15_nonzero_jets(self):
        r=self.realizer;v=np.linspace(-.03,.03,15);a=np.linspace(.01,-.01,15)
        known=Jet(self.q,v,a)
        target=r.model.coupled(known,r.stance_side,r.stance_pose,r.arm_map,r.leg_origin)
        independent=Jet(*(x[r.independent_indices]for x in known.arrays()))
        lower,result,receipt=r.derive(independent,target.pose(FEET[r.swing_side]),self.q)
        for actual,expected in zip(lower.arrays(),known.arrays()):np.testing.assert_allclose(actual,expected,atol=2e-9)
        self.assertLess(receipt['maximumVelocityResidual'],1e-10)
        self.assertLess(receipt['maximumAccelerationResidual'],1e-10)

    def test_root_always_stance_realized(self):
        q=self.q.copy();q[5]+=.01
        raw=self.realizer.model.coupled(Jet.rest(q),self.realizer.stance_side,self.realizer.stance_pose,self.realizer.arm_map,self.realizer.leg_origin)
        for a,b in zip(raw.pose(FEET[self.realizer.stance_side]).arrays(),self.realizer.stance_pose.arrays()):np.testing.assert_allclose(a,b,atol=1e-14)
        np.testing.assert_allclose(raw.root_model.T,self.realizer.root_for(Jet.rest(q)).T,atol=1e-14)

    def test_explicit_command_xy_never_advances_in_lift(self):
        p=np.array(self.context.feet[self.realizer.swing_side].translation.position)
        for t in np.linspace(0.,self.path.durations[0],33):
            s=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,0,float(t),float(t)))
            np.testing.assert_array_equal(np.array(s.translation.position)[:2],p[:2])
            np.testing.assert_array_equal(np.array(s.translation.velocity)[:2],[0.,0.])
            np.testing.assert_array_equal(np.array(s.translation.acceleration)[:2],[0.,0.])

    def test_exact182mm_and40mm_command_endpoints(self):
        initial=np.array(self.context.feet[self.realizer.swing_side].translation.position)
        high=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,0,self.path.durations[0],self.path.durations[0])).translation
        advanced=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,1,1.2,sum(self.path.durations[:2]))).translation
        np.testing.assert_allclose(np.array(high.position)-initial,[0.,0.,.04],atol=1e-15)
        self.assertAlmostEqual(float((np.array(advanced.position)-high.position)@self.path.heading),.182,places=14)

    def test_waits_are_c2(self):
        for phase,duration in zip(('lift','translate','land'),self.path.durations):
            k,t,total=self.path.clock(phase+'_wait',99.)
            self.assertEqual(t,duration)
            foot=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,k,t,total))
            self.assertEqual(foot.translation.velocity,(0.,0.,0.));self.assertEqual(foot.translation.acceleration,(0.,0.,0.))

    def test_causal_settle_required(self):
        for phase in ('settle','stand','quiet'):
            with self.assertRaisesRegex(PathRejected,'causal measured'):self.path.sample(phase,0.)

    def test_call_order_and_no_mutable_ik_seed(self):
        expected=self.path.sample('lift',0.,runtime_arms=self.context.arms)
        self.path.sample('lift',.01,runtime_arms=self.context.arms)
        again=self.path.sample('lift',0.,runtime_arms=self.context.arms)
        self.assertEqual(expected.lower_joints,again.lower_joints);self.assertEqual(expected.com,again.com)

    def test_initial_context_identity_and_arm_identity(self):
        with patch('scipy.optimize.minimize',side_effect=AssertionError('optimizer forbidden')):
            self.assertIs(self.path.prepare_context(self.context,self.support),self.context)
        sample=self.path.sample('lift',0.,runtime_arms=self.context.arms)
        self.assertEqual(sample.geometric.full_joints.position[15:],self.context.arms.position)
        self.assertFalse(sample.diagnostics['runtimeArmsReplaced']);self.assertFalse(sample.diagnostics['all21TasksExactlyCompatibleClaim'])

    def test_existing_pelvis_compatibility_and_gains_preserved(self):
        from source_reference_context import Orientation
        from scipy.spatial.transform import Rotation
        R=Rotation.from_euler('ZYX',[.2,.1,-.05]).as_matrix()
        pelvis=Orientation.make(4,R,[.01,.02,-.03],[.02,-.01,.01],kp=31.,kd=7.,weight=2.5)
        context=replace(self.context,orientations=self.context.orientations+(pelvis,))
        path=WholeBodyCycle(self.realizer,context,self.limits,self.inputs['controller']['heading'],self.knots,
            design_only=True,entry_elapsed=self.context.elapsed)
        sample=path.sample('lift',0.,runtime_arms=context.arms)
        for a,b in zip(sample.pelvis.arrays(),pelvis.arrays()):np.testing.assert_allclose(a,b,atol=1e-14)
        later=replace(context,source_step=context.source_step+1,source_time=context.source_time+.00125,elapsed=0.)
        support=replace(self.support,source_step=later.source_step,source_time=later.source_time)
        prepared=path.prepare_context(later,support)
        actual=next(o for o in prepared.orientations if o.body==4)
        self.assertEqual((actual.kp,actual.kd,actual.weight),(31.,7.,2.5))
        for a,b in zip(actual.arrays(),sample.pelvis.arrays()):np.testing.assert_allclose(a,b,atol=1e-14)
        self.assertIs(prepared.arms,later.arms);self.assertIs(prepared.arm_gains,later.arm_gains)
        self.assertIs(prepared.capture_constraints,later.capture_constraints)
        for a,b in zip(prepared.joints.arrays(),later.joints.arrays()):np.testing.assert_array_equal(a[15:],b[15:])

    def test_adapter_does_not_silently_add_pelvis(self):
        later=replace(self.context,source_step=self.context.source_step+1,source_time=self.context.source_time+.00125,elapsed=0.)
        support=replace(self.support,source_step=later.source_step,source_time=later.source_time)
        prepared=self.path.prepare_context(later,support)
        self.assertEqual([o.body for o in prepared.orientations],[1])

    def test_differing_weak_strong_arm_tail_never_moved_or_mutated(self):
        p,v,a=self.context.joints.arrays();p[15:]+=.001;v[15:]=np.linspace(.1,.2,6);a[15:]=np.linspace(.2,.3,6)
        weak=type(self.context.joints).make(p,v,a)
        context=replace(self.context,joints=weak)
        before=context.receipt()
        path=WholeBodyCycle(self.realizer,context,self.limits,self.inputs['controller']['heading'],self.knots,
            design_only=True,entry_elapsed=context.elapsed)
        self.assertIs(path.prepare_context(context,self.support),context)
        later=replace(context,source_step=context.source_step+1,source_time=context.source_time+.00125,elapsed=0.)
        support=replace(self.support,source_step=later.source_step,source_time=later.source_time)
        prepared=path.prepare_context(later,support)
        self.assertNotEqual(prepared.joints.position[15:],prepared.arms.position)
        for actual,original in zip(prepared.joints.arrays(),weak.arrays()):np.testing.assert_array_equal(actual[15:],original[15:])
        self.assertIs(prepared.arms,later.arms);self.assertIs(prepared.arm_gains,later.arm_gains)
        self.assertEqual(context.receipt(),before)

    def test_runtime_constructor_rejects_first_lift_witness(self):
        with self.assertRaisesRegex(ValueError,'stationary nominal'):
            WholeBodyCycle(self.realizer,self.context,self.limits,self.inputs['controller']['heading'],self.knots)

    def test_current_support_and_caps_not_relaxed(self):
        initial=self.path.sample('lift',0.,runtime_arms=self.context.arms)
        self.assertTrue(self.path.validate_sample(initial,self.support)['accepted'])
        fast=replace(initial,com=Jet(initial.com.position,[MAX_TOTAL_COM_SPEED+.01,0.,0.],[0.,0.,0.]))
        self.assertFalse(self.path.validate_sample(fast,self.support)['accepted'])
        accel=replace(initial,com=Jet(initial.com.position,[0.,0.,0.],[MAX_TOTAL_COM_ACCELERATION+.01,0.,0.]))
        self.assertFalse(self.path.validate_sample(accel,self.support)['accepted'])

    def test_contact_seek_retained_in_owned_command_jets(self):
        original=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,2,1.,sum(self.path.durations)))
        changed=pose_jet_to_source(self.path.command_foot(self.realizer.swing_side,2,1.,sum(self.path.durations),(.001,.002,.003)))
        for a,b,value in zip(original.translation.arrays(),changed.translation.arrays(),(.001,.002,.003)):
            np.testing.assert_allclose(a-b,[0.,0.,value],atol=1e-15)

    def test_bounded_immutable_knots(self):
        with self.assertRaises(ValueError):self.path.independent_knots[0,0]=0.
        for segment in self.path.independent_segments:
            bounds=segment.component_bounds();np.testing.assert_allclose(bounds[:,0],bounds[:,1],atol=1e-15)

    def test_portable_horizon_one_call_and_retained_rejection(self):
        import wholebody_horizon_solver
        from types import SimpleNamespace
        calls=[]
        def fake_optimizer(objective,initial,**kwargs):
            calls.append(kwargs)
            bad=np.array(initial,copy=True);bad[0]+=.2
            return SimpleNamespace(x=bad,success=False,message='deliberate pure fixture rejection',nit=1)
        with patch.object(wholebody_horizon_solver,'minimize',side_effect=fake_optimizer):
            result,receipt=wholebody_horizon_solver.solve_horizon(self.realizer,self.context,self.limits,
                self.inputs['controller']['heading'],self.support,design_only=True,
                entry_elapsed=self.context.elapsed,analytic_fractions=(0.,1.))
        self.assertEqual(len(calls),1);self.assertEqual(calls[0]['options']['maxiter'],80)
        self.assertEqual(receipt['optimizerCalls'],1);self.assertFalse(receipt['finiteSampledKinematicPass'])
        self.assertFalse(receipt['nativeAdmission']);self.assertGreater(len(receipt['analyticFailures']),0)
        self.assertIsNotNone(result)

    def test_future_producer_cooperative_cancel_retains_unadmitted_candidate(self):
        import wholebody_horizon_solver as solver
        calls=[]
        def fake_optimizer(objective,initial,**kwargs):
            calls.append(1);objective(initial);raise AssertionError('cancellation should have escaped')
        with patch.object(solver,'minimize',side_effect=fake_optimizer):
            path,receipt=solver.solve_horizon(self.realizer,self.context,self.limits,self.inputs['controller']['heading'],
                self.support,design_only=True,entry_elapsed=self.context.elapsed,cancellation_check=lambda:True,
                analytic_fractions=(0.,1.))
        self.assertEqual(len(calls),1);self.assertIsNone(path)
        self.assertEqual(receipt['terminalState'],'caller-cancelled')
        self.assertIsNotNone(receipt['unadmittedLastCandidateIndependentKnotsRad'])
        self.assertFalse(receipt['finiteSampledKinematicPass']);self.assertFalse(receipt['globalInfeasibilityProven'])

    def test_future_producer_wall_cap_is_rejected_and_retained(self):
        import wholebody_horizon_solver as solver
        calls=[];clock_calls=[]
        def clock():clock_calls.append(1);return 0. if len(clock_calls)==1 else 2.
        def fake_optimizer(objective,initial,**kwargs):calls.append(1);return objective(initial)
        with patch.object(solver,'minimize',side_effect=fake_optimizer),patch.object(solver.time,'monotonic',side_effect=clock):
            path,receipt=solver.solve_horizon(self.realizer,self.context,self.limits,self.inputs['controller']['heading'],
                self.support,design_only=True,entry_elapsed=self.context.elapsed,max_wall_seconds=1.,analytic_fractions=(0.,1.))
        self.assertEqual(len(calls),1);self.assertIsNone(path)
        self.assertEqual(receipt['terminalState'],'compute-cap');self.assertFalse(receipt['finiteSampledKinematicPass'])
        self.assertIsNotNone(receipt['unadmittedLastCandidateIndependentKnotsRad'])
        self.assertEqual(receipt['trace'],[])

    def test_future_producer_refuses_over_twenty_minute_cap(self):
        import wholebody_horizon_solver as solver
        with patch.object(solver,'minimize',side_effect=AssertionError('optimizer forbidden')):
            with self.assertRaisesRegex(ValueError,'at most1200'):
                solver.solve_horizon(self.realizer,self.context,self.limits,self.inputs['controller']['heading'],
                    self.support,design_only=True,max_wall_seconds=1200.1)

ARCHIVED_INPUT=Path(__file__).parent.parent/'phase-leg-reference/qualified-lift-entry-input.json'
@unittest.skipUnless(ARCHIVED_INPUT.exists(),'optional qualified archived design witness absent')
class OptionalQualifiedCycleTests(CycleTests):
    @classmethod
    def setUpClass(cls):
        from design_fixture import load
        cls.inputs,cls.context,cls.realizer,cls.limits,cls.support,cls.binding=load()
        cls.q=np.array(cls.context.joints.position)[:15]
        cls.knots=np.tile(cls.q[cls.realizer.independent_indices],(3,1))
        cls.path=WholeBodyCycle(cls.realizer,cls.context,cls.limits,cls.inputs['controller']['heading'],cls.knots,
            design_only=True,entry_elapsed=cls.context.elapsed)

if __name__=='__main__':unittest.main(verbosity=2)
