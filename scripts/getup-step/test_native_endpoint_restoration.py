"""Pure native-equilibrium algebra/API tests. No real optimizer/native world."""
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import native_endpoint_restoration as n

class AlgebraProvider:
    bounds=((-2.,2.),)*27
    shape_bounds='original-experimental'
    def __init__(self):self.calls=[]
    def __call__(self,k,q,*,strict=False):
        self.calls.append((k,np.asarray(q).copy(),strict))
        # Every path coordinate affects native motor demand, while force
        # columns and root balance remain exactly linear, finite everywhere.
        B=np.r_[np.zeros((6,21)),np.eye(21)];G=np.zeros((27,9));G[6:15]=np.eye(9)
        bias=np.r_[np.zeros(6),np.tile(q,3)[:21]*.01]
        result=n.stationary_terms(B,G,bias,1.)
        result.update(kinematic=np.ones(68)+np.sum(q)*.01,minimumCleanSelfGapM=.04,commandCOMHeightM=.8,selectedProfileGeometryAccepted=True,
            native=dict(nativeCounter=0,nativeCOMHeightM=.8,originalStrictLP=dict(success=True,requiredMotorFraction=.5,maximumEqualityResidual=0.,maximumFrictionViolation=0.)))
        return result

class EndpointTests(unittest.TestCase):
    def problem(self,provider=None):
        return n.EndpointForceProblem(provider or AlgebraProvider(),np.zeros((3,9)),np.zeros((3,9)),
            np.tile([0.,0.,1/3],9).reshape(3,3,3),binding=dict(shapeBounds='original-experimental'))

    def test_exact_schur_equilibrium_with_nonzero_root_actuator_rows(self):
        rng=np.random.default_rng(18);Bj=rng.normal(size=(21,21));Bj=Bj@Bj.T+np.eye(21)
        B=np.r_[rng.normal(size=(6,21)),Bj];G=rng.normal(size=(27,9));W=432.;f=rng.normal(size=9);u=rng.normal(size=21)
        bias=B@u+G@(W*f);record=n.stationary_terms(B,G,bias,W)|dict(kinematic=np.ones(68),minimumCleanSelfGapM=.04,commandCOMHeightM=.8)
        c,e,actual=n.endpoint_constraints(record,f);np.testing.assert_allclose(actual,u,atol=1e-12);np.testing.assert_allclose(e,0.,atol=1e-12)
        J,E=n.force_jacobians(record)
        for j in range(9):
            shifted=f.copy();shifted[j]+=1e-6;d=shifted[j]-f[j];a,b,_=n.endpoint_constraints(record,shifted)
            np.testing.assert_allclose((a-c)/d,J[:,j],rtol=1e-7,atol=1e-7);np.testing.assert_allclose((b-e)/d,E[:,j],rtol=1e-7,atol=1e-7)

    def test_exact_endpoint_cache_and_only27_native_fd_columns(self):
        p=self.problem();c,e,_=p.evaluate(p.initial);self.assertEqual((len(c),len(e)),(372,18))
        J,E=p.jacobians(p.initial);self.assertEqual(J.shape,(372,54));self.assertEqual(E.shape,(18,54))
        self.assertEqual(len(p.provider.calls),30);self.assertEqual(p.stats['nativePathFDColumns'],27);self.assertEqual(p.stats['analyticForceColumns'],27)
        p.jacobians(p.initial.copy());self.assertEqual(len(p.provider.calls),30)
        changed=p.initial.copy();changed[27]+=.001;p.evaluate(changed);self.assertEqual(len(p.provider.calls),30)
        np.testing.assert_array_equal(p.objective_jacobian(changed)[27:],0.);self.assertEqual(p.objective(changed),0.)

    def test_rejected_endpoint_does_not_create_a_synthetic_derivative(self):
        class Reject(AlgebraProvider):
            def __call__(self,k,q,*,strict=False):
                if q[0]>1e-7:raise ValueError('genuine endpoint branch failure')
                return super().__call__(k,q,strict=strict)
        p=self.problem(Reject())
        with self.assertRaisesRegex(ValueError,'genuine endpoint'):p.jacobians(p.initial)
        self.assertEqual(p.stats['nativePathFDColumns'],0);self.assertEqual(len(p.jac_cache),0)

    def test_mock_solve_has_zero_force_objective_and_never_admits_a_gait(self):
        p=self.problem()
        def optimizer(fun,z,**kw):
            self.assertEqual(len(z),54);self.assertEqual(len(kw['constraints']),2);self.assertEqual(kw['options']['maxiter'],80)
            np.testing.assert_array_equal(kw['jac'](z)[27:],0.);kw['callback'](z)
            return SimpleNamespace(x=z,success=True,message='MOCK only')
        with patch.object(n,'minimize',side_effect=optimizer):r=n.restore_endpoints(p)
        self.assertEqual(r['optimizerCalls'],1);self.assertTrue(r['endpointFeasible']);self.assertFalse(r['accepted']);self.assertFalse(r['nativeAdmission'])
        self.assertFalse(r['wholeCycleCertificate']);self.assertFalse(r['dynamicPathFeasibilityProven']);self.assertEqual(len(r['finalEndpointChecks']),3)

    def test_cap_retains_exact_completed_major_pose_and_force_witness(self):
        p=self.problem();events=[]
        def optimizer(fun,z,**kw):kw['callback'](z);raise n.ComputeCap('MOCK explicit cap')
        with patch.object(n,'minimize',side_effect=optimizer):r=n.restore_endpoints(p,progress_callback=events.append)
        self.assertEqual(r['terminalState'],'compute-cap');self.assertFalse(r['optimizerSuccess']);self.assertEqual(r['completedMajorIterations'],1)
        np.testing.assert_array_equal(r['unadmittedIndependentKnotsRad'],p.initial[:27].reshape(3,9))
        np.testing.assert_array_equal(r['unadmittedNormalizedForces'],p.initial[27:].reshape(3,3,3));self.assertEqual(len(events),1)

    def test_invalid_initial_forces_and_singular_actuation_fail_closed(self):
        with self.assertRaises(ValueError):n.EndpointForceProblem(AlgebraProvider(),np.zeros((3,9)),np.zeros((3,9)),np.ones((3,3,3))*2)
        with self.assertRaises(np.linalg.LinAlgError):n.stationary_terms(np.zeros((27,21)),np.zeros((27,9)),np.zeros(27),1.)
        with self.assertRaises(ValueError):n.EndpointForceProblem(AlgebraProvider(),np.zeros((3,9)),np.zeros((3,9)),np.zeros((3,3,3)),binding=dict(shapeBounds='source-physical'))

    def test_one_declared_numerical_interior_preserves_original_final_cap(self):
        self.assertEqual(n.MOTOR_CAP,.925);self.assertEqual(n.OPTIMIZER_MOTOR_CAP,.925-1e-7)
        p=self.problem();record=p.endpoint(0,np.zeros(9));c,_,u=n.endpoint_constraints(record,p.initial[27:36])
        np.testing.assert_array_equal(c[68:89],n.OPTIMIZER_MOTOR_CAP-u)
        self.assertEqual(p.binding['planningNumericalInterior'],1e-7);self.assertEqual(p.binding['originalStrictMotorCap'],.925)
        class FinalOriginal(AlgebraProvider):
            def __call__(self,k,q,*,strict=False):
                result=super().__call__(k,q,strict=strict)
                result['native']['originalStrictLP']['requiredMotorFraction']=.925-.5e-7
                return result
        with patch.object(n,'minimize',side_effect=lambda fun,z,**kw:SimpleNamespace(x=z,success=True,message='MOCK')):
            r=n.restore_endpoints(self.problem(FinalOriginal()))
        self.assertTrue(r['endpointFeasible']);self.assertEqual(r['optimizerMotorCap'],.925-1e-7);self.assertFalse(r['nativeAdmission'])

    def test_source_physical_profile_changes_only_experimental_lumbar_window(self):
        from wholebody_math_fixture import make_math_fixture
        f=make_math_fixture();context=replace(f.context,phase='transfer',elapsed=4.);support=replace(f.support,phase='transfer')
        knots=np.tile(np.asarray(context.joints.position)[f.realizer.independent_indices],(3,1))
        a=n.StanceEndpointProvider(f.realizer,context,f.limits,f.heading,support,knots,SimpleNamespace(),shape_bounds='original-experimental')
        b=n.StanceEndpointProvider(f.realizer,context,f.limits,f.heading,support,knots,SimpleNamespace(),shape_bounds='source-physical')
        np.testing.assert_array_equal(a.kin.lo[3:],b.kin.lo[3:]);np.testing.assert_array_equal(a.kin.hi[3:],b.kin.hi[3:])
        np.testing.assert_array_equal(b.kin.lo[:3],f.limits[:3,0]+.01);np.testing.assert_array_equal(b.kin.hi[:3],f.limits[:3,1]-.01)
        self.assertTrue(np.all(b.kin.lo[:3]<a.kin.lo[:3]));self.assertTrue(np.all(b.kin.hi[[6,12]]<=0.))

    def test_stationary_provider_rejects_moving_joint_or_root_jets_before_install(self):
        import copy
        from coupled_posture_reference import Jet,OrientationJet
        from coupled_posture_kinematics import MatrixJet
        from unittest.mock import Mock
        world=SimpleNamespace(mass=1.,h=0,l=SimpleNamespace(lh_dim=lambda *args:0),install_cold=Mock())
        provider=n.NativeStationaryScratch(world,[(1,(0.,0.,0.)),(1,(1.,0.,0.)),(1,(0.,1.,0.))])
        pose=MatrixJet.rest(np.eye(4));one=OrientationJet.rest(1,np.eye(3));four=OrientationJet.rest(4,np.eye(3));com=Jet.rest([0.,0.,.8])
        geometric=SimpleNamespace(full_joints=Jet.rest(np.zeros(21)),com_source=com,root_model=pose,thorax=one,pelvis=four,
            future_thigh_right_left=Jet.rest([0.,0.]),future_arm_delta_right_left=Jet.rest([0.,0.]),poses_model=(('torso',pose),('left_foot',pose),('right_foot',pose)),
            pose=lambda name:pose)
        baseline=SimpleNamespace(lower_joints=Jet.rest(np.zeros(15)),com=com,thorax=one,pelvis=four,feet=(pose,pose),geometric=geometric)
        self.assertEqual(n.stationary_sample_guard(baseline)['maximumStationaryJetRoundoff'],0.)
        for field in('lower_joints','com','thorax','pelvis','full_joints','root_model','foot','geometricCOM','COMoffset'):
            sample=copy.deepcopy(baseline)
            def moving(obj):
                fields=vars(obj).copy();fields['velocity']=np.ones(np.shape(obj.velocity))*1e-4;return SimpleNamespace(**fields)
            if field in('lower_joints','com','thorax','pelvis'):setattr(sample,field,moving(getattr(sample,field)))
            elif field in('full_joints','root_model'):setattr(sample.geometric,field,moving(getattr(sample.geometric,field)))
            elif field=='foot':sample.feet=(moving(pose),pose)
            elif field=='geometricCOM':sample.geometric.com_source=moving(com)
            else:sample.com=Jet.rest([.001,0.,.8])
            with self.subTest(field=field),self.assertRaises(ValueError):provider.evaluate(sample)
            world.install_cold.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
