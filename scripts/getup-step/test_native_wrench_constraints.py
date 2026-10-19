"""Pure cone/native algebra; mathematical force LPs, no native/trajectory solve."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from coupled_posture_reference import Jet
import native_wrench_constraints as n

class ConeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points=np.array([[-.09,-.04,.002],[.08,-.035,-.001],[.015,.06,.004]])
        cls.cone=n.ContactWrenchCone(cls.points,cls.points.mean(axis=0))

    def test_no_joggle_actual3d_facets_and_independent_boundary_outside_lp(self):
        c=self.cone;self.assertFalse(c.receipt['QJ']);self.assertFalse(c.receipt['flattenedContacts'])
        self.assertEqual(c.receipt['normalForceDomain'],'nonnegative-unbounded');self.assertGreater(len(c.facets),5)
        self.assertLessEqual(float(np.max(c.H@c.rays)),1e-11)
        self.assertTrue(c.primal_check(np.zeros(6))['success'])
        self.assertTrue(c.primal_check(c.rays.sum(axis=1))['success'])
        for facet in c.facets:
            boundary=np.mean(c.rays[:,facet['support']],axis=1)
            self.assertGreaterEqual(float(min(c.slacks(boundary))),-1e-10)
            self.assertTrue(c.primal_check(boundary)['success'])
            outside=(boundary/c.scale+1e-4*facet['normal'])*c.scale
            self.assertLess(float(min(c.slacks(outside))),-1e-7)
            self.assertFalse(c.primal_check(outside)['success'])

    def test_random_original_force_images_and_determinism(self):
        c=self.cone;rng=np.random.default_rng(24)
        for _ in range(25):
            f=rng.uniform(-.6,.6,(3,3));f[:,2]=rng.uniform(0.,10.,3);f[:,:2]*=f[:,2,None]
            w=c.D@f.reshape(9);self.assertGreaterEqual(float(min(c.slacks(w))),-1e-10);self.assertTrue(c.primal_check(w)['success'])
        again=n.ContactWrenchCone(self.points,self.points.mean(axis=0));np.testing.assert_array_equal(c.H,again.H)
        with self.assertRaises(ValueError):n.ContactWrenchCone(np.zeros((3,3)),np.zeros(3))

    def native_data(self,q=None):
        c=self.cone;rng=np.random.default_rng(10);B=np.r_[np.zeros((6,21)),np.diag(np.linspace(1.,3.,21))]
        J=np.c_[np.eye(6),rng.normal(size=(6,21))*.1];f=np.tile([0.,0.,1.],3);w=c.D@f
        u=np.linspace(-.3,.3,21)
        if q is not None:u[:9]+=np.asarray(q)*.01
        d=B@u+J.T@w
        return dict(B=B,stanceJacobian=J,G=J.T@c.D,demand=d,referenceOriginSource=c.origin.copy(),
            forceScaleN=3.,nativeCOMHeightM=.8,minimumCleanSelfGapM=.04),u,f

    def test_native_torque_unique_under_all_same_wrench_force_redistributions(self):
        data,u,f=self.native_data();c=self.cone;reduced=n.eliminate_native_wrench(data,c)
        np.testing.assert_allclose(reduced['motor'],u,atol=1e-14)
        _,_,V=np.linalg.svd(c.D,full_matrices=True);redistribution=V[-1]*.01
        np.testing.assert_allclose(c.D@(f+redistribution),c.D@f,atol=1e-14)
        motor=np.linalg.solve(data['B'][6:],data['demand'][6:]-data['G'][6:]@(f+redistribution))
        np.testing.assert_allclose(motor,u,atol=1e-14)
        final=n.independent_native_primal_check(data,c);self.assertTrue(final['accepted']);self.assertIsNone(final['normalForceUpperBound'])

    def test_root_actuation_wrong_origin_or_factorization_rejected(self):
        for kind in('root','origin','factorization'):
            data,_,_=self.native_data()
            if kind=='root':data['B'][0,0]=1e-6
            elif kind=='origin':data['referenceOriginSource'][0]+=1e-6
            else:data['G'][0,0]+=1e-4
            with self.subTest(kind=kind),self.assertRaises(ValueError):n.eliminate_native_wrench(data,self.cone)

    def test_trust_box_intersects_original_bounds_without_projecting_center(self):
        bounds=n.local_trust_bounds([(-1.,1.)]*27,np.zeros(27),.003012189503921292)
        self.assertEqual(bounds[0],(-.003012189503921292,.003012189503921292))
        with self.assertRaises(ValueError):n.local_trust_bounds([(-1.,1.)]*27,np.ones(27)*2.,.01)

class ThinEvaluatorTests(ConeTests):
    def test_candidate_and_jacobian_cache_hits_still_observe_caller_budget(self):
        def stopped():raise RuntimeError('caller cooperative cap')
        kin=SimpleNamespace(bounds=((-1.,1.),)*27,nodes=(),check_control=stopped)
        e=n.NativeWrenchPathEvaluator(kin,None,self.cone,sample_provider=None)
        x=np.zeros(27);key=x.tobytes()
        e.cache[key]=(np.ones(1),{});e.jac_cache[key]=np.ones((1,27))
        for call in(e.evaluate,e.jacobian):
            with self.subTest(call=call.__name__),self.assertRaisesRegex(RuntimeError,'cooperative cap'):call(x)

    def test_existing_geometry_rows_preserved27_columns_and_exact_cache(self):
        parent=self
        class Kin:
            bounds=((-1.,1.),)*27
            nodes=tuple((k,f)for k in range(3)for f in(0.,.25,.5,.75,1.)if not(k and f==0.))
            check_control=staticmethod(lambda:None)
            def path(self,x):
                x=np.asarray(x).copy()
                return SimpleNamespace(x=x,durations=(1.,1.,1.),independent_segments=[SimpleNamespace(at=lambda t,k=k:Jet.rest(x[9*k:9*k+9]))for k in range(3)])
            def evaluate(self,x):return np.ones(1102)+sum(x)*.01,tuple(dict(maximumPoseResidual=0.)for _ in range(231))
            def constraint_jacobian(self,x):return np.ones((1102,27))*.01
            def node(self,path,k,f):return np.ones(68)+sum(path.x[9*k:9*k+9])*.01,dict(maximumPoseResidual=0.)
        class Sampler:
            def sample(self,path,phase,t):
                k=('lift','translate','land').index(phase);return SimpleNamespace(q=path.x[9*k:9*k+9],com=Jet.rest([0.,0.,.8]))
        calls=[]
        def provider(sample,*,final=False):calls.append(sample.q.copy());return parent.native_data(sample.q)[0]
        e=n.NativeWrenchPathEvaluator(Kin(),provider,self.cone,sample_provider=Sampler());x=np.zeros(27)
        c,_=e.evaluate(x);np.testing.assert_array_equal(c[:1102],1.);before=len(calls)
        again,_=e.evaluate(x.copy());self.assertIs(c,again);self.assertEqual(len(calls),before)
        J=e.jacobian(x);self.assertEqual(J.shape,(len(c),27));np.testing.assert_array_equal(J[:1102],.01)
        self.assertEqual(e.stats['nativeFDColumns'],27);self.assertEqual(len(calls),130)
        with patch.object(n,'stationary_sample_guard',return_value={}):
            endpoint=n.NativeWrenchPathEvaluator(Kin(),provider,self.cone,endpoint_only=True,sample_provider=Sampler())
            v,_=endpoint.evaluate(x);self.assertEqual(len(v),204+3*(42+len(self.cone.H)+3));self.assertEqual(endpoint.jacobian(x).shape,(len(v),27))

if __name__=='__main__':unittest.main(verbosity=2)
