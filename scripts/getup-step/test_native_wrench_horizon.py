"""Pure mock contracts only: zero native worlds, FK, trajectory/QP optimizers."""
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from coupled_posture_reference import Jet, digest
from coupled_posture_kinematics import MatrixJet
from wholebody_cycle_reference import PathRejected, PHASES
import native_wrench_horizon as h


def primal_pass():
    return dict(success=True,accepted=True,maximumMotorFraction=.4,maximumNativeEquilibriumResidual=0.,
        maximumFrictionViolation=0.,normalForceUpperBound=None,friction=.6)
def static_pass():return dict(success=True,requiredMotorFraction=.4,maximumEqualityResidual=0.,maximumFrictionViolation=0.)


class GenuineBaselineTests(unittest.TestCase):
    def evaluator(self):
        obj=object.__new__(h.GenuineAnalyticPolishEvaluator)
        obj.check_control=lambda:None;obj.nodes=(0,);obj.labels=('a','b');obj.envelope_specs=()
        return obj
    def test_negative_genuine_slacks_remain_exact(self):
        values=np.array([-1.,2.]);rows=({'maximumPoseResidual':0.},)
        with patch.object(h.AnalyticPolishEvaluator,'evaluate',return_value=(values,rows)):
            result=self.evaluator().evaluate(np.zeros(27))
        self.assertIs(result[0],values);self.assertEqual(result[1],rows)
    def test_only_retained_pathrejected_is_domain(self):
        row=dict(errorType='PathRejected',error='exact fixed Newton branch rejected',phase='land',fraction=.5)
        with patch.object(h.AnalyticPolishEvaluator,'evaluate',return_value=(np.full(2,-1.),(row,))):
            with self.assertRaises(PathRejected)as caught:self.evaluator().evaluate(np.zeros(27))
        self.assertEqual(caught.exception.receipt['originalPrescreenError'],row)
        for kind in('ValueError','RuntimeError','ComputeCap'):
            with self.subTest(kind=kind),patch.object(h.AnalyticPolishEvaluator,'evaluate',return_value=(np.full(2,-1.),(dict(row,errorType=kind),))):
                with self.assertRaises(ValueError):self.evaluator().evaluate(np.zeros(27))
    def test_budget_reraised_immediately_after_parent_before_encoded_errors(self):
        obj=self.evaluator();calls=[]
        def control():
            calls.append(1)
            if len(calls)==2:raise h.ComputeCap('real budget')
        obj.check_control=control
        with patch.object(h.AnalyticPolishEvaluator,'evaluate',return_value=(np.full(2,-1.),({'errorType':'ComputeCap','error':'encoded'},))):
            with self.assertRaises(h.ComputeCap):obj.evaluate(np.zeros(27))
        self.assertEqual(len(calls),2)
    def test_incomplete_or_unproven_baseline_never_differentiable(self):
        for result in((np.ones(1),({'maximumPoseResidual':0.},)),(np.ones(2),({},))):
            with patch.object(h.AnalyticPolishEvaluator,'evaluate',return_value=result):
                with self.assertRaises(ValueError):self.evaluator().evaluate(np.zeros(27))


class CertificateTests(unittest.TestCase):
    def fixture(self):
        return SimpleNamespace(support=None,heading=np.array([1.,0.,0.]),
            binding=dict(explicitPureMock=True),context=SimpleNamespace(com=Jet.rest([0.,0.,.8]),joints=Jet.rest(np.zeros(21)),
                receipt=lambda:dict(contextSHA256='mock-context')))
    def sample_row(self,path,sample,k,f,*args):
        return dict(phase=PHASES[k],fraction=f,accepted=True,originalStrictMapEvaluationPassed=True,
            commandCOM=sample.com.position,nominalMetricRetainedDespiteNativeFailure=True)
    def run_certificate(self,provider=None,**kw):
        calls=[];static=[];sample=SimpleNamespace(com=Jet.rest([0.,0.,.8]))
        def default_provider(sample,*,final=False):
            calls.append(final);return dict(minimumCleanSelfGapM=.04,nativeCOMHeightM=.8,native={'mock':True})
        def checker(sample,data):static.append(1);return static_pass()
        with patch.object(h.StrictReferenceCache,'sample',return_value=sample),patch.object(h,'_sample_row',side_effect=self.sample_row),\
                patch.object(h,'independent_native_primal_check',return_value=primal_pass()),patch.object(h,'summarize_samples',return_value={'explicitMock':True}):
            receipt=h.certify_finite_horizon(SimpleNamespace(durations=(.8,1.2,1.),key='mock-path',independent_knots=np.zeros((3,9))),self.fixture(),provider or default_provider,None,checker,**kw)
        return receipt,calls,static
    def test_exact123_and603_order_and_entry_three_endpoint_static(self):
        receipt,calls,static=self.run_certificate();self.assertTrue(receipt['passed']);self.assertTrue(receipt['complete'])
        self.assertEqual(len(receipt['analyticSamples']),123);self.assertEqual(len(receipt['denseCommandSamples']),603)
        self.assertEqual(len(static),4);self.assertTrue(all(final is False for final in calls));self.assertFalse(receipt['wholePathProven'])
        for key,nodes in zip(('analyticSamples','denseCommandSamples'),h.finite_command_nodes((.8,1.2,1.))):
            self.assertEqual([(row['phase'],row['fraction'])for row in receipt[key]],[(PHASES[k],f)for k,t,f in nodes])
    def test_all_native_failures_keep_available_geometry_and_nominal_metrics(self):
        calls=[]
        def provider(sample,*,final=False):
            calls.append(1)
            if len(calls)==2:raise ValueError('mock genuine native-domain error')
            return dict(minimumCleanSelfGapM=.04,nativeCOMHeightM=.64,native={})
        receipt,_,_=self.run_certificate(provider);self.assertFalse(receipt['passed']);self.assertTrue(receipt['complete'])
        self.assertEqual(len(receipt['failures']),726)
        self.assertTrue(all(r.get('nominalMetricRetainedDespiteNativeFailure')for r in receipt['failures']))
        self.assertTrue(any(r.get('errorType')=='ValueError'for r in receipt['failures']))
    def test_certificate_cancellation_is_explicit_incomplete_not_pass(self):
        receipt,calls,static=self.run_certificate(cancellation_check=lambda:True)
        self.assertFalse(receipt['passed']);self.assertFalse(receipt['complete']);self.assertEqual(calls,[]);self.assertEqual(static,[])
        self.assertEqual(receipt['termination'],'certificate-caller-cancelled')
    def test_no_world_or_default_static_checker_is_invented(self):
        with self.assertRaises(ValueError):h.certify_finite_horizon(None,None,None,None,None)


class OrchestrationTests(unittest.TestCase):
    def problem(self):
        anchor=np.zeros((3,9));fixture=SimpleNamespace(realizer=SimpleNamespace(stance_side=1,stance_pose=MatrixJet.rest(np.eye(4))),limits=np.tile([-3.,3.],(21,1)),heading=np.array([1.,0.,0.]))
        fixture.context=SimpleNamespace(phase='transfer',receipt=lambda:dict(contextSHA256='context'))
        points=np.array([[-.1,-.1,0.],[.1,-.1,0.],[.1,.1,0.],[-.1,.1,0.]])
        fixture.support=SimpleNamespace(key='support',equations=lambda:np.zeros((9,3)),stance_points_xyz=points,surviving_corners=(0,1,2))
        fixture.binding=dict(entryLiftElapsedS=0.,contextSHA256='context',currentSupportSHA256='support')
        class Kin:
            def __init__(self,*a,**kw):
                self.nodes=(0,)*13;self.labels=(0,)*68;self.envelope_specs=(0,)*218;self.entry=SimpleNamespace(durations=(.8,1.2,1.))
                self.kw=kw;self.check_control=lambda:None
            def phase_coverage_receipt(self):return {'mockAllPhaseCoverage':True}
            def path(self,x,strict_map=False):
                assert strict_map;return SimpleNamespace(durations=(.8,1.2,1.),key='mock-path',independent_knots=np.asarray(x).reshape(3,9))
            def cache_receipt(self):return {'mock':True}
        class Native:
            def __init__(self,*a,**kw):self.bounds=((-1.,1.),)*27;self.stats={};self.last_rejection=None
            def evaluate(self,x):return np.ones(1),{}
            def jacobian(self,x):return np.zeros((1,27))
        patches=[patch.object(h,'ANCHOR_SHA256',digest(anchor.tolist())),patch.object(h,'GenuineAnalyticPolishEvaluator',Kin),
            patch.object(h,'NativeWrenchPathEvaluator',Native),patch.object(h,'AnalyticCOMGeometry',return_value=object()),
            patch.object(h,'AnalyticNativePlanningSampler',return_value=object())]
        for p in patches:p.start();self.addCleanup(p.stop)
        class Provider:
            contacts=tuple((1,p)for p in points[:3]);origin=np.zeros(3);stance_pose=np.eye(4);stance_side=1
            def __call__(self,*a,**kw):return None
        return h.NativeWrenchHorizon(fixture,Provider(),SimpleNamespace(H=np.zeros((26,6)),receipt={},points=points[:3].copy(),origin=np.zeros(3)),anchor,anchor,
            initial_binding=dict(candidateSHA256=digest(anchor.tolist()),explicitPureMock=True),static_checker=lambda *a:static_pass())
    def solver(self,*a,**kw):
        self.solver_kw=kw;kw['set_control'](lambda:None)
        return dict(coordinates=np.zeros(27).tolist(),numericallyFeasible=True,trajectorySolverInvocations=1,localQPOptimizerCalls=3,
            totalScipyMinimizeCalls=3,lastRequestedCoordinates=np.zeros(27).tolist(),domainRejectedTrials=[],
            linearSubproblemPolicy='row-normalized-feasible-start',linearFeasibilityLPCalls=3,totalScipyLinprogCalls=3,
            linearFeasibilitySubproblems=[dict(explicitPureMock=True)]*3)
    def complete_certificate(self,*a,**kw):
        original,dense=h.finite_command_nodes((.8,1.2,1.))
        def rows(nodes):return [dict(phase=PHASES[k],fraction=f,accepted=True,originalStrictMapEvaluationPassed=True,
            geometryAccepted=True,plannedNativeAccepted=True,originalPhysicalGuardsAccepted=True,
            plannedNativePrimalLP=primal_pass(),additionalStaticAccepted=True,additionalOriginalStaticLP=static_pass())for k,t,f in nodes]
        return dict(passed=True,complete=True,originalStrictMapUsed=True,candidateCertificateBinding=h.certificate_binding(a[0],a[1]),
            analyticSamples=rows(original),denseCommandSamples=rows(dense))
    def test_same1102_profile_and_honest_localqp_counts_with_strict_certificate(self):
        problem=self.problem()
        self.assertEqual(problem.binding['geometryConstraintRows'],1102);self.assertEqual(problem.binding['nativeConstraintRows'],13*(42+26+3))
        self.assertEqual(problem.kin.kw['global_support_phases'],PHASES);self.assertEqual(problem.kin.kw['envelope_brackets'],h.DECLARED_BRACKETS)
        self.assertIsNone(problem.kin.kw['lumbar_window_deg']);self.assertIsNone(problem.binding['experimentalLumbarWindowDeg'])
        self.assertTrue(problem.binding['experimentalLumbarWindowRemoved']);self.assertEqual(problem.binding['designRevision'],h.DESIGN_REVISION)
        with patch.object(h,'restore_feasibility',side_effect=self.solver):path,receipt=problem.solve(certificate_function=self.complete_certificate)
        self.assertIsNotNone(path);self.assertTrue(receipt['finiteNativeFlightFeasible']);self.assertFalse(receipt['accepted'])
        self.assertEqual(receipt['trajectorySolverInvocations'],1);self.assertEqual(receipt['optimizerCalls'],3)
        self.assertEqual(self.solver_kw['domain_exceptions'],(PathRejected,));self.assertEqual(self.solver_kw['max_wall_seconds'],300.)
        self.assertEqual(self.solver_kw['subproblem_reference'],'current');self.assertFalse(receipt['anchorObjectiveOptimizedDuringRestoration'])
        self.assertEqual(self.solver_kw['linear_subproblem_policy'],'row-normalized-feasible-start')
        self.assertEqual(receipt['linearFeasibilityLPCalls'],3);self.assertEqual(receipt['totalScipyLinprogCalls'],3)
        self.assertEqual(receipt['linearSubproblemPolicy'],'row-normalized-feasible-start')
        with self.assertRaises(ValueError):problem.initial[0]=1.
    def test_passflags_without_complete_ordered_native_rows_rejected(self):
        problem=self.problem()
        with patch.object(h,'restore_feasibility',side_effect=self.solver):
            path,receipt=problem.solve(certificate_function=lambda *a,**k:dict(passed=True,complete=True))
        self.assertIsNone(path);self.assertFalse(receipt['finiteNativeFlightFeasible'])
    def test_contact_point_side_order_origin_and_stance_pose_mismatches_rejected(self):
        problem=self.problem()
        for kind in('cone-point','cone-order','cone-origin','provider-side','provider-order','provider-origin','provider-stance'):
            cone=copy.deepcopy(problem.cone);provider=copy.deepcopy(problem.provider)
            if kind=='cone-point':cone.points[0,0]+=1e-8
            elif kind=='cone-order':cone.points=cone.points[::-1]
            elif kind=='cone-origin':cone.origin[0]+=1e-8
            elif kind=='provider-side':provider.contacts=tuple((0,p)for s,p in provider.contacts)
            elif kind=='provider-order':provider.contacts=provider.contacts[::-1]
            elif kind=='provider-origin':provider.origin=provider.origin+1e-8
            else:provider.stance_pose=provider.stance_pose.copy();provider.stance_pose[0,3]=1e-8
            with self.subTest(kind=kind),self.assertRaises(ValueError):h.actual_contact_binding(problem.fixture,provider,cone)
    def test_injected_certificate_wrong_candidate_or_context_rejected(self):
        problem=self.problem()
        for field in('candidateSHA256','contextSHA256','inputBindingSHA256','pathSHA256'):
            def wrong(*a,**kw):
                receipt=self.complete_certificate(*a,**kw);receipt['candidateCertificateBinding'][field]='changed';return receipt
            with self.subTest(field=field),patch.object(h,'restore_feasibility',side_effect=self.solver):
                path,receipt=problem.solve(certificate_function=wrong)
            self.assertIsNone(path);self.assertFalse(receipt['finiteNativeFlightFeasible'])
    def test_explicit_anchor_mode_recorded_without_silent_fallback(self):
        problem=self.problem()
        with patch.object(h,'restore_feasibility',side_effect=self.solver):
            _,receipt=problem.solve(certificate_function=self.complete_certificate,subproblem_reference='anchor')
        self.assertEqual(self.solver_kw['subproblem_reference'],'anchor');self.assertTrue(receipt['anchorObjectiveOptimizedDuringRestoration'])
    def test_invalid_certificate_cap_rejected_before_solver(self):
        problem=self.problem()
        with patch.object(h,'restore_feasibility')as solver:
            with self.assertRaises(ValueError):problem.solve(certificate_max_wall_seconds=181.)
        solver.assert_not_called()
    def test_failed_solver_does_not_spend_full_strict_native_certificate(self):
        problem=self.problem()
        def rejected(*a,**kw):return dict(self.solver(*a,**kw),numericallyFeasible=False)
        with patch.object(h,'restore_feasibility',side_effect=rejected),patch.object(h,'certify_finite_horizon')as cert:
            path,receipt=problem.solve(certificate_function=cert)
        cert.assert_not_called();self.assertIsNone(path);self.assertFalse(receipt['finiteNativeFlightFeasible'])
        self.assertTrue(receipt['strictFiniteCertificate']['notAttempted']);self.assertFalse(receipt['certificateFunction']['attempted'])
        self.assertFalse(receipt['certificateFunction']['originalDefaultCertificateUsed'])
        self.assertEqual(receipt['strictFiniteCertificate']['termination'],'not-attempted-solver-not-numerically-feasible')
    def test_injected_certificate_cannot_omit_original_static_reserve(self):
        problem=self.problem()
        def missing_static(*a,**kw):
            receipt=self.complete_certificate(*a,**kw)
            receipt['analyticSamples'][0]['additionalOriginalStaticLP']=None
            return receipt
        with patch.object(h,'restore_feasibility',side_effect=self.solver):
            path,receipt=problem.solve(certificate_function=missing_static)
        self.assertIsNone(path);self.assertFalse(receipt['finiteNativeFlightFeasible'])

if __name__=='__main__':unittest.main(verbosity=2)
