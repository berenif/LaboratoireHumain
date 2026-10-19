"""Focused new all-phase/actual-input contracts. All optimizers are MOCKS."""
import copy,json,math,os,unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from wholebody_math_fixture import make_math_fixture
from coupled_posture_reference import CurrentSupport,digest
import wholebody_feasibility_polish as p
import test_wholebody_feasibility_polish as base_tests
stage_for=base_tests.stage_for

class ActivationRetargetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f=make_math_fixture();cls.knots=np.tile(np.asarray(cls.f.lower.position)[cls.f.realizer.independent_indices],(3,1))

    def setup_context(self):
        f=self.f;context=replace(f.context,phase='transfer',elapsed=4.)
        center=np.asarray(context.com.position)[:2]
        points=np.array([[2*math.cos(j*math.pi/3),2*math.sin(j*math.pi/3)]for j in range(6)]+[[0.,0.],[.1,.1]])+center
        support=CurrentSupport(context.source_step,context.source_time,context.step_index,'transfer',tuple(map(tuple,points)),
            f.support.stance_points_xyz,f.support.present_corners,f.support.surviving_corners,True)
        self.assertEqual(len(support.equations()),9)
        return context,support

    def refinement(self):return base_tests.RefinementTests.refinement(self)

    def test_all_three_phases_nine_faces_1102_rows_cache_and_fixed_peak_jacobian(self):
        context,support=self.setup_context();nodes,binding=self.refinement();f=self.f
        e=p.AnalyticPolishEvaluator(f.realizer,context,f.limits,f.heading,support,self.knots,
            constraint_nodes=nodes,envelope_brackets=binding['sourceBrackets'],global_support_phases=p.PHASES)
        receipt=e.phase_coverage_receipt();self.assertEqual(len(e.envelope_specs),218);self.assertEqual(len(receipt['constraints']),216)
        expected={(phase,kind,face,a,b)for phase in p.PHASES for kind in('COM','capture')for face in range(9)for a,b in zip(p.FRACTIONS,p.FRACTIONS[1:])}
        self.assertEqual({(r['phase'],r['pointKind'],r['faceIndex'],*r['intervalFractions'])for r in receipt['constraints']},expected)
        self.assertEqual({r['rowIndex']for r in receipt['constraints']},set(range(886,1102)))
        calls=[]
        def node(path,k,fraction):
            value=float(np.sum(path.independent_knots))+fraction;block=np.ones(len(e.labels))*value
            return block,dict(phase=p.PHASES[k],fraction=fraction,maximumPoseResidual=0.)
        def scalar(fun,*,bounds,method,options):
            t=sum(bounds)/2.;calls.append(bounds)
            return SimpleNamespace(x=t,fun=fun(t),success=True,nfev=1,nit=1)
        x=self.knots.reshape(27)
        with patch.object(e,'node',side_effect=node),patch.object(p,'minimize_scalar',side_effect=scalar):
            c,rows=e.evaluate(x);again,_=e.evaluate(x.copy());jac=e.constraint_jacobian(x)
        self.assertEqual(c.shape,(1102,));self.assertIs(c,again);self.assertEqual(jac.shape,(1102,27))
        self.assertEqual(len(calls),218);self.assertEqual(e.stats['fixedPeakJacobianColumns'],27)
        self.assertEqual(len(rows),231);np.testing.assert_allclose(jac,1.,atol=1e-8)
        for row in rows[13:]:self.assertTrue(row['closedIntervalEndpointsCompared'])

    def scenario(self):
        context,support=self.setup_context();nodes,historical=self.refinement();f=self.f
        initial=self.knots.copy();initial[0,0]+=1e-7;anchor_sha=digest(self.knots.tolist());initial_sha=digest(initial.tolist())
        actual=dict(schema=1,inputKind='explicit-mathematical-retained-input',inputSHA256='1'*64,contextSHA256=context.receipt()['contextSHA256'],
            currentSupportSHA256=support.key,actualFeetModelSHA256='2'*64,actualPayloadSHA256='3'*64,compressedPayloadSHA256='4'*64,
            sourceNativeStep=context.source_step,sourceTimeS=context.source_time,entryLiftElapsedS=0.,support=dict(equations=support.equations().tolist()))
        transition=dict(schema=1,sourceProposalFilename='old-source.json',sourceProposalSHA256='5'*64,
            sourceProposalBindingFilename='old-binding.json',sourceProposalBindingSHA256='6'*64,
            sourceProposalCandidateSHA256=initial_sha,sourceProposalProducerOptimizerCalls=4,retargetOptimizerCalls=1,
            retainedActivationInputFilename='actual-input.json',retainedActivationInputSHA256=actual['inputSHA256'],
            activationSourceStep=context.source_step,activationSourceTimeS=context.source_time,activationContextSHA256=actual['contextSHA256'],
            actualPayloadSHA256=actual['actualPayloadSHA256'],compressedPayloadSHA256=actual['compressedPayloadSHA256'],
            initialContinuationCandidateSHA256=initial_sha,objectiveAnchorSHA256=anchor_sha,
            originalHardLimitsUnchanged=True,referenceFamilyUnchanged=True,physicalStateInstalled=False)
        stage=p._stage_binding(stage_for(self.knots),self.knots.reshape(27));stage.update(maximumCorrectionRad=.01,
            finalOriginalStrictMapRestored=True,declared123SampleCertificate=True,certificateReceipt=dict(historicalInputOnly=True,passed=True))
        return context,support,nodes,historical,initial,actual,transition,stage

    def run_mock(self,context,support,nodes,historical,initial,actual,transition,stage,history=None,**sampler_options):
        f=self.f
        def harness(fun,x,**kwargs):
            np.testing.assert_array_equal(x,initial.reshape(27));self.assertEqual(fun(self.knots.reshape(27)),0.);self.assertGreater(fun(x),0.)
            return SimpleNamespace(x=x,success=False,nit=0,message='MOCK only; no numerical solve')
        with patch.object(p,'minimize',side_effect=harness),patch.object(p.WholeBodyCycle,'sample',side_effect=ValueError('mock independent rejection')):
            return p.polish_horizon(f.realizer,context,f.limits,f.heading,support,self.knots,stage_binding=stage,
                input_binding=actual,constraint_nodes=nodes,collocation_refinement=historical,initial_knots=initial,
                activation_retargeting=transition,exchange_history=history,global_support_phases=p.PHASES,
                max_wall_seconds=300.,certificate_max_wall_seconds=180.,**sampler_options)

    def test_explicit_new_input_keeps_historical_stage_and_original_anchor(self):
        context,support,nodes,historical,initial,actual,transition,stage=self.scenario()
        path,r=self.run_mock(context,support,nodes,historical,initial,actual,transition,stage)
        self.assertIsNone(path);self.assertEqual(r['version'],p.VERSION_REFINEMENT);self.assertEqual(r['totalProducerOptimizerCalls'],5)
        self.assertNotIn('exchangeHistory',r);self.assertEqual(r['inputBinding'],actual);self.assertEqual(r['activationRetargeting'],transition)
        self.assertEqual(r['initialContinuationCandidateSHA256'],digest(initial.tolist()));self.assertEqual(r['objectiveAnchorSHA256'],digest(self.knots.tolist()))
        expected=copy.deepcopy(stage);expected['maximumCorrectionRad']=float(max(abs(initial.reshape(27)-self.knots.reshape(27))))
        self.assertEqual(r['twoStageFeasibilityRestoration'],expected)
        self.assertFalse(r['activationRetargeting123SampleCertificate']['passed']);self.assertFalse(r['finalOriginalStrictMapRestored'])
        self.assertEqual(r['activeConstraintRowCounts'],dict(original=884,retainedLiftLandEnvelopes=2,globalPhaseSupportEnvelopes=216,total=1102))
        self.assertEqual(r['path']['phaseDurationsS'],(.8,1.2,1.));self.assertEqual(r['referenceLandingCenterDifferenceM'],0.)
        self.assertFalse(r['accepted']);self.assertFalse(r['nativeAdmission'])
        self.assertFalse(r['strictCertificateCacheUsed']);self.assertIsNone(r['strictCertificateCacheSourceName'])

    def test_explicit_original_complete_sample_hook_is_bound_and_rejection_retained(self):
        scenario=self.scenario();calls=[]
        def sample(path,phase,t):
            calls.append((phase,t));raise ValueError('explicit complete strict sampler rejection')
        sampler=SimpleNamespace(sample=sample);binding=dict(filename='strict_reference_cache.py',SHA256='a'*64)
        path,r=self.run_mock(*scenario,certificate_sampler=sampler,certificate_sampler_binding=binding)
        self.assertIsNone(path);self.assertTrue(r['strictCertificateCacheUsed'])
        self.assertEqual(r['strictCertificateCacheSourceName'],binding['filename']);self.assertEqual(r['strictCertificateCacheSHA256'],binding['SHA256'])
        self.assertGreaterEqual(len(calls),603);self.assertFalse(r['accepted']);self.assertFalse(r['finalOriginalStrictMapRestored'])
        self.assertEqual(len(r['analyticSamples']),123);self.assertEqual(len(r['denseCommandSamples']),603)
        self.assertTrue(all(x.get('error')=='explicit complete strict sampler rejection'for x in r['analyticFailures']))
        for invalid in({},dict(filename='../cache.py',SHA256='a'*64),dict(filename='cache.py',SHA256='bad')):
            with self.subTest(binding=invalid),self.assertRaises(ValueError):
                self.run_mock(*scenario,certificate_sampler=sampler,certificate_sampler_binding=invalid)

    def test_same_actual_input_compute_cap_continuation_preserves_transition_and_anchor(self):
        context,support,nodes,historical,seed,actual,transition,stage=self.scenario()
        initial=seed.copy();initial[1,0]+=2e-7;candidate=digest(initial.tolist())
        history=[dict(filename='retarget-parent.json',SHA256='7'*64,inputBindingSHA256=digest(actual),
            activationRetargetingSHA256=digest(transition),sourceCandidateSHA256=candidate,finalCandidateSHA256=candidate,initialContinuationCandidateSHA256=candidate,
            objectiveAnchorSHA256=digest(self.knots.tolist()),optimizerCalls=1,optimizerSuccess=False,
            terminalState='compute-cap',finalCandidateOrigin='last-completed-major-iteration',completedMajorIteration=2,hardLimitsUnchanged=True)]
        path,r=self.run_mock(context,support,nodes,historical,initial,actual,transition,stage,history)
        self.assertIsNone(path);self.assertEqual(r['totalProducerOptimizerCalls'],6);self.assertEqual(r['exchangeHistory'],history)
        self.assertEqual(r['activationRetargeting'],transition);self.assertEqual(r['inputBinding'],actual)
        self.assertEqual(r['initialContinuationIndependentKnotsRad'],initial.tolist());self.assertEqual(r['initialContinuationCandidateSHA256'],candidate)
        self.assertNotEqual(r['activationRetargeting']['initialContinuationCandidateSHA256'],candidate)
        self.assertEqual(r['objectiveAnchorSHA256'],digest(self.knots.tolist()));self.assertNotIn('globalTranslateSupportEnvelopeCoverage',r)
        self.assertEqual(r['activeConstraintRowCounts']['total'],1102)
        for key,value in [('inputBindingSHA256','8'*64),('activationRetargetingSHA256','9'*64),
                ('completedMajorIteration',0),('finalCandidateOrigin','unadmitted-last-requested'),('optimizerSuccess',None)]:
            invalid=copy.deepcopy(history);invalid[-1][key]=value
            with self.subTest(key=key),patch.object(p,'minimize')as minimize_mock:
                with self.assertRaises(ValueError):self.run_mock(context,support,nodes,historical,initial,actual,transition,stage,invalid)
                minimize_mock.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
