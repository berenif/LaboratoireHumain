"""Fresh producer contracts; synthetic inputs and MOCK optimizer only."""
import copy,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from coupled_posture_reference import digest
import wholebody_feasibility_polish as p
import test_wholebody_activation_retarget as existing

class FreshProducerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        existing.ActivationRetargetTests.setUpClass();cls.owner=existing.ActivationRetargetTests()
        cls.f=cls.owner.f;cls.anchor=cls.owner.knots.copy()

    def scenario(self):
        context,support=self.owner.setup_context();key=digest(self.anchor.tolist())
        binding=dict(inputSHA256='1'*64,contextSHA256=context.receipt()['contextSHA256'],actualPayloadSHA256='2'*64,
            currentSupportSHA256=support.key,entryLiftElapsedS=0.)
        fresh=dict(schema=1,sourceCoordinateReceiptFilename='coordinates.json',sourceCoordinateReceiptSHA256='3'*64,
            sourceCandidateSHA256=key,objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key,
            regeneratedActivationInputFilename='fresh-input.json',regeneratedActivationInputSHA256=binding['inputSHA256'],
            inputBindingSHA256=digest(binding),activationContextSHA256=binding['contextSHA256'],actualPayloadSHA256=binding['actualPayloadSHA256'],
            sourceNativeStep=context.source_step,sourceTimeS=context.source_time,entryLiftElapsedS=0.,
            original33AnchorAvailable=False,previousRawReceiptsRecovered=False,oldCertificatesReused=False,
            originalHardLimitsUnchanged=True,originalFootPathAndClocksUnchanged=True,goalOrWeightScan=False,producerOptimizerCalls=1,
            declaredAdditionalEnvelopeBrackets=[dict(phase='lift',bracketFractions=[.775,.8,.8250000000000001]),
                dict(phase='land',bracketFractions=[.7000000000000001,.7250000000000001,.75])])
        return context,support,binding,fresh

    def call(self,context,support,binding,fresh,**kwargs):
        def optimizer(fun,x,**options):
            np.testing.assert_array_equal(x,self.anchor.reshape(27));self.assertEqual(fun(x),0.)
            self.assertIn('jac',options['constraints'][0]);self.assertEqual(options['options']['maxiter'],80)
            return SimpleNamespace(x=x,success=False,nit=0,message='MOCK fresh problem only')
        with patch.object(p,'minimize',side_effect=optimizer),patch.object(p.WholeBodyCycle,'sample',side_effect=ValueError('synthetic strict rejection')):
            return p.polish_horizon(self.f.realizer,context,self.f.limits,self.f.heading,support,self.anchor,
                input_binding=binding,initial_knots=self.anchor,global_support_phases=p.PHASES,fresh_recovery_problem=fresh,
                max_wall_seconds=300.,certificate_max_wall_seconds=180.,**kwargs)

    def test_fresh_anchor_and_input_without_historical_stage(self):
        context,support,binding,fresh=self.scenario();path,r=self.call(context,support,binding,fresh)
        self.assertIsNone(path);self.assertEqual(r['version'],p.VERSION_FRESH_RECOVERY);self.assertEqual(r['freshRecoveryProblem'],fresh)
        self.assertEqual(r['totalProducerOptimizerCalls'],1);self.assertEqual(r['optimizerCalls'],1)
        for key in('twoStageFeasibilityRestoration','activationRetargeting','collocationRefinement','exchangeHistory'):self.assertNotIn(key,r)
        self.assertEqual(r['inputBinding'],binding);self.assertEqual(r['objectiveAnchorSHA256'],digest(self.anchor.tolist()))
        self.assertEqual(r['initialContinuationIndependentKnotsRad'],self.anchor.tolist())
        self.assertEqual(r['activeConstraintRowCounts'],dict(original=884,declaredAdditionalEnvelopes=2,globalPhaseSupportEnvelopes=216,total=1102))
        self.assertEqual(len(r['globalPhaseSupportEnvelopeCoverage']['constraints']),216)
        self.assertTrue(all('historicalSourceBracketFractions'not in s for s in r['activeEnvelopeConfiguration']))
        self.assertEqual(len(r['analyticSamples']),123);self.assertEqual(len(r['denseCommandSamples']),603)
        self.assertFalse(r['freshRecovery123SampleCertificate']['passed']);self.assertFalse(r['accepted']);self.assertFalse(r['nativeAdmission'])

    def test_changed_identity_and_forged_history_reject(self):
        scenario=self.scenario();context,support,binding,fresh=scenario
        for key,value in [('objectiveAnchorSHA256','0'*64),('oldCertificatesReused',True),('original33AnchorAvailable',True),
                ('inputBindingSHA256','4'*64),('declaredAdditionalEnvelopeBrackets',[])]:
            changed=copy.deepcopy(fresh);changed[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.call(context,support,binding,changed)
        with self.assertRaises(ValueError):self.call(*scenario,stage_binding=dict(invented=True))
        with self.assertRaises(ValueError):self.call(*scenario,exchange_history=[dict(invented=True)])
        changed=copy.deepcopy(fresh);changed['declaredAdditionalEnvelopeBrackets'][0]['bracketFractions'][0]=.77
        with patch.object(p,'minimize')as optimizer:
            with self.assertRaises(ValueError):
                p.polish_horizon(self.f.realizer,context,self.f.limits,self.f.heading,support,self.anchor,input_binding=binding,
                    initial_knots=self.anchor,global_support_phases=p.PHASES,fresh_recovery_problem=changed)
            optimizer.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
