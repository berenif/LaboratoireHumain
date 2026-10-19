"""Fresh recovery protocol tests; certificates mocked, no native/optimizer calls."""
import copy,hashlib,json,tempfile,unittest,zlib
from pathlib import Path
from dataclasses import asdict,replace
from unittest.mock import patch
import numpy as np
import source_reviewed_wholebody as w
from source_reference_context import Jet,canonical
from coupled_posture_reference import CurrentSupport
from test_source_reviewed_wholebody import fixture_context

class FreshRecoveryPreflight(unittest.TestCase):
    def fixture(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);root=Path(tmp.name)
        knots=[[.2642308394146584,-.3228257277630144,-.07686273291796496,.19289327081458812,-.1857808996344065,.14543542838397905,-.245935876447375,-.17444672551954202,-.18387633701539655],
            [.09461317152599662,-.3686420785746894,-.07690995221452143,.2122177321920646,-.027528030762732157,.3514517583595922,-.1838179856399162,-.2251728849603546,-.17656584066548378],
            [.14547881696124276,-.36866257209534903,-.07687064814947847,.20525981727141518,-.07771521826228983,.2743265885294801,-.35698571673042917,-.3390658503988659,-.17727823443227217]]
        key=w.digest(knots);self.assertEqual(key,w.RECOVERED_ACCEPTED_COORDINATE_SHA256)
        coordinates=dict(independentKnotsRad=knots,candidateSHA256=key,expectedPreviouslyRecordedCandidateSHA256=key,
            exactCanonicalCoordinatesMatch=True,originalResultJSONBytesRecovered=False,originalCertificateRecovered=False,nativeAdmission=False)
        cp=root/'coordinates.json';cp.write_text(json.dumps(coordinates))
        context=fixture_context();config=json.loads(context.config_json)
        config.update(heading=[1.,0.,0.],phase_durations=dict(transfer=4.),leg_origin=[0.,0.])
        context=replace(context,joints=Jet.make(context.joints.position,[0.]*21,[0.]*21),config_json=canonical(config))
        points=((-2.,-1.),(2.,-1.),(0.,2.),(2.,2.))
        loaded=tuple((2.*float(np.cos(j*np.pi/3)),2.*float(np.sin(j*np.pi/3)))for j in range(6))
        support=CurrentSupport(3200,4.,1,'transfer',loaded,tuple((*p,0.)for p in points),(True,)*4,(0,1,2),True)
        binding=dict(schema=1,modelSHA256='a'*64,sourceXMLSHA256='b'*64,armMapSemanticSHA256='c'*64,
            originalSourceLimitsRad=[[-2.,2.]]*21,legOriginRightLeftRad=[0.,0.],stanceSide=1,swingSide=0)
        payload=b'fresh actual input; mock native bytes';pp=root/'native.zlib';pp.write_bytes(zlib.compress(payload))
        feet=[np.eye(4).tolist()]*2
        retained=dict(referenceContext=context.receipt(),currentSupport=asdict(support),currentFeetModel=feet,
            actualPayloadSHA256=hashlib.sha256(payload).hexdigest(),actualPayloadBytes=len(payload),
            retainedPayload=pp.name,compressedPayloadSHA256=w.sha(pp),proposal=dict(binding=binding,independentKnotsRad=knots))
        ip=root/'input.json';ip.write_text(json.dumps(retained))
        actual=dict(inputSHA256=w.sha(ip),contextSHA256=context.receipt()['contextSHA256'],currentSupportSHA256=support.key,
            actualFeetModelSHA256=w.digest(feet),actualPayloadSHA256=retained['actualPayloadSHA256'],compressedPayloadSHA256=w.sha(pp),
            sourceNativeStep=3200,sourceTimeS=4.,entryLiftElapsedS=0.,heading=[1.,0.,0.],
            support=dict(equations=support.equations().tolist()),**{k:binding[k]for k in('modelSHA256','sourceXMLSHA256','armMapSemanticSHA256')})
        problem=dict(schema=1,sourceCoordinateReceiptFilename=cp.name,sourceCoordinateReceiptSHA256=w.sha(cp),
            sourceCandidateSHA256=key,objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key,
            regeneratedActivationInputFilename=ip.name,regeneratedActivationInputSHA256=w.sha(ip),inputBindingSHA256=w.digest(actual),
            activationContextSHA256=actual['contextSHA256'],actualPayloadSHA256=actual['actualPayloadSHA256'],sourceNativeStep=3200,
            sourceTimeS=4.,entryLiftElapsedS=0.,original33AnchorAvailable=False,previousRawReceiptsRecovered=False,
            oldCertificatesReused=False,originalHardLimitsUnchanged=True,originalFootPathAndClocksUnchanged=True,
            goalOrWeightScan=False,producerOptimizerCalls=1,
            declaredAdditionalEnvelopeBrackets=[dict(phase='lift',bracketFractions=[.775,.8,.8250000000000001]),
                dict(phase='land',bracketFractions=[.7000000000000001,.7250000000000001,.75])])
        binding=dict(binding,freshRecoveryStageFiles=dict(coordinateReceipt=dict(filename=cp.name,SHA256=w.sha(cp)),
            activationInput=dict(filename=ip.name,SHA256=w.sha(ip)),payload=dict(filename=pp.name,SHA256=w.sha(pp))))
        family=dict(variables=27,footLiftM=.04,ownFootAdvanceM=.182,armAndMapReserveRad=.02,supportReserveM=.003,
            maximumCommandCOMSpeedMps=.19260552331163003,maximumCommandCOMAccelerationMps2=.3901818436690813,
            noSeedGoalWeightScan=True,strictOriginalMapInFinalPathAndRuntime=True,
            objective='minimum .5 Euclidean squared27-coordinate correction from exact recovered accepted coordinate vector')
        result=dict(version=w.FRESH_RECOVERY_VERSION,accepted=True,optimizerCalls=1,totalProducerOptimizerCalls=1,
            freshRecoveryProblem=problem,inputBinding=actual,independentKnotsRad=knots,
            objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key,initialContinuationIndependentKnotsRad=knots,
            analyticSamples=[dict(phase=p,fraction=f,accepted=True)for p in w.PHASES for f in w.ANALYTIC_FRACTIONS],
            maximumCorrectionFromFreshAnchorRad=0.,freshRecovery123SampleCertificate=dict(passed=True,originalStrictMapUsed=True,declaredSampleCount=123),
            finalOriginalStrictMapRestored=True,family=family,
            activeConstraintRowCounts=dict(original=884,declaredAdditionalEnvelopes=2,globalPhaseSupportEnvelopes=216,total=1102))
        result.update(nativeAdmission=False,physicalQualification=False,nativeWorldConstructions=0,physicsIntegrations=0,
            stateInstallations=0,exactWholeNonlinearPathAdmission=False,globalInfeasibilityProven=False,finiteProposalAdmittedForNative=False,
            finalCandidateSHA256=key,path=dict(independentKnotsRad=knots,initialContextSHA256=context.receipt()['contextSHA256'],
                independentIndices=[0,1,2,*w.LEG_INDICES[1]],savedFirstLiftIsOfflineDesignOnly=False,nativeAdmission=False,
                wholeNonlinearPathProof=False,physicsIntegrations=0,stateInstallations=0))
        return result,binding,root/'proposal.json',context,support,feet,payload

    def check(self,result,binding,path):
        with patch.object(w,'verify_finite_path_certificate')as finite,patch.object(w,'verify_global_phase_support_coverage')as coverage:
            w.verify_feasibility_stages(result,binding,path)
            finite.assert_called_once_with(result)
            coverage.assert_called_once_with(result,required_phases=w.PHASES,additional_envelope_role='declaredAdditionalEnvelopes')

    def test_new_exact_coordinate_receipt_is_not_old_evidence(self):
        result,binding,path,*_=self.fixture();before=copy.deepcopy(result)
        self.check(result,binding,path);self.assertEqual(result,before)

    def test_changed_problem_seed_proof_or_caps_fail_closed(self):
        mutations=[lambda r:r.update(twoStageFeasibilityRestoration={}),
            lambda r:r.update(totalProducerOptimizerCalls=2),
            lambda r:r.update(nativeAdmission=True),lambda r:r.update(physicsIntegrations=1),
            lambda r:r.update(finalCandidateSHA256='0'*64),
            lambda r:r['path'].update(independentKnotsRad=np.zeros((3,9)).tolist()),
            lambda r:r['path'].update(initialContextSHA256='0'*64),
            lambda r:r['freshRecoveryProblem'].update(original33AnchorAvailable=True),
            lambda r:r['freshRecoveryProblem'].update(oldCertificatesReused=True),
            lambda r:r['freshRecoveryProblem'].update(sourceCoordinateReceiptFilename='../coordinates.json'),
            lambda r:r['freshRecoveryProblem'].update(actualPayloadSHA256='0'*64),
            lambda r:r['inputBinding'].update(currentSupportSHA256='0'*64),
            lambda r:r['freshRecoveryProblem'].update(objectiveAnchorSHA256='0'*64),
            lambda r:r.update(initialContinuationIndependentKnotsRad=np.ones((3,9)).tolist()),
            lambda r:r['analyticSamples'].pop(),lambda r:r.update(finalOriginalStrictMapRestored=False),
            lambda r:r['family'].update(supportReserveM=.002),
            lambda r:r['freshRecoveryProblem']['declaredAdditionalEnvelopeBrackets'][0].update(bracketFractions=[.5,.6,.7])]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                result,binding,path,*_=self.fixture();mutate(result)
                with self.assertRaises(ValueError):self.check(result,binding,path)

    def test_rewritten_coordinate_receipt_cannot_choose_its_own_expected_hash(self):
        result,binding,path,*_=self.fixture();cp=path.parent/'coordinates.json'
        receipt=json.loads(cp.read_text());knots=np.zeros((3,9)).tolist();key=w.digest(knots)
        receipt.update(independentKnotsRad=knots,candidateSHA256=key,expectedPreviouslyRecordedCandidateSHA256=key)
        cp.write_text(json.dumps(receipt));problem=result['freshRecoveryProblem']
        problem.update(sourceCoordinateReceiptSHA256=w.sha(cp),sourceCandidateSHA256=key,objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key)
        result.update(objectiveAnchorSHA256=key,initialContinuationCandidateSHA256=key,initialContinuationIndependentKnotsRad=knots)
        binding['freshRecoveryStageFiles']['coordinateReceipt']['SHA256']=w.sha(cp)
        with self.assertRaisesRegex(ValueError,'numeric anchor'):self.check(result,binding,path)

    def test_stage_file_and_native_payload_corruption_are_rejected(self):
        result,binding,path,*_=self.fixture();binding['freshRecoveryStageFiles']['payload']['SHA256']='0'*64
        with self.assertRaises(ValueError):self.check(result,binding,path)
        result,binding,path,*_=self.fixture();(path.parent/'native.zlib').write_bytes(b'broken')
        with self.assertRaises(ValueError):self.check(result,binding,path)

    def test_new_runtime_input_retains_exact_native_and_geometry_checks(self):
        result,_,_,context,support,feet,payload=self.fixture()
        proposal=dict(freshRecoveryProblem=result['freshRecoveryProblem'],activationInputBinding=result['inputBinding'])
        w.verify_current_activation_identity(proposal,context,support,feet,payload)
        with self.assertRaises(ValueError):w.verify_current_activation_identity(proposal,context,support,feet,payload+b'changed')
        with self.assertRaises(ValueError):w.verify_current_activation_identity(proposal,replace(context,source_step=3204),support,feet,payload)

if __name__=='__main__':unittest.main(verbosity=2)
