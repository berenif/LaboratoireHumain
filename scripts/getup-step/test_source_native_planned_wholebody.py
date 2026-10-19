"""Pure mathematical receipts and mocked runtime hooks; no native execution.

The success certificate below is deliberately synthetic. It tests the evidence
parser and cannot establish real native flight feasibility or admit a pilot.
"""
import copy
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zlib

import numpy as np
from numpy.testing import assert_array_equal

import source_native_planned_wholebody as n
import source_reviewed_wholebody as old
from test_source_reviewed_wholebody import fixture_context, sample_for, controller
from source_reference_context import canonical, digest, ReferenceContext
from coupled_posture_reference import CurrentSupport
from coupled_posture_kinematics import MatrixJet, P
from native_wrench_constraints import ContactWrenchCone


class NativeProfileHooks(unittest.TestCase):
    def native_controller(self):
        base, state = controller()
        base.__class__ = n.NativePlannedController
        return base, state

    def test_default_policy_class_and_inherited_task_owners_remain_exact(self):
        c, _ = self.native_controller()
        self.assertEqual(c.wholebody_profile, n.FEATURE_PROFILE)
        self.assertEqual(c.flight_admission_receipt_field, 'finitePlannedJets')
        self.assertEqual(old.WholeBodyController.wholebody_profile, old.PROFILE)
        self.assertEqual(old.WholeBodyController.flight_admission_receipt_field, 'finiteStatic')
        for name in ('transform_reviewed_reference_context', 'solve', '_observe_support',
                'robust_support_ready', '_sample_current', 'corrected_com_reference',
                '_activate_settle', 'prepare_phase_handoff'):
            self.assertIs(getattr(n.NativePlannedController, name), getattr(old.WholeBodyController, name))
        self.assertIs(n.NativePlannedSteps.begin_step, old.WholeBodySteps.begin_step)
        self.assertIs(n.NativePlannedSteps.save, old.WholeBodySteps.save)

    def test_new_profile_changes_only_header_and_preserves_exact_arms_tasks(self):
        context = fixture_context(); sample = sample_for(context)
        baseline = old.owned_context(context, sample, {}, pelvis_weight=4.)
        changed = old.owned_context(context, sample, {}, pelvis_weight=4., profile=n.FEATURE_PROFILE)
        self.assertIs(changed.arms, context.arms)
        self.assertEqual(changed.arm_gains, context.arm_gains)
        a, b = baseline.receipt(), changed.receipt()
        self.assertEqual(b['diagnostics']['wholeBodyCycle'].pop('profile'), n.FEATURE_PROFILE)
        a['diagnostics']['wholeBodyCycle'].pop('profile')
        a.pop('contextSHA256'); b.pop('contextSHA256')
        self.assertEqual(a, b)

    def test_finite_policy_delegates_same_samples_current_points_and_fixed_origin(self):
        c, _ = self.native_controller(); samples, support = object(), object()
        feet = [np.eye(4), np.eye(4)]; feet[1][:3, 3] = [2., 3., 4.]
        contacts, answer = object(), object()
        with patch.object(old, 'actual_contacts', return_value=contacts) as actual, \
                patch.object(n, 'finite_planned_jet_admission', return_value=answer) as admit:
            self.assertIs(c._admit_flight_samples(samples, support, feet), answer)
        actual.assert_called_once_with(c, support)
        args = admit.call_args.args
        self.assertIs(args[0], c.w); self.assertIs(args[1], samples); self.assertIs(args[2], contacts)
        assert_array_equal(args[3], P@feet[1][:3, 3]); self.assertIs(args[4], feet[1])

    def test_physical_lumbar_cycle_hook_is_explicit_and_default_owner_preserved(self):
        c, _ = self.native_controller(); realizer, context = object(), object()
        c.w.limits, c.heading = object(), object()
        knots, marker = object(), object()
        with patch.object(old, 'WholeBodyCycle', return_value=marker) as construct:
            self.assertIs(c._construct_cycle(realizer, context, dict(independentKnotsRad=knots)), marker)
        construct.assert_called_once_with(realizer, context, c.w.limits, c.heading, knots,
            lumbar_window_deg=None)

    def test_pending_missing_and_wrong_profile_fail_before_base_activation(self):
        c, _ = self.native_controller()
        for value in (None, '', canonical({}), canonical(dict(featureProfile=old.PROFILE, nativePreflight=dict(passed=True))),
                canonical(dict(featureProfile='reviewed-native-planned-jets-wholebody-one-step-v1', nativePreflight=dict(passed=True)))):
            c._wholebody_proposal_json = value
            with patch.object(old.WholeBodyController, '_activate_wholebody', side_effect=AssertionError('must not activate')) as base:
                with self.assertRaises(ValueError): c._activate_wholebody(fixture_context())
                base.assert_not_called()

    def test_count_guard_precedes_any_base_constructor(self):
        for count in (0, 2, 10, True):
            with patch.object(old.WholeBodySteps, '__init__', side_effect=AssertionError('native/base construction')):
                with self.assertRaisesRegex(ValueError, 'count1'):
                    n.NativePlannedSteps(None, None, None, count, proposal_json=None)

    def test_runtime_freeze_extends_every_new_provider_wrench_profile_source(self):
        c, _ = self.native_controller()
        with patch.object(old.WholeBodySteps, '__init__', return_value=None):
            steps = n.NativePlannedSteps(None, None, None, proposal_json=None)
        self.assertTrue(set(old.RUNTIME) <= set(steps._wholebody_source_paths))
        for name in ('source_native_planned_wholebody.py', 'finite_planned_jet_admission.py',
                'native_planned_jet_provider.py', 'native_wrench_constraints.py', 'native_wrench_horizon.py'):
            self.assertEqual(steps._wholebody_code_start[name], old.sha(old.source_path(name)))
        with patch.object(old.WholeBodyController, '_native_source_bindings', return_value={'baseline': True}):
            bindings = c._native_source_bindings()
        self.assertTrue(bindings['baseline']); self.assertEqual(set(bindings['runtimeSHA256']), set(n.RUNTIME))


class SyntheticNativePreflight(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        evidence = old.ROOT/'outputs/reviewed-walk-recovery-20261019'
        for role in ('sourceDiagnosticReceipt', 'sourceLinearization'):
            name = n.DESIGN_REVISION[role+'Filename']
            (self.out/name).write_bytes((evidence/name).read_bytes())
        retained = old.ROOT/'outputs/reviewed-walk-recovery-20261019/held-native/generation-001/wholebody-activation-input.json'
        self.data = json.loads(retained.read_text())
        raw = b'EXPLICIT SYNTHETIC MATHEMATICAL RECEIPT; no native state'
        payload = self.out/'payload.zlib'; payload.write_bytes(zlib.compress(raw))
        self.data.update(actualPayloadSHA256=hashlib.sha256(raw).hexdigest(), actualPayloadBytes=len(raw),
            retainedPayload=payload.name, compressedPayloadSHA256=old.sha(payload))
        self.context = ReferenceContext.from_receipt(self.data['referenceContext'])
        self.support = CurrentSupport(**self.data['currentSupport'])
        self.feet = np.asarray(self.data['currentFeetModel']); config = json.loads(self.context.config_json)
        captured = self.data['proposal']['binding']; self.limits = np.asarray(captured['originalSourceLimitsRad'])
        self.stance, self.swing = captured['stanceSide'], captured['swingSide']
        self.input = self.out/'activation.json'; self.input.write_text(json.dumps(self.data))
        hashes = lambda names: {name: old.sha(old.source_path(name)) for name in names}
        self.bound = dict(schema=1, inputKind='retained-native-stationary-transfer-activation-v1',
            inputSHA256=old.sha(self.input), contextSHA256=self.context.receipt()['contextSHA256'],
            sourceNativeStep=self.context.source_step, sourceTimeS=self.context.source_time,
            referencePhase='transfer', referenceElapsedS=self.context.elapsed, entryLiftElapsedS=0.,
            actualPayloadSHA256=self.data['actualPayloadSHA256'], actualPayloadBytes=len(raw),
            compressedPayloadSHA256=old.sha(payload), payloadHashVerified=True, stateRestored=False,
            currentSupportSHA256=self.support.key, actualFeetModelSHA256=digest(self.feet.tolist()),
            currentFeetModelSHA256=digest(self.feet.tolist()), currentFeetModel=self.feet.tolist(),
            stancePoseModel=self.feet[self.stance].tolist(), heading=config['heading'],
            sourceProposalSHA256=self.data['proposal']['proposalSHA256'],
            frozenSourceHashes=hashes(('coupled_posture_kinematics.py', 'coupled_posture_reference.py',
                'phase_leg_reference.py', 'source_reference_context.py')),
            actualRetainedActivationInput=True, oldFirstLiftInputReused=False, nativeWorldConstructions=0, physicsIntegrations=0)
        self.bound.update({key: captured[key] for key in ('modelSHA256', 'sourceXMLSHA256', 'armMapSemanticSHA256')})
        self.bound['support'] = dict(sourceStep=self.context.source_step, sourceTimeS=self.context.source_time,
            phase='transfer', loadedPointsXY=[list(p) for p in self.support.loaded_points_xy],
            stancePointsXYZ=[list(p) for p in self.support.stance_points_xyz], presentCorners=list(self.support.present_corners),
            survivingCorners=list(self.support.surviving_corners), equations=self.support.equations().tolist(),
            originalSupportReserveM=.003, triangleFromActualLoadedPoints=True)
        indices = [i for i in range(15) if i not in n.LEG_INDICES[self.swing]]
        q0 = np.asarray(self.context.joints.position[:15]); self.knots = np.tile(q0[indices], (3, 1))
        anchor = digest(self.knots.tolist())
        patcher = patch.object(n, 'ANCHOR_SHA256', anchor); patcher.start(); self.addCleanup(patcher.stop)
        self.parent = self.out/'failed-parent.json'
        self.parent.write_text(json.dumps(dict(accepted=False, nativeAdmission=False, endpointFeasible=True,
            binding=dict(inputBinding=self.bound), unadmittedIndependentKnotsRad=self.knots.tolist(), finalCandidateSHA256=anchor)))
        points = np.asarray(self.support.stance_points_xyz)[list(self.support.surviving_corners)]
        origin = P@self.feet[self.stance][:3, 3]; cone = ContactWrenchCone(points, origin).receipt
        lo = self.limits[:15, 0]+.01; hi = self.limits[:15, 1]-.01
        hi[[6, 12]] = np.minimum(hi[[6, 12]], 0.)
        intervals = [[0., .25], [.25, .5], [.5, .75], [.75, 1.]]
        coverage = dict(schema=1, phases=list(n.PHASES), sourceSupportEquationsSHA256=digest(self.bound['support']['equations']),
            sourceSupportEquations=self.bound['support']['equations'], faceCount=9, pointKinds=['COM', 'capture'],
            originalSupportReserveM=.003, intervalsFractions=intervals, closedIntervalEndpointsCompared=True,
            wholePathProven=False, boundedScalarMethod='bounded', maximumIterationsPerEnvelope=64, fractionTolerance=1e-10)
        coverage['constraints'] = [dict(phase=phase, pointKind=kind, faceIndex=face, intervalFractions=interval, rowIndex=886+j)
            for j, (phase, kind, face, interval) in enumerate((phase, kind, face, interval)
                for phase in n.PHASES for kind in ('COM', 'capture') for face in range(9) for interval in intervals)]
        problem = dict(schema=1, version=n.VERSION, inputBinding=self.bound,
            initialCandidate=dict(filePath=self.parent.name, fileSHA256=old.sha(self.parent), candidateSHA256=anchor,
                role='historical-coordinate-seed-only', historicalCoordinateSeedOnly=True, admissionInherited=False,
                historicalNativePayloadEqualityClaimed=False, historicalInputBindingSHA256=digest(self.bound),
                currentInputBindingSHA256=digest(self.bound)),
            initialCandidateSHA256=anchor, initialIndependentKnotsRad=self.knots.tolist(), objectiveAnchorSHA256=anchor,
            objectiveAnchorIndependentKnotsRad=self.knots.tolist(), sourceJointLimitsRad=self.limits.tolist(),
            independentBoundsRad=np.tile(np.column_stack((lo[indices], hi[indices])), (3, 1)).tolist(),
            pathPhaseDurationS=[.8, 1.2, 1.], geometryConstraintRows=1102, nativeConstraintNodes=13,
            nativeConstraintRows=13*(42+len(cone['physicalFacetNormals'])+3), contactWrenchCone=cone,
            actualContactBinding=dict(stanceSide=self.stance, survivingCorners=list(self.support.surviving_corners),
                actualContactPointsSource=points.tolist(), actualContactPointBytesSHA256=hashlib.sha256(points.tobytes()).hexdigest(),
                fixedOriginSource=origin.tolist(), fixedStancePoseModel=self.feet[self.stance].tolist(), providerContactFieldsChecked=True),
            originalStrictMotorCap=.925, experimentalLumbarWindowDeg=None, experimentalLumbarWindowRemoved=True,
            experimentalLumbarIsPhysicalLimit=False,
            designRevision=copy.deepcopy(n.DESIGN_REVISION),
            noHyperextension=True, sourceJointInteriorRad=.01, armAndMapReserveRad=.02, originalPhysicalCOMHeightFloorM=.65,
            linearSubproblemPolicy='row-normalized-feasible-start',
            minimumCleanSelfGapM=.025, conservativeFriction=.6, normalForceDomain='nonnegative-unbounded',
            frozenSourceHashes=hashes(n.SOURCES), nativeWorldConstructions=0, physicsIntegrations=0, nativeAdmission=False,
            globalPhaseSupportEnvelopeCoverage=coverage, subproblemReference='current', anchorObjectiveOptimizedDuringRestoration=False)
        self.result = dict(schema=1, version=n.VERSION, binding=problem,
            domainAwareFeasibilitySolver=dict(version='genuine-domain-feasibility-sqp-v3-linear-feasible-start',
                terminalState='numerically-feasible-pending-independent-final-gates', numericallyFeasible=True,
                numericalFeasibilityTolerance=1e-10, coordinates=self.knots.reshape(27).tolist(), trajectorySolverInvocations=1,
                localQPOptimizerCalls=0, totalScipyMinimizeCalls=0, scalarSearchCalls=0,
                objectiveAnchor=self.knots.reshape(27).tolist(), objectiveOptimalityClaim=False,
                anchorObjectiveOptimizedDuringRestoration=False, solverException=None,
                projectionOrFabricatedConstraintValues=False, physicalAdmission=False, localQuadraticSubproblems=[],
                linearSubproblemPolicy='row-normalized-feasible-start', linearFeasibilityLPCalls=0,
                totalScipyLinprogCalls=0, linearFeasibilitySubproblems=[], localQPUnscaledFeasibilityTolerance=1e-10),
            trajectorySolverInvocations=1, localQPOptimizerCalls=0, totalScipyMinimizeCalls=0, optimizerCalls=0,
            linearSubproblemPolicy='row-normalized-feasible-start', linearFeasibilityLPCalls=0,
            totalScipyLinprogCalls=0, linearFeasibilitySubproblems=[],
            numericalFeasibilityOnly=True, objectiveOptimalityClaim=False, finalCandidateSHA256=anchor,
            subproblemReference='current', anchorObjectiveOptimizedDuringRestoration=False,
            unadmittedIndependentKnotsRad=self.knots.tolist(), certificateFunction=dict(module='native_wrench_horizon',
                name='certify_finite_horizon', originalDefaultCertificateUsed=True), finiteNativeFlightFeasible=True,
            accepted=False, nativeAdmission=False, wholePathProven=False, physicsIntegrations=0, nativeWorldConstructions=0,
            noSeedGoalWeightOrRadiusScan=True, live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True)
        self.result['strictFiniteCertificate'] = self.make_certificate(problem)
        self.proposal = self.out/'native.json'; self.annotation = self.out/'runtime.json'
        self.binding = dict(schema=1, profile=n.FEATURE_PROFILE, activationInputPath=self.input.name,
            activationInputSHA256=old.sha(self.input), frozenSourceHashes=hashes(n.RUNTIME))
        self.binding.update({key: captured[key] for key in n.ORIGINAL_BINDING_FIELDS})
        self.write()

    def make_certificate(self, problem):
        points = problem['actualContactBinding']['actualContactPointsSource']
        contacts = [dict(footSide=self.stance, pointSource=p) for p in points]
        primal = dict(success=True, accepted=True, originalStrictMotorCap=.925, normalForceUpperBound=None,
            friction=.6, nativeAdmission=False, motorCommands=[.1]*21, maximumMotorFraction=.1,
            forcesSource=[[0., 0., 10.]]*3, maximumNativeEquilibriumResidual=0., maximumFrictionViolation=0.)
        static = dict(success=True, requiredMotorFraction=.1, commands=[.1]*21, forcesN=[[0., 0., 10.]]*3,
            maximumEqualityResidual=0., maximumFrictionViolation=0., minimumNormalN=10., explicitContacts=contacts,
            originalMotorBounds=[-.95, .95], conservativeStaticFriction=.6, contactModelChanged=False)
        native = dict(version='counter-zero-native-planned-jets-v1', nativeCounter=0, rootAngularConvention='native torso-local',
            massTimesPlannedAccelerationPlusOriginalBias=True, biasIncludesOriginalPassive=True,
            fixedOriginSource=problem['actualContactBinding']['fixedOriginSource'], actualContacts=contacts,
            maximumStancePoseError=0., maximumStanceJetError=0., nativeCOMErrorM=0., nativeCOMVelocityError=0.,
            nativeCOMHeightM=.8, minimumCleanSelfGapM=.04, generalizedVelocity=[0.]*27, generalizedAcceleration=[0.]*27,
            physicsIntegrations=0, actualWorldStateWrites=0, nativeAdmission=False,
            sourceParityUsesGeometricJetsNotCompatibilityCommands=True, all21TasksExactlyCompatibleClaim=False,
            perPhysicalBodyParity=[dict(body=body, positionErrorM=0., rotationMatrixError=0., velocityError=0.) for body in n.PHYSICAL_BODIES])
        p = np.asarray(self.context.com.position); q = np.asarray(self.context.joints.position[:15])
        support = self.support.validate_com(old.Jet.rest(p)); margins = np.minimum(q-self.limits[:15, 0], self.limits[:15, 1]-q)
        def rows(nodes):
            return [dict(phase=n.PHASES[k], fraction=f, phaseTimeS=t, accepted=True,
                originalStrictMapEvaluationPassed=True, geometryAccepted=True, plannedNativeAccepted=True,
                originalPhysicalGuardsAccepted=True, additionalStaticAccepted=True, nativeAdmission=False,
                checks={key: True for key in ('lowerJointInterior', 'armReserve', 'mapReserve', 'lumbar',
                    'noHyperextension', 'speed', 'acceleration', 'clearance', 'supportCapture')},
                lowerJoints=q.tolist(), commandCOM=p.tolist(), commandCOMVelocity=[0.]*3, commandCOMAcceleration=[0.]*3,
                commandCOMHeightM=float(p[2]), commandSpeedMps=0., commandAccelerationMps2=0.,
                minimumLowerReserveRad=float(min(margins)), minimumArmReserveRad=.02,
                minimumMapSlackRad=0., minimumLumbarSlackRad=float(min(margins[:3]-.01)),
                plannedWholeFootClearanceM=.03, support=copy.deepcopy(support),
                supportCaptureMarginM=support['currentReferenceSupportMarginM'], newton=dict(maximumPoseResidual=0.),
                minimumCleanSelfGapM=.04, nativeCOMHeightM=.8, maximumMotorFraction=.1,
                plannedNativePrimalLP=copy.deepcopy(primal), native=copy.deepcopy(native),
                additionalOriginalStaticLP=copy.deepcopy(static) if (k == 0 and t == 0.) or t == (.8, 1.2, 1.)[k] else None)
                for k, t, f in nodes]
        path_key = digest(dict(version=n.CYCLE_VERSION, context=self.context.receipt()['contextSHA256'],
            knots=self.knots.tolist(), stance=MatrixJet.rest(self.feet[self.stance]).position,
            limits=self.limits.tolist(), entryElapsed=0., lumbarWindowDeg=None))
        original, dense = n.finite_command_nodes((.8, 1.2, 1.))
        return dict(schema=1, version=n.VERSION, passed=True, complete=True, termination=None,
            candidateCertificateBinding=dict(pathSHA256=path_key, candidateSHA256=digest(self.knots.tolist()),
                inputBindingSHA256=digest(self.bound), contextSHA256=self.context.receipt()['contextSHA256']),
            originalStrictMapUsed=True, original123Certificate=dict(passed=True, sampleCount=123, declaredSampleCount=123),
            denseCommandGridCertificate=dict(passed=True, sampleCount=603, declaredSampleCount=603, maximumStepS=.005,
                phaseDurationS=[.8, 1.2, 1.], originalStrictMapUsed=True, endpointInclusive=True),
            analyticSamples=rows(original), denseCommandSamples=rows(dense), failures=[], finiteSamplesOnly=True,
            wholePathProven=False, physicsIntegrations=0, nativeWorldConstructions=0, nativeAdmission=False,
            live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True)

    def write(self):
        self.proposal.write_text(json.dumps(self.result))
        self.binding['proposalSHA256'] = old.sha(self.proposal)
        self.annotation.write_text(json.dumps(self.binding))

    def preflight(self):
        with patch.object(old, 'prepare_impedance', side_effect=AssertionError('no preparation')), \
                patch.object(old, 'load_kinematics', side_effect=AssertionError('no kinematic/native construction')), \
                patch.object(n, 'finite_planned_jet_admission', side_effect=AssertionError('no native call')):
            return json.loads(n.frozen_native_proposal(self.proposal, self.annotation))

    def test_complete_synthetic_numeric_certificate_passes_pure_preflight(self):
        value = self.preflight()
        self.assertEqual(value['featureProfile'], n.FEATURE_PROFILE)
        self.assertTrue(value['nativePreflight']['candidateCertificateIndependentlyVerified'])
        self.assertEqual(value['independentKnotsRad'], self.knots.tolist())
        self.assertNotIn('freshRecoveryProblem', value)
        self.assertEqual(value['flightOptimizerCallsAtRuntime'], 0)

    def test_flags_missing_reordered_and_numeric_failures_reject(self):
        original = copy.deepcopy(self.result)
        changes = [lambda r: r.update(accepted=True), lambda r: r.update(nativeAdmission=True),
            lambda r: r.update(wholePathProven=True), lambda r: r.update(finiteNativeFlightFeasible=False),
            lambda r: r.update(finalPlanQualificationClaim=True), lambda r: r.update(requestedSteps=2),
            lambda r: r['strictFiniteCertificate'].pop('denseCommandSamples'),
            lambda r: r['strictFiniteCertificate']['analyticSamples'].reverse(),
            lambda r: r['strictFiniteCertificate']['denseCommandSamples'][37].update(commandSpeedMps=.9),
            lambda r: r['strictFiniteCertificate']['denseCommandSamples'][37]['plannedNativePrimalLP'].update(maximumMotorFraction=.925),
            lambda r: r['strictFiniteCertificate']['analyticSamples'][0].update(additionalOriginalStaticLP=None),
            lambda r: r['strictFiniteCertificate']['denseCommandSamples'][10]['native'].update(nativeCounter=4),
            lambda r: r['strictFiniteCertificate']['candidateCertificateBinding'].update(pathSHA256='0'*64),
            lambda r: r['certificateFunction'].update(originalDefaultCertificateUsed=False),
            lambda r: r['binding']['sourceJointLimitsRad'][0].__setitem__(0, -20.),
            lambda r: r['binding']['actualContactBinding'].update(stanceSide=1-self.stance),
            lambda r: r['binding']['frozenSourceHashes'].update(**{'native_wrench_horizon.py': '0'*64}),
            lambda r: r['binding']['globalPhaseSupportEnvelopeCoverage']['constraints'].pop(),
            lambda r: r['binding'].update(objectiveAnchorSHA256='0'*64),
            lambda r: r['binding'].update(experimentalLumbarWindowDeg=5.),
            lambda r: r['binding'].update(experimentalLumbarWindowRemoved=False),
            lambda r: r['binding'].pop('designRevision'),
            lambda r: r['domainAwareFeasibilitySolver'].update(version='genuine-domain-feasibility-sqp-v2-step-restoration'),
            lambda r: r['domainAwareFeasibilitySolver'].update(linearSubproblemPolicy='direct-slsqp'),
            lambda r: r.update(linearFeasibilityLPCalls=1),
            lambda r: r.update(version='native-wrench-whole-flight-feasibility-v1'),
            lambda r: r['unadmittedIndependentKnotsRad'][0].__setitem__(0, 20.)]
        for j, change in enumerate(changes):
            with self.subTest(mutation=j):
                self.result = copy.deepcopy(original); change(self.result); self.write()
                with self.assertRaises(ValueError): self.preflight()

    def test_input_payload_source_model_map_limits_and_annotation_cannot_change(self):
        original = copy.deepcopy(self.binding)
        for key, wrong in (('activationInputSHA256', '0'*64), ('modelSHA256', '0'*64),
                ('sourceXMLSHA256', '0'*64), ('armMapSemanticSHA256', '0'*64),
                ('stanceSide', 1-self.stance), ('legOriginRightLeftRad', [9., 9.])):
            with self.subTest(field=key):
                self.binding = copy.deepcopy(original); self.binding[key] = wrong; self.write()
                with self.assertRaises(ValueError): self.preflight()
        self.binding = original; self.write()
        (self.out/'payload.zlib').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'payload'): self.preflight()

    def test_initial_parent_bytes_and_qualification_preserved(self):
        self.parent.write_text('{}')
        with self.assertRaisesRegex(ValueError, 'initial candidate bytes'): self.preflight()

    def test_causal_design_diagnostic_and_linearization_bytes_required(self):
        path = self.out/n.DESIGN_REVISION['sourceLinearizationFilename']
        path.write_bytes(b'changed old linear diagnostic')
        with self.assertRaisesRegex(ValueError, 'causal sourceLinearization bytes'): self.preflight()

    def test_v3_lp_and_qp_counts_and_numeric_witnesses_remain_separate(self):
        coordinates = self.knots.reshape(27).tolist()
        count = self.result['binding']['geometryConstraintRows']+self.result['binding']['nativeConstraintRows']
        witness = dict(iteration=1, started=True, success=True, finiteWitness=True,
            maximumBoundViolation=0., currentPointCoordinates=coordinates, returnedCoordinates=coordinates,
            returnedStepCoordinates=[0.]*27, unscaledLinearizedSlacks=[1.]*count,
            minimumUnscaledLinearizedSlack=1., maximumUnscaledLinearizedViolation=0.,
            minimumNormalizedLinearizedSlack=1.)
        lp = dict(copy.deepcopy(witness), linprogReturned=True, status=0, method='highs-ds',
            normalizedPrimalFeasibilityTolerance=1e-9, normalizedDualFeasibilityTolerance=1e-9,
            initializationOnly=True, nonlinearCandidateOrSeed=False)
        qp = dict(copy.deepcopy(witness), minimizeReturned=True, linearSubproblemPolicy='row-normalized-feasible-start',
            localInitialCoordinates=coordinates)
        solver = self.result['domainAwareFeasibilitySolver']
        solver.update(linearFeasibilityLPCalls=1, totalScipyLinprogCalls=1, linearFeasibilitySubproblems=[lp],
            localQPOptimizerCalls=1, totalScipyMinimizeCalls=1, localQuadraticSubproblems=[qp])
        self.result.update(linearFeasibilityLPCalls=1, totalScipyLinprogCalls=1, linearFeasibilitySubproblems=[lp],
            localQPOptimizerCalls=1, totalScipyMinimizeCalls=1, optimizerCalls=1)
        self.write(); self.assertTrue(self.preflight()['nativePreflight']['passed'])
        qp['unscaledLinearizedSlacks'][0] = -1e-8
        qp.update(minimumUnscaledLinearizedSlack=-1e-8, maximumUnscaledLinearizedViolation=1e-8)
        self.write()
        with self.assertRaisesRegex(ValueError, 'numerical feasibility gate'): self.preflight()

    def test_above_five_degree_lumbar_sample_uses_only_original_physical_interior(self):
        row = self.result['strictFiniteCertificate']['denseCommandSamples'][37]
        q = np.asarray(row['lowerJoints']); q[0] += .12
        margins = np.minimum(q-self.limits[:15, 0], self.limits[:15, 1]-q)
        row.update(lowerJoints=q.tolist(), minimumLowerReserveRad=float(min(margins)),
            minimumLumbarSlackRad=float(min(margins[:3]-.01)))
        self.write()
        self.assertTrue(self.preflight()['nativePreflight']['passed'])
        q[0] = self.limits[0, 1]-.005
        margins = np.minimum(q-self.limits[:15, 0], self.limits[:15, 1]-q)
        row.update(lowerJoints=q.tolist(), minimumLowerReserveRad=float(min(margins)),
            minimumLumbarSlackRad=float(min(margins[:3]-.01)))
        self.write()
        with self.assertRaisesRegex(ValueError, 'original lower/lumbar'): self.preflight()

    def test_historical_seed_keeps_old_payload_without_inheriting_admission(self):
        parent = json.loads(self.parent.read_text())
        parent['binding']['inputBinding']['actualPayloadSHA256'] = '0'*64
        self.parent.write_text(json.dumps(parent))
        provenance = self.result['binding']['initialCandidate']
        provenance['fileSHA256'] = old.sha(self.parent)
        provenance['historicalInputBindingSHA256'] = digest(parent['binding']['inputBinding'])
        self.write()
        self.assertTrue(self.preflight()['nativePreflight']['passed'])
        provenance['historicalNativePayloadEqualityClaimed'] = True; self.write()
        with self.assertRaisesRegex(ValueError, 'historical numeric'): self.preflight()

    def test_legacy_preflight_rejects_native_profile(self):
        with patch.object(old, 'prepare_impedance', side_effect=AssertionError('no World')):
            with self.assertRaisesRegex(ValueError, 'offline whole-body proposal failed'):
                old.frozen_proposal(self.proposal, self.annotation)

    def test_exact_activation_identity_rejects_each_current_input_mutation(self):
        proposal = self.preflight(); payload = b'EXPLICIT SYNTHETIC MATHEMATICAL RECEIPT; no native state'
        n.verify_native_activation_identity(proposal, self.context, self.support, self.feet, payload)
        feet = self.feet.copy(); feet[0, 0, 3] += 1e-14
        changed_context = replace(self.context, source_step=self.context.source_step+1)
        changed_support = replace(self.support, source_step=self.support.source_step+1)
        for context, support, current_feet, current_payload in (
                (changed_context, self.support, self.feet, payload),
                (self.context, changed_support, self.feet, payload),
                (self.context, self.support, feet, payload),
                (self.context, self.support, self.feet, payload+b'x')):
            with self.assertRaisesRegex(ValueError, 'activation identity'):
                n.verify_native_activation_identity(proposal, context, support, current_feet, current_payload)
        proposal['independentKnotsRad'][0][0] += 1e-12
        with self.assertRaisesRegex(ValueError, 'changed after preflight'):
            n.verify_native_activation_identity(proposal, self.context, self.support, self.feet, payload)


if __name__ == '__main__':
    unittest.main(verbosity=2)
