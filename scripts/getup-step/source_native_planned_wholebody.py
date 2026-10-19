#!/usr/bin/env python3
"""Explicit, unqualified ONE-step source-physical-lumbar native-jet profile.

The read-only preflight consumes the original retained activation input and the
complete strict finite flight certificate. It constructs no World, runs no
optimizer and installs no state. Runtime task construction, arm ownership,
800Hz guards, fresh risk window and same-foot settle remain inherited.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import hashlib
import json
import math
from pathlib import Path
import zlib

import numpy as np

import source_reviewed_wholebody as original
from coupled_posture_kinematics import P, MatrixJet
from coupled_posture_reference import CurrentSupport, MAX_TOTAL_COM_SPEED, MAX_TOTAL_COM_ACCELERATION
from source_reference_context import ReferenceContext, canonical, digest
from finite_planned_jet_admission import finite_planned_jet_admission
from native_wrench_horizon import (VERSION as PRODUCER_VERSION, ANCHOR_SHA256,
    DESIGN_REVISION, SOURCES, finite_command_nodes)
from native_wrench_constraints import ContactWrenchCone
from wholebody_cycle_reference import VERSION as CYCLE_VERSION, PHASES, LEG_INDICES


FEATURE_PROFILE = 'reviewed-native-physical-lumbar-wholebody-one-step-v2'
VERSION = 'native-wrench-whole-flight-physical-lumbar-feasibility-v2'
RUNTIME = tuple(dict.fromkeys((*original.RUNTIME, *SOURCES,
    'finite_planned_jet_admission.py', 'native_planned_jet_provider.py',
    'native_wrench_constraints.py', 'source_native_planned_wholebody.py')))
ORIGINAL_BINDING_FIELDS = ('modelSHA256', 'sourceXMLSHA256', 'armMapSemanticSHA256',
    'originalSourceLimitsRad', 'legOriginRightLeftRad', 'stanceSide', 'swingSide')
PHYSICAL_BODIES = ('torso', 'head', 'lower_waist', 'pelvis', 'right_thigh', 'right_shin',
    'right_foot', 'left_thigh', 'left_shin', 'left_foot', 'right_upper_arm',
    'right_lower_arm', 'right_hand', 'left_upper_arm', 'left_lower_arm', 'left_hand')


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _array(value, shape, name):
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError('finite '+name+' required') from exc
    _require(result.shape == shape and np.isfinite(result).all(), 'finite '+name+' required')
    return result


def _number(value, name):
    _require(type(value) in (int, float) and math.isfinite(value), 'finite '+name+' required')
    return value


def _exact(value, expected, reason):
    _require(type(value) is type(expected) and value == expected, reason)


def _same(a, b, reason):
    _require(canonical(a) == canonical(b), reason)


def _sha(value, name):
    _require(isinstance(value, str) and len(value) == 64
        and all(c in '0123456789abcdef' for c in value), 'SHA256 required: '+name)
    return value


def _path(value, directory, name):
    _require(isinstance(value, str) and value, 'explicit '+name+' path required')
    path = Path(value)
    path = path if path.is_absolute() else directory/path
    _require(path.is_file(), 'retained '+name+' is absent')
    return path


def _hashes(sources, required, name):
    _require(isinstance(sources, dict) and set(required) <= set(sources), 'complete '+name+' source binding required')
    for filename, expected in sources.items():
        _require(isinstance(filename, str) and Path(filename).name == filename, 'source basename required')
        _require(original.sha(original.source_path(filename)) == _sha(expected, filename),
            name+' source differs: '+filename)


def _flags(value, expected, name):
    _require(isinstance(value, dict), name+' object required')
    for key, target in expected.items():
        _exact(value.get(key), target, name+' differs: '+key)


def _no_qualification(value):
    for key in ('finalPlanQualificationClaim', 'oldProfileQualificationClaim', 'visualQualificationClaim',
            'originalModelQualificationClaim', 'crossStepOwnershipReviewed', 'wholeNonlinearPathProven',
            'dynamicPathFeasibilityProven', 'native800HzQualified'):
        if key in value:
            _exact(value[key], False, 'contradictory native qualification claim: '+key)
    if 'requestedSteps' in value:
        _exact(value['requestedSteps'], 1, 'native planned profile count1 only')


def _read_input(annotation, directory, bound):
    path = _path(annotation.get('activationInputPath'), directory, 'activation input')
    _require(original.sha(path) == annotation.get('activationInputSHA256') == bound.get('inputSHA256'),
        'retained activation input bytes differ')
    data = json.loads(path.read_text())
    context = ReferenceContext.from_receipt(data['referenceContext'])
    support = CurrentSupport(**data['currentSupport'])
    support.validate_identity(context)
    support.validate_com(context.com)
    config = json.loads(context.config_json)
    causal = json.loads(context.causal_json)
    _require(context.phase == 'transfer' and context.elapsed >= config['phase_durations']['transfer']-1e-10
        and not any(context.com.velocity) and not any(context.com.acceleration)
        and not any(context.joints.velocity) and not any(context.joints.acceleration),
        'exact completed stationary transfer input required')
    correction = causal.get('robustCorrection', {})
    _require(not correction.get('selected') or correction.get('clock') == [1., 0., 0.],
        'captured original correction is incomplete')
    old = data['proposal']['binding']
    for key in ORIGINAL_BINDING_FIELDS:
        _same(annotation.get(key), old.get(key), 'runtime annotation replaced captured original '+key)
    limits = _array(old.get('originalSourceLimitsRad'), (21, 2), 'original source limits')
    _require(np.all(limits[:, 0] < limits[:, 1]), 'ordered original limits required')
    origins = _array(old.get('legOriginRightLeftRad'), (2,), 'original leg origins')
    stance, swing = old.get('stanceSide'), old.get('swingSide')
    _require(type(stance) is int and stance in (0, 1) and type(swing) is int and swing == 1-stance,
        'original stance/swing sides required')
    _same(config.get('stance_side'), stance, 'captured stance side differs')
    _same(config.get('swing_side'), swing, 'captured swing side differs')
    _same(config.get('leg_origin'), origins.tolist(), 'captured leg origins differ')
    feet = _array(data.get('currentFeetModel'), (2, 4, 4), 'actual feet model')
    for foot in feet:
        MatrixJet.rest(foot)
    heading = _array(config.get('heading'), (3,), 'original heading')
    _require(abs(np.linalg.norm(heading)-1.) <= 1e-8 and abs(heading[2]) <= 1e-12,
        'original horizontal unit heading required')
    payload_name = data.get('retainedPayload')
    _require(isinstance(payload_name, str) and Path(payload_name).name == payload_name, 'retained payload basename required')
    compressed = (path.parent/payload_name).read_bytes()
    _require(hashlib.sha256(compressed).hexdigest() == data.get('compressedPayloadSHA256'), 'compressed input payload differs')
    count = data.get('actualPayloadBytes')
    _require(type(count) is int and 0 < count <= 32*1024*1024, 'bounded input payload byte count required')
    decoder = zlib.decompressobj()
    try:
        payload = decoder.decompress(compressed, count+1)
    except zlib.error as exc:
        raise ValueError('invalid compressed input payload') from exc
    _require(len(payload) == count and decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail
        and hashlib.sha256(payload).hexdigest() == data.get('actualPayloadSHA256'), 'exact input payload bytes differ')
    _flags(bound, dict(schema=1, inputKind='retained-native-stationary-transfer-activation-v1',
        referencePhase='transfer', entryLiftElapsedS=0., payloadHashVerified=True,
        stateRestored=False, actualRetainedActivationInput=True, oldFirstLiftInputReused=False,
        nativeWorldConstructions=0, physicsIntegrations=0), 'native input binding')
    exact = dict(contextSHA256=context.receipt()['contextSHA256'], sourceNativeStep=context.source_step,
        sourceTimeS=context.source_time, referenceElapsedS=context.elapsed,
        actualPayloadSHA256=data['actualPayloadSHA256'], actualPayloadBytes=count,
        compressedPayloadSHA256=data['compressedPayloadSHA256'], currentSupportSHA256=support.key,
        actualFeetModelSHA256=digest(feet.tolist()), currentFeetModelSHA256=digest(feet.tolist()),
        currentFeetModel=feet.tolist(), stancePoseModel=feet[stance].tolist(), heading=heading.tolist(),
        modelSHA256=old['modelSHA256'], sourceXMLSHA256=old['sourceXMLSHA256'],
        armMapSemanticSHA256=old['armMapSemanticSHA256'], sourceProposalSHA256=data['proposal']['proposalSHA256'])
    for key, expected in exact.items():
        _same(bound.get(key), expected, 'native current input binding differs: '+key)
    expected_support = dict(sourceStep=context.source_step, sourceTimeS=context.source_time, phase=context.phase,
        loadedPointsXY=[list(p) for p in support.loaded_points_xy], stancePointsXYZ=[list(p) for p in support.stance_points_xyz],
        presentCorners=list(support.present_corners), survivingCorners=list(support.surviving_corners),
        equations=support.equations().tolist(), originalSupportReserveM=.003, triangleFromActualLoadedPoints=True)
    _same(bound.get('support'), expected_support, 'native actual support geometry differs')
    _require(support.require_triangle and len(support.equations()) == 9, 'original nine-face actual triangle support required')
    _require(original.sha(original.ROOT/'rust/crates/model/data/getup-v1.json') == old['modelSHA256'], 'original model JSON differs')
    _hashes(bound.get('frozenSourceHashes'), ('coupled_posture_kinematics.py', 'coupled_posture_reference.py',
        'phase_leg_reference.py', 'source_reference_context.py'), 'input')
    return data, context, support, feet, limits


def _lp(lp, *, stationary=False, contacts=None):
    _require(isinstance(lp, dict) and lp.get('success') is True, 'successful original LP required')
    motor_key = 'requiredMotorFraction' if stationary else 'maximumMotorFraction'
    eq_key = 'maximumEqualityResidual' if stationary else 'maximumNativeEquilibriumResidual'
    for key, cap in ((motor_key, .925), (eq_key, 1e-8), ('maximumFrictionViolation', 1e-8)):
        _require(0. <= _number(lp.get(key), key) < cap, 'original strict LP bound failed: '+key)
    motors = _array(lp.get('commands' if stationary else 'motorCommands'), (21,), 'LP motor witness')
    forces = _array(lp.get('forcesN' if stationary else 'forcesSource'), (3, 3), 'LP contact force witness')
    _require(np.max(abs(motors)) <= lp[motor_key]+1e-12 and np.max(abs(motors)) < .925
        and np.min(forces[:, 2]) >= 0., 'LP motor/normal witness differs')
    friction = float(max(0., np.max(abs(forces[:, :2])-.6*forces[:, 2, None])))
    _require(abs(friction-lp['maximumFrictionViolation']) <= 1e-12, 'LP friction witness differs')
    if stationary:
        _flags(lp, dict(originalMotorBounds=[-.95, .95], conservativeStaticFriction=.6, contactModelChanged=False), 'original static LP')
        _same(lp.get('explicitContacts'), contacts, 'original static LP actual contacts differ')
        _require(abs(_number(lp.get('minimumNormalN'), 'minimum normal')-min(forces[:, 2])) <= 1e-12,
            'static LP normal witness differs')
    else:
        _flags(lp, dict(accepted=True, originalStrictMotorCap=.925, normalForceUpperBound=None,
            friction=.6, nativeAdmission=False), 'planned native LP')
        _require(abs(float(np.max(abs(motors)))-lp[motor_key]) <= 1e-12, 'native motor maximum differs')


def _certificate_row(row, node, context, support, limits, contacts, origin):
    k, t, fraction = node
    _flags(row, dict(phase=PHASES[k], fraction=fraction, phaseTimeS=t, accepted=True,
        originalStrictMapEvaluationPassed=True, geometryAccepted=True, plannedNativeAccepted=True,
        originalPhysicalGuardsAccepted=True, additionalStaticAccepted=True, nativeAdmission=False), 'strict finite sample')
    _require(not any(key in row for key in ('error', 'errorType', 'domainReceipt')), 'certificate retains a failed sample')
    checks = row.get('checks')
    _require(isinstance(checks, dict) and set(checks) == {'lowerJointInterior', 'armReserve', 'mapReserve',
        'lumbar', 'noHyperextension', 'speed', 'acceleration', 'clearance', 'supportCapture'}
        and all(value is True for value in checks.values()), 'complete original geometry checks required')
    p = _array(row.get('commandCOM'), (3,), 'command COM')
    v = _array(row.get('commandCOMVelocity'), (3,), 'command COM velocity')
    a = _array(row.get('commandCOMAcceleration'), (3,), 'command COM acceleration')
    _require(p[2] >= .65 and row.get('commandCOMHeightM') == float(p[2]), 'command COM height guard differs')
    for key, actual, cap in (('commandSpeedMps', np.linalg.norm(v), MAX_TOTAL_COM_SPEED),
            ('commandAccelerationMps2', np.linalg.norm(a), MAX_TOTAL_COM_ACCELERATION)):
        _require(abs(_number(row.get(key), key)-actual) <= 1e-12 and actual <= cap+1e-10,
            'original COM jet bound differs')
    q = _array(row.get('lowerJoints'), (15,), 'lower joint sample')
    reserves = np.minimum(q-limits[:15, 0], limits[:15, 1]-q)
    # This separate v2 profile removes only the experimental +/-5deg shape
    # window. The original source joint limits plus .01rad interior still own
    # lumbar feasibility, exactly as they own every other lower coordinate.
    lumbar = reserves[:3]-.01
    _require(min(reserves) >= .01-1e-9 and max(q[[6, 12]]) <= 1e-10
        and abs(_number(row.get('minimumLowerReserveRad'), 'lower reserve')-min(reserves)) <= 1e-12
        and abs(_number(row.get('minimumLumbarSlackRad'), 'lumbar slack')-min(lumbar)) <= 1e-12
        and min(lumbar) >= -1e-9, 'original lower/lumbar/no-hyperextension guards differ')
    _require(_number(row.get('minimumArmReserveRad'), 'arm reserve') >= .02-1e-9
        and _number(row.get('minimumMapSlackRad'), 'map reserve') >= -1e-9,
        'original arm/map reserve failed')
    clearance = _number(row.get('plannedWholeFootClearanceM'), 'planned foot clearance')
    _require(k != 1 or clearance >= .025-1e-9, 'original translate foot clearance failed')
    supported = support.validate_com(original.Jet(p, v, a))
    _same(row.get('support'), supported, 'strict original COM support receipt differs')
    _require(abs(_number(row.get('supportCaptureMarginM'), 'support margin')
        -supported['currentReferenceSupportMarginM']) <= 1e-12, 'support/capture numeric margin differs')
    newton = row.get('newton', {})
    _require(0. <= _number(newton.get('maximumPoseResidual'), 'original fixed Newton residual') < 2e-11,
        'original strict fixed Newton map failed')
    _require(_number(row.get('minimumCleanSelfGapM'), 'clean self gap') > .025
        and _number(row.get('nativeCOMHeightM'), 'native COM height') >= .65, 'native physical geometry guard failed')
    _lp(row.get('plannedNativePrimalLP'))
    _require(row.get('maximumMotorFraction') == row['plannedNativePrimalLP']['maximumMotorFraction'],
        'row native motor maximum differs')
    native = row.get('native', {})
    _flags(native, dict(version='counter-zero-native-planned-jets-v1', nativeCounter=0,
        rootAngularConvention='native torso-local', massTimesPlannedAccelerationPlusOriginalBias=True,
        biasIncludesOriginalPassive=True, physicsIntegrations=0, actualWorldStateWrites=0, nativeAdmission=False,
        sourceParityUsesGeometricJetsNotCompatibilityCommands=True, all21TasksExactlyCompatibleClaim=False), 'native planned sample')
    _same(native.get('actualContacts'), contacts, 'native provider actual contacts differ')
    _same(native.get('fixedOriginSource'), origin, 'native provider fixed origin differs')
    for key in ('maximumStancePoseError', 'maximumStanceJetError', 'nativeCOMErrorM', 'nativeCOMVelocityError'):
        _require(0. <= _number(native.get(key), key) <= 1e-8, 'native/source parity failed: '+key)
    velocity = _array(native.get('generalizedVelocity'), (27,), 'native planned generalized velocity')
    acceleration = _array(native.get('generalizedAcceleration'), (27,), 'native planned generalized acceleration')
    parity = native.get('perPhysicalBodyParity', [])
    _require(len(parity) == 16 and {item.get('body') for item in parity} == set(PHYSICAL_BODIES),
        'all16 original physical body parity rows required')
    for item in parity:
        for key in ('positionErrorM', 'rotationMatrixError', 'velocityError'):
            _require(0. <= _number(item.get(key), key) <= 1e-8, 'original physical body parity failed')
    _require(native.get('nativeCOMHeightM') == row['nativeCOMHeightM']
        and native.get('minimumCleanSelfGapM') == row['minimumCleanSelfGapM'], 'native height/gap receipt differs')
    if (k == 0 and t == 0.) or t == (.8, 1.2, 1.)[k]:
        _lp(row.get('additionalOriginalStaticLP'), stationary=True, contacts=contacts)
        if t == (.8, 1.2, 1.)[k]:
            _require(max(np.max(abs(velocity)), np.max(abs(acceleration))) <= 1e-8,
                'phase endpoint full generalized jets must remain stationary')
    else:
        _require(row.get('additionalOriginalStaticLP') is None, 'unexpected substituted static reserve')


def verify_native_finite_certificate(certificate, problem, knots, context, support, feet, limits):
    """Check retained numeric evidence; never execute its producer or any LP."""
    _flags(certificate, dict(schema=1, version=VERSION, passed=True, complete=True, termination=None,
        originalStrictMapUsed=True, failures=[], finiteSamplesOnly=True, wholePathProven=False,
        physicsIntegrations=0, nativeWorldConstructions=0, nativeAdmission=False,
        live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True), 'strict finite certificate')
    stance = problem['actualContactBinding']['stanceSide']
    path_key = digest(dict(version=CYCLE_VERSION, context=context.receipt()['contextSHA256'],
        knots=knots.tolist(), stance=MatrixJet.rest(feet[stance]).position,
        limits=limits.tolist(), entryElapsed=0., lumbarWindowDeg=None))
    _same(certificate.get('candidateCertificateBinding'), dict(pathSHA256=path_key,
        candidateSHA256=digest(knots.tolist()), inputBindingSHA256=digest(problem['inputBinding']),
        contextSHA256=context.receipt()['contextSHA256']), 'certificate path/candidate/current-input binding differs')
    _flags(certificate.get('original123Certificate'), dict(passed=True, sampleCount=123, declaredSampleCount=123), 'original123 certificate')
    _flags(certificate.get('denseCommandGridCertificate'), dict(passed=True, sampleCount=603, declaredSampleCount=603,
        maximumStepS=.005, phaseDurationS=[.8, 1.2, 1.], originalStrictMapUsed=True, endpointInclusive=True), 'strict200Hz certificate')
    points = [list(support.stance_points_xyz[i]) for i in support.surviving_corners]
    contacts = [dict(footSide=stance, pointSource=p) for p in points]
    origin = (P@feet[stance][:3, 3]).tolist()
    for key, nodes in zip(('analyticSamples', 'denseCommandSamples'), finite_command_nodes((.8, 1.2, 1.))):
        rows = certificate.get(key)
        _require(isinstance(rows, list) and len(rows) == len(nodes), 'complete ordered '+key+' required')
        for row, node in zip(rows, nodes):
            _certificate_row(row, node, context, support, limits, contacts, origin)
    return path_key


def _support_coverage(problem, support):
    coverage = problem.get('globalPhaseSupportEnvelopeCoverage', {})
    equations = support.equations().tolist()
    intervals = [[0., .25], [.25, .5], [.5, .75], [.75, 1.]]
    _flags(coverage, dict(schema=1, phases=list(PHASES), sourceSupportEquationsSHA256=digest(equations),
        sourceSupportEquations=equations, faceCount=9, pointKinds=['COM', 'capture'],
        originalSupportReserveM=.003, intervalsFractions=intervals,
        closedIntervalEndpointsCompared=True, wholePathProven=False, boundedScalarMethod='bounded',
        maximumIterationsPerEnvelope=64, fractionTolerance=1e-10), 'all-phase support coverage')
    expected = [dict(phase=phase, pointKind=kind, faceIndex=face,
        intervalFractions=interval, rowIndex=886+index)
        for index, (phase, kind, face, interval) in enumerate(
            (phase, kind, face, interval) for phase in PHASES for kind in ('COM', 'capture')
            for face in range(9) for interval in intervals)]
    _same(coverage.get('constraints'), expected, 'all original faces/intervals/row counts must be retained')


def _design_revision(problem, directory):
    revision = problem.get('designRevision')
    _same(revision, DESIGN_REVISION, 'explicit causal physical-lumbar design revision required')
    files = {}
    for role in ('sourceDiagnosticReceipt', 'sourceLinearization'):
        name = revision[role+'Filename']
        _require(Path(name).name == name, 'causal diagnostic basename required')
        path = _path(name, directory, 'causal '+role)
        _require(original.sha(path) == revision[role+'SHA256'], 'retained causal '+role+' bytes differ')
        files[role] = path
    diagnostic = json.loads(files['sourceDiagnosticReceipt'].read_text())
    _flags(diagnostic, dict(schema=1, kind='exact-held002-native-linear-feasibility-diagnosis',
        failure=None, sourceHashesUnchanged=True, baselineEvaluations=1, fullJacobianEvaluations=1,
        nativeCounter=0, mathematicalLinprogCalls=3, trajectorySolverInvocations=0,
        localQPOptimizerCalls=0, nonlinearTrajectoryOptimizerCalls=0, physicsIntegrations=0,
        actualWorldStateWrites=0, nativeAdmission=False, physicalQualification=False,
        nonlinearFeasibilityProven=False, noGoalSeedWeightOrRadiusScan=True,
        finalCertificateOrPilotAttempted=False, physicalLumbarAlternativeLPReportsSuccess=True),
        'retained causal design diagnostic')
    _require(diagnostic.get('linearization', {}).get('SHA256') == revision['sourceLinearizationSHA256'],
        'causal diagnostic linearization hash differs')
    lps = diagnostic.get('LPs', [])
    _require(len(lps) == 3 and lps[0].get('status') == 2 and lps[0].get('success') is False
        and lps[2].get('status') == 0 and lps[2].get('success') is True,
        'causal original-window/physical-lumbar LP evidence differs')
    # This historical linear diagnostic authorizes a new problem design. Its
    # numerical LP witness is never substituted for a new nonlinear certificate.


def frozen_native_proposal(proposal_path, binding_path=None):
    """Fail-closed PURE preflight for this explicit profile, never legacy repair."""
    _require(PRODUCER_VERSION == VERSION, 'physical-lumbar v2 producer source required')
    path = Path(proposal_path)
    _require(path.is_file(), 'native flight proposal is absent')
    result = json.loads(path.read_text())
    _no_qualification(result)
    _flags(result, dict(schema=1, version=VERSION, finiteNativeFlightFeasible=True, accepted=False,
        nativeAdmission=False, wholePathProven=False, numericalFeasibilityOnly=True, objectiveOptimalityClaim=False,
        physicsIntegrations=0, nativeWorldConstructions=0, noSeedGoalWeightOrRadiusScan=True,
        live800HzGuardsFresh21WindowAndIndependentAuditStillRequired=True), 'native horizon receipt')
    _require(not any(key in result for key in ('freshRecoveryProblem', 'activationRetargeting', 'runtimeProposalBinding')),
        'legacy recovery/qualification cannot substitute for native profile')
    _require(binding_path is not None, 'separate explicit native runtime annotation required')
    binding_file = Path(binding_path)
    annotation = json.loads(binding_file.read_text())
    _no_qualification(annotation)
    _flags(annotation, dict(schema=1, profile=FEATURE_PROFILE, proposalSHA256=original.sha(path)), 'native runtime annotation')
    problem = result.get('binding', {})
    _no_qualification(problem)
    _flags(problem, dict(schema=1, version=VERSION, objectiveAnchorSHA256=ANCHOR_SHA256,
        pathPhaseDurationS=[.8, 1.2, 1.], geometryConstraintRows=1102, nativeConstraintNodes=13,
        originalStrictMotorCap=.925, experimentalLumbarWindowDeg=None,
        experimentalLumbarWindowRemoved=True, experimentalLumbarIsPhysicalLimit=False,
        noHyperextension=True, sourceJointInteriorRad=.01, armAndMapReserveRad=.02,
        originalPhysicalCOMHeightFloorM=.65, minimumCleanSelfGapM=.025, conservativeFriction=.6,
        linearSubproblemPolicy='row-normalized-feasible-start',
        normalForceDomain='nonnegative-unbounded', nativeWorldConstructions=0, physicsIntegrations=0, nativeAdmission=False), 'native horizon problem')
    _design_revision(problem, path.parent)
    anchor = _array(problem.get('objectiveAnchorIndependentKnotsRad'), (3, 9), 'unchanged objective anchor')
    _require(digest(anchor.tolist()) == ANCHOR_SHA256, 'unchanged095918 anchor coordinates required')
    knots = _array(result.get('unadmittedIndependentKnotsRad'), (3, 9), 'one immutable3x9 native candidate')
    _require(result.get('finalCandidateSHA256') == digest(knots.tolist()), 'native final candidate hash differs')
    _hashes(problem.get('frozenSourceHashes'), SOURCES, 'producer')
    _hashes(annotation.get('frozenSourceHashes'), RUNTIME, 'runtime')
    data, context, support, feet, limits = _read_input(annotation, binding_file.parent, problem.get('inputBinding', {}))
    _same(problem.get('sourceJointLimitsRad'), limits.tolist(), 'native source limits differ from exact captured input')
    _support_coverage(problem, support)
    stance = annotation['stanceSide']
    points = np.asarray(support.stance_points_xyz, dtype=np.float64)[list(support.surviving_corners)]
    contact = problem.get('actualContactBinding', {})
    _flags(contact, dict(stanceSide=stance, survivingCorners=list(support.surviving_corners),
        actualContactPointsSource=points.tolist(), actualContactPointBytesSHA256=hashlib.sha256(points.tobytes()).hexdigest(),
        fixedOriginSource=(P@feet[stance][:3, 3]).tolist(), fixedStancePoseModel=feet[stance].tolist(),
        providerContactFieldsChecked=True), 'native actual contact binding')
    cone = problem.get('contactWrenchCone', {})
    _flags(cone, dict(schema=1, version='fixed-stance-native-wrench-elimination-v1', pointsSource=points.tolist(),
        originSource=contact['fixedOriginSource'], actualPointBytesSHA256=contact['actualContactPointBytesSHA256'],
        pointSemanticSHA256=digest(points.tolist()), friction=.6, normalForceDomain='nonnegative-unbounded',
        candidateSubsets=792, QJ=False, flattenedContacts=False, physicalInequalityToleranceRelaxation=0.,
        classificationTolerance=1e-11, explicitTotalNormalSign=True), 'original actual contact cone')
    # SVD/enumeration only. This neither calls an LP nor evaluates native state.
    _same(cone, ContactWrenchCone(points, np.asarray(contact['fixedOriginSource'])).receipt,
        'retained actual contact cone numeric construction differs')
    facets = cone.get('physicalFacetNormals', [])
    _require(type(cone.get('facetCount')) is int and cone['facetCount'] > 0
        and len(facets) == cone['facetCount']+1, 'complete original cone facets required')
    _array(facets, (len(facets), 6), 'contact cone facets')
    _exact(problem.get('nativeConstraintRows'), 13*(42+len(facets)+3), 'native constraint row count differs')
    q0 = np.asarray(context.joints.position[:15]); lo = limits[:15, 0]+.01; hi = limits[:15, 1]-.01
    hi[[6, 12]] = np.minimum(hi[[6, 12]], 0.)
    indices = [i for i in range(15) if i not in LEG_INDICES[annotation['swingSide']]]
    bounds = np.tile(np.column_stack((lo[indices], hi[indices])), (3, 1))
    _same(problem.get('independentBoundsRad'), bounds.tolist(), 'native independent original bounds differ')
    _require(np.all(knots.reshape(27) >= bounds[:, 0]) and np.all(knots.reshape(27) <= bounds[:, 1]),
        'native candidate violates original independent bounds')
    initial = _array(problem.get('initialIndependentKnotsRad'), (3, 9), 'sole initial candidate')
    provenance = problem.get('initialCandidate', {})
    _require(isinstance(provenance.get('filePath'), str)
        and Path(provenance['filePath']).name == provenance['filePath'], 'retained initial candidate basename required')
    initial_path = _path(provenance.get('filePath'), path.parent, 'sole initial candidate')
    _require(original.sha(initial_path) == provenance.get('fileSHA256'), 'retained sole initial candidate bytes differ')
    parent = json.loads(initial_path.read_text())
    _require(parent.get('accepted') is False and parent.get('nativeAdmission') is False,
        'sole initial parent qualification was relabeled')
    _same(parent.get('unadmittedIndependentKnotsRad'), initial.tolist(), 'sole initial candidate differs from retained parent')
    _require(isinstance(parent.get('binding', {}).get('inputBinding'), dict), 'historical initial input binding must be retained')
    _require(parent.get('finalCandidateSHA256') == provenance.get('candidateSHA256')
        == problem.get('initialCandidateSHA256') == digest(initial.tolist()), 'sole initial candidate provenance hash differs')
    _flags(provenance, dict(role='historical-coordinate-seed-only', historicalCoordinateSeedOnly=True,
        admissionInherited=False, historicalNativePayloadEqualityClaimed=False,
        historicalInputBindingSHA256=digest(parent.get('binding', {}).get('inputBinding')),
        currentInputBindingSHA256=digest(problem['inputBinding'])), 'historical numeric initial-candidate provenance')
    solver = result.get('domainAwareFeasibilitySolver', {})
    _flags(solver, dict(version='genuine-domain-feasibility-sqp-v3-linear-feasible-start', numericallyFeasible=True,
        numericalFeasibilityTolerance=1e-10, trajectorySolverInvocations=1, scalarSearchCalls=0,
        linearSubproblemPolicy='row-normalized-feasible-start',
        localQPUnscaledFeasibilityTolerance=1e-10,
        objectiveOptimalityClaim=False, projectionOrFabricatedConstraintValues=False, physicalAdmission=False,
        solverException=None, terminalState='numerically-feasible-pending-independent-final-gates'), 'domain-aware feasibility solver')
    _same(solver.get('coordinates'), knots.reshape(27).tolist(), 'solver final native candidate differs')
    _same(solver.get('objectiveAnchor'), anchor.reshape(27).tolist(), 'solver unchanged objective anchor differs')
    qps = solver.get('localQuadraticSubproblems')
    _require(isinstance(qps, list), 'local QP accounting required')
    for key in ('localQPOptimizerCalls', 'totalScipyMinimizeCalls'):
        _exact(solver.get(key), len(qps), 'solver local QP count differs')
        _exact(result.get(key), len(qps), 'receipt local QP count differs')
    _exact(result.get('optimizerCalls'), len(qps), 'receipt optimizer count differs')
    _exact(result.get('trajectorySolverInvocations'), 1, 'one native trajectory restoration required')
    _exact(result.get('linearSubproblemPolicy'), 'row-normalized-feasible-start', 'native numerical policy differs')
    lps = solver.get('linearFeasibilitySubproblems')
    _require(isinstance(lps, list), 'separate linear feasibility LP accounting required')
    _same(result.get('linearFeasibilitySubproblems'), lps, 'retained LP records differ from solver')
    for key in ('linearFeasibilityLPCalls', 'totalScipyLinprogCalls'):
        _exact(solver.get(key), len(lps), 'solver linear feasibility LP count differs')
        _exact(result.get(key), len(lps), 'receipt linear feasibility LP count differs')
    _require(len(lps) == len(qps), 'successful native solve requires one bounded LP initializer per local QP')
    row_count = problem['geometryConstraintRows']+problem['nativeConstraintRows']
    for index, (lp, qp) in enumerate(zip(lps, qps), 1):
        _flags(lp, dict(iteration=index, started=True, linprogReturned=True, success=True,
            status=0, method='highs-ds', normalizedPrimalFeasibilityTolerance=1e-9,
            normalizedDualFeasibilityTolerance=1e-9, initializationOnly=True,
            nonlinearCandidateOrSeed=False, finiteWitness=True, maximumBoundViolation=0.), 'local LP initializer')
        _flags(qp, dict(iteration=index, started=True, minimizeReturned=True, success=True,
            linearSubproblemPolicy='row-normalized-feasible-start', finiteWitness=True,
            maximumBoundViolation=0.), 'local QP witness')
        center = _array(lp.get('currentPointCoordinates'), (27,), 'local genuine current point')
        initial = _array(lp.get('returnedCoordinates'), (27,), 'sole LP local initializer')
        _same(qp.get('currentPointCoordinates'), center.tolist(), 'QP local center differs')
        _same(qp.get('localInitialCoordinates'), initial.tolist(), 'QP did not use sole LP initializer')
        _same((center+_array(lp.get('returnedStepCoordinates'), (27,), 'LP step')).tolist(),
            initial.tolist(), 'LP step/position witness differs')
        _same((center+_array(qp.get('returnedStepCoordinates'), (27,), 'QP step')).tolist(),
            qp.get('returnedCoordinates'), 'QP step/position witness differs')
        for record, tolerance in ((lp, 1e-9), (qp, 1e-10)):
            witness = _array(record.get('returnedCoordinates'), (27,), 'local returned coordinates')
            _require(np.all(witness >= bounds[:, 0]) and np.all(witness <= bounds[:, 1]),
                'local witness changed original physical bounds')
            slacks = _array(record.get('unscaledLinearizedSlacks'), (row_count,), 'full original local slacks')
            _require(record.get('minimumUnscaledLinearizedSlack') == float(min(slacks))
                and record.get('maximumUnscaledLinearizedViolation') == float(max(0., -min(slacks))),
                'local raw numeric residual receipt differs')
            normalized = _number(record.get('minimumNormalizedLinearizedSlack'), 'normalized local slack')
            _require(normalized >= -tolerance if record is lp else min(slacks) >= -tolerance,
                'local initializer/QP numerical feasibility gate differs')
    mode = result.get('subproblemReference')
    _require(mode in ('anchor', 'current') and problem.get('subproblemReference') == mode, 'explicit native local objective required')
    for record in (result, problem, solver):
        _exact(record.get('anchorObjectiveOptimizedDuringRestoration'), mode == 'anchor', 'feasibility objective relabeled')
    _flags(result.get('certificateFunction'), dict(module='native_wrench_horizon', name='certify_finite_horizon',
        originalDefaultCertificateUsed=True), 'original strict certificate function')
    certificate = result.get('strictFiniteCertificate')
    path_key = verify_native_finite_certificate(certificate, problem, knots, context, support, feet, limits)
    preflight = dict(passed=True, featureProfile=FEATURE_PROFILE, requestedSteps=1,
        candidateCertificateIndependentlyVerified=True, pathSHA256=path_key,
        candidateSHA256=digest(knots.tolist()), activationContextSHA256=context.receipt()['contextSHA256'],
        inputPayloadSHA256=data['actualPayloadSHA256'], sourceBindings=annotation['frozenSourceHashes'],
        receiptSHA256=digest(result), candidateCertificateSHA256=digest(certificate),
        nativeWrenchProblemSHA256=digest(problem),
        physicsIntegrations=0, nativeWorldConstructions=0, nativeAdmission=False)
    return canonical(dict(featureProfile=FEATURE_PROFILE, proposalSHA256=original.sha(path), proposalFilename=path.name,
        binding=annotation, independentKnotsRad=knots.tolist(), offlineReceiptSHA256=digest(result),
        flightOptimizerCallsAtRuntime=0, nativeWrenchProblem=problem, activationInputBinding=problem['inputBinding'],
        nativePreflight=preflight, strictFiniteCertificate=certificate))


def verify_native_activation_identity(proposal, context, support, feet, payload):
    _require(proposal.get('featureProfile') == FEATURE_PROFILE
        and isinstance(proposal.get('nativeWrenchProblem'), dict), 'explicit native planned profile proposal required')
    bound = proposal.get('activationInputBinding', {})
    _same(proposal['nativeWrenchProblem'].get('inputBinding'), bound, 'native activation input binding was replaced')
    preflight = proposal.get('nativePreflight', {})
    _flags(preflight, dict(passed=True, featureProfile=FEATURE_PROFILE, requestedSteps=1,
        candidateCertificateIndependentlyVerified=True), 'native proposal preflight')
    expected = dict(contextSHA256=context.receipt()['contextSHA256'], sourceNativeStep=context.source_step,
        sourceTimeS=context.source_time, referencePhase=context.phase, referenceElapsedS=context.elapsed,
        currentSupportSHA256=support.key, actualFeetModelSHA256=digest([np.asarray(p).tolist() for p in feet]),
        actualPayloadSHA256=hashlib.sha256(payload).hexdigest(), actualPayloadBytes=len(payload))
    for key, value in expected.items():
        _same(bound.get(key), value, 'current native activation identity differs: '+key)
    _require(preflight.get('candidateSHA256') == digest(proposal.get('independentKnotsRad'))
        and preflight.get('inputPayloadSHA256') == expected['actualPayloadSHA256']
        and preflight.get('activationContextSHA256') == expected['contextSHA256']
        and preflight.get('candidateCertificateSHA256') == digest(proposal.get('strictFiniteCertificate'))
        and preflight.get('nativeWrenchProblemSHA256') == digest(proposal['nativeWrenchProblem']),
        'native proposal/certificate changed after preflight')


class NativePlannedController(original.WholeBodyController):
    wholebody_profile = FEATURE_PROFILE
    flight_admission_receipt_field = 'finitePlannedJets'

    def _activate_wholebody(self, context):
        value = getattr(self, '_wholebody_proposal_json', None)
        _require(isinstance(value, str) and value, 'valid native proposal required before activation')
        proposal = json.loads(value)
        _require(proposal.get('featureProfile') == FEATURE_PROFILE
            and proposal.get('nativePreflight', {}).get('passed') is True,
            'native proposal must pass pure preflight before activation')
        return super()._activate_wholebody(context)

    def _verify_activation_identity(self, proposal, context, support, feet, payload):
        verify_native_activation_identity(proposal, context, support, feet, payload)

    def _construct_cycle(self, realizer, context, proposal):
        return original.WholeBodyCycle(realizer, context, self.w.limits, self.heading,
            proposal['independentKnotsRad'], lumbar_window_deg=None)

    def _admit_flight_samples(self, samples, support, feet):
        return finite_planned_jet_admission(self.w, samples, original.actual_contacts(self, support),
            P@np.asarray(feet[self.stance_side])[:3, 3], feet[self.stance_side])

    def _native_source_bindings(self):
        result = super()._native_source_bindings()
        result['runtimeSHA256'] = {name: original.sha(original.source_path(name)) for name in RUNTIME}
        return result


class NativePlannedSteps(original.WholeBodySteps):
    reviewed_controller_class = NativePlannedController
    wholebody_profile = FEATURE_PROFILE

    def __init__(self, base, helper, output, count=1, *, proposal_json):
        _require(count == 1 and type(count) is int, 'native planned whole-body pilot count1 only')
        super().__init__(base, helper, output, count, proposal_json=proposal_json)
        self._wholebody_source_paths = {name: original.source_path(name) for name in RUNTIME}
        self._wholebody_code_start = {name: original.sha(path) for name, path in self._wholebody_source_paths.items()}
