"""Finite planned-native-jet checks, with unchanged entry/static reserve.

Only two independent counter-zero scratch worlds receive proposed states. This
is a separate, opt-in finite-sample policy, not runtime or whole-path admission.
Moving samples retain their geometric root/full21 jets for M*qdd+b. The first
sample and stationary generalized jets additionally receive the original static
LP at the SAME qpos, explicitly installed with zero velocity.
"""
import copy
import hashlib

import numpy as np

from coupled_posture_kinematics import MatrixJet
from coupled_posture_reference import MAX_TOTAL_COM_SPEED, MAX_TOTAL_COM_ACCELERATION
from native_planned_jet_provider import NativePlannedJetProvider, generalized_planned_jets
from native_wrench_constraints import ContactWrenchCone, independent_native_primal_check
from source_arm_knee_mobility import native_bytes
from source_step_probe import sha
from wholebody_static_admission import strict_contact_lp


POLICY = 'two-counter-zero-finite-planned-jets-with-entry-static-reserve-v1'
MOTOR_CAP = .925
STATIONARY_TOLERANCE = 1e-8


def _binding(world):
    return dict(modelSHA256=sha(world.xml), helperSHA256=sha(world.helper),
                bridgeSHA256=sha(world.base / 'source_bridge_v2.so'),
                engineSHA256=sha(world.base / 'dependency/official-wheel/mujoco/libmujoco_nogl.so.2.1.3'))


def _finite_array(value, shape, description):
    array = np.array(value, dtype=float, copy=True)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError('finite ' + description + ' required')
    return array


def _below(receipt, key, cap):
    value = receipt.get(key)
    return (isinstance(value, (float, int, np.floating, np.integer))
            and np.isfinite(value) and 0. <= value < cap)


def finite_planned_jet_admission(actual_world, samples, contacts,
                                 reference_origin_source, expected_stance_pose_model,
                                 *, factory=None):
    """Check a frozen ordered iterable of (label, CycleSample), without stepping.

    Contacts are exactly three unique current points on one actual stance foot;
    the fixed source origin and model-frame stance pose are caller-bound inputs.
    Rejection stops immediately, without changing or repairing the candidate. The
    factory, when supplied, must construct a fresh original-model scratch world.
    """
    actual = actual_world
    before = native_bytes(actual)
    counter = actual.l.lh_dim(actual.h, 6)
    worlds, checks = [], []
    planned_installations = static_installations = 0
    accepted, rejection = True, None
    try:
        rows = tuple(samples)
        if not rows:
            raise ValueError('nonempty finite planned path required')
        contacts = tuple((side, tuple(_finite_array(point, (3,), 'actual contact point')))
                         for side, point in contacts)
        if (len(contacts) != 3 or len({side for side, _ in contacts}) != 1
                or any(side not in (0, 1) for side, _ in contacts)
                or len(set(contacts)) != 3):
            raise ValueError('three unique actual contacts on the same stance foot required')
        origin = _finite_array(reference_origin_source, (3,), 'fixed source origin')
        stance_pose = _finite_array(expected_stance_pose_model, (4, 4), 'actual stance transform')
        MatrixJet.rest(stance_pose)
        origin.setflags(write=False)
        stance_pose.setflags(write=False)
        expected = _binding(actual)
        limits = _finite_array(actual.limits, (21, 2), 'original source joint limits')
        if np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError('ordered original source joint limits required')
        cone = ContactWrenchCone([point for _, point in contacts], origin)

        for repetition in range(2):
            w = factory(actual) if factory else type(actual)(actual.base, actual.helper, xml=actual.xml)
            if (w is actual or w.h == actual.h
                    or any(w is old or w.h == old.h for old in worlds)):
                raise ValueError('independent fresh counter-zero scratch required')
            worlds.append(w)
            if w.l is actual.l or any(w.l is old.l for old in worlds[:-1]):
                raise ValueError('independent scratch native interface required')
            if w.l.lh_dim(w.h, 6) != 0:
                raise ValueError('fresh counter-zero scratch required')
            if _binding(w) != expected:
                raise ValueError('scratch model/helper/bridge/engine binding differs')
            scratch_limits = _finite_array(w.limits, (21, 2), 'scratch source joint limits')
            if np.max(abs(scratch_limits - limits)) > 1e-12:
                raise ValueError('original source joint limits differ')

            def forbidden(*args, **kwargs):
                raise AssertionError('finite planned-jet admission must never integrate')

            w.l.lh_step_audited = forbidden
            provider = NativePlannedJetProvider(w, contacts, origin, stance_pose)
            for index, (label, sample) in enumerate(rows):
                qpos, velocity, acceleration = generalized_planned_jets(sample.geometric)
                geometric_jets = {
                    key: [_finite_array(value, shape, key).tolist() for value in jet.arrays()]
                    for key, shape, jet in (
                        ('geometricFullJointJet', (21,), sample.geometric.full_joints),
                        ('geometricRootModelJet', (4, 4), sample.geometric.root_model),
                        ('geometricCOMJet', (3,), sample.geometric.com_source))}
                command_com = tuple(_finite_array(value, (3,), 'command COM jet')
                                    for value in sample.com.arrays())
                stationary = bool(max(np.max(abs(velocity)), np.max(abs(acceleration)))
                                  <= STATIONARY_TOLERANCE)
                # Never substitute stationary jets for this native demand.
                data = provider(sample, final=True)
                planned_installations += 1
                if w.l.lh_dim(w.h, 6) != 0:
                    raise ValueError('planned-jet scratch counter changed')
                matrices = {key: _finite_array(data[key], shape, 'native ' + key)
                            for key, shape in (('B', (27, 21)), ('G', (27, 9)),
                                               ('demand', (27,)), ('stanceJacobian', (6, 27)))}
                data_origin = _finite_array(data['referenceOriginSource'], (3,), 'native fixed origin')
                if data_origin.tobytes() != origin.tobytes():
                    raise ValueError('native planned inputs changed the bound fixed origin')
                native = data['native']
                for key, value in (('generalizedVelocity', velocity),
                                   ('generalizedAcceleration', acceleration)):
                    if np.max(abs(_finite_array(native[key], (27,), key) - value)) > 1e-12:
                        raise ValueError('native receipt differs from declared planned jets')
                # Keep all native matrices and declared jets for independent audit.
                planned_inputs = dict(**{key: value.tolist() for key, value in matrices.items()},
                    qpos=qpos.tolist(), qvel=velocity.tolist(), qacc=acceleration.tolist(),
                    referenceOriginSource=data_origin.tolist(), forceScaleN=float(data['forceScaleN']),
                    **geometric_jets)
                primal = independent_native_primal_check(data, cone)
                gap, native_height = float(data['minimumCleanSelfGapM']), float(data['nativeCOMHeightM'])
                margins = np.minimum(qpos[7:] - limits[:, 0], limits[:, 1] - qpos[7:])
                speed, accel = (float(np.linalg.norm(value)) for value in command_com[1:])
                dynamic_ok = bool(primal.get('success') and primal.get('accepted')
                    and primal.get('originalStrictMotorCap') == MOTOR_CAP
                    and primal.get('normalForceUpperBound', 'missing') is None
                    and primal.get('friction') == .6
                    and _below(primal, 'maximumMotorFraction', MOTOR_CAP)
                    and _below(primal, 'maximumNativeEquilibriumResidual', 1e-8)
                    and _below(primal, 'maximumFrictionViolation', 1e-8))
                guards_ok = bool(np.isfinite([gap, native_height, planned_inputs['forceScaleN']]).all()
                    and planned_inputs['forceScaleN'] > 0. and gap > .025
                    and command_com[0][2] >= .65 and native_height >= .65
                    and speed <= MAX_TOTAL_COM_SPEED + 1e-10
                    and accel <= MAX_TOTAL_COM_ACCELERATION + 1e-10
                    and min(margins[:15]) >= .01 - 1e-9
                    and min(margins[15:]) >= .02 - 1e-9)
                additional_static = None
                static_ok = True
                if index == 0 or stationary:
                    # Entry is checked even if its actual-arm-conditioned jets move.
                    w.install_cold(dict(qpos=qpos.tolist(), qvel=[0.] * 27))
                    static_installations += 1
                    lp = strict_contact_lp(w, contacts)
                    static_ok = bool(lp.get('success')
                        and _below(lp, 'requiredMotorFraction', MOTOR_CAP)
                        and _below(lp, 'maximumEqualityResidual', 1e-8)
                        and _below(lp, 'maximumFrictionViolation', 1e-8))
                    additional_static = dict(policy='additional-original-static-reserve-at-same-qpos',
                        reasons=(['first-sample'] if index == 0 else [])
                                + (['stationary-full-generalized-jets'] if stationary else []),
                        qpos=qpos.tolist(), qvel=[0.] * 27, qacc=[0.] * 27,
                        staticLP=copy.deepcopy(lp), accepted=static_ok)
                if w.l.lh_dim(w.h, 6) != 0:
                    raise ValueError('planned/static scratch counter changed')
                ok = bool(dynamic_ok and guards_ok and static_ok)
                checks.append(dict(repetition=repetition, sampleIndex=index, label=label,
                    accepted=ok, plannedJetCheckAccepted=dynamic_ok,
                    originalPhysicalGuardsAccepted=guards_ok,
                    stationaryFullGeneralizedJets=stationary,
                    maximumPlannedGeneralizedVelocity=float(np.max(abs(velocity))),
                    maximumPlannedGeneralizedAcceleration=float(np.max(abs(acceleration))),
                    plannedNativeInputs=planned_inputs, plannedNative=copy.deepcopy(native),
                    plannedNativePrimalLP=copy.deepcopy(primal), additionalStaticReserve=additional_static,
                    minimumCleanSelfGapM=gap, commandCOMHeightM=float(command_com[0][2]),
                    nativeCOMHeightM=native_height, commandCOMJet=[value.tolist() for value in command_com],
                    commandCOMSpeedMps=speed, commandCOMAccelerationMps2=accel,
                    minimumLowerJointMarginRad=float(min(margins[:15])),
                    minimumArmJointMarginRad=float(min(margins[15:])), nativeCounter=0))
                if not ok:
                    accepted = False
                    rejection = 'finite planned-native/contact/static-reserve/physical-guard check'
                    break
            if not accepted:
                break
    finally:
        close_errors = []
        for w in worlds:
            try:
                w.close()
            except Exception as exc:
                close_errors.append(exc)
        after, after_counter = native_bytes(actual), actual.l.lh_dim(actual.h, 6)
        if after != before or after_counter != counter:
            raise ValueError('planned-jet admission mutated the actual native world')
        if close_errors:
            raise RuntimeError('planned-jet scratch close failed') from close_errors[0]
    return dict(schema=1, policy=POLICY, accepted=accepted, rejection=rejection,
        sourceStep=counter, actualNativePayloadSHA256=hashlib.sha256(before).hexdigest(), **expected,
        actualContacts=[dict(footSide=side, pointSource=list(point)) for side, point in contacts],
        referenceOriginSource=origin.tolist(), expectedStancePoseModel=stance_pose.tolist(),
        originalJointLimits=limits.tolist(), sampleCount=len(rows), worldsConstructed=len(worlds),
        checks=checks, contactWrenchCone=copy.deepcopy(cone.receipt),
        physicsIntegrations=0, actualStateWrites=0,
        counterZeroScratchInstallations=planned_installations + static_installations,
        plannedJetInstallations=planned_installations, additionalStaticInstallations=static_installations,
        staticReservePolicy='first-sample-and-stationary-full-generalized-jets',
        stationarityTolerance=STATIONARY_TOLERANCE,
        originalStrictMotorCap=MOTOR_CAP, maximumCommandCOMSpeedMps=MAX_TOTAL_COM_SPEED,
        maximumCommandCOMAccelerationMps2=MAX_TOTAL_COM_ACCELERATION,
        actualBytesAndCounterUnchanged=True, finiteSamplesOnly=True,
        wholePathOrDynamicQualification=False, nativeAdmission=False, fullPilotPending=True,
        freshMeasuredRiskAndOriginalEveryNativeGatesStillRequired=True)
