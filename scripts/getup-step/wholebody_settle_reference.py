"""One causal same-foot quiet-endpoint family; never installs a native pose.

This solves geometry only. A returned endpoint is NOT a finite-time path,
motor/contact certificate, measured settle readiness, or permission to stand.
The runtime owns the actual measured midpoint+35mm COM target. Both supplied
foot poses must come from that same current native sample, not predicted landing.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
import math

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from coupled_posture_reference import Jet, C2Segment, OrientationJet, chart, wrap
from coupled_posture_kinematics import MatrixJet, P, pose_jet_to_source

POLICY = 'causal-same-foot-wholebody-quiet-endpoint-v1'
MAX_ITERATIONS = 100
FEET = ('left_foot', 'right_foot')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def _array(value, shape, label):
    a = np.array(value, dtype=float, copy=True)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('invalid '+label)
    return a


def _rigid(value):
    a = _array(value, (4, 4), 'current foot pose')
    R = a[:3, :3]
    if (np.max(abs(a[3]-[0., 0., 0., 1.])) > 1e-10 or
            np.max(abs(R.T@R-np.eye(3))) > 1e-8 or abs(np.linalg.det(R)-1.) > 1e-8):
        raise ValueError('current foot pose is not rigid')
    return a


def _kinematic_bindings(model, limits, arm_map, leg_origin):
    """Semantic pure inputs, distinct from externally verified raw file hashes."""
    return dict(modelKinematicsSHA256=digest(asdict(model)),
        originalSourceLimitsRad=_array(limits, (21, 2), 'original limits').tolist(),
        armMapSHA256=digest(asdict(arm_map)),
        legOriginRightLeftRad=_array(leg_origin, (2,), 'persistent thigh origins').tolist())


def _same_numbers(a, b):
    try:
        left, right = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        return left.shape == right.shape and np.isfinite(left).all() and np.array_equal(left, right)
    except (TypeError, ValueError):
        return False


def _rest_fk(model, q, root):
    """Existing ModelFK graph and hinge convention, position-only fast path."""
    out = {'torso': root}
    for part in model.parts[1:]:
        hinge = np.eye(4)
        if part.qindex is not None:
            c, s = math.cos(q[part.qindex]), math.sin(q[part.qindex])
            hinge[1:3, 1:3] = [[c, -s], [s, c]]
        out[part.name] = (out[part.parent]@part.parent_frame.arrays()[0]@
            hinge@part.child_inverse.arrays()[0])
    return out


@dataclass(frozen=True)
class QuietEndpoint:
    """Detached immutable geometry/receipt; no state or controller references."""
    accepted: bool
    lower: tuple
    full_joints: tuple
    root_model: tuple
    receipt_json: str

    def receipt(self):
        geometry = dict(lower=list(self.lower), full_joints=list(self.full_joints),
            root_model=list(self.root_model))
        return dict(json.loads(self.receipt_json), **geometry,
            geometrySHA256=digest(geometry))


def solve_samefoot_endpoint(model, limits, arm_map, leg_origin, *, stance_side,
        current_feet_model, measured_com_target, initial_lower,
        lumbar_anchor, thorax_preference_source, pelvis_preference_source,
        actual_hull_equations, source_identity, heading_rad,
        optimizer=minimize):
    """One fixed 15-coordinate bounded solve, no fallback or parameter scan.

    Stance realizes the unactuated root. Other foot pose, measured final COM and
    torso heading are hard. Thorax/pelvis orientation and joint posture remain
    soft, using the inherited squared-row weights 4/4/.2. The original .01
    source-reference interior and .02 mapped-arm/range reserves are retained;
    conservative transfer-endpoint .06 ankle reserve is not imposed at rest.
    A caller may call this once at a real landing boundary and freeze its result.
    It must independently certify motors/contact/gap and a C2 path before use.
    """
    if stance_side not in (0, 1):
        raise ValueError('feet order is left/right')
    limits = _array(limits, (21, 2), 'original source limits')
    origin = _array(leg_origin, (2,), 'persistent thigh origins')
    q0 = _array(initial_lower, (15,), 'current lower initializer')
    anchor = _array(lumbar_anchor, (3,), 'lumbar anchor')
    final_com = _array(measured_com_target, (3,), 'measured COM target')
    feet = tuple(_rigid(x) for x in current_feet_model)
    if len(feet) != 2:
        raise ValueError('two same-sample actual feet required')
    prefs = tuple(_array(x, (3, 3), 'soft body preference') for x in
        (thorax_preference_source, pelvis_preference_source))
    for R in prefs:
        if np.max(abs(R.T@R-np.eye(3))) > 1e-8 or abs(np.linalg.det(R)-1.) > 1e-8:
            raise ValueError('soft body preference is not rigid')
    eq = np.array(actual_hull_equations, dtype=float, copy=True)
    if eq.ndim != 2 or eq.shape[1] != 3 or len(eq) < 3 or not np.isfinite(eq).all():
        raise ValueError('current actual hull equations required')
    if np.max(abs(np.linalg.norm(eq[:, :2], axis=1)-1.)) > 1e-8:
        raise ValueError('actual hull normals must have unit length')
    if not isinstance(source_identity, dict) or any(k not in source_identity for k in
            ('sourceStep', 'sourceTimeS', 'actualStateSHA256', 'modelSHA256')):
        raise ValueError('exact current source identity required')
    if not math.isfinite(heading_rad):
        raise ValueError('finite retained heading required')
    lo, hi = limits[:15, 0]+.01, limits[:15, 1]-.01
    lo[:3] = np.maximum(lo[:3], anchor-math.radians(5.))
    hi[:3] = np.minimum(hi[:3], anchor+math.radians(5.))
    hi[[6, 12]] = np.minimum(hi[[6, 12]], 0.)
    if np.any(lo > hi) or np.any(q0 < lo-1e-10) or np.any(q0 > hi+1e-10):
        raise ValueError('one declared initializer outside original joint/lumbar bounds; no clamp')
    if min(-(eq[:, :2]@final_com[:2]+eq[:, 2])) < .003-1e-10:
        raise ValueError('measured endpoint outside current 3mm hull reserve')
    ranges = np.array(arm_map.ranges)
    calls = 0
    cached_x = cached = None

    def geometry(x):
        nonlocal calls, cached_x, cached
        if cached_x is None or not np.array_equal(x, cached_x):
            calls += 1
            # Future-arm map is geometric prediction only, never a runtime arm
            # command. At quiet its delayed law should converge but must audit.
            rel = _rest_fk(model, np.r_[x, np.zeros(6)], np.eye(4))
            theta = []
            for name in ('right', 'left'):
                d = P@(rel[name+'_shin'][:3, 3]-rel[name+'_thigh'][:3, 3])
                theta.append(math.atan2(d[0], -d[2]))
            delta = -(np.array(theta)-origin)
            arms = np.array(arm_map.evaluate(Jet.rest(delta), reserve=0.).position)
            q = np.r_[x, arms]
            root = feet[stance_side]@np.linalg.inv(rel[FEET[stance_side]])
            poses = _rest_fk(model, q, root)
            com = sum(p.mass*(P@(poses[p.name]@np.r_[p.com, 1.])[:3])
                for p in model.parts)/model.total_mass
            R1 = P@root[:3, :3]@P.T
            R4 = P@poses['pelvis'][:3, :3]@P.T
            cached_x, cached = np.array(x, copy=True), (q, root, poses, com, delta, R1, R4)
        return cached

    def equality(x):
        q, root, poses, com, delta, R1, R4 = geometry(x)
        other = 1-stance_side
        pose = poses[FEET[other]]
        return np.r_[P@(pose[:3, 3]-feet[other][:3, 3]),
            Rotation.from_matrix(pose[:3, :3]@feet[other][:3, :3].T).as_rotvec(),
            com-final_com, wrap(chart(R1)[0]-heading_rad)]

    def inequality(x):
        q, root, poses, com, delta, R1, R4 = geometry(x)
        return np.r_[np.minimum(delta-ranges[:, 0], ranges[:, 1]-delta)-.02,
            np.minimum(q[15:]-limits[15:, 0], limits[15:, 1]-q[15:])-.02,
            -(eq[:, :2]@com[:2]+eq[:, 2])-.003]

    def objective(x):
        q, root, poses, com, delta, R1, R4 = geometry(x)
        errors = [Rotation.from_matrix(target@actual.T).as_rotvec()
            for target, actual in zip(prefs, (R1, R4))]
        return float(.5*(np.sum((.2*(x-q0))**2)+sum(np.sum((4.*e)**2) for e in errors)))

    declared = dict(policy=POLICY, source=source_identity, stanceSide=stance_side,
        kinematicBindings=_kinematic_bindings(model, limits, arm_map, origin),
        actualFeetModel=[x.tolist() for x in feet], measuredFinalCOM=final_com.tolist(),
        measuredCOMTargetPolicy='controller-owned current actual foot midpoint+35mm heading; unchanged height',
        initialLower=q0.tolist(), lumbarAnchor=anchor.tolist(), lowerBounds=lo.tolist(),
        upperBounds=hi.tolist(), hardReferenceInteriorRad=.01, mappedArmReserveRad=.02,
        originalHullEquations=eq.tolist(), hullInsetM=.003, retainedHeadingRad=heading_rad,
        softThoraxPreferenceSource=prefs[0].tolist(), softPelvisPreferenceSource=prefs[1].tolist(),
        squaredRowWeights=dict(thorax=4., pelvis=4., lower=.2), maximumIterations=MAX_ITERATIONS)
    trace = []
    def callback(x):
        trace.append(dict(iteration=len(trace)+1, objective=objective(x),
            maximumEqualityResidual=float(max(abs(equality(x)))),
            minimumInequalitySlack=float(min(inequality(x)))))
    result = optimizer(objective, q0.copy(), method='SLSQP', bounds=list(zip(lo, hi)),
        constraints=[dict(type='eq', fun=equality), dict(type='ineq', fun=inequality)],
        callback=callback, options=dict(maxiter=MAX_ITERATIONS, ftol=1e-11, disp=False))
    x = np.array(result.x, dtype=float)
    q, root, poses, com, delta, R1, R4 = geometry(x)
    error, slack = equality(x), inequality(x)
    margins = np.minimum(q-limits[:, 0], limits[:, 1]-q)
    accepted = bool(result.success and max(abs(error)) < 1e-6 and min(slack) >= -1e-8
        and min(margins[:15]) >= .01-1e-8 and min(margins[15:]) >= .02-1e-8)
    receipt = dict(schema=1, policy=POLICY, accepted=accepted, input=declared,
        inputSHA256=digest(declared), optimizerCalls=1, geometryCalls=calls,
        optimizerSuccess=bool(result.success), optimizerMessage=str(result.message),
        optimizerIterations=int(result.nit), trace=trace,
        maximumEqualityResidual=float(max(abs(error))), minimumInequalitySlack=float(min(slack)),
        allJointMarginsRad=margins.tolist(), geometricCOM=com.tolist(),
        geometricThoraxSource=R1.tolist(), geometricPelvisSource=R4.tolist(),
        thoraxChartRad=chart(R1).tolist(), pelvisChartRad=chart(R4).tolist(),
        finalFutureArmDeltaRightLeft=delta.tolist(),
        geometricFutureArmsNotRuntimeTargets=True, measuredDelayedArmLawUnchanged=True,
        rootRealizedFromCurrentStance=True, arbitraryRootInjection=False,
        nativeWorlds=0, physicsIntegrations=0, nativeStateInstallations=0,
        motorContactCollisionCertified=False, C2PathCertified=False,
        nativeReadinessCertified=False, physicalQualificationClaim=False,
        fallback=False, parameterScan=False)
    return QuietEndpoint(accepted, tuple(x), tuple(q), tuple(root.reshape(-1)),
        json.dumps(receipt, sort_keys=True, separators=(',', ':'), allow_nan=False))


class SameFootSettlePlan:
    """Finite causal return with one FK owner, not independent rest q0/rootR.

    The actual stance anchors root realization. The other geometric foot moves
    C2 from incoming lower-reference FK to its current measured pose, permitting
    the lower reference itself to start without a projection. Declared command
    slacks preserve prior COM/body/foot jets at activation and decay to zero.
    Endpoint geometry, including both feet and measured COM, is then common.
    Geometry preparation is finite sampling, NOT an interval/dynamic certificate.
    Every runtime command must repeat complete COM/support/joint checks and keep
    the original loaded-settle readiness. Strong arms are supplied by the caller.
    """
    def __init__(self, realizer, context, endpoint, current_other_foot_model,
            source_limits, *, current_identity, duration=1.52,
            saved_artifact_binding=None, phase_boundary=None):
        from wholebody_cycle_reference import (as_jet, add, subtract, pose_chart,
            chart_difference)
        from coupled_posture_reference import angular_chart
        if not endpoint.accepted:
            raise ValueError('accepted current same-foot endpoint required')
        if context.phase != 'settle' or context.elapsed < 0.:
            raise ValueError('causal settle boundary context required')
        if duration != 1.52:
            raise ValueError('original measured settle duration must remain 1.52s')
        if context.elapsed != 0.:
            boundary = phase_boundary or {}
            step = boundary.get('sourceStep'); time = boundary.get('sourceTimeS')
            if (type(step) is not int or not isinstance(time, (float, int))
                    or context.source_step-step not in range(4)
                    or abs(context.elapsed-(context.source_step-step)*.00125)>1e-10
                    or abs(context.source_time-time-context.elapsed)>1e-9):
                raise ValueError('first-command boundary identity/zero-to-three-sample lag required')
        self.original_duration = duration
        self.phase_start_elapsed = context.elapsed
        # Plan causally from the first actual command. End at the unchanged
        # original phase deadline, never backdate a plan using future state.
        duration -= context.elapsed
        receipt = endpoint.receipt()
        declared = receipt.get('input', {})
        if (receipt.get('accepted') is not True or receipt.get('policy') != POLICY
                or receipt.get('inputSHA256') != digest(declared)):
            raise ValueError('accepted endpoint input binding is invalid')
        source = declared.get('source', {})
        required = ('sourceStep', 'sourceTimeS', 'actualStateSHA256', 'modelSHA256')
        if not isinstance(current_identity, dict) or any(k not in current_identity for k in required):
            raise ValueError('verified current source/model identity required')
        if (current_identity['sourceStep'] != context.source_step
                or current_identity['sourceTimeS'] != context.source_time
                or any(current_identity[k] != source.get(k) for k in required)
                or ('sourceXMLSHA256' in source and
                    current_identity.get('sourceXMLSHA256') != source['sourceXMLSHA256'])):
            raise ValueError('endpoint differs from current source/model identity')
        bindings = declared.get('kinematicBindings')
        if bindings is None:
            # Only an explicitly independently verified old artifact may use this
            # bridge. The caller verifies its immutable receipt before supplying
            # it; the pure constructor never reads files or invents provenance.
            evidence = saved_artifact_binding or {}
            if (evidence.get('kind') != 'independently-verified-saved-artifact'
                    or evidence.get('endpointInputSHA256') != receipt['inputSHA256']
                    or evidence.get('endpointReceiptSHA256') != digest(receipt)
                    or not isinstance(evidence.get('verificationReceiptSHA256'), str)
                    or len(evidence['verificationReceiptSHA256']) != 64):
                raise ValueError('independently verified saved-artifact binding required')
            bindings = evidence.get('kinematicBindings')
        actual_bindings = _kinematic_bindings(realizer.model, source_limits,
            realizer.arm_map, realizer.leg_origin)
        if not isinstance(bindings, dict) or any(
                bindings.get(k) != actual_bindings[k] for k in
                ('modelKinematicsSHA256', 'armMapSHA256')) or any(
                not _same_numbers(bindings.get(k), actual_bindings[k]) for k in
                ('originalSourceLimitsRad', 'legOriginRightLeftRad')):
            raise ValueError('endpoint model/limits/map/origin binding differs')
        config = json.loads(context.config_json)
        feet = declared.get('actualFeetModel', ())
        if (len(feet) != 2 or declared.get('stanceSide') != realizer.stance_side
                or config.get('stance_side') != realizer.stance_side
                or not _same_numbers(feet[realizer.stance_side], realizer.stance_pose.T)
                or any(np.any(x) for x in realizer.stance_pose.arrays()[1:])
                or not _same_numbers(feet[realizer.swing_side], current_other_foot_model)):
            raise ValueError('endpoint current stance/other foot binding differs')
        if (not _same_numbers(declared.get('initialLower'), context.joints.position[:15])
                or not _same_numbers(declared.get('lumbarAnchor'), context.joints.position[:3])):
            raise ValueError('endpoint initializer/lumbar anchor differs from incoming context')
        thorax = next((o for o in context.orientations if o.body == 1), None)
        if (not _same_numbers(declared.get('measuredFinalCOM'), config.get('finalCom'))
                or thorax is None or declared.get('retainedHeadingRad') != chart(thorax.arrays()[0])[0]):
            raise ValueError('endpoint measured COM/retained heading differs from incoming context')
        self.realizer, self.entry, self.endpoint = realizer, context, endpoint
        self.duration = float(duration)
        self.limits = _array(source_limits, (21, 2), 'original limits')
        self.lower0 = Jet(*(np.array(x)[:15] for x in
            (context.joints.position, context.joints.velocity, context.joints.acceleration)))
        if any(self.lower0.velocity) or any(self.lower0.acceleration):
            raise ValueError('minimal settle family requires actual incoming rest lower jets, no zeroing')
        idx = realizer.independent_indices
        initial = Jet(*(x[idx] for x in self.lower0.arrays()))
        final = Jet.rest(np.array(endpoint.lower)[idx])
        self.independent = C2Segment.between(0., duration, initial, final)
        raw = realizer.model.coupled(self.lower0, realizer.stance_side,
            realizer.stance_pose, realizer.arm_map, realizer.leg_origin)
        actual = realizer.with_arms(raw, as_jet(context.arms))
        self.arm_slack = C2Segment.between(0., duration,
            subtract(as_jet(context.arms), Jet(*(x[15:] for x in raw.full_joints.arrays()))),
            Jet.rest(np.zeros(6)))
        other = realizer.swing_side
        raw_foot = pose_jet_to_source(raw.pose(FEET[other]))
        target_foot = pose_jet_to_source(MatrixJet.rest(_rigid(current_other_foot_model)))
        self.geometric_foot_translation = C2Segment.between(0., duration,
            raw_foot.translation, target_foot.translation)
        self.geometric_foot_angles = C2Segment.between(0., duration,
            pose_chart(raw.pose(FEET[other])), pose_chart(MatrixJet.rest(current_other_foot_model)))
        self.com_slack = C2Segment.between(0., duration,
            subtract(as_jet(context.com), actual.com_source), Jet.rest(np.zeros(3)))
        self.body_slacks = {}
        rows = {o.body:o for o in context.orientations}
        for body, geometric in ((1, actual.thorax), (4, actual.pelvis)):
            start = angular_chart(*rows[body].arrays()) if body in rows else angular_chart(*geometric.arrays())
            self.body_slacks[body] = C2Segment.between(0., duration,
                chart_difference(start, angular_chart(*geometric.arrays())), Jet.rest(np.zeros(3)))
        self.foot_slacks = []
        for side, command in enumerate(context.feet):
            foot = pose_jet_to_source(raw.pose(FEET[side]))
            self.foot_slacks.append((C2Segment.between(0., duration,
                subtract(as_jet(command.translation), foot.translation), Jet.rest(np.zeros(3))),
                C2Segment.between(0., duration, chart_difference(
                    angular_chart(*command.orientation.arrays()), pose_chart(raw.pose(FEET[side]))),
                    Jet.rest(np.zeros(3)))))
        self.key = digest(dict(policy=POLICY, endpoint=endpoint.receipt()['inputSHA256'],
            endpointGeometrySHA256=endpoint.receipt()['geometrySHA256'],
            incomingContext=context.receipt()['contextSHA256'], duration=duration,
            originalDuration=self.original_duration, firstCommandPhaseElapsed=self.phase_start_elapsed,
            phaseBoundary=phase_boundary))

    def sample(self, elapsed, *, runtime_arms=None):
        from wholebody_cycle_reference import (add, compose_pose, pose_chart,
            CycleSample, as_jet)
        from coupled_posture_reference import angular_chart, angular_world
        if not math.isfinite(elapsed) or elapsed < self.phase_start_elapsed:
            raise ValueError('settle clock before causal first command')
        t = min(elapsed-self.phase_start_elapsed, self.duration)
        target = compose_pose(self.geometric_foot_translation.at(t),
            self.geometric_foot_angles.at(t))
        independent = self.independent.at(t)
        lower, mapped, diagnostics = self.realizer.derive(independent, target,
            self.lower0.position)
        if runtime_arms is None:
            arms = add(Jet(*(x[15:] for x in mapped.full_joints.arrays())), self.arm_slack.at(t))
        else:
            arms = as_jet(runtime_arms)
        result = self.realizer.with_arms(mapped, arms)
        com = add(result.com_source, self.com_slack.at(t))
        orientations = []
        for body, raw in ((1, result.thorax), (4, result.pelvis)):
            R,w,a = angular_world(add(angular_chart(*raw.arrays()), self.body_slacks[body].at(t)))
            orientations.append(OrientationJet(body, R.reshape(-1), w, a))
        feet = []
        for side in (0, 1):
            raw = pose_jet_to_source(result.pose(FEET[side]))
            translation, rotation = self.foot_slacks[side]
            feet.append(compose_pose(add(raw.translation, translation.at(t)),
                add(pose_chart(result.pose(FEET[side])), rotation.at(t))))
        diagnostics.update(policy=POLICY, elapsedS=elapsed, boundedClockS=t,
            firstCommandPhaseElapsedS=self.phase_start_elapsed,
            originalPhaseDurationS=self.original_duration,
            measuredFinalCOMUnchanged=True, rootInjected=False,
            runtimeArmsSupplied=runtime_arms is not None, wholeIntervalProof=False,
            nativeAdmission=False)
        return CycleSample(lower, com, *orientations, tuple(feet), result, diagnostics)

    def prepare(self, actual_support, *, samples=33):
        """Reject any finite-sample violation; retain live validation requirement."""
        if type(samples) is not int or samples < 9:
            raise ValueError('at least nine finite path samples required')
        actual_support.validate_identity(self.entry)
        values = []
        reason = None
        for t in self.phase_start_elapsed+np.linspace(0., self.duration, samples):
            row = self.sample(float(t)); q = np.array(row.geometric.full_joints.position)
            margins = np.minimum(q-self.limits[:,0], self.limits[:,1]-q)
            try:
                if min(margins[:15]) < .01-1e-8 or min(margins[15:]) < .02-1e-8 or max(q[[6,12]]) > 1e-10:
                    raise ValueError('original source-reference joint/map-arm bounds')
                if np.linalg.norm(row.com.velocity) > .19260552331163003+1e-10:
                    raise ValueError('unchanged total COM speed cap')
                if np.linalg.norm(row.com.acceleration) > .3901818436690813+1e-10:
                    raise ValueError('unchanged total COM acceleration cap')
                actual_support.validate_com(row.com)
            except ValueError as error:
                reason = str(error)
            values.append(dict(elapsedS=float(t), minimumLowerMarginRad=float(min(margins[:15])),
                com=list(row.com.position), speedMps=float(np.linalg.norm(row.com.velocity)),
                accelerationMps2=float(np.linalg.norm(row.com.acceleration)), rejection=reason))
            if reason is not None:
                break
        return dict(policy=POLICY, planSHA256=self.key, accepted=reason is None,
            rejection=reason, sampledPathPoints=len(values), samples=values,
            intervalFeasibilityProven=False, currentActualSupportOnly=True,
            everyCommandRecheckRequired=True, originalMeasuredReadinessRequired=True,
            measuredFinalCOMUnchanged=True, originalDurationS=self.original_duration,
            remainingDurationS=self.duration, firstCommandPhaseElapsedS=self.phase_start_elapsed,
            physicsIntegrations=0, nativeStateInstallations=0, physicalQualificationClaim=False)
