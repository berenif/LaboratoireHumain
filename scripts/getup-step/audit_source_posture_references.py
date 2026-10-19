"""Pure saved-data replay for the explicit v3 posture reference seam.

No runtime reference, FK, certificate evaluator, optimizer, or native module is
imported here. Static receipt checks establish bindings and reference arithmetic;
they do not independently establish its reported native/LP feasibility claims.
"""
import copy
import hashlib
import json
import math
from functools import lru_cache

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

PROFILE = 'reviewed-coupled-posture-pilot-v3'
DURATION = 1.52
SPEED_CAP = .19260552331163003
ACCELERATION_CAP = .3901818436690813
PLAN_FIELDS = ('candidate', 'com', 'lower_joints', 'thorax', 'pelvis',
    'lumbar_anchor', 'joint_limits', 'lumbar_change_limit')
P = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
STATIC_POLICY = dict(objective='sum((all21q-active-referenceq)^2)+100*sum((horizontalCOM-active-totalCOM)^2)',
    forwardPitchIncreaseDeg=1., rollNonworsening=True, pelvisTotalTiltNonworsening=True,
    lumbarChangeCapDeg=5., localLowerJointChangeCapRad=.18, motorCap=.9249,
    strictMotorAdmissionExclusive=.925, supportInsetM=.004, cleanGapExclusiveM=.025,
    minimumJointInteriorRad=.02, minimumStaticAnkleReferenceRad=.06,
    minimumOriginalMapReserveRad=.02, matchDistanceM=.001,
    footPositionToleranceM=1e-6, footOrientationToleranceRad=1e-6)


def value_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def _array(value, shape):
    out = np.asarray(value, dtype=float)
    if out.shape != shape or not np.isfinite(out).all():
        raise ValueError('invalid finite posture array '+str(shape))
    return out


def _same(a, b, tolerance=1e-10):
    if isinstance(a, dict) or isinstance(b, dict):
        return isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys() and all(
            _same(a[k], b[k], tolerance) for k in a)
    if isinstance(a, (list, tuple, np.ndarray)) or isinstance(b, (list, tuple, np.ndarray)):
        try:
            return len(a) == len(b) and all(_same(x, y, tolerance) for x, y in zip(a, b))
        except TypeError:
            return False
    if isinstance(a, (str, bool)) or isinstance(b, (str, bool)) or a is None or b is None:
        return type(a) is type(b) and a == b
    try:
        return math.isfinite(float(a)) and math.isfinite(float(b)) and abs(float(a)-float(b)) <= tolerance
    except (ValueError, TypeError):
        return False


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _jet(raw, width):
    _require(isinstance(raw, dict) and set(raw) == {'position', 'velocity', 'acceleration'},
        'invalid posture jet keys')
    return tuple(_array(raw[k], (width,)) for k in ('position', 'velocity', 'acceleration'))


def _rest(position):
    p = np.asarray(position, dtype=float)
    return p, np.zeros_like(p), np.zeros_like(p)


def hermite_coefficients(initial, final, duration):
    """Closed-form Hermite solution, independent of the producer's linear solve."""
    _require(math.isfinite(duration) and duration > 0., 'invalid posture duration')
    p, v, a = initial; q, w, b = final
    c0, c1, c2 = p, duration*v, .5*duration**2*a
    r0, r1, r2 = q-c0-c1-c2, duration*w-c1-2*c2, duration**2*b-2*c2
    return np.array([c0, c1, c2, 10*r0-4*r1+.5*r2,
        -15*r0+7*r1-r2, 6*r0-3*r1+.5*r2])


def _evaluate(coefficients, start, duration, now):
    _require(math.isfinite(now) and now >= start-1e-10, 'posture sample precedes frozen plan')
    u = min(1., max(0., (now-start)/duration))
    return tuple(sum(math.prod(range(i-order+1, i+1))*coefficients[i]*u**(i-order)
        for i in range(order, 6))/duration**order for order in (0, 1, 2))


def _bounds(coefficients, duration, order=0):
    result = []
    for column in coefficients.T:
        poly = np.polynomial.Polynomial(column).deriv(order)/duration**order
        points = [0., 1.]+[float(r.real) for r in poly.deriv().roots()
            if abs(r.imag) < 1e-10 and 0. < r.real < 1.]
        values = [float(poly(x)) for x in points]
        result.append([min(values), max(values)])
    return np.asarray(result)


def _maximum_norm(coefficients, duration, order):
    polynomials = [np.polynomial.Polynomial(c).deriv(order)/duration**order for c in coefficients.T]
    square = sum((p*p for p in polynomials), np.polynomial.Polynomial([0.]))
    points = [0., 1.]+[float(r.real) for r in square.deriv().roots()
        if abs(r.imag) < 1e-10 and 0. < r.real < 1.]
    return math.sqrt(max(0., max(float(square(x)) for x in points)))


def _rotation(raw):
    R = _array(raw, (9,)).reshape(3, 3)
    _require(np.max(abs(R.T@R-np.eye(3))) <= 1e-9 and abs(np.linalg.det(R)-1.) <= 1e-9,
        'invalid posture rotation')
    return R


def _chart(R):
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    _require(abs(math.cos(pitch)) >= 1e-5, 'singular posture heading chart')
    return np.array([math.atan2(R[1, 0], R[0, 0]), pitch,
        math.atan2(R[2, 1], R[2, 2])])


def _basis(angles):
    yaw, pitch, _ = angles
    sy, cy, sp, cp = math.sin(yaw), math.cos(yaw), math.sin(pitch), math.cos(pitch)
    return np.array([[0., -sy, cy*cp], [0., cy, sy*cp], [1., 0., -sp]])


def _angular_bias(E, rates):
    z, y, x = E.T
    return np.cross(z*rates[0], y)*rates[1]+np.cross(z*rates[0]+y*rates[1], x)*rates[2]


def _angle_jet(row, body):
    _require(row['body'] == body, 'wrong posture orientation body')
    angles = _chart(_rotation(row['rotation'])); E = _basis(angles)
    rates = np.linalg.solve(E, _array(row['velocity'], (3,)))
    accelerations = np.linalg.solve(E, _array(row['acceleration'], (3,))-_angular_bias(E, rates))
    return angles, rates, accelerations


def _heading_coefficients(initial, final, body, duration):
    p, v, a = _angle_jet(initial, body); q, w, b = _angle_jet(final, body)
    _require(abs(math.atan2(math.sin(q[0]-p[0]), math.cos(q[0]-p[0]))) <= 1e-10,
        'posture changes retained heading')
    _require(max(abs(v[0]), abs(a[0]), abs(w[0]), abs(b[0])) <= 1e-10,
        'posture has nonstationary retained heading')
    q[0] = p[0]; v[0] = a[0] = w[0] = b[0] = 0.
    coefficients = hermite_coefficients((p, v, a), (q, w, b), duration)
    lo, hi = _bounds(coefficients, duration)[1]
    _require(-math.pi/2 < lo <= hi < math.pi/2 and min(abs(math.cos(lo)), abs(math.cos(hi))) >= 1e-5,
        'posture path leaves nonsingular chart')
    return coefficients, (p, v, a), (q, w, b)


def _orientation(body, angle_jet, weight):
    angles, rates, accelerations = angle_jet; E = _basis(angles)
    return dict(body=body, rotation=Rotation.from_euler('ZYX', angles).as_matrix().reshape(-1).tolist(),
        velocity=(E@rates).tolist(), acceleration=(E@accelerations+_angular_bias(E, rates)).tolist(),
        kp=40., kd=12., weight=float(weight))


def _segment(raw, initial, final, start, duration):
    _require(_same(raw['start'], start) and _same(raw['duration'], duration), 'posture segment clock changed')
    width = len(initial[0])
    _require(_same(_jet(raw['initial'], width), initial) and _same(_jet(raw['final'], width), final),
        'posture segment endpoint changed')
    coefficients = hermite_coefficients(initial, final, duration)
    _require(_same(_array(raw['coefficients'], (6, width)), coefficients), 'posture polynomial changed')
    return coefficients


def validate_posture_plan(plan, initial_receipt=None):
    """Raise on an unbound/changed path; return independently rebuilt coefficients.

    Pass the static receipt at the file boundary. The per-context arithmetic hook
    can omit it only after the caller has bound that exact plan to that receipt.
    """
    encoded = json.dumps(plan, sort_keys=True, separators=(',', ':'), allow_nan=False)
    bound = None if initial_receipt is None else json.dumps(initial_receipt,
        sort_keys=True, separators=(',', ':'), allow_nan=False)
    # Cache only exact immutable serialized inputs. Returned arrays/dicts own
    # copies, so callers cannot poison later contexts or skip a changed plan.
    return copy.deepcopy(_validated_plan(encoded, bound))


@lru_cache(maxsize=8)
def _validated_plan(encoded, bound):
    plan = json.loads(encoded); initial_receipt = None if bound is None else json.loads(bound)
    _require(plan['version'] == 'coupled-posture-pure-v1', 'wrong frozen posture version')
    _require(plan['referenceSHA256'] == value_sha({k: plan[k] for k in PLAN_FIELDS}), 'frozen posture digest mismatch')
    _require(plan['finiteTimePhysicalFeasibilityProven'] is False and plan['staticSeedsArePathProof'] is False,
        'posture reference overclaims physical proof')
    candidate = plan['candidate']; target = candidate['target']
    initial = dict(com=plan['com']['initial'], lower_joints=plan['lower_joints']['initial'])
    if initial_receipt is not None:
        inputs = initial_receipt['input']; input_sha = initial_receipt['inputSHA256']
        _require(value_sha(inputs) == input_sha and candidate['provenance_sha256'] == input_sha,
            'posture candidate input digest mismatch')
        _require(_same(candidate, initial_receipt['candidate']) and value_sha(candidate) == initial_receipt['candidateKey'],
            'posture candidate identity changed')
        initial = inputs['initialCoupledReference']
        _require(_same(plan['com']['start'], inputs['sourceTimeS']), 'posture starts at different bound sample')
    start = float(plan['com']['start']); duration = float(plan['com']['duration'])
    _require(math.isfinite(start) and start >= 0. and _same(duration, DURATION), 'wrong frozen posture clock')
    result = dict(start=start, duration=duration)
    for name, width in (('com', 3), ('lower_joints', 15)):
        incoming, endpoint = _jet(initial[name], width), _jet(target[name], width)
        _require(max(np.max(abs(x)) for x in incoming[1:]+endpoint[1:]) <= 1e-10,
            'minimal inherited posture requires stationary COM/joint endpoint jets')
        result[name] = _segment(plan[name], incoming, endpoint, start, duration)
    for name, body in (('thorax', 1), ('pelvis', 4)):
        path = plan[name]
        _require(path['body'] == body, 'wrong frozen posture heading body')
        _require(max(np.max(abs(_array(target[name][k], (3,)))) for k in ('velocity', 'acceleration')) <= 1e-10,
            'nonstationary candidate orientation')
        if name not in initial:
            # Recover the orientation only for internal per-context arithmetic;
            # the static boundary verifies it against the bound initial receipt.
            angle_raw = path['angles']['initial']; angles, rates, acc = _jet(angle_raw, 3)
            initial[name] = _orientation(body, (angles, rates, acc), 4.)
        coefficients, incoming, endpoint = _heading_coefficients(initial[name], target[name], body, duration)
        _segment(path['angles'], incoming, endpoint, start, duration)
        result[name] = coefficients
    limits = _array(plan['joint_limits'], (21, 2))
    _require(np.all(limits[:, 0] < limits[:, 1]), 'invalid original posture joint limits')
    q = _jet(initial['lower_joints'], 15)[0]; endpoint = _jet(target['lower_joints'], 15)[0]
    existing = np.minimum(q-limits[:15, 0], limits[:15, 1]-q)
    required = np.full(15, .02); required[[7, 13]] = .06
    floor = np.minimum(existing, required); bounds = _bounds(result['lower_joints'], duration)
    _require(np.min(existing) >= .02-1e-10, 'inherited posture lacks original .02 reserve')
    _require(np.all(np.minimum(endpoint-limits[:15, 0], limits[:15, 1]-endpoint) >= required-1e-10),
        'posture endpoint lacks reserved joints/ankles')
    _require(np.all(bounds[:, 0] >= limits[:15, 0]+floor-1e-10) and
        np.all(bounds[:, 1] <= limits[:15, 1]-floor+1e-10), 'whole posture path decreases inherited reserve')
    _require(_same(plan['lumbar_anchor'], q[:3]) and _same(plan['lumbar_change_limit'], math.radians(5.)),
        'changed inherited lumbar anchor/cap')
    _require(np.max(np.maximum(abs(bounds[:3, 0]-q[:3]), abs(bounds[:3, 1]-q[:3]))) <= math.radians(5.)+1e-10,
        'posture path exceeds anchored lumbar cap')
    _require(_maximum_norm(result['com'], duration, 1) <= SPEED_CAP+1e-10 and
        _maximum_norm(result['com'], duration, 2) <= ACCELERATION_CAP+1e-10 and
        _bounds(result['com'], duration)[2, 0] > 0., 'posture COM exceeds frozen envelope')
    result['inheritedTransition'] = dict(policy='explicit-inherited-reference-to-reserved-endpoint-v1',
        initialReserveRad=existing.tolist(), wholePathReferenceReserveFloorRad=floor.tolist(),
        endpointRequiredReserveRad=required.tolist(), genericWholePath06GuardUnchanged=True,
        nativePhysicalGatesUnchanged=True)
    return result


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def pelvis_reference(model_data, joint_jet, thorax):
    """Three-lumbar rotational FK, including analytic world angular derivatives."""
    R = _rotation(thorax['rotation']); w = _array(thorax['velocity'], (3,)); a = _array(thorax['acceleration'], (3,))
    matrices = (P.T@R@P, P.T@_skew(w)@R@P, P.T@(_skew(a)+_skew(w)@_skew(w))@R@P)
    q, qd, qdd = joint_jet
    rows = {row['id']: row for row in model_data['segments']}
    chain = []; name = 'pelvis'
    while name != 'torso':
        row = rows[name]; chain.append(row); name = row['parent']
    _require([row['id'] for row in reversed(chain)] == ['@abdomen_z', 'lower_waist', 'pelvis'],
        'unexpected original lumbar chain')
    for row in reversed(chain):
        profile = row['jointProfile']; i = row['reference']['hinge']['qposIndex']
        _require(i in (0, 1, 2), 'unexpected lumbar q-index')
        frame = lambda x: Rotation.from_quat([x['rotation'][k] for k in 'xyzw']).as_matrix()
        parent, child = frame(profile['parentFrame']), frame(profile['childFrame']).T
        c, s = math.cos(q[i]), math.sin(q[i])
        A = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
        D = np.array([[0., 0., 0.], [0., -s, -c], [0., c, -s]])
        H = np.array([[0., 0., 0.], [0., -c, s], [0., -s, -c]])
        A, D, H = parent@A@child, parent@(D*qd[i])@child, parent@(H*qd[i]**2+D*qdd[i])@child
        X, Y, Z = matrices; matrices = (X@A, Y@A+X@D, Z@A+2*Y@D+X@H)
    R, dR, ddR = [P@M@P.T for M in matrices]
    W = dR@R.T; A = ddR@R.T-W@W
    vee = lambda M: np.array([M[2, 1]-M[1, 2], M[0, 2]-M[2, 0], M[1, 0]-M[0, 1]])/2.
    return dict(body=4, rotation=R.reshape(-1).tolist(), velocity=vee(W).tolist(), acceleration=vee(A).tolist())


def _frame(raw):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([raw['rotation'][k] for k in 'xyzw']).as_matrix()
    out[:3, 3] = [raw['anchor'][k] for k in 'xyz']
    return out


def _positions(model_data, q, root):
    poses = {'torso': root}
    for row in model_data['segments']:
        if row['id'] == 'torso':
            continue
        profile = row['jointProfile']; hinge = (row.get('reference') or {}).get('hinge')
        motion = np.eye(4)
        if hinge:
            motion[:3, :3] = Rotation.from_rotvec([q[hinge['qposIndex']], 0., 0.]).as_matrix()
        poses[row['id']] = poses[row['parent']]@_frame(profile['parentFrame'])@motion@np.linalg.inv(_frame(profile['childFrame']))
    return poses


def _static_endpoint_checks(receipt, model_data):
    inputs = receipt['input']; target = receipt['candidate']['target']
    q = _array(receipt['fullFutureMappedJointReferenceRad'], (21,))
    feet = inputs['activeFootReferences']
    _require(len(feet) == 2 and _same(q[:15], target['lower_joints']['position']),
        'full future joints disagree with posture target')
    fixed = np.eye(4); fixed[:3, :3] = P.T@_rotation(feet[0]['orientation']['rotation'])@P
    fixed[:3, 3] = P.T@_jet(feet[0]['translation'], 3)[0]
    relative = _positions(model_data, q, np.eye(4))
    root = fixed@np.linalg.inv(relative['left_foot']); poses = _positions(model_data, q, root)
    mass = float(model_data['totalMassKg'])
    com = sum(float(row['massKg'])*(poses[row['id']]@np.r_[[row['goldenMass']['centerOfMass'][k]
        for k in 'xyz'], 1.])[:3] for row in model_data['segments'])/mass
    rotations = [P@poses[name][:3, :3]@P.T for name in ('torso', 'pelvis')]
    check_feet = True
    for name, foot in zip(('left_foot', 'right_foot'), feet):
        check_feet = check_feet and np.linalg.norm(P@poses[name][:3, 3]-_jet(foot['translation'], 3)[0]) <= 1e-6
        check_feet = check_feet and Rotation.from_matrix((P@poses[name][:3, :3]@P.T)@
            _rotation(foot['orientation']['rotation']).T).magnitude() <= 1e-6
    initial = inputs['initialCoupledReference']; old_angles = _chart(_rotation(initial['thorax']['rotation']))
    new_angles = _chart(rotations[0]); old_pelvis = _rotation(initial['pelvis']['rotation'])
    active = _array(inputs['activeBaselineJointRad'], (21,))
    return dict(independentEndpointFullJointFK=all(_same(target[name]['rotation'], R.reshape(-1))
            for name, R in zip(('thorax', 'pelvis'), rotations)) and _same(target['com']['position'], P@com),
        independentBothFrozenFootEndpoints=check_feet,
        independentStaticPostureRestrictions=np.max(abs(q[:15]-active[:15])) <= .18+1e-10
            and np.max(abs(q[:3]-active[:3])) <= math.radians(5.)+1e-10
            and abs((P@com)[2]-_jet(initial['com'], 3)[0][2]) <= 1e-6
            and new_angles[1] >= old_angles[1]+math.radians(1.)-1e-8
            and abs(new_angles[2]) <= abs(old_angles[2])+1e-8
            and math.acos(float(np.clip(rotations[1][2, 2], -1., 1.))) <=
                math.acos(float(np.clip(old_pelvis[2, 2], -1., 1.)))+1e-8)


def _whole_support_margin(support, coefficients, duration):
    loaded = np.asarray(support['loaded_points_xy'], dtype=float)
    corners = _array(support['stance_points_xyz'], (4, 3)); ids = support['surviving_corners']
    _require(loaded.ndim == 2 and loaded.shape[1] == 2 and np.isfinite(loaded).all() and len(loaded) >= 3,
        'invalid bound current loaded hull')
    _require(support['require_triangle'] is True and len(ids) == 3 and len(set(ids)) == 3
        and all(type(i) is int and i in range(4) for i in ids)
        and len(support['present_corners']) == 4 and all(support['present_corners'][i] is True for i in ids),
        'invalid bound current provisional support triangle')
    equations = np.vstack([ConvexHull(np.unique(loaded, axis=0)).equations,
        ConvexHull(corners[ids, :2]).equations])
    lo, hi = _bounds(coefficients, duration)[2]
    _require(lo > 0., 'invalid whole-path capture height')
    taus = [math.sqrt(lo/9.81), math.sqrt(hi/9.81)]
    ps = [np.polynomial.Polynomial(column) for column in coefficients.T]
    margin = math.inf
    for nx, ny, offset in equations:
        face = nx*ps[0]+ny*ps[1]+offset
        # The capture expression is affine in tau. Both height-bound taus
        # safely cover sqrt(z(u)/g), including nonconstant-height C2 paths.
        for poly in [face]+[face+tau*face.deriv()/duration for tau in taus]:
            points = [0., 1.]+[float(r.real) for r in poly.deriv().roots()
                if abs(r.imag) < 1e-10 and 0. < r.real < 1.]
            margin = min(margin, -max(float(poly(x)) for x in points))
    return margin


def audit_posture_static_receipt(receipt, *, expected_profile=PROFILE, model_data=None, joint_limits=None):
    """Check frozen inputs/path math; never replay native/LP feasibility claims.

    joint_limits, when supplied, must be independently read by the caller from
    the bound executed XML. This preserves original XML hip expansions rather
    than accidentally comparing them to the narrower source-model JSON limits.
    """
    checks = {}
    def check(name, condition):
        checks[name] = bool(condition)
    try:
        check('explicitV3PostureScope', expected_profile == PROFILE and receipt.get('profile', PROFILE) == PROFILE)
        check('explicitSingleStaticFamily', receipt['schema'] == 1 and receipt['version'] == 'current-reference-single-family-v1'
            and receipt['accepted'] is True and receipt['staticEndpointAdmitted'] is True and receipt['rejectionReasons'] == []
            and all(receipt.get(k) is True for k in ('oneBoundedFamily', 'noFallback', 'noParameterScan',
                'noNativeIntegration', 'noLiveStateInstall', 'initialJetsComeFromPreviousEffectiveContext',
                'changedEffectiveTasksRequireFreshRiskHistory', 'genericWholePath06GuardUnchanged',
                'actualInputAndHistoryUnchanged')) and receipt['maximumIterations'] == 100
            and receipt['physicsIntegrations'] == 0 and receipt['fullPhysicalPathProven'] is False
            and receipt['riskHistoryAdmissionInherited'] is False and receipt['originalAdmissionInheritedForPosture'] is False
            and receipt['automaticReplanningImplemented'] is False)
        inputs = receipt['input']; initial = inputs['initialCoupledReference']; plan = receipt['frozenPlan']
        check('boundStaticInputDigest', receipt['inputSHA256'] == value_sha(inputs))
        check('unchangedStaticFamilyPolicy', _same(receipt['policy'], STATIC_POLICY))
        check('boundStaticNativeSample', inputs['phase'] == 'transfer'
            and type(inputs['sourceStep']) is int and inputs['sourceStep'] >= 0
            and _same(inputs['sourceTimeS'], receipt['requestedSourceTimeS'])
            and inputs['sourceStep'] == receipt['requestedSourceStep'])
        check('boundInitialCompleteCOM', _same(initial['com'], inputs['activeCompleteCOM']))
        check('boundInitialLowerJoints', _same(initial['lower_joints']['position'], inputs['activeBaselineJointRad'][:15]))
        bodies = inputs['activeBodyReferences']
        for name, body in (('thorax', 1), ('pelvis', 4)):
            raw = bodies.get(str(body))
            if raw is None and body == 4:
                _require(model_data is not None, 'model data required for inherited pelvis reconstruction')
                raw = pelvis_reference(model_data, _jet(initial['lower_joints'], 15), initial['thorax'])
            check('boundInitial'+name.title(), raw is not None and all(_same(initial[name][k], raw[k])
                for k in ('body', 'rotation', 'velocity', 'acceleration')))
        rebuilt = validate_posture_plan(plan, receipt)
        check('independentFrozenPosturePaths', True)
        support = inputs['support']
        check('boundCurrentStaticSupportSample', support['source_step'] == inputs['sourceStep']
            and _same(support['source_time'], inputs['sourceTimeS']) and support['step_index'] == inputs['stepIndex']
            and support['phase'] == inputs['phase'])
        check('independentWholeCurrentHullAndProvisionalTriangle3mm',
            _whole_support_margin(support, rebuilt['com'], rebuilt['duration']) >= .003-1e-10)
        check('explicitInheritedReserveTransition', _same(receipt['inheritedTransition'], rebuilt['inheritedTransition']))
        if model_data is not None:
            limits = np.zeros((21, 2))
            for row in model_data['segments']:
                hinge = (row.get('reference') or {}).get('hinge')
                if hinge:
                    axis = row['jointProfile']['axes'][0]
                    limits[hinge['qposIndex']] = [axis['minRadians'], axis['maxRadians']]
            if joint_limits is not None:
                limits = _array(joint_limits, (21, 2))
                _require(np.all(limits[:, 0] < limits[:, 1]), 'invalid bound executed joint limits')
            check('originalModelJointLimits', _same(plan['joint_limits'], limits))
            for name, result in _static_endpoint_checks(receipt, model_data).items():
                check(name, result)
        check('staticSupportSnapshotDigest', inputs['supportSHA256'] == value_sha(inputs['support']))
    except (KeyError, TypeError, ValueError, IndexError, ArithmeticError, QhullError):
        checks['completeFinitePostureStaticEvidence'] = False
    return dict(passed=bool(checks) and all(checks.values()), checks=checks,
        failedChecks=[k for k, value in checks.items() if not value], nativeFeasibilityIndependentlyReplayed=False,
        physicsIntegrations=0)


def _return_references(config, now, elapsed, phase, rebuilt):
    ret = config['coupledPosture'].get('originalRestReturn')
    if phase not in ('settle', 'stand'):
        _require(ret is None, 'premature posture rest return')
        return None
    _require(isinstance(ret, dict), 'missing posture rest return')
    start, duration = float(ret['start']), float(ret['duration'])
    _require(_same(duration, config['phase_durations']['settle']) and
        (phase != 'settle' or _same(start, now-elapsed)), 'changed posture return clock')
    incoming = _evaluate(rebuilt['lower_joints'], rebuilt['start'], rebuilt['duration'], start)
    final = _jet(ret['lower']['final'], 15)
    _require(_same(final, _rest(final[0])), 'posture rest-return terminal jets changed')
    if phase == 'settle':
        _require(_same(final[0], _array(config['q0'], (21,))[:15]), 'posture return uses different measured settle q0')
    else:
        # The inherited settle→stand handoff remeasures q0. The already frozen
        # return must keep its original settle goal rather than follow that reset.
        _require(now >= start+duration-1e-10, 'stand precedes completed posture return')
    lower = _segment(ret['lower'], incoming, final, start, duration)
    _require([arc['body'] for arc in ret['arcs']] == [1, 4], 'wrong posture return bodies')
    fade = _evaluate(hermite_coefficients(_rest(np.zeros(1)), _rest(np.ones(1)), duration), start, duration, now)
    rows = []
    for arc, name in zip(ret['arcs'], ('thorax', 'pelvis')):
        initial = _orientation(arc['body'], _evaluate(rebuilt[name], rebuilt['start'], rebuilt['duration'], start), 4.)
        R = _rotation(initial['rotation']); axis = _array(arc['axis'], (3,))
        _require(_same(arc['initialRotation'], R) and
            np.max(np.abs(initial['velocity'])) <= 1e-10 and np.max(np.abs(initial['acceleration'])) <= 1e-10,
            'posture return initial orientation changed')
        rows.append(dict(body=arc['body'], rotation=(Rotation.from_rotvec(axis*fade[0][0]).as_matrix()@R).reshape(-1).tolist(),
            velocity=(axis*fade[1][0]).tolist(), acceleration=(axis*fade[2][0]).tolist(), kp=40., kd=12., weight=4.))
    return _evaluate(lower, start, duration, now), rows


def audit_posture_return(config, causal, elapsed, phase, model_data):
    """Bind SO(3) rest-return targets to rootR and the SAME lower-return q0."""
    try:
        coupled = config['coupledPosture']; rebuilt = validate_posture_plan(coupled['plan'])
        _return_references(config, float(causal['arm']['nativeSourceTimeS']), elapsed, phase, rebuilt)
        if phase in ('settle', 'stand'):
            ret = coupled['originalRestReturn']; root = _array(config['rootR'], (3, 3))
            thorax = dict(body=1, rotation=root.reshape(-1).tolist(), velocity=[0.]*3, acceleration=[0.]*3)
            frozen_q = np.r_[_jet(ret['lower']['final'], 15)[0], np.zeros(6)]
            pelvis = pelvis_reference(model_data, _rest(frozen_q), thorax)
            for arc, target in zip(ret['arcs'], (thorax, pelvis)):
                R = _array(arc['initialRotation'], (3, 3)); target_R = _rotation(target['rotation'])
                _require(_same(arc['axis'], Rotation.from_matrix(target_R@R.T).as_rotvec()),
                    'posture return does not target same frozen q0/rootR')
        return dict(passed=True, checks=dict(independentSameQ0FixedWorldAxisReturn=True))
    except (KeyError, TypeError, ValueError, IndexError, ArithmeticError):
        return dict(passed=False, checks=dict(independentSameQ0FixedWorldAxisReturn=False))


def posture_reference_jets(config, causal, elapsed, phase, baseline_com_jet,
                           baseline_joint_jet, baseline_orientations):
    """Replay changed refs, leaving independently audited later complete COM intact.

    First reconstruct ordinary references with config.com1 replaced by the
    returned/derivable candidate target COM minus completedBaselineCorrectionM.
    Pass those complete COM/joint/orientation jets here. In transfer the frozen
    posture COM replaces that ordinary COM, so the old correction is used once.
    Bounded-settle segment arithmetic and causal admission are separate checks.
    """
    if config.get('coupledPosture') is None:
        return dict(com=baseline_com_jet, joints=baseline_joint_jet, orientations=baseline_orientations,
            com1=_array(config['com1'], (3,)))
    coupled = config['coupledPosture']; plan = coupled['plan']
    _require(coupled['profile'] == PROFILE, 'posture requires explicit v3 profile')
    rebuilt = validate_posture_plan(plan)
    _require(_same(coupled['inheritedTransition'], rebuilt['inheritedTransition']) and
        coupled['originalAdmissionInheritedForPosture'] is False, 'changed inherited posture policy')
    _require(coupled['frozenCandidateInputSHA256'] == plan['candidate']['provenance_sha256'],
        'context candidate provenance changed')
    delta = _array(coupled['completedBaselineCorrectionM'], (3,))
    correction = causal['robustCorrection']
    _require(correction['selected'] is True and _same(delta, correction['deltaM']), 'completed baseline correction changed')
    com1 = _jet(plan['candidate']['target']['com'], 3)[0]-delta
    _require(_same(config['com1'], com1), 'posture adjusted base COM double counts old correction')
    now = float(causal['arm']['nativeSourceTimeS']); start, duration = rebuilt['start'], rebuilt['duration']
    com = _evaluate(rebuilt['com'], start, duration, now) if phase == 'transfer' else baseline_com_jet
    lower = _evaluate(rebuilt['lower_joints'], start, duration, now)
    fade = _evaluate(hermite_coefficients(_rest(np.zeros(1)), _rest(np.ones(1)), duration), start, duration, now)[0][0]
    rows = [_orientation(body, _evaluate(rebuilt[name], start, duration, now), weight)
        for name, body, weight in (('thorax', 1, 4.), ('pelvis', 4, 4.*fade))]
    returning = _return_references(config, now, elapsed, phase, rebuilt)
    if returning is not None:
        lower, rows = returning; rows[1]['weight'] = 4.*fade
    _require(_same(coupled['pelvisWeight'], 4.*fade), 'changed C2 pelvis task weight')
    joints = tuple(np.r_[a, np.asarray(b)[15:]] for a, b in zip(lower, baseline_joint_jet))
    orientations = [copy.deepcopy(row) for row in baseline_orientations if row['body'] not in (1, 4)]+rows
    clock = np.array([1., 0., 0.]) if phase == 'transfer' else np.asarray(correction['clock'], dtype=float)
    return dict(com=com, joints=joints, orientations=orientations, com1=com1, correctionClock=clock)
