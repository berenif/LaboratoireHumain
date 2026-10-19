"""Independent saved-data audit of the reviewed 1.52 s bounded COM settle.

No producer, native engine, controller, optimizer or physics is imported. The
bound endpoints and current support must come from the caller's event records;
self-consistent receipt claims alone cannot qualify a trajectory. ``expected``
is an independently reconstructed source-time evaluator, not a saved evaluator.
"""
import hashlib
import json
import math
from decimal import Decimal, localcontext

import numpy as np
from scipy.spatial import ConvexHull, QhullError

DURATION = 1.52
MAX_SPEED = .19260552331163003
MAX_ACCELERATION = .3901818436690813
RESERVE = .003
ZERO_JET_TOLERANCE = 1e-12
ROUNDING_TOLERANCE = 64 * np.finfo(float).eps
VERSION = 'bounded-c2-settle-v1'


def _array(value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('invalid finite bound input')
    return a


def _close(actual, expected, tolerance=3e-12):
    """Require complete finite numeric structure, including zero-valued fields."""
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and actual.keys() == expected.keys()
                and all(_close(actual[k], v, tolerance) for k, v in expected.items()))
    if isinstance(expected, (list, tuple, np.ndarray)):
        return (isinstance(actual, (list, tuple, np.ndarray)) and len(actual) == len(expected)
                and all(_close(a, b, tolerance) for a, b in zip(actual, expected)))
    if isinstance(expected, (bool, str)) or expected is None:
        return type(actual) is type(expected) and actual == expected
    return (isinstance(actual, (int, float, np.number)) and not isinstance(actual, bool)
            and math.isfinite(float(actual))
            and abs(float(actual)-float(expected)) <= tolerance * max(1., abs(float(expected))))


def _differentiate(c, order=1):
    c = np.asarray(c, dtype=float)
    for _ in range(order):
        c = np.arange(1, len(c)) * c[1:]
        if not len(c):
            c = np.zeros(1)
    return c


def _value(c, t):
    return np.polynomial.polynomial.polyval(t, c)


def _jet(c, t):
    c = np.asarray(c)
    return tuple(np.array([_value(_differentiate(c[:, axis], order), t)
                           for axis in range(3)]) for order in range(3))


def _bounds(c, duration):
    c = np.asarray(c, dtype=float)
    roots = np.polynomial.polynomial.polyroots(_differentiate(c))
    times = [0., float(duration)] + [float(r.real) for r in roots
        if abs(r.imag) <= 1e-9 * max(1., abs(r.real)) and 0. < r.real < duration]
    values = [_value(c, t) for t in times]
    if not np.isfinite(values).all():
        raise ValueError('nonfinite independently reconstructed bound')
    return dict(min=float(min(values)), max=float(max(values)), timesS=times)


def _enclose(c, duration):
    """Independent power-to-Bernstein enclosure, no samples or subdivisions."""
    c = np.asarray(c, dtype=float)
    result = _bounds(c, duration)
    degree = len(c)-1
    with localcontext() as context:
        context.prec = 90
        scaled = [Decimal.from_float(float(x))*Decimal.from_float(float(duration))**i
                  for i, x in enumerate(c)]
        bernstein = []
        for i in range(degree+1):
            bernstein.append(sum((scaled[k]*Decimal(math.factorial(i))*Decimal(math.factorial(degree-k))
                / Decimal(math.factorial(i-k))/Decimal(math.factorial(degree))
                for k in range(i+1)), Decimal(0)))
        cushion = max(Decimal(1), sum(abs(x) for x in scaled))*Decimal('1e-65')
        low = math.nextafter(float(min(bernstein)-cushion), -math.inf)
        high = math.nextafter(float(max(bernstein)+cushion), math.inf)
    result.update(conservativeMin=low, conservativeMax=high,
        enclosureMethod='outward Decimal80 Bernstein convex-hull;1e-65 weighted coefficient cushion')
    return result


def _norm(c, order, duration):
    square = np.zeros(1)
    for axis in range(3):
        d = _differentiate(np.asarray(c)[:, axis], order)
        square = np.polynomial.polynomial.polyadd(square, np.convolve(d, d))
    bound = _bounds(square, duration)
    return math.sqrt(max(0., bound['max'])), bound['timesS']


def _hull(points):
    raw = np.asarray(points, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 2 or len(raw) < 3 or not np.isfinite(raw).all():
        raise ValueError('invalid current actual support')
    unique = np.unique(raw, axis=0)
    vertices = unique[ConvexHull(unique).vertices]
    # Qhull supplies a CCW polygon; canonicalize the start independently.
    first = min(range(len(vertices)), key=lambda i: tuple(vertices[i]))
    vertices = np.roll(vertices, -first, axis=0)
    equations = []
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        delta = b-a
        n = np.array([delta[1], -delta[0]])/np.linalg.norm(delta)
        equations.append([*n, -float(n @ a)])
    return raw, vertices, np.asarray(equations)


def _reconstruct(initial, final, duration, start):
    """Hermite closed form and integrated jerk pulses, without producer code."""
    p, v, a = initial
    q, w, b = final
    residual = q-p-duration*v-.5*duration**2*a
    terminal_v = duration*(w-v)-duration**2*a
    terminal_a = duration**2*(b-a)
    normalized = np.array([p, duration*v, .5*duration**2*a,
        10*residual-4*terminal_v+.5*terminal_a,
        -15*residual+7*terminal_v-terminal_a,
        6*residual-3*terminal_v+.5*terminal_a])
    legacy_c = normalized/np.array([duration**i for i in range(6)])[:, None]
    legacy_speed = _norm(legacy_c, 1, duration)[0]
    legacy_acceleration = _norm(legacy_c, 2, duration)[0]
    shape = None
    if (legacy_speed <= MAX_SPEED+ROUNDING_TOLERANCE
            and legacy_acceleration <= MAX_ACCELERATION+ROUNDING_TOLERANCE):
        profile = 'legacy-quintic-exact-identity'
        segments = [dict(offsetS=0., durationS=duration, coefficients=legacy_c)]
        data = dict(startSourceTimeS=start, durationS=duration, coefficients=normalized.tolist())
    else:
        delta = q-p
        distance = float(np.linalg.norm(delta))
        if distance <= 0.:
            raise ValueError('invalid settle displacement')
        peak_v = min(MAX_SPEED, 2*distance/duration)
        accel_time = duration-distance/peak_v
        coast_time = duration-2*accel_time
        ramp = min(accel_time/2, accel_time-peak_v/MAX_ACCELERATION)
        if ramp <= 0. or coast_time < -ROUNDING_TOLERANCE:
            raise ValueError('bounded settle infeasible at frozen caps and duration')
        coast_time = max(0., coast_time)
        peak_a = peak_v/(accel_time-ramp)
        peak_j = peak_a/ramp
        durations = [ramp, accel_time-2*ramp, ramp, coast_time,
                     ramp, accel_time-2*ramp, ramp]
        jerks = [peak_j, 0., -peak_j, 0., -peak_j, 0., peak_j]
        if min(durations) < 0.:
            raise ValueError('negative segment length')
        boundaries = [0.]
        for length in durations:
            boundaries.append(boundaries[-1]+length)
        def scalar(t, order):
            power = 3-order
            return sum(j*(max(0., t-left)**power-max(0., t-right)**power)/math.factorial(power)
                       for j, left, right in zip(jerks, boundaries, boundaries[1:]))
        direction = delta/distance
        segments = [dict(offsetS=offset, durationS=length, coefficients=np.array([
            p+direction*scalar(offset, 0), direction*scalar(offset, 1),
            .5*direction*scalar(offset, 2), direction*jerk/6]))
            for offset, length, jerk in zip(boundaries, durations, jerks)]
        profile = 'seven-cubic-constant-jerk'
        data = dict(startSourceTimeS=start, durationS=duration,
            pieces=[dict(offsetS=s['offsetS'], durationS=s['durationS'],
                         coefficients=s['coefficients'].tolist()) for s in segments],
            initialPositionM=p.tolist(), finalPositionM=q.tolist())
        shape = dict(distanceM=distance, peakSpeedMps=peak_v, accelerationTimeS=accel_time,
            coastTimeS=coast_time, rampTimeS=ramp, peakAccelerationMps2=peak_a,
            peakJerkMps3=peak_j, pieceDurationsS=durations, scalarJerksMps3=jerks,
            integratedFinalScalarPositionM=scalar(duration, 0),
            integratedFinalScalarVelocityMps=scalar(duration, 1),
            integratedFinalScalarAccelerationMps2=scalar(duration, 2))
    def expected(now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('nonfinite evaluation time')
        elapsed = float(np.clip(now-start, 0., duration))
        if profile == 'seven-cubic-constant-jerk':
            if now <= start:
                return p.copy(), np.zeros(3), np.zeros(3)
            if now >= start+duration:
                return q.copy(), np.zeros(3), np.zeros(3)
        segment = next((s for s in segments if elapsed < s['offsetS']+s['durationS']), segments[-1])
        return _jet(segment['coefficients'], elapsed-segment['offsetS'])
    legacy = dict(startSourceTimeS=start, durationS=duration, coefficients=normalized.tolist())
    return profile, segments, data, legacy, legacy_speed, legacy_acceleration, shape, expected


def _certificate(segments, support_points):
    raw, vertices, equations = _hull(support_points)
    speed = acceleration = 0.
    position_margin = capture_margin = math.inf
    proofs = []
    for i, segment in enumerate(segments):
        c, duration = segment['coefficients'], segment['durationS']
        v, vt = _norm(c, 1, duration)
        a, at = _norm(c, 2, duration)
        speed, acceleration = max(speed, v), max(acceleration, a)
        height = _enclose(c[:, 2], duration)
        if height['conservativeMin'] <= 0.:
            raise ValueError('nonpositive whole-path COM height')
        with localcontext() as context:
            context.prec = 90
            g = Decimal.from_float(9.81)
            taus = [math.nextafter(float((Decimal.from_float(height['conservativeMin'])/g).sqrt()), -math.inf),
                    math.nextafter(float((Decimal.from_float(height['conservativeMax'])/g).sqrt()), math.inf)]
        faces = []
        for face, (nx, ny, offset) in enumerate(equations):
            position = nx*c[:, 0]+ny*c[:, 1]
            position[0] += offset
            pe = _enclose(position, duration)
            capture = [_enclose(np.polynomial.polynomial.polyadd(position,
                tau*_differentiate(position)), duration) for tau in taus]
            pm = -pe['conservativeMax']
            cm = -max(x['conservativeMax'] for x in capture)
            position_margin, capture_margin = min(position_margin, pm), min(capture_margin, cm)
            faces.append(dict(faceIndex=face, positionMinimumMarginM=pm,
                captureConservativeMinimumMarginM=cm, positionExtrema=pe, captureExtremaAtTauBounds=capture))
        proofs.append(dict(index=i, offsetS=segment['offsetS'], durationS=duration,
            maximumSpeedMps=v, speedExtremaTimesS=vt, maximumAccelerationMps2=a,
            accelerationExtremaTimesS=at, heightExtremaM=height, tauBoundsS=taus, faces=faces))
    return dict(method='all polynomial derivative roots/endpoints plus outward Bernstein enclosures; conservative per-piece tau interval',
        actualSupportPointsXY=raw.tolist(), hullVerticesXY=vertices.tolist(), normalizedHullEquations=equations.tolist(),
        actualPointsSHA256=hashlib.sha256(json.dumps(raw.tolist(), separators=(',', ':')).encode()).hexdigest(),
        reserveM=RESERVE, maximumSpeedCapMps=MAX_SPEED, maximumAccelerationCapMps2=MAX_ACCELERATION,
        certifiedMaximumSpeedMps=speed, certifiedMaximumAccelerationMps2=acceleration,
        minimumPositionMarginM=position_margin, conservativeMinimumCaptureMarginM=capture_margin,
        numericalRoundoffTolerance=ROUNDING_TOLERANCE, pieces=proofs,
        activePositionFaceIndices=sorted({f['faceIndex'] for k in proofs for f in k['faces']
            if abs(f['positionMinimumMarginM']-position_margin) < 1e-12}),
        activeCaptureFaceIndices=sorted({f['faceIndex'] for k in proofs for f in k['faces']
            if abs(f['captureConservativeMinimumMarginM']-capture_margin) < 1e-12}),
        currentActualSupportOnly=True, futureSupportClaim=False)


def _certificate_matches(actual, expected, key=''):
    # Repeated polynomial roots can split across a closed-interval boundary
    # under coefficient roundoff. Compare root locations as covered sets, rather
    # than trusting root multiplicity/order or accepting a sampled maximum.
    if key in ('timesS', 'speedExtremaTimesS', 'accelerationExtremaTimesS'):
        if not isinstance(actual, list) or not actual or not expected:
            return False
        if not all(isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t) for t in actual):
            return False
        return (min(actual) >= 0. and max(actual) <= max(expected)
                and 0. in actual and max(expected) in actual
                and all(min(abs(t-u) for u in expected) <= (3e-5 if min(abs(t), abs(t-max(expected))) <= 3e-5 else 2e-6) for t in actual)
                and all(min(abs(t-u) for u in actual) <= (3e-5 if min(abs(t), abs(t-max(expected))) <= 3e-5 else 2e-6) for t in expected))
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and actual.keys() == expected.keys()
                and all(_certificate_matches(actual[k], v, k) for k, v in expected.items()))
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(_certificate_matches(a, b) for a, b in zip(actual, expected)))
    return _close(actual, expected, 3e-11)


def audit_bounded_settle_receipt(receipt, initial, final, duration, support_points):
    """Return checks, independent segments/proof and ``expected(source_time)``.

    Invalid/missing receipts fail closed in the returned checks. The caller must
    separately bind the receipt's source start time and planning-native-step to
    the measured settle event; no event clock is inferred from this receipt.
    ``certificate`` and ``segments`` contain NumPy arrays and ``expected`` is a
    callable, so retain only checks/errors in a JSON audit summary.
    """
    checks, errors = {}, []
    result = dict(passed=False, checks=checks, errors=errors, expected=None,
                  segments=[], certificate=None)
    def check(name, condition):
        checks[name] = bool(condition)
    try:
        initial = _array(initial, (3, 3))
        final = _array(final, (3, 3))
        duration = float(duration)
        start = float(receipt['path']['startSourceTimeS'])
        check('boundedSettleFiniteBoundInputs', math.isfinite(start) and math.isfinite(duration)
              and duration > 0. and min(initial[0, 2], final[0, 2]) > 0.)
        check('boundedSettleZeroEndpointJets', np.max(np.abs(np.r_[initial[1:], final[1:]])) <= ZERO_JET_TOLERANCE)
        check('boundedSettleUnchangedDuration', duration == DURATION and receipt['path']['durationS'] == duration)
        if not all(checks.values()):
            return result
        profile, segments, data, legacy, lv, la, shape, expected = _reconstruct(initial, final, duration, start)
        result.update(expected=expected, segments=segments)
        check('boundedSettleSchemaVersion', type(receipt.get('schema')) is int and receipt.get('schema') == 1
              and receipt.get('version') == VERSION)
        check('boundedSettleReceiptJetsBound', _close(receipt.get('exactInitialJet'), initial.tolist(), 0.)
              and _close(receipt.get('exactFinalJet'), final.tolist(), 0.))
        check('boundedSettleProfileChosenIndependently', receipt.get('profile') == profile)
        check('boundedSettleLegacyReconstructed', _close(receipt.get('legacyQuintic'), legacy)
              and _close(receipt.get('legacyMaximumSpeedMps'), lv)
              and _close(receipt.get('legacyMaximumAccelerationMps2'), la))
        check('boundedSettlePathReconstructed', _close(receipt.get('path'), data))
        check('boundedSettleAnalyticalShapeReconstructed', _close(receipt.get('analyticalShape'), shape))
        check('boundedSettleProvenanceClaims', type(receipt.get('physicsIntegrations')) is int
              and receipt.get('physicsIntegrations') == 0 and type(receipt.get('optimizerCalls')) is int
              and receipt.get('optimizerCalls') == 0 and receipt.get('durationUnchanged') is True
              and receipt.get('measuredTargetUnchanged') is True and receipt.get('frozenCapsUnchanged') is True
              and receipt.get('endpointDerivativesCanonicalized') is (profile != 'legacy-quintic-exact-identity')
              and receipt.get('zeroDerivativeCanonicalizationTolerance') == ZERO_JET_TOLERANCE)
        joins = all(all(np.max(abs(a-b)) <= 1e-12 for a, b in zip(
            _jet(left['coefficients'], left['durationS']), _jet(right['coefficients'], 0.)))
            for left, right in zip(segments, segments[1:]))
        check('boundedSettleEndpointJetsAndC2', joins and all(
            np.max(abs(actual-bound)) <= 1e-12 for t, bound_jet in ((start, initial), (start+duration, final))
            for actual, bound in zip(expected(t), bound_jet)))
        certificate = _certificate(segments, support_points)
        result['certificate'] = certificate
        check('boundedSettleFrozenCaps', certificate['certifiedMaximumSpeedMps'] <= MAX_SPEED+ROUNDING_TOLERANCE
              and certificate['certifiedMaximumAccelerationMps2'] <= MAX_ACCELERATION+ROUNDING_TOLERANCE)
        check('boundedSettleWholePathHullAndCapture', min(certificate['minimumPositionMarginM'],
              certificate['conservativeMinimumCaptureMarginM']) >= RESERVE)
        declared = receipt.get('certificate', {})
        frozen_policy = dict(reserveM=RESERVE, maximumSpeedCapMps=MAX_SPEED,
            maximumAccelerationCapMps2=MAX_ACCELERATION, numericalRoundoffTolerance=ROUNDING_TOLERANCE,
            currentActualSupportOnly=True, futureSupportClaim=False)
        check('boundedSettleCertificateFrozenPolicy', isinstance(declared, dict)
              and all(_close(declared.get(k), v, 0.) for k, v in frozen_policy.items()))
        check('boundedSettleCurrentActualHull', isinstance(declared, dict)
              and _close(declared.get('actualSupportPointsXY'), certificate['actualSupportPointsXY'], 0.)
              and declared.get('actualPointsSHA256') == certificate['actualPointsSHA256'])
        check('boundedSettleCertificateRecomputed', _certificate_matches(declared, certificate))
        result['passed'] = all(checks.values())
    except (KeyError, TypeError, ValueError, OverflowError, IndexError, QhullError, np.linalg.LinAlgError) as error:
        check('boundedSettleReceiptWellFormed', False)
        errors.append(type(error).__name__+': '+str(error))
    return result
