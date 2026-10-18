"""Pure causal quintic-overlap envelope certificate; never samples a time grid.

The binary floating inputs are treated as exact rationals. Piecewise squared
speed/acceleration polynomials have stationary degrees at most seven/five.
Rational Sturm sequences isolate every real stationary root, including repeated
roots. Exact rational interval bounds decide admission conservatively; stable
quintic jets at root midpoints provide the reported numerical maxima. No native
world, contact observation, controller mutation, or physical gate lives here.
"""
from fractions import Fraction as F
from functools import reduce
import math
from numbers import Real

import numpy as np

MAX_TOTAL_SPEED_MPS = .19260552331163003
MAX_TOTAL_ACCEL_MPS2 = .3901818436690813
MAX_CORRECTION_M = .012
MIN_CORRECTION_DURATION_S = 1.
MAX_CORRECTION_SPEED_MPS = .025
MAX_CORRECTION_ACCEL_MPS2 = .15
POLICY = 'causal-quintic-overlap-v1'
_ROOT_WIDTH = F(1, 2**64)


def _scalar(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError('invalid '+name)
    try:
        value = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError('invalid '+name) from error
    if not math.isfinite(value):
        raise ValueError('nonfinite '+name)
    return value


def _vector(value, name):
    try:
        values = tuple(value)
    except TypeError as error:
        raise ValueError('invalid '+name) from error
    if len(values) != 3:
        raise ValueError('invalid '+name+' dimension')
    return tuple(_scalar(x, name) for x in values)


def _trim(p):
    p = list(p)
    while len(p) > 1 and p[-1] == 0:
        p.pop()
    return p


def _add(a, b):
    return _trim([(a[i] if i < len(a) else 0)+(b[i] if i < len(b) else 0)
                  for i in range(max(len(a), len(b)))])


def _mul(a, b):
    out = [F(0)]*(len(a)+len(b)-1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            out[i+j] += x*y
    return _trim(out)


def _deriv(p):
    return _trim([i*p[i] for i in range(1, len(p))] or [F(0)])


def _eval(p, x):
    out = F(0)
    for c in reversed(p):
        out = out*x+c
    return out


def _primitive(p):
    p = _trim(p)
    denominator = math.lcm(*(F(x).denominator for x in p))
    out = [F(x).numerator*(denominator//F(x).denominator) for x in p]
    divisor = reduce(math.gcd, out, 0)
    return [x//divisor for x in out] if divisor else [0]


def _divide(a, b):
    a, b = [F(x) for x in a], [F(x) for x in b]
    quotient = [F(0)]*max(1, len(a)-len(b)+1)
    while a != [0] and len(a) >= len(b):
        scale = a[-1]/b[-1]; offset = len(a)-len(b)
        quotient[offset] += scale
        for i, x in enumerate(b):
            a[i+offset] -= scale*x
        a = _trim(a)
    return _trim(quotient), a


def _remainder(a, b):
    return _divide(a, b)[1]


def _sign_at(p, x):
    # Homogenized integer Horner evaluation avoids Fraction gcd operations at
    # every Sturm subdivision while preserving the exact sign.
    n, d = x.numerator, x.denominator
    value, power = p[-1], d
    for c in reversed(p[:-1]):
        value = value*n+c*power; power *= d
    return (value > 0)-(value < 0)


def _variation(sequence, x):
    signs = [s for p in sequence if (s := _sign_at(p, x))]
    return sum(a != b for a, b in zip(signs, signs[1:]))


def _stationary_intervals(squared_norm):
    p = _primitive(_deriv(squared_norm))
    # Endpoints are already candidates. Remove their complete multiplicities
    # before applying the open-interval Sturm count.
    while len(p) > 1 and p[0] == 0:
        p = p[1:]
    while len(p) > 1 and sum(p) == 0:
        quotient = [0]*(len(p)-1); quotient[-1] = p[-1]
        for i in range(len(p)-3, -1, -1):
            quotient[i] = p[i+1]+quotient[i+1]
        p = _primitive(quotient)
    if len(p) <= 1:
        return []
    # Square-free reduction makes endpoint/midpoint variation conventions
    # valid even when several extrema coincide or a norm vanishes repeatedly.
    a, b = p, _primitive(_deriv(p))
    while b != [0]:
        a, b = b, _primitive(_remainder(a, b))
    p = _primitive(_divide(p, a)[0])
    sequence = [p, _primitive(_deriv(p))]
    while sequence[-1] != [0]:
        remainder = _primitive([-x for x in _remainder(sequence[-2], sequence[-1])])
        if remainder == [0]:
            break
        sequence.append(remainder)
    found = []
    def isolate(lo, hi, vlo, vhi, depth=0):
        count = vlo-vhi
        if count == 0:
            return
        if count < 0 or depth > 256:
            raise ValueError('stationary-root isolation could not certify completeness')
        if count == 1 and hi-lo <= _ROOT_WIDTH:
            found.append((lo, hi)); return
        middle = (lo+hi)/2; vmid = _variation(sequence, middle)
        if _sign_at(p, middle) == 0:
            found.append((middle, middle))
            # Variation at an exact root takes its right-hand value. This
            # excludes the already-recorded root from the left subproblem.
            isolate(lo, middle, vlo, vmid+1, depth+1)
            isolate(middle, hi, vmid, vhi, depth+1)
        else:
            isolate(lo, middle, vlo, vmid, depth+1)
            isolate(middle, hi, vmid, vhi, depth+1)
    isolate(F(0), F(1), _variation(sequence, F(0)), _variation(sequence, F(1)))
    return sorted(found)


def _stationary_upper(p, lo, hi):
    # q'(root)=0. Taylor about the root gives a quadratic enclosure error,
    # much tighter than ordinary interval Horner near a frozen cap. Since the
    # unit interval is [0,1], sum(abs(q'' coefficients)) bounds |q''| there.
    middle = (lo+hi)/2
    curvature_bound = sum(abs(c) for c in _deriv(_deriv(p)))
    return max(F(0), _eval(p, middle)+curvature_bound*(hi-lo)**2/8)


def _piece_vectors(delta, duration, start, left, right):
    if not (start < (left+right)/2 < start+duration):
        return [[F(0)]]*3, [[F(0)]]*3
    u = [(left-start)/duration, (right-left)/duration]
    one_minus = _add([F(1)], [-x for x in u])
    speed = [30*x/duration for x in _mul(_mul(u, u), _mul(one_minus, one_minus))]
    acceleration = [60*x/duration**2 for x in _mul(_mul(u, one_minus),
                                                       _add([F(1)], [-2*x for x in u]))]
    return ([[d*x for x in speed] for d in delta],
            [[d*x for x in acceleration] for d in delta])


def _squared_sum(vectors):
    out = [F(0)]
    for vector in vectors:
        out = _add(out, _mul(vector, vector))
    return out


def _jets(delta, duration, start, t):
    if t <= start:
        return np.zeros(3), np.zeros(3), np.zeros(3)
    if t >= start+duration:
        return np.asarray(delta, dtype=float).copy(), np.zeros(3), np.zeros(3)
    u = (t-start)/duration
    # Positive derivative forms and a symmetric position near u=1 avoid
    # cancellation at the C2 entry/exit waits.
    f = lambda x: x**3*(10.+x*(-15.+6.*x))
    s = f(u) if u <= .5 else 1.-f(1.-u)
    ds = 30.*u*u*(1.-u)**2/duration
    dds = 60.*u*(1.-u)*(1.-2.*u)/duration**2
    d = np.asarray(delta, dtype=float)
    return d*s, d*ds, d*dds


def _upper_float(squared):
    try:
        if squared == 0:
            return 0.
        exponent = squared.numerator.bit_length()-squared.denominator.bit_length()
        exponent -= exponent % 2
        scaled = squared/F(2)**exponent
        value = math.ldexp(math.sqrt(float(scaled)), exponent//2)
    except OverflowError as error:
        raise ValueError('overlap envelope exceeds finite numeric range') from error
    if not math.isfinite(value):
        raise ValueError('overlap envelope exceeds finite numeric range')
    if squared == 0:
        return 0.
    # Prove the displayed square root is rounded upward too. This retains an
    # exact representable cap at equality and handles subnormal magnitudes.
    while F(value)**2 < squared:
        value = math.nextafter(value, math.inf)
        if not math.isfinite(value):
            raise ValueError('overlap envelope exceeds finite numeric range')
    return value


def certify_overlap(nominal_delta3, nominal_duration, correction_delta3,
                    correction_start, correction_duration):
    """Return finite maxima/candidates and a conservative frozen-cap decision.

    There is no grid search. Roots cover all piecewise stationary candidates;
    the exact rational upper bound, not a rounded sampled maximum, gates passed.
    Finite envelope violations return passed=False. Malformed/nonfinite or
    nonpositive durations and negative start clocks raise ValueError.
    """
    nominal = _vector(nominal_delta3, 'nominal delta')
    correction = _vector(correction_delta3, 'correction delta')
    T = _scalar(nominal_duration, 'nominal duration')
    start = _scalar(correction_start, 'correction start')
    duration = _scalar(correction_duration, 'correction duration')
    if T <= 0. or duration <= 0. or start < 0.:
        raise ValueError('invalid positive duration or causal start')
    if not math.isfinite(start+duration) or start+duration <= start:
        raise ValueError('unrepresentable correction interval')
    n, c = tuple(map(F, nominal)), tuple(map(F, correction))
    tnominal, tstart, tcorr = F(T), F(start), F(duration)
    boundaries = sorted({F(0), tnominal, tstart, tstart+tcorr})
    candidate_times = {'speed': set(boundaries), 'acceleration': set(boundaries)}
    bounds = {'speed': F(0), 'acceleration': F(0)}
    lower_bounds = {'speed': F(0), 'acceleration': F(0)}
    windows = []
    for left, right in zip(boundaries, boundaries[1:]):
        nv, na = _piece_vectors(n, tnominal, F(0), left, right)
        cv, ca = _piece_vectors(c, tcorr, tstart, left, right)
        squared = dict(speed=_squared_sum([_add(x, y) for x, y in zip(nv, cv)]),
                       acceleration=_squared_sum([_add(x, y) for x, y in zip(na, ca)]))
        window = dict(startS=float(left), endS=float(right),
            nominalActive=bool(F(0) < (left+right)/2 < tnominal),
            correctionActive=bool(tstart < (left+right)/2 < tstart+tcorr))
        for name, poly in squared.items():
            roots = _stationary_intervals(poly)
            times = [left+(right-left)*(lo+hi)/2 for lo, hi in roots]
            candidate_times[name].update(times)
            upper = max([_eval(poly, F(0)), _eval(poly, F(1))]+
                        [_stationary_upper(poly, lo, hi) for lo, hi in roots])
            bounds[name] = max(bounds[name], upper)
            lower_bounds[name] = max([lower_bounds[name], _eval(poly, F(0)), _eval(poly, F(1))]+
                                    [_eval(poly, (lo+hi)/2) for lo, hi in roots])
            window[name+'StationaryTimesS'] = [float(t) for t in times]
            window[name+'StationaryRootCount'] = len(roots)
            window[name+'SquaredDerivativeDegree'] = len(_trim(_deriv(poly)))-1
            window[name+'UpperBound'] = _upper_float(upper)
        windows.append(window)
    def total_jets(t):
        a, b = _jets(nominal, T, 0., float(t)), _jets(correction, duration, start, float(t))
        with np.errstate(over='ignore', invalid='ignore'):
            result = tuple(x+y for x, y in zip(a, b))
        if not all(np.isfinite(x).all() for x in result):
            raise ValueError('overlap jets exceed finite numeric range')
        return result
    maxima, maximizers = {}, {}
    for name, index in (('speed', 1), ('acceleration', 2)):
        values = [(math.hypot(*total_jets(t)[index]), float(t)) for t in sorted(candidate_times[name])]
        value, time = max(values)
        maxima[name], maximizers[name] = value, time
    corr_norm2 = sum(x*x for x in c)
    corr_speed2 = F(15, 8)**2*corr_norm2/tcorr**2
    corr_accel2 = F(100, 3)*corr_norm2/tcorr**4
    checks = dict(correctionDuration=tcorr >= F(MIN_CORRECTION_DURATION_S),
        correctionMagnitude=corr_norm2 <= F(MAX_CORRECTION_M)**2,
        correctionSpeed=corr_speed2 <= F(MAX_CORRECTION_SPEED_MPS)**2,
        correctionAcceleration=corr_accel2 <= F(MAX_CORRECTION_ACCEL_MPS2)**2,
        totalSpeed=bounds['speed'] <= F(MAX_TOTAL_SPEED_MPS)**2,
        totalAcceleration=bounds['acceleration'] <= F(MAX_TOTAL_ACCEL_MPS2)**2)
    ambiguous = [name for name, cap in (('speed', MAX_TOTAL_SPEED_MPS),
                 ('acceleration', MAX_TOTAL_ACCEL_MPS2))
                 if lower_bounds[name] <= F(cap)**2 < bounds[name]]
    result = dict(schema=1, policy=POLICY, method='exact-rational-polynomial-sturm-interval-bounds',
        nominal=dict(deltaM=list(nominal), startS=0., durationS=T),
        correction=dict(deltaM=list(correction), startS=start, durationS=duration,
            magnitudeM=_upper_float(corr_norm2), maximumSpeedMps=_upper_float(corr_speed2),
            maximumAccelerationMps2=_upper_float(corr_accel2)),
        caps=dict(totalSpeedMps=MAX_TOTAL_SPEED_MPS, totalAccelerationMps2=MAX_TOTAL_ACCEL_MPS2,
            correctionMagnitudeM=MAX_CORRECTION_M, minimumCorrectionDurationS=MIN_CORRECTION_DURATION_S,
            correctionSpeedMps=MAX_CORRECTION_SPEED_MPS, correctionAccelerationMps2=MAX_CORRECTION_ACCEL_MPS2),
        maximumTotalSpeedMps=maxima['speed'], maximumTotalAccelerationMps2=maxima['acceleration'],
        maximumSpeedAtS=maximizers['speed'], maximumAccelerationAtS=maximizers['acceleration'],
        conservativeMaximumTotalSpeedMps=_upper_float(bounds['speed']),
        conservativeMaximumTotalAccelerationMps2=_upper_float(bounds['acceleration']),
        candidateCriticalTimesS={name: [float(t) for t in sorted(times)]
                                 for name, times in candidate_times.items()},
        windows=windows,
        boundaryJets=[dict(timeS=float(t), positionM=total_jets(t)[0].tolist(),
            velocityMps=total_jets(t)[1].tolist(), accelerationMps2=total_jets(t)[2].tolist())
            for t in boundaries],
        checks=checks, passed=all(checks.values()), rejections=[name for name, ok in checks.items() if not ok],
        unresolvedCapBounds=ambiguous)
    return result
