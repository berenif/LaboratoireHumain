"""Causal C2 COM path planning inside current measured support, no rollout."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import ConvexHull
from source_robust_support_steps import actual_stance_patch

# Frozen six-step witness, reconstructed without integration. This envelope is
# never widened to admit an overlapping nominal and corrective reference.
MAX_TOTAL_SPEED_MPS = .19260552331163003
MAX_TOTAL_ACCEL_MPS2 = .3901818436690813
FORWARD_SHIFT_M = .010
PREFERRED_INWARD_SHIFT_M = .0025
RESERVE_M = .003


@dataclass(frozen=True)
class QuinticSegment:
    start: float
    duration: float
    coefficients: tuple

    @classmethod
    def between(cls, start, duration, initial, final):
        if not duration > 0.:
            raise ValueError('nonpositive COM segment duration')
        p, v, a = [np.asarray(x, dtype=float) for x in initial]
        q, w, b = [np.asarray(x, dtype=float) for x in final]
        low = np.array([p, v*duration, .5*a*duration**2])
        rhs = np.array([q-low.sum(axis=0), w*duration-low[1]-2*low[2],
            b*duration**2-2*low[2]])
        high = np.linalg.solve(np.array([[1.,1.,1.],[3.,4.,5.],[6.,12.,20.]]), rhs)
        coefficients = np.vstack([low, high])
        return cls(float(start), float(duration), tuple(tuple(float(x) for x in row) for row in coefficients))

    def evaluate(self, now):
        u = float(np.clip((now-self.start)/self.duration, 0., 1.)); c = np.array(self.coefficients)
        p = sum(c[i]*u**i for i in range(6))
        if now >= self.start+self.duration:
            # Exact endpoint evaluation retains tiny polynomial roundoff, but
            # planned endpoint derivatives are mathematically zero in this mode.
            v = sum(i*c[i] for i in range(1, 6))/self.duration
            a = sum(i*(i-1)*c[i] for i in range(2, 6))/self.duration**2
        else:
            v = sum(i*c[i]*u**(i-1) for i in range(1, 6))/self.duration
            a = sum(i*(i-1)*c[i]*u**(i-2) for i in range(2, 6))/self.duration**2
        return p, v, a

    def receipt(self):
        return dict(startSourceTimeS=self.start, durationS=self.duration,
            coefficients=[list(row) for row in self.coefficients])


def actual_equations(c, phase):
    w = c.w; count = w.l.lh_support_count(w.h)
    points = np.unique(w.arr(21, (128,))[:2*count].reshape(-1, 2), axis=0)
    equations = [ConvexHull(points).equations]
    if phase in ('translate', 'land'):
        corners, present, _ = actual_stance_patch(w, c.stance_side)
        if not c.robust_support.get('selected') or not all(present[c._robust_survivors]):
            raise ValueError('continuous COM requires current selected actual triangle')
        equations.append(ConvexHull(corners[c._robust_survivors, :2]).equations)
    return np.vstack(equations)


def plan_forward(c, phase_start_source):
    """Choose a lateral scalar once; never project live references or use future support."""
    origin = c.com1+getattr(c, '_robust_delta', np.zeros(3))
    heading = c.heading[:2]; left = np.array([-heading[1], heading[0]])
    equations = actual_equations(c, 'translate')
    T = c.phase_duration('translate')+c.phase_duration('land')
    # Exact extrema of f=s+tau*sdot for a fixed-height quintic. Solve polynomial
    # derivative rather than relying on an under-sampled admissibility grid.
    tau = math.sqrt(origin[2]/9.81)
    f = np.polynomial.Polynomial([0.,0.,0.,10.,-15.,6.])
    capture = f+tau/T*f.deriv()
    samples = [0.,1.]
    samples += [float(r.real) for r in capture.deriv().roots() if abs(r.imag)<1e-10 and 0.<r.real<1.]
    factors = [float(poly(u)) for poly in (f, capture) for u in samples]
    lower, upper = -.012, .012
    for normal_x, normal_y, offset in equations:
        normal = np.array([normal_x, normal_y])
        base = float(normal@origin[:2]+offset+RESERVE_M)
        for factor in factors:
            a = factor*float(normal@left)
            b = -base-factor*FORWARD_SHIFT_M*float(normal@heading)
            if abs(a)<1e-14:
                if b < -1e-12:
                    raise ValueError('no support-bounded continuous COM path')
            elif a>0.:
                upper = min(upper, b/a)
            else:
                lower = max(lower, b/a)
    if lower>upper+1e-12:
        raise ValueError('no support-bounded continuous COM path')
    # Inward here means away from the excluded corner into the selected patch.
    candidates = [PREFERRED_INWARD_SHIFT_M, -PREFERRED_INWARD_SHIFT_M]
    preferred = min(candidates, key=lambda x: abs(x-float(np.clip(x, lower, upper))))
    lateral = float(np.clip(preferred, lower, upper))
    delta = np.r_[FORWARD_SHIFT_M*heading+lateral*left, 0.]
    segment = QuinticSegment.between(phase_start_source, T,
        (np.zeros(3), np.zeros(3), np.zeros(3)), (delta, np.zeros(3), np.zeros(3)))
    return segment, dict(plannedAtSourceTimeS=c.w.l.lh_time(c.w.h),
        phaseStartSourceTimeS=phase_start_source, forwardM=FORWARD_SHIFT_M,
        lateralM=lateral, lateralIntervalM=[lower, upper], actualSupportEquations=equations.tolist(),
        reserveM=RESERVE_M, referenceCaptureProxy='constant planned COM height sqrt(z/g)',
        correctionBoundsUnchanged=True, frozenMaximumTotalSpeedMps=MAX_TOTAL_SPEED_MPS,
        frozenMaximumTotalAccelerationMps2=MAX_TOTAL_ACCEL_MPS2)


def project_endpoint(desired, A, b, radius):
    """Unique closest point in a frozen 2D half-plane/disk intersection.

    This is a single phase-entry reference plan, never a live projection. All
    possible active sets are finite in 2D: an interior point, one line, two
    lines, the circle, or a line/circle intersection. No parameter search is
    used. The disk is the unchanged summed-jet envelope for a zero-jet quintic.
    """
    desired, A, b = (np.asarray(x, dtype=float) for x in (desired, A, b))
    if A.shape == (0,):
        A = A.reshape(0, 2)
    radius = float(radius)
    if (desired.shape != (2,) or A.ndim != 2 or A.shape[1] != 2
            or b.shape != (len(A),) or not np.isfinite(desired).all()
            or not np.isfinite(A).all() or not np.isfinite(b).all()
            or not math.isfinite(radius) or radius <= 0.):
        raise ValueError('invalid frozen COM projection problem')
    with np.errstate(over='ignore', invalid='ignore'):
        if not math.isfinite(float(desired@desired)) or not math.isfinite(radius*radius):
            raise ValueError('COM projection exceeds finite quadratic range')
    normals, bounds = [], []
    for normal, bound in zip(A, b):
        scale = float(np.hypot(*normal))
        if not math.isfinite(scale):
            raise ValueError('ill-conditioned frozen COM projection constraint')
        if scale == 0.:
            if bound < 0.:
                raise ValueError('infeasible frozen COM projection constraint')
            continue
        normal, bound = normal/scale, float(bound/scale)
        if not np.isfinite(normal).all() or not math.isfinite(bound):
            raise ValueError('ill-conditioned frozen COM projection constraint')
        normals.append(normal); bounds.append(bound)
    A, b = np.asarray(normals).reshape(-1, 2), np.asarray(bounds)
    candidates = [desired.copy(), np.zeros(2)]
    norm = math.hypot(*desired)
    if norm > 0.:
        candidates.append(desired*radius/norm)
    for i, (normal, bound) in enumerate(zip(A, b)):
        n2 = float(normal@normal)
        candidates.append(desired-normal*((normal@desired-bound)/n2))
        centre = normal*(bound/n2)
        remainder = radius**2-float(centre@centre) if abs(bound) <= radius+1e-12 else -math.inf
        if remainder >= -1e-15:
            tangent = np.array([-normal[1], normal[0]])/math.sqrt(n2)
            offset = math.sqrt(max(0., remainder))*tangent
            candidates.extend((centre+offset, centre-offset))
        for j in range(i):
            pair = np.array([normal, A[j]])
            if abs(float(np.linalg.det(pair))) > 1e-14:
                candidates.append(np.linalg.solve(pair, [bound, b[j]]))
    feasible = [x for x in candidates if np.isfinite(x).all()
        and math.hypot(*x) <= radius+1e-12
        and bool(np.all(A@x <= b+1e-12))]
    if not feasible:
        raise ValueError('no support-bounded projected COM endpoint')
    # Drop the constant desired·desired to avoid cancellation for distant goals.
    values = [float(.5*(x@x)-desired@x) for x in feasible]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('COM projection objective exceeds finite quadratic range')
    return feasible[min(range(len(feasible)), key=lambda i: values[i])].copy()


def plan_projected(c, phase_start_source):
    """Progress toward a planned landing goal using only current support.

    The hypothetical midpoint is a soft planning goal. The original measured
    received-feet midpoint remains the actual settle target. Face reserves are
    frozen at min(4mm, current origin margin), so an already admissible origin
    below4mm is preserved rather than instantaneously moved or rejected for a
    new preference. Every later command retains the unchanged3mm validator.
    """
    origin = c.com1+getattr(c, '_robust_delta', np.zeros(3))
    heading = np.asarray(c.heading[:2], dtype=float)
    if (origin.shape != (3,) or not np.isfinite(origin).all() or origin[2] <= 0.
            or not np.isfinite(heading).all()
            or abs(float(np.linalg.norm(heading))-1.) > 1e-10):
        raise ValueError('invalid projected COM origin or heading')
    left = np.array([-heading[1], heading[0]])
    basis = np.column_stack([heading, left])
    equations = actual_equations(c, 'translate')
    margins = -(equations[:, :2]@origin[:2]+equations[:, 2])
    if not np.isfinite(margins).all() or float(min(margins)) < RESERVE_M-1e-10:
        raise ValueError('projected COM origin outside current support inset')
    reserves = np.minimum(.004, margins)
    T = float(c.phase_duration('translate')+c.phase_duration('land'))
    if not math.isfinite(T) or T <= 0.:
        raise ValueError('invalid projected COM phase duration')
    tau = math.sqrt(origin[2]/9.81)
    s = np.polynomial.Polynomial([0., 0., 0., 10., -15., 6.])
    capture = s+tau/T*s.deriv()
    xs = [0., 1.]
    xs.extend(float(x.real) for x in capture.deriv().roots()
        if abs(x.imag) < 1e-10 and 0. < x.real < 1.)
    factors = [float(poly(x)) for poly in (s, capture) for x in xs]
    A, b = [], []
    for face, margin, reserve in zip(equations, margins, reserves):
        for factor in factors:
            A.append(factor*(face[:2]@basis)); b.append(float(margin-reserve))
    A.extend(([0., 1.], [0., -1.])); b.extend((.012, .012))
    A, b = np.asarray(A), np.asarray(b)
    radius = min(MAX_TOTAL_SPEED_MPS*T/1.875,
        MAX_TOTAL_ACCEL_MPS2*T*T/(10./math.sqrt(3.)))
    planned = (np.asarray(c.landing)+np.asarray(c.targets[c.stance_side][0]))/2.
    planned[:2] += .035*heading
    planned[2] = origin[2]
    desired = basis.T@(planned[:2]-origin[:2])
    local = project_endpoint(desired, A, b, radius)
    delta = np.r_[basis@local, 0.]
    segment = QuinticSegment.between(phase_start_source, T,
        (np.zeros(3), np.zeros(3), np.zeros(3)), (delta, np.zeros(3), np.zeros(3)))
    return segment, dict(endpointPolicy='closest-feasible-planned-midpoint-v1',
        plannedAtSourceTimeS=c.w.l.lh_time(c.w.h), phaseStartSourceTimeS=phase_start_source,
        forwardM=float(local[0]), lateralM=float(local[1]), plannedGoalSourceM=planned.tolist(),
        plannedDeltaLocalM=desired.tolist(), actualSupportEquations=equations.tolist(),
        planningOriginMarginsM=margins.tolist(), planningFaceReservesM=reserves.tolist(),
        planningReserveRule='min(4mm,current-origin-margin) per actual support face; reject origin below3mm',
        projectionConstraintsA=A.tolist(), projectionConstraintsB=b.tolist(),
        projectionLocalEndpointM=local.tolist(), maximumDisplacementM=radius,
        wholePolynomialExtremaU=xs, wholePolynomialFactors=factors,
        referenceCaptureProxy='constant planned COM height sqrt(z/g)',
        reserveM=RESERVE_M, designReserveCapM=.004, correctionBoundsUnchanged=True,
        frozenMaximumTotalSpeedMps=MAX_TOTAL_SPEED_MPS,
        frozenMaximumTotalAccelerationMps2=MAX_TOTAL_ACCEL_MPS2)


def validate_reference(c, phase, jet):
    p, v, a = (np.asarray(x, dtype=float) for x in jet)
    if any(x.shape != (3,) or not np.isfinite(x).all() for x in (p, v, a)) or p[2] <= 0.:
        raise ValueError('invalid finite positive-height COM reference jet')
    speed, acceleration = float(np.linalg.norm(v)), float(np.linalg.norm(a))
    if speed>MAX_TOTAL_SPEED_MPS+1e-10 or acceleration>MAX_TOTAL_ACCEL_MPS2+1e-10:
        raise ValueError('total COM reference jet exceeds frozen witness envelope')
    equations = actual_equations(c, phase)
    capture = p[:2]+math.sqrt(p[2]/9.81)*v[:2]
    margin = float(min(np.min(-(equations[:, :2]@p[:2]+equations[:, 2])),
        np.min(-(equations[:, :2]@capture+equations[:, 2]))))
    if margin<RESERVE_M-1e-10:
        raise ValueError('continuous COM reference outside current support inset')
    return dict(referenceSupportMarginM=margin, totalSpeedMps=speed,
        totalAccelerationMps2=acceleration, actualSupportOnly=True,
        maximumTotalSpeedMps=MAX_TOTAL_SPEED_MPS, maximumTotalAccelerationMps2=MAX_TOTAL_ACCEL_MPS2)
