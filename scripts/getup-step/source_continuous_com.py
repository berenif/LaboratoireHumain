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
