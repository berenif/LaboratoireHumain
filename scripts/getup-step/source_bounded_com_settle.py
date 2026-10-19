"""Opt-in analytical C2 settle reference. No native world, physics or optimizer.

Only the frozen ACTUAL support supplied by the caller is certified. This is not
future support or dynamic qualification; retain the original live validator.
"""
from dataclasses import dataclass
import bisect
from decimal import Decimal, localcontext
import hashlib
import json
import math
import numpy as np
from numpy.polynomial import Polynomial

VERSION = 'bounded-c2-settle-v1'
MAX_SPEED = .19260552331163003
MAX_ACCELERATION = .3901818436690813
RESERVE = .003
ZERO_JET_TOLERANCE = 1e-12
ROUNDING_TOLERANCE = 64*np.finfo(float).eps


def _finite(value, shape, label):
    value = np.asarray(value, dtype=float)
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError('invalid finite '+label)
    return value


def _extrema(coefficients, duration):
    """All closed-interval polynomial extrema, not a time/sample grid."""
    p = Polynomial(np.asarray(coefficients, dtype=float))
    candidates = [0., float(duration)]
    for root in p.deriv().roots():
        if abs(root.imag) <= 1e-9*max(1., abs(root.real)) and 0. < root.real < duration:
            candidates.append(float(root.real))
    values = [float(p(t)) for t in candidates]
    if not np.isfinite(values).all():
        raise ValueError('nonfinite polynomial bound')
    return dict(min=min(values), max=max(values), timesS=candidates)



def _enclosure(coefficients, duration):
    """Complete-root extrema plus an outward-rounded Bernstein envelope.

    Bernstein convex-hull bounds are independent of approximate root locations.
    Decimal80 evaluation and an explicit decimal error cushion are rounded
    outwards to binary64. No time subdivision/sample grid is used.
    """
    c = np.asarray(coefficients, dtype=float)
    exact = _extrema(c, duration)
    n = len(c)-1
    with localcontext() as context:
        context.prec = 80
        T = Decimal.from_float(float(duration))
        powers = [Decimal.from_float(float(c[k]))*T**k for k in range(n+1)]
        beta = [sum((powers[k]*Decimal(math.comb(i,k))/Decimal(math.comb(n,k))
                     for k in range(i+1)), Decimal(0)) for i in range(n+1)]
        cushion = max(Decimal(1),sum(abs(v) for v in powers))*Decimal('1e-65')
        lower = math.nextafter(float(min(beta)-cushion), -math.inf)
        upper = math.nextafter(float(max(beta)+cushion), math.inf)
    if not math.isfinite(lower) or not math.isfinite(upper):
        raise ValueError('nonfinite conservative polynomial enclosure')
    return dict(**exact, conservativeMin=lower, conservativeMax=upper,
                enclosureMethod='outward Decimal80 Bernstein convex-hull;1e-65 weighted coefficient cushion')


def _jet(coefficients, t):
    c = np.asarray(coefficients)
    return tuple(sum((math.prod(range(i-order+1, i+1)) if order else 1)
                     *c[i]*t**(i-order) for i in range(order, len(c)))
                 for order in range(3))


def _norm_max(coefficients, order, duration):
    c = np.asarray(coefficients)
    square = Polynomial([0.])
    for axis in range(3):
        derivative = Polynomial(c[:, axis]).deriv(order)
        square += derivative*derivative
    bound = _extrema(square.coef, duration)
    return math.sqrt(max(0., bound['max'])), bound['timesS']


def _actual_hull(points):
    raw = np.asarray(points, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 2 or len(raw) < 3 or not np.isfinite(raw).all():
        raise ValueError('actual support requires finite Nx2 points')
    pts = sorted(set(map(tuple, raw.tolist())))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for p in pts:
        while len(lower)>1 and cross(lower[-2], lower[-1], p)<=0.: lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper)>1 and cross(upper[-2], upper[-1], p)<=0.: upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1]+upper[:-1])
    if len(hull)<3: raise ValueError('actual support hull is degenerate')
    equations = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b-a; length = math.hypot(*edge)
        if not length>0. or not math.isfinite(length): raise ValueError('invalid actual hull edge')
        normal = np.array([edge[1], -edge[0]])/length
        equations.append([*normal, -float(normal@a)])
    return raw, hull, np.array(equations)


@dataclass(frozen=True)
class QuinticPath:
    start: float
    duration: float
    coefficients: tuple

    @classmethod
    def between(cls, start, duration, initial, final):
        p, v, a = (np.asarray(j) for j in initial)
        q, w, b = (np.asarray(j) for j in final)
        low = np.array([p, v*duration, .5*a*duration**2])
        rhs = np.array([q-low.sum(axis=0), w*duration-low[1]-2*low[2],
                        b*duration**2-2*low[2]])
        high = np.linalg.solve(np.array([[1.,1.,1.],[3.,4.,5.],[6.,12.,20.]]), rhs)
        return cls(float(start), float(duration), tuple(map(tuple, np.vstack([low, high]))))

    def evaluate(self, now):
        u = float(np.clip((now-self.start)/self.duration, 0., 1.))
        c = np.array(self.coefficients)
        p = sum(c[i]*u**i for i in range(6))
        v = sum(i*c[i]*u**(i-1) for i in range(1, 6))/self.duration
        a = sum(i*(i-1)*c[i]*u**(i-2) for i in range(2, 6))/self.duration**2
        return p, v, a

    def receipt(self):
        return dict(startSourceTimeS=self.start, durationS=self.duration,
                    coefficients=[list(row) for row in self.coefficients])


@dataclass(frozen=True)
class CubicPiece:
    offset: float
    duration: float
    coefficients: tuple


@dataclass(frozen=True)
class BoundedSettlePath:
    start: float
    duration: float
    pieces: tuple
    initial_position: tuple
    final_position: tuple

    def evaluate(self, now):
        if not math.isfinite(float(now)): raise ValueError('nonfinite evaluation time')
        elapsed = now-self.start
        if now <= self.start: return np.array(self.initial_position), np.zeros(3), np.zeros(3)
        if now >= self.start+self.duration: return np.array(self.final_position), np.zeros(3), np.zeros(3)
        ends = [p.offset+p.duration for p in self.pieces]
        index = min(bisect.bisect_right(ends, elapsed), len(self.pieces)-1)
        piece = self.pieces[index]
        return _jet(piece.coefficients, float(np.clip(elapsed-piece.offset, 0., piece.duration)))

    def receipt(self):
        return dict(startSourceTimeS=self.start, durationS=self.duration,
                    pieces=[dict(offsetS=p.offset, durationS=p.duration,
                                 coefficients=[list(row) for row in p.coefficients]) for p in self.pieces],
                    initialPositionM=list(self.initial_position), finalPositionM=list(self.final_position))


def _pieces(path):
    if hasattr(path, 'pieces'):
        return list(path.pieces)
    c = _finite(path.coefficients, (6,3), 'legacy coefficients')
    return [CubicPiece(0., path.duration, tuple(map(tuple,
        c/np.array([path.duration**i for i in range(6)])[:,None])))]


def certify_path(path, actual_support_points_xy):
    """Polynomial certificate on supplied current actual hull, including capture.

    On each piece z extrema enclose tau=sqrt(z/g). Every face expression is
    affine in tau, so both endpoint-tau polynomials bound the varying-height
    capture curve conservatively over the entire closed piece interval.
    """
    raw, hull, equations = _actual_hull(actual_support_points_xy)
    pieces = _pieces(path)
    if not pieces: raise ValueError('empty settle path')
    speed, acceleration, position_margin, capture_margin = 0., 0., float('inf'), float('inf')
    certificates = []; previous = None; expected_offset = 0.
    for index, piece in enumerate(pieces):
        c = np.asarray(piece.coefficients, dtype=float)
        if c.ndim!=2 or c.shape[1]!=3 or not np.isfinite(c).all(): raise ValueError('invalid polynomial coefficients')
        if not math.isfinite(piece.duration) or piece.duration<0. or abs(piece.offset-expected_offset)>1e-12:
            raise ValueError('invalid contiguous piece clocks')
        initial, final = _jet(c, 0.), _jet(c, piece.duration)
        if previous is not None and any(np.max(abs(a-b))>1e-12 for a,b in zip(initial, previous)):
            raise ValueError('piece boundary is not C2')
        previous = final; expected_offset += piece.duration
        v, vt = _norm_max(c, 1, piece.duration); a, at = _norm_max(c, 2, piece.duration)
        speed = max(speed, v); acceleration = max(acceleration, a)
        z = _enclosure(c[:,2], piece.duration)
        if z['conservativeMin'] <= 0.: raise ValueError('nonpositive whole-path COM height')
        with localcontext() as context:
            context.prec=80
            g=Decimal.from_float(9.81)
            taus = [math.nextafter(float((Decimal.from_float(z['conservativeMin'])/g).sqrt()), -math.inf),
                    math.nextafter(float((Decimal.from_float(z['conservativeMax'])/g).sqrt()), math.inf)]
        face_bounds = []
        for face_index, (nx, ny, offset) in enumerate(equations):
            pos = nx*Polynomial(c[:,0])+ny*Polynomial(c[:,1])+offset
            posmax = _enclosure(pos.coef, piece.duration)
            caps = [_enclosure((pos+tau*(nx*Polynomial(c[:,0]).deriv()+ny*Polynomial(c[:,1]).deriv())).coef,
                             piece.duration) for tau in taus]
            pm, cm = -posmax['conservativeMax'], -max(k['conservativeMax'] for k in caps)
            position_margin = min(position_margin, pm); capture_margin = min(capture_margin, cm)
            face_bounds.append(dict(faceIndex=face_index, positionMinimumMarginM=pm,
                                    captureConservativeMinimumMarginM=cm,
                                    positionExtrema=posmax, captureExtremaAtTauBounds=caps))
        certificates.append(dict(index=index, offsetS=piece.offset, durationS=piece.duration,
            maximumSpeedMps=v, speedExtremaTimesS=vt, maximumAccelerationMps2=a,
            accelerationExtremaTimesS=at, heightExtremaM=z, tauBoundsS=taus, faces=face_bounds))
    if abs(expected_offset-path.duration)>1e-12: raise ValueError('piece durations do not equal unchanged duration')
    first = _jet(pieces[0].coefficients, 0.)
    if hasattr(path, 'initial_position'):
        for jet, pos in ((first, path.initial_position), (previous, path.final_position)):
            if np.max(abs(jet[0]-pos))>1e-12 or any(np.max(abs(v))>1e-12 for v in jet[1:]):
                raise ValueError('wrong exact endpoint/zero jet')
    if speed>MAX_SPEED+ROUNDING_TOLERANCE or acceleration>MAX_ACCELERATION+ROUNDING_TOLERANCE:
        raise ValueError('whole-path total jet exceeds unchanged frozen caps')
    if min(position_margin, capture_margin)<RESERVE:
        raise ValueError('whole path outside supplied actual hull inset')
    return dict(method='all polynomial derivative roots/endpoints plus outward Bernstein enclosures; conservative per-piece tau interval',
        actualSupportPointsXY=raw.tolist(), hullVerticesXY=hull.tolist(), normalizedHullEquations=equations.tolist(),
        actualPointsSHA256=hashlib.sha256(json.dumps(raw.tolist(),separators=(',',':')).encode()).hexdigest(),
        reserveM=RESERVE, maximumSpeedCapMps=MAX_SPEED, maximumAccelerationCapMps2=MAX_ACCELERATION,
        certifiedMaximumSpeedMps=speed, certifiedMaximumAccelerationMps2=acceleration,
        minimumPositionMarginM=position_margin, conservativeMinimumCaptureMarginM=capture_margin,
        numericalRoundoffTolerance=ROUNDING_TOLERANCE, pieces=certificates,
        activePositionFaceIndices=sorted({f['faceIndex'] for k in certificates for f in k['faces'] if abs(f['positionMinimumMarginM']-position_margin)<1e-12}),
        activeCaptureFaceIndices=sorted({f['faceIndex'] for k in certificates for f in k['faces'] if abs(f['captureConservativeMinimumMarginM']-capture_margin)<1e-12}),
        currentActualSupportOnly=True, futureSupportClaim=False)


def build_bounded_settle(start, duration, initial, final, actual_support_points_xy, legacy=None):
    """Return (path, reconstructible receipt); no target/duration/cap adaptation."""
    if not math.isfinite(start) or not math.isfinite(duration) or duration<=0.:
        raise ValueError('invalid settle clock')
    initial = tuple(_finite(j, (3,), 'initial jet') for j in initial)
    final = tuple(_finite(j, (3,), 'final jet') for j in final)
    if any(np.max(abs(j))>ZERO_JET_TOLERANCE for j in (*initial[1:], *final[1:])):
        raise ValueError('bounded settle requires mathematically zero endpoint derivatives within1e-12')
    if min(initial[0][2],final[0][2])<=0.: raise ValueError('nonpositive endpoint height')
    if legacy is None: legacy = QuinticPath.between(start, duration, initial, final)
    if legacy.start != start or legacy.duration != duration: raise ValueError('legacy clock differs')
    for jet, expected in zip((legacy.evaluate(start),legacy.evaluate(start+duration)),(initial,final)):
        if any(np.max(abs(a-b))>1e-12 for a,b in zip(jet,expected)): raise ValueError('legacy endpoints differ')
    legacy_pieces = _pieces(legacy)
    v = _norm_max(legacy_pieces[0].coefficients, 1, duration)[0]
    a = _norm_max(legacy_pieces[0].coefficients, 2, duration)[0]
    shape = None
    if v<=MAX_SPEED+ROUNDING_TOLERANCE and a<=MAX_ACCELERATION+ROUNDING_TOLERANCE:
        path, profile = legacy, 'legacy-quintic-exact-identity'
    else:
        change = final[0]-initial[0]; D = float(np.linalg.norm(change))
        if not D>0. or not math.isfinite(D): raise ValueError('invalid settle distance')
        V = min(MAX_SPEED, 2.*D/duration)
        t_acc = duration-D/V; coast = duration-2.*t_acc
        ramp = min(t_acc/2., t_acc-V/MAX_ACCELERATION)
        if not math.isfinite(ramp) or ramp<=0. or coast < -ROUNDING_TOLERANCE:
            raise ValueError('analytical bounded settle is infeasible at unchanged caps/duration')
        coast = max(0.,coast); A = V/(t_acc-ramp); J = A/ramp
        durations = [ramp,t_acc-2.*ramp,ramp,coast,ramp,t_acc-2.*ramp,ramp]
        jerks = [J,0.,-J,0.,-J,0.,J]
        if any(d<0. or not math.isfinite(d) for d in durations): raise ValueError('invalid analytical piece duration')
        direction = change/D; position=0.; velocity=0.; acceleration=0.; offset=0.; pieces=[]
        for dt, jerk in zip(durations,jerks):
            coefficients = np.array([initial[0]+direction*position,direction*velocity,
                                     direction*acceleration/2.,direction*jerk/6.])
            pieces.append(CubicPiece(offset,float(dt),tuple(map(tuple,coefficients))))
            position += velocity*dt+.5*acceleration*dt*dt+jerk*dt**3/6.
            velocity += acceleration*dt+.5*jerk*dt*dt
            acceleration += jerk*dt; offset += dt
        path = BoundedSettlePath(float(start),float(duration),tuple(pieces),tuple(initial[0]),tuple(final[0]))
        profile = 'seven-cubic-constant-jerk'
        shape = dict(distanceM=D, peakSpeedMps=V, accelerationTimeS=t_acc, coastTimeS=coast,
                     rampTimeS=ramp, peakAccelerationMps2=A, peakJerkMps3=J,
                     pieceDurationsS=durations, scalarJerksMps3=jerks,
                     integratedFinalScalarPositionM=position, integratedFinalScalarVelocityMps=velocity,
                     integratedFinalScalarAccelerationMps2=acceleration)
    certificate = certify_path(path, actual_support_points_xy)
    receipt = dict(schema=1, version=VERSION, profile=profile, path=path.receipt(),
        legacyQuintic=legacy.receipt(), legacyMaximumSpeedMps=v, legacyMaximumAccelerationMps2=a,
        exactInitialJet=[j.tolist() for j in initial], exactFinalJet=[j.tolist() for j in final],
        zeroDerivativeCanonicalizationTolerance=ZERO_JET_TOLERANCE,
        endpointDerivativesCanonicalized=(profile!='legacy-quintic-exact-identity'),
        analyticalShape=shape, certificate=certificate, physicsIntegrations=0, optimizerCalls=0,
        durationUnchanged=True, measuredTargetUnchanged=True, frozenCapsUnchanged=True,
        analyticalProof=['V*(T-t_acc)=D; symmetric acceleration area gives V=A*(t_acc-r).',
            'r<=t_acc/2 keeps both acceleration plateaus nonnegative; r<=t_acc-V/Acap implies A<=Acap.',
            'J=V/[r*(t_acc-r)] decreases for0<r<=t_acc/2, so largest admissible r minimizes peak jerk in this fixedV/t_acc family.',
            'Each cubic integrates constant jerk; propagating p,v,a makes every join C2, and symmetric negative acceleration returns v,a tozero.',
            'Direction=(target-origin)/3D-distance gives full vector norm caps; z/capture certificates use only supplied current actual hull.'],
        requiredLiveValidation='Every evaluation must retain unchanged total-jet and CURRENT actual-hull3mm checks; never project commands or infer future support.')
    return path, receipt


def from_receipt(receipt):
    """Rebuild coefficients and re-prove bounds from actual points in receipt."""
    if receipt.get('version') != VERSION: raise ValueError('unknown bounded settle version')
    data = receipt['path']; start=float(data['startSourceTimeS']); duration=float(data['durationS'])
    if not math.isfinite(start) or not math.isfinite(duration) or duration<=0.: raise ValueError('invalid receipt clock')
    profile=receipt.get('profile')
    if profile=='legacy-quintic-exact-identity':
        path=QuinticPath(start,duration,tuple(map(tuple,_finite(data['coefficients'],(6,3),'receipt quintic'))))
    elif profile=='seven-cubic-constant-jerk':
        pieces=tuple(CubicPiece(float(p['offsetS']),float(p['durationS']),
            tuple(map(tuple,_finite(p['coefficients'],(4,3),'receipt cubic')))) for p in data['pieces'])
        path=BoundedSettlePath(start,duration,pieces,
            tuple(_finite(data['initialPositionM'],(3,),'receipt initial position')),
            tuple(_finite(data['finalPositionM'],(3,),'receipt final position')))
    else: raise ValueError('unknown settle profile')
    initial=tuple(_finite(j,(3,),'receipt initial jet') for j in receipt['exactInitialJet'])
    final=tuple(_finite(j,(3,),'receipt final jet') for j in receipt['exactFinalJet'])
    if len(initial)!=3 or len(final)!=3 or any(np.max(abs(j))>ZERO_JET_TOLERANCE for j in (*initial[1:],*final[1:])):
        raise ValueError('receipt declares nonzero endpoint derivatives')
    for jet, expected in zip((path.evaluate(start),path.evaluate(start+duration)),(initial,final)):
        if any(np.max(abs(a-b))>1e-12 for a,b in zip(jet,expected)): raise ValueError('receipt endpoint jets differ')
    certify_path(path, receipt['certificate']['actualSupportPointsXY'])
    return path
