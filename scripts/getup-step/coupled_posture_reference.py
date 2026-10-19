"""Pure proposed pelvis/thorax hook. No controller, native world, solve, or rollout.

All public reference data own immutable tuples. This is deliberately not installed
in the repository. Orientations use SOURCE axes: x forward, y left, z up;
R = Rz(heading) Ry(forward_pitch) Rx(roll). Angular jets are WORLD vectors.
"""
from dataclasses import dataclass, asdict, replace
import hashlib
import json
import math
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

VERSION = 'coupled-posture-pure-v1'
THORAX, PELVIS = 1, 4
LOWER_COUNT = 15
ANKLES_RIGHT_LEFT = (7, 13)
LUMBAR = (0, 1, 2)
MAX_TOTAL_COM_SPEED = .19260552331163003
MAX_TOTAL_COM_ACCELERATION = .3901818436690813


def frozen(value, n=None):
    a = np.asarray(value, dtype=float).reshape(-1)
    if not np.isfinite(a).all() or (n is not None and len(a) != n):
        raise ValueError('invalid finite reference dimensions')
    return tuple(float(x) for x in a)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def is_sha256(value):
    return isinstance(value,str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Jet:
    position: tuple
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        p = frozen(self.position)
        if not p:
            raise ValueError('empty jet')
        for key, value in (('position', p), ('velocity', self.velocity),
                           ('acceleration', self.acceleration)):
            object.__setattr__(self, key, frozen(value, len(p)))

    @classmethod
    def rest(cls, p):
        p = frozen(p)
        return cls(p, (0.,)*len(p), (0.,)*len(p))

    def arrays(self):
        return tuple(np.array(x) for x in (self.position, self.velocity, self.acceleration))


@dataclass(frozen=True)
class C2Segment:
    start: float
    duration: float
    initial: Jet
    final: Jet
    coefficients: tuple  # ascending powers in normalized time

    def __post_init__(self):
        if (not math.isfinite(self.start) or not math.isfinite(self.duration)
                or self.duration <= 0. or len(self.initial.position) != len(self.final.position)
                or len(self.coefficients) != 6):
            raise ValueError('invalid immutable C2 segment')
        object.__setattr__(self,'coefficients',tuple(
            frozen(row,len(self.initial.position)) for row in self.coefficients))

    @classmethod
    def between(cls, start, duration, initial, final):
        start, duration = float(start), float(duration)
        if not math.isfinite(start) or not math.isfinite(duration) or duration <= 0.:
            raise ValueError('invalid C2 segment clock')
        if len(initial.position) != len(final.position):
            raise ValueError('different endpoint dimensions')
        p, v, a = initial.arrays(); q, w, b = final.arrays()
        low = np.array([p, v*duration, .5*a*duration**2])
        rhs = np.array([q-low.sum(axis=0), w*duration-low[1]-2*low[2],
            b*duration**2-2*low[2]])
        high = np.linalg.solve(np.array([[1.,1.,1.], [3.,4.,5.], [6.,12.,20.]]), rhs)
        c = np.vstack([low, high])
        if not np.isfinite(c).all():
            raise ValueError('nonfinite C2 coefficients')
        return cls(start, duration, initial, final, tuple(frozen(row) for row in c))

    def at(self, now):
        now = float(now)
        if not math.isfinite(now) or now < self.start:
            raise ValueError('C2 reference requested before its causal start')
        if now == self.start:
            return self.initial
        if now >= self.start+self.duration:
            if now > self.start+self.duration and (
                    any(self.final.velocity) or any(self.final.acceleration)):
                raise ValueError('cannot freeze nonzero endpoint jets; create a handoff')
            return self.final
        u = (now-self.start)/self.duration
        c = np.array(self.coefficients)
        return Jet(sum(c[i]*u**i for i in range(6)),
            sum(i*c[i]*u**(i-1) for i in range(1,6))/self.duration,
            sum(i*(i-1)*c[i]*u**(i-2) for i in range(2,6))/self.duration**2)

    def component_bounds(self, order=0):
        """Whole polynomial extrema; no sample grid and no trajectory clipping."""
        if order not in (0, 1, 2):
            raise ValueError('unsupported jet derivative')
        bounds = []
        for column in np.array(self.coefficients).T:
            p = np.polynomial.Polynomial(column).deriv(order)/self.duration**order
            xs = [0., 1.]+[float(r.real) for r in p.deriv().roots()
                if abs(r.imag) < 1e-10 and 0. < r.real < 1.]
            values = [float(p(u)) for u in xs]
            bounds.append((min(values), max(values)))
        return np.array(bounds)

    def maximum_norm(self, order):
        ps = [np.polynomial.Polynomial(column).deriv(order)/self.duration**order
            for column in np.array(self.coefficients).T]
        squared = sum((p*p for p in ps), np.polynomial.Polynomial([0.]))
        xs = [0., 1.]+[float(r.real) for r in squared.deriv().roots()
            if abs(r.imag) < 1e-10 and 0. < r.real < 1.]
        return math.sqrt(max(0., max(float(squared(u)) for u in xs)))


def rotation(angles):
    return Rotation.from_euler('ZYX', np.asarray(angles)).as_matrix()


def chart(R):
    R = np.asarray(R, dtype=float)
    if (R.shape != (3,3) or not np.isfinite(R).all()
            or np.max(abs(R.T@R-np.eye(3))) > 1e-9 or abs(np.linalg.det(R)-1.) > 1e-9):
        raise ValueError('invalid source rotation')
    pitch = math.atan2(-R[2,0], math.hypot(R[0,0], R[1,0]))
    if abs(math.cos(pitch)) < 1e-5:
        raise ValueError('heading chart is singular; do not clamp the angle')
    return np.array([math.atan2(R[1,0], R[0,0]), pitch,
        math.atan2(R[2,1], R[2,2])])


def angular_basis(angles):
    yaw, pitch, _ = np.asarray(angles)
    z = np.array([0.,0.,1.])
    y = Rotation.from_rotvec(z*yaw).apply([0.,1.,0.])
    x = rotation([yaw,pitch,0.])@np.array([1.,0.,0.])
    return np.column_stack([z,y,x])


def angular_world(angle_jet):
    p, v, a = angle_jet.arrays(); E = angular_basis(p)
    z, y, x = E.T
    omega = E@v
    alpha = E@a+np.cross(z*v[0], y)*v[1]+np.cross(z*v[0]+y*v[1], x)*v[2]
    return rotation(p), omega, alpha


def angular_chart(R, omega, alpha):
    p = chart(R); E = angular_basis(p)
    v = np.linalg.solve(E, np.asarray(omega, dtype=float))
    z, y, x = E.T
    bias = np.cross(z*v[0], y)*v[1]+np.cross(z*v[0]+y*v[1], x)*v[2]
    a = np.linalg.solve(E, np.asarray(alpha, dtype=float)-bias)
    return Jet(p, v, a)


@dataclass(frozen=True)
class OrientationJet:
    body: int
    rotation: tuple
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        R = np.asarray(self.rotation, dtype=float).reshape(3,3)
        chart(R)
        object.__setattr__(self, 'rotation', frozen(R, 9))
        object.__setattr__(self, 'velocity', frozen(self.velocity, 3))
        object.__setattr__(self, 'acceleration', frozen(self.acceleration, 3))
        if self.body not in (THORAX, PELVIS):
            raise ValueError('coupled orientation must name source body1 or body4')

    @classmethod
    def rest(cls, body, R):
        return cls(body, frozen(R), (0.,)*3, (0.,)*3)

    def arrays(self):
        return np.array(self.rotation).reshape(3,3), np.array(self.velocity), np.array(self.acceleration)


@dataclass(frozen=True)
class HeadingPath:
    body: int
    angles: C2Segment

    @classmethod
    def between(cls, start, duration, initial, target):
        if initial.body != target.body:
            raise ValueError('orientation handoff changes body')
        a = angular_chart(*initial.arrays()); b = angular_chart(*target.arrays())
        pa, va, aa = a.arrays(); pb, vb, ab = b.arrays()
        if abs(wrap(pb[0]-pa[0])) > 1e-10:
            raise ValueError('posture candidate changes retained heading')
        if max(abs(va[0]), abs(aa[0]), abs(vb[0]), abs(ab[0])) > 1e-10:
            raise ValueError('nonstationary incoming heading needs its own certified path')
        # Canonicalize only the SAME physical heading and numerical zero jets.
        pb[0] = pa[0]; va[0] = aa[0] = vb[0] = ab[0] = 0.
        path = C2Segment.between(start,duration,Jet(pa,va,aa),Jet(pb,vb,ab))
        pitch_bounds = path.component_bounds()[1]
        if min(abs(math.cos(x)) for x in pitch_bounds) < 1e-5 or not (
                -math.pi/2 < pitch_bounds[0] <= pitch_bounds[1] < math.pi/2):
            raise ValueError('orientation polynomial leaves nonsingular heading chart')
        return cls(initial.body,path)

    def at(self, now):
        R, v, a = angular_world(self.angles.at(now))
        return OrientationJet(self.body,frozen(R),frozen(v),frozen(a))


@dataclass(frozen=True)
class CoupledSample:
    com: Jet
    lower_joints: Jet
    thorax: OrientationJet
    pelvis: OrientationJet

    def __post_init__(self):
        if (len(self.com.position) != 3 or len(self.lower_joints.position) != LOWER_COUNT
                or self.thorax.body != THORAX or self.pelvis.body != PELVIS):
            raise ValueError('wrong coupled sample dimensions or body mapping')

    def fingerprint(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class Candidate:
    name: str
    provenance_sha256: str
    target: CoupledSample

    def __post_init__(self):
        if not self.name or not is_sha256(self.provenance_sha256):
            raise ValueError('explicit named, source-bound candidate required')
        for jet in (self.target.com, self.target.lower_joints):
            if any(jet.velocity) or any(jet.acceleration):
                raise ValueError('candidate wait target must have zero terminal jets')
        for o in (self.target.thorax, self.target.pelvis):
            if any(o.velocity) or any(o.acceleration):
                raise ValueError('candidate orientation must have zero terminal jets')

    @property
    def key(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class CoupledPlan:
    candidate: Candidate
    com: C2Segment
    lower_joints: C2Segment
    thorax: HeadingPath
    pelvis: HeadingPath
    lumbar_anchor: tuple
    joint_limits: tuple
    lumbar_change_limit: float = math.radians(5.)

    def __post_init__(self):
        object.__setattr__(self,'lumbar_anchor',frozen(self.lumbar_anchor,3))
        object.__setattr__(self,'joint_limits',tuple(frozen(row,2) for row in self.joint_limits))
        paths = (self.com,self.lower_joints,self.thorax.angles,self.pelvis.angles)
        if (len(self.joint_limits) != 21 or any(p.start != self.com.start or
                p.duration != self.com.duration for p in paths)
                or self.thorax.body != THORAX or self.pelvis.body != PELVIS):
            raise ValueError('inconsistent coupled reference clocks or dimensions')

    @classmethod
    def between(cls, start, duration, initial, candidate, *, joint_limits,
                lumbar_anchor, lumbar_change_limit=math.radians(5.)):
        limits = np.asarray(joint_limits,dtype=float)
        anchor = frozen(lumbar_anchor,3)
        if (limits.shape != (21,2) or not np.isfinite(limits).all()
                or np.any(limits[:,0] >= limits[:,1])
                or not 0. < lumbar_change_limit <= math.radians(5.)):
            raise ValueError('invalid unchanged joint/lumbar bounds')
        result = cls(candidate,
            C2Segment.between(start,duration,initial.com,candidate.target.com),
            C2Segment.between(start,duration,initial.lower_joints,candidate.target.lower_joints),
            HeadingPath.between(start,duration,initial.thorax,candidate.target.thorax),
            HeadingPath.between(start,duration,initial.pelvis,candidate.target.pelvis),
            anchor,tuple(frozen(row) for row in limits),float(lumbar_change_limit))
        bounds = result.lower_joints.component_bounds()
        reserve = np.full(15,.02); reserve[list(ANKLES_RIGHT_LEFT)] = .06
        if (np.any(bounds[:,0] < limits[:15,0]+reserve-1e-12)
                or np.any(bounds[:,1] > limits[:15,1]-reserve+1e-12)):
            raise ValueError('whole joint polynomial violates interior/ankle reference reserves')
        if np.any(np.maximum(abs(bounds[:3,0]-anchor),abs(bounds[:3,1]-anchor)) > lumbar_change_limit+1e-12):
            raise ValueError('whole joint polynomial exceeds anchored lumbar change bound')
        if (result.com.maximum_norm(1) > MAX_TOTAL_COM_SPEED+1e-10
                or result.com.maximum_norm(2) > MAX_TOTAL_COM_ACCELERATION+1e-10):
            raise ValueError('whole total COM jet exceeds unchanged witness envelope')
        if result.com.component_bounds()[2,0] <= 0.:
            raise ValueError('nonpositive planned COM height')
        return result

    def at(self, now):
        return CoupledSample(self.com.at(now),self.lower_joints.at(now),
            self.thorax.at(now),self.pelvis.at(now))

    def handoff(self, now, duration, candidate):
        """Use at every new target, including settle/stand; never reset v or a."""
        return self.between(now,duration,self.at(now),candidate,
            joint_limits=self.joint_limits,lumbar_anchor=self.lumbar_anchor,
            lumbar_change_limit=self.lumbar_change_limit)

    def receipt(self):
        return dict(version=VERSION,**asdict(self),referenceSHA256=digest(asdict(self)),
            finiteTimePhysicalFeasibilityProven=False,staticSeedsArePathProof=False)


@dataclass(frozen=True)
class CurrentSupport:
    source_step: int
    source_time: float
    step_index: int
    phase: str
    loaded_points_xy: tuple
    stance_points_xyz: tuple
    present_corners: tuple
    surviving_corners: tuple
    require_triangle: bool

    def __post_init__(self):
        object.__setattr__(self,'loaded_points_xy',tuple(frozen(p,2) for p in self.loaded_points_xy))
        object.__setattr__(self,'stance_points_xyz',tuple(frozen(p,3) for p in self.stance_points_xyz))
        if any(not isinstance(x,(bool,np.bool_)) for x in self.present_corners):
            raise ValueError('measured corner presence must be boolean')
        object.__setattr__(self,'present_corners',tuple(bool(x) for x in self.present_corners))
        object.__setattr__(self,'surviving_corners',tuple(self.surviving_corners))
        if (type(self.source_step) is not int or self.source_step < 0
                or not math.isfinite(self.source_time) or self.source_time < 0.
                or type(self.require_triangle) is not bool or len(self.stance_points_xyz) != 4
                or len(self.present_corners) != 4):
            raise ValueError('invalid actual support snapshot')
        if self.phase in ('translate','land') and not self.require_triangle:
            raise ValueError('existing translate/land policy requires current selected triangle')
        if self.require_triangle:
            ids = self.surviving_corners
            if (len(ids) != 3 or len(set(ids)) != 3
                    or any(type(i) is not int or i not in range(4) for i in ids)
                    or not all(self.present_corners[i] for i in ids)):
                raise ValueError('three unique currently measured surviving corners required')
        self.equations()  # reject absent/degenerate measured geometry now

    def equations(self):
        p = np.unique(np.asarray(self.loaded_points_xy),axis=0)
        if len(p) < 3:
            raise ValueError('actual loaded hull is absent')
        try:
            eq = [ConvexHull(p).equations]
            if self.require_triangle:
                q = np.asarray(self.stance_points_xyz)[list(self.surviving_corners),:2]
                eq.append(ConvexHull(q).equations)
        except Exception as exc:
            raise ValueError('degenerate actual support geometry') from exc
        return np.vstack(eq)

    @property
    def key(self):
        return digest(asdict(self))

    def validate_identity(self, context):
        if (self.source_step != context.source_step or self.source_time != context.source_time
                or self.step_index != context.step_index or self.phase != context.phase):
            raise ValueError('stale or wrong-phase measured support snapshot')

    def validate_com(self, jet, reserve=.003):
        p,v,a = jet.arrays()
        if p.shape != (3,) or p[2] <= 0. or reserve < .003 or not math.isfinite(reserve):
            raise ValueError('invalid height or relaxed support reserve')
        capture = p[:2]+math.sqrt(p[2]/9.81)*v[:2]
        eq = self.equations()
        margin = min(float(np.min(-(eq[:,:2]@x+eq[:,2]))) for x in (p[:2],capture))
        speed,acceleration = float(np.linalg.norm(v)),float(np.linalg.norm(a))
        if margin < reserve-1e-10:
            raise ValueError('reference COM/capture leaves CURRENT measured hull/triangle inset')
        if speed > MAX_TOTAL_COM_SPEED+1e-10 or acceleration > MAX_TOTAL_COM_ACCELERATION+1e-10:
            raise ValueError('total COM jet exceeds unchanged envelope')
        return dict(currentReferenceSupportMarginM=margin,totalSpeedMps=speed,
            totalAccelerationMps2=acceleration,currentActualSupportOnly=True)


@dataclass(frozen=True)
class OptIn:
    candidate_name: str
    candidate_key: str
    thorax_weight: float = 4.
    pelvis_weight: float = 4.
    kp: float = 40.
    kd: float = 12.

    def __post_init__(self):
        if (not self.candidate_name or not is_sha256(self.candidate_key)
                or any(not math.isfinite(x) or x <= 0. for x in
                    (self.thorax_weight,self.pelvis_weight,self.kp,self.kd))):
            raise ValueError('explicit candidate and finite soft weights required')


def prepare_context(context, *, opt_in=None, plan=None, support=None):
    """Pure replacement for the EXISTING make_context result, once per command.

    Disabled is the identical object, with no validation, imports, or observers.
    COM is the complete already-combined reference; never add a correction after
    this hook. Delayed arms, feet, capture barriers and causal history are retained.
    This prepares data only. It is not permission or a physical pilot admission.
    """
    if opt_in is None:
        return context
    if plan is None or support is None:
        raise ValueError('explicit plan and fresh actual support required')
    if (opt_in.candidate_name != plan.candidate.name
            or opt_in.candidate_key != plan.candidate.key):
        raise ValueError('candidate configuration/reference mismatch')
    support.validate_identity(context)
    sample = plan.at(context.source_time)
    diagnostic = support.validate_com(sample.com)
    if len(context.arms.position) != 6:
        raise ValueError('unchanged six delayed arm targets required')
    lower = sample.lower_joints.arrays(); arm = context.arms.arrays()
    joints = type(context.joints).make(*(np.r_[a,b] for a,b in zip(lower,arm)))
    com = type(context.com).make(*sample.com.arrays())
    if not context.orientations:
        raise ValueError('existing Orientation factory unavailable')
    factory = type(context.orientations[0]).make
    orientations = tuple(o for o in context.orientations if o.body not in (THORAX,PELVIS))
    orientations += tuple(factory(o.body,*o.arrays(),kp=opt_in.kp,kd=opt_in.kd,weight=weight)
        for o,weight in ((sample.thorax,opt_in.thorax_weight),(sample.pelvis,opt_in.pelvis_weight)))
    config = json.loads(context.config_json)
    config['coupledPosture'] = dict(optIn=asdict(opt_in),plan=plan.receipt())
    diagnostics = json.loads(context.diagnostics_json)
    diagnostics['coupledPosture'] = dict(**diagnostic,version=VERSION,
        candidate=plan.candidate.name,sampleSHA256=sample.fingerprint(),
        orientationTasksAreSoft=True,orientationSlackMustBeReported=True,
        delayedArmTaskUnchanged=True,futureArmMappingRequiredForStaticFK=True)
    canonical = lambda x: json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)
    return replace(context,com=com,joints=joints,orientations=orientations,
        config_json=canonical(config),diagnostics_json=canonical(diagnostics))


def orientation_residual(reference, measured_R, measured_omega, *, kp=40.,kd=12.,
                         achieved_angular_acceleration=None):
    """Existing weighted LS is equivalent to unbounded penalized slack e.

    Minimize weight**2 ||e||**2, e = achieved_acceleration - desired_acceleration.
    This is reporting math, not another QP or a hard orientation constraint.
    """
    target_R,target_w,target_a = reference.arrays()
    measured_R = np.asarray(measured_R,dtype=float); chart(measured_R)
    err = Rotation.from_matrix(target_R@measured_R.T).as_rotvec()
    desired = target_a+kp*err+kd*(target_w-np.asarray(measured_omega,dtype=float))
    target_angles,actual_angles = chart(target_R),chart(measured_R)
    out = dict(rotationErrorWorldRad=frozen(err),desiredAngularAccelerationWorld=frozen(desired),
        forwardPitchPreferenceMissRad=float(target_angles[1]-actual_angles[1]),
        rollPreferenceMissRad=wrap(target_angles[2]-actual_angles[2]),
        headingErrorRad=wrap(target_angles[0]-actual_angles[0]))
    if achieved_angular_acceleration is not None:
        out['softAccelerationSlackWorldRadps2'] = frozen(
            np.asarray(achieved_angular_acceleration,dtype=float)-desired,3)
    return out
