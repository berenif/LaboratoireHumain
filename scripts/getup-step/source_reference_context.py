"""Immutable, causally prepared QP references for the reviewed opt-in gait.

This module never integrates, writes a native state, observes support history, or
updates finite differences. Tuples own their scalar data; arrays() returns copies.
The legacy controller does not construct or consume this context.
"""
from dataclasses import dataclass, asdict, replace
import copy
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from source_step_probe import clock

SCHEMA = 2
REFERENCE_VERSION = 'reviewed-reference-v1'


def vector(value):
    a = np.asarray(value, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('nonfinite reference')
    return tuple(float(x) for x in a.reshape(-1))


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


@dataclass(frozen=True)
class Jet:
    position: tuple
    velocity: tuple
    acceleration: tuple

    @classmethod
    def make(cls, p, v, a):
        result = cls(vector(p), vector(v), vector(a))
        if not len(result.position) == len(result.velocity) == len(result.acceleration):
            raise ValueError('inconsistent reference jet')
        return result

    def arrays(self):
        return tuple(np.array(x, dtype=float) for x in (self.position, self.velocity, self.acceleration))


@dataclass(frozen=True)
class Orientation:
    body: int
    rotation: tuple
    velocity: tuple
    acceleration: tuple
    kp: float = 40.
    kd: float = 12.
    weight: float = 4.

    @classmethod
    def make(cls, body, R, v, a, **kwargs):
        return cls(body, vector(R), vector(v), vector(a), **kwargs)

    def arrays(self):
        return np.array(self.rotation).reshape(3, 3), np.array(self.velocity), np.array(self.acceleration)


@dataclass(frozen=True)
class Foot:
    translation: Jet
    orientation: Orientation

    def arrays(self):
        return (*self.translation.arrays(), *self.orientation.arrays())


@dataclass(frozen=True)
class ReferenceContext:
    schema: int
    version: str
    source_step: int
    source_time: float
    step_index: int
    phase: str
    elapsed: float
    epoch: int
    com: Jet
    joints: Jet
    arms: Jet
    arm_gains: tuple
    orientations: tuple
    feet: tuple
    capture_constraints: tuple
    unload_admitted: bool
    causal_json: str
    config_json: str
    diagnostics_json: str
    hypothetical_changes: tuple = ()

    def validate(self, c, t, phase):
        if self.schema != SCHEMA or self.version != REFERENCE_VERSION:
            raise ValueError('unsupported reference context')
        if (self.source_step != c.w.l.lh_dim(c.w.h, 6)
                or self.source_time != c.w.l.lh_time(c.w.h)
                or self.phase != phase or self.elapsed != t
                or self.step_index != c._reviewed_step_index
                or self.epoch != c._reference_epoch):
            raise ValueError('stale reference command identity')
        if self.hypothetical_changes and self.hypothetical_changes != ('unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm'):
            raise ValueError('undeclared hypothetical reference change')

    def receipt(self):
        result = asdict(self)
        result['causal'] = json.loads(result.pop('causal_json'))
        result['config'] = json.loads(result.pop('config_json'))
        result['diagnostics'] = json.loads(result.pop('diagnostics_json'))
        result['contextSHA256'] = digest(result)
        return result

    @classmethod
    def from_receipt(cls, receipt):
        d = copy.deepcopy(receipt)
        claimed = d.pop('contextSHA256')
        if digest(d) != claimed:
            raise ValueError('reference context digest mismatch')
        d['causal_json'] = canonical(d.pop('causal'))
        d['config_json'] = canonical(d.pop('config'))
        d['diagnostics_json'] = canonical(d.pop('diagnostics'))
        for name in ('com', 'joints', 'arms'):
            d[name] = Jet.make(**{dict(position='p', velocity='v', acceleration='a')[k]: v for k, v in d[name].items()})
        d['orientations'] = tuple(Orientation(**{k: tuple(v) if isinstance(v, list) else v for k, v in item.items()}) for item in d['orientations'])
        feet = []
        for item in d['feet']:
            tr = item['translation']; o = item['orientation']
            feet.append(Foot(Jet.make(tr['position'], tr['velocity'], tr['acceleration']), Orientation(**{k: tuple(v) if isinstance(v, list) else v for k, v in o.items()})))
        d['feet'] = tuple(feet)
        d['capture_constraints'] = tuple((tuple(row), float(bound)) for row, bound in d['capture_constraints'])
        d['arm_gains'] = tuple(d['arm_gains'])
        d['hypothetical_changes'] = tuple(d['hypothetical_changes'])
        return cls(**d)


def nominal_com(c, t, phase):
    s, ds, dds = clock(t, c.phase_duration('transfer'))
    target = c.com0+(c.com1-c.com0)*s if phase == 'transfer' else c.com1.copy()
    vel = (c.com1-c.com0)*ds if phase == 'transfer' else np.zeros(3)
    acc = (c.com1-c.com0)*dds if phase == 'transfer' else np.zeros(3)
    if phase in ('settle', 'stand'):
        s, ds, dds = clock(t, c.phase_duration('settle')) if phase == 'settle' else (1., 0., 0.)
        target = c.com1+(c.finalCom-c.com1)*s
        vel = (c.finalCom-c.com1)*ds
        acc = (c.finalCom-c.com1)*dds
    return target, vel, acc


def nominal_joints(c, t, phase):
    if phase == 'transfer' and hasattr(c, 'posture_start'):
        s, ds, dds = clock(t, c.phase_duration('transfer'))
        delta = c.q0-c.posture_start
        return c.posture_start+delta*s, delta*ds, delta*dds
    return c.q0.copy(), np.zeros(21), np.zeros(21)


def nominal_orientations(c, t, phase):
    R = c.rootR.copy(); v = np.zeros(3); a = np.zeros(3)
    if c.self_avoidance:
        s, ds, dds = clock(t, c.phase_duration('transfer')) if phase == 'transfer' else (1., 0., 0.)
        if phase == 'settle':
            s, ds, dds = clock(t, c.phase_duration('settle')); s, ds, dds = 1.-s, -ds, -dds
        elif phase == 'stand':
            s, ds, dds = 0., 0., 0.
        R = Rotation.from_rotvec(c.lean_axis*s).as_matrix()@c.rootR
        v, a = c.lean_axis*ds, c.lean_axis*dds
    return (Orientation.make(1, R, v, a),)


def nominal_feet(c, t, phase, landing_adjustment=None, floor_targets=None):
    floor = c.floorTargets if floor_targets is None else floor_targets
    references = []
    for side in (0, 1):
        pos0, Rinitial = c.targets[side]; Rflat = c.flatRotations[side]
        axis = Rotation.from_matrix(Rflat@Rinitial.T).as_rotvec()
        pt = pos0.copy(); pv = np.zeros(3); pa = np.zeros(3)
        wv = np.zeros(3); wa = np.zeros(3); R = Rflat.copy()
        if phase == 'transfer':
            s, ds, dds = clock(t, c.phase_duration('transfer'))
            R = Rotation.from_rotvec(axis*s).as_matrix()@Rinitial
            wv, wa = axis*ds, axis*dds
            pt[2] += (floor[side]-pos0[2])*s
            pv[2], pa[2] = (floor[side]-pos0[2])*ds, (floor[side]-pos0[2])*dds
        else:
            pt[2] = floor[side]
        if side == c.swing_side and phase != 'transfer':
            s, ds, dds = clock(t, c.phase_duration(phase)) if phase in ('lift', 'translate', 'land') else (1., 0., 0.)
            if phase == 'lift':
                pt[2] += .04*s; pv[2], pa[2] = .04*ds, .04*dds
            elif phase == 'translate':
                pt[2] += .04; pt += c.step_distance*c.heading*s
                pv, pa = c.step_distance*c.heading*ds, c.step_distance*c.heading*dds
            elif phase == 'land':
                pt = c.landing.copy(); pt[2] += .04*(1-s)-.0005*s
                pv[2], pa[2] = -.0405*ds, -.0405*dds
            else:
                pt = c.landing.copy(); pt[2] -= .0005
            if landing_adjustment is not None:
                offset, rate, acceleration = landing_adjustment
                pt[2] -= offset; pv[2] -= rate; pa[2] -= acceleration
        references.append(Foot(Jet.make(pt, pv, pa), Orientation.make(c.w.ids['left_foot' if side == 0 else 'right_foot'], R, wv, wa)))
    return tuple(references)


def make_context(c, t, phase, com, *, causal, config, seek, diagnostics=None):
    w = c.w; v = w.arr(1, (27,)).copy(); measured_com = w.arr(6, (17, 3))[1].copy()
    J = w.read('lh_com_jac', (3, 27))
    single = phase in ('lift', 'translate') or (phase == 'land' and w.arr(20, (17,))[c.swing_body] < 3.)
    width = 48+(4 if single else 8)*3
    # A detached controller may emit diagnostics, but cannot mutate live history.
    detached = copy.copy(c)
    for key, value in vars(c).items():
        if key != 'w' and isinstance(value, (dict, list, np.ndarray)):
            setattr(detached, key, copy.deepcopy(value))
    rows, bounds = [], []
    detached.add_capture_constraints(phase, width, J, measured_com, v, rows, bounds)
    diagnostic = dict(diagnostics or {})
    diagnostic.update(captureBarrier=getattr(detached, 'capture_barrier', []),
        opposedArmRangeBarrier=getattr(detached, 'arm_range_barrier', []),
        robustSupportBarriers=getattr(detached, 'robust_support_barrier', []))
    return ReferenceContext(SCHEMA, REFERENCE_VERSION, w.l.lh_dim(w.h, 6), w.l.lh_time(w.h),
        c._reviewed_step_index, phase, t, c._reference_epoch, Jet.make(*com),
        Jet.make(*nominal_joints(c, t, phase)), Jet.make(c.arm_posture, c.arm_velocity, c.arm_acceleration),
        (c.arm_kp, c.arm_kd, c.arm_weight), nominal_orientations(c, t, phase),
        nominal_feet(c, t, phase, seek), tuple((vector(r), float(b)) for r, b in zip(rows, bounds)),
        bool(getattr(c, 'unload_admitted', False)), canonical(causal), canonical(config), canonical(diagnostic))


def prospective_unload_context(c, context):
    """Same prepared tasks/barriers, only the declared transfer unload changes."""
    side = c.swing_side
    old_floor = json.loads(context.config_json)['floorTargets'][side]
    new_floor = c.w.ground_height(c.targets[side][0])+c.radii[side]+.0001
    s, ds, dds = clock(context.elapsed, c.phase_duration('transfer'))
    p, v, a = context.feet[side].translation.arrays()
    delta = new_floor-old_floor
    p[2] += delta*s; v[2] += delta*ds; a[2] += delta*dds
    feet = list(context.feet)
    feet[side] = replace(feet[side], translation=Jet.make(p, v, a))
    return replace(context, feet=tuple(feet), unload_admitted=True,
        hypothetical_changes=('unload-normal-force-zero', 'swing-transfer-floor-plus-0.1mm'))
