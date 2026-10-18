"""Immutable causal acquired-load preference and one-slack QP border.

This module is pure mathematics: it never creates a native world, measures a
load, changes a control, or advances physics. The caller owns the original
qualified land-to-settle event and contact-to-force-column mapping. A context's
outer canonical digest binds the strict JSON receipt returned here.
"""
from dataclasses import asdict, dataclass, fields
import math
from numbers import Integral, Real

import numpy as np

SCHEMA = 1
POLICY = 'retained-acquired-load-hinge-v1'
WEIGHT = .0005
ACTIVATION_S = .1
SETTLE_S = 1.52
PHYSICS_STEP_S = .00125
CLOCK_TOLERANCE_S = 1e-9


def _integer(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value < 0:
        raise ValueError('invalid '+name)
    return int(value)


def _finite(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError('invalid '+name)
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('nonfinite '+name)
    return value


def _exact_keys(value, expected, name):
    if type(value) is not dict or set(value) != set(expected):
        raise ValueError('invalid '+name+' fields')


@dataclass(frozen=True)
class AcquiredLoad:
    """Measured load at the original qualified landing event, never re-anchored.

    source_time labels the measured handoff stage. Its cached load belongs to
    the previous solved stage (source_time - .00125s), not the new geometry.
    The activation clock starts at the measured handoff stage.
    """
    source_step: int
    source_time: float
    step_index: int
    new_side: int
    measured_normal_n: float
    body_weight_n: float
    qualified_landing_samples: int

    def __post_init__(self):
        for name in ('source_step', 'step_index', 'new_side', 'qualified_landing_samples'):
            object.__setattr__(self, name, _integer(getattr(self, name), name))
        if self.new_side not in (0, 1):
            raise ValueError('invalid new_side')
        for name in ('source_time', 'measured_normal_n', 'body_weight_n'):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.source_time < 0. or self.measured_normal_n < 0. or self.body_weight_n <= 0.:
            raise ValueError('invalid acquired-load measurement')

    def receipt(self):
        return asdict(self)

    @classmethod
    def from_receipt(cls, receipt):
        _exact_keys(receipt, (f.name for f in fields(cls)), 'acquired-load receipt')
        return cls(**receipt)


def _qualified_anchor(anchor):
    if type(anchor) is not AcquiredLoad:
        raise ValueError('invalid causal acquired-load anchor')
    # Validate again at consumption boundaries, not just at construction.
    AcquiredLoad(**anchor.receipt())
    if anchor.qualified_landing_samples < 80 or anchor.measured_normal_n < 3.:
        raise ValueError('invalid causal acquired-load anchor')


@dataclass(frozen=True)
class SoftLoadPreference:
    anchor: AcquiredLoad
    floor_jet: tuple
    weight: float = WEIGHT

    def __post_init__(self):
        _qualified_anchor(self.anchor)
        try:
            values = tuple(self.floor_jet)
        except TypeError as error:
            raise ValueError('invalid load floor jet') from error
        if len(values) != 3:
            raise ValueError('invalid load floor jet dimension')
        jet = tuple(_finite(x, 'load floor jet') for x in values)
        if jet[0] < 0.:
            raise ValueError('negative load floor')
        weight = _finite(self.weight, 'soft-load weight')
        if weight != WEIGHT:
            raise ValueError('soft-load weight must remain .0005')
        object.__setattr__(self, 'floor_jet', jet)
        object.__setattr__(self, 'weight', weight)

    def receipt(self):
        return dict(schema=SCHEMA, policy=POLICY, anchor=self.anchor.receipt(),
                    floor_jet=list(self.floor_jet), weight=self.weight)

    @classmethod
    def from_receipt(cls, receipt):
        _exact_keys(receipt, ('schema', 'policy', 'anchor', 'floor_jet', 'weight'),
                    'soft-load preference receipt')
        if (type(receipt['schema']) is not int or receipt['schema'] != SCHEMA
                or type(receipt['policy']) is not str or receipt['policy'] != POLICY
                or type(receipt['floor_jet']) is not list):
            raise ValueError('invalid soft-load receipt schema/policy/jet')
        return cls(AcquiredLoad.from_receipt(receipt['anchor']),
                   receipt['floor_jet'], receipt['weight'])


def _quintic(t, duration):
    if t <= 0.:
        return 0., 0., 0.
    if t >= duration:
        return 1., 0., 0.
    u = t/duration
    return (u**3*(10.+u*(-15.+6.*u)),
            30.*u*u*(1.-u)**2/duration,
            60.*u*(1.-u)*(1.-2.*u)/duration**2)


def prepare_preference(anchor, phase, elapsed, source_step, source_time,
                       settle_duration=SETTLE_S):
    """C2 activation times C2 fade, active only during the unchanged settle.

    Native counter age and native time age must both equal elapsed. Waiting
    past the nominal settle endpoint retains the anchor and an exact zero jet.
    """
    if phase != 'settle':
        return None
    _qualified_anchor(anchor)
    elapsed = _finite(elapsed, 'settle elapsed')
    source_step = _integer(source_step, 'native source_step')
    source_time = _finite(source_time, 'native source_time')
    duration = _finite(settle_duration, 'settle duration')
    if (elapsed < 0. or duration < ACTIVATION_S
            or source_step < anchor.source_step or source_time < anchor.source_time
            or abs(PHYSICS_STEP_S*(source_step-anchor.source_step)-elapsed) > CLOCK_TOLERANCE_S
            or abs(source_time-anchor.source_time-elapsed) > CLOCK_TOLERANCE_S):
        raise ValueError('invalid causal acquired-load clock or settle duration')
    if elapsed >= duration or elapsed == 0.:
        return SoftLoadPreference(anchor, (0., 0., 0.))
    a, da, dda = _quintic(elapsed, ACTIVATION_S)
    # Reversed quintic equals 1-s(elapsed/duration), with stable nonnegative
    # values near the endpoint instead of subtracting nearly equal numbers.
    fade, reversed_rate, ddfade = _quintic(duration-elapsed, duration)
    dfade = -reversed_rate
    initial = min(anchor.measured_normal_n, .1*anchor.body_weight_n)
    return SoftLoadPreference(anchor, (initial*a*fade,
        initial*(da*fade+a*dfade),
        initial*(dda*fade+2.*da*dfade+a*ddfade)))


def _matrix(value, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in 'fiu':
            raise ValueError('nonnumeric '+name)
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('invalid '+name) from error
    if not np.isfinite(result).all():
        raise ValueError('nonfinite '+name)
    return result


def extend_qp(P, q, A, b, normal_indices, preference):
    """Border one already assembled QP using its A*x <= b convention.

    Existing rows, columns and costs are copied exactly. Append only zeta,
    -zeta<=0 and -sum(Fz_new)-zeta<=-floor. Its cost is
    .5*(.0005*zeta)**2, with no extra regularizer or new force bound. The caller
    preserves its equality cone size and increases its final nonnegative cone
    by two. Force columns exclude zeta when reading the resulting solution.
    With no preference, even object identity and original dtypes are retained.
    """
    if preference is None:
        return P, q, A, b
    if type(preference) is not SoftLoadPreference:
        raise ValueError('invalid soft-load preference')
    # Revalidate at the use boundary, including against unsafe object-level
    # mutation that deliberately bypassed a frozen dataclass.
    SoftLoadPreference(preference.anchor, preference.floor_jet, preference.weight)
    PP, qq, AA, bb = (_matrix(x, name) for x, name in
                      ((P, 'P'), (q, 'q'), (A, 'A'), (b, 'b')))
    if qq.ndim != 1 or bb.ndim != 1:
        raise ValueError('invalid QP vector dimensions')
    n = qq.size
    if n == 0 or PP.shape != (n, n) or AA.shape != (bb.size, n):
        raise ValueError('invalid QP matrix dimensions')
    try:
        indices = tuple(normal_indices)
    except TypeError as error:
        raise ValueError('invalid normal-force indices') from error
    if not indices or any(isinstance(i, (bool, np.bool_)) or not isinstance(i, Integral)
                          or not 0 <= i < n for i in indices):
        raise ValueError('invalid normal-force indices')
    indices = tuple(int(i) for i in indices)
    if len(indices) != len(set(indices)):
        raise ValueError('duplicate normal-force indices')
    bordered_P = np.zeros((n+1, n+1)); bordered_P[:n, :n] = PP
    bordered_P[n, n] = WEIGHT**2
    bordered_q = np.r_[qq, 0.]
    bordered_A = np.zeros((bb.size+2, n+1)); bordered_A[:bb.size, :n] = AA
    bordered_A[-2, n] = -1.
    bordered_A[-1, list(indices)] = -1.; bordered_A[-1, n] = -1.
    bordered_b = np.r_[bb, 0., -preference.floor_jet[0]]
    return bordered_P, bordered_q, bordered_A, bordered_b
