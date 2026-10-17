"""Independently written equations for the pinned GetUp interfaces.

No simulator, state installation, optimizer or network access. Source quirks are
kept explicit; the stepping task reward is deliberately not defined here.
"""
from __future__ import annotations
import math
import numpy as np

POLICY_DT = .025
NATIVE_DT = .00125
AHEAD_DECISIONS = 5
ORDER = np.array([1, 0, 2, 3, 4, 5, 6, 8, 7, 9, 10, 11, 12, 14, 13, 15, 16, 17, 18, 19, 20])
P = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
SOURCE_COM_HEIGHT = .80326


def vector(value, length, name):
    x = np.asarray(value, dtype=np.float64)
    if x.shape != (length,) or not np.isfinite(x).all():
        raise ValueError(f'{name} must be {length} finite values')
    return x


def finite_float32(value, name):
    with np.errstate(over='ignore', invalid='ignore'):
        x = np.asarray(value, dtype=np.float32)
    if not np.isfinite(x).all():
        raise ValueError(f'{name} is not finite float32')
    return x


def finite_scalars(*values):
    if not all(math.isfinite(x) for x in values):
        raise ValueError('nonfinite source scalar')


def target(q, com_height, torso_vertical):
    if not math.isfinite(com_height):
        raise ValueError('COM height must be finite')
    # Targets are not implicitly normalized: exact source splines need not be.
    return np.r_[vector(q, 21, 'joint target'), com_height,
                 vector(torso_vertical, 3, 'target vertical')]


def student_observation(physical, now, ahead):
    return finite_float32(np.r_[vector(physical, 67, 'physical observation'),
                 vector(now, 25, 'now'), vector(ahead, 25, 'ahead'), 1.], 'student observation')


def teacher_observation(physical, power):
    if not math.isfinite(power) or not 0 <= power <= 1:
        raise ValueError('power must be within [0,1]')
    return finite_float32(np.r_[vector(physical, 67, 'physical observation'), power], 'teacher observation')


def conditioned_teacher_observation(physical, now, ahead, power):
    result = student_observation(physical, now, ahead)
    result[-1] = teacher_observation(physical, power)[-1]
    return result


def slow_controls(reference, residual, q, dq, holding=False):
    """qpos-order radians in; actuator-order normalized motor controls out."""
    ref, a, q, dq = [vector(x, 21, name) for x, name in
                     [(reference, 'reference'), (residual, 'residual'), (q, 'q'), (dq, 'dq')]]
    if np.any(abs(a) > 1):
        raise ValueError('source residual must be in [-1,1] radians')
    kd = .2 if holding else .1
    return np.clip(ref + a - q - kd * dq, -1., 1.)[ORDER]


def teacher_controls(action, power):
    """Actor already applied power. Never multiply by power or gear twice."""
    a = vector(action, 21, 'actuator torque control')
    teacher_observation(np.zeros(67), power)
    if np.any(abs(a) > power + 1e-7):
        raise ValueError('teacher action exceeds declared power')
    return np.clip(a, -1., 1.)


def tolerance(x, bounds=(0., 0.), margin=0., sigmoid='gaussian', value_at_margin=.1):
    """The dm_control tolerance families used by the pinned source reward."""
    x = np.asarray(x, dtype=np.float64)
    lo, hi = bounds
    if lo > hi or margin < 0 or not 0 <= value_at_margin < 1:
        raise ValueError('invalid tolerance parameters')
    if not np.isfinite(x).all():
        raise ValueError('nonfinite tolerance input')
    inside = (lo <= x) & (x <= hi)
    if margin == 0:
        return inside.astype(float)
    d = np.maximum(np.maximum(lo - x, x - hi), 0.) / margin
    if sigmoid == 'gaussian':
        if value_at_margin <= 0:
            raise ValueError('Gaussian margin value must be positive')
        value = np.exp(math.log(value_at_margin) * d * d)
    elif sigmoid == 'long_tail':
        if value_at_margin <= 0:
            raise ValueError('long-tail margin value must be positive')
        value = 1. / (1. + (1. / value_at_margin - 1.) * d * d)
    elif sigmoid == 'linear':
        value = np.maximum(0., 1. - (1. - value_at_margin) * d)
    elif sigmoid == 'quadratic':
        value = np.maximum(0., 1. - (1. - value_at_margin) * d * d)
    else:
        raise ValueError('unsupported source tolerance')
    return np.where(inside, 1., value)


def teacher_reward(*, com_height, torso_up, head_height, horizontal_velocity, foot_separation):
    finite_scalars(com_height, torso_up, head_height, foot_separation)
    if foot_separation < 0:
        raise ValueError('foot separation is a nonnegative distance')
    up = 1. if com_height < .5 else float(tolerance(torso_up, (.9, np.inf), 1.9, 'linear', 0.))
    head = float(tolerance(head_height, (1.55, np.inf), 1.55 / 4, 'long_tail', .2))
    quiet = float(np.mean(tolerance(vector(horizontal_velocity, 2, 'horizontal COM velocity'), (-.3, .3), 1.2)))
    feet = float(tolerance(foot_separation, (0., .9), .38, 'quadratic', 0.))
    return up * head * quiet * feet


def slow_moving_reward(qvel27, com_height, torso_vertical, reference_qvel27, reference_target):
    finite_scalars(com_height)
    dq = vector(qvel27, 27, 'source qvel')
    dr = vector(reference_qvel27, 27, 'source reference qvel')
    ref = vector(reference_target, 25, 'source target')
    hip = (2. + np.mean(tolerance(dq[[11, 17]] - dr[[11, 17]], (-.5, .5), 1.3))) / 3.
    height = tolerance(ref[21] - com_height, (0., 0.), .5)
    vertical = np.prod(tolerance(ref[22:] - vector(torso_vertical, 3, 'vertical'), (-.02, .02), .6, value_at_margin=.3))
    return float(hip * height * vertical)


def slow_hold_reward(q, com_height, torso_up, horizontal_velocity, standing_q, *, standing_height=SOURCE_COM_HEIGHT):
    finite_scalars(com_height, torso_up, standing_height)
    up = 1. if com_height < .5 else float(tolerance(torso_up, (.9, np.inf), 1.9, 'linear', 0.))
    quiet = np.mean(tolerance(vector(horizontal_velocity, 2, 'horizontal velocity'), (0., 0.), 1.2))
    pose = np.sum((vector(q, 21, 'q') - vector(standing_q, 21, 'standing q')) ** 2)
    return float(up * quiet * np.exp(-10. * (com_height - standing_height) ** 2) * np.exp(-pose / 4.))


def source_boundary(action_index, moving_frames):
    """Last moving command receives post-increment holding reward in source."""
    if action_index < 0 or moving_frames < 1:
        raise ValueError('invalid source reference index')
    return dict(commandHolding=action_index >= moving_frames,
                rewardHolding=action_index + 1 >= moving_frames,
                nowIndex=min(action_index, moving_frames),
                aheadIndex=min(action_index + AHEAD_DECISIONS, moving_frames))


def source_terminal(post_action_index, moving_frames, com_height, previous_reference_height):
    finite_scalars(com_height, previous_reference_height)
    if post_action_index < 0 or moving_frames < 1:
        raise ValueError('invalid source terminal index')
    physical = com_height < .5 if post_action_index >= moving_frames else abs(com_height - previous_reference_height) > .5
    return dict(terminated=bool(physical), truncated=post_action_index >= moving_frames + 150 and not physical)
