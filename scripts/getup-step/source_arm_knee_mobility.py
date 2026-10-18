#!/usr/bin/env python3
"""Bounded opt-in arm/knee experiments, preserving the successful default.

One uninterrupted original MuJoCo world per variant; torque-only commands.
No inter-step state installation, changed assets, strength, timing or friction.
"""
import argparse
import ctypes as C
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from source_step_probe import World, Controller, ROOT, clock, sha
from source_multistep_probe import MultiStep
from plan_source_arm_knee import plan_posture

KNEE_GOAL_RAD = np.deg2rad([-27.5, -27.5])
ARM_OFFSET_LIMIT_RAD = .035
ARM_TARGET_RATE_RADPS = .08


def native_bytes(w):
    size = w.l.lh_checkpoint_size(w.h)
    blob = C.create_string_buffer(size)
    assert w.l.lh_checkpoint_export(w.h, blob, size) == size
    return blob.raw


def arms_along_body_goal(w):
    """Scratch-only FK fit, hands outside thighs; no moving the live body."""
    entry = w.packet()
    original = native_bytes(w)
    scratch = World(w.base, w.helper)
    try:
        scratch.install_cold(entry)
        Controller(scratch, 0, .032, True)  # bind read-only self-separation helper
        q = np.array(entry['qpos'])
        shoulders = scratch.arr(3, (17, 3))[[11, 14]].copy()
        targets = []
        for shoulder in shoulders:
            outward = shoulder - scratch.arr(3, (17, 3))[1]
            outward[2] = 0.
            outward /= np.linalg.norm(outward)
            targets.extend([shoulder + .06*outward + [0, 0, -.295],
                            shoulder + .11*outward + [0, 0, -.58]])

        def residual(arms):
            trial = q.copy()
            trial[22:] = arms
            scratch.install_cold(dict(qpos=trial, qvel=[0.]*27))
            positions = scratch.arr(3, (17, 3))[[12, 13, 15, 16]]
            raw = scratch.read('lh_self_separation', (897,))
            pairs = raw[1:1+14*int(raw[0])].reshape(-1, 14)
            collision = max(.025-min(pairs[:, 4], default=1.), 0.)
            return np.r_[(positions-np.array(targets)).ravel()*5,
                         collision*20, (arms-q[22:])*.015]

        lower = w.limits[15:, 0]+.02
        upper = w.limits[15:, 1]-.02
        result = least_squares(residual, q[22:].clip(lower, upper),
                               bounds=(lower, upper), max_nfev=100)
        assert result.success
        residual(result.x)
        raw = scratch.read('lh_self_separation', (897,))
        pairs = raw[1:1+14*int(raw[0])].reshape(-1, 14)
        assert min(pairs[:, 4], default=1.) > .025
        return result.x.copy()
    finally:
        scratch.close()
        assert native_bytes(w) == original


def balance_arm_offset(w, c, target):
    """Measured tilt/rate and COM capture error request a bounded reaction.

    The mass-matrix arm coupling maps joint acceleration to base angular
    momentum rate. Its opposite reaction is requested; motor/contact/clearance
    constraints in the unchanged whole-body QP remain authoritative.
    This causal mapping is an experiment, not a claim of improved stability.
    """
    v = w.arr(1, (27,)).copy()
    com = w.arr(6, (17, 3))[1].copy()
    comv = w.read('lh_com_jac', (3, 27)) @ v
    R = w.arr(5, (17, 3, 3))[1]
    omega = w.read('lh_body_jac', (6, 27), 1)[3:] @ v
    tilt = Rotation.from_matrix(c.rootR @ R.T).as_rotvec()
    capture_error = np.r_[target[:2]-com[:2]-comv[:2]*math.sqrt(com[2]/9.81), 0.]
    desired_reaction_world = 3.*tilt - .8*omega + .04*w.mass*9.81*np.cross([0., 0., 1.], capture_error)
    # MuJoCo free-joint rotational velocity / mass rows are body-local.
    coupling = w.read('lh_mass_matrix', (27, 27))[3:6, 21:27]
    desired_reaction_local = R.T @ desired_reaction_world
    offset = -np.linalg.pinv(coupling, rcond=.05) @ desired_reaction_local / 20.
    return np.clip(offset, -ARM_OFFSET_LIMIT_RAD, ARM_OFFSET_LIMIT_RAD), dict(
        measuredCaptureErrorM=capture_error.tolist(), measuredTorsoAngularVelocityRadps=omega.tolist(),
        requestedReactionTorqueNm=desired_reaction_world.tolist())


class ArmKneeController(Controller):
    @classmethod
    def from_controller(cls, original, reference, arms, knees):
        c = cls.__new__(cls)
        c.__dict__.update(original.__dict__)
        c.q0 = np.array(reference['qReference'])
        c.com1 = np.array(reference['comReferenceSource'])
        c.lean_axis = Rotation.from_matrix(np.array(reference['rootReferenceSource']) @ c.rootR.T).as_rotvec()
        c.posture_start = c.w.arr(0, (28,))[7:].copy()
        c.arm_start = c.posture_start[15:].copy()
        c.arm_goal = c.q0[15:].copy()
        c.knee_goal = c.q0[[6, 12]].copy()
        c.arms_enabled, c.knees_enabled = arms, knees
        c.balance_offset = np.zeros(6)
        c.last_offset_time = None
        return c

    def solve(self, t, phase='transfer'):
        info = {}
        if self.knees_enabled:
            self.q0[[6, 12]] = self.knee_goal
        if self.arms_enabled:
            scale = getattr(self, 'trajectory_time_scale', 1.)
            s, ds, dds = clock(t, 2.*scale) if phase == 'transfer' else (1., 0., 0.)
            goal = self.arm_start + s*(self.arm_goal-self.arm_start)
            velocity = ds*(self.arm_goal-self.arm_start)
            acceleration = dds*(self.arm_goal-self.arm_start)
            target = self.com0+(self.com1-self.com0)*s if phase == 'transfer' else self.com1
            if phase in ('settle', 'stand'):
                f = clock(t, 2.*scale)[0] if phase == 'settle' else 1.
                target = self.com1 + (self.finalCom-self.com1)*f
            desired, info = balance_arm_offset(self.w, self, target)
            now = self.w.l.lh_time(self.w.h)
            dt = 0. if self.last_offset_time is None else max(0., now-self.last_offset_time)
            self.balance_offset += np.clip(desired-self.balance_offset,
                                            -ARM_TARGET_RATE_RADPS*dt, ARM_TARGET_RATE_RADPS*dt)
            self.last_offset_time = now
            self.arm_posture = goal + self.balance_offset + .5*velocity + .05*acceleration
            self.q0[15:] = goal + self.balance_offset
            info.update(armReferenceRad=goal.tolist(), armBalanceOffsetRad=self.balance_offset.tolist(),
                        armOffsetLimitRad=ARM_OFFSET_LIMIT_RAD, armTargetRateLimitRadps=ARM_TARGET_RATE_RADPS)
        u, core = super().solve(t, phase)
        core['postureExperiment'] = info
        return u, core


class ArmKneeMultiStep(MultiStep):
    def __init__(self, base, helper, output, variant):
        assert variant in ('arms', 'knees', 'combined')
        self.variant = variant
        self.experiment_arm_goal = None
        super().__init__(base, helper, output, (.03, .06, .09, .045), True, 2.)

    def begin_step(self):
        # Reuse the exact state machine and baseline admission setup. All calls
        # before replacement are read-only; no integration or physical rewrite.
        super().begin_step()
        w = self.world
        before = native_bytes(w)
        arms = self.variant in ('arms', 'combined')
        knees = self.variant in ('knees', 'combined')
        if arms and self.experiment_arm_goal is None:
            self.experiment_arm_goal = arms_along_body_goal(w)
        reference = plan_posture(w, self.controller.stance_side, True, .9,
                                 self.experiment_arm_goal if arms else None,
                                 KNEE_GOAL_RAD if knees else None)
        assert native_bytes(w) == before
        (self.folder/f'stance-reference-{self.step_index+1:02d}.json').write_text(json.dumps(reference, indent=2)+'\n')
        assert reference['staticMotorContactFeasible'] and reference['minimumRequiredMotorFraction'] < .925, 'no reserved static posture reference'
        assert reference['minimumSelfSeparationM'] > .025, 'posture reference too close to self collision'
        self.controller = ArmKneeController.from_controller(self.controller, reference, arms, knees)
        self.controller.solve(0., 'transfer')
        self.world.checkpoint(self.folder/f'step-{self.step_index+1:02d}-transfer.bin')
        record = {k: v for k, v in vars(self.controller).items() if k != 'w'}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(json.dumps(record, default=lambda x: x.tolist())+'\n')
        self.step_records[-1].update(postureVariant=self.variant,
                                    staticRequiredMotorFraction=reference['minimumRequiredMotorFraction'],
                                    staticMinimumSelfSeparationM=reference['minimumSelfSeparationM'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, required=True)
    p.add_argument('--helper', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--variant', choices=('arms', 'knees', 'combined'), required=True)
    args = p.parse_args()
    assert not args.output.exists()
    live = ArmKneeMultiStep(args.base, args.helper, args.output, args.variant)
    live.paused = False
    started = time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started < 300., '300s process cap'
            live.advance(4)
            if live.steps % 800 == 0:
                print('progress', live.steps*.00125, 'step', live.step_index+1, live.phase, 'slip', live.slip, flush=True)
        reason = 'measured-alternating-multistep-plus5squiet' if live.finished else live.failure
        live.save(reason)
        print(reason, [(s['requestedDistanceM'], s.get('achievedForwardM')) for s in live.step_records], flush=True)
    except Exception as error:
        live.failure = repr(error)
        live.save(live.failure)
        raise
    finally:
        live.close()


if __name__ == '__main__':
    main()
