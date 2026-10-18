#!/usr/bin/env python3
"""Opt-in equal-angle opposed arm swing on the selected wider-hip/knee base.

Measured ipsilateral thigh pitch drives shoulder-hand pitch with gain -1. The
oblique mirrored shoulder hinges are inverted with scratch-only FK. The live
body receives the unchanged constrained torque QP, never an installed pose.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from source_step_probe import Controller, ROOT, clock, sha
from source_selected_step import prepare
from source_multistep_probe import MultiStep
from source_arm_knee_mobility import native_bytes, arms_along_body_goal, KNEE_GOAL_RAD
from plan_source_arm_knee import plan_posture
from source_step_preview import fk, P

SIDES = ('right', 'left')
ARM_KP = 100.
ARM_KD = 20.
ARM_WEIGHT = 2.


def pitch(vector):
    """Forward-positive angle of a down-pointing limb in torso source axes."""
    return np.arctan2(vector[..., 0], -vector[..., 2])


def limb_telemetry(w, kind):
    R = w.arr(5, (17, 3, 3))[1]
    positions = w.arr(3, (17, 3))
    velocity = w.arr(1, (27,))
    omega = R.T @ (w.read('lh_body_jac', (6, 27), 1)[3:] @ velocity)
    values, rates, vectors = [], [], []
    for side in SIDES:
        a, b = ((side+'_thigh', side+'_shin') if kind == 'leg'
                else (side+'_upper_arm', side+'_hand'))
        ia, ib = [w.l.lh_id(w.h, 1, name.encode()) for name in (a, b)]
        d = R.T @ (positions[ib]-positions[ia])
        dv = R.T @ ((w.read('lh_body_jac', (6, 27), ib)[:3]
                    - w.read('lh_body_jac', (6, 27), ia)[:3]) @ velocity)-np.cross(omega, d)
        values.append(float(pitch(d)))
        rates.append(float((-d[2]*dv[0]+d[0]*dv[2])/(d[0]**2+d[2]**2)))
        vectors.append(d.tolist())
    return np.array(values), np.array(rates), np.array(vectors)


class SagittalArmMap:
    """Read-only FK lookup, preserving lateral clearance while swinging in x/z."""
    def __init__(self, q, limits, goal, ranges=None):
        self.goal = np.array(goal)
        model = json.loads((ROOT/'rust/crates/model/data/getup-v1.json').read_text())
        kin = fk.Kinematics(model)
        self.grid = np.linspace(-.45, .45, 91)
        self.ranges = np.array(ranges if ranges is not None else [[-.45,.45],[-.45,.45]])
        self.extended = ranges is not None
        self.curves = []
        self.neutral_vectors = []
        self.maximum_fit_error_m = 0.
        for side_index, side in enumerate(SIDES):
            offset = 15+3*side_index
            fixed = np.array(q).copy(); fixed[15:] = goal
            def vector(shoulders):
                trial = fixed.copy(); trial[offset:offset+2] = shoulders
                poses = kin.forward(trial, np.eye(4))
                return P @ (poses[side+'_hand'][:3, 3]-poses[side+'_upper_arm'][:3, 3])
            neutral = vector(goal[3*side_index:3*side_index+2])
            self.neutral_vectors.append(neutral.tolist())
            samples = []
            lower, upper = limits[offset:offset+2, 0]+.02, limits[offset:offset+2, 1]-.02
            grid=(np.linspace(*self.ranges[side_index],round(np.ptp(self.ranges[side_index])/.01)+1)
                  if self.extended else self.grid)
            for angle in grid:
                target = Rotation.from_rotvec([0., -angle, 0.]).apply(neutral)
                fit = least_squares(lambda shoulder: vector(shoulder)-target,
                    np.array(goal[3*side_index:3*side_index+2]).clip(lower+1e-8, upper-1e-8),
                    bounds=(lower, upper), ftol=1e-12, xtol=1e-12, gtol=1e-12)
                error = np.linalg.norm(vector(fit.x)-target)
                self.maximum_fit_error_m = max(self.maximum_fit_error_m, float(error))
                assert fit.success and error < 1e-7, 'sagittal shoulder target infeasible'
                samples.append(np.r_[fit.x, goal[3*side_index+2]])
            self.curves.append(CubicSpline(grid, np.array(samples), axis=0))
        self.neutral_pitch = pitch(np.array(self.neutral_vectors))

    def evaluate(self, delta):
        assert np.all(delta>=self.ranges[:,0]) and np.all(delta<=self.ranges[:,1]), 'measured leg excursion exceeds verified arm map'
        return tuple(np.concatenate([curve(delta[i], order) for i, curve in enumerate(self.curves)])
                     for order in (0, 1, 2))

    def receipt(self):
        result = dict(mapping='arm sagittal pitch change = - ipsilateral measured thigh pitch change',
                    angularGain=-1., sideOrder=list(SIDES), coordinateFrame='torso-local source x-forward z-up',
                    shoulderHinges='two oblique, mirrored axes, inverted with exact model FK',
                    verifiedSwingDeltaRangeRad=[-.45,.45], maximumFKFitErrorM=self.maximum_fit_error_m,
                    neutralArmPitchRad=self.neutral_pitch.tolist(), neutralArmVectorsM=self.neutral_vectors,
                    elbowNeutralRad=self.goal[[2,5]].tolist(), goalRad=self.goal.tolist(),
                    placementDurationS=4., positionGain=ARM_KP, velocityGain=ARM_KD, taskWeight=ARM_WEIGHT,
                    balanceArmOffsetEnabled=False, sameAmplitudeTarget=True, clippingEnabled=False)
        if self.extended:
            result.pop('verifiedSwingDeltaRangeRad')
            result['verifiedSwingDeltaRangeBySideRad']=self.ranges.tolist()
        return result


class OpposedArmController(Controller):
    @classmethod
    def from_controller(cls, original, reference, mapping, leg_origin, placement):
        c = cls.__new__(cls); c.__dict__.update(original.__dict__)
        c.q0 = np.array(reference['qReference'])
        c.com1 = np.array(reference['comReferenceSource'])
        c.lean_axis = Rotation.from_matrix(np.array(reference['rootReferenceSource']) @ c.rootR.T).as_rotvec()
        c.posture_start = c.w.arr(0, (28,))[7:].copy()
        c.knee_goal = c.q0[[6, 12]].copy()
        c.arm_start = c.posture_start[15:].copy()
        c.leg_origin = np.array(leg_origin)
        c._mapping = mapping
        c.placement = placement
        c.arm_kp, c.arm_kd, c.arm_weight = ARM_KP, ARM_KD, ARM_WEIGHT
        c.previous_leg_rate = None
        c.previous_time = None
        return c

    def solve(self, t, phase='transfer'):
        if hasattr(self, 'prepared_arm_measurement'):
            legs, leg_rate, leg_acc = self.prepared_arm_measurement()
        else:
            legs, leg_rate, _ = limb_telemetry(self.w, 'leg')
            now = self.w.l.lh_time(self.w.h)
            dt = None if self.previous_time is None else now-self.previous_time
            leg_acc = ((leg_rate-self.previous_leg_rate)/dt
                       if dt is not None and dt > 1e-8 else np.zeros(2))
            self.previous_time, self.previous_leg_rate = now, leg_rate.copy()
        delta = -(legs-self.leg_origin)
        mapped, jac, hessian = self._mapping.evaluate(delta)
        dq = jac*np.repeat(-leg_rate, 3)
        ddq = hessian*np.repeat(leg_rate**2, 3)-jac*np.repeat(leg_acc, 3)
        s, ds, dds = clock(t, getattr(self,'arm_placement_duration',4.)) if self.placement and phase == 'transfer' else (1., 0., 0.)
        target = self.arm_start+s*(mapped-self.arm_start)
        velocity = ds*(mapped-self.arm_start)+s*dq
        acceleration = dds*(mapped-self.arm_start)+2*ds*dq+s*ddq
        self.q0[[6, 12]] = self.knee_goal
        self.q0[15:] = target
        self.arm_posture = target
        self.arm_velocity = velocity
        self.arm_acceleration = acceleration
        if hasattr(self, 'prepare_reference_context'):
            self.prepare_reference_context(t, phase)
        u, core = super().solve(t, phase)
        actual, _, vectors = limb_telemetry(self.w, 'arm')
        core['opposedArmSwing'] = dict(measuredLegPitchRad=legs.tolist(), measuredLegPitchRateRadps=leg_rate.tolist(),
            legPitchOriginRad=self.leg_origin.tolist(), targetArmPitchChangeRad=delta.tolist(),
            targetArmPitchRad=(self._mapping.neutral_pitch+delta).tolist(),
            measuredArmPitchRad=actual.tolist(), measuredArmVectorsM=vectors.tolist(),
            targetArmJointRad=target.tolist(), targetArmVelocityRadps=velocity.tolist(),
            targetArmAccelerationRadps2=acceleration.tolist(), placementFraction=s,
            actualSagittalErrorRad=(actual-self._mapping.neutral_pitch-delta).tolist(),
            mappingClipped=False, amplitudeGain=-1., feedbackUsesMeasuredLeg=True)
        return u, core


class OpposedArmMultiStep(MultiStep):
    def __init__(self, base, helper, output, distances=(.03,.06,.09,.045)):
        self.arm_map = None
        self.leg_origin = None
        self.code_at_start = {name: sha(Path(__file__).with_name(name)) for name in
            ('source_opposed_arm_swing.py','source_step_probe.py','source_multistep_probe.py',
             'plan_source_arm_knee.py','source_arm_knee_mobility.py','source_selected_step.py')}
        super().__init__(base, helper, output, distances, True, 2.)

    def begin_step(self):
        super().begin_step()
        w = self.world; before = native_bytes(w)
        if self.arm_map is None:
            goal = arms_along_body_goal(w)
            self.arm_map = getattr(self,'arm_mapping_type',SagittalArmMap)(w.arr(0, (28,))[7:], w.limits, goal)
            self.leg_origin = limb_telemetry(w, 'leg')[0]
            (self.folder/'arm-mapping.json').write_text(json.dumps(self.arm_map.receipt(), indent=2)+'\n')
        arms = self.arm_map.evaluate(-(limb_telemetry(w, 'leg')[0]-self.leg_origin))[0]
        reference = plan_posture(w, self.controller.stance_side, True, .9, arms, KNEE_GOAL_RAD)
        assert native_bytes(w) == before
        assert reference['staticMotorContactFeasible'] and reference['minimumRequiredMotorFraction'] < .925, 'no reserved static posture reference'
        assert reference['minimumSelfSeparationM'] > .025, 'posture reference too close to self collision'
        (self.folder/f'stance-reference-{self.step_index+1:02d}.json').write_text(json.dumps(reference, indent=2)+'\n')
        self.controller = OpposedArmController.from_controller(self.controller, reference, self.arm_map,
                                                              self.leg_origin, self.step_index == 0)
        self.controller.solve(0., 'transfer')
        w.checkpoint(self.folder/f'step-{self.step_index+1:02d}-transfer.bin')
        record = {k:v for k,v in vars(self.controller).items() if k != 'w' and not k.startswith('_')}
        (self.folder/f'controller-step-{self.step_index+1:02d}.json').write_text(json.dumps(record, default=lambda x:x.tolist())+'\n')
        self.step_records[-1].update(postureVariant='selected-hips-knees-opposed-arms',
            staticRequiredMotorFraction=reference['minimumRequiredMotorFraction'],
            staticMinimumSelfSeparationM=reference['minimumSelfSeparationM'])

    def save(self, reason):
        super().save(reason)
        if hasattr(self, 'folder'):
            receipt = dict(stopped=reason, finished=self.finished, actualIntegrations=self.steps,
                physicsStepS=.00125, torqueUpdateS=.005, selectedPresetChanged=False,
                originalMotorStrength=True, originalFriction=True, rootAssistance=False,
                physicalStateInstallationsAfterInitialization=0, codeSHA256AtStart=self.code_at_start,
                codeUnchangedDuringRun=all(sha(Path(__file__).with_name(name)) == digest
                    for name, digest in self.code_at_start.items()))
            (self.folder/'opposed-arm-run.json').write_text(json.dumps(receipt, indent=2)+'\n')

    def decorate(self, row):
        row = super().decorate(row)
        if self.arm_map is not None:
            legs, _, _ = limb_telemetry(self.world, 'leg')
            arms, _, vectors = limb_telemetry(self.world, 'arm')
            row['opposedArmSwing'] = dict(legPitchRad=legs.tolist(), armPitchRad=arms.tolist(),
                armVectorTorsoM=vectors.tolist(), targetArmPitchRad=(self.arm_map.neutral_pitch-legs+self.leg_origin).tolist())
        return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--one-step', action='store_true', help='first 30mm step followed by full 5s quiet')
    args = p.parse_args()
    session, helper, selected = prepare(args.output)
    live = OpposedArmMultiStep(session, helper, args.output/'run', (.03,) if args.one_step else (.03,.06,.09,.045))
    live.paused = False; started = time.monotonic()
    try:
        while not live.finished and not live.failure:
            assert time.monotonic()-started < 300., '300s process cap'
            live.advance(4)
            if live.steps % 800 == 0:
                print('progress', live.steps*.00125, 'step', live.step_index+1, live.phase, 'slip', live.slip, flush=True)
        print(live.failure or 'measured-opposed-arms-plus5squiet', flush=True)
    except Exception as error:
        live.failure = repr(error); raise
    finally:
        live.close()
    raise SystemExit(0 if live.finished else 1)


if __name__ == '__main__':
    main()
