"""Pure algebra and fake-world adapter tests. No MuJoCo or physical steps."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from source_implicit_arm_coordination import (
    build_implicit_arm_task, implicit_arm_task, SIDE_ORDER)


class ImplicitArmAlgebra(unittest.TestCase):
    maximum_residual_error = 0.
    maximum_weighted_residual_error = 0.
    random_cases = 0

    def test_random_residual_matches_original_true_acceleration(self):
        rng = np.random.default_rng(731194)
        # Endpoint and interior placement fractions; arbitrary nonzero ds/dds
        # exercise both analytic placement terms independently.
        for index in range(3000):
            s = (0., 1.)[index] if index < 2 else float(rng.random())
            ds, dds = rng.normal(size=2)
            jt, bt, rate = rng.normal(size=(2, 27)), rng.normal(size=2), rng.normal(size=2)
            mapped, gp, gpp, start, qarm, varm = rng.normal(size=(6, 6))
            mapped_velocity = -gp*np.repeat(rate, 3)
            posture = start+s*(mapped-start)
            velocity = ds*(mapped-start)+s*mapped_velocity
            kp, kd, weight = 100., 20., 2.
            task = build_implicit_arm_task(
                thigh_jacobian=jt, thigh_bias=bt, measured_leg_rate=rate,
                mapped=mapped, mapping_jacobian=gp, mapping_hessian=gpp,
                arm_start=start, arm_posture=posture, arm_velocity=velocity,
                measured_arm_position=qarm, measured_arm_velocity=varm,
                placement_s=s, placement_ds=ds, placement_dds=dds,
                kp=kp, kd=kd, weight=weight)
            qdd = rng.normal(size=27)
            true_thigh_acceleration = jt@qdd+bt
            true_mapped_acceleration = (gpp*np.repeat(rate**2, 3)
                                       -gp*np.repeat(true_thigh_acceleration, 3))
            original_acceleration = (dds*(mapped-start)+2.*ds*mapped_velocity
                                     +s*true_mapped_acceleration)
            original_target = original_acceleration+kp*(posture-qarm)+kd*(velocity-varm)
            original_residual = qdd[21:27]-original_target
            implicit_residual = task.jacobian@qdd-task.target
            error = float(np.max(np.abs(original_residual-implicit_residual)))
            weighted_error = float(np.max(np.abs(weight*original_residual-task.weight*implicit_residual)))
            type(self).maximum_residual_error = max(type(self).maximum_residual_error, error)
            type(self).maximum_weighted_residual_error = max(type(self).maximum_weighted_residual_error, weighted_error)
            type(self).random_cases += 1
            np.testing.assert_allclose(implicit_residual, original_residual, atol=3e-12, rtol=1e-12)
            np.testing.assert_allclose(task.weight*implicit_residual, weight*original_residual, atol=6e-12, rtol=1e-12)
            self.assertEqual(task.weight, 2.)
            self.assertEqual(task.telemetry["positionGain"], 100.)
            self.assertEqual(task.telemetry["velocityGain"], 20.)

    def test_adapter_preserves_switches_targets_history_and_state(self):
        rng = np.random.default_rng(191)
        qpos, qvel = rng.normal(size=28), rng.normal(size=27)
        before_qpos, before_qvel = qpos.copy(), qvel.copy()
        world = SimpleNamespace(arr=lambda code, shape: qpos if code == 0 else qvel)
        legs, rate = rng.normal(size=(2, 2))
        jt, bt = rng.normal(size=(2, 27)), rng.normal(size=2)
        mapped, gp, gpp, arm_start, arm_posture, arm_velocity = rng.normal(size=(6, 6))
        delta_seen = []
        def evaluate(delta):
            delta_seen.append(delta.copy())
            return mapped.copy(), gp.copy(), gpp.copy()
        c = SimpleNamespace(w=world, leg_origin=np.array([.2, -.3]),
            _mapping=SimpleNamespace(evaluate=evaluate), placement=True,
            arm_placement_duration=3.7, arm_start=arm_start,
            arm_posture=arm_posture, arm_velocity=arm_velocity,
            arm_kp=100., arm_kd=20., arm_weight=2., previous_time=17.,
            previous_leg_rate=np.array([.11, .12]))
        history = c.previous_leg_rate.copy()
        clock_calls, jet_calls = [], []
        def jet(w, side):
            self.assertIs(w, world)
            jet_calls.append(side)
            i = SIDE_ORDER.index(side)
            return float(legs[i]), jt[i].copy(), float(bt[i])
        def telemetry(w, kind):
            self.assertIs(w, world); self.assertEqual(kind, "leg")
            return legs.copy(), rate.copy(), np.zeros((2, 3))
        def clock(t, duration):
            clock_calls.append((t, duration)); return .37, .21, -.17
        for placement, phase, expected_jet in ((True, "transfer", (.37, .21, -.17)),
                                               (True, "settle", (1., 0., 0.)),
                                               (False, "transfer", (1., 0., 0.))):
            c.placement = placement
            result = implicit_arm_task(c, 1.1, phase, thigh_jet_fn=jet,
                limb_telemetry_fn=telemetry, clock_fn=clock)
            direct = build_implicit_arm_task(
                thigh_jacobian=jt, thigh_bias=bt, measured_leg_rate=rate,
                mapped=mapped, mapping_jacobian=gp, mapping_hessian=gpp,
                arm_start=arm_start, arm_posture=arm_posture, arm_velocity=arm_velocity,
                measured_arm_position=qpos[22:], measured_arm_velocity=qvel[21:],
                placement_s=expected_jet[0], placement_ds=expected_jet[1],
                placement_dds=expected_jet[2], kp=100., kd=20., weight=2.)
            np.testing.assert_array_equal(result.jacobian, direct.jacobian)
            np.testing.assert_array_equal(result.target, direct.target)
        self.assertEqual(clock_calls, [(1.1, 3.7)])
        self.assertEqual(jet_calls, ["right", "left"]*3)
        for delta in delta_seen:
            np.testing.assert_array_equal(delta, -(legs-c.leg_origin))
        np.testing.assert_array_equal(qpos, before_qpos)
        np.testing.assert_array_equal(qvel, before_qvel)
        np.testing.assert_array_equal(c.previous_leg_rate, history)
        np.testing.assert_array_equal(c.arm_posture, arm_posture)
        np.testing.assert_array_equal(c.arm_velocity, arm_velocity)
        self.assertEqual(c.previous_time, 17.)


if __name__ == '__main__': unittest.main()
