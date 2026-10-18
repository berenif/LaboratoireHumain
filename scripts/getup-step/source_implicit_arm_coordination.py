"""Opt-in same-QP arm/thigh acceleration task; read-only kinematic construction.

Integration point in an isolated copy of Controller.solve, after the existing
OpposedArmController has populated arm_posture and arm_velocity:

    arm = implicit_arm_task(self, t, phase)
    task(arm.jacobian, arm.target, arm.weight)

Replace only the existing hasattr(self, 'arm_kp') arm task. Do not add this
beside that task. Keep every other objective, constraint and runtime audit.
The generic posture task is deliberately untouched.
"""
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


SIDE_ORDER = ("right", "left")


@dataclass(frozen=True)
class ImplicitArmTask:
    jacobian: np.ndarray
    target: np.ndarray
    weight: float
    telemetry: dict[str, Any]


def _array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


def build_implicit_arm_task(*, thigh_jacobian, thigh_bias, measured_leg_rate,
                            mapped, mapping_jacobian, mapping_hessian,
                            arm_start, arm_posture, arm_velocity,
                            measured_arm_position, measured_arm_velocity,
                            placement_s, placement_ds, placement_dds,
                            kp, kd, weight):
    """Pure algebra for the original 27-velocity, six-arm-coordinate model.

    delta = -(theta - leg_origin), qref = arm_start+s*(g(delta)-arm_start).
    theta_ddot = Jtheta @ qdd + btheta.

    Move -s*gprime*Jtheta@qdd from the original arm target to its task
    Jacobian. This preserves the residual of that arm task exactly when its
    acceleration target uses the true current theta_ddot, including placement.
    Existing measured angle targets, mapped velocities, gains and weight are
    inputs and are never clipped, filtered or changed here.
    """
    jt = _array(thigh_jacobian, (2, 27), "thigh_jacobian")
    bt = _array(thigh_bias, (2,), "thigh_bias")
    rate = _array(measured_leg_rate, (2,), "measured_leg_rate")
    mapped = _array(mapped, (6,), "mapped")
    gp = _array(mapping_jacobian, (6,), "mapping_jacobian")
    gpp = _array(mapping_hessian, (6,), "mapping_hessian")
    start = _array(arm_start, (6,), "arm_start")
    posture = _array(arm_posture, (6,), "arm_posture")
    velocity = _array(arm_velocity, (6,), "arm_velocity")
    qarm = _array(measured_arm_position, (6,), "measured_arm_position")
    varm = _array(measured_arm_velocity, (6,), "measured_arm_velocity")
    s, ds, dds, kp, kd, weight = map(float,
        (placement_s, placement_ds, placement_dds, kp, kd, weight))
    if not np.isfinite([s, ds, dds, kp, kd, weight]).all():
        raise ValueError("placement and gains must be finite")
    if not 0. <= s <= 1. or min(kp, kd) < 0. or weight <= 0.:
        raise ValueError("invalid placement fraction, gain or task weight")

    # Mapping order is right shoulder1/2/elbow, left shoulder1/2/elbow.
    repeated_jt = np.repeat(jt, 3, axis=0)
    repeated_bt = np.repeat(bt, 3)
    repeated_rate = np.repeat(rate, 3)
    mapped_velocity = -gp * repeated_rate
    acceleration_bias = (dds * (mapped-start) + 2.*ds*mapped_velocity
                         + s*(gpp*repeated_rate**2-gp*repeated_bt))
    pd = kp*(posture-qarm) + kd*(velocity-varm)
    selector = np.eye(27)[21:27]
    jacobian = selector + s*gp[:, None]*repeated_jt
    target = acceleration_bias + pd
    return ImplicitArmTask(jacobian, target, weight, {
        "mode": "same-QP-implicit-measured-thigh-acceleration",
        "sideOrder": list(SIDE_ORDER),
        "placementFraction": s, "placementRatePerS": ds,
        "placementAccelerationPerS2": dds,
        "positionGain": kp, "velocityGain": kd, "taskWeight": weight,
        "finiteDifferenceAccelerationUsed": False,
        "measuredLegRateRadps": rate.tolist(),
        "thighAccelerationBiasRadps2": bt.tolist(),
        "armAccelerationBiasRadps2": acceleration_bias.tolist(),
        "targetClippingEnabled": False,
    })


def implicit_arm_task(controller, t, phase="transfer", *,
                      thigh_jet_fn: Callable | None = None,
                      limb_telemetry_fn: Callable | None = None,
                      clock_fn: Callable | None = None):
    """Read-only controller adapter using the existing tested kinematic jet.

    Call after the outer OpposedArmController.solve has set arm_posture and
    arm_velocity, inside its call to base Controller.solve. This adapter does
    not call controller.solve, update differentiation history or use qacc.
    Native model/state, controls, limits and integration counters are untouched.

    Normal use needs scripts/getup-step on sys.path. Dependency arguments
    permit tests without loading a native library or creating a World.
    """
    if thigh_jet_fn is None:
        from source_cadenced_ten_steps import thigh_pitch_jet
        thigh_jet_fn = thigh_pitch_jet
    if limb_telemetry_fn is None:
        from source_opposed_arm_swing import limb_telemetry
        limb_telemetry_fn = limb_telemetry
    if clock_fn is None:
        from source_step_probe import clock
        clock_fn = clock

    w = controller.w
    legs, rate, _ = limb_telemetry_fn(w, "leg")
    delta = -(np.asarray(legs)-np.asarray(controller.leg_origin))
    mapped, gp, gpp = controller._mapping.evaluate(delta)
    jets = [thigh_jet_fn(w, side) for side in SIDE_ORDER]
    jt = np.array([jet[1] for jet in jets], dtype=float)
    bt = np.array([jet[2] for jet in jets], dtype=float)
    # Use exactly the original initial-placement switch and duration.
    if controller.placement and phase == "transfer":
        duration = getattr(controller, "arm_placement_duration", 4.)
        s, ds, dds = clock_fn(t, duration)
    else:
        s, ds, dds = 1., 0., 0.
    q = np.asarray(w.arr(0, (28,)))[7:]
    v = np.asarray(w.arr(1, (27,)))
    return build_implicit_arm_task(
        thigh_jacobian=jt, thigh_bias=bt, measured_leg_rate=rate,
        mapped=mapped, mapping_jacobian=gp, mapping_hessian=gpp,
        arm_start=controller.arm_start,
        arm_posture=controller.arm_posture,
        arm_velocity=controller.arm_velocity,
        measured_arm_position=q[15:], measured_arm_velocity=v[21:],
        placement_s=s, placement_ds=ds, placement_dds=dds,
        kp=controller.arm_kp, kd=controller.arm_kd,
        weight=controller.arm_weight)
