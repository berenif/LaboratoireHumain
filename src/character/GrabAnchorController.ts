import type { RigidBody } from "@dimforge/rapier3d-compat";
import type { Vec3 } from "../core/types";
import { add, clampLength, cross, dot, length, normalize, scale, sub, worldPoint } from "./math";

/** Successor S1: frozen before its first corrected acceptance replay. */
export const GRAB_CONTROL_LIMITS = Object.freeze({
  angularFrequencyRadPerS: 16,
  targetSpeedMps: 1.5,
  targetAccelerationMps2: 8,
  forceN: 180,
  torqueNm: 12,
  positivePowerW: 36,
});

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };

export interface GrabControlDiagnostics {
  active: boolean;
  rawTarget: Vec3;
  controlTarget: Vec3;
  targetVelocity: Vec3;
  targetError: Vec3;
  anchorWorld: Vec3;
  anchorVelocity: Vec3;
  force: Vec3;
  impulse: Vec3;
  torque: Vec3;
  angularImpulse: Vec3;
  bodyLinearVelocity: Vec3;
  bodyAngularVelocity: Vec3;
  effectiveMassKg: number;
  targetSpeedMps: number;
  selectedAnchorErrorM: number;
  injectedWorkJ: number;
  cumulativeInjectedWorkJ: number;
  storedUserForceN: number;
  storedUserTorqueNm: number;
  linearImpulseLimitNs: number;
  angularImpulseLimitNms: number;
  positiveWorkLimitJ: number;
}

export interface HandoffDiagnostics {
  sequence: number;
  maxTranslationErrorM: number;
  maxAngularErrorDegrees: number;
  selectedAnchorErrorM: number;
  rawTargetErrorM: number;
  localAnchorErrorM: number;
  jointSeparationM: number;
  targetDerivativeSpeedMps: number;
}

export function emptyGrabDiagnostics(cumulativeInjectedWorkJ = 0): GrabControlDiagnostics {
  return {
    active: false, rawTarget: ZERO, controlTarget: ZERO, targetVelocity: ZERO,
    targetError: ZERO, anchorWorld: ZERO, anchorVelocity: ZERO,
    force: ZERO, impulse: ZERO, torque: ZERO, angularImpulse: ZERO,
    bodyLinearVelocity: ZERO, bodyAngularVelocity: ZERO,
    effectiveMassKg: 0, targetSpeedMps: 0, selectedAnchorErrorM: 0,
    injectedWorkJ: 0, cumulativeInjectedWorkJ, storedUserForceN: 0,
    storedUserTorqueNm: 0, linearImpulseLimitNs: 0, angularImpulseLimitNms: 0,
    positiveWorkLimitJ: 0,
  };
}

/** Owns only the control target and a one-step impulse, never a body transform. */
export class GrabAnchorController {
  private controlTarget: Vec3;
  private targetVelocity: Vec3 = ZERO;
  private cumulativeInjectedWorkJ = 0;

  constructor(unchangedRawTarget: Vec3) {
    this.controlTarget = { ...unchangedRawTarget };
  }

  apply(body: RigidBody, localAnchor: Vec3, rawTarget: Vec3, dt: number): GrabControlDiagnostics {
    const remaining = sub(rawTarget, this.controlTarget);
    const distance = length(remaining);
    const brakingSpeed = Math.sqrt(2 * GRAB_CONTROL_LIMITS.targetAccelerationMps2 * distance);
    const desiredVelocity = scale(normalize(remaining, ZERO), Math.min(
      GRAB_CONTROL_LIMITS.targetSpeedMps, brakingSpeed, distance / dt,
    ));
    this.targetVelocity = add(this.targetVelocity, clampLength(
      sub(desiredVelocity, this.targetVelocity), GRAB_CONTROL_LIMITS.targetAccelerationMps2 * dt,
    ));
    // S1-F1: arriving at a target must not snap velocity after the acceleration
    // clamp. Integrate the clamped velocity through arrival as well.
    this.controlTarget = add(this.controlTarget, scale(this.targetVelocity, dt));

    const anchor = worldPoint(body.translation(), body.rotation(), localAnchor);
    const radius = sub(anchor, body.worldCom());
    const bodyLinearVelocity = { ...body.linvel() };
    const bodyAngularVelocity = { ...body.angvel() };
    const anchorVelocity = add(bodyLinearVelocity, cross(bodyAngularVelocity, radius));
    const inverseMass = body.effectiveInvMass();
    const inverseInertia = body.effectiveWorldInvInertia();
    const multiplyInertia = (v: Vec3): Vec3 => ({
      x: inverseInertia.m11 * v.x + inverseInertia.m12 * v.y + inverseInertia.m13 * v.z,
      y: inverseInertia.m21 * v.x + inverseInertia.m22 * v.y + inverseInertia.m23 * v.z,
      z: inverseInertia.m31 * v.x + inverseInertia.m32 * v.y + inverseInertia.m33 * v.z,
    });
    // Point inverse mass K = M^-1 - [r]x Iworld^-1 [r]x.
    const pointResponse = (j: Vec3): Vec3 => add({
      x: inverseMass.x * j.x, y: inverseMass.y * j.y, z: inverseMass.z * j.z,
    }, cross(multiplyInertia(cross(radius, j)), radius));
    const kx = pointResponse({ x: 1, y: 0, z: 0 });
    const ky = pointResponse({ x: 0, y: 1, z: 0 });
    const kz = pointResponse({ x: 0, y: 0, z: 1 });
    const determinant = dot(kx, cross(ky, kz));
    if (!Number.isFinite(determinant) || determinant <= 1e-12) {
      throw new Error("INVALID_GRAB_EFFECTIVE_MASS");
    }

    const error = sub(this.controlTarget, anchor);
    const omega = GRAB_CONTROL_LIMITS.angularFrequencyRadPerS;
    const desiredDeltaVelocity = scale(add(
      scale(error, omega * omega),
      scale(sub(this.targetVelocity, anchorVelocity), 2 * omega),
    ), dt / (1 + 2 * omega * dt + omega * omega * dt * dt));
    let impulse: Vec3 = {
      x: dot(desiredDeltaVelocity, cross(ky, kz)) / determinant,
      y: dot(kx, cross(desiredDeltaVelocity, kz)) / determinant,
      z: dot(kx, cross(ky, desiredDeltaVelocity)) / determinant,
    };
    const linearImpulseLimitNs = GRAB_CONTROL_LIMITS.forceN * dt;
    const angularImpulseLimitNms = GRAB_CONTROL_LIMITS.torqueNm * dt;
    const positiveWorkLimitJ = GRAB_CONTROL_LIMITS.positivePowerW * dt;
    impulse = clampLength(impulse, linearImpulseLimitNs);
    const angularSize = length(cross(radius, impulse));
    if (angularSize > angularImpulseLimitNms) impulse = scale(impulse, angularImpulseLimitNms / angularSize);

    // Exact kinetic energy change of this impulse before constraints solve.
    // Damping (negative work) is allowed; positive injection is power bounded.
    const quadratic = 0.5 * dot(impulse, pointResponse(impulse));
    const linear = dot(impulse, anchorVelocity);
    if (linear + quadratic > positiveWorkLimitJ) {
      const discriminant = Math.sqrt(linear * linear + 4 * quadratic * positiveWorkLimitJ);
      const workScale = linear >= 0
        ? 2 * positiveWorkLimitJ / (linear + discriminant)
        : (-linear + discriminant) / (2 * quadratic);
      impulse = scale(impulse, Math.max(0, Math.min(1, workScale)));
    }
    const angularImpulse = cross(radius, impulse);
    const injectedWorkJ = Math.max(0, dot(impulse, anchorVelocity) + 0.5 * dot(impulse, pointResponse(impulse)));
    this.cumulativeInjectedWorkJ += injectedWorkJ;
    const direction = normalize(impulse, normalize(error));
    const effectiveMassKg = 1 / Math.max(1e-9, dot(direction, pointResponse(direction)));
    body.applyImpulseAtPoint(impulse, anchor, true);
    return {
      active: true,
      rawTarget: { ...rawTarget }, controlTarget: { ...this.controlTarget },
      targetVelocity: { ...this.targetVelocity }, targetError: error,
      anchorWorld: anchor, anchorVelocity, force: scale(impulse, 1 / dt), impulse,
      torque: scale(angularImpulse, 1 / dt), angularImpulse,
      bodyLinearVelocity, bodyAngularVelocity, effectiveMassKg,
      targetSpeedMps: length(this.targetVelocity), selectedAnchorErrorM: length(sub(rawTarget, anchor)),
      injectedWorkJ, cumulativeInjectedWorkJ: this.cumulativeInjectedWorkJ,
      storedUserForceN: length(body.userForce()), storedUserTorqueNm: length(body.userTorque()),
      linearImpulseLimitNs, angularImpulseLimitNms, positiveWorkLimitJ,
    };
  }
}
