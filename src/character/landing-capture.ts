import { SEGMENT_BY_ID } from "../core/humanoid";
import type { SupportingContact, Vec3 } from "../core/types";
import { minimumSupportTorqueLimit, planContactLoads } from "./contact-loads";
import { add, clampLength, scale, sub } from "./math";

export interface LandingCaptureInput {
  centerOfMass: Readonly<Vec3>;
  centerOfMassVelocity: Readonly<Vec3>;
  /** Filtered measured acceleration, including any external force response. */
  centerOfMassAcceleration: Readonly<Vec3>;
  floorY: number;
  stepDurationS: number;
  /** One shared ramp distance for all candidates being compared. */
  transferRampDistanceM: number;
  transferSpeedMps: number;
  /** Continuous qualified measured history; the caller resets invalid history. */
  readinessAgeS: number;
  remainingTransferWindowS: number;
  readinessDurationS?: number;
}

export type LandingCapturePrediction = {
  valid: true;
  horizonS: number;
  acceleratedCapturePoint: Vec3;
  constantVelocityCapturePoint: Vec3;
} | { valid: false; reason: string };

const finiteVector = (value: Readonly<Vec3>): boolean =>
  [value.x, value.y, value.z].every(Number.isFinite);

/**
 * Common measured-motion forecasts for comparing landing footprints.
 * The bounded ramp/persistence horizon is a timing estimate, not proof that
 * transfer or touchdown is physically feasible. No candidate-specific target,
 * support allocation or external force is added to the measured motion.
 * Constant height determines omega; returned horizontal points have y=0.
 */
export function predictLandingCapture(input: LandingCaptureInput): LandingCapturePrediction {
  const readinessDurationS = input.readinessDurationS ?? 0.10;
  if (![input.centerOfMass, input.centerOfMassVelocity, input.centerOfMassAcceleration].every(finiteVector)
    || ![input.floorY, input.stepDurationS, input.transferRampDistanceM, input.transferSpeedMps,
      input.readinessAgeS, input.remainingTransferWindowS, readinessDurationS].every(Number.isFinite)
    || input.stepDurationS <= 0 || input.transferSpeedMps <= 0 || readinessDurationS <= 0
    || input.transferRampDistanceM < 0 || input.readinessAgeS < 0 || input.remainingTransferWindowS < 0) {
    return { valid: false, reason: "invalid-input" };
  }
  const height = input.centerOfMass.y - input.floorY;
  const omega = Math.sqrt(9.81 / height);
  if (height <= 0 || !Number.isFinite(omega) || omega <= 0) {
    return { valid: false, reason: "invalid-height" };
  }
  // Match the runtime persistence gate's floating-point tolerance. Qualified
  // readiness already achieved must not acquire another ramp interval.
  const ready = input.readinessAgeS + 1e-9 >= readinessDurationS;
  const transferS = ready ? 0 : Math.min(input.remainingTransferWindowS,
    input.transferRampDistanceM / input.transferSpeedMps
      + Math.max(0, readinessDurationS - input.readinessAgeS));
  const horizonS = input.stepDurationS + transferS;
  const constantVelocityCapturePoint = { x: 0, y: 0, z: 0 };
  const acceleratedCapturePoint = { x: 0, y: 0, z: 0 };
  for (const coordinate of ["x", "z"] as const) {
    const position = input.centerOfMass[coordinate];
    const velocity = input.centerOfMassVelocity[coordinate];
    const acceleration = input.centerOfMassAcceleration[coordinate];
    constantVelocityCapturePoint[coordinate] = position + velocity * horizonS + velocity / omega;
    // Capture uses the future velocity as well as the future COM position.
    acceleratedCapturePoint[coordinate] = position + velocity * horizonS
      + 0.5 * acceleration * horizonS ** 2 + (velocity + acceleration * horizonS) / omega;
  }
  if (!Number.isFinite(horizonS) || !finiteVector(constantVelocityCapturePoint)
    || !finiteVector(acceleratedCapturePoint)) {
    return { valid: false, reason: "non-finite-prediction" };
  }
  return { valid: true, horizonS, acceleratedCapturePoint, constantVelocityCapturePoint };
}

export interface ControlledLandingCaptureInput extends Omit<LandingCaptureInput, "centerOfMassAcceleration"> {
  /** Current support target plus the neutral COM offset, in world coordinates. */
  currentDesiredCenterOfMass: Readonly<Vec3>;
  /** Existing committed target; initial selection keeps the current double-support target. */
  committedDesiredCenterOfMass: Readonly<Vec3>;
  contacts: readonly SupportingContact[];
  /** Omit before initial selection so every candidate shares double-support motion. */
  retainedSide?: "left" | "right";
  externalForce: Readonly<Vec3>;
  massKg: number;
  maxBalanceAccelerationMps2?: number;
  frictionCoefficient?: number;
}

export type ControlledLandingCapturePrediction = {
  valid: true;
  horizonS: number;
  capturePoint: Vec3;
  centerOfMass: Vec3;
  velocity: Vec3;
  pressureFeasible: boolean;
  maxPressureForceResidualNm: number;
} | { valid: false; reason: string };

/**
 * Finite rollout of the committed command through the runtime contact allocator.
 * Height, external force and measured patches stay fixed; vertical reaction is
 * bodyweight. A selected-side swing is conditional on the existing 52% normal
 * load launch gate, rather than retaining a weak pretransfer load as its cap.
 * This predicts neither contact creation nor actual motor delivery.
 * Estimated launch time never overrides the runtime measured readiness gate.
 */
export function predictControlledLandingCapture(input: ControlledLandingCaptureInput): ControlledLandingCapturePrediction {
  const zero: Vec3 = { x: 0, y: 0, z: 0 };
  const horizontal = (value: Readonly<Vec3>): Vec3 => ({ x: value.x, y: 0, z: value.z });
  const timing = predictLandingCapture({ ...input, centerOfMassAcceleration: zero });
  if (!timing.valid) return timing;
  const maxAcceleration = input.maxBalanceAccelerationMps2 ?? 3.6;
  const frictionCoefficient = input.frictionCoefficient ?? 1.2;
  if (![input.currentDesiredCenterOfMass, input.committedDesiredCenterOfMass, input.externalForce].every(finiteVector)
    || !Number.isFinite(input.massKg) || input.massKg <= 0
    || !Number.isFinite(maxAcceleration) || maxAcceleration <= 0
    || !Number.isFinite(frictionCoefficient) || frictionCoefficient < 0
    || (input.retainedSide !== undefined && !["left", "right"].includes(input.retainedSide))) {
    return { valid: false, reason: "invalid-input" };
  }
  // Guard malformed durations without letting a pure planning call run forever.
  // Normal balance horizons are at most 1.2 + .48 seconds (101 integration steps).
  if (Math.ceil(timing.horizonS * 60) > 10000) return { valid: false, reason: "prediction-window-too-long" };
  const contacts = input.contacts.filter(contact => {
    const role = SEGMENT_BY_ID.get(contact.segment)?.role;
    return (role === "hindfoot" || role === "forefoot") && contact.loadBearing
      && contact.normalY >= 0.65 && contact.forceN >= 3;
  });
  if (!contacts.length) return { valid: false, reason: "no-qualified-support" };
  if (contacts.some(contact => !Number.isFinite(contact.forceN) || !Number.isFinite(contact.normalY)
    || !finiteVector(contact.point) || contact.points?.some(point => !finiteVector(point)))) {
    return { valid: false, reason: "invalid-input" };
  }
  const swingContacts = input.retainedSide === undefined ? contacts
    : contacts.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === input.retainedSide);
  if (!swingContacts.length) return { valid: false, reason: "no-retained-support" };
  const weight = input.massKg * 9.81;
  const omega = Math.sqrt(9.81 / Math.max(0.4, input.centerOfMass.y - input.floorY));
  const transferS = timing.horizonS - input.stepDurationS;
  const externalAcceleration = scale(horizontal(input.externalForce), 1 / input.massKg);
  const goal = horizontal(input.committedDesiredCenterOfMass);
  let desired = horizontal(input.currentDesiredCenterOfMass);
  let position = { ...input.centerOfMass };
  let velocity = horizontal(input.centerOfMassVelocity);
  let pressureFeasible = true, maxPressureForceResidualNm = 0;
  for (const phase of [{ duration: transferS, contacts, transferring: true },
    { duration: input.stepDurationS, contacts: swingContacts, transferring: false }]) {
    const measuredFraction = phase.contacts.reduce((sum, contact) =>
      sum + contact.forceN * contact.normalY, 0) / weight;
    // Match the runtime's 0.52 * bodyweight retained-normal-load threshold.
    // The cap uses vertical load, so tilted normals reduce its guaranteed
    // launch component. This conditional future budget never changes measured
    // forces, creates contacts, or grants actual readiness to an unloaded leg.
    const launchFraction = !phase.transferring && input.retainedSide !== undefined
      ? 0.52 * Math.min(...phase.contacts.map(contact => contact.normalY)) : 0;
    const fraction = Math.min(1, Math.max(measuredFraction, launchFraction));
    const torqueLimit = minimumSupportTorqueLimit(phase.contacts);
    for (let elapsed = 0; elapsed < phase.duration - 1e-12;) {
      const dt = Math.min(1 / 60, phase.duration - elapsed);
      desired = phase.transferring
        ? add(desired, clampLength(sub(goal, desired), input.transferSpeedMps * dt)) : { ...goal };
      const controlPoint = phase.transferring
        ? add(horizontal(position), scale(velocity, 1 / omega)) : horizontal(position);
      const requested = clampLength(add(scale(sub(desired, controlPoint), 34), scale(velocity, -8.5)), maxAcceleration);
      const plan = planContactLoads(phase.contacts, position, velocity,
        { x: input.massKg * requested.x * fraction, y: weight, z: input.massKg * requested.z * fraction },
        { projectMeasuredPressure: true, allowSupportMoment: phase.transferring, frictionCoefficient,
          maxHorizontalForceN: input.massKg * maxAcceleration * fraction, maxJointTorqueNm: torqueLimit });
      pressureFeasible &&= plan.pressureFeasible !== false;
      maxPressureForceResidualNm = Math.max(maxPressureForceResidualNm, plan.pressureForceResidualNm ?? 0);
      const acceleration = add(scale(horizontal(plan.allocatedForce), 1 / input.massKg), externalAcceleration);
      velocity = add(velocity, scale(acceleration, dt));
      position = add(position, scale(velocity, dt));
      elapsed += dt;
    }
  }
  const capturePoint = add(horizontal(position), scale(velocity, 1 / omega));
  if (![capturePoint, position, velocity].every(finiteVector) || !Number.isFinite(maxPressureForceResidualNm)) {
    return { valid: false, reason: "non-finite-prediction" };
  }
  return { valid: true, horizonS: timing.horizonS, capturePoint, centerOfMass: position, velocity,
    pressureFeasible, maxPressureForceResidualNm };
}
