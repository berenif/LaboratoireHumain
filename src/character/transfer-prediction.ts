import { SEGMENT_BY_ID } from "../core/humanoid";
import type { SupportingContact, Vec3 } from "../core/types";
import { add, clampLength, scale, sub } from "./math";
import { distributeSupportLoad } from "./support-loads";

const GRAVITY = 9.81;
const DT = 1 / 60;
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const horizontal = (value: Vec3): Vec3 => ({ x: value.x, y: 0, z: value.z });
const finite = (value: Vec3): boolean => [value.x, value.y, value.z].every(Number.isFinite);

export interface TransferPredictionInput {
  centerOfMass: Vec3;
  centerOfMassVelocity: Vec3;
  /** Current measured patches; unqualified and non-sole contacts are excluded. */
  contacts: readonly SupportingContact[];
  retainedSide: "left" | "right";
  /** Candidate retained ankle plus the controller's neutral COM offset. */
  desiredCenterOfMass: Vec3;
  externalForce: Vec3;
  massKg: number;
  maxBalanceAccelerationMps2?: number;
  stepDurationS?: number;
  maxTransferDurationS?: number;
  readinessDurationS?: number;
  /** Measured continuous readiness for this retained-side candidate only. */
  readinessAgeS?: number;
  /** Match bounded stance control that can use finite hip/torso moment. */
  allowSupportMoment?: boolean;
}

export type TransferPrediction = {
  feasible: true;
  touchdownCapturePoint: Vec3;
  touchdownCenterOfMass: Vec3;
  touchdownVelocity: Vec3;
  transferDurationS: number;
} | {
  feasible: false;
  reason: "invalid-input" | "no-qualified-support" | "no-retained-support" | "transfer-timeout" | "force-budget";
  transferDurationS: number;
  lastCapturePoint: Vec3;
  lastVelocity: Vec3;
};

/**
 * A candidate-specific horizontal forecast, never a physical support claim.
 * Assumptions: constant measured COM height, fixed measured contact patches,
 * constant external force, body-weight normal reaction and no centroidal
 * angular-momentum strategy. Joint tracking, friction and touchdown geometry
 * are not simulated; runtime measured load/unload/touchdown gates still apply.
 * A feasible forecast may leave capture outside retained support: the caller
 * must validate that a reachable landing footprint catches touchdown capture.
 */
export function predictTransferTouchdown(input: TransferPredictionInput): TransferPrediction {
  const maxAcceleration = input.maxBalanceAccelerationMps2 ?? 3.6;
  const swingDuration = input.stepDurationS ?? 0.48;
  const maxTransferDuration = input.maxTransferDurationS ?? 1.2;
  const readinessDuration = input.readinessDurationS ?? 0.10;
  const readinessAge = input.readinessAgeS ?? 0;
  let position = { ...input.centerOfMass };
  let velocity = horizontal(input.centerOfMassVelocity);
  let omega = 1;
  let transferDurationS = 0;
  const capturePoint = (): Vec3 => add(horizontal(position), scale(velocity, 1 / omega));
  const failed = (reason: Extract<TransferPrediction, { feasible: false }>['reason']): TransferPrediction => ({
    feasible: false, reason, transferDurationS,
    lastCapturePoint: capturePoint(), lastVelocity: { ...velocity },
  });
  if (![input.centerOfMass, input.centerOfMassVelocity, input.desiredCenterOfMass, input.externalForce].every(finite)
    || ![input.massKg, maxAcceleration, swingDuration, maxTransferDuration, readinessDuration]
      .every(value => Number.isFinite(value) && value > 0)
    || !Number.isFinite(readinessAge) || readinessAge < 0) return failed("invalid-input");

  const contacts = input.contacts.filter(contact => {
    const definition = SEGMENT_BY_ID.get(contact.segment);
    return (definition?.role === "hindfoot" || definition?.role === "forefoot")
      && contact.loadBearing && contact.normalY >= 0.65 && contact.forceN >= 3
      && contact.persistenceS >= 0.05 && finite(contact.point)
      && (!contact.points?.length || contact.points.every(finite));
  });
  if (!contacts.length) return failed("no-qualified-support");
  const retained = contacts.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === input.retainedSide);
  const patchPoints = (patches: readonly SupportingContact[]) => patches.flatMap(contact => contact.points?.length
    ? contact.points : [contact.point]);
  const retainedPoints = patchPoints(retained);
  // A measured point or line supplies constrained pressure, not an invented
  // support area. Its dynamics are forecast as-is; only absent support fails.
  if (!retainedPoints.length) return failed("no-retained-support");
  const floor = Math.min(...contacts.flatMap(contact => contact.points?.length
    ? contact.points.map(point => point.y) : [contact.point.y]));
  const height = input.centerOfMass.y - floor;
  if (!Number.isFinite(height) || height <= 0) return failed("invalid-input");
  omega = Math.sqrt(GRAVITY / height);
  const weight = input.massKg * GRAVITY;
  const externalAcceleration = scale(horizontal(input.externalForce), 1 / input.massKg);

  // The helper's triangle-fan load ownership depends on its hull start vertex.
  // Use a candidate-aligned frame so rotating identical measured manifolds
  // cannot change per-side shares (and therefore readiness) via world-X sorting.
  const mean = (points: readonly Vec3[]) => scale(points.reduce(add, ZERO), 1 / points.length);
  const retainedCenter = mean(retainedPoints);
  const movingPoints = patchPoints(contacts.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side !== input.retainedSide));
  let direction = movingPoints.length ? horizontal(sub(retainedCenter, mean(movingPoints))) : ZERO;
  if (Math.hypot(direction.x, direction.z) < 1e-9) {
    const distinct = [...retainedPoints, ...movingPoints].find(point =>
      Math.hypot(point.x - retainedCenter.x, point.z - retainedCenter.z) > 1e-9);
    // For a single unique support point, pressure is fixed in every frame.
    direction = distinct ? horizontal(sub(distinct, retainedCenter)) : { x: 1, y: 0, z: 0 };
  }
  const axis = scale(direction, 1 / Math.hypot(direction.x, direction.z));
  const toFrame = (point: Vec3): Vec3 => ({ x: axis.x * point.x + axis.z * point.z,
    y: point.y, z: -axis.z * point.x + axis.x * point.z });
  const fromFrame = (point: Vec3): Vec3 => ({ x: axis.x * point.x - axis.z * point.z,
    y: point.y, z: axis.z * point.x + axis.x * point.z });
  const framedContacts = contacts.map(contact => ({ ...contact, point: toFrame(contact.point),
    ...(contact.points ? { points: contact.points.map(toFrame) } : {}) }));
  const framedRetained = framedContacts.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === input.retainedSide);

  const intent = (transferring: boolean) => {
    const controlPoint = transferring ? capturePoint() : horizontal(position);
    const requestedAcceleration = clampLength(add(
      scale(sub(horizontal(input.desiredCenterOfMass), controlPoint), 34),
      scale(velocity, -8.5),
    ), maxAcceleration);
    let pressure = input.allowSupportMoment && transferring
      ? horizontal(input.desiredCenterOfMass)
      : sub(horizontal(position), scale(requestedAcceleration, height / GRAVITY));
    if (input.allowSupportMoment && transferring) pressure = distributeSupportLoad(retained, pressure,
      { rotationInvariant: true }).reduce((sum, load) => add(sum, scale(load.point, load.share)), ZERO);
    const loads = distributeSupportLoad(transferring ? framedContacts : framedRetained, toFrame(pressure),
      { rotationInvariant: true, ...(input.allowSupportMoment && transferring
        ? { preferredSegments: new Set(retained.map(contact => contact.segment)) } : {}) })
      .map(load => ({ ...load, point: fromFrame(load.point) }));
    const projectedPressure = loads.reduce((sum, load) => add(sum, scale(load.point, load.share)), ZERO);
    const supportAcceleration = scale(horizontal(sub(position, projectedPressure)), GRAVITY / height);
    return { loads, withinForceBudget: input.allowSupportMoment
      || Math.hypot(supportAcceleration.x, supportAcceleration.z) <= maxAcceleration + 1e-9,
      acceleration: add(input.allowSupportMoment ? requestedAcceleration : supportAcceleration, externalAcceleration) };
  };
  const advance = (acceleration: Vec3, dt: number) => {
    velocity = add(velocity, scale(acceleration, dt));
    position = add(position, scale(velocity, dt));
  };

  const ready = (plan: ReturnType<typeof intent>) => {
    const retainedLoad = plan.loads.reduce((sum, load) => sum
      + (SEGMENT_BY_ID.get(load.segment)?.side === input.retainedSide ? load.share * weight : 0), 0);
    const movingLoad = Math.max(0, weight - retainedLoad);
    return retainedLoad >= 0.52 * weight - 1e-8 && movingLoad <= 0.25 * weight + 1e-8;
  };
  const initialPlan = intent(true);
  if (!initialPlan.withinForceBudget) return failed("force-budget");
  const measuredRetainedLoad = retained.reduce((sum, contact) => sum + contact.forceN, 0);
  const measuredMovingLoad = input.contacts.reduce((sum, contact) => {
    const definition = SEGMENT_BY_ID.get(contact.segment);
    return sum + (definition?.side !== input.retainedSide && contact.normalY >= 0.65
      && (definition?.role === "hindfoot" || definition?.role === "forefoot") ? contact.forceN : 0);
  }, 0);
  const measuredReady = measuredRetainedLoad >= 0.52 * weight && measuredMovingLoad <= 0.25 * weight;
  // Measured persistence belongs to the candidate, not the instantaneous
  // pressure allocation of this reduced model. Replan the remaining interval
  // from current motion; runtime still resets history on any actual load loss.
  let readyAge = measuredReady ? Math.min(readinessAge, readinessDuration) : 0;
  let transferred = readyAge >= readinessDuration - 1e-10;
  while (!transferred && transferDurationS < maxTransferDuration - 1e-10) {
    const plan = intent(true);
    if (!plan.withinForceBudget) return failed("force-budget");
    const loadsReady = measuredReady || ready(plan);
    const dt = Math.min(DT, maxTransferDuration - transferDurationS,
      loadsReady ? readinessDuration - readyAge : DT);
    readyAge = loadsReady ? readyAge + dt : 0;
    advance(plan.acceleration, dt);
    transferDurationS += dt;
    // Recheck load qualification at the endpoint of the persistence interval.
    // Capture need not fit retained support during a dynamic recovery step.
    if (readyAge >= readinessDuration - 1e-10) {
      const endpoint = intent(true);
      if (!endpoint.withinForceBudget) return failed("force-budget");
      if (measuredReady || ready(endpoint)) { transferred = true; break; }
      readyAge = 0;
    }
  }
  if (!transferred) return failed("transfer-timeout");

  let swingAge = 0;
  while (swingAge < swingDuration - 1e-10) {
    const dt = Math.min(DT, swingDuration - swingAge);
    const plan = intent(false);
    if (!plan.withinForceBudget) return failed("force-budget");
    advance(plan.acceleration, dt);
    swingAge += dt;
  }
  return { feasible: true, touchdownCapturePoint: capturePoint(),
    touchdownCenterOfMass: { ...position }, touchdownVelocity: { ...velocity }, transferDurationS };
}
