import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, TOTAL_MASS_KG } from "../core/humanoid";
import { geometryHalfExtents, lowestWorldPoint } from "../core/geometry";
import type { Quat, RegionId, SegmentId, SegmentPose, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, clampLength, dot, length, lerp, normalize, quatFromAxisAngle, rotate, scale, sub, worldPoint } from "./math";
import { hindfootFromAnkle } from "./leg-target-frame";
import { measureMassState } from "./mass-state";
import { findLandingPlan, validateLanding, type LandingForecast, type LandingInput, type LandingPlan } from "./landing-plan";
import { predictControlledLandingCapture } from "./landing-capture";
import { composeUprightPose, horizontal, midpoint, restPoseMap, type StepMotion } from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const RIGHT: Vec3 = { x: 1, y: 0, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const FEET = ["leftFoot", "rightFoot"] as const;
type Foot = (typeof FEET)[number];

function footHalfExtents(foot: Foot): Vec3 {
  const geometry = SEGMENT_BY_ID.get(foot)?.geometry;
  if (!geometry) throw new Error(`${foot} requires shared convex geometry`);
  return geometryHalfExtents(geometry);
}

function footCenterHeight(foot: Foot, floorY: number): number {
  return floorY + footHalfExtents(foot).y;
}

function footCorners(position: Vec3, rotation: SegmentPose["rotation"], foot: Foot): Vec3[] {
  const geometry = SEGMENT_BY_ID.get(foot)!.geometry;
  return (geometry.supportPatch ?? geometry.vertices.filter((vertex) =>
    Math.abs(vertex.y - geometry.localBounds.min.y) < 1e-6
  )).map((point) => worldPoint(position, rotation, point));
}

/** Horizontal ankle projection: a quieter COM target than the geometric sole centre. */
function ankleProjection(position: Vec3, rotation: SegmentPose["rotation"], foot: Foot): Vec3 {
  const ankle = SEGMENT_BY_ID.get(foot)?.jointAnchorChild ?? ZERO;
  return horizontal(worldPoint(position, rotation, ankle));
}

/** SI thresholds frozen by deterministic balance scenarios. */
export const BALANCE_LIMITS = Object.freeze({
  floorClearanceM: 0.035,
  stanceTargetErrorM: 0.045,
  stancePersistenceS: 0.05,
  liftedGrabDistanceM: 0.05,
  pullSpringNpm: 620,
  pullDampingNsPm: 12,
  maxPullForceN: 620,
  maxBalanceAccelerationMps2: 3.6,
  maxRootSpeedMps: 2.5,
  supportTransferSpeedMps: 1.5,
  stepPredictionHorizonS: 1.4,
  stepTriggerMarginM: 0.14,
  maxStepReachM: 0.36,
  maxStepTravelM: 0.43,
  stepDurationS: 0.68,
  retainedWeightFraction: 0.52,
  minimumContactForceN: 3,
  minStepDurationS: 0.48,
  // Let the next correction begin as soon as the prior step has a qualified
  // loaded touchdown; measured retained load still guards every lift.
  stepCooldownS: 0,
  marginalInstabilityS: 0.12,
  unrecoverableMarginM: -0.43,
  marginalMarginM: -0.24,
  unrecoverableSpeedMps: 1.85,
});

export interface BalanceGrab {
  region: RegionId;
  segment?: SegmentId;
  localAnchor?: Vec3;
  target: Vec3;
  startTarget: Vec3;
  startSegmentPosition: Vec3;
  targetVelocity?: Vec3;
}

export interface BalanceDiagnostics {
  centerOfMass: Vec3;
  centerOfMassVelocity: Vec3;
  capturePoint: Vec3;
  supportCenter: Vec3;
  /** Feet available to the stance controller (excludes manipulation intent). */
  supportingFeet: Foot[];
  /** Actual environment-loaded feet, independent of planned swing or grab. */
  measuredSupportingFeet: Foot[];
  supportMarginM: number;
  instabilitySeconds: number;
  recoveryCapacityM: number;
  externalForce: Vec3;
  balanceAcceleration: Vec3;
  stepTarget: Vec3 | null;
  candidateValidity: { valid: boolean; reason: string | null; forecast?: LandingForecast };
  transferAgeS: number;
  transferReadyAgeS: number;
  transferCaptureMarginM: number | null;
}

export interface BalanceInput {
  dt: number;
  poses: ReadonlyMap<SegmentId, SegmentPose>;
  rootPosition: Vec3;
  activeGrab: BalanceGrab | null;
  /** Applied impulse / dt from the physical grab controller, not a second spring. */
  appliedGrabForce?: Vec3;
  floorY?: number;
  surfaceHeight?: (x: number, z: number) => number;
  heading?: number;
  /** Runtime supplies measured Rapier contacts; omission supports pose-only planning fixtures. */
  contacts?: readonly SupportingContact[];
  /** Compatibility alias for saved planning fixtures; appliedGrabForce wins. */
  externalForce?: Vec3;
}

export interface BalanceOutput {
  rootTarget: Vec3;
  reactionOffset: Vec3;
  kneeFlexion: number;
  supportFeet: Record<Foot, Vec3>;
  supportFootRotations: Record<Foot, Quat>;
  step: StepMotion | null;
  stepCount: number;
  state: "upright" | "reacting" | "stepping";
  shouldFall: boolean;
  fallDirection: Vec3;
  appliedGrabForceN: number;
  diagnostics: BalanceDiagnostics;
}

export function massState(poses: ReadonlyMap<SegmentId, SegmentPose>): { position: Vec3; velocity: Vec3 } {
  const { position, velocity } = measureMassState(poses.values());
  return { position, velocity };
}

function neutralComOffset(heading: number): Vec3 {
  const rest = restPoseMap();
  const poses = composeUprightPose({
    rootTranslation: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 },
    heading: 0,
    kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
    reactionOffset: ZERO,
    simulationTime: 0,
    activeGrab: null,
    supportFeet: {
      leftFoot: rest.get("leftFoot")!.position,
      rightFoot: rest.get("rightFoot")!.position,
    },
    step: null,
  }).poses;
  const ankleCenter = midpoint(
    ankleProjection(poses.get("leftFoot")!.position, poses.get("leftFoot")!.rotation, "leftFoot"),
    ankleProjection(poses.get("rightFoot")!.position, poses.get("rightFoot")!.rotation, "rightFoot"),
  );
  const offset = horizontal(sub(massState(poses).position, ankleCenter));
  return rotate(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading), offset);
}

function cross2(a: Vec3, b: Vec3, c: Vec3): number {
  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

function supportHull(points: Vec3[]): Vec3[] {
  const ordered = [...points].sort((a, b) => a.x - b.x || a.z - b.z);
  if (ordered.length < 3) return ordered;
  const lower: Vec3[] = [];
  for (const point of ordered) {
    while (lower.length > 1 && cross2(lower[lower.length - 2], lower[lower.length - 1], point) <= 0) lower.pop();
    lower.push(point);
  }
  const upper: Vec3[] = [];
  for (const point of [...ordered].reverse()) {
    while (upper.length > 1 && cross2(upper[upper.length - 2], upper[upper.length - 1], point) <= 0) upper.pop();
    upper.push(point);
  }
  return [...lower.slice(0, -1), ...upper.slice(0, -1)];
}

function polygonMargin(point: Vec3, polygon: Vec3[]): number {
  if (polygon.length < 3) return -1;
  let inside = true;
  let nearest = Infinity;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i];
    const b = polygon[(i + 1) % polygon.length];
    const edge = horizontal(sub(b, a));
    const t = clamp(dot(horizontal(sub(point, a)), edge) / Math.max(dot(edge, edge), 1e-9), 0, 1);
    nearest = Math.min(nearest, length(horizontal(sub(point, add(a, scale(edge, t))))));
    inside &&= cross2(a, b, point) >= -1e-8;
  }
  return inside ? nearest : -nearest;
}

/** Procedural balance owns stance intent and finite actuator capacity, never Rapier. */
export class BalanceController {
  private feet: Record<Foot, Vec3> = {
    leftFoot: { x: 0, y: footCenterHeight("leftFoot", 0), z: 0 },
    rightFoot: { x: 0, y: footCenterHeight("rightFoot", 0), z: 0 },
  };
  private stanceAge: Record<Foot, number> = { leftFoot: 0.05, rightFoot: 0.05 };
  private footRotations: Record<Foot, Quat> = {
    leftFoot: { x: 0, y: 0, z: 0, w: 1 }, rightFoot: { x: 0, y: 0, z: 0, w: 1 },
  };
  private step: StepMotion | null = null;
  private stepCount = 0;
  private cooldown = 0;
  private instability = 0;
  private velocity = ZERO;
  private comVelocity = ZERO;
  private comAcceleration = ZERO;
  private reaction = ZERO;
  private nextFoot: Foot = "leftFoot";
  private candidateValidity: { valid: boolean; reason: string | null; transferForecastReason?: string | null;
    forecast?: LandingForecast } = { valid: false, reason: "idle" };
  private liftedFoot: Foot | null = null;
  private touchdownAge = 0;
  private stepTransferAge = 0;
  private transferReadyAge = 0;
  private unloadReadyAge = 0;
  private stepHasUnloaded = false;
  private touchdownTargetAdjusted = false;
  private disturbanceSeen = false;
  private nominalHeight: number = HUMAN_PROPORTIONS.pelvis.centerHeightM;
  private neutralComOffset = ZERO;
  private neutralRootFromCom = ZERO;
  private supportTarget = ZERO;

  reset(poses: ReadonlyMap<SegmentId, SegmentPose>, heading = 0, floorY = 0): void {
    for (const foot of FEET) {
      this.feet[foot] = { ...poses.get(foot)!.position };
      this.footRotations[foot] = { ...poses.get(foot)!.rotation };
    }
    // composeUprightPose lowers its root by the requested knee flexion.  Store
    // the pre-flexion height here so resetting the controller from an already
    // composed physical pose does not apply that lowering a second time.
    this.nominalHeight = (poses.get("pelvis")?.position.y ?? HUMAN_PROPORTIONS.pelvis.centerHeightM)
      + HUMAN_PROPORTIONS.stance.neutralKneeFlexion * 0.10 - floorY;
    this.velocity = horizontal(poses.get("pelvis")?.linearVelocity ?? ZERO);
    const mass = massState(poses);
    this.comVelocity = mass.velocity;
    this.comAcceleration = ZERO;
    this.neutralComOffset = neutralComOffset(heading);
    this.neutralRootFromCom = horizontal(sub(
      poses.get("pelvis")?.position ?? { x: 0, y: this.nominalHeight, z: 0 },
      mass.position,
    ));
    this.supportTarget = midpoint(
      ankleProjection(poses.get("leftFoot")!.position, poses.get("leftFoot")!.rotation, "leftFoot"),
      ankleProjection(poses.get("rightFoot")!.position, poses.get("rightFoot")!.rotation, "rightFoot"),
    );
    this.step = null;
    this.stepCount = 0;
    // Let measured floor contacts persist through two stance-validation
    // windows before permitting an immediate corrective step. A long startup
    // delay leaves no time to recover from a steadily increasing pull.
    this.cooldown = BALANCE_LIMITS.stancePersistenceS * 2;
    this.instability = 0;
    this.reaction = ZERO;
    this.stanceAge = { leftFoot: BALANCE_LIMITS.stancePersistenceS, rightFoot: BALANCE_LIMITS.stancePersistenceS };
    this.nextFoot = "leftFoot";
    this.candidateValidity = { valid: false, reason: "idle" };
    this.liftedFoot = null;
    this.touchdownAge = 0;
    this.stepTransferAge = 0;
    this.transferReadyAge = 0;
    this.unloadReadyAge = 0;
    this.stepHasUnloaded = false;
    this.touchdownTargetAdjusted = false;
    this.disturbanceSeen = false;
  }

  update(input: BalanceInput): BalanceOutput {
    const dt = clamp(input.dt, 1 / 240, 1 / 20);
    const floorY = input.floorY ?? 0;
    for (const foot of FEET) {
      // Every loaded support is measured, including flat-floor sliding. A
      // cached pre-slip position is not a contact the body can balance over.
      const pose = input.poses.get(foot)!;
      if (this.step?.foot !== foot && input.contacts?.some(contact => contact.loadBearing
        && contact.segment === foot)) {
        this.feet[foot] = { ...pose.position,
          y: input.surfaceHeight ? pose.position.y : footCenterHeight(foot, floorY) };
        if (input.surfaceHeight) this.footRotations[foot] = { ...pose.rotation };
        else {
          // Follow measured sliding in the support plane, but do not turn a
          // loaded heel/toe roll into the next raised, tilted posture target.
          const forward = rotate(this.footRotations[foot], FORWARD);
          this.footRotations[foot] = quatFromAxisAngle(UP, Math.atan2(forward.x, forward.z));
        }
      }
    }
    const headingRadians = input.heading ?? 0;
    const heading = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, headingRadians);
    const forward = rotate(heading, { x: 0, y: 0, z: 1 });
    const right = rotate(heading, { x: 1, y: 0, z: 0 });
    const grab = input.activeGrab;
    const drag = grab ? sub(grab.target, grab.startTarget) : ZERO;
    const grabbedFoot = grab && FEET.includes(grab.region as Foot) && length(drag) > BALANCE_LIMITS.liftedGrabDistanceM ? grab.region as Foot : null;
    const releasedFoot = this.liftedFoot && this.liftedFoot !== grabbedFoot ? this.liftedFoot : null;
    this.liftedFoot = grabbedFoot;
    this.cooldown = Math.max(0, this.cooldown - dt);
    let transferLoadsReady = false;
    let transferCaptureMarginM: number | null = null;
    if (this.step) {
      // Committed targets remain in world space as body yaw changes.
      const side = this.step.foot === "leftFoot" ? "left" : "right";
      const footContacts = (input.contacts ?? []).filter(contact => {
        const definition = SEGMENT_BY_ID.get(contact.segment);
        return definition?.side === side && (definition.role === "hindfoot" || definition.role === "forefoot")
          && contact.normalY >= 0.65;
      });
      const movingLoad = footContacts.reduce((sum, contact) => sum + contact.forceN, 0);
      const retainedLoad = (input.contacts ?? []).filter(contact => {
        const definition = SEGMENT_BY_ID.get(contact.segment);
        return definition?.side !== side && (definition?.role === "hindfoot" || definition?.role === "forefoot")
          && contact.normalY >= 0.65 && contact.loadBearing && contact.persistenceS >= BALANCE_LIMITS.stancePersistenceS;
      }).reduce((sum, contact) => sum + contact.forceN, 0);
      const measured = input.contacts !== undefined;
      this.unloadReadyAge = measured && movingLoad < BALANCE_LIMITS.minimumContactForceN
        ? this.unloadReadyAge + dt : 0;
      this.stepHasUnloaded ||= !measured || this.unloadReadyAge >= 0.05;
      if (this.step.elapsed < 0) {
        this.stepTransferAge += dt;
        const retainedThresholdN = TOTAL_MASS_KG * 9.81 * BALANCE_LIMITS.retainedWeightFraction;
        transferLoadsReady = retainedLoad >= retainedThresholdN
          && movingLoad <= TOTAL_MASS_KG * 9.81 * 0.25;
        // A sustained majority load on the retained sole authorizes the motor
        // to begin lifting. Requiring the moving sole to be unloaded here is a
        // deadlock: it cannot shed its remaining load until the swing starts.
        // `stepHasUnloaded` independently requires actual unload before a
        // later touchdown can complete the step.
        if (!measured) {
          this.step.elapsed = 0;
        }
        this.step.phase = this.step.elapsed < 0 ? "unloading" : "swing";
      } else {
        // The foot keeps following the bounded lift/travel/lower trajectory
        // while the measured sole unloads. Holding time at the lift boundary
        // can leave too little travel time and strand the leg in a deadlock.
        // stepHasUnloaded still gates the later loaded touchdown.
        this.step.elapsed += dt;
        this.step.phase = this.step.elapsed < this.step.duration ? "swing" : "touchdown";
      }
      if (measured && this.step.elapsed >= this.step.duration && !this.touchdownTargetAdjusted
        && movingLoad < BALANCE_LIMITS.minimumContactForceN) {
        const swingPose = input.poses.get(this.step.foot)!;
        const landingFloor = input.surfaceHeight?.(swingPose.position.x, swingPose.position.z) ?? floorY;
        const clearance = swingPose.position.y - footCenterHeight(this.step.foot, landingFloor);
        const downwardSpeed = -swingPose.linearVelocity.y;
        if (clearance > 0.005 && clearance < 0.04 && downwardSpeed > 0.15
          && length(horizontal(swingPose.linearVelocity)) > 0.2) {
          // Rapier's measured foot can be carried several centimetres between
          // the commanded arc end and first sole contact. Predict that first
          // contact once from its current world velocity, before any loaded
          // contact exists; never chase a foot after contact is established.
          const contactTime = Math.min(0.06, clearance / downwardSpeed);
          const predicted = {
            x: swingPose.position.x + swingPose.linearVelocity.x * contactTime,
            y: footCenterHeight(this.step.foot,
              input.surfaceHeight?.(
                swingPose.position.x + swingPose.linearVelocity.x * contactTime,
                swingPose.position.z + swingPose.linearVelocity.z * contactTime,
              ) ?? landingFloor),
            z: swingPose.position.z + swingPose.linearVelocity.z * contactTime,
          };
          const validation = validateLanding(this.landingInput(this.step.foot, this.step.from,
            input.rootPosition, this.step.to, this.step.heading ?? headingRadians,
            input.poses.get("pelvis")?.rotation, floorY, input.surfaceHeight), predicted);
          if (validation.feasible && length(horizontal(sub(predicted, this.step.to))) <= 0.04) {
            this.step.to = validation.to;
            this.step.requested = { ...validation.to };
            this.touchdownTargetAdjusted = true;
          }
        }
      }
      if (this.step.elapsed >= this.step.duration) {
        const side = this.step.foot === "leftFoot" ? "left" : "right";
        const loadedTouchdown = input.contacts?.some((contact) => {
          const definition = SEGMENT_BY_ID.get(contact.segment);
          return contact.loadBearing && definition?.side === side
            && (definition.role === "hindfoot" || definition.role === "forefoot");
        }) ?? false;
        const solved = input.poses.get(this.step.foot)!.position;
        const targetError = length(horizontal(sub(solved, this.step.to)));
        // Elapsed time finishes the swing interpolation; measured loaded contact
        // finishes the step. Keep commanding the reachable landing pose until
        // the physical foot has arrived instead of declaring an airborne leg planted.
        this.touchdownAge = this.stepHasUnloaded && loadedTouchdown
          && targetError < 0.09
          ? this.touchdownAge + dt : 0;
        if (this.touchdownAge > 0) this.step.phase = "loading";
        if (this.touchdownAge + 1e-9 >= 0.10) {
          this.stepCount += 1; // Count a physically completed touchdown.
          this.feet[this.step.foot] = { x: solved.x,
            y: footCenterHeight(this.step.foot, input.surfaceHeight?.(solved.x, solved.z) ?? floorY), z: solved.z };
          this.footRotations[this.step.foot] = input.surfaceHeight
            ? { ...input.poses.get(this.step.foot)!.rotation }
            : quatFromAxisAngle(UP, this.step.heading ?? headingRadians);
          // The measured COM is already balanced over the old stance foot at
          // touchdown. Preserve that equilibrium inside the new, wider support
          // polygon; snapping immediately to the soles' midpoint creates an
          // artificial whole-body lunge and can unload both contacts.
          this.supportTarget = horizontal(sub(massState(input.poses).position, this.neutralComOffset));
          this.stanceAge[this.step.foot] = BALANCE_LIMITS.stancePersistenceS;
          this.step = null;
          this.candidateValidity = { valid: false, reason: "idle" };
          this.touchdownTargetAdjusted = false;
                this.cooldown = BALANCE_LIMITS.stepCooldownS;
          // Margin estimates are intentionally conservative during a swing.
          // Once Rapier confirms the new loaded patch, none of that transient
          // single-support history belongs to the next balance decision.
          this.instability = 0;
          this.touchdownAge = 0;
        }
      }
    }
    // A released manipulated foot needs its own landing step even if held close to the floor.
    if (!this.step && releasedFoot) {
      this.beginStep(releasedFoot, input.poses.get(releasedFoot)!.position, input.rootPosition, forward, right, floorY, ZERO, headingRadians);
    }
    if (!this.step && !grabbedFoot && this.cooldown === 0 && this.disturbanceSeen) {
      const airborne = FEET.find((foot) => {
        const pose = input.poses.get(foot)!;
        return pose.position.y - (input.surfaceHeight?.(pose.position.x, pose.position.z) ?? floorY) > footHalfExtents(foot).y + BALANCE_LIMITS.floorClearanceM
          && length(horizontal(sub(pose.position, this.feet[foot]))) > 0.04;
      });
      if (airborne) this.beginStep(airborne, input.poses.get(airborne)!.position, input.rootPosition, forward, right, floorY, ZERO, headingRadians);
    }

    const supportingFeet: Foot[] = [];
    const corners: Vec3[] = [];
    for (const foot of FEET) {
      const measured = input.contacts?.filter((contact) => {
        const definition = SEGMENT_BY_ID.get(contact.segment);
        return contact.loadBearing && definition?.side === (foot === "leftFoot" ? "left" : "right")
          && (definition.role === "ankle" || definition.role === "hindfoot" || definition.role === "forefoot");
      }) ?? [];
      // Once the planned swing has finished, a measured loaded touchdown is
      // real support even while contact persistence is still being validated.
      const activelySwinging = this.step?.foot === foot
        && this.step.elapsed >= 0
        && this.step.elapsed < this.step.duration;
      const pose = input.poses.get(foot)!;
      const geometry = SEGMENT_BY_ID.get(foot)!.geometry;
      const surfaceY = input.surfaceHeight?.(pose.position.x, pose.position.z) ?? floorY;
      const clearance = lowestWorldPoint(geometry, pose.position, pose.rotation).y - surfaceY;
      const up = rotate(pose.rotation, { x: 0, y: 1, z: 0 });
      const eligible = !activelySwinging && foot !== grabbedFoot && (measured.length > 0
        || (input.contacts === undefined && clearance >= -0.025 && clearance <= BALANCE_LIMITS.floorClearanceM
          && up.y > 0.8
          && length(horizontal(sub(pose.position, this.feet[foot]))) < BALANCE_LIMITS.stanceTargetErrorM));
      this.stanceAge[foot] = eligible ? this.stanceAge[foot] + dt : 0;
      if (!eligible || this.stanceAge[foot] < BALANCE_LIMITS.stancePersistenceS) continue;
      supportingFeet.push(foot);
      if (measured.length) {
        corners.push(...measured.flatMap((contact) => contact.points?.length ? contact.points : [contact.point]));
      } else {
        corners.push(...footCorners(pose.position, pose.rotation, foot));
        const forefootId = `${foot === "leftFoot" ? "left" : "right"}Forefoot` as SegmentId;
        const forefootPose = input.poses.get(forefootId);
        const forefoot = SEGMENT_BY_ID.get(forefootId);
        if (forefootPose && forefoot) {
          corners.push(...(forefoot.geometry.supportPatch ?? []).map((point) =>
            worldPoint(forefootPose.position, forefootPose.rotation, point)
          ));
        }
      }
    }
    const mass = massState(input.poses);
    const previousComVelocity = this.comVelocity;
    this.comVelocity = lerp(this.comVelocity, mass.velocity, 1 - Math.exp(-dt * 14));
    this.comAcceleration = lerp(this.comAcceleration, clampLength(horizontal(scale(
      sub(this.comVelocity, previousComVelocity), 1 / dt,
    )), 6), 1 - Math.exp(-dt * 8));
    const speed = length(horizontal(this.comVelocity));
    const omega = Math.sqrt(9.81 / Math.max(0.4, mass.position.y - floorY));
    const capturePoint = add(horizontal(mass.position), scale(horizontal(this.comVelocity), 1 / omega));
    const predictedCapturePoint = add(capturePoint, add(
      scale(horizontal(this.comVelocity), BALANCE_LIMITS.stepPredictionHorizonS),
      scale(this.comAcceleration, 0.5 * BALANCE_LIMITS.stepPredictionHorizonS ** 2),
    ));
    const supportCenter = supportingFeet.length === 2
      ? midpoint(input.poses.get("leftFoot")!.position, input.poses.get("rightFoot")!.position)
      : supportingFeet.length === 1 ? input.poses.get(supportingFeet[0])!.position
      : midpoint(this.feet.leftFoot, this.feet.rightFoot);
    const supportMargin = polygonMargin(capturePoint, supportHull(corners));
    const appliedForce = input.appliedGrabForce ?? input.externalForce;
    let force = appliedForce ?? ZERO;
    // Only standalone fixtures may estimate a spring; runtime supplies measured
    // bounded grab effort through appliedGrabForce, including an explicit zero.
    if (grab && appliedForce === undefined) {
      const grabbedPose = input.poses.get(grab.segment ?? grab.region)!;
      const anchor = worldPoint(grabbedPose.position, grabbedPose.rotation, grab.localAnchor ?? ZERO);
      const targetVelocity = grab.targetVelocity ?? ZERO;
      force = clampLength(add(
        scale(sub(grab.target, anchor), BALANCE_LIMITS.pullSpringNpm),
        scale(sub(clampLength(targetVelocity, 8), grabbedPose.linearVelocity), BALANCE_LIMITS.pullDampingNsPm),
      ), BALANCE_LIMITS.maxPullForceN);
    }
    if (grab || length(force) > 5) this.disturbanceSeen = true;
    // The stance motor has a finite horizontal acceleration budget. Large forces
    // therefore accumulate real COM momentum instead of becoming larger pose offsets.
    // Preview a bounded portion of the requested reach. The grab controller is
    // force/power limited, so a long slow pull should be allowed to begin a
    // corrective step before COM momentum has already escaped the footprint.
    const anticipation = add(
      add(capturePoint, scale(horizontal(force), 0.09 / TOTAL_MASS_KG)),
      scale(clampLength(horizontal(drag), 0.8), 0.18),
    );
    const reachCorrection = horizontal(sub(anticipation, horizontal(input.rootPosition)));
    const predictedCorrection = horizontal(sub(predictedCapturePoint, horizontal(input.rootPosition)));
    const landingCorrection = add(
      scale(right, dot(reachCorrection, right)),
      scale(forward, dot(predictedCorrection, forward)),
    );
    // Refresh feasibility on current measurements until lift; never move a
    // committed world destination after swing has started.
    if (this.step && this.step.elapsed < 0) {
      this.transferReadyAge = transferLoadsReady ? this.transferReadyAge + dt : 0;
      const currentPlan = this.planStep(this.step.foot, input.poses.get(this.step.foot)!.position,
        input.rootPosition, forward, right, floorY, landingCorrection, headingRadians,
        input.poses.get("pelvis")?.rotation, 0, 0.12, input, capturePoint);
      const selected = !currentPlan.feasible && supportingFeet.length === 2
        ? this.selectStep(input, supportingFeet, forward, right, floorY,
          landingCorrection, headingRadians, capturePoint) : null;
      // Replan placement continuously, but let a valid selected transfer
      // finish. Reversing its side changes the motion being predicted and
      // discards measured readiness before the load can settle.
      const switchSide = selected && selected.foot !== this.step.foot
        && !currentPlan.feasible;
      if (switchSide) {
        this.beginStep(selected.foot, input.poses.get(selected.foot)!.position,
          input.rootPosition, forward, right, floorY, landingCorrection,
          headingRadians, input.poses.get("pelvis")?.rotation, selected.plan);
        transferLoadsReady = false;
      }
      const plan = switchSide ? selected.plan : currentPlan;
      this.candidateValidity = { valid: plan.feasible, reason: plan.feasible ? null : plan.reason,
        ...(plan.forecast ? { forecast: plan.forecast } : {}),
        ...(plan.feasible ? { transferForecastReason: plan.transferForecastReason ?? null } : {}) };
      if (!plan.feasible) {
        this.step = null;
        this.transferReadyAge = 0;
        this.supportTarget = horizontal(sub(mass.position, this.neutralComOffset));
        this.cooldown = BALANCE_LIMITS.stancePersistenceS;
      } else {
        this.step.from = { ...input.poses.get(this.step.foot)!.position };
        this.step.to = plan.to;
        this.step.requested = plan.requested;
        this.step.duration = plan.duration;
        const retainedSide = this.step.foot === "leftFoot" ? "right" : "left";
        const retainedPatch = (input.contacts ?? []).filter(contact => contact.loadBearing
          && contact.normalY >= 0.65 && SEGMENT_BY_ID.get(contact.segment)?.side === retainedSide
          && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(contact.segment)?.role ?? ""))
          .flatMap(contact => contact.points?.length ? contact.points : [contact.point]);
        // Report retained capture separately: a corrective step may catch
        // momentum outside this sole if its validated touchdown can support it.
        transferCaptureMarginM = input.contacts
          ? polygonMargin(capturePoint, supportHull(retainedPatch)) : null;
        if (this.transferReadyAge + 1e-9 >= 0.10) {
          this.step.elapsed = 0;
          this.step.phase = "swing";
        }
      }
    }
    const demandedReach = length(horizontal(drag));
    const outwardCapture = speed > 0.07
      && supportMargin < 0.08
      && dot(horizontal(sub(capturePoint, supportCenter)), horizontal(this.comVelocity)) > 0.005;
    if (!this.step && this.cooldown === 0 && this.disturbanceSeen && supportingFeet.length > 0
      && (grabbedFoot || length(force) > 3 || speed > 0.2 || demandedReach > 0.08 || outwardCapture)
      && (polygonMargin(anticipation, supportHull(corners)) < BALANCE_LIMITS.stepTriggerMarginM
        || demandedReach > 0.08 || grabbedFoot)) {
      if (!grabbedFoot) {
        const selected = this.selectStep(input, supportingFeet, forward, right, floorY,
          landingCorrection, headingRadians, capturePoint);
        if (selected) this.beginStep(selected.foot, input.poses.get(selected.foot)!.position,
          input.rootPosition, forward, right, floorY, landingCorrection, headingRadians,
          input.poses.get("pelvis")?.rotation, selected.plan);
      }
    }

    // A swing is temporarily single-supported. Judge its recoverability against
    // the reachable landing footprint and momentum expected before touchdown.
    if (this.step) {
      const landingFloor = input.surfaceHeight?.(this.step.to.x, this.step.to.z) ?? floorY;
      // The swing arc clears the ground until its commanded landing time.
      // Only then ask for a small downward preload while waiting for Rapier's
      // measured touchdown; driving 40 mm below the floor mid-swing collapses
      // the knee and spends leg torque on an impossible target.
      this.step.to = { ...this.step.to, y: footCenterHeight(this.step.foot, landingFloor)
        - (this.step.elapsed >= this.step.duration ? 0.005 : 0) };
    }
    // Candidate/phase changes must precede the intents sent to this tick's
    // motors; otherwise a cancellation or side switch sends stale support.
    const stanceFoot = this.step ? (this.step.foot === "leftFoot" ? "rightFoot" : "leftFoot") : null;
    const plannedAnkle = (foot: Foot): Vec3 => ankleProjection(
      this.feet[foot], this.footRotations[foot], foot,
    );
    const unloadingSwing = this.step !== null && this.step.elapsed < this.step.duration;
    const desiredAnkleCenter = unloadingSwing && stanceFoot
      ? plannedAnkle(stanceFoot)
      : supportingFeet.length > 0
        ? scale(supportingFeet.reduce((sum, foot) => add(sum, plannedAnkle(foot)), ZERO), 1 / supportingFeet.length)
        : horizontal(supportCenter);
    if (this.step) {
      this.supportTarget = this.step.elapsed < 0
        ? add(this.supportTarget, clampLength(
          sub(desiredAnkleCenter, this.supportTarget), dt * BALANCE_LIMITS.supportTransferSpeedMps,
        )) : { ...desiredAnkleCenter };
    } else if (this.stepCount === 0) {
      this.supportTarget = { ...desiredAnkleCenter };
    }
    const desiredCom = add(this.supportTarget, this.neutralComOffset);
    const transferControlPoint = this.step && this.step.elapsed < 0 ? capturePoint : horizontal(mass.position);
    const balanceAcceleration = supportingFeet.length === 0 ? ZERO : clampLength(add(
      scale(sub(desiredCom, transferControlPoint), 34),
      scale(horizontal(this.comVelocity), -8.5),
    ), BALANCE_LIMITS.maxBalanceAccelerationMps2);
    const acceleration = add(scale(horizontal(force), 1 / TOTAL_MASS_KG), balanceAcceleration);
    this.velocity = clampLength(add(this.velocity, scale(acceleration, dt)), BALANCE_LIMITS.maxRootSpeedMps);
    this.velocity = scale(this.velocity, Math.exp(-dt * 0.35));
    const rootTarget = { ...add(desiredCom, this.neutralRootFromCom), y: this.nominalHeight + floorY };

    let futureMargin = supportMargin;
    if (this.step) {
      const landingCorners = [...corners];
      landingCorners.push(...footCorners(this.step.to, heading, this.step.foot));
      const remaining = Math.max(0, this.step.duration - this.step.elapsed);
      const touchdownCapture = add(capturePoint, scale(horizontal(this.comVelocity), remaining * 0.5));
      futureMargin = polygonMargin(touchdownCapture, supportHull(landingCorners));
    }
    const stepCapacity = this.step ? Math.max(0, futureMargin) : BALANCE_LIMITS.maxStepReachM;
    const noSupport = supportingFeet.length === 0;
    const transientStepSupportLoss = noSupport && this.step !== null && this.step.elapsed < 0.20;
    const immediate = (noSupport && !transientStepSupportLoss && speed > 0.9)
      || ((speed > 0.6 || length(force) > 100)
        && supportMargin < BALANCE_LIMITS.unrecoverableMarginM && (!this.step || futureMargin < -0.02))
      || (speed > BALANCE_LIMITS.unrecoverableSpeedMps && supportMargin < -0.12);
    const marginal = (noSupport && !transientStepSupportLoss && speed > 0.3)
      || ((speed > 0.25 || length(force) > 45)
        && supportMargin < BALANCE_LIMITS.marginalMarginM && (!this.step || futureMargin < -0.02))
      || (supportMargin < -0.10 && futureMargin < -0.04 && speed > 0.75);
    this.instability = marginal ? this.instability + dt : Math.max(0, this.instability - dt * 2);
    // Once a correction has committed, intermittent single-point contact
    // manifolds can make the instantaneous polygon margin jump outside the
    // sole while the swing is still physically viable.  Let the active step
    // resolve; EmbodiedCharacter continues to enforce measured support loss,
    // torso lean, and pelvis height throughout the motion.
    const correctingStep = this.step !== null && (this.step.elapsed >= 0 || this.stepTransferAge < 1.2);
    // A measured touchdown deliberately enters a short double-support cooldown
    // before another step may start. Keep that bounded recovery opportunity
    // alive while at least one real support remains; torso lean, pelvis height,
    // and sustained support loss are still enforced by EmbodiedCharacter.
    const settlingAfterTouchdown = !correctingStep
      && this.stepCount > 0
      && this.cooldown > 0
      && supportingFeet.length > 0;
    const shouldFall = !correctingStep
      && !settlingAfterTouchdown
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);
    // Ankle torque handles small errors; the hips counter-lean once momentum grows.
    // Both remain bounded, so an abrupt pull can still overwhelm the recovery step.
    const captureError = horizontal(sub(capturePoint, desiredCom));
    const measuredTorso = input.poses.get("torso");
    const measuredTorsoUp = measuredTorso ? rotate(measuredTorso.rotation, UP) : UP;
    const posturalCorrection = scale(horizontal(measuredTorsoUp), -0.9);
    const torsoAngularVelocity = measuredTorso?.angularVelocity ?? ZERO;
    // Damp the measured lean rate as well as its angle. World X rotation moves
    // torso-up toward Z; world Z rotation moves torso-up toward -X.
    const posturalDamping = {
      x: torsoAngularVelocity.z * 0.10,
      y: 0,
      z: -torsoAngularVelocity.x * 0.10,
    };
    const desiredReaction = clampLength(add(
      add(
        add(scale(captureError, -0.82), scale(horizontal(force), -0.00045)),
        posturalCorrection,
      ),
      posturalDamping,
    ), 0.54);
    this.reaction = lerp(this.reaction, desiredReaction, 1 - Math.exp(-dt * 12));
    // The distal target provides swing clearance; retain the stance posture
    // while a corrective foot is in flight.
    const kneeFlexion = clamp(
      HUMAN_PROPORTIONS.stance.neutralKneeFlexion + (this.step
        ? 0
        : length(this.reaction) * 0.7),
      0,
      1,
    );
    const state = this.step ? "stepping" : grab || speed > 0.04 || length(this.reaction) > 0.025 ? "reacting" : "upright";
    return {
      rootTarget, reactionOffset: this.reaction, kneeFlexion,
      supportFeet: { leftFoot: { ...this.feet.leftFoot }, rightFoot: { ...this.feet.rightFoot } },
      supportFootRotations: { leftFoot: { ...this.footRotations.leftFoot }, rightFoot: { ...this.footRotations.rightFoot } },
      step: this.step ? { ...this.step, from: { ...this.step.from }, to: { ...this.step.to },
        requested: this.step.requested ? { ...this.step.requested } : undefined,
        unloaded: this.stepHasUnloaded } : null,
      stepCount: this.stepCount, state, shouldFall,
      fallDirection: normalize(add(horizontal(this.comVelocity), scale(horizontal(force), 0.002)), forward),
      appliedGrabForceN: length(force),
      diagnostics: {
        centerOfMass: mass.position, centerOfMassVelocity: { ...this.comVelocity }, capturePoint,
        measuredSupportingFeet: FEET.filter(foot => (input.contacts ?? []).some(contact => {
          const definition = SEGMENT_BY_ID.get(contact.segment);
          return contact.loadBearing && definition?.side === (foot === "leftFoot" ? "left" : "right")
            && (definition.role === "hindfoot" || definition.role === "forefoot");
        })),
        supportCenter: { ...supportCenter }, supportingFeet: supportingFeet.filter(foot =>
          !(this.step?.foot === foot && this.step.elapsed >= 0
            && this.step.elapsed < this.step.duration)
        ), supportMarginM: supportMargin,
        instabilitySeconds: this.instability, recoveryCapacityM: stepCapacity, externalForce: force,
        balanceAcceleration, stepTarget: this.step ? { ...this.step.to } : null,
        candidateValidity: { ...this.candidateValidity }, transferAgeS: this.stepTransferAge,
        transferReadyAgeS: this.transferReadyAge,
        transferCaptureMarginM,
      },
    };
  }

  private beginStep(
    foot: Foot, from: Vec3, root: Vec3, forward: Vec3, right: Vec3,
    floorY: number, correction: Vec3, headingRadians = 0, pelvisRotation?: Quat,
    selected?: Extract<LandingPlan, { feasible: true }>,
  ): void {
    const plan = selected ?? this.planStep(foot, from, root, forward, right, floorY,
      correction, headingRadians, pelvisRotation);
    this.candidateValidity = { valid: plan.feasible, reason: plan.feasible ? null : plan.reason,
      ...(plan.forecast ? { forecast: plan.forecast } : {}),
      ...(plan.feasible ? { transferForecastReason: plan.transferForecastReason ?? null } : {}) };
    if (!plan.feasible) return;
    this.step = { phase: "unloading", foot, from: { ...from }, to: plan.to,
      requested: plan.requested, heading: headingRadians, elapsed: -0.18, duration: plan.duration };
    this.nextFoot = foot === "leftFoot" ? "rightFoot" : "leftFoot";
    this.stepTransferAge = 0;
    this.transferReadyAge = 0;
    this.unloadReadyAge = 0;
    this.touchdownAge = 0;
    this.stepHasUnloaded = false;
    this.touchdownTargetAdjusted = false;
  }

  private landingInput(foot: Foot, from: Vec3, root: Vec3, requested: Vec3,
    heading: number, rotation: Quat | undefined, floorY: number,
    surfaceHeight?: (x: number, z: number) => number): LandingInput {
    return { foot, from, pelvis: { position: root, rotation: rotation ?? quatFromAxisAngle(UP, heading) },
      heading, requested, retainedFoot: this.feet[foot === "leftFoot" ? "rightFoot" : "leftFoot"],
      floorY, surfaceHeight, maxTravelM: BALANCE_LIMITS.maxStepTravelM,
      maxReachM: BALANCE_LIMITS.maxStepReachM };
  }

  private planStep(foot: Foot, from: Vec3, root: Vec3, forward: Vec3, right: Vec3,
    floorY: number, correction: Vec3, headingRadians: number, pelvisRotation?: Quat,
    _reachRootYOffsetM = 0, lateralMagnitudeM = 0.22, input?: BalanceInput,
    capturePoint?: Vec3): LandingPlan {
    const sign = foot === "leftFoot" ? -1 : 1;
    const localOffset = hindfootFromAnkle(sign < 0 ? "left" : "right", ZERO, { x: 0, y: 0, z: 0, w: 1 });
    const offset = add(scale(right, localOffset.x), scale(forward, localOffset.z));
    const requested = add(root, clampLength(add(offset,
      add(scale(right, sign * lateralMagnitudeM), correction)), BALANCE_LIMITS.maxStepReachM));
    const context = this.landingInput(foot, from, root, requested, headingRadians,
      pelvisRotation, floorY, input?.surfaceHeight);
    let transferForecastReason: string | null = null;
    let forecast: LandingForecast | undefined;
    if (input?.contacts && capturePoint) {
      const retainedSide = sign < 0 ? "right" : "left";
      context.retainedContacts = input.contacts.filter(c => SEGMENT_BY_ID.get(c.segment)?.side === retainedSide
        && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(c.segment)?.role ?? ""));
      if (this.step?.foot === foot && this.step.elapsed < 0
        && this.stepTransferAge >= 1.2 && this.transferReadyAge + 1e-9 < 0.10)
        return { feasible: false, reason: "transfer:measured-timeout" };
      // Each candidate must predict the support command it would actually
      // issue, starting from the same measured state and current command.
      const retainedFoot = `${retainedSide}Foot` as Foot;
      const retainedAnkle = ankleProjection(this.feet[retainedFoot], this.footRotations[retainedFoot], retainedFoot);
      const rampDistance = length(sub(retainedAnkle, this.supportTarget));
      const ownsTransfer = this.step?.foot === foot && this.step.elapsed < 0;
      const currentDesiredCenterOfMass = add(this.supportTarget, this.neutralComOffset);
      const common = predictControlledLandingCapture({
        centerOfMass: massState(input.poses).position,
        centerOfMassVelocity: this.comVelocity,
        currentDesiredCenterOfMass,
        committedDesiredCenterOfMass: add(retainedAnkle, this.neutralComOffset),
        contacts: input.contacts,
        retainedSide,
        externalForce: input.appliedGrabForce ?? input.externalForce ?? ZERO,
        massKg: TOTAL_MASS_KG,
        maxBalanceAccelerationMps2: BALANCE_LIMITS.maxBalanceAccelerationMps2,
        floorY,
        stepDurationS: BALANCE_LIMITS.minStepDurationS,
        transferRampDistanceM: rampDistance,
        transferSpeedMps: BALANCE_LIMITS.supportTransferSpeedMps,
        readinessAgeS: ownsTransfer ? this.transferReadyAge : 0,
        remainingTransferWindowS: Math.max(0, 1.2 - (ownsTransfer ? this.stepTransferAge : 0)),
      });
      if (!common.valid) return { feasible: false, reason: `capture:${common.reason}` };
      context.capturePoint = common.capturePoint;
      transferForecastReason = common.pressureFeasible ? null : "pressure-limited";
      forecast = { capturePoint: common.capturePoint, horizonS: common.horizonS,
        pressureFeasible: common.pressureFeasible };
    }
    const landing = findLandingPlan(context);
    if (landing.feasible && context.capturePoint && landing.captureMarginM < -1e-8)
      return { feasible: false, reason: "capture-support", forecast };
    return landing.feasible ? { ...landing, transferForecastReason, forecast } : { ...landing, forecast };
  }

  private selectStep(input: BalanceInput, supporting: Foot[], forward: Vec3, right: Vec3,
    floorY: number, correction: Vec3, heading: number, capturePoint: Vec3) {
    const candidates = FEET.flatMap(foot => {
      const retained = foot === "leftFoot" ? "rightFoot" : "leftFoot";
      if (!supporting.includes(retained)) return [];
      const plan = this.planStep(foot, input.poses.get(foot)!.position, input.rootPosition,
        forward, right, floorY, correction, heading, input.poses.get("pelvis")?.rotation,
        0, 0.12, input, capturePoint);
      if (!plan.feasible) {
        this.candidateValidity = { valid: false, reason: plan.reason,
          ...(plan.forecast ? { forecast: plan.forecast } : {}) };
        return [];
      }
      const retainedSide = retained === "leftFoot" ? "left" : "right";
      const load = (input.contacts ?? []).filter(c => c.loadBearing && c.normalY >= 0.65
        && c.persistenceS >= BALANCE_LIMITS.stancePersistenceS && SEGMENT_BY_ID.get(c.segment)?.side === retainedSide
        && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(c.segment)?.role ?? ""))
        .reduce((sum, c) => sum + c.forceN, 0);
      const movingLoad = (input.contacts ?? []).filter(c => c.normalY >= 0.65
        && SEGMENT_BY_ID.get(c.segment)?.side !== retainedSide
        && ["hindfoot", "forefoot"].includes(SEGMENT_BY_ID.get(c.segment)?.role ?? ""))
        .reduce((sum, c) => sum + c.forceN, 0);
      const demand = Math.max(0, TOTAL_MASS_KG * 9.81 * BALANCE_LIMITS.retainedWeightFraction - load)
        + Math.max(0, movingLoad - TOTAL_MASS_KG * 9.81 * 0.25);
      return [{ foot, plan, demand }];
    });
    // Preserve the capture margin ordering even when both candidates contain
    // the prediction. Equalizing all positive margins can favor replacing a
    // trailing foot without extending support in the direction of momentum.
    candidates.sort((a, b) => (Math.abs(b.plan.captureMarginM - a.plan.captureMarginM) > 1e-9
      ? b.plan.captureMarginM - a.plan.captureMarginM : 0)
      || (Math.abs(b.plan.landingCaptureMarginM - a.plan.landingCaptureMarginM) > 1e-9
        ? b.plan.landingCaptureMarginM - a.plan.landingCaptureMarginM : 0)
      || a.demand - b.demand || a.plan.travelM - b.plan.travelM
      || Number(b.foot === this.nextFoot) - Number(a.foot === this.nextFoot));
    return candidates[0] ?? null;
  }
}
