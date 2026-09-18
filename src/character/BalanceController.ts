import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } from "../core/humanoid";
import { geometryHalfExtents, lowestWorldPoint } from "../core/geometry";
import type { RegionId, SegmentId, SegmentPose, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, clampLength, dot, length, lerp, normalize, quatFromAxisAngle, rotate, scale, sub, worldPoint } from "./math";
import { hindfootFromAnkle } from "./leg-target-frame";
import { composeUprightPose, horizontal, midpoint, restPoseMap, type StepMotion } from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
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
  stepTriggerMarginM: Math.min(0.03, footHalfExtents("leftFoot").x * 0.5),
  maxStepReachM: 0.36,
  maxStepTravelM: 0.43,
  stepDurationS: 0.68,
  minStepDurationS: 0.48,
  // Give double support time to settle before starting another correction.
  stepCooldownS: 0.18,
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
  supportingFeet: Foot[];
  supportMarginM: number;
  instabilitySeconds: number;
  recoveryCapacityM: number;
  externalForce: Vec3;
  balanceAcceleration: Vec3;
  stepTarget: Vec3 | null;
}

export interface BalanceInput {
  dt: number;
  poses: ReadonlyMap<SegmentId, SegmentPose>;
  rootPosition: Vec3;
  activeGrab: BalanceGrab | null;
  floorY?: number;
  heading?: number;
  /** Measured Rapier contacts; pose-derived support is only the startup fallback. */
  contacts?: readonly SupportingContact[];
}

export interface BalanceOutput {
  rootTarget: Vec3;
  reactionOffset: Vec3;
  kneeFlexion: number;
  supportFeet: Record<Foot, Vec3>;
  step: StepMotion | null;
  stepCount: number;
  state: "upright" | "reacting" | "stepping";
  shouldFall: boolean;
  fallDirection: Vec3;
  appliedGrabForceN: number;
  diagnostics: BalanceDiagnostics;
}

export function massState(poses: ReadonlyMap<SegmentId, SegmentPose>): { position: Vec3; velocity: Vec3 } {
  let position = ZERO;
  let velocity = ZERO;
  let mass = 0;
  for (const definition of SEGMENTS) {
    const pose = poses.get(definition.id);
    if (!pose) continue;
    mass += definition.massKg;
    position = add(position, scale(pose.position, definition.massKg));
    velocity = add(velocity, scale(pose.linearVelocity, definition.massKg));
  }
  return { position: scale(position, 1 / Math.max(mass, 1)), velocity: scale(velocity, 1 / Math.max(mass, 1)) };
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
  private step: StepMotion | null = null;
  private stepCount = 0;
  private cooldown = 0;
  private instability = 0;
  private velocity = ZERO;
  private comVelocity = ZERO;
  private reaction = ZERO;
  private nextFoot: Foot = "leftFoot";
  private liftedFoot: Foot | null = null;
  private touchdownAge = 0;
  private disturbanceSeen = false;
  private nominalHeight: number = HUMAN_PROPORTIONS.pelvis.centerHeightM;
  private neutralComOffset = ZERO;
  private neutralRootFromCom = ZERO;
  private supportTarget = ZERO;

  reset(poses: ReadonlyMap<SegmentId, SegmentPose>, heading = 0): void {
    for (const foot of FEET) this.feet[foot] = { ...poses.get(foot)!.position };
    // composeUprightPose lowers its root by the requested knee flexion.  Store
    // the pre-flexion height here so resetting the controller from an already
    // composed physical pose does not apply that lowering a second time.
    this.nominalHeight = (poses.get("pelvis")?.position.y ?? HUMAN_PROPORTIONS.pelvis.centerHeightM)
      + HUMAN_PROPORTIONS.stance.neutralKneeFlexion * 0.10;
    this.velocity = horizontal(poses.get("pelvis")?.linearVelocity ?? ZERO);
    const mass = massState(poses);
    this.comVelocity = mass.velocity;
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
    // Let the physical contacts load and the coupled muscles settle before a
    // capture-point excursion is allowed to become a corrective step.
    this.cooldown = 0.6;
    this.instability = 0;
    this.reaction = ZERO;
    this.stanceAge = { leftFoot: BALANCE_LIMITS.stancePersistenceS, rightFoot: BALANCE_LIMITS.stancePersistenceS };
    this.nextFoot = "leftFoot";
    this.liftedFoot = null;
    this.touchdownAge = 0;
    this.disturbanceSeen = false;
  }

  update(input: BalanceInput): BalanceOutput {
    const dt = clamp(input.dt, 1 / 240, 1 / 20);
    const floorY = input.floorY ?? 0;
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
    if (this.step) {
      // from/to are immutable WORLD hindfoot centres. Body yaw never rebases a
      // committed landing or changes the meaning of an already-travelled arc.
      this.step.elapsed += dt;
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
        this.touchdownAge = loadedTouchdown && targetError < 0.09
          ? this.touchdownAge + dt : 0;
        if (this.touchdownAge >= 0.10) {
          this.feet[this.step.foot] = { x: solved.x, y: footCenterHeight(this.step.foot, floorY), z: solved.z };
          // The measured COM is already balanced over the old stance foot at
          // touchdown. Preserve that equilibrium inside the new, wider support
          // polygon; snapping immediately to the soles' midpoint creates an
          // artificial whole-body lunge and can unload both contacts.
          this.supportTarget = horizontal(sub(massState(input.poses).position, this.neutralComOffset));
          this.stanceAge[this.step.foot] = BALANCE_LIMITS.stancePersistenceS;
          this.step = null;
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
        return pose.position.y - floorY > footHalfExtents(foot).y + BALANCE_LIMITS.floorClearanceM
          && length(horizontal(sub(pose.position, this.feet[foot]))) > 0.04;
      });
      if (airborne) this.beginStep(airborne, input.poses.get(airborne)!.position, input.rootPosition, forward, right, floorY, ZERO, headingRadians);
    }

    const supportingFeet: Foot[] = [];
    const corners: Vec3[] = [];
    for (const foot of FEET) {
      const pose = input.poses.get(foot)!;
      const geometry = SEGMENT_BY_ID.get(foot)!.geometry;
      const up = rotate(pose.rotation, { x: 0, y: 1, z: 0 });
      const measured = input.contacts?.filter((contact) => {
        const definition = SEGMENT_BY_ID.get(contact.segment);
        return contact.loadBearing && definition?.side === (foot === "leftFoot" ? "left" : "right")
          && (definition.role === "ankle" || definition.role === "hindfoot" || definition.role === "forefoot");
      }) ?? [];
      const clearance = lowestWorldPoint(geometry, pose.position, pose.rotation).y - floorY;
      const activelySwinging = this.step?.foot === foot && this.step.elapsed >= 0;
      const eligible = !activelySwinging && foot !== grabbedFoot && (measured.length > 0
        || (clearance >= -0.025 && clearance <= BALANCE_LIMITS.floorClearanceM
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
    this.comVelocity = lerp(this.comVelocity, mass.velocity, 1 - Math.exp(-dt * 14));
    const speed = length(horizontal(this.comVelocity));
    const omega = Math.sqrt(9.81 / Math.max(0.4, mass.position.y - floorY));
    const capturePoint = add(horizontal(mass.position), scale(horizontal(this.comVelocity), 1 / omega));
    const supportCenter = supportingFeet.length === 2
      ? midpoint(input.poses.get("leftFoot")!.position, input.poses.get("rightFoot")!.position)
      : supportingFeet.length === 1 ? input.poses.get(supportingFeet[0])!.position
      : midpoint(this.feet.leftFoot, this.feet.rightFoot);
    const supportMargin = polygonMargin(capturePoint, supportHull(corners));
    let force = ZERO;
    if (grab) {
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
    const stanceFoot = this.step ? (this.step.foot === "leftFoot" ? "rightFoot" : "leftFoot") : null;
    const plannedAnkle = (foot: Foot): Vec3 => horizontal(add(
      this.feet[foot],
      rotate(heading, SEGMENT_BY_ID.get(foot)?.jointAnchorChild ?? ZERO),
    ));
    const desiredAnkleCenter = this.step && stanceFoot
      // During swing, translate the COM over the retained stance anchor. The
      // new two-foot midpoint becomes valid only after measured touchdown.
      ? plannedAnkle(stanceFoot)
      : supportingFeet.length > 0
        ? scale(supportingFeet.reduce(
          (sum, foot) => add(sum, plannedAnkle(foot)),
          ZERO,
        ), 1 / supportingFeet.length)
        : horizontal(supportCenter);
    if (this.step) {
      this.supportTarget = { ...desiredAnkleCenter };
    } else if (this.stepCount === 0) {
      this.supportTarget = { ...desiredAnkleCenter };
    }
    const desiredCom = add(this.supportTarget, this.neutralComOffset);
    const balanceAcceleration = clampLength(add(
      scale(sub(desiredCom, horizontal(mass.position)), 34),
      scale(horizontal(this.comVelocity), -8.5),
    ), BALANCE_LIMITS.maxBalanceAccelerationMps2);
    const acceleration = add(scale(horizontal(force), 1 / TOTAL_MASS_KG), balanceAcceleration);
    this.velocity = clampLength(add(this.velocity, scale(acceleration, dt)), BALANCE_LIMITS.maxRootSpeedMps);
    this.velocity = scale(this.velocity, Math.exp(-dt * 0.35));
    // In the dynamic controller this is an IK/motor target, never a body
    // transform. Keep it registered to the loaded support instead of
    // integrating the old kinematic root velocity and chasing a fall.
    const rootTarget = {
      ...add(desiredCom, this.neutralRootFromCom),
      y: this.nominalHeight,
    };

    // Preview a bounded portion of the requested reach. The grab controller is
    // force/power limited, so a long slow pull should be allowed to begin a
    // corrective step before COM momentum has already escaped the footprint.
    const anticipation = add(
      add(capturePoint, scale(horizontal(force), 0.09 / TOTAL_MASS_KG)),
      scale(clampLength(horizontal(drag), 0.8), 0.18),
    );
    const demandedReach = length(horizontal(drag));
    if (!this.step && this.cooldown === 0 && this.disturbanceSeen && supportingFeet.length > 0
      && (grabbedFoot || length(force) > 3 || speed > 0.2 || demandedReach > 0.08)
      && (polygonMargin(anticipation, supportHull(corners)) < BALANCE_LIMITS.stepTriggerMarginM
        || demandedReach > 0.15 || grabbedFoot)) {
      const direction = normalize(horizontal(sub(anticipation, supportCenter)), forward);
      const lateralDirection = dot(direction, right);
      // The first lateral recovery step widens the base toward the disturbance.
      // Subsequent steps alternate so the trailing leg can follow instead of
      // repeatedly stretching the same leading leg beyond stance reach.
      const reachingSide = grab?.region.startsWith("right") ? "rightFoot" as const
        : grab?.region.startsWith("left") ? "leftFoot" as const
          : this.nextFoot;
      let foot = this.stepCount === 0
        ? lateralDirection > 0.25 ? "rightFoot" as const : lateralDirection < -0.25 ? "leftFoot" as const : reachingSide
        : this.nextFoot;
      // While a foot is held, the other must remain the stance foot.
      if (grabbedFoot) foot = grabbedFoot;
      if (!grabbedFoot) {
        this.beginStep(foot, input.poses.get(foot)!.position, input.rootPosition, forward, right, floorY,
          horizontal(sub(anticipation, horizontal(input.rootPosition))), headingRadians);
        this.nextFoot = foot === "leftFoot" ? "rightFoot" : "leftFoot";
      }
    }
    // A swing is temporarily single-supported. Judge its recoverability against
    // the reachable landing footprint and momentum expected before touchdown.
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
    const correctingStep = this.step !== null;
    const shouldFall = !correctingStep
      && (immediate || this.instability >= BALANCE_LIMITS.marginalInstabilityS);
    // Ankle torque handles small errors; the hips counter-lean once momentum grows.
    // Both remain bounded, so an abrupt pull can still overwhelm the recovery step.
    const captureError = horizontal(sub(capturePoint, desiredCom));
    const desiredReaction = clampLength(add(
      scale(captureError, -0.82),
      scale(horizontal(force), -0.00045),
    ), 0.54);
    this.reaction = lerp(this.reaction, desiredReaction, 1 - Math.exp(-dt * 12));
    // The distal target already supplies swing clearance. Extra knee crouch
    // shortens the leg and can prevent a physically loaded touchdown because
    // the pelvis itself is never translated by the controller.
    const swingCrouch = 0;
    const kneeFlexion = clamp(
      HUMAN_PROPORTIONS.stance.neutralKneeFlexion + length(this.reaction) * 0.7 + swingCrouch,
      0,
      1,
    );
    const state = this.step ? "stepping" : grab || speed > 0.04 || length(this.reaction) > 0.025 ? "reacting" : "upright";
    return {
      rootTarget, reactionOffset: this.reaction, kneeFlexion,
      supportFeet: { leftFoot: { ...this.feet.leftFoot }, rightFoot: { ...this.feet.rightFoot } },
      step: this.step ? { ...this.step, from: { ...this.step.from }, to: { ...this.step.to },
        requested: this.step.requested ? { ...this.step.requested } : undefined } : null,
      stepCount: this.stepCount, state, shouldFall,
      fallDirection: normalize(add(horizontal(this.comVelocity), scale(horizontal(force), 0.002)), forward),
      appliedGrabForceN: length(force),
      diagnostics: {
        centerOfMass: mass.position, centerOfMassVelocity: { ...this.comVelocity }, capturePoint,
        supportCenter: { ...supportCenter }, supportingFeet: supportingFeet.filter(foot =>
          !(this.step?.foot === foot && this.step.elapsed >= 0)
        ), supportMarginM: supportMargin,
        instabilitySeconds: this.instability, recoveryCapacityM: stepCapacity, externalForce: force,
        balanceAcceleration, stepTarget: this.step ? { ...this.step.to } : null,
      },
    };
  }

  private beginStep(
    foot: Foot,
    from: Vec3,
    root: Vec3,
    forward: Vec3,
    right: Vec3,
    floorY: number,
    correction: Vec3,
    headingRadians: number,
  ): void {
    const side = foot === "leftFoot" ? -1 : 1;
    const lateral = scale(right, side * 0.15);
    const sideName = foot === "leftFoot" ? "left" : "right";
    const localOffset = hindfootFromAnkle(sideName, ZERO, { x: 0, y: 0, z: 0, w: 1 });
    const hindfootOffset = add(scale(right, localOffset.x), scale(forward, localOffset.z));
    const reach = clampLength(add(hindfootOffset, add(lateral, correction)), BALANCE_LIMITS.maxStepReachM);
    const requested = add(root, reach);
    // Commit to a reachable first correction instead of asking one leg to
    // consume the whole capture-point error.  The alternating controller can
    // follow with another bounded step when momentum still requires it.
    const travel = scale(
      clampLength(horizontal(sub(requested, from)), BALANCE_LIMITS.maxStepTravelM),
      0.25,
    );
    // A landing is the flat hindfoot centre. Subtracting a whole-foot preload
    // asks IK for an ankle below its reachable floor-contact height.
    const to = { ...add(from, travel), y: footCenterHeight(foot, floorY) };
    const distance = length(travel);
    const urgency = clamp(length(correction) / BALANCE_LIMITS.maxStepReachM, 0, 1);
    const duration = clamp(
      0.16 + distance / 2.4 - urgency * 0.07,
      BALANCE_LIMITS.minStepDurationS,
      BALANCE_LIMITS.stepDurationS,
    );
    // Shift the measured COM over the retained stance anchor before unloading
    // the swing sole. Positions and velocities remain wholly Rapier-owned.
    this.step = { foot, from: { ...from }, to, requested: { ...requested, y: to.y }, heading: headingRadians, elapsed: -0.50, duration };
    this.touchdownAge = 0;
    this.stanceAge[foot] = 0;
    this.stepCount += 1;
  }
}
