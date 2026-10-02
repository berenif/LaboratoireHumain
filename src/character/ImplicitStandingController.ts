import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type { JointCoordinate, SegmentId, SegmentPose, Vec3 } from "../core/types";
import { COORDINATED_STANDING, STANDING_FEET, STANDING_UPDATE_ORDER, type StandingFailure } from "./CoordinatedStandingController";
import type { JointMotorCommand, JointMotorResult } from "./joint-motors";
import { jointCoordinates, jointCoordinateTargetError, jointFrameAxesWorld, jointRotationFromCoordinates } from "./joint-coordinates";
import { add, angularVelocity, clamp, cross, dot, length, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";
import { composeUprightPose } from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const IDENTITY = { ...ZERO, w: 1 };
export const H75_MOTOR_ROUNDING_TOLERANCE_NM = 0.00000190735;
export const IMPLICIT_STANDING = Object.freeze({
  ...COORDINATED_STANDING,
  id: "h77-v1",
  nativeRequestRoundingToleranceNm: H75_MOTOR_ROUNDING_TOLERANCE_NM,
  reacquirePersistenceS: 0.10,
});

export type StandingAuthority = "standing" | "suspended" | "reacquiring" | "transition";
export type StandingStepTrigger = "capture" | "airborne-foot" | "external";

export interface ImplicitStandingTelemetry {
  id: string;
  mode: "investigating" | "transition";
  authority: StandingAuthority;
  reason: StandingFailure | null;
  tick: number;
  solveMs: number;
  work: number;
  requestedHeadroom: number;
  headroom: number;
  nativePdRequest: number;
  residualFeedforward: number;
  nativeReadback: {
    axes: number;
    maximumRequestErrorNm: number;
    toleranceNm: number;
    equivalent: boolean;
  };
  stanceRevision: number;
  triggerReason: StandingStepTrigger | null;
  objective: { feet: number; pelvis: number; motion: number; posture: number; effort: number };
  forwardOffsetM: number;
}

export interface ImplicitFeedbackInput {
  poses: ReadonlyMap<SegmentId, SegmentPose>;
  nominal: readonly JointMotorCommand[];
  gravity: ReadonlyMap<SegmentId, Vec3>;
  contacts: readonly {
    segment: SegmentId;
    loadBearing: boolean;
    forceN: number;
    normalY: number;
    persistenceS?: number;
  }[];
  tick: number;
  dt: number;
  quiet: boolean;
  grabActive?: boolean;
  stepActive?: boolean;
  touchdownQualified?: boolean;
  supportMarginM?: number;
  triggerReason?: StandingStepTrigger | null;
  /** Acceptance-only fault injection; never provided by the application. */
  fault?: StandingFailure;
}

const descendants = new Map(SEGMENTS.map(joint => [joint.id, SEGMENTS.filter(segment => {
  let current: SegmentId | null = segment.id;
  while (current) {
    if (current === joint.id) return true;
    current = SEGMENT_BY_ID.get(current)?.parent ?? null;
  }
  return false;
}).map(segment => segment.id)]));

function stanceReference(poses: ReadonlyMap<SegmentId, SegmentPose>, heading: number, forwardOffsetM: number) {
  const root = add(poses.get("pelvis")!.position,
    rotate({ x: 0, y: Math.sin(heading / 2), z: 0, w: Math.cos(heading / 2) }, { x: 0, y: 0, z: forwardOffsetM }));
  const target = composeUprightPose({ rootTranslation: root, heading,
    kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion, reactionOffset: ZERO,
    simulationTime: 0, activeGrab: null,
    supportFeet: { leftFoot: poses.get("leftFoot")!.position, rightFoot: poses.get("rightFoot")!.position },
    supportFootRotations: { leftFoot: poses.get("leftFoot")!.rotation, rightFoot: poses.get("rightFoot")!.rotation },
    step: null });
  return new Map([...target.poses].map(([id, pose]) => [id, structuredClone(pose)]));
}

/** H77 keeps H74's bounded coordinated request but gives the real reference,
 * kp and kd to Rapier. Only the residual needed to reproduce the current-state
 * request is encoded as feedforward, so coordinate/rate changes inside every
 * solver substep receive native implicit feedback. */
export class ImplicitStandingController {
  reference: ReadonlyMap<SegmentId, SegmentPose>;
  protected telemetry: ImplicitStandingTelemetry;
  private pendingStanceRevision = false;

  constructor(poses: ReadonlyMap<SegmentId, SegmentPose>, private readonly heading: number,
    readonly forwardOffsetM = 0, protected readonly clock = () => performance.now()) {
    if (![0, -0.005, 0.005].includes(forwardOffsetM)) throw new Error("Unknown H77 reference");
    this.reference = stanceReference(poses, heading, forwardOffsetM);
    this.telemetry = { id: IMPLICIT_STANDING.id, mode: "investigating", authority: "standing", reason: null,
      tick: 0, solveMs: 0, work: 0, requestedHeadroom: 1, headroom: 1,
      nativePdRequest: 0, residualFeedforward: 0,
      nativeReadback: { axes: 0, maximumRequestErrorNm: 0,
        toleranceNm: H75_MOTOR_ROUNDING_TOLERANCE_NM, equivalent: true },
      stanceRevision: 0, triggerReason: null, forwardOffsetM,
      objective: { feet: 0, pelvis: 0, motion: 0, posture: 0, effort: 0 } };
  }

  diagnostics(): ImplicitStandingTelemetry { return structuredClone(this.telemetry); }
  serialize() { return { configuration: IMPLICIT_STANDING, reference: [...this.reference],
    telemetry: this.diagnostics(), model: null, cache: null, backupRegion: null,
    pendingStanceRevision: this.pendingStanceRevision, updateOrder: STANDING_UPDATE_ORDER }; }
  restore(state: ReturnType<ImplicitStandingController["serialize"]>): void {
    this.reference = new Map(structuredClone(state.reference));
    this.telemetry = structuredClone(state.telemetry);
    this.pendingStanceRevision = state.pendingStanceRevision;
  }
  /** Quiet-standing authority only. Suspended/reacquiring states intentionally
   * leave the existing bounded balance motors and contact plan in control. */
  get active(): boolean { return this.telemetry.authority === "standing"; }
  handoff(reason: StandingFailure): void { this.transition(reason); }

  private transition(reason: StandingFailure): null {
    this.telemetry.mode = "transition";
    this.telemetry.authority = "transition";
    this.telemetry.reason ??= reason;
    return null;
  }

  private bothFeetPersisted(input: ImplicitFeedbackInput): boolean {
    const sides = new Set(input.contacts.filter(contact => STANDING_FEET.includes(contact.segment)
      && contact.loadBearing && contact.forceN >= 3 && contact.normalY >= 0.65
      && (contact.persistenceS ?? 0) + 1e-9 >= IMPLICIT_STANDING.reacquirePersistenceS)
      .map(contact => SEGMENT_BY_ID.get(contact.segment)!.side));
    return sides.size === 2;
  }

  update(input: ImplicitFeedbackInput): readonly JointMotorCommand[] | null {
    if (this.telemetry.authority === "transition") return null;
    const started = this.clock(), cfg = IMPLICIT_STANDING;
    this.telemetry.tick = input.tick;
    this.telemetry.work = 0;
    this.telemetry.triggerReason = input.triggerReason ?? this.telemetry.triggerReason;
    const finishWithoutAuthority = () => {
      this.telemetry.solveMs = this.clock() - started;
      return null;
    };
    const fail = (reason: StandingFailure) => {
      this.telemetry.solveMs = this.clock() - started;
      return this.transition(reason);
    };
    if (input.fault) return fail(input.fault);
    if (input.dt !== 1 / 60 || input.poses.size !== 25 || [...input.poses.values()].some(p =>
      ![p.position, p.rotation, p.linearVelocity, p.angularVelocity].every(v => Object.values(v).every(Number.isFinite)))) return fail("invalid-state");

    if (input.grabActive || input.stepActive || !input.quiet) {
      this.pendingStanceRevision ||= Boolean(input.stepActive);
      this.telemetry.authority = "suspended";
      return finishWithoutAuthority();
    }
    if (this.telemetry.authority === "suspended") this.telemetry.authority = "reacquiring";
    if (this.telemetry.authority === "reacquiring") {
      const landingAndCapturePass = !this.pendingStanceRevision
        || (Boolean(input.touchdownQualified) && (input.supportMarginM ?? -Infinity) >= 0);
      if (!this.bothFeetPersisted(input) || !landingAndCapturePass) return finishWithoutAuthority();
      if (this.pendingStanceRevision) {
        // Replace, never mutate, the prior stance reference after a physically
        // qualified touchdown and stable double support.
        this.reference = stanceReference(input.poses, this.heading, 0);
        this.telemetry.stanceRevision += 1;
      }
      this.pendingStanceRevision = false;
      this.telemetry.authority = "standing";
    }

    const pelvis = input.poses.get("pelvis")!, refPelvis = this.reference.get("pelvis")!;
    const pelvisError = angularVelocity(pelvis.rotation, refPelvis.rotation, 1);
    if (input.tick > cfg.contactRequiredAfterTick) {
      const sides = new Set(input.contacts.filter(c => STANDING_FEET.includes(c.segment)
        && c.loadBearing && c.forceN >= 3 && c.normalY >= 0.65).map(c => SEGMENT_BY_ID.get(c.segment)!.side));
      if (sides.size !== 2) return fail("invalid-contact");
      if (length(sub(pelvis.position, refPelvis.position)) > cfg.maximumPelvisDeviationM
        || length(pelvisError) > cfg.maximumPelvisRotationRad
        || STANDING_FEET.some(id => length(sub(input.poses.get(id)!.position, this.reference.get(id)!.position)) > cfg.maximumFootDeviationM)) return fail("backup-region-exit");
    }

    const objective = { feet: 0, pelvis: 0, motion: 0, posture: 0, effort: 0 };
    const forces = new Map<SegmentId, Vec3>(), moments = new Map<SegmentId, Vec3>();
    for (const definition of SEGMENTS) {
      const pose = input.poses.get(definition.id)!;
      let force = scale(pose.linearVelocity, -definition.massKg * cfg.bodyLinearDampingPerS);
      const moment = scale(pose.angularVelocity, -definition.massKg * cfg.bodyAngularDamping);
      objective.motion += definition.massKg * (length(pose.linearVelocity) ** 2 * cfg.bodyLinearDampingPerS
        + length(pose.angularVelocity) ** 2 * cfg.bodyAngularDamping) / 2;
      if (STANDING_FEET.includes(definition.id)) {
        const error = sub(this.reference.get(definition.id)!.position, pose.position);
        force = add(force, add(scale(error, cfg.footStiffnessNpm), scale(pose.linearVelocity, -cfg.footDampingNspm)));
        objective.feet += cfg.footStiffnessNpm * length(error) ** 2 / 2;
      }
      forces.set(definition.id, force);
      moments.set(definition.id, moment);
    }
    const parentMoment = add(scale(pelvisError, cfg.pelvisStiffnessNmprad), scale(pelvis.angularVelocity, -cfg.pelvisDampingNmsprad));
    objective.pelvis = cfg.pelvisStiffnessNmprad * length(pelvisError) ** 2 / 2
      + SEGMENTS.reduce((mass, definition) => mass + definition.massKg, 0)
        * cfg.pelvisPositionKpPerS2 * length(sub(pelvis.position, refPelvis.position)) ** 2 / 2;

    const commands: JointMotorCommand[] = [];
    let maximumRequestedRatio = 0;
    let maximumRequest = 0;
    let maximumResidual = 0;
    for (const nominal of input.nominal) {
      if (++this.telemetry.work > cfg.maximumWork) return fail("work-limit");
      if (this.clock() - started > cfg.deadlineMs) return fail("timeout");
      const definition = SEGMENT_BY_ID.get(nominal.id)!;
      if (!definition.parent || !definition.jointProfile) return fail("infeasible");
      const profile = definition.jointProfile;
      const parent = input.poses.get(definition.parent)!, child = input.poses.get(nominal.id)!;
      const target = jointCoordinates(IDENTITY, quatMultiply(quatInverse(this.reference.get(definition.parent)!.rotation),
        this.reference.get(nominal.id)!.rotation), profile);
      const coordinates = jointCoordinates(parent.rotation, child.rotation, profile);
      const error = jointCoordinateTargetError(coordinates, target, profile);
      const basis = jointFrameAxesWorld(parent.rotation, profile);
      const anchor = worldPoint(child.position, child.rotation, profile.childFrame.anchor);
      let taskTorque = input.gravity.get(nominal.id) ?? ZERO;
      for (const id of descendants.get(nominal.id)!) {
        this.telemetry.work++;
        const pose = input.poses.get(id)!;
        taskTorque = add(taskTorque, add(cross(sub(pose.position, anchor), forces.get(id)!), moments.get(id)!));
      }
      if (definition.role === "thigh") taskTorque = add(taskTorque, scale(parentMoment, -0.5));
      let feedforwardWorld = ZERO;
      const relativeVelocity = sub(child.angularVelocity, parent.angularVelocity);
      const standingRequestByAxis: Partial<Record<JointCoordinate, number>> = {};
      const standingUnboundedByAxis: Partial<Record<JointCoordinate, number>> = {};
      const standingResidualByAxis: Partial<Record<JointCoordinate, number>> = {};
      for (const axis of profile.axes) {
        this.telemetry.work++;
        const coordinate = axis.coordinate;
        const e = error[coordinate], rate = dot(relativeVelocity, basis[coordinate]);
        const raw = (nominal.stiffness * e - nominal.damping * rate
          + dot(taskTorque, basis[coordinate])) / (1 + cfg.effortRegularization);
        const cap = axis.maxMotorTorqueNm * nominal.strengthScale;
        if (![raw, cap, nominal.stiffness, nominal.damping].every(Number.isFinite)
          || cap <= 0 || nominal.stiffness <= 0 || nominal.damping <= 0
          || nominal.strengthScale !== 1) return fail("infeasible");
        const request = clamp(raw, -cap, cap);
        standingUnboundedByAxis[coordinate] = raw;
        // The scalar target stored by Rapier is f32. Use that configured
        // reference for the residual identity so target rounding and velocity
        // rounding do not compound beyond H75's retained ABI tolerance.
        const configuredTarget = { ...target, [coordinate]: Math.fround(target[coordinate]) };
        const configuredError = jointCoordinateTargetError(coordinates, configuredTarget, profile)[coordinate];
        const residual = request - nominal.stiffness * configuredError + nominal.damping * rate;
        standingRequestByAxis[coordinate] = request;
        standingResidualByAxis[coordinate] = residual;
        feedforwardWorld = add(feedforwardWorld, scale(basis[coordinate], residual));
        maximumRequestedRatio = Math.max(maximumRequestedRatio, Math.abs(request) / cap);
        maximumRequest = Math.max(maximumRequest, Math.abs(request));
        maximumResidual = Math.max(maximumResidual, Math.abs(residual));
        objective.posture += nominal.stiffness * e * e / 2;
        objective.effort += cfg.effortRegularization * (request / cap) ** 2 / 2;
      }
      commands.push({ ...nominal, targetLocalRotation: jointRotationFromCoordinates(target, profile),
        stiffness: nominal.stiffness, feedforwardWorld, standingRequestByAxis, standingResidualByAxis, standingUnboundedByAxis });
    }
    if (commands.length !== 24 || this.telemetry.work > cfg.maximumWork) return fail("infeasible");
    this.telemetry.solveMs = this.clock() - started;
    if (this.telemetry.solveMs > cfg.deadlineMs) return fail("timeout");
    this.telemetry.requestedHeadroom = 1 - maximumRequestedRatio;
    this.telemetry.headroom = this.telemetry.requestedHeadroom;
    this.telemetry.nativePdRequest = maximumRequest;
    this.telemetry.residualFeedforward = maximumResidual;
    this.telemetry.objective = objective;
    return commands;
  }

  observeNativeResults(results: ReadonlyMap<SegmentId, JointMotorResult>): void {
    if (this.telemetry.authority !== "standing") return;
    let axes = 0;
    let maximumRequest = 0;
    let maximumError = 0;
    for (const result of results.values()) for (const axis of result.nativeMotorAxes ?? []) {
      if (axis.intendedRequestNm === undefined) continue;
      axes++;
      maximumRequest = Math.max(maximumRequest, Math.abs(axis.currentStateRequestNm));
      maximumError = Math.max(maximumError, Math.abs(axis.currentStateRequestNm - axis.intendedRequestNm));
    }
    const equivalent = axes > 0 && maximumError <= H75_MOTOR_ROUNDING_TOLERANCE_NM;
    this.telemetry.nativePdRequest = maximumRequest;
    this.telemetry.nativeReadback = { axes, maximumRequestErrorNm: maximumError,
      toleranceNm: H75_MOTOR_ROUNDING_TOLERANCE_NM, equivalent };
    if (!equivalent) this.transition("invalid-model");
  }
}
