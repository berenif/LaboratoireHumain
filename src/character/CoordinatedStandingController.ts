import { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import type { SegmentId, SegmentPose, Vec3 } from "../core/types";
import type { JointMotorCommand } from "./joint-motors";
import { jointCoordinates, jointCoordinateTargetError, jointFrameAxesWorld, jointRotationFromCoordinates } from "./joint-coordinates";
import { add, angularVelocity, clamp, cross, dot, length, quatInverse, quatMultiply, rotate, scale, sub, worldPoint } from "./math";
import { composeUprightPose } from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const IDENTITY = { ...ZERO, w: 1 };
export const STANDING_FEET: readonly SegmentId[] = ["leftFoot", "leftForefoot", "rightFoot", "rightForefoot"];
export const COORDINATED_STANDING = Object.freeze({
  id: "h74-v1", footStiffnessNpm: 1200, footDampingNspm: 80,
  pelvisStiffnessNmprad: 250, pelvisDampingNmsprad: 30,
  pelvisPositionKpPerS2: 10, pelvisPositionKdPerS: 6,
  bodyLinearDampingPerS: 1, bodyAngularDamping: 0.15, effortRegularization: 0.02,
  deadlineMs: 8, maximumWork: 1600, maximumPelvisDeviationM: 0.1,
  maximumFootDeviationM: 0.03, maximumPelvisRotationRad: 0.3, contactRequiredAfterTick: 120,
});
export const STANDING_UPDATE_ORDER = "balance/contact-plan -> coordinated-feedback -> native-motors -> Rapier -> poses -> contacts -> original-fall-guards";
export type StandingFailure = "timeout" | "infeasible" | "invalid-contact" | "invalid-model"
  | "backup-invalid" | "backup-region-exit" | "invalid-state" | "interaction" | "work-limit";
export interface StandingTelemetry {
  id: string; mode: "investigating" | "transition"; reason: StandingFailure | null;
  tick: number; solveMs: number; work: number; requestedHeadroom: number;
  objective: { feet: number; pelvis: number; motion: number; posture: number; effort: number };
  forwardOffsetM: number;
}
interface FeedbackInput {
  poses: ReadonlyMap<SegmentId, SegmentPose>;
  nominal: readonly JointMotorCommand[];
  gravity: ReadonlyMap<SegmentId, Vec3>;
  contacts: readonly {
    segment: SegmentId; loadBearing: boolean; forceN: number; normalY: number;
  }[];
  tick: number; dt: number; quiet: boolean;
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

/** Experimental feedback, with no predictive or validated-backup claim.
 * All task contributions enter one capped motor command. No world copies,
 * state setters, solver changes, cached commands or moving stance references.
 */
export class CoordinatedStandingController {
  readonly reference: ReadonlyMap<SegmentId, SegmentPose>;
  private telemetry: StandingTelemetry;
  constructor(poses: ReadonlyMap<SegmentId, SegmentPose>, heading: number,
    readonly forwardOffsetM = 0, private readonly clock = () => performance.now()) {
    if (![0, -0.005, 0.005].includes(forwardOffsetM)) throw new Error("Unknown H74 reference");
    const root = add(poses.get("pelvis")!.position,
      rotate({ x: 0, y: Math.sin(heading / 2), z: 0, w: Math.cos(heading / 2) }, { x: 0, y: 0, z: forwardOffsetM }));
    const target = composeUprightPose({ rootTranslation: root, heading,
      kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion, reactionOffset: ZERO,
      simulationTime: 0, activeGrab: null,
      supportFeet: { leftFoot: poses.get("leftFoot")!.position, rightFoot: poses.get("rightFoot")!.position },
      supportFootRotations: { leftFoot: poses.get("leftFoot")!.rotation, rightFoot: poses.get("rightFoot")!.rotation }, step: null });
    this.reference = new Map([...target.poses].map(([id, pose]) => [id, structuredClone(pose)]));
    this.telemetry = { id: COORDINATED_STANDING.id, mode: "investigating", reason: null,
      tick: 0, solveMs: 0, work: 0, requestedHeadroom: 1, forwardOffsetM,
      objective: { feet: 0, pelvis: 0, motion: 0, posture: 0, effort: 0 } };
  }
  diagnostics(): StandingTelemetry { return structuredClone(this.telemetry); }
  serialize() { return { configuration: COORDINATED_STANDING, reference: [...this.reference],
    telemetry: this.diagnostics(), model: null, cache: null, backupRegion: null, updateOrder: STANDING_UPDATE_ORDER }; }
  get active(): boolean { return this.telemetry.mode === "investigating"; }
  handoff(reason: StandingFailure): void { this.transition(reason); }
  private transition(reason: StandingFailure): null {
    this.telemetry.mode = "transition";
    this.telemetry.reason ??= reason;
    return null;
  }
  update(input: FeedbackInput): readonly JointMotorCommand[] | null {
    if (!this.active) return null;
    const started = this.clock(), cfg = COORDINATED_STANDING;
    this.telemetry.tick = input.tick;
    this.telemetry.work = 0;
    const fail = (reason: StandingFailure) => {
      this.telemetry.solveMs = this.clock() - started;
      return this.transition(reason);
    };
    if (input.fault) return fail(input.fault);
    if (!input.quiet) return fail("interaction");
    if (input.dt !== 1 / 60 || input.poses.size !== 25 || [...input.poses.values()].some(p =>
      ![p.position, p.rotation, p.linearVelocity, p.angularVelocity].every(v => Object.values(v).every(Number.isFinite)))) return fail("invalid-state");
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
      forces.set(definition.id, force); moments.set(definition.id, moment);
    }
    const parentMoment = add(scale(pelvisError, cfg.pelvisStiffnessNmprad), scale(pelvis.angularVelocity, -cfg.pelvisDampingNmsprad));
    objective.pelvis = cfg.pelvisStiffnessNmprad * length(pelvisError) ** 2 / 2
      + SEGMENTS.reduce((mass,d) => mass+d.massKg,0) * cfg.pelvisPositionKpPerS2 * length(sub(pelvis.position,refPelvis.position)) ** 2 / 2;
    const commands: JointMotorCommand[] = [];
    let maximumRequestedRatio = 0;
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
      for (const axis of profile.axes) {
        this.telemetry.work++;
        const e = error[axis.coordinate], speed = dot(relativeVelocity, basis[axis.coordinate]);
        const request = (nominal.stiffness * e - nominal.damping * speed + dot(taskTorque, basis[axis.coordinate])) / (1 + cfg.effortRegularization);
        const cap = axis.maxMotorTorqueNm * nominal.strengthScale;
        if (![request, cap, nominal.damping].every(Number.isFinite) || cap <= 0 || nominal.damping <= 0 || nominal.strengthScale !== 1) return fail("infeasible");
        maximumRequestedRatio = Math.max(maximumRequestedRatio, Math.abs(request) / cap);
        const torque = clamp(request, -cap, cap);
        // Encode a current-state torque plus implicit native damping. The
        // ForceBased model and each native ceiling remain unchanged.
        feedforwardWorld = add(feedforwardWorld, scale(basis[axis.coordinate], torque + nominal.damping * speed));
        objective.posture += nominal.stiffness * e * e / 2;
        objective.effort += cfg.effortRegularization * (torque / cap) ** 2 / 2;
      }
      commands.push({ ...nominal, targetLocalRotation: jointRotationFromCoordinates(target, profile),
        stiffness: 0, feedforwardWorld });
    }
    if (commands.length !== 24 || this.telemetry.work > cfg.maximumWork) return fail("infeasible");
    this.telemetry.solveMs = this.clock() - started;
    if (this.telemetry.solveMs > cfg.deadlineMs) return fail("timeout");
    this.telemetry.requestedHeadroom = 1 - maximumRequestedRatio;
    this.telemetry.objective = objective;
    return commands;
  }
}
