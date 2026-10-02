import { SEGMENT_BY_ID } from "../core/humanoid";
import type { JointCoordinate, SegmentId, SegmentPose, SupportingContact, Vec3 } from "../core/types";
import { ImplicitStandingController, IMPLICIT_STANDING, type ImplicitFeedbackInput } from "./ImplicitStandingController";
import { contactJointTorque, isBelowJoint } from "./contact-joint-torque";
import { CONTACT_SOLVE, solveContactForces, type ContactMotorRow, type ContactSolveInput } from "./coordinated-contact-solve";
import { createSupportState, soleContacts, type SupportState } from "./support-state";
import { jointCoordinates, jointCoordinateTargetError, jointFrameAxesWorld } from "./joint-coordinates";
import { add, clamp, dot, scale, sub, worldPoint } from "./math";
import type { JointMotorCommand, JointMotorResult } from "./joint-motors";

const ZERO = { x: 0, y: 0, z: 0 }, IDENTITY = { ...ZERO, w: 1 };
export const CONTACT_STANDING = Object.freeze({ ...IMPLICIT_STANDING, id: "h78-v1" });
export interface ContactFeedbackInput extends ImplicitFeedbackInput {
  allocation?: { contacts: readonly SupportingContact[]; requestedForce: Vec3; requestedMoment: Vec3;
    centerOfMass: Vec3; weightN: number; momentLengthM: number; maxHorizontalForceN: number;
    supportState: SupportState };
}

/** H77 posture/reference/gains with contact allocation inside the same motor
 * ceilings. Recovery remains a latched bounded handoff, never standing backup. */
export class ContactStandingController extends ImplicitStandingController {
  protected get candidateId(): string { return CONTACT_STANDING.id; }
  protected get reuseContactColumns(): boolean { return false; }
  protected allocate(input: ContactSolveInput) { return solveContactForces(input); }
  get transitioning(): boolean { return this.telemetry.authority === "transition"; }
  supportState: SupportState;
  private allocation: ReturnType<typeof solveContactForces> | null = null;
  private requestedForce = { ...ZERO };
  private requestedMoment = { ...ZERO };
  private measuredForce = { x: null, y: 0, z: null };
  constructor(poses: ReadonlyMap<SegmentId, SegmentPose>, heading: number, offset = 0, clock = () => performance.now()) {
    super(poses, heading, offset, clock);
    this.telemetry.id = this.candidateId;
    this.supportState = createSupportState(poses.get("pelvis")!.position);
  }
  override diagnostics() { return { ...super.diagnostics(), allocation: structuredClone(this.allocation),
    requestedForce: { ...this.requestedForce }, requestedMoment: { ...this.requestedMoment },
    measuredForce: { ...this.measuredForce }, supportState: structuredClone(this.supportState),
    forecastError: { status: "unvalidated", footLoadChangeWeightFraction: null, horizontalComM: null, readinessTimeS: null },
    measuredForceQualification: "Normal impulse measurement only; horizontal contact impulses are not exposed by this API." }; }
  override serialize() { return { ...super.serialize(), candidateConfiguration: { ...CONTACT_STANDING, id: this.candidateId },
    solverConfiguration: CONTACT_SOLVE, supportState: structuredClone(this.supportState) }; }
  override restore(state: ReturnType<ImplicitStandingController["serialize"]>): void {
    super.restore(state);
    const full = state as ReturnType<ContactStandingController["serialize"]>;
    if (!full.supportState || full.telemetry.id !== this.candidateId) throw new Error("Invalid contact controller state");
    this.supportState = structuredClone(full.supportState);
    const telemetry = full.telemetry as ReturnType<ContactStandingController["diagnostics"]>;
    this.allocation = structuredClone(telemetry.allocation);
    this.requestedForce = { ...telemetry.requestedForce };
    this.requestedMoment = { ...telemetry.requestedMoment };
    this.measuredForce = { ...telemetry.measuredForce };
  }
  override update(input: ContactFeedbackInput): readonly JointMotorCommand[] | null {
    const started = this.clock();
    const posture = super.update(input);
    if (this.telemetry.authority === "transition") return null;
    if (!input.allocation) { this.handoff("invalid-state"); return null; }
    const a = input.allocation, contacts = soleContacts(a.contacts);
    this.supportState = structuredClone(a.supportState);
    this.supportState.stanceRevision = Math.max(this.supportState.stanceRevision, this.telemetry.stanceRevision);
    this.supportState.previousAllocation = this.supportState.previousAllocation.filter(p => contacts.some(c => c.segment === p.segment));
    this.requestedForce = { ...a.requestedForce }; this.requestedMoment = { ...a.requestedMoment };
    this.measuredForce = { x: null, y: contacts.reduce((sum, c) => sum + (c.measuredForceN ?? c.forceN) * c.normalY, 0), z: null };
    // Startup has no invented floor support. H77's original contact deadline
    // still decides when this becomes an invalid-contact transition.
    if (!contacts.length) { this.allocation = null; return posture; }
    const source = posture ?? input.nominal;
    if (source.length !== 24 || source.some(command => command.strengthScale !== 1
      || !SEGMENT_BY_ID.get(command.id)?.jointProfile
      || ![command.stiffness, command.damping].every(value => Number.isFinite(value) && value > 0))) {
      this.handoff("infeasible"); return null;
    }
    const rows: ContactMotorRow[] = [];
    const metadata = source.map(command => {
      const definition = SEGMENT_BY_ID.get(command.id)!, profile = definition.jointProfile!;
      const parent = input.poses.get(definition.parent!)!, child = input.poses.get(command.id)!;
      const basis = jointFrameAxesWorld(parent.rotation, profile);
      const target = jointCoordinates(IDENTITY, command.targetLocalRotation, profile);
      const current = jointCoordinates(parent.rotation, child.rotation, profile);
      const error = jointCoordinateTargetError(current, target, profile);
      const anchor = worldPoint(child.position, child.rotation, profile.childFrame.anchor);
      const rates = sub(child.angularVelocity, parent.angularVelocity);
      const regularization = 1 + IMPLICIT_STANDING.effortRegularization;
      // H80 computes each world-space contact column once per joint, then
      // projects it onto all native axes. H78/H79 retain their frozen path.
      const columns = this.reuseContactColumns ? contacts.flatMap(c => {
        if (!isBelowJoint(c.segment, command.id)) return [ZERO, ZERO, ZERO];
        return ["x", "y", "z"].map(key => contactJointTorque(command.id, anchor, c.segment, c.point,
          { x: Number(key === "x"), y: Number(key === "y"), z: Number(key === "z") }));
      }) : null;
      for (const axis of profile.axes) {
        const coordinate = axis.coordinate;
        const baseNm = command.standingUnboundedByAxis?.[coordinate]
          ?? (command.stiffness * error[coordinate] - command.damping * dot(rates, basis[coordinate])
            + dot(command.feedforwardWorld ?? ZERO, basis[coordinate])) / regularization;
        rows.push({ id: command.id, coordinate, baseNm, capNm: axis.maxMotorTorqueNm * command.strengthScale,
          coefficients: columns ? columns.map(column => dot(column, basis[coordinate]) * a.weightN / regularization)
            : contacts.flatMap(c => ["x", "y", "z"].map(key => dot(
            contactJointTorque(command.id, anchor, c.segment, c.point,
              { x: Number(key === "x"), y: Number(key === "y"), z: Number(key === "z") }), basis[coordinate]) * a.weightN / regularization)) });
      }
      return { command, profile, basis, current, target, rates };
    });
    this.allocation = this.allocate({ contacts, origin: a.centerOfMass, requestedForce: a.requestedForce,
      requestedMoment: a.requestedMoment, weightN: a.weightN, momentLengthM: a.momentLengthM,
      maxHorizontalForceN: a.maxHorizontalForceN, previous: this.supportState.previousAllocation,
      motors: rows, clock: this.clock, deadlineAt: started + CONTACT_SOLVE.deadlineMs });
    if (this.allocation.status !== "allocated") {
      this.telemetry.solveMs = this.clock() - started;
      this.handoff(this.allocation.status); return null;
    }
    const solution = this.allocation;
    this.supportState.previousAllocation = solution.loads.map(c => ({ segment: c.segment, force: { ...c.plannedForce } }));
    const forces = solution.loads.flatMap(c => Object.values(scale(c.plannedForce, 1 / a.weightN)));
    const commands = metadata.map(({ command, profile, basis, current, target, rates }) => {
      let feedforwardWorld = { ...ZERO };
      const standingRequestByAxis: Partial<Record<JointCoordinate, number>> = {};
      const standingResidualByAxis: Partial<Record<JointCoordinate, number>> = {};
      for (const axis of profile.axes) {
        const coordinate = axis.coordinate, row = rows.find(r => r.id === command.id && r.coordinate === coordinate)!;
        const request = clamp(row.baseNm + row.coefficients.reduce((sum, c, i) => sum + c * forces[i], 0), -row.capNm, row.capNm);
        const configuredError = jointCoordinateTargetError(current, { ...target, [coordinate]: Math.fround(target[coordinate]) }, profile)[coordinate];
        const residual = request - command.stiffness * configuredError + command.damping * dot(rates, basis[coordinate]);
        standingRequestByAxis[coordinate] = request; standingResidualByAxis[coordinate] = residual;
        feedforwardWorld = add(feedforwardWorld, scale(basis[coordinate], residual));
      }
      return { ...command, feedforwardWorld, standingRequestByAxis, standingResidualByAxis };
    });
    this.telemetry.requestedHeadroom = solution.headroom; this.telemetry.headroom = solution.headroom;
    this.telemetry.solveMs = this.clock() - started;
    if (this.telemetry.solveMs > CONTACT_SOLVE.deadlineMs) { this.handoff("timeout"); return null; }
    return commands;
  }
  override observeNativeResults(results: ReadonlyMap<SegmentId, JointMotorResult>): void {
    if (this.transitioning || this.allocation?.status !== "allocated") return;
    const authority = this.telemetry.authority;
    this.telemetry.authority = "standing";
    super.observeNativeResults(results);
    if (!this.transitioning) this.telemetry.authority = authority;
  }
}
