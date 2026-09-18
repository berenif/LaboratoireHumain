import { configureHumanoidWorld } from "./physics-settings";
import RAPIER, {
  type Collider,
  type EventQueue,
  type ImpulseJoint,
  type PhysicsHooks,
  type RigidBody,
  type World,
} from "@dimforge/rapier3d-compat";

import {
  HUMAN_PROPORTIONS,
  REGION_TO_SEGMENT,
  SEGMENT_BY_ID,
  SEGMENTS,
} from "../core/humanoid";
import { flattenGeometryIndices, flattenGeometryVertices, lowestWorldPoint } from "../core/geometry";
import { pickRegionProxies } from "../core/picking";
import type {
  CharacterController,
  DiagnosticsSnapshot,
  GrabCommand,
  GrabControlDiagnostics,
  JointProfile,
  JointStateDiagnostics,
  MotionState,
  PickResult,
  PoseSnapshot,
  Quat,
  Ray,
  RegionId,
  RendererMode,
  SegmentDefinition,
  SegmentId,
  SegmentPose,
  SupportState,
  Vec3,
} from "../core/types";
import { distributeSupportLoad } from "./support-loads";
import { copyStandingContacts, standingChainDiagnostics } from "./standing-chain-diagnostics";
import { BalanceController, type BalanceDiagnostics } from "./BalanceController";
import { DynamicRecovery, emptyRecoveryDiagnostics } from "./DynamicRecovery";
import { GrabAnchorController, emptyGrabDiagnostics } from "./GrabAnchorController";
import { NativeJointMotors } from "./native-joint-motors";
import {
  clampJointCoordinates,
  jointCoordinates,
  jointLimitError,
  jointLimitErrorMagnitude,
  jointRotationFromCoordinates,
} from "./joint-coordinates";
import {
  applyPassiveJointResistance,
  type JointMotorCommand,
  type JointMotorResult,
} from "./joint-motors";
import {
  add,
  clamp,
  clampLength,
  cross,
  dot,
  length,
  normalize,
  quatFromAxisAngle,
  quatInverse,
  quatMultiply,
  rotate,
  scale,
  smooth01,
  sub,
  worldPoint,
} from "./math";
import { createRapierJointLimitAdapter } from "./rapier-joint-adapter";
import {
  composeUprightPose,
  horizontal,
  immutablePose,
  poseAnchor,
  restPoseMap,
  type MutablePose,
  type StepMotion,
} from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const INITIAL_ROOT: Vec3 = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
// Character colliders see both character and environment. The hook removes only
// the explicit anatomical exclusions, leaving nonadjacent self-collision on.
const CHARACTER_GROUP = (0x0001 << 16) | 0x0003;
const ENVIRONMENT_GROUP = (0x0002 << 16) | 0x0001;

type ActiveGrab = {
  pointerId: number;
  region: RegionId;
  segment: SegmentId;
  localAnchor: Vec3;
  target: Vec3;
  startTarget: Vec3;
  startSegmentPosition: Vec3;
  targetVelocity: Vec3;
  lastCommandTimestampMs: number;
  controller: GrabAnchorController;
};

let rapierInitialization: Promise<void> | null = null;

function ensureRapier(): Promise<void> {
  rapierInitialization ??= RAPIER.init();
  return rapierInitialization;
}

function convexCollider(definition: SegmentDefinition): RAPIER.ColliderDesc {
  const descriptor = RAPIER.ColliderDesc.convexMesh(
    flattenGeometryVertices(definition.geometry),
    flattenGeometryIndices(definition.geometry),
  );
  if (!descriptor) throw new Error(`INVALID_CONVEX_GEOMETRY:${definition.id}`);
  return descriptor;
}

function finiteVec(value: Vec3): boolean {
  return Number.isFinite(value.x) && Number.isFinite(value.y) && Number.isFinite(value.z);
}

function finiteQuat(value: Quat): boolean {
  return Number.isFinite(value.x) && Number.isFinite(value.y)
    && Number.isFinite(value.z) && Number.isFinite(value.w);
}

function pairKey(a: SegmentId, b: SegmentId): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

function localAxis(profile: JointProfile): Vec3 {
  const coordinate = profile.axes[0]?.coordinate ?? "x";
  const basis = coordinate === "x" ? { x: 1, y: 0, z: 0 }
    : coordinate === "y" ? { x: 0, y: 1, z: 0 }
      : { x: 0, y: 0, z: 1 };
  return rotate(profile.parentFrame.rotation, basis);
}

function roleGains(definition: SegmentDefinition): Readonly<{ stiffness: number; damping: number }> {
  switch (definition.role) {
    case "lumbar": return { stiffness: 220, damping: 32 };
    case "ribcage": return { stiffness: 200, damping: 30 };
    case "neck": return { stiffness: 12, damping: 1.4 };
    case "head": return { stiffness: 10, damping: 1.2 };
    case "shoulder-girdle": return { stiffness: 12, damping: 1.4 };
    case "upper-arm": return { stiffness: 16, damping: 1.8 };
    case "forearm": return { stiffness: 18, damping: 2 };
    case "forearm-twist": return { stiffness: 8, damping: 0.8 };
    case "hand": return { stiffness: 7, damping: 0.7 };
    case "thigh": return { stiffness: 180, damping: 30 };
    case "shin": return { stiffness: 240, damping: 32 };
    case "ankle": return { stiffness: 120, damping: 20 };
    case "hindfoot": return { stiffness: 90, damping: 16 };
    case "forefoot": return { stiffness: 65, damping: 10 };
    default: return { stiffness: 12, damping: 1.5 };
  }
}

function roleEffort(definition: SegmentDefinition): number {
  void definition;
  return 1;
}

class EmbodiedCharacter implements CharacterController {
  private readonly world: World;
  private readonly nativeMotors: NativeJointMotors;
  /** Kept under the historical name so existing QA probes can inspect handles. */
  private readonly ragdollBodies = new Map<SegmentId, RigidBody>();
  private readonly ragdollColliders = new Map<SegmentId, Collider>();
  private readonly ragdollJoints: ImpulseJoint[] = [];
  private readonly jointsByChild = new Map<SegmentId, ImpulseJoint>();
  private readonly colliderSegments = new Map<number, SegmentId>();
  private readonly excludedPairs = new Set<string>();
  private readonly physicsHooks: PhysicsHooks;
  /** Rapier 0.20 only forwards PhysicsHooks when an EventQueue is supplied. */
  private readonly eventQueue: EventQueue;
  private poses = restPoseMap();
  private previousPoses = restPoseMap();
  private renderer: RendererMode;
  private state: MotionState = "upright";
  private activeGrab: ActiveGrab | null = null;
  private readonly balance = new BalanceController();
  private balanceData: BalanceDiagnostics | null = null;
  private kneeFlexion: number = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
  private recovery = new DynamicRecovery();
  private readonly floorCollider: Collider;
  private heading: number;
  private readonly initialHeading: number;
  private readonly initialPosition: Vec3;
  private grabControlDiagnostics: GrabControlDiagnostics = emptyGrabDiagnostics();
  private paused = false;
  private disposed = false;
  private sequence = 0;
  private simulationTime = 0;
  private reactionOffset: Vec3 = ZERO;
  private leanRadians = 0;
  private supportFeet: Record<"leftFoot" | "rightFoot", Vec3>;
  private step: StepMotion | null = null;
  private stepCount = 0;
  private appliedGrabForceN = 0;
  private fixedSteps = 0;
  private fallingTime = 0;
  private unsupportedTime = 0;
  private grounded = false;
  private uprightBlendTime = Number.POSITIVE_INFINITY;
  private readonly uprightBlendStart = new Map<SegmentId, Vec3>();
  private lastMotorResults = new Map<SegmentId, JointMotorResult>();
  private standingTargets: ReadonlyMap<SegmentId, MutablePose> = new Map();
  private standingCommands: readonly JointMotorCommand[] = [];
  private standingMotorSampleTimeS = 0;
  private standingMotorPelvis: Pick<SegmentPose, "id" | "position" | "rotation"> | null = null;
  private lastContacts = emptyRecoveryDiagnostics().contacts;
  private readonly runtimeErrors: string[] = [];

  constructor(renderer: RendererMode, options: CharacterInitialOptions = {}) {
    this.renderer = renderer;
    this.initialHeading = options.heading ?? 0;
    this.heading = this.initialHeading;
    this.initialPosition = options.position ?? INITIAL_ROOT;
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    configureHumanoidWorld(this.world);
    this.nativeMotors = new NativeJointMotors(this.world);
    this.eventQueue = new RAPIER.EventQueue(true);
    this.physicsHooks = {
      filterContactPair: (collider1, collider2) => {
        const first = this.colliderSegments.get(collider1);
        const second = this.colliderSegments.get(collider2);
        if (first && second && this.excludedPairs.has(pairKey(first, second))) return null;
        return RAPIER.SolverFlags.COMPUTE_IMPULSE;
      },
      filterIntersectionPair: () => true,
    };
    for (const definition of SEGMENTS) {
      for (const excluded of definition.collisionExclusions) {
        this.excludedPairs.add(pairKey(definition.id, excluded));
      }
    }
    const floorBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.08, 0),
    );
    this.floorCollider = this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(5, 0.08, 5)
        .setFriction(4)
        .setRestitution(0.01)
        .setCollisionGroups(ENVIRONMENT_GROUP),
      floorBody,
    );
    this.world.step(this.eventQueue, this.physicsHooks);
    this.poses = this.initialPose();
    this.previousPoses = this.clonePoses(this.poses);
    this.supportFeet = {
      leftFoot: { ...this.poses.get("leftFoot")!.position },
      rightFoot: { ...this.poses.get("rightFoot")!.position },
    };
    this.createDynamicAssembly(this.poses);
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.floorCollider, this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading);
  }

  fixedUpdate(dt: number, command: GrabCommand | null): void {
    if (this.disposed || !Number.isFinite(dt) || dt <= 0) return;
    this.processCommand(command);
    if (this.paused) return;
    const stepDt = clamp(dt, 1 / 240, 1 / 20);
    this.world.timestep = stepDt;
    this.sequence += 1;
    this.fixedSteps += 1;
    this.simulationTime += stepDt;
    this.previousPoses = this.clonePoses(this.poses);
    try {
      if (this.isRecoveryState()) this.updateRecovery(stepDt);
      else this.updateStanding(stepDt);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      if (!this.runtimeErrors.includes(message)) this.runtimeErrors.push(message);
      this.appliedGrabForceN = 0;
    }
  }

  getSnapshot(renderer: RendererMode): PoseSnapshot {
    this.renderer = renderer;
    const pelvis = this.poses.get("pelvis")!;
    return {
      sequence: this.sequence,
      simulationTime: this.simulationTime,
      state: this.state,
      rootPosition: { ...pelvis.position },
      rootRotation: { ...pelvis.rotation },
      segments: SEGMENTS.map((definition) => immutablePose(this.poses.get(definition.id)!)),
      support: this.supportSnapshot(),
      diagnostics: this.diagnostics(),
    };
  }

  pick(ray: Ray): PickResult | null {
    if (this.disposed || !this.diagnostics().bodyInputAvailable) return null;
    return pickRegionProxies(ray, [...this.poses.values()]);
  }

  clearBodyInput(): void { this.clearGrab(); }

  pause(): void {
    this.paused = true;
    this.clearGrab();
  }

  resume(): void {
    if (this.disposed) return;
    this.paused = false;
    this.clearGrab();
  }

  dispose(): void {
    if (this.disposed) return;
    this.paused = true;
    this.clearGrab();
    this.clearDynamicAssembly();
    this.eventQueue.free();
    this.world.free();
    this.disposed = true;
  }

  reset(): void {
    if (this.disposed) return;
    this.clearGrab();
    this.clearDynamicAssembly();
    this.state = "upright";
    this.heading = this.initialHeading;
    this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
    this.balanceData = null;
    this.standingTargets = new Map();
    this.standingCommands = [];
    this.standingMotorSampleTimeS = 0;
    this.standingMotorPelvis = null;
    this.paused = false;
    this.sequence = 0;
    this.simulationTime = 0;
    this.reactionOffset = ZERO;
    this.leanRadians = 0;
    this.step = null;
    this.stepCount = 0;
    this.appliedGrabForceN = 0;
    this.fixedSteps = 0;
    this.fallingTime = 0;
    this.unsupportedTime = 0;
    this.grounded = false;
    this.lastContacts = [];
    this.uprightBlendTime = Number.POSITIVE_INFINITY;
    this.uprightBlendStart.clear();
    this.lastMotorResults.clear();
    this.grabControlDiagnostics = emptyGrabDiagnostics();
    this.runtimeErrors.length = 0;
    this.recovery = new DynamicRecovery();
    this.poses = this.initialPose();
    this.previousPoses = this.clonePoses(this.poses);
    this.supportFeet = {
      leftFoot: { ...this.poses.get("leftFoot")!.position },
      rightFoot: { ...this.poses.get("rightFoot")!.position },
    };
    this.createDynamicAssembly(this.poses);
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.floorCollider, this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading);
  }

  diagnostics(): DiagnosticsSnapshot {
    const root = this.poses.get("pelvis")?.position ?? INITIAL_ROOT;
    const finite = [...this.poses.values()].every((pose) =>
      finiteVec(pose.position) && finiteQuat(pose.rotation)
      && finiteVec(pose.linearVelocity) && finiteVec(pose.angularVelocity)
    );
    const errors = [...this.runtimeErrors];
    if (!finite) errors.push("NONFINITE_CHARACTER_STATE");
    const jointDiagnostics = this.measureJoints();
    const loadBearing = this.lastContacts.filter((contact) => contact.loadBearing);
    return {
      balance: this.balanceData ? structuredClone(this.balanceData) : null,
      bodyInputAvailable: !this.disposed && !this.paused && !this.isRecoveryState(),
      recovery: this.isRecoveryState() ? this.recovery.diagnostics() : emptyRecoveryDiagnostics(),
      grabControl: { ...this.grabControlDiagnostics },
      physicsOwnership: "rapier-dynamic",
      standingChain: this.isRecoveryState() || this.standingTargets.size === 0 ? null
        : standingChainDiagnostics(this.poses, this.standingTargets, this.standingCommands,
          this.step, this.heading, this.standingMotorSampleTimeS, this.simulationTime, this.lastContacts, this.standingMotorPelvis),
      jointDiagnostics,
      contactDiagnostics: {
        contacts: copyStandingContacts(this.lastContacts),
        count: this.lastContacts.length,
        loadBearingCount: loadBearing.length,
        totalNormalForceN: loadBearing.reduce((sum, contact) => sum + contact.forceN, 0),
        supportingSegments: [...new Set(loadBearing.map((contact) => contact.segment))],
      },
      maxJointLimitErrorRad: jointDiagnostics.reduce((maximum, joint) =>
        Math.max(maximum, joint.limitErrorMagnitudeRad), 0),
      maxMotorSaturationRatio: jointDiagnostics.reduce((maximum, joint) =>
        Math.max(maximum, joint.motorSaturationRatio), 0),
      state: this.state,
      simulationReady: true,
      interactiveViewReady: true,
      renderer: this.renderer,
      activeGrab: this.activeGrab !== null,
      activePointerId: this.activeGrab?.pointerId ?? null,
      selectedRegion: this.activeGrab?.region ?? null,
      selectedSegment: this.activeGrab?.segment ?? null,
      queuedTarget: false,
      appliedGrabForceN: this.appliedGrabForceN,
      leanRadians: this.leanRadians,
      rootDisplacementM: Math.hypot(root.x - this.initialPosition.x, root.z - this.initialPosition.z),
      maxJointSeparationM: this.maximumJointSeparation(),
      maxFloorPenetrationM: this.maximumFloorPenetration(),
      stepCount: this.stepCount,
      support: this.supportSnapshot(),
      fixedSteps: this.fixedSteps,
      droppedTimeMs: 0,
      finite,
      errors,
    };
  }

  /** Test fixture entrypoint: reconstruction is explicitly scoped as fixture initialization/reset. */
  seedRecoveryFixture(poses: ReadonlyMap<SegmentId, MutablePose>, direction: Vec3): void {
    if (this.disposed) return;
    this.clearGrab();
    this.clearDynamicAssembly();
    this.poses = this.clonePoses(poses);
    this.previousPoses = this.clonePoses(poses);
    this.createDynamicAssembly(this.poses);
    this.readPhysicsPoses();
    this.activateRagdoll(direction);
  }

  private isRecoveryState(): boolean {
    return this.state === "falling" || this.state === "fallen" || this.state === "recovering";
  }

  private processCommand(command: GrabCommand | null): void {
    if (this.isRecoveryState()) { this.clearGrab(); return; }
    if (!command) {
      if (this.activeGrab) this.activeGrab.targetVelocity = scale(this.activeGrab.targetVelocity, 0.72);
      return;
    }
    if (command.kind === "begin") {
      if (this.paused || this.activeGrab || command.region === undefined) return;
      const segment = command.segment ?? REGION_TO_SEGMENT.get(command.region);
      if (!segment || !SEGMENT_BY_ID.has(segment)) return;
      const pose = this.poses.get(segment);
      if (!pose) return;
      const localAnchor = command.localAnchor ?? ZERO;
      const anchorWorld = worldPoint(pose.position, pose.rotation, localAnchor);
      const target = command.worldTarget ?? anchorWorld;
      this.activeGrab = {
        pointerId: command.pointerId,
        region: command.region,
        segment,
        localAnchor: { ...localAnchor },
        target: { ...target },
        startTarget: { ...target },
        startSegmentPosition: { ...pose.position },
        targetVelocity: ZERO,
        lastCommandTimestampMs: command.timestampMs,
        controller: new GrabAnchorController(target),
      };
      return;
    }
    if (!this.activeGrab || command.pointerId !== this.activeGrab.pointerId) return;
    if (command.kind === "move" && command.worldTarget) {
      const commandDt = clamp(
        (command.timestampMs - this.activeGrab.lastCommandTimestampMs) / 1000,
        1 / 240,
        0.1,
      );
      this.activeGrab.targetVelocity = clampLength(
        scale(sub(command.worldTarget, this.activeGrab.target), 1 / commandDt),
        8,
      );
      this.activeGrab.target = { ...command.worldTarget };
      this.activeGrab.lastCommandTimestampMs = command.timestampMs;
      return;
    }
    if (command.kind === "end" || command.kind === "cancel") this.clearGrab();
  }

  private clearGrab(): void {
    if (this.balanceData) this.balanceData.externalForce = ZERO;
    this.activeGrab = null;
    this.grabControlDiagnostics = emptyGrabDiagnostics(
      this.grabControlDiagnostics.cumulativeInjectedWorkJ,
    );
    this.appliedGrabForceN = 0;
  }

  private updateStanding(dt: number): void {
    // Removing support is an external world change, so a resting island must
    // wake and enter genuine ballistic motion instead of remaining asleep.
    if (!this.floorCollider.isEnabled()) {
      for (const body of this.ragdollBodies.values()) body.wakeUp();
    }
    if (this.activeGrab) {
      const body = this.ragdollBodies.get(this.activeGrab.segment);
      if (body) {
        this.grabControlDiagnostics = this.activeGrab.controller.apply(
          body,
          this.activeGrab.localAnchor,
          this.activeGrab.target,
          dt,
        );
        this.appliedGrabForceN = length(this.grabControlDiagnostics.force);
      }
    } else {
      this.appliedGrabForceN = 0;
    }

    const pelvis = this.poses.get("pelvis")!;
    // Keep the planning heading explicit. Rapier may rotate the pelvis, but
    // that motion must not silently rotate an already committed world target.
    const balanceGrab = this.activeGrab && this.grabControlDiagnostics.active ? {
      ...this.activeGrab,
      target: { ...this.grabControlDiagnostics.controlTarget },
      targetVelocity: { ...this.grabControlDiagnostics.targetVelocity },
    } : this.activeGrab;
    const balance = this.balance.update({
      dt,
      poses: this.poses,
      rootPosition: pelvis.position,
      activeGrab: balanceGrab,
      externalForce: this.activeGrab ? this.grabControlDiagnostics.force : ZERO,
      heading: this.heading,
      contacts: this.lastContacts,
    });
    this.balanceData = balance.diagnostics;
    this.reactionOffset = balance.reactionOffset;
    this.kneeFlexion = balance.kneeFlexion;
    this.supportFeet = balance.supportFeet;
    this.step = balance.step;
    this.stepCount = balance.stepCount;
    this.state = balance.state;

    const target = composeUprightPose({
      rootTranslation: balance.rootTarget,
      heading: this.heading,
      kneeFlexion: this.kneeFlexion,
      reactionOffset: this.reactionOffset,
      simulationTime: this.simulationTime,
      activeGrab: this.activeGrab,
      supportFeet: this.supportFeet,
      step: this.step,
    });
    this.leanRadians = target.leanRadians;
    this.standingTargets = target.poses;
    this.standingCommands = this.motorCommands(target.poses);
    this.standingMotorSampleTimeS = this.simulationTime - dt;
    this.standingMotorPelvis = { id: "pelvis", position: { ...pelvis.position }, rotation: { ...pelvis.rotation } };
    this.lastMotorResults = this.nativeMotors.apply(
      this.ragdollBodies, this.jointsByChild, this.standingCommands,
    );
    this.world.step(this.eventQueue, this.physicsHooks);
    this.readPhysicsPoses();
    this.observeContacts(dt);

    const hasFootSupport = this.supportSnapshot().planted.length > 0;
    this.unsupportedTime = hasFootSupport ? 0 : this.unsupportedTime + dt;
    const supportLossLimit = this.step ? 0.36 : 0.14;
    const physicalFall = this.currentTorsoLean() > 1.25
      || this.poses.get("pelvis")!.position.y < 0.56
      || this.unsupportedTime > supportLossLimit;
    if ((this.activeGrab && balance.shouldFall && this.simulationTime > 0.25) || physicalFall) {
      this.activateRagdoll(balance.fallDirection);
    }
  }

  private updateRecovery(dt: number): void {
    this.clearGrab();
    applyPassiveJointResistance(this.ragdollBodies, dt);
    const result = this.recovery.apply(this.ragdollBodies, dt);
    this.state = result.state;
    this.world.step(this.eventQueue, this.physicsHooks);
    this.readPhysicsPoses();
    this.observeContacts(dt);
    this.fallingTime += dt;
    this.leanRadians = this.currentTorsoLean();
    this.lastMotorResults.clear();
    if (result.recovered) this.finishRecovery();
  }

  /** State-only transition: bodies, positions, rotations, and velocities are untouched. */
  private activateRagdoll(direction: Vec3): void {
    if (this.isRecoveryState()) return;
    const pelvisVelocity = this.poses.get("pelvis")?.linearVelocity ?? ZERO;
    const fallDirection = normalize(
      add(horizontal(direction), scale(horizontal(pelvisVelocity), 0.12)),
      rotate(quatFromAxisAngle(UP, this.heading), FORWARD),
    );
    this.nativeMotors.disable(this.jointsByChild);
    this.state = "falling";
    this.recovery.reset(this.heading, fallDirection);
    this.balanceData = null;
    this.clearGrab();
    this.fallingTime = 0;
    this.unsupportedTime = 0;
    this.uprightBlendStart.clear();
    this.lastMotorResults.clear();
  }

  /** Recovery changes motor intent only; the continuous Rapier state remains authoritative. */
  private finishRecovery(): void {
    const pelvis = this.poses.get("pelvis")!;
    const forward = horizontal(rotate(pelvis.rotation, FORWARD));
    if (length(forward) > 1e-5) this.heading = Math.atan2(forward.x, forward.z);
    this.supportFeet = {
      leftFoot: { ...this.poses.get("leftFoot")!.position },
      rightFoot: { ...this.poses.get("rightFoot")!.position },
    };
    this.reactionOffset = ZERO;
    this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
    this.step = null;
    this.state = "upright";
    this.grounded = this.lastContacts.some(contact => contact.loadBearing);
    this.unsupportedTime = 0;
    this.uprightBlendTime = 0;
    this.uprightBlendStart.clear();
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointProfile) continue;
      const parent = this.ragdollBodies.get(definition.parent)!;
      const child = this.ragdollBodies.get(definition.id)!;
      this.uprightBlendStart.set(
        definition.id,
        jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile),
      );
    }
    this.balance.reset(this.poses, this.heading);
    this.balanceData = null;
  }

  private motorCommands(targets: ReadonlyMap<SegmentId, MutablePose>): JointMotorCommand[] {
    const blend = smooth01(clamp(this.uprightBlendTime / 0.55, 0, 1));
    const strength = this.uprightBlendTime === Number.POSITIVE_INFINITY ? 1 : 0.12 + 0.88 * blend;
    if (this.uprightBlendTime !== Number.POSITIVE_INFINITY) {
      this.uprightBlendTime += this.world.timestep;
      if (this.uprightBlendTime >= 0.55) {
        this.uprightBlendTime = Number.POSITIVE_INFINITY;
        this.uprightBlendStart.clear();
      }
    }
    const commands: JointMotorCommand[] = [];
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointProfile) continue;
      const parentTarget = targets.get(definition.parent);
      const childTarget = targets.get(definition.id);
      if (!parentTarget || !childTarget) continue;
      const relative = quatMultiply(quatInverse(parentTarget.rotation), childTarget.rotation);
      const desired = clampJointCoordinates(
        jointCoordinates({ x: 0, y: 0, z: 0, w: 1 }, relative, definition.jointProfile),
        definition.jointProfile,
      );
      const start = this.uprightBlendStart.get(definition.id);
      const coordinates = start ? {
        x: start.x + (desired.x - start.x) * blend,
        y: start.y + (desired.y - start.y) * blend,
        z: start.z + (desired.z - start.z) * blend,
      } : desired;
      const gains = roleGains(definition);
      commands.push({
        id: definition.id,
        targetLocalRotation: jointRotationFromCoordinates(coordinates, definition.jointProfile),
        stiffness: gains.stiffness,
        damping: gains.damping,
        strengthScale: strength,
        effortScale: roleEffort(definition),
        feedforwardWorld: this.gravityCompensation(definition),
      });
    }
    return commands;
  }

  /**
   * Cancel descendant gravity and route the requested horizontal ground wrench
   * through the loaded foot chains.  The chosen pressure point is the COM for
   * static equilibrium and shifts by h/g*a while balance is actively correcting.
   */
  private gravityCompensation(jointDefinition: SegmentDefinition): Vec3 {
    const child = this.ragdollBodies.get(jointDefinition.id);
    if (!child || !jointDefinition.jointProfile) return ZERO;
    const jointWorld = worldPoint(
      child.translation(),
      child.rotation(),
      jointDefinition.jointProfile.childFrame.anchor,
    );
    let torque: Vec3 = ZERO;
    for (const candidate of SEGMENTS) {
      let ancestor: SegmentId | null = candidate.id;
      let descendant = false;
      while (ancestor) {
        if (ancestor === jointDefinition.id) { descendant = true; break; }
        ancestor = SEGMENT_BY_ID.get(ancestor)?.parent ?? null;
      }
      if (!descendant) continue;
      const body = this.ragdollBodies.get(candidate.id);
      if (!body) continue;
      torque = add(torque, cross(
        sub(body.worldCom(), jointWorld),
        { x: 0, y: body.mass() * 9.81, z: 0 },
      ));
    }
    const contacts = this.lastContacts.filter((contact) => {
      const role = SEGMENT_BY_ID.get(contact.segment)?.role;
      return this.floorCollider.isEnabled() && contact.forceN >= 3 && contact.normalY >= 0.65
        && (role === "hindfoot" || role === "forefoot");
    });
    // No fallback soles, desired landing patches or internal self-contacts.
    // A transient weak contact cannot authorize a full body-weight reaction.
    const measuredLoad = contacts.reduce((sum, contact) => sum + contact.forceN * contact.normalY, 0);
    if (measuredLoad <= 0) return torque;
    const mass = [...this.ragdollBodies.values()].reduce((sum, body) => sum + body.mass(), 0);
    const acceleration = this.balanceData?.balanceAcceleration ?? ZERO;
    const capacity = Math.min(measuredLoad, mass * 9.81);
    const groundForce = {
      x: mass * acceleration.x * capacity / (mass * 9.81),
      y: capacity,
      z: mass * acceleration.z * capacity / (mass * 9.81),
    };
    const com = this.balanceData?.centerOfMass ?? ZERO;
    const floorY = contacts.reduce((sum, contact) => sum + contact.point.y, 0) / contacts.length;
    const height = Math.max(0, com.y - floorY);
    const supports = distributeSupportLoad(contacts, {
      x: com.x - height * groundForce.x / capacity,
      y: floorY,
      z: com.z - height * groundForce.z / capacity,
    });
    for (const support of supports) {
      let ancestor: SegmentId | null = support.segment;
      let belowJoint = false;
      while (ancestor) {
        if (ancestor === jointDefinition.id) { belowJoint = true; break; }
        ancestor = SEGMENT_BY_ID.get(ancestor)?.parent ?? null;
      }
      if (!belowJoint) continue;
      torque = add(torque, cross(
        sub(support.point, jointWorld),
        {
          x: -groundForce.x * support.share,
          y: -groundForce.y * support.share,
          z: -groundForce.z * support.share,
        },
      ));
    }
    return torque;
  }

  private createDynamicAssembly(
    seed: ReadonlyMap<SegmentId, MutablePose>,
  ): void {
    if (this.ragdollBodies.size || this.ragdollJoints.length) {
      throw new Error("DYNAMIC_ASSEMBLY_ALREADY_EXISTS");
    }
    for (const definition of SEGMENTS) {
      const pose = seed.get(definition.id);
      if (!pose) throw new Error(`MISSING_SEED_POSE:${definition.id}`);
      const body = this.world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(pose.position.x, pose.position.y, pose.position.z)
          .setRotation(pose.rotation)
          .setLinvel(pose.linearVelocity.x, pose.linearVelocity.y, pose.linearVelocity.z)
          .setAngvel(pose.angularVelocity)
          .setLinearDamping(0)
          .setAngularDamping(0.52)
          .setCanSleep(false)
          .setCcdEnabled(true)
          .setAdditionalSolverIterations(5),
      );
      const collider = this.world.createCollider(
        convexCollider(definition)
          .setMass(definition.massKg)
          .setFriction(
            definition.role === "hindfoot" || definition.role === "forefoot" ? 4 : 0.45,
          )
          .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
          .setRestitution(0.02)
          .setContactSkin(0.0015)
          .setCollisionGroups(CHARACTER_GROUP)
          .setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS),
        body,
      );
      body.recomputeMassPropertiesFromColliders();
      this.ragdollBodies.set(definition.id, body);
      this.ragdollColliders.set(definition.id, collider);
      this.colliderSegments.set(collider.handle, definition.id);
    }
    const adapter = createRapierJointLimitAdapter(this.world);
    for (const definition of SEGMENTS) {
      const profile = definition.jointProfile;
      if (!definition.parent || !profile) continue;
      const parent = this.ragdollBodies.get(definition.parent)!;
      const child = this.ragdollBodies.get(definition.id)!;
      const xHinge = profile.kind === "hinge" && profile.axes[0]?.coordinate === "x";
      // Absent coordinates are bilateral structural locks, not zero-width
      // unilateral limit rows. Under the full body's load a [0, 0] limit on
      // a spherical joint can creep; the subtalar foot then gains a false
      // sagittal hinge that bypasses the ankle motor entirely.
      const lockedMask = RAPIER.JointAxesMask.LinX | RAPIER.JointAxesMask.LinY | RAPIER.JointAxesMask.LinZ
        | (profile.axes.some(axis => axis.coordinate === "x") ? 0 : RAPIER.JointAxesMask.AngX)
        | (profile.axes.some(axis => axis.coordinate === "y") ? 0 : RAPIER.JointAxesMask.AngY)
        | (profile.axes.some(axis => axis.coordinate === "z") ? 0 : RAPIER.JointAxesMask.AngZ);
      const data = xHinge
        ? RAPIER.JointData.revoluteWithAxes(
          profile.parentFrame.anchor,
          profile.childFrame.anchor,
          localAxis(profile),
          rotate(profile.childFrame.rotation, { x: 1, y: 0, z: 0 }),
        )
         : RAPIER.JointData.generic(
          profile.parentFrame.anchor, profile.childFrame.anchor,
          localAxis(profile), lockedMask,
        );
      const joint = this.world.createImpulseJoint(data, parent, child, false);
      joint.setLocalFrame1(profile.parentFrame.anchor, profile.parentFrame.rotation);
      joint.setLocalFrame2(profile.childFrame.anchor, profile.childFrame.rotation);
      joint.setContactsEnabled(false);
      adapter.constrain(joint, profile);
      this.ragdollJoints.push(joint);
      this.jointsByChild.set(definition.id, joint);
    }
  }

  private clearDynamicAssembly(): void {
    for (const joint of this.ragdollJoints.splice(0)) {
      if (joint.isValid()) this.world.removeImpulseJoint(joint, false);
    }
    this.jointsByChild.clear();
    for (const body of this.ragdollBodies.values()) {
      if (body.isValid()) this.world.removeRigidBody(body);
    }
    this.ragdollBodies.clear();
    this.ragdollColliders.clear();
    this.colliderSegments.clear();
  }

  private initialPose(): Map<SegmentId, MutablePose> {
    const base = restPoseMap();
    const heading = quatFromAxisAngle(UP, this.initialHeading);
    const transformed = new Map<SegmentId, MutablePose>();
    for (const [id, pose] of base) {
      transformed.set(id, {
        id,
        position: add(this.initialPosition, rotate(heading, sub(pose.position, INITIAL_ROOT))),
        rotation: quatMultiply(heading, pose.rotation),
        linearVelocity: ZERO,
        angularVelocity: ZERO,
      });
    }
    const feet = {
      leftFoot: transformed.get("leftFoot")!.position,
      rightFoot: transformed.get("rightFoot")!.position,
    };
    return composeUprightPose({
      rootTranslation: this.initialPosition,
      heading: this.initialHeading,
      kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
      reactionOffset: ZERO,
      simulationTime: 0,
      activeGrab: null,
      supportFeet: feet,
      step: null,
    }).poses;
  }

  private readPhysicsPoses(): void {
    for (const definition of SEGMENTS) {
      const body = this.ragdollBodies.get(definition.id);
      if (!body) continue;
      const translation = body.translation();
      const rotation = body.rotation();
      this.poses.set(definition.id, {
        id: definition.id,
        massKg: body.mass(),
        centerOfMass: { ...body.worldCom() },
        position: { x: translation.x, y: translation.y, z: translation.z },
        rotation: { x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w },
        linearVelocity: { ...body.linvel() },
        angularVelocity: { ...body.angvel() },
      });
    }
  }

  private observeContacts(dt: number): void {
    this.recovery.observe(
      this.world,
      this.floorCollider,
      this.floorCollider.isEnabled() ? this.ragdollColliders : new Map(),
      this.ragdollBodies,
      dt,
    );
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({
      ...contact,
      point: { ...contact.point },
      points: contact.points?.map((point) => ({ ...point })),
    }));
    this.grounded = this.lastContacts.some((contact) => contact.loadBearing);
  }

  private measureJoints(): JointStateDiagnostics[] {
    const result: JointStateDiagnostics[] = [];
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointProfile) continue;
      const parent = this.ragdollBodies.get(definition.parent);
      const child = this.ragdollBodies.get(definition.id);
      if (!parent || !child) continue;
      const coordinates = jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile);
      const error = jointLimitError(coordinates, definition.jointProfile);
      const motor = this.lastMotorResults.get(definition.id);
      result.push({
        segment: definition.id,
        coordinates,
        targetCoordinates: motor?.targetCoordinates ?? coordinates,
        limitError: error,
        limitErrorMagnitudeRad: jointLimitErrorMagnitude(coordinates, definition.jointProfile),
        motorTorqueWorld: { ...(motor?.torqueWorld ?? ZERO) },
        motorTorqueSource: motor?.torqueSource ?? "applied-impulse",
        motorTorqueNm: length(motor?.torqueWorld ?? ZERO),
        motorSaturationRatio: motor?.saturationRatio ?? 0,
      });
    }
    return result;
  }

  private clonePoses(source: ReadonlyMap<SegmentId, MutablePose>): Map<SegmentId, MutablePose> {
    return new Map([...source].map(([id, pose]) => [id, {
      id,
      massKg: pose.massKg,
      centerOfMass: pose.centerOfMass ? { ...pose.centerOfMass } : undefined,
      position: { ...pose.position },
      rotation: { ...pose.rotation },
      linearVelocity: { ...pose.linearVelocity },
      angularVelocity: { ...pose.angularVelocity },
    }]));
  }

  private supportSnapshot(): SupportState {
    if (this.disposed) {
      return { planted: [], swingFoot: null, stepProgress: 0, grounded: false };
    }
    const planted = (["left", "right"] as const).flatMap((side) => {
      const foot = `${side}Foot` as const;
      // A requested swing/grab is intent, not evidence that a real contact unloaded.
      const pose = this.poses.get(foot);
      const definition = SEGMENT_BY_ID.get(foot);
      const soleNearFloor = Boolean(pose && definition
        && lowestWorldPoint(definition.geometry, pose.position, pose.rotation).y <= 0.025
        && dot(rotate(pose.rotation, UP), UP) > 0.5);
      const hasContact = this.lastContacts.some((contact) => {
        const definition = SEGMENT_BY_ID.get(contact.segment);
        return contact.loadBearing && definition?.side === side
          && (definition.role === "ankle" || definition.role === "hindfoot" || definition.role === "forefoot");
      }) && soleNearFloor;
      return hasContact ? [foot] : [];
    });
    return {
      planted,
      swingFoot: this.isRecoveryState() || !this.step || this.step.elapsed < 0 ? null : this.step.foot,
      stepProgress: !this.isRecoveryState() && this.step
        ? clamp(this.step.elapsed / this.step.duration, 0, 1) : 0,
      grounded: this.grounded || planted.length > 0,
    };
  }

  private maximumJointSeparation(): number {
    let maximum = 0;
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointAnchorParent || !definition.jointAnchorChild) continue;
      const parent = this.poses.get(definition.parent);
      const child = this.poses.get(definition.id);
      if (!parent || !child) continue;
      maximum = Math.max(maximum, length(sub(
        poseAnchor(parent, definition.jointAnchorParent),
        poseAnchor(child, definition.jointAnchorChild),
      )));
    }
    return maximum;
  }

  private maximumFloorPenetration(): number {
    if (this.disposed || !this.floorCollider.isEnabled()) return 0;
    let penetration = 0;
    for (const collider of this.ragdollColliders.values()) {
      const contact = this.floorCollider.contactCollider(collider, 0);
      if (contact) penetration = Math.max(penetration, -contact.distance);
    }
    return penetration;
  }

  private currentTorsoLean(): number {
    const torso = this.poses.get("torso");
    if (!torso) return 0;
    return Math.acos(clamp(dot(normalize(rotate(torso.rotation, UP)), UP), -1, 1));
  }
}

export interface CharacterInitialOptions { heading?: number; position?: Vec3 }

export async function createEmbodiedCharacter(
  initialRenderer: RendererMode = "canvas2d",
  options: CharacterInitialOptions = {},
): Promise<CharacterController> {
  await ensureRapier();
  return new EmbodiedCharacter(initialRenderer, options);
}
