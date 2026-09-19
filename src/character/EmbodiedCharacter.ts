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
  TOTAL_MASS_KG,
} from "../core/humanoid";
import { flattenGeometryIndices, flattenGeometryVertices, lowestWorldPoint } from "../core/geometry";
import { pickRegionProxies } from "../core/picking";
import { ARENA, playgroundStation, type PlaygroundConfig } from "../core/playground";
import { PROTOCOL_ROOM } from "../core/protocol";
import { PhysicsPlayground } from "./PhysicsPlayground";
import { PhysicsStriker } from "./PhysicsStriker";
import type {
  CharacterController,
  DiagnosticsSnapshot,
  GrabCommand,
  GrabControlDiagnostics,
  JointCoordinate,
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
  SupportState,
  Vec3,
} from "../core/types";
import { BalanceController, BALANCE_LIMITS, type BalanceDiagnostics } from "./BalanceController";
import { minimumSupportTorqueLimit, planContactLoads, type ContactLoadPlan } from "./contact-loads";
import { DynamicRecovery, emptyRecoveryDiagnostics } from "./DynamicRecovery";
import { GrabAnchorController, emptyGrabDiagnostics } from "./GrabAnchorController";
import {
  collisionAwareLimbTarget,
  initializeLimbCollisionQueries,
  limbBodyClearance,
  segmentBodyClearance,
} from "./limb-collisions";
import type { ArticulatedSupportConstraint } from "./articulated-inertia";
import {
  clampJointCoordinates,
  jointCoordinates,
  jointLimitError,
  jointLimitErrorMagnitude,
  jointRotationFromCoordinates,
} from "./joint-coordinates";
import {
  applyCoupledJointMotors,
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
  quatFromTo,
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
  type UprightPoseInput,
} from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const INITIAL_ROOT: Vec3 = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
// Character colliders see both character and environment. The hook removes only
// the explicit anatomical exclusions, leaving nonadjacent self-collision on.
const CHARACTER_GROUP = (0x0001 << 16) | 0x0003;
const ENVIRONMENT_GROUP = (0x0002 << 16) | 0x0001;
const COORDINATES = ["x", "y", "z"] as const satisfies readonly JointCoordinate[];
const SEGMENT_BOUNDING_RADII = new Map(SEGMENTS.map((definition) => [
  definition.id,
  Math.max(...definition.geometry.vertices.map(({ x, y, z }) => Math.hypot(x, y, z))),
]));

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
  rapierInitialization ??= RAPIER.init().then(initializeLimbCollisionQueries);
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
  private initialPosition: Vec3;
  private playgroundConfig?: PlaygroundConfig;
  private playground: PhysicsPlayground | null = null;
  private readonly room: boolean;
  private readonly striker: PhysicsStriker | null;
  private protocolStrikes = 0;
  private protocolRecoveries = 0;
  private protocolMessage: NonNullable<PoseSnapshot["protocol"]>["message"] = null;
  private grabControlDiagnostics: GrabControlDiagnostics = emptyGrabDiagnostics();
  private paused = false;
  private disposed = false;
  private sequence = 0;
  private simulationTime = 0;
  private reactionOffset: Vec3 = ZERO;
  private leanRadians = 0;
  private supportFeet: Record<"leftFoot" | "rightFoot", Vec3>;
  private supportFootRotations: Record<"leftFoot" | "rightFoot", Quat>;
  private contactLoadPlan: ContactLoadPlan = {
    loads: [], requestedForce: ZERO, allocatedForce: ZERO, pressurePoint: ZERO,
  };
  private step: StepMotion | null = null;
  private stepCount = 0;
  private appliedGrabForceN = 0;
  private fixedSteps = 0;
  private fallingTime = 0;
  private unsupportedTime = 0;
  private grounded = true;
  private uprightBlendTime = Number.POSITIVE_INFINITY;
  private readonly uprightBlendStart = new Map<SegmentId, Vec3>();
  private lastMotorResults = new Map<SegmentId, JointMotorResult>();
  private lastContacts = emptyRecoveryDiagnostics().contacts;
  private readonly runtimeErrors: string[] = [];

  constructor(renderer: RendererMode, options: CharacterInitialOptions = {}) {
    this.renderer = renderer;
    this.room = Boolean(options.room);
    this.initialHeading = options.heading ?? 0;
    this.heading = this.initialHeading;
    this.initialPosition = options.position ?? INITIAL_ROOT;
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    this.world.timestep = 1 / 60;
    // The room's fixed striker produces a sharp Rapier contact. Extra solver
    // iterations resolve that impulse across the bounded shoulder joints.
    this.world.numSolverIterations = this.room ? 80 : 20;
    this.world.numInternalPgsIterations = 4;
    this.world.integrationParameters.maxCcdSubsteps = 4;
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
      RAPIER.ColliderDesc.cuboid(options.playground ? ARENA.width / 2 + 2 : PROTOCOL_ROOM.width / 2, 0.08,
        options.playground ? ARENA.depth / 2 + 2 : PROTOCOL_ROOM.depth / 2)
        .setFriction(4)
        .setRestitution(0.01)
        .setCollisionGroups(ENVIRONMENT_GROUP),
      floorBody,
    );
    this.striker = this.room ? new PhysicsStriker(this.world) : null;
    if (options.playground) this.buildPlayground(options.playground);
    this.world.step(this.eventQueue, this.physicsHooks);
    this.poses = this.initialPose();
    this.previousPoses = this.clonePoses(this.poses);
    this.supportFeet = {
      leftFoot: { ...this.poses.get("leftFoot")!.position },
      rightFoot: { ...this.poses.get("rightFoot")!.position },
    };
    this.supportFootRotations = {
      leftFoot: { ...this.poses.get("leftFoot")!.rotation },
      rightFoot: { ...this.poses.get("rightFoot")!.rotation },
    };
    this.createDynamicAssembly(this.poses, !this.playground || this.playgroundConfig?.station === "flat"
      || this.playgroundConfig?.station === "hurdles");
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.environmentColliders(), this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading, this.supportHeight());
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
    this.playground?.update(this.simulationTime);
    this.striker?.beforeStep(stepDt);
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
      ...(this.playgroundConfig ? { playground: { ...this.playgroundConfig } } : {}),
      ...(this.room ? { room: PROTOCOL_ROOM, striker: this.striker!.snapshot(),
        protocol: { strikes: this.protocolStrikes, recoveries: this.protocolRecoveries,
          message: this.protocolMessage } } : {}),
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
    if (this.room || this.disposed || !this.diagnostics().bodyInputAvailable) return null;
    return pickRegionProxies(ray, [...this.poses.values()]);
  }

  clearBodyInput(): void { this.clearGrab(); }

  requestStrike(): boolean {
    if (!this.room || this.disposed || this.paused || !this.striker) return false;
    const torso = this.poses.get("torso");
    if (!torso || !this.striker.request(torso.position, this.ragdollColliders)) return false;
    this.protocolMessage = "Essai d’arrêt en cours";
    return true;
  }

  setPlayground(config: PlaygroundConfig): void {
    if (this.disposed || this.room) return;
    this.buildPlayground(config);
    this.reset();
  }

  private buildPlayground(config: PlaygroundConfig): void {
    this.floorCollider.setShape(new RAPIER.Cuboid(ARENA.width / 2 + 2, 0.08, ARENA.depth / 2 + 2));
    this.playground?.dispose();
    this.playgroundConfig = { ...config };
    this.playground = new PhysicsPlayground(this.world, config.difficulty);
    const { position } = playgroundStation(config.station);
    // Spawn above the complete sole footprint, including on an incline. Reset
    // is the only operation that repositions the subject; all trials stay dynamic.
    let height = 0;
    for (const x of [-0.28, 0, 0.28]) for (const z of [-0.16, 0.1, 0.28]) {
      height = Math.max(height, this.playground.heightAt(position.x + x, position.z + z));
    }
    this.initialPosition = { x: position.x, y: INITIAL_ROOT.y + height, z: position.z };
  }

  private environmentColliders(): Collider[] {
    return [this.floorCollider, ...(this.playground?.colliders ?? []),
      ...(this.striker?.roomColliders ?? [])].filter(collider => collider.isEnabled());
  }

  private supportHeight(): number {
    if (!this.playground) return 0;
    const feet = [this.poses.get("leftFoot"), this.poses.get("rightFoot")];
    return feet.reduce((sum, foot) => sum + (foot
      ? this.playground!.heightAt(foot.position.x, foot.position.z, foot.position.y + 0.18) : 0), 0) / 2;
  }

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
    this.playground?.dispose();
    this.striker?.dispose();
    this.eventQueue.free();
    this.world.free();
    this.disposed = true;
  }

  reset(): void {
    if (this.disposed) return;
    this.clearGrab();
    this.clearDynamicAssembly();
    this.striker?.reset();
    this.protocolStrikes = 0;
    this.protocolRecoveries = 0;
    this.protocolMessage = null;
    this.playground?.reset();
    this.state = "upright";
    this.heading = this.initialHeading;
    this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
    this.balanceData = null;
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
    this.grounded = true;
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
    this.supportFootRotations = {
      leftFoot: { ...this.poses.get("leftFoot")!.rotation },
      rightFoot: { ...this.poses.get("rightFoot")!.rotation },
    };
    this.contactLoadPlan = { loads: [], requestedForce: ZERO, allocatedForce: ZERO, pressurePoint: ZERO };
    this.createDynamicAssembly(this.poses, !this.playground || this.playgroundConfig?.station === "flat"
      || this.playgroundConfig?.station === "hurdles");
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.environmentColliders(), this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading, this.supportHeight());
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
    const selfPenetration = this.maximumSelfPenetration();
    return {
      balance: this.balanceData ? structuredClone(this.balanceData) : null,
      bodyInputAvailable: !this.room && !this.disposed && !this.paused && !this.isRecoveryState(),
      recovery: this.isRecoveryState() ? this.recovery.diagnostics() : emptyRecoveryDiagnostics(),
      grabControl: { ...this.grabControlDiagnostics },
      physicsOwnership: "rapier-dynamic",
      jointDiagnostics,
      contactDiagnostics: {
        count: this.lastContacts.length,
        loadBearingCount: loadBearing.length,
        totalNormalForceN: loadBearing.reduce((sum, contact) => sum + contact.forceN, 0),
        supportingSegments: [...new Set(loadBearing.map((contact) => contact.segment))],
        supportLoads: this.isRecoveryState()
          ? this.recovery.diagnostics().supportLoads ?? []
          : this.lastContacts.map(contact => {
            const planned = contact.loadBearing
              ? this.contactLoadPlan.loads.find(load => load.segment === contact.segment) : undefined;
            return { segment: contact.segment, plannedForceN: planned?.plannedForce.y ?? 0,
              measuredForceN: contact.measuredForceN ?? contact.forceN };
          }),
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
      maxSelfPenetrationM: selfPenetration.depthM,
      selfPenetrationPair: selfPenetration.pair,
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
    this.lastMotorResults.clear();
    this.activateRagdoll(direction);
  }

  private isRecoveryState(): boolean {
    return this.state === "falling" || this.state === "fallen" || this.state === "recovering";
  }

  private processCommand(command: GrabCommand | null): void {
    if (this.room) { this.clearGrab(); return; }
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

  private reachableGrabTarget(grab: ActiveGrab, input: UprightPoseInput): Vec3 {
    if (grab.region !== "leftHand" && grab.region !== "rightHand"
        && grab.region !== "leftFoot" && grab.region !== "rightFoot") return grab.target;
    const grabbed = this.poses.get(grab.segment);
    if (!grabbed) return grab.target;
    const side = grab.region.startsWith("left") ? "left" : "right";
    const limb = grab.region.endsWith("Hand") ? "arm" : "leg";
    const anchor = worldPoint(grabbed.position, grabbed.rotation, grab.localAnchor);
    const previewInput: UprightPoseInput = { ...input, collisionPlanning: false };
    const preview = composeUprightPose(previewInput).poses;
    const parent = preview.get(limb === "arm" ? "torso" : "pelvis")!;
    const routed = collisionAwareLimbTarget({
      side, limb, start: anchor, requested: grab.target, parent,
      solve: (target) => composeUprightPose({
        ...previewInput,
        activeGrab: { ...grab, target },
      }).poses,
      clearance: (target, candidate) => {
        const reconstructed = limbBodyClearance(candidate, side, limb);
        const shifted = new Map<SegmentId, MutablePose>(this.poses);
        shifted.set(grab.segment, {
          ...grabbed,
          position: add(grabbed.position, sub(target, anchor)),
        });
        const grabbedPiece = segmentBodyClearance(shifted, grab.segment);
        return grabbedPiece.clearanceM < reconstructed.clearanceM ? grabbedPiece : reconstructed;
      },
    });
    return routed.target;
  }

  private updateStanding(dt: number): void {
    // Removing support is an external world change, so a resting island must
    // wake and enter genuine ballistic motion instead of remaining asleep.
    if (!this.floorCollider.isEnabled()) {
      for (const body of this.ragdollBodies.values()) body.wakeUp();
    }
    const pelvis = this.poses.get("pelvis")!;
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
      heading: this.heading,
      contacts: this.floorCollider.isEnabled() ? this.lastContacts : [],
      floorY: this.supportHeight(),
      ...(this.playground ? { surfaceHeight: (x: number, z: number) =>
        this.playground!.heightAt(x, z, pelvis.position.y + 0.2) } : {}),
    });
    this.balanceData = balance.diagnostics;
    this.reactionOffset = balance.reactionOffset;
    this.kneeFlexion = balance.kneeFlexion;
    this.supportFeet = balance.supportFeet;
    this.supportFootRotations = balance.supportFootRotations;
    this.step = balance.step;
    this.stepCount = balance.stepCount;
    this.state = balance.state;
    const excluded = new Set<SegmentId>();
    const unavailableFoot = this.step?.elapsed !== undefined && this.step.elapsed >= 0
      ? this.step.foot : this.activeGrab?.region === "leftFoot" || this.activeGrab?.region === "rightFoot"
        ? this.activeGrab.region : null;
    if (unavailableFoot) {
      const side = unavailableFoot === "leftFoot" ? "left" : "right";
      for (const id of [`${side}Ankle`, `${side}Foot`, `${side}Forefoot`] as SegmentId[]) excluded.add(id);
    }
    if (this.activeGrab?.region === "leftHand" || this.activeGrab?.region === "rightHand") {
      const side = this.activeGrab.region === "leftHand" ? "left" : "right";
      for (const id of [`${side}Forearm`, `${side}ForearmTwist`, `${side}Hand`] as SegmentId[]) excluded.add(id);
    }
    const loadContacts = this.floorCollider.isEnabled() ? this.lastContacts : [];
    const loadAcceleration = this.activeGrab || this.step || this.stepCount > 0
      ? balance.diagnostics.balanceAcceleration : ZERO;
    this.contactLoadPlan = planContactLoads(loadContacts, balance.diagnostics.centerOfMass,
      balance.diagnostics.centerOfMassVelocity,
      { x: TOTAL_MASS_KG * loadAcceleration.x, y: TOTAL_MASS_KG * 9.81,
        z: TOTAL_MASS_KG * loadAcceleration.z },
      { excluded, frictionCoefficient: 1.2,
        maxHorizontalForceN: TOTAL_MASS_KG * BALANCE_LIMITS.maxBalanceAccelerationMps2,
        maxJointTorqueNm: minimumSupportTorqueLimit(loadContacts.filter(contact => !excluded.has(contact.segment))) });

    const poseInput: UprightPoseInput = {
      rootTranslation: balance.rootTarget,
      heading: this.heading,
      kneeFlexion: this.kneeFlexion,
      reactionOffset: this.reactionOffset,
      simulationTime: this.simulationTime,
      activeGrab: this.activeGrab,
      supportFeet: this.supportFeet,
      supportFootRotations: this.supportFootRotations,
      measuredPoses: this.poses,
      step: this.step,
    };
    if (this.activeGrab) {
      const body = this.ragdollBodies.get(this.activeGrab.segment);
      if (body) {
        const reachableTarget = this.reachableGrabTarget(this.activeGrab, poseInput);
        this.grabControlDiagnostics = this.activeGrab.controller.apply(
          body,
          this.activeGrab.localAnchor,
          this.activeGrab.target,
          dt,
          reachableTarget,
        );
        this.appliedGrabForceN = length(this.grabControlDiagnostics.force);
      }
    } else {
      this.appliedGrabForceN = 0;
    }

    const target = composeUprightPose(poseInput);
    this.leanRadians = target.leanRadians;
    this.lastMotorResults = new Map(applyCoupledJointMotors(
      this.ragdollBodies,
      this.motorCommands(target.poses),
      dt,
      {
        supports: this.motorSupportConstraints(),
        passiveResistance: true,
      },
    ));
    this.world.step(this.eventQueue, this.physicsHooks);
    this.readPhysicsPoses();
    this.observeContacts(dt);
    const struck = this.striker?.afterStep(this.ragdollColliders) ?? false;
    if (struck) {
      this.protocolStrikes = this.striker!.impactId;
      this.protocolMessage = "Le sujet insiste";
      this.activateRagdoll(this.striker!.strikeDirection);
      return;
    }

    const hasFootSupport = this.supportSnapshot().planted.length > 0;
    this.unsupportedTime = hasFootSupport ? 0 : this.unsupportedTime + dt;
    const supportLossLimit = this.playground && this.simulationTime < 0.6 ? 0.5 : this.step ? 0.36 : 0.14;
    const physicalFall = this.currentTorsoLean() > 1.25
      || this.poses.get("pelvis")!.position.y < 0.56
      || this.unsupportedTime > supportLossLimit;
    if ((this.activeGrab && balance.shouldFall && this.simulationTime > 0.25) || physicalFall) {
      this.activateRagdoll(balance.fallDirection);
    }
  }

  private updateRecovery(dt: number): void {
    this.clearGrab();
    // A falling hand can travel several centimetres in one fixed step before
    // reaching the head or ribcage. Give Rapier a larger predictive contact
    // margin only while that hand is moving rapidly; normal brace contacts
    // retain the small resting skin.
    for (const side of ["left", "right"] as const) {
      const id = `${side}Hand` as const;
      const body = this.ragdollBodies.get(id)!;
      const collider = this.ragdollColliders.get(id)!;
      const speed = length(body.linvel()) + 0.09 * length(body.angvel());
      collider.setContactSkin(speed > 1.5 ? 0.02
        : this.recovery.diagnostics().phase === "settle" ? 0.01 : 0.004);
    }
    const result = this.recovery.apply(this.ragdollBodies, dt);
    // Keep the actual recovery actuator evidence. Clearing this map used to
    // report every target as the measured pose and every recovery torque as
    // zero, including while protective and rising motors were active.
    this.lastMotorResults = new Map(this.recovery.motorResults());
    for (const [id, passive] of this.recovery.passiveTorques()) {
      const definition = SEGMENT_BY_ID.get(id)!;
      const profile = definition.jointProfile!;
      const motor = this.lastMotorResults.get(id);
      const coordinates = motor?.coordinates ?? jointCoordinates(
        this.ragdollBodies.get(definition.parent!)!.rotation(),
        this.ragdollBodies.get(id)!.rotation(), profile,
      );
      const torqueWorld = add(motor?.torqueWorld ?? ZERO, passive);
      const aggregateCap = Math.hypot(...profile.axes.map(axis => axis.maxMotorTorqueNm));
      this.lastMotorResults.set(id, {
        coordinates,
        targetCoordinates: motor?.targetCoordinates ?? coordinates,
        coordinateError: motor?.coordinateError ?? ZERO,
        torqueWorld,
        saturationRatio: Math.max(motor?.saturationRatio ?? 0, length(torqueWorld) / aggregateCap),
      });
    }
    this.state = result.state;
    this.world.step(this.eventQueue, this.physicsHooks);
    this.readPhysicsPoses();
    this.observeContacts(dt);
    const struck = this.striker?.afterStep(this.ragdollColliders) ?? false;
    if (struck) {
      this.protocolStrikes = this.striker!.impactId;
      this.protocolMessage = "Le sujet insiste";
      this.recovery.interruptForImpact(this.ragdollBodies, this.striker!.strikeDirection);
    }
    this.fallingTime += dt;
    this.leanRadians = this.currentTorsoLean();
    if (!struck && this.recovery.confirmPostStep(this.ragdollBodies, dt, this.room ? 1.0 : undefined)) this.finishRecovery();
    void result;
  }

  /** State-only transition: bodies, positions, rotations, and velocities are untouched. */
  private activateRagdoll(direction: Vec3): void {
    if (this.isRecoveryState()) return;
    const pelvisVelocity = this.poses.get("pelvis")?.linearVelocity ?? ZERO;
    const fallDirection = normalize(
      add(horizontal(direction), scale(horizontal(pelvisVelocity), 0.12)),
      rotate(quatFromAxisAngle(UP, this.heading), FORWARD),
    );
    this.state = "falling";
    this.recovery.reset(this.heading, fallDirection);
    this.balanceData = null;
    this.clearGrab();
    this.fallingTime = 0;
    this.unsupportedTime = 0;
    this.uprightBlendStart.clear();
    // The standing motors already acted on this integrated step. Preserve
    // their evidence across the state-only fall transition; the next recovery
    // decision replaces it with that step's actual actuator results.
  }

  /** Recovery changes motor intent only; the continuous Rapier state remains authoritative. */
  private finishRecovery(): void {
    if (this.room) {
      this.protocolRecoveries += 1;
      this.protocolMessage = "Rectification : essai en cours";
    }
    const pelvis = this.poses.get("pelvis")!;
    const forward = horizontal(rotate(pelvis.rotation, FORWARD));
    if (length(forward) > 1e-5) this.heading = Math.atan2(forward.x, forward.z);
    this.supportFeet = {
      leftFoot: { ...this.poses.get("leftFoot")!.position },
      rightFoot: { ...this.poses.get("rightFoot")!.position },
    };
    this.supportFootRotations = {
      leftFoot: { ...this.poses.get("leftFoot")!.rotation },
      rightFoot: { ...this.poses.get("rightFoot")!.rotation },
    };
    this.reactionOffset = ZERO;
    this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
    this.step = null;
    this.state = "upright";
    this.grounded = true;
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
    this.balance.reset(this.poses, this.heading, this.supportHeight());
    this.balanceData = null;
  }

  /** Use only the measured stance soles; a moving swing foot remains unconstrained. */
  private motorSupportConstraints(): ArticulatedSupportConstraint[] {
    if (!this.step || this.step.elapsed < 0) return [];
    const swingSide = this.step.foot === "leftFoot" ? "left" : "right";
    const supportBySide = new Map<"left" | "right", typeof this.lastContacts[number]>();
    for (const contact of this.lastContacts) {
      const definition = SEGMENT_BY_ID.get(contact.segment);
      if (!contact.loadBearing || !definition?.side || definition.side === swingSide
        || (definition.role !== "hindfoot" && definition.role !== "forefoot")) {
        continue;
      }
      const existing = supportBySide.get(definition.side);
      if (!existing || contact.forceN > existing.forceN) {
        supportBySide.set(definition.side, contact);
      }
    }
    // One sticking point per loaded side captures the translational ground
    // reaction without pretending that unilateral contacts weld every sole
    // and toe rotational degree of freedom to the floor.
    return [...supportBySide.values()].map((contact) => ({
      segment: contact.segment,
      points: [{ ...contact.point }],
      // Tangential sticking supplies the horizontal reaction seen by the
      // floating-root motor solve. The unilateral normal is deliberately left
      // to Rapier so prediction cannot turn a sole into a vertical weld.
      directions: [{ x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }],
    }));
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
    if (!child || !jointDefinition.jointProfile || !this.contactLoadPlan.loads.length) return ZERO;
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
        { x: 0, y: candidate.massKg * 9.81, z: 0 },
      ));
    }
    for (const support of this.contactLoadPlan.loads) {
      let ancestor: SegmentId | null = support.segment;
      let belowJoint = false;
      while (ancestor) {
        if (ancestor === jointDefinition.id) { belowJoint = true; break; }
        ancestor = SEGMENT_BY_ID.get(ancestor)?.parent ?? null;
      }
      if (!belowJoint) continue;
      torque = add(torque, cross(
        sub(support.point, jointWorld),
        scale(support.plannedForce, -1),
      ));
    }
    return torque;
  }

  private createDynamicAssembly(
    seed: ReadonlyMap<SegmentId, MutablePose>,
    initiallySleeping = false,
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
          .setLinearDamping(0.38)
          .setAngularDamping(0.52)
          .setCanSleep(true)
          .setSleeping(initiallySleeping)
          .setCcdEnabled(true)
          .setSoftCcdPrediction(
            definition.role === "ankle" || definition.role === "hindfoot" || definition.role === "forefoot"
              ? 0 : 0.12,
          )
          .setAdditionalSolverIterations(5),
      );
      const collider = this.world.createCollider(
        convexCollider(definition)
          .setMass(definition.massKg)
          .setFriction(
            definition.role === "hindfoot" || definition.role === "forefoot" ? 4 : 1.2,
          )
          .setRestitution(0.02)
          .setContactSkin(0.004)
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
      const data = xHinge
        ? RAPIER.JointData.revoluteWithAxes(
          profile.parentFrame.anchor,
          profile.childFrame.anchor,
          localAxis(profile),
          rotate(profile.childFrame.rotation, { x: 1, y: 0, z: 0 }),
        )
        : RAPIER.JointData.spherical(profile.parentFrame.anchor, profile.childFrame.anchor);
      const joint = this.world.createImpulseJoint(data, parent, child, false);
      joint.setLocalFrame1(profile.parentFrame.anchor, profile.parentFrame.rotation);
      joint.setLocalFrame2(profile.childFrame.anchor, profile.childFrame.rotation);
      joint.setContactsEnabled(false);
      if (xHinge) {
        adapter.constrain(joint, profile);
      } else {
        for (const coordinate of COORDINATES) {
          const specified = profile.axes.find((axis) => axis.coordinate === coordinate);
          adapter.constrainAxis(joint, specified ?? {
            coordinate,
            minRadians: 0,
            maxRadians: 0,
            passiveStiffnessNmPerRad: 0,
            dampingNmsPerRad: 0,
            maxMotorTorqueNm: 0,
          });
        }
      }
      this.ragdollJoints.push(joint);
      this.jointsByChild.set(definition.id, joint);
    }
    if (initiallySleeping) {
      // Build floor/contact manifolds at initialization without producing a
      // visible settling step, then park the solved dynamic island.  The tiny
      // timestep keeps the initialized pose within replay precision.
      const timestep = this.world.timestep;
      this.world.timestep = 1e-6;
      this.world.step(this.eventQueue, this.physicsHooks);
      this.world.timestep = timestep;
      for (const body of this.ragdollBodies.values()) body.sleep();
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
    const footRotations: Partial<Record<"leftFoot" | "rightFoot", Quat>> = {};
    if (this.playground) for (const foot of ["leftFoot", "rightFoot"] as const) {
      const center = feet[foot];
      const surface = this.playground.surfaceAt(center.x, center.z);
      const rotation = quatMultiply(quatFromTo(UP, surface.normal), heading);
      const geometry = SEGMENT_BY_ID.get(foot)!.geometry;
      let height = surface.height;
      // Seat the sole on the measured slope and clear every corner of rough
      // ground at reset. Runtime terrain adaptation remains motor-driven.
      for (const vertex of geometry.vertices) {
        const offset = rotate(rotation, vertex);
        height = Math.max(height, this.playground.heightAt(center.x + offset.x, center.z + offset.z) - offset.y);
      }
      feet[foot] = { x: center.x, y: height + 0.001, z: center.z };
      footRotations[foot] = rotation;
    }
    return composeUprightPose({
      rootTranslation: this.initialPosition,
      heading: this.initialHeading,
      kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
      reactionOffset: ZERO,
      simulationTime: 0,
      activeGrab: null,
      supportFeet: feet,
      supportFootRotations: footRotations,
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
      this.environmentColliders(),
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
        motorTorqueNm: length(motor?.torqueWorld ?? ZERO),
        motorSaturationRatio: motor?.saturationRatio ?? 0,
      });
    }
    return result;
  }

  private clonePoses(source: ReadonlyMap<SegmentId, MutablePose>): Map<SegmentId, MutablePose> {
    return new Map([...source].map(([id, pose]) => [id, {
      id,
      position: { ...pose.position },
      rotation: { ...pose.rotation },
      linearVelocity: { ...pose.linearVelocity },
      angularVelocity: { ...pose.angularVelocity },
    }]));
  }

  private supportSnapshot(): SupportState {
    if (this.disposed || !this.floorCollider.isEnabled()) {
      return { planted: [], swingFoot: null, stepProgress: 0, grounded: false };
    }
    const planted = (["left", "right"] as const).flatMap((side) => {
      const foot = `${side}Foot` as const;
      if ((this.step?.foot === foot && this.step.elapsed >= 0) || this.activeGrab?.region === foot) return [];
      const pose = this.poses.get(foot);
      const definition = SEGMENT_BY_ID.get(foot);
      const floorY = pose && this.playground
        ? this.playground.heightAt(pose.position.x, pose.position.z, pose.position.y + 0.18) : 0;
      const soleNearFloor = Boolean(pose && definition
        && pose.position.y <= floorY + 0.10
        && lowestWorldPoint(definition.geometry, pose.position, pose.rotation).y <= floorY + 0.025
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
      const surfaces: Collider[] = this.playground ? [] : [this.floorCollider];
      if (this.playground) this.world.contactPairsWith(collider, surface => {
        if (!this.colliderSegments.has(surface.handle) && surface.isEnabled()) surfaces.push(surface);
      });
      for (const surface of surfaces) {
        const contact = surface.contactCollider(collider, 0);
        if (contact) penetration = Math.max(penetration, -contact.distance);
      }
    }
    return penetration;
  }

  private maximumSelfPenetration(): {
    depthM: number;
    pair: readonly [SegmentId, SegmentId] | null;
  } {
    if (this.disposed) return { depthM: 0, pair: null };
    const colliders = [...this.ragdollColliders];
    let depthM = 0;
    let pair: readonly [SegmentId, SegmentId] | null = null;
    for (let first = 0; first < colliders.length; first += 1) {
      const [firstId, firstCollider] = colliders[first];
      const firstCenter = firstCollider.translation();
      for (let second = first + 1; second < colliders.length; second += 1) {
        const [secondId, secondCollider] = colliders[second];
        if (this.excludedPairs.has(pairKey(firstId, secondId))) continue;
        const secondCenter = secondCollider.translation();
        const radius = SEGMENT_BOUNDING_RADII.get(firstId)! + SEGMENT_BOUNDING_RADII.get(secondId)!;
        if (Math.hypot(
          firstCenter.x - secondCenter.x,
          firstCenter.y - secondCenter.y,
          firstCenter.z - secondCenter.z,
        ) > radius) continue;
        const contact = firstCollider.contactCollider(secondCollider, 0);
        if (contact && -contact.distance > depthM) {
          depthM = -contact.distance;
          pair = [firstId, secondId];
        }
      }
    }
    return { depthM, pair };
  }

  private currentTorsoLean(): number {
    const torso = this.poses.get("torso");
    if (!torso) return 0;
    return Math.acos(clamp(dot(normalize(rotate(torso.rotation, UP)), UP), -1, 1));
  }
}

export interface CharacterInitialOptions { heading?: number; position?: Vec3; playground?: PlaygroundConfig; room?: boolean }

export async function createEmbodiedCharacter(
  initialRenderer: RendererMode = "canvas2d",
  options: CharacterInitialOptions = {},
): Promise<CharacterController> {
  await ensureRapier();
  return new EmbodiedCharacter(initialRenderer, options);
}
