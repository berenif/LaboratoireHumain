import { configureHumanoidWorld } from "./physics-settings";
import { ContactStandingController } from "./ContactStandingController";
import { ContactStandingControllerH79 } from "./ContactStandingControllerH79";
import { ContactStandingControllerH80 } from "./ContactStandingControllerH80";
import { contactJointTorque } from "./contact-joint-torque";
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
import { integrateConvexMass } from "../core/geometry-mass";
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
import { copyStandingContacts, standingChainDiagnostics } from "./standing-chain-diagnostics";
import { NativeJointMotors } from "./native-joint-motors";
import { COORDINATED_STANDING, CoordinatedStandingController } from "./CoordinatedStandingController";
import { ImplicitStandingController } from "./ImplicitStandingController";
import { LegTargetDynamics } from "./leg-target-dynamics";
import {
  clampJointCoordinates,
  jointCoordinates,
  jointCoordinateTargetError,
  jointFrameAxesWorld,
  jointLimitError,
  jointLimitErrorMagnitude,
  jointRotationFromCoordinates,
} from "./joint-coordinates";
import {
  type JointMotorCommand,
  type JointMotorResult,
} from "./joint-motors";
import {
  add,
  angularVelocity,
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
const RIGHT: Vec3 = { x: 1, y: 0, z: 0 };
const INITIAL_ROOT: Vec3 = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
// Character colliders see both character and environment. The hook removes only
// the explicit anatomical exclusions, leaving nonadjacent self-collision on.
const CHARACTER_GROUP = (0x0001 << 16) | 0x0003;
const ENVIRONMENT_GROUP = (0x0002 << 16) | 0x0001;
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
  const vertices = flattenGeometryVertices(definition.geometry);
  const indices = flattenGeometryIndices(definition.geometry);
  const descriptor = RAPIER.ColliderDesc.convexMesh(
    vertices,
    indices,
  );
  if (!descriptor) throw new Error(`INVALID_CONVEX_GEOMETRY:${definition.id}`);
  const mass = integrateConvexMass(Array.from(vertices), Array.from(indices), definition.massKg);
  return descriptor.setMassProperties(mass.mass, mass.centerOfMass, mass.principalInertia, mass.principalFrame);
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
  private coordinatedStanding: CoordinatedStandingController | ImplicitStandingController | null = null;
  private readonly standingCandidateOptions: CharacterInitialOptions["standingCandidate"];
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
  private recovery = this.createRecoveryController();
  private recoveryRollArmMotorsActive = false;
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
  private balanceFall: DiagnosticsSnapshot["balanceFall"] = null;
  private grounded = false;
  private uprightBlendTime = Number.POSITIVE_INFINITY;
  private readonly uprightBlendStart = new Map<SegmentId, Vec3>();
  private lastMotorResults = new Map<SegmentId, JointMotorResult>();
  private standingTargets: ReadonlyMap<SegmentId, MutablePose> = new Map();
  private standingCommands: readonly JointMotorCommand[] = [];
  private swingCommandFoot: SegmentId | null = null;
  private readonly swingCommandCoordinates = new Map<SegmentId, Vec3>();
  private readonly legTargetDynamics = new LegTargetDynamics();
  private standingMotorSampleTimeS = 0;
  private standingMotorPelvis: Pick<SegmentPose, "id" | "position" | "rotation"> | null = null;
  private lastContacts = emptyRecoveryDiagnostics().contacts;
  private readonly runtimeErrors: string[] = [];

  constructor(renderer: RendererMode, options: CharacterInitialOptions = {}) {
    if (options.standingCandidate && (options.room || options.playground)) throw new Error("Standing candidates require the declared flat-floor solver configuration");
    this.standingCandidateOptions = options.standingCandidate;
    this.renderer = renderer;
    this.room = Boolean(options.room);
    this.initialHeading = options.heading ?? 0;
    this.heading = this.initialHeading;
    this.initialPosition = options.position ?? INITIAL_ROOT;
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    configureHumanoidWorld(this.world);
    // The room's fixed striker produces a sharp Rapier contact. Extra solver
    // iterations resolve that impulse across the bounded shoulder joints.
    if (this.room) this.world.numSolverIterations = 80;
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
    this.createDynamicAssembly(this.poses);
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.environmentColliders(), this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading, this.supportHeight());
    this.initializeStandingCandidate();
  }

  private initializeStandingCandidate(): void {
    const options = this.standingCandidateOptions;
    this.balance.useCoordinatedSupport = ["h78-v1", "h79-v1", "h80-v1"].includes(options?.controllerId ?? "legacy");
    this.coordinatedStanding = !options || options.controllerId === "legacy" ? null
      : options.controllerId === "h74-v1"
        ? new CoordinatedStandingController(this.poses, this.heading, options.forwardOffsetM ?? 0)
        : options.controllerId === "h78-v1"
          ? new ContactStandingController(this.poses, this.heading, options.forwardOffsetM ?? 0)
        : options.controllerId === "h79-v1"
          ? new ContactStandingControllerH79(this.poses, this.heading, options.forwardOffsetM ?? 0)
        : options.controllerId === "h80-v1"
          ? new ContactStandingControllerH80(this.poses, this.heading, options.forwardOffsetM ?? 0)
        : new ImplicitStandingController(this.poses, this.heading, options.forwardOffsetM ?? 0);
  }

  private createRecoveryController(): DynamicRecovery {
    return new DynamicRecovery(commands => this.nativeMotors.apply(
      this.ragdollBodies, this.jointsByChild, commands, { passiveResistance: true },
    ));
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
    this.standingTargets = new Map();
    this.standingCommands = [];
    this.swingCommandFoot = null;
    this.swingCommandCoordinates.clear();
    this.legTargetDynamics.reset();
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
    this.balanceFall = null;
    this.grounded = false;
    this.lastContacts = [];
    this.uprightBlendTime = Number.POSITIVE_INFINITY;
    this.uprightBlendStart.clear();
    this.lastMotorResults.clear();
    this.grabControlDiagnostics = emptyGrabDiagnostics();
    this.runtimeErrors.length = 0;
    this.recovery = this.createRecoveryController();
    this.recoveryRollArmMotorsActive = false;
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
    this.createDynamicAssembly(this.poses);
    this.readPhysicsPoses();
    this.recovery.observe(this.world, this.environmentColliders(), this.ragdollColliders, this.ragdollBodies, 1 / 60);
    this.lastContacts = this.recovery.diagnostics().contacts.map((contact) => ({ ...contact }));
    this.balance.reset(this.poses, this.heading, this.supportHeight());
    this.initializeStandingCandidate();
  }

  diagnostics(): DiagnosticsSnapshot {
    const root = this.poses.get("pelvis")?.position ?? INITIAL_ROOT;
    const finite = [...this.poses.values()].every((pose) =>
      finiteVec(pose.position) && finiteQuat(pose.rotation)
      && finiteVec(pose.linearVelocity) && finiteVec(pose.angularVelocity)
      && (pose.massKg === undefined || Number.isFinite(pose.massKg))
      && (pose.centerOfMass === undefined || finiteVec(pose.centerOfMass))
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
      ...(this.coordinatedStanding ? { coordinatedStanding: this.coordinatedStanding.diagnostics() } : {}),
      standingChain: this.isRecoveryState() || this.standingTargets.size === 0 ? null
        : standingChainDiagnostics(this.poses, this.standingTargets, this.standingCommands,
          this.step, this.heading, this.standingMotorSampleTimeS, this.simulationTime, this.lastContacts, this.standingMotorPelvis),
      jointDiagnostics,
      contactDiagnostics: {
        standingPlan: this.isRecoveryState() ? undefined : {
          requestedForce: { ...this.contactLoadPlan.requestedForce },
          allocatedForce: { ...this.contactLoadPlan.allocatedForce },
          pressurePoint: { ...this.contactLoadPlan.pressurePoint },
          pressureFeasible: this.contactLoadPlan.pressureFeasible,
          pressureForceResidualNm: this.contactLoadPlan.pressureForceResidualNm,
        },
        contacts: copyStandingContacts(this.lastContacts),
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
      balanceFall: this.balanceFall ? { ...this.balanceFall, reasons: [...this.balanceFall.reasons] } : null,
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
    // Apply the bounded physical interaction once, then let balance respond to
    // that same force and control target. Collision routing uses the current
    // measured body before that impulse is applied; it never reconstructs or
    // teleports the live assembly.
    if (this.activeGrab) {
      const body = this.ragdollBodies.get(this.activeGrab.segment);
      if (body) {
        const previewInput: UprightPoseInput = {
          rootTranslation: pelvis.position,
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
        // Hands are intentionally allowed to load the trunk through Rapier's
        // real self-contact manifold. Routing them around the chest here made
        // the grab spring and the motor target evade the very contact the
        // physical interaction is meant to exercise. Feet retain the bounded
        // collision-aware route used by playground manipulation.
        const reachableTarget = this.activeGrab.region === "leftFoot"
          || this.activeGrab.region === "rightFoot"
          ? this.reachableGrabTarget(this.activeGrab, previewInput)
          : this.activeGrab.target;
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

    // Keep the planning heading explicit. Rapier may rotate the pelvis, but
    // that motion must not silently rotate an already committed world target.
    const balanceGrab = this.activeGrab && this.grabControlDiagnostics.active ? {
      ...this.activeGrab,
      target: { ...this.grabControlDiagnostics.controlTarget },
      targetVelocity: { ...this.grabControlDiagnostics.targetVelocity },
    } : this.activeGrab;
    const measuredContacts = this.floorCollider.isEnabled() ? this.lastContacts : [];
    const balance = this.balance.update({
      dt,
      poses: this.poses,
      rootPosition: pelvis.position,
      activeGrab: balanceGrab,
      appliedGrabForce: this.activeGrab && this.grabControlDiagnostics.active
        ? this.grabControlDiagnostics.force : ZERO,
      heading: this.heading,
      contacts: measuredContacts,
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

    // Ground-reaction feedforward is authorized only by current measured sole
    // contacts. The measured report retains every real patch, while the motor
    // plan omits the actively travelling swing side so it can unload. Once the
    // arc ends, measured touchdown is eligible again. This is bounded motor
    // intent; actual measured loads alone authorize lift and touchdown.
    const activeSwingSide = this.step && this.step.elapsed < this.step.duration
      && this.step.elapsed >= 0
      ? (this.step.foot === "leftFoot" ? "left" : "right") : null;
    const loadContacts = measuredContacts.filter((contact) => {
      const definition = SEGMENT_BY_ID.get(contact.segment);
      const role = definition?.role;
      return contact.loadBearing
        && contact.forceN >= BALANCE_LIMITS.minimumContactForceN
        && contact.normalY >= 0.65
        && definition?.side !== activeSwingSide
        && (role === "hindfoot" || role === "forefoot");
    });
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
      measuredLegFrame: true,
      ...(this.activeGrab?.region === "leftHand" || this.activeGrab?.region === "rightHand"
        ? { collisionPlanning: false as const } : {}),
      step: this.step,
    };
    const target = composeUprightPose(poseInput);
    const totalMassKg = [...this.ragdollBodies.values()]
      .reduce((sum, body) => sum + body.mass(), 0);
    const bodyWeightN = totalMassKg * 9.81;
    const measuredLoadN = loadContacts.reduce((sum, contact) =>
      sum + contact.forceN * contact.normalY, 0);
    const supportedLoadN = Math.min(measuredLoadN, bodyWeightN);
    const supportedFraction = bodyWeightN > 0 ? supportedLoadN / bodyWeightN : 0;
    // Measured-frame legs keep the soles fixed; this bounded task supplies
    // horizontal restoration even after release or an abandoned transfer.
    const referencePelvis = this.coordinatedStanding?.active && !this.activeGrab && !this.step
      ? this.coordinatedStanding.reference.get("pelvis") : null;
    const loadAcceleration = referencePelvis ? clampLength(add(
      scale(sub(referencePelvis.position, pelvis.position), COORDINATED_STANDING.pelvisPositionKpPerS2),
      scale(pelvis.linearVelocity, -COORDINATED_STANDING.pelvisPositionKdPerS)), BALANCE_LIMITS.maxBalanceAccelerationMps2)
      : balance.diagnostics.balanceAcceleration;
    const heightTarget = referencePelvis?.position.y ?? target.poses.get("pelvis")!.position.y;
    const heightOmega = Math.sqrt(9.81 / Math.max(0.4,
      balance.diagnostics.centerOfMass.y - this.supportHeight()));
    // Critically damped height intent at the body's pendulum frequency. This
    // is a contact-wrench request, realized solely by the existing capped joint
    // motors on qualified measured contacts. The present reaction is feedback,
    // not a capacity limit: scaling by a falling reaction would suppress the
    // restoring request precisely when the body needs to regain support.
    const verticalAcceleration = heightOmega * heightOmega * (heightTarget - pelvis.position.y)
      - 2 * heightOmega * pelvis.linearVelocity.y;
    const verticalIntent = clamp(totalMassKg * (9.81 + verticalAcceleration),
      0, bodyWeightN * 1.35);
    const frozenH74 = this.standingCandidateOptions?.controllerId === "h74-v1";
    const coordinatedH77 = this.coordinatedStanding instanceof ImplicitStandingController
      && this.coordinatedStanding.active && !this.activeGrab && !this.step;
    this.contactLoadPlan = planContactLoads(loadContacts, balance.diagnostics.centerOfMass,
      balance.diagnostics.centerOfMassVelocity,
      { x: totalMassKg * loadAcceleration.x * supportedFraction, y: verticalIntent,
        z: totalMassKg * loadAcceleration.z * supportedFraction },
      { frictionCoefficient: 1.2,
        projectMeasuredPressure: true,
        // During touchdown qualification, the solver has already established
        // the only admissible real load split. Re-projecting that first landing
        // frame entirely onto the old sole unloads and rebounds the new contact
        // before it can earn the required 0.10 s persistence. Retain half of
        // those measured shares for this bounded loading phase; eligibility,
        // total force, friction, joint torque and motor ceilings remain unchanged.
        minimumMeasuredShareFraction:
          this.step && this.step.elapsed >= this.step.duration ? 0.5 : 0,
        // The loaded hip/torso motors already provide a bounded support
        // moment under their unchanged axis caps. Preserve the requested
        // braking direction instead of replacing it with the opposite force
        // when the instantaneous COM lies outside the measured pressure hull.
        // H74 retains its frozen pressure-feasible allocator. H77 follows that
        // constraint; the legacy rescue path may use its bounded joint moment
        // while a step is active.
        allowSupportMoment: Boolean(this.step) && !frozenH74 && !coordinatedH77,
        maxHorizontalForceN: totalMassKg * BALANCE_LIMITS.maxBalanceAccelerationMps2 * supportedFraction,
        maxJointTorqueNm: minimumSupportTorqueLimit(loadContacts) });

    this.leanRadians = target.leanRadians;
    this.standingTargets = target.poses;
    this.standingCommands = this.motorCommands(target.poses);
    if (this.coordinatedStanding) {
      const contactAuthority = this.coordinatedStanding instanceof ContactStandingController
        && !this.coordinatedStanding.transitioning;
      const isFrozenH74 = this.standingCandidateOptions?.controllerId === "h74-v1";
      const commands = this.coordinatedStanding.update({ poses: this.poses, nominal: this.standingCommands,
        gravity: new Map(SEGMENTS.filter(d => d.jointProfile).map(d => [d.id, this.gravityCompensation(d)])),
        contacts: measuredContacts, tick: this.fixedSteps, dt,
        quiet: !this.activeGrab && !this.step && (!isFrozenH74 || this.stepCount === 0)
          && ["upright", "reacting"].includes(this.state),
        grabActive: Boolean(this.activeGrab), stepActive: Boolean(this.step),
        touchdownQualified: balance.diagnostics.touchdownQualified,
        supportMarginM: balance.diagnostics.supportMarginM,
        triggerReason: balance.diagnostics.triggerReason,
        ...(this.coordinatedStanding instanceof ContactStandingController ? { allocation: {
          contacts: measuredContacts, centerOfMass: balance.diagnostics.centerOfMass,
          requestedForce: { x: totalMassKg * loadAcceleration.x * supportedFraction,
            y: verticalIntent, z: totalMassKg * loadAcceleration.z * supportedFraction },
          requestedMoment: ZERO, weightN: bodyWeightN,
          momentLengthM: Math.max(0.4, balance.diagnostics.centerOfMass.y - this.supportHeight()),
          maxHorizontalForceN: Math.min(totalMassKg * BALANCE_LIMITS.maxBalanceAccelerationMps2 * supportedFraction,
            minimumSupportTorqueLimit(loadContacts) / Math.max(0.3, balance.diagnostics.centerOfMass.y - this.supportHeight())),
          supportState: this.balance.supportState,
        } } : {}) });
      if (commands) {
        this.standingCommands = commands;
        if (this.coordinatedStanding.active) this.standingTargets = this.coordinatedStanding.reference;
      }
      if (!commands && contactAuthority && this.coordinatedStanding instanceof ContactStandingController && this.coordinatedStanding.transitioning) {
        // Complete the original bounded fallback in this same update.
        this.standingCommands = this.standingCommands.map(command => {
          const definition = SEGMENT_BY_ID.get(command.id)!, child = this.ragdollBodies.get(command.id)!;
          const anchor = worldPoint(child.translation(), child.rotation(), definition.jointProfile!.childFrame.anchor);
          const contactTorque = this.contactLoadPlan.loads.reduce((sum, load) => add(sum,
            contactJointTorque(command.id, anchor, load.segment, load.point, load.plannedForce)), ZERO);
          return { ...command, feedforwardWorld: add(command.feedforwardWorld ?? ZERO, contactTorque) };
        });
      }
      if (commands && this.coordinatedStanding instanceof ContactStandingController) {
        this.balance.supportState = structuredClone(this.coordinatedStanding.supportState);
        const allocation = this.coordinatedStanding.diagnostics().allocation;
        if (allocation?.status === "allocated") this.contactLoadPlan = {
          ...this.contactLoadPlan, loads: allocation.loads, allocatedForce: allocation.allocatedForce,
          pressureForceResidualNm: length(allocation.momentResidual),
        };
      }
      // null means a latched, same-update handoff to existing bounded balance
      // commands and the original fall/recovery guards, never a standing pass.
    }
    this.standingMotorSampleTimeS = this.simulationTime - dt;
    this.standingMotorPelvis = { id: "pelvis", position: { ...pelvis.position }, rotation: { ...pelvis.rotation } };
    this.lastMotorResults = this.nativeMotors.apply(
      this.ragdollBodies, this.jointsByChild, this.standingCommands,
    );
    if (this.coordinatedStanding instanceof ImplicitStandingController) {
      this.coordinatedStanding.observeNativeResults(this.lastMotorResults);
    }
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
      this.balanceFall = { timeS: this.simulationTime,
        reasons: [
          ...(this.currentTorsoLean() > 1.25 ? ["torso-lean"] : []),
          ...(this.poses.get("pelvis")!.position.y < 0.56 ? ["pelvis-height"] : []),
          ...(this.unsupportedTime > supportLossLimit ? ["support-loss"] : []),
          ...(this.activeGrab && balance.shouldFall && this.simulationTime > 0.25 ? ["capture-instability"] : []),
        ], pelvisHeightM: this.poses.get("pelvis")!.position.y,
        torsoLeanRadians: this.currentTorsoLean(), unsupportedTimeS: this.unsupportedTime,
        supportLossLimitS: supportLossLimit };
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
    const rollArmCommands = this.recovery.rollArmMotorCommands();
    if (rollArmCommands.length) {
      const nativeResults = this.nativeMotors.apply(
        this.ragdollBodies, this.jointsByChild, rollArmCommands,
      );
      this.recovery.recordNativeRollArmResults(nativeResults);
      this.recoveryRollArmMotorsActive = true;
    } else if (this.recoveryRollArmMotorsActive) {
      this.nativeMotors.disable(this.jointsByChild);
      this.recoveryRollArmMotorsActive = false;
    }
    // Keep recovery actuator evidence; native results identify themselves as
    // requests because Rapier does not expose the delivered motor impulse.
    // Clearing this map would report measured poses and zero recovery torque.
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
    // A protective-arm reversal and floor impact can couple the elbow to the
    // full trunk in one 60 Hz step. Sixty-four internal passes keep that
    // contact/joint solve within the 10 mm anchor bound; restore standing's budget
    // immediately afterward so ordinary balance cost and motion are unchanged.
    const standingPgsIterations = this.world.numInternalPgsIterations;
    this.world.numInternalPgsIterations = Math.max(standingPgsIterations, 64);
    try {
      this.world.step(this.eventQueue, this.physicsHooks);
    } finally {
      this.world.numInternalPgsIterations = standingPgsIterations;
    }
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
    this.coordinatedStanding?.handoff("invalid-state");
    const pelvisVelocity = this.poses.get("pelvis")?.linearVelocity ?? ZERO;
    const fallDirection = normalize(
      add(horizontal(direction), scale(horizontal(pelvisVelocity), 0.12)),
      rotate(quatFromAxisAngle(UP, this.heading), FORWARD),
    );
    this.nativeMotors.disable(this.jointsByChild);
    this.recoveryRollArmMotorsActive = false;
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
    if (this.recoveryRollArmMotorsActive) {
      this.nativeMotors.disable(this.jointsByChild);
      this.recoveryRollArmMotorsActive = false;
    }
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
    this.balance.reset(this.poses, this.heading, this.supportHeight());
    this.balanceData = null;
  }

  private motorCommands(targets: ReadonlyMap<SegmentId, MutablePose>): JointMotorCommand[] {
    const swingFoot = this.step && this.step.elapsed >= 0 ? this.step.foot : null;
    const movingSide = swingFoot ? swingFoot === "leftFoot" ? "left" : "right" : null;
    if (swingFoot !== this.swingCommandFoot) this.swingCommandCoordinates.clear();
    this.swingCommandFoot = swingFoot;
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
      // Every dynamic parent can lag the commanded world pose. Express the
      // child target against its measured parent so local motor coordinates
      // still aim for the solved world orientation instead of compounding the
      // ancestor's current tracking error down the chain.
      const legJoint = ["thigh", "shin", "ankle", "hindfoot", "forefoot"].includes(definition.role);
      // A solved leg is one kinematic chain. Re-expressing every distal target
      // against a different measured parent changes its FK endpoint. Only the
      // hip uses the measured pelvis that was supplied to the leg solver.
      const parentRotation = legJoint
        ? definition.parent === "pelvis"
          ? this.poses.get("pelvis")!.rotation : parentTarget.rotation
        : this.poses.get(definition.parent)?.rotation ?? parentTarget.rotation;
      const relative = quatMultiply(quatInverse(parentRotation), childTarget.rotation);
      const rawDesired = { ...jointCoordinates({ x: 0, y: 0, z: 0, w: 1 }, relative,
        definition.jointProfile) };
      const desired = clampJointCoordinates(rawDesired, definition.jointProfile);
      const start = this.uprightBlendStart.get(definition.id);
      const coordinates = start ? {
        x: start.x + (desired.x - start.x) * blend,
        y: start.y + (desired.y - start.y) * blend,
        z: start.z + (desired.z - start.z) * blend,
      } : desired;
      const gains = roleGains(definition);
      let feedforwardWorld = this.gravityCompensation(definition);
      if (legJoint && definition.side === movingSide) {
        const previous = this.swingCommandCoordinates.get(definition.id);
        if (previous) {
          // Damping tracks the moving trajectory's velocity instead of
          // opposing it. This intent shares the same native torque ceiling
          // with posture and gravity; it adds no force or actuator capacity.
          const delta = jointCoordinateTargetError(previous, coordinates, definition.jointProfile);
          const basis = jointFrameAxesWorld(this.poses.get(definition.parent)!.rotation, definition.jointProfile);
          for (const axis of definition.jointProfile.axes) {
            const motionTorque = clamp(gains.damping * delta[axis.coordinate] / this.world.timestep,
              -axis.maxMotorTorqueNm, axis.maxMotorTorqueNm);
            feedforwardWorld = add(feedforwardWorld, scale(basis[axis.coordinate], motionTorque));
          }
        }
        this.swingCommandCoordinates.set(definition.id, { ...coordinates });
      }
      if (definition.role === "thigh" && this.contactLoadPlan.loads.length) {
        const pelvis = this.poses.get("pelvis")!;
        const measuredTotal = this.contactLoadPlan.loads.reduce((sum, load) => sum + load.measuredForceN, 0);
        const measuredSide = this.contactLoadPlan.loads.reduce((sum, load) => sum
          + (SEGMENT_BY_ID.get(load.segment)?.side === definition.side ? load.measuredForceN : 0), 0);
        // Restore the virtual pelvis posture through the loaded hips, while
        // measured-frame leg IK keeps each sole's world constraint intact.
        // The equal/opposite parent reaction supplies the restoring moment;
        // this is still native joint actuation under the same torque ceilings.
        const postureError = angularVelocity(pelvis.rotation, targets.get("pelvis")!.rotation, 1);
        const parentTorque = clampLength(add(scale(postureError, 250),
          scale(pelvis.angularVelocity, -30)), 100);
        const share = measuredTotal > 0 ? measuredSide / measuredTotal : 0;
        feedforwardWorld = add(feedforwardWorld, scale(parentTorque, -share));
      }
      commands.push({
        id: definition.id,
        targetLocalRotation: jointRotationFromCoordinates(coordinates, definition.jointProfile),
        stiffness: gains.stiffness,
        damping: gains.damping,
        strengthScale: strength,
        effortScale: roleEffort(definition),
        feedforwardWorld,
      });
    }
    // Differentiate the final, joint-limited command chain in the same measured
    // pelvis frame used by actuation. Native motors combine this inertial intent
    // with posture/gravity and retain their existing per-axis torque ceilings.
    const inertialTorques = this.legTargetDynamics.sample(
      this.poses.get("pelvis")!, commands, this.ragdollBodies, movingSide, this.world.timestep,
    );
    return commands.map((command) => {
      const inertia = inertialTorques.get(command.id);
      return inertia ? { ...command, feedforwardWorld: add(command.feedforwardWorld ?? ZERO, inertia) } : command;
    });
  }

  /**
   * Cancel descendant gravity and route the requested horizontal ground wrench
   * through the loaded foot chains.  The chosen pressure point is the COM for
   * static equilibrium and shifts by h/g*a while balance is actively correcting.
   */
  private gravityCompensation(jointDefinition: SegmentDefinition): Vec3 {
    const coordinatedContact = this.coordinatedStanding instanceof ContactStandingController
      && !this.coordinatedStanding.transitioning;
    const child = this.ragdollBodies.get(jointDefinition.id);
    if (!child || !jointDefinition.jointProfile
      || (!this.contactLoadPlan.loads.length && !coordinatedContact)) return ZERO;
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
    if (!coordinatedContact) for (const support of this.contactLoadPlan.loads) {
      torque = add(torque, contactJointTorque(jointDefinition.id, jointWorld,
        support.segment, support.point, support.plannedForce));
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
          .setSoftCcdPrediction(
            definition.role === "ankle" || definition.role === "hindfoot" || definition.role === "forefoot"
              ? 0 : 0.12,
          )
          .setAdditionalSolverIterations(5),
      );
      const collider = this.world.createCollider(
        convexCollider(definition)
          .setFriction(
            definition.role === "hindfoot" || definition.role === "forefoot" ? 4
              // Min combining makes the body material the limiting grip on
              // the floor. Give get-up supports the recovery planner's 1.2
              // coefficient so planted limbs can push without skating.
              : definition.role === "hand" || definition.role === "forearm"
                || definition.role === "forearm-twist" || definition.role === "shin"
                || definition.role === "ankle" ? 1.2 : 0.45,
          )
          .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
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
    if (this.disposed || !this.floorCollider.isEnabled()) {
      return { planted: [], swingFoot: null, stepProgress: 0, grounded: false };
    }
    const planted = (["left", "right"] as const).flatMap((side) => {
      const foot = `${side}Foot` as const;
      // A requested swing/grab is intent, not evidence that a real contact unloaded.
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

export type StandingControllerId = "legacy" | "h74-v1" | "h77-v1" | "h78-v1" | "h79-v1" | "h80-v1";
export interface CharacterInitialOptions { heading?: number; position?: Vec3; playground?: PlaygroundConfig; room?: boolean;
  standingCandidate?: { controllerId: StandingControllerId; forwardOffsetM?: number } }

export async function createEmbodiedCharacter(
  initialRenderer: RendererMode = "canvas2d",
  options: CharacterInitialOptions = {},
): Promise<CharacterController> {
  await ensureRapier();
  return new EmbodiedCharacter(initialRenderer, options);
}
