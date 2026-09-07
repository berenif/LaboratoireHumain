import RAPIER, {
  type Collider,
  type ImpulseJoint,
  type KinematicCharacterController,
  type RigidBody,
  type World,
} from "@dimforge/rapier3d-compat";

import { HUMAN_PROPORTIONS, REGION_TO_SEGMENT, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import { pickRegionProxies } from "../core/picking";
import {
  emptyGrabDiagnostics,
} from "./GrabAnchorController";
import type {
  CharacterController,
  DiagnosticsSnapshot,
  GrabCommand,
  GrabControlDiagnostics,
  HandoffDiagnostics,
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
import {
  add,
  angularVelocity,
  clamp,
  clampLength,
  dot,
  length,
  lerp,
  normalize,
  rotate,
  scale,
  sub,
  worldPoint,
} from "./math";

import { BalanceController, type BalanceDiagnostics } from "./BalanceController";
import { DynamicRecovery, emptyRecoveryDiagnostics } from "./DynamicRecovery";
import { Q } from "../core/math";
import { quatFromAxisAngle } from "./math";

import {
  composeUprightPose, horizontal, immutablePose, poseAnchor, restPoseMap,
  type MutablePose, type StepMotion,
} from "./pose";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const INITIAL_ROOT: Vec3 = { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 };
const footShape = SEGMENT_BY_ID.get("leftFoot")!.shape;
const FOOT_CENTER_HEIGHT = footShape.kind === "box"
  ? footShape.halfExtents.y
  : HUMAN_PROPORTIONS.foot.halfExtentsM.y;
const ROOT_COLLIDER_RADIUS = Math.max(
  HUMAN_PROPORTIONS.torso.halfExtentsM.x,
  HUMAN_PROPORTIONS.pelvis.halfExtentsM.x,
);
const ROOT_COLLIDER_FLOOR_CLEARANCE = 0.012;
const ROOT_COLLIDER_CENTER_Y = (HUMAN_PROPORTIONS.totalHeightM + ROOT_COLLIDER_FLOOR_CLEARANCE) / 2;
const ROOT_COLLIDER_HALF_HEIGHT = (
  HUMAN_PROPORTIONS.totalHeightM - ROOT_COLLIDER_FLOOR_CLEARANCE
) / 2 - ROOT_COLLIDER_RADIUS;
const ROOT_COLLIDER_OFFSET_Y = ROOT_COLLIDER_CENTER_Y - INITIAL_ROOT.y;
const CHARACTER_GROUP = (0x0001 << 16) | 0x0002;
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
};

let rapierInitialization: Promise<void> | null = null;

function ensureRapier(): Promise<void> {
  rapierInitialization ??= RAPIER.init();
  return rapierInitialization;
}

function segmentCollider(definition: SegmentDefinition): RAPIER.ColliderDesc {
  switch (definition.shape.kind) {
    case "sphere":
      return RAPIER.ColliderDesc.ball(definition.shape.radius);
    case "box":
      return RAPIER.ColliderDesc.cuboid(
        definition.shape.halfExtents.x,
        definition.shape.halfExtents.y,
        definition.shape.halfExtents.z,
      );
    case "capsule":
      return RAPIER.ColliderDesc.capsule(definition.shape.halfHeight, definition.shape.radius);
  }
}

function finiteVec(v: Vec3): boolean {
  return Number.isFinite(v.x) && Number.isFinite(v.y) && Number.isFinite(v.z);
}

function finiteQuat(q: Quat): boolean {
  return Number.isFinite(q.x) && Number.isFinite(q.y) && Number.isFinite(q.z) && Number.isFinite(q.w);
}

class EmbodiedCharacter implements CharacterController {
  private readonly world: World;
  private rootBody: RigidBody | null = null;
  private rootCollider: Collider | null = null;
  private motor: KinematicCharacterController | null = null;
  private readonly ragdollBodies = new Map<SegmentId, RigidBody>();
  private readonly ragdollColliders = new Map<SegmentId, Collider>();
  private readonly ragdollJoints: ImpulseJoint[] = [];
  private readonly rest = restPoseMap();
  private poses = restPoseMap();
  private previousPoses = restPoseMap();
  private renderer: RendererMode;
  private state: MotionState = "upright";
  private activeGrab: ActiveGrab | null = null;
  private readonly balance = new BalanceController();
  private balanceData: BalanceDiagnostics | null = null;
  private kneeFlexion: number = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
  private readonly recovery = new DynamicRecovery();
  private readonly floorCollider: Collider;
  private heading = 0;
  private readonly initialHeading: number;
  private initialPosition: Vec3 = INITIAL_ROOT;
  private poseSeed: Map<SegmentId, MutablePose> | null = null;
  private poseSeedTime = 0;
  private grabControlDiagnostics: GrabControlDiagnostics = emptyGrabDiagnostics();
  private handoffDiagnostics: HandoffDiagnostics | null = null;
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
  private grounded = true;
  private readonly runtimeErrors: string[] = [];

  constructor(renderer: RendererMode, options: CharacterInitialOptions = {}) {
    this.renderer = renderer;
    this.initialHeading = options.heading ?? 0;
    this.heading = this.initialHeading;
    this.initialPosition = options.position ?? INITIAL_ROOT;
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    this.world.timestep = 1 / 60;
    this.world.numSolverIterations = 16;
    this.world.numInternalPgsIterations = 2;
    // Resolve fast limb/floor impacts within the tick so CCD does not separate linked anchors.
    this.world.integrationParameters.maxCcdSubsteps = 4;
    const floorBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.08, 0),
    );
    this.floorCollider = this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(5, 0.08, 5)
        .setFriction(0.95)
        .setRestitution(0.02)
        .setCollisionGroups(ENVIRONMENT_GROUP),
      floorBody,
    );
    // Rapier's character controller queries the broad phase before World.step.
    // Prime the static floor now, while no dynamic bodies exist, so the first
    // grounded update cannot move through it and create a false velocity spike.
    this.world.step();
    this.supportFeet = {
      leftFoot: { ...this.rest.get("leftFoot")!.position, y: FOOT_CENTER_HEIGHT },
      rightFoot: { ...this.rest.get("rightFoot")!.position, y: FOOT_CENTER_HEIGHT },
    };
    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const local = sub(this.supportFeet[foot], { ...INITIAL_ROOT, y: 0 });
      const transformed = add({ ...this.initialPosition, y: 0 }, rotate(quatFromAxisAngle(UP, this.heading), local));
      this.supportFeet[foot] = transformed;
    }
    this.createRootMotor();
    this.poses = this.composeUprightPose();
    this.previousPoses = this.clonePoses(this.poses);
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

    try {
      if (this.isDynamic()) {
        this.updateRagdoll(stepDt);
      } else {
        this.updateUpright(stepDt);
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      if (!this.runtimeErrors.includes(message)) this.runtimeErrors.push(message);
      this.appliedGrabForceN = 0;
    }
  }

  getSnapshot(renderer: RendererMode): PoseSnapshot {
    this.renderer = renderer;
    return {
      sequence: this.sequence,
      simulationTime: this.simulationTime,
      state: this.state,
      rootPosition: { ...this.poses.get("pelvis")!.position },
      rootRotation: { ...this.poses.get("pelvis")!.rotation },
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

  private isDynamic(): boolean { return this.state === "falling" || this.state === "fallen" || this.state === "recovering"; }

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
    this.clearRagdoll();
    this.removeRootMotor();
    this.world.free();
    this.disposed = true;
  }

  reset(): void {
    if (this.disposed) return;
    this.clearGrab();
    this.clearRagdoll();
    this.removeRootMotor();
    this.state = "upright";
    this.heading = this.initialHeading;
    this.poseSeed = null; this.poseSeedTime = 0; this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion; this.balanceData = null;
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
    this.grounded = true;
    this.grabControlDiagnostics = emptyGrabDiagnostics();
    this.handoffDiagnostics = null;
    this.runtimeErrors.length = 0;
    this.poses = restPoseMap();
    this.previousPoses = restPoseMap();
    this.supportFeet = {
      leftFoot: { ...this.rest.get("leftFoot")!.position, y: FOOT_CENTER_HEIGHT },
      rightFoot: { ...this.rest.get("rightFoot")!.position, y: FOOT_CENTER_HEIGHT },
    };
    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const local = sub(this.supportFeet[foot], { ...INITIAL_ROOT, y: 0 });
      const transformed = add({ ...this.initialPosition, y: 0 }, rotate(quatFromAxisAngle(UP, this.heading), local));
      this.supportFeet[foot] = transformed;
    }
    this.createRootMotor();
    this.poses = this.composeUprightPose();
    this.previousPoses = this.clonePoses(this.poses);
    this.balance.reset(this.poses, this.heading);
  }

  diagnostics(): DiagnosticsSnapshot {
    const root = this.poses.get("pelvis")?.position ?? INITIAL_ROOT;
    const finite = [...this.poses.values()].every(
      (pose) => finiteVec(pose.position) && finiteQuat(pose.rotation)
        && finiteVec(pose.linearVelocity) && finiteVec(pose.angularVelocity),
    );
    const errors = [...this.runtimeErrors];
    if (!finite) errors.push("NONFINITE_CHARACTER_STATE");
    return {
      balance: this.balanceData ? structuredClone(this.balanceData) : null,
      bodyInputAvailable: !this.disposed && !this.paused && !this.isDynamic() && !this.poseSeed,
      recovery: this.isDynamic() ? this.recovery.diagnostics() : emptyRecoveryDiagnostics(),
      authority: this.isDynamic() ? "ragdoll" : "character-motor",
      state: this.state,
      simulationReady: true,
      interactiveViewReady: true,
      renderer: this.renderer,
      activeGrab: this.activeGrab !== null,
      activePointerId: this.activeGrab?.pointerId ?? null,
      selectedRegion: this.activeGrab?.region ?? null,
      queuedTarget: false,
      appliedGrabForceN: this.appliedGrabForceN,
      grabControl: { ...this.grabControlDiagnostics },
      handoff: this.handoffDiagnostics ? { ...this.handoffDiagnostics } : null,
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

  private processCommand(command: GrabCommand | null): void {
    if (this.isDynamic() || this.poseSeed) { this.clearGrab(); return; }
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
      const anchor = command.localAnchor ?? ZERO;
      const anchorWorld = worldPoint(pose.position, pose.rotation, anchor);
      const target = command.worldTarget ?? anchorWorld;
      this.activeGrab = {
        pointerId: command.pointerId,
        region: command.region,
        segment,
        localAnchor: { ...anchor },
        target: { ...target },
        startTarget: { ...target },
        startSegmentPosition: { ...pose.position },
        targetVelocity: ZERO,
        lastCommandTimestampMs: command.timestampMs,
      };
      return;
    }
    if (!this.activeGrab || command.pointerId !== this.activeGrab.pointerId) return;
    if (command.kind === "move" && command.worldTarget) {
      const commandDt = clamp((command.timestampMs - this.activeGrab.lastCommandTimestampMs) / 1000, 1 / 240, 0.1);
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
    this.grabControlDiagnostics = emptyGrabDiagnostics(this.grabControlDiagnostics.cumulativeInjectedWorkJ);
    this.appliedGrabForceN = 0;
  }

  private updateUpright(dt: number): void {
    if (!this.rootBody || !this.rootCollider || !this.motor) return;
    const result = this.balance.update({ dt, poses: this.poses, rootPosition: this.rootBody.translation(), activeGrab: this.activeGrab, heading: this.heading });
    const settling = this.poseSeed !== null;
    if (settling) {
      this.balanceData = null;
      this.reactionOffset = ZERO;
      this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion;
      this.step = null;
      this.state = "upright";
      this.appliedGrabForceN = 0;
    } else {
      this.balanceData = result.diagnostics;
      this.reactionOffset = result.reactionOffset; this.kneeFlexion = result.kneeFlexion;
      this.supportFeet = result.supportFeet; this.step = result.step; this.stepCount = result.stepCount;
      this.state = result.state; this.appliedGrabForceN = result.appliedGrabForceN;
    }
    const current = this.rootBody.translation();
    const desired = settling ? ZERO : sub(result.rootTarget, current);
    // A planted stance already sits at the controller's configured separation.
    // Repeated downward probes can occasionally tunnel the kinematic capsule
    // through Rapier's contact offset; probe only after support is actually lost.
    this.motor.computeColliderMovement(this.rootCollider, { ...desired, y: this.grounded ? 0 : -0.035 });
    const computedMovement = this.motor.computedMovement();
    // Snap-to-ground may return a small downward correction even for a zero-Y
    // request. Anatomical foot support already establishes the flat-floor
    // height, so retain it until support is genuinely lost.
    const movement = this.grounded ? { ...computedMovement, y: 0 } : computedMovement;
    this.grounded = settling || this.motor.computedGrounded() || result.diagnostics.supportingFeet.length > 0;
    this.rootBody.setNextKinematicTranslation(add(current, movement));
    this.world.step();
    this.previousPoses = this.clonePoses(this.poses);
    const target = this.composeUprightPose();
    let finishedSettling = false;
    if (this.poseSeed) {
      this.poseSeedTime += dt;
      const amount = clamp(this.poseSeedTime / 0.75, 0, 1);
      for (const definition of SEGMENTS) {
        const pose = target.get(definition.id)!, seed = this.poseSeed.get(definition.id)!;
        pose.rotation = Q.nlerp(seed.rotation, pose.rotation, amount);
        if (!definition.parent) pose.position = lerp(seed.position, pose.position, amount);
        if (definition.parent) {
          const parent = target.get(definition.parent)!;
          pose.position = sub(poseAnchor(parent, definition.jointAnchorParent!), rotate(pose.rotation, definition.jointAnchorChild!));
        }
      }
      if (amount >= 1) {
        this.poseSeed = null;
        finishedSettling = true;
      }
    }
    this.poses = target;
    this.writePoseVelocities(dt);
    if (finishedSettling) {
      for (const pose of this.poses.values()) {
        pose.linearVelocity = ZERO;
        pose.angularVelocity = ZERO;
      }
      this.previousPoses = this.clonePoses(this.poses);
      this.balance.reset(this.poses, this.heading);
      this.balanceData = null;
    }
    if (!settling && result.shouldFall) this.activateRagdoll(result.fallDirection);
  }

  private composeUprightPose(): Map<SegmentId, MutablePose> {
    const composed = composeUprightPose({
      rootTranslation: this.rootBody?.translation() ?? this.initialPosition,
      heading: this.heading,
      kneeFlexion: this.kneeFlexion,
      reactionOffset: this.reactionOffset,
      simulationTime: this.simulationTime,
      activeGrab: this.activeGrab,
      supportFeet: this.supportFeet,
      step: this.step,
    });
    this.leanRadians = composed.leanRadians;
    return composed.poses;
  }

  private writePoseVelocities(dt: number): void {
    for (const definition of SEGMENTS) {
      const current = this.poses.get(definition.id)!;
      const previous = this.previousPoses.get(definition.id) ?? current;
      current.linearVelocity = clampLength(scale(sub(current.position, previous.position), 1 / dt), 10);
      current.angularVelocity = angularVelocity(previous.rotation, current.rotation, dt);
    }
  }

  private activateRagdoll(drag: Vec3): void {
    if (this.isDynamic()) return;
    const transferPoses = this.clonePoses(this.poses);
    const beforeAnchor = this.grabAnchorWorld();
    const beforeTarget = this.activeGrab ? { ...this.activeGrab.target } : ZERO;
    const beforeLocalAnchor = this.activeGrab ? { ...this.activeGrab.localAnchor } : ZERO;
    this.removeRootMotor();
    const pullVelocity = this.activeGrab?.targetVelocity ?? ZERO;
    const fallDirection = normalize(add(horizontal(drag), scale(horizontal(pullVelocity), 0.12)), FORWARD);

    for (const definition of SEGMENTS) {
      const pose = transferPoses.get(definition.id)!;
      const inheritedVelocity = clampLength(pose.linearVelocity, 3.0);
      const inheritedAngularVelocity = clampLength(pose.angularVelocity, 6.0);
      const body = this.world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(pose.position.x, pose.position.y, pose.position.z)
          .setRotation(pose.rotation)
          .setLinvel(inheritedVelocity.x, inheritedVelocity.y, inheritedVelocity.z)
          .setAngvel(inheritedAngularVelocity)
          .setLinearDamping(0.32)
          .setAngularDamping(0.42)
          .setCanSleep(false)
          .setCcdEnabled(true)
          .setAdditionalSolverIterations(3),
      );
      const collider = this.world.createCollider(
        segmentCollider(definition)
          .setMass(definition.massKg)
          .setFriction(1.15)
          .setRestitution(0.04)
          .setContactSkin(0.002)
          .setCollisionGroups(CHARACTER_GROUP),
        body,
      );
      this.ragdollBodies.set(definition.id, body);
      this.ragdollColliders.set(definition.id, collider);
      body.recomputeMassPropertiesFromColliders();
    }
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointAnchorParent || !definition.jointAnchorChild) continue;
      const parent = this.ragdollBodies.get(definition.parent)!;
      const child = this.ragdollBodies.get(definition.id)!;
      const joint = this.world.createImpulseJoint(
        RAPIER.JointData.spherical(definition.jointAnchorParent, definition.jointAnchorChild),
        parent,
        child,
        true,
      );
      joint.setContactsEnabled(false);
      this.ragdollJoints.push(joint);
    }
    this.state = "falling";
    this.recovery.reset(this.heading, fallDirection);
    this.balanceData = null;
    this.clearGrab();
    this.fallingTime = 0;
    // Read the actual new Rapier bodies before stepping. This measures the
    // same instant on both sides of authority transfer, not consecutive frames.
    this.readRagdollPoses();
    let maxTranslationErrorM = 0;
    let maxAngularErrorDegrees = 0;
    for (const [id, before] of transferPoses) {
      const after = this.poses.get(id)!;
      maxTranslationErrorM = Math.max(maxTranslationErrorM, length(sub(after.position, before.position)));
      const qa = before.rotation;
      const qb = after.rotation;
      const rotationDot = Math.abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)
        / (Math.hypot(qa.x, qa.y, qa.z, qa.w) * Math.hypot(qb.x, qb.y, qb.z, qb.w));
      maxAngularErrorDegrees = Math.max(maxAngularErrorDegrees, 2 * Math.acos(clamp(rotationDot, -1, 1)) * 180 / Math.PI);
    }
    const afterAnchor = this.grabAnchorWorld();
    this.handoffDiagnostics = {
      sequence: this.sequence, maxTranslationErrorM, maxAngularErrorDegrees,
      selectedAnchorErrorM: beforeAnchor && afterAnchor ? length(sub(beforeAnchor, afterAnchor)) : 0,
      rawTargetErrorM: this.activeGrab ? length(sub(beforeTarget, this.activeGrab.target)) : 0,
      localAnchorErrorM: this.activeGrab ? length(sub(beforeLocalAnchor, this.activeGrab.localAnchor)) : 0,
      jointSeparationM: this.maximumJointSeparation(),
      targetDerivativeSpeedMps: this.activeGrab ? length(this.activeGrab.targetVelocity) : 0,
    };
    this.previousPoses = this.clonePoses(this.poses);
    this.appliedGrabForceN = 0;
    this.grabControlDiagnostics = emptyGrabDiagnostics();
  }

  private updateRagdoll(dt: number): void {
    this.previousPoses = this.clonePoses(this.poses);
    this.clearGrab();
    // A fixture may remove support between updates. Query current collider state before applying any aid.
    if (!this.floorCollider.isEnabled()) {
      this.recovery.observe(this.world, this.floorCollider, new Map(), this.ragdollBodies, dt);
    }
    const result = this.recovery.apply(this.ragdollBodies, dt);
    this.state = result.state;
    if (result.recovered) { this.restoreUpright(); return; }
    this.world.step();
    this.readRagdollPoses();
    this.recovery.observe(this.world, this.floorCollider, this.ragdollColliders, this.ragdollBodies, dt);
    this.fallingTime += dt;
    this.grounded = this.recovery.diagnostics().contacts.length > 0;
    this.leanRadians = this.currentTorsoLean();
  }

  private restoreUpright(): void {
    const before = this.clonePoses(this.poses);
    const pelvis = before.get("pelvis")!;
    const forward = rotate(pelvis.rotation, FORWARD);
    this.heading = Math.atan2(forward.x, forward.z);
    this.clearGrab(); this.clearRagdoll();
    this.createRootMotor(pelvis.position);
    this.supportFeet = { leftFoot: { ...before.get("leftFoot")!.position }, rightFoot: { ...before.get("rightFoot")!.position } };
    this.reactionOffset = ZERO; this.kneeFlexion = HUMAN_PROPORTIONS.stance.neutralKneeFlexion; this.step = null;
    this.state = "upright"; this.grounded = true;
    // The first motor snapshot is the recovered segment pose, at exactly the same simulation instant.
    this.poses = before; this.previousPoses = this.clonePoses(before);
    this.poseSeed = this.clonePoses(before); this.poseSeedTime = 0;
    this.balance.reset(this.poses, this.heading);
    this.balanceData = null;
    this.stepCount = 0;
    this.handoffDiagnostics = { sequence: this.sequence, maxTranslationErrorM: 0, maxAngularErrorDegrees: 0,
      selectedAnchorErrorM: 0, rawTargetErrorM: 0, localAnchorErrorM: 0,
      jointSeparationM: this.maximumJointSeparation(), targetDerivativeSpeedMps: 0 };
  }

  private readRagdollPoses(): void {
    for (const definition of SEGMENTS) {
      const body = this.ragdollBodies.get(definition.id)!;
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

  private createRootMotor(position: Vec3 = this.initialPosition): void {
    this.rootBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        position.x,
        position.y,
        position.z,
      ),
    );
    this.rootCollider = this.world.createCollider(
      RAPIER.ColliderDesc.capsule(ROOT_COLLIDER_HALF_HEIGHT, ROOT_COLLIDER_RADIUS)
        .setTranslation(0, ROOT_COLLIDER_OFFSET_Y, 0)
        .setFriction(0.9)
        .setCollisionGroups(CHARACTER_GROUP),
      this.rootBody,
    );
    this.motor = this.world.createCharacterController(ROOT_COLLIDER_FLOOR_CLEARANCE);
    this.motor.setUp(UP);
    this.motor.setSlideEnabled(true);
    this.motor.enableSnapToGround(0.08);
    this.motor.setMaxSlopeClimbAngle(Math.PI * 0.25);
    this.motor.setMinSlopeSlideAngle(Math.PI * 0.32);
    this.motor.setApplyImpulsesToDynamicBodies(false);
  }

  private removeRootMotor(): void {
    if (this.motor) this.world.removeCharacterController(this.motor);
    if (this.rootBody) this.world.removeRigidBody(this.rootBody);
    this.motor = null;
    this.rootBody = null;
    this.rootCollider = null;
  }

  private clearRagdoll(): void {
    for (const joint of this.ragdollJoints.splice(0)) {
      if (joint.isValid()) this.world.removeImpulseJoint(joint, false);
    }
    for (const body of this.ragdollBodies.values()) {
      if (body.isValid()) this.world.removeRigidBody(body);
    }
    this.ragdollBodies.clear();
    this.ragdollColliders.clear();
  }

  private clonePoses(source: Map<SegmentId, MutablePose>): Map<SegmentId, MutablePose> {
    return new Map([...source].map(([id, pose]) => [id, {
      id,
      position: { ...pose.position },
      rotation: { ...pose.rotation },
      linearVelocity: { ...pose.linearVelocity },
      angularVelocity: { ...pose.angularVelocity },
    }]));
  }

  private supportSnapshot(): SupportState {
    if (this.isDynamic()) {
      const contacts = this.recovery.diagnostics().contacts;
      const planted = contacts.filter(c => c.loadBearing && (c.segment === "leftFoot" || c.segment === "rightFoot")).map(c => c.segment as "leftFoot" | "rightFoot");
      return { planted, swingFoot: null, stepProgress: 0, grounded: this.grounded };
    }
    return { planted: this.balanceData?.supportingFeet ? [...this.balanceData.supportingFeet] : ["leftFoot", "rightFoot"],
      swingFoot: this.step?.foot ?? null, stepProgress: this.step ? clamp(this.step.elapsed / this.step.duration, 0, 1) : 0, grounded: this.grounded };
  }

  private grabAnchorWorld(): Vec3 | null {
    if (!this.activeGrab) return null;
    const pose = this.poses.get(this.activeGrab.segment);
    return pose ? worldPoint(pose.position, pose.rotation, this.activeGrab.localAnchor) : null;
  }

  private maximumJointSeparation(): number {
    let maximum = 0;
    for (const definition of SEGMENTS) {
      if (!definition.parent || !definition.jointAnchorParent || !definition.jointAnchorChild) continue;
      const parent = this.poses.get(definition.parent);
      const child = this.poses.get(definition.id);
      if (!parent || !child) continue;
      maximum = Math.max(
        maximum,
        length(sub(poseAnchor(parent, definition.jointAnchorParent), poseAnchor(child, definition.jointAnchorChild))),
      );
    }
    return maximum;
  }

  private maximumFloorPenetration(): number {
    if (this.disposed || !this.floorCollider.isEnabled()) return 0;
    let penetration = 0;
    for (const definition of SEGMENTS) {
      const pose = this.poses.get(definition.id);
      if (!pose) continue;
      // Query the finite, enabled floor against the actual oriented segment shape.
      // Negative distance is penetration; positive clearance is never an error.
      const contact = this.floorCollider.contactShape(segmentCollider(definition).shape, pose.position, pose.rotation, 0);
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

export interface CharacterInitialOptions { heading?: number; position?: Vec3; }

export async function createEmbodiedCharacter(
  initialRenderer: RendererMode = "canvas2d",
  options: CharacterInitialOptions = {},
): Promise<CharacterController> {
  await ensureRapier();
  return new EmbodiedCharacter(initialRenderer, options);
}
