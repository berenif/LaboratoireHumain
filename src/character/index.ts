import RAPIER, {
  type Collider,
  type ImpulseJoint,
  type KinematicCharacterController,
  type RigidBody,
  type World,
} from "@dimforge/rapier3d-compat";

import { REGION_TO_SEGMENT, SEGMENT_BY_ID, SEGMENTS } from "../core/humanoid";
import { pickRegionProxies } from "../interaction/picking";
import {
  emptyGrabDiagnostics,
  GrabAnchorController,
  type GrabControlDiagnostics,
  type HandoffDiagnostics,
} from "./GrabAnchorController";
import type {
  CharacterController,
  DiagnosticsSnapshot,
  GrabCommand,
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
import {
  add,
  angularVelocity,
  clamp,
  clampLength,
  cross,
  dot,
  length,
  lerp,
  normalize,
  quatFromTo,
  quatMultiply,
  rotate,
  scale,
  smooth01,
  sub,
  worldPoint,
} from "./math";

const IDENTITY: Quat = { x: 0, y: 0, z: 0, w: 1 };
const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const UP: Vec3 = { x: 0, y: 1, z: 0 };
const FORWARD: Vec3 = { x: 0, y: 0, z: 1 };
const INITIAL_ROOT: Vec3 = { x: 0, y: 1.03, z: 0 };
const CHARACTER_GROUP = (0x0001 << 16) | 0x0002;
const ENVIRONMENT_GROUP = (0x0002 << 16) | 0x0001;

type MutablePose = {
  id: SegmentId;
  position: Vec3;
  rotation: Quat;
  linearVelocity: Vec3;
  angularVelocity: Vec3;
};

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

type StepMotion = {
  foot: "leftFoot" | "rightFoot";
  from: Vec3;
  to: Vec3;
  elapsed: number;
  duration: number;
};

type LegSide = "left" | "right";

let rapierInitialization: Promise<void> | null = null;

function ensureRapier(): Promise<void> {
  rapierInitialization ??= RAPIER.init();
  return rapierInitialization;
}

function immutablePose(pose: MutablePose): SegmentPose {
  return {
    id: pose.id,
    position: { ...pose.position },
    rotation: { ...pose.rotation },
    linearVelocity: { ...pose.linearVelocity },
    angularVelocity: { ...pose.angularVelocity },
  };
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

function restPoseMap(): Map<SegmentId, MutablePose> {
  const poses = new Map<SegmentId, MutablePose>();
  for (const definition of SEGMENTS) {
    const parent = definition.parent ? poses.get(definition.parent) : null;
    const rotation = parent
      ? quatMultiply(parent.rotation, definition.restLocalRotation)
      : definition.restLocalRotation;
    const position = parent
      ? add(parent.position, rotate(parent.rotation, definition.localOffset))
      : definition.localOffset;
    poses.set(definition.id, {
      id: definition.id,
      position,
      rotation,
      linearVelocity: ZERO,
      angularVelocity: ZERO,
    });
  }
  return poses;
}

function makePose(id: SegmentId, position: Vec3, rotation: Quat = IDENTITY): MutablePose {
  return { id, position, rotation, linearVelocity: ZERO, angularVelocity: ZERO };
}

function poseAnchor(pose: MutablePose, anchor: Vec3): Vec3 {
  return worldPoint(pose.position, pose.rotation, anchor);
}

function solveTwoBone(
  start: Vec3,
  requestedEnd: Vec3,
  firstLength: number,
  secondLength: number,
  preferredBend: Vec3,
): Readonly<{ middle: Vec3; end: Vec3 }> {
  const raw = sub(requestedEnd, start);
  const direction = normalize(raw, { x: 0, y: -1, z: 0 });
  const minimum = Math.abs(firstLength - secondLength) + 0.005;
  const maximum = firstLength + secondLength - 0.002;
  const distance = clamp(length(raw), minimum, maximum);
  const end = add(start, scale(direction, distance));
  const along = (firstLength * firstLength - secondLength * secondLength + distance * distance) / (2 * distance);
  const bendHeight = Math.sqrt(Math.max(0, firstLength * firstLength - along * along));
  let bend = sub(preferredBend, scale(direction, dot(preferredBend, direction)));
  if (length(bend) < 1e-4) bend = cross(direction, { x: 1, y: 0, z: 0 });
  bend = normalize(bend, FORWARD);
  return { middle: add(add(start, scale(direction, along)), scale(bend, bendHeight)), end };
}

function midpoint(a: Vec3, b: Vec3): Vec3 {
  return scale(add(a, b), 0.5);
}

function horizontal(v: Vec3): Vec3 {
  return { x: v.x, y: 0, z: v.z };
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
  private grabController: GrabAnchorController | null = null;
  private grabControlDiagnostics: GrabControlDiagnostics = emptyGrabDiagnostics();
  private handoffDiagnostics: HandoffDiagnostics | null = null;
  private paused = false;
  private sequence = 0;
  private simulationTime = 0;
  private reactionOffset: Vec3 = ZERO;
  private leanRadians = 0;
  private supportFeet: Record<"leftFoot" | "rightFoot", Vec3>;
  private step: StepMotion | null = null;
  private stepCount = 0;
  private nextDepthStepSide: LegSide = "left";
  private lastStepPullDistance = 0;
  private effort = 0;
  private appliedGrabForceN = 0;
  private fixedSteps = 0;
  private fallingTime = 0;
  private grounded = true;
  private readonly runtimeErrors: string[] = [];

  constructor(renderer: RendererMode) {
    this.renderer = renderer;
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    this.world.timestep = 1 / 60;
    this.world.numSolverIterations = 8;
    this.world.numInternalPgsIterations = 2;
    const floorBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.08, 0),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(5, 0.08, 5)
        .setFriction(0.95)
        .setRestitution(0.02)
        .setCollisionGroups(ENVIRONMENT_GROUP),
      floorBody,
    );
    this.supportFeet = {
      leftFoot: { ...this.rest.get("leftFoot")!.position, y: 0.065 },
      rightFoot: { ...this.rest.get("rightFoot")!.position, y: 0.065 },
    };
    this.createRootMotor();
    this.poses = this.composeUprightPose();
    this.previousPoses = this.clonePoses(this.poses);
  }

  fixedUpdate(dt: number, command: GrabCommand | null): void {
    if (!Number.isFinite(dt) || dt <= 0) return;
    this.processCommand(command);
    if (this.paused) return;

    const stepDt = clamp(dt, 1 / 240, 1 / 20);
    this.world.timestep = stepDt;
    this.sequence += 1;
    this.fixedSteps += 1;
    this.simulationTime += stepDt;

    try {
      if (this.state === "falling" || this.state === "fallen") {
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
    return pickRegionProxies(ray, [...this.poses.values()]);
  }

  pause(): void {
    this.paused = true;
    this.clearGrab();
  }

  resume(): void {
    this.paused = false;
    this.clearGrab();
  }

  reset(): void {
    this.clearGrab();
    this.clearRagdoll();
    this.removeRootMotor();
    this.state = "upright";
    this.paused = false;
    this.sequence = 0;
    this.simulationTime = 0;
    this.reactionOffset = ZERO;
    this.leanRadians = 0;
    this.step = null;
    this.stepCount = 0;
    this.nextDepthStepSide = "left";
    this.lastStepPullDistance = 0;
    this.effort = 0;
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
      leftFoot: { ...this.rest.get("leftFoot")!.position, y: 0.065 },
      rightFoot: { ...this.rest.get("rightFoot")!.position, y: 0.065 },
    };
    this.createRootMotor();
    this.poses = this.composeUprightPose();
    this.previousPoses = this.clonePoses(this.poses);
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
      authority: this.state === "falling" || this.state === "fallen" ? "ragdoll" : "character-motor",
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
      rootDisplacementM: Math.hypot(root.x - INITIAL_ROOT.x, root.z - INITIAL_ROOT.z),
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
      if (this.state === "falling" || this.state === "fallen") {
        this.grabController = new GrabAnchorController(target);
      }
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
    this.activeGrab = null;
    this.grabController = null;
    this.grabControlDiagnostics = emptyGrabDiagnostics(this.grabControlDiagnostics.cumulativeInjectedWorkJ);
    this.effort = 0;
    this.lastStepPullDistance = 0;
    this.appliedGrabForceN = 0;
  }

  private updateUpright(dt: number): void {
    const drag = this.activeGrab ? sub(this.activeGrab.target, this.activeGrab.startTarget) : ZERO;
    const smoothing = 1 - Math.exp(-dt * (this.activeGrab ? 12 : 7));
    this.reactionOffset = lerp(this.reactionOffset, drag, smoothing);
    const meaningful = Math.max(0, length(drag) - 0.14);
    this.effort = this.activeGrab
      ? clamp(this.effort + meaningful * dt - (meaningful === 0 ? dt * 0.8 : 0), 0, 2)
      : Math.max(0, this.effort - dt * 1.8);

    this.advanceStep(dt, drag);
    this.moveRootMotor(dt, drag);
    this.previousPoses = this.clonePoses(this.poses);
    this.poses = this.composeUprightPose();
    this.writePoseVelocities(dt);

    const activeAnchor = this.activeGrab ? this.grabAnchorWorld() : null;
    const error = activeAnchor && this.activeGrab ? sub(this.activeGrab.target, activeAnchor) : ZERO;
    this.appliedGrabForceN = this.activeGrab ? Math.min(680, length(error) * 520) : 0;

    const pullDistance = length(drag);
    const horizontalPull = length(horizontal(drag));
    const pullRate = this.activeGrab ? length(this.activeGrab.targetVelocity) : 0;
    const supportExhausted = this.supportReach() > 0.28 || horizontalPull > 0.58;
    const overload = this.activeGrab !== null && (
      pullDistance > 0.92
      || (pullDistance > 0.58 && supportExhausted && this.effort > 0.13)
      || (pullDistance > 0.62 && pullRate > 2.6)
      || (this.leanRadians > 0.52 && pullDistance > 0.5)
    );
    if (overload) this.activateRagdoll(drag);
  }

  private moveRootMotor(dt: number, drag: Vec3): void {
    if (!this.rootBody || !this.rootCollider || !this.motor) return;
    const current = this.rootBody.translation();
    const left = this.supportFeet.leftFoot;
    const right = this.supportFeet.rightFoot;
    const supportCenter = midpoint(left, right);
    const restSupportCenter = midpoint(
      this.rest.get("leftFoot")!.position,
      this.rest.get("rightFoot")!.position,
    );
    const supportDelta = sub(supportCenter, restSupportCenter);
    const regionWeight = this.activeGrab
      ? this.activeGrab.region === "torso" || this.activeGrab.region === "pelvis"
        ? 0.5
        : this.activeGrab.region === "head" ? 0.38 : 0.3
      : 0;
    const directShift = clampLength(scale(horizontal(drag), regionWeight), this.step ? 0.42 : 0.3);
    const target = {
      x: INITIAL_ROOT.x + supportDelta.x * 0.78 + directShift.x,
      y: INITIAL_ROOT.y,
      z: INITIAL_ROOT.z + supportDelta.z * 0.78 + directShift.z,
    };
    const response = 1 - Math.exp(-dt * 8);
    const desired = {
      x: (target.x - current.x) * response,
      y: -0.035,
      z: (target.z - current.z) * response,
    };
    this.motor.computeColliderMovement(this.rootCollider, desired);
    const movement = this.motor.computedMovement();
    this.grounded = this.motor.computedGrounded() || current.y <= INITIAL_ROOT.y + 0.02;
    this.rootBody.setNextKinematicTranslation({
      x: current.x + movement.x,
      y: current.y + movement.y,
      z: current.z + movement.z,
    });
    this.world.step();
  }

  private advanceStep(dt: number, drag: Vec3): void {
    if (this.step) {
      this.step.elapsed += dt;
      if (this.step.elapsed >= this.step.duration) {
        this.supportFeet[this.step.foot] = { ...this.step.to };
        this.step = null;
        this.state = this.activeGrab ? "reacting" : "upright";
      } else {
        this.state = "stepping";
      }
      return;
    }
    if (!this.activeGrab) {
      this.state = length(this.reactionOffset) > 0.025 ? "reacting" : "upright";
      return;
    }
    const direction = normalize(horizontal(drag), FORWARD);
    const plantedReach = this.supportReach();
    const pullDistance = length(horizontal(drag));
    if (
      pullDistance < 0.34
      || pullDistance < this.lastStepPullDistance + 0.14
      || (plantedReach < 0.14 && pullDistance < 0.4)
    ) {
      this.state = "reacting";
      return;
    }
    let foot: "leftFoot" | "rightFoot";
    if (this.activeGrab.region === "leftFoot" || this.activeGrab.region === "rightFoot") {
      foot = this.activeGrab.region;
    } else if (Math.abs(direction.x) > 0.3) {
      foot = direction.x < 0 ? "leftFoot" : "rightFoot";
    } else {
      foot = `${this.nextDepthStepSide}Foot`;
      this.nextDepthStepSide = this.nextDepthStepSide === "left" ? "right" : "left";
    }
    const from = this.supportFeet[foot];
    const root = this.rootBody?.translation() ?? INITIAL_ROOT;
    const sideOffset = foot === "leftFoot" ? -0.14 : 0.14;
    const travel = clamp(length(horizontal(drag)) * 0.52, 0.28, 0.46);
    const to = {
      x: root.x + sideOffset + direction.x * travel,
      y: 0.065,
      z: root.z + 0.075 + direction.z * travel,
    };
    this.step = { foot, from: { ...from }, to, elapsed: 0, duration: 0.42 };
    this.lastStepPullDistance = pullDistance;
    this.stepCount += 1;
    this.state = "stepping";
  }

  private composeUprightPose(): Map<SegmentId, MutablePose> {
    const output = new Map<SegmentId, MutablePose>();
    const rootTranslation = this.rootBody?.translation() ?? INITIAL_ROOT;
    const root: Vec3 = { x: rootTranslation.x, y: rootTranslation.y, z: rootTranslation.z };
    const drag = this.activeGrab ? sub(this.activeGrab.target, this.activeGrab.startTarget) : ZERO;
    const horizontalReaction = horizontal(this.reactionOffset);
    this.leanRadians = clamp(length(horizontalReaction) * 0.78, 0, 0.62);
    const pelvisTilt = quatFromTo(UP, normalize({
      x: horizontalReaction.x * 0.25,
      y: 1,
      z: horizontalReaction.z * 0.25,
    }));
    const torsoTilt = quatFromTo(UP, normalize({
      x: horizontalReaction.x * 0.7,
      y: 1,
      z: horizontalReaction.z * 0.7,
    }));

    const pelvis = makePose("pelvis", root, pelvisTilt);
    output.set("pelvis", pelvis);
    const torsoDefinition = SEGMENT_BY_ID.get("torso")!;
    const torsoJoint = poseAnchor(pelvis, torsoDefinition.jointAnchorParent!);
    const torso = makePose(
      "torso",
      sub(torsoJoint, rotate(torsoTilt, torsoDefinition.jointAnchorChild!)),
      torsoTilt,
    );
    output.set("torso", torso);

    const headDrag = this.activeGrab?.region === "head" ? clampLength(drag, 0.42) : ZERO;
    const headTilt = quatFromTo(UP, normalize({
      x: horizontalReaction.x * 0.45 + headDrag.x * 1.05,
      y: 1 + headDrag.y * 0.2,
      z: horizontalReaction.z * 0.45 + headDrag.z * 1.05,
    }));
    const neckDefinition = SEGMENT_BY_ID.get("neck")!;
    const neckJoint = poseAnchor(torso, neckDefinition.jointAnchorParent!);
    const neck = makePose("neck", sub(neckJoint, rotate(torsoTilt, neckDefinition.jointAnchorChild!)), torsoTilt);
    output.set("neck", neck);
    const headDefinition = SEGMENT_BY_ID.get("head")!;
    const headJoint = poseAnchor(neck, headDefinition.jointAnchorParent!);
    const head = makePose("head", sub(headJoint, rotate(headTilt, headDefinition.jointAnchorChild!)), headTilt);
    output.set("head", head);

    this.composeArm("left", torso, output, drag);
    this.composeArm("right", torso, output, drag);
    this.composeLeg("left", pelvis, output, drag);
    this.composeLeg("right", pelvis, output, drag);
    return output;
  }

  private composeArm(
    side: LegSide,
    torso: MutablePose,
    output: Map<SegmentId, MutablePose>,
    drag: Vec3,
  ): void {
    const sign = side === "left" ? -1 : 1;
    const upperId = `${side}UpperArm` as SegmentId;
    const forearmId = `${side}Forearm` as SegmentId;
    const handId = `${side}Hand` as RegionId;
    const upperDefinition = SEGMENT_BY_ID.get(upperId)!;
    const shoulder = poseAnchor(torso, upperDefinition.jointAnchorParent!);
    const idle = Math.sin(this.simulationTime * 1.7 + (side === "left" ? 0 : Math.PI)) * 0.018;
    let desiredHand: Vec3 = add(shoulder, rotate(torso.rotation, {
      x: sign * 0.075,
      y: -0.625,
      z: idle,
    }));
    if (this.activeGrab?.region === handId) {
      desiredHand = add(this.activeGrab.startSegmentPosition, clampLength(drag, 0.72));
    } else {
      desiredHand = add(desiredHand, scale(horizontal(this.reactionOffset), -0.13));
    }

    const approximateAxis = normalize(sub(shoulder, desiredHand), UP);
    const requestedWrist = add(desiredHand, scale(approximateAxis, 0.105));
    const solved = solveTwoBone(
      shoulder,
      requestedWrist,
      0.28,
      0.26,
      normalize({ x: sign * 0.22, y: 0, z: 1 }),
    );
    const upperAxis = normalize(sub(shoulder, solved.middle), UP);
    const forearmAxis = normalize(sub(solved.middle, solved.end), UP);
    const upper = makePose(upperId, midpoint(shoulder, solved.middle), quatFromTo(UP, upperAxis));
    const forearm = makePose(forearmId, midpoint(solved.middle, solved.end), quatFromTo(UP, forearmAxis));
    const hand = makePose(
      handId,
      sub(solved.end, scale(forearmAxis, 0.105)),
      quatFromTo(UP, forearmAxis),
    );
    output.set(upperId, upper);
    output.set(forearmId, forearm);
    output.set(handId, hand);
  }

  private composeLeg(
    side: LegSide,
    pelvis: MutablePose,
    output: Map<SegmentId, MutablePose>,
    drag: Vec3,
  ): void {
    const thighId = `${side}Thigh` as SegmentId;
    const shinId = `${side}Shin` as SegmentId;
    const footId = `${side}Foot` as "leftFoot" | "rightFoot";
    const thighDefinition = SEGMENT_BY_ID.get(thighId)!;
    const footDefinition = SEGMENT_BY_ID.get(footId)!;
    const hip = poseAnchor(pelvis, thighDefinition.jointAnchorParent!);
    let footPosition = this.supportFeet[footId];

    if (this.step?.foot === footId) {
      const progress = smooth01(this.step.elapsed / this.step.duration);
      const base = lerp(this.step.from, this.step.to, progress);
      footPosition = { ...base, y: base.y + Math.sin(Math.PI * progress) * 0.17 };
    } else if (this.activeGrab?.region === footId) {
      footPosition = add(this.activeGrab.startSegmentPosition, clampLength(drag, 0.58));
      footPosition = { ...footPosition, y: Math.max(0.07, footPosition.y) };
    }

    const footRotation = IDENTITY;
    const ankle = worldPoint(footPosition, footRotation, footDefinition.jointAnchorChild!);
    const solved = solveTwoBone(hip, ankle, 0.38, 0.36, FORWARD);
    const thighAxis = normalize(sub(hip, solved.middle), UP);
    const shinAxis = normalize(sub(solved.middle, solved.end), UP);
    output.set(thighId, makePose(thighId, midpoint(hip, solved.middle), quatFromTo(UP, thighAxis)));
    output.set(shinId, makePose(shinId, midpoint(solved.middle, solved.end), quatFromTo(UP, shinAxis)));
    output.set(footId, makePose(footId, footPosition, footRotation));
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
    if (this.state === "falling" || this.state === "fallen") return;
    const transferPoses = this.clonePoses(this.poses);
    const beforeAnchor = this.grabAnchorWorld();
    const beforeTarget = this.activeGrab ? { ...this.activeGrab.target } : ZERO;
    const beforeLocalAnchor = this.activeGrab ? { ...this.activeGrab.localAnchor } : ZERO;
    this.removeRootMotor();
    const pullVelocity = this.activeGrab?.targetVelocity ?? ZERO;
    const fallDirection = normalize(add(horizontal(drag), scale(horizontal(pullVelocity), 0.12)), FORWARD);

    for (const definition of SEGMENTS) {
      const pose = transferPoses.get(definition.id)!;
      const poseCarry = scale(clampLength(pose.linearVelocity, 2.4), 0.35);
      const directionalCarry = scale(
        fallDirection,
        definition.id === this.activeGrab?.segment ? 0.7 : 0.24,
      );
      const boundedVelocity = clampLength(add(poseCarry, directionalCarry), 1.9);
      const inheritedVelocity = {
        ...boundedVelocity,
        y: clamp(boundedVelocity.y, -1.2, 0.8),
      };
      const inheritedAngularVelocity = clampLength(scale(pose.angularVelocity, 0.18), 2.8);
      const body = this.world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(pose.position.x, pose.position.y, pose.position.z)
          .setRotation(pose.rotation)
          .setLinvel(inheritedVelocity.x, inheritedVelocity.y, inheritedVelocity.z)
          .setAngvel(inheritedAngularVelocity)
          .setLinearDamping(0.32)
          .setAngularDamping(0.42)
          .setCanSleep(true)
          .setCcdEnabled(true)
          .setAdditionalSolverIterations(3),
      );
      const collider = this.world.createCollider(
        segmentCollider(definition)
          .setMass(definition.massKg)
          .setFriction(0.82)
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
    if (this.activeGrab) {
      this.activeGrab.targetVelocity = ZERO;
      this.grabController = new GrabAnchorController(this.activeGrab.target);
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
    this.appliedGrabForceN = 0;
    this.grabControlDiagnostics = emptyGrabDiagnostics(this.grabControlDiagnostics.cumulativeInjectedWorkJ);
    if (this.activeGrab) {
      const body = this.ragdollBodies.get(this.activeGrab.segment);
      if (body) {
        this.grabController ??= new GrabAnchorController(this.activeGrab.target);
        this.grabControlDiagnostics = this.grabController.apply(
          body, this.activeGrab.localAnchor, this.activeGrab.target, dt,
        );
        this.appliedGrabForceN = length(this.grabControlDiagnostics.force);
      }
    }
    this.world.step();
    this.readRagdollPoses();
    this.grabControlDiagnostics.storedUserForceN = [...this.ragdollBodies.values()].reduce(
      (sum, body) => sum + length(body.userForce()), 0,
    );
    this.grabControlDiagnostics.storedUserTorqueNm = [...this.ragdollBodies.values()].reduce(
      (sum, body) => sum + length(body.userTorque()), 0,
    );
    this.fallingTime += dt;
    const pelvis = this.ragdollBodies.get("pelvis")!;
    this.grounded = this.maximumFloorPenetration() > 0 || [...this.poses.values()].some(
      (pose) => pose.position.y < 0.3,
    );
    const pelvisSpeed = length(pelvis.linvel());
    const torsoDown = this.grounded && pelvis.translation().y < 0.78 && pelvisSpeed < 1.4;
    const settledOnContact = this.fallingTime > 1.25 && this.grounded && pelvisSpeed < 0.9;
    if (this.fallingTime > 0.45 && (torsoDown || settledOnContact)) this.state = "fallen";
    this.leanRadians = this.currentTorsoLean();
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

  private createRootMotor(): void {
    this.rootBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
        INITIAL_ROOT.x,
        INITIAL_ROOT.y,
        INITIAL_ROOT.z,
      ),
    );
    this.rootCollider = this.world.createCollider(
      RAPIER.ColliderDesc.capsule(0.725, 0.24)
        .setFriction(0.9)
        .setCollisionGroups(CHARACTER_GROUP),
      this.rootBody,
    );
    this.motor = this.world.createCharacterController(0.012);
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

  private supportReach(): number {
    const root = this.rootBody?.translation() ?? this.poses.get("pelvis")!.position;
    const center = midpoint(this.supportFeet.leftFoot, this.supportFeet.rightFoot);
    return Math.hypot(root.x - center.x, root.z - center.z);
  }

  private supportSnapshot(): SupportState {
    if (this.state === "falling" || this.state === "fallen") {
      return { planted: [], swingFoot: null, stepProgress: 0, grounded: this.grounded };
    }
    let swing = this.step?.foot ?? null;
    if (!swing && this.activeGrab && (this.activeGrab.region === "leftFoot" || this.activeGrab.region === "rightFoot")) {
      if (length(sub(this.activeGrab.target, this.activeGrab.startTarget)) > 0.06) swing = this.activeGrab.region;
    }
    const planted = (["leftFoot", "rightFoot"] as const).filter((foot) => foot !== swing);
    return {
      planted,
      swingFoot: swing,
      stepProgress: this.step ? clamp(this.step.elapsed / this.step.duration, 0, 1) : 0,
      grounded: this.grounded,
    };
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
    let penetration = 0;
    for (const definition of SEGMENTS) {
      const pose = this.poses.get(definition.id);
      if (!pose) continue;
      let verticalExtent: number;
      if (definition.shape.kind === "sphere") {
        verticalExtent = definition.shape.radius;
      } else if (definition.shape.kind === "capsule") {
        verticalExtent = Math.abs(rotate(pose.rotation, UP).y) * definition.shape.halfHeight
          + definition.shape.radius;
      } else {
        const xAxis = rotate(pose.rotation, { x: 1, y: 0, z: 0 });
        const yAxis = rotate(pose.rotation, UP);
        const zAxis = rotate(pose.rotation, FORWARD);
        verticalExtent = Math.abs(xAxis.y) * definition.shape.halfExtents.x
          + Math.abs(yAxis.y) * definition.shape.halfExtents.y
          + Math.abs(zAxis.y) * definition.shape.halfExtents.z;
      }
      penetration = Math.max(penetration, Math.max(0, verticalExtent - pose.position.y));
    }
    return penetration;
  }

  private currentTorsoLean(): number {
    const torso = this.poses.get("torso");
    if (!torso) return 0;
    return Math.acos(clamp(dot(normalize(rotate(torso.rotation, UP)), UP), -1, 1));
  }
}

export async function createEmbodiedCharacter(
  initialRenderer: RendererMode = "canvas2d",
): Promise<CharacterController> {
  await ensureRapier();
  return new EmbodiedCharacter(initialRenderer);
}
