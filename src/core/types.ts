export const REGION_IDS = [
  "head",
  "torso",
  "pelvis",
  "leftHand",
  "rightHand",
  "leftFoot",
  "rightFoot",
] as const;

export type RegionId = (typeof REGION_IDS)[number];
export type SegmentId =
  | RegionId
  | "neck"
  | "leftUpperArm"
  | "leftForearm"
  | "rightUpperArm"
  | "rightForearm"
  | "leftThigh"
  | "leftShin"
  | "rightThigh"
  | "rightShin";

export type MotionState = "upright" | "reacting" | "stepping" | "falling" | "fallen" | "recovering";
export type RendererMode = "webgl" | "canvas2d";
export type Vec3 = Readonly<{ x: number; y: number; z: number }>;
export type Quat = Readonly<{ x: number; y: number; z: number; w: number }>;

export type SegmentShape =
  | Readonly<{ kind: "capsule"; radius: number; halfHeight: number }>
  | Readonly<{ kind: "box"; halfExtents: Vec3 }>
  | Readonly<{ kind: "sphere"; radius: number }>;

export interface SegmentDefinition {
  id: SegmentId;
  parent: SegmentId | null;
  region: RegionId | null;
  massKg: number;
  shape: SegmentShape;
  localOffset: Vec3;
  restLocalRotation: Quat;
  jointAnchorParent: Vec3 | null;
  jointAnchorChild: Vec3 | null;
  jointLimitRadians: Readonly<{ x: number; y: number; z: number }> | null;
  collisionGroup: number;
}

export interface SegmentPose {
  id: SegmentId;
  position: Vec3;
  rotation: Quat;
  linearVelocity: Vec3;
  angularVelocity: Vec3;
}

export interface SupportState {
  planted: ReadonlyArray<"leftFoot" | "rightFoot">;
  swingFoot: "leftFoot" | "rightFoot" | null;
  stepProgress: number;
  grounded: boolean;
}

export interface GrabControlDiagnostics {
  active: boolean;
  rawTarget: Vec3;
  controlTarget: Vec3;
  targetVelocity: Vec3;
  targetError: Vec3;
  anchorWorld: Vec3;
  anchorVelocity: Vec3;
  force: Vec3;
  impulse: Vec3;
  torque: Vec3;
  angularImpulse: Vec3;
  bodyLinearVelocity: Vec3;
  bodyAngularVelocity: Vec3;
  effectiveMassKg: number;
  targetSpeedMps: number;
  selectedAnchorErrorM: number;
  injectedWorkJ: number;
  cumulativeInjectedWorkJ: number;
  storedUserForceN: number;
  storedUserTorqueNm: number;
  linearImpulseLimitNs: number;
  angularImpulseLimitNms: number;
  positiveWorkLimitJ: number;
}

export interface HandoffDiagnostics {
  sequence: number;
  maxTranslationErrorM: number;
  maxAngularErrorDegrees: number;
  selectedAnchorErrorM: number;
  rawTargetErrorM: number;
  localAnchorErrorM: number;
  jointSeparationM: number;
  targetDerivativeSpeedMps: number;
}

export type RecoveryPhase = "none" | "protect" | "settle" | "roll" | "brace" | "kneel" | "stand";
export interface SupportingContact {
  segment: SegmentId;
  normalY: number;
  forceN: number;
  persistenceS: number;
  point: Vec3;
  loadBearing: boolean;
}
export interface RecoveryDiagnostics {
  phase: RecoveryPhase;
  orientation: "forward" | "backward" | "left" | "right";
  contacts: SupportingContact[];
  phaseTimeS: number;
  settledTimeS: number;
  stableTimeS: number;
  stalledTimeS: number;
  retries: number;
  assistanceForce: Vec3;
  assistanceTorque: Vec3;
  assistanceForceCapN: number;
  assistanceTorqueCapNm: number;
  maxMotorTorqueNm: number;
  supporting: SegmentId[];
}
export interface BalanceStateDiagnostics {
  centerOfMass: Vec3; centerOfMassVelocity: Vec3; capturePoint: Vec3; supportCenter: Vec3;
  supportingFeet: ("leftFoot" | "rightFoot")[]; supportMarginM: number; instabilitySeconds: number;
  recoveryCapacityM: number; externalForce: Vec3; balanceAcceleration: Vec3; stepTarget: Vec3 | null;
}
export interface DiagnosticsSnapshot {
  balance: BalanceStateDiagnostics | null;
  bodyInputAvailable: boolean;
  recovery: RecoveryDiagnostics;
  grabControl: GrabControlDiagnostics;
  handoff: HandoffDiagnostics | null;
  authority: "character-motor" | "ragdoll";
  state: MotionState;
  simulationReady: boolean;
  interactiveViewReady: boolean;
  renderer: RendererMode;
  activeGrab: boolean;
  activePointerId: number | null;
  selectedRegion: RegionId | null;
  queuedTarget: boolean;
  appliedGrabForceN: number;
  leanRadians: number;
  rootDisplacementM: number;
  maxJointSeparationM: number;
  maxFloorPenetrationM: number;
  stepCount: number;
  support: SupportState;
  fixedSteps: number;
  droppedTimeMs: number;
  finite: boolean;
  errors: ReadonlyArray<string>;
}

export interface PoseSnapshot {
  sequence: number;
  simulationTime: number;
  state: MotionState;
  rootPosition: Vec3;
  rootRotation: Quat;
  segments: ReadonlyArray<SegmentPose>;
  support: SupportState;
  diagnostics: DiagnosticsSnapshot;
}

export interface CameraState {
  position: Vec3;
  target: Vec3;
  up: Vec3;
  fovYRadians: number;
  near: number;
  far: number;
  viewportWidth: number;
  viewportHeight: number;
}

export interface Ray {
  origin: Vec3;
  direction: Vec3;
}

export interface PickResult {
  region: RegionId;
  segment: SegmentId;
  localAnchor: Vec3;
  worldPoint: Vec3;
  distance: number;
}

export interface GrabCommand {
  kind: "begin" | "move" | "end" | "cancel";
  pointerId: number;
  region?: RegionId;
  segment?: SegmentId;
  localAnchor?: Vec3;
  worldTarget?: Vec3;
  timestampMs: number;
}

export interface CameraProjection {
  reset?(): void;
  getState(): CameraState;
  screenToRay(clientX: number, clientY: number, rect: DOMRectReadOnly): Ray;
  project(worldPoint: Vec3): Readonly<{ x: number; y: number; depth: number; visible: boolean }>;
  intersectDragPlane(ray: Ray, planePoint: Vec3, planeNormal: Vec3): Vec3 | null;
}

export interface PoseView {
  readonly mode: RendererMode;
  mount(container: HTMLElement): void;
  setSnapshot(previous: PoseSnapshot, current: PoseSnapshot, alpha: number): void;
  render(): void;
  resize(width: number, height: number, devicePixelRatio: number): void;
  dispose(): void;
  getProjection(): CameraProjection;
}

export interface CharacterController {
  clearBodyInput(): void;
  fixedUpdate(dt: number, command: GrabCommand | null): void;
  getSnapshot(renderer: RendererMode): PoseSnapshot;
  pick(ray: Ray): PickResult | null;
  pause(): void;
  resume(): void;
  reset(): void;
  dispose(): void;
  diagnostics(): DiagnosticsSnapshot;
}

export interface InteractionController {
  attach(
    element: HTMLElement,
    projection: CameraProjection,
    pick: (ray: Ray) => PickResult | null,
  ): void;
  detach(): void;
  consumeCommand(): GrabCommand | null;
  clear(reason: "release" | "cancel" | "lost-capture" | "blur" | "pause" | "reset"): void;
  reset(): void;
  getActivePointerId(): number | null;
}

export const WORLD = Object.freeze({
  units: "metres",
  upAxis: "+Y",
  forwardAxis: "+Z",
  fixedHz: 60,
  fixedDt: 1 / 60,
  maxCatchUpSteps: 5,
});
