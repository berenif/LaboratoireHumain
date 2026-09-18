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
export const SEGMENT_IDS = [
  "pelvis",
  "lumbar",
  "torso",
  "neck",
  "head",
  "leftShoulderGirdle",
  "leftUpperArm",
  "leftForearm",
  "leftForearmTwist",
  "leftHand",
  "rightShoulderGirdle",
  "rightUpperArm",
  "rightForearm",
  "rightForearmTwist",
  "rightHand",
  "leftThigh",
  "leftShin",
  "leftAnkle",
  "leftFoot",
  "leftForefoot",
  "rightThigh",
  "rightShin",
  "rightAnkle",
  "rightFoot",
  "rightForefoot",
] as const;

export type SegmentId = (typeof SEGMENT_IDS)[number];

export type MotionState = "upright" | "reacting" | "stepping" | "falling" | "fallen" | "recovering";
export type RendererMode = "webgl" | "canvas2d";
export type Vec3 = Readonly<{ x: number; y: number; z: number }>;
export type Quat = Readonly<{ x: number; y: number; z: number; w: number }>;

export type SegmentSide = "left" | "right" | null;
export type SegmentRole =
  | "pelvis"
  | "lumbar"
  | "ribcage"
  | "neck"
  | "head"
  | "shoulder-girdle"
  | "upper-arm"
  | "forearm"
  | "forearm-twist"
  | "hand"
  | "thigh"
  | "shin"
  | "ankle"
  | "hindfoot"
  | "forefoot";

export type JointCoordinate = "x" | "y" | "z";

export interface JointFrame {
  anchor: Vec3;
  rotation: Quat;
}

export interface JointAxisProfile {
  coordinate: JointCoordinate;
  minRadians: number;
  maxRadians: number;
  passiveStiffnessNmPerRad: number;
  dampingNmsPerRad: number;
  maxMotorTorqueNm: number;
}

export interface JointProfile {
  kind: "hinge" | "multi-axis";
  parentFrame: JointFrame;
  childFrame: JointFrame;
  axes: readonly JointAxisProfile[];
  limitSoftZoneFraction: number;
}

/** Canonical local-space surface shared by collision, rendering, and picking. */
export interface ConvexGeometry {
  kind: "convex";
  vertices: readonly Vec3[];
  triangles: readonly (readonly [number, number, number])[];
  localBounds: Readonly<{ min: Vec3; max: Vec3 }>;
  supportPatch?: readonly Vec3[];
}

export interface SegmentDefinition {
  id: SegmentId;
  parent: SegmentId | null;
  region: RegionId | null;
  side: SegmentSide;
  role: SegmentRole;
  massKg: number;
  geometry: ConvexGeometry;
  localOffset: Vec3;
  restLocalRotation: Quat;
  jointAnchorParent: Vec3 | null;
  jointAnchorChild: Vec3 | null;
  jointProfile: JointProfile | null;
  collisionExclusions: readonly SegmentId[];
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

export type RecoveryPhase = "none" | "protect" | "settle" | "roll" | "brace" | "kneel" | "stand";
export interface SupportingContact {
  segment: SegmentId;
  normalY: number;
  forceN: number;
  persistenceS: number;
  point: Vec3;
  /** Copied world-space solver contact patch. */
  points?: readonly Vec3[];
  loadBearing: boolean;
}
export interface RecoveryTransferGuardDiagnostics {
  segment: SegmentId;
  hasContact: boolean;
  persistenceS: number;
  forceN: number;
  footUpDot: number;
  footHeightM: number;
  targetDistanceM: number | null;
  supportMarginM: number;
  massVelocityYMps: number;
  leadingLoadN: number;
  readyToLift: boolean;
  qualified: boolean;
}
export interface RecoveryDiagnostics {
  phase: RecoveryPhase;
  route: "none" | "crouch" | "half-kneel" | "prone" | "roll";
  leadingSide: "left" | "right" | null;
  rollSide: "left" | "right" | null;
  /** Measured axis/heading captured in settle for roll-aware planning. */
  recoveryHeading?: number | null;
  /** Planned recovery axis in world frame captured during settle. */
  recoveryAxis?: Vec3;
  /** Planned support segments that are currently being chased in recovery. */
  plannedSupportSources?: SegmentId[];
  /** Established support currently allowed for transfer/authority decisions. */
  establishedSupportSources?: SegmentId[];
  transferStage: "none" | "roll" | "arm-preparation" | "push-brace" | "brace" | "tuck-knee" | "plant-lead" | "shift-weight" | "bring-trailing" | "extend" | "relax";
  transferGuard?: RecoveryTransferGuardDiagnostics | null;
  supportMarginM: number;
  centerOfMass: Vec3;
  projectedCenterOfMass: Vec3;
  plantedTargets: { segment: SegmentId; position: Vec3; rotation: Quat; driftM: number }[];
  releasedSupports: SegmentId[];
  releaseMarginM: number | null;
  extension: number;
  progressError: number | null;
  stageAction?: string;
  blockingPredicate?: string | null;
  progressMeasure?: string;
  noSupportTimeS: number;
  orientation: "forward" | "backward" | "left" | "right";
  contacts: SupportingContact[];
  phaseTimeS: number;
  settledTimeS: number;
  stableTimeS: number;
  stalledTimeS: number;
  /** Reason captured when a retry is queued. */
  retryReason?: string | null;
  /** Reason captured while support is temporarily missing. */
  noSupportReason?: string | null;
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
export interface JointStateDiagnostics {
  segment: SegmentId;
  coordinates: Vec3;
  targetCoordinates: Vec3;
  limitError: Vec3;
  limitErrorMagnitudeRad: number;
  motorTorqueWorld: Vec3;
  motorTorqueNm: number;
  motorSaturationRatio: number;
}
export interface ContactStateDiagnostics {
  /** Copied, measured contacts in standing as well as recovery. */
  contacts: SupportingContact[];
  count: number;
  loadBearingCount: number;
  totalNormalForceN: number;
  supportingSegments: SegmentId[];
}
/** Read-only world-space targets and FK reconstruction, never physics state. */
export interface StandingChainPose {
  id: SegmentId;
  position: Vec3;
  rotation: Quat;
  forward: Vec3;
}
export interface StandingLegReach {
  requestedAnkle: Vec3;
  radiusM: number;
  distanceM: number;
  excessM: number;
  /** Radial reach only; does not certify joint limits or load-bearing contact. */
  radiallyReachable: boolean;
}
export interface StandingChainDiagnostics {
  frame: "world-hindfoot-center";
  motorSampleTimeS: number;
  physicalSampleTimeS: number;
  reconstruction: "local-commands-on-sampled-physical-pelvis";
  pelvis: StandingChainPose | null;
  desiredPelvis: StandingChainPose | null;
  motorInputPelvis: StandingChainPose | null;
  /** Actual shin-to-ankle joint, not the hindfoot centre or ankle-to-foot joint. */
  stanceAnkle: Vec3 | null;
  supportPoints: Vec3[];
  supportPolygon: Vec3[];
  step: {
    foot: "leftFoot" | "rightFoot";
    from: Vec3;
    to: Vec3;
    requested: Vec3;
    rebased: Vec3;
    heading: number;
    elapsedS: number;
    durationS: number;
  } | null;
  legs: Array<{
    side: "left" | "right";
    hip: Vec3;
    landingReach: StandingLegReach | null;
    segments: Array<{
      id: SegmentId;
      physical: StandingChainPose | null;
      desired: StandingChainPose | null;
      commanded: StandingChainPose | null;
      targetLocalRotation: Quat | null;
      desiredErrorM: number | null;
      commandFrameErrorM: number | null;
    }>;
  }>;
  armForward: Array<{
    id: SegmentId;
    physical: StandingChainPose | null;
    desired: StandingChainPose | null;
  }>;
}
export interface DiagnosticsSnapshot {
  standingChain: StandingChainDiagnostics | null;
  balance: BalanceStateDiagnostics | null;
  bodyInputAvailable: boolean;
  recovery: RecoveryDiagnostics;
  grabControl: GrabControlDiagnostics;
  /** The continuous dynamic assembly owns every rendered transform in every state. */
  physicsOwnership: "rapier-dynamic";
  jointDiagnostics: JointStateDiagnostics[];
  contactDiagnostics: ContactStateDiagnostics;
  maxJointLimitErrorRad: number;
  maxMotorSaturationRatio: number;
  state: MotionState;
  simulationReady: boolean;
  interactiveViewReady: boolean;
  renderer: RendererMode;
  activeGrab: boolean;
  activePointerId: number | null;
  selectedRegion: RegionId | null;
  /** Exact picked rigid segment; region remains the seven-item UI grouping. */
  selectedSegment?: SegmentId | null;
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
