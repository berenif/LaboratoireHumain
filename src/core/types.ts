import type { GrabControlDiagnostics, HandoffDiagnostics } from "../character/GrabAnchorController";

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

export type MotionState = "upright" | "reacting" | "stepping" | "falling" | "fallen";
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

export interface DiagnosticsSnapshot {
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
  fixedUpdate(dt: number, command: GrabCommand | null): void;
  getSnapshot(renderer: RendererMode): PoseSnapshot;
  pick(ray: Ray): PickResult | null;
  pause(): void;
  resume(): void;
  reset(): void;
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
