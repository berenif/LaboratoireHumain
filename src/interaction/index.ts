import type {
  CameraProjection,
  GrabCommand,
  InteractionController,
  PickResult,
  RegionId,
  SegmentId,
  Vec3,
} from "../core/types";

export type InteractionClearReason = Parameters<InteractionController["clear"]>[0];

export interface InteractionStatus {
  activePointerId: number | null;
  selectedRegion: RegionId | null;
  selectedSegment: SegmentId | null;
  localAnchor: Vec3 | null;
  worldTarget: Vec3 | null;
  dragDistanceM: number;
  hasQueuedCommand: boolean;
  ignoredSecondaryPointers: number;
}

export interface PointerInteractionOptions {
  onStatusChange?: (status: Readonly<InteractionStatus>) => void;
  now?: () => number;
}

interface ActiveGrab {
  pointerId: number;
  region: RegionId;
  segment: SegmentId;
  localAnchor: Vec3;
  pickedWorldPoint: Vec3;
  worldTarget: Vec3;
  dragPlanePoint: Vec3;
  dragPlaneNormal: Vec3;
}

const CAPTURE_LISTENER = { capture: true } as const;
const ACTIVE_LISTENER = { capture: true, passive: false } as const;

const cloneVec3 = (value: Vec3): Vec3 => ({
  x: value.x,
  y: value.y,
  z: value.z,
});

const distance = (a: Vec3, b: Vec3): number =>
  Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

const normalizedFromTo = (from: Vec3, to: Vec3): Vec3 => {
  const x = to.x - from.x;
  const y = to.y - from.y;
  const z = to.z - from.z;
  const magnitude = Math.hypot(x, y, z);

  // A camera looking straight through its target always has a usable forward
  // vector. The fallback keeps synthetic harness cameras deterministic.
  if (magnitude < 1e-8 || !Number.isFinite(magnitude)) {
    return { x: 0, y: 0, z: 1 };
  }

  return { x: x / magnitude, y: y / magnitude, z: z / magnitude };
};

/**
 * Renderer-independent pointer input.
 *
 * The interaction keeps a camera-facing drag plane fixed for the full grab.
 * `move` events are coalesced, while `end` and `cancel` pre-empt every older
 * queued command so a fixed update can never apply a stale grab after input
 * has ended.
 */
export class PointerInteraction implements InteractionController {
  private element: HTMLElement | null = null;
  private projection: CameraProjection | null = null;
  private pick: ((ray: ReturnType<CameraProjection["screenToRay"]>) => PickResult | null) | null = null;
  private ownerWindow: Window | null = null;
  private activeGrab: ActiveGrab | null = null;

  private pendingBegin: GrabCommand | null = null;
  private pendingMove: GrabCommand | null = null;
  private pendingTerminal: GrabCommand | null = null;

  private ignoredSecondaryPointers = 0;
  private previousTouchAction = "";
  private previousUserSelect = "";
  private readonly onStatusChange?: PointerInteractionOptions["onStatusChange"];
  private readonly now: () => number;

  constructor(options: PointerInteractionOptions = {}) {
    this.onStatusChange = options.onStatusChange;
    this.now = options.now ?? (() => globalThis.performance?.now?.() ?? Date.now());
  }

  attach(
    element: HTMLElement,
    projection: CameraProjection,
    pick: (ray: ReturnType<CameraProjection["screenToRay"]>) => PickResult | null,
  ): void {
    if (this.element === element && this.projection === projection && this.pick === pick) {
      return;
    }

    this.detach();
    this.element = element;
    this.projection = projection;
    this.pick = pick;
    this.ownerWindow = element.ownerDocument.defaultView;

    this.previousTouchAction = element.style.touchAction;
    this.previousUserSelect = element.style.userSelect;
    element.style.touchAction = "none";
    element.style.userSelect = "none";

    element.addEventListener("pointerdown", this.handlePointerDown, ACTIVE_LISTENER);
    element.addEventListener("pointermove", this.handlePointerMove, ACTIVE_LISTENER);
    element.addEventListener("pointerup", this.handlePointerUp, ACTIVE_LISTENER);
    element.addEventListener("pointercancel", this.handlePointerCancel, ACTIVE_LISTENER);
    element.addEventListener("lostpointercapture", this.handleLostPointerCapture, CAPTURE_LISTENER);
    element.addEventListener("contextmenu", this.handleContextMenu, ACTIVE_LISTENER);
    this.ownerWindow?.addEventListener("blur", this.handleWindowBlur, CAPTURE_LISTENER);
    this.ownerWindow?.addEventListener("pointerup", this.handleWindowPointerEnd, CAPTURE_LISTENER);
    this.ownerWindow?.addEventListener("pointercancel", this.handleWindowPointerEnd, CAPTURE_LISTENER);
    this.emitStatus();
  }

  detach(): void {
    const element = this.element;
    if (!element) {
      return;
    }

    // Cancelling before listeners are removed ensures a renderer switch cannot
    // leave a contribution active in the simulation.
    this.clear("cancel");
    element.removeEventListener("pointerdown", this.handlePointerDown, CAPTURE_LISTENER);
    element.removeEventListener("pointermove", this.handlePointerMove, CAPTURE_LISTENER);
    element.removeEventListener("pointerup", this.handlePointerUp, CAPTURE_LISTENER);
    element.removeEventListener("pointercancel", this.handlePointerCancel, CAPTURE_LISTENER);
    element.removeEventListener("lostpointercapture", this.handleLostPointerCapture, CAPTURE_LISTENER);
    element.removeEventListener("contextmenu", this.handleContextMenu, CAPTURE_LISTENER);
    this.ownerWindow?.removeEventListener("blur", this.handleWindowBlur, CAPTURE_LISTENER);
    this.ownerWindow?.removeEventListener("pointerup", this.handleWindowPointerEnd, CAPTURE_LISTENER);
    this.ownerWindow?.removeEventListener("pointercancel", this.handleWindowPointerEnd, CAPTURE_LISTENER);

    element.style.touchAction = this.previousTouchAction;
    element.style.userSelect = this.previousUserSelect;
    this.element = null;
    this.projection = null;
    this.pick = null;
    this.ownerWindow = null;
    this.emitStatus();
  }

  consumeCommand(): GrabCommand | null {
    // Terminal commands deliberately win. If a complete tap begins and ends
    // between fixed updates, the simulation sees the end/cancel rather than a
    // one-frame ghost grab.
    if (this.pendingTerminal) {
      const command = this.pendingTerminal;
      this.pendingTerminal = null;
      this.pendingBegin = null;
      this.pendingMove = null;
      this.emitStatus();
      return command;
    }

    if (this.pendingBegin) {
      const command = this.pendingBegin;
      this.pendingBegin = null;
      this.emitStatus();
      return command;
    }

    if (this.pendingMove) {
      const command = this.pendingMove;
      this.pendingMove = null;
      this.emitStatus();
      return command;
    }

    return null;
  }

  clear(reason: InteractionClearReason): void {
    const grab = this.activeGrab;
    const pointerId = grab?.pointerId ?? this.pendingBegin?.pointerId ?? null;

    this.activeGrab = null;
    this.pendingBegin = null;
    this.pendingMove = null;

    if (pointerId !== null) {
      const kind = reason === "release" ? "end" : "cancel";
      this.pendingTerminal = this.makeCommand(kind, pointerId, grab ?? undefined);
      this.releaseCapture(pointerId);
    } else if (reason === "pause" || reason === "reset") {
      // No local pointer means there is no interaction contribution to cancel.
      // Clear any already-consumed terminal too, so reset cannot replay input.
      this.pendingTerminal = null;
    }

    this.emitStatus();
  }

  reset(): void {
    // A reset clears target/begin/move data but retains one explicit cancel for
    // the next fixed update when a grab was active. Character reset can consume
    // it harmlessly; a delayed character reset still cannot receive stale force.
    this.clear("reset");
    this.ignoredSecondaryPointers = 0;
    this.emitStatus();
  }

  getActivePointerId(): number | null {
    return this.activeGrab?.pointerId ?? null;
  }

  getStatus(): Readonly<InteractionStatus> {
    const grab = this.activeGrab;
    return Object.freeze({
      activePointerId: grab?.pointerId ?? null,
      selectedRegion: grab?.region ?? null,
      selectedSegment: grab?.segment ?? null,
      localAnchor: grab ? cloneVec3(grab.localAnchor) : null,
      worldTarget: grab ? cloneVec3(grab.worldTarget) : null,
      dragDistanceM: grab ? distance(grab.pickedWorldPoint, grab.worldTarget) : 0,
      hasQueuedCommand: Boolean(this.pendingTerminal || this.pendingBegin || this.pendingMove),
      ignoredSecondaryPointers: this.ignoredSecondaryPointers,
    });
  }

  private readonly handlePointerDown = (event: PointerEvent): void => {
    if (!this.element || !this.projection || !this.pick) {
      return;
    }

    if (this.activeGrab || !this.isPrimaryPointer(event)) {
      this.ignoredSecondaryPointers += 1;
      if (this.activeGrab) {
        this.blockCameraGesture(event);
      }
      this.emitStatus();
      return;
    }

    if (event.pointerType === "mouse" && event.button !== 0) {
      return;
    }

    const rect = this.element.getBoundingClientRect();
    const ray = this.projection.screenToRay(event.clientX, event.clientY, rect);
    const hit = this.pick(ray);
    if (!hit) {
      return;
    }

    const camera = this.projection.getState();
    const worldPoint = cloneVec3(hit.worldPoint);
    const grab: ActiveGrab = {
      pointerId: event.pointerId,
      region: hit.region,
      segment: hit.segment,
      localAnchor: cloneVec3(hit.localAnchor),
      pickedWorldPoint: worldPoint,
      worldTarget: worldPoint,
      dragPlanePoint: worldPoint,
      dragPlaneNormal: normalizedFromTo(camera.target, camera.position),
    };

    this.activeGrab = grab;
    this.pendingTerminal = null;
    this.pendingMove = null;
    this.pendingBegin = this.makeCommand("begin", event.pointerId, grab, event.timeStamp);

    try {
      this.element.setPointerCapture(event.pointerId);
    } catch {
      // Synthetic input and older touch implementations can reject capture.
      // Window blur and element-level lifecycle events still cancel the grab.
    }

    this.blockCameraGesture(event);
    this.emitStatus();
  };

  private readonly handlePointerMove = (event: PointerEvent): void => {
    const grab = this.activeGrab;
    if (!grab || event.pointerId !== grab.pointerId || !this.element || !this.projection) {
      if (grab) {
        this.blockCameraGesture(event);
      }
      return;
    }

    const rect = this.element.getBoundingClientRect();
    const ray = this.projection.screenToRay(event.clientX, event.clientY, rect);
    const worldTarget = this.projection.intersectDragPlane(
      ray,
      grab.dragPlanePoint,
      grab.dragPlaneNormal,
    );

    if (worldTarget && this.isFiniteVec3(worldTarget)) {
      grab.worldTarget = cloneVec3(worldTarget);
      this.pendingMove = this.makeCommand("move", event.pointerId, grab, event.timeStamp);
    }

    this.blockCameraGesture(event);
    this.emitStatus();
  };

  private readonly handlePointerUp = (event: PointerEvent): void => {
    if (event.pointerId !== this.activeGrab?.pointerId) {
      if (this.activeGrab) {
        this.blockCameraGesture(event);
      }
      return;
    }
    this.blockCameraGesture(event);
    this.finish("release", event.timeStamp);
  };

  private readonly handlePointerCancel = (event: PointerEvent): void => {
    if (event.pointerId !== this.activeGrab?.pointerId) {
      if (this.activeGrab) {
        this.blockCameraGesture(event);
      }
      return;
    }
    this.blockCameraGesture(event);
    this.finish("cancel", event.timeStamp);
  };

  private readonly handleLostPointerCapture = (event: PointerEvent): void => {
    if (event.pointerId === this.activeGrab?.pointerId) {
      this.finish("lost-capture", event.timeStamp);
    }
  };

  private readonly handleWindowBlur = (): void => {
    this.clear("blur");
  };

  private readonly handleWindowPointerEnd = (event: PointerEvent): void => {
    if (event.pointerId !== this.activeGrab?.pointerId) return;
    // Native pointer capture normally routes the end to the host. This handles
    // release outside it when capture is unavailable, without double dispatch.
    if (event.target instanceof Node && this.element?.contains(event.target)) return;
    this.finish(event.type === "pointerup" ? "release" : "cancel", event.timeStamp);
  };

  private readonly handleContextMenu = (event: MouseEvent): void => {
    if (this.activeGrab) {
      event.preventDefault();
      event.stopImmediatePropagation();
    }
  };

  private finish(reason: "release" | "cancel" | "lost-capture", timestampMs: number): void {
    const grab = this.activeGrab;
    if (!grab) {
      return;
    }

    this.activeGrab = null;
    this.pendingBegin = null;
    this.pendingMove = null;
    this.pendingTerminal = this.makeCommand(
      reason === "release" ? "end" : "cancel",
      grab.pointerId,
      grab,
      timestampMs,
    );
    this.releaseCapture(grab.pointerId);
    this.emitStatus();
  }

  private makeCommand(
    kind: GrabCommand["kind"],
    pointerId: number,
    grab?: ActiveGrab,
    timestampMs?: number,
  ): GrabCommand {
    return {
      kind,
      pointerId,
      region: grab?.region,
      segment: grab?.segment,
      localAnchor: grab ? cloneVec3(grab.localAnchor) : undefined,
      worldTarget: grab ? cloneVec3(grab.worldTarget) : undefined,
      timestampMs: Number.isFinite(timestampMs) ? timestampMs! : this.now(),
    };
  }

  private releaseCapture(pointerId: number): void {
    const element = this.element;
    if (!element) {
      return;
    }

    try {
      if (element.hasPointerCapture(pointerId)) {
        element.releasePointerCapture(pointerId);
      }
    } catch {
      // Losing an element or synthetic capture during teardown is already a
      // cancelled interaction; no follow-up action is needed.
    }
  }

  private isPrimaryPointer(event: PointerEvent): boolean {
    // Programmatic PointerEvents created without pointer metadata use an empty
    // pointerType. Treat that single stream as primary so harness replays can
    // still pass through the real DOM listeners.
    return event.isPrimary || event.pointerType === "";
  }

  private blockCameraGesture(event: Event): void {
    if (event.cancelable) {
      event.preventDefault();
    }
    event.stopImmediatePropagation();
  }

  private isFiniteVec3(value: Vec3): boolean {
    return Number.isFinite(value.x) && Number.isFinite(value.y) && Number.isFinite(value.z);
  }

  private emitStatus(): void {
    this.onStatusChange?.(this.getStatus());
  }
}

export default PointerInteraction;
