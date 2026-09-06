import { V3 } from "../core/math";
import type { Vec3 } from "../core/types";
import { SharedCameraProjection } from "./camera";

export type CameraControlAction = "orbit" | "pan" | "zoom";

export interface CameraControlsOptions {
  /** Prevent a camera gesture from starting, for example while a body grab owns the pointer. */
  readonly canStartPointer?: (event: PointerEvent, action: Exclude<CameraControlAction, "zoom">) => boolean;
  /** Prevent wheel zoom, for example while a body grab is active. */
  readonly canZoomWheel?: (event: WheelEvent) => boolean;
  /** Single-touch orbit should only be enabled when blank-space/body-pick arbitration is installed. */
  readonly allowSingleTouchOrbit?: boolean;
  readonly orbitRadiansPerPixel?: number;
  readonly wheelZoomExponentPerPixel?: number;
  readonly minDistance?: number;
  readonly maxDistance?: number;
  readonly minPolarAngleRadians?: number;
  readonly maxPolarAngleRadians?: number;
  readonly maxTargetOffset?: number;
  readonly onChange?: () => void;
}

interface PointerPosition {
  readonly x: number;
  readonly y: number;
}

interface MouseDrag {
  readonly pointerId: number;
  readonly action: "orbit" | "pan";
  position: PointerPosition;
}

interface TouchReference {
  readonly centroid: PointerPosition;
  readonly distance: number;
}

const WORLD_UP: Vec3 = Object.freeze({ x: 0, y: 1, z: 0 });

function finiteOr(value: number | undefined, fallback: number): number {
  return Number.isFinite(value) ? value! : fallback;
}

function positiveOr(value: number | undefined, fallback: number): number {
  return Number.isFinite(value) && value! > 0 ? value! : fallback;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function pointerPosition(event: PointerEvent): PointerPosition {
  return { x: event.clientX, y: event.clientY };
}

/**
 * Rendering-only orbit, pan, and zoom controls for SharedCameraProjection.
 *
 * The controller never reads or writes simulation state. Because picking and
 * both render adapters consume the same projection, the visible camera and
 * interaction rays remain aligned after every camera movement. Blank-space
 * primary/right drag orbits, Shift+primary/right or middle drag pans, the
 * wheel zooms, and touch uses one-finger orbit or two-finger pan/pinch.
 */
export class OrbitCameraController {
  private readonly projection: SharedCameraProjection;
  private readonly canStartPointer: NonNullable<CameraControlsOptions["canStartPointer"]>;
  private readonly canZoomWheel: NonNullable<CameraControlsOptions["canZoomWheel"]>;
  private readonly allowSingleTouchOrbit: boolean;
  private readonly orbitRadiansPerPixel: number;
  private readonly wheelZoomExponentPerPixel: number;
  private readonly minDistance: number;
  private readonly maxDistance: number;
  private readonly minPolarAngleRadians: number;
  private readonly maxPolarAngleRadians: number;
  private readonly maxTargetOffset: number;
  private readonly onChange: () => void;
  private readonly defaultTarget: Vec3;
  private element: HTMLElement | null = null;
  private enabled = true;
  private mouseDrag: MouseDrag | null = null;
  private readonly touches = new Map<number, PointerPosition>();
  private touchReference: TouchReference | null = null;

  constructor(projection: SharedCameraProjection, options: CameraControlsOptions = {}) {
    this.projection = projection;
    this.canStartPointer = options.canStartPointer ?? (() => true);
    this.canZoomWheel = options.canZoomWheel ?? (() => true);
    this.allowSingleTouchOrbit = options.allowSingleTouchOrbit ?? false;
    this.orbitRadiansPerPixel = positiveOr(options.orbitRadiansPerPixel, 0.006);
    this.wheelZoomExponentPerPixel = positiveOr(options.wheelZoomExponentPerPixel, 0.0015);
    this.minDistance = positiveOr(options.minDistance, 1.15);
    this.maxDistance = Math.max(this.minDistance, positiveOr(options.maxDistance, 12));
    this.minPolarAngleRadians = clamp(
      finiteOr(options.minPolarAngleRadians, 0.12),
      0.01,
      Math.PI - 0.02,
    );
    this.maxPolarAngleRadians = clamp(
      finiteOr(options.maxPolarAngleRadians, Math.PI * 0.48),
      this.minPolarAngleRadians + 0.01,
      Math.PI - 0.01,
    );
    this.maxTargetOffset = positiveOr(options.maxTargetOffset, 8);
    this.onChange = options.onChange ?? (() => undefined);
    this.defaultTarget = projection.getState().target;
  }

  attach(element: HTMLElement): void {
    if (this.element === element) return;
    this.detach();
    this.element = element;
    element.addEventListener("pointerdown", this.handlePointerDown);
    element.addEventListener("pointermove", this.handlePointerMove);
    element.addEventListener("pointerup", this.handlePointerEnd);
    element.addEventListener("pointercancel", this.handlePointerEnd);
    element.addEventListener("lostpointercapture", this.handlePointerEnd);
    element.addEventListener("wheel", this.handleWheel, { passive: false });
    element.addEventListener("contextmenu", this.handleContextMenu);
    if (typeof window !== "undefined") window.addEventListener("blur", this.handleBlur);
  }

  detach(): void {
    const element = this.element;
    if (!element) return;
    element.removeEventListener("pointerdown", this.handlePointerDown);
    element.removeEventListener("pointermove", this.handlePointerMove);
    element.removeEventListener("pointerup", this.handlePointerEnd);
    element.removeEventListener("pointercancel", this.handlePointerEnd);
    element.removeEventListener("lostpointercapture", this.handlePointerEnd);
    element.removeEventListener("wheel", this.handleWheel);
    element.removeEventListener("contextmenu", this.handleContextMenu);
    if (typeof window !== "undefined") window.removeEventListener("blur", this.handleBlur);
    this.releasePointers();
    this.element = null;
  }

  setEnabled(enabled: boolean): void {
    if (this.enabled === enabled) return;
    this.enabled = enabled;
    if (!enabled) this.releasePointers();
  }

  isEnabled(): boolean {
    return this.enabled;
  }

  /** Restores the exact configured initial view while retaining the current viewport. */
  reset(): void {
    this.releasePointers();
    this.projection.reset();
    this.onChange();
  }

  dispose(): void {
    this.detach();
  }

  orbit(deltaX: number, deltaY: number): void {
    if (!Number.isFinite(deltaX) || !Number.isFinite(deltaY)) return;
    const state = this.projection.getState();
    const offset = V3.sub(state.position, state.target);
    const radius = clamp(V3.length(offset), this.minDistance, this.maxDistance);
    const theta = Math.atan2(offset.x, offset.z) - deltaX * this.orbitRadiansPerPixel;
    const currentPolar = Math.acos(clamp(offset.y / Math.max(V3.length(offset), 1e-7), -1, 1));
    const polar = clamp(
      currentPolar - deltaY * this.orbitRadiansPerPixel,
      this.minPolarAngleRadians,
      this.maxPolarAngleRadians,
    );
    const horizontalRadius = radius * Math.sin(polar);
    const position = {
      x: state.target.x + horizontalRadius * Math.sin(theta),
      y: state.target.y + radius * Math.cos(polar),
      z: state.target.z + horizontalRadius * Math.cos(theta),
    };
    this.projection.setView(position, state.target, WORLD_UP);
    this.onChange();
  }

  pan(deltaX: number, deltaY: number): void {
    if (!Number.isFinite(deltaX) || !Number.isFinite(deltaY)) return;
    const state = this.projection.getState();
    const distance = clamp(V3.length(V3.sub(state.position, state.target)), this.minDistance, this.maxDistance);
    const worldPerPixel = 2 * distance * Math.tan(state.fovYRadians * 0.5) /
      Math.max(1, state.viewportHeight);
    const basis = this.projection.getBasis();
    const translation = V3.add(
      V3.scale(basis.right, -deltaX * worldPerPixel),
      V3.scale(basis.up, deltaY * worldPerPixel),
    );
    const desiredTarget = V3.add(state.target, translation);
    const targetOffset = V3.sub(desiredTarget, this.defaultTarget);
    const offsetLength = V3.length(targetOffset);
    const boundedTarget = offsetLength > this.maxTargetOffset
      ? V3.add(this.defaultTarget, V3.scale(targetOffset, this.maxTargetOffset / offsetLength))
      : desiredTarget;
    const acceptedTranslation = V3.sub(boundedTarget, state.target);
    this.projection.setView(V3.add(state.position, acceptedTranslation), boundedTarget, WORLD_UP);
    this.onChange();
  }

  zoom(scale: number): void {
    if (!Number.isFinite(scale) || scale <= 0) return;
    const state = this.projection.getState();
    const offset = V3.sub(state.position, state.target);
    const currentDistance = Math.max(V3.length(offset), 1e-7);
    const nextDistance = clamp(currentDistance * scale, this.minDistance, this.maxDistance);
    if (Math.abs(nextDistance - currentDistance) < 1e-9) return;
    this.projection.setView(
      V3.add(state.target, V3.scale(offset, nextDistance / currentDistance)),
      state.target,
      WORLD_UP,
    );
    this.onChange();
  }

  private readonly handlePointerDown = (event: PointerEvent): void => {
    if (!this.enabled || !this.element) return;
    if (event.pointerType === "touch") {
      const action = this.allowSingleTouchOrbit ? "orbit" : "pan";
      if (!this.canStartPointer(event, action)) return;
      this.touches.set(event.pointerId, pointerPosition(event));
      this.capturePointer(event.pointerId);
      this.touchReference = this.makeTouchReference();
      if (this.allowSingleTouchOrbit || this.touches.size >= 2) event.preventDefault();
      return;
    }

    const action = this.mouseAction(event);
    if (!action || !this.canStartPointer(event, action)) return;
    this.mouseDrag = { pointerId: event.pointerId, action, position: pointerPosition(event) };
    this.capturePointer(event.pointerId);
    event.preventDefault();
  };

  private readonly handlePointerMove = (event: PointerEvent): void => {
    if (!this.enabled) return;
    if (event.pointerType === "touch" && this.touches.has(event.pointerId)) {
      const previous = this.touchReference;
      this.touches.set(event.pointerId, pointerPosition(event));
      const current = this.makeTouchReference();
      this.touchReference = current;
      if (!previous || !current) return;
      if (this.touches.size === 1 && this.allowSingleTouchOrbit) {
        this.orbit(current.centroid.x - previous.centroid.x, current.centroid.y - previous.centroid.y);
      } else if (this.touches.size >= 2) {
        this.pan(current.centroid.x - previous.centroid.x, current.centroid.y - previous.centroid.y);
        if (previous.distance > 1e-4 && current.distance > 1e-4) {
          this.zoom(previous.distance / current.distance);
        }
      }
      event.preventDefault();
      return;
    }

    const drag = this.mouseDrag;
    if (!drag || event.pointerId !== drag.pointerId) return;
    const current = pointerPosition(event);
    const deltaX = current.x - drag.position.x;
    const deltaY = current.y - drag.position.y;
    drag.position = current;
    if (drag.action === "orbit") this.orbit(deltaX, deltaY);
    else this.pan(deltaX, deltaY);
    event.preventDefault();
  };

  private readonly handlePointerEnd = (event: PointerEvent): void => {
    if (this.mouseDrag?.pointerId === event.pointerId) this.mouseDrag = null;
    if (this.touches.delete(event.pointerId)) this.touchReference = this.makeTouchReference();
    this.releasePointer(event.pointerId);
  };

  private readonly handleWheel = (event: WheelEvent): void => {
    if (!this.enabled || !this.canZoomWheel(event)) return;
    const normalizedDelta = event.deltaMode === WheelEvent.DOM_DELTA_LINE
      ? event.deltaY * 16
      : event.deltaMode === WheelEvent.DOM_DELTA_PAGE
        ? event.deltaY * Math.max(1, this.projection.getState().viewportHeight)
        : event.deltaY;
    this.zoom(Math.exp(normalizedDelta * this.wheelZoomExponentPerPixel));
    event.preventDefault();
  };

  private readonly handleContextMenu = (event: MouseEvent): void => {
    if (this.enabled) event.preventDefault();
  };

  private readonly handleBlur = (): void => {
    this.releasePointers();
  };

  private mouseAction(event: PointerEvent): "orbit" | "pan" | null {
    if (event.button === 2) return event.shiftKey ? "pan" : "orbit";
    if (event.button === 1) return "pan";
    if (event.button === 0) return event.shiftKey ? "pan" : "orbit";
    return null;
  }

  private makeTouchReference(): TouchReference | null {
    const points = [...this.touches.values()];
    if (points.length === 0) return null;
    if (points.length === 1) return { centroid: points[0], distance: 0 };
    const first = points[0];
    const second = points[1];
    return {
      centroid: { x: (first.x + second.x) * 0.5, y: (first.y + second.y) * 0.5 },
      distance: Math.hypot(second.x - first.x, second.y - first.y),
    };
  }

  private capturePointer(pointerId: number): void {
    try {
      this.element?.setPointerCapture(pointerId);
    } catch {
      // Capture can fail when a native pointer ended before event delivery.
    }
  }

  private releasePointer(pointerId: number): void {
    try {
      if (this.element?.hasPointerCapture(pointerId)) this.element.releasePointerCapture(pointerId);
    } catch {
      // Native cancellation may already have released capture.
    }
  }

  private releasePointers(): void {
    if (this.mouseDrag) this.releasePointer(this.mouseDrag.pointerId);
    for (const pointerId of this.touches.keys()) this.releasePointer(pointerId);
    this.mouseDrag = null;
    this.touches.clear();
    this.touchReference = null;
  }
}

export function createCameraControls(
  projection: SharedCameraProjection,
  options: CameraControlsOptions = {},
): OrbitCameraController {
  return new OrbitCameraController(projection, options);
}
