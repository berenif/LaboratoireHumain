import { V3 } from "../core/math";
import type { CameraProjection, CameraState, Ray, Vec3 } from "../core/types";

const DEFAULT_CAMERA: CameraState = Object.freeze({
  // Frontal enough to keep every region center selectable across idle arm sway.
  // This exact default/reset view changes only through user camera input and
  // never follows the character or conceals its physics trajectory.
  position: { x: 1.6, y: 2.25, z: 5.1 },
  target: { x: 0, y: 1.02, z: 0 },
  up: { x: 0, y: 1, z: 0 },
  fovYRadians: Math.PI / 4.2,
  near: 0.05,
  far: 40,
  viewportWidth: 960,
  viewportHeight: 720,
});

export interface CameraBasis {
  readonly forward: Vec3;
  readonly right: Vec3;
  readonly up: Vec3;
}

function finitePositive(value: number, fallback: number): number {
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function copyVec3(value: Vec3): Vec3 {
  return { x: value.x, y: value.y, z: value.z };
}

/**
 * A renderer-independent perspective camera. Both views use this exact math for
 * screen projection, picking rays, and drag-plane intersections.
 */
export class SharedCameraProjection implements CameraProjection {
  private readonly defaults: CameraState;
  private state: CameraState;

  constructor(initial: Partial<CameraState> = {}) {
    const defaults = this.makeState({ ...DEFAULT_CAMERA, ...initial });
    this.defaults = defaults;
    this.state = defaults;
  }

  getState(): CameraState {
    return {
      ...this.state,
      position: copyVec3(this.state.position),
      target: copyVec3(this.state.target),
      up: copyVec3(this.state.up),
    };
  }

  getBasis(): CameraBasis {
    const forward = V3.normalize(V3.sub(this.state.target, this.state.position));
    let right = V3.cross(forward, V3.normalize(this.state.up));
    if (V3.length(right) < 1e-7) {
      right = V3.cross(forward, { x: 0, y: 0, z: 1 });
    }
    right = V3.normalize(right);
    const up = V3.normalize(V3.cross(right, forward));
    return { forward, right, up };
  }

  setViewport(width: number, height: number): void {
    this.state = this.makeState({
      ...this.state,
      viewportWidth: finitePositive(width, 1),
      viewportHeight: finitePositive(height, 1),
    });
  }

  setView(position: Vec3, target: Vec3, up: Vec3 = this.state.up): void {
    this.state = this.makeState({ ...this.state, position, target, up });
  }

  reset(): void {
    const { viewportWidth, viewportHeight } = this.state;
    this.state = this.makeState({ ...this.defaults, viewportWidth, viewportHeight });
  }

  screenToRay(clientX: number, clientY: number, rect: DOMRectReadOnly): Ray {
    const width = finitePositive(rect.width, this.state.viewportWidth);
    const height = finitePositive(rect.height, this.state.viewportHeight);
    const ndcX = ((clientX - rect.left) / width) * 2 - 1;
    const ndcY = 1 - ((clientY - rect.top) / height) * 2;
    const aspect = width / height;
    const tanHalfFov = Math.tan(this.state.fovYRadians * 0.5);
    const { forward, right, up } = this.getBasis();
    const direction = V3.normalize(
      V3.add(
        forward,
        V3.add(
          V3.scale(right, ndcX * tanHalfFov * aspect),
          V3.scale(up, ndcY * tanHalfFov),
        ),
      ),
    );
    return { origin: copyVec3(this.state.position), direction };
  }

  project(worldPoint: Vec3): Readonly<{ x: number; y: number; depth: number; visible: boolean }> {
    const relative = V3.sub(worldPoint, this.state.position);
    const { forward, right, up } = this.getBasis();
    const depth = V3.dot(relative, forward);
    const width = finitePositive(this.state.viewportWidth, 1);
    const height = finitePositive(this.state.viewportHeight, 1);
    const tanHalfFov = Math.tan(this.state.fovYRadians * 0.5);
    const aspect = width / height;
    const safeDepth = Math.max(depth, 1e-7);
    const ndcX = V3.dot(relative, right) / (safeDepth * tanHalfFov * aspect);
    const ndcY = V3.dot(relative, up) / (safeDepth * tanHalfFov);
    return {
      x: (ndcX + 1) * width * 0.5,
      y: (1 - ndcY) * height * 0.5,
      depth,
      visible:
        depth >= this.state.near &&
        depth <= this.state.far &&
        Math.abs(ndcX) <= 1.08 &&
        Math.abs(ndcY) <= 1.08,
    };
  }

  worldRadiusToPixels(worldPoint: Vec3, radius: number): number {
    const center = this.project(worldPoint);
    const { right, up } = this.getBasis();
    const horizontal = this.project(V3.add(worldPoint, V3.scale(right, radius)));
    const vertical = this.project(V3.add(worldPoint, V3.scale(up, radius)));
    const horizontalRadius = Math.hypot(horizontal.x - center.x, horizontal.y - center.y);
    const verticalRadius = Math.hypot(vertical.x - center.x, vertical.y - center.y);
    return Math.max(0.5, (horizontalRadius + verticalRadius) * 0.5);
  }

  intersectDragPlane(ray: Ray, planePoint: Vec3, planeNormal: Vec3): Vec3 | null {
    const normal = V3.normalize(planeNormal);
    const denominator = V3.dot(ray.direction, normal);
    if (Math.abs(denominator) < 1e-7) return null;
    const distance = V3.dot(V3.sub(planePoint, ray.origin), normal) / denominator;
    if (!Number.isFinite(distance) || distance < 0) return null;
    return V3.add(ray.origin, V3.scale(ray.direction, distance));
  }

  private makeState(value: CameraState): CameraState {
    const fov = Number.isFinite(value.fovYRadians)
      ? Math.min(Math.PI - 0.01, Math.max(0.05, value.fovYRadians))
      : DEFAULT_CAMERA.fovYRadians;
    const near = finitePositive(value.near, DEFAULT_CAMERA.near);
    const far = Math.max(near + 0.01, finitePositive(value.far, DEFAULT_CAMERA.far));
    return Object.freeze({
      position: copyVec3(value.position),
      target: copyVec3(value.target),
      up: copyVec3(value.up),
      fovYRadians: fov,
      near,
      far,
      viewportWidth: finitePositive(value.viewportWidth, DEFAULT_CAMERA.viewportWidth),
      viewportHeight: finitePositive(value.viewportHeight, DEFAULT_CAMERA.viewportHeight),
    });
  }
}

export function createSharedCamera(initial: Partial<CameraState> = {}): SharedCameraProjection {
  return new SharedCameraProjection(initial);
}
