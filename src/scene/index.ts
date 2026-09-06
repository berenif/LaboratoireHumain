import type { PoseView, RendererMode } from "../core/types";
import { SharedCameraProjection } from "./camera";
import { Canvas2DView } from "./canvas2d-view";
import type { SceneViewOptions } from "./options";
import { WebGLView } from "./webgl-view";

export { SharedCameraProjection, createSharedCamera } from "./camera";
export { OrbitCameraController, createCameraControls } from "./camera-controls";
export type { CameraControlAction, CameraControlsOptions } from "./camera-controls";
export { checkGraphicsCapabilities } from "./capabilities";
export type { GraphicsCapabilities } from "./capabilities";
export { Canvas2DView } from "./canvas2d-view";
export { WebGLView } from "./webgl-view";
export type { WebGLRenderMetrics } from "./webgl-view";
export type { SceneViewOptions } from "./options";
export { interpolatePoseSnapshot } from "./pose";
export { inspectScenePresentation, projectVisibleRegions } from "./diagnostics";
export type { ProjectedRegionDiagnostic } from "./diagnostics";

const sharedFactoryCamera = new SharedCameraProjection();

/** Creates one read-only presentation adapter; both modes share one camera. */
export function createPoseView(mode: RendererMode, options: SceneViewOptions = {}): PoseView {
  const resolved = { ...options, camera: options.camera ?? sharedFactoryCamera };
  return mode === "webgl" ? new WebGLView(resolved) : new Canvas2DView(resolved);
}
