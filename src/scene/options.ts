import type { SharedCameraProjection } from "./camera";

export interface SceneViewOptions {
  readonly canvas?: HTMLCanvasElement;
  readonly camera?: SharedCameraProjection;
  readonly className?: string;
}
