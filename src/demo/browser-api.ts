import type { CameraState, DiagnosticsSnapshot, RegionId, RendererMode } from "../core/types";
import type { DemoRuntime } from "./DemoRuntime";
import { summarizeFrames } from "./telemetry";
import type { BrowserCapabilities, FrameSummary } from "./telemetry";

export interface DemoBrowserApi {
  ready: () => boolean;
  diagnostics: () => DiagnosticsSnapshot | null;
  capabilities: () => BrowserCapabilities | null;
  renderer: () => RendererMode;
  frameSummary: () => FrameSummary;
  camera: () => CameraState | null;
  regionPoint: (region: RegionId) => { x: number; y: number; visible: boolean } | null;
}

declare global {
  interface Window {
    __EMBODIED_DEMO__?: DemoBrowserApi;
  }
}

/** Browser automation reads current runtime state without taking ownership of it. */
export function installBrowserApi(
  getRuntime: () => DemoRuntime | null,
  getCapabilities: () => BrowserCapabilities | null,
): () => void {
  const api: DemoBrowserApi = {
    ready: () => Boolean(getRuntime()?.current.diagnostics.interactiveViewReady),
    diagnostics: () => getRuntime()?.current.diagnostics ?? null,
    capabilities: getCapabilities,
    renderer: () => getRuntime()?.renderer ?? "canvas2d",
    frameSummary: () => getRuntime()?.frameSummary() ?? summarizeFrames([]),
    camera: () => getRuntime()?.camera.getState() ?? null,
    regionPoint: (region) => getRuntime()?.regionPoint(region) ?? null,
  };
  window.__EMBODIED_DEMO__ = api;
  return () => {
    if (window.__EMBODIED_DEMO__ === api) delete window.__EMBODIED_DEMO__;
  };
}
