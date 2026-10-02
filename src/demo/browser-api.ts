import type { CameraState, DiagnosticsSnapshot, RegionId, RendererMode, RenderMetrics, RenderQuality } from "../core/types";
import type { DemoRuntime } from "./DemoRuntime";
import type { LoopStats } from "../core/FixedStepLoop";
import { summarizeFrames } from "./telemetry";
import type { BrowserCapabilities, FrameSummary } from "./telemetry";

export interface DemoBrowserApi {
  ready: () => boolean;
  diagnostics: () => (DiagnosticsSnapshot & { presentation: RenderMetrics; renderQuality: RenderQuality }) | null;
  capabilities: () => BrowserCapabilities | null;
  renderer: () => RendererMode;
  frameSummary: () => FrameSummary;
  timing: () => Readonly<LoopStats> | null;
  presentation: () => RenderMetrics | null;
  quality: () => RenderQuality;
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
    diagnostics: () => {
      const runtime = getRuntime();
      return runtime ? { ...runtime.current.diagnostics, presentation: runtime.renderMetrics, renderQuality: runtime.quality } : null;
    },
    capabilities: getCapabilities,
    renderer: () => getRuntime()?.renderer ?? "canvas2d",
    frameSummary: () => getRuntime()?.frameSummary() ?? summarizeFrames([]),
    timing: () => getRuntime()?.timing ?? null,
    presentation: () => getRuntime()?.renderMetrics ?? null,
    quality: () => getRuntime()?.quality ?? "auto",
    camera: () => getRuntime()?.camera.getState() ?? null,
    regionPoint: (region) => getRuntime()?.regionPoint(region) ?? null,
  };
  window.__EMBODIED_DEMO__ = api;
  return () => {
    if (window.__EMBODIED_DEMO__ === api) delete window.__EMBODIED_DEMO__;
  };
}
