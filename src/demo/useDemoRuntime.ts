"use client";

import { useEffect, useRef, useState } from "react";
import type { RefObject } from "react";
import { REGION_IDS } from "../core/types";
import type { DiagnosticsSnapshot, RegionId, RendererMode } from "../core/types";
import { installBrowserApi } from "./browser-api";
import { createDemoRuntime } from "./DemoRuntime";
import type { DemoRuntime } from "./DemoRuntime";
import { summarizeFrames } from "./telemetry";
import type { BrowserCapabilities, FrameSummary } from "./telemetry";

type RegionPoint = NonNullable<ReturnType<DemoRuntime["regionPoint"]>>;

interface DemoState {
  runtime: DemoRuntime | null;
  capabilities: BrowserCapabilities | null;
  diagnostics: DiagnosticsSnapshot | null;
  renderer: RendererMode;
  paused: boolean;
  anchorPoints: Partial<Record<RegionId, RegionPoint>>;
  frameSummary: FrameSummary;
  status: string;
  fatalError: string | null;
  qaMode: boolean;
}

function initialState(): DemoState {
  return {
    runtime: null,
    capabilities: null,
    diagnostics: null,
    renderer: "canvas2d",
    paused: false,
    anchorPoints: {},
    frameSummary: summarizeFrames([]),
    status: "Initializing real physics…",
    fatalError: null,
    qaMode: false,
  };
}

/** Adapts the browser runtime lifecycle and throttled snapshots to React. */
export function useDemoRuntime(hostRef: RefObject<HTMLDivElement | null>): DemoState {
  const runtimeRef = useRef<DemoRuntime | null>(null);
  const [state, setState] = useState(initialState);

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const abort = new AbortController();
    const qaMode = new URLSearchParams(window.location.search).get("qa") === "1";
    let unsubscribe: (() => void) | undefined;
    let lastSummaryMs = 0;
    let frameSummary = summarizeFrames([]);

    const captureError = (value: unknown) => {
      if (abort.signal.aborted) return;
      runtimeRef.current?.stop();
      const fatalError = value instanceof Error ? value.message : String(value);
      setState((previous) => ({ ...previous, fatalError, status: "Simulation stopped" }));
    };
    const onError = (event: ErrorEvent) => captureError(event.error ?? event.message);
    const onRejection = (event: PromiseRejectionEvent) => captureError(event.reason);
    const onBlur = () => runtimeRef.current?.pause("blur");
    const onVisibility = () => { if (document.hidden) onBlur(); };

    const removeBrowserApi = installBrowserApi(
      () => runtimeRef.current,
      () => runtimeRef.current?.capabilities ?? null,
    );
    window.addEventListener("error", onError);
    window.addEventListener("unhandledrejection", onRejection);
    window.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibility);

    void createDemoRuntime(host, abort.signal).then((runtime) => {
      if (abort.signal.aborted) {
        runtime.dispose();
        return;
      }
      runtimeRef.current = runtime;
      const update = (current: DemoRuntime, nowMs: number) => {
        if (abort.signal.aborted) return;
        if (nowMs - lastSummaryMs > 1000) {
          frameSummary = current.frameSummary();
          lastSummaryMs = nowMs;
        }
        const anchorPoints: DemoState["anchorPoints"] = {};
        for (const region of REGION_IDS) {
          const point = current.regionPoint(region);
          if (point) anchorPoints[region] = point;
        }
        setState((previous) => ({
          ...previous,
          runtime: current,
          capabilities: current.capabilities,
          diagnostics: current.current.diagnostics,
          renderer: current.renderer,
          paused: current.paused,
          anchorPoints,
          frameSummary,
          status: current.status,
          qaMode,
        }));
      };
      unsubscribe = runtime.subscribe(update);
      update(runtime, performance.now());
    }).catch(captureError);

    return () => {
      abort.abort();
      unsubscribe?.();
      window.removeEventListener("error", onError);
      window.removeEventListener("unhandledrejection", onRejection);
      window.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
      removeBrowserApi();
      runtimeRef.current?.dispose();
      runtimeRef.current = null;
    };
  }, [hostRef]);

  return state;
}
