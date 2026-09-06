"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { createEmbodiedCharacter } from "../character";
import { runInteractionReplay } from "../interaction/browser-replay";
import { PointerInteraction } from "../interaction";
import {
  checkGraphicsCapabilities,
  createCameraControls,
  createPoseView,
  createSharedCamera,
  inspectScenePresentation,
} from "../scene";
import type { OrbitCameraController, SharedCameraProjection } from "../scene";
import { ControlPanel } from "../ui/ControlPanel";
import { FixedStepLoop } from "./FixedStepLoop";
import { REGION_IDS } from "./types";
import type {
  CameraState,
  CharacterController,
  DiagnosticsSnapshot,
  PoseSnapshot,
  PoseView,
  RegionId,
  RendererMode,
} from "./types";

interface GraphicsCapabilities {
  webgl2: boolean;
  canvas2d: boolean;
  webglError?: string;
  canvas2dError?: string;
  vendor?: string;
  renderer?: string;
  browser?: string;
  devicePixelRatio?: number;
  viewport?: { width: number; height: number };
}

interface DemoRuntime {
  camera: SharedCameraProjection;
  cameraControls: OrbitCameraController;
  character: CharacterController;
  interaction: PointerInteraction;
  view: PoseView;
  loop: FixedStepLoop;
  previous: PoseSnapshot;
  current: PoseSnapshot;
  resizeObserver: ResizeObserver;
}

declare global {
  interface Window {
    __EMBODIED_DEMO__?: {
      ready: () => boolean;
      diagnostics: () => DiagnosticsSnapshot | null;
      capabilities: () => GraphicsCapabilities | null;
      renderer: () => RendererMode;
      frameSummary: () => { samples: number; p50Ms: number; p95Ms: number; p99Ms: number; averageFps: number };
      camera: () => CameraState | null;
      regionPoint: (region: RegionId) => { x: number; y: number; visible: boolean } | null;
    };
  }
}

function summarizeFrames(samples: number[]) {
  if (!samples.length) return { samples: 0, p50Ms: 0, p95Ms: 0, p99Ms: 0, averageFps: 0 };
  const sorted = [...samples].sort((a, b) => a - b);
  const at = (p: number) => sorted[Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * p))];
  const mean = sorted.reduce((sum, value) => sum + value, 0) / sorted.length;
  return { samples: sorted.length, p50Ms: at(0.5), p95Ms: at(0.95), p99Ms: at(0.99), averageFps: mean > 0 ? 1000 / mean : 0 };
}

export function EmbodiedDemo() {
  const hostRef = useRef<HTMLDivElement>(null);
  const pausedRef = useRef(false);
  const [qaMode, setQaMode] = useState(false);
  const [qaResult, setQaResult] = useState<unknown>(null);
  const [qaStage, setQaStage] = useState("");
  const [qaRunning, setQaRunning] = useState(false);
  const [liveEvidence, setLiveEvidence] = useState<unknown>(null);
  const [qaTrace, setQaTrace] = useState("[]");
  const eventTrace = useRef<unknown[]>([]);
  const runtimeRef = useRef<DemoRuntime | null>(null);
  const capabilitiesRef = useRef<GraphicsCapabilities | null>(null);
  const rendererRef = useRef<RendererMode>("canvas2d");
  const frameSamples = useRef<number[]>([]);
  const lastFrameMs = useRef<number | null>(null);
  const warmupUntilMs = useRef(0);
  const lastUiMs = useRef(0);
  const lastFrameSummaryMs = useRef(0);
  const [capabilities, setCapabilities] = useState<GraphicsCapabilities | null>(null);
  const [renderer, setRenderer] = useState<RendererMode>("canvas2d");
  const [diagnostics, setDiagnostics] = useState<DiagnosticsSnapshot | null>(null);
  const [paused, setPaused] = useState(false);
  const [anchorPoints, setAnchorPoints] = useState<Partial<Record<RegionId, { x: number; y: number; visible: boolean }>>>({});
  const [frameSummary, setFrameSummary] = useState(() => summarizeFrames([]));
  const [status, setStatus] = useState("Initializing real physics…");
  const [fatalError, setFatalError] = useState<string | null>(null);

  const resizeView = useCallback((view: PoseView, host: HTMLElement) => {
    const rect = host.getBoundingClientRect();
    view.resize(Math.max(1, rect.width), Math.max(1, rect.height), Math.min(2, window.devicePixelRatio || 1));
  }, []);

  useEffect(() => {
    const qaEnabled = new URLSearchParams(window.location.search).get("qa") === "1";
    let cancelled = false;
    const host = hostRef.current;
    if (!host) return;
    const captureError = (value: unknown) => {
      const message = value instanceof Error ? value.message : String(value);
      setFatalError(message);
      setStatus("Simulation stopped");
    };
    const onError = (event: ErrorEvent) => captureError(event.error ?? event.message);
    const onRejection = (event: PromiseRejectionEvent) => captureError(event.reason);
    window.addEventListener("error", onError);
    window.addEventListener("unhandledrejection", onRejection);

    void (async () => {
      try {
        const checked: GraphicsCapabilities = {
          ...checkGraphicsCapabilities(),
          browser: navigator.userAgent,
          devicePixelRatio: window.devicePixelRatio || 1,
          viewport: { width: window.innerWidth, height: window.innerHeight },
        };
        capabilitiesRef.current = checked;
        setCapabilities(checked);
        if (!checked.canvas2d && !checked.webgl2) throw new Error("Neither Canvas2D nor WebGL2 is available.");
        const initialRenderer: RendererMode = checked.webgl2 ? "webgl" : "canvas2d";
        rendererRef.current = initialRenderer;
        setRenderer(initialRenderer);
        const character = await createEmbodiedCharacter(initialRenderer);
        if (cancelled) return;
        setQaMode(qaEnabled);
        const interaction = new PointerInteraction();
        const camera = createSharedCamera();
        const view = createPoseView(initialRenderer, { camera });
        view.mount(host);
        resizeView(view, host);
        interaction.attach(host, camera, (ray) => character.pick(ray));
        const cameraControls = createCameraControls(camera, {
          allowSingleTouchOrbit: true,
          canStartPointer: () => interaction.getActivePointerId() === null,
          canZoomWheel: () => interaction.getActivePointerId() === null,
        });
        cameraControls.attach(host);
        let previous = character.getSnapshot(initialRenderer);
        let current = previous;
        view.setSnapshot(previous, current, 0);
        view.render();
        warmupUntilMs.current = performance.now() + 1000;

        const loop = new FixedStepLoop(
          (dt) => {
            character.fixedUpdate(dt, interaction.consumeCommand());
            previous = runtimeRef.current?.current ?? current;
            current = character.getSnapshot(rendererRef.current);
            const runtime = runtimeRef.current;
            if (runtime) {
              runtime.previous = previous;
              runtime.current = current;
            }
          },
          (alpha, nowMs) => {
            const runtime = runtimeRef.current;
            if (!runtime) return;
            runtime.view.setSnapshot(runtime.previous, runtime.current, alpha);
            runtime.view.render();
            if (lastFrameMs.current !== null && nowMs > warmupUntilMs.current) {
              frameSamples.current.push(nowMs - lastFrameMs.current);
              if (frameSamples.current.length > 1200) frameSamples.current.shift();
            }
            lastFrameMs.current = nowMs;
            if (nowMs - lastUiMs.current > 100) {
              lastUiMs.current = nowMs;
              setDiagnostics(runtime.current.diagnostics);
              if (qaEnabled) {
              const canvas = host.querySelector("canvas");
              const scene = canvas ? inspectScenePresentation(runtime.current, runtime.view.getProjection(), canvas, ray => runtime.character.pick(ray)) : null;
              const evidence = {diagnostics: runtime.current.diagnostics, scene, interaction: runtime.interaction.getStatus(), paused: pausedRef.current};
              setLiveEvidence(evidence);
              eventTrace.current.push(evidence);
              if (eventTrace.current.length > 180) eventTrace.current.shift();
              setQaTrace(JSON.stringify(eventTrace.current));
              }
              const rect = host.getBoundingClientRect();
              const nextAnchors: Partial<Record<RegionId, { x: number; y: number; visible: boolean }>> = {};
              for (const region of REGION_IDS) {
                const pose = runtime.current.segments.find((segment) => segment.id === region);
                if (!pose) continue;
                const point = runtime.view.getProjection().project(pose.position);
                nextAnchors[region] = { x: rect.left + point.x, y: rect.top + point.y, visible: point.visible };
              }
              setAnchorPoints(nextAnchors);
              if (nowMs - lastFrameSummaryMs.current > 1000) {
                lastFrameSummaryMs.current = nowMs;
                setFrameSummary(summarizeFrames(frameSamples.current));
              }
            }
          },
        );
        const resizeObserver = new ResizeObserver(() => resizeView(runtimeRef.current?.view ?? view, host));
        resizeObserver.observe(host);
        runtimeRef.current = {
          camera,
          cameraControls,
          character,
          interaction,
          view,
          loop,
          previous,
          current,
          resizeObserver,
        };
        setDiagnostics(current.diagnostics);
        setStatus("Ready — drag any highlighted body region");
        loop.start();
      } catch (error) {
        captureError(error);
      }
    })();

    const clearForFocusLoss = () => {
      const runtime = runtimeRef.current;
      if (!runtime) return;
      runtime.interaction.clear("blur");
      runtime.character.fixedUpdate(1 / 60, runtime.interaction.consumeCommand());
      runtime.character.pause();
      runtime.loop.pause();
      runtime.current = runtime.character.getSnapshot(rendererRef.current);
      runtime.previous = runtime.current;
      setDiagnostics(runtime.current.diagnostics);
      pausedRef.current = true;
      setPaused(true);
      setStatus("Paused after focus loss");
    };
    const onVisibility = () => { if (document.hidden) clearForFocusLoss(); };
    window.addEventListener("blur", clearForFocusLoss);
    document.addEventListener("visibilitychange", onVisibility);

    return () => {
      cancelled = true;
      window.removeEventListener("error", onError);
      window.removeEventListener("unhandledrejection", onRejection);
      window.removeEventListener("blur", clearForFocusLoss);
      document.removeEventListener("visibilitychange", onVisibility);
      const runtime = runtimeRef.current;
      if (runtime) {
        runtime.loop.stop();
        runtime.cameraControls.dispose();
        runtime.interaction.detach();
        runtime.view.dispose();
        runtime.resizeObserver.disconnect();
      }
      runtimeRef.current = null;
      delete window.__EMBODIED_DEMO__;
    };
  }, [resizeView]);

  const switchRenderer = useCallback((next: RendererMode) => {
    const runtime = runtimeRef.current;
    const host = hostRef.current;
    const caps = capabilitiesRef.current;
    if (!runtime || !host || next === rendererRef.current) return;
    if ((next === "webgl" && !caps?.webgl2) || (next === "canvas2d" && !caps?.canvas2d)) return;
    runtime.interaction.clear("cancel");
    runtime.character.fixedUpdate(1 / 60, runtime.interaction.consumeCommand());
    runtime.interaction.detach();
    runtime.view.dispose();
    const view = createPoseView(next, { camera: runtime.camera });
    view.mount(host);
    resizeView(view, host);
    runtime.view = view;
    runtime.previous = runtime.character.getSnapshot(next);
    runtime.current = runtime.previous;
    view.setSnapshot(runtime.previous, runtime.current, 0);
    runtime.interaction.attach(host, runtime.camera, (ray) => runtime.character.pick(ray));
    rendererRef.current = next;
    setRenderer(next);
    frameSamples.current = [];
    lastFrameMs.current = null;
    warmupUntilMs.current = performance.now() + 1000;
    setStatus(`${next === "webgl" ? "WebGL2" : "Canvas2D"} view active`);
  }, [resizeView]);

  const togglePause = useCallback(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    if (pausedRef.current) {
      runtime.interaction.reset();
      runtime.character.resume();
      runtime.loop.resume();
      pausedRef.current = false;
      setPaused(false);
      setStatus("Ready — drag any highlighted body region");
    } else {
      runtime.interaction.clear("pause");
      runtime.character.fixedUpdate(1 / 60, runtime.interaction.consumeCommand());
      runtime.character.pause();
      runtime.loop.pause();
      runtime.current = runtime.character.getSnapshot(rendererRef.current);
      runtime.previous = runtime.current;
      setDiagnostics(runtime.current.diagnostics);
      pausedRef.current = true;
      setPaused(true);
      setStatus("Paused");
    }
  }, []);

  const reset = useCallback(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    runtime.interaction.clear("reset");
    runtime.character.fixedUpdate(1 / 60, runtime.interaction.consumeCommand());
    runtime.interaction.reset();
    runtime.character.reset();
    if (pausedRef.current) runtime.character.pause();
    runtime.cameraControls.reset();
    runtime.loop.resetTiming();
    frameSamples.current = [];
    lastFrameMs.current = null;
    warmupUntilMs.current = performance.now() + 1000;
    eventTrace.current = [];
    runtime.previous = runtime.character.getSnapshot(rendererRef.current);
    runtime.current = runtime.previous;
    runtime.view.setSnapshot(runtime.previous, runtime.current, 0);
    setDiagnostics(runtime.current.diagnostics);
    setStatus(pausedRef.current ? "Reset — paused" : "Reset — ready to pull");
  }, []);

  useEffect(() => {
    window.__EMBODIED_DEMO__ = {
      ready: () => Boolean(runtimeRef.current?.current.diagnostics.interactiveViewReady),
      diagnostics: () => runtimeRef.current?.current.diagnostics ?? null,
      capabilities: () => capabilitiesRef.current,
      renderer: () => rendererRef.current,
      frameSummary: () => summarizeFrames(frameSamples.current),
      camera: () => runtimeRef.current?.camera.getState() ?? null,
      regionPoint: (region) => {
        const runtime = runtimeRef.current;
        const host = hostRef.current;
        if (!runtime || !host) return null;
        const pose = runtime.current.segments.find((segment) => segment.id === region);
        if (!pose) return null;
        const projected = runtime.view.getProjection().project(pose.position);
        const rect = host.getBoundingClientRect();
        return { x: rect.left + projected.x, y: rect.top + projected.y, visible: projected.visible };
      },
    };
  });

  const runBrowserReplay = async (pointerType: "mouse" | "touch" = "mouse") => {
    const runtime = runtimeRef.current;
    const host = hostRef.current;
    if (!runtime || !host || qaRunning) return;
    setQaRunning(true);
    setQaResult(null);
    try {
      const result = await runInteractionReplay({
        host, interaction: runtime.interaction,
        diagnostics: () => runtime.character.diagnostics(),
        snapshot: () => runtime.character.getSnapshot(rendererRef.current),
        projection: () => runtime.view.getProjection(),
      }, {pointerType, onProgress: stage => setQaStage(stage)});
      setQaResult(result);
    } catch (error) {
      setQaResult({error: error instanceof Error ? error.message : String(error)});
    } finally { setQaRunning(false); }
  };

  return (
    <main className="demo-shell" data-simulation-ready={diagnostics?.simulationReady ? "true" : "false"} data-interactive-view-ready={diagnostics?.interactiveViewReady ? "true" : "false"} data-renderer={renderer} data-frame-samples={frameSummary.samples} data-frame-p50-ms={frameSummary.p50Ms.toFixed(3)} data-frame-p95-ms={frameSummary.p95Ms.toFixed(3)} data-frame-p99-ms={frameSummary.p99Ms.toFixed(3)} data-average-fps={frameSummary.averageFps.toFixed(2)} data-browser={capabilities?.browser ?? ""} data-viewport={capabilities?.viewport ? `${capabilities.viewport.width}x${capabilities.viewport.height}` : ""} data-dpr={capabilities?.devicePixelRatio ?? ""}>
      <h1 className="sr-only">Embodied Character</h1>
      <section className="sim-workspace" aria-label="Interactive character simulation">
        <div ref={hostRef} className="view-host" data-testid="simulation-view" />
        <div className="test-anchors" aria-hidden="true">
          {REGION_IDS.map((region) => {
            const point = anchorPoints[region];
            return <span key={region} data-region={region} data-client-x={point?.x.toFixed(1) ?? ""} data-client-y={point?.y.toFixed(1) ?? ""} data-visible={point?.visible ? "true" : "false"} />;
          })}
        </div>
        {fatalError || !diagnostics?.interactiveViewReady ? (
          <div className={`startup-status${fatalError ? " error" : ""}`} role={fatalError ? "alert" : "status"} aria-live="polite">
            {fatalError ?? status}
          </div>
        ) : null}
        <ControlPanel className="control-panel" diagnostics={diagnostics} renderer={renderer} webglAvailable={Boolean(capabilities?.webgl2)} canvas2dAvailable={Boolean(capabilities?.canvas2d)} paused={paused} onRendererChange={switchRenderer} onPauseToggle={togglePause} onReset={reset} />
      </section>
      {qaMode && <details open style={{position:"fixed", left:20, top:95, width:340, maxHeight:240, overflow:"auto", zIndex:10, background:"#12202b", color:"#eaf3fa", padding:12}}>
        <summary>Browser verification</summary>
        <button onClick={() => runBrowserReplay("mouse")} disabled={qaRunning}>{qaRunning ? "Browser replay running" : "Run browser input replay"}</button>
        <button onClick={() => runBrowserReplay("touch")} disabled={qaRunning}>Run touch DOM replay</button>
        <span data-testid="qa-stage">{qaStage}</span>
        <output data-testid="qa-result" style={{display:"block", maxHeight:50, overflow:"auto", overflowWrap:"anywhere"}}>{JSON.stringify(qaResult)}</output>
        <pre data-testid="qa-live" style={{maxHeight:80, overflow:"auto", fontSize:11, whiteSpace:"pre-wrap", overflowWrap:"anywhere"}}>{JSON.stringify(liveEvidence)}</pre>
        <span data-testid="qa-trace" data-trace={qaTrace} />
      </details>}
    </main>
  );
}
