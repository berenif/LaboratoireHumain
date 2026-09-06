"use client";

import { useEffect, useRef, useState } from "react";
import type { InteractionReplayResult } from "../interaction/browser-replay";
import { inspectScenePresentation } from "../scene/diagnostics";
import type { DemoRuntime } from "./DemoRuntime";

function captureEvidence(runtime: DemoRuntime) {
  const canvas = runtime.host.querySelector("canvas");
  const scene = canvas
    ? inspectScenePresentation(runtime.current, runtime.view.getProjection(), canvas, (ray) => runtime.character.pick(ray))
    : null;
  return {
    diagnostics: runtime.current.diagnostics,
    scene,
    interaction: runtime.interaction.getStatus(),
    paused: runtime.paused,
  };
}

type LiveEvidence = ReturnType<typeof captureEvidence>;

/** Optional QA presentation; mount only when the URL explicitly enables QA. */
export function BrowserVerification({ runtime }: { runtime: DemoRuntime }) {
  const [qaResult, setQaResult] = useState<InteractionReplayResult | { error: string } | null>(null);
  const [qaStage, setQaStage] = useState("");
  const [runningRuntime, setRunningRuntime] = useState<DemoRuntime | null>(null);
  const [liveEvidence, setLiveEvidence] = useState<LiveEvidence | null>(null);
  const [qaTrace, setQaTrace] = useState("[]");
  const replayController = useRef<AbortController | null>(null);
  const qaRunning = runningRuntime === runtime;

  useEffect(() => {
    const eventTrace: LiveEvidence[] = [];
    let resetVersion = runtime.resetVersion;
    const unsubscribe = runtime.subscribe((current) => {
      if (current.resetVersion !== resetVersion) {
        eventTrace.length = 0;
        resetVersion = current.resetVersion;
      }
      const evidence = captureEvidence(current);
      setLiveEvidence(evidence);
      eventTrace.push(evidence);
      if (eventTrace.length > 180) eventTrace.shift();
      setQaTrace(JSON.stringify(eventTrace));
    });
    return () => {
      unsubscribe();
      replayController.current?.abort();
      replayController.current = null;
    };
  }, [runtime]);

  const runBrowserReplay = async (pointerType: "mouse" | "touch") => {
    if (replayController.current) return;
    const controller = new AbortController();
    replayController.current = controller;
    const { signal } = controller;
    setRunningRuntime(runtime);
    setQaResult(null);
    try {
      const { runInteractionReplay } = await import("../interaction/browser-replay");
      signal.throwIfAborted();
      const result = await runInteractionReplay({
        host: runtime.host,
        interaction: runtime.interaction,
        diagnostics: () => runtime.character.diagnostics(),
        snapshot: () => runtime.character.getSnapshot(runtime.renderer),
        projection: () => runtime.view.getProjection(),
      }, { pointerType, signal, onProgress: (stage) => { if (!signal.aborted) setQaStage(stage); } });
      if (!signal.aborted) setQaResult(result);
    } catch (error) {
      if (!signal.aborted) setQaResult({ error: error instanceof Error ? error.message : String(error) });
    } finally {
      if (replayController.current === controller) replayController.current = null;
      if (!signal.aborted) setRunningRuntime(null);
    }
  };

  return (
    <details open style={{ position: "fixed", left: 20, top: 95, width: 340, maxHeight: 240, overflow: "auto", zIndex: 10, background: "#12202b", color: "#eaf3fa", padding: 12 }}>
      <summary>Browser verification</summary>
      <button onClick={() => runBrowserReplay("mouse")} disabled={qaRunning}>{qaRunning ? "Browser replay running" : "Run browser input replay"}</button>
      <button onClick={() => runBrowserReplay("touch")} disabled={qaRunning}>Run touch DOM replay</button>
      <span data-testid="qa-stage">{qaStage}</span>
      <output data-testid="qa-result" style={{ display: "block", maxHeight: 50, overflow: "auto", overflowWrap: "anywhere" }}>{JSON.stringify(qaResult)}</output>
      <pre data-testid="qa-live" style={{ maxHeight: 80, overflow: "auto", fontSize: 11, whiteSpace: "pre-wrap", overflowWrap: "anywhere" }}>{JSON.stringify(liveEvidence)}</pre>
      <span data-testid="qa-trace" data-trace={qaTrace} />
    </details>
  );
}
