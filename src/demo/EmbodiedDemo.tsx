"use client";

import { useRef } from "react";
import { REGION_IDS } from "../core/types";
import { ControlPanel } from "../ui/ControlPanel";
import { PlaygroundPanel } from "../ui/PlaygroundPanel";
import { ProtocolPanel } from "../ui/ProtocolPanel";
import { BrowserVerification } from "./BrowserVerification";
import { useDemoRuntime } from "./useDemoRuntime";

export function EmbodiedDemo() {
  const hostRef = useRef<HTMLDivElement>(null);
  const {
    runtime, capabilities, diagnostics, renderer, paused, anchorPoints,
    frameSummary, status, fatalError, qaMode, playground, trial,
  } = useDemoRuntime(hostRef);
  const snapshot = runtime?.current;
  const playgroundMode = Boolean(snapshot?.playground);
  const striker = snapshot?.striker;
  const protocol = snapshot?.protocol;

  return (
    <main
      className={`demo-shell${playgroundMode ? "" : " protocol-mode"}`}
      data-simulation-ready={diagnostics?.simulationReady ? "true" : "false"}
      data-interactive-view-ready={diagnostics?.interactiveViewReady ? "true" : "false"}
      data-body-input-available={diagnostics?.bodyInputAvailable && !paused ? "true" : "false"}
      data-strike-available={!paused && striker?.available ? "true" : "false"}
      data-renderer={renderer}
      data-frame-samples={frameSummary.samples}
      data-frame-p50-ms={frameSummary.p50Ms.toFixed(3)}
      data-frame-p95-ms={frameSummary.p95Ms.toFixed(3)}
      data-frame-p99-ms={frameSummary.p99Ms.toFixed(3)}
      data-average-fps={frameSummary.averageFps.toFixed(2)}
      data-browser={capabilities?.browser ?? ""}
      data-viewport={capabilities?.viewport ? capabilities.viewport.width + "x" + capabilities.viewport.height : ""}
      data-dpr={capabilities?.devicePixelRatio ?? ""}
    >
      <h1 className="sr-only">{playgroundMode ? "Embodied Character" : "Protocole d’arrêt"}</h1>
      <section className="sim-workspace" aria-label={playgroundMode ? "Interactive character simulation" : "Salle du protocole d’arrêt"}>
        <div ref={hostRef} className="view-host" data-testid="simulation-view" />
        <div className="test-anchors" aria-hidden="true">
          {REGION_IDS.map((region) => {
            const point = anchorPoints[region];
            return (
              <span
                key={region}
                data-region={region}
                data-client-x={point?.x.toFixed(1) ?? ""}
                data-client-y={point?.y.toFixed(1) ?? ""}
                data-visible={point?.visible ? "true" : "false"}
              />
            );
          })}
        </div>
        {fatalError || !diagnostics?.interactiveViewReady ? (
          <div className={"startup-status" + (fatalError ? " error" : "")} role={fatalError ? "alert" : "status"} aria-live="polite">
            {fatalError ?? (!runtime && !playgroundMode ? "Initialisation de la physique…" : status)}
          </div>
        ) : null}
        {playgroundMode ? <><ControlPanel
          className="control-panel"
          diagnostics={diagnostics}
          renderer={renderer}
          quality={runtime?.quality ?? "auto"}
          onQualityChange={next => runtime?.setQuality(next)}
          webglAvailable={Boolean(capabilities?.webgl2)}
          canvas2dAvailable={Boolean(capabilities?.canvas2d)}
          paused={paused}
          onRendererChange={(next) => runtime?.switchRenderer(next)}
          onPauseToggle={() => runtime?.togglePause()}
          onReset={() => runtime?.reset()}
        />
        <PlaygroundPanel
          config={playground}
          trial={trial}
          supportCount={diagnostics?.support.planted.length ?? 0}
          ready={Boolean(diagnostics?.interactiveViewReady) && !fatalError}
          onChange={change => runtime?.setPlayground(change)}
          onOverview={() => runtime?.showArena()}
          onFocus={() => runtime?.focusSubject()}
        /></> : <ProtocolPanel
          renderer={renderer}
          quality={runtime?.quality ?? "auto"}
          onQualityChange={next => runtime?.setQuality(next)}
          webglAvailable={Boolean(capabilities?.webgl2)}
          paused={paused}
          ready={Boolean(diagnostics?.interactiveViewReady) && !fatalError}
          strikeAvailable={Boolean(striker?.available)}
          phase={striker?.phase ?? "idle"}
          impactId={striker?.impactId ?? 0}
          strikes={protocol?.strikes ?? 0}
          recoveries={protocol?.recoveries ?? 0}
          message={protocol?.message ?? null}
          onStrike={() => runtime?.requestStrike()}
          onRendererChange={next => runtime?.switchRenderer(next)}
          onPauseToggle={() => runtime?.togglePause()}
          onReset={() => runtime?.reset()}
          onOverview={() => runtime?.showArena()}
          onFocus={() => runtime?.focusSubject()}
        />}
      </section>
      {qaMode && runtime ? <BrowserVerification runtime={runtime} /> : null}
    </main>
  );
}
