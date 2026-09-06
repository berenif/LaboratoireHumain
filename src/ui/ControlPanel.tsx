"use client";

import * as React from "react";

import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import type { DiagnosticsSnapshot, RegionId, RendererMode } from "../core/types";

export interface ControlPanelProps {
  diagnostics: Readonly<DiagnosticsSnapshot> | null;
  renderer: RendererMode;
  paused: boolean;
  webglAvailable?: boolean | null;
  canvas2dAvailable?: boolean | null;
  onRendererChange: (renderer: RendererMode) => void;
  onPauseToggle: () => void;
  onReset: () => void;
  className?: string;
}

const REGION_LABELS: Readonly<Record<RegionId, string>> = {
  head: "Head",
  torso: "Torso",
  pelvis: "Pelvis",
  leftHand: "Left hand",
  rightHand: "Right hand",
  leftFoot: "Left foot",
  rightFoot: "Right foot",
};

const labelRegion = (region: RegionId | null): string =>
  region ? REGION_LABELS[region] : "None";

const labelToken = (value: string): string =>
  value
    .split("-")
    .map((token) => token.charAt(0).toUpperCase() + token.slice(1))
    .join(" ");

export function ControlPanel({
  diagnostics,
  renderer,
  paused,
  webglAvailable = null,
  canvas2dAvailable = true,
  onRendererChange,
  onPauseToggle,
  onReset,
  className = "",
}: ControlPanelProps): React.JSX.Element {
  const errors = diagnostics?.errors ?? [];
  const force = Number.isFinite(diagnostics?.appliedGrabForceN)
    ? Math.max(0, diagnostics?.appliedGrabForceN ?? 0)
    : 0;
  const ready = Boolean(diagnostics?.simulationReady && diagnostics.interactiveViewReady);
  const bodyInputAvailable = Boolean(!paused && diagnostics?.bodyInputAvailable);
  const statusLabel = paused ? "Paused" : !ready ? "Starting" : bodyInputAvailable ? "Ready" : labelToken(diagnostics!.state);
  const compact = Boolean(diagnostics?.activeGrab || diagnostics?.authority === "ragdoll");

  return (
    <aside
      className={`rounded-2xl border border-white/10 bg-slate-950/88 p-3 text-slate-100 shadow-2xl shadow-black/30 backdrop-blur-md ${className}`}
      aria-label="Character controls"
      data-testid="control-panel"
      data-compact={compact ? "true" : "false"}
    >
      <div className="flex items-center gap-2">
        <label htmlFor="renderer-select" className="sr-only">
          Renderer
        </label>
        <Select
          value={renderer}
          onValueChange={(value) => onRendererChange(value as RendererMode)}
        >
          <SelectTrigger
            id="renderer-select"
            className="h-11 min-w-0 flex-1 border-white/15 bg-white/5 text-slate-100 hover:bg-white/10"
            aria-label="Renderer"
            aria-describedby={webglAvailable === false ? "webgl-note" : undefined}
            data-testid="renderer-picker"
          >
            <SelectValue />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="canvas2d" disabled={canvas2dAvailable === false}>
              Canvas 2D{canvas2dAvailable === false ? " — unavailable" : ""}
            </SelectItem>
            <SelectItem value="webgl" disabled={webglAvailable === false}>
              WebGL2{webglAvailable === false ? " — unavailable" : ""}
            </SelectItem>
          </SelectContent>
        </Select>

        <Button
          type="button"
          variant="secondary"
          className="h-11 shrink-0 bg-white/10 px-3 text-white hover:bg-white/15"
          onClick={onPauseToggle}
          aria-pressed={paused}
          data-testid="pause-toggle"
        >
          {paused ? "Resume" : "Pause"}
        </Button>
        <Button
          type="button"
          variant="outline"
          className="h-11 shrink-0 border-white/15 bg-transparent px-3 text-white hover:bg-white/10 hover:text-white"
          onClick={onReset}
          data-testid="reset-button"
        >
          Reset
        </Button>
      </div>

      <p className="mt-2 truncate text-xs text-slate-400" aria-live="polite">
        <span data-testid="motion-state">
          {paused ? "Paused" : diagnostics ? labelToken(diagnostics.state) : "Starting"}
        </span>
        <span aria-hidden="true"> · </span>
        <span>
          {paused
            ? "Resume to continue · camera and Reset remain available"
            : ready && !bodyInputAvailable
              ? diagnostics?.state === "recovering"
                ? "Getting up · body control returns after stable standing"
                : "Protecting the fall · automatic recovery follows"
            : diagnostics?.activeGrab
            ? `Moving ${labelRegion(diagnostics.selectedRegion)} · release to keep momentum`
            : "Drag body · drag empty space to orbit · wheel or pinch to zoom"}
        </span>
      </p>

      <div className="sr-only">
        <span data-testid="readiness-status">{statusLabel}</span>
        <span data-testid="body-input-availability">{bodyInputAvailable ? "Available" : "Unavailable"}</span>
        <span data-testid="motion-authority">
          {diagnostics ? labelToken(diagnostics.authority) : "Waiting"}
        </span>
        <span data-testid="selected-region">
          {labelRegion(diagnostics?.selectedRegion ?? null)}
        </span>
        <span data-testid="step-count">{diagnostics?.stepCount ?? 0}</span>
        <span data-testid="pull-effort">{Math.round(force)} N</span>
      </div>

      {webglAvailable === false ? (
        <p id="webgl-note" className="sr-only">
          WebGL2 is unavailable in this browser. Canvas 2D remains interactive.
        </p>
      ) : null}

      {errors.length > 0 ? (
        <p
          className="mt-2 text-xs text-rose-100"
          role="alert"
          data-testid="diagnostic-error"
        >
          {errors[0]}
        </p>
      ) : null}
    </aside>
  );
}

export default ControlPanel;
