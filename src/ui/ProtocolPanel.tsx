"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type { RendererMode, RenderQuality } from "../core/types";
import { QualityPicker } from "./QualityPicker";

interface ProtocolPanelProps {
  renderer: RendererMode;
  quality: RenderQuality;
  onQualityChange: (quality: RenderQuality) => void;
  webglAvailable: boolean;
  paused: boolean;
  ready: boolean;
  strikeAvailable: boolean;
  phase: "idle" | "positioning" | "striking" | "retracting";
  impactId: number;
  strikes: number;
  recoveries: number;
  message: string | null;
  onStrike: () => void;
  onRendererChange: (renderer: RendererMode) => void;
  onPauseToggle: () => void;
  onReset: () => void;
  onOverview: () => void;
  onFocus: () => void;
}

const PHASE_LABELS: Record<ProtocolPanelProps["phase"], string> = {
  idle: "Dispositif prêt",
  positioning: "Alignement du percuteur",
  striking: "Percussion en cours",
  retracting: "Rétraction du percuteur",
};

function playMechanicalImpact(context: AudioContext): void {
  const now = context.currentTime;
  const oscillator = context.createOscillator();
  const gain = context.createGain();
  oscillator.type = "sawtooth";
  oscillator.frequency.setValueAtTime(128, now);
  oscillator.frequency.exponentialRampToValueAtTime(42, now + 0.16);
  gain.gain.setValueAtTime(0.0001, now);
  gain.gain.exponentialRampToValueAtTime(0.16, now + 0.008);
  gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.19);
  oscillator.connect(gain).connect(context.destination);
  oscillator.start(now);
  oscillator.stop(now + 0.2);
}

export function ProtocolPanel({
  renderer, quality, onQualityChange, webglAvailable, paused, ready, strikeAvailable, phase, impactId,
  strikes, recoveries, message, onStrike, onRendererChange, onPauseToggle,
  onReset, onOverview, onFocus,
}: ProtocolPanelProps) {
  const [soundEnabled, setSoundEnabled] = useState(true);
  const audioRef = useRef<AudioContext | null>(null);
  const previousImpactRef = useRef(impactId);
  const canStrike = ready && !paused && strikeAvailable;

  const requestStrike = useCallback(() => {
    if (!canStrike) return;
    if (soundEnabled && typeof window !== "undefined" && window.AudioContext) {
      try {
        audioRef.current ??= new window.AudioContext();
        void audioRef.current.resume().catch(() => {});
      } catch {
        // Audio is optional; a blocked device must not block the procedure.
      }
    }
    onStrike();
  }, [canStrike, onStrike, soundEnabled]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.code !== "KeyP" || event.repeat || event.altKey || event.ctrlKey || event.metaKey) return;
      const target = event.target;
      if (target instanceof HTMLElement && (target.isContentEditable || ["INPUT", "SELECT", "TEXTAREA"].includes(target.tagName))) return;
      event.preventDefault();
      requestStrike();
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [requestStrike]);

  useEffect(() => {
    if (impactId > previousImpactRef.current && soundEnabled && audioRef.current) {
      playMechanicalImpact(audioRef.current);
    }
    previousImpactRef.current = impactId;
  }, [impactId, soundEnabled]);

  useEffect(() => () => { void audioRef.current?.close(); }, []);

  return <>
    <div className="protocol-brand">
      <div className="protocol-eyebrow"><span className="protocol-status-light" /> LABORATOIRE HUMAIN <span>/ SALLE 01</span></div>
      <h1>Protocole <em>d’arrêt</em><span>.</span></h1>
      <p>Une procédure simple. Un sujet obstiné.</p>
    </div>

    <aside className="protocol-tools" aria-label="Réglages de la salle">
      <label className="protocol-renderer">Rendu
        <select value={renderer} onChange={event => onRendererChange(event.target.value as RendererMode)} aria-label="Rendu" data-testid="renderer-picker">
          <option value="webgl" disabled={!webglAvailable}>WebGL</option>
          <option value="canvas2d">Canvas 2D</option>
        </select>
      </label>
      <QualityPicker quality={quality} onChange={onQualityChange} french />
      <button type="button" onClick={onPauseToggle} aria-pressed={paused} data-testid="pause-toggle">{paused ? "Reprendre" : "Pause"}</button>
      <button type="button" onClick={() => setSoundEnabled(value => !value)} aria-pressed={soundEnabled} data-testid="sound-toggle">Son {soundEnabled ? "activé" : "coupé"}</button>
      <button type="button" onClick={onReset} data-testid="reset-button">Nouvelle session</button>
    </aside>

    <div className="protocol-bottom">
      <div className="protocol-readout">
        <div className="protocol-readout-heading"><span className="protocol-readout-index">01 / PROCÉDURE</span><span className={`protocol-phase ${phase}`} data-testid="striker-phase">{paused ? "En pause" : PHASE_LABELS[phase]}</span></div>
        <p className="protocol-message" role="status" aria-live="polite" data-testid="protocol-message">{message ?? "Le sujet attend."}</p>
        <div className="protocol-action-row">
          <button type="button" className="protocol-strike" onClick={requestStrike} disabled={!canStrike} data-testid="strike-button">
            <span className="protocol-strike-symbol" aria-hidden="true">↗</span>
            <span>Appliquer la procédure</span>
            <kbd>P</kbd>
          </button>
          <p>Un appui déclenche une seule tentative.<br />La caméra reste libre pendant le relevage.</p>
        </div>
      </div>
      <div className="protocol-side">
        <dl className="protocol-metrics" aria-label="Mesures de la session">
          <div><dt>Percussions</dt><dd data-testid="strike-count">{strikes.toString().padStart(2, "0")}</dd></div>
          <div><dt>Retours debout</dt><dd data-testid="recovery-count">{recoveries.toString().padStart(2, "0")}</dd></div>
        </dl>
        <div className="protocol-camera">
          <button type="button" onClick={onOverview} disabled={!ready}>Vue d’ensemble</button>
          <button type="button" onClick={onFocus} disabled={!ready}>Cadrer le sujet</button>
        </div>
      </div>
    </div>
    {impactId > 0 ? <div className="protocol-impact" key={impactId} aria-hidden="true" /> : null}
  </>;
}
