"use client";

import type { CSSProperties } from "react";
import { DIFFICULTY_LABELS, PLAYGROUND_STATIONS, playgroundStation,
  type PlaygroundConfig, type PlaygroundDifficulty, type PlaygroundStation, type PlaygroundTrial } from "../core/playground";

const GLYPHS: Record<PlaygroundStation, string> = {
  flat: "M3 20h34M13 14V8m7 6V4m7 10V8",
  slope: "M3 21h34L30 3 3 21m12-7 3 5m6-11 3 5",
  rubble: "m2 20 8-7 6 3 7-12 6 9 9 7M2 23h36",
  beam: "M3 9h34M3 13h34M9 13v10m22-10v10M18 2h4",
  stones: "M2 20h8v-4H2v4Zm13-5h9v-5h-9v5Zm14-7h9V2h-9v6Z",
  wobble: "m3 7 34 8M17 13l-5 10h15l-5-9M4 18l3 3m26-15 3 3",
  hurdles: "M3 23V10h9v13m5 0V5h9v18m5 0V1h7v22",
};

function StationGlyph({ station }: { station: PlaygroundStation }) {
  return <svg viewBox="0 0 40 26" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d={GLYPHS[station]} /></svg>;
}

interface PlaygroundPanelProps {
  config: PlaygroundConfig;
  trial: PlaygroundTrial;
  supportCount: number;
  ready: boolean;
  onChange: (change: Partial<PlaygroundConfig>) => void;
  onOverview: () => void;
  onFocus: () => void;
}

export function PlaygroundPanel({ config, trial, supportCount, ready, onChange, onOverview, onFocus }: PlaygroundPanelProps) {
  const active = playgroundStation(config.station);
  return <>
    <div className="playground-brand">
      <span className="playground-eyebrow"><span className="lab-light" /> LABORATOIRE HUMAIN <span className="lab-version">/ 02</span></span>
      <h2>Balance playground<span>.</span></h2>
      <p>Seven ways to lose your footing.</p>
    </div>
    <aside className="playground-panel" aria-label="Balance playground" data-testid="playground-panel">
      <div className="playground-toolbar">
        <div className="playground-instruction">
          <span className="station-dot" style={{ backgroundColor: active.color }} />
          <div><strong>{active.name}</strong><p>{active.instruction}</p></div>
        </div>
        <label className="difficulty-picker">Difficulty
          <select aria-label="Course difficulty" value={config.difficulty} disabled={!ready}
            onChange={event => onChange({ difficulty: event.target.value as PlaygroundDifficulty })}>
            {Object.entries(DIFFICULTY_LABELS).map(([id, label]) => <option value={id} key={id}>{label}</option>)}
          </select>
        </label>
      </div>
      <div className="playground-stations" role="group" aria-label="Choose a balance station">
        {PLAYGROUND_STATIONS.map((station, index) => <button type="button" key={station.id}
          className="station-button" style={{ "--station-color": station.color } as CSSProperties}
          aria-pressed={config.station === station.id} disabled={!ready}
          onClick={() => onChange({ station: station.id })} data-testid={`station-${station.id}`}>
          <span className="station-topline"><StationGlyph station={station.id} /><span>{String(index + 1).padStart(2, "0")}</span></span>
          <strong>{station.name}</strong><small>{station.subtitle}</small>
        </button>)}
      </div>
      <div className="playground-footer">
        <dl className="trial-metrics" aria-label="Current trial measurements">
          <div><dt>Upright</dt><dd data-testid="upright-time">{trial.uprightSeconds.toFixed(1)}<span>s</span></dd></div>
          <div><dt>Best this trial</dt><dd>{trial.bestSeconds.toFixed(1)}<span>s</span></dd></div>
          <div><dt>Falls</dt><dd data-testid="trial-falls">{trial.falls}</dd></div>
          <div><dt>Foot support</dt><dd>{supportCount}<span>/ 2</span></dd></div>
        </dl>
        <div className="playground-camera">
          <button type="button" onClick={onOverview} disabled={!ready} title="Show the whole arena">Arena view <span aria-hidden="true">↗</span></button>
          <button type="button" onClick={onFocus} disabled={!ready} title="Bring the subject closer">Focus body <span aria-hidden="true">⊙</span></button>
        </div>
      </div>
    </aside>
  </>;
}
