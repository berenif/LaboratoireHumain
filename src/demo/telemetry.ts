import type { GraphicsCapabilities } from "../scene/capabilities";

export interface BrowserCapabilities extends GraphicsCapabilities {
  browser: string;
  devicePixelRatio: number;
  viewport: { width: number; height: number };
}

export interface FrameSummary {
  samples: number;
  p50Ms: number;
  p95Ms: number;
  p99Ms: number;
  averageFps: number;
}

export function summarizeFrames(samples: readonly number[]): FrameSummary {
  if (!samples.length) return { samples: 0, p50Ms: 0, p95Ms: 0, p99Ms: 0, averageFps: 0 };
  const sorted = [...samples].sort((a, b) => a - b);
  const at = (percentile: number) => sorted[Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * percentile))];
  const mean = sorted.reduce((sum, value) => sum + value, 0) / sorted.length;
  return {
    samples: sorted.length,
    p50Ms: at(0.5),
    p95Ms: at(0.95),
    p99Ms: at(0.99),
    averageFps: mean > 0 ? 1000 / mean : 0,
  };
}

/** Keeps the latest rendered frame intervals, excluding startup and reset warmup. */
export class FrameSampler {
  private samples: number[] = [];
  private lastFrameMs: number | null = null;
  private warmupUntilMs: number;

  constructor(nowMs = performance.now()) {
    this.warmupUntilMs = nowMs + 1000;
  }

  sample(nowMs: number): void {
    if (this.lastFrameMs !== null && nowMs > this.warmupUntilMs) {
      this.samples.push(nowMs - this.lastFrameMs);
      if (this.samples.length > 1200) this.samples.shift();
    }
    this.lastFrameMs = nowMs;
  }

  reset(nowMs = performance.now()): void {
    this.samples = [];
    this.lastFrameMs = null;
    this.warmupUntilMs = nowMs + 1000;
  }

  summary(): FrameSummary {
    return summarizeFrames(this.samples);
  }
}
