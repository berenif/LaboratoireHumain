import type { RenderQuality } from "../core/types";

export function qualitySettings(quality: RenderQuality, deviceRatio: number, autoRatio = 1.25) {
  const cap = quality === "low" ? 0.75 : quality === "high" ? 1.5 : Math.max(0.75, Math.min(1.5, autoRatio));
  return {
    pixelRatio: Math.min(Number.isFinite(deviceRatio) && deviceRatio > 0 ? deviceRatio : 1, cap),
    shadowResolution: quality === "low" || (quality === "auto" && cap < 1.25) ? 512 : 1024,
  };
}

/** Rolling one-second intervals, with hysteresis and a two-second change limit. */
export class AutoQuality {
  ratio = 1.25;
  private readonly intervals = new Float64Array(120);
  private index = 0;
  private count = 0;
  private sum = 0;
  private previous: number | null = null;
  private warmupUntil = 0;
  private lastChange = -Infinity;
  private slowSince: number | null = null;
  private fastSince: number | null = null;

  exclude(nowMs: number): void {
    this.previous = null;
    this.warmupUntil = nowMs + 1000;
    this.count = this.index = this.sum = 0;
    this.fastSince = this.slowSince = null;
  }

  sample(nowMs: number): boolean {
    const before = this.previous;
    this.previous = nowMs;
    if (before === null || nowMs < this.warmupUntil) return false;
    const interval = nowMs - before;
    if (interval <= 0) return false;
    // Keep approximately one second, bounded even on high-refresh displays.
    while (this.count && (this.sum >= 1000 || this.count === this.intervals.length)) {
      this.sum -= this.intervals[(this.index - this.count + this.intervals.length) % this.intervals.length];
      this.count--;
    }
    this.intervals[this.index] = interval;
    this.index = (this.index + 1) % this.intervals.length;
    this.sum += interval;
    this.count++;
    if (this.sum < 750) return false;
    const mean = this.sum / this.count;
    this.slowSince = mean > 20 ? this.slowSince ?? nowMs : null;
    this.fastSince = mean < 17 ? this.fastSince ?? nowMs : null;
    if (nowMs - this.lastChange < 2000) return false;
    const next = this.slowSince !== null && nowMs - this.slowSince >= 1000
      ? Math.max(0.75, this.ratio - 0.25)
      : this.fastSince !== null && nowMs - this.fastSince >= 5000
        ? Math.min(1.5, this.ratio + 0.25) : this.ratio;
    if (next === this.ratio) return false;
    this.ratio = next;
    this.lastChange = nowMs;
    this.exclude(nowMs);
    return true;
  }
}
