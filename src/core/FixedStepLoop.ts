import { WORLD } from "./types";

export interface LoopStats {
  fixedSteps: number;
  droppedTimeMs: number;
  lastStepMs: number;
  meanStepMs: number;
  maxStepMs: number;
}

export class FixedStepLoop {
  private frame = 0;
  private previousTimeMs: number | null = null;
  private accumulator = 0;
  private paused = false;
  private stepSamples = 0;
  private statsValue: LoopStats = {
    fixedSteps: 0,
    droppedTimeMs: 0,
    lastStepMs: 0,
    meanStepMs: 0,
    maxStepMs: 0,
  };

  constructor(
    private readonly fixedUpdate: (dt: number) => void,
    private readonly render: (alpha: number, nowMs: number) => void,
  ) {}

  start(): void {
    if (this.frame) return;
    this.previousTimeMs = null;
    this.frame = requestAnimationFrame(this.tick);
  }

  stop(): void {
    if (this.frame) cancelAnimationFrame(this.frame);
    this.frame = 0;
    this.previousTimeMs = null;
    this.accumulator = 0;
  }

  pause(): void {
    this.paused = true;
    this.previousTimeMs = null;
    this.accumulator = 0;
  }

  resume(): void {
    this.paused = false;
    this.previousTimeMs = null;
    this.accumulator = 0;
  }

  resetTiming(): void {
    this.previousTimeMs = null;
    this.accumulator = 0;
    this.stepSamples = 0;
    this.statsValue = { fixedSteps: 0, droppedTimeMs: 0, lastStepMs: 0, meanStepMs: 0, maxStepMs: 0 };
  }

  stats(): Readonly<LoopStats> {
    return this.statsValue;
  }

  private tick = (nowMs: number): void => {
    this.frame = requestAnimationFrame(this.tick);
    if (this.paused) {
      this.previousTimeMs = null;
      this.render(0, nowMs);
      return;
    }

    if (this.previousTimeMs === null) this.previousTimeMs = nowMs;
    const elapsed = Math.min(0.25, Math.max(0, (nowMs - this.previousTimeMs) / 1000));
    this.previousTimeMs = nowMs;
    this.accumulator += elapsed;

    let steps = 0;
    while (this.accumulator >= WORLD.fixedDt && steps < WORLD.maxCatchUpSteps) {
      const start = performance.now();
      this.fixedUpdate(WORLD.fixedDt);
      const duration = performance.now() - start;
      this.stepSamples += 1;
      this.statsValue.fixedSteps += 1;
      this.statsValue.lastStepMs = duration;
      this.statsValue.maxStepMs = Math.max(this.statsValue.maxStepMs, duration);
      this.statsValue.meanStepMs += (duration - this.statsValue.meanStepMs) / this.stepSamples;
      this.accumulator -= WORLD.fixedDt;
      steps += 1;
    }

    if (this.accumulator >= WORLD.fixedDt) {
      const dropped = this.accumulator - (this.accumulator % WORLD.fixedDt);
      this.statsValue.droppedTimeMs += dropped * 1000;
      this.accumulator %= WORLD.fixedDt;
    }

    this.render(this.accumulator / WORLD.fixedDt, nowMs);
  };
}
