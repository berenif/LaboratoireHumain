import { createEmbodiedCharacter } from "../character";
import { coordinatedStandingOptions } from "../character/standing-selection";
import { characterFrame } from "../character/character-frame";
import { FixedStepLoop } from "../core/FixedStepLoop";
import { DEFAULT_PLAYGROUND, type PlaygroundConfig, type PlaygroundTrial } from "../core/playground";
import type { CharacterController, PoseSnapshot, PoseView, RegionId, RendererMode, RenderQuality } from "../core/types";
import { AutoQuality } from "../scene/quality";
import { PointerInteraction } from "../interaction";
import { createSharedCamera } from "../scene/camera";
import { createCameraControls } from "../scene/camera-controls";
import { checkGraphicsCapabilities } from "../scene/capabilities";
import { createPoseView } from "../scene";
import { PROTOCOL_ARENA_CAMERA } from "../scene/protocol-visuals";
import type { SceneViewOptions } from "../scene/options";
import { FrameSampler } from "./telemetry";
import type { BrowserCapabilities } from "./telemetry";

type RuntimeListener = (runtime: DemoRuntime, nowMs: number) => void;
type ViewFactory = (renderer: RendererMode, options: SceneViewOptions) => PoseView;

interface DemoRuntimeOptions {
  host: HTMLElement;
  character: CharacterController;
  capabilities: BrowserCapabilities;
  renderer: RendererMode;
  createView?: ViewFactory;
}

/** Owns one simulation, its browser resources, and the selected presentation adapter. */
export class DemoRuntime {
  readonly host: HTMLElement;
  readonly character: CharacterController;
  readonly capabilities: BrowserCapabilities;
  readonly camera = createSharedCamera();
  readonly interaction = new PointerInteraction();
  readonly cameraControls = createCameraControls(this.camera, {
    allowSingleTouchOrbit: true,
    canStartPointer: () => this.interaction.getActivePointerId() === null,
    canZoomWheel: () => this.interaction.getActivePointerId() === null,
  });

  private readonly createView: ViewFactory;
  private readonly frames = new FrameSampler();
  private readonly autoQuality = new AutoQuality();
  private qualityValue: RenderQuality = "auto";
  private unsubscribeCamera: (() => void) | null = null;
  private readonly listeners = new Set<RuntimeListener>();
  private readonly loop: FixedStepLoop;
  private resizeObserver: ResizeObserver | null = null;
  private activeView: PoseView | null = null;
  private rendererValue: RendererMode;
  private previous: PoseSnapshot;
  private snapshot: PoseSnapshot;
  private pausedValue = false;
  private resetCount = 0;
  private lastUiMs = 0;
  private statusValue = "Ready — drag any highlighted body region";
  private disposed = false;
  private cameraMode: "arena" | "subject" = "arena";
  private trialValue: PlaygroundTrial = { uprightSeconds: 0, bestSeconds: 0, falls: 0 };

  constructor(options: DemoRuntimeOptions) {
    this.host = options.host;
    this.character = options.character;
    this.capabilities = options.capabilities;
    this.rendererValue = options.renderer;
    this.createView = options.createView ?? createPoseView;
    this.snapshot = this.character.getSnapshot(this.renderer);
    this.previous = this.snapshot;
    if (this.snapshot.playground || this.snapshot.room) this.positionCamera();
    this.syncBodyInput();
    this.loop = new FixedStepLoop(this.fixedUpdate, this.render);
    try {
      this.activeView = this.createView(this.renderer, { camera: this.camera });
      this.activeView.setQuality(this.qualityValue, this.autoQuality.ratio);
      this.activeView.mount(this.host);
      this.resizeView(this.activeView);
      this.interaction.attach(this.host, this.camera, (ray) => this.character.pick(ray));
      this.cameraControls.attach(this.host);
      this.present();
      this.unsubscribeCamera = this.camera.subscribe(() => this.loop.requestRender());
      this.resizeObserver = new ResizeObserver(() => {
        this.resizeView(this.view);
        this.excludeTiming();
        this.loop.requestRender();
      });
      this.resizeObserver.observe(this.host);
      this.frames.reset();
      this.autoQuality.exclude(performance.now());
      this.loop.start();
    } catch (error) {
      this.dispose();
      throw error;
    }
  }

  get view(): PoseView {
    if (!this.activeView) throw new Error("The demo view is not mounted.");
    return this.activeView;
  }

  get current(): PoseSnapshot { return this.snapshot; }
  get renderer(): RendererMode { return this.rendererValue; }
  get quality(): RenderQuality { return this.qualityValue; }
  get renderMetrics() { return this.view.getMetrics(); }

  setQuality(quality: RenderQuality): void {
    if (this.disposed || quality === this.qualityValue) return;
    this.qualityValue = quality;
    this.view.setQuality(quality, this.autoQuality.ratio);
    this.excludeTiming();
    this.loop.requestRender();
    this.notify();
  }

  private excludeTiming(): void {
    this.autoQuality.exclude(performance.now());
    this.frames.reset();
  }
  get paused(): boolean { return this.pausedValue; }
  get resetVersion(): number { return this.resetCount; }
  get trial(): PlaygroundTrial { return { ...this.trialValue }; }
  get timing() { return { ...this.loop.stats() }; }
  get status(): string {
    if (this.paused) return this.statusValue;
    if (this.current.room) return this.current.protocol?.message ?? "Prêt pour la procédure";
    if (!this.current.diagnostics.bodyInputAvailable) {
      return this.current.state === "recovering"
        ? "Getting up — body control resumes after stable standing"
        : "Protecting the fall — body control resumes after recovery";
    }
    return this.statusValue;
  }

  subscribe(listener: RuntimeListener): () => void {
    this.listeners.add(listener);
    return () => { this.listeners.delete(listener); };
  }

  frameSummary() { return this.frames.summary(); }

  regionPoint(region: RegionId) {
    const pose = this.current.segments.find((segment) => segment.id === region);
    if (!pose) return null;
    const projected = this.camera.project(pose.position);
    const rect = this.host.getBoundingClientRect();
    return { x: rect.left + projected.x, y: rect.top + projected.y, visible: projected.visible };
  }

  requestStrike(): boolean {
    if (this.disposed || this.paused) return false;
    const accepted = this.character.requestStrike();
    if (accepted) {
      this.refreshSnapshots();
      this.notify();
    }
    return accepted;
  }

  switchRenderer(next: RendererMode): void {
    if (this.disposed || next === this.renderer) return;
    if (next === "webgl" ? !this.capabilities.webgl2 : !this.capabilities.canvas2d) return;
    // Prepare the replacement before releasing the working view.
    const nextView = this.createView(next, { camera: this.camera });
    try {
      nextView.setQuality(this.qualityValue, this.autoQuality.ratio);
      nextView.mount(this.host);
      this.resizeView(nextView);
    } catch (error) {
      nextView.dispose();
      throw error;
    }
    this.clearInteraction("cancel");
    this.interaction.detach();
    this.view.dispose();
    this.activeView = nextView;
    this.rendererValue = next;
    this.refreshSnapshots();
    this.present();
    this.interaction.attach(this.host, this.camera, (ray) => this.character.pick(ray));
    this.excludeTiming();
    this.statusValue = (next === "webgl" ? "WebGL2" : "Canvas2D") + " view active";
    this.notify();
  }

  pause(reason: "pause" | "blur" = "pause"): void {
    if (this.disposed) return;
    this.clearInteraction(reason);
    this.character.pause();
    this.loop.pause();
    this.pausedValue = true;
    this.excludeTiming();
    this.refreshSnapshots();
    this.statusValue = reason === "blur" ? "Paused after focus loss" : "Paused";
    this.notify();
  }

  togglePause(): void {
    if (this.disposed) return;
    if (!this.paused) {
      this.pause();
      return;
    }
    this.interaction.reset();
    this.character.resume();
    this.loop.resume();
    this.pausedValue = false;
    this.excludeTiming();
    this.refreshSnapshots();
    this.statusValue = "Ready — drag any highlighted body region";
    this.notify();
  }

  reset(): void {
    if (this.disposed) return;
    this.clearInteraction("reset");
    this.interaction.reset();
    this.character.reset();
    this.finishReset();
  }

  setPlayground(change: Partial<PlaygroundConfig>): void {
    if (this.disposed) return;
    this.clearInteraction("reset");
    this.interaction.reset();
    this.character.setPlayground({ ...(this.current.playground ?? DEFAULT_PLAYGROUND), ...change });
    this.cameraMode = "subject";
    this.finishReset();
  }

  showArena(): void {
    this.cameraMode = "arena";
    this.positionCamera();
    this.present();
  }

  focusSubject(): void {
    this.cameraMode = "subject";
    this.positionCamera();
    this.present();
  }

  private positionCamera(): void {
    if (this.current.room) {
      const root = this.current.rootPosition;
      const target = this.cameraMode === "subject"
        ? { x: root.x, y: Math.max(0.7, root.y), z: root.z }
        : PROTOCOL_ARENA_CAMERA.target;
      this.camera.setView(this.cameraMode === "subject"
        ? { x: target.x + 2, y: target.y + 1.8, z: target.z + 3.8 }
        : PROTOCOL_ARENA_CAMERA.position, target);
      return;
    }
    if (!this.current.playground) return;
    if (this.cameraMode === "arena") {
      this.camera.setView({ x: 10.5, y: 12.5, z: 17 }, { x: 0, y: 0.2, z: -0.4 });
    } else {
      const root = this.current.rootPosition;
      const target = { x: root.x, y: Math.max(0.7, root.y - 0.1), z: root.z };
      this.camera.setView({ x: target.x + 2.4, y: target.y + 2.4, z: target.z + 5.4 }, target);
    }
  }

  private finishReset(): void {
    if (this.paused) this.character.pause();
    this.cameraControls.reset();
    this.loop.resetTiming();
    this.frames.reset();
    this.autoQuality.exclude(performance.now());
    this.resetCount += 1;
    this.trialValue = { uprightSeconds: 0, bestSeconds: 0, falls: 0 };
    this.refreshSnapshots();
    this.positionCamera();
    this.present();
    this.statusValue = this.current.room
      ? this.paused ? "Remise à zéro — pause" : "Prêt pour la procédure"
      : this.paused ? "Reset — paused" : "Reset — ready to pull";
    this.notify();
  }

  stop(): void { this.loop.stop(); }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.loop.stop();
    this.resizeObserver?.disconnect();
    this.unsubscribeCamera?.();
    this.cameraControls.dispose();
    this.interaction.detach();
    this.activeView?.dispose();
    this.activeView = null;
    this.character.dispose();
    this.listeners.clear();
  }

  private resizeView(view: PoseView): void {
    const rect = this.host.getBoundingClientRect();
    const dpr = this.host.ownerDocument.defaultView?.devicePixelRatio || 1;
    view.resize(Math.max(1, rect.width), Math.max(1, rect.height), dpr);
  }

  private clearInteraction(reason: Parameters<PointerInteraction["clear"]>[0]): void {
    this.interaction.clear(reason);
    // Cancellation is bookkeeping, never a simulation integration. In
    // particular, pause and renderer changes must not advance recovery.
    this.interaction.consumeCommand();
    this.character.clearBodyInput();
  }

  private refreshSnapshots(): void {
    this.snapshot = this.character.getSnapshot(this.renderer);
    this.previous = this.snapshot;
    this.syncBodyInput();
  }

  private syncBodyInput(): void {
    this.interaction.setBodyInputAvailable(!this.paused && this.snapshot.diagnostics.bodyInputAvailable);
  }

  private present(alpha = 0): void {
    this.view.setSnapshot(this.previous, this.current, alpha);
    this.view.render();
  }

  private fixedUpdate = (dt: number): void => {
    this.syncBodyInput();
    this.previous = this.current;
    this.snapshot = characterFrame(this.character, dt, this.interaction.consumeCommand(), this.renderer);
    if (this.snapshot.playground) {
      const falling = ["falling", "fallen", "recovering"].includes(this.snapshot.state);
      const wasFalling = ["falling", "fallen", "recovering"].includes(this.previous.state);
      if (falling && !wasFalling) this.trialValue.falls++;
      const supported = this.snapshot.support.planted.length > 0;
      this.trialValue.uprightSeconds = !falling && supported ? this.trialValue.uprightSeconds + dt : 0;
      this.trialValue.bestSeconds = Math.max(this.trialValue.bestSeconds, this.trialValue.uprightSeconds);
    }
    // FixedStepLoop may call us repeatedly in one animation frame. Release
    // capture and discard queued moves before it starts the next substep.
    this.syncBodyInput();
  };

  private render = (alpha: number, nowMs: number): void => {
    const active = !this.paused && !this.host.ownerDocument.hidden;
    if (active && this.qualityValue === "auto" && this.autoQuality.sample(nowMs)) {
      this.view.setQuality("auto", this.autoQuality.ratio);
    }
    this.present(alpha);
    if (active) this.frames.sample(nowMs);
    else this.autoQuality.exclude(nowMs);
    if (nowMs - this.lastUiMs > 100) {
      this.lastUiMs = nowMs;
      this.notify(nowMs);
    }
  };

  private notify(nowMs = performance.now()): void {
    for (const listener of this.listeners) listener(this, nowMs);
  }
}

export async function createDemoRuntime(host: HTMLElement, signal: AbortSignal): Promise<DemoRuntime> {
  signal.throwIfAborted();
  const capabilities: BrowserCapabilities = {
    ...checkGraphicsCapabilities(),
    browser: navigator.userAgent,
    devicePixelRatio: window.devicePixelRatio || 1,
    viewport: { width: window.innerWidth, height: window.innerHeight },
  };
  if (!capabilities.canvas2d && !capabilities.webgl2) {
    throw new Error("Neither Canvas2D nor WebGL2 is available.");
  }
  const renderer = capabilities.webgl2 ? "webgl" : "canvas2d";
  const playgroundMode = new URLSearchParams(window.location.search).get("mode") === "playground";
  const standingCandidate = new URLSearchParams(window.location.search).get("standingCandidate");
  const character = await createEmbodiedCharacter(renderer,
    standingCandidate ? coordinatedStandingOptions(standingCandidate)
      : playgroundMode ? { playground: { ...DEFAULT_PLAYGROUND } } : { room: true });
  if (signal.aborted) {
    character.dispose();
    signal.throwIfAborted();
  }
  try {
    return new DemoRuntime({ host, character, capabilities, renderer });
  } catch (error) {
    character.dispose();
    throw error;
  }
}
