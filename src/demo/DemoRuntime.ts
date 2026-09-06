import { createEmbodiedCharacter } from "../character";
import { FixedStepLoop } from "../core/FixedStepLoop";
import type { CharacterController, PoseSnapshot, PoseView, RegionId, RendererMode } from "../core/types";
import { PointerInteraction } from "../interaction";
import { createSharedCamera } from "../scene/camera";
import { createCameraControls } from "../scene/camera-controls";
import { checkGraphicsCapabilities } from "../scene/capabilities";
import { createPoseView } from "../scene";
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

  constructor(options: DemoRuntimeOptions) {
    this.host = options.host;
    this.character = options.character;
    this.capabilities = options.capabilities;
    this.rendererValue = options.renderer;
    this.createView = options.createView ?? createPoseView;
    this.snapshot = this.character.getSnapshot(this.renderer);
    this.previous = this.snapshot;
    this.syncBodyInput();
    this.loop = new FixedStepLoop(this.fixedUpdate, this.render);
    try {
      this.activeView = this.createView(this.renderer, { camera: this.camera });
      this.activeView.mount(this.host);
      this.resizeView(this.activeView);
      this.interaction.attach(this.host, this.camera, (ray) => this.character.pick(ray));
      this.cameraControls.attach(this.host);
      this.present();
      this.resizeObserver = new ResizeObserver(() => this.resizeView(this.view));
      this.resizeObserver.observe(this.host);
      this.frames.reset();
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
  get paused(): boolean { return this.pausedValue; }
  get resetVersion(): number { return this.resetCount; }
  get status(): string {
    if (this.paused) return this.statusValue;
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

  switchRenderer(next: RendererMode): void {
    if (this.disposed || next === this.renderer) return;
    if (next === "webgl" ? !this.capabilities.webgl2 : !this.capabilities.canvas2d) return;
    // Prepare the replacement before releasing the working view.
    const nextView = this.createView(next, { camera: this.camera });
    try {
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
    this.frames.reset();
    this.statusValue = (next === "webgl" ? "WebGL2" : "Canvas2D") + " view active";
    this.notify();
  }

  pause(reason: "pause" | "blur" = "pause"): void {
    if (this.disposed) return;
    this.clearInteraction(reason);
    this.character.pause();
    this.loop.pause();
    this.pausedValue = true;
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
    this.refreshSnapshots();
    this.statusValue = "Ready — drag any highlighted body region";
    this.notify();
  }

  reset(): void {
    if (this.disposed) return;
    this.clearInteraction("reset");
    this.interaction.reset();
    this.character.reset();
    if (this.paused) this.character.pause();
    this.cameraControls.reset();
    this.loop.resetTiming();
    this.frames.reset();
    this.resetCount += 1;
    this.refreshSnapshots();
    this.present();
    this.statusValue = this.paused ? "Reset — paused" : "Reset — ready to pull";
    this.notify();
  }

  stop(): void { this.loop.stop(); }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.loop.stop();
    this.resizeObserver?.disconnect();
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
    view.resize(Math.max(1, rect.width), Math.max(1, rect.height), Math.min(2, dpr));
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
    this.character.fixedUpdate(dt, this.interaction.consumeCommand());
    this.previous = this.current;
    this.snapshot = this.character.getSnapshot(this.renderer);
    // FixedStepLoop may call us repeatedly in one animation frame. Release
    // capture and discard queued moves before it starts the next substep.
    this.syncBodyInput();
  };

  private render = (alpha: number, nowMs: number): void => {
    this.present(alpha);
    this.frames.sample(nowMs);
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
  const character = await createEmbodiedCharacter(renderer);
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
