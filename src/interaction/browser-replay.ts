import { V3 } from "../core/math";
import { REGION_IDS } from "../core/types";
import type { CameraProjection, DiagnosticsSnapshot, PoseSnapshot, RegionId, Vec3 } from "../core/types";
import type { PointerInteraction } from "./index";
import { pickRegionProxies } from "./picking";

export interface InteractionReplayAdapter {
  host: HTMLElement;
  interaction: PointerInteraction;
  diagnostics: () => DiagnosticsSnapshot;
  snapshot: () => PoseSnapshot;
  projection: () => CameraProjection;
}

export interface ReplayRecord {
  name: string;
  passed: boolean;
  failures: string[];
  metrics: Record<string, unknown>;
}

export interface InteractionReplayOptions {
  suite?: "selection" | "lifecycle" | "cycles" | "all";
  pointerType?: "mouse" | "touch";
  cycles?: number;
  onProgress?: (stage: string, record?: ReplayRecord) => void;
}

export interface InteractionReplayResult {
  schema: number;
  kind: string;
  synthetic: true;
  trusted: false;
  pointerType: "mouse" | "touch";
  capturedAt: string;
  elapsedMs: number;
  passed: boolean;
  results: ReplayRecord[];
}

const wait = (ms: number) => new Promise<void>((resolve) => window.setTimeout(resolve, ms));
const maxSpeed = (snapshot: PoseSnapshot) => Math.max(...snapshot.segments.map((pose) => V3.length(pose.linearVelocity)));

/**
 * Browser DOM replay: every grab/move/end enters installed PointerEvent listeners.
 * Events are synthetic (isTrusted=false); this never claims native touch/capture.
 * Physics remains on the application's fixed clock. No direct grab API is used.
 */
export async function runInteractionReplay(
  adapter: InteractionReplayAdapter,
  options: InteractionReplayOptions = {},
): Promise<InteractionReplayResult> {
  const started = performance.now();
  const pointerType = options.pointerType ?? "mouse";
  const suite = options.suite ?? "all";
  const results: ReplayRecord[] = [];
  let pointerId = 4000;
  let clientPoint = { x: 0, y: 0 };
  const uiButton = (id: string) => {
    const button = adapter.host.ownerDocument.querySelector<HTMLButtonElement>(`[data-testid="${id}"]`);
    if (!button) throw new Error(`Replay UI button missing: ${id}`);
    return button;
  };
  const resetUI = async () => {
    const pause = uiButton("pause-toggle");
    if (pause.getAttribute("aria-pressed") === "true") {
      pause.click();
      await wait(60);
    }
    uiButton("reset-button").click();
    await wait(100);
  };
  const dispatch = (type: string, point = clientPoint, id = pointerId, isPrimary = true) => {
    clientPoint = { ...point };
    adapter.host.dispatchEvent(new PointerEvent(type, {
      bubbles: true,
      cancelable: true,
      pointerId: id,
      pointerType,
      isPrimary,
      clientX: point.x,
      clientY: point.y,
      button: type === "pointermove" ? -1 : 0,
      buttons: type === "pointerup" || type === "pointercancel" ? 0 : 1,
      pressure: type === "pointerup" || type === "pointercancel" ? 0 : 0.5,
    }));
  };
  const screenPoint = (world: Vec3) => {
    const point = adapter.projection().project(world);
    const rect = adapter.host.getBoundingClientRect();
    return { x: rect.left + point.x, y: rect.top + point.y };
  };
  const regionPoint = (region: RegionId) => {
    const pose = adapter.snapshot().segments.find((segment) => segment.id === region);
    if (!pose) throw new Error(`Replay segment missing: ${region}`);
    return screenPoint(pose.position);
  };
  const begin = async (region: RegionId) => {
    pointerId += 1;
    dispatch("pointerdown", regionPoint(region));
    await wait(65);
    return adapter.interaction.getStatus();
  };
  const releasedFailures = (): string[] => {
    const diagnostics = adapter.diagnostics();
    const failures: string[] = [];
    if (adapter.interaction.getActivePointerId() !== null || diagnostics.activeGrab) failures.push("grab remained active");
    if (diagnostics.appliedGrabForceN >= 0.01) failures.push(`released force ${diagnostics.appliedGrabForceN}N`);
    if (!diagnostics.finite || diagnostics.errors.length) failures.push("nonfinite state or application diagnostic error");
    return failures;
  };
  const record = (name: string, failures: string[], metrics: Record<string, unknown>) => {
    const result = { name, passed: failures.length === 0, failures, metrics };
    results.push(result);
    options.onProgress?.(name, result);
  };
  const visibleSelectableRegions = (): RegionId[] => {
    const snapshot = adapter.snapshot();
    const projection = adapter.projection();
    const rect = adapter.host.getBoundingClientRect();
    return REGION_IDS.filter((region) => {
      const point = regionPoint(region);
      if (point.x < rect.left || point.x > rect.right || point.y < rect.top || point.y > rect.bottom) return false;
      const topElement = document.elementFromPoint(point.x, point.y);
      if (!topElement || !adapter.host.contains(topElement)) return false;
      return pickRegionProxies(projection.screenToRay(point.x, point.y, rect), snapshot.segments)?.region === region;
    });
  };
  const captureVisibilityMiss = (selected: RegionId, elapsedMs: number, includeSnapshot: boolean) => {
    const snapshot = adapter.snapshot();
    const projection = adapter.projection();
    const rect = adapter.host.getBoundingClientRect();
    const panel = document.querySelector<HTMLElement>('[data-testid="control-panel"]');
    const panelRect = panel?.getBoundingClientRect();
    const checks = [...new Set<RegionId>([selected, "torso", "pelvis"])].map((region) => {
      const pose = snapshot.segments.find((segment) => segment.id === region)!;
      const projected = projection.project(pose.position);
      const point = { x: rect.left + projected.x, y: rect.top + projected.y };
      const insideHost = point.x >= rect.left && point.x <= rect.right && point.y >= rect.top && point.y <= rect.bottom;
      const insideViewport = point.x >= 0 && point.x < window.innerWidth && point.y >= 0 && point.y < window.innerHeight;
      const topElement = document.elementFromPoint(point.x, point.y);
      const hostReceivesInput = Boolean(topElement && adapter.host.contains(topElement));
      const pickedRegion = pickRegionProxies(projection.screenToRay(point.x, point.y, rect), snapshot.segments)?.region ?? null;
      return {
        region, point, depthM: projected.depth, insideHost, insideViewport, pickedRegion, hostReceivesInput,
        reason: !insideHost || !insideViewport ? "offscreen"
          : !hostReceivesInput ? "covered-by-dom"
            : pickedRegion !== region ? "nearest-other-region" : "selectable",
        topElement: topElement ? {
          tag: topElement.tagName,
          testId: topElement.getAttribute("data-testid"),
          className: topElement.getAttribute("class"),
          insideControlPanel: Boolean(topElement.closest('[data-testid="control-panel"]')),
        } : null,
      };
    });
    return {
      elapsedMs, sequence: snapshot.sequence, rootPosition: snapshot.rootPosition, state: snapshot.state, checks,
      hostRect: { left: rect.left, top: rect.top, right: rect.right, bottom: rect.bottom },
      viewport: { width: window.innerWidth, height: window.innerHeight },
      panelRect: panelRect ? { left: panelRect.left, top: panelRect.top, right: panelRect.right, bottom: panelRect.bottom } : null,
      panelCompact: panel?.dataset.compact ?? null,
      ...(includeSnapshot ? { snapshot } : {}),
    };
  };

  try {
    if (suite === "all" || suite === "selection") {
      for (const region of REGION_IDS) {
        options.onProgress?.(`selecting ${region}`);
        await resetUI();
        const status = await begin(region);
        const failures = status.selectedRegion === region ? [] : [`visible ${region} center selected ${status.selectedRegion}`];
        const diagnostics = adapter.diagnostics();
        if (diagnostics.selectedRegion !== region) failures.push(`simulation selected ${diagnostics.selectedRegion}`);
        dispatch("pointerup");
        await wait(70);
        failures.push(...releasedFailures());
        record(`center-select-${region}`, failures, { selectedRegion: status.selectedRegion, localAnchor: status.localAnchor, coordinate: clientPoint });
      }
    }
    if (suite === "all" || suite === "lifecycle") {
      for (const action of ["release", "cancel", "lost-capture", "focus-loss", "pause", "reset"] as const) {
        options.onProgress?.(`interrupt ${action}`);
        await resetUI();
        const initial = await begin("rightHand");
        dispatch("pointermove", { x: clientPoint.x + 22, y: clientPoint.y - 3 });
        await wait(65);
        const before = adapter.diagnostics();
        if (action === "release") dispatch("pointerup");
        if (action === "cancel") dispatch("pointercancel");
        if (action === "lost-capture") dispatch("lostpointercapture");
        if (action === "focus-loss") window.dispatchEvent(new Event("blur"));
        if (action === "pause") uiButton("pause-toggle").click();
        if (action === "reset") uiButton("reset-button").click();
        await wait(100);
        const after = adapter.diagnostics();
        const failures = releasedFailures();
        if (initial.selectedRegion !== "rightHand" || !before.activeGrab) failures.push("pre-interruption grab was not active");
        if (action === "reset" && (after.authority !== "character-motor" || after.stepCount !== 0)) failures.push("reset did not restore upright controller");
        record(`interrupt-${action}`, failures, { before, after, inputStatus: adapter.interaction.getStatus(), eventSource: "synthetic DOM event / integrated UI click" });
      }
      await resetUI();
      await begin("rightHand");
      const before = adapter.interaction.getStatus();
      const cameraBefore = adapter.projection().getState();
      const primaryPoint = { ...clientPoint };
      dispatch("pointerdown", regionPoint("leftHand"), pointerId + 100, false);
      dispatch("pointermove", { x: 20, y: 20 }, pointerId + 100, false);
      dispatch("pointerup", { x: 20, y: 20 }, pointerId + 100, false);
      await wait(70);
      const after = adapter.interaction.getStatus();
      const cameraAfter = adapter.projection().getState();
      const failures: string[] = [];
      if (after.activePointerId !== before.activePointerId || after.selectedRegion !== before.selectedRegion) failures.push("secondary pointer stole grab");
      if (JSON.stringify(after.worldTarget) !== JSON.stringify(before.worldTarget)) failures.push("secondary pointer changed target");
      if (JSON.stringify(cameraAfter) !== JSON.stringify(cameraBefore)) failures.push("secondary pointer moved camera");
      if (after.ignoredSecondaryPointers <= before.ignoredSecondaryPointers) failures.push("secondary pointer was not observed");
      dispatch("pointerup", primaryPoint);
      await wait(70);
      failures.push(...releasedFailures());
      record("secondary-pointer-isolation", failures, { before, after, cameraBefore, cameraAfter });
    }
    if (suite === "all" || suite === "cycles") {
      const cycles = Math.max(1, Math.min(5, options.cycles ?? 5));
      for (let cycle = 0; cycle < cycles; cycle += 1) {
        const region: RegionId = cycle % 2 === 0 ? "rightHand" : "leftHand";
        const sign = region === "rightHand" ? 1 : -1;
        options.onProgress?.(`cycle ${cycle + 1}/${cycles}: step`);
        await resetUI();
        const grabbed = await begin(region);
        const failures: string[] = [];
        if (grabbed.selectedRegion !== region || !grabbed.worldTarget) throw new Error(`cycle ${cycle + 1}: ${region} center not selected`);
        const startWorld = grabbed.worldTarget;
        let maxPelvisHeightM = 0;
        let maxRootDisplacementM = 0;
        let maxJointSeparationM = 0;
        let maxFloorPenetrationM = 0;
        let finite = true;
        const observe = () => {
          const snapshot = adapter.snapshot();
          maxPelvisHeightM = Math.max(maxPelvisHeightM, snapshot.rootPosition.y);
          maxRootDisplacementM = Math.max(maxRootDisplacementM, snapshot.diagnostics.rootDisplacementM);
          maxJointSeparationM = Math.max(maxJointSeparationM, snapshot.diagnostics.maxJointSeparationM);
          maxFloorPenetrationM = Math.max(maxFloorPenetrationM, snapshot.diagnostics.maxFloorPenetrationM);
          finite &&= snapshot.diagnostics.finite && snapshot.diagnostics.errors.length === 0;
        };
        for (let tick = 1; tick <= 45; tick += 1) {
          dispatch("pointermove", screenPoint(V3.add(startWorld, { x: sign * 0.46 * tick / 45, y: 0, z: 0.08 * tick / 45 })));
          await wait(17);
          observe();
        }
        await wait(350);
        observe();
        const stepSnapshot = adapter.snapshot();
        if (stepSnapshot.diagnostics.stepCount < 1) failures.push("no corrective step");
        options.onProgress?.(`cycle ${cycle + 1}/${cycles}: handoff`);
        for (let tick = 1; tick <= 5; tick += 1) {
          const t = tick / 5;
          dispatch("pointermove", screenPoint(V3.add(startWorld, { x: sign * (0.46 + 0.494 * t), y: 0.485 * t, z: 0.08 - 0.786 * t })));
          await wait(17);
          observe();
        }
        await wait(400);
        observe();
        const handoffSnapshot = adapter.snapshot();
        if (handoffSnapshot.diagnostics.authority !== "ragdoll" || !handoffSnapshot.diagnostics.activeGrab) failures.push("connected ragdoll handoff with retained grab missing");
        options.onProgress?.(`cycle ${cycle + 1}/${cycles}: fallen drag 3 seconds`);
        let visibleSamples = 0;
        let totalSamples = 0;
        const visibilityMisses: ReturnType<typeof captureVisibilityMiss>[] = [];
        const dragStarted = performance.now();
        while (performance.now() - dragStarted < 3000) {
          const elapsed = (performance.now() - dragStarted) / 1000;
          const torso = adapter.snapshot().segments.find((segment) => segment.id === "torso")!;
          dispatch("pointermove", screenPoint(V3.add(torso.position, { x: sign * (0.28 + Math.sin(elapsed * 2) * 0.1), y: 0.05, z: 0 })));
          await wait(33);
          observe();
          const selectable = visibleSelectableRegions();
          if (selectable.includes(region) || selectable.includes("torso") || selectable.includes("pelvis")) visibleSamples += 1;
          else visibilityMisses.push(captureVisibilityMiss(region, performance.now() - dragStarted, visibilityMisses.length === 0));
          totalSamples += 1;
        }
        const beforeRelease = adapter.snapshot();
        dispatch("pointerup");
        await wait(65);
        const afterRelease = adapter.snapshot();
        observe();
        failures.push(...releasedFailures());
        if (visibleSamples !== totalSamples) failures.push(`visible/selectable fallen drag ${visibleSamples}/${totalSamples}`);
        if (maxPelvisHeightM > 2.4) failures.push(`pelvis height ${maxPelvisHeightM.toFixed(4)}m`);
        if (maxRootDisplacementM > 3) failures.push(`horizontal displacement ${maxRootDisplacementM.toFixed(4)}m`);
        if (maxJointSeparationM > 0.08) failures.push(`joint separation ${maxJointSeparationM.toFixed(4)}m`);
        if (maxFloorPenetrationM > 0.08) failures.push(`floor penetration ${maxFloorPenetrationM.toFixed(4)}m`);
        if (!finite) failures.push("nonfinite/error state");
        if (maxSpeed(afterRelease) <= 0.001 || afterRelease.diagnostics.authority !== "ragdoll") failures.push("release lost physical momentum or reset body");
        record(`${pointerType}-dom-cycle-${cycle + 1}`, failures, {
          region, stepCount: stepSnapshot.diagnostics.stepCount, maxPelvisHeightM, maxRootDisplacementM,
          maxJointSeparationM, maxFloorPenetrationM, finite, visibleSamples, totalSamples, visibilityMisses,
          dragDurationMs: performance.now() - dragStarted, beforeReleaseMaxSpeed: maxSpeed(beforeRelease),
          afterReleaseMaxSpeed: maxSpeed(afterRelease), releaseForceN: afterRelease.diagnostics.appliedGrabForceN,
          captures: { step: stepSnapshot, handoff: handoffSnapshot, fallenDrag: beforeRelease, release: afterRelease },
        });
      }
    }
  } catch (error) {
    record("replay-exception", [error instanceof Error ? error.message : String(error)], {});
  } finally {
    dispatch("pointercancel");
    await resetUI();
  }
  return {
    schema: 1,
    kind: "synthetic PointerEvent replay through integrated browser listeners",
    synthetic: true,
    trusted: false,
    pointerType,
    capturedAt: new Date().toISOString(),
    elapsedMs: performance.now() - started,
    passed: results.length > 0 && results.every((result) => result.passed),
    results,
  };
}
