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
  signal?: AbortSignal;
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

// Browser acceptance limit is simulated time; wall time only detects a stopped clock.
export const BROWSER_RECOVERY_LIMIT_SECONDS = 25;

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
  const signal = options.signal;
  const checkAborted = () => signal?.throwIfAborted();
  checkAborted();
  const wait = async (ms: number): Promise<void> => {
    checkAborted();
    await new Promise<void>((resolve, reject) => {
      const onAbort = () => {
        window.clearTimeout(timeout);
        signal?.removeEventListener("abort", onAbort);
        reject(signal?.reason);
      };
      const timeout = window.setTimeout(() => {
        signal?.removeEventListener("abort", onAbort);
        resolve();
      }, ms);
      signal?.addEventListener("abort", onAbort, { once: true });
    });
    checkAborted();
  };
  const readSnapshot = () => { checkAborted(); return adapter.snapshot(); };
  const readDiagnostics = () => { checkAborted(); return adapter.diagnostics(); };
  const readProjection = () => { checkAborted(); return adapter.projection(); };
  const readInteractionStatus = () => { checkAborted(); return adapter.interaction.getStatus(); };
  const reportProgress = (stage: string, record?: ReplayRecord) => {
    checkAborted();
    options.onProgress?.(stage, record);
    checkAborted();
  };
  const started = performance.now();
  const pointerType = options.pointerType ?? "mouse";
  const suite = options.suite ?? "all";
  const results: ReplayRecord[] = [];
  let pointerId = 4000;
  let clientPoint = { x: 0, y: 0 };
  const uiButton = (id: string) => {
    checkAborted();
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
    checkAborted();
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
    const point = readProjection().project(world);
    const rect = adapter.host.getBoundingClientRect();
    return { x: rect.left + point.x, y: rect.top + point.y };
  };
  const regionPoint = (region: RegionId) => {
    const pose = readSnapshot().segments.find((segment) => segment.id === region);
    if (!pose) throw new Error(`Replay segment missing: ${region}`);
    return screenPoint(pose.position);
  };
  const begin = async (region: RegionId) => {
    pointerId += 1;
    dispatch("pointerdown", regionPoint(region));
    await wait(65);
    return readInteractionStatus();
  };
  const releasedFailures = (): string[] => {
    const diagnostics = readDiagnostics();
    const failures: string[] = [];
    if (adapter.interaction.getActivePointerId() !== null || diagnostics.activeGrab) failures.push("grab remained active");
    if (diagnostics.appliedGrabForceN >= 0.01) failures.push(`released force ${diagnostics.appliedGrabForceN}N`);
    if (!diagnostics.finite || diagnostics.errors.length) failures.push("nonfinite state or application diagnostic error");
    return failures;
  };
  const record = (name: string, failures: string[], metrics: Record<string, unknown>) => {
    checkAborted();
    const result = { name, passed: failures.length === 0, failures, metrics };
    results.push(result);
    reportProgress(name, result);
  };
  const visibleSelectableRegions = (): RegionId[] => {
    const snapshot = readSnapshot();
    const projection = readProjection();
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
    const snapshot = readSnapshot();
    const projection = readProjection();
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
        reportProgress(`selecting ${region}`);
        await resetUI();
        const status = await begin(region);
        const failures = status.selectedRegion === region ? [] : [`visible ${region} center selected ${status.selectedRegion}`];
        const diagnostics = readDiagnostics();
        if (diagnostics.selectedRegion !== region) failures.push(`simulation selected ${diagnostics.selectedRegion}`);
        dispatch("pointerup");
        await wait(70);
        failures.push(...releasedFailures());
        record(`center-select-${region}`, failures, { selectedRegion: status.selectedRegion, localAnchor: status.localAnchor, coordinate: clientPoint });
      }
    }
    if (suite === "all" || suite === "lifecycle") {
      for (const action of ["release", "cancel", "lost-capture", "focus-loss", "pause", "reset"] as const) {
        reportProgress(`interrupt ${action}`);
        await resetUI();
        const initial = await begin("rightHand");
        dispatch("pointermove", { x: clientPoint.x + 22, y: clientPoint.y - 3 });
        await wait(65);
        const before = readDiagnostics();
        if (action === "release") dispatch("pointerup");
        if (action === "cancel") dispatch("pointercancel");
        if (action === "lost-capture") dispatch("lostpointercapture");
        if (action === "focus-loss") window.dispatchEvent(new Event("blur"));
        if (action === "pause") uiButton("pause-toggle").click();
        if (action === "reset") uiButton("reset-button").click();
        await wait(100);
        const after = readDiagnostics();
        const failures = releasedFailures();
        if (initial.selectedRegion !== "rightHand" || !before.activeGrab) failures.push("pre-interruption grab was not active");
        if (action === "reset" && (after.authority !== "character-motor" || after.stepCount !== 0)) failures.push("reset did not restore upright controller");
        record(`interrupt-${action}`, failures, { before, after, inputStatus: readInteractionStatus(), eventSource: "synthetic DOM event / integrated UI click" });
      }
      await resetUI();
      await begin("rightHand");
      const before = readInteractionStatus();
      const cameraBefore = readProjection().getState();
      const primaryPoint = { ...clientPoint };
      dispatch("pointerdown", regionPoint("leftHand"), pointerId + 100, false);
      dispatch("pointermove", { x: 20, y: 20 }, pointerId + 100, false);
      dispatch("pointerup", { x: 20, y: 20 }, pointerId + 100, false);
      await wait(70);
      const after = readInteractionStatus();
      const cameraAfter = readProjection().getState();
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
      await resetUI();
      for (let cycle = 0; cycle < cycles; cycle += 1) {
        const region: RegionId = cycle % 2 === 0 ? "rightHand" : "leftHand";
        const sign = region === "rightHand" ? 1 : -1;
        reportProgress(`cycle ${cycle + 1}/${cycles}: corrective step`);
        const grabbed = await begin(region);
        const failures: string[] = [];
        if (grabbed.selectedRegion !== region || !grabbed.worldTarget) throw new Error(`cycle ${cycle + 1}: ${region} center not selected`);
        const startWorld = grabbed.worldTarget;
        const initialRotation = readSnapshot().rootRotation;
        const heading = Math.atan2(
          2 * (initialRotation.x * initialRotation.z + initialRotation.w * initialRotation.y),
          1 - 2 * (initialRotation.x ** 2 + initialRotation.y ** 2),
        );
        const targetPoint = (offset: Vec3) => screenPoint(V3.add(startWorld, {
          x: offset.x * Math.cos(heading) + offset.z * Math.sin(heading),
          y: offset.y,
          z: offset.z * Math.cos(heading) - offset.x * Math.sin(heading),
        }));
        const initialStepCount = readDiagnostics().stepCount;
        let maxJointSeparationM = 0;
        let maxFloorPenetrationM = 0;
        let finite = true;
        let lockoutSamples = 0;
        const captures: Record<string, PoseSnapshot> = {};
        const lockoutFailures = new Set<string>();
        const observe = () => {
          const snapshot = readSnapshot();
          const diagnostics = snapshot.diagnostics;
          maxJointSeparationM = Math.max(maxJointSeparationM, diagnostics.maxJointSeparationM);
          maxFloorPenetrationM = Math.max(maxFloorPenetrationM, diagnostics.maxFloorPenetrationM);
          finite &&= diagnostics.finite && diagnostics.errors.length === 0;
          captures[snapshot.state] ??= snapshot;
          if (!diagnostics.bodyInputAvailable) {
            lockoutSamples += 1;
            if (diagnostics.activeGrab || diagnostics.activePointerId !== null) lockoutFailures.add("simulation retained a locked grab");
            if (diagnostics.appliedGrabForceN !== 0) lockoutFailures.add("external grab force was not exactly zero during lockout");
            const input = readInteractionStatus();
            if (input.activePointerId !== null || input.hasQueuedCommand) lockoutFailures.add("runtime retained a pointer or queued input during lockout");
            if (adapter.host.hasPointerCapture(pointerId)) lockoutFailures.add("fall entry retained pointer capture");
          }
          return snapshot;
        };
        for (let tick = 1; tick <= 90; tick += 1) {
          dispatch("pointermove", targetPoint({ x: sign * 0.60 * tick / 90, y: 0, z: 0.08 * tick / 90 }));
          await wait(17);
          observe();
        }
        await wait(350);
        const stepSnapshot = observe();
        if (stepSnapshot.diagnostics.stepCount <= initialStepCount) failures.push("no corrective step");
        reportProgress(`cycle ${cycle + 1}/${cycles}: protective fall`);
        for (let tick = 1; tick <= 5; tick += 1) {
          const t = tick / 5;
          dispatch("pointermove", targetPoint({ x: sign * (0.60 + 0.65 * t), y: 0.15 * t, z: 0.08 + 0.12 * t }));
          await wait(17);
          observe();
        }
        // Keep the overpowering target held, including while a corrective step
        // completes. Fall entry must clear it before the next physics substep.
        const forceDeadline = readSnapshot().simulationTime + 2;
        const forceWallDeadline = performance.now() + 10000;
        while (readDiagnostics().bodyInputAvailable && readSnapshot().simulationTime < forceDeadline && performance.now() < forceWallDeadline) {
          await wait(33);
          observe();
        }
        const handoffSnapshot = observe();
        if (handoffSnapshot.diagnostics.authority !== "ragdoll" || handoffSnapshot.diagnostics.bodyInputAvailable) {
          failures.push("overpowering pull did not enter body lockout");
        }
        reportProgress(`cycle ${cycle + 1}/${cycles}: automatic recovery with rejected body input`);
        const fallTime = captures.falling?.simulationTime ?? handoffSnapshot.simulationTime;
        const wallDeadline = performance.now() + BROWSER_RECOVERY_LIMIT_SECONDS * 3000;
        let recovered = handoffSnapshot;
        // A separate press during lockout must never be deferred into a grab.
        const rejectedPointer = pointerId + 100;
        dispatch("pointerdown", regionPoint("torso"), rejectedPointer);
        while (!recovered.diagnostics.bodyInputAvailable
          && recovered.simulationTime - fallTime < BROWSER_RECOVERY_LIMIT_SECONDS
          && performance.now() < wallDeadline) {
          const torso = recovered.segments.find((segment) => segment.id === "torso")!;
          dispatch("pointermove", screenPoint(V3.add(torso.position, { x: sign * 0.3, y: 0.1, z: 0 })));
          await wait(33);
          recovered = observe();
        }
        // Both pointers remain held across recovery: moves alone cannot grab.
        dispatch("pointermove", regionPoint("torso"));
        dispatch("pointermove", regionPoint("torso"), rejectedPointer);
        await wait(35);
        const heldAfterRecovery = observe();
        if (readInteractionStatus().activePointerId !== null || heldAfterRecovery.diagnostics.activeGrab) failures.push("held pointer resumed a grab after recovery");
        dispatch("pointerup");
        dispatch("pointerup", clientPoint, rejectedPointer);
        failures.push(...lockoutFailures);
        if (!captures.falling || !captures.fallen || !captures.recovering) failures.push("protective fall, settled landing, or recovery phase was not observed");
        if (!recovered.diagnostics.bodyInputAvailable || recovered.diagnostics.authority !== "character-motor" || recovered.state !== "upright") {
          failures.push(`stable standing did not return within ${BROWSER_RECOVERY_LIMIT_SECONDS}s simulated time`);
        }
        if (maxJointSeparationM > 0.08) failures.push(`joint separation ${maxJointSeparationM.toFixed(4)}m`);
        if (maxFloorPenetrationM > 0.08) failures.push(`floor penetration ${maxFloorPenetrationM.toFixed(4)}m`);
        if (!finite) failures.push("nonfinite/error state");
        const selectable = visibleSelectableRegions();
        const freshRegion = selectable.includes(region) ? region : selectable[0];
        const visibilityMiss = freshRegion ? null : captureVisibilityMiss(region, 0, true);
        if (!freshRegion) failures.push("recovered body has no visible selectable region");
        else if (recovered.diagnostics.bodyInputAvailable) {
          const freshPress = await begin(freshRegion);
          if (freshPress.selectedRegion !== freshRegion) failures.push("fresh pointer press was rejected after recovery");
          dispatch("pointerup");
          await wait(35);
          failures.push(...releasedFailures());
        }
        record(`${pointerType}-dom-cycle-${cycle + 1}`, failures, {
          region, stepCount: stepSnapshot.diagnostics.stepCount - initialStepCount,
          maxJointSeparationM, maxFloorPenetrationM, finite, lockoutSamples,
          recoveryTimeSeconds: recovered.simulationTime - fallTime,
          recoveryLimitSeconds: BROWSER_RECOVERY_LIMIT_SECONDS,
          recoveredMaxSpeedMps: maxSpeed(recovered), visibilityMiss,
          captures: { ...captures, step: stepSnapshot, handoff: handoffSnapshot, recovered, heldAfterRecovery },
        });
        if (!recovered.diagnostics.bodyInputAvailable) break;
      }
    }
  } catch (error) {
    checkAborted();
    record("replay-exception", [error instanceof Error ? error.message : String(error)], {});
  } finally {
    // A disposed runtime owns its cleanup. Aborted replay must not dispatch into it.
    if (!signal?.aborted) {
      dispatch("pointercancel");
      await resetUI();
    }
  }
  checkAborted();
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
