import { createRequire } from "node:module";
import { mkdir, writeFile, readFile, copyFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import { dirname, resolve } from "node:path";

const runtimeNode = process.env.CODEX_MCP_NODE_PATH;
if (!runtimeNode) throw new Error("CODEX_MCP_NODE_PATH must identify the bundled Node runtime.");
const requireRuntime = createRequire(resolve(dirname(runtimeNode), "package.json"));
const { chromium } = requireRuntime("playwright");
const mode = process.argv[2] ?? "quick";
const outputDirectory = resolve("evidence/visual-20260906", process.argv[3] ?? mode);
await mkdir(outputDirectory, { recursive: true });
// Keep active ffmpeg files outside Vite's watched project tree on Windows.
const videoDirectory = resolve(process.env.TEMP ?? process.env.TMP ?? outputDirectory, "codex-visual-replay", process.argv[3] ?? mode);
await mkdir(videoDirectory, { recursive: true });
const browser = await chromium.launch({ channel: "msedge", headless: true, args: ["--enable-unsafe-swiftshader"] });
const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1, recordVideo: { dir: videoDirectory, size: { width: 1440, height: 1000 } } });
const replayVideo = page.video();
const errors = [];
const evidence = [];
const samples = [];
const sourceFiles = ["src/character/EmbodiedCharacter.ts", "src/character/BalanceController.ts", "src/character/DynamicRecovery.ts", "src/character/pose.ts", "src/core/types.ts", "src/demo/DemoRuntime.ts", "src/interaction/PointerInteraction.ts"];
const fingerprint = async () => Object.fromEntries(await Promise.all(sourceFiles.map(async file => [file, createHash("sha256").update(await readFile(file)).digest("hex")])));
const sourceBefore = await fingerprint();
await writeFile(resolve(outputDirectory, "source-before.json"), JSON.stringify(sourceBefore, null, 2));
async function attemptedPointer(type, point) {
  await page.evaluate(({ type, point }) => document.querySelector('[data-testid="simulation-view"]').dispatchEvent(new PointerEvent(type, {
    bubbles: true, cancelable: true, pointerId: 901, pointerType: "touch", isPrimary: true,
    clientX: point.x, clientY: point.y, button: type === "pointermove" ? -1 : 0,
    buttons: type === "pointerup" ? 0 : 1,
  })), { type, point });
}
const readEvidence = () => page.evaluate(() => ({
  diagnostics: window.__EMBODIED_DEMO__.diagnostics(),
  input: JSON.parse(document.querySelector('[data-testid="qa-live"]').textContent || "null")?.interaction,
}));
async function capture(renderer, label) {
  const data = await readEvidence();
  if (data.diagnostics.renderer !== renderer) throw new Error(`Source reload changed renderer: expected ${renderer}, observed ${data.diagnostics.renderer}`);
  const path = resolve(outputDirectory, `${renderer}-${label}.png`);
  await page.screenshot({ path });
  evidence.push({ renderer, label, path, ...data });
  console.log(JSON.stringify({ stage: `${renderer}-${label}`, state: data.diagnostics.state, phase: data.diagnostics.recovery.phase, steps: data.diagnostics.stepCount, externalForceN: data.diagnostics.appliedGrabForceN }));
}
async function beginPull() {
  const point = await page.evaluate(() => window.__EMBODIED_DEMO__.regionPoint("rightHand"));
  await page.mouse.move(point.x, point.y);
  await page.mouse.down();
  await page.waitForTimeout(150);
  const input = (await readEvidence()).input;
  if (input?.selectedRegion !== "rightHand" || !input.worldTarget) throw new Error(`Right hand was not selected: ${JSON.stringify(input)}`);
  return input.worldTarget;
}
async function targetPoints(start, offsets) {
  return page.evaluate(async ({ start, offsets }) => {
    const { SharedCameraProjection } = await import("/src/scene/camera.ts");
    const camera = new SharedCameraProjection(window.__EMBODIED_DEMO__.camera());
    const rect = document.querySelector('[data-testid="simulation-view"]').getBoundingClientRect();
    return offsets.map(offset => {
      const point = camera.project({ x: start.x + offset.x, y: start.y + offset.y, z: start.z + offset.z });
      return { x: rect.left + point.x, y: rect.top + point.y };
    });
  }, { start, offsets });
}
page.on("pageerror", error => errors.push(error.message));
page.on("console", message => { if (message.type() === "error") errors.push(message.text()); });
try {
  await page.goto("http://127.0.0.1:5173/?qa=1", { waitUntil: "domcontentloaded", timeout: 90000 });
  await page.waitForFunction(() => window.__EMBODIED_DEMO__?.ready(), null, { timeout: 90000 });
  console.log(JSON.stringify({ stage: "ready", mode, capabilities: await page.evaluate(() => window.__EMBODIED_DEMO__.capabilities()) }));
  // Test-only observer: delegates each reset/update exactly once, with unchanged inputs.
  await page.evaluate(async () => {
    const details = document.querySelector("details");
    const fiberKey = Object.keys(details).find(key => key.startsWith("__reactFiber$"));
    let fiber = details[fiberKey];
    while (fiber && !fiber.memoizedProps?.runtime?.character) fiber = fiber.return;
    if (!fiber) throw new Error("Could not find the test-only BrowserVerification runtime prop.");
    const runtime = fiber.memoizedProps.runtime;
    const character = runtime.character;
    const { SEGMENTS } = await import("/src/core/humanoid.ts");
    const totalMass = SEGMENTS.reduce((sum, segment) => sum + segment.massKg, 0);
    const mass = Object.fromEntries(SEGMENTS.map(segment => [segment.id, segment.massKg]));
    const trace = window.__VISUAL_FIXED_TRACE__ = { schema: 1, initial: character.getSnapshot(runtime.renderer), resets: [], updates: [], transfers: [], phaseSnapshots: [] };
    let resetId = 0;
    const reset = character.reset.bind(character);
    character.reset = () => {
      reset();
      resetId += 1;
      trace.resets.push({ resetId, snapshot: character.getSnapshot(runtime.renderer) });
    };
    const update = character.fixedUpdate.bind(character);
    character.fixedUpdate = (dt, command) => {
      const before = character.getSnapshot(runtime.renderer);
      update(dt, command);
      const after = character.getSnapshot(runtime.renderer);
      const recovery = after.diagnostics.recovery;
      const torso = after.segments.find(segment => segment.id === "torso");
      const pelvis = after.segments.find(segment => segment.id === "pelvis");
      const upDot = rotation => 1 - 2 * (rotation.x ** 2 + rotation.z ** 2);
      const squareLength = vector => vector.x ** 2 + vector.y ** 2 + vector.z ** 2;
      const rms = field => Math.sqrt(after.segments.reduce((sum, segment) => sum + mass[segment.id] * squareLength(segment[field]), 0) / totalMass);
      trace.updates.push({ resetId, dt, command: command ? structuredClone(command) : null, sequence: after.sequence, state: after.state,
        phase: recovery.phase, recovery,
        authority: after.diagnostics.authority, bodyInputAvailable: after.diagnostics.bodyInputAvailable,
        activeGrab: after.diagnostics.activeGrab, externalGrabForceN: after.diagnostics.appliedGrabForceN,
        maxJointSeparationM: after.diagnostics.maxJointSeparationM, maxFloorPenetrationM: after.diagnostics.maxFloorPenetrationM,
        finite: after.diagnostics.finite, pelvisHeightM: pelvis.position.y, torsoUp: upDot(torso.rotation), pelvisUp: upDot(pelvis.rotation),
        rmsLinearMps: rms("linearVelocity"), rmsAngularRadps: rms("angularVelocity"), bestError: character.recovery.bestError,
        noSupportTimeS: character.recovery.noSupportTime, rise: character.recovery.rise, targetOrigin: { ...character.recovery.targetOrigin },
      });
      if (before.diagnostics.authority !== after.diagnostics.authority) trace.transfers.push({ resetId, before, after });
      if (before.state !== after.state || before.diagnostics.recovery.phase !== recovery.phase) trace.phaseSnapshots.push({ resetId, snapshot: after });
    };
  });
  for (const renderer of (mode === "trace" ? ["webgl"] : ["webgl", "canvas2d"])) {
    await page.getByTestId("renderer-picker").click();
    await page.getByRole("option", { name: renderer === "webgl" ? "WebGL2" : "Canvas 2D", exact: true }).click();
    await page.getByTestId("reset-button").click();
    await page.waitForTimeout(100);
    await page.locator("summary").filter({ hasText: "Browser verification" }).click();
    await capture(renderer, "standing");
    const slowStart = await beginPull();
    const slowPoints = await targetPoints(slowStart, Array.from({ length: 90 }, (_, index) => ({ x: .60 * (index + 1) / 90, y: 0, z: .08 * (index + 1) / 90 })));
    for (let index = 0; index < slowPoints.length; index += 1) {
      await page.mouse.move(slowPoints[index].x, slowPoints[index].y);
      await page.waitForTimeout(17);
      if (index === 55) await capture(renderer, "slow-corrective-step");
    }
    await page.mouse.up();
    await page.waitForTimeout(500);
    await capture(renderer, "slow-release");
    await page.getByTestId("reset-button").click();
    for (let cycle = 1; cycle <= (mode === "full" ? 2 : 1); cycle += 1) {
    const prefix = `cycle-${cycle}`;
    const fastStart = await beginPull();
    const fastPoints = await targetPoints(fastStart, Array.from({ length: 5 }, (_, index) => ({ x: 1.25 * (index + 1) / 5, y: .15 * (index + 1) / 5, z: .2 * (index + 1) / 5 })));
    for (const point of fastPoints) {
      await page.mouse.move(point.x, point.y);
      await page.waitForTimeout(17);
    }
    await page.waitForTimeout(200);
    await capture(renderer, `${prefix}-overpower-protective`);
    const attemptedPoint = await page.evaluate(() => window.__EMBODIED_DEMO__.regionPoint("torso"));
    await attemptedPointer("pointerdown", attemptedPoint);
    let previousPhase = "";
    const deadline = Date.now() + (mode !== "quick" ? 90000 : 5000);
    const initialDiagnostics = (await readEvidence()).diagnostics;
    const initialFixedSteps = initialDiagnostics.handoff?.sequence ?? initialDiagnostics.fixedSteps;
    let latest;
    while (Date.now() < deadline) {
      latest = await readEvidence();
      samples.push({ renderer, cycle, ...latest });
      const phase = `${latest.diagnostics.state}-${latest.diagnostics.recovery.phase}`;
      if (phase !== previousPhase) {
        await capture(renderer, `${prefix}-phase-${phase}`);
        previousPhase = phase;
      }
      if (latest.diagnostics.renderer !== renderer) throw new Error("Source changed during visual acceptance run.");
      if (!latest.diagnostics.bodyInputAvailable && latest.diagnostics.appliedGrabForceN !== 0) throw new Error("External force leaked during lockout.");
      if (latest.diagnostics.bodyInputAvailable && latest.diagnostics.state === "upright") break;
      if (latest.diagnostics.fixedSteps - initialFixedSteps >= 25 * 60) break;
      await attemptedPointer("pointermove", { x: attemptedPoint.x + .1, y: attemptedPoint.y });
      await page.waitForTimeout(100);
    }
    if (mode !== "quick" && !latest?.diagnostics.bodyInputAvailable) {
      errors.push(`${renderer}: stable standing did not return within the fixed 25-second simulated recovery limit`);
    }
    const heldPoint = await page.evaluate(() => window.__EMBODIED_DEMO__.regionPoint("torso"));
    await page.mouse.move(heldPoint.x, heldPoint.y);
    await page.waitForTimeout(100);
    if ((await readEvidence()).diagnostics.activeGrab) throw new Error("Held pointer resumed body input without a new press.");
    await attemptedPointer("pointermove", attemptedPoint);
    await page.mouse.up();
    await attemptedPointer("pointerup", attemptedPoint);
    await capture(renderer, `${prefix}-${mode === "full" ? "recovery-final" : "preliminary-final"}`);
    if (latest?.diagnostics.bodyInputAvailable) {
      await beginPull();
      if (!(await readEvidence()).diagnostics.activeGrab) throw new Error("Fresh press was not accepted after recovery.");
      await capture(renderer, `${prefix}-fresh-press-after-recovery`);
      await page.mouse.up();
      await page.waitForTimeout(150);
    } else break;
    }
    await page.locator("summary").filter({ hasText: "Browser verification" }).click();
  }
  const nativeTrace = await page.evaluate(() => window.__VISUAL_FIXED_TRACE__);
  await writeFile(resolve(outputDirectory, "native-fixed-trace.json"), JSON.stringify(nativeTrace, null, 2));
  const updates = nativeTrace.updates;
  const perStepSummary = {
    updates: updates.length,
    maxJointSeparationM: Math.max(...updates.map(update => update.maxJointSeparationM)),
    maxFloorPenetrationM: Math.max(...updates.map(update => update.maxFloorPenetrationM)),
    allFinite: updates.every(update => update.finite),
    lockoutSteps: updates.filter(update => !update.bodyInputAvailable).length,
    lockoutExternalForceExactlyZero: updates.every(update => update.bodyInputAvailable || update.externalGrabForceN === 0),
    lockoutGrabInactive: updates.every(update => update.bodyInputAvailable || !update.activeGrab),
    recoveryTimesSeconds: [],
  };
  let lastFall = null;
  for (const transfer of nativeTrace.transfers) {
    if (transfer.after.diagnostics.authority === "ragdoll") lastFall = transfer.after;
    else if (lastFall) {
      perStepSummary.recoveryTimesSeconds.push(transfer.after.simulationTime - lastFall.simulationTime);
      lastFall = null;
    }
  }
  if (perStepSummary.maxJointSeparationM > .08) errors.push(`Native joint separation ${perStepSummary.maxJointSeparationM}m exceeded .08m`);
  if (perStepSummary.maxFloorPenetrationM > .08) errors.push(`Native floor penetration ${perStepSummary.maxFloorPenetrationM}m exceeded .08m`);
  if (!perStepSummary.allFinite) errors.push("Native trace contained a nonfinite pose or velocity.");
  if (!perStepSummary.lockoutExternalForceExactlyZero || !perStepSummary.lockoutGrabInactive) errors.push("Native per-step input lockout failed.");
  if (mode === "full" && (perStepSummary.recoveryTimesSeconds.length !== 4 || perStepSummary.recoveryTimesSeconds.some(time => time > 25))) errors.push("Four native recoveries within25simseconds were not completed.");
  await writeFile(resolve(outputDirectory, "per-step-summary.json"), JSON.stringify(perStepSummary, null, 2));
  console.log(JSON.stringify({ stage: "per-step-validation", ...perStepSummary, errors }));
  const sourceAfter = await fingerprint();
  await writeFile(resolve(outputDirectory, "source-after.json"), JSON.stringify(sourceAfter, null, 2));
  if (JSON.stringify(sourceBefore) !== JSON.stringify(sourceAfter)) throw new Error("Physics or input source changed during visual acceptance.");
  await writeFile(resolve(outputDirectory, `${mode}-samples.json`), JSON.stringify(samples, null, 2));
  await writeFile(resolve(outputDirectory, `${mode}-browser-errors.json`), JSON.stringify(errors, null, 2));
  await writeFile(resolve(outputDirectory, `${mode}-visual-evidence.json`), JSON.stringify(evidence, null, 2));
  if (errors.length) process.exitCode = 1;
} catch (error) {
  const trace = await page.evaluate(() => window.__VISUAL_FIXED_TRACE__ ?? null).catch(() => null);
  await writeFile(resolve(outputDirectory, "native-fixed-trace.json"), JSON.stringify(trace, null, 2));
  await page.screenshot({ path: resolve(outputDirectory, `${mode}-error.png`) }).catch(() => undefined);
  await writeFile(resolve(outputDirectory, `${mode}-error.json`), JSON.stringify({ error: String(error), browserErrors: errors, evidence }, null, 2));
  throw error;
} finally {
  await page.close().catch(() => undefined);
  const videoPath = resolve(videoDirectory, `${mode}-both-renderers.webm`);
  await replayVideo?.saveAs(videoPath).catch(() => undefined);
  await copyFile(videoPath, resolve(outputDirectory, `${mode}-both-renderers.webm`)).catch(() => undefined);
  await browser.close();
}
