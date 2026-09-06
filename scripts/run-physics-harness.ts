import { mkdirSync, writeFileSync } from "node:fs";
import { performance } from "node:perf_hooks";
import RAPIER from "@dimforge/rapier3d-compat";
import { createEmbodiedCharacter as createCharacter } from "../src/character/index";
import { SharedCameraProjection } from "../src/scene/camera";
import { rotate } from "../src/character/math";
import { REGION_IDS, type CharacterController, type GrabCommand, type PoseSnapshot, type RegionId, type Vec3 } from "../src/core/types";

const DT = 1 / 60;
const THRESHOLDS = Object.freeze({
  idleSeconds: 30,
  idleRootDriftM: 0.08,
  maxJointSeparationM: 0.08,
  maxFloorPenetrationM: 0.08,
  handoffJointSeparationM: 0.05,
  stepMinimum: 1,
  releasedForceN: 0.01,
  readableFallRootDisplacementM: 3,
  readableFallPelvisHeightM: 2.4,
  repeatedCycles: 10,
  handoffTranslationM: 0.025,
  handoffAngleDegrees: 3,
  handoffAnchorM: 0.03,
});

// Every fixed update is sampled, including the handoff and release frame.
let activeScenario = "";
const traceRows: unknown[] = [];
const controllers = new Map<CharacterController, {maxHeight: number; maxDisplacement: number; maxJoint: number; maxPenetration: number; errors: string[]}>();
async function createEmbodiedCharacter(renderer: "canvas2d") {
  const controller = await createCharacter(renderer);
  const record = {maxHeight: 0, maxDisplacement: 0, maxJoint: 0, maxPenetration: 0, errors: [] as string[]};
  const original = controller.fixedUpdate.bind(controller);
  controller.fixedUpdate = (dt, command) => {
    original(dt, command);
    const snapshot = controller.getSnapshot(renderer), d = snapshot.diagnostics;
    record.maxHeight = Math.max(record.maxHeight, snapshot.rootPosition.y);
    record.maxDisplacement = Math.max(record.maxDisplacement, d.rootDisplacementM);
    record.maxJoint = Math.max(record.maxJoint, d.maxJointSeparationM);
    record.maxPenetration = Math.max(record.maxPenetration, d.maxFloorPenetrationM);
    if (!d.finite || d.errors.length) record.errors.push(...d.errors, ...(!d.finite ? ["nonfinite"] : []));
    if (d.authority === "ragdoll") traceRows.push({scenario: activeScenario, sequence: snapshot.sequence, time: snapshot.simulationTime, root: snapshot.rootPosition, diagnostics: d});
  };
  controllers.set(controller, record);
  return controller;
}

interface ScenarioResult {
  name: string;
  passed: boolean;
  durationMs: number;
  metrics: Record<string, unknown>;
  failures: string[];
}

const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const sub = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const scale = (v: Vec3, amount: number): Vec3 => ({ x: v.x * amount, y: v.y * amount, z: v.z * amount });
const magnitude = (v: Vec3): number => Math.hypot(v.x, v.y, v.z);
const normalize = (v: Vec3): Vec3 => {
  const size = magnitude(v) || 1;
  return scale(v, 1 / size);
};

function pose(snapshot: PoseSnapshot, region: RegionId) {
  const value = snapshot.segments.find((segment) => segment.id === region);
  if (!value) throw new Error(`Missing pose for ${region}`);
  return value;
}

function step(controller: CharacterController, count: number, command: GrabCommand | null = null): void {
  for (let index = 0; index < count; index += 1) controller.fixedUpdate(DT, index === 0 ? command : null);
}

async function scenario(name: string, run: () => Promise<Record<string, unknown>>): Promise<ScenarioResult> {
  activeScenario = name;
  controllers.clear();
  const started = performance.now();
  try {
    const metrics = await run();
    const failures = (metrics.failures as string[] | undefined) ?? [];
    delete metrics.failures;
    const trajectory = [...controllers.values()];
    for (const record of trajectory) {
      if (record.maxJoint > THRESHOLDS.maxJointSeparationM) failures.push(`trajectory joint separation ${record.maxJoint}`);
      if (record.maxPenetration > THRESHOLDS.maxFloorPenetrationM) failures.push(`trajectory floor penetration ${record.maxPenetration}`);
      if (record.errors.length) failures.push(`trajectory errors ${record.errors.join(",")}`);
    }
    metrics.trajectory = trajectory;
    return { name, passed: failures.length === 0, durationMs: performance.now() - started, metrics, failures };
  } catch (error) {
    return {
      name,
      passed: false,
      durationMs: performance.now() - started,
      metrics: {},
      failures: [error instanceof Error ? error.message : String(error)],
    };
  }
}

function begin(controller: CharacterController, region: RegionId, pointerId = 1, offCenter = false): { start: Vec3; command: GrabCommand } {
  const snapshot = controller.getSnapshot("canvas2d");
  const segment = pose(snapshot, region);
  const localAnchor = offCenter ? { x: 0.045, y: 0.025, z: 0.02 } : { x: 0, y: 0, z: 0 };
  const start = add(segment.position, rotate(segment.rotation, localAnchor));
  return {
    start,
    command: { kind: "begin", pointerId, region, segment: region, localAnchor, worldTarget: start, timestampMs: 0 },
  };
}

function moveOver(controller: CharacterController, pointerId: number, start: Vec3, delta: Vec3, frames: number, startMs = 0): void {
  for (let index = 1; index <= frames; index += 1) {
    const amount = index / frames;
    controller.fixedUpdate(DT, {
      kind: "move",
      pointerId,
      worldTarget: add(start, scale(delta, amount)),
      timestampMs: startMs + index * DT * 1000,
    });
  }
}

const results: ScenarioResult[] = [];

results.push(await scenario("idle-30-seconds", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const initial = controller.getSnapshot("canvas2d");
  step(controller, THRESHOLDS.idleSeconds * 60);
  const final = controller.getSnapshot("canvas2d");
  const drift = magnitude(sub(final.rootPosition, initial.rootPosition));
  const d = final.diagnostics;
  const failures: string[] = [];
  if (final.state !== "upright") failures.push(`state=${final.state}`);
  if (!d.support.grounded) failures.push("support not grounded");
  if (drift > THRESHOLDS.idleRootDriftM) failures.push(`root drift ${drift.toFixed(4)}m`);
  if (d.maxJointSeparationM > THRESHOLDS.maxJointSeparationM) failures.push(`joint separation ${d.maxJointSeparationM.toFixed(4)}m`);
  if (d.maxFloorPenetrationM > THRESHOLDS.maxFloorPenetrationM) failures.push(`floor penetration ${d.maxFloorPenetrationM.toFixed(4)}m`);
  if (!d.finite || d.errors.length) failures.push(`errors=${d.errors.join(",") || "nonfinite"}`);
  return { failures, state: final.state, driftM: drift, grounded: d.support.grounded, maxJointSeparationM: d.maxJointSeparationM, maxFloorPenetrationM: d.maxFloorPenetrationM, fixedSteps: d.fixedSteps };
}));

results.push(await scenario("seven-region-picking", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const snapshot = controller.getSnapshot("canvas2d");
  const camera = new SharedCameraProjection().getState().position;
  const picked: Record<string, string | null> = {};
  for (const region of REGION_IDS) {
    const center = pose(snapshot, region).position;
    const direction = normalize(sub(center, camera));
    const origin = camera;
    picked[region] = controller.pick({ origin, direction })?.region ?? null;
  }
  const failures = REGION_IDS.filter((region) => picked[region] !== region).map((region) => `${region} picked ${picked[region]}`);
  return { failures, picked, count: REGION_IDS.length - failures.length };
}));

results.push(await scenario("small-pull-recovers", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "rightHand", 2, true);
  controller.fixedUpdate(DT, grab.command);
  moveOver(controller, 2, grab.start, { x: 0.2, y: 0.03, z: 0.02 }, 30);
  controller.fixedUpdate(DT, { kind: "end", pointerId: 2, timestampMs: 520 });
  step(controller, 120);
  const snapshot = controller.getSnapshot("canvas2d");
  const failures: string[] = [];
  if (snapshot.state === "falling" || snapshot.state === "fallen") failures.push(`unexpected ${snapshot.state}`);
  if (!snapshot.diagnostics.finite || snapshot.diagnostics.errors.length) failures.push("invalid diagnostics");
  return { failures, state: snapshot.state, rootDisplacementM: snapshot.diagnostics.rootDisplacementM, offCenterAnchor: grab.command.localAnchor };
}));

results.push(await scenario("corrective-step", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "leftHand", 3);
  controller.fixedUpdate(DT, grab.command);
  moveOver(controller, 3, grab.start, { x: -0.46, y: 0, z: 0.08 }, 45);
  step(controller, 30);
  const snapshot = controller.getSnapshot("canvas2d");
  const failures: string[] = [];
  if (snapshot.diagnostics.stepCount < THRESHOLDS.stepMinimum) failures.push("no corrective step");
  if (snapshot.state === "falling" || snapshot.state === "fallen") failures.push(`stepping pull overloaded to ${snapshot.state}`);
  return { failures, state: snapshot.state, stepCount: snapshot.diagnostics.stepCount, rootDisplacementM: snapshot.diagnostics.rootDisplacementM, support: snapshot.support };
}));

results.push(await scenario("negligible-stationary-pull", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "torso", 4);
  controller.fixedUpdate(DT, grab.command);
  step(controller, 600);
  const snapshot = controller.getSnapshot("canvas2d");
  const failures = snapshot.state === "falling" || snapshot.state === "fallen" ? [`stationary pull caused ${snapshot.state}`] : [];
  return { failures, state: snapshot.state, activeGrab: snapshot.diagnostics.activeGrab, effortN: snapshot.diagnostics.appliedGrabForceN };
}));

results.push(await scenario("handoff-drag-release", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "rightHand", 5, true);
  controller.fixedUpdate(DT, grab.command);
  moveOver(controller, 5, grab.start, { x: 0.75, y: 0.08, z: -0.18 }, 75);
  let snapshot = controller.getSnapshot("canvas2d");
  const atTransfer = snapshot.diagnostics;
  step(controller, 35);
  snapshot = controller.getSnapshot("canvas2d");
  const beforeReleaseSpeed = magnitude(pose(snapshot, "pelvis").linearVelocity);
  controller.fixedUpdate(DT, { kind: "end", pointerId: 5, timestampMs: 1900 });
  const afterRelease = controller.getSnapshot("canvas2d");
  step(controller, 1);
  const next = controller.getSnapshot("canvas2d");
  const afterReleaseSpeed = magnitude(pose(next, "pelvis").linearVelocity);
  const failures: string[] = [];
  const handoff = snapshot.diagnostics.handoff;
  if (!handoff) failures.push("missing same-instant handoff measurements");
  else {
    if (handoff.maxTranslationErrorM > THRESHOLDS.handoffTranslationM) failures.push("handoff translation");
    if (handoff.maxAngularErrorDegrees > THRESHOLDS.handoffAngleDegrees) failures.push("handoff angle");
    if (handoff.selectedAnchorErrorM > THRESHOLDS.handoffAnchorM) failures.push("handoff anchor");
    if (handoff.jointSeparationM > THRESHOLDS.handoffJointSeparationM) failures.push("handoff joints");
    if (handoff.rawTargetErrorM > 1e-9 || handoff.localAnchorErrorM > 1e-9 || handoff.targetDerivativeSpeedMps > 1e-9) failures.push("handoff target/derivative continuity");
  }
  if (atTransfer.authority !== "ragdoll") failures.push(`authority=${atTransfer.authority}`);
  if (!atTransfer.activeGrab) failures.push("grab target not preserved at handoff");
  if (atTransfer.maxJointSeparationM > THRESHOLDS.handoffJointSeparationM) failures.push(`handoff separation ${atTransfer.maxJointSeparationM.toFixed(4)}m`);
  if (afterRelease.diagnostics.activeGrab || next.diagnostics.appliedGrabForceN > THRESHOLDS.releasedForceN) failures.push("grab contribution survived release");
  if (beforeReleaseSpeed > 0.05 && afterReleaseSpeed < 0.02) failures.push("momentum stopped on release");
  return { failures, handoff, transferState: atTransfer.state, authority: atTransfer.authority, grabPreserved: atTransfer.activeGrab, maxJointSeparationM: atTransfer.maxJointSeparationM, releasedForceN: next.diagnostics.appliedGrabForceN, beforeReleaseSpeed, afterReleaseSpeed };
}));

results.push(await scenario("fast-overload-readable-fall", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "rightHand", 6, true);
  // Preserve the original frozen replay's literal world target construction.
  // Other new scenarios correctly rotate a local anchor into world space.
  grab.start = add(pose(controller.getSnapshot("canvas2d"), "rightHand").position, grab.command.localAnchor!);
  grab.command.worldTarget = grab.start;
  controller.fixedUpdate(DT, grab.command);
  const delta = { x: 0.954, y: 0.485, z: -0.706 };
  moveOver(controller, 6, grab.start, delta, 5, 0);
  step(controller, 30);
  controller.fixedUpdate(DT, { kind: "end", pointerId: 6, timestampMs: 600 });
  step(controller, 120);
  const snapshot = controller.getSnapshot("canvas2d");
  const failures: string[] = [];
  const peaks = controllers.get(controller)!;
  const rootDisplacement = peaks.maxDisplacement;
  if (rootDisplacement > THRESHOLDS.readableFallRootDisplacementM) failures.push(`root displacement ${rootDisplacement.toFixed(3)}m exceeds readable bound`);
  if (peaks.maxHeight > THRESHOLDS.readableFallPelvisHeightM) failures.push(`pelvis height ${peaks.maxHeight.toFixed(3)}m indicates launch`);
  if (snapshot.diagnostics.maxFloorPenetrationM > THRESHOLDS.maxFloorPenetrationM) failures.push(`floor penetration ${snapshot.diagnostics.maxFloorPenetrationM.toFixed(3)}m`);
  if (!snapshot.diagnostics.finite || snapshot.diagnostics.errors.length) failures.push("nonfinite/error state");
  return { failures, state: snapshot.state, rootPosition: snapshot.rootPosition, rootDisplacementM: rootDisplacement, maxPelvisHeightM: peaks.maxHeight, handoff: snapshot.diagnostics.handoff, maxJointSeparationM: snapshot.diagnostics.maxJointSeparationM, maxFloorPenetrationM: snapshot.diagnostics.maxFloorPenetrationM };
}));

results.push(await scenario("ten-real-engine-cycles", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const cycles: Array<Record<string, unknown>> = [];
  const failures: string[] = [];
  for (let cycle = 0; cycle < THRESHOLDS.repeatedCycles; cycle += 1) {
    controller.reset();
    const region: RegionId = cycle % 2 === 0 ? "leftHand" : "rightHand";
    const sign = region === "leftHand" ? -1 : 1;
    const grab = begin(controller, region, 100 + cycle, true);
    controller.fixedUpdate(DT, grab.command);
    moveOver(controller, 100 + cycle, grab.start, { x: sign * 0.76, y: 0.04, z: cycle < 5 ? 0.16 : -0.16 }, 70);
    step(controller, 25);
    controller.fixedUpdate(DT, { kind: "end", pointerId: 100 + cycle, timestampMs: 1700 });
    step(controller, 50);
    const snapshot = controller.getSnapshot("canvas2d");
    const valid = snapshot.diagnostics.finite && !snapshot.diagnostics.errors.length && !snapshot.diagnostics.activeGrab && snapshot.diagnostics.appliedGrabForceN <= THRESHOLDS.releasedForceN;
    if (!valid) failures.push(`cycle ${cycle + 1} invalid`);
    cycles.push({ cycle: cycle + 1, inputProfile: cycle < 5 ? "desktop-equivalent-direct" : "touch-equivalent-direct", state: snapshot.state, stepCount: snapshot.diagnostics.stepCount, finite: snapshot.diagnostics.finite, errors: snapshot.diagnostics.errors, jointSeparationM: snapshot.diagnostics.maxJointSeparationM, rootDisplacementM: snapshot.diagnostics.rootDisplacementM });
  }
  return { failures, cycles };
}));

results.push(await scenario("both-hands-feet-gentle-directional", async () => {
  const responses = [];
  const failures: string[] = [];
  for (const region of ["leftHand", "rightHand", "leftFoot", "rightFoot"] as const) {
    const controller = await createEmbodiedCharacter("canvas2d");
    const grab = begin(controller, region, 210);
    controller.fixedUpdate(DT, grab.command);
    const sign = region.startsWith("left") ? -1 : 1;
    moveOver(controller, 210, grab.start, {x: sign * 0.16, y: region.endsWith("Foot") ? 0.10 : 0.03, z: 0.05}, 30);
    const current = controller.getSnapshot("canvas2d");
    const displacement = sub(pose(current, region).position, grab.start);
    if (current.diagnostics.authority !== "character-motor" || displacement.x * sign <= 0.015) failures.push(`${region} directional response failed`);
    responses.push({region, displacement, root: current.rootPosition, state: current.state});
  }
  return {failures, responses};
}));

results.push(await scenario("corrective-step-opposite-direction", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "rightHand", 220);
  controller.fixedUpdate(DT, grab.command);
  moveOver(controller, 220, grab.start, {x: 0.46, y: 0, z: -0.08}, 45);
  step(controller, 30);
  const d = controller.diagnostics();
  return {failures: d.stepCount >= 1 && d.authority === "character-motor" ? [] : ["opposite step failed"], stepCount: d.stepCount, state: d.state};
}));

results.push(await scenario("fallen-regrab-drag-three-seconds-reset", async () => {
  const controller = await createEmbodiedCharacter("canvas2d");
  const grab = begin(controller, "rightHand", 230, true);
  controller.fixedUpdate(DT, grab.command);
  moveOver(controller, 230, grab.start, {x: 0.954, y: 0.485, z: -0.706}, 5);
  step(controller, 30);
  controller.fixedUpdate(DT, {kind: "end", pointerId: 230, timestampMs: 600});
  step(controller, 120);
  const camera = new SharedCameraProjection();
  const projection = camera.getState();
  const candidates = ["rightHand", "torso", "pelvis"] as const;
  const visiblePick = () => candidates.find(region => {
    const p = pose(controller.getSnapshot("canvas2d"), region).position;
    const projected = camera.project(p);
    const hit = controller.pick({origin: projection.position, direction: normalize(sub(p, projection.position))});
    return projected.x >= 0 && projected.x < projection.viewportWidth && projected.y >= 0 && projected.y < projection.viewportHeight && projected.depth > 0 && hit?.region === region;
  });
  const region = visiblePick();
  if (!region) return {failures: ["no visible selectable fallen target"]};
  const regrab = begin(controller, region, 231);
  controller.fixedUpdate(DT, regrab.command);
  let visibleFrames = 0;
  for (let i=0; i<180; i++) {
    controller.fixedUpdate(DT, {kind: "move", pointerId: 231, worldTarget: add(regrab.start, {x: 0.3*i/180, y: 0.12*i/180, z: 0}), timestampMs: 3000+i*DT*1000});
    if (visiblePick()) visibleFrames++;
  }
  const before = controller.getSnapshot("canvas2d");
  controller.fixedUpdate(DT, {kind: "end", pointerId: 231, timestampMs: 6100});
  const released = controller.getSnapshot("canvas2d");
  const releaseSpeed = magnitude(pose(released, region).linearVelocity);
  const failures: string[] = [];
  if (visibleFrames !== 180) failures.push(`visible frames ${visibleFrames}/180`);
  if (released.diagnostics.activeGrab || released.diagnostics.appliedGrabForceN >= 0.01) failures.push("released contribution remains");
  if (releaseSpeed <= 0.001 || released.diagnostics.authority !== "ragdoll") failures.push("release momentum/reset failure");
  controller.reset();
  const reset = controller.diagnostics();
  if (reset.activeGrab || reset.authority !== "character-motor" || reset.stepCount !== 0) failures.push("explicit reset failed");
  return {failures, region, visibleFrames, requiredFrames:180, seconds:3, beforeReleaseState:before.state, releaseSpeed, releasedForceN:released.diagnostics.appliedGrabForceN, reset};
}));

const failed = results.filter((result) => !result.passed);
const output = {
  schema: 2,
  correction: "S1-effective-mass-anchor",
  generatedAt: new Date().toISOString(),
  engine: `Rapier ${RAPIER.version()}`,
  fixedHz: 60,
  thresholds: THRESHOLDS,
  summary: { passed: failed.length === 0, passedScenarios: results.length - failed.length, failedScenarios: failed.length },
  results,
};

mkdirSync("evidence", { recursive: true });
writeFileSync("evidence/successor-trace.ndjson", traceRows.map(row => JSON.stringify(row)).join("\n") + "\n", "utf8");
writeFileSync("evidence/physics-results.json", `${JSON.stringify(output, null, 2)}\n`, "utf8");
console.log(JSON.stringify(output, null, 2));
process.exitCode = failed.length ? 1 : 0;
