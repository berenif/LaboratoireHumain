import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import { register } from "tsx/esm/api";

// Full acceptance is deliberately expensive: diagnostics are read after every
// Rapier step so brief contact, penetration and joint-limit peaks are retained.
const dt = 1 / 60;
const maxPostImpactS = 25;
const stableFramesRequired = 60;
const stages = ["falling", "floor", "arm-support", "kneeling", "pre-stable"];
const sourcePaths = [
  "../src/character/BalanceController.ts",
  "../src/character/DynamicRecovery.ts",
  "../src/character/EmbodiedCharacter.ts",
  "../src/character/PhysicsStriker.ts",
  "../src/character/contact-loads.ts",
  "../src/character/joint-motors.ts",
  "../src/character/recovery-forefoot-anchor.ts",
  "../src/character/recovery-foot-targets.ts",
  "../src/character/recovery-joints.ts",
  "../src/character/recovery-support.ts",
  "../src/core/protocol.ts",
  "../src/core/humanoid.ts",
  "../src/core/geometry.ts",
];
const hashSources = () => Object.fromEntries(sourcePaths.map((path) => [path,
  createHash("sha256").update(readFileSync(new URL(path, import.meta.url))).digest("hex")]));
const startedAt = new Date().toISOString();
const startingHashes = hashSources();
const unregister = register();
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { recoveryStandingPosture } = await import("../src/character/DynamicRecovery.ts");
const { isRecoveryFootSegment, recoveryMassState, supportGeometry } = await import("../src/character/recovery-support.ts");
const { SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const { ACCEPTANCE } = await import("./physics-fixtures.ts");
const character = await createEmbodiedCharacter("canvas2d", { room: true });
const bodies = new Map(character.ragdollBodies);
const reset = character.reset.bind(character);
let resetCalls = 0;
character.reset = (...args) => { resetCalls++; return reset(...args); };
const failures = [];
const cycleResults = [];
const metrics = {
  maxJointSeparationM: 0,
  maxFloorPenetrationM: 0,
  maxSelfPenetrationM: 0,
  maxJointLimitErrorRad: 0,
  maxMotorSaturationRatio: 0,
  maxContactCount: 0,
  maxPositionStepM: 0,
  maxLinearVelocityMps: 0,
  errors: new Set(),
};
let priorPositions = null;

const write = (event) => process.stdout.write(`${JSON.stringify(event)}\n`);
const upDot = (q) => 1 - 2 * (q.x ** 2 + q.z ** 2);
const length = (v) => Math.hypot(v.x, v.y, v.z);

function physicalStanding(frame) {
  // The room keeps state "recovering" throughout its post-step, one-second
  // stability observation. Measure that same physical interval here.
  const recovery = frame.diagnostics.recovery;
  if (recovery.phase !== "stand") return false;
  const poses = new Map(frame.segments.map((segment) => [segment.id, segment]));
  let linear = 0;
  let angular = 0;
  for (const definition of SEGMENTS) {
    const segment = poses.get(definition.id);
    linear += definition.massKg * length(segment.linearVelocity) ** 2;
    angular += definition.massKg * length(segment.angularVelocity) ** 2;
  }
  const selected = new Set(recovery.supporting);
  const feet = recovery.contacts.filter((contact) => contact.loadBearing
    && selected.has(contact.segment) && isRecoveryFootSegment(contact.segment)
    && upDot(poses.get(contact.segment).rotation) > 0.85);
  const sides = new Set(feet.map((contact) => SEGMENT_BY_ID.get(contact.segment).side));
  const geometry = supportGeometry(feet, poses, recoveryMassState(poses.values()));
  return sides.has("left") && sides.has("right") && geometry.marginM >= 0
    && recoveryStandingPosture(poses, {
      linear: Math.sqrt(linear / TOTAL_MASS_KG),
      angular: Math.sqrt(angular / TOTAL_MASS_KG),
    });
}

function stageReached(stage, frame, stableFrames) {
  const recovery = frame.diagnostics.recovery;
  const loaded = new Set(recovery.contacts.filter((contact) => contact.loadBearing)
    .map((contact) => contact.segment));
  if (stage === "falling") return frame.state === "falling";
  if (stage === "floor") return frame.state === "fallen" && frame.rootPosition.y < 0.65;
  if (stage === "arm-support") return frame.state === "recovering"
    && ["arm-preparation", "push-brace", "brace"].includes(recovery.transferStage)
    && [...loaded].some((id) => /Hand|Forearm/.test(id));
  if (stage === "kneeling") return frame.state === "recovering"
    && ["kneel", "stand"].includes(recovery.phase)
    && (loaded.has("leftShin") || loaded.has("rightShin"))
    && frame.rootPosition.y > 0.25 && frame.rootPosition.y < 0.9;
  return frame.state === "recovering" && stableFrames < stableFramesRequired
    && frame.rootPosition.y > 0.85
    && (recovery.phase === "stand" || recovery.transferStage === "extend")
    && frame.support.planted.length === 2;
}

function recordPhysics(frame) {
  const d = frame.diagnostics;
  metrics.maxJointSeparationM = Math.max(metrics.maxJointSeparationM, d.maxJointSeparationM);
  metrics.maxFloorPenetrationM = Math.max(metrics.maxFloorPenetrationM, d.maxFloorPenetrationM);
  metrics.maxSelfPenetrationM = Math.max(metrics.maxSelfPenetrationM, d.maxSelfPenetrationM);
  metrics.maxJointLimitErrorRad = Math.max(metrics.maxJointLimitErrorRad, d.maxJointLimitErrorRad);
  metrics.maxMotorSaturationRatio = Math.max(metrics.maxMotorSaturationRatio, d.maxMotorSaturationRatio);
  metrics.maxContactCount = Math.max(metrics.maxContactCount, d.contactDiagnostics.count);
  for (const error of d.errors) metrics.errors.add(error);
  const positions = new Map();
  for (const segment of frame.segments) {
    positions.set(segment.id, segment.position);
    metrics.maxLinearVelocityMps = Math.max(metrics.maxLinearVelocityMps, length(segment.linearVelocity));
    const previous = priorPositions?.get(segment.id);
    if (previous) metrics.maxPositionStepM = Math.max(metrics.maxPositionStepM,
      length({ x: segment.position.x - previous.x, y: segment.position.y - previous.y,
        z: segment.position.z - previous.z }));
  }
  priorPositions = positions;
  if (bodies.size !== 25 || character.ragdollBodies.size !== 25
    || [...bodies].some(([id, body]) => character.ragdollBodies.get(id) !== body || !body.isDynamic())) {
    failures.push(`The same 25 dynamic bodies were not retained at t=${frame.simulationTime.toFixed(3)} s`);
  }
}

function eventFrame(frame, kind, extra = {}) {
  write({ kind, t: frame.simulationTime, impactId: frame.striker.impactId,
    strikes: frame.protocol.strikes, returns: frame.protocol.recoveries,
    state: frame.state, route: frame.diagnostics.recovery.route,
    phase: frame.diagnostics.recovery.phase, stage: frame.diagnostics.recovery.transferStage,
    pelvisY: frame.rootPosition.y, supporting: frame.diagnostics.contactDiagnostics.supportingSegments,
    planted: frame.support.planted, ...extra });
}

try {
  const initial = character.getSnapshot("canvas2d");
  recordPhysics(initial);
  write({ kind: "start", startedAt, sourceHashes: startingHashes, bodies: bodies.size });
  for (const stage of stages) {
    const cycle = cycleResults.length + 1;
    const before = character.getSnapshot("canvas2d");
    const baseImpactId = before.striker.impactId;
    const baseReturns = before.protocol.recoveries;
    if (!before.striker.available || !character.requestStrike()) {
      failures.push(`Cycle ${cycle}: neutral strike was not available`);
      break;
    }
    eventFrame(before, "neutral-request", { cycle, targetStage: stage });
    let lastImpactTime = null;
    let firstImpactTime = null;
    let stageRequestTime = null;
    let stageImpactTime = null;
    let firstStandingTime = null;
    let returnCountTime = null;
    let stableFrames = 0;
    let stableStartTime = null;
    let sawFall = false;
    let previousImpactId = baseImpactId;
    let previousState = before.state;
    let previousPhase = before.diagnostics.recovery.phase;
    let previousReturns = baseReturns;
    let previousFrame = before;
    let outcome = "simulation-limit";
    for (let tick = 1; tick <= 60 * (maxPostImpactS + 6); tick++) {
      character.fixedUpdate(dt, null);
      const frame = character.getSnapshot("canvas2d");
      recordPhysics(frame);
      if (failures.length) { outcome = "physical-ownership-error"; break; }
      if (["falling", "fallen", "recovering"].includes(frame.state)) sawFall = true;
      if (frame.striker.impactId !== previousImpactId) {
        if (firstImpactTime === null) firstImpactTime = frame.simulationTime;
        else {
          stageImpactTime = frame.simulationTime;
          if (!stageReached(stage, previousFrame, stableFrames)) {
            failures.push(`Cycle ${cycle}: ${stage} impact occurred outside the requested physical stage`);
            outcome = "wrong-impact-stage";
          }
        }
        lastImpactTime = frame.simulationTime;
        stableFrames = 0;
        eventFrame(frame, "physical-impact", { cycle,
          preImpactState: previousFrame.state,
          preImpactStage: previousFrame.diagnostics.recovery.transferStage });
        if (outcome === "wrong-impact-stage") break;
      }
      if (frame.state === "upright" && previousState !== "upright" && firstStandingTime === null) {
        firstStandingTime = frame.simulationTime;
        eventFrame(frame, "upright-transition", { cycle });
      }
      if (frame.state !== previousState || frame.diagnostics.recovery.phase !== previousPhase) {
        eventFrame(frame, "motion-transition", { cycle });
      }
      if (lastImpactTime !== null && physicalStanding(frame)) {
        if (stableFrames === 0) stableStartTime = frame.simulationTime;
        stableFrames++;
      } else stableFrames = 0;
      if (frame.protocol.recoveries !== previousReturns) {
        returnCountTime = frame.simulationTime;
        eventFrame(frame, "return-counted", { cycle, stableS: stableFrames * dt,
          controllerStableS: frame.diagnostics.recovery.stableTimeS });
        if (stableFrames < stableFramesRequired
          || frame.diagnostics.recovery.stableTimeS + 1e-9 < 1) {
          failures.push(`Cycle ${cycle}: return counted before one measured stable second`);
          outcome = "premature-return";
          break;
        }
      }
      if (firstImpactTime !== null && stageRequestTime === null && frame.striker.available
        && stageReached(stage, frame, stableFrames)) {
        if (!character.requestStrike()) {
          failures.push(`Cycle ${cycle}: stage ${stage} request was rejected despite availability`);
          outcome = "stage-request-rejected";
          break;
        }
        stageRequestTime = frame.simulationTime;
        eventFrame(frame, "stage-request", { cycle, targetStage: stage });
      }
      if (stageRequestTime !== null && stageImpactTime === null && frame.striker.available
        && frame.simulationTime - stageRequestTime > dt) {
        failures.push(`Cycle ${cycle}: stage ${stage} shot missed`);
        outcome = "missed-stage-impact";
        break;
      }
      if (stageImpactTime !== null && sawFall && frame.protocol.recoveries > baseReturns
        && stableFrames >= stableFramesRequired) {
        outcome = "stable-return";
        eventFrame(frame, "cycle-complete", { cycle, stableS: stableFrames * dt });
        break;
      }
      if (firstImpactTime === null && frame.simulationTime - before.simulationTime >= 5) {
        failures.push(`Cycle ${cycle}: neutral shot had no physical impact within 5 s`);
        outcome = "missed-neutral-impact";
        break;
      }
      if (lastImpactTime !== null && frame.simulationTime - lastImpactTime >= maxPostImpactS) {
        failures.push(`Cycle ${cycle}: no stable return within ${maxPostImpactS} s after the last impact`);
        outcome = "recovery-timeout";
        break;
      }
      previousImpactId = frame.striker.impactId;
      previousState = frame.state;
      previousPhase = frame.diagnostics.recovery.phase;
      previousReturns = frame.protocol.recoveries;
      previousFrame = frame;
    }
    const final = character.getSnapshot("canvas2d");
    cycleResults.push({ cycle, targetStage: stage, outcome, firstImpactTime, stageRequestTime,
      stageImpactTime, lastImpactTime, firstStandingTime, returnCountTime, stableStartTime,
      stableS: stableFrames * dt, finalTime: final.simulationTime,
      impactId: final.striker.impactId, returns: final.protocol.recoveries,
      route: final.diagnostics.recovery.route, phase: final.diagnostics.recovery.phase,
      transferStage: final.diagnostics.recovery.transferStage });
    if (outcome !== "stable-return") {
      if (!failures.length) failures.push(`Cycle ${cycle}: ${outcome}`);
      break;
    }
  }
  const finishedAt = new Date().toISOString();
  const finishingHashes = hashSources();
  const changedSources = sourcePaths.filter((path) => startingHashes[path] !== finishingHashes[path]);
  const numericMetrics = { ...metrics, errors: [...metrics.errors] };
  const thresholds = {
    maxJointSeparationM: ACCEPTANCE.maxJointSeparationM,
    maxFloorPenetrationM: ACCEPTANCE.maxFloorPenetrationM,
    maxSelfPenetrationM: ACCEPTANCE.maxNonExcludedSelfPenetrationM,
    maxJointLimitErrorRad: ACCEPTANCE.maximumStructuralLimitErrorRad,
  };
  for (const [key, limit] of Object.entries(thresholds)) {
    if (numericMetrics[key] > limit) failures.push(`${key}=${numericMetrics[key]} exceeds ${limit}`);
  }
  if (metrics.errors.size) failures.push(`Runtime errors: ${[...metrics.errors].join(", ")}`);
  if (changedSources.length) failures.push(`Simulation sources changed during run: ${changedSources.join(", ")}`);
  if (cycleResults.length !== 5) failures.push(`Completed ${cycleResults.length}/5 cycles`);
  if (resetCalls) failures.push(`Automatic reset called ${resetCalls} times`);
  write({ kind: "summary", startedAt, finishedAt, changedSources, fixedHz: 60,
    resetCalls, cycleResults, physics: numericMetrics, thresholds, failures,
    passed: failures.length === 0 && cycleResults.every((cycle) => cycle.outcome === "stable-return") });
  if (failures.length) process.exitCode = 1;
} finally {
  character.dispose();
  unregister();
}
