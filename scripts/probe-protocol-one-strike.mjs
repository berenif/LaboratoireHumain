import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import { register } from "tsx/esm/api";

const sourcePaths = [
  "../src/character/BalanceController.ts",
  "../src/character/DynamicRecovery.ts",
  "../src/character/EmbodiedCharacter.ts",
  "../src/character/PhysicsStriker.ts",
  "../src/character/contact-loads.ts",
  "../src/character/joint-coordinates.ts",
  "../src/character/joint-motors.ts",
  "../src/character/native-joint-motors.ts",
  "../src/character/physics-settings.ts",
  "../src/character/recovery-forefoot-anchor.ts",
  "../src/character/recovery-foot-targets.ts",
  "../src/character/recovery-joints.ts",
  "../src/character/recovery-support.ts",
  "../src/core/protocol.ts",
  "../src/core/humanoid.ts",
  "../src/core/geometry.ts",
  "../src/core/types.ts",
  "./probe-protocol-one-strike.mjs",
];
const sourceHashes = () => Object.fromEntries(sourcePaths.map((path) => [path,
  createHash("sha256").update(readFileSync(new URL(path, import.meta.url))).digest("hex")]));
const startedAt = new Date().toISOString();
const startingSourceHashes = sourceHashes();
process.stdout.write(`${JSON.stringify({ sourceStart: startedAt, hashes: startingSourceHashes })}\n`);
const unregister = register();
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { recoveryStandingPosture } = await import("../src/character/DynamicRecovery.ts");
const { isRecoveryFootSegment, recoveryMassState, supportGeometry } = await import("../src/character/recovery-support.ts");
const { SEGMENT_BY_ID, SEGMENTS, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const dt = 1 / 60;
const probeSeconds = process.env.PROTOCOL_PROBE_SECONDS === undefined
  ? null : Number(process.env.PROTOCOL_PROBE_SECONDS);
if (probeSeconds !== null && (!Number.isFinite(probeSeconds) || probeSeconds <= 0)) {
  throw new Error("PROTOCOL_PROBE_SECONDS must be a positive number of simulated seconds");
}
const character = await createEmbodiedCharacter("canvas2d", { room: true });
const trialSolverIterations = process.env.PROTOCOL_RECOVERY_SOLVER_ITERATIONS === undefined
  ? null : Number(process.env.PROTOCOL_RECOVERY_SOLVER_ITERATIONS);
if (trialSolverIterations !== null && (!Number.isInteger(trialSolverIterations) || trialSolverIterations < 1)) throw new Error("Invalid solver trial count");
const softRollTrial = process.env.PROTOCOL_SOFT_ROLL === "1";
if (softRollTrial) {
  const apply = character.recovery.motorBackend;
  character.recovery.motorBackend = commands => apply(commands.map(command => {
    if (character.recovery.diagnostics().transferStage !== "roll") return command;
    return { ...command, stiffness: command.stiffness * .08, damping: command.damping * .3 };
  }));
}
// Nested buckets intentionally overlap; world integration is disjoint from planning.
const sections = {};
for (const [object, methods] of [[character.world, ["step"]],
  [character.recovery, ["chooseRoute", "transfer", "actuate", "limbTargets", "legalArmCommand"]]]) {
  for (const method of methods) {
    const original = object[method].bind(object);
    object[method] = (...args) => {
      const start = performance.now();
      try { return original(...args); }
      finally {
        const bucket = sections[method] ??= { calls: 0, totalMs: 0, maxMs: 0 };
        const elapsed = performance.now() - start;
        bucket.calls++;
        bucket.totalMs += elapsed;
        bucket.maxMs = Math.max(bucket.maxMs, elapsed);
      }
    };
  }
}
const bodyRefs = new Map(character.ragdollBodies);
const sampledMax = { jointGapM: 0, floorDepthM: 0, selfDepthM: 0, limitErrorRad: 0 };
const everyTickMax = { ...sampledMax };
const trajectoryHash = createHash("sha256");

function upDot(rotation) {
  // Y component of q * (0, 1, 0) * q^-1.
  return 1 - 2 * (rotation.x ** 2 + rotation.z ** 2);
}

function physicalStanding(frame) {
  const recovery = frame.diagnostics.recovery;
  if (recovery.phase !== "stand") return false;
  const poses = new Map(frame.segments.map(segment => [segment.id, segment]));
  let linear = 0;
  let angular = 0;
  for (const definition of SEGMENTS) {
    const body = poses.get(definition.id);
    const velocity = body.linearVelocity;
    const spin = body.angularVelocity;
    linear += definition.massKg * (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2);
    angular += definition.massKg * (spin.x ** 2 + spin.y ** 2 + spin.z ** 2);
  }
  const selected = new Set(recovery.supporting);
  const feet = recovery.contacts.filter(contact => contact.loadBearing
    && selected.has(contact.segment) && isRecoveryFootSegment(contact.segment)
    && upDot(poses.get(contact.segment).rotation) > 0.85);
  const sides = new Set(feet.map(contact => SEGMENT_BY_ID.get(contact.segment).side));
  const geometry = supportGeometry(feet, poses, recoveryMassState(poses.values()));
  return sides.has("left") && sides.has("right") && geometry.marginM >= 0
    && recoveryStandingPosture(poses, {
      linear: Math.sqrt(linear / TOTAL_MASS_KG),
      angular: Math.sqrt(angular / TOTAL_MASS_KG),
    });
}

function report(frame, tick, label, stableTicks) {
  const recovery = frame.diagnostics.recovery;
  const d = frame.diagnostics;
  sampledMax.jointGapM = Math.max(sampledMax.jointGapM, d.maxJointSeparationM);
  sampledMax.floorDepthM = Math.max(sampledMax.floorDepthM, d.maxFloorPenetrationM);
  sampledMax.selfDepthM = Math.max(sampledMax.selfDepthM, d.maxSelfPenetrationM);
  sampledMax.limitErrorRad = Math.max(sampledMax.limitErrorRad, d.maxJointLimitErrorRad);
  const values = {
    tick,
    timeS: Number(frame.simulationTime.toFixed(3)),
    sinceImpactS: frame.striker.impactId > 0 ? Number((frame.simulationTime - impactTime).toFixed(3)) : null,
    label,
    machine: frame.striker.phase,
    machineAvailable: frame.striker.available,
    impactId: frame.striker.impactId,
    state: frame.state,
    recoveryPhase: recovery.phase,
    recoveryRoute: recovery.route,
    transferStage: recovery.transferStage,
    blockingPredicate: recovery.blockingPredicate,
    retryReason: recovery.retryReason,
    recoveryRetries: recovery.retries,
    stalledTimeS: recovery.stalledTimeS,
    progressError: recovery.progressError,
    contacts: d.contactDiagnostics.count,
    loadBearingContacts: d.contactDiagnostics.loadBearingCount,
    supporting: d.contactDiagnostics.supportingSegments,
    planted: frame.support.planted,
    pelvisY: Number(frame.rootPosition.y.toFixed(3)),
    stableS: Number((stableTicks * dt).toFixed(3)),
    returns: frame.protocol.recoveries,
    jointGapM: Number(d.maxJointSeparationM.toFixed(4)),
    floorDepthM: Number(d.maxFloorPenetrationM.toFixed(4)),
    selfDepthM: Number(d.maxSelfPenetrationM.toFixed(4)),
    limitErrorRad: Number(d.maxJointLimitErrorRad.toFixed(4)),
    motorSaturation: Number(d.maxMotorSaturationRatio.toFixed(3)),
    errors: d.errors,
  };
  process.stdout.write(`${JSON.stringify(values)}\n`);
}

let impactTime = null;
let firstUprightTime = null;
let firstReturnCountTime = null;
let stableStartTime = null;
let stableCompletionTime = null;
let stableTicks = 0;
let lastState = character.state;
let lastPhase = character.striker.phase;
let lastImpactId = character.striker.impactId;
let lastReturns = character.protocolRecoveries;
let endReason = "time-limit";
const stepDurations = [];
const phaseTimings = {};
const replayStart = performance.now();
try {
  const initial = character.getSnapshot("canvas2d");
  report(initial, 0, "initial", stableTicks);
  if (!character.requestStrike()) throw new Error("Initial strike request was rejected");
  for (let tick = 1; tick <= (probeSeconds === null ? 1800 : Math.ceil(probeSeconds / dt)); tick++) {
    const phaseBefore = character.getSnapshot("canvas2d").diagnostics.recovery.phase;
    if (tick > 56 && trialSolverIterations !== null) character.world.numSolverIterations = trialSolverIterations;
    const stepStart = performance.now();
    character.fixedUpdate(dt, null);
    const stepMs = performance.now() - stepStart;
    stepDurations.push(stepMs);
    const timing = phaseTimings[phaseBefore] ??= { steps: 0, totalMs: 0, maxMs: 0 };
    timing.steps++;
    timing.totalMs += stepMs;
    timing.maxMs = Math.max(timing.maxMs, stepMs);
    const frame = character.getSnapshot("canvas2d");
    trajectoryHash.update(JSON.stringify(frame.segments));
    for (const [key, value] of Object.entries({ jointGapM: frame.diagnostics.maxJointSeparationM,
      floorDepthM: frame.diagnostics.maxFloorPenetrationM, selfDepthM: frame.diagnostics.maxSelfPenetrationM,
      limitErrorRad: frame.diagnostics.maxJointLimitErrorRad })) everyTickMax[key] = Math.max(everyTickMax[key], value);
    if (physicalStanding(frame) && impactTime !== null) {
      if (stableTicks === 0) stableStartTime = character.simulationTime;
      stableTicks++;
      if (stableTicks === 60 && stableCompletionTime === null) stableCompletionTime = character.simulationTime;
    } else stableTicks = 0;
    const phase = character.striker.phase;
    const state = character.state;
    const impactId = character.striker.impactId;
    const returns = character.protocolRecoveries;
    if (impactId > lastImpactId) impactTime = character.simulationTime;
    if (state === "upright" && lastState !== "upright" && firstUprightTime === null) {
      firstUprightTime = character.simulationTime;
    }
    if (returns > lastReturns && firstReturnCountTime === null) firstReturnCountTime = character.simulationTime;
    const changed = phase !== lastPhase || state !== lastState
      || impactId !== lastImpactId || returns !== lastReturns;
    if (changed || tick % 60 === 0) {
      report(character.getSnapshot("canvas2d"), tick, changed ? "transition" : "sample", stableTicks);
    }
    lastPhase = phase;
    lastState = state;
    lastImpactId = impactId;
    lastReturns = returns;
    if (impactTime === null && tick >= 300) {
      endReason = "no-impact-by-5s";
      break;
    }
    if (returns > 0 && stableTicks >= 60) {
      endReason = "stable-return";
      break;
    }
    if (probeSeconds !== null && character.simulationTime >= probeSeconds - 1e-9) {
      endReason = "probe-limit";
      break;
    }
    if (impactTime !== null && character.simulationTime - impactTime >= 25) break;
  }
  const final = character.getSnapshot("canvas2d");
  const bodiesRetained = bodyRefs.size === 25 && character.ragdollBodies.size === 25
    && [...bodyRefs].every(([id, body]) => character.ragdollBodies.get(id) === body && body.isDynamic());
  const finishedAt = new Date().toISOString();
  const finishingSourceHashes = sourceHashes();
  const changedSources = sourcePaths.filter((path) => startingSourceHashes[path] !== finishingSourceHashes[path]);
  report(final, final.sequence, `final:${endReason}`, stableTicks);
  process.stdout.write(`${JSON.stringify({
    summary: true,
    performance: {
      wallSeconds: (performance.now() - replayStart) / 1000,
      simulatedSeconds: final.simulationTime,
      physicsSeconds: stepDurations.reduce((sum, ms) => sum + ms, 0) / 1000,
      physicsRealtimeRatio: final.simulationTime / (stepDurations.reduce((sum, ms) => sum + ms, 0) / 1000),
      p95StepMs: [...stepDurations].sort((a, b) => a - b)[Math.ceil(stepDurations.length * .95) - 1],
      phaseTimings,
      sections,
      trialSolverIterations,
      softRollTrial,
    },
    startedAt,
    finishedAt,
    changedSources,
    endReason,
    impactTime,
    firstUprightTime,
    firstReturnCountTime,
    stableStartTime,
    stableCompletionTime,
    elapsedAfterImpactS: impactTime === null ? null : final.simulationTime - impactTime,
    strikes: final.protocol.strikes,
    recoveries: final.protocol.recoveries,
    stableS: stableTicks * dt,
    bodiesRetained,
    bodyCount: character.ragdollBodies.size,
    sampledMax,
    everyTickMax,
    trajectorySha256: trajectoryHash.digest("hex"),
    probeValid: bodiesRetained && final.protocol.strikes === 1,
    recoveryPassed: endReason === "stable-return" && bodiesRetained && final.protocol.strikes === 1,
  })}\n`);
  if ((probeSeconds === null && endReason !== "stable-return")
    || !bodiesRetained || final.protocol.strikes !== 1) process.exitCode = 1;
} finally {
  character.dispose();
  unregister();
}
