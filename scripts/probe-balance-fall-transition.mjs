import { mkdirSync, appendFileSync } from "node:fs";
import { join } from "node:path";
import { register } from "tsx/esm/api";

const unregister = register();
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { worldPoint } = await import("../src/character/math.ts");

const dt = 1 / 60;
const zero = { x: 0, y: 0, z: 0 };
const recoveryStates = new Set(["falling", "fallen", "recovering"]);

function round(value, digits = 4) {
  return Number.isFinite(value) ? Number(value.toFixed(digits)) : null;
}

function point(vector) {
  return vector ? { x: round(vector.x), y: round(vector.y), z: round(vector.z) } : null;
}

function frame(snapshot) {
  const balance = snapshot.diagnostics.balance;
  const swingId = snapshot.support.swingFoot;
  const swing = swingId ? snapshot.segments.find((pose) => pose.id === swingId) : null;
  const target = balance?.stepTarget ?? null;
  const recoveryContacts = snapshot.diagnostics.contactDiagnostics?.contacts
    ?? snapshot.diagnostics.recovery?.contacts ?? [];
  const contacts = recoveryContacts
    .map((contact) => ({
      segment: contact.segment,
      forceN: round(contact.forceN, 1),
      loadBearing: contact.loadBearing,
      persistenceS: round(contact.persistenceS),
      point: point(contact.point),
      points: contact.points?.map(point),
    }));
  const motors = snapshot.diagnostics.jointDiagnostics
    .filter((joint) => /^(left|right)(Thigh|Shin|Ankle|Foot|Forefoot)$/.test(joint.segment))
    .map((joint) => ({
      segment: joint.segment,
      errorRad: round(joint.limitErrorMagnitudeRad),
      saturation: round(joint.motorSaturationRatio, 3),
      torqueNm: round(joint.motorTorqueNm, 1),
      torqueWorld: point(joint.motorTorqueWorld),
      coordinates: point(joint.coordinates),
      targetCoordinates: point(joint.targetCoordinates),
      limitError: point(joint.limitError),
    }));
  return {
    t: round(snapshot.simulationTime, 3),
    state: snapshot.state,
    rootY: round(snapshot.rootPosition.y),
    lean: round(snapshot.diagnostics.leanRadians),
    stepCount: snapshot.diagnostics.stepCount,
    progress: round(snapshot.support.stepProgress, 3),
    planted: snapshot.support.planted,
    swing: swingId,
    target: point(target),
    actual: point(swing?.position),
    velocity: point(swing?.linearVelocity),
    horizontalErrorM: swing && target
      ? round(Math.hypot(swing.position.x - target.x, swing.position.z - target.z)) : null,
    verticalErrorM: swing && target ? round(swing.position.y - target.y) : null,
    supportingFeet: balance?.supportingFeet ?? [],
    supportMarginM: round(balance?.supportMarginM),
    contactSummary: snapshot.diagnostics.contactDiagnostics,
    chain: snapshot.diagnostics.standingChain ?? null,
    integrity: {
      finite: snapshot.diagnostics.finite,
      errors: snapshot.diagnostics.errors,
      ownership: snapshot.diagnostics.physicsOwnership,
      maxJointSeparationM: snapshot.diagnostics.maxJointSeparationM,
      maxFloorPenetrationM: snapshot.diagnostics.maxFloorPenetrationM,
    },
    contacts,
    motors,
  };
}

function pushHistory(history, snapshot, scenario) {
  const sample = frame(snapshot);
  if (process.env.BALANCE_TRACE_DIR) {
    mkdirSync(process.env.BALANCE_TRACE_DIR, { recursive: true });
    appendFileSync(join(process.env.BALANCE_TRACE_DIR, `${scenario}.jsonl`), `${JSON.stringify(sample)}\n`);
  }
  history.push(sample);
  if (history.length > 8) history.shift();
}

function reportFailure(label, tick, history) {
  console.error(label, JSON.stringify({ tick, frames: history }));
}

async function runSlowPull() {
  const character = await createEmbodiedCharacter("canvas2d");
  const history = [];
  try {
    const start = character.getSnapshot("canvas2d").segments.find((pose) => pose.id === "rightHand").position;
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 3, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: start, timestampMs: 0,
    });
    for (let tick = 1; tick <= 510; tick += 1) {
      let command = null;
      if (tick <= 90) {
        command = {
          kind: "move", pointerId: 3,
          worldTarget: { x: start.x + 0.60 * tick / 90, y: start.y, z: start.z + 0.08 * tick / 90 },
          timestampMs: tick * dt * 1000,
        };
      } else if (tick <= 180) {
        command = {
          kind: "move", pointerId: 3,
          worldTarget: { x: start.x + 0.6 - (tick - 90) * 0.9 / 90, y: start.y, z: start.z + 0.08 },
          timestampMs: tick * dt * 1000,
        };
      } else if (tick === 181) {
        command = { kind: "end", pointerId: 3, timestampMs: tick * dt * 1000 };
      }
      character.fixedUpdate(dt, command);
      const snapshot = character.getSnapshot("canvas2d");
      pushHistory(history, snapshot, "slow-pull");
      if (!snapshot.diagnostics.finite || snapshot.diagnostics.errors.length
        || snapshot.diagnostics.maxJointSeparationM > 0.08
        || snapshot.diagnostics.maxFloorPenetrationM > 0.08) {
        reportFailure("PHYSICAL_INTEGRITY_FAILURE", tick, history);
        return false;
      }
      if (recoveryStates.has(snapshot.state)) {
        reportFailure("SLOW_PULL_FIRST_RECOVERY", tick, history);
        return false;
      }
    }
    if (character.getSnapshot("canvas2d").support.swingFoot !== null) {
      reportFailure("SLOW_PULL_UNFINISHED_STEP", 510, history);
      return false;
    }
    console.log("SLOW_PULL_NO_RECOVERY", JSON.stringify(history.at(-1)));
    return true;
  } finally {
    character.dispose();
  }
}

async function runPlantedReversal() {
  const character = await createEmbodiedCharacter("canvas2d");
  const localAnchor = { x: 0.025, y: 0.015, z: 0.01 };
  const history = [];
  try {
    const hand = character.getSnapshot("canvas2d").segments.find((pose) => pose.id === "rightHand");
    const start = worldPoint(hand.position, hand.rotation, localAnchor);
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 41, region: "rightHand", segment: "rightHand",
      localAnchor, worldTarget: start, timestampMs: 0,
    });
    for (let tick = 1; tick <= 510; tick += 1) {
      let command = null;
      if (tick < 300) {
        const x = tick <= 90 ? 0.75 * tick / 90
          : tick <= 180 ? 0.75 - 1.5 * (tick - 90) / 90 : -0.75;
        command = {
          kind: "move", pointerId: 41,
          worldTarget: { x: start.x + x, y: start.y + 0.02 * Math.min(tick, 90) / 90, z: start.z },
          timestampMs: tick * dt * 1000,
        };
      } else if (tick === 300) {
        command = { kind: "end", pointerId: 41, timestampMs: tick * dt * 1000 };
      }
      character.fixedUpdate(dt, command);
      const snapshot = character.getSnapshot("canvas2d");
      pushHistory(history, snapshot, "planted-reversal");
      if (!snapshot.diagnostics.finite || snapshot.diagnostics.errors.length
        || snapshot.diagnostics.maxJointSeparationM > 0.08
        || snapshot.diagnostics.maxFloorPenetrationM > 0.08) {
        reportFailure("PHYSICAL_INTEGRITY_FAILURE", tick, history);
        return false;
      }
      if (recoveryStates.has(snapshot.state)) {
        reportFailure("PLANTED_REVERSAL_FIRST_RECOVERY", tick, history);
        return false;
      }
    }
    if (character.getSnapshot("canvas2d").support.swingFoot !== null) {
      reportFailure("PLANTED_REVERSAL_UNFINISHED_STEP", 510, history);
      return false;
    }
    console.log("PLANTED_REVERSAL_NO_RECOVERY", JSON.stringify(history.at(-1)));
    return true;
  } finally {
    character.dispose();
  }
}

try {
  const slowPullStable = await runSlowPull();
  const plantedReversalStable = await runPlantedReversal();
  if (!slowPullStable || !plantedReversalStable) {
    throw new Error("A balance scenario failed recovery, integrity, or step completion");
  }
} finally {
  unregister();
}
