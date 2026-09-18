import { register } from "tsx/esm/api";

const unregister = register();
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { length, sub, worldPoint } = await import("../src/character/math.ts");

const dt = 1 / 60;
const zero = { x: 0, y: 0, z: 0 };
const recoveryStates = new Set(["falling", "fallen", "recovering"]);

function roundedVec(vector, digits = 4) {
  if (!vector) return null;
  return Object.fromEntries(Object.entries(vector).map(([key, value]) => [key, Number(value.toFixed(digits))]));
}

function compact(snapshot) {
  const balance = snapshot.diagnostics.balance;
  const byId = new Map(snapshot.segments.map((pose) => [pose.id, pose]));
  const target = balance?.stepTarget ?? null;
  const swingId = snapshot.support.swingFoot;
  const swing = swingId ? byId.get(swingId) : null;
  const horizontalError = swing && target
    ? Math.hypot(swing.position.x - target.x, swing.position.z - target.z)
    : null;
  const trackedSegments = ["leftFoot", "leftForefoot", "rightFoot", "rightForefoot"];
  const trackedJoints = new Set(["leftThigh", "leftShin", "leftAnkle", "leftFoot", "rightThigh", "rightShin", "rightAnkle", "rightFoot"]);
  return {
    sequence: snapshot.sequence,
    time: Number(snapshot.simulationTime.toFixed(3)),
    state: snapshot.state,
    stepCount: snapshot.diagnostics.stepCount,
    root: roundedVec(snapshot.rootPosition),
    lean: Number(snapshot.diagnostics.leanRadians.toFixed(4)),
    support: snapshot.support,
    contacts: snapshot.diagnostics.contactDiagnostics,
    activeGrab: snapshot.diagnostics.activeGrab,
    appliedGrabForceN: Number(snapshot.diagnostics.appliedGrabForceN.toFixed(2)),
    feet: Object.fromEntries(trackedSegments.map((id) => {
      const pose = byId.get(id);
      return [id, pose ? { position: roundedVec(pose.position), velocity: roundedVec(pose.linearVelocity) } : null];
    })),
    swingTracking: swing && target ? {
      foot: swingId,
      target: roundedVec(target),
      actual: roundedVec(swing.position),
      errorM: Number(length(sub(swing.position, target)).toFixed(4)),
      horizontalErrorM: Number(horizontalError.toFixed(4)),
      verticalErrorM: Number((swing.position.y - target.y).toFixed(4)),
    } : null,
    legMotors: snapshot.diagnostics.jointDiagnostics
      .filter((joint) => trackedJoints.has(joint.segment))
      .map((joint) => ({
        segment: joint.segment,
        target: roundedVec(joint.targetCoordinates),
        actual: roundedVec(joint.coordinates),
        limitErrorRad: Number(joint.limitErrorMagnitudeRad.toFixed(4)),
        saturation: Number(joint.motorSaturationRatio.toFixed(3)),
        torqueNm: Number(joint.motorTorqueNm.toFixed(2)),
      })),
    balance: balance ? {
      supportMarginM: Number(balance.supportMarginM.toFixed(4)),
      instabilitySeconds: Number(balance.instabilitySeconds.toFixed(4)),
      recoveryCapacityM: Number(balance.recoveryCapacityM.toFixed(4)),
      supportingFeet: balance.supportingFeet,
      centerOfMassVelocity: roundedVec(balance.centerOfMassVelocity),
      externalForce: roundedVec(balance.externalForce, 2),
      stepTarget: roundedVec(balance.stepTarget),
    } : null,
  };
}

function pushHistory(history, snapshot) {
  history.push(compact(snapshot));
  if (history.length > 12) history.shift();
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
      pushHistory(history, snapshot);
      if (recoveryStates.has(snapshot.state)) {
        console.error("SLOW_PULL_FIRST_RECOVERY", JSON.stringify({ tick, history }, null, 2));
        return false;
      }
    }
    console.log("SLOW_PULL_NO_RECOVERY", JSON.stringify(history.at(-1), null, 2));
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
      pushHistory(history, snapshot);
      if (recoveryStates.has(snapshot.state)) {
        console.error("PLANTED_REVERSAL_FIRST_RECOVERY", JSON.stringify({ tick, history }, null, 2));
        return false;
      }
    }
    console.log("PLANTED_REVERSAL_NO_RECOVERY", JSON.stringify(history.at(-1), null, 2));
    return true;
  } finally {
    character.dispose();
  }
}

try {
  const slowPullStable = await runSlowPull();
  const plantedReversalStable = await runPlantedReversal();
  if (!slowPullStable || !plantedReversalStable) {
    throw new Error("A balance scenario entered the recovery state");
  }
} finally {
  unregister();
}
