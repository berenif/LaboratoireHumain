import { register } from "tsx/esm/api";

const unregister = register();
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { quatFromAxisAngle, rotate, worldPoint } = await import("../src/character/math.ts");

const dt = 1 / 60;
const zero = { x: 0, y: 0, z: 0 };
const recoveryStates = new Set(["falling", "fallen", "recovering"]);

function compact(snapshot) {
  const balance = snapshot.diagnostics.balance;
  return {
    sequence: snapshot.sequence,
    time: Number(snapshot.simulationTime.toFixed(3)),
    state: snapshot.state,
    stepCount: snapshot.diagnostics.stepCount,
    root: Object.fromEntries(Object.entries(snapshot.rootPosition).map(([key, value]) => [key, Number(value.toFixed(4))])),
    lean: Number(snapshot.diagnostics.leanRadians.toFixed(4)),
    support: snapshot.support,
    contacts: snapshot.diagnostics.contactDiagnostics,
    activeGrab: snapshot.diagnostics.activeGrab,
    appliedGrabForceN: Number(snapshot.diagnostics.appliedGrabForceN.toFixed(2)),
    balance: balance ? {
      supportMarginM: Number(balance.supportMarginM.toFixed(4)),
      instabilitySeconds: Number(balance.instabilitySeconds.toFixed(4)),
      recoveryCapacityM: Number(balance.recoveryCapacityM.toFixed(4)),
      supportingFeet: balance.supportingFeet,
      centerOfMassVelocity: Object.fromEntries(Object.entries(balance.centerOfMassVelocity).map(([key, value]) => [key, Number(value.toFixed(4))])),
      externalForce: Object.fromEntries(Object.entries(balance.externalForce).map(([key, value]) => [key, Number(value.toFixed(2))])),
      stepTarget: balance.stepTarget,
    } : null,
  };
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
      history.push(compact(snapshot));
      if (history.length > 12) history.shift();
      if (recoveryStates.has(snapshot.state)) {
        console.log("SLOW_PULL_FIRST_RECOVERY", JSON.stringify({ tick, history }, null, 2));
        return;
      }
    }
    console.log("SLOW_PULL_NO_RECOVERY", JSON.stringify(history, null, 2));
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
      history.push(compact(snapshot));
      if (history.length > 12) history.shift();
      if (recoveryStates.has(snapshot.state)) {
        console.log("PLANTED_REVERSAL_FIRST_RECOVERY", JSON.stringify({ tick, history }, null, 2));
        return;
      }
    }
    console.log("PLANTED_REVERSAL_NO_RECOVERY", JSON.stringify(history, null, 2));
  } finally {
    character.dispose();
  }
}

try {
  await runSlowPull();
  await runPlantedReversal();
} finally {
  unregister();
}
