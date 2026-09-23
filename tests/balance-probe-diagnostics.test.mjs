import assert from "node:assert/strict";
import test from "node:test";
import { stepLoadDiagnostics } from "../scripts/balance-probe-diagnostics.mjs";

test("transfer loads use the planned moving foot before a swing exists", () => {
  const definitions = new Map([
    ["leftFoot", { side: "left", role: "hindfoot" }],
    ["rightFoot", { side: "right", role: "hindfoot" }],
    ["rightForefoot", { side: "right", role: "forefoot" }],
    ["rightHand", { side: "right", role: "hand" }],
  ]);
  const contact = (segment, forceN, extra = {}) => ({ segment, forceN,
    normalY: 1, loadBearing: true, persistenceS: 0.1, ...extra });
  const snapshot = { support: { swingFoot: null }, diagnostics: {
    standingChain: { step: { foot: "leftFoot", elapsedS: -0.001 } },
    contactDiagnostics: { contacts: [contact("leftFoot", 90), contact("rightFoot", 400),
      contact("rightForefoot", 30, { persistenceS: 0.01 }), contact("rightHand", 100),
      contact("leftFoot", 10, { normalY: 0.2 })] },
  } };
  assert.deepEqual(stepLoadDiagnostics(snapshot, definitions), {
    movingFoot: "leftFoot", movingSide: "left", movingLoadN: 90,
    movingQualifiedLoadN: 90, retainedLoadN: 400, retainedNormalLoadN: 430,
  });
  snapshot.support.swingFoot = "leftFoot";
  snapshot.diagnostics.standingChain.step.elapsedS = 0.1;
  assert.equal(stepLoadDiagnostics(snapshot, definitions).retainedLoadN, 400);
});
