import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { SEGMENTS } = await import("../src/core/humanoid.ts");
const { REGION_IDS, SEGMENT_IDS } = await import("../src/core/types.ts");
const { ACCEPTANCE } = await import("../scripts/physics-fixtures.ts");

const fixtureFile = new URL("../scripts/fixtures/native-fixed-streams.json", import.meta.url);
const capture = JSON.parse(readFileSync(fixtureFile, "utf8"));
const addedSegments = [
  "lumbar",
  "leftShoulderGirdle",
  "leftForearmTwist",
  "rightShoulderGirdle",
  "rightForearmTwist",
  "leftAnkle",
  "leftForefoot",
  "rightAnkle",
  "rightForefoot",
];
const diagnosticsKeys = [
  "activeGrab", "activePointerId", "appliedGrabForceN", "balance", "bodyInputAvailable",
  "contactDiagnostics", "droppedTimeMs", "errors", "finite", "fixedSteps", "grabControl",
  "interactiveViewReady", "jointDiagnostics", "leanRadians", "maxFloorPenetrationM",
  "maxJointLimitErrorRad", "maxJointSeparationM", "maxMotorSaturationRatio", "physicsOwnership",
  "queuedTarget", "recovery", "renderer", "rootDisplacementM", "selectedRegion",
  "selectedSegment", "simulationReady", "state", "stepCount", "support",
].sort();
const streamSummary = new Map([
  ["native-webgl-two-cycles", { updates: 1235, commands: 16, lastSequence: 1235 }],
  ["native-canvas2d-two-cycles", { updates: 1194, commands: 16, lastSequence: 1194 }],
]);

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function rotationDifferenceDegrees(a, b) {
  const cosine = Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    / (Math.hypot(a.x, a.y, a.z, a.w) * Math.hypot(b.x, b.y, b.z, b.w));
  return 2 * Math.acos(Math.max(-1, Math.min(1, cosine))) * 180 / Math.PI;
}

test("native fixed streams retain their captured commands under the migrated schema", () => {
  assert.equal(capture.schema, 2);
  assert.deepEqual(capture.migration.preservedCaptureData, ["updates", "recordedTransfers"]);
  assert.equal(capture.migration.anatomy, "25-segment");
  assert.equal(capture.migration.physicsOwnership, "rapier-dynamic");
  assert.equal(capture.fixtures.length, 2);

  for (const fixture of capture.fixtures) {
    const expected = streamSummary.get(fixture.id);
    assert.ok(expected, `unexpected fixture ${fixture.id}`);
    assert.equal(fixture.updates.length, expected.updates, `${fixture.id} update count`);
    assert.equal(fixture.updates.at(-1).sequence, expected.lastSequence, `${fixture.id} final sequence`);
    assert.equal(fixture.updates.filter(({ command }) => command).length, expected.commands,
      `${fixture.id} command count`);
    assert.equal(fixture.recordedTransfers.length, 2, `${fixture.id} historical recovery count`);
    for (const [index, update] of fixture.updates.entries()) {
      assert.equal(update.sequence, index + 1, `${fixture.id} sequence ${index + 1}`);
      assert.equal(update.dt, ACCEPTANCE.fixedDtS, `${fixture.id} fixed dt at ${index + 1}`);
      if (!update.command) continue;
      if (update.command.region !== undefined) assert.ok(REGION_IDS.includes(update.command.region));
      if (update.command.segment !== undefined) assert.ok(SEGMENT_IDS.includes(update.command.segment));
    }
  }
});

test("native fixed streams start from the current 25-segment continuous dynamic assembly", async () => {
  for (const fixture of capture.fixtures) {
    const renderer = fixture.id.includes("canvas2d") ? "canvas2d" : "webgl";
    const snapshot = fixture.initialSnapshot;
    assert.equal(snapshot.renderer, undefined, "renderer belongs to diagnostics, not the pose root");
    assert.deepEqual(snapshot.segments.map(({ id }) => id), SEGMENT_IDS, `${fixture.id} segment order`);
    assert.ok(addedSegments.every((id) => snapshot.segments.some((segment) => segment.id === id)));
    assert.equal(snapshot.diagnostics.physicsOwnership, "rapier-dynamic");
    assert.equal(snapshot.diagnostics.renderer, renderer);
    assert.equal(Object.hasOwn(snapshot.diagnostics, "authority"), false);
    assert.equal(Object.hasOwn(snapshot.diagnostics, "handoff"), false);
    assert.deepEqual(Object.keys(snapshot.diagnostics).sort(), diagnosticsKeys);
    assert.deepEqual(
      snapshot.diagnostics.jointDiagnostics.map(({ segment }) => segment),
      SEGMENTS.filter(({ parent }) => parent).map(({ id }) => id),
      `${fixture.id} joint diagnostics`,
    );
    assert.equal(snapshot.diagnostics.jointDiagnostics.length, 24);
    assert.ok(snapshot.diagnostics.jointDiagnostics.every((joint) =>
      Number.isFinite(joint.limitErrorMagnitudeRad)
      && Number.isFinite(joint.motorSaturationRatio)
      && Number.isFinite(joint.motorTorqueNm)));
    assert.ok(Number.isFinite(snapshot.diagnostics.contactDiagnostics.count));
    assert.ok(Number.isFinite(snapshot.diagnostics.contactDiagnostics.loadBearingCount));
    assert.ok(Number.isFinite(snapshot.diagnostics.contactDiagnostics.totalNormalForceN));
    assert.equal(snapshot.diagnostics.finite, true);
    assert.deepEqual(snapshot.diagnostics.errors, []);

    const character = await createEmbodiedCharacter(renderer);
    try {
      const current = character.getSnapshot(renderer);
      for (const id of SEGMENT_IDS) {
        const expected = snapshot.segments.find((segment) => segment.id === id);
        const actual = current.segments.find((segment) => segment.id === id);
        assert.ok(distance(expected.position, actual.position) <= ACCEPTANCE.trajectoryPositionToleranceM,
          `${fixture.id} ${id} position`);
        assert.ok(rotationDifferenceDegrees(expected.rotation, actual.rotation)
          <= ACCEPTANCE.trajectoryRotationToleranceDegrees, `${fixture.id} ${id} rotation`);
        assert.ok(distance(expected.linearVelocity, actual.linearVelocity)
          <= ACCEPTANCE.trajectoryVelocityTolerance, `${fixture.id} ${id} linear velocity`);
        assert.ok(distance(expected.angularVelocity, actual.angularVelocity)
          <= ACCEPTANCE.trajectoryVelocityTolerance, `${fixture.id} ${id} angular velocity`);
      }
    } finally {
      character.dispose();
    }
  }
});
