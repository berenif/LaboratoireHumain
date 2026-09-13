import assert from "node:assert/strict";
import test, { after } from "node:test";
import RAPIER from "@dimforge/rapier3d-compat";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { composeUprightPose, poseAnchor, restPoseMap } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS, SEGMENTS_BY_REGION, TOTAL_MASS_KG } = await import("../src/core/humanoid.ts");
const { raycastConvex } = await import("../src/core/geometry.ts");
const { pickRegionProxies } = await import("../src/core/picking.ts");
const { pickRegionProxies: legacyPick } = await import("../src/interaction/picking.ts");
const { jointCoordinates, jointLimitErrorMagnitude } = await import("../src/character/joint-coordinates.ts");
const { quatInverse, rotate, sub } = await import("../src/character/math.ts");

const zero = { x: 0, y: 0, z: 0 };
const identity = { x: 0, y: 0, z: 0, w: 1 };
const dt = 1 / 60;
const footCenterHeight = -SEGMENT_BY_ID.get("leftFoot").geometry.localBounds.min.y;
const recoveryMotion = (state) => ["falling", "fallen", "recovering"].includes(state);

function deepFreeze(value) {
  if (value && typeof value === "object") {
    for (const child of Object.values(value)) deepFreeze(child);
    Object.freeze(value);
  }
  return value;
}

function segment(id, position, rotation = identity) {
  return { id, position, rotation, linearVelocity: zero, angularVelocity: zero };
}

test("dispose frees each continuous dynamic world exactly once", async (t) => {
  const free = RAPIER.World.prototype.free;
  const freedWorlds = [];
  t.mock.method(RAPIER.World.prototype, "free", function () {
    freedWorlds.push(this);
    return free.call(this);
  });

  for (const ragdoll of [false, true]) {
    const character = await createEmbodiedCharacter();
    try {
      if (ragdoll) {
        character.seedRecoveryFixture(restPoseMap(), { x: 1, y: 0, z: 0 });
        assert.ok(recoveryMotion(character.diagnostics().state));
        assert.equal(character.diagnostics().physicsOwnership, "rapier-dynamic");
      }
      const sequence = character.getSnapshot("canvas2d").sequence;
      character.dispose();
      character.dispose();
      character.resume();
      character.reset();
      character.fixedUpdate(dt, null);
      assert.equal(character.getSnapshot("canvas2d").sequence, sequence);
      assert.equal(character.pick({ origin: { x: 0, y: 1, z: 5 }, direction: { x: 0, y: 0, z: -1 } }), null);
    } finally {
      character.dispose();
    }
  }
  assert.equal(freedWorlds.length, 2);
  assert.equal(new Set(freedWorlds).size, 2);
});

test("published snapshots cannot mutate the character's retained pose", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const expected = character.getSnapshot("canvas2d");
    const exposed = character.getSnapshot("canvas2d");
    exposed.rootPosition.x = 100;
    exposed.segments[0].position.y = -100;
    exposed.segments[0].rotation.w = 0;
    exposed.segments[0].linearVelocity.z = 100;
    assert.deepEqual(character.getSnapshot("canvas2d"), expected);
  } finally {
    character.dispose();
  }
});

test("diagnostics expose continuous ownership, joint limits, motor saturation and contacts", async () => {
  const character = await createEmbodiedCharacter();
  try {
    character.fixedUpdate(dt, null);
    const diagnostics = character.diagnostics();
    assert.equal(diagnostics.physicsOwnership, "rapier-dynamic");
    assert.equal(diagnostics.jointDiagnostics.length, SEGMENTS.length - 1);
    assert.ok(Number.isFinite(diagnostics.maxJointLimitErrorRad));
    assert.ok(Number.isFinite(diagnostics.maxMotorSaturationRatio));
    assert.deepEqual(diagnostics.contactDiagnostics.supportingSegments,
      [...new Set(diagnostics.contactDiagnostics.supportingSegments)]);
    for (const value of [
      diagnostics.contactDiagnostics.count,
      diagnostics.contactDiagnostics.loadBearingCount,
      diagnostics.contactDiagnostics.totalNormalForceN,
    ]) assert.ok(Number.isFinite(value) && value >= 0);
    for (const joint of diagnostics.jointDiagnostics) {
      assert.ok(SEGMENT_BY_ID.has(joint.segment));
      assert.ok(Object.values(joint.coordinates).every(Number.isFinite));
      assert.ok(Object.values(joint.targetCoordinates).every(Number.isFinite));
      assert.ok(Object.values(joint.limitError).every(Number.isFinite));
      assert.ok(Number.isFinite(joint.limitErrorMagnitudeRad) && joint.limitErrorMagnitudeRad >= 0);
      assert.ok(Number.isFinite(joint.motorSaturationRatio) && joint.motorSaturationRatio >= 0);
    }
  } finally {
    character.dispose();
  }
});

test("Rapier receives the self-collision filter during every simulated step", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const original = character.physicsHooks.filterContactPair;
    let calls = 0;
    character.physicsHooks.filterContactPair = (...args) => {
      calls += 1;
      return original(...args);
    };
    character.fixedUpdate(dt, null);
    assert.ok(calls > 0, "Rapier 0.20 bypasses hooks when World.step is given no EventQueue");
  } finally {
    character.dispose();
  }
});

test("initial dynamic settling remains bounded and creates no velocity spike", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const before = character.getSnapshot("canvas2d");
    const beforePelvis = before.segments.find(({ id }) => id === "pelvis");
    assert.equal(character.ragdollBodies.size, SEGMENTS.length);
    assert.equal(character.ragdollColliders.size, SEGMENTS.length);
    assert.ok([...character.ragdollBodies.values()].every((body) => body.isDynamic()));
    assert.ok([...character.ragdollBodies.values()].every((body) => body.isSleeping()),
      "the initialized equilibrium may sleep without changing dynamic ownership");
    assert.equal(character.diagnostics().physicsOwnership, "rapier-dynamic");
    let maximumPelvisDrop = 0;
    let maximumSpeed = 0;
    for (let tick = 0; tick < 12; tick++) {
      character.fixedUpdate(dt, null);
      const after = character.getSnapshot("canvas2d");
      const afterPelvis = after.segments.find(({ id }) => id === "pelvis");
      maximumPelvisDrop = Math.max(maximumPelvisDrop, beforePelvis.position.y - afterPelvis.position.y);
      maximumSpeed = Math.max(maximumSpeed, ...after.segments.map(({ linearVelocity }) => Math.hypot(
        linearVelocity.x,
        linearVelocity.y,
        linearVelocity.z,
      )));
    }
    assert.ok([...character.ragdollBodies].every(([, body]) => body.isSleeping()),
      "woken after idle steps: " + [...character.ragdollBodies]
        .filter(([, body]) => !body.isSleeping()).map(([id]) => id).join(", "));
    assert.ok(maximumPelvisDrop < 0.01 && maximumSpeed < 0.5,
      `initial-settling pelvis drop: ${maximumPelvisDrop} m; maximum segment speed: ${maximumSpeed} m/s`);
  } finally {
    character.dispose();
  }
});

test("fall and recovery motion reject body input and require a fresh press", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const hand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 81, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: hand.position, timestampMs: 0,
    });
    // The expanded dynamic chain can complete a bounded protective step before
    // the overpowering pull exhausts support. Wait through that physical
    // response instead of treating the old two-second 16-segment deadline as
    // part of the input-lockout contract this test is meant to verify.
    for (let tick = 1; tick <= 180 && !recoveryMotion(character.diagnostics().state); tick++) {
      character.fixedUpdate(dt, tick <= 5 ? {
        kind: "move", pointerId: 81,
        worldTarget: { x: hand.position.x + 1.25 * tick / 5, y: hand.position.y + 0.15 * tick / 5, z: hand.position.z + 0.2 * tick / 5 },
        timestampMs: tick * 1000 / 60,
      } : null);
    }
    let diagnostics = character.diagnostics();
    assert.ok(recoveryMotion(diagnostics.state));
    assert.equal(diagnostics.physicsOwnership, "rapier-dynamic");
    assert.equal(diagnostics.bodyInputAvailable, false);
    assert.equal(diagnostics.activeGrab, false);
    assert.equal(diagnostics.appliedGrabForceN, 0);

    const lockedHand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 82, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: lockedHand.position, timestampMs: 500,
    });
    diagnostics = character.diagnostics();
    assert.equal(diagnostics.activeGrab, false);
    assert.equal(diagnostics.appliedGrabForceN, 0);

    character.reset();
    diagnostics = character.diagnostics();
    assert.equal(diagnostics.physicsOwnership, "rapier-dynamic");
    assert.equal(diagnostics.state, "upright");
    assert.equal(diagnostics.bodyInputAvailable, true);
    assert.equal(diagnostics.activeGrab, false, "the rejected held press is never resumed");
    const readyHand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 83, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: readyHand.position, timestampMs: 1000,
    });
    assert.equal(character.diagnostics().activeGrab, true);
    assert.equal(character.diagnostics().selectedSegment, "rightHand");
  } finally {
    character.dispose();
  }
});

test("upright composition accepts frozen simulation inputs and keeps a complete finite pose", () => {
  const rest = restPoseMap();
  const input = deepFreeze({
    rootTranslation: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 },
    reactionOffset: { x: 0.2, y: 0.1, z: -0.1 },
    simulationTime: 0.75,
    activeGrab: {
      region: "leftHand", target: { x: -2, y: 2, z: 1 },
      startTarget: { ...rest.get("leftHand").position },
      startSegmentPosition: { ...rest.get("leftHand").position },
    },
    supportFeet: {
      leftFoot: { ...rest.get("leftFoot").position, y: footCenterHeight },
      rightFoot: { ...rest.get("rightFoot").position, y: footCenterHeight },
    },
    step: {
      foot: "rightFoot", from: { ...rest.get("rightFoot").position, y: footCenterHeight },
      to: { x: 0.4, y: footCenterHeight, z: 0.2 }, elapsed: 0.21, duration: 0.42,
    },
  });
  const before = JSON.stringify(input);
  const { poses, leanRadians } = composeUprightPose(input);
  assert.deepEqual([...poses.keys()].sort(), SEGMENTS.map(({ id }) => id).sort());
  for (const pose of poses.values()) {
    for (const value of [pose.position, pose.rotation, pose.linearVelocity, pose.angularVelocity]) {
      assert.ok(Object.values(value).every(Number.isFinite));
    }
  }
  assert.ok(leanRadians > 0);
  assert.ok(poses.get("rightFoot").position.y > input.supportFeet.rightFoot.y);
  assert.equal(JSON.stringify(input), before);
  assert.deepEqual(composeUprightPose(input).poses, poses);
});

test("adult proportions form a connected 1.84 metre rest pose on the floor", () => {
  assert.equal(SEGMENTS.length, 25);
  assert.ok(Math.abs(TOTAL_MASS_KG - 72.2) < 1e-12);
  assert.equal(HUMAN_PROPORTIONS.totalHeightM, 1.84);
  assert.equal(HUMAN_PROPORTIONS.arm.upperLengthM, 0.31);
  assert.equal(HUMAN_PROPORTIONS.arm.proximalForearmLengthM, 0.135);
  assert.equal(HUMAN_PROPORTIONS.arm.distalForearmLengthM, 0.135);
  assert.equal(HUMAN_PROPORTIONS.leg.thighLengthM, 0.42);
  assert.equal(HUMAN_PROPORTIONS.leg.shinLengthM, 0.40);
  assert.equal(HUMAN_PROPORTIONS.lumbar.halfHeightM, 0.06);
  assert.equal(HUMAN_PROPORTIONS.foot.forefootHalfLengthM, 0.06);

  const poses = restPoseMap();
  assert.equal(poses.size, 25);
  assert.equal(poses.get("pelvis").position.y, HUMAN_PROPORTIONS.pelvis.centerHeightM);
  for (const definition of SEGMENTS) {
    assert.equal(definition.geometry.kind, "convex");
    assert.ok(definition.geometry.vertices.length >= 4, `${definition.id} vertices`);
    assert.ok(definition.geometry.triangles.length >= 4, `${definition.id} triangles`);
    assert.ok(definition.geometry.triangles.every((triangle) => triangle.every((index) =>
      Number.isInteger(index) && index >= 0 && index < definition.geometry.vertices.length)), `${definition.id} indices`);
    if (!definition.parent) continue;
    assert.equal(definition.jointAnchorParent, definition.jointProfile.parentFrame.anchor);
    assert.equal(definition.jointAnchorChild, definition.jointProfile.childFrame.anchor);
    assert.ok(definition.collisionExclusions.includes(definition.parent), `${definition.id} excludes connected parent`);
    assert.ok(SEGMENT_BY_ID.get(definition.parent).collisionExclusions.includes(definition.id), `${definition.id} exclusion is symmetric`);
    const parentAnchor = poseAnchor(poses.get(definition.parent), definition.jointAnchorParent);
    const childAnchor = poseAnchor(poses.get(definition.id), definition.jointAnchorChild);
    assert.ok(Math.hypot(
      parentAnchor.x - childAnchor.x,
      parentAnchor.y - childAnchor.y,
      parentAnchor.z - childAnchor.z,
    ) < 1e-12, `${definition.id} rest joint`);
    const coordinates = jointCoordinates(poses.get(definition.parent).rotation, poses.get(definition.id).rotation, definition.jointProfile);
    assert.ok(jointLimitErrorMagnitude(coordinates, definition.jointProfile) < 1e-12, `${definition.id} rest profile`);
  }

  const worldVertices = SEGMENTS.flatMap((definition) => definition.geometry.vertices.map((vertex) =>
    poseAnchor(poses.get(definition.id), vertex)));
  const bottom = Math.min(...worldVertices.map(({ y }) => y));
  const top = Math.max(...worldVertices.map(({ y }) => y));
  const stature = top - bottom;
  assert.ok(Math.abs(bottom) < 1e-12, `rest pose bottom ${bottom}`);
  assert.ok(Math.abs(stature - HUMAN_PROPORTIONS.totalHeightM) < 1e-12, `stature ${stature}`);

  for (const foot of ["leftFoot", "rightFoot", "leftForefoot", "rightForefoot"]) {
    const pose = poses.get(foot);
    const bounds = SEGMENT_BY_ID.get(foot).geometry.localBounds;
    assert.ok(Math.abs(pose.position.y + bounds.min.y) < 1e-12);
  }

  const actualSpan = (segmentId, childId) => {
    const segment = SEGMENT_BY_ID.get(segmentId);
    const child = SEGMENT_BY_ID.get(childId);
    return Math.hypot(
      segment.jointAnchorChild.x - child.jointAnchorParent.x,
      segment.jointAnchorChild.y - child.jointAnchorParent.y,
      segment.jointAnchorChild.z - child.jointAnchorParent.z,
    );
  };
  const limbSpans = [
    ["leftUpperArm", "leftForearm", HUMAN_PROPORTIONS.arm.upperLengthM],
    ["leftForearm", "leftForearmTwist", HUMAN_PROPORTIONS.arm.proximalForearmLengthM],
    ["leftForearmTwist", "leftHand", HUMAN_PROPORTIONS.arm.distalForearmLengthM],
    ["leftThigh", "leftShin", HUMAN_PROPORTIONS.leg.thighLengthM],
    ["leftShin", "leftAnkle", HUMAN_PROPORTIONS.leg.shinLengthM],
  ];
  for (const [segmentId, childId, expectedLength] of limbSpans) {
    const definition = SEGMENT_BY_ID.get(segmentId);
    assert.ok(Math.abs(actualSpan(segmentId, childId) - expectedLength) < 1e-12);
    const colliderLength = definition.geometry.localBounds.max.y - definition.geometry.localBounds.min.y;
    assert.ok(Math.abs(colliderLength - expectedLength) < 1e-12);
  }

  const torsoGeometry = SEGMENT_BY_ID.get("torso").geometry;
  const pelvisGeometry = SEGMENT_BY_ID.get("pelvis").geometry;
  assert.ok(torsoGeometry.vertices.length > 8, "torso uses a rounded procedural surface");
  assert.ok(pelvisGeometry.vertices.length > 8, "pelvis uses a rounded procedural surface");
  const height = stature;
  const ratio = (value) => value / height;
  const inRange = (value, min, max, label) =>
    assert.ok(value >= min && value <= max, `${label}: ${value}`);
  const headBounds = SEGMENT_BY_ID.get("head").geometry.localBounds;
  const torsoWidth = torsoGeometry.localBounds.max.x - torsoGeometry.localBounds.min.x;
  inRange(ratio(headBounds.max.y - headBounds.min.y), 0.12, 0.13, "head-to-height ratio");
  inRange(ratio(torsoWidth), 0.21, 0.27, "ribcage-width ratio");
  inRange(ratio(actualSpan("leftThigh", "leftShin") + actualSpan("leftShin", "leftAnkle")), 0.42, 0.47, "leg-length ratio");
  const leftFootPoints = ["leftFoot", "leftForefoot"].flatMap((id) => {
    const pose = poses.get(id);
    return SEGMENT_BY_ID.get(id).geometry.vertices.map((vertex) => poseAnchor(pose, vertex));
  });
  const footLength = Math.max(...leftFootPoints.map(({ z }) => z)) - Math.min(...leftFootPoints.map(({ z }) => z));
  inRange(ratio(footLength), 0.13, 0.16, "foot-length ratio");
  assert.ok(pelvisGeometry.localBounds.max.x < torsoGeometry.localBounds.max.x);

  const axis = (segmentId, coordinate) => SEGMENT_BY_ID.get(segmentId).jointProfile.axes.find((entry) => entry.coordinate === coordinate);
  assert.deepEqual(SEGMENT_BY_ID.get("leftForearm").jointProfile.axes.map(({ coordinate }) => coordinate), ["x"]);
  assert.deepEqual(SEGMENT_BY_ID.get("rightForearm").jointProfile.axes.map(({ coordinate }) => coordinate), ["x"]);
  assert.ok(Math.abs(axis("leftForearm", "x").minRadians) < 1e-12);
  assert.ok(Math.abs(axis("leftForearm", "x").maxRadians - 145 * Math.PI / 180) < 1e-12);
  assert.deepEqual(SEGMENT_BY_ID.get("leftShin").jointProfile.axes.map(({ coordinate }) => coordinate), ["x"]);
  assert.ok(Math.abs(axis("leftShin", "x").maxRadians - 140 * Math.PI / 180) < 1e-12);
  assert.deepEqual(SEGMENT_BY_ID.get("leftForearmTwist").jointProfile.axes.map(({ coordinate }) => coordinate), ["y"]);
  assert.ok(Math.abs(axis("leftForearmTwist", "y").minRadians + 80 * Math.PI / 180) < 1e-12);
  assert.ok(Math.abs(axis("leftForearmTwist", "y").maxRadians - 80 * Math.PI / 180) < 1e-12);
  assert.ok(Math.abs(axis("leftAnkle", "x").minRadians + 45 * Math.PI / 180) < 1e-12);
  assert.ok(Math.abs(axis("leftAnkle", "x").maxRadians - 20 * Math.PI / 180) < 1e-12);
  assert.deepEqual(SEGMENTS_BY_REGION.get("leftFoot"), ["leftAnkle", "leftFoot", "leftForefoot"]);
  assert.ok(!SEGMENT_BY_ID.get("head").collisionExclusions.includes("pelvis"), "nonadjacent parts retain self-collision");
});

test("shared picking keeps nearest rotated surfaces and the original entrypoint", () => {
  assert.equal(legacyPick, pickRegionProxies);
  const quarterTurn = { x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2 };
  const poses = [
    segment("head", { x: 0, y: 0, z: -1 }),
    segment("pelvis", zero, quarterTurn),
  ];
  const hit = pickRegionProxies({ origin: { x: 0, y: 0, z: 3 }, direction: { x: 0, y: 0, z: -2 } }, poses);
  assert.equal(hit.region, "pelvis");
  assert.equal(hit.segment, "pelvis");
  const localOrigin = rotate(quatInverse(quarterTurn), { x: 0, y: 0, z: 3 });
  const localDirection = rotate(quatInverse(quarterTurn), { x: 0, y: 0, z: -1 });
  const expectedDistance = raycastConvex(SEGMENT_BY_ID.get("pelvis").geometry, localOrigin, localDirection);
  assert.ok(Math.abs(hit.distance - expectedDistance) < 1e-12);
  assert.ok(Math.hypot(...Object.values(sub(hit.worldPoint, rotate(quarterTurn, hit.localAnchor)))) < 1e-12);
  const forefootHit = pickRegionProxies(
    { origin: { x: 0, y: 0, z: 1 }, direction: { x: 0, y: 0, z: -1 } },
    [segment("leftForefoot", zero)],
  );
  assert.equal(forefootHit.region, "leftFoot");
  assert.equal(forefootHit.segment, "leftForefoot", "grouped region retains exact picked segment");
  assert.equal(pickRegionProxies({ origin: zero, direction: zero }, poses), null);
  assert.equal(pickRegionProxies({ origin: { x: NaN, y: 0, z: 0 }, direction: { x: 0, y: 0, z: -1 } }, poses), null);
});
