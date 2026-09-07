import assert from "node:assert/strict";
import test, { after } from "node:test";
import RAPIER from "@dimforge/rapier3d-compat";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { composeUprightPose, poseAnchor, restPoseMap } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { pickRegionProxies } = await import("../src/core/picking.ts");
const { pickRegionProxies: legacyPick } = await import("../src/interaction/picking.ts");

const zero = { x: 0, y: 0, z: 0 };
const identity = { x: 0, y: 0, z: 0, w: 1 };
const dt = 1 / 60;
const footCenterHeight = SEGMENT_BY_ID.get("leftFoot").shape.halfExtents.y;

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

test("dispose frees upright and ragdoll worlds exactly once", async (t) => {
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
        const hand = character.getSnapshot("canvas2d").segments.find((pose) => pose.id === "rightHand");
        character.fixedUpdate(dt, {
          kind: "begin", pointerId: 1, region: "rightHand", segment: "rightHand",
          localAnchor: zero, worldTarget: hand.position, timestampMs: 0,
        });
        // A brief bounded pull must build momentum before balance fails.
        for (let tick = 1; tick <= 60 && character.diagnostics().authority !== "ragdoll"; tick++) {
          character.fixedUpdate(dt, tick <= 5 ? {
            kind: "move", pointerId: 1,
            worldTarget: { x: hand.position.x + 1.25 * tick / 5, y: hand.position.y + 0.15 * tick / 5, z: hand.position.z + 0.2 * tick / 5 },
            timestampMs: tick * 1000 / 60,
          } : null);
        }
        assert.equal(character.diagnostics().authority, "ragdoll");
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

test("the first grounded update neither sinks the body nor creates a velocity spike", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const before = character.getSnapshot("canvas2d");
    const beforePelvis = before.segments.find(({ id }) => id === "pelvis");
    const rootCollider = character.rootCollider;
    const rootBodyPosition = character.rootBody.translation();
    const colliderOffset = rootCollider.translationWrtParent();
    const expectedRadius = Math.max(
      HUMAN_PROPORTIONS.torso.halfExtentsM.x,
      HUMAN_PROPORTIONS.pelvis.halfExtentsM.x,
    );
    const expectedHalfHeight = (HUMAN_PROPORTIONS.totalHeightM - 0.012) / 2 - expectedRadius;
    assert.equal(rootCollider.shapeType(), RAPIER.ShapeType.Capsule);
    assert.ok(colliderOffset);
    assert.ok(Math.abs(rootCollider.radius() - expectedRadius) < 1e-6);
    assert.ok(Math.abs(rootCollider.halfHeight() - expectedHalfHeight) < 1e-6);
    const colliderCenterY = rootBodyPosition.y + colliderOffset.y;
    assert.ok(Math.abs(colliderCenterY - (HUMAN_PROPORTIONS.totalHeightM + 0.012) / 2) < 1e-6);
    assert.ok(Math.abs(colliderCenterY - rootCollider.halfHeight() - rootCollider.radius() - 0.012) < 1e-6);
    assert.ok(Math.abs(colliderCenterY + rootCollider.halfHeight() + rootCollider.radius() - HUMAN_PROPORTIONS.totalHeightM) < 1e-6);
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
    assert.ok(maximumPelvisDrop < 0.001, `first-ticks pelvis drop: ${maximumPelvisDrop} m`);
    assert.ok(maximumSpeed < 0.5, `first-tick segment speed: ${maximumSpeed} m/s`);
  } finally {
    character.dispose();
  }
});

test("recovery settling rejects body input until balance control resumes", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const hand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 81, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: hand.position, timestampMs: 0,
    });
    for (let tick = 1; tick <= 60 && character.diagnostics().authority !== "ragdoll"; tick++) {
      character.fixedUpdate(dt, tick <= 5 ? {
        kind: "move", pointerId: 81,
        worldTarget: { x: hand.position.x + 1.25 * tick / 5, y: hand.position.y + 0.15 * tick / 5, z: hand.position.z + 0.2 * tick / 5 },
        timestampMs: tick * 1000 / 60,
      } : null);
    }
    assert.equal(character.diagnostics().authority, "ragdoll");

    for (let tick = 0; tick < 1800 && character.diagnostics().authority === "ragdoll"; tick++) {
      character.fixedUpdate(dt, null);
    }
    let diagnostics = character.diagnostics();
    assert.equal(diagnostics.authority, "character-motor");
    assert.equal(diagnostics.bodyInputAvailable, false);

    let seedTicks = 0;
    while (!character.diagnostics().bodyInputAvailable && seedTicks < 60) {
      diagnostics = character.diagnostics();
      assert.equal(diagnostics.authority, "character-motor");
      assert.equal(diagnostics.balance, null);
      assert.equal(diagnostics.activeGrab, false);
      assert.equal(diagnostics.appliedGrabForceN, 0);
      const settlingHand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
      character.fixedUpdate(dt, {
        kind: "begin", pointerId: 82, region: "rightHand", segment: "rightHand",
        localAnchor: zero, worldTarget: settlingHand.position, timestampMs: seedTicks * 1000 / 60,
      });
      seedTicks++;
    }
    assert.equal(seedTicks, 45);
    assert.equal(character.diagnostics().bodyInputAvailable, true);
    assert.equal(character.diagnostics().activeGrab, false);
    assert.equal(character.diagnostics().balance, null);
    character.fixedUpdate(dt, null);
    assert.notEqual(character.diagnostics().balance, null);
    const readyHand = character.getSnapshot("canvas2d").segments.find(({ id }) => id === "rightHand");
    character.fixedUpdate(dt, {
      kind: "begin", pointerId: 83, region: "rightHand", segment: "rightHand",
      localAnchor: zero, worldTarget: readyHand.position, timestampMs: 1000,
    });
    assert.equal(character.diagnostics().activeGrab, true);
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
  assert.deepEqual({
    totalHeightM: HUMAN_PROPORTIONS.totalHeightM,
    pelvis: HUMAN_PROPORTIONS.pelvis,
    torso: HUMAN_PROPORTIONS.torso,
    neck: HUMAN_PROPORTIONS.neck,
    head: HUMAN_PROPORTIONS.head,
    arm: {
      upperLengthM: HUMAN_PROPORTIONS.arm.upperLengthM,
      upperRadiusM: HUMAN_PROPORTIONS.arm.upperRadiusM,
      forearmLengthM: HUMAN_PROPORTIONS.arm.forearmLengthM,
      forearmRadiusM: HUMAN_PROPORTIONS.arm.forearmRadiusM,
      handHalfExtentsM: HUMAN_PROPORTIONS.arm.handHalfExtentsM,
    },
    leg: HUMAN_PROPORTIONS.leg,
    foot: {
      halfExtentsM: HUMAN_PROPORTIONS.foot.halfExtentsM,
      ankleOffsetZM: HUMAN_PROPORTIONS.foot.ankleOffsetZM,
    },
    stance: HUMAN_PROPORTIONS.stance,
  }, {
    totalHeightM: 1.84,
    pelvis: { centerHeightM: 0.99, halfExtentsM: { x: 0.17, y: 0.13, z: 0.11 }, spineAnchorYM: 0.13, hipAnchorXM: 0.09, hipAnchorYM: -0.08 },
    torso: { halfExtentsM: { x: 0.225, y: 0.22, z: 0.11 }, pelvisAnchorYM: -0.20, neckAnchorYM: 0.22, shoulderAnchorXM: 0.225, shoulderAnchorYM: 0.13 },
    neck: { radiusM: 0.05, halfHeightM: 0.015, anchorYM: 0.035 },
    head: { radiusM: 0.115 },
    arm: { upperLengthM: 0.31, upperRadiusM: 0.055, forearmLengthM: 0.27, forearmRadiusM: 0.045, handHalfExtentsM: { x: 0.045, y: 0.09, z: 0.025 } },
    leg: { thighLengthM: 0.42, thighRadiusM: 0.075, shinLengthM: 0.40, shinRadiusM: 0.06 },
    foot: { halfExtentsM: { x: 0.05, y: 0.045, z: 0.135 }, ankleOffsetZM: -0.085 },
    stance: { neutralKneeFlexion: 0.12 },
  });

  const poses = restPoseMap();
  assert.equal(poses.get("pelvis").position.y, HUMAN_PROPORTIONS.pelvis.centerHeightM);
  for (const definition of SEGMENTS) {
    if (!definition.parent) continue;
    const parentAnchor = poseAnchor(poses.get(definition.parent), definition.jointAnchorParent);
    const childAnchor = poseAnchor(poses.get(definition.id), definition.jointAnchorChild);
    assert.ok(Math.hypot(
      parentAnchor.x - childAnchor.x,
      parentAnchor.y - childAnchor.y,
      parentAnchor.z - childAnchor.z,
    ) < 1e-12, `${definition.id} rest joint`);
  }

  const head = poses.get("head");
  const headShape = SEGMENT_BY_ID.get("head").shape;
  assert.equal(headShape.kind, "sphere");
  const stature = head.position.y + headShape.radius;
  assert.ok(stature >= 1.82 && stature <= 1.86);
  assert.ok(Math.abs(stature - HUMAN_PROPORTIONS.totalHeightM) < 1e-12);

  for (const foot of ["leftFoot", "rightFoot"]) {
    const pose = poses.get(foot);
    const shape = SEGMENT_BY_ID.get(foot).shape;
    assert.ok(Math.abs(pose.position.y - shape.halfExtents.y) < 1e-12);
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
    ["leftForearm", "leftHand", HUMAN_PROPORTIONS.arm.forearmLengthM],
    ["leftThigh", "leftShin", HUMAN_PROPORTIONS.leg.thighLengthM],
    ["leftShin", "leftFoot", HUMAN_PROPORTIONS.leg.shinLengthM],
  ];
  for (const [segmentId, childId, expectedLength] of limbSpans) {
    const definition = SEGMENT_BY_ID.get(segmentId);
    assert.equal(definition.shape.kind, "capsule");
    const colliderLength = 2 * (definition.shape.halfHeight + definition.shape.radius);
    assert.ok(Math.abs(actualSpan(segmentId, childId) - expectedLength) < 1e-12);
    assert.ok(Math.abs(colliderLength - expectedLength) < 1e-12);
  }

  const torsoShape = SEGMENT_BY_ID.get("torso").shape;
  const pelvisShape = SEGMENT_BY_ID.get("pelvis").shape;
  const footShape = SEGMENT_BY_ID.get("leftFoot").shape;
  assert.equal(torsoShape.kind, "box");
  assert.equal(pelvisShape.kind, "box");
  assert.equal(footShape.kind, "box");
  const height = stature;
  const ratio = (value) => value / height;
  const inRange = (value, min, max, label) =>
    assert.ok(value >= min && value <= max, `${label}: ${value}`);
  inRange(ratio(headShape.radius * 2), 0.12, 0.13, "head-to-height ratio");
  inRange(ratio(torsoShape.halfExtents.x * 2), 0.22, 0.27, "shoulder-width ratio");
  inRange(ratio(actualSpan("leftThigh", "leftShin") + actualSpan("leftShin", "leftFoot")), 0.42, 0.47, "leg-length ratio");
  inRange(ratio(footShape.halfExtents.z * 2), 0.13, 0.16, "foot-length ratio");
  assert.ok(pelvisShape.halfExtents.x < torsoShape.halfExtents.x);
});

test("shared picking keeps nearest rotated surfaces and the original entrypoint", () => {
  assert.equal(legacyPick, pickRegionProxies);
  const quarterTurn = { x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2 };
  const poses = [
    segment("head", { x: 0, y: 0, z: -1 }),
    segment("pelvis", zero, quarterTurn),
  ];
  const hit = pickRegionProxies({ origin: { x: 0, y: 0, z: 3 }, direction: { x: 0, y: 0, z: -2 } }, poses);
  const pelvisHalfWidth = HUMAN_PROPORTIONS.pelvis.halfExtentsM.x;
  assert.equal(hit.region, "pelvis");
  assert.ok(Math.abs(hit.distance - (3 - pelvisHalfWidth)) < 1e-12);
  assert.ok(Math.abs(hit.worldPoint.z - pelvisHalfWidth) < 1e-12);
  assert.ok(Math.abs(hit.localAnchor.x + pelvisHalfWidth) < 1e-12);
  assert.equal(pickRegionProxies({ origin: zero, direction: zero }, poses), null);
  assert.equal(pickRegionProxies({ origin: { x: NaN, y: 0, z: 0 }, direction: { x: 0, y: 0, z: -1 } }, poses), null);
});
