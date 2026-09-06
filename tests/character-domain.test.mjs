import assert from "node:assert/strict";
import test, { after } from "node:test";
import RAPIER from "@dimforge/rapier3d-compat";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { SEGMENTS } = await import("../src/core/humanoid.ts");
const { pickRegionProxies } = await import("../src/core/picking.ts");
const { pickRegionProxies: legacyPick } = await import("../src/interaction/picking.ts");

const zero = { x: 0, y: 0, z: 0 };
const identity = { x: 0, y: 0, z: 0, w: 1 };
const dt = 1 / 60;

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

test("upright composition accepts frozen simulation inputs and keeps a complete finite pose", () => {
  const rest = restPoseMap();
  const input = deepFreeze({
    rootTranslation: { x: 0, y: 1.03, z: 0 },
    reactionOffset: { x: 0.2, y: 0.1, z: -0.1 },
    simulationTime: 0.75,
    activeGrab: {
      region: "leftHand", target: { x: -2, y: 2, z: 1 },
      startTarget: { ...rest.get("leftHand").position },
      startSegmentPosition: { ...rest.get("leftHand").position },
    },
    supportFeet: {
      leftFoot: { ...rest.get("leftFoot").position, y: 0.065 },
      rightFoot: { ...rest.get("rightFoot").position, y: 0.065 },
    },
    step: {
      foot: "rightFoot", from: { ...rest.get("rightFoot").position, y: 0.065 },
      to: { x: 0.4, y: 0.065, z: 0.2 }, elapsed: 0.21, duration: 0.42,
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

test("shared picking keeps nearest rotated surfaces and the original entrypoint", () => {
  assert.equal(legacyPick, pickRegionProxies);
  const quarterTurn = { x: 0, y: Math.SQRT1_2, z: 0, w: Math.SQRT1_2 };
  const poses = [
    segment("head", { x: 0, y: 0, z: -1 }),
    segment("pelvis", zero, quarterTurn),
  ];
  const hit = pickRegionProxies({ origin: { x: 0, y: 0, z: 3 }, direction: { x: 0, y: 0, z: -2 } }, poses);
  assert.equal(hit.region, "pelvis");
  assert.ok(Math.abs(hit.distance - 2.78) < 1e-12);
  assert.ok(Math.abs(hit.worldPoint.z - 0.22) < 1e-12);
  assert.ok(Math.abs(hit.localAnchor.x + 0.22) < 1e-12);
  assert.equal(pickRegionProxies({ origin: zero, direction: zero }, poses), null);
  assert.equal(pickRegionProxies({ origin: { x: NaN, y: 0, z: 0 }, direction: { x: 0, y: 0, z: -1 } }, poses), null);
});
