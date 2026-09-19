import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/EmbodiedCharacter.ts");
const { collisionAwareLimbTarget, limbTrunkClearance } = await import("../src/character/limb-collisions.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { quatFromAxisAngle, quatMultiply, rotate } = await import("../src/character/math.ts");

test("upper arms retain torso contact with clear proximal geometry", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const poses = new Map(character.getSnapshot("canvas2d").segments.map((pose) => [pose.id, pose]));
    for (const side of ["left", "right"]) {
      const upper = `${side}UpperArm`;
      assert.ok(!SEGMENT_BY_ID.get(upper).collisionExclusions.includes("torso"));
      assert.ok(limbTrunkClearance(poses, side, "arm").clearanceM >= 0.005);
      for (const heading of [Math.PI / 3, Math.PI, -Math.PI / 2]) {
        const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
        const turned = new Map([...poses].map(([id, pose]) => [id, {
          ...pose, position: rotate(yaw, pose.position), rotation: quatMultiply(yaw, pose.rotation),
        }]));
        assert.ok(Math.abs(limbTrunkClearance(turned, side, "arm").clearanceM
          - limbTrunkClearance(poses, side, "arm").clearanceM) < 1e-5);
      }
    }
    assert.equal(character.diagnostics().maxSelfPenetrationM, 0);
    assert.equal(character.diagnostics().selfPenetrationPair, null);
  } finally {
    character.dispose();
  }
});

test("collision-aware hand target takes a front route around the trunk", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const poses = new Map(character.getSnapshot("canvas2d").segments.map((pose) => [pose.id, pose]));
    const hand = poses.get("leftHand");
    const start = { x: -0.36, y: poses.get("torso").position.y, z: 0 };
    const requested = { x: 0.36, y: start.y, z: 0 };
    const solve = (target) => new Map(poses).set("leftHand", { ...hand, position: target });
    const route = collisionAwareLimbTarget({
      side: "left", limb: "arm", start, requested, parent: poses.get("torso"), solve,
    });
    assert.equal(route.blocked, true);
    assert.ok(route.target.x < -0.25 && route.target.z > 0.15, JSON.stringify(route.target));
    assert.ok(route.clearance.clearanceM >= 0.004);
    const blocked = solve(poses.get("torso").position);
    assert.ok(limbTrunkClearance(blocked, "left", "arm").clearanceM < -0.05);
  } finally {
    character.dispose();
  }
});

test("self-penetration diagnostics name the deepest enabled pair", async () => {
  const character = await createEmbodiedCharacter();
  try {
    const hand = character.ragdollBodies.get("leftHand");
    hand.setTranslation(character.ragdollBodies.get("torso").translation(), true);
    character.world.propagateModifiedBodyPositionsToColliders();
    const diagnostics = character.diagnostics();
    assert.ok(diagnostics.maxSelfPenetrationM > 0.05);
    assert.ok(diagnostics.selfPenetrationPair?.includes("leftHand"));
    assert.ok(diagnostics.selfPenetrationPair?.includes("torso"));
  } finally {
    character.dispose();
  }
});
