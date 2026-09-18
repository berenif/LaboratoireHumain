import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register();
after(unregister);
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { SEGMENTS } = await import("../src/core/humanoid.ts");
const { worldPoint } = await import("../src/character/math.ts");
const dt = 1 / 60;
const locked = new Set(["falling", "fallen", "recovering"]);
const torsoRoles = new Set(["pelvis", "lumbar", "ribcage"]);
const limbRoles = new Set(["upper-arm", "forearm", "hand", "thigh", "shin", "ankle", "hindfoot", "forefoot"]);

function verifyOwnedPose(character, snapshot, originals) {
  assert.equal(snapshot.diagnostics.physicsOwnership, "rapier-dynamic");
  assert.equal(snapshot.diagnostics.finite, true);
  assert.deepEqual(snapshot.diagnostics.errors, []);
  assert.ok(snapshot.diagnostics.maxJointSeparationM <= 0.08);
  assert.ok(snapshot.diagnostics.maxFloorPenetrationM <= 0.08);
  for (const pose of snapshot.segments) {
    const body = character.ragdollBodies.get(pose.id);
    assert.equal(body, originals.get(pose.id), `${pose.id}: dynamic body was replaced`);
    assert.ok(body.isDynamic(), `${pose.id}: non-dynamic ownership`);
    assert.deepEqual({ ...body.translation() }, pose.position, `${pose.id}: rendered position differs from Rapier`);
    assert.deepEqual({ ...body.rotation() }, pose.rotation, `${pose.id}: rendered rotation differs from Rapier`);
  }
}

function torsoPenetration(character) {
  let maximum = { depthM: 0, pair: null };
  for (const torso of SEGMENTS.filter(d => torsoRoles.has(d.role))) {
    for (const limb of SEGMENTS.filter(d => limbRoles.has(d.role))) {
      // Preserve existing explicit anatomical exclusions. All other pairs must collide.
      if (torso.collisionExclusions.includes(limb.id) || limb.collisionExclusions.includes(torso.id)) continue;
      const first = character.ragdollColliders.get(torso.id);
      const second = character.ragdollColliders.get(limb.id);
      assert.notEqual(character.physicsHooks.filterContactPair(first.handle, second.handle), null,
        `${torso.id}/${limb.id}: legitimate torso collision disabled`);
      const contact = first.contactCollider(second, 0);
      const depthM = Math.max(0, -(contact?.distance ?? 0));
      if (depthM > maximum.depthM) maximum = { depthM, pair: `${torso.id}/${limb.id}` };
    }
  }
  return maximum;
}

test("a five-tick strong hand pull still produces a connected Rapier-owned physical fall", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    const originals = new Map(character.ragdollBodies);
    const hand = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand");
    const localAnchor = { x: 0.025, y: 0.015, z: 0.01 };
    const start = worldPoint(hand.position, hand.rotation, localAnchor);
    character.fixedUpdate(dt, { kind: "begin", pointerId: 52, region: "rightHand", segment: "rightHand",
      localAnchor, worldTarget: start, timestampMs: 0 });
    let firstFallTick = null;
    let minimumPelvisY = Infinity;
    for (let tick = 1; tick <= 180; tick++) {
      const fraction = Math.min(1, tick / 5);
      const command = tick < 50 ? { kind: "move", pointerId: 52,
        worldTarget: { x: start.x + 1.25 * fraction, y: start.y + 0.12 * fraction, z: start.z },
        timestampMs: tick * dt * 1000 } : tick === 50 ? { kind: "end", pointerId: 52, timestampMs: tick * dt * 1000 } : null;
      character.fixedUpdate(dt, command);
      const snapshot = character.getSnapshot("canvas2d");
      verifyOwnedPose(character, snapshot, originals);
      if (locked.has(snapshot.state)) {
        firstFallTick ??= tick;
        assert.equal(snapshot.diagnostics.activeGrab, false);
        assert.equal(snapshot.diagnostics.appliedGrabForceN, 0);
      }
      minimumPelvisY = Math.min(minimumPelvisY, snapshot.rootPosition.y);
    }
    assert.notEqual(firstFallTick, null, "strong pull was prevented from overwhelming balance");
    assert.ok(minimumPelvisY < 0.55, `fall label without physical descent: ${minimumPelvisY}m`);
  } finally { character.dispose(); }
});

test("ordinary inward hand manipulation preserves nonadjacent torso collision and owned transforms", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    const originals = new Map(character.ragdollBodies);
    const start = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand").position;
    character.fixedUpdate(dt, { kind: "begin", pointerId: 53, region: "rightHand", segment: "rightHand",
      localAnchor: { x: 0, y: 0, z: 0 }, worldTarget: start, timestampMs: 0 });
    for (let tick = 1; tick <= 90; tick++) {
      character.fixedUpdate(dt, { kind: "move", pointerId: 53,
        worldTarget: { x: start.x - 0.32 * tick / 90, y: start.y + 0.10 * tick / 90, z: start.z },
        timestampMs: tick * dt * 1000 });
      verifyOwnedPose(character, character.getSnapshot("canvas2d"), originals);
      const penetration = torsoPenetration(character);
      assert.ok(penetration.depthM <= 0.015,
        `${penetration.pair}: ${penetration.depthM}m torso penetration at tick ${tick}`);
    }
  } finally { character.dispose(); }
});
