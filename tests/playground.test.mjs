import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
import RAPIER from "@dimforge/rapier3d-compat";

const unregister = register();
after(unregister);
const { PhysicsPlayground } = await import("../src/character/PhysicsPlayground.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { PLAYGROUND_STATIONS } = await import("../src/core/playground.ts");
await RAPIER.init();

test("terrain raycasts find raised footholds, slopes, gaps, and a narrower extreme beam", () => {
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  try {
    const gentle = new PhysicsPlayground(world, "gentle");
    assert.equal(gentle.heightAt(0, 3.3), 0);
    assert.ok(gentle.heightAt(-5, -3) > gentle.heightAt(-5, -1));
    assert.ok(Math.abs(gentle.heightAt(0, -1.6) - 0.3) < 0.005);
    assert.ok(gentle.heightAt(0.28, -1.6) > 0.2);
    assert.ok(gentle.heightAt(-5, 1.5) > 0.1);
    assert.equal(gentle.heightAt(-5, 1.9), 0, "gap between islands is real");
    assert.ok(gentle.heightAt(5, -2.1) > 0.1);
    const easySlope = gentle.heightAt(-5, -2.1);
    gentle.dispose();
    const extreme = new PhysicsPlayground(world, "extreme");
    assert.ok(extreme.heightAt(-5, -2.1) > easySlope * 2);
    assert.equal(extreme.heightAt(0.28, -1.6), 0);
    assert.ok(Math.abs(extreme.heightAt(0, -1.6) - 0.76) < 0.005);
    extreme.dispose();
    assert.equal(world.bodies.len(), 0, "changing difficulty leaves no old obstacles");
  } finally { world.free(); }
});

test("the moving deck has a physical angular velocity and reset restores its initial surface", () => {
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  try {
    const playground = new PhysicsPlayground(world, "challenging");
    const startHeight = playground.heightAt(0.9, -5.8);
    const deck = playground.colliders.find(collider => collider.parent().isKinematic()).parent();
    for (let step = 1; step <= 240; step++) {
      playground.update(step / 60);
      world.step();
    }
    assert.ok(Math.hypot(...Object.values(deck.angvel())) > 0.03);
    assert.ok(Math.abs(playground.heightAt(0.9, -5.8) - startHeight) > 0.025);
    playground.reset();
    assert.ok(Math.abs(playground.heightAt(0.9, -5.8) - startHeight) < 0.002);
    playground.dispose();
  } finally { world.free(); }
});

test("every station spawns clear of its surface and records physical contact without nonfinite motion", async () => {
  const character = await createEmbodiedCharacter("canvas2d", { playground: { station: "flat", difficulty: "challenging" } });
  try {
    for (const station of PLAYGROUND_STATIONS) {
      character.setPlayground({ station: station.id, difficulty: "challenging" });
      assert.equal(character.getSnapshot("canvas2d").playground.station, station.id);
      assert.ok(character.diagnostics().maxFloorPenetrationM < 0.025, `${station.id}: spawn intersects terrain`);
      let loaded = false;
      let elevatedContact = false;
      for (let frame = 0; frame < 60; frame++) {
        character.fixedUpdate(1 / 60, null);
        const diagnostics = character.diagnostics();
        assert.equal(diagnostics.finite, true, station.id);
        assert.deepEqual(diagnostics.errors, [], station.id);
        loaded ||= diagnostics.contactDiagnostics.loadBearingCount > 0;
        elevatedContact ||= character.lastContacts.some(contact => contact.loadBearing && contact.point.y > 0.06);
      }
      assert.ok(loaded, `${station.id}: no loaded support detected`);
      if (!["flat", "hurdles"].includes(station.id)) assert.ok(elevatedContact, `${station.id}: raised surface is invisible to support sensing`);
    }
    character.setPlayground({ station: "wobble", difficulty: "extreme" });
    for (let frame = 0; frame < 120; frame++) character.fixedUpdate(1 / 60, null);
    character.reset();
    const reset = character.getSnapshot("webgl");
    assert.equal(reset.simulationTime, 0);
    assert.deepEqual(reset.playground, { station: "wobble", difficulty: "extreme" });
    reset.playground.station = "flat";
    assert.equal(character.getSnapshot("webgl").playground.station, "wobble");
  } finally { character.dispose(); }
});
