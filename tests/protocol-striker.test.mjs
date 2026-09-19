import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);

const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { default: RAPIER } = await import("@dimforge/rapier3d-compat");
const { PhysicsStriker } = await import("../src/character/PhysicsStriker.ts");
const { PROTOCOL_STRIKER_PIECES, STRIKER_HEAD } = await import("../src/core/protocol.ts");
const { rotate } = await import("../src/character/math.ts");
const dt = 1 / 60;

function snapshot(character, renderer = "canvas2d") {
  return character.getSnapshot(renderer);
}

function torsoSpeed(frame) {
  const velocity = frame.segments.find(({ id }) => id === "torso")?.linearVelocity;
  assert.ok(velocity, "the torso remains part of the physical assembly");
  return Math.hypot(velocity.x, velocity.y, velocity.z);
}

function step(character, frames = 1) {
  let frame;
  for (let index = 0; index < frames; index++) {
    character.fixedUpdate(dt, null);
    frame = snapshot(character);
  }
  return frame;
}

function assertFiniteTransform(striker) {
  assert.equal(typeof striker.phase, "string");
  assert.ok(striker.phase.length > 0);
  assert.ok([striker.position.x, striker.position.y, striker.position.z,
    striker.rotation.x, striker.rotation.y, striker.rotation.z, striker.rotation.w]
    .every(Number.isFinite), "the machine publishes a finite world transform");
}

async function withRoom(run) {
  const character = await createEmbodiedCharacter("canvas2d", { room: true });
  try {
    return await run(character);
  } finally {
    character.dispose();
  }
}

/** Wait for a complete mechanical attempt, retaining observations from every physics tick. */
function completeAttempt(character, initialImpactId, maxFrames = 360) {
  let impactFrame = null;
  let maxSpeed = 0;
  let unavailableFrames = 0;
  const phases = new Set();
  for (let tick = 1; tick <= maxFrames; tick++) {
    const frame = step(character);
    phases.add(frame.striker.phase);
    maxSpeed = Math.max(maxSpeed, torsoSpeed(frame));
    if (!frame.striker.available) unavailableFrames++;
    if (frame.striker.impactId !== initialImpactId && impactFrame === null) impactFrame = tick;
    if (frame.striker.available) {
      return { frame, impactFrame, maxSpeed, unavailableFrames, phases, ticks: tick };
    }
  }
  assert.fail(`the percuteur did not fully retract within ${maxFrames * dt} simulated seconds`);
}

test("protocol room exposes a separate strike control and retains 25 dynamic bodies", async () => {
  await withRoom((character) => {
    const frame = snapshot(character);
    assert.ok(frame.room, "the closed laboratory is present in its snapshot");
    assert.ok(frame.striker, "the mobile percuteur is present in its snapshot");
    assert.deepEqual(frame.protocol, { strikes: 0, recoveries: 0, message: null });
    assertFiniteTransform(frame.striker);
    assert.equal(frame.striker.available, true);
    assert.equal(frame.diagnostics.bodyInputAvailable, false, "body grabbing is disabled in this room");
    assert.equal(character.pick({ origin: { x: 0, y: 1, z: 4 }, direction: { x: 0, y: 0, z: -1 } }), null);
    assert.equal(character.ragdollBodies.size, 25);
    assert.ok([...character.ragdollBodies.values()].every((body) => body.isDynamic()));
    assert.equal(typeof character.requestStrike, "function");
  });
});

test("each click creates one contact strike, retracts, and never queues a busy click", async () => {
  await withRoom((character) => {
    const initial = snapshot(character);
    const bodies = new Map(character.ragdollBodies);
    assert.equal(character.requestStrike(), true);
    assert.equal(character.requestStrike(), false, "a second click during the active attempt is rejected");
    const first = completeAttempt(character, initial.striker.impactId);
    assert.ok(first.unavailableFrames > 0, "the machine was unavailable during motion");
    assert.notEqual(first.impactFrame, null, "the first attempt contacted the subject");
    assert.ok(first.maxSpeed > 0.02, `the torso responded physically (${first.maxSpeed} m/s)`);
    assert.equal(first.frame.protocol.strikes, first.frame.striker.impactId);
    assert.equal(first.frame.protocol.strikes, 1);
    assertFiniteTransform(first.frame.striker);
    assert.equal(first.frame.striker.available, true, "a complete retraction enables a fresh click");

    const completedImpactId = first.frame.striker.impactId;
    const afterIdle = step(character, 60);
    assert.equal(afterIdle.striker.available, true, "the rejected click did not become a delayed strike");
    assert.equal(afterIdle.striker.impactId, completedImpactId);
    assert.equal(afterIdle.protocol.strikes, 1);
    assert.equal(character.requestStrike(), true, "a new click is accepted after retraction");
    const second = completeAttempt(character, completedImpactId);
    assert.ok(second.impactFrame !== null, "the fresh click produced its own contact");
    assert.equal(second.frame.protocol.strikes, 2);
    assert.equal(character.ragdollBodies.size, bodies.size);
    for (const [id, body] of bodies) {
      assert.equal(character.ragdollBodies.get(id), body, `${id} survived both attempts`);
      assert.equal(body.isDynamic(), true);
    }
  });
});

test("pause freezes the machine and renderer changes only the published view", async () => {
  await withRoom((character) => {
    assert.equal(character.requestStrike(), true);
    const moving = step(character, 3);
    assert.equal(moving.striker.available, false);
    character.pause();
    const paused = snapshot(character);
    assert.equal(character.requestStrike(), false, "a paused request is neither fired nor queued");
    step(character, 30);
    const held = snapshot(character);
    assert.equal(held.sequence, paused.sequence);
    assert.equal(held.simulationTime, paused.simulationTime);
    assert.deepEqual(held.striker, paused.striker);
    assert.deepEqual(held.segments, paused.segments);

    const otherRenderer = snapshot(character, "webgl");
    assert.equal(otherRenderer.sequence, held.sequence);
    assert.deepEqual(otherRenderer.striker, held.striker);
    assert.deepEqual(otherRenderer.segments, held.segments);

    character.resume();
    const resumed = step(character);
    assert.equal(resumed.sequence, held.sequence + 1);
    assert.ok(resumed.simulationTime > held.simulationTime);
    const completed = completeAttempt(character, held.striker.impactId);
    assert.equal(completed.frame.striker.available, true);
    const idle = step(character, 30);
    assert.equal(idle.striker.impactId, completed.frame.striker.impactId,
      "the click rejected during pause did not fire on resume");
  });
});

test("a missed strike near a wall retracts without inventing an impact", async () => {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const striker = new PhysicsStriker(world);
  try {
    assert.equal(striker.request({ x: 4.4, y: 1.2, z: 0 }), true);
    assert.ok(striker.strikeDirection.x < -0.04,
      "the locked stroke points back into the room at the right wall");
    let completed = false;
    for (let tick = 0; tick < 180; tick++) {
      striker.beforeStep(dt);
      world.step();
      assert.equal(striker.afterStep(new Map()), false);
      const { position } = striker.snapshot();
      assert.ok(Math.abs(position.x) <= 4.64 + 1e-6 && Math.abs(position.z) <= 4.64 + 1e-6,
        `the mobile head stays clear of room walls at ${JSON.stringify(position)}`);
      const pose = striker.snapshot();
      for (const piece of PROTOCOL_STRIKER_PIECES) for (const vertex of piece.geometry.vertices) {
        const local = { x: piece.position.x + vertex.x,
          y: piece.position.y + vertex.y, z: piece.position.z + vertex.z };
        const offset = rotate(pose.rotation, local);
        assert.ok(Math.abs(pose.position.x + offset.x) <= 4.98 + 1e-6
          && Math.abs(pose.position.z + offset.z) <= 4.98 + 1e-6,
        `the complete drawn assembly stays inside the wall: ${piece.id}`);
      }
      if (striker.available) { completed = true; break; }
    }
    assert.equal(completed, true);
    assert.equal(striker.impactId, 0);
    assert.equal(striker.request({ x: 0, y: 1.2, z: 0 }), true,
      "the miss leaves no deferred command and allows a new click");
  } finally {
    striker.dispose();
    world.free();
  }
});

test("a fresh click after retraction can contact again during the measured fall", async () => {
  await withRoom((character) => {
    assert.equal(character.requestStrike(), true);
    let firstImpact = false;
    let secondRequested = false;
    let secondImpactBefore = null;
    const timeline = [];
    let previous = snapshot(character);
    for (let tick = 0; tick < 180; tick++) {
      const frame = step(character);
      if (frame.striker.phase !== previous.striker.phase || frame.state !== previous.state
        || frame.striker.impactId !== previous.striker.impactId) {
        timeline.push({ tick, state: frame.state, phase: frame.striker.phase,
          impactId: frame.striker.impactId });
      }
      if (frame.striker.impactId === 1) firstImpact = true;
      if (firstImpact && !secondRequested && frame.striker.available) {
        assert.equal(frame.state, "falling", "the device is ready before the fall ends");
        assert.equal(character.requestStrike(), true);
        secondRequested = true;
      }
      if (frame.striker.impactId === 2) {
        secondImpactBefore = previous.state;
        break;
      }
      previous = frame;
    }
    assert.equal(secondRequested, true);
    assert.equal(secondImpactBefore, "falling",
      `the second solver contact occurs during the fall: ${JSON.stringify(timeline)}`);
  });
});

test("repositioning aborts without impact when a body enters the locked start", async () => {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const blocker = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased()
    .setTranslation(3, 1.2, 0));
  const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(0.2, 0.2, 0.2), blocker);
  const striker = new PhysicsStriker(world);
  try {
    assert.equal(striker.request({ x: 0, y: 1.2, z: 0 }, new Map([["torso", collider]])), true);
    blocker.setTranslation({ x: 0, y: 1.2, z: 1.28 }, true);
    let aborted = false;
    let abortPosition = null;
    let lastAbortY = -Infinity;
    for (let tick = 0; tick < 90; tick++) {
      striker.beforeStep(dt);
      world.step();
      striker.afterStep(new Map([["torso", collider]]));
      if (striker.phase === "retracting" && striker.impactId === 0) {
        aborted = true;
        const { position } = striker.snapshot();
        abortPosition ??= position;
        if (position.y < 3.45 - 1e-6) assert.ok(Math.abs(position.x - abortPosition.x) < 1e-6
          && Math.abs(position.z - abortPosition.z) < 1e-6,
        "an aborted head rises before moving horizontally toward its central home");
        assert.ok(position.y + 1e-6 >= lastAbortY);
        lastAbortY = position.y;
      }
      if (striker.available) break;
    }
    assert.equal(aborted, true, "a new obstruction cancels the attempt before contact");
    assert.equal(striker.available, true);
    assert.equal(striker.impactId, 0);
  } finally {
    striker.dispose();
    world.free();
  }
});

test("near-wall repositioning keeps the head clear of a measured static torso", async () => {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const torso = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(4.4, 1.2, 0));
  const torsoCollider = world.createCollider(RAPIER.ColliderDesc.cuboid(0.225, 0.15, 0.11), torso);
  const striker = new PhysicsStriker(world);
  const head = new RAPIER.Cuboid(STRIKER_HEAD.x, STRIKER_HEAD.y, STRIKER_HEAD.z);
  try {
    assert.equal(striker.request({ x: 4.4, y: 1.2, z: 0 }, new Map([["torso", torsoCollider]])), true);
    for (let tick = 0; tick < 20; tick++) {
      striker.beforeStep(dt);
      world.step();
      if (striker.phase !== "positioning") break;
      const pose = striker.snapshot();
      let overlap = false;
      world.intersectionsWithShape(pose.position, pose.rotation, head,
        collider => { if (collider.handle === torsoCollider.handle) overlap = true; return !overlap; });
      assert.equal(overlap, false);
    }
    assert.equal(striker.phase, "striking");
  } finally {
    striker.dispose();
    world.free();
  }
});

test("a new obstruction at the locked start cannot trap the retraction", async () => {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const blocker = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(0, 1.2, 0).setGravityScale(0));
  const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(0.3, 0.3, 0.3), blocker);
  const striker = new PhysicsStriker(world);
  const anatomy = new Map([["torso", collider]]);
  try {
    assert.equal(striker.request({ x: 0, y: 1.2, z: 0 }, anatomy), true);
    let start = null;
    let contacted = false;
    for (let tick = 0; tick < 180; tick++) {
      striker.beforeStep(dt);
      world.step();
      if (striker.phase === "striking" && start === null) start = striker.snapshot().position;
      if (striker.afterStep(anatomy)) { contacted = true; break; }
    }
    assert.equal(contacted, true, "a real dynamic-body contact begins the retraction");
    assert.ok(start);
    blocker.setBodyType(RAPIER.RigidBodyType.Fixed, true);
    blocker.setTranslation(start, true);

    let tookEscapePath = false;
    for (let tick = 0; tick < 180 && !striker.available; tick++) {
      striker.beforeStep(dt);
      world.step();
      striker.afterStep(anatomy);
      if (striker.phase === "retracting" && !striker.collider.isEnabled()) tookEscapePath = true;
    }
    assert.equal(tookEscapePath, true, "the blocked retraction disables contact and stows above the subject");
    assert.equal(striker.available, true, "the blocker cannot leave the control permanently unavailable");
    assert.equal(striker.impactId, 1, "the obstacle does not invent a second impact");
  } finally {
    striker.dispose();
    world.free();
  }
});
