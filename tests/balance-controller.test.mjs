import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register();
after(unregister);
const { BalanceController, BALANCE_LIMITS, massState } = await import("../src/character/BalanceController.ts");
const { composeUprightPose, restPoseMap, poseAnchor } = await import("../src/character/pose.ts");
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID, SEGMENTS } = await import("../src/core/humanoid.ts");
const { legTargetReach } = await import("../src/character/leg-target-frame.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { coordinatedStandingOptions } = await import("../src/character/standing-selection.ts");
const candidateOptions = () => process.env.STANDING_CANDIDATE ? coordinatedStandingOptions(process.env.STANDING_CANDIDATE) : {};
const { quatFromAxisAngle, quatMultiply, rotate, sub, length, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;
const footCenterHeight = -SEGMENT_BY_ID.get("leftFoot").geometry.localBounds.min.y;

test("transfer persistence accepts six qualified 60Hz frames and never five", () => {
  const poses = restPoseMap();
  const controller = new BalanceController();
  controller.reset(poses);
  const from = poses.get("rightFoot").position;
  const plan = { feasible: true, to: { ...from }, requested: { ...from },
    duration: 0.48, captureMarginM: 0.02, travelM: 0, targetErrorM: 0 };
  // Isolate the measured duration gate from the separate landing predictor.
  controller.planStep = () => plan;
  controller.beginStep("rightFoot", from, poses.get("pelvis").position,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero);
  const contacts = ["leftFoot", "rightFoot"].map((segment, index) => ({
    segment, point: { ...poses.get(segment).position }, normalY: 1,
    forceN: 72.2 * 9.81 * (index ? 0.2 : 0.8), persistenceS: 1, loadBearing: true,
  }));
  let output;
  for (let frame = 1; frame <= 6; frame++) {
    output = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
      activeGrab: null, contacts });
    assert.equal(output.step.foot, "rightFoot");
    if (frame < 6) assert.ok(output.step.elapsed < 0, `frame ${frame} has not earned 0.10s`);
  }
  assert.equal(output.step.elapsed, 0);
  assert.equal(output.step.phase, "swing");
});

for (const movingFoot of ["leftFoot", "rightFoot"]) {
  test(`a dynamically accepted ${movingFoot} step keeps its foot and world target through launch`, () => {
    const poses = restPoseMap();
    const controller = new BalanceController();
    controller.reset(poses);
    const from = poses.get(movingFoot).position;
    const to = { ...from, z: from.z - 0.08 };
    const committed = { feasible: true, to, requested: { ...to }, duration: 0.48,
      captureMarginM: 0.02, travelM: 0.08, targetErrorM: 0,
      forecast: { capturePoint: { ...to }, horizonS: 0.5, pressureFeasible: true } };
    controller.beginStep(movingFoot, from, poses.get("pelvis").position,
      { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0, undefined,
      committed, "capture");
    // The post-admission state has worsened. It remains useful telemetry, but
    // must not revoke the already validated touchdown while load transfer is
    // completing.
    controller.planStep = () => ({ feasible: false, reason: "capture-support" });
    const retainedFoot = movingFoot === "leftFoot" ? "rightFoot" : "leftFoot";
    const weight = 72.2 * 9.81;
    const contacts = [retainedFoot, movingFoot].map((segment, index) => ({
      segment, point: { ...poses.get(segment).position }, normalY: 1,
      forceN: weight * (index ? 0.2 : 0.8), persistenceS: 1, loadBearing: true,
    }));
    let output;
    for (let frame = 1; frame <= 6; frame++) {
      output = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
        activeGrab: null, contacts });
      assert.equal(output.step?.foot, movingFoot);
      assert.deepEqual(output.step?.to, { ...to, y: footCenterHeight });
      if (frame < 6) assert.ok(output.step.elapsed < 0);
    }
    assert.equal(output.step.elapsed, 0);
    assert.equal(output.step.phase, "swing");
    assert.equal(output.diagnostics.candidateValidity.reason, "capture-support");
  });
}

test("an expired transfer replacement starts its own age and readiness history", () => {
  for (const expiredFoot of ["leftFoot", "rightFoot"]) {
    const poses = restPoseMap(), controller = new BalanceController();
    controller.reset(poses);
    const replacementFoot = expiredFoot === "leftFoot" ? "rightFoot" : "leftFoot";
    const plans = Object.fromEntries(["leftFoot", "rightFoot"].map(foot => {
      const from = poses.get(foot).position;
      return [foot, { feasible: true, to: { ...from }, requested: { ...from },
        duration: 0.48, captureMarginM: 0.02, travelM: 0, targetErrorM: 0 }];
    }));
    // Isolate replacement lifecycle from the separately tested landing geometry.
    // Only the original, expired candidate is rejected; its replacement is valid.
    controller.planStep = foot => foot === expiredFoot && controller.stepTransferAge >= 1.2
      ? { feasible: false, reason: "transfer:measured-timeout" } : plans[foot];
    controller.beginStep(expiredFoot, poses.get(expiredFoot).position, poses.get("pelvis").position,
      { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0, undefined, plans[expiredFoot]);
    controller.stepTransferAge = 1.2 - dt;
    controller.transferReadyAge = 0.05;
    const contacts = ["leftFoot", "rightFoot"].map(segment => ({
      segment, point: { ...poses.get(segment).position }, normalY: 1,
      forceN: 72.2 * 9.81 * (segment === expiredFoot ? 0.2 : 0.8),
      persistenceS: 1, loadBearing: true,
    }));
    const input = { dt, poses, rootPosition: poses.get("pelvis").position, activeGrab: null, contacts };
    const replaced = controller.update(input);
    assert.equal(replaced.step?.foot, replacementFoot);
    assert.ok(replaced.step.elapsed < 0, "replacement must earn its own measured readiness");
    assert.equal(controller.stepTransferAge, 0, "expired candidate age cannot belong to the replacement");
    assert.equal(controller.transferReadyAge, 0, "the opposite retained sole has no inherited readiness");
    const continued = controller.update(input);
    assert.equal(continued.step?.foot, replacementFoot, "a fresh replacement must not immediately expire");
    assert.equal(controller.stepTransferAge, dt);
    assert.equal(controller.transferReadyAge, 0, "the replacement is still loaded on its moving foot");
  }
});

test("a transfer follows the measured support when its intended retained sole disappears", () => {
  const poses = restPoseMap(), controller = new BalanceController();
  controller.reset(poses);
  const plan = foot => {
    const from = poses.get(foot).position;
    return { feasible: true, to: { ...from }, requested: { ...from }, duration: 0.48,
      captureMarginM: 0.02, landingCaptureMarginM: 0.02, travelM: 0, targetErrorM: 0,
      forecast: { capturePoint: { ...from }, horizonS: 0.5, pressureFeasible: true } };
  };
  controller.beginStep("leftFoot", poses.get("leftFoot").position, poses.get("pelvis").position,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0, undefined,
    plan("leftFoot"), "capture");
  controller.planStep = foot => foot === "leftFoot"
    ? { feasible: false, reason: "capture:no-retained-support" } : plan(foot);
  const left = poses.get("leftFoot").position;
  const input = { dt, poses, rootPosition: poses.get("pelvis").position,
    activeGrab: null, contacts: [{ segment: "leftFoot", point: { ...left }, normalY: 1,
      forceN: 72.2 * 9.81, persistenceS: 1, loadBearing: true }] };
  for (let frame = 1; frame < 6; frame++) {
    const waiting = controller.update(input);
    assert.equal(waiting.step?.foot, "leftFoot", `frame ${frame}: transient contact loss cannot switch ownership`);
  }
  const output = controller.update(input);
  assert.equal(output.step?.foot, "rightFoot", "the newly moving foot must preserve the loaded left sole");
  assert.ok(output.step.elapsed < 0);
  assert.equal(output.diagnostics.transferAgeS, 0);
  assert.equal(output.diagnostics.transferReadyAgeS, 0);
});

function poseInput(heading = 0) {
  const rest = restPoseMap(), yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  return { rootTranslation: { x: 0, y: HUMAN_PROPORTIONS.pelvis.centerHeightM, z: 0 }, reactionOffset: zero,
    simulationTime: 0, activeGrab: null, step: null, heading,
    kneeFlexion: HUMAN_PROPORTIONS.stance.neutralKneeFlexion,
    supportFeet: Object.fromEntries(["leftFoot", "rightFoot"].map(foot => [foot, rotate(yaw, { ...rest.get(foot).position, y: footCenterHeight })])),
  };
}

function assertConnected(poses) {
  for (const definition of SEGMENTS) {
    if (!definition.parent) continue;
    const separation = length(sub(poseAnchor(poses.get(definition.parent), definition.jointAnchorParent), poseAnchor(poses.get(definition.id), definition.jointAnchorChild)));
    assert.ok(separation < 1e-8, `${definition.id}: joint separation ${separation}`);
  }
}

test("solved ankles stay connected for unreachable grabs and steps at rotated headings", () => {
  for (const heading of [0, Math.PI / 2, -0.7, 2.2]) {
    const base = poseInput(heading);
    const rest = composeUprightPose(base).poses;
    for (const side of ["left", "right"]) {
      const foot = `${side}Foot`;
      const start = rest.get(foot).position;
      const pose = composeUprightPose({ ...base, kneeFlexion: 0.65, reactionOffset: { x: 0.5, y: 0, z: -0.3 },
        activeGrab: { region: foot, startTarget: start, startSegmentPosition: start, target: { x: start.x + 4, y: start.y + 0.8, z: start.z - 2 } },
      }).poses;
      assertConnected(pose);
      const stepping = composeUprightPose({ ...base, step: { foot, from: start, to: { x: 3, y: footCenterHeight, z: -2 }, elapsed: 0.30, duration: 0.34 } }).poses;
      assertConnected(stepping);
    }
  }
});

test("heading rotates every composed segment and its joint geometry consistently", () => {
  const heading = 1.1, yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
  const base = composeUprightPose({ ...poseInput(), kneeFlexion: 0.4 }).poses;
  const turned = composeUprightPose({ ...poseInput(heading), kneeFlexion: 0.4 }).poses;
  for (const [id, pose] of base) {
    assert.ok(length(sub(turned.get(id).position, rotate(yaw, pose.position))) < 1e-8, `${id} position`);
    const expected = quatMultiply(yaw, pose.rotation), actual = turned.get(id).rotation;
    assert.ok(Math.abs(expected.x * actual.x + expected.y * actual.y + expected.z * actual.z + expected.w * actual.w) > 1 - 1e-8, `${id} rotation`);
  }
});

test("balance mass estimate includes segment masses and rejects a manipulated near-floor foot", () => {
  const input = poseInput(), poses = composeUprightPose(input).poses;
  for (const pose of poses.values()) pose.linearVelocity = { x: 0.4, y: -0.2, z: 0.1 };
  const mass = massState(poses);
  assert.ok(length(sub(mass.velocity, { x: 0.4, y: -0.2, z: 0.1 })) < 1e-12);
  const controller = new BalanceController(); controller.reset(poses);
  const start = poses.get("rightFoot").position;
  const left = poses.get("leftFoot").position;
  const contacts = [{ segment: "leftFoot", normalY: 1, forceN: 400, persistenceS: 1,
    point: { x: left.x, y: 0, z: left.z },
    points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({ x: left.x + sx * .05, y: 0, z: left.z + sz * .1 }))),
    loadBearing: true }];
  const output = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
    activeGrab: { region: "rightFoot", target: { ...start, x: start.x + 0.1 }, startTarget: start, startSegmentPosition: start },
    contacts,
  });
  assert.deepEqual(output.diagnostics.supportingFeet, ["leftFoot"]);
  assert.ok(output.appliedGrabForceN <= BALANCE_LIMITS.maxPullForceN);
});

test("a measured airborne foot starts its landing arc without a second unload gate", () => {
  const poses = composeUprightPose(poseInput()).poses, controller = new BalanceController();
  controller.reset(poses);
  controller.cooldown = 0;
  const right = poses.get("rightFoot");
  right.position = { ...right.position, x: right.position.x - 0.05, y: right.position.y + 0.09 };
  const left = poses.get("leftFoot").position;
  const output = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
    activeGrab: null, contacts: [{ segment: "leftFoot", point: { ...left }, normalY: 1,
      forceN: 72.2 * 9.81, persistenceS: 1, loadBearing: true }] });
  assert.equal(output.diagnostics.triggerReason, "airborne-foot");
  assert.equal(output.step?.foot, "rightFoot");
  assert.equal(output.step?.elapsed, 0);
  assert.equal(output.step?.phase, "swing");
  assert.equal(output.step?.unloaded, true);
});

test("the swing target clears the floor until landing and preload cannot substitute for measured touchdown", () => {
  // Place the COM above the retained sole before supplying single-support
  // loads; a centered two-foot pose with invented left-only loads is unready.
  const poses = composeUprightPose({ ...poseInput(),
    rootTranslation: { x: -0.09, y: 0.96, z: 0 } }).poses, controller = new BalanceController();
  controller.reset(poses);
  const pelvis = poses.get("pelvis").position;
  controller.beginStep("rightFoot", poses.get("rightFoot").position, pelvis,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  const left = poses.get("leftFoot").position;
  const contacts = [{ segment: "leftFoot", normalY: 1, forceN: 600,
    persistenceS: 1, loadBearing: true, point: { ...left, y: 0 },
    points: [-1, 1].flatMap(x => [-1, 1].map(z => ({ x: left.x + .05 * x, y: 0, z: left.z + .1 * z }))) }];
  let landedTargetSeen = false;
  for (let frame = 0; frame < 70; frame++) {
    const output = controller.update({ dt, poses, rootPosition: pelvis, activeGrab: null, contacts });
    assert.ok(output.step, "an unloaded swing foot cannot complete a step");
    const landing = output.step.elapsed >= output.step.duration;
    assert.ok(Math.abs(output.step.to.y - (footCenterHeight - (landing ? .005 : 0))) < 1e-12);
    landedTargetSeen ||= landing;
  }
  assert.ok(landedTargetSeen);
});

test("a retained load majority cannot authorize an unreachable touchdown capture", () => {
  const poses = composeUprightPose(poseInput()).poses, controller = new BalanceController();
  // The outward momentum makes touchdown unreachable; load alone is insufficient.
  for (const pose of poses.values()) pose.linearVelocity = { x: 1.2, y: 0, z: 0 };
  controller.reset(poses);
  const pelvis = poses.get("pelvis").position;
  controller.beginStep("rightFoot", poses.get("rightFoot").position, pelvis,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0, zero, 0);
  const left = poses.get("leftFoot").position;
  const contacts = [{ segment: "leftFoot", normalY: 1, forceN: 600,
    persistenceS: 1, loadBearing: true, point: { ...left, y: 0 },
    points: [-1, 1].flatMap(x => [-1, 1].map(z => ({ x: left.x + .05 * x, y: 0, z: left.z + .1 * z }))) }];
  for (let frame = 0; frame < 20; frame++) {
    const output = controller.update({ dt, poses, rootPosition: pelvis, activeGrab: null, contacts });
    assert.ok(!output.step || output.step.elapsed < 0, "load majority alone must not lift");
    assert.equal(output.stepCount, 0);
  }
});

test("active step retains its committed world target when the body heading changes", () => {
  const input = poseInput(), poses = composeUprightPose(input).poses;
  const controller = new BalanceController();
  controller.reset(poses, 0);
  const foot = "rightFoot";
  const from = poses.get(foot).position;
  controller.beginStep(
    foot,
    from,
    poses.get("pelvis").position,
    { x: 0, y: 0, z: 1 },
    { x: 1, y: 0, z: 0 },
    0,
    { x: 0.24, y: 0, z: 0.08 },
    0,
  );
  const before = controller.update({
    dt, poses, rootPosition: poses.get("pelvis").position, activeGrab: null, heading: 0,
  }).step;
  assert.ok(before);
  const turn = Math.PI / 3;
  const after = controller.update({
    dt, poses, rootPosition: poses.get("pelvis").position, activeGrab: null, heading: turn,
  }).step;
  assert.ok(after);
  assert.deepEqual(after.from, before.from);
  assert.deepEqual(after.to, before.to);
  assert.deepEqual(after.requested, before.requested);
  assert.equal(after.heading, 0);
});

test("the swing leg lands toward the disturbance when the loaded foot must stay planted", () => {
  const input = poseInput(), poses = composeUprightPose(input).poses;
  const controller = new BalanceController();
  controller.reset(poses, 0);
  const from = poses.get("leftFoot").position;
  const physicalRoot = { ...input.rootTranslation, y: input.rootTranslation.y - 0.015 };
  controller.beginStep(
    "leftFoot",
    from,
    physicalRoot,
    { x: 0, y: 0, z: 1 },
    { x: 1, y: 0, z: 0 },
    0,
    { x: 0.24, y: 0, z: 0 },
    0,
  );
  const step = controller.update({
    dt, poses, rootPosition: poses.get("pelvis").position, activeGrab: null,
  }).step;
  assert.ok(step);
  assert.ok(step.to.x > from.x + 0.05,
    `swing target ${step.to.x} must move toward the positive-X disturbance from ${from.x}`);
  const hip = poseAnchor({ ...poses.get("pelvis"), position: physicalRoot },
    SEGMENT_BY_ID.get("leftThigh").jointAnchorParent);
  const reach = legTargetReach("left", hip, step.to, quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0));
  assert.ok(reach.distanceM <= reach.radiusM - 0.007,
    `swing target radial reach ${reach.distanceM}m exceeds the anatomical margin`);
});

test("step planning finds a reachable rear landing off the starting-foot ray", () => {
  const controller = new BalanceController();
  controller.reset(composeUprightPose(poseInput()).poses);
  const from = { x: 0.0902, y: footCenterHeight, z: 0.0352 };
  const root = { x: -0.0713, y: 0.9729, z: 0.0014 };
  const pelvisRotation = { x: -0.0631741732, y: -0.0109345969,
    z: 0.0215290785, w: 0.9977103472 };
  const plan = controller.planStep("rightFoot", from, root,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0,
    { x: 0.20, y: 0, z: -0.28 }, 0, pelvisRotation, 0, 0.02);
  assert.equal(plan.feasible, true, plan.reason);
  const hip = poseAnchor({ position: root, rotation: pelvisRotation },
    SEGMENT_BY_ID.get("rightThigh").jointAnchorParent);
  const reach = legTargetReach("right", hip, plan.to,
    quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0));
  assert.ok(plan.requested.z < -0.15);
  assert.ok(plan.to.z < 0.005, `reachable landing should move behind the starting heel: ${plan.to.z}`);
  assert.ok(reach.distanceM <= reach.radiusM - 0.007,
    `landing reach ${reach.distanceM} exceeds ${reach.radiusM - 0.007}`);
  assert.ok(length(sub({ ...plan.to, y: 0 }, { ...from, y: 0 })) <= BALANCE_LIMITS.maxStepTravelM + 1e-8);
});

test("a follow-up landing stays on the swing foot side of measured stance", () => {
  const controller = new BalanceController();
  controller.stepCount = 1;
  controller.feet.rightFoot = { x: 0.045, y: footCenterHeight, z: -0.02 };
  const from = { x: -0.086, y: footCenterHeight, z: 0.04 };
  const root = { x: 0.017, y: 0.945, z: -0.048 };
  const plan = controller.planStep("leftFoot", from, root,
    { x: 0, y: 0, z: 1 }, { x: 1, y: 0, z: 0 }, 0,
    { x: 0.35, y: 0, z: -0.05 }, 0);
  assert.ok(plan.to.x <= controller.feet.rightFoot.x - 0.10 + 1e-8,
    `left landing ${plan.to.x} crossed the right sole at ${controller.feet.rightFoot.x}`);
  assert.ok(length(sub({ ...plan.to, y: 0 }, { ...from, y: 0 })) <= BALANCE_LIMITS.maxStepTravelM + 1e-8);
});

test("outward momentum can request correction after release regardless of completed step count", () => {
  for (const completed of [0, 1, 2]) {
    const poses = composeUprightPose({ ...poseInput(),
      rootTranslation: { x: 0.06, y: 0.93, z: 0 } }).poses;
    for (const pose of poses.values()) pose.linearVelocity = { x: 0.12, y: 0, z: 0 };
    const controller = new BalanceController();
    controller.reset(poses);
    controller.stepCount = completed;
    controller.cooldown = 0;
    // Isolate the trigger from landing search; its measured speed is below
    // the later .2m/s trigger, and there is no active grab or reach command.
    controller.planStep = (_foot, from) => ({ feasible: true, to: { ...from }, requested: { ...from },
      duration: 0.48, captureMarginM: 0.02, travelM: 0, targetErrorM: 0 });
    const contacts = ["leftFoot", "rightFoot"].map(segment => {
      const p = poses.get(segment).position;
      return { segment, forceN: 350, normalY: 1, loadBearing: true, persistenceS: 1,
        point: { ...p, y: 0 }, points: [-1, 1].flatMap(x => [-1, 1].map(z =>
          ({ x: p.x + .05 * x, y: 0, z: p.z + .1 * z }))) };
    });
    const output = controller.update({ dt, poses, rootPosition: poses.get("pelvis").position,
      activeGrab: null, contacts });
    assert.ok(output.step, `completed ${completed}: outward capture must request a correction`);
    assert.ok(output.step.elapsed < 0, "requesting correction must still wait for measured transfer");
    assert.equal(output.diagnostics.triggerReason, "capture");
  }
});

test("slow pulls stay connected and stepping remains available after release and reversal", async () => {
  for (const heading of [0, 1.1, -1.7]) {
    const character = await createEmbodiedCharacter("canvas2d", { heading, ...candidateOptions() });
    if(process.env.STANDING_PREDICTION_VALIDATED === '1')character.balance.forecastValidated = true;
    const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
    try {
      const start = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand").position;
      character.fixedUpdate(dt, { kind: "begin", pointerId: 3, region: "rightHand", segment: "rightHand", localAnchor: zero, worldTarget: start, timestampMs: 0 });

      for (let tick = 1; tick <= 510; tick += 1) {
        let command = null;
        if (tick <= 90) {
          const delta = rotate(yaw, { x: 0.60 * tick / 90, y: 0, z: 0.08 * tick / 90 });
          command = { kind: "move", pointerId: 3, worldTarget: { x: start.x + delta.x, y: start.y, z: start.z + delta.z }, timestampMs: tick * dt * 1000 };
        } else if (tick <= 180) {
          const delta = rotate(yaw, { x: 0.6 - (tick - 90) * 0.9 / 90, y: 0, z: 0.08 });
          command = { kind: "move", pointerId: 3, worldTarget: { x: start.x + delta.x, y: start.y, z: start.z + delta.z }, timestampMs: tick * dt * 1000 };
        } else if (tick === 181) {
          command = { kind: "end", pointerId: 3, timestampMs: tick * dt * 1000 };
        }
        character.fixedUpdate(dt, command);
        const snapshot = character.getSnapshot("canvas2d");
      assert.ok(!["falling", "fallen", "recovering"].includes(snapshot.state),
        `unexpected ${snapshot.state} at tick ${tick}`);

        assert.equal(snapshot.diagnostics.physicsOwnership, "rapier-dynamic", `heading ${heading}, tick ${tick}`);
        assert.equal(
          snapshot.diagnostics.bodyInputAvailable,
          !["falling", "fallen", "recovering"].includes(snapshot.state),
          `heading ${heading}, tick ${tick}`,
        );
        assert.ok(
          snapshot.diagnostics.maxJointSeparationM <= 0.08,
          `heading ${heading}, tick ${tick}: joint separation ${snapshot.diagnostics.maxJointSeparationM}m`,
        );
        assert.ok(snapshot.diagnostics.maxFloorPenetrationM <= 0.08);
        assert.ok(snapshot.diagnostics.finite);
        assert.ok(snapshot.support.planted.every((foot) => foot === "leftFoot" || foot === "rightFoot"));
        assert.equal(new Set(snapshot.support.planted).size, snapshot.support.planted.length);
      }
      const done = character.getSnapshot("canvas2d");
      assert.ok(
        done.diagnostics.stepCount >= 1,
        `heading ${heading}: ${done.state}, ${done.diagnostics.stepCount} steps, ${done.diagnostics.rootDisplacementM}m root displacement`,
      );
      assert.equal(done.state, "upright", `heading ${heading}`);

    } finally { character.dispose(); }
  }
});

test("a planted reversal preserves ownership without a transient unsupported fall", async () => {
  const character = await createEmbodiedCharacter("canvas2d", candidateOptions());
  if(process.env.STANDING_PREDICTION_VALIDATED === '1')character.balance.forecastValidated = true;
  const localAnchor = { x: 0.025, y: 0.015, z: 0.01 };
  try {
    const hand = character.getSnapshot("canvas2d").segments.find(p => p.id === "rightHand");
    const start = worldPoint(hand.position, hand.rotation, localAnchor);
    character.fixedUpdate(dt, { kind: "begin", pointerId: 41, region: "rightHand", segment: "rightHand", localAnchor, worldTarget: start, timestampMs: 0 });

    for (let tick = 1; tick <= 510; tick += 1) {
      let command = null;
      if (tick < 300) {
        const x = tick <= 90 ? 0.75 * tick / 90
          : tick <= 180 ? 0.75 - 1.5 * (tick - 90) / 90 : -0.75;
        command = { kind: "move", pointerId: 41, worldTarget: { x: start.x + x, y: start.y + 0.02 * Math.min(tick, 90) / 90, z: start.z }, timestampMs: tick * dt * 1000 };
      } else if (tick === 300) {
        command = { kind: "end", pointerId: 41, timestampMs: tick * dt * 1000 };
      }
      character.fixedUpdate(dt, command);
      const snapshot = character.getSnapshot("canvas2d");
      assert.ok(!["falling", "fallen", "recovering"].includes(snapshot.state),
        `unexpected ${snapshot.state} at tick ${tick}`);
      assert.equal(snapshot.diagnostics.physicsOwnership, "rapier-dynamic", `tick ${tick}`);
      assert.equal(
        snapshot.diagnostics.bodyInputAvailable,
        !["falling", "fallen", "recovering"].includes(snapshot.state),
        `tick ${tick}`,
      );
    }

    const done = character.getSnapshot("canvas2d");
    assert.ok(done.diagnostics.stepCount >= 2,
      `${done.state}, ${done.diagnostics.stepCount} steps, ${done.diagnostics.rootDisplacementM}m root displacement`);
    assert.equal(done.state, "upright");
  } finally { character.dispose(); }
});
