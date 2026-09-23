import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { findLandingPlan, validateLanding } = await import("../src/character/landing-plan.ts");
const { composeTouchdownHindfoot, composeUprightPose, restPoseMap } = await import("../src/character/pose.ts");
const { createEmbodiedCharacter } = await import("../src/character/index.ts");
const { standingChainDiagnostics } = await import("../src/character/standing-chain-diagnostics.ts");
const { planContactLoads } = await import("../src/character/contact-loads.ts");
const { legTargetReach } = await import("../src/character/leg-target-frame.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { worldPoint, rotate, quatFromAxisAngle, quatMultiply } = await import("../src/character/math.ts");
const { recoverySupportHull, recoverySupportMargin } = await import("../src/character/recovery-support.ts");
const zero = { x: 0, y: 0, z: 0 };

test("transfer keeps the moving forefoot planted until measured readiness starts swing", () => {
  const feet = { leftFoot: { x: -0.09, y: 0.045, z: 0.035 },
    rightFoot: { x: 0.09, y: 0.045, z: 0.035 } };
  const input = { rootTranslation: { x: 0, y: 0.97, z: 0 },
    reactionOffset: zero, kneeFlexion: 0.12, simulationTime: 0, activeGrab: null,
    supportFeet: feet, heading: 0, step: null };
  const planted = composeUprightPose(input).poses;
  for (const side of ["left", "right"]) {
    const foot = `${side}Foot`, toe = `${side}Forefoot`;
    const step = { foot, from: feet[foot], to: { ...feet[foot], z: 0.2 },
      elapsed: -0.18, duration: 0.48, heading: 0 };
    const transfer = composeUprightPose({ ...input, step }).poses;
    assert.deepEqual(transfer.get(foot), planted.get(foot));
    assert.deepEqual(transfer.get(toe), planted.get(toe));
    const swing = composeUprightPose({ ...input, step: { ...step, elapsed: 0 } }).poses;
    assert.notDeepEqual(swing.get(toe).rotation, planted.get(toe).rotation);
  }
});

test("unloading keeps the planted target when the measured moving foot drifts upward", () => {
  const feet = { leftFoot: { x: -0.09, y: 0.045, z: 0.035 },
    rightFoot: { x: 0.09, y: 0.045, z: 0.035 } };
  const compose = from => composeUprightPose({ rootTranslation: { x: -0.09, y: 0.97, z: 0 },
    reactionOffset: zero, kneeFlexion: 0.12, simulationTime: 0, activeGrab: null,
    supportFeet: feet, heading: 0,
    step: { foot: "rightFoot", from, to: { x: 0.2, y: 0.045, z: 0.1 },
      elapsed: -0.18, duration: 0.48, heading: 0 } }).poses;
  const planted = compose(feet.rightFoot);
  const drifted = compose({ x: 0.11, y: 0.065, z: 0.04 });
  assert.deepEqual(drifted.get("rightFoot"), planted.get("rightFoot"));
  assert.deepEqual(drifted.get("pelvis"), planted.get("pelvis"));
});

test("transfer posture preserves reach for both planted legs while shifting weight", () => {
  const feet = { leftFoot: { x: -0.09, y: 0.045, z: 0.035 },
    rightFoot: { x: 0.09, y: 0.039, z: 0.035 } };
  const identity = { x: 0, y: 0, z: 0, w: 1 };
  const poses = composeUprightPose({ rootTranslation: { x: -0.09, y: 0.99, z: 0 },
    reactionOffset: zero, kneeFlexion: 0.12, simulationTime: 0, activeGrab: null,
    supportFeet: feet, heading: 0,
    step: { foot: "rightFoot", from: feet.rightFoot, to: { x: 0.2, y: 0.045, z: 0.1 },
      elapsed: -0.18, duration: 0.48, heading: 0 } }).poses;
  const pelvis = poses.get("pelvis");
  assert.ok(pelvis.position.y < 0.978);
  for (const side of ["left", "right"]) {
    const foot = `${side}Foot`;
    const hip = worldPoint(pelvis.position, pelvis.rotation, SEGMENT_BY_ID.get(`${side}Thigh`).jointAnchorParent);
    const reach = legTargetReach(side, hip, feet[foot], identity);
    assert.ok(reach.distanceM <= reach.radiusM - 0.008 + 1e-9);
    assert.ok(Math.hypot(...["x", "y", "z"].map(axis => poses.get(foot).position[axis] - feet[foot][axis])) < 0.004);
  }
});

// Measured at the previously failing third-step launch, not a fabricated easy pose.
const reversal = {
  foot: "rightFoot", from: { x: 0.0485282876, y: 0.0456389189, z: -0.0143593065 },
  pelvis: { position: { x: -0.0719109029, y: 0.9483067393, z: -0.0698641166 },
    rotation: { x: -0.2902095020, y: -0.0519015789, z: 0.0647668839, w: 0.9533572197 } },
  heading: 0, requested: { x: -0.1093139079, y: 0.045, z: -0.4279158101 },
  retainedFoot: { x: -0.0692598671, y: 0.0429582074, z: -0.0199502222 },
  floorY: 0, maxTravelM: 0.43, maxReachM: 0.36,
};

test("an infeasible starting foot is never returned as an unchecked landing fallback", () => {
  assert.equal(validateLanding(reversal, reversal.from).feasible, false);
  const impossible = { ...reversal,
    pelvis: { ...reversal.pelvis, position: { ...reversal.pelvis.position, y: 1.5 } } };
  const plan = findLandingPlan(impossible);
  assert.equal(plan.feasible, false);
  assert.match(plan.reason, /radial-reach/);
  assert.equal("to" in plan, false);
});

test("joint-limited landing search finds an off-ray placement when the entire request ray fails", () => {
  for (let i = 0; i <= 100; i++) {
    const candidate = Object.fromEntries(["x", "y", "z"].map(axis =>
      [axis, reversal.from[axis] + (reversal.requested[axis] - reversal.from[axis]) * i / 100]));
    assert.equal(validateLanding(reversal, candidate).feasible, false);
  }
  const plan = findLandingPlan(reversal);
  assert.equal(plan.feasible, true, plan.reason);
  const bounded = composeTouchdownHindfoot("right", reversal.pelvis, plan.to, 0);
  assert.ok(Math.hypot(bounded.x - plan.to.x, bounded.z - plan.to.z) <= 0.04);
  assert.ok(Math.abs(bounded.y - plan.to.y) <= 0.04);
  assert.ok(Math.hypot(plan.to.x - reversal.from.x, plan.to.z - reversal.from.z) <= 0.43);
  const rayX = reversal.requested.x - reversal.from.x, rayZ = reversal.requested.z - reversal.from.z;
  assert.ok(Math.abs((plan.to.x - reversal.from.x) * rayZ - (plan.to.z - reversal.from.z) * rayX) > 0.001);
});

test("equal combined capture support prefers a useful landing sole over a zero-travel lift", () => {
  // Recorded first launch at .95s: the retained sole contains forecast capture,
  // so its boundary caps the combined margin even when the new sole is far away.
  const recorded = {
    foot: "rightFoot", from: { x: 0.0941939726471901, y: 0.04690068960189819, z: 0.038334012031555176 },
    pelvis: { position: { x: -0.07423019409179688, y: 0.9615345597267151, z: -0.0105861471965909 },
      rotation: { x: -0.0032503900583833456, y: -0.0034198304638266563,
        z: 0.007718315813690424, w: 0.9999591708183289 } },
    heading: 0, requested: { x: 0.1203, y: 0.045, z: 0.0623 },
    retainedFoot: { x: -0.08948052674531937, y: 0.043990928679704666, z: 0.03350062668323517 },
    floorY: 0, maxTravelM: 0.43, maxReachM: 0.36,
    capturePoint: { x: -0.0910382740801762, y: 0, z: 0.0067535920045191326 },
    retainedContacts: [
      { segment: "leftFoot", normalY: 1, forceN: 492.39466667175293,
        persistenceS: 0.9333333333333346, loadBearing: true,
        point: { x: -0.0862925536930561, y: 0.0006420876597985625, z: -0.020747612230479717 },
        points: [
          { x: -0.13249170780181885, y: -0.00010633282363414764, z: -0.039547957479953766 },
          { x: -0.13249170780181885, y: -0.0001063365489244461, z: -0.03954799845814705 },
          { x: -0.044507477432489395, y: 0.0007526481058448553, z: -0.03941789269447327 },
          { x: -0.035679321736097336, y: 0.0020283719059079885, z: 0.03552339971065521 },
        ] },
      { segment: "leftForefoot", normalY: 1, forceN: 81.48088574409485,
        persistenceS: 0.35, loadBearing: true,
        point: { x: -0.14089221507310867, y: 0.0025123904924839735, z: 0.11448176205158234 },
        points: [
          { x: -0.14089222252368927, y: 0.0025123932864516973, z: 0.11448170244693756 },
          { x: -0.14089220762252808, y: 0.0025123876985162497, z: 0.11448182165622711 },
        ] },
    ],
  };
  const freeze = value => {
    if (value && typeof value === "object") { Object.values(value).forEach(freeze); Object.freeze(value); }
    return value;
  };
  for (const heading of [0, 0.7, Math.PI / 2]) {
    const rotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading), r = point => rotate(rotation, point);
    const input = { ...recorded, heading, from: r(recorded.from), requested: r(recorded.requested),
      retainedFoot: r(recorded.retainedFoot), capturePoint: r(recorded.capturePoint),
      pelvis: { position: r(recorded.pelvis.position), rotation: quatMultiply(rotation, recorded.pelvis.rotation) },
      retainedContacts: recorded.retainedContacts.map(contact => ({ ...contact,
        point: r(contact.point), points: contact.points.map(r) })) };
    const before = structuredClone(input);
    freeze(input);
    const baseline = validateLanding(input, input.from), chosen = findLandingPlan(input);
    assert.equal(baseline.feasible, true, baseline.reason);
    assert.equal(chosen.feasible, true, chosen.reason);
    assert.equal(baseline.travelM, 0);
    assert.ok(Math.abs(chosen.captureMarginM - baseline.captureMarginM) <= 1e-5,
      "the retained-patch bottleneck leaves these placements tied on combined support");
    const soleMargin = plan => recoverySupportMargin(input.capturePoint, recoverySupportHull(
      SEGMENT_BY_ID.get(input.foot).geometry.supportPatch.map(point => worldPoint(plan.to, rotation, point))));
    assert.ok(soleMargin(chosen) > soleMargin(baseline) + 0.05,
      "the new sole must improve capture support before minimizing foot travel");
    assert.ok(Math.abs(chosen.landingCaptureMarginM - soleMargin(chosen)) < 1e-9);
    assert.ok(chosen.travelM > 0.05 && chosen.travelM <= input.maxTravelM);
    assert.deepEqual(validateLanding(input, chosen.to), chosen,
      "the chosen placement must pass the same complete geometry validator");
    assert.deepEqual(input, before, "candidate ranking must preserve measured observations");
  }
});

test("landing preview and motor FK use the same measured pelvis despite a different posture target", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    const plan = findLandingPlan(reversal);
    assert.equal(plan.feasible, true, plan.reason);
    const measured = restPoseMap();
    measured.set("pelvis", { ...measured.get("pelvis"), ...reversal.pelvis });
    const step = { foot: "rightFoot", from: reversal.from, to: plan.to,
      elapsed: 0.48, duration: 0.48, heading: 0, unloaded: true };
    const desired = composeUprightPose({ rootTranslation: { x: 0.1, y: 0.99, z: 0.12 },
      reactionOffset: { x: 0.1, y: 0, z: 0.1 }, kneeFlexion: 0.12,
      simulationTime: 0, activeGrab: null, heading: 0,
      supportFeet: { leftFoot: reversal.retainedFoot, rightFoot: reversal.from },
      measuredPoses: measured, measuredLegFrame: true, step }).poses;
    character.poses = measured;
    character.step = step;
    const commands = character.motorCommands(desired);
    const chain = standingChainDiagnostics(measured, desired, commands, step, 0, 0, 0, [], measured.get("pelvis"));
    const foot = chain.legs.find(leg => leg.side === "right").segments.find(segment => segment.id === "rightFoot");
    const preview = composeTouchdownHindfoot("right", reversal.pelvis, plan.to, 0);
    assert.ok(Math.hypot(...["x", "y", "z"].map(axis => preview[axis] - foot.commanded.position[axis])) < 1e-8);
    assert.ok(foot.commandFrameErrorM < 1e-8);
  } finally { character.dispose(); }
});

test("both planted sole targets agree with measured-pelvis motor FK throughout prelift", async () => {
  const character = await createEmbodiedCharacter("canvas2d");
  try {
    for (const heading of [0, 0.7]) {
      const rotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading);
      const supportFeet = { leftFoot: rotate(rotation, { x: -0.09, y: 0.045, z: 0.035 }),
        rightFoot: rotate(rotation, { x: 0.09, y: 0.045, z: 0.035 }) };
      const supportFootRotations = { leftFoot: rotation, rightFoot: rotation };
      const measured = composeUprightPose({ rootTranslation: { x: 0, y: 0.97, z: 0 },
        reactionOffset: zero, kneeFlexion: 0.12,
        simulationTime: 0, activeGrab: null, heading, supportFeet, supportFootRotations, step: null }).poses;
      for (const movingFoot of ["leftFoot", "rightFoot"]) {
        const step = { foot: movingFoot, from: supportFeet[movingFoot],
          to: { ...supportFeet[movingFoot], z: supportFeet[movingFoot].z + 0.12 },
          elapsed: -0.18, duration: 0.48, heading };
        const desired = composeUprightPose({ rootTranslation: { x: 0.08, y: 0.99, z: 0.09 },
          reactionOffset: { x: -0.1, y: 0, z: 0.1 }, kneeFlexion: 0.12,
          simulationTime: 0, activeGrab: null, heading, supportFeet, supportFootRotations,
          measuredPoses: measured, measuredLegFrame: true, step }).poses;
        assert.ok(Math.hypot(...["x", "y", "z"].map(axis =>
          desired.get("pelvis").position[axis] - measured.get("pelvis").position[axis])) > 0.05);
        character.poses = measured;
        character.step = step;
        const commands = character.motorCommands(desired);
        const chain = standingChainDiagnostics(measured, desired, commands, step, heading,
          0, 0, [], measured.get("pelvis"));
        for (const leg of chain.legs) {
          const id = `${leg.side}Foot`, foot = leg.segments.find(segment => segment.id === id);
          const preview = composeTouchdownHindfoot(leg.side, measured.get("pelvis"), supportFeet[id], heading);
          assert.ok(Math.hypot(...["x", "y", "z"].map(axis =>
            preview[axis] - supportFeet[id][axis])) < 1e-8, `${id} fixture must be reachable`);
          assert.ok(foot.commandFrameErrorM < 1e-8, `${id} command must retain its world frame`);
          for (const point of SEGMENT_BY_ID.get(id).geometry.supportPatch) {
            const expected = worldPoint(supportFeet[id], supportFootRotations[id], point);
            const composed = worldPoint(foot.desired.position, foot.desired.rotation, point);
            const commanded = worldPoint(foot.commanded.position, foot.commanded.rotation, point);
            assert.ok(Math.hypot(...["x", "y", "z"].map(axis => composed[axis] - expected[axis])) < 1e-8,
              `${id} composed sole must remain on its planted world target`);
            assert.ok(Math.hypot(...["x", "y", "z"].map(axis => commanded[axis] - expected[axis])) < 1e-8,
              `${id} local-command FK must preserve the planted sole`);
          }
        }
      }
    }
  } finally { character.dispose(); }
});

test("transfer pressure conserves force with duplicate measured manifold points", () => {
  const contacts = ["leftFoot", "rightFoot"].map((segment, i) => {
    const x = i ? 0.1 : -0.1;
    return { segment, normalY: 1, forceN: 350, loadBearing: true, persistenceS: 0.1,
      point: { x, y: 0, z: 0 }, points: [{ x, y: 0, z: 0 }, { x, y: 0, z: 0 }] };
  });
  const result = planContactLoads(contacts, { x: 0, y: 1, z: 0 }, zero,
    { x: 0, y: 700, z: 0 }, { frictionCoefficient: 1.2,
      maxHorizontalForceN: 200, maxJointTorqueNm: 200, projectMeasuredPressure: true });
  assert.equal(result.loads.length, 2);
  assert.ok(result.loads.every(load => load.plannedForce.y >= 0));
  assert.ok(Math.abs(result.allocatedForce.y - 700) < 1e-8);
  assert.ok(Math.abs(result.loads.reduce((sum, load) => sum + load.plannedForce.y, 0) - 700) < 1e-8);
});

test("a clipped transfer pressure reduces force to the same achievable wrench", () => {
  const contact = { segment: "rightFoot", normalY: 1, forceN: 700,
    loadBearing: true, persistenceS: 0.1, point: { x: 0.14, y: 0, z: 0 },
    points: [{ x: 0.05, y: 0, z: -0.04 }, { x: 0.14, y: 0, z: -0.04 },
      { x: 0.14, y: 0, z: 0.12 }, { x: 0.05, y: 0, z: 0.12 }] };
  const result = planContactLoads([contact], { x: 0, y: 1, z: 0 }, zero,
    { x: -180, y: 700, z: 0 }, { frictionCoefficient: 1.2,
      maxHorizontalForceN: 200, maxJointTorqueNm: 200, projectMeasuredPressure: true });
  assert.equal(result.requestedForce.x, -180);
  assert.ok(Math.abs(result.allocatedForce.x + 98) < 1e-8);
  assert.ok(Math.abs(result.allocatedForce.x + result.pressurePoint.x * 700) < 1e-8);
  assert.equal(result.pressureFeasible, true);
  assert.ok(result.pressureForceResidualNm < 1e-8);
});

test("forward momentum cannot reverse braking while COM remains behind measured pressure", () => {
  const contact = { segment: "rightFoot", normalY: 1, forceN: 700,
    loadBearing: true, persistenceS: 0.1, point: { x: 0.1464, y: 0, z: 0 },
    points: [{ x: 0.05, y: 0, z: -0.04 }, { x: 0.1464, y: 0, z: -0.04 },
      { x: 0.1464, y: 0, z: 0.12 }, { x: 0.05, y: 0, z: 0.12 }] };
  const result = planContactLoads([contact], { x: 0.1427, y: 1, z: 0 },
    { x: 0.2498, y: 0, z: 0 }, { x: -259.5, y: 700, z: 0 },
    { frictionCoefficient: 1.2, maxHorizontalForceN: 260,
      maxJointTorqueNm: 260, projectMeasuredPressure: true });
  assert.ok(result.allocatedForce.x < 0);
  assert.ok(Math.abs(result.allocatedForce.x - (0.1427 - result.pressurePoint.x) * 700) < 1e-8);
});

test("rotating skewed contact patches preserves the commanded load on each sole", () => {
  const contacts = ["leftFoot", "rightFoot"].map((segment, i) => {
    const x = i ? 0.12 : -0.12;
    return { segment, point: { x, y: 0, z: 0 }, forceN: 350,
      normalY: 1, persistenceS: 1, loadBearing: true,
      points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({
        x: x + sx * .05 + (sz > 0 ? .01 : 0), y: 0,
        z: sz * .1 + (i ? -.02 : .015) + (sx > 0 ? .012 : 0),
      }))) };
  });
  const com = { x: 0.02, y: 1, z: 0 }, force = { x: 60, y: 700, z: 35 };
  const options = { frictionCoefficient: 1.2, maxHorizontalForceN: 260,
    maxJointTorqueNm: 260, projectMeasuredPressure: true };
  const reference = planContactLoads(contacts, com, zero, force, options);
  for (const angle of [Math.PI / 3, Math.PI / 2, -Math.PI * .73]) {
    const rotation = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, angle);
    const r = vector => rotate(rotation, vector);
    const result = planContactLoads(contacts.map(contact => ({ ...contact,
      point: r(contact.point), points: contact.points.map(r) })), r(com), zero, r(force), options);
    for (const original of reference.loads) {
      const actual = result.loads.find(load => load.segment === original.segment);
      assert.ok(Math.abs(actual.plannedForce.y - original.plannedForce.y) < 1e-7);
    }
  }
});

test("transfer can request retained sole pressure without removing the loaded moving contact", () => {
  const contacts = ["leftFoot", "rightFoot"].map((segment, index) => {
    const x = index ? 0.12 : -0.12;
    return { segment, point: { x, y: 0, z: 0 }, forceN: 350,
      normalY: 1, persistenceS: 0.2, loadBearing: true,
      points: [-1, 1].flatMap(sx => [-1, 1].map(sz => ({
        x: x + sx * 0.05, y: 0, z: sz * 0.09,
      }))) };
  });
  const result = planContactLoads(contacts, { x: 0.02, y: 1, z: 0 }, zero,
    { x: 100, y: 700, z: 0 }, { frictionCoefficient: 1.2,
      maxHorizontalForceN: 200, maxJointTorqueNm: 200,
      projectMeasuredPressure: true, allowSupportMoment: true,
      pressureTarget: { x: 0.12, y: 0, z: 0 }, retainedSide: "right" });
  assert.deepEqual(result.loads.map(load => load.segment), ["leftFoot", "rightFoot"]);
  assert.equal(result.loads[0].measuredForceN, 350, "measured moving load is never replaced by intent");
  assert.ok(result.loads[0].plannedForce.y < 1e-8);
  assert.ok(Math.abs(result.loads[1].plannedForce.y - 700) < 1e-8);
  assert.ok(Math.abs(result.pressurePoint.x - 0.12) < 1e-8);
  assert.ok(Math.abs(result.allocatedForce.x - 100) < 1e-8);
  assert.ok(result.pressureForceResidualNm > 0, "the transient moment demand remains explicit");
});
