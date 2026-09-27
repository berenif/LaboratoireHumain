import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);
const { recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { reachableFootTarget } = await import("../src/character/recovery-foot-targets.ts");
const { reachableArmBraceTarget } = await import("../src/character/recovery-support.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { dot, sub, rotate, quatFromAxisAngle, worldPoint } = await import("../src/character/math.ts");

function floor(poses, ids) {
  return Math.min(...ids.flatMap(id => {
    const pose = poses.get(id);
    return SEGMENT_BY_ID.get(id).geometry.vertices.map(vertex =>
      worldPoint(pose.position, pose.rotation, vertex).y);
  }));
}

test("a legal measured plant ahead of the search grid is retained, not displaced backward", () => {
  for (const side of ["left", "right"]) for (const heading of [0, 0.73, Math.PI / 2]) {
    const poses = recoveryFixturePoses({ id: "forward-plant", pose: "crouch", side, heading });
    const pelvis = poses.get("pelvis"), measuredFoot = poses.get(`${side}Foot`);
    const hip = worldPoint(pelvis.position, pelvis.rotation,
      SEGMENT_BY_ID.get(`${side}Thigh`).jointAnchorParent);
    const forward = rotate(quatFromAxisAngle({ x: 0, y: 1, z: 0 }, heading), { x: 0, y: 0, z: 1 });
    assert.ok(dot(sub(measuredFoot.position, hip), forward) > 0.10,
      "this valid input lies outside the old coarse grid");
    const target = reachableFootTarget(side, poses, heading);
    assert.equal(target.kind, "floor-plant");
    assert.equal(target.feasible, true);
    assert.ok(target.travelM < 0.005, `nearby sole moved ${target.travelM} m`);
    assert.ok(target.reachErrorM < 1e-8);
    assert.ok(target.jointLimitErrorRad < 1e-8);
    assert.ok(target.floorClearanceM >= -1e-8);
  }
});

test("half-kneel construction aligns both intended supports through connected joint angles", () => {
  for (const side of ["left", "right"]) for (const heading of [0, 0.73, Math.PI / 2]) {
    const poses = recoveryFixturePoses({ id: "support-height", pose: "half-kneel", side, heading });
    const other = side === "left" ? "right" : "left";
    const leading = floor(poses, [`${side}Foot`, `${side}Forefoot`]);
    const shin = floor(poses, [`${other}Shin`]);
    assert.ok(Math.abs(leading - shin) < 1e-10);
    assert.ok(Math.abs(leading - 0.001) < 1e-10);
    assert.ok(floor(poses, [`${other}Foot`, `${other}Forefoot`]) > leading + 0.02);
  }
});

test("a planned brace converges to the floor rather than borrowing runtime contact tolerance", () => {
  for (const side of ["left", "right"]) for (const heading of [0, 0.73, Math.PI / 2]) {
    const poses = recoveryFixturePoses({ id: "converged-brace", pose: "prone", side, heading });
    const target = reachableArmBraceTarget(side, poses, heading);
    assert.equal(target.floorReachable, true);
    assert.ok(target.reachErrorM < 1e-8, `unconverged target: ${target.reachErrorM} m`);
    assert.ok(target.jointLimitErrorRad < 1e-8);
    const hand = `${side}Hand`;
    const targetPoses = new Map(poses);
    targetPoses.set(hand, { ...poses.get(hand), position: target.position, rotation: target.rotation });
    assert.ok(Math.abs(floor(targetPoses, [hand]) - 0.002) < 1e-8);
  }
});
