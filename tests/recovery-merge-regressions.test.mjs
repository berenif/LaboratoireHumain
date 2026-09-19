import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const unregister = register();
after(unregister);
const { HUMAN_PROPORTIONS, SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { recoveryFixturePoses } = await import("../scripts/recovery-fixtures.ts");
const { reachableFootTarget } = await import("../src/character/recovery-foot-targets.ts");
const { solveRecoveryArmTarget } = await import("../src/character/recovery-support.ts");
const { add, sub, scale, length, normalize, rotate, quatFromAxisAngle } = await import("../src/character/math.ts");
const UP = { x: 0, y: 1, z: 0 };

// These are additional regressions. Existing physics thresholds and tests are unchanged.
test("near-extended recovery arm endpoints remain covariant without a spurious elbow bend", () => {
  const shoulder = { x: 0.1, y: 0.9, z: -0.2 };
  const direction = normalize({ x: 0.2, y: -0.7, z: 0.4 });
  const bend = normalize({ x: 0.2, y: 0.1, z: -0.7 });
  const anchor = SEGMENT_BY_ID.get("leftHand").jointProfile.childFrame.anchor;
  for (const flexion of [-0.45, 0, 0.45]) {
    const wrist = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, flexion);
    const effectiveLength = length(add({ x: 0, y: HUMAN_PROPORTIONS.arm.forearmLengthM, z: 0 }, rotate(wrist, anchor)));
    const reach = HUMAN_PROPORTIONS.arm.upperLengthM + effectiveLength;
    for (const fraction of [1, 1 - 1e-14, 1 - 1e-9, 1.1]) {
      const requested = add(shoulder, scale(direction, reach * fraction));
      const reference = solveRecoveryArmTarget(shoulder, requested, bend, wrist);
      assert.ok(Math.abs(length(sub(reference.elbow, shoulder)) - HUMAN_PROPORTIONS.arm.upperLengthM) < 1e-10);
      assert.ok(Math.abs(reference.reachErrorM - Math.max(0, reach * (fraction - 1))) < 1e-10);
      for (const heading of [-2.7, -1.7, 0.73, Math.PI / 2, 2.2]) {
        const yaw = quatFromAxisAngle(UP, heading);
        const result = solveRecoveryArmTarget(rotate(yaw, shoulder), rotate(yaw, requested), rotate(yaw, bend), wrist, heading);
        for (const key of ["handPosition", "elbow", "wrist"]) {
          assert.ok(length(sub(result[key], rotate(yaw, reference[key]))) < 1e-9,
            `${key}: flexion ${flexion}, fraction ${fraction}, heading ${heading}`);
        }
      }
    }
  }
});

test("a forward half-kneel plant stays near its measured sole instead of searching only behind the hip", () => {
  for (const side of ["left", "right"]) for (const heading of [0, 1.1, -1.7]) {
    const poses = recoveryFixturePoses({ id: "forward-plant", pose: "half-kneel", side, heading });
    const target = reachableFootTarget(side, poses, heading);
    assert.equal(target.feasible, true);
    assert.equal(target.kind, "floor-plant");
    assert.ok(target.travelM < 0.03, `${side}/${heading}: ${target.travelM} m`);
    const local = rotate(quatFromAxisAngle(UP, -heading), sub(target.position, target.hip));
    assert.ok(local.z > 0.30, "the feasible sole lies forward of the hip");
    assert.ok(target.jointLimitErrorRad < 1e-8);
    assert.ok(target.floorClearanceM >= -0.003);
  }
});
