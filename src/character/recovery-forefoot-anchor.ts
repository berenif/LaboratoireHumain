import { SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SegmentId, SegmentPose, Vec3 } from "../core/types";
import { clampJointCoordinates, jointCoordinates, jointRotationFromCoordinates } from "./joint-coordinates";
import { limbTrunkClearance } from "./limb-collisions";
import { add, clamp, length, quatInverse, rotate, sub } from "./math";
import { reconstructRecoveryLimb } from "./recovery-joints";
import type { RecoveryLegJointRotations } from "./recovery-foot-targets";
import type { RecoverySide } from "./recovery-support";

export interface CapturedForefootPatch {
  /** Pose of the forefoot when its real floor contact was captured. */
  position: Vec3;
  rotation: Quat;
  contact: Vec3;
}

export interface RecoveryForefootAnchorTarget {
  jointRotations: RecoveryLegJointRotations;
  patchErrorM: number;
  floorClearanceM: number;
  bodyClearanceM: number;
}

type Coordinate = { id: SegmentId; axis: "x" | "y" | "z"; maximumChange: number };
type Candidate = {
  angles: number[];
  rotations: RecoveryLegJointRotations;
  point: Vec3;
  errorM: number;
  floorClearanceM: number;
  bodyClearanceM: number;
  valid: boolean;
};

function solveThree(matrix: number[][], rhs: number[]): number[] | null {
  const rows = matrix.map((row, index) => [...row, rhs[index]]);
  for (let column = 0; column < 3; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < 3; row += 1)
      if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
    if (Math.abs(rows[pivot][column]) < 1e-10) return null;
    [rows[column], rows[pivot]] = [rows[pivot], rows[column]];
    const divisor = rows[column][column];
    for (let index = column; index < 4; index += 1) rows[column][index] /= divisor;
    for (let row = 0; row < 3; row += 1) {
      if (row === column) continue;
      const factor = rows[row][column];
      for (let index = column; index < 4; index += 1) rows[row][index] -= factor * rows[column][index];
    }
  }
  return rows.map(row => row[3]);
}

/**
 * Keep a measured rear forefoot patch on the floor while the pelvis rises.
 * Only bounded, anatomically legal joint targets are returned. Rapier remains
 * responsible for moving every body and for deciding whether contact persists.
 */
export function solveRecoveryForefootAnchor(
  side: RecoverySide,
  poses: ReadonlyMap<SegmentId, SegmentPose>,
  captured: CapturedForefootPatch,
): RecoveryForefootAnchorTarget | null {
  const pelvis = poses.get("pelvis");
  if (!pelvis) return null;
  const names = ["Thigh", "Shin", "Ankle", "Foot", "Forefoot"] as const;
  const ids = names.map(name => `${side}${name}` as SegmentId);
  if (ids.some(id => !poses.has(id))) return null;
  const profiles = ids.map(id => SEGMENT_BY_ID.get(id)!.jointProfile!);
  const measured = ids.map((id, index) => {
    const parent = poses.get(SEGMENT_BY_ID.get(id)!.parent!)!;
    return clampJointCoordinates(jointCoordinates(parent.rotation, poses.get(id)!.rotation, profiles[index]), profiles[index]);
  });
  // Sagittal hip, knee, and ankle motion supplies reach. Small hip yaw/roll
  // corrections retain the patch when the body drifts laterally.
  const coordinates: Coordinate[] = [
    { id: ids[0], axis: "x", maximumChange: .30 },
    { id: ids[0], axis: "y", maximumChange: .12 },
    { id: ids[0], axis: "z", maximumChange: .12 },
    { id: ids[1], axis: "x", maximumChange: .30 },
    { id: ids[2], axis: "x", maximumChange: .24 },
  ];
  const original = coordinates.map(({ id, axis }) => measured[ids.indexOf(id)][axis]);
  const localPatch = rotate(quatInverse(captured.rotation), sub(captured.contact, captured.position));
  const bounds = SEGMENT_BY_ID.get(`${side}Forefoot`)!.geometry.localBounds;
  if ((["x", "y", "z"] as const).some(axis => localPatch[axis] < bounds.min[axis] - .02
    || localPatch[axis] > bounds.max[axis] + .02)) return null;
  const evaluate = (angles: number[]): Candidate => {
    const values = measured.map(value => ({ ...value }));
    for (let index = 0; index < coordinates.length; index += 1) {
      const { id, axis, maximumChange } = coordinates[index];
      const limbIndex = ids.indexOf(id);
      values[limbIndex][axis] = clamp(angles[index], original[index] - maximumChange,
        original[index] + maximumChange);
    }
    const rotations = Object.fromEntries(names.map((name, index) => [name.toLowerCase(),
      jointRotationFromCoordinates(clampJointCoordinates(values[index], profiles[index]), profiles[index])])) as unknown as RecoveryLegJointRotations;
    const reconstructed = reconstructRecoveryLimb(side, false, pelvis,
      new Map(ids.map((id, index) => [id, rotations[names[index].toLowerCase() as keyof RecoveryLegJointRotations]])));
    const complete = new Map(poses);
    for (const pose of reconstructed.poses) complete.set(pose.id, pose);
    const forefoot = complete.get(`${side}Forefoot`)!;
    const point = add(forefoot.position, rotate(forefoot.rotation, localPatch));
    const bodyClearanceM = limbTrunkClearance(complete, side, "leg").clearanceM;
    return {
      angles, rotations, point, errorM: length(sub(point, captured.contact)),
      floorClearanceM: reconstructed.floorClearanceM, bodyClearanceM,
      valid: reconstructed.floorClearanceM >= -.003 && bodyClearanceM >= .004,
    };
  };
  let best = evaluate(original);
  // Damped least squares over a world-space material point. Each accepted step
  // is checked against the actual reconstructed collision geometry.
  for (let iteration = 0; iteration < 6 && best.errorM > .002; iteration += 1) {
    const h = .002;
    const derivatives = coordinates.map((_, index) => {
      const angles = [...best.angles]; angles[index] += h;
      const sample = evaluate(angles);
      return { x: (sample.point.x - best.point.x) / h,
        y: (sample.point.y - best.point.y) / h,
        z: (sample.point.z - best.point.z) / h };
    });
    const components = ["x", "y", "z"] as const;
    const matrix = components.map((row, rowIndex) => components.map((column, columnIndex) =>
      derivatives.reduce((sum, derivative) => sum + derivative[row] * derivative[column], 0)
        + (rowIndex === columnIndex ? .002 : 0)));
    const desired = sub(captured.contact, best.point);
    const dual = solveThree(matrix, [desired.x, desired.y, desired.z]);
    if (!dual) break;
    const step = derivatives.map(derivative => clamp(
      derivative.x * dual[0] + derivative.y * dual[1] + derivative.z * dual[2], -.08, .08));
    let improved = false;
    for (const fraction of [1, .5, .25, .125]) {
      const angles = best.angles.map((value, index) => value + fraction * step[index]);
      const candidate = evaluate(angles);
      if (candidate.valid && candidate.errorM < best.errorM - .0001) {
        best = candidate; improved = true; break;
      }
    }
    if (!improved) break;
  }
  if (!best.valid || best.errorM > .018) return null;
  // The motor may pass through intermediate rotations. Verify that the direct
  // blend from this measured configuration does not intersect the floor/trunk.
  for (const fraction of [.25, .5, .75]) {
    const angles = original.map((value, index) => value + (best.angles[index] - value) * fraction);
    if (!evaluate(angles).valid) return null;
  }
  return { jointRotations: best.rotations, patchErrorM: best.errorM,
    floorClearanceM: best.floorClearanceM, bodyClearanceM: best.bodyClearanceM };
}
