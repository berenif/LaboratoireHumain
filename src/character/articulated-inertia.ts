import type { RigidBody } from "@dimforge/rapier3d-compat";

import { SEGMENTS } from "../core/humanoid";
import type { JointCoordinate, SegmentId, Vec3 } from "../core/types";
import { jointCoordinateKinematics, jointCoordinates } from "./joint-coordinates";
import { add, cross, scale, worldPoint } from "./math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const BASE_DOF_COUNT = 6;

type SymmetricMatrix3 = Readonly<{
  m11: number; m12: number; m13: number;
  m22: number; m23: number; m33: number;
}>;

type SpatialJacobian = Readonly<{
  linear: readonly [Float64Array, Float64Array, Float64Array];
  angular: readonly [Float64Array, Float64Array, Float64Array];
}>;

export interface ArticulatedCoordinateResponse {
  get(
    leftId: SegmentId,
    leftCoordinate: JointCoordinate,
    rightId: SegmentId,
    rightCoordinate: JointCoordinate,
  ): number | undefined;
}

/** A measured, sticking contact patch used only to predict constrained response. */
export interface ArticulatedSupportConstraint {
  readonly segment: SegmentId;
  readonly points: readonly Vec3[];
  /** Sticking velocity directions; omit for all three world directions. */
  readonly directions?: readonly Vec3[];
}

function coordinateKey(id: SegmentId, coordinate: JointCoordinate): string {
  return `${id}:${coordinate}`;
}

function invertSymmetric3(matrix: SymmetricMatrix3): SymmetricMatrix3 {
  const a = matrix.m22 * matrix.m33 - matrix.m23 * matrix.m23;
  const b = matrix.m13 * matrix.m23 - matrix.m12 * matrix.m33;
  const c = matrix.m12 * matrix.m23 - matrix.m13 * matrix.m22;
  const e = matrix.m11 * matrix.m33 - matrix.m13 * matrix.m13;
  const f = matrix.m12 * matrix.m13 - matrix.m11 * matrix.m23;
  const g = matrix.m11 * matrix.m22 - matrix.m12 * matrix.m12;
  const determinant = matrix.m11 * a + matrix.m12 * b + matrix.m13 * c;
  if (!(determinant > 1e-15) || !Number.isFinite(determinant)) {
    throw new RangeError("Articulated body inertia must be positive definite.");
  }
  return {
    m11: a / determinant,
    m12: b / determinant,
    m13: c / determinant,
    m22: e / determinant,
    m23: f / determinant,
    m33: g / determinant,
  };
}

function multiplySymmetric3(matrix: SymmetricMatrix3, vector: Vec3): Vec3 {
  return {
    x: matrix.m11 * vector.x + matrix.m12 * vector.y + matrix.m13 * vector.z,
    y: matrix.m12 * vector.x + matrix.m22 * vector.y + matrix.m23 * vector.z,
    z: matrix.m13 * vector.x + matrix.m23 * vector.y + matrix.m33 * vector.z,
  };
}

function solveDense(matrix: number[][], rhs: number[]): number[] {
  const size = rhs.length;
  for (let column = 0; column < size; column++) {
    let pivot = column;
    for (let row = column + 1; row < size; row++) {
      if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
    }
    if (Math.abs(matrix[pivot][column]) < 1e-12) {
      throw new RangeError("Joint coordinate differential is singular.");
    }
    [matrix[column], matrix[pivot]] = [matrix[pivot], matrix[column]];
    [rhs[column], rhs[pivot]] = [rhs[pivot], rhs[column]];
    for (let row = column + 1; row < size; row++) {
      const ratio = matrix[row][column] / matrix[column][column];
      for (let inner = column; inner < size; inner++) {
        matrix[row][inner] -= ratio * matrix[column][inner];
      }
      rhs[row] -= ratio * rhs[column];
    }
  }
  const result = new Array<number>(size).fill(0);
  for (let row = size - 1; row >= 0; row--) {
    let value = rhs[row];
    for (let column = row + 1; column < size; column++) {
      value -= matrix[row][column] * result[column];
    }
    result[row] = value / matrix[row][row];
  }
  return result;
}

/** Physical relative-angular-velocity columns for unit anatomical coordinate rates. */
function coordinateVelocityAxes(rows: readonly Vec3[]): Vec3[] {
  const size = rows.length;
  const gram = rows.map((left) => rows.map((right) => (
    left.x * right.x + left.y * right.y + left.z * right.z
  )));
  return rows.map((_, coordinate) => {
    const rhs = new Array<number>(size).fill(0);
    rhs[coordinate] = 1;
    const weights = solveDense(gram.map((row) => [...row]), rhs);
    return rows.reduce((sum, row, index) => add(sum, scale(row, weights[index])), ZERO);
  });
}

function cloneRows(rows: SpatialJacobian["linear"]): [Float64Array, Float64Array, Float64Array] {
  return [new Float64Array(rows[0]), new Float64Array(rows[1]), new Float64Array(rows[2])];
}

function addCrossColumn(
  target: [Float64Array, Float64Array, Float64Array],
  source: SpatialJacobian["angular"],
  arm: Vec3,
  sign: number,
): void {
  for (let column = 0; column < source[0].length; column++) {
    const contribution = cross({
      x: source[0][column],
      y: source[1][column],
      z: source[2][column],
    }, arm);
    target[0][column] += contribution.x * sign;
    target[1][column] += contribution.y * sign;
    target[2][column] += contribution.z * sign;
  }
}

function cholesky(matrix: Float64Array, size: number): Float64Array {
  const factor = new Float64Array(matrix);
  for (let row = 0; row < size; row++) {
    for (let column = 0; column <= row; column++) {
      let value = factor[row * size + column];
      for (let inner = 0; inner < column; inner++) {
        value -= factor[row * size + inner] * factor[column * size + inner];
      }
      if (row === column) {
        if (!(value > 0) || !Number.isFinite(value)) {
          throw new RangeError("Articulated mass matrix must be positive definite.");
        }
        factor[row * size + column] = Math.sqrt(value);
      } else {
        factor[row * size + column] = value / factor[column * size + column];
      }
    }
  }
  return factor;
}

function solveCholesky(factor: Float64Array, rhs: Float64Array): Float64Array {
  const size = rhs.length;
  const result = new Float64Array(rhs);
  for (let row = 0; row < size; row++) {
    for (let column = 0; column < row; column++) {
      result[row] -= factor[row * size + column] * result[column];
    }
    result[row] /= factor[row * size + row];
  }
  for (let row = size - 1; row >= 0; row--) {
    for (let column = row + 1; column < size; column++) {
      result[row] -= factor[column * size + row] * result[column];
    }
    result[row] /= factor[row * size + row];
  }
  return result;
}

function dotDense(left: Float64Array, right: Float64Array): number {
  let value = 0;
  for (let index = 0; index < left.length; index++) value += left[index] * right[index];
  return value;
}

type ConstraintRow = Readonly<{
  row: Float64Array;
  inverseMassResponse: Float64Array;
}>;

/**
 * Keep only independent contact rows in the inverse-mass metric. A convex
 * patch often reports redundant coplanar points; projecting all of them
 * through a regularized inverse would make the motor response depend on the
 * number and ordering of solver contacts.
 */
function independentConstraintRows(
  candidates: readonly Float64Array[],
  massFactor: Float64Array,
): ConstraintRow[] {
  const selected: ConstraintRow[] = [];
  let maximumDiagonal = 0;
  for (const row of candidates) {
    const inverseMassResponse = solveCholesky(massFactor, row);
    const diagonal = dotDense(row, inverseMassResponse);
    if (!(diagonal > 0) || !Number.isFinite(diagonal)) continue;
    maximumDiagonal = Math.max(maximumDiagonal, diagonal);

    let residual = diagonal;
    if (selected.length > 0) {
      const size = selected.length;
      const gram = new Float64Array(size * size);
      const crossResponse = new Float64Array(size);
      for (let left = 0; left < size; left++) {
        crossResponse[left] = dotDense(selected[left].row, inverseMassResponse);
        for (let right = 0; right <= left; right++) {
          const value = dotDense(selected[left].row, selected[right].inverseMassResponse);
          gram[left * size + right] = value;
          gram[right * size + left] = value;
        }
      }
      const weights = solveCholesky(cholesky(gram, size), crossResponse);
      residual -= dotDense(crossResponse, weights);
    }
    if (residual > Math.max(1e-11, maximumDiagonal * 1e-9)) {
      selected.push({ row, inverseMassResponse });
    }
  }
  return selected;
}

/**
 * Build the free-floating articulated mass response in anatomical coordinates.
 *
 * The previous motor solve used each collider's unconstrained COM inertia.
 * That is only the response before Rapier enforces connected joint anchors and
 * locked axes; for the small ankle housing it can overstate real response by
 * more than an order of magnitude. Here every body's velocity Jacobian already
 * satisfies the structural joints, so the inverse mass matrix predicts the
 * same constrained assembly that receives the motor impulses.
 */
export function articulatedCoordinateResponse(
  bodies: ReadonlyMap<SegmentId, RigidBody>,
  supports: readonly ArticulatedSupportConstraint[] = [],
): ArticulatedCoordinateResponse {
  const coordinateIndices = new Map<string, number>();
  let dofCount = BASE_DOF_COUNT;
  for (const definition of SEGMENTS) {
    for (const axis of definition.jointProfile?.axes ?? []) {
      coordinateIndices.set(coordinateKey(definition.id, axis.coordinate), dofCount++);
    }
  }

  const jacobians = new Map<SegmentId, SpatialJacobian>();
  const root = SEGMENTS.find((definition) => definition.parent === null);
  if (!root) throw new RangeError("Articulated body requires a root segment.");
  const rootLinear: [Float64Array, Float64Array, Float64Array] = [
    new Float64Array(dofCount), new Float64Array(dofCount), new Float64Array(dofCount),
  ];
  const rootAngular: [Float64Array, Float64Array, Float64Array] = [
    new Float64Array(dofCount), new Float64Array(dofCount), new Float64Array(dofCount),
  ];
  rootLinear[0][0] = 1; rootLinear[1][1] = 1; rootLinear[2][2] = 1;
  rootAngular[0][3] = 1; rootAngular[1][4] = 1; rootAngular[2][5] = 1;
  jacobians.set(root.id, { linear: rootLinear, angular: rootAngular });

  for (const definition of SEGMENTS) {
    if (!definition.parent || !definition.jointProfile) continue;
    const parentBody = bodies.get(definition.parent);
    const childBody = bodies.get(definition.id);
    const parentJacobian = jacobians.get(definition.parent);
    if (!parentBody || !childBody || !parentJacobian) {
      throw new RangeError(`Missing articulated body state for ${definition.id}.`);
    }
    const coordinates = jointCoordinates(
      parentBody.rotation(), childBody.rotation(), definition.jointProfile,
    );
    const kinematics = jointCoordinateKinematics(
      ZERO, ZERO, parentBody.rotation(), coordinates, definition.jointProfile,
    );
    const coordinateRows = definition.jointProfile.axes.map(
      (axis) => kinematics.torqueAxesWorld[axis.coordinate],
    );
    const velocityAxes = coordinateVelocityAxes(coordinateRows);
    const angular = cloneRows(parentJacobian.angular);
    for (const [axisIndex, axis] of definition.jointProfile.axes.entries()) {
      const index = coordinateIndices.get(coordinateKey(definition.id, axis.coordinate))!;
      angular[0][index] += velocityAxes[axisIndex].x;
      angular[1][index] += velocityAxes[axisIndex].y;
      angular[2][index] += velocityAxes[axisIndex].z;
    }

    const parentAnchor = worldPoint(
      parentBody.translation(), parentBody.rotation(), definition.jointProfile.parentFrame.anchor,
    );
    const childAnchor = worldPoint(
      childBody.translation(), childBody.rotation(), definition.jointProfile.childFrame.anchor,
    );
    const jointPoint = scale(add(parentAnchor, childAnchor), 0.5);
    const linear = cloneRows(parentJacobian.linear);
    addCrossColumn(linear, parentJacobian.angular, {
      x: jointPoint.x - parentBody.worldCom().x,
      y: jointPoint.y - parentBody.worldCom().y,
      z: jointPoint.z - parentBody.worldCom().z,
    }, 1);
    addCrossColumn(linear, angular, {
      x: jointPoint.x - childBody.worldCom().x,
      y: jointPoint.y - childBody.worldCom().y,
      z: jointPoint.z - childBody.worldCom().z,
    }, -1);
    jacobians.set(definition.id, { linear, angular });
  }

  const massMatrix = new Float64Array(dofCount * dofCount);
  for (const definition of SEGMENTS) {
    const body = bodies.get(definition.id);
    const jacobian = jacobians.get(definition.id);
    if (!body || !jacobian) throw new RangeError(`Missing articulated inertia for ${definition.id}.`);
    const rawInverse = body.effectiveWorldInvInertia();
    const inertia = invertSymmetric3({
      m11: rawInverse.m11, m12: rawInverse.m12, m13: rawInverse.m13,
      m22: rawInverse.m22, m23: rawInverse.m23, m33: rawInverse.m33,
    });
    const mass = body.mass();
    for (let left = 0; left < dofCount; left++) {
      const angularLeft = {
        x: jacobian.angular[0][left], y: jacobian.angular[1][left], z: jacobian.angular[2][left],
      };
      for (let right = 0; right <= left; right++) {
        const angularRight = {
          x: jacobian.angular[0][right], y: jacobian.angular[1][right], z: jacobian.angular[2][right],
        };
        const rotated = multiplySymmetric3(inertia, angularRight);
        const value = mass * (
          jacobian.linear[0][left] * jacobian.linear[0][right]
          + jacobian.linear[1][left] * jacobian.linear[1][right]
          + jacobian.linear[2][left] * jacobian.linear[2][right]
        ) + angularLeft.x * rotated.x + angularLeft.y * rotated.y + angularLeft.z * rotated.z;
        massMatrix[left * dofCount + right] += value;
        if (left !== right) massMatrix[right * dofCount + left] += value;
      }
    }
  }

  const factor = cholesky(massMatrix, dofCount);
  const contactRows: Float64Array[] = [];
  for (const support of supports) {
    const body = bodies.get(support.segment);
    const jacobian = jacobians.get(support.segment);
    if (!body || !jacobian) {
      throw new RangeError(`Missing articulated support body state for ${support.segment}.`);
    }
    for (const point of support.points) {
      if (![point.x, point.y, point.z].every(Number.isFinite)) {
        throw new RangeError("Articulated support points must be finite.");
      }
      const pointLinear = cloneRows(jacobian.linear);
      addCrossColumn(pointLinear, jacobian.angular, {
        x: point.x - body.worldCom().x,
        y: point.y - body.worldCom().y,
        z: point.z - body.worldCom().z,
      }, 1);
      const directions = support.directions ?? [
        { x: 1, y: 0, z: 0 },
        { x: 0, y: 1, z: 0 },
        { x: 0, y: 0, z: 1 },
      ];
      for (const rawDirection of directions) {
        const magnitude = Math.hypot(rawDirection.x, rawDirection.y, rawDirection.z);
        if (!(magnitude > 1e-8) || !Number.isFinite(magnitude)) {
          throw new RangeError("Articulated support directions must be finite and nonzero.");
        }
        const direction = {
          x: rawDirection.x / magnitude,
          y: rawDirection.y / magnitude,
          z: rawDirection.z / magnitude,
        };
        const row = new Float64Array(dofCount);
        for (let dof = 0; dof < dofCount; dof++) {
          row[dof] = direction.x * pointLinear[0][dof]
            + direction.y * pointLinear[1][dof]
            + direction.z * pointLinear[2][dof];
        }
        contactRows.push(row);
      }
    }
  }
  const constraints = independentConstraintRows(contactRows, factor);
  const contactCount = constraints.length;
  const contactFactor = contactCount > 0 ? (() => {
    const gram = new Float64Array(contactCount * contactCount);
    for (let left = 0; left < contactCount; left++) {
      for (let right = 0; right <= left; right++) {
        const value = dotDense(constraints[left].row, constraints[right].inverseMassResponse);
        gram[left * contactCount + right] = value;
        gram[right * contactCount + left] = value;
      }
    }
    return cholesky(gram, contactCount);
  })() : null;
  const jointCount = dofCount - BASE_DOF_COUNT;
  const response = new Float64Array(jointCount * jointCount);
  for (let right = BASE_DOF_COUNT; right < dofCount; right++) {
    const rhs = new Float64Array(dofCount);
    rhs[right] = 1;
    const solution = solveCholesky(factor, rhs);
    if (contactFactor) {
      const contactVelocity = new Float64Array(contactCount);
      for (let row = 0; row < contactCount; row++) {
        contactVelocity[row] = dotDense(constraints[row].row, solution);
      }
      const reactions = solveCholesky(contactFactor, contactVelocity);
      for (let row = 0; row < contactCount; row++) {
        const reactionResponse = constraints[row].inverseMassResponse;
        for (let dof = 0; dof < dofCount; dof++) {
          solution[dof] -= reactionResponse[dof] * reactions[row];
        }
      }
    }
    for (let left = BASE_DOF_COUNT; left < dofCount; left++) {
      response[(left - BASE_DOF_COUNT) * jointCount + right - BASE_DOF_COUNT] = solution[left];
    }
  }

  // Both H^-1 and its contact projection are symmetric. Average round-off from
  // the independent contact solve so the downstream bounded solve remains SPD.
  for (let left = 0; left < jointCount; left++) {
    for (let right = 0; right < left; right++) {
      const value = 0.5 * (
        response[left * jointCount + right] + response[right * jointCount + left]
      );
      response[left * jointCount + right] = value;
      response[right * jointCount + left] = value;
    }
  }

  return {
    get(leftId, leftCoordinate, rightId, rightCoordinate) {
      const left = coordinateIndices.get(coordinateKey(leftId, leftCoordinate));
      const right = coordinateIndices.get(coordinateKey(rightId, rightCoordinate));
      if (left === undefined || right === undefined) return undefined;
      return response[(left - BASE_DOF_COUNT) * jointCount + right - BASE_DOF_COUNT];
    },
  };
}
