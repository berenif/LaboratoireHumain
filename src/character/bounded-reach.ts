import type { Vec3 } from "../core/types";
import { clamp, dot, length, sub } from "./math";

/** Refine a geometric seed while keeping every anatomical coordinate bounded.
 * This changes a kinematic motor target only, never a physical body's pose.
 */
export function refineBoundedReach(
  initial: readonly number[],
  bounds: readonly (readonly [number, number])[],
  target: Vec3,
  positionAt: (coordinates: readonly number[]) => Vec3,
): number[] {
  let values = initial.map((v, i) => clamp(v, bounds[i][0], bounds[i][1]));
  let error = sub(target, positionAt(values));
  const h = 1e-4;
  for (let iteration = 0; iteration < 16 && length(error) > 0.0005; iteration++) {
    const columns = values.map((v, i) => {
      const plus = [...values], minus = [...values];
      plus[i] = clamp(v + h, bounds[i][0], bounds[i][1]);
      minus[i] = clamp(v - h, bounds[i][0], bounds[i][1]);
      const difference = sub(positionAt(plus), positionAt(minus));
      const interval = plus[i] - minus[i];
      return interval > 0 ? { x: difference.x / interval, y: difference.y / interval, z: difference.z / interval }
        : { x: 0, y: 0, z: 0 };
    });
    // Damped least squares Jᵀ (J Jᵀ + λ² I)⁻¹ e. The 1 cm length
    // regularization keeps near-straight arms finite without widening limits.
    const axes = ["x", "y", "z"] as const, free = columns.map(() => true);
    let delta = values.map(() => 0);
    for (let activeSet = 0; activeSet <= values.length; activeSet++) {
      const matrix = axes.map((a, i) => axes.map((b, j) => columns.reduce((sum, c, k) => sum + (free[k] ? c[a] * c[b] : 0), 0)
        + (i === j ? 0.0001 : 0)));
      const rhs = [error.x, error.y, error.z];
      for (let k = 0; k < 3; k++) {
        for (let i = k + 1; i < 3; i++) {
          const ratio = matrix[i][k] / matrix[k][k];
          for (let j = k; j < 3; j++) matrix[i][j] -= ratio * matrix[k][j];
          rhs[i] -= ratio * rhs[k];
        }
      }
      const solution = [0, 0, 0];
      for (let i = 2; i >= 0; i--) {
        solution[i] = (rhs[i] - matrix[i].reduce((sum, v, j) => sum + (j > i ? v * solution[j] : 0), 0)) / matrix[i][i];
      }
      delta = columns.map((c, i) => free[i] ? c.x * solution[0] + c.y * solution[1] + c.z * solution[2] : 0);
      let blocked = false;
      for (let i = 0; i < values.length; i++) {
        if (free[i] && ((values[i] <= bounds[i][0] + 1e-10 && delta[i] < 0)
          || (values[i] >= bounds[i][1] - 1e-10 && delta[i] > 0))) {
          free[i] = false; blocked = true;
        }
      }
      // Re-solve without coordinates that would push farther through a limit;
      // clipping an unconstrained DLS step alone can stall the remaining joints.
      if (!blocked) break;
    }
    const maximum = Math.max(...delta.map(Math.abs));
    const stepScale = maximum > 0.25 ? 0.25 / maximum : 1;
    let improved = false;
    for (const lineScale of [1, 0.5, 0.25, 0.125]) {
      const candidate = values.map((v, i) => clamp(v + delta[i] * stepScale * lineScale, bounds[i][0], bounds[i][1]));
      const next = positionAt(candidate), nextError = sub(target, next);
      if (dot(nextError, nextError) < dot(error, error) - 1e-14) {
        values = candidate; error = nextError; improved = true; break;
      }
    }
    if (!improved) break;
  }
  return values;
}
