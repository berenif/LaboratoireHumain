import type { PlannedContactLoad } from "./contact-loads";
import { CONTACT_SOLVE, solveContactForces, type ContactSolveInput } from "./coordinated-contact-solve";
import { add, cross, scale, sub } from "./math";
import { soleContacts } from "./support-state";

type Result = ReturnType<typeof solveContactForces> & { method: "analytic" | "h78-fallback" };
const dot = (a: number[], b: number[]) => a.reduce((sum, value, i) => sum + value * b[i], 0);

/** Solve the unchanged H78 quadratic when its unconstrained minimizer satisfies
 * every original inequality. Otherwise spend only the remaining original time
 * budget in H78's frozen constrained solver. No actuator or objective changes. */
export function solveContactForcesH79(input: ContactSolveInput,
  fallbackSolve: (input: ContactSolveInput) => ReturnType<typeof solveContactForces> = solveContactForces): Result {
  const cfg = CONTACT_SOLVE, clock = input.clock ?? (() => performance.now());
  const started = clock(), deadline = input.deadlineAt ?? started + cfg.deadlineMs;
  const empty = { loads: [] as PlannedContactLoad[], allocatedForce: { x: 0, y: 0, z: 0 },
    allocatedMoment: { x: 0, y: 0, z: 0 }, forceResidual: { ...input.requestedForce },
    momentResidual: { ...input.requestedMoment }, headroom: 0, iterations: 0,
    constraintViolation: 0, objective: 0, converged: false, solveMs: 0 };
  const finiteResult = (result: ReturnType<typeof solveContactForces>) => [
    ...Object.values(result.allocatedForce), ...Object.values(result.allocatedMoment),
    ...Object.values(result.forceResidual), ...Object.values(result.momentResidual),
    result.headroom, result.constraintViolation, result.objective,
    ...result.loads.flatMap(load => Object.values(load.plannedForce)),
  ].every(Number.isFinite);
  const fallback = (): Result => {
    const result = fallbackSolve({ ...input, clock, deadlineAt: deadline });
    const finished = clock(), solveMs = finished - started;
    if (result.status === "allocated" && !finiteResult(result))
      return { ...empty, solveMs, status: "invalid-state", method: "h78-fallback" };
    return { ...result, solveMs, method: "h78-fallback",
      status: result.status === "allocated" && finished > deadline ? "timeout" : result.status };
  };
  const timedOut = (): Result => ({ ...empty, status: "timeout", solveMs: clock() - started, method: "analytic" });
  const contacts = soleContacts(input.contacts), n = contacts.length * 3;
  // Match H78 validation before considering its fast path, including rejected
  // contacts and selected previous loads: filtering must not hide malformed data.
  const invalid = ![input.weightN, input.momentLengthM, input.maxHorizontalForceN,
    ...Object.values(input.origin), ...Object.values(input.requestedForce), ...Object.values(input.requestedMoment)].every(Number.isFinite)
    || input.weightN <= 0 || input.momentLengthM <= 0 || input.maxHorizontalForceN < 0
    || input.contacts.some(c => !Number.isFinite(c.forceN) || !Number.isFinite(c.measuredForceN ?? c.forceN)
      || !Number.isFinite(c.normalY) || !Object.values(c.point).every(Number.isFinite))
    || input.previous.some(p => contacts.some(c => c.segment === p.segment) && !Object.values(p.force).every(Number.isFinite))
    || new Set(contacts.map(c => c.segment)).size !== contacts.length
    || input.motors.some(r => r.coefficients.length !== n || ![r.baseNm, r.capNm, ...r.coefficients].every(Number.isFinite) || r.capNm <= 0);
  if (invalid || !n) return fallback();
  if (clock() > deadline) return timedOut();

  const wrench = Array.from({ length: 6 }, () => Array(n).fill(0) as number[]);
  contacts.forEach((contact, i) => {
    for (let axis = 0; axis < 3; axis++) {
      wrench[axis][i * 3 + axis] = 1;
      const direction = { x: Number(axis === 0), y: Number(axis === 1), z: Number(axis === 2) };
      const moment = scale(cross(sub(contact.point, input.origin), direction), 1 / input.momentLengthM);
      for (const [k, value] of Object.values(moment).entries()) wrench[k + 3][i * 3 + axis] = value;
    }
  });
  const target = [...Object.values(scale(input.requestedForce, 1 / input.weightN)),
    ...Object.values(scale(input.requestedMoment, 1 / (input.weightN * input.momentLengthM)))];
  const prior = contacts.flatMap(contact => Object.values(scale(
    input.previous.find(p => p.segment === contact.segment)?.force ?? { x: 0, y: 0, z: 0 }, 1 / input.weightN)));
  const motors = input.motors.map(row => ({ a: row.coefficients.map(value => value / row.capNm), b: row.baseNm / row.capNm }));
  const effortRoot = Math.sqrt(cfg.effortRegularization);
  const rows = [...wrench, ...motors.map(row => row.a.map(value => value * effortRoot))];
  const goals = [...target, ...motors.map(row => -row.b * effortRoot)];
  const matrix = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => i === j ? cfg.changeRegularization : 0));
  const rhs = prior.map(value => cfg.changeRegularization * value);
  for (let k = 0; k < rows.length; k++) {
    const row = rows[k];
    for (let i = 0; i < n; i++) {
      rhs[i] += row[i] * goals[k];
      for (let j = 0; j <= i; j++) matrix[i][j] += row[i] * row[j];
    }
  }
  for (let i = 0; i < n; i++) for (let j = 0; j < i; j++) matrix[j][i] = matrix[i][j];
  if (clock() > deadline) return timedOut();

  // The unchanged positive prior regularizer makes this SPD. Reject pivots
  // indistinguishable from roundoff at the actual matrix scale; H78 then handles
  // that input instead of claiming a certified analytical optimum.
  const scaleMatrix = Math.max(...matrix.map((row, i) => row[i]));
  const lower = Array.from({ length: n }, () => Array(n).fill(0) as number[]);
  for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
    let value = matrix[i][j];
    for (let k = 0; k < j; k++) value -= lower[i][k] * lower[j][k];
    if (!Number.isFinite(value) || (i === j && value <= Number.EPSILON * n * scaleMatrix)) return fallback();
    lower[i][j] = i === j ? Math.sqrt(value) : value / lower[j][j];
  }
  const y = Array(n).fill(0) as number[], x = Array(n).fill(0) as number[];
  for (let i = 0; i < n; i++) {
    let value = rhs[i];
    for (let j = 0; j < i; j++) value -= lower[i][j] * y[j];
    y[i] = value / lower[i][i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let value = y[i];
    for (let j = i + 1; j < n; j++) value -= lower[j][i] * x[j];
    x[i] = value / lower[i][i];
  }
  if (!x.every(Number.isFinite)) return fallback();
  // Backward-error certification protects the normal-equation solution without
  // relaxing any physical constraint or changing the frozen objective weights.
  const residual = Math.max(...matrix.map((row, i) => Math.abs(dot(row, x) - rhs[i])));
  const numericalScale = Math.max(1, ...matrix.map((row, i) => row.reduce((sum, value, j) => sum + Math.abs(value * x[j]), Math.abs(rhs[i]))));
  if (!Number.isFinite(residual) || residual > 64 * Number.EPSILON * n * numericalScale) return fallback();
  if (clock() > deadline) return timedOut();

  const loads = contacts.map((contact, i): PlannedContactLoad => ({ segment: contact.segment, point: { ...contact.point },
    plannedForce: { x: x[i * 3] * input.weightN, y: x[i * 3 + 1] * input.weightN, z: x[i * 3 + 2] * input.weightN },
    measuredPoint: { ...contact.point }, measuredForceN: contact.forceN }));
  const allocatedForce = loads.reduce((sum, load) => add(sum, load.plannedForce), { x: 0, y: 0, z: 0 });
  const allocatedMoment = loads.reduce((sum, load) => add(sum, cross(sub(load.point, input.origin), load.plannedForce)), { x: 0, y: 0, z: 0 });
  const utilization = Math.max(0, ...motors.map(row => Math.abs(row.b + dot(row.a, x))));
  const violation = Math.max(0, utilization - 1, allocatedForce.y / input.weightN - cfg.maximumVerticalWeight,
    Math.hypot(allocatedForce.x, allocatedForce.z) / input.weightN - input.maxHorizontalForceN / input.weightN,
    ...loads.flatMap(load => [-load.plannedForce.y / input.weightN,
      (Math.hypot(load.plannedForce.x, load.plannedForce.z) - cfg.frictionCoefficient * load.plannedForce.y) / input.weightN]));
  const result = { loads, allocatedForce, allocatedMoment, forceResidual: sub(input.requestedForce, allocatedForce),
    momentResidual: sub(input.requestedMoment, allocatedMoment), headroom: 1 - utilization, iterations: 1,
    constraintViolation: violation, converged: true,
    objective: rows.reduce((sum, row, k) => sum + (dot(row, x) - goals[k]) ** 2, 0)
      + cfg.changeRegularization * x.reduce((sum, value, i) => sum + (value - prior[i]) ** 2, 0),
    solveMs: 0, status: "allocated" as const };
  if (!finiteResult(result) || violation > cfg.constraintTolerance) return fallback();
  const finished = clock();
  return { ...result, solveMs: finished - started, status: finished > deadline ? "timeout" : "allocated", method: "analytic" };
}
