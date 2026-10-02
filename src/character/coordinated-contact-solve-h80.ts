import type { PlannedContactLoad } from "./contact-loads";
import { CONTACT_SOLVE, type ContactSolveInput, type solveContactForces } from "./coordinated-contact-solve";
import { solveContactForcesH79 } from "./coordinated-contact-solve-h79";
import { add, cross, scale, sub } from "./math";
import { soleContacts } from "./support-state";

interface SparseRow { indices: number[]; values: number[]; norm: number; }
function sparse(row: number[]): SparseRow {
  const indices: number[] = [], values: number[] = []; let norm = 0;
  for (let i = 0; i < row.length; i++) {
    const value = row[i]; norm += value * value;
    if (value !== 0) { indices.push(i); values.push(value); }
  }
  return { indices, values, norm };
}
function product(row: SparseRow, vector: Float64Array): number {
  let result = 0;
  for (let i = 0; i < row.indices.length; i++) result += row.values[i] * vector[row.indices[i]];
  return result;
}

/** H78's exact row/gradient/projection sequence, with invariant sparse rows
 * precomputed and reusable numerical buffers instead of per-iteration arrays.
 * Skipping zero products preserves the order of every nonzero accumulation. */
export function solveContactForcesH80Constrained(input: ContactSolveInput): ReturnType<typeof solveContactForces> {
  const cfg = CONTACT_SOLVE, clock = input.clock ?? (() => performance.now());
  const started = clock(), deadline = input.deadlineAt ?? started + cfg.deadlineMs;
  const contacts = soleContacts(input.contacts), n = contacts.length * 3;
  const invalid = ![input.weightN, input.momentLengthM, input.maxHorizontalForceN,
    ...Object.values(input.origin), ...Object.values(input.requestedForce), ...Object.values(input.requestedMoment)].every(Number.isFinite)
    || input.weightN <= 0 || input.momentLengthM <= 0 || input.maxHorizontalForceN < 0
    || input.contacts.some(c => !Number.isFinite(c.forceN) || !Number.isFinite(c.measuredForceN ?? c.forceN)
      || !Number.isFinite(c.normalY) || !Object.values(c.point).every(Number.isFinite))
    || input.previous.some(p => contacts.some(c => c.segment === p.segment) && !Object.values(p.force).every(Number.isFinite))
    || new Set(contacts.map(c => c.segment)).size !== contacts.length
    || input.motors.some(r => r.coefficients.length !== n || ![r.baseNm, r.capNm, ...r.coefficients].every(Number.isFinite) || r.capNm <= 0);
  const empty = { loads: [] as PlannedContactLoad[], allocatedForce: { x: 0, y: 0, z: 0 },
    allocatedMoment: { x: 0, y: 0, z: 0 }, forceResidual: { ...input.requestedForce },
    momentResidual: { ...input.requestedMoment }, headroom: 0, iterations: 0,
    constraintViolation: 0, objective: 0, converged: false, solveMs: 0 };
  if (invalid) return { ...empty, status: "invalid-state" };
  if (!n) return { ...empty, status: "invalid-contact" };

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
  const prior = Float64Array.from(contacts.flatMap(contact => Object.values(scale(
    input.previous.find(p => p.segment === contact.segment)?.force ?? { x: 0, y: 0, z: 0 }, 1 / input.weightN))));
  const motors = input.motors.map(row => ({ ...sparse(row.coefficients.map(value => value / row.capNm)), b: row.baseNm / row.capNm }));
  const effortRoot = Math.sqrt(cfg.effortRegularization);
  const rows = [...wrench.map(sparse), ...motors.map(row => {
    const values = row.values.map(value => value * effortRoot);
    let norm = 0; for (const value of values) norm += value * value;
    return { indices: row.indices, values, norm };
  })];
  const goals = [...target, ...motors.map(row => -row.b * effortRoot)];
  let lipschitz = cfg.changeRegularization;
  for (const row of rows) lipschitz += row.norm;
  const horizontalCap = input.maxHorizontalForceN / input.weightN;
  const project = (values: Float64Array) => {
    for (let sweep = 0; sweep < cfg.projectionSweeps; sweep++) {
      let changed = false;
      for (let i = 0; i < n; i += 3) {
        const horizontal = Math.hypot(values[i], values[i + 2]), normal = values[i + 1], mu = cfg.frictionCoefficient;
        if (normal < 0 || horizontal > mu * normal) {
          changed = true;
          const projected = Math.max(0, (normal + mu * horizontal) / (1 + mu * mu));
          const ratio = horizontal > 0 ? Math.min(1, mu * projected / horizontal) : 0;
          values[i] *= ratio; values[i + 2] *= ratio; values[i + 1] = projected;
        }
      }
      let vertical = 0;
      for (let i = 1; i < n; i += 3) vertical += values[i];
      if (vertical > cfg.maximumVerticalWeight) {
        changed = true;
        const excess = (vertical - cfg.maximumVerticalWeight) / contacts.length;
        for (let i = 1; i < n; i += 3) values[i] -= excess;
      }
      let hx = 0, hz = 0;
      for (let i = 0; i < n; i += 3) { hx += values[i]; hz += values[i + 2]; }
      const horizontal = Math.hypot(hx, hz);
      if (horizontal > horizontalCap) {
        const dx = hx * (1 - horizontalCap / horizontal) / contacts.length;
        const dz = hz * (1 - horizontalCap / horizontal) / contacts.length;
        for (let i = 0; i < n; i += 3) { changed = true; values[i] -= dx; values[i + 2] -= dz; }
      }
      for (const motor of motors) {
        if (motor.norm <= 1e-20) continue;
        const value = motor.b + product(motor, values);
        if (Math.abs(value) > 1) {
          changed = true;
          const excess = (value - Math.max(-1, Math.min(1, value))) / motor.norm;
          for (let i = 0; i < motor.indices.length; i++) values[motor.indices[i]] -= excess * motor.values[i];
        }
      }
      if (!changed) break;
    }
    return values;
  };
  let x: Float64Array = project(prior.slice()), next: Float64Array = new Float64Array(n);
  let iterations = 0, converged = false;
  const gradient = new Float64Array(n);
  for (; iterations < cfg.iterations; iterations++) {
    if (clock() > deadline) return { ...empty, status: "timeout", iterations, solveMs: clock() - started };
    for (let j = 0; j < n; j++) gradient[j] = cfg.changeRegularization * (x[j] - prior[j]);
    for (let k = 0; k < rows.length; k++) {
      const row = rows[k], residual = product(row, x) - goals[k];
      for (let i = 0; i < row.indices.length; i++) gradient[row.indices[i]] += row.values[i] * residual;
    }
    for (let j = 0; j < n; j++) next[j] = x[j] - gradient[j] / lipschitz;
    project(next);
    let change = 0;
    for (let j = 0; j < n; j++) change = Math.max(change, Math.abs(next[j] - x[j]));
    const swap = x; x = next; next = swap;
    if (change <= cfg.convergenceTolerance) { converged = true; iterations++; break; }
  }
  const loads = contacts.map((contact, i): PlannedContactLoad => ({ segment: contact.segment, point: { ...contact.point },
    plannedForce: { x: x[i * 3] * input.weightN, y: x[i * 3 + 1] * input.weightN, z: x[i * 3 + 2] * input.weightN },
    measuredPoint: { ...contact.point }, measuredForceN: contact.forceN }));
  const allocatedForce = loads.reduce((sum, load) => add(sum, load.plannedForce), { x: 0, y: 0, z: 0 });
  const allocatedMoment = loads.reduce((sum, load) => add(sum, cross(sub(load.point, input.origin), load.plannedForce)), { x: 0, y: 0, z: 0 });
  let utilization = 0;
  for (const motor of motors) utilization = Math.max(utilization, Math.abs(motor.b + product(motor, x)));
  const violation = Math.max(0, utilization - 1, allocatedForce.y / input.weightN - cfg.maximumVerticalWeight,
    Math.hypot(allocatedForce.x, allocatedForce.z) / input.weightN - horizontalCap,
    ...loads.flatMap(load => [-load.plannedForce.y / input.weightN,
      (Math.hypot(load.plannedForce.x, load.plannedForce.z) - cfg.frictionCoefficient * load.plannedForce.y) / input.weightN]));
  const solveMs = clock() - started;
  if (![...x, ...Object.values(allocatedForce), ...Object.values(allocatedMoment), violation, utilization].every(Number.isFinite))
    return { ...empty, status: "invalid-state", iterations, solveMs };
  let objective = 0, previousObjective = 0;
  for (let k = 0; k < rows.length; k++) objective += (product(rows[k], x) - goals[k]) ** 2;
  for (let i = 0; i < n; i++) previousObjective += (x[i] - prior[i]) ** 2;
  objective += cfg.changeRegularization * previousObjective;
  return { loads, allocatedForce, allocatedMoment, forceResidual: sub(input.requestedForce, allocatedForce),
    momentResidual: sub(input.requestedMoment, allocatedMoment), headroom: 1 - utilization, iterations,
    constraintViolation: violation, converged, objective, solveMs,
    status: solveMs + started > deadline ? "timeout" : violation > cfg.constraintTolerance ? "infeasible" : "allocated" };
}

/** H79's certified analytical path followed by the optimized identical H78
 * constrained sequence, sharing the original absolute deadline throughout. */
export function solveContactForcesH80(input: ContactSolveInput) {
  const result = solveContactForcesH79(input, solveContactForcesH80Constrained);
  return { ...result, method: result.method === "analytic" ? "analytic" as const : "h80-constrained" as const };
}
