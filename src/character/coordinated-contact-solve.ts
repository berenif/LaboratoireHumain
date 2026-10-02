import type { JointCoordinate, SegmentId, SupportingContact, Vec3 } from "../core/types";
import type { PlannedContactLoad } from "./contact-loads";
import { add, cross, scale, sub } from "./math";
import { soleContacts, type SupportState } from "./support-state";

export const CONTACT_SOLVE = Object.freeze({ iterations: 64, projectionSweeps: 24,
  constraintTolerance: 0.00001, convergenceTolerance: 0.000001,
  changeRegularization: 0.02, effortRegularization: 0.02, frictionCoefficient: 1.2,
  maximumVerticalWeight: 1.35, deadlineMs: 8 });
export interface ContactMotorRow {
  id: SegmentId; coordinate: JointCoordinate; baseNm: number; capNm: number;
  /** Nm per bodyweight of each x/y/z contact component, already regularized. */
  coefficients: number[];
}
export interface ContactSolveInput {
  contacts: readonly SupportingContact[]; origin: Vec3; requestedForce: Vec3; requestedMoment: Vec3;
  weightN: number; momentLengthM: number; maxHorizontalForceN: number;
  previous: SupportState["previousAllocation"]; motors: ContactMotorRow[];
  clock?: () => number; deadlineAt?: number;
}
const dotArray = (a: number[], b: number[]) => a.reduce((sum, v, i) => sum + v * b[i], 0);

/** Bounded projected least squares over measured sole resultants. All motor
 * halfspaces include posture before projection. No prescribed normal shares. */
export function solveContactForces(input: ContactSolveInput) {
  const cfg = CONTACT_SOLVE, clock = input.clock ?? (() => performance.now());
  const started = clock(), deadline = input.deadlineAt ?? started + cfg.deadlineMs;
  const contacts = soleContacts(input.contacts), n = contacts.length * 3;
  const invalid = ![input.weightN, input.momentLengthM, input.maxHorizontalForceN,
    ...Object.values(input.origin), ...Object.values(input.requestedForce), ...Object.values(input.requestedMoment)].every(Number.isFinite)
    || input.weightN <= 0 || input.momentLengthM <= 0 || input.maxHorizontalForceN < 0
    || input.contacts.some(c => !Number.isFinite(c.forceN) || !Number.isFinite(c.measuredForceN ?? c.forceN) || !Number.isFinite(c.normalY)
      || !Object.values(c.point).every(Number.isFinite))
    || input.previous.some(p => contacts.some(c => c.segment === p.segment) && !Object.values(p.force).every(Number.isFinite))
    || new Set(contacts.map(c => c.segment)).size !== contacts.length
    || input.motors.some(r => r.coefficients.length !== n || ![r.baseNm, r.capNm, ...r.coefficients].every(Number.isFinite) || r.capNm <= 0);
  const empty = { loads: [] as PlannedContactLoad[], allocatedForce: { x: 0, y: 0, z: 0 },
    allocatedMoment: { x: 0, y: 0, z: 0 }, forceResidual: { ...input.requestedForce },
    momentResidual: { ...input.requestedMoment }, headroom: 0, iterations: 0,
    constraintViolation: 0, objective: 0, converged: false, solveMs: 0 };
  if (invalid) return { ...empty, status: "invalid-state" as const };
  if (!n) return { ...empty, status: "invalid-contact" as const };
  const wrench = Array.from({ length: 6 }, () => Array(n).fill(0) as number[]);
  contacts.forEach((c, i) => {
    for (let a = 0; a < 3; a++) {
      wrench[a][i * 3 + a] = 1;
      const axis = { x: Number(a === 0), y: Number(a === 1), z: Number(a === 2) };
      const moment = scale(cross(sub(c.point, input.origin), axis), 1 / input.momentLengthM);
      for (const [k, value] of Object.values(moment).entries()) wrench[k + 3][i * 3 + a] = value;
    }
  });
  const target = [...Object.values(scale(input.requestedForce, 1 / input.weightN)),
    ...Object.values(scale(input.requestedMoment, 1 / (input.weightN * input.momentLengthM)))];
  const prior = contacts.flatMap(c => Object.values(scale(
    input.previous.find(p => p.segment === c.segment)?.force ?? { x: 0, y: 0, z: 0 }, 1 / input.weightN)));
  const motors = input.motors.map(r => {
    const a = r.coefficients.map(v => v / r.capNm);
    return { a, b: r.baseNm / r.capNm, norm: dotArray(a, a) };
  });
  const rows = [...wrench, ...motors.map(r => r.a.map(v => v * Math.sqrt(cfg.effortRegularization)))];
  const goals = [...target, ...motors.map(r => -r.b * Math.sqrt(cfg.effortRegularization))];
  const lipschitz = rows.reduce<number>((sum, r) => sum + dotArray(r, r), cfg.changeRegularization);
  const horizontalCap = input.maxHorizontalForceN / input.weightN;
  const project = (values: number[]) => {
    // Cyclic convex projections, bounded independently of the objective solve.
    for (let sweep = 0; sweep < cfg.projectionSweeps; sweep++) {
      let changed = false;
      for (let i = 0; i < n; i += 3) {
        const horizontal = Math.hypot(values[i], values[i + 2]);
        const normal = values[i + 1], mu = cfg.frictionCoefficient;
        if (normal < 0 || horizontal > mu * normal) {
          changed = true;
          const projected = Math.max(0, (normal + mu * horizontal) / (1 + mu * mu));
          const ratio = horizontal > 0 ? Math.min(1, mu * projected / horizontal) : 0;
          values[i] *= ratio; values[i + 2] *= ratio; values[i + 1] = projected;
        }
      }
      const vertical = contacts.reduce((sum, _, i) => sum + values[i * 3 + 1], 0);
      if (vertical > cfg.maximumVerticalWeight) {
        changed = true;
        const excess = (vertical - cfg.maximumVerticalWeight) / contacts.length;
        for (let i = 1; i < n; i += 3) values[i] -= excess;
      }
      const hx = contacts.reduce((sum, _, i) => sum + values[i * 3], 0);
      const hz = contacts.reduce((sum, _, i) => sum + values[i * 3 + 2], 0);
      const h = Math.hypot(hx, hz);
      if (h > horizontalCap) for (let i = 0; i < n; i += 3) {
        changed = true;
        values[i] -= hx * (1 - horizontalCap / h) / contacts.length;
        values[i + 2] -= hz * (1 - horizontalCap / h) / contacts.length;
      }
      for (const motor of motors) {
        if (motor.norm <= 1e-20) continue;
        const value = motor.b + dotArray(motor.a, values);
        if (Math.abs(value) > 1) {
          changed = true;
          const excess = (value - Math.max(-1, Math.min(1, value))) / motor.norm;
          for (let j = 0; j < n; j++) values[j] -= excess * motor.a[j];
        }
      }
      if (!changed) break; // Remaining projections would be exact identities.
    }
    return values;
  };
  let x = project([...prior]), iterations = 0, converged = false;
  for (; iterations < cfg.iterations; iterations++) {
    if (clock() > deadline) return { ...empty, status: "timeout" as const, iterations, solveMs: clock() - started };
    const gradient = x.map((v, i) => cfg.changeRegularization * (v - prior[i]));
    rows.forEach((r, k) => {
      const residual = dotArray(r, x) - goals[k];
      for (let j = 0; j < n; j++) gradient[j] += r[j] * residual;
    });
    const next = project(x.map((v, i) => v - gradient[i] / lipschitz));
    const change = Math.max(...next.map((v, i) => Math.abs(v - x[i])));
    x = next;
    if (change <= cfg.convergenceTolerance) { converged = true; iterations++; break; }
  }
  const loads = contacts.map((c, i): PlannedContactLoad => ({ segment: c.segment, point: { ...c.point },
    plannedForce: { x: x[i * 3] * input.weightN, y: x[i * 3 + 1] * input.weightN, z: x[i * 3 + 2] * input.weightN },
    measuredPoint: { ...c.point }, measuredForceN: c.forceN }));
  const allocatedForce = loads.reduce((sum, c) => add(sum, c.plannedForce), { x: 0, y: 0, z: 0 });
  const allocatedMoment = loads.reduce((sum, c) => add(sum, cross(sub(c.point, input.origin), c.plannedForce)), { x: 0, y: 0, z: 0 });
  const utilization = Math.max(0, ...motors.map(r => Math.abs(r.b + dotArray(r.a, x))));
  const violation = Math.max(0, utilization - 1, allocatedForce.y / input.weightN - cfg.maximumVerticalWeight,
    Math.hypot(allocatedForce.x, allocatedForce.z) / input.weightN - horizontalCap,
    ...loads.flatMap(c => [-c.plannedForce.y / input.weightN,
      (Math.hypot(c.plannedForce.x, c.plannedForce.z) - cfg.frictionCoefficient * c.plannedForce.y) / input.weightN]));
  const solveMs = clock() - started;
  if (![...x, ...Object.values(allocatedForce), ...Object.values(allocatedMoment), violation, utilization].every(Number.isFinite))
    return { ...empty, status: "invalid-state" as const, iterations, solveMs };
  return { loads, allocatedForce, allocatedMoment, forceResidual: sub(input.requestedForce, allocatedForce),
    momentResidual: sub(input.requestedMoment, allocatedMoment), headroom: 1 - utilization,
    iterations, constraintViolation: violation, converged,
    objective: rows.reduce((sum, r, k) => sum + (dotArray(r, x) - goals[k]) ** 2, 0)
      + cfg.changeRegularization * x.reduce((sum, v, i) => sum + (v - prior[i]) ** 2, 0),
    solveMs, status: solveMs + started > deadline ? "timeout" as const
      : violation > cfg.constraintTolerance ? "infeasible" as const : "allocated" as const };
}
