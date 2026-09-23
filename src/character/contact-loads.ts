import { SEGMENT_BY_ID, TOTAL_MASS_KG } from "../core/humanoid";
import type { SegmentId, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, clampLength, dot, scale, sub } from "./math";
import { distributeSupportLoad } from "./support-loads";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const HORIZONTAL = (v: Vec3): Vec3 => ({ x: v.x, y: 0, z: v.z });

export interface PlannedContactLoad {
  segment: SegmentId;
  point: Vec3;
  plannedForce: Vec3;
  measuredForceN: number;
  measuredPoint: Vec3;
}

export interface ContactLoadPlan {
  loads: PlannedContactLoad[];
  requestedForce: Vec3;
  allocatedForce: Vec3;
  pressurePoint: Vec3;
  pressureFeasible?: boolean;
  pressureForceResidualNm?: number;
}

export interface ContactLoadOptions {
  excluded?: ReadonlySet<SegmentId>;
  frictionCoefficient: number;
  maxHorizontalForceN: number;
  maxJointTorqueNm: number;
  projectionSeconds?: number;
  /** Retain this fraction of each eligible patch's measured share during transfer. */
  minimumMeasuredShareFraction?: number;
  /** Solve pressure on the full measured hull for standing balance transfers. */
  projectMeasuredPressure?: boolean;
  /** Bounded hip/torso action may request a nonzero centroidal moment. */
  allowSupportMoment?: boolean;
  /** Normal-load transfer target, independently of horizontal braking intent. */
  pressureTarget?: Vec3;
  retainedSide?: "left" | "right";
}

/** Allocate a bounded wrench only over loaded, measured solver patches. */
export function planContactLoads(
  contacts: readonly SupportingContact[],
  centerOfMass: Vec3,
  centerOfMassVelocity: Vec3,
  requestedForce: Vec3,
  options: ContactLoadOptions,
): ContactLoadPlan {
  const valid = contacts.filter(contact => contact.loadBearing && contact.forceN > 0
    && contact.normalY >= 0.65 && !options.excluded?.has(contact.segment));
  const empty = { loads: [] as PlannedContactLoad[], requestedForce, allocatedForce: ZERO,
    pressurePoint: HORIZONTAL(centerOfMass) };
  if (!valid.length) return empty;

  const vertical = clamp(requestedForce.y, 0, TOTAL_MASS_KG * 9.81 * 1.35);
  const height = Math.max(0.3, centerOfMass.y - Math.min(...valid.map(contact => contact.point.y)));
  const frictionLimit = Math.max(0, options.frictionCoefficient) * vertical;
  const torqueLimit = Math.max(0, options.maxJointTorqueNm) / height;
  const horizontalLimit = Math.min(frictionLimit, Math.max(0, options.maxHorizontalForceN), torqueLimit);
  const horizontal = clampLength(HORIZONTAL(requestedForce), horizontalLimit);
  let force = { x: horizontal.x, y: vertical, z: horizontal.z };
  // Wrench compatibility is about the current physical COM. Momentum already
  // enters the requested acceleration; projecting this moment origin can
  // turn a braking request into forward force inside the measured footprint.
  const capture = add(HORIZONTAL(centerOfMass), scale(HORIZONTAL(centerOfMassVelocity),
    options.projectMeasuredPressure ? 0 : options.projectionSeconds ?? 0.15));
  let desiredPressure = options.pressureTarget
    ?? sub(capture, scale(horizontal, height / Math.max(vertical, 1)));
  const retained = options.retainedSide
    ? valid.filter(contact => SEGMENT_BY_ID.get(contact.segment)?.side === options.retainedSide) : [];
  const preferredSegments = new Set(retained.map(contact => contact.segment));
  if (retained.length) desiredPressure = distributeSupportLoad(retained, desiredPressure,
    { rotationInvariant: true }).reduce((sum, load) => add(sum, scale(load.point, load.share)), ZERO);
  const measuredTotal = valid.reduce((sum, contact) => sum + contact.forceN, 0);
  // A segment's reported contact point is the centroid of its solver manifold.
  // Using only that centroid makes a single loaded hindfoot unable to request
  // heel pressure even when Rapier reports real heel contact points. Allocate
  // within each measured manifold, then return its equivalent resultant so
  // downstream joint moments and per-segment load reports retain their shape.
  const candidates = valid.flatMap((contact, contactIndex) => {
    const patch = contact.points?.length ? contact.points : [contact.point];
    const measuredShare = contact.forceN / measuredTotal / patch.length;
    return patch.map(point => ({ contactIndex, point, measuredShare }));
  });
  const points = candidates.map(candidate => candidate.point);
  const measuredShare = candidates.map(candidate => candidate.measuredShare);
  const retainedFraction = clamp(options.minimumMeasuredShareFraction ?? 0, 0, 1);
  const minimumShare = measuredShare.map(share => retainedFraction * share);
  let shares = [...measuredShare];
  let pressureFeasible = true;
  if (options.projectMeasuredPressure) {
    const project = (point: Vec3) => distributeSupportLoad(valid, point);
    const center = (loads: ReturnType<typeof project>) => loads.reduce((sum, load) =>
      add(sum, scale(load.point, load.share)), ZERO);
    let pressure = center(project(desiredPressure));
    const radius = horizontalLimit * height / Math.max(vertical, 1);
    const nearest = center(project(capture));
    const inside = (point: Vec3) => lengthSquared(HORIZONTAL(sub(point, capture))) <= radius * radius + 1e-12;
    pressureFeasible = inside(nearest);
    if (!options.allowSupportMoment && !pressureFeasible) pressure = nearest;
    else if (!options.allowSupportMoment && !inside(pressure)) {
      // Intersect the measured hull with the force-budget disk. The segment
      // remains in the convex hull, and its feasible prefix is monotone.
      let low = 0, high = 1;
      for (let i = 0; i < 32; i++) {
        const fraction = (low + high) / 2;
        const candidate = add(nearest, scale(sub(pressure, nearest), fraction));
        if (inside(candidate)) low = fraction; else high = fraction;
      }
      pressure = add(nearest, scale(sub(pressure, nearest), low));
    }
    const distribution = distributeSupportLoad(valid, pressure, { rotationInvariant: true, preferredSegments });
    shares = candidates.map(() => 0);
    for (const load of distribution) {
      const index = candidates.findIndex(candidate =>
        load.segment === valid[candidate.contactIndex].segment
        && lengthSquared(sub(load.point, candidate.point)) < 1e-16);
      if (index < 0) throw new Error("Projected pressure lost its measured contact owner");
      shares[index] += load.share;
    }
    shares = shares.map((share, index) => minimumShare[index] + (1 - retainedFraction) * share);
  }
  // Project the pressure centre toward COM + momentum while retaining the
  // measured load split when several patches can supply the same moment.
  for (let iteration = 0; !options.projectMeasuredPressure && iteration < 16 && shares.length > 1; iteration += 1) {
    const center = points.reduce((sum, point, index) => add(sum, scale(point, shares[index])), ZERO);
    const error = HORIZONTAL(sub(desiredPressure, center));
    const updated = shares.map((share, index) => Math.max(minimumShare[index], share
      + 1.8 * dot(HORIZONTAL(sub(points[index], center)), error)
      + 0.045 * (measuredShare[index] - share)));
    const excess = updated.map((share, index) => share - minimumShare[index]);
    const total = excess.reduce((sum, share) => sum + share, 0);
    shares = total > 1e-9
      ? excess.map((share, index) => minimumShare[index] + (1 - retainedFraction) * share / total)
      : [...measuredShare];
  }
  const pressurePoint = points.reduce((sum, point, index) => add(sum, scale(point, shares[index])), ZERO);
  const compatibleHorizontal = scale(HORIZONTAL(sub(capture, pressurePoint)), vertical / height);
  if (options.projectMeasuredPressure && !options.allowSupportMoment) {
    const bounded = clampLength(compatibleHorizontal, horizontalLimit);
    force = { x: bounded.x, y: vertical, z: bounded.z };
  }
  const pressureForceResidualNm = Math.sqrt(lengthSquared(sub(compatibleHorizontal, HORIZONTAL(force)))) * height;
  pressureFeasible &&= pressureForceResidualNm < 1e-6;
  const loads = valid.map((contact, contactIndex) => {
    const share = candidates.reduce((sum, candidate, index) =>
      sum + (candidate.contactIndex === contactIndex ? shares[index] : 0), 0);
    const weightedPoint = candidates.reduce((sum, candidate, index) =>
      candidate.contactIndex === contactIndex
        ? add(sum, scale(candidate.point, shares[index])) : sum, ZERO);
    // This point is the equivalent moment of forces on the measured manifold,
    // not an additional floor contact or an independent source of support.
    return {
      segment: contact.segment,
      point: share > 1e-9 ? scale(weightedPoint, 1 / share) : { ...contact.point },
      plannedForce: scale(force, share),
      measuredForceN: contact.forceN,
      measuredPoint: { ...(contact.measuredPressurePoint ?? contact.point) },
    };
  });
  const allocatedForce = loads.reduce((sum, load) => add(sum, load.plannedForce), ZERO);
  return { loads, requestedForce, allocatedForce, pressurePoint,
    pressureFeasible, pressureForceResidualNm };
}

function lengthSquared(vector: Vec3): number { return dot(vector, vector); }

/** Combined moment budget of the actuated joints above a contact. */
export function supportChainTorqueLimit(segment: SegmentId): number {
  let current: SegmentId | null = segment;
  let limit = 0;
  while (current) {
    const definition = SEGMENT_BY_ID.get(current);
    if (!definition) break;
    const axes = definition.jointProfile?.axes ?? [];
    if (axes.length) limit += Math.max(...axes.map(axis => axis.maxMotorTorqueNm));
    current = definition.parent;
  }
  return limit;
}

export function minimumSupportTorqueLimit(contacts: readonly SupportingContact[]): number {
  const countedJoints = new Set<SegmentId>();
  let limit = 0;
  for (const contact of contacts) {
    if (!contact.loadBearing || contact.forceN <= 0) continue;
    let current: SegmentId | null = contact.segment;
    while (current) {
      const definition = SEGMENT_BY_ID.get(current);
      if (!definition) break;
      if (!countedJoints.has(current) && definition.jointProfile?.axes.length) {
        limit += Math.max(...definition.jointProfile.axes.map(axis => axis.maxMotorTorqueNm));
        countedJoints.add(current);
      }
      current = definition.parent;
    }
  }
  return limit;
}
