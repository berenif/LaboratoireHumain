import { SEGMENT_BY_ID, TOTAL_MASS_KG } from "../core/humanoid";
import type { SegmentId, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, clampLength, dot, scale, sub } from "./math";

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const HORIZONTAL = (v: Vec3): Vec3 => ({ x: v.x, y: 0, z: v.z });

export interface PlannedContactLoad {
  segment: SegmentId;
  point: Vec3;
  plannedForce: Vec3;
  measuredForceN: number;
}

export interface ContactLoadPlan {
  loads: PlannedContactLoad[];
  requestedForce: Vec3;
  allocatedForce: Vec3;
  pressurePoint: Vec3;
}

export interface ContactLoadOptions {
  excluded?: ReadonlySet<SegmentId>;
  frictionCoefficient: number;
  maxHorizontalForceN: number;
  maxJointTorqueNm: number;
  projectionSeconds?: number;
  /** Retain this fraction of each eligible patch's measured share during transfer. */
  minimumMeasuredShareFraction?: number;
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
  const horizontal = clampLength(HORIZONTAL(requestedForce),
    Math.min(frictionLimit, Math.max(0, options.maxHorizontalForceN), torqueLimit));
  const force = { x: horizontal.x, y: vertical, z: horizontal.z };
  const capture = add(HORIZONTAL(centerOfMass), scale(HORIZONTAL(centerOfMassVelocity),
    options.projectionSeconds ?? 0.15));
  const desiredPressure = sub(capture, scale(horizontal, height / Math.max(vertical, 1)));
  const points = valid.map(contact => {
    const patch = contact.points?.length ? contact.points : [contact.point];
    return scale(patch.reduce(add, ZERO), 1 / patch.length);
  });
  const measuredTotal = valid.reduce((sum, contact) => sum + contact.forceN, 0);
  const measuredShare = valid.map(contact => contact.forceN / measuredTotal);
  const retainedFraction = clamp(options.minimumMeasuredShareFraction ?? 0, 0, 1);
  const minimumShare = measuredShare.map(share => retainedFraction * share);
  let shares = [...measuredShare];
  // Project the pressure centre toward COM + momentum while retaining the
  // measured load split when several patches can supply the same moment.
  for (let iteration = 0; iteration < 16 && shares.length > 1; iteration += 1) {
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
  const loads = valid.map((contact, index) => ({
    segment: contact.segment,
    point: points[index],
    plannedForce: scale(force, shares[index]),
    measuredForceN: contact.forceN,
  }));
  const pressurePoint = points.reduce((sum, point, index) => add(sum, scale(point, shares[index])), ZERO);
  const allocatedForce = loads.reduce((sum, load) => add(sum, load.plannedForce), ZERO);
  return { loads, requestedForce, allocatedForce, pressurePoint };
}

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
