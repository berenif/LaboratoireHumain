import { SEGMENT_BY_ID } from "../core/humanoid";
import type { Quat, SupportingContact, Vec3 } from "../core/types";
import { legTargetReach, type HindfootId } from "./leg-target-frame";
import { add, clampLength, dot, length, quatFromAxisAngle, rotate, scale, sub, worldPoint } from "./math";
import { composeTouchdownHindfoot, horizontal } from "./pose";
import { recoverySupportHull, recoverySupportMargin } from "./recovery-support";

export interface LandingInput {
  foot: HindfootId;
  from: Vec3;
  pelvis: { position: Vec3; rotation: Quat };
  heading: number;
  requested: Vec3;
  retainedFoot: Vec3;
  floorY: number;
  surfaceHeight?: (x: number, z: number) => number;
  maxTravelM: number;
  maxReachM: number;
  capturePoint?: Vec3;
  measuredMomentumCapturePoint?: Vec3;
  retainedContacts?: readonly SupportingContact[];
}

export interface LandingForecast {
  capturePoint: Vec3;
  horizonS: number;
  pressureFeasible: boolean;
}

export type LandingPlan = ({
  feasible: true;
  to: Vec3;
  requested: Vec3;
  duration: number;
  captureMarginM: number;
  landingCaptureMarginM: number;
  travelM: number;
  targetErrorM: number;
  transferForecastReason?: string | null;
} | { feasible: false; reason: string }) & { forecast?: LandingForecast };

/** The same joint-limited leg solve as actuation, rooted in the measured pelvis. */
export function validateLanding(input: LandingInput, point: Vec3): LandingPlan {
  const side = input.foot === "leftFoot" ? "left" : "right";
  const sign = side === "left" ? -1 : 1;
  const heading = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, input.heading);
  const right = rotate(heading, { x: 1, y: 0, z: 0 });
  const definition = SEGMENT_BY_ID.get(input.foot)!;
  const floor = input.surfaceHeight?.(point.x, point.z) ?? input.floorY;
  if (!Number.isFinite(floor)) return { feasible: false, reason: "terrain" };
  const to = { ...point, y: floor - definition.geometry.localBounds.min.y };
  const travelM = length(horizontal(sub(to, input.from)));
  if (travelM > input.maxTravelM + 1e-8) return { feasible: false, reason: "travel" };
  if (length(horizontal(sub(to, input.pelvis.position))) > input.maxReachM + 1e-8)
    return { feasible: false, reason: "reach" };
  if (sign * dot(sub(to, input.retainedFoot), right) < 0.10 - 1e-8)
    return { feasible: false, reason: "leg-separation" };
  const hip = worldPoint(input.pelvis.position, input.pelvis.rotation,
    SEGMENT_BY_ID.get(`${side}Thigh`)!.jointAnchorParent!);
  const radial = legTargetReach(side, hip, to, heading);
  if (radial.distanceM > radial.radiusM - 0.008 + 1e-8)
    return { feasible: false, reason: "radial-reach" };
  const bounded = composeTouchdownHindfoot(side, input.pelvis, to, input.heading);
  const horizontalError = length(horizontal(sub(bounded, to)));
  if (horizontalError > 0.04 || Math.abs(bounded.y - to.y) > 0.04)
    return { feasible: false, reason: "joint-limits" };
  const retained = input.retainedContacts?.filter(c => c.loadBearing && c.normalY >= 0.65)
    .flatMap(c => c.points?.length ? c.points : [c.point]) ?? [];
  if (!retained.length) retained.push(input.retainedFoot);
  const patch = definition.geometry.supportPatch ?? [];
  const landingHull = recoverySupportHull(patch.map(p => worldPoint(to, heading, p)));
  const hull = recoverySupportHull([...retained, ...landingHull]);
  const capturePoints = [input.capturePoint, input.measuredMomentumCapturePoint]
    .filter((point): point is Vec3 => point !== undefined);
  const captureMarginM = capturePoints.length
    ? Math.min(...capturePoints.map(point => recoverySupportMargin(point, hull)))
    : -length(horizontal(sub(to, input.requested)));
  const landingCaptureMarginM = capturePoints.length
    ? Math.min(...capturePoints.map(point => recoverySupportMargin(point, landingHull)))
    : captureMarginM;
  return { feasible: true, to, requested: { ...input.requested, y: to.y },
    duration: 0.48, captureMarginM, landingCaptureMarginM, travelM, targetErrorM: length(sub(bounded, to)) };
}

/** Finite 2-D search, including off-ray candidates. Every returned point is validated. */
export function findLandingPlan(input: LandingInput): LandingPlan {
  const points: Vec3[] = [input.requested, input.from,
    add(input.from, clampLength(horizontal(sub(input.requested, input.from)), input.maxTravelM))];
  const heading = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, input.heading);
  const right = rotate(heading, { x: 1, y: 0, z: 0 });
  const forward = rotate(heading, { x: 0, y: 0, z: 1 });
  const side = input.foot === "leftFoot" ? "left" : "right";
  const hip = worldPoint(input.pelvis.position, input.pelvis.rotation,
    SEGMENT_BY_ID.get(`${side}Thigh`)!.jointAnchorParent!);
  const ground = { ...input.requested,
    y: (input.surfaceHeight?.(input.requested.x, input.requested.z) ?? input.floorY)
      - SEGMENT_BY_ID.get(input.foot)!.geometry.localBounds.min.y };
  const reach = legTargetReach(side, hip, ground, heading);
  const center = horizontal(sub(hip, sub(reach.requestedAnkle, ground)));
  const radius = Math.sqrt(Math.max(0,
    (reach.radiusM - 0.008) ** 2 - (reach.requestedAnkle.y - hip.y) ** 2));
  // Near full extension the feasible disk can be much smaller than a grid
  // cell. Geometric seeds still pass the complete joint/terrain validator.
  points.push(center, add(center, clampLength(horizontal(sub(input.requested, center)), radius)));
  for (let ring = 1; ring <= 4; ring++) for (let angle = 0; angle < 24; angle++) {
    const theta = angle * Math.PI / 12;
    points.push(add(center, add(scale(right, radius * ring / 4 * Math.cos(theta)),
      scale(forward, radius * ring / 4 * Math.sin(theta)))));
  }
  // Polar samples cover the entire permitted disk, independently of the request ray.
  for (let ring = 1; ring <= 6; ring++) {
    const radius = input.maxTravelM * ring / 6;
    for (let angle = 0; angle < 24; angle++) {
      const theta = angle * Math.PI / 12;
      points.push(add(input.from, add(scale(right, radius * Math.cos(theta)),
        scale(forward, radius * Math.sin(theta)))));
    }
  }
  let best: Extract<LandingPlan, { feasible: true }> | null = null;
  const rejected = new Map<string, number>();
  const consider = (point: Vec3) => {
    const plan = validateLanding(input, point);
    if (!plan.feasible) {
      rejected.set(plan.reason, (rejected.get(plan.reason) ?? 0) + 1);
      return;
    }
    // The retained patch can cap the combined margin for many placements.
    // Prefer support supplied by the landing sole before minimizing travel;
    // otherwise that plateau favors lifting back into an unhelpful footprint.
    if (!best || plan.captureMarginM > best.captureMarginM + 1e-5
      || (Math.abs(plan.captureMarginM - best.captureMarginM) <= 1e-5
        && (plan.landingCaptureMarginM > best.landingCaptureMarginM + 1e-5
          || (Math.abs(plan.landingCaptureMarginM - best.landingCaptureMarginM) <= 1e-5
            && plan.travelM < best.travelM)))) best = plan;
  };
  points.forEach(consider);
  // Refine the best feasible neighbourhood in both axes; never assume a ray is monotone.
  for (const span of [input.maxTravelM / 12, input.maxTravelM / 48]) {
    const center = (best as Extract<LandingPlan, { feasible: true }> | null)?.to;
    if (!center) break;
    for (let x = -2; x <= 2; x++) for (let z = -2; z <= 2; z++)
      consider(add(center, add(scale(right, x * span / 2), scale(forward, z * span / 2))));
  }
  return best ?? { feasible: false, reason: [...rejected].map(([reason, count]) => `${reason}:${count}`).join(",") };
}
