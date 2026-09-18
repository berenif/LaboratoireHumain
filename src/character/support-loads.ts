import type { SegmentId, SupportingContact, Vec3 } from "../core/types";
import { add, clamp, dot, scale, sub } from "./math";
import { recoverySupportHull, recoverySupportMargin } from "./recovery-support";

type SupportPoint = Vec3 & { segment: SegmentId };
export interface SupportLoad { segment: SegmentId; point: Vec3; share: number }

/** Nonnegative load distribution whose pressure lies in the measured contact
 * hull. These are bounded actuator intents, never forces applied to the floor.
 * No target feet, reconstructed soles, extrapolation or negative loads enter it.
 */
export function distributeSupportLoad(contacts: readonly SupportingContact[], requested: Vec3): SupportLoad[] {
  const points: SupportPoint[] = contacts.flatMap(contact =>
    (contact.points?.length ? contact.points : [contact.point])
      .map(point => ({ ...point, segment: contact.segment })),
  );
  // The planar hull helper intentionally strips metadata and projects y=0.
  // Recover the original contact, including its segment and measured height;
  // a type assertion here would silently lose every support-chain owner.
  const hull = recoverySupportHull(points).map(vertex => {
    const source = points.find(point => Math.abs(point.x - vertex.x) < 1e-10
      && Math.abs(point.z - vertex.z) < 1e-10);
    if (!source) throw new Error("Support hull vertex has no measured contact owner");
    return source;
  });
  if (!hull.length) return [];
  const load = (point: SupportPoint, share: number): SupportLoad => ({
    segment: point.segment, point: { x: point.x, y: point.y, z: point.z }, share,
  });
  if (hull.length === 1) return [load(hull[0], 1)];
  const horizontal = (v: Vec3): Vec3 => ({ x: v.x, y: 0, z: v.z });
  let pressure = horizontal(requested);
  if (hull.length < 3 || recoverySupportMargin(pressure, hull) < 0) {
    let closest = hull[0] as Vec3, nearest = Infinity;
    for (let i = 0; i < hull.length; i++) {
      const a = horizontal(hull[i]), b = horizontal(hull[(i + 1) % hull.length]);
      const edge = sub(b, a);
      const amount = clamp(dot(sub(pressure, a), edge) / Math.max(dot(edge, edge), 1e-12), 0, 1);
      const candidate = add(a, scale(edge, amount));
      const error = sub(pressure, candidate), distance = dot(error, error);
      if (distance < nearest) { nearest = distance; closest = candidate; }
    }
    pressure = closest;
  }
  if (hull.length === 2) {
    const edge = horizontal(sub(hull[1], hull[0]));
    const share = clamp(dot(horizontal(sub(pressure, hull[0])), edge) / Math.max(dot(edge, edge), 1e-12), 0, 1);
    return [load(hull[0], 1 - share), load(hull[1], share)];
  }
  // A convex hull's triangle fan covers its complete interior.
  const a = hull[0];
  for (let i = 1; i + 1 < hull.length; i++) {
    const b = hull[i], c = hull[i + 1];
    const denominator = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
    if (Math.abs(denominator) < 1e-12) continue;
    const wa = ((b.z - c.z) * (pressure.x - c.x) + (c.x - b.x) * (pressure.z - c.z)) / denominator;
    const wb = ((c.z - a.z) * (pressure.x - c.x) + (a.x - c.x) * (pressure.z - c.z)) / denominator;
    const wc = 1 - wa - wb;
    if (Math.min(wa, wb, wc) < -1e-7) continue;
    const weights = [wa, wb, wc].map(w => Math.max(0, w));
    const sum = weights.reduce((s, w) => s + w, 0);
    return [a, b, c].map((p, j) => load(p, weights[j] / sum));
  }
  throw new Error("Measured support hull cannot represent its projected pressure");
}
