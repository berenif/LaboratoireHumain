import type { Collider, TempContactManifold } from "@dimforge/rapier3d-compat";
import type { Vec3 } from "../core/types";
import { add, dot, scale, sub, worldPoint } from "./math";

export interface ActiveContactObservation {
  point: Vec3;
  distanceM: number;
  /** Rapier's reported impulse, without a solver-iteration rescaling. */
  impulseNs: number;
  geometricIndex: number | null;
}

/** Minimum-cost one-to-one assignment, including an unmatched column per row. */
function assign(costs: number[][]): number[] {
  const rows = costs.length;
  if (!rows) return [];
  const columns = costs[0].length;
  const u = Array(rows + 1).fill(0), v = Array(columns + 1).fill(0);
  const owners = Array(columns + 1).fill(0), previous = Array(columns + 1).fill(0);
  for (let row = 1; row <= rows; row++) {
    owners[0] = row;
    let column = 0;
    const minimum = Array(columns + 1).fill(Infinity), used = Array(columns + 1).fill(false);
    do {
      used[column] = true;
      const currentRow = owners[column];
      let delta = Infinity, nextColumn = 0;
      for (let candidate = 1; candidate <= columns; candidate++) {
        if (used[candidate]) continue;
        const cost = costs[currentRow - 1][candidate - 1] - u[currentRow] - v[candidate];
        if (cost < minimum[candidate]) { minimum[candidate] = cost; previous[candidate] = column; }
        if (minimum[candidate] < delta) { delta = minimum[candidate]; nextColumn = candidate; }
      }
      for (let candidate = 0; candidate <= columns; candidate++) {
        if (used[candidate]) { u[owners[candidate]] += delta; v[candidate] -= delta; }
        else minimum[candidate] -= delta;
      }
      column = nextColumn;
    } while (owners[column] !== 0);
    do {
      const prior = previous[column];
      owners[column] = owners[prior]; column = prior;
    } while (column !== 0);
  }
  const result = Array(rows).fill(-1);
  for (let column = 1; column <= columns; column++) {
    if (owners[column]) result[owners[column] - 1] = column - 1;
  }
  return result;
}

/**
 * Read only active solver contacts. Geometric contacts omitted by manifold
 * reduction can retain impulses from older steps in pinned Rapier 0.20.
 * Its solver point is the midpoint of both CURRENT collider witnesses, with
 * the contact-skin offset. A one-to-one match preserves duplicate contacts.
 */
export function observeActiveContactManifold(
  manifold: TempContactManifold,
  surface: Collider,
  collider: Collider,
  flipped: boolean,
): { normal: Vec3; contacts: ActiveContactObservation[] } {
  const normal = scale({ ...manifold.normal() }, flipped ? -1 : 1);
  const solver = Array.from({ length: manifold.numSolverContacts() }, (_, index) => {
    const point = manifold.solverContactPoint(index);
    return point ? { point: { ...point }, distanceM: manifold.solverContactDist(index) } : null;
  }).filter((contact): contact is { point: Vec3; distanceM: number } => contact !== null);
  if (!solver.length) return { normal, contacts: [] };
  const surfacePosition = { ...surface.translation() }, surfaceRotation = { ...surface.rotation() };
  const bodyPosition = { ...collider.translation() }, bodyRotation = { ...collider.rotation() };
  const surfaceSkin = surface.contactSkin(), bodySkin = collider.contactSkin();
  const skinOffset = scale(normal, (bodySkin - surfaceSkin) * 0.5);
  const geometric = Array.from({ length: manifold.numContacts() }, (_, index) => {
    // Copy each WASM result before the next scratch-storage query.
    const first = manifold.localContactPoint1(index);
    const firstPoint = first ? { ...first } : null;
    const second = manifold.localContactPoint2(index);
    const secondPoint = second ? { ...second } : null;
    if (!firstPoint || !secondPoint) return null;
    const surfacePoint = worldPoint(surfacePosition, surfaceRotation, flipped ? secondPoint : firstPoint);
    const bodyPoint = worldPoint(bodyPosition, bodyRotation, flipped ? firstPoint : secondPoint);
    return { index, point: add(scale(add(surfacePoint, bodyPoint), 0.5), skinOffset),
      distanceM: manifold.contactDist(index) - surfaceSkin - bodySkin,
      impulseNs: Math.max(0, manifold.contactImpulse(index)) };
  });
  const unmatchedCost = 2 * (solver.length + 1);
  const costs = solver.map(contact => [
    ...geometric.map(candidate => {
      if (!candidate) return unmatchedCost * 2;
      // Identity tolerance follows f32 world-coordinate precision, not a
      // physical contact-distance or support eligibility threshold.
      const magnitude = Math.max(1, ...Object.values(contact.point).map(Math.abs),
        ...Object.values(candidate.point).map(Math.abs));
      const tolerance = 8 * 2 ** -23 * magnitude;
      const difference = sub(contact.point, candidate.point);
      const positionError2 = dot(difference, difference);
      const distanceError = Math.abs(contact.distanceM - candidate.distanceM);
      return positionError2 <= tolerance * tolerance && distanceError <= tolerance
        ? (positionError2 + distanceError * distanceError) / (tolerance * tolerance)
        : unmatchedCost * 2;
    }),
    ...solver.map(() => unmatchedCost),
  ]);
  const assignments = assign(costs);
  return { normal, contacts: solver.map((contact, index) => {
    const column = assignments[index];
    const match = column >= 0 && column < geometric.length && costs[index][column] < unmatchedCost
      ? geometric[column] : null;
    return { ...contact, impulseNs: match?.impulseNs ?? 0, geometricIndex: match?.index ?? null };
  }) };
}
