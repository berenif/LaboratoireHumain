import type { ConvexGeometry, Quat, Vec3 } from "./types";

const EPSILON = 1e-10;

function freezeVec3(value: Vec3): Vec3 {
  return Object.freeze({ x: value.x, y: value.y, z: value.z });
}

function rotate(rotation: Quat, point: Vec3): Vec3 {
  const magnitude = Math.hypot(rotation.x, rotation.y, rotation.z, rotation.w) || 1;
  const qx = rotation.x / magnitude;
  const qy = rotation.y / magnitude;
  const qz = rotation.z / magnitude;
  const qw = rotation.w / magnitude;
  const tx = 2 * (qy * point.z - qz * point.y);
  const ty = 2 * (qz * point.x - qx * point.z);
  const tz = 2 * (qx * point.y - qy * point.x);
  return {
    x: point.x + qw * tx + (qy * tz - qz * ty),
    y: point.y + qw * ty + (qz * tx - qx * tz),
    z: point.z + qw * tz + (qx * ty - qy * tx),
  };
}

/** Finalize and freeze one reusable local-space convex surface. */
export function createConvexGeometry(
  vertices: readonly Vec3[],
  triangles: readonly (readonly [number, number, number])[],
  supportPatch?: readonly Vec3[],
): ConvexGeometry {
  if (vertices.length < 4) throw new Error("Convex geometry requires at least four vertices.");
  if (!triangles.length) throw new Error("Convex geometry requires a triangulated surface.");
  const copiedVertices = Object.freeze(vertices.map(freezeVec3));
  const copiedTriangles = Object.freeze(triangles.map((triangle) => {
    if (triangle.some((index) => !Number.isInteger(index) || index < 0 || index >= copiedVertices.length)) {
      throw new Error("Convex geometry triangle contains an invalid vertex index.");
    }
    return Object.freeze([triangle[0], triangle[1], triangle[2]] as const);
  }));
  const min = { x: Infinity, y: Infinity, z: Infinity };
  const max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const vertex of copiedVertices) {
    if (![vertex.x, vertex.y, vertex.z].every(Number.isFinite)) {
      throw new Error("Convex geometry contains a non-finite vertex.");
    }
    min.x = Math.min(min.x, vertex.x);
    min.y = Math.min(min.y, vertex.y);
    min.z = Math.min(min.z, vertex.z);
    max.x = Math.max(max.x, vertex.x);
    max.y = Math.max(max.y, vertex.y);
    max.z = Math.max(max.z, vertex.z);
  }
  return Object.freeze({
    kind: "convex" as const,
    vertices: copiedVertices,
    triangles: copiedTriangles,
    localBounds: Object.freeze({ min: freezeVec3(min), max: freezeVec3(max) }),
    ...(supportPatch?.length
      ? { supportPatch: Object.freeze(supportPatch.map(freezeVec3)) }
      : {}),
  });
}

export interface EllipsoidGeometryOptions {
  radialSegments?: number;
  latitudeSegments?: number;
  /** Relative cross-section scale at the negative-Y pole. */
  bottomScale?: number;
  /** Relative cross-section scale at the positive-Y pole. */
  topScale?: number;
}

/**
 * A faceted ellipsoid with an optional gentle Y taper. Every consumer receives
 * these exact vertices rather than recreating a nominal sphere or capsule.
 */
export function createEllipsoidGeometry(
  radii: Vec3,
  options: EllipsoidGeometryOptions = {},
): ConvexGeometry {
  const radialSegments = Math.max(6, Math.floor(options.radialSegments ?? 12));
  const latitudeSegments = Math.max(4, Math.floor(options.latitudeSegments ?? 8));
  const bottomScale = Math.max(0.25, options.bottomScale ?? 1);
  const topScale = Math.max(0.25, options.topScale ?? 1);
  const vertices: Vec3[] = [{ x: 0, y: -radii.y, z: 0 }];
  const triangles: [number, number, number][] = [];
  const rings: number[][] = [];

  for (let latitude = 1; latitude < latitudeSegments; latitude += 1) {
    const phi = -Math.PI / 2 + Math.PI * latitude / latitudeSegments;
    const normalizedY = Math.sin(phi);
    const taper = bottomScale + (topScale - bottomScale) * (normalizedY + 1) / 2;
    const ringRadius = Math.cos(phi) * taper;
    const ring: number[] = [];
    for (let radial = 0; radial < radialSegments; radial += 1) {
      const angle = 2 * Math.PI * radial / radialSegments;
      ring.push(vertices.length);
      vertices.push({
        x: Math.cos(angle) * radii.x * ringRadius,
        y: normalizedY * radii.y,
        z: Math.sin(angle) * radii.z * ringRadius,
      });
    }
    rings.push(ring);
  }

  const top = vertices.length;
  vertices.push({ x: 0, y: radii.y, z: 0 });
  const first = rings[0];
  for (let radial = 0; radial < radialSegments; radial += 1) {
    const next = (radial + 1) % radialSegments;
    triangles.push([0, first[radial], first[next]]);
  }
  for (let ringIndex = 0; ringIndex < rings.length - 1; ringIndex += 1) {
    const lower = rings[ringIndex];
    const upper = rings[ringIndex + 1];
    for (let radial = 0; radial < radialSegments; radial += 1) {
      const next = (radial + 1) % radialSegments;
      triangles.push(
        [lower[radial], upper[radial], lower[next]],
        [lower[next], upper[radial], upper[next]],
      );
    }
  }
  const last = rings.at(-1)!;
  for (let radial = 0; radial < radialSegments; radial += 1) {
    const next = (radial + 1) % radialSegments;
    triangles.push([top, last[next], last[radial]]);
  }
  return createConvexGeometry(vertices, triangles);
}

export interface PrismSection {
  z: number;
  halfWidth: number;
}

/** Flat-soled, tapered convex prism used for the articulated foot pieces. */
export function createTaperedPrismGeometry(
  sections: readonly PrismSection[],
  bottomY: number,
  topY: number,
): ConvexGeometry {
  if (sections.length < 2) throw new Error("A tapered prism needs at least two sections.");
  const ordered = [...sections].sort((a, b) => a.z - b.z);
  const vertices: Vec3[] = [];
  const triangles: [number, number, number][] = [];
  for (const section of ordered) {
    vertices.push(
      { x: -section.halfWidth, y: bottomY, z: section.z },
      { x: section.halfWidth, y: bottomY, z: section.z },
      { x: -section.halfWidth, y: topY, z: section.z },
      { x: section.halfWidth, y: topY, z: section.z },
    );
  }
  for (let index = 0; index < ordered.length - 1; index += 1) {
    const a = index * 4;
    const b = (index + 1) * 4;
    triangles.push(
      [a, a + 1, b + 1], [a, b + 1, b],
      [a + 2, b + 2, b + 3], [a + 2, b + 3, a + 3],
      [a, b, b + 2], [a, b + 2, a + 2],
      [a + 1, a + 3, b + 3], [a + 1, b + 3, b + 1],
    );
  }
  const first = 0;
  const last = (ordered.length - 1) * 4;
  triangles.push(
    [first, first + 2, first + 3], [first, first + 3, first + 1],
    [last, last + 1, last + 3], [last, last + 3, last + 2],
  );
  return createConvexGeometry(
    vertices,
    triangles,
    ordered.flatMap((section) => [
      { x: -section.halfWidth, y: bottomY, z: section.z },
      { x: section.halfWidth, y: bottomY, z: section.z },
    ]),
  );
}

export function flattenGeometryVertices(geometry: ConvexGeometry): Float32Array {
  return new Float32Array(geometry.vertices.flatMap(({ x, y, z }) => [x, y, z]));
}

export function flattenGeometryIndices(geometry: ConvexGeometry): Uint32Array {
  return new Uint32Array(geometry.triangles.flatMap(([a, b, c]) => [a, b, c]));
}

export function geometryHalfExtents(geometry: ConvexGeometry): Vec3 {
  return {
    x: (geometry.localBounds.max.x - geometry.localBounds.min.x) / 2,
    y: (geometry.localBounds.max.y - geometry.localBounds.min.y) / 2,
    z: (geometry.localBounds.max.z - geometry.localBounds.min.z) / 2,
  };
}

export function geometryBounds(geometry: ConvexGeometry): ConvexGeometry["localBounds"] {
  return geometry.localBounds;
}

/** Maximum oriented distance above or below the local origin. */
export function verticalExtent(geometry: ConvexGeometry, rotation: Quat): number {
  let extent = 0;
  for (const vertex of geometry.vertices) extent = Math.max(extent, Math.abs(rotate(rotation, vertex).y));
  return extent;
}

/** Exact lowest canonical vertex after a rigid transform. */
export function lowestWorldPoint(geometry: ConvexGeometry, position: Vec3, rotation: Quat): Vec3 {
  let lowest: Vec3 | null = null;
  for (const vertex of geometry.vertices) {
    const oriented = rotate(rotation, vertex);
    const world = { x: position.x + oriented.x, y: position.y + oriented.y, z: position.z + oriented.z };
    if (!lowest || world.y < lowest.y) lowest = world;
  }
  return lowest!;
}

/** Ray/triangle distance against the exact convex surface used by both views. */
export function raycastConvex(
  geometry: ConvexGeometry,
  origin: Vec3,
  direction: Vec3,
): number | null {
  const directionMagnitude = Math.hypot(direction.x, direction.y, direction.z);
  if (![origin.x, origin.y, origin.z, directionMagnitude].every(Number.isFinite)
      || directionMagnitude < EPSILON) return null;
  const unitDirection = {
    x: direction.x / directionMagnitude,
    y: direction.y / directionMagnitude,
    z: direction.z / directionMagnitude,
  };
  let nearest = Infinity;
  for (const [ia, ib, ic] of geometry.triangles) {
    const a = geometry.vertices[ia];
    const b = geometry.vertices[ib];
    const c = geometry.vertices[ic];
    const edgeAB = { x: b.x - a.x, y: b.y - a.y, z: b.z - a.z };
    const edgeAC = { x: c.x - a.x, y: c.y - a.y, z: c.z - a.z };
    const p = {
      x: unitDirection.y * edgeAC.z - unitDirection.z * edgeAC.y,
      y: unitDirection.z * edgeAC.x - unitDirection.x * edgeAC.z,
      z: unitDirection.x * edgeAC.y - unitDirection.y * edgeAC.x,
    };
    const determinant = edgeAB.x * p.x + edgeAB.y * p.y + edgeAB.z * p.z;
    if (Math.abs(determinant) < EPSILON) continue;
    const inverse = 1 / determinant;
    const fromA = { x: origin.x - a.x, y: origin.y - a.y, z: origin.z - a.z };
    const u = (fromA.x * p.x + fromA.y * p.y + fromA.z * p.z) * inverse;
    if (u < -EPSILON || u > 1 + EPSILON) continue;
    const q = {
      x: fromA.y * edgeAB.z - fromA.z * edgeAB.y,
      y: fromA.z * edgeAB.x - fromA.x * edgeAB.z,
      z: fromA.x * edgeAB.y - fromA.y * edgeAB.x,
    };
    const v = (unitDirection.x * q.x + unitDirection.y * q.y + unitDirection.z * q.z) * inverse;
    if (v < -EPSILON || u + v > 1 + EPSILON) continue;
    const distance = (edgeAC.x * q.x + edgeAC.y * q.y + edgeAC.z * q.z) * inverse;
    if (distance >= -EPSILON) nearest = Math.min(nearest, Math.max(0, distance));
  }
  return Number.isFinite(nearest) ? nearest : null;
}

/** Clip a convex surface to n·p <= offset and close the cut with an outward cap.
 * Used to remove only embedded axillary volume, not to add invisible barriers.
 */
export function clipConvexGeometry(geometry: ConvexGeometry, normal: Vec3, offset: number): ConvexGeometry {
  const magnitude = Math.hypot(normal.x, normal.y, normal.z);
  if (!(magnitude > EPSILON) || !Number.isFinite(offset)) throw new RangeError("Invalid clipping plane");
  const n = { x: normal.x / magnitude, y: normal.y / magnitude, z: normal.z / magnitude };
  const d = offset / magnitude;
  const distance = (p: Vec3): number => n.x * p.x + n.y * p.y + n.z * p.z - d;
  const vertices: Vec3[] = [];
  const triangles: [number, number, number][] = [];
  const byKey = new Map<string, number>();
  const cap = new Set<number>();
  const index = (p: Vec3): number => {
    const key = [p.x, p.y, p.z].map(v => Math.round(v * 1e10)).join(",");
    const previous = byKey.get(key);
    if (previous !== undefined) return previous;
    byKey.set(key, vertices.length);
    vertices.push(p);
    return vertices.length - 1;
  };
  for (const face of geometry.triangles) {
    const polygon: Vec3[] = [];
    for (let edge = 0; edge < 3; edge++) {
      const a = geometry.vertices[face[edge]], b = geometry.vertices[face[(edge + 1) % 3]];
      const da = distance(a), db = distance(b);
      if (da <= EPSILON) polygon.push(a);
      if ((da < -EPSILON && db > EPSILON) || (da > EPSILON && db < -EPSILON)) {
        const t = da / (da - db);
        const point = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t };
        polygon.push(point);
        cap.add(index(point));
      } else if (Math.abs(da) <= EPSILON) cap.add(index(a));
    }
    const ids = polygon.map(index);
    for (let i = 1; i + 1 < ids.length; i++) triangles.push([ids[0], ids[i], ids[i + 1]]);
  }
  if (cap.size >= 3) {
    const ids = [...cap];
    const center = ids.reduce((p, i) => ({ x: p.x + vertices[i].x / ids.length, y: p.y + vertices[i].y / ids.length, z: p.z + vertices[i].z / ids.length }), { x: 0, y: 0, z: 0 });
    const axis = Math.abs(n.y) < 0.9 ? { x: 0, y: 1, z: 0 } : { x: 1, y: 0, z: 0 };
    const u = { x: axis.y * n.z - axis.z * n.y, y: axis.z * n.x - axis.x * n.z, z: axis.x * n.y - axis.y * n.x };
    const v = { x: n.y * u.z - n.z * u.y, y: n.z * u.x - n.x * u.z, z: n.x * u.y - n.y * u.x };
    const angle = (i: number): number => {
      const p = { x: vertices[i].x - center.x, y: vertices[i].y - center.y, z: vertices[i].z - center.z };
      return Math.atan2(p.x * v.x + p.y * v.y + p.z * v.z, p.x * u.x + p.y * u.y + p.z * u.z);
    };
    ids.sort((a, b) => angle(a) - angle(b));
    for (let i = 1; i + 1 < ids.length; i++) triangles.push([ids[0], ids[i], ids[i + 1]]);
  }
  return createConvexGeometry(vertices, triangles);
}
