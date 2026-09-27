import type { Quat, Vec3 } from "./types";

function requireValid(condition: unknown): asserts condition {
  if (!condition) throw new Error("INVALID_CONVEX_MASS_PROPERTIES");
}

export interface ConvexMassProperties {
  volume: number;
  mass: number;
  centerOfMass: Vec3;
  inertia: number[][];
  principalInertia: Vec3;
  principalFrame: Quat;
}

const vec = (a: readonly number[]): Vec3 => ({ x: a[0], y: a[1], z: a[2] });

/** Integrate the actual closed, oriented convex mesh at the requested mass.
 * Rapier 0.20's automatic convex inertia misorients principal axes for several
 * small segment meshes. Supplying the geometric tensor explicitly preserves
 * their shape and mass without relying on that decomposition.
 */
export function integrateConvexMass(vertices: readonly number[], indices: readonly number[], mass: number): ConvexMassProperties {
  requireValid(vertices.length % 3 === 0 && indices.length % 3 === 0 && mass > 0);
  let volume = 0;
  const first = [0, 0, 0], second = Array.from({ length: 3 }, () => [0, 0, 0]);
  for (let t = 0; t < indices.length; t += 3) {
    const p = indices.slice(t, t + 3).map(i => vertices.slice(i * 3, i * 3 + 3));
    const [a, b, c] = p;
    const v = (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
      + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    volume += v;
    const sum = [0, 1, 2].map(i => a[i] + b[i] + c[i]);
    for (let i = 0; i < 3; i++) {
      first[i] += v * sum[i] / 4;
      for (let j = 0; j < 3; j++) second[i][j] += v * (sum[i] * sum[j] + p.reduce((s, q) => s + q[i] * q[j], 0)) / 20;
    }
  }
  requireValid(Number.isFinite(volume) && Math.abs(volume) > 1e-15);
  const com = first.map(value => value / volume);
  const covariance = second.map((row, i) => row.map((value, j) => mass * (value / volume - com[i] * com[j])));
  const trace = covariance.reduce((sum, row, i) => sum + row[i], 0);
  const inertia = covariance.map((row, i) => row.map((value, j) => (i === j ? trace : 0) - value));
  return { volume: Math.abs(volume), mass, centerOfMass: vec(com), inertia, ...diagonalize(inertia) };
}

export function diagonalize(matrix: readonly (readonly number[])[]): Pick<ConvexMassProperties, "principalInertia" | "principalFrame"> {
  const a = matrix.map(row => [...row]);
  const vectors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const norm = Math.max(...a.flat().map(Math.abs));
  requireValid(norm > 0 && Number.isFinite(norm));
  for (let sweep = 0; sweep < 40; sweep++) {
    let p = 0, q = 1;
    for (const [i, j] of [[0, 2], [1, 2]]) if (Math.abs(a[i][j]) > Math.abs(a[p][q])) [p, q] = [i, j];
    if (Math.abs(a[p][q]) <= norm * 1e-14) break;
    const angle = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
    const c = Math.cos(angle), s = Math.sin(angle), pp = a[p][p], qq = a[q][q], pq = a[p][q];
    a[p][p] = c * c * pp - 2 * s * c * pq + s * s * qq;
    a[q][q] = s * s * pp + 2 * s * c * pq + c * c * qq;
    a[p][q] = a[q][p] = 0;
    for (let i = 0; i < 3; i++) {
      if (i !== p && i !== q) {
        const ip = a[i][p], iq = a[i][q];
        a[i][p] = a[p][i] = c * ip - s * iq;
        a[i][q] = a[q][i] = s * ip + c * iq;
      }
      const ip = vectors[i][p], iq = vectors[i][q];
      vectors[i][p] = c * ip - s * iq;
      vectors[i][q] = s * ip + c * iq;
    }
  }
  const principal = a.map((row, i) => row[i]);
  requireValid(principal.every(value => value > 0));
  const r = vectors, tr = r[0][0] + r[1][1] + r[2][2];
  let quaternion: number[];
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    quaternion = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, s / 4];
  } else {
    let i = 0;
    if (r[1][1] > r[i][i]) i = 1;
    if (r[2][2] > r[i][i]) i = 2;
    const j = (i + 1) % 3, k = (i + 2) % 3, s = Math.sqrt(1 + r[i][i] - r[j][j] - r[k][k]) * 2;
    quaternion = [0, 0, 0, 0];
    quaternion[i] = s / 4; quaternion[j] = (r[j][i] + r[i][j]) / s; quaternion[k] = (r[k][i] + r[i][k]) / s;
    quaternion[3] = (r[k][j] - r[j][k]) / s;
  }
  const magnitude = Math.hypot(...quaternion);
  const [x, y, z, w] = quaternion.map(value => value / magnitude);
  const result = { principalInertia: vec(principal), principalFrame: { x, y, z, w } };
  const reconstructed = inertiaTensor(result.principalInertia, result.principalFrame);
  requireValid(Math.max(...matrix.flatMap((row, i) => row.map((v, j) => Math.abs(v - reconstructed[i][j])))) <= norm * 1e-11);
  return result;
}

export function inertiaTensor(principal: Vec3, quaternion: Quat): number[][] {
  let { x, y, z, w } = quaternion;
  const n = Math.hypot(x, y, z, w); x /= n; y /= n; z /= n; w /= n;
  const r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]];
  const d = [principal.x, principal.y, principal.z];
  return r.map(ri => r.map(rj => ri.reduce((value, rik, k) => value + rik * d[k] * rj[k], 0)));
}
