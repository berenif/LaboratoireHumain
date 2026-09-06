import type { Quat, Vec3 } from "./types";

export const V3 = {
  zero: (): Vec3 => ({ x: 0, y: 0, z: 0 }),
  add: (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
  sub: (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
  scale: (v: Vec3, s: number): Vec3 => ({ x: v.x * s, y: v.y * s, z: v.z * s }),
  dot: (a: Vec3, b: Vec3): number => a.x * b.x + a.y * b.y + a.z * b.z,
  cross: (a: Vec3, b: Vec3): Vec3 => ({
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  }),
  length: (v: Vec3): number => Math.hypot(v.x, v.y, v.z),
  normalize: (v: Vec3): Vec3 => {
    const length = Math.hypot(v.x, v.y, v.z) || 1;
    return { x: v.x / length, y: v.y / length, z: v.z / length };
  },
  lerp: (a: Vec3, b: Vec3, t: number): Vec3 => ({
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  }),
};

export const Q = {
  identity: (): Quat => ({ x: 0, y: 0, z: 0, w: 1 }),
  normalize: (q: Quat): Quat => {
    const length = Math.hypot(q.x, q.y, q.z, q.w) || 1;
    return { x: q.x / length, y: q.y / length, z: q.z / length, w: q.w / length };
  },
  nlerp: (a: Quat, b: Quat, t: number): Quat => {
    const sign = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0 ? -1 : 1;
    return Q.normalize({
      x: a.x + (b.x * sign - a.x) * t,
      y: a.y + (b.y * sign - a.y) * t,
      z: a.z + (b.z * sign - a.z) * t,
      w: a.w + (b.w * sign - a.w) * t,
    });
  },
};
