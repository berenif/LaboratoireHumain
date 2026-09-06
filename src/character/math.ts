import type { Quat, Vec3 } from "../core/types";

export const EPSILON = 1e-7;

export function add(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

export function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

export function scale(v: Vec3, amount: number): Vec3 {
  return { x: v.x * amount, y: v.y * amount, z: v.z * amount };
}

export function dot(a: Vec3, b: Vec3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

export function length(v: Vec3): number {
  return Math.hypot(v.x, v.y, v.z);
}

export function normalize(v: Vec3, fallback: Vec3 = { x: 0, y: 1, z: 0 }): Vec3 {
  const magnitude = length(v);
  return magnitude > EPSILON ? scale(v, 1 / magnitude) : fallback;
}

export function clamp(value: number, minimum: number, maximum: number): number {
  return Math.max(minimum, Math.min(maximum, value));
}

export function clampLength(v: Vec3, maximum: number): Vec3 {
  const magnitude = length(v);
  return magnitude > maximum && magnitude > EPSILON ? scale(v, maximum / magnitude) : v;
}

export function lerp(a: Vec3, b: Vec3, t: number): Vec3 {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  };
}

export function smooth01(value: number): number {
  const t = clamp(value, 0, 1);
  return t * t * (3 - 2 * t);
}

export function quatNormalize(q: Quat): Quat {
  const magnitude = Math.hypot(q.x, q.y, q.z, q.w) || 1;
  return { x: q.x / magnitude, y: q.y / magnitude, z: q.z / magnitude, w: q.w / magnitude };
}

export function quatMultiply(a: Quat, b: Quat): Quat {
  return quatNormalize({
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  });
}

export function quatInverse(q: Quat): Quat {
  const normalized = quatNormalize(q);
  return { x: -normalized.x, y: -normalized.y, z: -normalized.z, w: normalized.w };
}

export function rotate(q: Quat, v: Vec3): Vec3 {
  const vector = { x: q.x, y: q.y, z: q.z };
  const uv = cross(vector, v);
  const uuv = cross(vector, uv);
  return add(v, add(scale(uv, 2 * q.w), scale(uuv, 2)));
}

export function quatFromAxisAngle(axis: Vec3, radians: number): Quat {
  const unit = normalize(axis);
  const half = radians * 0.5;
  const sine = Math.sin(half);
  return quatNormalize({ x: unit.x * sine, y: unit.y * sine, z: unit.z * sine, w: Math.cos(half) });
}

export function quatFromTo(from: Vec3, to: Vec3): Quat {
  const a = normalize(from);
  const b = normalize(to);
  const cosine = clamp(dot(a, b), -1, 1);
  if (cosine > 1 - EPSILON) return { x: 0, y: 0, z: 0, w: 1 };
  if (cosine < -1 + EPSILON) {
    const candidate = Math.abs(a.x) < 0.8 ? { x: 1, y: 0, z: 0 } : { x: 0, y: 0, z: 1 };
    return quatFromAxisAngle(normalize(cross(a, candidate)), Math.PI);
  }
  const axis = cross(a, b);
  return quatNormalize({ x: axis.x, y: axis.y, z: axis.z, w: 1 + cosine });
}

export function angularVelocity(previous: Quat, current: Quat, dt: number): Vec3 {
  let delta = quatMultiply(current, quatInverse(previous));
  if (delta.w < 0) delta = { x: -delta.x, y: -delta.y, z: -delta.z, w: -delta.w };
  const angle = 2 * Math.acos(clamp(delta.w, -1, 1));
  const sine = Math.sqrt(Math.max(0, 1 - delta.w * delta.w));
  if (sine < 1e-5 || angle < 1e-5 || dt <= 0) return { x: 0, y: 0, z: 0 };
  return clampLength(scale({ x: delta.x / sine, y: delta.y / sine, z: delta.z / sine }, angle / dt), 18);
}

export function worldPoint(position: Vec3, rotation: Quat, localPoint: Vec3): Vec3 {
  return add(position, rotate(rotation, localPoint));
}

export function localPoint(position: Vec3, rotation: Quat, point: Vec3): Vec3 {
  return rotate(quatInverse(rotation), sub(point, position));
}

