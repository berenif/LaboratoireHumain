import assert from 'node:assert/strict';

/** Actual integrated pelvis pose change, without velocity clipping or a deadband. */
export function pelvisPoseRates(initial, current, dt) {
  assert.ok(Number.isFinite(dt) && dt > 0);
  const normalize = q => {
    const length = Math.hypot(q.x, q.y, q.z, q.w);
    assert.ok(Number.isFinite(length) && length > 0);
    return { x: q.x / length, y: q.y / length, z: q.z / length, w: q.w / length };
  };
  const a = normalize(initial.rotation), b = normalize(current.rotation);
  let x = -b.w * a.x + b.x * a.w - b.y * a.z + b.z * a.y;
  let y = -b.w * a.y + b.x * a.z + b.y * a.w - b.z * a.x;
  let z = -b.w * a.z - b.x * a.y + b.y * a.x + b.z * a.w;
  let w = b.w * a.w + b.x * a.x + b.y * a.y + b.z * a.z;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const sine = Math.hypot(x, y, z);
  const factor = sine > 0 ? 2 * Math.atan2(sine, w) / (sine * dt) : 0;
  const linear = ['x', 'y', 'z'].map(axis => (current.position[axis] - initial.position[axis]) / dt);
  const angular = [x * factor, y * factor, z * factor];
  return { pelvisLinearResidual: linear, pelvisAngularResidual: angular,
    maxPelvisLinearRate: Math.hypot(...linear), maxPelvisAngularRate: Math.hypot(...angular) };
}

export function pelvisBoundedResidual(rates) {
  const excess = (vector, limit) => {
    const norm = Math.hypot(...vector);
    return vector.map(v => v * (norm > limit ? 1 - limit / norm : 0));
  };
  return [...excess(rates.pelvisLinearResidual, 0.01), ...excess(rates.pelvisAngularResidual, 0.05)];
}
