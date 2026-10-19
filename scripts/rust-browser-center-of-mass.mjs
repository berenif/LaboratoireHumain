import assert from 'node:assert/strict';

function f32(value) {
  assert.ok(Number.isFinite(value), 'Finite exported/model component');
  const result = Math.fround(value);
  assert.ok(Number.isFinite(result), 'Finite f32 component');
  return result;
}

// Match the pinned engine's raw quaternion-vector product, including its
// non-unit behavior. Do not normalize away exported quaternion norm drift.
export function rotateRawQuaternion(q, p) {
  const vectorNormSquared = q.x * q.x + q.y * q.y + q.z * q.z;
  const dot = q.x * p.x + q.y * p.y + q.z * p.z;
  const scalar = q.w * q.w - vectorNormSquared;
  return {
    x: scalar * p.x + 2 * dot * q.x + 2 * q.w * (q.y * p.z - q.z * p.y),
    y: scalar * p.y + 2 * dot * q.y + 2 * q.w * (q.z * p.x - q.x * p.z),
    z: scalar * p.z + 2 * dot * q.z + 2 * q.w * (q.x * p.y - q.y * p.x),
  };
}

// Observe the model's material centroid from exported body-frame poses.
// Restore f32 source constants/poses, then accumulate independently in f64.
// This is not the engine's cached COM or cached body-velocity observer.
export function measurePhysicalCenterOfMass(model, snapshot) {
  const poses = new Map(snapshot.segments.map(pose => [pose.id, pose]));
  assert.equal(poses.size, snapshot.segments.length, 'Unique snapshot pose IDs');
  const modelIds = model.segments.map(segment => segment.id);
  assert.equal(new Set(modelIds).size, modelIds.length, 'Unique model segment IDs');
  assert.deepEqual([...poses.keys()].sort(), [...modelIds].sort(), 'Complete model pose identities');
  let mass = 0, x = 0, y = 0, z = 0, contributors = 0, maxQuaternionNormError = 0;
  for (const segment of model.segments) {
    assert.ok(Number.isFinite(segment.massKg) && segment.massKg >= 0, 'Nonnegative model mass');
    if (segment.massKg === 0) continue;
    const pose = poses.get(segment.id);
    const position = Object.fromEntries(['x', 'y', 'z'].map(key => [key, f32(pose.position[key])]));
    const q = Object.fromEntries(['x', 'y', 'z', 'w'].map(key => [key, f32(pose.rotation[key])]));
    const local = Object.fromEntries(['x', 'y', 'z'].map(key => [key, f32(segment.goldenMass.centerOfMass[key])]));
    const norm = Math.hypot(q.x, q.y, q.z, q.w);
    assert.ok(norm > 0, 'Nonzero exported quaternion');
    maxQuaternionNormError = Math.max(maxQuaternionNormError, Math.abs(norm - 1));
    const offset = rotateRawQuaternion(q, local), bodyMass = f32(segment.massKg);
    assert.ok(bodyMass > 0, 'Positive f32 body mass');
    mass += bodyMass;
    x += bodyMass * (position.x + offset.x);
    y += bodyMass * (position.y + offset.y);
    z += bodyMass * (position.z + offset.z);
    contributors++;
  }
  assert.ok(mass > 0, 'Positive total physical mass');
  return { massKg: mass, contributors, position: { x: x / mass, y: y / mass, z: z / mass }, maxQuaternionNormError };
}
