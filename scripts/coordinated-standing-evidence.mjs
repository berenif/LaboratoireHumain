import assert from 'node:assert/strict';
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join, relative } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

export const json = (path, value) => writeFileSync(path, JSON.stringify(value, null, 2) + '\n', { flag: 'wx' });
export const norm = v => Math.hypot(v.x, v.y, v.z);
export const distance = (a, b) => Math.hypot(a.x - b.x, a.z - b.z);
export function captureBodies(world, handles) {
  return Object.fromEntries(handles.map(([id, handle]) => {
    const b = world.getRigidBody(handle);
    return [id, { position: { ...b.translation() }, rotation: { ...b.rotation() }, velocity: { ...b.linvel() },
      angularVelocity: { ...b.angvel() }, centerOfMass: { ...b.worldCom() }, mass: b.mass() }];
  }));
}
export function collisionContext(c) {
  return { colliderSegments: [...c.colliderSegments], excludedPairs: [...c.excludedPairs],
    hook: 'direct-pair exclusions return null; otherwise COMPUTE_IMPULSE; intersections true' };
}
export function restoredHooks(RAPIER, context) {
  const segments = new Map(context.colliderSegments), excluded = new Set(context.excludedPairs);
  return { filterIntersectionPair: () => true, filterContactPair(a, b) {
    const first = segments.get(a), second = segments.get(b);
    const pair = first < second ? `${first}|${second}` : `${second}|${first}`;
    return first && second && excluded.has(pair) ? null : RAPIER.SolverFlags.COMPUTE_IMPULSE;
  } };
}
export function percentiles(values) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const p = n => sorted[Math.min(sorted.length - 1, Math.ceil(sorted.length * n) - 1)];
  return { count: values.length, p50: p(0.5), p95: p(0.95), p99: p(0.99), max: sorted.at(-1) };
}
export function inventory(directory) {
  const files = {};
  const visit = dir => {
    for (const entry of readdirSync(dir, { withFileTypes: true })) {
      const path = join(dir, entry.name);
      if (entry.isDirectory()) visit(path);
      else files[relative(directory, path).replaceAll('\\', '/')] = { bytes: readFileSync(path).length, sha256: sha256(readFileSync(path)) };
    }
  };
  visit(directory); return files;
}
export function verifyInventory(directory, files) {
  for (const [path, expected] of Object.entries(files)) {
    const bytes = readFileSync(join(directory, path));
    assert.equal(bytes.length, expected.bytes, path); assert.equal(sha256(bytes), expected.sha256, path);
  }
}
export async function replayFailure(directory, RAPIER) {
  const failure = JSON.parse(readFileSync(join(directory, 'first-failure.json')));
  const bytes = readFileSync(join(directory, 'first-failure.bin'));
  assert.equal(sha256(bytes), failure.nativeSnapshotSha256);
  const world = RAPIER.World.restoreSnapshot(bytes), queue = new RAPIER.EventQueue(true);
  try {
    assert.deepEqual(captureBodies(world, failure.handles), failure.preBodies);
    world.step(queue, restoredHooks(RAPIER, failure.collision));
    const bodies = captureBodies(world, failure.handles);
    assert.deepEqual(bodies, failure.postBodies, 'Full native replay differs');
    return { tick: failure.tick, exactBodies: 25, nativeSnapshotSha256: failure.nativeSnapshotSha256,
      outputSha256: sha256(JSON.stringify(bodies)), capturePhase: failure.capturePhase };
  } finally { queue.free(); world.free(); }
}

/** Independent geometry/ownership checks, also active before settling ends. */
export async function makeInvariantMonitor(c, RAPIER) {
  const { SEGMENTS, SEGMENT_BY_ID, TOTAL_MASS_KG } = await import('../src/core/humanoid.ts');
  const { flattenGeometryVertices, flattenGeometryIndices } = await import('../src/core/geometry.ts');
  const { worldPoint, sub } = await import('../src/character/math.ts');
  const { ACCEPTANCE: A } = await import('./physics-fixtures.ts');
  const refs = new Map(c.ragdollBodies), colliders = new Map(c.ragdollColliders), setterCalls = [];
  const shapes = new Map(SEGMENTS.map(d => [d.id, new RAPIER.ConvexPolyhedron(flattenGeometryVertices(d.geometry), flattenGeometryIndices(d.geometry))]));
  for (const [id, body] of refs) for (const name of ['setTranslation','setRotation','setLinvel','setAngvel','setBodyType','setNextKinematicTranslation','setNextKinematicRotation']) {
    const original = body[name];
    body[name] = function (...args) { if (setterCalls.length < 8) setterCalls.push({ id, name }); return original.apply(this, args); };
  }
  const maxima = { jointM: 0, floorM: 0, selfM: 0, limitRad: 0, motorRatio: 0 };
  return { maxima, check(snapshot) {
    const violations = [], poses = new Map(snapshot.segments.map(p => [p.id, p]));
    if (setterCalls.length) violations.push('runtime-state-setter');
    if (c.ragdollBodies.size !== 25 || c.ragdollColliders.size !== 25) violations.push('assembly-count');
    let mass = 0;
    for (const d of SEGMENTS) {
      const b = c.ragdollBodies.get(d.id), p = poses.get(d.id);
      if (b !== refs.get(d.id) || c.ragdollColliders.get(d.id) !== colliders.get(d.id) || !b?.isDynamic()) violations.push(`ownership:${d.id}`);
      if (!p || ![p.position,p.rotation,p.linearVelocity,p.angularVelocity,b.worldCom()].every(v => Object.values(v).every(Number.isFinite))) violations.push(`finite:${d.id}`);
      mass += b.mass();
      if (d.parent) {
        const parent = poses.get(d.parent);
        maxima.jointM = Math.max(maxima.jointM, norm(sub(worldPoint(parent.position, parent.rotation, d.jointAnchorParent), worldPoint(p.position, p.rotation, d.jointAnchorChild))));
      }
      if (c.floorCollider.isEnabled()) {
        const contact = c.floorCollider.contactShape(shapes.get(d.id), p.position, p.rotation, 0);
        if (contact) maxima.floorM = Math.max(maxima.floorM, -contact.distance);
      }
    }
    const diagnostics = snapshot.diagnostics;
    maxima.selfM = Math.max(maxima.selfM, diagnostics.maxSelfPenetrationM);
    maxima.limitRad = Math.max(maxima.limitRad, diagnostics.maxJointLimitErrorRad);
    for (const joint of diagnostics.jointDiagnostics) {
      const cap = Math.hypot(...SEGMENT_BY_ID.get(joint.segment).jointProfile.axes.map(a => a.maxMotorTorqueNm));
      maxima.motorRatio = Math.max(maxima.motorRatio, joint.motorTorqueNm / cap);
      if (![joint.motorTorqueNm,joint.motorSaturationRatio,joint.limitErrorMagnitudeRad,
        ...Object.values(joint.coordinates),...Object.values(joint.targetCoordinates)].every(Number.isFinite)) violations.push(`joint-finite:${joint.segment}`);
    }
    if (Math.abs(mass - TOTAL_MASS_KG) > 1e-4) violations.push('mass');
    if (!diagnostics.finite || diagnostics.errors.length) violations.push('runtime-error');
    if (maxima.jointM > A.maxJointSeparationM) violations.push('anchor-separation');
    if (maxima.floorM > A.maxFloorPenetrationM) violations.push('floor-penetration');
    if (maxima.selfM > A.maxNonExcludedSelfPenetrationM) violations.push('self-penetration');
    if (maxima.limitRad > A.maximumStructuralLimitErrorRad) violations.push('anatomical-limit');
    if (maxima.motorRatio > 1 + 1e-6) violations.push('motor-cap');
    if (norm(diagnostics.recovery.assistanceForce) || norm(diagnostics.recovery.assistanceTorque)) violations.push('pelvis-assistance');
    const contacts = diagnostics.contactDiagnostics;
    if (![contacts.count,contacts.loadBearingCount,contacts.totalNormalForceN].every(Number.isFinite)
      || contacts.count < contacts.loadBearingCount || contacts.loadBearingCount < 0 || contacts.totalNormalForceN < 0) violations.push('contact-diagnostics');
    if (contacts.supportLoads.reduce((s,l) => s+l.plannedForceN,0) > 1.35*TOTAL_MASS_KG*9.81+1e-6) violations.push('planned-load-cap');
    if (contacts.supportLoads.some(l => !Number.isFinite(l.plannedForceN) || l.plannedForceN < 0
      || (!contacts.supportingSegments.includes(l.segment) && l.plannedForceN > 1e-8))) violations.push('unsupported-load');
    if (c.world.timestep !== Math.fround(1/60) && c.world.timestep !== 1/60) violations.push('timestep');
    if (c.world.numSolverIterations !== 20 || (!['falling','fallen','recovering'].includes(snapshot.state) && c.world.numInternalPgsIterations !== 20)) violations.push('solver-settings');
    return [...new Set(violations)];
  } };
}
