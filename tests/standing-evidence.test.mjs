import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { execFileSync, spawnSync } from 'node:child_process';
import { existsSync, readFileSync, writeFileSync, mkdtempSync, mkdirSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { gunzipSync } from 'node:zlib';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from '../scripts/capture-physics-baseline.mjs';
import { holdPatchStandingLoads, projectStandingLoads, projectHindfootHullStandingLoads,
  projectEqualContourStandingLoads, projectMeasuredPressureStandingLoads } from '../scripts/standing-support-projection.mjs';

const unregister = register(); after(unregister);
const { distributeSupportLoad } = await import('../src/character/support-loads.ts');
const { recoverySupportHull, recoverySupportMargin } = await import('../src/character/recovery-support.ts');
const archive = 'docs/checkpoints/2026-09-26/h8-evaluation';
// Diagnostic checkpoints are intentionally local and are not published in Git.
// Keep their replay assertions enabled whenever the local archive is present.
const archiveAvailable = existsSync(archive);
const archivedTest = (name, fn) => test(name,
  { skip: archiveAvailable ? false : 'Local diagnostic checkpoint is not present' }, fn);
const rows = archiveAvailable
  ? gunzipSync(readFileSync(`${archive}/standing-regenerated-20260926-02/1-observe.jsonl.gz`))
    .toString('utf8').trim().split('\n').map(JSON.parse)
  : [];
const before = archiveAvailable ? rows.find(row => row.tick === 175).contactPlan : null;
const afterRow = rows.find(row => row.tick === 176), plan = archiveAvailable ? afterRow.contactPlan : null;
const contacts = archiveAvailable
  ? afterRow.input.contacts.filter(contact => plan.loads.some(load => load.segment === contact.segment))
  : [];

function checkPhysicalConstraints(result) {
  for (const axis of ['x', 'y', 'z']) assert.ok(Math.abs(result.loads.reduce((sum, load) => sum + load.plannedForce[axis], 0) - plan.allocatedForce[axis]) < 1e-8);
  for (const axis of ['x', 'z']) assert.ok(Math.abs(result.loads.reduce((sum, load) => sum + load.plannedForce.y * load.point[axis], 0) / plan.allocatedForce.y - plan.pressurePoint[axis]) < 1e-8);
  for (const load of result.loads) {
    assert.ok(load.plannedForce.y >= 0);
    const contact = contacts.find(contact => contact.segment === load.segment);
    assert.ok(contact, 'Every support must retain its real owner');
    const hull = recoverySupportHull(contact.points);
    // The production margin helper deliberately floors tiny edge lengths; for
    // a near-coincident two-point patch, test exact convex-segment membership.
    let margin = recoverySupportMargin(load.point, hull);
    if (hull.length === 2) {
      const dx = hull[1].x - hull[0].x, dz = hull[1].z - hull[0].z;
      const denominator = dx * dx + dz * dz;
      const fraction = denominator ? Math.max(0, Math.min(1,
        ((load.point.x - hull[0].x) * dx + (load.point.z - hull[0].z) * dz) / denominator)) : 0;
      margin = -Math.hypot(load.point.x - hull[0].x - fraction * dx, load.point.z - hull[0].z - fraction * dz);
    }
    assert.ok(margin >= -1e-8, 'Resultant must remain in its current measured patch');
  }
}

archivedTest('H8 uses current measured manifolds and preserves force/moment on the regenerated discontinuity', () => {
  const result = holdPatchStandingLoads(plan, contacts, distributeSupportLoad, before);
  assert.equal(result.reason, 'held-patch-references-feasible');
  checkPhysicalConstraints(result.plan);
  const jump = Math.max(...result.plan.loads.map(load => Math.abs(load.plannedForce.y - before.loads.find(prior => prior.segment === load.segment).plannedForce.y)));
  assert.ok(jump <= 20);
});

archivedTest('H8 explicitly reports missing references, changed support, and infeasible pressure', () => {
  const initialize = holdPatchStandingLoads(plan, contacts, distributeSupportLoad, undefined);
  assert.equal(initialize.reason, 'initialize'); assert.equal(initialize.plan, plan);
  const changed = holdPatchStandingLoads(plan, contacts.slice(1), distributeSupportLoad, before);
  assert.equal(changed.reason, 'support-membership-changed'); assert.equal(changed.plan, plan);
  const infeasible = { ...plan, pressureFeasible: false };
  const result = holdPatchStandingLoads(infeasible, contacts, distributeSupportLoad, before);
  assert.equal(result.reason, 'pressure-infeasible'); assert.equal(result.plan, infeasible);
});

archivedTest('H9 allows patch shares to move without inventing force, pressure, or contacts', () => {
  const result = projectStandingLoads(plan, contacts, distributeSupportLoad, before);
  assert.equal(result.reason, 'projected-current-measured-points');
  checkPhysicalConstraints(result.plan);
});

archivedTest('current contour projections retain measured owners and force/moment constraints', () => {
  for (const solve of [projectHindfootHullStandingLoads, projectEqualContourStandingLoads, projectMeasuredPressureStandingLoads]) {
    const result = solve(plan, contacts, distributeSupportLoad, null, recoverySupportHull);
    assert.match(result.reason, /^projected-current-/);
    checkPhysicalConstraints(result.plan);
    const duplicated = contacts.map(contact => ({ ...contact, points: [...contact.points, ...contact.points].reverse() }));
    const second = solve(plan, duplicated, distributeSupportLoad, null, recoverySupportHull);
    for (const load of result.plan.loads) {
      const match = second.plan.loads.find(value => value.segment === load.segment);
      for (const axis of ['x', 'y', 'z']) {
        assert.ok(Math.abs(load.plannedForce[axis] - match.plannedForce[axis]) < 1e-8, 'Duplicate witnesses cannot reweight a real patch');
      }
    }
  }
});

test('measured-pressure projection retains a feasible measured patch wrench', () => {
  const patches = [-1, 1].map((side, index) => ({ segment: index ? 'rightFoot' : 'leftFoot', forceN: index ? 400 : 300,
    point: { x: side * .1, y: .002, z: 0 }, measuredPressurePoint: { x: side * .1 + .01, y: .002, z: .02 },
    points: [-.04, .04].flatMap(x => [-.06, .06].map(z => ({ x: side * .1 + x, y: .002, z }))) }));
  const pressure = { x: patches.reduce((sum, patch) => sum + patch.measuredPressurePoint.x * patch.forceN, 0) / 700, y: .002, z: .02 };
  const request = { loads: patches.map(patch => ({ segment: patch.segment })), pressureFeasible: true,
    allocatedForce: { x: 0, y: 700, z: 0 }, pressurePoint: pressure };
  const result = projectMeasuredPressureStandingLoads(request, patches, distributeSupportLoad, null, recoverySupportHull);
  assert.equal(result.reason, 'projected-current-measured-points');
  for (const load of result.plan.loads) {
    const patch = patches.find(patch => patch.segment === load.segment);
    assert.ok(Math.abs(load.plannedForce.y - patch.forceN) < 1e-8);
    for (const axis of ['x', 'y', 'z']) assert.ok(Math.abs(load.point[axis] - patch.measuredPressurePoint[axis]) < 1e-8);
  }
});

archivedTest('portable standing archive verifies compressed and decompressed digests', () => {
  const output = execFileSync(process.execPath, ['scripts/archive-standing-evidence.mjs', 'verify', `${archive}/manifest.json`], { encoding: 'utf8' });
  assert.equal(JSON.parse(output).verified, 29);
});

test('portable standing archive preserves binary snapshots and package archives and detects corruption', () => {
  const directory = mkdtempSync(join(tmpdir(), 'standing-snapshot-test-'));
  try {
    const bytes = Buffer.from([0, 255, 128, 10, 13, 0, 42]);
    const input = join(directory, 'fixture');
    mkdirSync(input);
    writeFileSync(join(input, 'world.bin'), bytes);
    writeFileSync(join(input, 'runtime.tgz'), bytes);
    const output = join(directory, 'archive');
    execFileSync(process.execPath, ['scripts/archive-standing-evidence.mjs', 'create', output, input]);
    const manifestPath = join(output, 'manifest.json');
    const manifest = JSON.parse(readFileSync(manifestPath));
    const entry = manifest.files.find(file => file.original.endsWith('world.bin'));
    const packageEntry = manifest.files.find(file => file.original.endsWith('runtime.tgz'));
    assert.ok(entry);
    assert.ok(packageEntry);
    assert.deepEqual(gunzipSync(readFileSync(join(output, entry.archive))), bytes);
    assert.deepEqual(gunzipSync(readFileSync(join(output, packageEntry.archive))), bytes);
    execFileSync(process.execPath, ['scripts/archive-standing-evidence.mjs', 'verify', manifestPath]);
    writeFileSync(join(output, entry.archive), Buffer.from('corrupted'));
    const verification = spawnSync(process.execPath, ['scripts/archive-standing-evidence.mjs', 'verify', manifestPath]);
    assert.notEqual(verification.status, 0, 'Corrupted binary archive must fail verification');
  } finally {
    assert.ok(directory.startsWith(join(tmpdir(), 'standing-snapshot-test-')));
    rmSync(directory, { recursive: true, force: true });
  }
});

test('staged evaluator rejects a changed trace before writing a result', () => {
  const directory = mkdtempSync(join(tmpdir(), 'standing-evidence-test-'));
  try {
    const source = fingerprints(), trace = join(directory, 'trace.jsonl'), report = join(directory, 'report.json');
    writeFileSync(trace, 'changed\n');
    writeFileSync(report, JSON.stringify({ sourceBefore: source, sourceAfter: source,
      comparisons: ['observe', 'observe-repeat'].map(mode => ({ mode, beforeEqual: true, firstDifferenceTick: null })),
      runs: [{ mode: 'observe', heading: Math.PI / 3, trace: { rows: 2, file: 'trace.jsonl', sha256: sha256('original\n') } }] }));
    const result = spawnSync(process.execPath, ['scripts/evaluate-standing-h8.mjs', report, '176', join(directory, 'result.json')], { encoding: 'utf8' });
    assert.notEqual(result.status, 0); assert.match(result.stderr, /Trace digest mismatch/);
  } finally { rmSync(directory, { recursive: true, force: true }); }
});

archivedTest('evidence archive refuses to overwrite a completed run', () => {
  const result = spawnSync(process.execPath, ['scripts/archive-standing-evidence.mjs', 'create', archive, 'evidence/unused'], { encoding: 'utf8' });
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /Usage/);
});
