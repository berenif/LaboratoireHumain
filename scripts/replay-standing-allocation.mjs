import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { register } from 'tsx/esm/api';
import { gunzipSync } from 'node:zlib';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const unregister = register();
const { planContactLoads, minimumSupportTorqueLimit } = await import('../src/character/contact-loads.ts');
const { BALANCE_LIMITS } = await import('../src/character/BalanceController.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { measureMassState } = await import('../src/character/mass-state.ts');
const { recoverySupportHull, recoverySupportMargin } = await import('../src/character/recovery-support.ts');
const [tracePath, tickText, output] = process.argv.slice(2);
if (!tracePath || !output || existsSync(output)) throw new Error('Usage: trace.jsonl tick fresh-output.json');
const tick = Number(tickText), source = fingerprints();
const bytes = readFileSync(tracePath);
const text = (tracePath.endsWith('.gz') ? gunzipSync(bytes) : bytes).toString('utf8');
const rows = text.trim().split('\n').map(JSON.parse).filter(row => row.tick === tick - 1 || row.tick === tick);
assert.deepEqual(rows.map(row => row.tick), [tick - 1, tick]);
const zero = { x: 0, y: 0, z: 0 };
function inputs(row) {
  assert.equal(row.input.step, null, 'Replay is scoped to idle double support');
  const contacts = row.input.contacts.filter(contact => contact.loadBearing && contact.forceN >= BALANCE_LIMITS.minimumContactForceN
    && contact.normalY >= 0.65 && ['hindfoot', 'forefoot'].includes(SEGMENT_BY_ID.get(contact.segment)?.role));
  const mass = measureMassState(row.input.poses);
  const fraction = Math.min(contacts.reduce((sum, contact) => sum + contact.forceN * contact.normalY, 0), mass.massKg * 9.81) / (mass.massKg * 9.81);
  return { contacts, com: mass.position, requested: row.contactPlan.requestedForce, options: { frictionCoefficient: 1.2,
    projectMeasuredPressure: true, maxHorizontalForceN: mass.massKg * BALANCE_LIMITS.maxBalanceAccelerationMps2 * fraction,
    maxJointTorqueNm: minimumSupportTorqueLimit(contacts) } };
}
const evaluate = input => planContactLoads(input.contacts, input.com, zero, input.requested, input.options);
const before = inputs(rows[0]), after = inputs(rows[1]);
const summary = result => ({ force: result.allocatedForce, pressure: result.pressurePoint, feasible: result.pressureFeasible,
  residualNm: result.pressureForceResidualNm, loads: Object.fromEntries(result.loads.map(load => [load.segment, load.plannedForce.y])) });
for (const [index, input] of [before, after].entries()) {
  const result = evaluate(input);
  // Velocity is deliberately zero: projectMeasuredPressure uses the measured COM,
  // not a momentum projection. Exact replay proves it does not affect this case.
  // JSON traces normalize signed zero; compare their exact serialized semantics.
  assert.deepEqual(JSON.parse(JSON.stringify(result)), rows[index].contactPlan, 'Reconstructed allocator inputs do not reproduce recorded production output');
}
const cases = [{ name: 'before', input: before }, { name: 'after', input: after },
  { name: 'after-with-before-request', input: { ...after, requested: before.requested } },
  { name: 'after-with-before-com', input: { ...after, com: before.com } },
  { name: 'after-with-before-manifolds', input: { ...after, contacts: after.contacts.map(contact => {
    const old = before.contacts.find(item => item.segment === contact.segment);
    return old ? { ...contact, point: old.point, points: old.points, measuredPressurePoint: old.measuredPressurePoint } : contact;
  }) } },
  ...after.contacts.map(contact => ({ name: `after-with-before-manifold-${contact.segment}`, input: { ...after,
    contacts: after.contacts.map(item => {
      const old = before.contacts.find(previous => previous.segment === item.segment);
      return item.segment === contact.segment && old ? { ...item, point: old.point, points: old.points, measuredPressurePoint: old.measuredPressurePoint } : item;
    }) } }))];
try {
  const results = cases.map(test => {
    const points = test.input.contacts.flatMap(contact => contact.points ?? [contact.point]);
    const hull = recoverySupportHull(points);
    return { ...test, result: evaluate(test.input), hull,
      maxMeasuredPointOutsideHullM: Math.max(0, ...points.map(point => -recoverySupportMargin(point, hull))) };
  });
  writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
    nodeVersion: process.version, source, sourceFingerprint: sha256(JSON.stringify(source)), trace: { path: tracePath, sha256: sha256(text), ticks: [tick - 1, tick] },
    note: 'Offline geometry substitutions only; these are not real contacts authorized for runtime actuation.', results }, null, 2) + '\n');
  for (const result of results) console.log(JSON.stringify({ name: result.name, maxMeasuredPointOutsideHullM: result.maxMeasuredPointOutsideHullM, ...summary(result.result) }));
} finally { unregister(); }
