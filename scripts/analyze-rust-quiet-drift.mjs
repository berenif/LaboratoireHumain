// Endpoint and sampled-contact diagnostic, without a physics acceptance claim.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';

const output = resolve(process.argv[2] ?? '');
assert.ok(process.argv[2] && !existsSync(output), 'Use a fresh evidence file');
const paths = {
  native: process.argv[3] ?? 'evidence/rust-worker-quiet-drift-native-01/native/worker-quiet.json',
  wasm: process.argv[4] ?? 'evidence/rust-worker-quiet-drift-browser-02/protocol-plus60.json',
  nativeBaseline: 'evidence/rust-worker-quiet-native-01/native/worker-quiet.json',
  wasmBaseline: 'evidence/rust-worker-quiet-browser-01/protocol-plus60.json',
};
const read = path => JSON.parse(readFileSync(path, 'utf8'));
const native = read(paths.native)[0], wasm = read(paths.wasm).measurement;
const nativeBaseline = read(paths.nativeBaseline).find(row => row.heading === native.heading);
const wasmBaseline = read(paths.wasmBaseline).measurement;
const keys = ['heading', 'mode', 'profile', 'initial', 'settled', 'final', 'progress', 'measurements'];
function difference(a, b, path = '$') {
  if (Object.is(a, b)) return null;
  if (!a || !b || typeof a !== 'object' || typeof b !== 'object') return { path, actual: a, baseline: b };
  const ak = Object.keys(a), bk = Object.keys(b);
  if (ak.length !== bk.length || ak.some(key => !Object.hasOwn(b, key))) return { path, actualKeys: ak, baselineKeys: bk };
  for (const key of ak) {
    const found = difference(a[key], b[key], `${path}.${key}`);
    if (found) return found;
  }
  return null;
}
const core = row => Object.fromEntries(keys.map(key => [key, row[key]]));
const comparisons = {
  native: difference(core(native), core(nativeBaseline)),
  wasm: difference(core(wasm), core(wasmBaseline)),
};
const round = n => Number(n.toFixed(6));
function analyze(row) {
  const samples = row.driftTrace.samples;
  const c = Math.cos(row.heading), s = Math.sin(row.heading);
  const local = (a, b) => {
    const x = a.x - b.x, z = a.z - b.z;
    return [c * x - s * z, a.y - b.y, s * x + c * z];
  };
  const settled = samples.find(sample => sample.completedSubsteps === 480);
  assert.ok(settled && samples.length <= 34, 'Bounded trace must include the settled boundary');
  const rows = samples.filter(sample => sample.completedSubsteps >= 480).map(sample => ({
    timeS: sample.simulationTimeS,
    comFromInitialMm: local(sample.com, sample.referenceCom).map(v => round(v * 1000)),
    comFromSettledMm: local(sample.com, settled.com).map(v => round(v * 1000)),
    pelvisPitchRollDeg: [sample.pelvisInHeading.x, sample.pelvisInHeading.z].map(v => round(2 * Math.atan2(v, sample.pelvisInHeading.w) * 180 / Math.PI)),
    feet: sample.floorContacts.map(contact => {
      const points = contact.selectedSolverPoints;
      const normal = points.reduce((sum, point) => sum + point.normalImpulseNs, 0);
      const tangent = points.reduce((sum, point) => sum + (point.rawReportedTangentImpulseMagnitudeNs ?? point.tangentImpulseMagnitudeNs), 0);
      const cap = points.reduce((sum, point) => sum + point.normalImpulseNs * point.friction, 0);
      const pose = sample.bodies.find(body => body.id === contact.id);
      const reference = settled.bodies.find(body => body.id === contact.id);
      const joint = sample.legs.find(leg => leg.id === contact.id);
      return {
        id: contact.id,
        localDeltaMm: local(pose.position, reference.position).map(v => round(v * 1000)),
        sampledNormalLoadN: round(normal / row.profile.dt_s),
        selectedPointCount: points.length,
        tangentReportingAdmitted: sample.tangentReporting?.admitted === true,
        tangentReportingReason: sample.tangentReporting?.reason ?? 'Historical trace did not qualify tangential reporting; Simplified writeback leaves per-point tangent_impulse unpopulated.',
        rawReportedSumTangentMagnitudesOverSumMuN: cap > 0 ? round(tangent / cap) : null,
        coordinateRadians: joint.coordinateRadians,
        preSolveTargetsRadians: joint.preSolveTargetsRadians,
      };
    }),
    ankles: sample.legs.filter(leg => leg.id.endsWith('Ankle')),
  }));
  return { progress: row.progress, sampledRows: rows };
}
const result = {
  scope: 'Read-only once-per-second contact/state diagnostic. Unobserved substeps, contact-point tangential velocities, delivered motor effort and causal slip identification remain unqualified.',
  inputs: Object.fromEntries(Object.entries(paths).map(([key, path]) => [key, { path, sha256: createHash('sha256').update(readFileSync(path)).digest('hex') }])),
  comparisons,
  exactOriginalPhysicsAndMeasurements: Object.values(comparisons).every(value => value === null),
  native: analyze(native), wasm: analyze(wasm), releaseAccepted: false,
};
writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ output, exactOriginalPhysicsAndMeasurements: result.exactOriginalPhysicsAndMeasurements, comparisons,
  samples: { native: result.native.sampledRows.length, wasm: result.wasm.sampledRows.length },
  endpoints: Object.fromEntries(['native', 'wasm'].map(key => [key, result[key].sampledRows.at(-1)])) }));
if (!result.exactOriginalPhysicsAndMeasurements) process.exitCode = 1;
