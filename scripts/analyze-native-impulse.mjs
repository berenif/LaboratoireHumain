import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const [directory, replayDirectory, output] = process.argv.slice(2);
if (!output || existsSync(output)) throw new Error('Usage: node scripts/analyze-native-impulse.mjs native-directory verified-wasm-replay-directory fresh-output.json');
const sourceBefore = fingerprints(), receipts = [];
const read = file => {
  const bytes = readFileSync(file); receipts.push({ file, sha256: sha256(bytes) });
  return JSON.parse(bytes);
};
const report = read(join(directory, 'report.json')), replay = read(join(replayDirectory, 'report.json'));
assert.equal(report.results.length, replay.results.length);
assert.ok(replay.results.every(row => row.matches.control && row.matches.driven && row.actual === row.originalActual));
const unregister = register();
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { jointCoordinates } = await import('../src/character/joint-coordinates.ts');
try {
  const cases = [], regimes = new Set();
  for (const row of report.results) for (const precision of ['f32', 'f64']) for (const side of ['control', 'driven']) {
    const native = read(join(directory, `${row.index}-${precision}-${side}.json`));
    if (precision === 'f64') assert.equal(native.serializedStateTransferVerified, true);
    assert.deepEqual(native.parameters, native.originalParameters);
    const final = native.samples.at(-1), bodies = final.measured;
    const limits = [...SEGMENT_BY_ID.values()].filter(d => d.jointProfile).flatMap(d => {
      const q = jointCoordinates(bodies[d.parent].rotation, bodies[d.id].rotation, d.jointProfile);
      return d.jointProfile.axes.map(a => ({ id: d.id, coordinate: a.coordinate,
        value: q[a.coordinate], margin: Math.min(q[a.coordinate] - a.minRadians, a.maxRadians - q[a.coordinate]) }));
    });
    const regime = { activeLimits: limits.filter(l => l.margin <= 0).map(l => [l.id, l.coordinate]),
      groundContacts: final.groundContacts.map(c => ({ first: c.first, second: c.second, solverContacts: c.solverContacts })),
      selfContacts: final.selfContacts.length };
    regimes.add(JSON.stringify(regime));
    cases.push({ index: row.index, precision, side, regime, limits,
      groundContacts: final.groundContacts, serializedStateTransferVerified: native.serializedStateTransferVerified });
  }
  assert.equal(regimes.size, 1, 'Contact membership or joint-limit regime differs');
  assert.ok(cases.every(c => c.regime.activeLimits.length === 0 && c.regime.selfContacts === 0));
  const metrics = Object.fromEntries(['wasm', 'f32', 'f64'].map(precision => {
    const response = row => precision === 'wasm' ? row.wasm : row.responses[precision];
    const positive = report.results.filter(r => r.impulseNmS > 0 && r.impulseNmS < .001);
    const slopes = positive.slice(1).map((r, index) => {
      const previous = positive[index];
      return (response(r).actual * r.impulseNmS - response(previous).actual * previous.impulseNmS)
        / (r.impulseNmS - previous.impulseNmS);
    });
    return [precision, { maxRelativeError: Math.max(...report.results.map(r => response(r).relativeError)),
      failuresAtOriginalTolerance: report.results.filter(r => response(r).relativeError >= .1).length,
      positiveResponseRange: [Math.min(...positive.map(r => response(r).actual)), Math.max(...positive.map(r => response(r).actual))],
      adjacentIncrementSlopes: slopes }];
  }));
  assert.ok(metrics.f64.maxRelativeError < .1);
  const sourceAfter = fingerprints(); assert.deepEqual(sourceBefore, sourceAfter);
  const summary = { generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
    sourceBefore, sourceAfter, receipts, metrics, regime: JSON.parse([...regimes][0]), cases,
    qualification: 'Precision-only native comparison after matching Cargo features and exact serialized physical-state transfer. WASM snapshot replay is exact with collision hooks. This diagnoses finite-precision sensitivity at tiny impulse magnitudes; it is not a standing repair or an accepted runtime change.' };
  writeFileSync(output, JSON.stringify(summary, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ metrics, regime: summary.regime, cases: cases.length }));
} finally { unregister(); }
