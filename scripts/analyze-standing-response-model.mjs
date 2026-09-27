import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

const [reportPath, controlPath, output] = process.argv.slice(2);
assert.ok(reportPath && controlPath && output, 'Usage: model-report.json control-report.json fresh-analysis.json');
const read = path => JSON.parse(readFileSync(path));
const report = read(reportPath), control = read(controlPath);
assert.deepEqual(report.sourceBefore, report.sourceAfter);
const footIds = ['leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const features = sample => [...footIds.flatMap(id => ['x', 'z'].map(axis => sample.bodies[id].position[axis])),
  sample.centerOfMass.x, sample.centerOfMass.z, sample.bodies.pelvis.position.y];
const difference = (a, b) => a.map((value, i) => value - b[i]);
const errors = delta => ({ footM: Math.max(...footIds.map((_, i) => Math.hypot(delta[2 * i], delta[2 * i + 1]))),
  comM: Math.hypot(delta[8], delta[9]), pelvisHeightM: Math.abs(delta[10]) });
const results = [];
for (const run of report.runs) {
  const reference = control.runs.find(item => item.heading === run.heading && item.mode === 'quiet-held-pressure-contour-world-damping');
  assert.ok(reference);
  assert.equal(run.initialHash, reference.initialHash);
  assert.equal(run.trajectoryHash, reference.trajectoryHash);
  assert.deepEqual(run.response, reference.response);
  for (const record of run.responseModel.samples) {
    const path = join(dirname(reportPath), record.file), bytes = readFileSync(path);
    assert.equal(sha256(bytes), record.sha256);
    assert.equal(sha256(readFileSync(join(dirname(reportPath), record.snapshot))), record.snapshotSha256);
    const model = JSON.parse(bytes);
    assert.equal(model.exactLiveBaselineBodies, 25);
    assert.equal(model.axes.length, 14);
    assert.equal(model.cases.length, 89);
    const baseline = model.cases.find(c => c.kind === 'baseline');
    const samples = [];
    for (const horizon of [1, 6, 30]) {
      const sample = c => features(c.samples.find(s => s.tick === horizon));
      const base = sample(baseline), columns = [], symmetry = [];
      for (let axis = 0; axis < model.axes.length; axis++) {
        const find = sign => model.cases.find(c => c.kind === 'calibration' && c.deltaTorqueNm[axis] === sign
          && c.deltaTorqueNm.every((value, i) => i === axis || value === 0));
        const plus = sample(find(1)), minus = sample(find(-1));
        columns.push(plus.map((value, i) => (value - minus[i]) / 2));
        symmetry.push(errors(plus.map((value, i) => (value + minus[i]) / 2 - base[i])));
      }
      const heldOut = model.cases.filter(c => c.kind.startsWith('held-out-')).map(c => {
        const actual = sample(c), predicted = base.map((value, row) => value
          + columns.reduce((sum, column, axis) => sum + column[row] * c.deltaTorqueNm[axis], 0));
        return { name: c.name, kind: c.kind, predictionError: errors(difference(actual, predicted)),
          actualChange: errors(difference(actual, base)) };
      });
      const groups = ['held-out-axis', 'held-out-combined'].map(kind => {
        const values = heldOut.filter(c => c.kind === kind);
        return { kind, cases: values.length,
          maxFootErrorM: Math.max(...values.map(c => c.predictionError.footM)),
          rmsCaseMaxFootErrorM: Math.sqrt(values.reduce((sum, c) => sum + c.predictionError.footM ** 2, 0) / values.length),
          maxComErrorM: Math.max(...values.map(c => c.predictionError.comM)),
          maxPelvisHeightErrorM: Math.max(...values.map(c => c.predictionError.pelvisHeightM)),
          maxActualFootChangeM: Math.max(...values.map(c => c.actualChange.footM)),
          maxActualComChangeM: Math.max(...values.map(c => c.actualChange.comM)) };
      });
      samples.push({ horizonTicks: horizon, horizonSeconds: horizon / 60,
        featureOrder: [...footIds.flatMap(id => [`${id}.x`, `${id}.z`]), 'centerOfMass.x', 'centerOfMass.z', 'pelvis.y'],
        baseline: base, columns, symmetry, heldOut, groups });
    }
    results.push({ heading: run.heading, modelPath: path, modelSha256: sha256(bytes), exactLiveControlReplay: true,
      axes: model.axes.map(({ id, coordinate }) => ({ id, coordinate })), samples });
  }
}
writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
  report: { path: reportPath, sha256: sha256(readFileSync(reportPath)) }, control: { path: controlPath, sha256: sha256(readFileSync(controlPath)) }, results,
  qualification: 'Local position response with frozen native commands. Held-out amplitudes are excluded from calibration. RMS is over each case maximum foot error, not over every foot coordinate. No production transfer prediction, load readiness or standing gate is accepted.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify(results.map(result => ({ heading: result.heading, horizons: result.samples.map(sample => ({ ticks: sample.horizonTicks, groups: sample.groups })) }))));
