import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

const [reportPath, output] = process.argv.slice(2);
if (!reportPath || !output || existsSync(output)) throw new Error('Usage: report.json fresh-output.json');
const reportBytes = readFileSync(reportPath), report = JSON.parse(reportBytes);
const dt = 1 / 60, ids = ['pelvis', 'leftFoot', 'leftForefoot', 'rightFoot', 'rightForefoot'];
const runs = report.runs.map(run => {
  const bytes = readFileSync(join(dirname(reportPath), run.trace.file));
  assert.equal(sha256(bytes), run.trace.sha256, 'Trace digest mismatch');
  const rows = bytes.toString().trim().split('\n').filter(Boolean).map(JSON.parse);
  assert.ok(rows.length > 1, 'At least two complete trace rows required');
  assert.ok(rows.every((row, index) => index === 0 || row.tick === rows[index - 1].tick + 1), 'Contiguous trace required');
  const segments = Object.fromEntries(ids.map(id => {
    const actual = { x: 0, z: 0 }, integratedEndVelocity = { x: 0, z: 0 }, integratedTrapezoidVelocity = { x: 0, z: 0 };
    let squaredRateDifference = 0, peakRateDifferenceMps = 0, displacementPathM = 0, velocityPathM = 0;
    for (const row of rows) {
      const before = row.input.poses.find(pose => pose.id === id), after = row.output.poses.find(pose => pose.id === id);
      const delta = { x: after.centerOfMass.x - before.centerOfMass.x, z: after.centerOfMass.z - before.centerOfMass.z };
      for (const axis of ['x', 'z']) {
        actual[axis] += delta[axis];
        integratedEndVelocity[axis] += after.linearVelocity[axis] * dt;
        integratedTrapezoidVelocity[axis] += (before.linearVelocity[axis] + after.linearVelocity[axis]) * .5 * dt;
      }
      const rateDifference = Math.hypot(delta.x / dt - after.linearVelocity.x, delta.z / dt - after.linearVelocity.z);
      squaredRateDifference += rateDifference ** 2;
      peakRateDifferenceMps = Math.max(peakRateDifferenceMps, rateDifference);
      displacementPathM += Math.hypot(delta.x, delta.z);
      velocityPathM += Math.hypot(after.linearVelocity.x, after.linearVelocity.z) * dt;
    }
    return [id, { actual, integratedEndVelocity, integratedTrapezoidVelocity,
      endpointIntegralDifferenceM: Math.hypot(actual.x - integratedEndVelocity.x, actual.z - integratedEndVelocity.z),
      displacementPathM, velocityPathM, peakRateDifferenceMps, rmsRateDifferenceMps: Math.sqrt(squaredRateDifference / rows.length) }];
  }));
  return { heading: run.heading, mode: run.mode, ticks: [rows[0].tick, rows.at(-1).tick], samples: rows.length,
    trace: { file: run.trace.file, sha256: run.trace.sha256 }, segments };
});
const result = { generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
  report: { path: reportPath, sha256: sha256(reportBytes) },
  qualification: 'Measured COM displacement versus post-step Rapier COM velocity. Their discrepancy does not by itself distinguish within-step velocity changes from position stabilization; no native impulse or friction claim is inferred.', runs };
writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify(runs.map(run => ({ heading: run.heading, ticks: run.ticks,
  endpointIntegralDifferenceM: Object.fromEntries(Object.entries(run.segments).map(([id, value]) => [id, value.endpointIntegralDifferenceM])) }))));
