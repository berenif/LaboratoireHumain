import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { register } from 'tsx/esm/api';
import { sha256 } from './capture-physics-baseline.mjs';

const [reportPath, output] = process.argv.slice(2);
assert.ok(reportPath && output, 'Usage: report.json fresh-analysis.json');
const reportBytes = readFileSync(reportPath), report = JSON.parse(reportBytes);
for (const path of ['src/character/math.ts', 'src/core/humanoid.ts']) {
  assert.equal(sha256(readFileSync(path)), report.sourceAfter[path], `Analysis dependency changed: ${path}`);
}
const unregister = register();
try {
  const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
  const { angularVelocity, sub, dot } = await import('../src/character/math.ts');
  const results = [];
  for (const run of report.runs.filter(run => run.mode === 'quiet-held-pressure-contour-world-damping')) {
    const path = join(dirname(reportPath), run.trace.file), bytes = readFileSync(path);
    assert.equal(sha256(bytes), run.trace.sha256);
    const rows = bytes.toString('utf8').trim().split('\n').map(line => JSON.parse(line));
    const measurements = [];
    for (let i = 1; i < rows.length; i++) {
      const prior = rows[i - 1], current = rows[i];
      assert.equal(current.tick, prior.tick + 1, 'Consecutive command samples are required');
      const before = new Map(prior.input.poses.map(p => [p.id, p]));
      const after = new Map(current.input.poses.map(p => [p.id, p]));
      for (const motor of current.motors) {
        const definition = SEGMENT_BY_ID.get(motor.id);
        if (!['thigh', 'shin', 'ankle', 'hindfoot', 'forefoot'].includes(definition.role)) continue;
        const interval = id => angularVelocity(before.get(id).rotation, after.get(id).rotation, 1 / 60);
        const relative = sub(interval(motor.id), interval(definition.parent));
        for (const axis of motor.axes) {
          const intervalRate = dot(relative, axis.basisWorld);
          measurements.push({ tick: current.tick, id: motor.id, coordinate: axis.coordinate,
            publishedRate: axis.relativeSpeed, intervalRate, difference: axis.relativeSpeed - intervalRate });
        }
      }
    }
    results.push({ heading: run.heading, trace: path, traceSha256: sha256(bytes), measurements });
  }
  writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
    reportPath, reportSha256: sha256(reportBytes), results,
    qualification: 'Backward angular displacement over one full tick versus the published pre-command velocity; they are different measurement intervals. This is not proof of invalid velocity readback or of a successful damping correction.' }, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(results.map(run => ({ heading: run.heading, samples: run.measurements.length,
    largestDifference: run.measurements.reduce((a, b) => Math.abs(a.difference) > Math.abs(b.difference) ? a : b) }))));
} finally { unregister(); }
