import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

const [reportPath, controlPath, output, calibrationPath] = process.argv.slice(2);
if (!output || existsSync(output)) throw new Error('Usage: contact-report.json control-report.json fresh-output.json');
const reportBytes = readFileSync(reportPath), report = JSON.parse(reportBytes);
const controlBytes = readFileSync(controlPath), control = JSON.parse(controlBytes);
assert.ok(report.parameters.traceContacts, 'Native contact trace required');
const runs = report.runs.map(run => {
  const reference = control.runs.find(item => item.heading === run.heading && item.mode === run.mode);
  assert.equal(run.trajectoryHash, reference?.trajectoryHash, 'Native contact observer changed trajectory');
  const trace = readFileSync(join(dirname(reportPath), run.trace.file));
  assert.equal(sha256(trace), run.trace.sha256);
  const segments = {};
  for (const line of trace.toString('utf8').trim().split('\n')) {
    const row = JSON.parse(line);
    for (const manifold of row.groundManifolds) {
      const normal = manifold.impulses.reduce((sum, impulse) => sum + impulse.normal, 0);
      if (normal < 3 / 60 || !manifold.points.length) continue;
      const tangentX = manifold.impulses.reduce((sum, impulse) => sum + impulse.tangentX, 0);
      const tangentY = manifold.impulses.reduce((sum, impulse) => sum + impulse.tangentY, 0);
      const ratio = Math.hypot(tangentX, tangentY) / (normal * manifold.friction);
      const speeds = manifold.points.filter(point => point.velocity).map(point => Math.hypot(point.velocity.x, point.velocity.z));
      const pose = row.output.poses.find(pose => pose.id === manifold.segment);
      const current = segments[manifold.segment] ?? { samples: 0, friction: new Set(), maxFrictionRatio: 0,
        sumFrictionRatio: 0, maxPointSpeedMps: 0, sumPointSpeedSquared: 0, pointSamples: 0,
        maxCenterSpeedMps: 0, centerPathLengthM: 0, minimumPointPathLengthM: 0, maximumPointPathLengthM: 0 };
      current.samples++; current.friction.add(manifold.friction);
      current.maxFrictionRatio = Math.max(current.maxFrictionRatio, ratio); current.sumFrictionRatio += ratio;
      current.maxPointSpeedMps = Math.max(current.maxPointSpeedMps, ...speeds);
      current.sumPointSpeedSquared += speeds.reduce((sum, speed) => sum + speed * speed, 0); current.pointSamples += speeds.length;
      const centerSpeed = Math.hypot(pose.linearVelocity.x, pose.linearVelocity.z);
      current.maxCenterSpeedMps = Math.max(current.maxCenterSpeedMps, centerSpeed);
      current.centerPathLengthM += centerSpeed / 60;
      current.minimumPointPathLengthM += Math.min(...speeds) / 60;
      current.maximumPointPathLengthM += Math.max(...speeds) / 60;
      segments[manifold.segment] = current;
    }
  }
  return { heading: run.heading, mode: run.mode, trajectoryMatchesControl: true,
    segments: Object.fromEntries(Object.entries(segments).map(([id, value]) => [id, {
      samples: value.samples, friction: [...value.friction], maxReportedTangentRatio: value.maxFrictionRatio,
      meanReportedTangentRatio: value.sumFrictionRatio / value.samples, maxPointSpeedMps: value.maxPointSpeedMps,
      rmsPointSpeedMps: Math.sqrt(value.sumPointSpeedSquared / value.pointSamples), maxCenterSpeedMps: value.maxCenterSpeedMps,
      centerPathLengthM: value.centerPathLengthM, minimumPointPathLengthM: value.minimumPointPathLengthM,
      maximumPointPathLengthM: value.maximumPointPathLengthM,
    }])) };
});
const result = { generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
  report: { path: reportPath, sha256: sha256(reportBytes) }, control: { path: controlPath, sha256: sha256(controlBytes) },
  ...(calibrationPath ? { calibration: { path: calibrationPath, sha256: sha256(readFileSync(calibrationPath)) } } : {}),
  tangentReadbackQualification: 'Per-contact tangent fields are not established as delivered total friction. The independent box calibration reports zero while momentum balance requires -5 N s. Reported tangent ratios must not be used to claim friction saturation or absence.',
  qualification: 'Post-step velocities at current solver points in loaded manifolds; no assertion that every solver point carries load. Path lengths integrate speed magnitudes, not endpoint displacement.', runs };
writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify(runs));
