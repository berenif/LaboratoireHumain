import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { holdPatchStandingLoads, preferHindfootStandingLoads, projectHindfootStandingLoads, projectHindfootHullStandingLoads, projectMeasuredContourStandingLoads, projectEqualContourStandingLoads, projectMeasuredPressureStandingLoads, projectStandingLoads } from './standing-support-projection.mjs';

// A staged diagnostic, never a production acceptance gate. A failed static or
// local prediction rejects H8; callers must not continue to a parameter sweep.
const hypothesis = process.argv.find(arg => arg.startsWith('--hypothesis='))?.split('=')[1] ?? 'h8';
const hypotheses = {
  h8: { modes: ['hold-patch-loads', 'held-patch-world-damping'], solve: holdPatchStandingLoads, reason: 'held-patch-references-feasible' },
  h9: { modes: ['project-prior-loads', 'projected-prior-world-damping'], solve: projectStandingLoads, reason: 'projected-current-measured-points' },
  h11: { modes: ['hindfoot-loads', 'hindfoot-world-damping'], solve: preferHindfootStandingLoads, reason: 'current-hindfoot-pressure-feasible', mapBefore: true },
  h12: { modes: ['projected-hindfoot-loads', 'projected-hindfoot-world-damping'], solve: projectHindfootStandingLoads, reason: 'projected-current-hindfoot-points', mapBefore: true },
  h13: { modes: ['contour-hindfoot-loads', 'contour-hindfoot-world-damping'], solve: projectHindfootHullStandingLoads, reason: 'projected-current-hindfoot-points', mapBefore: true },
  h15: { modes: ['measured-contour-loads', 'measured-contour-world-damping'], solve: projectMeasuredContourStandingLoads, reason: 'projected-current-measured-points', mapBefore: true },
  h19: { modes: ['equal-contour-loads', 'equal-contour-world-damping'], solve: projectEqualContourStandingLoads, reason: 'projected-current-measured-points', mapBefore: true },
  h20: { modes: ['pressure-contour-loads', 'pressure-contour-world-damping'], solve: projectMeasuredPressureStandingLoads, reason: 'projected-current-measured-points', mapBefore: true },
};
assert.ok(hypotheses[hypothesis], 'Unknown hypothesis');
const { modes, solve, reason, mapBefore } = hypotheses[hypothesis];
const [capturePath, tickText, output, localPath, screenPath] = process.argv.slice(2).filter(arg => !arg.startsWith('--hypothesis='));
if (!capturePath || !output || existsSync(output)) throw new Error('Usage: capture-report.json tick fresh-output.json [local-report.json [screen-report.json]]');
const tick = Number(tickText);
assert.ok(Number.isInteger(tick) && tick > 1);
const source = fingerprints();
const readReport = path => {
  const bytes = readFileSync(path), report = JSON.parse(bytes);
  assert.deepEqual(report.sourceBefore, report.sourceAfter, 'Capture source changed');
  const productionPaths = values => Object.keys(values).filter(path => path.startsWith('src/')
    || ['package.json', 'package-lock.json'].includes(path)).sort();
  assert.deepEqual(productionPaths(report.sourceBefore), productionPaths(source), 'Incomplete production fingerprint');
  // Diagnostic callers may evolve between stages; every production module and
  // package must still match the original physical run.
  for (const [path, digest] of Object.entries(report.sourceBefore)) {
    if (path.startsWith('src/') || ['package.json', 'package-lock.json'].includes(path)) {
      assert.equal(source[path], digest, `Capture source mismatch: ${path}`);
    }
  }
  return { path, sha256: sha256(bytes), report };
};
const capture = readReport(capturePath);
const observer = capture.report.runs.find(run => run.mode === 'observe');
assert.ok(observer && observer.trace.rows > 0, 'Nonempty observed trace required');
assert.ok(capture.report.comparisons.filter(item => item.mode.startsWith('observe')).length >= 2);
assert.ok(capture.report.comparisons.every(item => item.beforeEqual && item.firstDifferenceTick === null), 'Control and replay differ');
const tracePath = join(dirname(capturePath), observer.trace.file), bytes = readFileSync(tracePath);
assert.equal(sha256(bytes), observer.trace.sha256, 'Trace digest mismatch');
const rows = bytes.toString('utf8').trim().split('\n').map(JSON.parse).filter(row => [tick - 1, tick].includes(row.tick));
assert.deepEqual(rows.map(row => row.tick), [tick - 1, tick]);
const unregister = register();
try {
  const { distributeSupportLoad } = await import('../src/character/support-loads.ts');
  const { recoverySupportHull } = await import('../src/character/recovery-support.ts');
  const before = rows[0].contactPlan, after = rows[1].contactPlan;
  const contacts = rows[1].input.contacts.filter(contact => after.loads.some(load => load.segment === contact.segment));
  const held = solve(after, contacts, distributeSupportLoad, before, recoverySupportHull);
  const mappedBefore = mapBefore ? solve(before,
    rows[0].input.contacts.filter(contact => before.loads.some(load => load.segment === contact.segment)), distributeSupportLoad, null, recoverySupportHull).plan : before;
  const jump = (plan, reference = before) => Math.max(...plan.loads.map(load => Math.abs(load.plannedForce.y
    - (reference.loads.find(previous => previous.segment === load.segment)?.plannedForce.y ?? 0))));
  const total = held.plan.loads.reduce((sum, load) => sum + load.plannedForce.y, 0);
  const forceResidualN = Math.max(...['x', 'y', 'z'].map(axis => Math.abs(held.plan.loads.reduce((sum, load) => sum + load.plannedForce[axis], 0) - after.allocatedForce[axis])));
  const width = Math.max(.01, ...contacts.flatMap(contact => contact.points?.length ? contact.points : [contact.point])
    .map(point => Math.hypot(point.x - after.pressurePoint.x, point.z - after.pressurePoint.z)));
  const momentResidualNormalized = Math.max(...['x', 'z'].map(axis => Math.abs(held.plan.loads.reduce((sum, load) => sum + load.point[axis] * load.plannedForce.y, 0) / total - after.pressurePoint[axis]) / width));
  const minimumShare = Math.min(...held.plan.loads.map(load => load.plannedForce.y / total));
  const staticResult = { reason: held.reason, iterations: held.iterations, productionJumpN: jump(after), heldJumpN: jump(held.plan, mappedBefore),
    forceResidualN, momentResidualNormalized, minimumShare, plan: held.plan };
  staticResult.passed = held.reason === reason && staticResult.heldJumpN <= 20
    && forceResidualN / total <= 1e-8 && momentResidualNormalized <= 1e-8 && minimumShare >= 0;
  let local = null, screen = null;
  if (localPath) {
    assert.ok(staticResult.passed, 'H8 rejected at static stage; no runtime promotion');
    const evidence = readReport(localPath), report = evidence.report;
    assert.equal(report.parameters.startTick, tick); assert.equal(report.parameters.durationTicks, 1);
    const control = report.runs.find(run => run.mode === 'plain' && run.heading === observer.heading);
    const intervention = report.runs.find(run => run.mode === modes[0] && run.heading === observer.heading);
    assert.ok(control && intervention);
    assert.deepEqual(control.response, capture.report.runs.find(run => run.mode === 'plain').response, 'Runtime control history changed');
    const comparison = report.comparisons.find(item => item.mode === modes[0] && item.heading === observer.heading);
    assert.ok(comparison?.beforeEqual && comparison.firstDifferenceTick === tick, 'Mismatched intervention/prehistory');
    const baseline = control.response[tick - 1].footAngularSpeeds.leftForefoot;
    assert.ok(Number.isFinite(baseline) && baseline > 0, 'Invalid baseline response');
    const response = intervention.response[tick - 1].footAngularSpeeds.leftForefoot;
    const followingFive = intervention.response.slice(tick, tick + 5).map(row => row.footAngularSpeeds.leftForefoot);
    assert.equal(followingFive.length, 5);
    local = { path: evidence.path, sha256: evidence.sha256, baseline, response, reductionFraction: 1 - response / baseline,
      followingFive, projectionReasons: intervention.projectionReasons,
      passed: response <= .8 * baseline && Math.max(...followingFive) <= .8 * baseline };
  }
  if (screenPath) {
    assert.ok(local?.passed, 'H8 rejected at local stage; no startup screen');
    const evidence = readReport(screenPath), report = evidence.report;
    assert.equal(report.parameters.frames, 720); assert.equal(report.parameters.startTick, 1);
    assert.ok(report.parameters.durationTicks >= 720);
    const runs = report.runs.filter(run => modes.includes(run.mode));
    assert.equal(runs.length, 6);
    for (const mode of modes) {
      assert.deepEqual(runs.filter(run => run.mode === mode).map(run => run.heading), [0, Math.PI / 3, -Math.PI / 4]);
    }
    screen = { path: evidence.path, sha256: evidence.sha256, runs: runs.map(run => ({ heading: run.heading, mode: run.mode,
      peaks: run.peaks, endpointPelvisDriftM: run.endpointPelvisDriftM, endpointFootDriftM: run.endpointFootDriftM,
      maxPelvisExcursionM: run.maxPelvisExcursionM, maxFootExcursionM: run.maxFootExcursionM,
      projectionReasons: run.projectionReasons, maxJointSeparationM: run.maxJointSeparationM,
      maxSelfPenetrationM: run.maxSelfPenetrationM, maxFloorPenetrationM: run.maxFloorPenetrationM,
      passed: run.peaks.linear.value <= .1 && run.peaks.angular.value <= .5
        && run.endpointPelvisDriftM <= .03 && run.endpointFootDriftM <= .01 && run.steps === 0
        && run.firstSupportLossTick === null && run.firstNonUprightTick === null
        && run.maxJointSeparationM <= .01 && run.maxSelfPenetrationM <= .005 && run.maxFloorPenetrationM <= .08 })) };
    screen.passed = screen.runs.every(run => run.passed);
  }
  const rejected = !staticResult.passed || local?.passed === false || screen?.passed === false;
  const result = { hypothesis, generatedAt: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)], source,
    capture: { path: capture.path, sha256: capture.sha256 }, trace: { path: tracePath, sha256: sha256(bytes), ticks: [tick - 1, tick], heading: observer.heading },
    static: staticResult, local, screen, decision: rejected ? 'rejected' : screen ? 'screen-passed; official acceptance still required' : 'continue-to-next-stage',
    productionAdoption: false };
  assert.deepEqual(fingerprints(), source, 'Source changed during evaluation');
  writeFileSync(output, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ ...result, source: undefined, static: { ...staticResult, plan: undefined } }));
  if (rejected) process.exitCode = 1;
} finally { unregister(); }
