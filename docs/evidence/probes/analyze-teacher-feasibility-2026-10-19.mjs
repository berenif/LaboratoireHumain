// Deterministic retained-data analysis only. No actor, world, browser, or integration.
import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { resolve, join, relative } from 'node:path';
import { createHash } from 'node:crypto';
const lane = resolve('outputs/browser-capability-20261019/recovery-fixture/teacher-feasibility');
const output = join(lane, 'actual-back-teacher');
const read = name => JSON.parse(readFileSync(join(output, name), 'utf8'));
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const receipt = path => { const bytes = readFileSync(path); return { path: relative(process.cwd(), path), bytes: bytes.length, sha256: sha(bytes) }; };
const result = read('result.json'), evidence = read('browser-evidence.json');
const probe = read('teacher-feasibility-raw-export.json');
const legacyPath = resolve('outputs/browser-capability-20261019/recovery-fixture/terminal-tail-ab/actual-back-pair/baseline/stop-controller.json');
const legacy = JSON.parse(readFileSync(legacyPath));
const seed = JSON.parse(probe.teacherInitialJson), frames = JSON.parse(probe.teacherFramesJson);
assert.equal(result.passed, true); // Observation validity, never physical recovery qualification.
assert.equal(result.observationComplete, true);
assert.equal(probe.status, 'no-full-standing-endpoint-within-ceiling');
assert.equal(probe.realNativeSubsteps, 480); assert.equal(probe.realPhysicsIntegrations, 0);
assert.equal(probe.forecastNativeIntegrations, 5000); assert.equal(probe.decisionsStarted, 250);
assert.equal(probe.totalDecisionCeiling, 250); assert.equal(probe.originalHeightCutDecisions, 51);
assert.equal(probe.nativeAccountingValid, true); assert.equal(probe.unobservedNativeTail, 0);
assert.equal(probe.endpoint, null); assert.equal(probe.failure, null);
assert.equal(probe.referenceGenerated, false); assert.equal(probe.canonicalAppended, false); assert.equal(probe.realHandoffActivated, false);
assert.deepEqual(seed, legacy.teacher_initial); assert.deepEqual(frames.slice(0, 51), legacy.teacher_frames);
assert.equal(frames.length, 250); assert.equal(probe.nativeTrace.length, 5000);
const before = readFileSync(join(output, 'teacher-export-before-checkpoint.bin'));
const after = readFileSync(join(output, 'teacher-export-after-checkpoint.bin'));
assert.ok(before.equals(after) && before.length > 0);
for (const name of ['controller', 'fixture']) assert.ok(readFileSync(join(output, `before-${name}.json`)).equals(readFileSync(join(output, `after-${name}.json`))));
assert.equal(evidence.terminal.exportCalls, 1); assert.equal(evidence.workerInstances, 1);
assert.equal(evidence.terminalDuplicates, 1); assert.equal(evidence.lastPublicationNative, 480);
assert.deepEqual(evidence.publications.slice(-2).map(x => [x.native, x.completedTicks]), [[480, 1], [480, 0]]);
assert.deepEqual(evidence.physicalInputs, []); assert.deepEqual(evidence.rawWorkerFaults, []); assert.equal(evidence.rawObserverFault, null);
const f32Words = values => Array.from(new Uint32Array(new Float32Array(values).buffer));
const rows = probe.nativeTrace;
for (let index = 0; index < rows.length; index++) {
  const row = rows[index], current = row.current;
  assert.equal(row.forecastNativeSubstep, index + 1); assert.equal(current.nativeSubsteps, index + 1);
  assert.equal(row.decisionIndex, Math.floor(index / 20)); assert.equal(row.withinDecisionNative, index % 20 + 1);
  assert.equal(row.internalPgsIterations, 32); assert.equal(current.physicsIntegrations, 0);
  assert.deepEqual(f32Words(row.action), f32Words(frames[Math.floor(index / 20)].action));
  assert.equal(current.predicates.angularMotion, current.motion.every(x => x.angularPassed));
  assert.equal(current.predicates.linearMotion, current.motion.every(x => x.linearPassed));
  assert.equal(current.predicates.footDriftFromSuppliedAnchors, null);
  assert.equal(row.standingPassed, false); assert.equal(row.consecutiveFullStandingSamples, 0);
  assert.equal(row.prospectiveAnchorsUsed, null); assert.equal(row.prospectiveAnchorsForNext, null);
}
function summarize(part) {
  const predicates = Object.fromEntries(Object.keys(part[0].current.predicates).map(key => [key, part.filter(row => row.current.predicates[key] === true).length]));
  const best = kind => part.reduce((best, row) => {
    const body = row.current.motion.reduce((a, b) => b[kind] > a[kind] ? b : a);
    return body[kind] < best.value ? { value: body[kind], forecastNative: row.forecastNativeSubstep, limitingBody: body.body } : best;
  }, { value: Infinity, forecastNative: null, limitingBody: null });
  const supportPointCounts = {};
  for (const row of part) { const n = row.current.solverSupportPoints.length; supportPointCounts[n] = (supportPointCounts[n] ?? 0) + 1; }
  return { samples: part.length, nativeRange: [part[0].forecastNativeSubstep, part.at(-1).forecastNativeSubstep], truePredicateSamples: predicates,
    smallestAllBodyMaximumAngularRadps: best('angularRadps'), smallestAllBodyMaximumLinearMps: best('linearMps'), solverSupportPointCountHistogram: supportPointCounts };
}
const summary = { observedAt: result.at, diagnosticValid: true, outcome: probe.status,
  actualRealNative: 480, realRecoveryIntegrations: 0, realExportIntegrations: 0,
  forecastDecisions: 250, forecastNativeIntegrations: 5000, unobservedNativeTail: 0,
  originalHeightCutDecisions: 51, completeSeedAndFirst51LegacyFramesExactlyEqual: true,
  sourceCompatibleArchiveContactsAreLegacyCache: true, currentSupportEvidenceSource: 'nativeTrace.current only',
  checkpointBytesExactlyEqual: true, checkpointBytes: before.length, exportCalls: 1,
  strictFullStandingEndpoint: null, maximumConsecutiveFullStandingSamples: probe.maximumConsecutiveFullStandingSamples,
  thresholds: rows[0].current.thresholds, wholeForecast: summarize(rows), afterOriginalHeightCut: summarize(rows.slice(1020)),
  prospectiveAnchorsEverEstablished: false, driftPredicateNullSamples: rows.filter(row => row.current.predicates.footDriftFromSuppliedAnchors === null).length,
  interpretation: 'Continued unchanged WEAK teaching does not reach the existing full-standing endpoint within the total250-decision ceiling. Angular-motion fails every sample, independently of COM/support-hull interpretation; linear-motion also fails every sample after the original51-frame cut. No prospective anchor interval is seeded, so the null drift predicate is unavailable/untested, not measured drift. No real recovery run or suffix/handoff is justified by this probe.',
  supportHullCaveat: 'Existing COM/capture predicates can reject fewer than three hull vertices. Predicate counts are reported without claiming every false value proves a nondegenerate geometric departure; independent motion failures still preclude this endpoint.',
  physicalRecoveryAttempted: false, policyAdmitted: false, performanceClaim: false,
  receipts: ['result.json', 'browser-evidence.json', 'teacher-feasibility-raw-export.json', 'teacher-export-before-checkpoint.bin', 'teacher-export-after-checkpoint.bin', 'before-controller.json', 'after-controller.json', 'before-fixture.json', 'after-fixture.json', 'teacher-initial-typed.json', 'teacher-frames-typed.json'].map(name => receipt(join(output, name))).concat(receipt(legacyPath)) };
assert.equal(summary.maximumConsecutiveFullStandingSamples, 0);
writeFileSync(join(lane, 'teacher-feasibility-analysis.json'), JSON.stringify(summary, null, 2) + '\n');
console.log(JSON.stringify({ outcome: summary.outcome, actualRealNative: summary.actualRealNative, forecastDecisions: summary.forecastDecisions,
  maximumConsecutiveFullStandingSamples: summary.maximumConsecutiveFullStandingSamples, wholeForecast: summary.wholeForecast, afterOriginalHeightCut: summary.afterOriginalHeightCut }, null, 2));
