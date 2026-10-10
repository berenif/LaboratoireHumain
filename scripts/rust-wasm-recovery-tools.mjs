import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { closeSync, lstatSync, mkdirSync, openSync, realpathSync, renameSync, writeSync } from 'node:fs';
import { dirname, isAbsolute, join, relative, resolve, sep } from 'node:path';
import { archiveGeneratedReport, physicalReportBytes, readPhysicalJson } from './rust-physical-archive-tools.mjs';
import { recoveryCasePlan } from './rust-recovery-case-tools.mjs';
import { persistReceipt } from './rust-release-tools.mjs';
import { recoveryBrowserLimits } from './rust-recovery-browser-limits.mjs';

function profileMatches(actual, expected) {
  if (typeof expected === 'number') return typeof actual === 'number' && Number.isFinite(actual)
    && Math.fround(actual) === Math.fround(expected);
  if (expected && typeof expected === 'object') return actual !== null && typeof actual === 'object'
    && JSON.stringify(Object.keys(actual).sort()) === JSON.stringify(Object.keys(expected).sort())
    && Object.entries(expected).every(([name, value]) => profileMatches(actual[name], value));
  return actual === expected;
}
export function validateSelectedRecoveryReport(report, fixture, profile) {
  assert.equal(report?.kind, 'recovery'); assert.equal(report.releaseAccepted, false);
  assert.equal(report.fixtureId, fixture.id, 'Selected WASM fixture substitution');
  assert.equal(report.fixtureHeading, Math.fround(fixture.heading), 'Selected WASM fixture heading substitution');
  assert.ok(profileMatches(report.profile, profile), 'Selected WASM profile substitution');
  assert.ok(Array.isArray(report.cases) && report.cases.length === 1, 'Selected WASM must return exactly one case');
  const row = report.cases[0];
  assert.equal(row.id, fixture.id, 'Selected WASM case substitution');
  assert.equal(row.heading, Math.fround(fixture.heading), 'Selected WASM case heading substitution');
  const recoveryLimitS = recoveryBrowserLimits.recoveryLimitSubsteps / recoveryBrowserLimits.substepHz;
  const stableObservationS = recoveryBrowserLimits.stableActualMs / 1000;
  assert.equal(row.recoveryLimitS, recoveryLimitS, 'Selected WASM recovery deadline substitution');
  assert.equal(row.stableObservationS, stableObservationS, 'Selected WASM uninterrupted hold substitution');
  return row.passed === true && row.dynamicOwnership === true && row.failure === null
    && Number.isFinite(row.stableReturnTimeS) && row.stableReturnTimeS >= 0 && row.stableReturnTimeS <= recoveryLimitS
    && Number.isSafeInteger(row.handoffStandingSubsteps) && row.handoffStandingSubsteps >= recoveryBrowserLimits.stableSubsteps;
}

function containedArchive(root, file) {
  assert.ok(isAbsolute(file), 'WASM archive path must be absolute');
  const actualRoot = realpathSync(root), actualFile = realpathSync(file), local = relative(actualRoot, actualFile);
  assert.ok(local && !isAbsolute(local) && local !== '..' && !local.startsWith(`..${sep}`), 'WASM archive escaped browser output');
  for (let current = resolve(file);; current = dirname(current)) {
    assert.ok(!lstatSync(current).isSymbolicLink(), 'Refuse linked WASM archive ancestor');
    if (current === resolve(root)) break;
    assert.notEqual(current, dirname(current), 'WASM archive root is not an ancestor');
  }
  assert.ok(lstatSync(file).isFile(), 'WASM archive must be a file');
}

// Serialize the original foundation shape deterministically, one retained case
// at a time. Descriptors keep hashes and summaries, never duplicate large traces.
function aggregateFragments(entries, profile, accept) {
  let applied = profile;
  for (const entry of entries) if (entry.storageEvidence) {
    const first = readPhysicalJson(entry.path, entry.storageEvidence);
    if (profileMatches(first.profile, profile)) { applied = first.profile; break; }
  }
  return function* () {
    yield `{"kind":"recovery","profile":${JSON.stringify(applied)},"cases":[\n`;
    let count = 0;
    for (const entry of entries) {
      if (!entry.storageEvidence) continue;
      const report = readPhysicalJson(entry.path, entry.storageEvidence);
      accept?.(entry, report);
      assert.ok(Array.isArray(report.cases), 'Retained selected report has no cases');
      for (const row of report.cases) { if (count++) yield ',\n'; yield JSON.stringify(row); }
    }
    yield '\n],"releaseAccepted":false}\n';
  };
}

export async function collectWasmRecoveryCases({ output, profile, fixtureCases, workerIdentity, identity = {}, request, checkpoint }) {
  assert.ok(typeof workerIdentity === 'string' && workerIdentity.length > 0, 'Missing diagnostic worker identity');
  const plan = recoveryCasePlan(fixtureCases), entries = [];
  const execution = { ...identity, expectedCases: plan.length, completedAttempts: 0, completedCaseReports: 0,
    partial: true, workerIdentity, releaseAccepted: false };
  const save = () => {
    execution.completedAttempts = entries.filter(value => value.completed).length;
    execution.completedCaseReports = entries.filter(value => value.storageEvidence).length;
    execution.partial = execution.completedAttempts !== plan.length || execution.completedCaseReports !== plan.length;
    checkpoint?.(entries, execution);
    persistReceipt(join(output, 'wasm-recovery-case-execution.json'), { schema: 1, ...execution, cases: entries });
  };
  mkdirSync(join(output, 'wasm-recovery-cases'), { recursive: true }); save();
  for (const fixture of plan) {
    const path = join(output, 'wasm-recovery-cases', fixture.key + '.json');
    const entry = { fixtureId: fixture.id, heading: Math.fround(fixture.heading), ordinal: fixture.ordinal,
      requestId: fixture.ordinal, path, completed: false, identityMatched: false, passed: false, releaseAccepted: false };
    entries.push(entry); save();
    let packet;
    try { packet = await request({ kind: 'recovery', profile, fixtureId: fixture.id, requestId: entry.requestId }); }
    catch (error) {
      entry.failure = String(error); entry.transportFailure = true; execution.transportFailure = entry.failure;
      save(); break; // A timed-out synchronous WASM call cannot safely receive a subsequent request.
    }
    entry.completed = true; entry.responseRequestId = packet.requestId; entry.workerIdentity = packet.workerIdentity;
    // Retain even a physically failed or substituted report before validation.
    if (packet.report !== undefined) {
      persistReceipt(path, packet.report);
      try {
        entry.storageEvidence = archiveGeneratedReport(path, output);
        entry.sha256 = entry.storageEvidence.decodedSha256;
      } catch (error) {
        entry.failure = String(error); entry.archiveFailure = true; execution.archiveFailure = entry.failure;
        save(); break;
      }
    }
    try {
      assert.equal(packet.requestId, entry.requestId, 'WASM request boundary substitution');
      assert.equal(packet.workerIdentity, workerIdentity, 'Diagnostic WASM worker replaced');
      assert.equal(packet.type, 'foundation', 'WASM fixture failed: ' + packet.message);
      entry.passed = validateSelectedRecoveryReport(packet.report, fixture, profile);
      entry.identityMatched = true;
      entry.summary = { cases: 1, passedCases: entry.passed ? 1 : 0, profileApplied: true };
    } catch (error) { entry.failure = String(error); }
    save();
  }
  const file = join(output, 'wasm-recovery.json'), temporary = file + '.tmp';
  const fd = openSync(temporary, 'wx');
  try {
    for (const fragment of aggregateFragments(entries, profile)()) {
      const bytes = Buffer.from(fragment);
      for (let offset = 0; offset < bytes.length;) offset += writeSync(fd, bytes, offset, bytes.length - offset);
    }
  } finally { closeSync(fd); }
  renameSync(temporary, file);
  const storageEvidence = archiveGeneratedReport(file, output);
  const passed = entries.length === plan.length && entries.every(value => value.completed && value.identityMatched && value.passed);
  save();
  return { kind: 'recovery', path: file, sha256: storageEvidence.decodedSha256, storageEvidence,
    summary: { cases: execution.completedCaseReports, passedCases: entries.filter(value => value.passed).length,
      profileApplied: !execution.partial && entries.every(value => value.identityMatched), releaseAccepted: false },
    passed, partial: execution.partial, execution };
}

// Finalization does not trust completion flags or an aggregate alone. Re-read
// every exact archive path and independently reproduce its aggregate bytes.
export function verifyWasmRecoveryCases(output, entries, execution, profile, fixtures, aggregateStorage, expectedIdentity = {}) {
  const plan = recoveryCasePlan(fixtures);
  assert.ok(Array.isArray(entries) && entries.length === plan.length, 'Missing selected WASM case coverage');
  assert.equal(execution?.expectedCases, plan.length);
  assert.equal(execution.completedAttempts, plan.length); assert.equal(execution.completedCaseReports, plan.length);
  assert.equal(execution.partial, false); assert.equal(execution.transportFailure, undefined);
  assert.equal(execution.archiveFailure, undefined);
  assert.equal(execution.releaseAccepted, false);
  for (const [name, value] of Object.entries(expectedIdentity)) assert.equal(execution[name], value, 'Selected WASM frozen input identity changed');
  assert.ok(typeof execution.workerIdentity === 'string' && execution.workerIdentity.length > 0);
  const checkpointFile = join(output, 'wasm-recovery-case-execution.json');
  containedArchive(output, checkpointFile);
  assert.deepEqual(readPhysicalJson(checkpointFile), { schema: 1, ...execution, cases: entries }, 'Selected WASM checkpoint changed');
  for (const [index, fixture] of plan.entries()) {
    const entry = entries[index], expectedPath = join(output, 'wasm-recovery-cases', fixture.key + '.json');
    assert.equal(entry.fixtureId, fixture.id, 'Selected WASM case order or coverage differs');
    assert.equal(entry.heading, Math.fround(fixture.heading)); assert.equal(entry.ordinal, fixture.ordinal);
    assert.equal(entry.requestId, fixture.ordinal); assert.equal(entry.responseRequestId, entry.requestId, 'WASM request boundary changed');
    assert.equal(entry.workerIdentity, execution.workerIdentity, 'Selected WASM worker identity changed');
    assert.equal(entry.path, expectedPath, 'Selected WASM report path substitution');
    assert.ok(entry.completed && entry.identityMatched && entry.passed && entry.failure === undefined, 'Failed selected WASM case');
    assert.equal(entry.releaseAccepted, false);
    containedArchive(output, expectedPath + '.gz'); containedArchive(output, expectedPath + '.archive.json');
    const retained = physicalReportBytes(expectedPath, entry.storageEvidence);
    assert.equal(retained.evidence.storage, 'gzip', 'Selected WASM report must be archived');
    assert.equal(retained.evidence.decodedSha256, entry.sha256, 'Selected WASM case hash changed');
    assert.equal(validateSelectedRecoveryReport(JSON.parse(retained.bytes), fixture, profile), true, 'Selected WASM physical evidence failed');
  }
  assert.equal(aggregateStorage.file, join(output, 'wasm-recovery.json'), 'WASM aggregate path substitution');
  containedArchive(output, aggregateStorage.archiveFile); containedArchive(output, aggregateStorage.metadataFile);
  assert.equal(physicalReportBytes(aggregateStorage.file, aggregateStorage).evidence.storage, 'gzip');
  const hash = createHash('sha256');
  for (const fragment of aggregateFragments(entries, profile)()) hash.update(fragment);
  assert.equal(hash.digest('hex'), aggregateStorage.decodedSha256, 'WASM aggregate differs from retained selected cases');
  return true;
}
