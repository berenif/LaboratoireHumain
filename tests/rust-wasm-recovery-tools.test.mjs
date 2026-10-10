import assert from 'node:assert/strict';
import { copyFileSync, existsSync, mkdirSync, mkdtempSync, readFileSync, readdirSync, renameSync, symlinkSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import test from 'node:test';
import { gzipSync } from 'node:zlib';
import { collectWasmRecoveryCases, verifyWasmRecoveryCases } from '../scripts/rust-wasm-recovery-tools.mjs';
import { deriveRecoveryBrowserLimits, recoveryBrowserLimits, recoveryContractSha256 } from '../scripts/rust-recovery-browser-limits.mjs';
import { physicalReportEvidence, readPhysicalJson } from '../scripts/rust-physical-archive-tools.mjs';
import { recoveryCasePlan } from '../scripts/rust-recovery-case-tools.mjs';
import { sha256 } from '../scripts/rust-release-tools.mjs';

const fixtures = JSON.parse(readFileSync('rust/crates/sim/data/recovery-v1.json')).cases;
const profile = JSON.parse(readFileSync('rust/crates/sim/data/recovery-profile-v24.json'));
const contract = JSON.parse(readFileSync('rust/crates/sim/data/recovery-contract-v1.json'));
const workerIdentity = 'one-uninitialized-diagnostic-worker';
const identity = { sourceFingerprint: 'fixture-frozen-source', inputProfileSha256: sha256(readFileSync('rust/crates/sim/data/recovery-profile-v24.json')),
  contractSha256: recoveryContractSha256 };
const f32 = value => typeof value === 'number' ? Math.fround(value) : value && typeof value === 'object'
  ? Object.fromEntries(Object.entries(value).map(([key, child]) => [key, f32(child)])) : value;
function outputDirectory() {
  const parent = resolve('evidence/recovery-wasm-cases-draft-2026-10-10/test-output');
  mkdirSync(parent, { recursive: true }); return mkdtempSync(join(parent, 'cases-'));
}
function packet(message, mutate = () => {}) {
  const fixture = fixtures.find(value => value.id === message.fixtureId);
  const result = { type: 'foundation', requestId: message.requestId, workerIdentity,
    report: { kind: 'recovery', fixtureId: fixture.id, fixtureHeading: Math.fround(fixture.heading), profile: f32(profile),
      cases: [{ id: fixture.id, heading: Math.fround(fixture.heading), passed: true, dynamicOwnership: true,
        failure: null, handoffStandingSubsteps: recoveryBrowserLimits.stableSubsteps,
        stableReturnTimeS: 20, recoveryLimitS: 25, stableObservationS: 1,
        trace: [{ retainedFixture: fixture.id }] }], releaseAccepted: false } };
  mutate(result); return result;
}
async function collect(output, request, checkpoint) {
  return collectWasmRecoveryCases({ output, profile, fixtureCases: fixtures, workerIdentity, identity, request, checkpoint });
}
function checkpoint(output) { return readPhysicalJson(join(output, 'wasm-recovery-case-execution.json')); }
function verify(output, result) {
  const receipt = checkpoint(output);
  return verifyWasmRecoveryCases(output, receipt.cases, receiptToExecution(receipt), profile, fixtures, result.storageEvidence, identity);
}
function receiptToExecution(receipt) {
  const execution = { ...receipt }; delete execution.schema; delete execution.cases; return execution;
}

test('frozen browser limits derive exact integrated deadlines, held drag and repeated cycles', () => {
  assert.deepEqual(recoveryBrowserLimits, { substepHz: 240, substepsPerTick: 4, tickHz: 60,
    recoveryLimitSubsteps: 6000, stableSubsteps: 240, stableTicks: 60, stableActualMs: 1000,
    repeatedCycles: 5, dragMetres: 0.02, dragActualMs: 250, dragSubsteps: 60 });
  assert.throws(() => deriveRecoveryBrowserLimits(contract, { ...profile, dt_s: 0.0041666 }), /duration/);
  assert.throws(() => deriveRecoveryBrowserLimits(contract, { ...profile, tick_substeps: 3 }), /tick clock/);
  assert.throws(() => deriveRecoveryBrowserLimits({ ...contract, acceptance: { ...contract.acceptance, stableObservationS: 0.001 } }, profile), /exact integrated/);
  assert.equal(deriveRecoveryBrowserLimits({ ...contract, acceptance: { ...contract.acceptance, repeatedCycles: 7 } }, profile).repeatedCycles, 7);
});

test('serial selected requests checkpoint verified gzip before the next case and reconstruct the full report', async () => {
  const output = outputDirectory(); let outstanding = 0, maximum = 0, completed = 0;
  const result = await collect(output, async message => {
    assert.equal(message.requestId, completed + 1);
    assert.equal(message.fixtureId, fixtures[completed].id);
    if (completed) {
      const previous = checkpoint(output);
      assert.equal(previous.completedCaseReports, completed);
      const retained = previous.cases.find(value => value.requestId === completed);
      assert.equal(readPhysicalJson(retained.path, retained.storageEvidence).cases.length, 1);
      assert.ok(!existsSync(retained.path), 'Raw duplicate is removed only after verified archive');
    }
    outstanding++; maximum = Math.max(maximum, outstanding);
    await new Promise(resolve => setImmediate(resolve));
    outstanding--; completed++; return packet(message);
  });
  assert.equal(maximum, 1); assert.equal(completed, 24); assert.equal(result.passed, true); assert.equal(result.partial, false);
  assert.equal(verify(output, result), true);
  const original = readPhysicalJson(result.path, result.storageEvidence);
  assert.equal(original.kind, 'recovery'); assert.deepEqual(original.cases.map(value => value.id), fixtures.map(value => value.id));
  const retained = checkpoint(output);
  assert.ok(retained.cases.every(value => value.report === undefined && value.storageEvidence.storage === 'gzip'));
  assert.ok(JSON.stringify(retained).length < 40000, 'Receipt contains descriptors rather than duplicate physical reports');
});

test('actual archive containment rejects a linked case directory despite unchanged bytes and descriptor paths', async t => {
  const output = outputDirectory(), result = await collect(output, message => packet(message));
  const receipt = checkpoint(output), outside = outputDirectory(), directory = join(output, 'wasm-recovery-cases');
  for (const file of readdirSync(directory)) copyFileSync(join(directory, file), join(outside, file));
  renameSync(directory, directory + '-original');
  try { symlinkSync(outside, directory, process.platform === 'win32' ? 'junction' : 'dir'); }
  catch (error) {
    renameSync(directory + '-original', directory);
    if (['EPERM', 'EACCES', 'ENOTSUP'].includes(error.code)) {
      t.skip('This filesystem does not permit directory links; path substitution containment remains covered.'); return;
    }
    throw error;
  }
  assert.throws(() => verifyWasmRecoveryCases(output, receipt.cases, receiptToExecution(receipt), profile, fixtures,
    result.storageEvidence), /archive escaped browser output|linked WASM archive ancestor/);
});

test('a failed physical case is retained and later serial cases still run', async () => {
  const output = outputDirectory(); let count = 0;
  const result = await collect(output, message => { count++; return packet(message, result => {
    if (message.requestId === 2) { result.report.cases[0].passed = false; result.report.cases[0].failure = { quantity: 'physical-failure' }; }
  }); });
  const receipt = checkpoint(output);
  assert.equal(count, 24); assert.equal(result.passed, false); assert.equal(result.partial, false);
  assert.equal(receipt.cases[1].identityMatched, true); assert.equal(receipt.cases[1].passed, false);
  assert.equal(readPhysicalJson(receipt.cases[1].path).cases[0].failure.quantity, 'physical-failure');
  assert.throws(() => verify(output, result), /Failed selected WASM case/);
});

test('a timeout leaves earlier cases, failed request and partial aggregate durable without replacing the worker', async () => {
  const output = outputDirectory(); const requests = [];
  const result = await collect(output, message => {
    requests.push(message.requestId);
    if (message.requestId === 4) throw new Error('fixture transport timed out');
    return packet(message);
  });
  const receipt = checkpoint(output);
  assert.deepEqual(requests, [1, 2, 3, 4]); assert.equal(result.passed, false); assert.equal(result.partial, true);
  assert.equal(receipt.completedAttempts, 3); assert.equal(receipt.completedCaseReports, 3);
  for (const [name, value] of Object.entries(identity)) assert.equal(receipt[name], value);
  assert.equal(receipt.cases[3].transportFailure, true);
  assert.deepEqual(readPhysicalJson(result.path).cases.map(value => value.id), fixtures.slice(0, 3).map(value => value.id));
  assert.ok(receipt.cases.slice(0, 3).every(value => value.workerIdentity === workerIdentity));
  assert.throws(() => verify(output, result), /Missing selected WASM case coverage/);
});

test('request, worker, fixture, heading, profile and single-case substitutions cannot qualify', async () => {
  const mutations = [
    value => { value.requestId++; }, value => { value.workerIdentity = 'replacement'; },
    value => { value.report.fixtureId = fixtures[1].id; }, value => { value.report.fixtureHeading += 0.1; },
    value => { value.report.cases[0].id = fixtures[1].id; }, value => { value.report.cases.push(value.report.cases[0]); },
    value => { value.report.profile = { ...value.report.profile, extraUnpinnedField: true }; },
    value => { value.report.profile.solver_iterations++; },
    value => { value.report.cases[0].recoveryLimitS = 26; },
    value => { value.report.cases[0].stableObservationS = 0.5; },
  ];
  for (const mutate of mutations) {
    const output = outputDirectory();
    const result = await collect(output, message => packet(message, value => { if (message.requestId === 1) mutate(value); }));
    const receipt = checkpoint(output);
    assert.equal(result.passed, false); assert.equal(receipt.cases[0].identityMatched, false);
    assert.ok(receipt.cases[0].storageEvidence, 'Substituted physical reply remains inspectable');
    assert.equal(receipt.completedCaseReports, 24);
    assert.throws(() => verify(output, result), /Failed selected WASM case|request boundary changed|worker identity changed/);
  }
});

test('independent selected-case physics requires finite bounded return and uninterrupted ordinary handoff', async () => {
  for (const changed of [{ stableReturnTimeS: undefined }, { stableReturnTimeS: null },
    { stableReturnTimeS: 25.001 }, { stableReturnTimeS: -1 }, { handoffStandingSubsteps: 239 },
    { dynamicOwnership: false }, { failure: { quantity: 'handoff-interruption' } }]) {
    const output = outputDirectory();
    const result = await collect(output, message => packet(message, value => {
      if (message.requestId === 1) Object.assign(value.report.cases[0], changed);
    }));
    assert.equal(result.passed, false); assert.equal(checkpoint(output).cases[0].passed, false);
    assert.throws(() => verify(output, result), /Failed selected WASM case/);
  }
});

test('independent case verification rejects changed report bytes even after all archive and receipt hashes refresh', async () => {
  const output = outputDirectory(), result = await collect(output, message => packet(message));
  const receipt = checkpoint(output), entry = receipt.cases[0], report = readPhysicalJson(entry.path);
  report.cases[0].id = fixtures[1].id;
  const bytes = Buffer.from(JSON.stringify(report)), compressed = gzipSync(bytes);
  const metadata = JSON.parse(readFileSync(entry.storageEvidence.metadataFile));
  Object.assign(metadata, { archiveBytes: compressed.length, decodedBytes: bytes.length,
    archiveSha256: sha256(compressed), decodedSha256: sha256(bytes) });
  writeFileSync(entry.storageEvidence.archiveFile, compressed); writeFileSync(entry.storageEvidence.metadataFile, JSON.stringify(metadata));
  entry.storageEvidence = physicalReportEvidence(entry.path); entry.sha256 = entry.storageEvidence.decodedSha256;
  writeFileSync(join(output, 'wasm-recovery-case-execution.json'), JSON.stringify(receipt));
  assert.throws(() => verifyWasmRecoveryCases(output, receipt.cases, receiptToExecution(receipt), profile, fixtures, result.storageEvidence), /case substitution/);
});

test('archive path substitution and checkpoint rewriting cannot bypass exact browser containment', async () => {
  const output = outputDirectory(), result = await collect(output, message => packet(message));
  const receipt = checkpoint(output);
  receipt.cases[0].path = join(outputDirectory(), recoveryCasePlan(fixtures)[0].key + '.json');
  writeFileSync(join(output, 'wasm-recovery-case-execution.json'), JSON.stringify(receipt));
  assert.throws(() => verifyWasmRecoveryCases(output, receipt.cases, receiptToExecution(receipt), profile, fixtures, result.storageEvidence), /report path substitution/);
  const intact = await collect(outputDirectory(), message => packet(message));
  const intactRoot = resolve(intact.path, '..'), original = checkpoint(intactRoot);
  original.cases[0].storageEvidence.archiveFile = join(output, 'outside.gz');
  writeFileSync(join(intactRoot, 'wasm-recovery-case-execution.json'), JSON.stringify(original));
  assert.throws(() => verifyWasmRecoveryCases(intactRoot, original.cases, receiptToExecution(original), profile, fixtures, intact.storageEvidence), /storage identity mismatch/);
});
