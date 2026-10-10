import assert from 'node:assert/strict';
import { existsSync, mkdirSync, mkdtempSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import test from 'node:test';
import { assembleRecoveryCases, inspectRecoveryCase, recoveryCasePlan } from '../scripts/rust-recovery-case-tools.mjs';
import { archiveGeneratedReport, readPhysicalJson } from '../scripts/rust-physical-archive-tools.mjs';

const fixtures = Array.from({ length: 24 }, (_, index) => ({ id: `case-${index + 1}`, heading: index % 2 ? Math.PI / 3 : 0 }));
function example() {
  mkdirSync(resolve('evidence'), { recursive: true });
  const root = mkdtempSync(resolve('evidence/rust-recovery-case-check-'));
  return { root, plan: recoveryCasePlan(fixtures) };
}
function emit(root, fixture, row = { id: fixture.id, heading: Math.fround(fixture.heading), passed: true }) {
  const directory = join(root, fixture.key); mkdirSync(directory);
  writeFileSync(join(directory, 'profile.json'), '{"id":"pinned-fixture-profile"}\n');
  if (row !== null) {
    writeFileSync(join(directory, 'recovery.json'), JSON.stringify([row]));
    archiveGeneratedReport(join(directory, 'recovery.json'), root);
  }
  return inspectRecoveryCase(directory, fixture);
}
test('case planning is stable, unique and rejects unsafe or incomplete fixture sets', () => {
  const plan = recoveryCasePlan(fixtures);
  assert.equal(plan[0].runSuffix, 'fixture-01-case-1'); assert.equal(plan[23].key, '24-case-24');
  assert.throws(() => recoveryCasePlan(fixtures.slice(1)), /24/);
  assert.throws(() => recoveryCasePlan(fixtures.map(row => ({ ...row, id: 'same' }))), /Duplicate/);
  assert.throws(() => recoveryCasePlan(fixtures.map((row, index) => index === 0 ? { ...row, id: '../escape' } : row)), /Unsafe/);
});
test('completed and failed case files survive a later missing report; partial aggregate cannot imply full coverage', () => {
  const { root, plan } = example();
  const completed = emit(root, plan[0]);
  const failed = emit(root, plan[1], { id: plan[1].id, heading: Math.fround(plan[1].heading), passed: false });
  const missing = emit(root, plan[2], null);
  assert.equal(missing.jsonReadable, false); assert.equal(missing.profileSha256.length, 64);
  const aggregate = assembleRecoveryCases(root, [completed, failed, missing], root);
  assert.equal(aggregate.completeCaseEvidence, false); assert.equal(aggregate.actualCases, 2);
  assert.deepEqual(aggregate.missingFixtures, [plan[2].id]);
  const rows = readPhysicalJson(aggregate.file, aggregate.storageEvidence); assert.deepEqual(rows.map(row => row.passed), [true, false]);
  assert.equal(readPhysicalJson(failed.reportFile, failed.reportStorageEvidence)[0].passed, false);
  assert.equal(readPhysicalJson(completed.reportFile, completed.reportStorageEvidence)[0].id, plan[0].id);
  assert.equal(existsSync(aggregate.file), false); assert.equal(existsSync(failed.reportFile), false);
});
test('two complete serial case collections assemble identical physical reports in frozen fixture order', () => {
  const first = example(), second = example();
  const a = assembleRecoveryCases(first.root, first.plan.map(fixture => emit(first.root, fixture)), first.root);
  const b = assembleRecoveryCases(second.root, second.plan.map(fixture => emit(second.root, fixture)), second.root);
  assert.equal(a.completeCaseEvidence, true); assert.equal(a.actualCases, 24); assert.equal(a.sha256, b.sha256);
});
test('case identity or retained report mutation cannot silently qualify aggregation', () => {
  const { root, plan } = example();
  const wrong = emit(root, plan[0], { id: 'substitute', heading: 0, passed: true });
  assert.equal(wrong.identityMatched, false);
  const original = emit(root, plan[1]); writeFileSync(original.reportFile, '[]');
  assert.throws(() => assembleRecoveryCases(root, [original], root), /differs|changed before aggregation/);
});
