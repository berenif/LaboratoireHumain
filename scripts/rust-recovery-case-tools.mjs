import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { closeSync, copyFileSync, existsSync, openSync, readFileSync, renameSync, writeSync } from 'node:fs';
import { join } from 'node:path';
import { archiveGeneratedReport, physicalReportBytes } from './rust-physical-archive-tools.mjs';

const digest = file => createHash('sha256').update(readFileSync(file)).digest('hex');
export function recoveryCasePlan(fixtures) {
  assert.ok(Array.isArray(fixtures) && fixtures.length === 24, 'Expected 24 frozen recovery fixtures');
  assert.equal(new Set(fixtures.map(fixture => fixture.id)).size, 24, 'Duplicate recovery fixture identity');
  return fixtures.map((fixture, index) => {
    assert.match(fixture.id, /^[a-z0-9-]+$/, 'Unsafe recovery fixture path');
    assert.ok(Number.isFinite(fixture.heading), 'Invalid fixture heading');
    const key = `${String(index + 1).padStart(2, '0')}-${fixture.id}`;
    return { ...fixture, ordinal: index + 1, key, runSuffix: `fixture-${key}` };
  });
}
export function inspectRecoveryCase(directory, fixture) {
  const reportFile = join(directory, 'recovery.json'), profileFile = join(directory, 'profile.json');
  const result = { fixtureId: fixture.id, ordinal: fixture.ordinal, directory,
    reportFile, profileFile, reportSha256: null, reportStorageEvidence: null,
    profileSha256: existsSync(profileFile) ? digest(profileFile) : null,
    jsonReadable: false, identityMatched: false, reportedCases: 0 };
  try {
    const retained = physicalReportBytes(reportFile);
    result.reportSha256 = retained.evidence.decodedSha256; result.reportStorageEvidence = retained.evidence;
    const rows = JSON.parse(retained.bytes.toString('utf8'));
    assert.ok(Array.isArray(rows), 'Recovery report must be an array');
    result.jsonReadable = true; result.reportedCases = rows.length;
    result.identityMatched = rows.length === 1 && rows[0].id === fixture.id
      && rows[0].heading === Math.fround(fixture.heading);
  } catch (error) { result.reportError = String(error); }
  return result;
}
// Each case's exact original bytes are already durable before aggregation.
// Stream one parsed case at a time so 24 large traces do not accumulate in JS.
export function assembleRecoveryCases(destination, cases, currentRunRoot) {
  const file = join(destination, 'recovery.json'), temporary = file + '.tmp';
  const fd = openSync(temporary, 'wx');
  let rowCount = 0;
  const write = value => {
    const bytes = Buffer.from(value);
    for (let offset = 0; offset < bytes.length;) offset += writeSync(fd, bytes, offset, bytes.length - offset);
  };
  try {
    write('[\n');
    for (const entry of cases) {
      if (!entry.jsonReadable) continue;
      const retained = physicalReportBytes(entry.reportFile, entry.reportStorageEvidence);
      assert.equal(retained.evidence.decodedSha256, entry.reportSha256, 'Retained case report changed before aggregation');
      const rows = JSON.parse(retained.bytes.toString('utf8'));
      assert.ok(Array.isArray(rows));
      for (const row of rows) {
        if (rowCount++) write(',\n');
        write(JSON.stringify(row, null, 2));
      }
    }
    write('\n]\n');
  } finally { closeSync(fd); }
  renameSync(temporary, file);
  const storageEvidence = archiveGeneratedReport(file, currentRunRoot);
  const emitted = cases.find(entry => entry.profileSha256 !== null);
  if (emitted) {
    assert.equal(digest(emitted.profileFile), emitted.profileSha256, 'Retained emitted profile changed before aggregation');
    copyFileSync(emitted.profileFile, join(destination, 'profile.json'));
  }
  return { file, sha256: storageEvidence.decodedSha256, storageEvidence, actualCases: rowCount,
    completeCaseEvidence: cases.length === 24 && cases.every(entry => entry.jsonReadable && entry.identityMatched),
    missingFixtures: cases.filter(entry => !entry.jsonReadable).map(entry => entry.fixtureId) };
}
