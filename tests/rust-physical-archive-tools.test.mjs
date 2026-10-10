import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, mkdtempSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { gzipSync } from 'node:zlib';
import test from 'node:test';
import { archiveGeneratedReport, physicalReportBytes, readPhysicalJson } from '../scripts/rust-physical-archive-tools.mjs';

const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
function example(bytes = Buffer.from('[\n {"id":"fixture","passed":false,"trace":[1,2,3]}\n]\n')) {
  mkdirSync(resolve('evidence'), { recursive: true });
  const root = mkdtempSync(resolve('evidence/rust-report-archive-check-'));
  const file = join(root, 'report.json'); writeFileSync(file, bytes);
  return { root, file, bytes };
}

test('completed failure reports archive byte-perfect originals with separately verified container and decoded hashes', () => {
  const { root, file, bytes } = example();
  const evidence = archiveGeneratedReport(file, root);
  assert.equal(existsSync(file), false); assert.equal(evidence.storage, 'gzip');
  assert.equal(evidence.decodedSha256, sha256(bytes));
  assert.equal(evidence.archiveSha256, sha256(readFileSync(evidence.archiveFile)));
  assert.ok(physicalReportBytes(file, evidence).bytes.equals(bytes));
  assert.equal(readPhysicalJson(file, evidence)[0].passed, false);
});

test('corrupted containers and altered decoded reports fail independently', () => {
  const a = example(), b = example();
  const archivedA = archiveGeneratedReport(a.file, a.root);
  writeFileSync(archivedA.archiveFile, 'corrupt');
  assert.throws(() => physicalReportBytes(a.file, archivedA), /container SHA-256/);
  const archivedB = archiveGeneratedReport(b.file, b.root);
  const changed = gzipSync(Buffer.from(b.bytes.toString().replace('false', 'true ')));
  const metadata = JSON.parse(readFileSync(archivedB.metadataFile));
  metadata.archiveSha256 = sha256(changed); metadata.archiveBytes = changed.length;
  writeFileSync(archivedB.archiveFile, changed); writeFileSync(archivedB.metadataFile, JSON.stringify(metadata));
  assert.throws(() => physicalReportBytes(b.file), /decoded report SHA-256/);
});

test('a fully substituted valid archive still fails against its frozen storage identity', () => {
  const { root, file, bytes } = example();
  const evidence = archiveGeneratedReport(file, root);
  const replacement = Buffer.from(bytes.toString().replace('fixture', 'changed'));
  const container = gzipSync(replacement);
  const metadata = JSON.parse(readFileSync(evidence.metadataFile));
  Object.assign(metadata, { archiveSha256: sha256(container), archiveBytes: container.length,
    decodedSha256: sha256(replacement), decodedBytes: replacement.length });
  writeFileSync(evidence.archiveFile, container); writeFileSync(evidence.metadataFile, JSON.stringify(metadata));
  assert.ok(physicalReportBytes(file).bytes.equals(replacement));
  assert.throws(() => physicalReportBytes(file, evidence), /storage identity/);
});

test('archival refuses foreign or preexisting targets and retains generated raw bytes on failure', () => {
  const a = example(), foreign = example();
  assert.throws(() => archiveGeneratedReport(foreign.file, a.root), /current workspace output/);
  assert.ok(readFileSync(foreign.file).equals(foreign.bytes));
  writeFileSync(a.file + '.gz', 'previous archive');
  assert.throws(() => archiveGeneratedReport(a.file, a.root), /existing report archive/);
  assert.ok(readFileSync(a.file).equals(a.bytes));
});

test('partial non-JSON output is preserved losslessly while JSON admission still fails', () => {
  const { root, file, bytes } = example(Buffer.from('[{"partial":'));
  const evidence = archiveGeneratedReport(file, root);
  assert.ok(physicalReportBytes(file, evidence).bytes.equals(bytes));
  assert.throws(() => readPhysicalJson(file, evidence), SyntaxError);
});
