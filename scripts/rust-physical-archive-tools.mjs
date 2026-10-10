import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, lstatSync, readFileSync, realpathSync, renameSync, unlinkSync, writeFileSync } from 'node:fs';
import { basename, dirname, isAbsolute, relative, resolve, sep } from 'node:path';
import { gunzipSync, gzipSync } from 'node:zlib';

const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const paths = file => ({ file: resolve(file), archiveFile: resolve(file) + '.gz', metadataFile: resolve(file) + '.archive.json' });
function safeGeneratedFile(file, currentRunRoot) {
  assert.ok(isAbsolute(file) && isAbsolute(currentRunRoot), 'Archive inputs must use absolute paths');
  const root = realpathSync(currentRunRoot), workspace = realpathSync(process.cwd());
  const workspaceRelative = relative(workspace, root), local = relative(root, realpathSync(file));
  for (const value of [workspaceRelative, local]) assert.ok(value && !isAbsolute(value) && value !== '..'
    && !value.startsWith(`..${sep}`), 'Archive target must stay in the current workspace output');
  assert.ok(lstatSync(file).isFile() && !lstatSync(file).isSymbolicLink(), 'Refuse linked or nonfile report');
  for (let current = dirname(file);; current = dirname(current)) {
    assert.ok(!lstatSync(current).isSymbolicLink(), 'Refuse linked report ancestor');
    if (resolve(current) === resolve(currentRunRoot)) break;
    assert.notEqual(current, dirname(current), 'Archive root is not an ancestor');
  }
}

export const physicalReportAvailable = file => existsSync(file) || existsSync(file + '.archive.json');
export function physicalReportBytes(file, expectedEvidence) {
  const selected = paths(file);
  let bytes, evidence;
  if (existsSync(selected.metadataFile)) {
    assert.ok(lstatSync(selected.metadataFile).isFile() && !lstatSync(selected.metadataFile).isSymbolicLink(), 'Refuse linked archive metadata');
    const metadataBytes = readFileSync(selected.metadataFile), metadata = JSON.parse(metadataBytes);
    assert.equal(metadata.schema, 1); assert.equal(metadata.encoding, 'gzip');
    assert.equal(metadata.releaseAccepted, false);
    assert.equal(metadata.originalName, basename(selected.file));
    assert.equal(metadata.archiveName, basename(selected.archiveFile), 'Archive path substitution');
    assert.ok(Number.isSafeInteger(metadata.decodedBytes) && metadata.decodedBytes >= 0 && metadata.decodedBytes < 2 ** 31);
    assert.ok(lstatSync(selected.archiveFile).isFile() && !lstatSync(selected.archiveFile).isSymbolicLink(), 'Refuse linked report archive');
    const container = readFileSync(selected.archiveFile);
    assert.equal(hash(container), metadata.archiveSha256, 'Physical archive container SHA-256 mismatch');
    assert.equal(container.length, metadata.archiveBytes, 'Physical archive container length mismatch');
    bytes = gunzipSync(container, { maxOutputLength: metadata.decodedBytes + 1 });
    assert.equal(bytes.length, metadata.decodedBytes, 'Physical archive decoded length mismatch');
    assert.equal(hash(bytes), metadata.decodedSha256, 'Physical archive decoded report SHA-256 mismatch');
    if (existsSync(selected.file)) {
      assert.ok(lstatSync(selected.file).isFile() && !lstatSync(selected.file).isSymbolicLink(), 'Refuse linked retained report');
      assert.ok(readFileSync(selected.file).equals(bytes), 'Retained raw report differs from verified archive');
    }
    evidence = { ...selected, storage: 'gzip', archiveSha256: metadata.archiveSha256,
      decodedSha256: metadata.decodedSha256, metadataSha256: hash(metadataBytes), decodedBytes: bytes.length };
  } else {
    assert.ok(lstatSync(selected.file).isFile() && !lstatSync(selected.file).isSymbolicLink(), 'Refuse linked raw report');
    bytes = readFileSync(selected.file);
    evidence = { ...selected, storage: 'raw', archiveSha256: null, decodedSha256: hash(bytes),
      metadataSha256: null, decodedBytes: bytes.length };
  }
  if (expectedEvidence) assert.deepEqual(evidence, expectedEvidence, 'Physical report storage identity mismatch');
  return { bytes, evidence };
}
export const physicalReportEvidence = file => physicalReportBytes(file).evidence;
export const readPhysicalJson = (file, evidence) => JSON.parse(physicalReportBytes(file, evidence).bytes.toString('utf8'));

// Only invoke on a file produced by this fresh qualification operation.
// Any failure before the final unlink retains the original report bytes.
export function archiveGeneratedReport(file, currentRunRoot) {
  safeGeneratedFile(file, currentRunRoot);
  const selected = paths(file), temporary = selected.archiveFile + '.tmp', metadataTemporary = selected.metadataFile + '.tmp';
  for (const target of [selected.archiveFile, selected.metadataFile, temporary, metadataTemporary]) {
    assert.ok(!existsSync(target), 'Refuse existing report archive or checkpoint');
  }
  const original = readFileSync(selected.file), container = gzipSync(original, { level: 9 });
  assert.ok(gunzipSync(container).equals(original), 'Archive failed byte-perfect round trip');
  writeFileSync(temporary, container, { flag: 'wx' });
  const stored = readFileSync(temporary);
  assert.ok(stored.equals(container) && gunzipSync(stored).equals(original), 'Stored archive failed byte-perfect verification');
  const metadata = { schema: 1, encoding: 'gzip', originalName: basename(selected.file), archiveName: basename(selected.archiveFile),
    decodedBytes: original.length, decodedSha256: hash(original), archiveBytes: container.length, archiveSha256: hash(container),
    releaseAccepted: false };
  writeFileSync(metadataTemporary, JSON.stringify(metadata, null, 2) + '\n', { flag: 'wx' });
  renameSync(temporary, selected.archiveFile); renameSync(metadataTemporary, selected.metadataFile);
  const verified = physicalReportBytes(selected.file);
  assert.ok(verified.bytes.equals(original), 'Committed archive differs from original report');
  safeGeneratedFile(selected.file, currentRunRoot);
  assert.ok(readFileSync(selected.file).equals(original), 'Raw report changed before removal');
  unlinkSync(selected.file);
  // Verify the archive remains sufficient after the generated raw file is removed.
  assert.ok(physicalReportBytes(selected.file, verified.evidence).bytes.equals(original));
  return verified.evidence;
}
