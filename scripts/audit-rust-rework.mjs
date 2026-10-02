// Source preservation and evidence audit; this is not physical acceptance.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';

const base = 'evidence/rust-rework-20261001/baseline';
const frozen = process.argv[2];
const out = path.resolve(process.argv[3] ?? '');
assert.ok(frozen && process.argv[3] && !fs.existsSync(out)
  && out.startsWith(`${process.cwd()}${path.sep}`), 'Provide frozen evidence and a fresh workspace audit directory');
fs.mkdirSync(out);
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const read = file => JSON.parse(fs.readFileSync(file));
const manifest = read(`${base}/manifest.json`);
const execution = read(`${frozen}/execution.json`);
assert.ok(execution.finished && execution.sourceUnchanged, 'Frozen run must have completed without source changes');
function command(name, executable, args) {
  const r = spawnSync(executable, args, { windowsHide: true, maxBuffer: 16 * 1024 * 1024, timeout: 60000 });
  fs.writeFileSync(path.join(out, `${name}.log`), Buffer.concat([r.stdout ?? Buffer.alloc(0), r.stderr ?? Buffer.alloc(0)]));
  assert.equal(r.status, 0, `${name}: ${r.error?.message ?? `exit ${r.status}`}`);
  return r.stdout;
}
const currentRevision = command('revision', 'git', ['rev-parse', 'HEAD']).toString().trim();
assert.equal(currentRevision, manifest.revision);
const files = manifest.files.map(f => ({ name: f.name,
  currentMatches: fs.existsSync(f.name) && hash(fs.readFileSync(f.name)) === f.sha256,
  archiveMatches: hash(fs.readFileSync(`${base}/source/${f.name}`)) === f.sha256,
}));
assert.ok(files.every(f => f.currentMatches && f.archiveMatches), 'Original source changed or archive corrupted');
const tracked = command('tracked-diff', 'git', ['diff', '--binary', 'HEAD']);
const index = command('index-diff', 'git', ['diff', '--binary', '--cached']);
assert.ok(tracked.equals(fs.readFileSync(`${base}/tracked.patch`)), 'Original tracked diff changed');
assert.ok(index.equals(fs.readFileSync(`${base}/index.patch`)), 'Original index diff changed');
const foundation = Object.entries(execution.sourceBefore).map(([name, sha256]) => ({ name, sha256,
  currentMatches: hash(fs.readFileSync(name)) === sha256,
  archiveMatches: hash(fs.readFileSync(`${frozen}/source/${name}`)) === sha256,
}));
assert.ok(foundation.every(f => f.currentMatches && f.archiveMatches), 'Final foundation/source snapshot changed');
// Older foundation manifests list only native files. Newer ones also retain
// independently repeated terrain and striker reports.
const resultGroups = Array.isArray(execution.resultFiles)
  ? { native: execution.resultFiles } : execution.resultFiles;
const resultEquality = Object.entries(resultGroups).flatMap(([check, files]) => files.map(file => {
  const a = fs.readFileSync(`${frozen}/${check}-1/${file}`), b = fs.readFileSync(`${frozen}/${check}-2/${file}`);
  return { check, file, identical: a.equals(b), sha256: hash(a) };
}));
assert.ok(execution.repeatable && resultEquality.every(f => f.identical), 'Native results differ');
// Reuse the prior extraction of the checksum-verified original registry archive.
// Walk this explicitly named directory only, never evidence junctions or caches.
const original = 'evidence/rust-rework-20261001/final-audit/unmodified-package/rapier3d-0.35.0';
const vendor = 'rust/vendor/rapier3d';
const vendorFiles = [];
function visit(directory, relative = '') {
  for (const e of fs.readdirSync(directory, { withFileTypes: true })) {
    assert.ok(!e.isSymbolicLink(), `Refuse link: ${directory}/${e.name}`);
    const name = relative ? `${relative}/${e.name}` : e.name;
    if (e.isDirectory()) visit(`${directory}/${e.name}`, name);
    else vendorFiles.push(name);
  }
}
visit(original);
const modified = vendorFiles.flatMap(name => {
  const before = hash(fs.readFileSync(`${original}/${name}`));
  const after = hash(fs.readFileSync(`${vendor}/${name}`));
  return before === after ? [] : [{ name, before, after }];
});
assert.equal(modified.length, 5, 'Unexpected extra upstream modification');
const provenance = read('rust/vendor/contact-provenance.json');
for (const [name, sha256] of Object.entries(provenance.after)) {
  assert.equal(hash(fs.readFileSync(`${vendor}/${name}`)), sha256, `Contact provenance ${name}`);
  assert.equal(hash(fs.readFileSync(`${original}/${name}`)), provenance.before[name]);
}
const crateSha256 = hash(fs.readFileSync('evidence/rapier-calibration-cache/registry/cache/index.crates.io-1949cf8c6b5b557f/rapier3d-0.35.0.crate'));
assert.equal(crateSha256, provenance.crateSha256);
const oldAudit = read('evidence/rust-rework-20261001/final-audit/verification.json');
assert.deepEqual(modified.sort((a,b)=>a.name.localeCompare(b.name)), oldAudit.vendorModifiedFiles.sort((a,b)=>a.name.localeCompare(b.name)), 'Vendor differs from audited patch');
for (const file of ['coordinate.patch', 'vendor-diff.patch']) {
  fs.copyFileSync(`evidence/rust-rework-20261001/final-audit/${file}`, path.join(out, file));
}
const ancillaryNames = [
  '.github/workflows/rust-rework.yml', 'docs/rust-rework.md', 'docs/rust-rework-performance-v1.json',
  'scripts/capture-rust-rework-baseline.mjs', 'scripts/backport-rust-contact.mjs',
  'scripts/export-rust-canonical.ts', 'scripts/export-rust-scenarios.ts',
  'scripts/run-rust-gates.mjs', 'scripts/run-rust-disturbance-probe.mjs', 'scripts/audit-rust-rework.mjs',
];
const ancillary = ancillaryNames.map(name => {
  const bytes = fs.readFileSync(name), target = path.join(out, 'ancillary-source', name);
  fs.mkdirSync(path.dirname(target), { recursive: true });
  fs.writeFileSync(target, bytes, { flag: 'wx' });
  if (name.endsWith('.mjs')) command(path.basename(name), process.execPath, ['--check', name]);
  return { name, sha256: hash(bytes) };
});
command('diff-check', 'git', ['diff', '--check']);
fs.writeFileSync(path.join(out, 'final-status.txt'), command('status', 'git', ['status', '--short']));
const report = read(`${frozen}/native-1/report.json`);
const verification = { schema: 2, at: new Date().toISOString(), command: [process.execPath, ...process.argv.slice(1)],
  complete: false, releaseAccepted: false, productionSwitched: false,
  baselinePreserved: true, baselineRevision: currentRevision, baselineFileCount: files.length,
  indexAndTrackedDiffsUnchanged: true, foundationFingerprint: execution.sourceFingerprint,
  foundationUnchanged: true, foundationFiles: foundation.length, resultEquality,
  nativeGateStatus: { A: report.gateA, B: report.gateB, C: report.gateC, D: report.gateD, EThroughI: report.gatesEThroughI },
  browserQualified: false, performanceQualified: false, crateSha256, vendorFileCount: vendorFiles.length,
  vendorModifiedFiles: modified, contactProvenanceMatchesCurrent: true, ancillary,
  reviewScope: 'Original tracked/untracked source and index; final frozen foundation and all repeated result bytes; exact original vendor package plus five audited changes; ancillary JavaScript syntax and git whitespace. Physical acceptance is reported separately and fails D. CI and browser paths were not executed.',
};
fs.writeFileSync(path.join(out, 'verification.json'), JSON.stringify(verification, null, 2) + '\n');
fs.writeFileSync(path.join(out, 'baseline-preservation.json'), JSON.stringify(files, null, 2) + '\n');
console.log(JSON.stringify({ out, baselineFilesPreserved: files.length, frozenSourceUnchanged: true, nativeGateStatus: verification.nativeGateStatus, complete: false, productionSwitched: false }));
