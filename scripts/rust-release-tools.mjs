import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, lstatSync, readFileSync, readdirSync, renameSync, writeFileSync } from 'node:fs';
import { join, relative } from 'node:path';

export const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
// A failed write must leave the last valid, unaccepted checkpoint intact.
export function persistReceipt(path, receipt, io = {writeFileSync,renameSync}) {
  const temporary = path + '.tmp';
  io.writeFileSync(temporary, JSON.stringify(receipt,null,2) + '\n');
  io.renameSync(temporary,path);
}
export function treeHashes(directory, source = false) {
  const result = {};
  function visit(path) {
    for (const entry of readdirSync(path, { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
      if (source && ['target', 'dist', '.git', 'checkpoints'].includes(entry.name)) continue;
      const file = join(path, entry.name);
      assert.ok(!lstatSync(file).isSymbolicLink(), `Refuse linked input ${file}`);
      if (entry.isDirectory()) visit(file);
      else result[relative(directory, file).replaceAll('\\', '/')] = sha256(readFileSync(file));
    }
  }
  visit(directory);
  return result;
}
export function sourceHashes() {
  const result = {};
  for (const directory of ['rust', 'scripts', 'tests', 'docs', '.github']) {
    if (existsSync(directory)) for (const [name, hash] of Object.entries(treeHashes(directory, true))) result[`${directory}/${name}`] = hash;
  }
  for (const file of ['package.json', 'package-lock.json']) {
    if (existsSync(file)) result[file] = sha256(readFileSync(file));
  }
  return Object.fromEntries(Object.entries(result).sort(([a], [b]) => a.localeCompare(b)));
}
function canonical(value) {
  if (Array.isArray(value)) return value.map(canonical);
  if (value !== null && typeof value === 'object') {
    return Object.fromEntries(Object.entries(value).sort(([a], [b]) => a < b ? -1 : a > b ? 1 : 0)
      .map(([name, item]) => [name, canonical(item)]));
  }
  return value;
}
export const fingerprint = entries => sha256(Buffer.from(JSON.stringify(canonical(entries))));
export function nativeTestCount(output) {
  return [...output.matchAll(/test result: ok\. (\d+) passed; 0 failed;/g)]
    .reduce((total, match) => total + Number(match[1]), 0);
}
export const requiredReleaseStages = ['format', 'native-clippy', 'wasm-clippy', 'tooling', 'native-tests',
  'scalar-cached-equivalence', 'native-repeat', 'contact-stress', 'force-controls', 'anatomy-structure-collision-ownership',
  'actuators', 'disturbances', 'strict-steps', 'legacy-four', 'recovery', 'cycles-five', 'terrain-21',
  'production-profile', 'root-browser', 'subpath-browser', 'wasm-physics-equivalence', 'WebGPU-performance',
  'WebGL2-performance', 'soak-120', 'physical-touchscreen', 'application-build-and-tests', 'source-unchanged', 'asset-unchanged'];
export function releaseDecision(stages) {
  const missing = requiredReleaseStages.filter(name => !stages.some(stage => stage.name === name));
  const failed = stages.filter(stage => stage.passed !== true).map(stage => stage.name);
  const duplicate = stages.map(stage => stage.name).filter((name, index, names) => names.indexOf(name) !== index);
  return { complete: missing.length === 0 && duplicate.length === 0,
    releaseAccepted: missing.length === 0 && failed.length === 0 && duplicate.length === 0,
    missing, failed, duplicate };
}
export function validateCandidate(candidate, receipt, assets) {
  assert.equal(candidate.schema, 1);
  assert.equal(candidate.candidateId, fingerprint({source:candidate.sourceFingerprint,
    assets:candidate.assetsFingerprint, mount:candidate.mount}), 'Candidate identity mismatch');
  assert.equal(receipt.sourceFingerprint, candidate.sourceFingerprint, 'Receipt source mismatch');
  assert.equal(receipt.candidateId, candidate.candidateId, 'Receipt candidate mismatch');
  assert.equal(receipt.assetsFingerprint, candidate.assetsFingerprint, 'Receipt asset mismatch');
  assert.deepEqual(assets, candidate.assets, 'Candidate assets changed');
  assert.equal(fingerprint(assets), candidate.assetsFingerprint, 'Asset manifest mismatch');
  const decision = releaseDecision(receipt.stages);
  assert.equal(decision.releaseAccepted, true, `Unaccepted candidate: ${JSON.stringify(decision)}`);
  assert.equal(receipt.releaseAccepted, true);
  assert.equal(receipt.sourceUnchanged, true);
  assert.equal(receipt.assetsUnchanged, true);
  return decision;
}
