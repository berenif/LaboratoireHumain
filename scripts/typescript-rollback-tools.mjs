import assert from 'node:assert/strict';
import { fingerprint } from './rust-release-tools.mjs';

export const rollbackStages = ['published-assets', 'local-browser', 'published-browser', 'artifact-unchanged'];
export const rollbackCases = ['protocol:webgl', 'protocol:canvas2d', 'playground:webgl', 'playground:canvas2d'];

export function validateRollbackAssets(assets) {
  assert.ok(assets && typeof assets === 'object' && !Array.isArray(assets));
  assert.ok(assets['index.html'] && assets['404.html'], 'Rollback requires both Pages entrypoints');
  assert.ok(Object.keys(assets).some(name => name.startsWith('_next/') && name.endsWith('.js')),
    'Rollback must contain the existing TypeScript export');
  for (const [name, digest] of Object.entries(assets)) {
    assert.ok(!name.startsWith('/') && !name.includes('\\') && !name.includes(':')
      && name.split('/').every(part => part && part !== '.' && part !== '..'), 'Invalid rollback asset path');
    assert.match(digest, /^[a-f0-9]{64}$/, 'Invalid rollback asset digest');
  }
}

export function validateRollbackBrowser(report) {
  assert.equal(report?.passed, true, 'Rollback browser qualification failed');
  assert.deepEqual(report.cases.map(row => `${row.mode}:${row.renderer}`).sort(), [...rollbackCases].sort(),
    'Incomplete or duplicate rollback browser coverage');
  for (const row of report.cases) {
    assert.equal(row.passed, true);
    assert.equal(row.finite, true);
    assert.ok(row.finalFixedSteps > row.firstFixedSteps, 'Rollback simulation did not advance');
    assert.deepEqual(row.errors, []);
  }
}

export function rollbackIdentity(candidate) {
  return fingerprint({runtime: 'typescript', assets: candidate.assetsFingerprint, mount: candidate.mount,
    publishedUrl: candidate.publishedUrl});
}

export function validateTypescriptRollback(candidate, receipt, assets) {
  assert.equal(candidate.schema, 1);
  assert.equal(candidate.runtime, 'typescript');
  assert.equal(candidate.mount, '/LaboratoireHumain/');
  const url = new URL(candidate.publishedUrl);
  assert.equal(url.origin, 'https://berenif.github.io', 'Rollback must match the deployed repository site');
  assert.equal(url.pathname, candidate.mount);
  assert.equal(url.search + url.hash, '', 'Rollback URL must identify the mounted site');
  validateRollbackAssets(assets);
  assert.deepEqual(assets, candidate.assets, 'Rollback artifact changed');
  assert.equal(fingerprint(assets), candidate.assetsFingerprint, 'Rollback asset identity mismatch');
  assert.equal(rollbackIdentity(candidate), candidate.rollbackId, 'Rollback identity mismatch');
  assert.equal(receipt.rollbackId, candidate.rollbackId, 'Rollback receipt mismatch');
  assert.equal(receipt.assetsFingerprint, candidate.assetsFingerprint);
  assert.deepEqual(receipt.stages.map(stage => stage.name).sort(), [...rollbackStages].sort(),
    'Missing or duplicate rollback qualification stages');
  assert.ok(receipt.stages.every(stage => stage.passed === true), 'Unqualified TypeScript rollback');
  assert.equal(receipt.qualified, true);
  assert.equal(receipt.artifactUnchanged, true);
  assert.deepEqual(receipt.publishedAssets, assets, 'Rollback bytes do not match the deployed artifact');
  validateRollbackBrowser(receipt.localBrowser);
  validateRollbackBrowser(receipt.publishedBrowser);
  return true;
}
