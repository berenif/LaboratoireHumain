// Validate existing rollback bytes before any Pages deployment.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { cpSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { workspaceOutput } from './rust-browser-tools.mjs';
import { fingerprint, treeHashes } from './rust-release-tools.mjs';
import { validateTypescriptRollback } from './typescript-rollback-tools.mjs';

const [bundleArg, destinationArg, runtime = 'rust'] = process.argv.slice(2);
assert.ok(bundleArg && destinationArg && ['rust', 'typescript'].includes(runtime));
if (runtime === 'rust') {
  const result = spawnSync(process.execPath, ['scripts/promote-rust-pages.mjs', bundleArg, destinationArg],
    {stdio: 'inherit', windowsHide: true});
  assert.equal(result.status, 0, 'Rust rollback validation failed');
} else {
  const bundle = resolve(bundleArg), destination = workspaceOutput(destinationArg);
  const candidate = JSON.parse(readFileSync(join(bundle, 'candidate.json')));
  const receipt = JSON.parse(readFileSync(join(bundle, 'receipt.json')));
  validateTypescriptRollback(candidate, receipt, treeHashes(join(bundle, 'site')));
  assert.ok(!existsSync(destination), 'Refuse to overwrite a rollback destination');
  mkdirSync(destination, {recursive: true});
  cpSync(join(bundle, 'site'), destination, {recursive: true, errorOnExist: true});
  assert.deepEqual(treeHashes(destination), candidate.assets, 'Restoration changed the rollback artifact');
  console.log(JSON.stringify({rollbackId: candidate.rollbackId, receiptId: fingerprint(receipt), destination}));
}
