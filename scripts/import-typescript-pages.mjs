// GitHub's Pages artifact contains artifact.tar. Reject links and unsafe member
// names before extracting the existing site; no build tools are involved.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync } from 'node:fs';
import { resolve } from 'node:path';
import { workspaceOutput } from './rust-browser-tools.mjs';
import { treeHashes } from './rust-release-tools.mjs';
import { validateRollbackAssets } from './typescript-rollback-tools.mjs';

const [archiveArg, outputArg] = process.argv.slice(2);
assert.ok(archiveArg && outputArg, 'Supply existing Pages artifact.tar and fresh export directory');
const archive = resolve(archiveArg), output = workspaceOutput(outputArg);
assert.ok(!existsSync(output), 'Refuse to overwrite an imported Pages export');
function tar(args) {
  const result = spawnSync('tar', args, {encoding: 'utf8', windowsHide: true, maxBuffer: 16 * 1024 * 1024});
  assert.equal(result.status, 0, result.error?.message ?? result.stderr);
  return result.stdout;
}
const names = tar(['-tf', archive]).trim().split(/\r?\n/);
const details = tar(['-tvf', archive]).trim().split(/\r?\n/);
assert.ok(names.length > 0 && names.length === details.length, 'Incomplete archive inventory');
assert.ok(details.every(line => line[0] === '-' || line[0] === 'd'), 'Refuse linked or special archive members');
const seen = new Set();
for (const raw of names) {
  const name = raw.replace(/^\.\//, '').replace(/\/$/, '');
  if (name === '.' || !name) continue;
  assert.ok(!name.startsWith('/') && !name.includes('\\') && !name.includes(':')
    && !name.includes('\0') && name.split('/').every(part => part && part !== '..' && part !== '.'),
    `Unsafe Pages archive member: ${raw}`);
  const key = name.toLowerCase();
  assert.ok(!seen.has(key), `Duplicate Pages archive member: ${raw}`); seen.add(key);
}
mkdirSync(output, {recursive: true});
tar(['--no-same-owner', '--no-same-permissions', '-xf', archive, '-C', output]);
const assets = treeHashes(output); validateRollbackAssets(assets);
console.log(JSON.stringify({directory: output, assets: Object.keys(assets).length}));
