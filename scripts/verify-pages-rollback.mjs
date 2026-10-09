import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { persistReceipt, sha256, treeHashes } from './rust-release-tools.mjs';
import { validateTypescriptRollback } from './typescript-rollback-tools.mjs';
import { checkTypescriptBrowser } from './verify-typescript-browser.mjs';

const [outputArg, urlArg, bundleArg, runtime = 'rust'] = process.argv.slice(2);
assert.ok(outputArg && urlArg && bundleArg && ['rust', 'typescript'].includes(runtime));
if (runtime === 'rust') {
  const result = spawnSync(process.execPath, ['scripts/verify-rust-published.mjs', outputArg, urlArg, bundleArg],
    {stdio: 'inherit', windowsHide: true});
  assert.equal(result.status, 0, 'Restored Rust publication failed verification');
} else {
  const output = resolve(outputArg), bundle = resolve(bundleArg);
  const candidate = JSON.parse(readFileSync(join(bundle, 'candidate.json')));
  const receipt = JSON.parse(readFileSync(join(bundle, 'receipt.json')));
  validateTypescriptRollback(candidate, receipt, treeHashes(join(bundle, 'site')));
  assert.ok(!existsSync(output)); mkdirSync(output, {recursive: true});
  const report = {rollbackId: candidate.rollbackId, assets: {}, passed: false};
  try {
    const base = new URL(urlArg); assert.equal(base.pathname, candidate.mount);
    for (const [name, digest] of Object.entries(candidate.assets)) {
      if (name === '.nojekyll') continue;
      const location = new URL(name.split('/').map(encodeURIComponent).join('/'), base);
      location.searchParams.set('rollback', candidate.rollbackId);
      const response = await fetch(location, {cache: 'no-store', signal: AbortSignal.timeout(30000)});
      assert.ok(response.ok, `Restored asset missing: ${name}`);
      report.assets[name] = sha256(Buffer.from(await response.arrayBuffer()));
      assert.equal(report.assets[name], digest, `Restored asset mismatch: ${name}`);
    }
    report.browser = await checkTypescriptBrowser(base.href, output);
    report.passed = report.browser.passed;
  } catch (error) {report.error = String(error);}
  persistReceipt(join(output, 'report.json'), report);
  console.log(JSON.stringify({passed: report.passed, rollbackId: report.rollbackId, error: report.error}));
  process.exitCode = report.passed ? 0 : 1;
}
