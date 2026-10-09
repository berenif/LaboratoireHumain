// Import and qualify an existing Pages export. This command never rebuilds it.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { cpSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fingerprint, persistReceipt, sha256, treeHashes } from './rust-release-tools.mjs';
import { workspaceOutput } from './rust-browser-tools.mjs';
import { rollbackIdentity, validateRollbackAssets, validateTypescriptRollback } from './typescript-rollback-tools.mjs';
import { checkTypescriptBrowser } from './verify-typescript-browser.mjs';

const [inputArg, urlArg, outputArg] = process.argv.slice(2);
assert.ok(inputArg && urlArg && outputArg, 'Supply existing export directory, deployed URL, and fresh bundle directory');
const input = resolve(inputArg), output = workspaceOutput(outputArg), base = new URL(urlArg);
assert.equal(base.pathname, '/LaboratoireHumain/');
assert.equal(base.search + base.hash, '');
assert.equal(base.origin, 'https://berenif.github.io', 'Qualify the deployed repository site');
assert.ok(!base.username && !base.password);
assert.ok(!existsSync(output), 'Refuse to overwrite a rollback bundle');
const assets = treeHashes(input); validateRollbackAssets(assets);
mkdirSync(output, {recursive: true});
cpSync(input, join(output, 'site'), {recursive: true, errorOnExist: true});
const candidate = {schema: 1, runtime: 'typescript', mount: base.pathname, publishedUrl: base.href,
  assets, assetsFingerprint: fingerprint(assets)};
candidate.rollbackId = rollbackIdentity(candidate);
persistReceipt(join(output, 'candidate.json'), candidate);
const receipt = {schema: 1, runtime: 'typescript', rollbackId: candidate.rollbackId,
  assetsFingerprint: candidate.assetsFingerprint, importedFrom: input, started: new Date().toISOString(),
  stages: [], publishedAssets: {}, qualified: false};
const save = () => persistReceipt(join(output, 'receipt.json'), receipt);
let server;
try {
  save();
  for (const [name, digest] of Object.entries(assets)) {
    // The empty deployment marker is deliberately hidden by GitHub Pages.
    if (name === '.nojekyll') {
      assert.equal(readFileSync(join(input, name)).length, 0);
      receipt.publishedAssets[name] = sha256(Buffer.alloc(0)); continue;
    }
    const location = new URL(name.split('/').map(encodeURIComponent).join('/'), base);
    location.searchParams.set('rollback', candidate.rollbackId);
    const response = await fetch(location, {cache: 'no-store', signal: AbortSignal.timeout(30000)});
    assert.ok(response.ok, `Deployed rollback asset missing: ${name}`);
    receipt.publishedAssets[name] = sha256(Buffer.from(await response.arrayBuffer()));
    assert.equal(receipt.publishedAssets[name], digest, `Deployed rollback asset mismatch: ${name}`);
    save();
  }
  receipt.stages.push({name: 'published-assets', passed: true}); save();
  server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', join(output, 'site'), '0', base.pathname], {windowsHide: true});
  const local = await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('Rollback preview startup timeout')), 15000); let text = '';
    server.stdout.on('data', chunk => {text += chunk; const match = text.match(/Rust preview: (http:\/\/127\.0\.0\.1:\d+\/[^\s]*)/);
      if (match) {clearTimeout(timer); resolve(match[1]);}});
    server.on('error', error => {clearTimeout(timer); reject(error);});
    server.on('exit', code => {clearTimeout(timer); reject(new Error(`Rollback preview exited ${code}`));});
  });
  mkdirSync(join(output, 'local-browser')); mkdirSync(join(output, 'published-browser'));
  receipt.localBrowser = await checkTypescriptBrowser(local, join(output, 'local-browser'));
  receipt.stages.push({name: 'local-browser', passed: receipt.localBrowser.passed}); save();
  receipt.publishedBrowser = await checkTypescriptBrowser(base.href, join(output, 'published-browser'));
  receipt.stages.push({name: 'published-browser', passed: receipt.publishedBrowser.passed});
  receipt.artifactUnchanged = fingerprint(treeHashes(input)) === candidate.assetsFingerprint
    && fingerprint(treeHashes(join(output, 'site'))) === candidate.assetsFingerprint;
  receipt.stages.push({name: 'artifact-unchanged', passed: receipt.artifactUnchanged});
  receipt.qualified = receipt.stages.every(stage => stage.passed === true);
  validateTypescriptRollback(candidate, receipt, treeHashes(join(output, 'site')));
} catch (error) {receipt.qualified = false; receipt.error = String(error);}
finally {server?.kill(); receipt.finished = new Date().toISOString(); save();}
console.log(JSON.stringify({rollbackId: candidate.rollbackId, qualified: receipt.qualified, error: receipt.error}));
process.exitCode = receipt.qualified ? 0 : 1;
