import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createWriteStream, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { workspaceOutput } from './rust-browser-tools.mjs';

assert.ok(process.argv.length <= 3, 'Usage: node scripts/verify-rust-app.mjs [fresh-output-directory]');
const output = workspaceOutput(process.argv[2] ?? `evidence/rust-browser-${new Date().toISOString().replaceAll(/[:.]/g, '-')}`);
assert.ok(!existsSync(output), 'Use a fresh evidence directory');
mkdirSync(output, { recursive: true });
const report = { started: new Date().toISOString(), scope: 'Built root/subpath UI, graphics faults, worker lifecycle, six flat-floor standing trials, terrain contacts and physical striker integration',
  checks: [], passed: false, releaseAccepted: false };
const save = () => writeFileSync(join(output, 'execution.json'), JSON.stringify(report, null, 2) + '\n');
async function run(name, script, args) {
  const started = performance.now();
  const log = createWriteStream(join(output, name + '.log'));
  const result = await new Promise(resolve => {
    const child = spawn(process.execPath, [script, ...args], { windowsHide: true });
    child.stdout.pipe(log, { end: false }); child.stderr.pipe(log, { end: false });
    const timeout = setTimeout(() => child.kill(), 10 * 60 * 1000);
    let error;
    child.on('error', value => { error = value.message; });
    child.on('close', (exit, signal) => { clearTimeout(timeout); log.end(); resolve({ exit, signal, error }); });
  });
  report.checks.push({ name, command: [process.execPath, script, ...args], ...result, wallMs: performance.now() - started });
  save(); console.log(JSON.stringify(report.checks.at(-1)));
  return result.exit === 0 && !result.error && !result.signal;
}
async function serve(artifact, mount) {
  const child = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', artifact, '0', mount], { windowsHide: true });
  const origin = await new Promise((resolve, reject) => {
    let stdout = '', stderr = '';
    const timeout = setTimeout(() => { child.kill(); reject(new Error('Preview startup timed out')); }, 15000);
    child.stdout.on('data', chunk => {
      stdout += chunk;
      const match = stdout.match(/Rust preview: (http:\/\/127\.0\.0\.1:\d+\/[^\s]*)/);
      if (match) { clearTimeout(timeout); resolve(match[1]); }
    });
    child.stderr.on('data', chunk => { stderr += chunk; });
    child.on('error', error => { clearTimeout(timeout); reject(error); });
    child.on('exit', code => { clearTimeout(timeout); reject(new Error(`Preview exited ${code}: ${stderr}`)); });
  });
  return { child, origin };
}
try {
  for (const [name, mount] of [['root', '/'], ['subpath', '/LaboratoireHumain/']]) {
    const artifact = join(output, name + '-app');
    if (!await run(name + '-build', 'scripts/build-rust-browser.mjs', [artifact, mount])) continue;
    const server = await serve(artifact, mount);
    try {
      const checks = [
        [name + '-ui', 'scripts/verify-rust-browser.mjs', [artifact]],
        [name + '-faults', 'scripts/verify-rust-browser-faults.mjs', []],
        ...(name === 'root' ? [
          ['worker', 'scripts/verify-rust-worker.mjs', [artifact]],
          ['quiet', 'scripts/verify-rust-worker-quiet.mjs', [artifact]],
          ['terrain', 'scripts/verify-rust-terrain.mjs', [artifact]],
          ['striker', 'scripts/verify-rust-striker.mjs', [artifact]],
        ] : []),
      ];
      for (const [check, script, extra] of checks) {
        await run(check, script, [join(output, check), server.origin, ...extra]);
      }
    } finally {
      server.child.kill();
      await new Promise(resolve => server.child.exitCode !== null ? resolve() : server.child.once('exit', resolve));
    }
  }
  report.passed = report.checks.length === 10 && report.checks.every(check => check.exit === 0 && !check.error && !check.signal);
} catch (error) { report.error = String(error); }
finally { report.finished = new Date().toISOString(); save(); }
console.log(JSON.stringify({ output: resolve(output), passed: report.passed, releaseAccepted: false }));
process.exitCode = report.passed ? 0 : 1;
