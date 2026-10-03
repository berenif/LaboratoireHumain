// A partial physics diagnostic, never a promotion or release gate.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawnSync } from 'node:child_process';

const root = process.cwd();
const output = path.resolve(process.argv[2] ?? '');
if (!process.argv[2] || !output.startsWith(`${root}${path.sep}`) || fs.existsSync(output)) {
  throw new Error('Provide a fresh workspace output directory');
}
const sha = data => crypto.createHash('sha256').update(data).digest('hex');
const sources = {};
function capture(directory) {
  for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
    if (['target', 'dist', '.git'].includes(entry.name)) continue;
    const file = path.join(directory, entry.name);
    if (entry.isSymbolicLink()) throw new Error(`Refuse source link: ${file}`);
    if (entry.isDirectory()) capture(file);
    else if (entry.isFile()) {
      const name = path.relative(root, file).replaceAll('\\', '/');
      const bytes = fs.readFileSync(file);
      sources[name] = sha(bytes);
      const archived = path.join(output, 'source', name);
      fs.mkdirSync(path.dirname(archived), { recursive: true });
      fs.writeFileSync(archived, bytes, { flag: 'wx' });
    }
  }
}
fs.mkdirSync(output, { recursive: false });
capture(path.join(root, 'rust'));
const probeFile = 'scripts/run-rust-disturbance-probe.mjs';
const probeBytes = fs.readFileSync(probeFile);
sources[probeFile] = sha(probeBytes);
fs.mkdirSync(path.join(output, 'source', 'scripts'), { recursive: true });
fs.writeFileSync(path.join(output, 'source', probeFile), probeBytes, { flag: 'wx' });
const env = { ...process.env, CARGO_TARGET_DIR: path.resolve('rust/target') };
if (fs.existsSync('evidence/rapier-calibration-cache')) env.CARGO_HOME = path.resolve('evidence/rapier-calibration-cache');
const toolchain = process.platform === 'win32' ? '+1.89.0-x86_64-pc-windows-gnu' : '+1.89.0';
const receipt = {
  schema: 1, partial: true, releaseAccepted: false, command: [process.execPath, ...process.argv.slice(1)],
  started: new Date().toISOString(), sourceBefore: sources,
  sourceFingerprint: sha(JSON.stringify(sources)), runs: [],
};
function run(name, executable, args) {
  const started = performance.now();
  const result = spawnSync(executable, args, { env, encoding: 'utf8', windowsHide: true, timeout: 300000, maxBuffer: 16 * 1024 * 1024 });
  fs.writeFileSync(path.join(output, `${name}.log`), (result.stdout ?? '') + (result.stderr ?? ''));
  const record = { name, command: [executable, ...args], exit: result.status, signal: result.signal, error: result.error?.message ?? null, wallMs: performance.now() - started };
  receipt.runs.push(record);
  console.log(JSON.stringify(record));
  return result.status === 0;
}
try {
  if (run('build', 'cargo', [toolchain, 'build', '--offline', '--locked', '--release', '--manifest-path', 'rust/Cargo.toml', '-p', 'lh-acceptance'])) {
    const executable = path.resolve(`rust/target/release/lh-acceptance${process.platform === 'win32' ? '.exe' : ''}`);
    receipt.executableSha256 = sha(fs.readFileSync(executable));
    const options = process.argv.slice(3);
    const profileIndex = options.indexOf('--profile');
    if (profileIndex >= 0) {
      const profile = fs.readFileSync(options[profileIndex + 1]);
      receipt.inputProfileSha256 = sha(profile);
      fs.writeFileSync(path.join(output, 'input-profile.json'), profile, { flag: 'wx' });
    }
    run('disturbances', executable, [path.join(output, 'native'), '--diagnose-disturbances', ...options]);
  }
} finally {
  receipt.finished = new Date().toISOString();
  receipt.sourceUnchanged = Object.entries(sources).every(([name, hash]) => fs.existsSync(name) && sha(fs.readFileSync(name)) === hash);
  receipt.implementedChecksPassed = receipt.runs.length === 2
    && receipt.runs.every(run => run.exit === 0 && !run.error && !run.signal)
    && receipt.sourceUnchanged;
  fs.writeFileSync(path.join(output, 'execution.json'), JSON.stringify(receipt, null, 2) + '\n', { flag: 'wx' });
}
process.exitCode = receipt.implementedChecksPassed ? 0 : 1;
