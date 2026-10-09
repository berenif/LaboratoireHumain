import { createHash } from 'node:crypto';
import { existsSync, lstatSync, mkdirSync, readFileSync, readdirSync, renameSync, statSync, utimesSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';

// Cargo can reuse another checkpoint after a timestamp-preserving restore.
// Keep content history outside the source tree and refresh changed/restored
// local input timestamps before invoking the compiler.
export function prepareRustBuild(root = process.cwd()) {
  root = resolve(root);
  const source = join(root, 'rust');
  const target = join(source, 'target');
  if (lstatSync(source).isSymbolicLink() || (existsSync(target) && lstatSync(target).isSymbolicLink())) {
    throw new Error('Refuse linked Rust source/build directory');
  }
  const ledgerPath = join(target, 'source-freshness.json');
  const inputs = {};
  function visit(directory) {
    for (const entry of readdirSync(directory, { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
      if (['target', 'dist', '.git', 'checkpoints'].includes(entry.name)) continue;
      const file = join(directory, entry.name);
      if (entry.isSymbolicLink()) throw new Error(`Refuse linked Rust build input: ${file}`);
      if (entry.isDirectory()) visit(file);
      else if (entry.isFile()) {
        const name = relative(root, file).replaceAll('\\', '/');
        inputs[name] = { sha256: createHash('sha256').update(readFileSync(file)).digest('hex'), mtimeMs: statSync(file).mtimeMs };
      }
    }
  }
  visit(source);
  let previous;
  if (existsSync(ledgerPath)) {
    try {
      previous = JSON.parse(readFileSync(ledgerPath, 'utf8'));
      if (previous.schema !== 1 || !previous.inputs || Object.values(previous.inputs).some(input =>
        !/^[a-f0-9]{64}$/.test(input.sha256) || !Number.isFinite(input.mtimeMs))) previous = undefined;
    } catch { previous = undefined; }
  }
  const removed = previous ? Object.keys(previous.inputs).filter(name => !(name in inputs)) : [];
  const refreshedInputs = Object.keys(inputs).filter(name => !previous || removed.length ||
    previous.inputs[name]?.sha256 !== inputs[name].sha256 || previous.inputs[name].mtimeMs > inputs[name].mtimeMs);
  const refreshTime = Math.max(Date.now(), ...refreshedInputs.map(name => inputs[name].mtimeMs + 1),
    ...refreshedInputs.map(name => (previous?.inputs[name]?.mtimeMs ?? 0) + 1));
  for (const name of refreshedInputs) {
    const file = join(root, name);
    utimesSync(file, statSync(file).atime, new Date(refreshTime));
    inputs[name].mtimeMs = statSync(file).mtimeMs;
  }
  const hashes = Object.fromEntries(Object.entries(inputs).map(([name, input]) => [name, input.sha256]));
  const inputFingerprint = createHash('sha256').update(JSON.stringify(hashes)).digest('hex');
  mkdirSync(target, { recursive: true });
  const ledger = { schema: 1, inputFingerprint, inputs };
  if (refreshedInputs.length || !previous) {
    const temporary = ledgerPath + '.tmp';
    writeFileSync(temporary, JSON.stringify(ledger, null, 2) + '\n');
    renameSync(temporary, ledgerPath);
  }
  return { inputFingerprint, refreshedInputs, removedInputs: removed };
}
