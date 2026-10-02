import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { copyFileSync, existsSync, lstatSync, mkdirSync, readFileSync, readdirSync, readlinkSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';

// Deliberately inventories evidence only one level deep. Never follows junctions.
const root = resolve('.');
const out = resolve(process.argv[2] ?? 'evidence/rust-rework-20261001/baseline');
if (!out.startsWith(root + '\\') || existsSync(out)) throw new Error('Require a fresh workspace output directory');
const run = (cmd, args) => {
  const result = spawnSync(cmd, args, { cwd: root, encoding: 'utf8', windowsHide: true });
  if (result.status !== 0) throw new Error(`${cmd}: ${result.stderr}`);
  return result.stdout;
};
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const files = run('git', ['ls-files', '-z', '--cached', '--others', '--exclude-standard']).split('\0').filter(Boolean).sort();
const status = run('git', ['status', '--porcelain=v1', '-uall']);
mkdirSync(join(out, 'source'), { recursive: true });
const entries = [];
for (const name of files) {
  const path = join(root, name);
  if (!existsSync(path)) { entries.push({ name, deleted: true }); continue; }
  // Validate every ancestor so even a tracked path through a junction is not traversed.
  const parts = name.split('/');
  let ancestor = root;
  for (const part of parts) {
    ancestor = join(ancestor, part);
    if (lstatSync(ancestor).isSymbolicLink()) throw new Error(`Refuse linked source: ${name}`);
  }
  const bytes = readFileSync(path);
  const target = join(out, 'source', name);
  mkdirSync(dirname(target), { recursive: true });
  copyFileSync(path, target);
  if (hash(readFileSync(target)) !== hash(bytes)) throw new Error(`Copy mismatch: ${name}`);
  entries.push({ name, bytes: bytes.length, sha256: hash(bytes) });
}
const evidence = readdirSync(join(root, 'evidence'), { withFileTypes: true }).filter(e => e.name !== 'rust-rework-20261001').map(e => {
  const path = join(root, 'evidence', e.name), stat = lstatSync(path);
  return { name: e.name, kind: stat.isSymbolicLink() ? 'link' : e.isDirectory() ? 'directory' : 'file',
    target: stat.isSymbolicLink() ? readlinkSync(path) : null, bytes: stat.isFile() ? stat.size : null };
});
const references = [];
for (const { name } of entries.filter(e => e.name.startsWith('docs/') && e.name.endsWith('.md'))) {
  const content = readFileSync(join(root, name), 'utf8');
  for (const ref of new Set(content.match(/evidence\/[\w./+-]+/g) ?? [])) references.push({ document: name, reference: ref, available: existsSync(join(root, ref)) });
}
writeFileSync(join(out, 'tracked.patch'), run('git', ['diff', '--binary', 'HEAD']), { flag: 'wx' });
writeFileSync(join(out, 'index.patch'), run('git', ['diff', '--cached', '--binary']), { flag: 'wx' });
writeFileSync(join(out, 'status.txt'), status, { flag: 'wx' });
const manifest = { schema: 1, capturedAt: new Date().toISOString(), root, revision: run('git', ['rev-parse', 'HEAD']).trim(),
  node: process.version, rustc: run('rustc', ['--version','--verbose']), cargo: run('cargo', ['--version']),
  toolchains: run('rustup', ['show']), status, files: entries, evidence, references,
  sourceFingerprint: hash(Buffer.from(JSON.stringify(entries))),
  dependencies: JSON.parse(readFileSync('package-lock.json', 'utf8')).packages,
  scope: 'Working-tree source and configuration, including dirty tracked and non-ignored untracked files. Evidence links inventoried, not followed. Caches and ignored external bulk evidence not copied.' };
writeFileSync(join(out, 'manifest.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ out, files: entries.length, sourceFingerprint: manifest.sourceFingerprint, linkedEvidence: evidence.filter(e => e.kind === 'link'), references: references.length }));
