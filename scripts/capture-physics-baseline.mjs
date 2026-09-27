import { createHash } from 'node:crypto';
import { existsSync, lstatSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { dirname, isAbsolute, join, relative, resolve, sep } from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

export const sha256 = value => createHash('sha256').update(value).digest('hex');
const sourceDirectories = ['src', 'scripts', 'tests', 'app', 'components', '.github', 'hooks', 'lib',
  'worker', 'db', 'drizzle', 'public', 'vendor', 'examples'];
const configurationFiles = ['package.json', 'package-lock.json', 'tsconfig.json', 'next.config.ts', 'vite.config.ts',
  'eslint.config.mjs', 'postcss.config.mjs', 'drizzle.config.ts', 'components.json', '.npmrc', '.gitignore'];
const completionDirectory = 'evidence/physics-repair-20260924-completion';
const focusedPath = `${completionDirectory}/focused-restored-retained-source.json`;
const historicalPaths = {
  [focusedPath]: 'd11d97bb6daa5cc1deca5624ee9a3495edfa586fb1ffee69422887e023546c88',
  [`${completionDirectory}/focused-restored-retained-source.log`]: '4ad83cdfd28f059d04fe39b8162661d79dfae78771e66f8858388c37b6e586fe',
  'evidence/physics-standing-20260926/index.json': null,
  'evidence/physics-standing-20260926/production-ticks-175-176.jsonl.gz': 'f4ae9784f98fc88bcf72cda036944af220704683de88d878c16890bca8b8daa8',
};

export function fingerprints(root = process.cwd()) {
  const paths = [];
  function visit(directory) {
    for (const entry of readdirSync(join(root, directory), { withFileTypes: true })) {
      const path = `${directory}/${entry.name}`;
      if (entry.isDirectory()) visit(path);
      else if (entry.isFile()) paths.push(path);
    }
  }
  for (const directory of sourceDirectories) if (existsSync(join(root, directory))) visit(directory);
  paths.push(...configurationFiles.filter(path => existsSync(join(root, path))));
  return Object.fromEntries(paths.sort().map(path => [path, sha256(readFileSync(join(root, path)))]));
}

function fileAvailability(path, expectedSha256 = null) {
  if (!existsSync(path)) return { availability: 'missing', expectedSha256 };
  if (!lstatSync(path).isFile()) return { availability: 'not-a-file', expectedSha256 };
  const contents = readFileSync(path), digest = sha256(contents);
  return { availability: contents.length ? 'available' : 'empty', bytes: contents.length, sha256: digest,
    expectedSha256, matchesExpected: expectedSha256 ? digest === expectedSha256 : null };
}

function evidenceInventory() {
  const files = {};
  function visit(directory) {
    for (const entry of readdirSync(directory, { withFileTypes: true })) {
      const path = `${directory}/${entry.name}`;
      if (entry.isDirectory()) visit(path);
      else files[path] = fileAvailability(path, historicalPaths[path]);
    }
  }
  const directoryExists = existsSync(completionDirectory) && lstatSync(completionDirectory).isDirectory();
  if (directoryExists) visit(completionDirectory);
  for (const [path, digest] of Object.entries(historicalPaths)) files[path] = fileAvailability(path, digest);
  return { completionDirectory: { path: completionDirectory, availability: directoryExists ? 'available' : 'missing' }, files };
}

function capture(output, fresh) {
  if (existsSync(output)) throw new Error('Specify a new evidence directory. Existing runs, including partial runs, are immutable.');
  const outputPath = resolve(output);
  for (const path of [...sourceDirectories, ...configurationFiles, 'node_modules', '.git']) {
    const offset = relative(resolve(path), outputPath);
    if (!offset || (!isAbsolute(offset) && offset !== '..' && !offset.startsWith(`..${sep}`))) {
      throw new Error('Store evidence outside source, configuration, dependencies, and Git directories.');
    }
  }
  const startedAt = new Date().toISOString();
  const source = fingerprints();
  const evidence = evidenceInventory();
  let savedFocused = null;
  if (!fresh) {
    if (evidence.files[focusedPath].availability !== 'available') {
      throw new Error('Historical focused report unavailable. Use --fresh to capture a new baseline without inherited results.');
    }
    if (!evidence.files[focusedPath].matchesExpected) {
      throw new Error('Historical focused report does not match its recorded digest. Use --fresh for a new baseline.');
    }
    const saved = JSON.parse(readFileSync(focusedPath, 'utf8').replace(/^\uFEFF/, ''));
    const matches = Object.entries(saved.sourceAfter).map(([path, hash]) => ({ path, hash,
      matches: source[path] === hash && saved.sourceBefore[path] === hash }));
    if (!matches.length || matches.some(item => !item.matches)) {
      throw new Error('Saved focused evidence no longer matches. Use --fresh for a new baseline: ' + JSON.stringify(matches.filter(item => !item.matches)));
    }
    savedFocused = { path: focusedPath, matches, summary: saved.summary, exitCode: saved.exitCode,
      scope: 'Historical result on its recorded source subset; no tests executed by this capture.' };
  }
  const git = args => execFileSync('git', args, { encoding: 'utf8', windowsHide: true, maxBuffer: 64 * 1024 * 1024 });
  const status = git(['status', '--porcelain=v1']);
  const modified = git(['diff', 'HEAD', '--name-only', '--no-renames', '-z']).split('\0').filter(Boolean);
  const untracked = git(['ls-files', '--others', '--exclude-standard', '-z']).split('\0').filter(Boolean);
  const patches = { 'worktree.patch': git(['diff', '--binary', '--no-ext-diff', '--no-textconv']),
    'index.patch': git(['diff', '--cached', '--binary', '--no-ext-diff', '--no-textconv']) };
  const pkg = JSON.parse(readFileSync('package.json', 'utf8'));
  const dependencies = Object.fromEntries([...new Set([...Object.keys(pkg.dependencies ?? {}), ...Object.keys(pkg.devDependencies ?? {})])].sort().map(name => {
    const path = `node_modules/${name}/package.json`;
    if (!existsSync(path)) throw new Error(`Dependency ${name} is unavailable. Install the locked dependencies before capturing a baseline.`);
    const contents = readFileSync(path);
    return [name, { version: JSON.parse(contents).version, packageHash: sha256(contents) }];
  }));
  const node = { executable: process.execPath, version: process.version, versions: process.versions,
    executableHash: sha256(readFileSync(process.execPath)), platform: process.platform, arch: process.arch };

  // Reserve the run only after preflight. Never overwrite even an incomplete capture.
  mkdirSync(dirname(outputPath), { recursive: true });
  mkdirSync(outputPath);
  const preservedWorktree = {};
  for (const path of [...new Set([...modified, ...untracked])].sort()) {
    if (!existsSync(path)) { preservedWorktree[path] = { state: 'deleted' }; continue; }
    if (!lstatSync(path).isFile()) throw new Error(`Cannot preserve non-regular worktree file: ${path}`);
    const contents = readFileSync(path);
    const destination = join(output, 'preserved-worktree', path);
    mkdirSync(dirname(destination), { recursive: true });
    writeFileSync(destination, contents, { flag: 'wx' });
    preservedWorktree[path] = { state: untracked.includes(path) ? 'untracked' : 'modified',
      bytes: contents.length, sha256: sha256(contents) };
  }
  for (const [path, contents] of Object.entries(patches)) writeFileSync(join(output, path), contents, { flag: 'wx' });
  const sourceAfter = fingerprints();
  if (JSON.stringify(source) !== JSON.stringify(sourceAfter)) {
    throw new Error('Source changed during capture; partial evidence retained. Retry in a new directory.');
  }
  const record = { schemaVersion: 2, startedAt, createdAt: new Date().toISOString(), mode: fresh ? 'fresh' : 'historical',
    command: [process.execPath, ...process.argv.slice(1)], cwd: process.cwd(), captureExitCode: 0,
    node, dependencies, head: git(['rev-parse', 'HEAD']).trim(), status, preservedWorktree,
    patches: Object.fromEntries(Object.entries(patches).map(([path, contents]) => [path, sha256(contents)])),
    source, sourceFingerprint: sha256(JSON.stringify(source)), sourceAfter, sourceUnchanged: true,
    savedFocused, evidence, physicsSettings: readFileSync('src/character/physics-settings.ts', 'utf8'),
    priorEvidenceHashes: Object.fromEntries(Object.entries(evidence.files).filter(([, entry]) => entry.sha256).map(([path, entry]) => [path, entry.sha256])),
    scenarios: { execution: 'not-run', renderer: 'canvas2d', dt: 1 / 60, headings: [0, Math.PI / 3, -Math.PI / 4], flatGround: true,
      idleSettlingS: 2, idleObservationS: 30, purpose: 'Frozen planned standing inputs; capture runs no simulation or acceptance tests.' } };
  writeFileSync(join(output, 'baseline.json'), JSON.stringify(record, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify({ output, mode: record.mode, sourceFiles: Object.keys(source).length,
    sourceFingerprint: record.sourceFingerprint, historyAvailable: Object.values(evidence.files).filter(entry => entry.availability === 'available').length,
    historyMissing: Object.values(evidence.files).filter(entry => entry.availability === 'missing').length,
    node: node.version, testsExecuted: 0 }));
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2), fresh = args.includes('--fresh');
  const paths = args.filter(arg => !arg.startsWith('--'));
  if (paths.length !== 1 || args.some(arg => arg.startsWith('--') && arg !== '--fresh') || args.length !== paths.length + Number(fresh)) {
    throw new Error('Usage: node scripts/capture-physics-baseline.mjs [--fresh] new-evidence-directory');
  }
  capture(paths[0], fresh);
}
