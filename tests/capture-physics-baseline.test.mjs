import assert from 'node:assert/strict';
import { execFileSync, spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { basename, dirname, join, resolve } from 'node:path';
import { tmpdir } from 'node:os';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { fingerprints, sha256 } from '../scripts/capture-physics-baseline.mjs';

const script = fileURLToPath(new URL('../scripts/capture-physics-baseline.mjs', import.meta.url));
const historicalReport = 'evidence/physics-repair-20260924-completion/focused-restored-retained-source.json';
const trace = 'evidence/physics-standing-20260926/production-ticks-175-176.jsonl.gz';

function fixture(t) {
  const root = mkdtempSync(join(tmpdir(), 'physics-baseline-test-'));
  t.after(() => {
    assert.equal(dirname(resolve(root)), resolve(tmpdir()));
    assert.ok(basename(root).startsWith('physics-baseline-test-'));
    rmSync(root, { recursive: true, force: true });
  });
  const write = (path, contents) => {
    mkdirSync(dirname(join(root, path)), { recursive: true });
    writeFileSync(join(root, path), contents);
  };
  const git = args => execFileSync('git', args, { cwd: root, encoding: 'utf8', windowsHide: true });
  write('.gitignore', '/evidence/\n/node_modules/\n');
  write('package.json', JSON.stringify({ dependencies: { 'fixture-engine': '1.2.3' } }));
  write('package-lock.json', '{"lockfileVersion":3}\n');
  write('node_modules/fixture-engine/package.json', '{"version":"1.2.3"}\n');
  write('src/character/physics-settings.ts', 'export const dt = 1 / 60;\n');
  write('src/controller.ts', 'original controller\n');
  write('src/deleted.ts', 'deleted later\n');
  write('src/staged-deletion.ts', 'deleted from index later\n');
  write('hooks/fixture.ts', 'fixture hook\n');
  write('eslint.config.mjs', 'export default [];\n');
  git(['init', '-q']);
  git(['config', 'core.autocrlf', 'false']);
  git(['add', '.']);
  git(['-c', 'user.name=Baseline test', '-c', 'user.email=baseline@example.invalid', 'commit', '-qm', 'fixture']);
  const run = (...args) => spawnSync(process.execPath, [script, ...args], { cwd: root, encoding: 'utf8', windowsHide: true });
  const read = path => readFileSync(join(root, path));
  return { root, write, git, run, read };
}

test('fresh CLI capture succeeds without history and scopes versions, hashes, and planned inputs', t => {
  const f = fixture(t);
  const result = f.run('--fresh', 'evidence/new-baseline');
  assert.equal(result.status, 0, result.stderr);
  const record = JSON.parse(f.read('evidence/new-baseline/baseline.json'));
  assert.equal(record.mode, 'fresh');
  assert.equal(record.savedFocused, null);
  assert.equal(record.captureExitCode, 0);
  assert.equal(record.node.version, process.version);
  assert.equal(record.node.executableHash, sha256(readFileSync(process.execPath)));
  assert.equal(record.dependencies['fixture-engine'].version, '1.2.3');
  assert.equal(record.dependencies['fixture-engine'].packageHash, sha256(f.read('node_modules/fixture-engine/package.json')));
  assert.deepEqual(record.source, fingerprints(f.root));
  assert.deepEqual(record.sourceAfter, record.source);
  assert.equal(record.sourceFingerprint, sha256(JSON.stringify(record.source)));
  assert.ok(record.source['package-lock.json']);
  assert.ok(record.source['hooks/fixture.ts']);
  assert.ok(record.source['eslint.config.mjs']);
  assert.equal(record.evidence.completionDirectory.availability, 'missing');
  assert.equal(record.evidence.files[historicalReport].availability, 'missing');
  assert.equal(record.evidence.files[trace].availability, 'missing');
  assert.equal(record.evidence.files[trace].expectedSha256, 'f4ae9784f98fc88bcf72cda036944af220704683de88d878c16890bca8b8daa8');
  assert.deepEqual(record.priorEvidenceHashes, {});
  assert.equal(record.scenarios.execution, 'not-run');
  assert.deepEqual(record.scenarios.headings, [0, Math.PI / 3, -Math.PI / 4]);
  assert.equal(record.scenarios.idleSettlingS, 2);
  assert.equal(record.scenarios.idleObservationS, 30);
  assert.deepEqual(record.command.slice(-2), ['--fresh', 'evidence/new-baseline']);
  assert.equal(JSON.parse(result.stdout).testsExecuted, 0);
});

test('capture preserves staged, unstaged, binary, deleted and untracked work without changing Git', t => {
  const f = fixture(t);
  f.write('src/controller.ts', 'staged controller\n');
  f.write('src/staged.bin', Buffer.from([0, 255, 128, 1]));
  f.git(['add', 'src/controller.ts', 'src/staged.bin']);
  f.git(['rm', '-q', 'src/staged-deletion.ts']);
  f.write('src/controller.ts', 'working controller\n');
  rmSync(join(f.root, 'src/deleted.ts'));
  f.write('tests/untracked space é.mjs', 'untracked test\n');
  const statusBefore = f.git(['status', '--porcelain=v1']);
  const result = f.run('--fresh', 'evidence/dirty');
  assert.equal(result.status, 0, result.stderr);
  assert.equal(f.git(['status', '--porcelain=v1']), statusBefore);
  const record = JSON.parse(f.read('evidence/dirty/baseline.json'));
  assert.equal(record.status, statusBefore);
  for (const path of ['src/controller.ts', 'src/staged.bin', 'tests/untracked space é.mjs']) {
    assert.deepEqual(f.read(`evidence/dirty/preserved-worktree/${path}`), f.read(path));
    assert.equal(record.preservedWorktree[path].sha256, sha256(f.read(path)));
  }
  assert.equal(record.preservedWorktree['src/deleted.ts'].state, 'deleted');
  assert.equal(record.preservedWorktree['src/staged-deletion.ts'].state, 'deleted');
  assert.equal(record.preservedWorktree['tests/untracked space é.mjs'].state, 'untracked');
  const staged = f.read('evidence/dirty/index.patch').toString();
  const unstaged = f.read('evidence/dirty/worktree.patch').toString();
  assert.match(staged, /\+staged controller/);
  assert.match(staged, /GIT binary patch/);
  assert.match(staged, /staged-deletion/);
  assert.match(unstaged, /\+working controller/);
  assert.match(unstaged, /deleted file mode/);
  assert.equal(record.patches['index.patch'], sha256(staged));
  assert.equal(record.patches['worktree.patch'], sha256(unstaged));
});

test('fresh capture inventories empty and changed history without parsing or inheriting its results', t => {
  const f = fixture(t);
  f.write(historicalReport, '{"summary":{"passed":140,"total":142},"exitCode":1}');
  f.write(trace, '');
  f.write('evidence/physics-repair-20260924-completion/nested/partial.json', 'invalid JSON');
  const original = f.read(historicalReport);
  const result = f.run('evidence/with-history', '--fresh');
  assert.equal(result.status, 0, result.stderr);
  const record = JSON.parse(f.read('evidence/with-history/baseline.json'));
  assert.equal(record.savedFocused, null);
  assert.equal(record.evidence.files[historicalReport].availability, 'available');
  assert.equal(record.evidence.files[historicalReport].matchesExpected, false);
  assert.equal(record.evidence.files[trace].availability, 'empty');
  assert.equal(record.evidence.files[trace].matchesExpected, false);
  assert.equal(record.priorEvidenceHashes[historicalReport], sha256(original));
  assert.equal(record.summary, undefined);
  assert.deepEqual(f.read(historicalReport), original);
  f.write(historicalReport, 'invalid JSON');
  assert.equal(f.run('--fresh', 'evidence/malformed-history').status, 0);
  const strict = f.run('evidence/strict-history');
  assert.notEqual(strict.status, 0);
  assert.match(strict.stderr, /does not match its recorded digest/);
  assert.equal(existsSync(join(f.root, 'evidence/strict-history')), false);
});

test('strict mode fails clearly without history and missing dependencies fail before reserving a run', t => {
  const f = fixture(t);
  const strict = f.run('evidence/strict');
  assert.notEqual(strict.status, 0);
  assert.match(strict.stderr, /Use --fresh/);
  assert.equal(existsSync(join(f.root, 'evidence/strict')), false);
  rmSync(join(f.root, 'node_modules/fixture-engine/package.json'));
  const missing = f.run('--fresh', 'evidence/missing-dependency');
  assert.notEqual(missing.status, 0);
  assert.match(missing.stderr, /Install the locked dependencies/);
  assert.equal(existsSync(join(f.root, 'evidence/missing-dependency')), false);
});

test('existing runs and source directories cannot be used as output, and unknown flags fail', t => {
  const f = fixture(t);
  assert.equal(f.run('--fresh', 'evidence/immutable').status, 0);
  const baseline = f.read('evidence/immutable/baseline.json');
  const repeat = f.run('--fresh', 'evidence/immutable');
  assert.notEqual(repeat.status, 0);
  assert.match(repeat.stderr, /immutable/);
  assert.deepEqual(f.read('evidence/immutable/baseline.json'), baseline);
  f.write('evidence/partial/trace.jsonl', 'partial evidence');
  assert.notEqual(f.run('--fresh', 'evidence/partial').status, 0);
  assert.equal(f.read('evidence/partial/trace.jsonl').toString(), 'partial evidence');
  for (const args of [['--fresh', 'src/evidence'], ['--unknown', 'evidence/invalid'],
    ['--fresh', '--fresh', 'evidence/invalid'], ['--fresh']]) {
    assert.notEqual(f.run(...args).status, 0, args.join(' '));
  }
  assert.equal(existsSync(join(f.root, 'src/evidence')), false);
  assert.equal(existsSync(join(f.root, 'evidence/invalid')), false);
});
