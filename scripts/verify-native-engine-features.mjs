import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, readFileSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { sha256 } from './capture-physics-baseline.mjs';

const [output] = process.argv.slice(2);
if (!output || existsSync(output)) throw new Error('Specify a fresh output JSON file. Set the isolated CARGO_HOME and RUSTUP_HOME first.');
const command = ['+1.89.0-x86_64-pc-windows-gnu', 'metadata', '--offline', '--format-version', '1', '--features', 'f64', '--manifest-path', 'scripts/rapier-calibration/Cargo.toml'];
const result = spawnSync('cargo', command, { encoding: 'utf8', windowsHide: true, maxBuffer: 16 * 1024 * 1024 });
assert.equal(result.status, 0, result.stderr);
const metadata = JSON.parse(result.stdout), packages = [], comparisons = [];
for (const [single, double] of [['rapier3d', 'rapier3d-f64'], ['parry3d', 'parry3d-f64']]) {
  const pair = [single, double].map(name => {
    const pkg = metadata.packages.find(p => p.name === name);
    const features = metadata.resolve.nodes.find(n => n.id === pkg.id).features;
    const root = dirname(pkg.manifest_path), featureUses = new Set();
    const files = Object.fromEntries(readdirSync(join(root, 'src'), { recursive: true })
      .filter(file => statSync(join(root, 'src', file)).isFile())
      .map(file => {
        const bytes = readFileSync(join(root, 'src', file));
        for (const match of bytes.toString('utf8').matchAll(/feature\s*=\s*"([^"]+)"/g)) featureUses.add(match[1]);
        return [file.replaceAll('\\', '/'), sha256(bytes)];
      }));
    const dependencies = metadata.resolve.nodes.find(n => n.id === pkg.id).deps
      .filter(d => d.dep_kinds.some(k => k.kind === null)).map(d => {
        const dependency = metadata.packages.find(p => p.id === d.pkg);
        return { name: dependency.name.replace('-f64', ''), version: dependency.version };
      }).sort((a, b) => a.name.localeCompare(b.name));
    const item = { name, version: pkg.version, features, manifestSha256: sha256(readFileSync(pkg.manifest_path)),
      vcs: JSON.parse(readFileSync(join(root, '.cargo_vcs_info.json'))), featureUses: [...featureUses].sort(), dependencies, files };
    packages.push(item); return item;
  });
  assert.deepEqual(pair[0].files, pair[1].files, 'Published engine sources differ');
  // Some published manifests spell always-used dependencies as optional while
  // the other precision spells them as required. Check actual dependencies and
  // source cfg features; an implicit dependency feature alone is not a branch.
  // Rapier's f32 SolverBlock/RawBlock uses `wide` for the four-lane data
  // transpose; f64 uses AutoF64x4/plain arrays (src/utils/mod.rs). This is part
  // of the precision implementation, not an independently enabled solver.
  // Parry additionally compiles f32 SIMD layout assertions; static_assertions
  // emits compile-time checks and cannot alter runtime physics.
  const precisionDependencies = single === 'rapier3d' ? ['wide'] : ['static_assertions'];
  const commonDependencies = dependencies => dependencies.filter(d => !precisionDependencies.includes(d.name));
  assert.deepEqual(commonDependencies(pair[0].dependencies), commonDependencies(pair[1].dependencies));
  const normalize = features => features.filter(f => pair[0].featureUses.includes(f))
    .map(f => f === 'f32' || f === 'f64' ? 'precision' : f).sort();
  assert.deepEqual(normalize(pair[0].features), normalize(pair[1].features), 'Enabled engine features differ beyond precision');
  assert.ok(pair.every(p => !p.features.includes('block-solver')));
  comparisons.push({ single, double, identicalSourceFiles: Object.keys(pair[0].files).length,
    normalizedFeatures: normalize(pair[0].features), precisionDependencies });
}
const wrapper = ['scripts/rapier-calibration/Cargo.toml', 'scripts/rapier-calibration/Cargo.lock', 'scripts/rapier-calibration/src/main.rs'];
writeFileSync(output, JSON.stringify({ generatedAt: new Date().toISOString(), command: ['cargo', ...command],
  exitCode: result.status, wrapperHashes: Object.fromEntries(wrapper.map(file => [file, sha256(readFileSync(file))])),
  comparisons, packages }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify(comparisons));
