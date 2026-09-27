import assert from 'node:assert/strict';
import { copyFileSync, cpSync, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { spawnSync } from 'node:child_process';
import { inventory, json, verifyInventory } from './coordinated-standing-evidence.mjs';
import { sha256 } from './capture-physics-baseline.mjs';

const output = resolve(process.argv[2]);
assert.ok(!existsSync(output), 'Use a fresh native build directory');
mkdirSync(output, { recursive: true });
const experiment = 'docs/experiments/standing-h75-v1.json';
copyFileSync(experiment, join(output, 'experiment.json'));
const manifest = JSON.parse(readFileSync(experiment));
const original = resolve('evidence/rapier-calibration-cache/registry/src/index.crates.io-1949cf8c6b5b557f/rapier3d-0.35.0');
const native = join(output, 'rapier3d');
const prior = JSON.parse(readFileSync('evidence/impulse-native-precision-20260927-06/engine-features.json'));
const pinned = prior.packages.find(p => p.name === 'rapier3d');
for (const [path, hash] of Object.entries(pinned.files)) assert.equal(sha256(readFileSync(join(original, 'src', path))), hash, path);
cpSync(original, native, { recursive: true, errorOnExist: true, force: false });
const patches = [
  ['src/dynamics/mod.rs', 'pub(crate) mod solver;', 'pub mod solver;'],
  ['src/dynamics/solver/mod.rs', 'pub(crate) use joint_constraint::MotorParameters;', 'pub use joint_constraint::MotorParameters;'],
  ['src/dynamics/solver/joint_constraint/mod.rs', 'pub use joint_velocity_constraint::{JointSolverBody, MotorParameters, WritebackId};',
    'pub use joint_velocity_constraint::{JointConstraint, JointSolverBody, MotorParameters, WritebackId};'],
  ['src/dynamics/joint/generic_joint.rs', 'pub(crate) fn motor_params(&self, dt: Real)', 'pub fn motor_params(&self, dt: Real)'],
];
for (const [path, before, after] of patches) {
  const file = join(native, path), source = readFileSync(file, 'utf8');
  assert.equal(source.split(before).length, 2, path);
  writeFileSync(file, source.replace(before, after));
}
mkdirSync(join(output, 'src'));
copyFileSync('scripts/standing-angular-native.rs', join(output, 'src/main.rs'));
copyFileSync('scripts/rapier-calibration/Cargo.lock', join(output, 'Cargo.lock'));
writeFileSync(join(output, 'Cargo.toml'), `[package]\nname = "standing-angular-native"\nversion = "0.1.0"\nedition = "2024"\n\n[dependencies]\nrapier3d = { path = "rapier3d", features = ["serde-serialize"] }\nbincode = "1.3.3"\nserde_json = { version = "1", features = ["arbitrary_precision", "float_roundtrip"] }\n`);
const toolchain = resolve(process.env.H75_TOOLCHAIN_BIN ?? 'evidence/standing-h75-v1/toolchain/toolchains/1.89.0-x86_64-pc-windows-gnu/bin');
const environment = { CARGO_HOME: resolve('evidence/rapier-calibration-cache'), RUSTC: join(toolchain, 'rustc.exe'), CARGO_TARGET_DIR: resolve(process.env.H75_NATIVE_TARGET ?? join(output, 'target')) };
const command = [join(toolchain, 'cargo.exe'), 'build', '--offline', '--release', '--manifest-path', join(output, 'Cargo.toml')];
const start = performance.now();
const result = spawnSync(command[0], command.slice(1), { env: { ...process.env, ...environment }, encoding: 'utf8', windowsHide: true, timeout: manifest.budget.maximumBuildWallS * 1000 });
writeFileSync(join(output, 'build.log'), result.stdout + result.stderr, { flag: 'wx' });
const metadata = spawnSync(command[0], ['metadata', '--offline', '--format-version', '1', '--manifest-path', join(output, 'Cargo.toml')],
  { env: { ...process.env, ...environment }, encoding: 'utf8', windowsHide: true, timeout: 30000, maxBuffer: 16 * 2**20 });
if (metadata.status === 0) json(join(output, 'metadata.json'), JSON.parse(metadata.stdout));
const executable = join(output, 'standing-angular-native.exe');
if (result.status === 0) copyFileSync(join(environment.CARGO_TARGET_DIR, 'release/standing-angular-native.exe'), executable);
const sources = inventory(join(output, 'src')), nativeSources = inventory(join(native, 'src'));
json(join(output, 'build-report.json'), { status: result.status === 0 ? 'pass' : 'fail', command, environment,
  exit: result.status, error: result.error?.message ?? null, wallMs: performance.now() - start,
  toolchain: { rustc: spawnSync(environment.RUSTC, ['--version', '--verbose'], { encoding: 'utf8', windowsHide: true }).stdout,
    cargoSha256: sha256(readFileSync(command[0])), rustcSha256: sha256(readFileSync(environment.RUSTC)) },
  experimentSha256: sha256(readFileSync(experiment)), patches, originalEngineSourceMatches: Object.keys(pinned.files).length,
  originalEngineFeaturesSha256: sha256(readFileSync('evidence/impulse-native-precision-20260927-06/engine-features.json')),
  executable, executableSha256: existsSync(executable) ? sha256(readFileSync(executable)) : null, sources, nativeSources,
  qualification: 'Visibility-only changes in an isolated copy; native row reconstruction calls the retained engine implementation. No native simulation, production dependency change or WASM substep trace.' });
verifyInventory(join(native, 'src'), nativeSources);
console.log(JSON.stringify({ exit: result.status, wallMs: performance.now() - start, output }));
process.exitCode = result.status === 0 ? 0 : 1;
