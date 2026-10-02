import assert from 'node:assert/strict';
import { copyFileSync, cpSync, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { spawnSync } from 'node:child_process';
import { inventory, json, verifyInventory } from './coordinated-standing-evidence.mjs';
import { sha256 } from './capture-physics-baseline.mjs';

const output = resolve(process.argv[2]);
assert.ok(!existsSync(output), 'Use a fresh native build directory');
mkdirSync(output, { recursive: true });
const experiment = 'docs/experiments/standing-h77-v1.json';
if (existsSync(experiment)) copyFileSync(experiment, join(output, 'experiment.json'));

const original = resolve('evidence/rapier-calibration-cache/registry/src/index.crates.io-1949cf8c6b5b557f/rapier3d-0.35.0');
const native = join(output, 'rapier3d');
const prior = JSON.parse(readFileSync('evidence/impulse-native-precision-20260927-06/engine-features.json'));
const pinned = prior.packages.find(packageInfo => packageInfo.name === 'rapier3d');
for (const [path, hash] of Object.entries(pinned.files)) {
  assert.equal(sha256(readFileSync(join(original, 'src', path))), hash, path);
}
cpSync(original, native, { recursive: true, errorOnExist: true, force: false });

function replaceOnce(path, before, after) {
  const file = join(native, path);
  const source = readFileSync(file, 'utf8');
  assert.equal(source.split(before).length, 2, `Patch anchor must be unique: ${path}`);
  writeFileSync(file, source.replace(before, after));
}

replaceOnce('src/dynamics/mod.rs', 'pub(crate) mod solver;', 'pub mod solver;');
replaceOnce('src/dynamics/solver/mod.rs', 'mod velocity_solver;', 'mod velocity_solver;\npub mod standing_trace;');
copyFileSync('scripts/standing-substep-trace-module.rs', join(native, 'src/dynamics/solver/standing_trace.rs'));

const workerPath = 'src/dynamics/solver/staged_island_solver/worker.rs';
replaceOnce(workerPath, 'use parry::math::SIMD_WIDTH;',
  'use parry::math::SIMD_WIDTH;\nuse simba::simd::SimdValue;');
const helper = readFileSync('scripts/standing-substep-worker-trace.rs', 'utf8');
replaceOnce(workerPath,
  'const MAX_ROTATION: Real = core::f64::consts::FRAC_PI_4 as Real;',
  `const MAX_ROTATION: Real = core::f64::consts::FRAC_PI_4 as Real;\n\n${helper}`);

const barrier = phase => `
            if crate::dynamics::solver::standing_trace::enabled() {
                if worker_id == 0 {
                    unsafe { standing_trace_state(ctx, group, substep_id, "${phase}") };
                    sync.complete(stage, 1, 1);
                }
                stage = sync.sync(stage, 1);
            }
`;
replaceOnce(workerPath,
  '            let solved_dt = substep_id as Real * params.dt;\n',
  `            let solved_dt = substep_id as Real * params.dt;\n${barrier('pre')}`);
replaceOnce(workerPath,
  '            /*\n             * Stages: solve with bias.\n',
  `${barrier('warmstarted')}\n            /*\n             * Stages: solve with bias.\n`);
replaceOnce(workerPath,
  '            }\n        }\n\n        /*\n         * Stages: end-of-step restitution',
  `            }\n${barrier('solved')}        }\n\n        /*\n         * Stages: end-of-step restitution`);

mkdirSync(join(output, 'src'));
copyFileSync('scripts/standing-substep-native.rs', join(output, 'src/main.rs'));
copyFileSync('scripts/rapier-calibration/Cargo.lock', join(output, 'Cargo.lock'));
writeFileSync(join(output, 'Cargo.toml'), `[package]\nname = "standing-substep-native"\nversion = "0.1.0"\nedition = "2024"\n\n[dependencies]\nrapier3d = { path = "rapier3d", features = ["serde-serialize"] }\nbincode = "1.3.3"\nserde_json = { version = "1", features = ["arbitrary_precision", "float_roundtrip"] }\n`);

const retainedToolchain = resolve('evidence/standing-h75-v1/toolchain/toolchains/1.89.0-x86_64-pc-windows-gnu/bin');
const installedToolchain = resolve(process.env.USERPROFILE ?? '', '.rustup/toolchains/1.89.0-x86_64-pc-windows-gnu/bin');
const toolchain = resolve(process.env.H75_TOOLCHAIN_BIN
  ?? (existsSync(join(retainedToolchain, 'cargo.exe')) ? retainedToolchain : installedToolchain));
const rustTarget = process.env.H75_RUST_TARGET
  ?? (toolchain === retainedToolchain ? null : 'x86_64-pc-windows-gnu');
const environment = {
  CARGO_HOME: resolve('evidence/rapier-calibration-cache'),
  RUSTC: join(toolchain, 'rustc.exe'),
  CARGO_TARGET_DIR: resolve(process.env.H77_NATIVE_TARGET ?? join(output, 'target')),
};
const command = [join(toolchain, 'cargo.exe'), 'build', '--offline', '--release',
  ...(rustTarget ? ['--target', rustTarget] : []), '--manifest-path', join(output, 'Cargo.toml')];
const started = performance.now();
const result = spawnSync(command[0], command.slice(1), {
  env: { ...process.env, ...environment }, encoding: 'utf8', windowsHide: true,
  timeout: 20 * 60 * 1000, maxBuffer: 32 * 2 ** 20,
});
writeFileSync(join(output, 'build.log'), `${result.stdout ?? ''}${result.stderr ?? ''}${result.error ? `\nspawn error: ${result.error.message}\n` : ''}`, { flag: 'wx' });
const executable = join(output, 'standing-substep-native.exe');
if (result.status === 0) {
  copyFileSync(join(environment.CARGO_TARGET_DIR, ...(rustTarget ? [rustTarget] : []),
    'release/standing-substep-native.exe'), executable);
}
const sources = inventory(join(output, 'src'));
const nativeSources = inventory(join(native, 'src'));
json(join(output, 'build-report.json'), {
  status: result.status === 0 ? 'pass' : 'fail', command, environment, rustTarget,
  exit: result.status, error: result.error?.message ?? null, wallMs: performance.now() - started,
  originalEngineSourceMatches: Object.keys(pinned.files).length,
  executable, executableSha256: existsSync(executable) ? sha256(readFileSync(executable)) : null,
  sources, nativeSources,
  qualification: 'Isolated source copy with output-neutral reads at solver barriers. Production Rapier/WASM and application dependencies are unchanged.',
});
verifyInventory(join(native, 'src'), nativeSources);
console.log(JSON.stringify({ exit: result.status, wallMs: performance.now() - started, output }));
process.exitCode = result.status === 0 ? 0 : 1;
