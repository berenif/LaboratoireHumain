import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { registerHooks, stripTypeScriptTypes } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { sha256 } from './capture-physics-baseline.mjs';

// Opt-in, process-local runtime experiment. No package, production source or
// test assertion is rewritten. The adapter's version pin alone is overlaid;
// its raw-method validation, angular limits and readback checks are retained.
assert.ok(process.env.DIAGNOSTIC_RAPIER_PACKAGE, 'Set DIAGNOSTIC_RAPIER_PACKAGE explicitly');
const packagePath = resolve(process.env.DIAGNOSTIC_RAPIER_PACKAGE);
const packageBytes = readFileSync(resolve(packagePath, 'package.json'));
const metadata = JSON.parse(packageBytes);
assert.equal(metadata.name, '@dimforge/rapier3d-compat');
assert.ok(['0.19.3', '0.20.0', '0.21.0'].includes(metadata.version), 'Undeclared diagnostic runtime');
const esm = resolve(packagePath, metadata.module), runtimeUrl = pathToFileURL(esm).href;
const adapterPath = resolve('src/character/rapier-joint-adapter.ts');
const adapter = readFileSync(adapterPath, 'utf8');
const pin = 'export const EXPECTED_RAPIER_VERSION = "0.20.0";';
assert.equal(adapter.split(pin).length, 2, 'Adapter source changed; review the diagnostic overlay');
const overlay = adapter.replace(pin, `export const EXPECTED_RAPIER_VERSION = "${metadata.version}";`);
const receipt = { version: metadata.version, packagePath, packageSha256: sha256(packageBytes),
  esmSha256: sha256(readFileSync(esm)),
  wasmSha256: sha256(readFileSync(resolve(dirname(esm), 'rapier_wasm3d_bg.wasm'))),
  adapterPath, adapterSourceSha256: sha256(adapter), adapterOverlaySha256: sha256(overlay),
  qualification: 'Process-local dependency experiment. Only the adapter version-pin literal is changed in memory; production files, raw ABI validations and all test assertions remain unchanged.' };
globalThis[Symbol.for('laboratoire.rapierRuntimeOverride')] = receipt;
registerHooks({
  resolve(specifier, context, nextResolve) {
    return specifier === '@dimforge/rapier3d-compat'
      ? { url: runtimeUrl, shortCircuit: true } : nextResolve(specifier, context);
  },
  load(url, context, nextLoad) {
    if (url.startsWith('file:') && fileURLToPath(url).toLowerCase() === adapterPath.toLowerCase()) {
      return { format: 'module', source: stripTypeScriptTypes(overlay, { mode: 'strip' }), shortCircuit: true };
    }
    return nextLoad(url, context);
  },
});
console.error(JSON.stringify({ diagnosticRuntimeOverride: receipt }));
