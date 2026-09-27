import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { registerHooks, stripTypeScriptTypes } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { gunzipSync } from 'node:zlib';
import { sha256 } from './capture-physics-baseline.mjs';
const archive = 'docs/checkpoints/2026-09-27/inertia-production-candidate/production-source.json.gz';
const bytes = readFileSync(archive), baseline = JSON.parse(gunzipSync(bytes));
const entry = 'src/character/pose.ts', path = resolve(entry);
for (const [file, contents] of Object.entries(baseline)) {
  if (file !== entry) assert.equal(sha256(readFileSync(file)), sha256(contents), `Baseline dependency changed: ${file}`);
}
assert.ok(!baseline[entry].includes('refineBoundedReach'));
const receipt = { archive, archiveSha256: sha256(bytes), entry, baselineSha256: sha256(baseline[entry]),
  currentSha256: sha256(readFileSync(path)), verifiedUnchangedDependencies: Object.keys(baseline).length - 1,
  qualification: 'Read-only process-local replay of the archived pose compositor. Current tests and all other production files are unchanged.' };
registerHooks({ load(url, context, nextLoad) {
  if (url.startsWith('file:') && fileURLToPath(url).toLowerCase() === path.toLowerCase()) {
    return { format: 'module', source: stripTypeScriptTypes(baseline[entry], { mode: 'strip' }), shortCircuit: true };
  }
  return nextLoad(url, context);
} });
console.error(JSON.stringify({ diagnosticPoseBaseline: receipt }));
