// Read-only observer added only to an already pinned isolated fixture artifact.
import assert from 'node:assert/strict';
import { cpSync, existsSync, lstatSync, readFileSync, readdirSync, realpathSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { createHash } from 'node:crypto';
const input = resolve(process.argv[2]), output = resolve(process.argv[3]);
const manifestPath = input + '-fixture-adapter-manifest.json';
const manifest = JSON.parse(readFileSync(manifestPath));
const root = realpathSync(process.cwd()), hash = b => createHash('sha256').update(b).digest('hex');
assert.ok(input.startsWith(root + sep) && output.startsWith(root + sep));
assert.equal(realpathSync(input), input);
assert.equal(realpathSync(dirname(output)), dirname(output));
assert.ok(!output.startsWith(input + sep) && !input.startsWith(output + sep));
assert.ok(!existsSync(output) && !existsSync(output + '-fixture-adapter-manifest.json'));
function hashes(path, base = path) {
  return Object.fromEntries(readdirSync(path).flatMap(name => {
    const p = join(path, name), stat = lstatSync(p);
    assert.ok(!stat.isSymbolicLink());
    if (stat.isDirectory()) return Object.entries(hashes(p, base));
    assert.ok(stat.isFile());
    return [[relative(base, p), hash(readFileSync(p))]];
  }));
}
assert.deepEqual(hashes(input), manifest.artifactSha256);
assert.equal(manifest.artifactSha256['sim-worker.js'], 'da24141dabdd72cba4e57c84b6d40cdd2e876f8ac1f9555c369db8ed33947df2');
assert.equal(manifest.artifactSha256['lh_worker_bg.wasm'], '89408b1a8501bfeec8e203d8c97491e343aa4081d1212ae6ab30fd984307eb59');
assert.deepEqual(manifest.sourceBuildFeatures, ['lh-sim/standing-dynamics-experiment', 'lh-sim/getup-terminal-balance-experiment']);
let source = readFileSync(join(input, 'sim-worker.js'), 'utf8');
function replaceOnce(a, b) { assert.equal(source.split(a).length, 2); source = source.replace(a, b); }
replaceOnce('let postPassAnchors = null;', 'let postPassAnchors = null;\nlet historicalStableFeet = null;');
replaceOnce('  let postPassStanding = null;', '  let postPassStanding = null;\n  let historicalStanding = null;');
replaceOnce("    if (postPassAnchors === null) postPassAnchors = ['left_foot', 'right_foot'].map(id => {", `    if (historicalStableFeet === null) {
      const controller = JSON.parse(runtime.getup_report());
      if (!Array.isArray(controller.stable_feet) || controller.stable_feet.length !== 2
        || !controller.stable_feet.every(p => ['x','y','z'].every(k => Number.isFinite(p[k])))) {
        throw new Error('Exact original controller stable_feet required at first pass');
      }
      historicalStableFeet = controller.stable_feet;
    }
    const nativeHistorical = JSON.parse(runtime.getup_standing_diagnostic(JSON.stringify(historicalStableFeet)));
    if (nativeHistorical.physicsIntegrations !== 0
      || nativeHistorical.nativeSubsteps !== Math.round(reply.snapshot.simulation_time_s * 800)) {
      throw new Error('Historical-anchor observer integrated or mismatched native clock');
    }
    historicalStanding = { nativeSubsteps: nativeHistorical.nativeSubsteps,
      physicsIntegrations: nativeHistorical.physicsIntegrations, predicates: nativeHistorical.predicates,
      footDriftM: nativeHistorical.footDriftM, feet: nativeHistorical.feet,
      originalStableFeet: historicalStableFeet, thresholds: nativeHistorical.thresholds };
    if (postPassAnchors === null) postPassAnchors = ['left_foot', 'right_foot'].map(id => {`);
replaceOnce('triggerNative, triggerEvidence, postPassAnchors, postPassStanding },',
  'triggerNative, triggerEvidence, postPassAnchors, postPassStanding, historicalStanding },');
cpSync(input, output, { recursive: true });
writeFileSync(join(output, 'sim-worker.js'), source);
writeFileSync(output + '-fixture-adapter-manifest.json', JSON.stringify({
  at: new Date().toISOString(), input, output, isolatedTestOnly: true,
  scope: 'Only copied worker adds existing read-only getup_report once at first pass and standing_diagnostic with its exact original stable_feet each40Hz reply. No physical integration/control/request cadence or WASM change.',
  parentAdapterManifest: { path: manifestPath, sha256: hash(readFileSync(manifestPath)) },
  sourceBuildFeatures: manifest.sourceBuildFeatures, artifactSha256: hashes(output),
}, null, 2) + '\n');
console.log('Prepared read-only original-anchor diagnostic artifact ' + output);
