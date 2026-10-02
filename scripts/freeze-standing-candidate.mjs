import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const id = process.argv[2];
assert.ok(['h78-v1','h79-v1','h80-v1'].includes(id), 'Only the three authorized identities may be frozen');
const path = `docs/experiments/standing-${id}.json`, lock = path.replace('.json','.lock.json');
assert.ok(!existsSync(lock), 'A frozen candidate cannot be overwritten; preserve its failure and justify a successor');
const manifestDependencies = {};
let current = path;
while(current) {
  assert.ok(!manifestDependencies[current], 'Manifest inheritance cycle');
  const bytes = readFileSync(current), manifest = JSON.parse(bytes);
  manifestDependencies[current] = sha256(bytes);
  current = manifest.extends ? join(dirname(current),manifest.extends).replaceAll('\\','/') : null;
}
writeFileSync(lock, JSON.stringify({ schema: 1, id, frozenAt: new Date().toISOString(), source: fingerprints(),
  manifestDependencies, policy: 'No source/config changes during acceptance. A timeout is failed or incomplete evidence; no fourth candidate.' },null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({id,lock,sha256:sha256(readFileSync(lock))}));
