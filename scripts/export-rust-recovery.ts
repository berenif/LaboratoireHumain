import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { RECOVERY_POSE_FIXTURES, recoveryFixturePoses } from './recovery-fixtures';

const sources = ['scripts/recovery-fixtures.ts', 'src/core/humanoid.ts',
  'src/character/joint-coordinates.ts', 'src/character/math.ts'];
const sha = (path: string) => createHash('sha256').update(readFileSync(path)).digest('hex');
const cases = RECOVERY_POSE_FIXTURES.map(fixture => ({
  ...fixture, poses: [...recoveryFixturePoses(fixture).values()],
}));
const output = 'rust/crates/sim/data/recovery-v1.json';
writeFileSync(output, JSON.stringify({schema:1, cases}, null, 2)+'\n', {flag:'wx'});
writeFileSync('rust/crates/sim/data/recovery-provenance-v1.json', JSON.stringify({
  schema:1, command:'node --import tsx scripts/export-rust-recovery.ts',
  sources:Object.fromEntries(sources.map(path=>[path,sha(path)])), output:{path:output,sha256:sha(output)},
},null,2)+'\n', {flag:'wx'});
console.log(JSON.stringify({cases:cases.length,sha256:sha(output)}));
