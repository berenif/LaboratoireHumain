import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';

const upstream = readFileSync('evidence/rust-rework-20261001/baseline/contact-upstream.patch', 'utf8');
const retained = [];
for (const section of upstream.split(/(?=^diff --git )/m).slice(1)) {
  if (!section.startsWith('diff --git a/src/')) continue;
  const [header, ...hunks] = section.split(/(?=^@@ )/m);
  // 0.35.0 already banks before scaling. Retain every accumulator initialization
  // and documentation hunk; don't reapply the later upstream ordering change.
  const chosen = hunks.filter(h => !h.includes('-                normal_part.impulse *=') && !h.includes('-            tangent_part.impulse *='));
  retained.push(header + chosen.join(''));
}
const patch = retained.join('').replace(/\n-- \n[\s\S]*$/, '\n');
writeFileSync('rust/vendor/contact-reporting.patch', patch, { flag: 'wx' });
const before = {};
const paths = [...patch.matchAll(/^diff --git a\/(\S+) /gm)].map(m => m[1]);
const sha = b => createHash('sha256').update(b).digest('hex');
for (const path of paths) before[path] = sha(readFileSync(`rust/vendor/rapier3d/${path}`));
const applied = spawnSync('git', ['apply','--directory=rust/vendor/rapier3d','rust/vendor/contact-reporting.patch'], { encoding: 'utf8', windowsHide: true });
assert.equal(applied.status, 0, applied.stderr);
for (const path of paths.filter(p => !p.endsWith('contact_constraint_element.rs'))) {
  const text = readFileSync(`rust/vendor/rapier3d/${path}`, 'utf8');
  assert.match(text, /normal_part\.impulse_accumulator \+= normal_part\.impulse;\s*normal_part\.impulse \*=/);
  assert.match(text, /tangent_part\.impulse_accumulator \+= tangent_part\.impulse;\s*tangent_part\.impulse \*=/);
}
writeFileSync('rust/vendor/contact-provenance.json', JSON.stringify({ schema: 1,
  upstream: 'https://github.com/dimforge/rapier/commit/37eeac7dec6addd6a94a237b4874ca0b676d70da',
  package: 'rapier3d 0.35.0', crateSha256: 'c56bf7b5596ef716abcf578636db53b3780162043fada35717d46f79d4022fa4',
  upstreamPatchSha256: sha(Buffer.from(upstream)), patchSha256: sha(Buffer.from(patch)), before,
  after: Object.fromEntries(paths.map(p => [p, sha(readFileSync(`rust/vendor/rapier3d/${p}`))])),
  scope: 'Reporting accumulators only. 0.35.0 already banks before warm-start scaling; omitted upstream ordering hunks checked against source. No solver actuation or constraints changed. Calibration required before admission.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ paths, status: 'applied; awaiting calibration' }));
