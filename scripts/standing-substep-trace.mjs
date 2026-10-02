import assert from 'node:assert/strict';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { spawnSync } from 'node:child_process';

const [executableArg, inputArg, collisionArg, outputArg] = process.argv.slice(2);
assert.ok(executableArg && inputArg && collisionArg && outputArg,
  'standing-substep-trace.mjs executable input.bin first-failure.json output.json');
const executable = resolve(executableArg);
const input = resolve(inputArg);
const collisionSource = JSON.parse(readFileSync(resolve(collisionArg)));
const output = resolve(outputArg);
assert.ok(!existsSync(output), `Refusing to overwrite ${output}`);

function rawIndex(handle) {
  const buffer = new ArrayBuffer(8);
  const view = new DataView(buffer);
  view.setFloat64(0, handle, true);
  return view.getUint32(0, true);
}

const colliderBySegment = new Map(collisionSource.collision.colliderSegments
  .map(([handle, segment]) => [segment, rawIndex(handle)]));
const excludedColliderIndices = collisionSource.collision.excludedPairs.map(pair => {
  const [first, second] = pair.split('|');
  const indices = [colliderBySegment.get(first), colliderBySegment.get(second)];
  assert.ok(indices.every(Number.isInteger), `Missing collider for ${pair}`);
  return indices.sort((a, b) => a - b);
});
const context = `${output}.context.json`;
writeFileSync(context, `${JSON.stringify({ excludedColliderIndices,
  source: resolve(collisionArg), hook: collisionSource.collision.hook }, null, 2)}\n`, { flag: 'wx' });
const result = spawnSync(executable, [input, output, context], {
  encoding: 'utf8', windowsHide: true, timeout: 120000, maxBuffer: 16 * 2 ** 20,
});
if (result.stdout) process.stdout.write(result.stdout);
if (result.stderr) process.stderr.write(result.stderr);
assert.equal(result.status, 0, result.error?.message ?? `trace exit ${result.status}`);
console.log(JSON.stringify({ output, context, input }));
