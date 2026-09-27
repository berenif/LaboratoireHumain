import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { isDeepStrictEqual } from 'node:util';
import { register } from 'tsx/esm/api';
import RAPIER from '@dimforge/rapier3d-compat';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const [input, output] = process.argv.slice(2);
if (!input || !output || existsSync(output)) throw new Error('Usage: node scripts/verify-impulse-snapshot-replay.mjs exported-report.json fresh-output-directory');
const unregister = register();
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { jointCoordinateKinematics, jointCoordinates } = await import('../src/character/joint-coordinates.ts');
const sourceBefore = fingerprints(), report = JSON.parse(readFileSync(input)), root = dirname(input), results = [];
assert.ok(report.parameters.exportSnapshots);
mkdirSync(output, { recursive: true });
await RAPIER.init();
const parts = handle => { const view = new DataView(new ArrayBuffer(8)); view.setFloat64(0, handle, true); return [view.getUint32(0, true), view.getUint32(4, true)]; };
const handleFromParts = ([index, generation]) => { const view = new DataView(new ArrayBuffer(8)); view.setUint32(0, index, true); view.setUint32(4, generation, true); return view.getFloat64(0, true); };
const pairKey = (a, b) => [a.join(':'), b.join(':')].sort().join('|');
const physical = (world, metadata) => Object.entries(metadata.bodyHandles).map(([id, handle]) => {
  const body = world.getRigidBody(handleFromParts(handle));
  return { id, position: body.translation(), rotation: body.rotation(), linearVelocity: body.linvel(), angularVelocity: body.angvel() };
});
const plain = value => JSON.parse(JSON.stringify(value));
try {
  for (const [index, item] of report.results.entries()) {
    const evidenceBytes = readFileSync(join(root, item.evidence.file));
    assert.equal(sha256(evidenceBytes), item.evidence.sha256);
    const evidence = JSON.parse(evidenceBytes), pair = {};
    for (const side of ['control', 'driven']) {
      const snapshot = evidence.snapshots[side], metadataBytes = readFileSync(join(root, snapshot.metadataFile));
      assert.equal(sha256(metadataBytes), snapshot.metadataSha256);
      const metadata = JSON.parse(metadataBytes);
      const bytes = readFileSync(join(root, snapshot.snapshotFile)); assert.equal(sha256(bytes), snapshot.snapshotSha256);
      const world = RAPIER.World.restoreSnapshot(bytes), eventQueue = new RAPIER.EventQueue(true);
      try {
        assert.ok(isDeepStrictEqual(plain(physical(world, metadata)), metadata.bodiesBeforeStep), 'Restoration changed an initial body value');
        const excluded = new Set(metadata.excludedColliderPairs.map(([a, b]) => pairKey(a, b)));
        // Rapier JS only forwards hooks through stepWithEvents. Omitting the
        // queue silently disables collision exclusions in the wrapper.
        world.step(eventQueue, { filterContactPair: (a, b) => excluded.has(pairKey(parts(a), parts(b))) ? null : RAPIER.SolverFlags.COMPUTE_IMPULSE,
          filterIntersectionPair: () => true });
        pair[side] = plain(physical(world, metadata));
      } finally { eventQueue.free(); world.free(); }
    }
    const rate = side => {
      const definition = SEGMENT_BY_ID.get(item.sourceId), bodies = new Map(pair[side].map(b => [b.id, b]));
      const a = bodies.get(definition.parent), b = bodies.get(item.sourceId);
      const q = jointCoordinates(a.rotation, b.rotation, definition.jointProfile);
      return jointCoordinateKinematics(a.angularVelocity, b.angularVelocity, a.rotation, q, definition.jointProfile).rates.x;
    };
    const actual = (rate('driven') - rate('control')) / item.impulseNmS;
    const matches = Object.fromEntries(['control', 'driven'].map(side => [side, isDeepStrictEqual(pair[side], evidence[side].bodies)]));
    const row = { index, impulseNmS: item.impulseNmS, actual, originalActual: item.actual, matches, pair };
    const bytes = JSON.stringify(row) + '\n', file = `${index}.json`;
    writeFileSync(join(output, file), bytes, { flag: 'wx' });
    results.push({ index, impulseNmS: item.impulseNmS, actual, originalActual: item.actual, matches, file, sha256: sha256(bytes) });
  }
  const sourceAfter = fingerprints(); assert.deepEqual(sourceAfter, sourceBefore);
  writeFileSync(join(output, 'report.json'), JSON.stringify({ generatedAt: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version, engineVersion: RAPIER.version(),
    input: { file: input, sha256: sha256(readFileSync(input)) }, sourceBefore, sourceAfter, results }, null, 2) + '\n', { flag: 'wx' });
  console.log(JSON.stringify(results.map(({ index, impulseNmS, actual, originalActual, matches }) => ({ index, impulseNmS, actual, originalActual, matches }))));
} finally { unregister(); }
