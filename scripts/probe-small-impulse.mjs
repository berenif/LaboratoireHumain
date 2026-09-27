import assert from 'node:assert/strict';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { register } from 'tsx/esm/api';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';

const output = process.argv[2];
if (!output || existsSync(output)) throw new Error('Specify a fresh output directory.');
const headings = JSON.parse(process.env.IMPULSE_HEADINGS ?? '[0,1.0471975511965976,-0.7853981633974483]');
const sources = (process.env.IMPULSE_SOURCES ?? 'rightThigh,rightAnkle').split(',');
const magnitudes = JSON.parse(process.env.IMPULSE_MAGNITUDES ?? '[0.0005,0.001,0.002]');
const repetitions = Number(process.env.IMPULSE_REPETITIONS ?? 1);
const originOffsets = JSON.parse(process.env.IMPULSE_ORIGIN_OFFSETS ?? '[0]');
const exportSnapshots = process.env.IMPULSE_EXPORT_SNAPSHOTS === '1';
const freshPipeline = process.env.IMPULSE_FRESH_PIPELINE === '1';
assert.ok(headings.length && headings.every(Number.isFinite));
assert.ok(sources.length && sources.every(id => ['rightThigh', 'rightAnkle'].includes(id)));
assert.ok(magnitudes.length && magnitudes.every(value => Number.isFinite(value) && value > 0));
assert.ok(Number.isInteger(repetitions) && repetitions > 0);
assert.ok(originOffsets.length && originOffsets.every(Number.isFinite));
mkdirSync(output, { recursive: true });
const sourceBefore = fingerprints(), started = new Date().toISOString();
const unregister = register();
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { articulatedCoordinateResponse } = await import('../src/character/articulated-inertia.ts');
const { jointCoordinateKinematics, jointCoordinates } = await import('../src/character/joint-coordinates.ts');
const { scale, dot, sub } = await import('../src/character/math.ts');
const zero = { x: 0, y: 0, z: 0 }, dt = 1 / 60;
const state = c => [...c.ragdollBodies].map(([id, body]) => ({ id, position: body.translation(), rotation: body.rotation(),
  linearVelocity: body.linvel(), angularVelocity: body.angvel() }));
function saveNativeSnapshot(c, stem, context) {
  const rawParts = handle => {
    const view = new DataView(new ArrayBuffer(8)); view.setFloat64(0, handle, true);
    return [view.getUint32(0, true), view.getUint32(4, true)];
  };
  const colliders = [...c.ragdollColliders.values()], excludedColliderPairs = [];
  for (let i = 0; i < colliders.length; i++) for (let j = i + 1; j < colliders.length; j++) {
    if (c.physicsHooks.filterContactPair(colliders[i].handle, colliders[j].handle) === null) {
      excludedColliderPairs.push([rawParts(colliders[i].handle), rawParts(colliders[j].handle)]);
    }
  }
  const bodiesBeforeStep = state(c), bytes = c.world.takeSnapshot();
  assert.deepEqual(state(c), bodiesBeforeStep, 'Snapshot export changed a physical body');
  const snapshotFile = `${stem}.bin`, metadataFile = `${stem}.metadata.json`;
  writeFileSync(join(output, snapshotFile), bytes, { flag: 'wx' });
  const metadata = { ...context, steps: 1, snapshotSha256: sha256(bytes), bodiesBeforeStep,
    bodyHandles: Object.fromEntries([...c.ragdollBodies].map(([id, body]) => [id, rawParts(body.handle)])), excludedColliderPairs };
  const metadataBytes = JSON.stringify(metadata, null, 2) + '\n';
  writeFileSync(join(output, metadataFile), metadataBytes, { flag: 'wx' });
  return { snapshotFile, snapshotSha256: sha256(bytes), metadataFile, metadataSha256: sha256(metadataBytes) };
}
const coordinates = (c, id) => {
  const definition = SEGMENT_BY_ID.get(id), parent = c.ragdollBodies.get(definition.parent), child = c.ragdollBodies.get(id);
  return jointCoordinates(parent.rotation(), child.rotation(), definition.jointProfile);
};
const kinematics = (c, id) => {
  const definition = SEGMENT_BY_ID.get(id), parent = c.ragdollBodies.get(definition.parent), child = c.ragdollBodies.get(id);
  return jointCoordinateKinematics(parent.angvel(), child.angvel(), parent.rotation(), coordinates(c, id), definition.jointProfile);
};
const limits = c => [...SEGMENT_BY_ID.values()].filter(d => d.jointProfile).flatMap(d => {
  const q = coordinates(c, d.id);
  return d.jointProfile.axes.map(axis => ({ id: d.id, coordinate: axis.coordinate, value: q[axis.coordinate],
    lower: axis.minRadians, upper: axis.maxRadians, margin: Math.min(q[axis.coordinate] - axis.minRadians, axis.maxRadians - q[axis.coordinate]) }));
});
const results = [];
try {
  for (let repeat = 0; repeat < repetitions; repeat++) for (const originOffsetX of originOffsets) for (const heading of headings) for (const sourceId of sources) {
    for (const magnitude of magnitudes) for (const sign of [-1, 1]) {
      const driven = await createEmbodiedCharacter('canvas2d', { heading });
      const control = await createEmbodiedCharacter('canvas2d', { heading });
      try {
        const definition = SEGMENT_BY_ID.get(sourceId);
        for (const c of [driven, control]) {
          for (let tick = 0; tick < 120; tick++) c.fixedUpdate(dt, null);
          c.nativeMotors.disable(c.jointsByChild);
          c.ragdollColliders.get('rightFoot').setCollisionGroups(0);
          c.ragdollColliders.get('rightForefoot').setCollisionGroups(0);
          for (const body of c.ragdollBodies.values()) body.wakeUp();
          c.world.step(c.eventQueue, c.physicsHooks);
          if (sourceId === 'rightThigh') {
            const axis = kinematics(c, sourceId).torqueAxesWorld.x;
            c.ragdollBodies.get(sourceId).applyTorqueImpulse(scale(axis, .05), true);
            c.ragdollBodies.get(definition.parent).applyTorqueImpulse(scale(axis, -.05), true);
            for (let tick = 0; tick < 3; tick++) c.world.step(c.eventQueue, c.physicsHooks);
            const lower = definition.jointProfile.axes.find(axis => axis.coordinate === 'x').minRadians;
            assert.ok(coordinates(c, sourceId).x > lower + .02, 'Hip fixture must clear lower limit');
            for (const body of c.ragdollBodies.values()) { body.setLinvel(zero, true); body.setAngvel(zero, true); }
          }
        }
        driven.observeContacts(dt);
        const untranslatedContacts = structuredClone(driven.lastContacts);
        const untranslatedBefore = state(driven);
        if (originOffsetX !== 0) for (const c of [driven, control]) {
          // Isolated coordinate-origin diagnostic, never a standing/production
          // action. Shift the entire assembly and surface by the same vector.
          for (const body of c.ragdollBodies.values()) {
            const position = body.translation();
            body.setTranslation({ ...position, x: position.x + originOffsetX }, true);
          }
          const floor = c.floorCollider.translation();
          c.floorCollider.setTranslation({ ...floor, x: floor.x + originOffsetX });
          c.world.propagateModifiedBodyPositionsToColliders();
        }
        const before = state(driven);
        const translationRoundoffM = Math.max(...before.map((body, index) => Math.abs(
          body.position.x - originOffsetX - untranslatedBefore[index].position.x)));
        for (let index = 0; index < before.length; index++) {
          for (const key of ['rotation', 'linearVelocity', 'angularVelocity']) assert.deepEqual(before[index][key], untranslatedBefore[index][key]);
          assert.equal(before[index].position.y, untranslatedBefore[index].position.y);
          assert.equal(before[index].position.z, untranslatedBefore[index].position.z);
        }
        assert.deepEqual(before, state(control), 'Paired initial physical states must match exactly');
        // Rapier's cached solver witnesses do not move until the next step.
        // Map the just-measured contacts through the same rigid translation for
        // the analytic comparison only; never inject them into physics.
        const shiftPoint = point => ({ ...point, x: point.x + originOffsetX });
        const contactsBefore = untranslatedContacts.map(contact => ({ ...contact,
          point: shiftPoint(contact.point),
          ...(contact.points ? { points: contact.points.map(shiftPoint) } : {}),
          ...(contact.measuredPressurePoint ? { measuredPressurePoint: shiftPoint(contact.measuredPressurePoint) } : {}) }));
        const supports = contactsBefore.filter(contact => contact.loadBearing && SEGMENT_BY_ID.get(contact.segment).side === 'left'
          && ['hindfoot', 'forefoot'].includes(SEGMENT_BY_ID.get(contact.segment).role))
          .map(contact => ({ segment: contact.segment, points: contact.points?.length ? contact.points : [contact.point],
            directions: [{ x: 1, y: 0, z: 0 }, { x: 0, y: 1, z: 0 }, { x: 0, y: 0, z: 1 }] }));
        assert.ok(supports.length, 'Measured retained contact is required');
        const response = support => articulatedCoordinateResponse(driven.ragdollBodies, support).get(sourceId, 'x', sourceId, 'x');
        const predicted = response(supports), predictions = { stickingPatches: predicted, free: response([]),
          normalOnly: response(supports.map(support => ({ ...support, directions: [{ x: 0, y: 1, z: 0 }] }))),
          hindfootOnly: response(supports.filter(support => support.segment === 'leftFoot')) };
        const axis = kinematics(driven, sourceId).torqueAxesWorld.x, beforeLimits = limits(driven);
        const impulse = sign * magnitude;
        driven.ragdollBodies.get(sourceId).applyTorqueImpulse(scale(axis, impulse), true);
        driven.ragdollBodies.get(definition.parent).applyTorqueImpulse(scale(axis, -impulse), true);
        const snapshots = exportSnapshots ? Object.fromEntries([['driven', driven], ['control', control]].map(([name, c]) =>
          [name, saveNativeSnapshot(c, `${results.length}-${sourceId}-${name}`, { heading, sourceId, impulseNmS: impulse, predicted, axis })])) : undefined;
        if (freshPipeline) for (const c of [driven, control]) {
          const Pipeline = c.world.physicsPipeline.constructor;
          c.world.physicsPipeline.free();
          c.world.physicsPipeline = new Pipeline();
        }
        driven.world.step(driven.eventQueue, driven.physicsHooks);
        control.world.step(control.eventQueue, control.physicsHooks);
        const rate = c => kinematics(c, sourceId).rates.x;
        const frozenRate = c => dot(sub(c.ragdollBodies.get(sourceId).angvel(), c.ragdollBodies.get(definition.parent).angvel()), axis);
        const actual = (rate(driven) - rate(control)) / impulse;
        const relativeError = Math.abs(predicted - actual) / Math.abs(actual);
        driven.observeContacts(dt); control.observeContacts(dt);
        const row = { repeat, originOffsetX, translationRoundoffM, heading, sourceId, impulseNmS: impulse, predictions, actual, relativeError,
          originalTolerance: .1, passed: relativeError < .1, frozenFrameResponse: (frozenRate(driven) - frozenRate(control)) / impulse,
          rates: { driven: rate(driven), control: rate(control) }, axis, supports, before, beforeLimits,
          contactsBefore, driven: { bodies: state(driven), contacts: driven.lastContacts, limits: limits(driven) },
          control: { bodies: state(control), contacts: control.lastContacts, limits: limits(control) }, ...(snapshots ? { snapshots } : {}) };
        const file = `${results.length}-${sourceId}.json`, bytes = JSON.stringify(row) + '\n';
        writeFileSync(join(output, file), bytes, { flag: 'wx' });
        const summary = { repeat, originOffsetX, translationRoundoffM, heading, sourceId, impulseNmS: impulse, predicted, actual, relativeError, passed: row.passed,
          beforeSha256: sha256(JSON.stringify(before)), evidence: { file, sha256: sha256(bytes) } };
        results.push(summary); console.log(JSON.stringify(summary));
      } finally { driven.dispose(); control.dispose(); }
    }
  }
  const sourceAfter = fingerprints(); assert.deepEqual(sourceAfter, sourceBefore, 'Source changed during probe');
  writeFileSync(join(output, 'report.json'), JSON.stringify({ started, finished: new Date().toISOString(),
    command: [process.execPath, ...process.argv.slice(1)], nodeVersion: process.version,
    parameters: { headings, sources, magnitudes, repetitions, originOffsets, timestep: dt, exportSnapshots, freshPipeline }, sourceBefore, sourceAfter,
    qualification: 'New paired diagnostic, modeled on the unchanged focused fixture; not a recovered historical 36-case artifact. Fixture-only velocity clearing and swing-sole collision disabling are never production actions.',
    results, failures: results.filter(result => !result.passed).length }, null, 2) + '\n', { flag: 'wx' });
  process.exitCode = results.some(result => !result.passed) ? 1 : 0;
} finally { unregister(); }
