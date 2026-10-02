import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { register } from 'tsx/esm/api';
import { sha256 } from './capture-physics-baseline.mjs';
import { captureBodies, collisionContext } from './coordinated-standing-evidence.mjs';

const [outputArg, controllerId = 'h77-v1', referenceArg = '0', tickArg = '332'] = process.argv.slice(2);
assert.ok(outputArg, 'capture-standing-native-state.mjs output-directory [controller-id] [reference] [tick]');
const output = resolve(outputArg);
assert.ok(!existsSync(output), `Use a fresh capture directory: ${output}`);
mkdirSync(output, { recursive: true });
const referenceIndex = Number(referenceArg);
const captureTick = Number(tickArg);
assert.ok([0, 1, 2].includes(referenceIndex), `Unknown reference ${referenceArg}`);
assert.ok(Number.isInteger(captureTick) && captureTick > 0, `Invalid capture tick ${tickArg}`);

const scenario = JSON.parse(readFileSync(resolve(
  `evidence/standing-h75-v1/attempt-01/reference-${referenceIndex}/scenario.json`)));
assert.ok(captureTick <= scenario.steps, 'Capture tick is beyond the preserved scenario');
const unregister = register();
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { coordinatedStandingOptions } = await import('../src/character/standing-selection.ts');
const character = await createEmbodiedCharacter('canvas2d', {
  heading: scenario.heading,
  ...coordinatedStandingOptions(controllerId, scenario.offset),
});
const handles = [...character.ragdollBodies].map(([id, body]) => [id, body.handle]);
const jointHandles = [...character.jointsByChild].map(([id, joint]) => [id, joint.handle]);
const collision = collisionContext(character);
const rawCalls = [];
for (const method of ['jointConfigureMotorModel', 'jointSetMotorMaxForce', 'jointConfigureMotor']) {
  const native = character.world.impulseJoints.raw;
  const original = native[method].bind(native);
  native[method] = (...args) => {
    rawCalls.push({ method, args });
    return original(...args);
  };
}
let activeTick = 0;
let capture = null;
const originalStep = character.world.step.bind(character.world);
character.world.step = (...args) => {
  if (activeTick === captureTick && !capture) {
    const bytes = character.world.takeSnapshot();
    capture = {
      bytes,
      preBodies: captureBodies(character.world, handles),
      commands: structuredClone(character.standingCommands),
      nativeMotorResults: [...character.lastMotorResults],
      nativeMotorCalls: structuredClone(rawCalls),
      controller: character.coordinatedStanding?.serialize() ?? null,
    };
  }
  const result = originalStep(...args);
  if (activeTick === captureTick && capture && !capture.postBodies) {
    capture.postBodies = captureBodies(character.world, handles);
  }
  return result;
};

try {
  for (let tick = 1; tick <= captureTick; tick++) {
    activeTick = tick;
    rawCalls.length = 0;
    character.fixedUpdate(1 / 60, null);
  }
  assert.ok(capture?.postBodies, `No world step captured at tick ${captureTick}`);
  const binPath = join(output, 'pre-step.bin');
  writeFileSync(binPath, capture.bytes, { flag: 'wx' });
  const metadata = {
    schema: 1,
    id: 'standing-native-live-state',
    controllerId,
    reference: referenceIndex,
    tick: captureTick,
    scenario,
    handles,
    jointHandles,
    collision,
    nativeSnapshotSha256: sha256(capture.bytes),
    capturePhase: 'After H77/H74 motor configuration and immediately before the live world.step.',
    preBodies: capture.preBodies,
    postBodies: capture.postBodies,
    commands: capture.commands,
    nativeMotorResults: capture.nativeMotorResults,
    nativeMotorCalls: capture.nativeMotorCalls,
    controller: capture.controller,
    postStepSnapshot: character.getSnapshot('canvas2d'),
    qualification: 'Diagnostic live-state capture only; not standing acceptance.',
  };
  writeFileSync(join(output, 'capture.json'), `${JSON.stringify(metadata, null, 2)}\n`, { flag: 'wx' });
  console.log(JSON.stringify({ output, controllerId, reference: referenceIndex,
    tick: captureTick, nativeSnapshotSha256: metadata.nativeSnapshotSha256,
    nativeReadback: metadata.controller?.telemetry?.nativeReadback ?? null }));
} finally {
  character.dispose();
  unregister();
}
