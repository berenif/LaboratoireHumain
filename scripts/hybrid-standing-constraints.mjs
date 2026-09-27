import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import RAPIER from '@dimforge/rapier3d-compat';
import { sha256 } from './capture-physics-baseline.mjs';

// Diagnostic initialization only. Keep the active World, bodies and colliders;
// transfer only the two native joint sets, then rebind impulse-joint wrappers.
export function installHybridStandingConstraints(character, output, index, variant = 'translations') {
  assert.ok(['translations', 'hinges'].includes(variant));
  const world = character.world;
  const beforeBodies = new Map(character.ragdollBodies), beforeColliders = new Map(character.ragdollColliders);
  const physical = () => [...beforeBodies].map(([id, body]) => ({ id, position: body.translation(),
    rotation: body.rotation(), linearVelocity: body.linvel(), angularVelocity: body.angvel(), mass: body.mass() }));
  const before = physical(), handles = [...character.jointsByChild].map(([id, joint]) => [id, joint.handle]);
  const executable = resolve(`evidence/rapier-calibration-f32-target/release/${variant === 'hinges' ? 'hinge_constraints' : 'hybrid_constraints'}.exe`);
  const executableSha256 = sha256(readFileSync(executable));
  const input = join(output, `${index}-hybrid-input.bin`), converted = join(output, `${index}-hybrid-output.bin`);
  const receipt = join(output, `${index}-hybrid-native.json`), snapshot = world.takeSnapshot();
  writeFileSync(input, snapshot, { flag: 'wx' });
  const command = [executable, input, converted, receipt];
  const result = spawnSync(executable, command.slice(1), { encoding: 'utf8', windowsHide: true, timeout: 60000 });
  const invocation = { variant, command, exitCode: result.status, signal: result.signal, stdout: result.stdout, stderr: result.stderr,
    inputSha256: sha256(snapshot), executableSha256 };
  writeFileSync(join(output, `${index}-hybrid-command.json`), JSON.stringify(invocation, null, 2) + '\n', { flag: 'wx' });
  assert.equal(result.status, 0, JSON.stringify(invocation));
  const nativeReceipt = JSON.parse(readFileSync(receipt, 'utf8'));
  const convertedBytes = readFileSync(converted), restored = RAPIER.World.restoreSnapshot(convertedBytes);
  try {
    assert.equal(restored.impulseJoints.len(), 24);
    assert.equal(restored.multibodyJoints.len(), variant === 'translations' ? 24 : nativeReceipt.joints.length);
    [world.impulseJoints, restored.impulseJoints] = [restored.impulseJoints, world.impulseJoints];
    [world.multibodyJoints, restored.multibodyJoints] = [restored.multibodyJoints, world.multibodyJoints];
    world.impulseJoints.finalizeDeserialization(world.bodies);
    character.jointsByChild.clear();
    character.ragdollJoints.length = 0;
    for (const [id, handle] of handles) {
      const joint = world.impulseJoints.get(handle);
      assert.ok(joint?.isValid());
      character.jointsByChild.set(id, joint); character.ragdollJoints.push(joint);
    }
    const NativeMotors = character.nativeMotors.constructor;
    character.nativeMotors = new NativeMotors(world);
  } finally { restored.free(); }
  assert.deepEqual(physical(), before, 'Joint conversion moved a body');
  assert.equal(character.world, world);
  for (const [id, body] of beforeBodies) assert.equal(character.ragdollBodies.get(id), body);
  for (const [id, collider] of beforeColliders) assert.equal(character.ragdollColliders.get(id), collider);
  assert.equal(sha256(readFileSync(executable)), executableSha256);
  return { ...invocation, convertedSha256: sha256(convertedBytes), nativeReceiptSha256: sha256(readFileSync(receipt)),
    retainedBodyObjects: beforeBodies.size, retainedColliderObjects: beforeColliders.size,
    originalImpulseJoints: world.impulseJoints.len(),
    ...(variant === 'translations' ? { addedSphericalMultibodyLinks: world.multibodyJoints.len() }
      : { addedHingeMultibodyLinks: world.multibodyJoints.len(), nativeAssemblies: nativeReceipt.assemblies }) };
}
