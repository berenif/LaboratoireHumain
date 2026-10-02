import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';

const unregister = register();
after(unregister);
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { characterFrame } = await import('../src/character/character-frame.ts');
const { coordinatedStandingOptions } = await import('../src/character/standing-selection.ts');
const { CoordinatedStandingController } = await import('../src/character/CoordinatedStandingController.ts');
const { H75_MOTOR_ROUNDING_TOLERANCE_NM, ImplicitStandingController } = await import('../src/character/ImplicitStandingController.ts');
const { SEGMENT_BY_ID } = await import('../src/core/humanoid.ts');
const { dot } = await import('../src/character/math.ts');
const { jointFrameAxesWorld } = await import('../src/character/joint-coordinates.ts');

async function sampleInput() {
  const character = await createEmbodiedCharacter('canvas2d', coordinatedStandingOptions('h77-v1'));
  const poses = new Map([...character.poses].map(([id, pose]) => [id, structuredClone(pose)]));
  let input;
  character.coordinatedStanding.update = value => { input = { ...value, poses }; return null; };
  characterFrame(character, 1 / 60, null, 'canvas2d');
  return { character, input, poses };
}

test('H77 preserves H74 current-state requests while configuring native kp, kd and ceilings', async () => {
  const { character, input, poses } = await sampleInput();
  const actuator = await createEmbodiedCharacter('canvas2d', coordinatedStandingOptions('legacy'));
  try {
    const h74 = new CoordinatedStandingController(poses, 0, 0, () => 0);
    const h77 = new ImplicitStandingController(poses, 0, 0, () => 0);
    const h74Commands = h74.update(input), h77Commands = h77.update(input);
    assert.equal(h74Commands.length, 24);
    assert.equal(h77Commands.length, 24);
    const h74Results = actuator.nativeMotors.apply(actuator.ragdollBodies, actuator.jointsByChild, h74Commands);
    const h77Results = actuator.nativeMotors.apply(actuator.ragdollBodies, actuator.jointsByChild, h77Commands);
    h77.observeNativeResults(h77Results);
    const telemetry = h77.diagnostics();
    assert.equal(telemetry.nativeReadback.axes, 46);
    assert.equal(telemetry.nativeReadback.equivalent, true);
    assert.ok(telemetry.nativeReadback.maximumRequestErrorNm <= H75_MOTOR_ROUNDING_TOLERANCE_NM);
    assert.equal(telemetry.headroom, telemetry.requestedHeadroom);
    assert.ok(telemetry.residualFeedforward > 0);
    for (const command of h77Commands) {
      const nominal = input.nominal.find(value => value.id === command.id);
      assert.equal(command.stiffness, nominal.stiffness);
      assert.equal(command.damping, nominal.damping);
      const definition = SEGMENT_BY_ID.get(command.id), parent = poses.get(definition.parent);
      const basis = jointFrameAxesWorld(parent.rotation, definition.jointProfile);
      const h74Result = h74Results.get(command.id), h77Result = h77Results.get(command.id);
      for (const axis of definition.jointProfile.axes) {
        const requested = command.standingRequestByAxis[axis.coordinate];
        assert.ok(Math.abs(dot(h74Result.torqueWorld, basis[axis.coordinate]) - requested) < 1e-9,
          `${command.id}.${axis.coordinate}: H74/H77 request`);
        const readback = h77Result.nativeMotorAxes.find(value => value.coordinate === axis.coordinate);
        assert.equal(readback.stiffness, Math.fround(nominal.stiffness));
        assert.equal(readback.damping, Math.fround(nominal.damping));
        assert.equal(readback.maxForce, Math.fround(axis.maxMotorTorqueNm));
        // With the same feedforward, a coordinate/rate change inside a native
        // substep changes the force-based request without another JS update.
        const perturbed = readback.currentStateRequestNm - readback.stiffness * 0.001 - readback.damping * 0.002;
        assert.notEqual(perturbed, readback.currentStateRequestNm);
      }
    }
  } finally { character.dispose(); actuator.dispose(); }
});

test('H77 suspends for a step and replaces its stance only after qualified persistent touchdown', async () => {
  const { character, input, poses } = await sampleInput();
  try {
    const controller = new ImplicitStandingController(poses, 0, 0, () => 0);
    const originalReference = controller.reference;
    assert.equal(controller.update({ ...input, quiet: false, stepActive: true,
      triggerReason: 'capture' }), null);
    assert.equal(controller.diagnostics().authority, 'suspended');
    assert.equal(controller.active, false);
    const shifted = new Map([...poses].map(([id, pose]) => [id, structuredClone(pose)]));
    shifted.get('rightFoot').position.x += 0.02;
    const contacts = ['leftFoot', 'rightFoot'].map(segment => ({ segment, loadBearing: true,
      forceN: 300, normalY: 1, persistenceS: 0.10 }));
    assert.equal(controller.update({ ...input, poses: shifted, contacts, quiet: true,
      stepActive: false, touchdownQualified: false, supportMarginM: 0.02 }), null);
    assert.equal(controller.diagnostics().authority, 'reacquiring');
    const commands = controller.update({ ...input, poses: shifted, contacts, quiet: true,
      stepActive: false, touchdownQualified: true, supportMarginM: 0.02 });
    assert.equal(commands.length, 24);
    assert.equal(controller.diagnostics().authority, 'standing');
    assert.equal(controller.diagnostics().stanceRevision, 1);
    assert.equal(controller.diagnostics().triggerReason, 'capture');
    assert.notEqual(controller.reference, originalReference);
    assert.equal(originalReference.get('rightFoot').position.x, poses.get('rightFoot').position.x);
  } finally { character.dispose(); }
});

test('selector supports frozen H74, opt-in H77 and explicit legacy rollback', async (t) => {
  assert.equal(coordinatedStandingOptions('h74-v1').standingCandidate.controllerId, 'h74-v1');
  assert.equal(coordinatedStandingOptions('h77-v1').standingCandidate.controllerId, 'h77-v1');
  assert.equal(coordinatedStandingOptions('legacy').standingCandidate.controllerId, 'legacy');
  assert.throws(() => coordinatedStandingOptions('h81-v1'));
  const legacy = await createEmbodiedCharacter('canvas2d', coordinatedStandingOptions('legacy'));
  const h77 = await createEmbodiedCharacter('canvas2d', coordinatedStandingOptions('h77-v1'));
  // This checks selection and lifecycle under concurrent unit-test load.
  // Deadline behavior is checked separately with a controlled clock; the
  // actual-browser acceptance suite retains the real wall-clock budget.
  t.mock.method(h77.coordinatedStanding, 'clock', () => 0);
  try {
    assert.equal(legacy.getSnapshot('canvas2d').diagnostics.coordinatedStanding, undefined);
    for (let tick = 0; tick < 60; tick++) characterFrame(h77, 1 / 60, null, 'canvas2d');
    const snapshot = h77.getSnapshot('canvas2d');
    assert.equal(snapshot.diagnostics.coordinatedStanding.authority, 'standing');
    assert.equal(snapshot.diagnostics.stepCount, 0);
    assert.equal(snapshot.diagnostics.coordinatedStanding.stanceRevision, 0);
  } finally { legacy.dispose(); h77.dispose(); }
});

test('H77 admits the 8 ms boundary and gives up authority immediately beyond it', async () => {
  const { character, input, poses } = await sampleInput();
  try {
    for (const elapsedMs of [8, 8.001]) {
      let reads = 0;
      const controller = new ImplicitStandingController(poses, 0, 0, () => reads++ === 0 ? 0 : elapsedMs);
      const commands = controller.update(input);
      const diagnostics = controller.diagnostics();
      assert.equal(diagnostics.solveMs, elapsedMs);
      if (elapsedMs === 8) {
        assert.equal(commands.length, 24);
        assert.equal(diagnostics.authority, 'standing');
        assert.equal(diagnostics.reason, null);
      } else {
        assert.equal(commands, null);
        assert.equal(diagnostics.authority, 'transition');
        assert.equal(diagnostics.reason, 'timeout');
        assert.equal(controller.update(input), null, 'expired authority cannot restart on the next tick');
      }
    }
  } finally { character.dispose(); }
});
