import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { register } from 'tsx/esm/api';

const unregister = register();
after(unregister);
const { createEmbodiedCharacter } = await import('../src/character/index.ts');
const { SEGMENTS } = await import('../src/core/humanoid.ts');
const dt = 1 / 60;
const speed = v => Math.hypot(v.x, v.y, v.z);
const physicalState = c => [...c.ragdollBodies].map(([id, b]) => ({ id,
  position: { ...b.translation() }, rotation: { ...b.rotation() },
  linear: { ...b.linvel() }, angular: { ...b.angvel() } }));

test('a real strike and floor recovery do not inject pre-solver velocity kicks or launch the body', async () => {
  const c = await createEmbodiedCharacter('canvas2d', { room: true });
  const original = c.recovery.actuate.bind(c.recovery);
  let decisions = 0, floorTick = null, activeTicks = 0, peakPelvisY = 0, peakLinear = 0;
  const bodies = new Map(c.ragdollBodies);
  c.recovery.actuate = (...args) => {
    const before = physicalState(c);
    original(...args);
    assert.deepEqual(physicalState(c), before,
      'recovery must configure constrained motors; a frame-sized impulse can give a light segment thousands of rad/s before contact is solved');
    decisions++;
  };
  try {
    assert.equal(c.requestStrike(), true);
    for (let tick = 1; tick <= 600; tick++) {
      c.fixedUpdate(dt, null);
      assert.deepEqual(c.runtimeErrors, [], 'fixedUpdate must not swallow a failed actuation assertion');
      const d = c.recovery.diagnostics(), pelvis = c.ragdollBodies.get('pelvis');
      if (floorTick === null && pelvis.translation().y < .2
        && d.contacts.some(contact => contact.loadBearing && ['pelvis', 'torso'].includes(contact.segment))) floorTick = tick;
      if (floorTick !== null) {
        peakPelvisY = Math.max(peakPelvisY, pelvis.translation().y);
        peakLinear = Math.max(peakLinear, ...[...bodies.values()].map(body => speed(body.linvel())));
        if (c.state === 'recovering') activeTicks++;
        // A floor recovery may rise to ordinary standing height, but cannot
        // launch toward the 4.2 m ceiling. This is a launch regression, not
        // a replacement for the stricter recovery acceptance contract.
        assert.ok(peakPelvisY < 1.25, `uncommanded launch: pelvis ${peakPelvisY} m`);
        assert.ok(peakLinear < 20, `uncontrolled recovery speed: ${peakLinear} m/s`);
      }
    }
    assert.equal(c.protocolStrikes, 1);
    assert.ok(floorTick !== null && activeTicks > 60 && decisions > 120,
      'exercise a real landing and active recovery, not disabled motors or a frozen character');
    assert.ok([...c.lastMotorResults.values()].some(result => speed(result.torqueWorld) > .01));
    for (const [id, body] of bodies) {
      assert.equal(c.ragdollBodies.get(id), body);
      assert.ok(body.isDynamic());
    }
    console.log(JSON.stringify({ floorTick, activeTicks, peakPelvisY, peakLinear, recovered: c.protocolRecoveries }));
  } finally { c.dispose(); }
});

test('settling replaces active posture motors with bounded passive native resistance', async () => {
  const c = await createEmbodiedCharacter('canvas2d', { room: true });
  const raw = c.world.impulseJoints.raw;
  const configure = raw.jointConfigureMotor.bind(raw), setCap = raw.jointSetMotorMaxForce.bind(raw);
  const commands = [], caps = [];
  raw.jointConfigureMotor = (...args) => { commands.push(args); return configure(...args); };
  raw.jointSetMotorMaxForce = (...args) => { caps.push(args); return setCap(...args); };
  try {
    c.recovery.data.phase = 'settle';
    const before = physicalState(c);
    c.recovery.actuate(c.ragdollBodies, c.poses, false, dt);
    assert.deepEqual(physicalState(c), before);
    const profiles = SEGMENTS.filter(d => d.jointProfile);
    assert.equal(commands.length, profiles.reduce((sum, d) => sum + d.jointProfile.axes.length, 0));
    for (const d of profiles) for (const axis of d.jointProfile.axes) {
      const handle = c.jointsByChild.get(d.id).handle;
      const index = { x: 3, y: 4, z: 5 }[axis.coordinate];
      const command = commands.find(row => row[0] === handle && row[1] === index);
      const cap = caps.find(row => row[0] === handle && row[1] === index)[2];
      assert.ok(command[3] === 0, 'settling has no active feedforward velocity (either signed zero)');
      assert.equal(command[5], axis.dampingNmsPerRad);
      assert.ok(command[4] === 0 || command[4] === axis.passiveStiffnessNmPerRad);
      const zone = Math.max(1e-5, (axis.maxRadians - axis.minRadians) * d.jointProfile.limitSoftZoneFraction);
      assert.equal(cap, Math.max(axis.maxMotorTorqueNm * .5, axis.passiveStiffnessNmPerRad * zone * 2));
    }
    assert.ok([...c.recovery.motorResults().values()].every(result => result.torqueSource === 'native-request'));
  } finally { c.dispose(); }
});
