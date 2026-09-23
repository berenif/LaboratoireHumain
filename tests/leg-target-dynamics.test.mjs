import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { LegTargetDynamics, worldInertiaProduct, targetInertialMoment } = await import("../src/character/leg-target-dynamics.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const { restPoseMap } = await import("../src/character/pose.ts");
const { standingChainDiagnostics } = await import("../src/character/standing-chain-diagnostics.ts");
const { clampJointCoordinates, jointRotationFromCoordinates } = await import("../src/character/joint-coordinates.ts");
const { add, cross, quatFromAxisAngle, quatMultiply, rotate, scale, sub, worldPoint } = await import("../src/character/math.ts");
const zero = { x: 0, y: 0, z: 0 }, identity = { x: 0, y: 0, z: 0, w: 1 }, dt = 1 / 60;
const ids = side => ["Thigh", "Shin", "Ankle", "Foot", "Forefoot"].map(suffix => side + suffix);
const close = (actual, expected, tolerance = 1e-8) => {
  for (const axis of ["x", "y", "z"]) assert.ok(Math.abs(actual[axis] - expected[axis]) <= tolerance,
    `${axis}: ${actual[axis]} != ${expected[axis]}`);
};
const zeros = torques => { for (const torque of torques.values()) close(torque, zero); };
function fixture() {
  const bodies = new Map(), commands = [];
  for (const side of ["left", "right"]) for (const [index, id] of ids(side).entries()) {
    const localCom = { x: 0.015, y: -0.01, z: 0.025 }, inertia = { x: 0.02, y: 0.03, z: 0.04 };
    bodies.set(id, { mass: () => index + 1, localCom: () => localCom,
      principalInertia: () => inertia, principalInertiaLocalFrame: () => identity });
    commands.push({ id, targetLocalRotation: identity, stiffness: 1, damping: 1, strengthScale: 1 });
  }
  return { bodies, commands, pelvis: { position: { x: 0, y: 1, z: 0 }, rotation: identity } };
}
function commandedTargets(pelvis, commands, side) {
  const rest = restPoseMap();
  const chain = standingChainDiagnostics(rest, rest, commands, null, 0, 0, 0, [], { id: "pelvis", ...pelvis });
  return chain.legs.find(leg => leg.side === side).segments.map(segment => segment.commanded);
}

test("inertial translation produces the child torque sign about a displaced joint", () => {
  close(targetInertialMoment(zero, { centerOfMass: { x: 0, y: -2, z: 0 }, massKg: 3,
    linearAcceleration: { x: 4, y: 0, z: 0 }, angularVelocity: zero, angularAcceleration: zero,
    principalInertia: { x: 2, y: 3, z: 5 }, principalFrameWorld: identity }), { x: 0, y: 0, z: 24 });
});

test("principal-frame rotation and gyroscopic inertia contribute their analytic moments", () => {
  const principalInertia = { x: 2, y: 3, z: 5 };
  close(worldInertiaProduct(principalInertia, quatFromAxisAngle({ x: 0, y: 0, z: 1 }, Math.PI / 2),
    { x: 1, y: 2, z: 3 }), { x: 3, y: 4, z: 15 });
  close(targetInertialMoment(zero, { centerOfMass: zero, massKg: 1, linearAcceleration: zero,
    angularVelocity: { x: 1, y: 2, z: 3 }, angularAcceleration: { x: 0.4, y: -0.2, z: 0.1 },
    principalInertia, principalFrameWorld: identity }), { x: 12.8, y: -9.6, z: 2.5 });
});

test("two target intervals recover translated-base acceleration for every descendant subtree", () => {
  const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
  const acceleration = { x: 2, y: -0.4, z: 0.7 };
  let output, target;
  // Unequal intervals additionally check the derivative's actual sample timing.
  const times = [0, 0.01, 0.035];
  for (const [index, time] of times.entries()) {
    target = { ...pelvis, position: add(pelvis.position, scale(acceleration, 0.5 * time * time)) };
    output = dynamics.sample(target, commands, bodies, "left", index ? time - times[index - 1] : dt);
    if (index < 2) { assert.equal(output.size, 5); zeros(output); }
  }
  const poses = commandedTargets(target, commands, "left");
  for (const [index, pose] of poses.entries()) {
    const anchor = worldPoint(pose.position, pose.rotation, SEGMENT_BY_ID.get(pose.id).jointAnchorChild);
    let expected = zero;
    for (const descendant of poses.slice(index)) {
      const body = bodies.get(descendant.id), com = worldPoint(descendant.position, descendant.rotation, body.localCom());
      expected = add(expected, cross(sub(com, anchor), scale(acceleration, body.mass())));
    }
    close(output.get(pose.id), expected);
  }
});

test("angular acceleration uses the rotated local inertia and propagates to proximal joints", () => {
  const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
  const localCom = SEGMENT_BY_ID.get("leftForefoot").jointAnchorChild;
  bodies.set("leftForefoot", { mass: () => 1, localCom: () => localCom,
    principalInertia: () => ({ x: 2, y: 3, z: 5 }),
    principalInertiaLocalFrame: () => quatFromAxisAngle({ x: 0, y: 1, z: 0 }, Math.PI / 2) });
  let output;
  for (let sample = 0; sample < 3; sample++) {
    const time = sample * dt;
    const current = commands.map(command => command.id === "leftForefoot"
      ? { ...command, targetLocalRotation: quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.5 * 4 * time * time) } : command);
    output = dynamics.sample(pelvis, current, bodies, "left", dt);
  }
  for (const id of ids("left")) close(output.get(id), { x: 20, y: 0, z: 0 });
});

test("the final joint-limited local command chain determines COM lever arms", () => {
  const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
  const final = commands.map(command => {
    const profile = SEGMENT_BY_ID.get(command.id).jointProfile;
    const coordinates = clampJointCoordinates({ x: 99, y: -99, z: 99 }, profile);
    return { ...command, targetLocalRotation: jointRotationFromCoordinates(coordinates, profile) };
  });
  const acceleration = { x: 1, y: 0, z: 0 };
  let target, output;
  for (let sample = 0; sample < 3; sample++) {
    target = { ...pelvis, rotation: quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0.7),
      position: add(pelvis.position, scale(acceleration, 0.5 * (sample * dt) ** 2)) };
    output = dynamics.sample(target, final, bodies, "left", dt);
  }
  const poses = commandedTargets(target, final, "left");
  const thigh = poses[0], anchor = worldPoint(thigh.position, thigh.rotation, SEGMENT_BY_ID.get(thigh.id).jointAnchorChild);
  const expected = poses.reduce((sum, pose) => add(sum, cross(sub(
    worldPoint(pose.position, pose.rotation, bodies.get(pose.id).localCom()), anchor),
    scale(acceleration, bodies.get(pose.id).mass()))), zero);
  close(output.get("leftThigh"), expected);
  assert.ok(Math.hypot(expected.x, expected.y, expected.z) > 0.1);
});

test("world rotation and translation transform all sampled torque vectors covariantly", () => {
  const { bodies, commands, pelvis } = fixture(), first = new LegTargetDynamics(), second = new LegTargetDynamics();
  const yaw = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, 0.83), offset = { x: 2, y: -3, z: 1 };
  let a, b;
  for (let sample = 0; sample < 4; sample++) {
    const t = sample * dt;
    const pose = { position: add(pelvis.position, { x: t * t, y: 0, z: -0.3 * t * t }),
      rotation: quatFromAxisAngle({ x: 1, y: 0, z: 0 }, 0.7 * t * t) };
    a = first.sample(pose, commands, bodies, "right", dt);
    b = second.sample({ position: add(offset, rotate(yaw, pose.position)),
      rotation: quatMultiply(yaw, pose.rotation) }, commands, bodies, "right", dt);
  }
  for (const id of ids("right")) close(b.get(id), rotate(yaw, a.get(id)), 1e-7);
});

test("reset, release, side changes and invalid intervals discard stale target derivatives", () => {
  const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
  const sample = (side, displacement, interval = dt) => dynamics.sample(
    { ...pelvis, position: { ...pelvis.position, x: displacement } }, commands, bodies, side, interval);
  zeros(sample("left", 0)); zeros(sample("left", 0.01));
  assert.ok(Math.hypot(...Object.values(sample("left", 0.04).get("leftThigh"))) > 1);
  zeros(sample("right", 2)); zeros(sample("right", 3));
  assert.deepEqual([...sample("right", 5).keys()], ids("right"));
  assert.equal(sample(null, 0).size, 0);
  zeros(sample("right", 7)); zeros(sample("right", 8));
  dynamics.reset(); zeros(sample("right", 20)); zeros(sample("right", 21));
  assert.equal(sample("right", 22, 0).size, 0);
  zeros(sample("right", 30)); zeros(sample("right", 31));
  assert.equal(dynamics.sample(pelvis, commands.slice(0, 1), bodies, "left", dt).size, 0);
  zeros(sample("left", 40)); zeros(sample("left", 41));
});

test("stationary and constant-speed targets add no gravity or contact moment", () => {
  for (const speed of [0, 0.2]) {
    const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
    for (let sample = 0; sample < 5; sample++) zeros(dynamics.sample(
      { ...pelvis, position: { ...pelvis.position, x: speed * sample * dt } }, commands, bodies, "left", dt));
  }
});

test("target sampling reads frozen mass properties and commands without mutation or output aliasing", () => {
  const { bodies, commands, pelvis } = fixture(), dynamics = new LegTargetDynamics();
  function freeze(value) {
    if (value && typeof value === "object") { Object.values(value).forEach(freeze); Object.freeze(value); }
  }
  freeze(commands); freeze(pelvis);
  for (const body of bodies.values()) { freeze(body.localCom()); freeze(body.principalInertia()); freeze(body.principalInertiaLocalFrame()); freeze(body); }
  const before = JSON.stringify({ commands, pelvis });
  dynamics.sample(pelvis, commands, bodies, "left", dt);
  dynamics.sample(pelvis, commands, bodies, "left", dt);
  const result = dynamics.sample(pelvis, commands, bodies, "left", dt);
  result.get("leftThigh").x = 999;
  zeros(dynamics.sample(pelvis, commands, bodies, "left", dt));
  assert.equal(JSON.stringify({ commands, pelvis }), before);
});
