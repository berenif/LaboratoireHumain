import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
const unregister = register(); after(unregister);
const { solveRecoveryMotorTorques } = await import("../src/character/recovery-motors.ts");
const zero = { x: 0, y: 0, z: 0 };
const diagonal = (x = 1, y = x, z = x) => ({ m11: x, m12: 0, m13: 0, m22: y, m23: 0, m33: z });
const close = (actual, expected, tolerance = 1e-10) => assert.ok(Math.abs(actual - expected) < tolerance, `${actual} differs from ${expected}`);
const motor = (id, parent, extra = {}) => ({ id, parent, error: zero, velocity: zero, kp: 150, kd: 6, feedforward: zero, cap: 110, ...extra });

test("single motor equals the two-body implicit solution on all axes", () => {
  const dt = 1 / 60, kp = 200, kd = 7;
  const intent = motor("leftShin", "leftThigh", { kp, kd, error: { x: .2, y: -.1, z: .3 }, velocity: { x: -.2, y: .4, z: .1 }, feedforward: { x: 2, y: -3, z: 4 } });
  const inverse = new Map([["leftShin", diagonal(3, 4, 5)], ["leftThigh", diagonal(2, 1, 3)]]);
  const result = solveRecoveryMotorTorques([intent], inverse, dt, { feedforwardMode: "command" }).get(intent.id);
  for (const [axis, inv] of [["x", 5], ["y", 5], ["z", 8]]) close(result[axis], (kp * intent.error[axis] - kd * intent.velocity[axis] + intent.feedforward[axis]) / (1 + (kd * dt + kp * dt * dt) * inv));
});

test("shared-body coupling matches an independent two-joint closed-form solution", () => {
  const dt = 1 / 60;
  const intents = [motor("leftThigh", "pelvis", { kp: 180, kd: 5, error: { ...zero, x: .2 }, velocity: { ...zero, x: -.1 } }), motor("leftShin", "leftThigh", { kp: 90, kd: 8, error: { ...zero, x: -.1 }, velocity: { ...zero, x: .3 } })];
  const inverse = new Map([["pelvis", diagonal(1)], ["leftThigh", diagonal(2)], ["leftShin", diagonal(4)]]);
  const [j, k] = intents, kj = j.kd * dt + j.kp * dt * dt, kk = k.kd * dt + k.kp * dt * dt;
  const a = 1 + kj * 3, b = -kj * 2, c = -kk * 2, d = 1 + kk * 6;
  const r = j.kp * j.error.x - j.kd * j.velocity.x, s = k.kp * k.error.x - k.kd * k.velocity.x;
  const result = solveRecoveryMotorTorques(intents, inverse, dt);
  close(result.get(j.id).x, (d * r - b * s) / (a * d - b * c));
  close(result.get(k.id).x, (a * s - c * r) / (a * d - b * c));
  const reordered = solveRecoveryMotorTorques([...intents].reverse(), inverse, dt);
  for (const intent of intents) close(reordered.get(intent.id).x, result.get(intent.id).x);
});

test("simultaneous chain damping dissipates energy and preserves angular momentum", () => {
  const inverse = new Map([["pelvis", diagonal(1)], ["leftThigh", diagonal(2)], ["leftShin", diagonal(4)]]);
  const velocity = new Map([["pelvis", 1], ["leftThigh", -2], ["leftShin", .5]]), dt = 1 / 60;
  const energy = () => [...velocity].reduce((sum, [id, value]) => sum + .5 * value * value / inverse.get(id).m11, 0);
  const momentum = () => [...velocity].reduce((sum, [id, value]) => sum + value / inverse.get(id).m11, 0);
  const initialMomentum = momentum(); let previousEnergy = energy();
  for (let step = 0; step < 240; step++) {
    const intents = [motor("leftThigh", "pelvis", { kp: 0, kd: 8, velocity: { ...zero, x: velocity.get("leftThigh") - velocity.get("pelvis") } }), motor("leftShin", "leftThigh", { kp: 0, kd: 8, velocity: { ...zero, x: velocity.get("leftShin") - velocity.get("leftThigh") } })];
    const torques = solveRecoveryMotorTorques(intents, inverse, dt), applied = new Map([...velocity.keys()].map(id => [id, 0]));
    for (const intent of intents) { applied.set(intent.id, applied.get(intent.id) + torques.get(intent.id).x); applied.set(intent.parent, applied.get(intent.parent) - torques.get(intent.id).x); }
    close([...applied.values()].reduce((sum, value) => sum + value, 0), 0);
    for (const id of velocity.keys()) velocity.set(id, velocity.get(id) + inverse.get(id).m11 * applied.get(id) * dt);
    assert.ok(energy() <= previousEnergy + 1e-12); previousEnergy = energy(); close(momentum(), initialMomentum);
  }
  close(velocity.get("pelvis"), velocity.get("leftThigh"), 1e-8); close(velocity.get("leftThigh"), velocity.get("leftShin"), 1e-8);
});

test("equilibrium feedforward preserves a static load while command feedforward predicts acceleration", () => {
  const inverse = new Map([["pelvis", diagonal(1)], ["leftThigh", diagonal(2)], ["leftShin", diagonal(4)]]);
  const intents = [motor("leftThigh", "pelvis", { feedforward: { x: 25, y: -3, z: 4 } }), motor("leftShin", "leftThigh", { feedforward: { x: -12, y: 4, z: 2 } })];
  const equilibrium = solveRecoveryMotorTorques(intents, inverse, 1 / 60);
  for (const intent of intents) for (const axis of ["x", "y", "z"]) close(equilibrium.get(intent.id)[axis], intent.feedforward[axis]);
  const commanded = solveRecoveryMotorTorques(intents, inverse, 1 / 60, { feedforwardMode: "command" });
  assert.ok(commanded.get("leftThigh").x < 25); assert.ok(commanded.get("leftShin").x > -12);
});

test("anisotropic world inertia couples axes and motor caps bound the final combined torque", () => {
  const inverse = new Map([["pelvis", diagonal(0)], ["leftThigh", { m11: 2, m12: .5, m13: 0, m22: 1, m23: 0, m33: 3 }]]);
  const intent = motor("leftThigh", "pelvis", { kp: 100, kd: 4, error: { x: .2, y: -.1, z: 0 } }), dt = 1 / 60;
  const k = intent.kd * dt + intent.kp * dt * dt, a = 1 + 2 * k, b = .5 * k, d = 1 + k;
  const result = solveRecoveryMotorTorques([intent], inverse, dt).get(intent.id);
  close(result.x, (20 * d + 10 * b) / (a * d - b * b)); close(result.y, (-10 * a - 20 * b) / (a * d - b * b));
  const capped = solveRecoveryMotorTorques([{ ...intent, error: zero, feedforward: { x: 200, y: -300, z: 400 }, cap: 30 }], inverse, dt).get(intent.id);
  close(Math.hypot(capped.x, capped.y, capped.z), 30, 1e-7);
  const delta={x:capped.x-200,y:capped.y+300,z:capped.z-400};
  const gradient={x:(2+1/k)*delta.x+.5*delta.y,y:.5*delta.x+(1+1/k)*delta.y,z:(3+1/k)*delta.z};
  const multiplier=-(gradient.x*capped.x+gradient.y*capped.y+gradient.z*capped.z)/(30*30);
  assert.ok(multiplier>0);
  for(const axis of ["x","y","z"]) close(gradient[axis]+multiplier*capped[axis],0,1e-5);
});

test("zero-gain command motors still affect neighboring feedback motors", () => {
  const inverse = new Map([["pelvis", diagonal(1)], ["leftThigh", diagonal(1)], ["leftShin", diagonal(1)]]);
  const command = motor("leftThigh", "pelvis", { kp: 0, kd: 0, feedforward: { ...zero, x: 10 } });
  const follower = motor("leftShin", "leftThigh", { kp: 0, kd: 6 });
  const result = solveRecoveryMotorTorques([command, follower], inverse, 1 / 60, { feedforwardMode: "command" });
  close(result.get(command.id).x, 10); close(result.get(follower.id).x, 1 / 1.2);
  assert.equal(solveRecoveryMotorTorques([], new Map(), 1 / 60).size, 0);
});

test("saturation is propagated into a free neighboring joint and satisfies its stationarity equation", () => {
  const dt=1/60, inverse=new Map([["pelvis",diagonal(1)],["leftThigh",diagonal(2)],["leftShin",diagonal(4)]]);
  const a=motor("leftThigh","pelvis",{kp:600,kd:10,error:{...zero,x:1},cap:2});
  const b=motor("leftShin","leftThigh",{kp:60,kd:4,error:{...zero,x:.1}});
  const ka=a.kd*dt+a.kp*dt*dt,kb=b.kd*dt+b.kp*dt*dt;
  const h11=3+1/ka,h22=6+1/kb,r1=600/ka,r2=6/kb;
  const bounded=solveRecoveryMotorTorques([a,b],inverse,dt);
  const x=bounded.get(a.id).x,y=bounded.get(b.id).x;
  close(x,2,1e-7);close(y,(r2+2*x)/h22,1e-7);
  close(h22*y-2*x-r2,0,1e-6);
  assert.ok(h11*x-2*y-r1<0,"capped joint has an outward desired torque");
  const freeY=(h11*r2+2*r1)/(h11*h22-4);
  assert.ok(Math.abs(h22*freeY-2*2-r2)>100,"post-clamping would leave a large neighbor residual");
  const reversed=solveRecoveryMotorTorques([b,a],inverse,dt);
  close(reversed.get(a.id).x,x,1e-7);close(reversed.get(b.id).x,y,1e-7);
});

test("equilibrium feedforward caps participate in the coupled feasible solution", () => {
  const inverse=new Map([["pelvis",diagonal(1)],["leftThigh",diagonal(1)],["leftShin",diagonal(1)]]),dt=1/60;
  const a=motor("leftThigh","pelvis",{kp:0,kd:6,feedforward:{...zero,x:100},cap:5});
  const b=motor("leftShin","leftThigh",{kp:0,kd:6,feedforward:{...zero,x:0},cap:110});
  const bounded=solveRecoveryMotorTorques([a,b],inverse,dt);
  close(bounded.get(a.id).x,5,1e-7);
  close(bounded.get(b.id).x,(5-100)/12,1e-7);
  const zeroGain=solveRecoveryMotorTorques([{...a,kd:0},b],inverse,dt);
  close(zeroGain.get(a.id).x,5,1e-7);close(zeroGain.get(b.id).x,(5-100)/12,1e-7);
});