import assert from 'node:assert/strict';
import { appendFileSync, closeSync, copyFileSync, existsSync, mkdirSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { gzipSync, gunzipSync } from 'node:zlib';
import os from 'node:os';
import { fingerprints, sha256 } from './capture-physics-baseline.mjs';
import { captureBodies, inventory, json, norm, restoredHooks, verifyInventory } from './coordinated-standing-evidence.mjs';
import { SEGMENT_BY_ID } from '../src/core/humanoid.ts';
import { jointCoordinates, jointCoordinateKinematics, jointFrameAxesWorld, jointLimitErrorMagnitude } from '../src/character/joint-coordinates.ts';
import { angularVelocity, dot, quatInverse, quatMultiply, rotate, sub, worldPoint } from '../src/character/math.ts';

const manifestPath = 'docs/experiments/standing-h75-v1.json';
const m = JSON.parse(readFileSync(manifestPath));
const axes = ['x', 'y', 'z'];
const rawParts = handle => { const b = new DataView(new ArrayBuffer(8)); b.setFloat64(0, handle, true); return [b.getUint32(0, true), b.getUint32(4, true)]; };
const handleOf = parts => { const b = new DataView(new ArrayBuffer(8)); b.setUint32(0, parts[0], true); b.setUint32(4, parts[1], true); return b.getFloat64(0, true); };
const vector = v => Array.isArray(v) ? { x: v[0], y: v[1], z: v[2] } : v;
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);
const hashFile = p => sha256(readFileSync(p));
let activeRapier;
function inspectorBuild(directory) {
  if (!directory.endsWith('retained-inspector')) return JSON.parse(readFileSync(join(directory, 'build-report.json')));
  const prior = JSON.parse(readFileSync('evidence/standing-h74-v1/analysis-01/report.json'));
  return { status: 'pass', rowStatus: 'incomplete', executable: resolve('evidence/rapier-calibration-f32-target/release/snapshot_motors.exe'),
    executableSha256: prior.results[0].native.executableSha256,
    qualification: 'Previously verified motor inspector plus WASM frame/limit readback. Compiled raw/finalized row reconstruction is incomplete after three failed builds; independent motor probes use no new build.' };
}
function finite(v) { if (typeof v === 'number') assert.ok(Number.isFinite(v)); else if (v && typeof v === 'object') Object.values(v).forEach(finite); }
function verifyPrior() {
  const root = resolve(m.inputs.h74Root), receipts = [];
  for (const name of m.inputs.requiredInventories) {
    const dir = join(root, name), files = JSON.parse(readFileSync(join(dir, 'artifacts.json')));
    verifyInventory(dir, files); receipts.push({ name, files: Object.keys(files).length, inventorySha256: hashFile(join(dir, 'artifacts.json')) });
  }
  const run = JSON.parse(readFileSync(join(root, m.inputs.attempt, 'run-manifest.json')));
  for (const [path, hash] of Object.entries(run.source)) assert.equal(hashFile(path), hash, `Retained source changed: ${path}`);
  for (const [path, hash] of Object.entries(run.dependencies)) assert.equal(hashFile(path), hash, path);
  assert.equal(process.version, run.node); assert.equal(hashFile(process.execPath), run.nodeExecutableSha256);
  const saved = JSON.parse(gunzipSync(readFileSync(join(root, m.inputs.attempt, 'source-snapshot.json.gz'))));
  for (const [path, source] of Object.entries(saved)) assert.equal(sha256(source), run.source[path], path);
  const oldNative = JSON.parse(readFileSync('evidence/constraint-hinge-analysis-20260927-01/native-source.json'));
  for (const [path, source] of Object.entries(oldNative)) assert.equal(hashFile(path), sha256(source), path);
  return { receipts, sourceFiles: Object.keys(run.source).length, sourceSnapshotMembers: Object.keys(saved).length,
    dependencies: run.dependencies, node: process.version, nodeExecutableSha256: run.nodeExecutableSha256,
    nativeArchiveMembers: Object.keys(oldNative).length };
}
function nativeInspect(executable, input, output, deadline) {
  const start = performance.now();
  const result = spawnSync(executable, [input, output], { encoding: 'utf8', windowsHide: true,
    timeout: Math.max(1, Math.min(m.budget.nativeInspectorTimeoutS * 1000, deadline - Date.now())) });
  appendFileSync(join(resolve(output, '..'), 'native-commands.ndjson'), JSON.stringify({ command: [executable, input, output], exit: result.status,
    error: result.error?.message ?? null, stdout: result.stdout, stderr: result.stderr, wallMs: performance.now() - start }) + '\n');
  assert.equal(result.status, 0, result.stderr || result.error?.message);
  const resultData = JSON.parse(readFileSync(output));
  if (resultData.impulseJointPayloadExact !== undefined) {
    assert.ok(resultData.impulseJointPayloadExact && resultData.otherSectionsExact && resultData.wakeUpMembersExact && resultData.joinMembersExact);
    const world = activeRapier.World.restoreSnapshot(readFileSync(input));
    try {
      for (const n of resultData.joints) {
        const j = world.impulseJoints.get(handleOf(n.handle)), parent = j.body1(), child = j.body2();
        const f1 = quatMultiply(parent.rotation(), j.frameX1()), f2 = quatMultiply(child.rotation(), j.frameX2());
        let q = quatMultiply(quatInverse(f1), f2); const length = Math.hypot(q.x, q.y, q.z, q.w);
        q = Object.fromEntries(Object.entries(q).map(([k, v]) => [k, v / length * (q.w < 0 ? -1 : 1)]));
        n.parentFrame = { rotation: f1, anchor: worldPoint(parent.translation(), parent.rotation(), j.anchor1()) };
        n.childFrame = { rotation: f2, anchor: worldPoint(child.translation(), child.rotation(), j.anchor2()) };
        n.localFrames = [{ rotation: j.frameX1(), anchor: j.anchor1() }, { rotation: j.frameX2(), anchor: j.anchor2() }];
        n.coupledAxes = null;
        n.limits = Array.from({ length: 6 }, (_, axis) => ({ min: world.impulseJoints.raw.jointLimitsMin(j.handle, axis),
          max: world.impulseJoints.raw.jointLimitsMax(j.handle, axis), enabled: world.impulseJoints.raw.jointLimitsEnabled(j.handle, axis) }));
        n.limitAxes = n.limits.reduce((mask, l, axis) => mask | (l.enabled ? 1 << axis : 0), 0);
        n.coordinates = axes.map((a, i) => ({ axis: i + 3, limitCoordinate: 2 * Math.atan2(q[a], q.w), motorCoordinate: 2 * Math.asin(Math.max(-1, Math.min(1, q[a]))) }));
        n.rawAngularRows = axes.flatMap((a, i) => n.motorAxes & (1 << (i + 3)) ? [{ kind: `Motor(${i + 3})`,
          childAngularJacobian: rotate(f1, { x: a === 'x' ? 1 : 0, y: a === 'y' ? 1 : 0, z: a === 'z' ? 1 : 0 }) }] : []);
        n.finalizedRows = null;
      }
    } finally { world.free(); }
    resultData.serializationPayloadExact = true; resultData.unorderedMembersExact = true;
    resultData.rowStatus = 'incomplete'; resultData.coordinateProvenance = 'Native WASM frames and limits; coordinates and raw motor basis evaluated in JS from retained native formulas. Full native raw/finalized row execution unavailable. Not a native f32 or substep row measurement.';
    json(output + '.frames.json', resultData);
  }
  assert.equal(resultData.serializationPayloadExact, true); assert.equal(resultData.unorderedMembersExact, true);
  return resultData;
}
function compareCoordinates(failure, native) {
  const rows = [];
  for (const [id, handle] of failure.handles) {
    const d = SEGMENT_BY_ID.get(id); if (!d.jointProfile) continue;
    const parent = failure.preBodies[d.parent], child = failure.preBodies[id], profile = d.jointProfile;
    const n = native.joints.find(j => same(j.child, rawParts(handle))); assert.ok(n, id);
    assert.ok(n.coupledAxes === 0 || n.coupledAxes === null); assert.equal(n.parent[0], rawParts(new Map(failure.handles).get(d.parent))[0]);
    const coordinates = jointCoordinates(parent.rotation, child.rotation, profile);
    const k = jointCoordinateKinematics(parent.angularVelocity, child.angularVelocity, parent.rotation, coordinates, profile);
    const basis = jointFrameAxesWorld(parent.rotation, profile);
    const jacobianChecks = axes.map(a => {
      const finiteDifference = {};
      for (const b of axes) {
        const q = sign => ({ x: b === 'x' ? sign * Math.sin(5e-7) : 0, y: b === 'y' ? sign * Math.sin(5e-7) : 0,
          z: b === 'z' ? sign * Math.sin(5e-7) : 0, w: Math.cos(5e-7) });
        const plus = jointCoordinates(parent.rotation, quatMultiply(q(1), child.rotation), profile)[a];
        const minus = jointCoordinates(parent.rotation, quatMultiply(q(-1), child.rotation), profile)[a];
        finiteDifference[b] = (plus - minus) / 2e-6;
      }
      const error = norm(sub(finiteDifference, k.torqueAxesWorld[a]));
      assert.ok(error <= m.criteria.analyticJacobianFiniteDifferenceTolerance, `${id}:${a} derivative`);
      const ni = n.coordinates[axes.indexOf(a)], enabled = profile.axes.find(x => x.coordinate === a);
      const motor = n.motors[axes.indexOf(a) + 3];
      const limitError = Math.abs(ni.limitCoordinate - coordinates[a]);
      assert.ok(limitError <= m.criteria.nativeVsApplicationLimitRad, `${id}:${a} limit coordinate`);
      if (enabled) {
        assert.equal(motor.maxForce, Math.fround(enabled.maxMotorTorqueNm)); assert.equal(motor.model, 'ForceBased');
        assert.ok(n.motorAxes & (1 << (axes.indexOf(a) + 3))); assert.equal(n.lockedAxes & (1 << (axes.indexOf(a) + 3)), 0);
        assert.equal(n.limits[axes.indexOf(a) + 3].min, Math.fround(enabled.minRadians));
        assert.equal(n.limits[axes.indexOf(a) + 3].max, Math.fround(enabled.maxRadians));
      } else {
        assert.ok(n.lockedAxes & (1 << (axes.indexOf(a) + 3)));
      }
      const nativeMotorRow = n.rawAngularRows.find(r => r.kind === `Motor(${axes.indexOf(a) + 3})`);
      const rawMotorJacobian = nativeMotorRow ? vector(nativeMotorRow.childAngularJacobian) : null;
      if (rawMotorJacobian) assert.ok(norm(sub(rawMotorJacobian, basis[a])) < 2e-6);
      const relative = sub(child.angularVelocity, parent.angularVelocity);
      return { axis: a, enabled: !!enabled, applicationCoordinate: coordinates[a], nativeLimitCoordinate: ni.limitCoordinate,
        nativeMotorCoordinate: ni.motorCoordinate, limitDifferenceRad: limitError, motorCoordinateDifferenceRad: ni.motorCoordinate - coordinates[a],
        nativePositionStiffness: motor.stiffness, motorPositionRequestDifferenceNm: motor.stiffness * (coordinates[a] - ni.motorCoordinate),
        applicationJacobian: k.torqueAxesWorld[a], finiteDifference, finiteDifferenceError: error, nativeMotorJacobian: rawMotorJacobian,
        applicationVsNativeJacobianNorm: rawMotorJacobian ? norm(sub(k.torqueAxesWorld[a], rawMotorJacobian)) : null,
        applicationRate: dot(relative, k.torqueAxesWorld[a]), nativeAxisRate: dot(relative, basis[a]) };
    });
    rows.push({ id, handle: handleOf(n.handle), native: n, jacobianChecks });
  }
  return rows;
}
function contacts(world, failure) {
  const ids = new Map(failure.collision.colliderSegments), result = [];
  const all = []; world.colliders.forEach(c => all.push(c));
  for (let i = 0; i < all.length; i++) for (let j = i + 1; j < all.length; j++) world.contactPair(all[i], all[j], (p, flipped) => {
    result.push({ pair: [ids.get(all[i].handle) ?? 'floor', ids.get(all[j].handle) ?? 'floor'], flipped, normal: { ...p.normal() },
      contacts: Array.from({ length: p.numContacts() }, (_, k) => ({ distance: p.contactDist(k), normalImpulse: p.contactImpulse(k),
        tangentImpulse: [p.contactTangentImpulseX(k), p.contactTangentImpulseY(k)], local1: p.localContactPoint1(k), local2: p.localContactPoint2(k) })),
      solverContacts: Array.from({ length: p.numSolverContacts() }, (_, k) => ({ point: p.solverContactPoint(k), distance: p.solverContactDist(k) })),
      friction: p.friction(), restitution: p.restitution() });
  });
  return result;
}
function metrics(bodies, failure) {
  let anchorM = 0, limitRad = 0;
  const joints = {};
  for (const [id, state] of Object.entries(bodies)) {
    const d = SEGMENT_BY_ID.get(id); if (!d.jointProfile) continue;
    const parent = bodies[d.parent], beforeParent = failure.preBodies[d.parent];
    const basis = jointFrameAxesWorld(beforeParent.rotation, d.jointProfile);
    const relative = sub(state.angularVelocity, parent.angularVelocity);
    const coordinates = jointCoordinates(parent.rotation, state.rotation, d.jointProfile);
    anchorM = Math.max(anchorM, norm(sub(worldPoint(parent.position, parent.rotation, d.jointAnchorParent), worldPoint(state.position, state.rotation, d.jointAnchorChild))));
    limitRad = Math.max(limitRad, jointLimitErrorMagnitude(coordinates, d.jointProfile));
    joints[id] = { relativeRates: Object.fromEntries(axes.map(a => [a, dot(relative, basis[a])])),
      bodyRates: Object.fromEntries(axes.map(a => [a, dot(state.angularVelocity, basis[a])])), coordinates };
  }
  const rates = Object.entries(bodies).map(([id, b]) => ({ id, angular: norm(b.angularVelocity), linear: norm(b.velocity) }));
  const maximumAngular = Math.max(...rates.map(r => r.angular)), maximumLinear = Math.max(...rates.map(r => r.linear));
  return { maximumAngular, maximumLinear, peakAngularSegment: rates.find(r => r.angular === maximumAngular).id, anchorM, limitRad,
    officialSpeedWithin: maximumAngular <= 0.5 && maximumLinear <= 0.1, joints,
    forefeet: Object.fromEntries(['leftForefoot', 'rightForefoot'].map(id => [id, { before: failure.preBodies[id].angularVelocity,
      after: bodies[id].angularVelocity, poseRate: angularVelocity(failure.preBodies[id].rotation, bodies[id].rotation, 1/60), ...joints[id] }])) };
}
async function worker(directory, stateName, buildDirectory, deadline) {
  mkdirSync(directory);
  const report = { status: 'incomplete', stateName, steps: 0, trials: [], comparisons: [] }, started = performance.now();
  try {
    const wasmMemories = new Set(), instantiate = WebAssembly.instantiate;
    WebAssembly.instantiate = async function (...args) {
      const result = await instantiate.apply(this, args), instance = result.instance ?? result;
      for (const v of Object.values(instance.exports)) if (v instanceof WebAssembly.Memory) wasmMemories.add(v);
      return result;
    };
    const { default: RAPIER } = await import('@dimforge/rapier3d-compat'); await RAPIER.init();
    activeRapier = RAPIER;
    WebAssembly.instantiate = instantiate;
    const input = resolve(m.inputs.h74Root, m.inputs.attempt, stateName), failure = JSON.parse(readFileSync(join(input, 'first-failure.json')));
    assert.equal(failure.tick, m.inputs.ticks[m.inputs.states.indexOf(stateName)]);
    for (const name of ['first-failure.bin', 'first-failure.json', 'scenario.json']) copyFileSync(join(input, name), join(directory, name));
    const bytes = readFileSync(join(input, 'first-failure.bin')); assert.equal(sha256(bytes), failure.nativeSnapshotSha256);
    const build = inspectorBuild(buildDirectory); assert.equal(build.status, 'pass'); report.nativeRows = build.rowStatus ?? 'pass';
    assert.equal(hashFile(build.executable), build.executableSha256);
    const native = nativeInspect(build.executable, join(directory, 'first-failure.bin'), join(directory, 'native-pre.json'), deadline);
    const coordinates = compareCoordinates(failure, native); json(join(directory, 'coordinates.json'), coordinates);
    const probes = m.perturbations.segments.flatMap(id => SEGMENT_BY_ID.get(id).jointProfile.axes.map(a => ({ id, axis: a.coordinate })));
    assert.equal(probes.length, m.budget.axesPerState);
    const commands = [{ id: 'control', bias: 0 }, ...probes.flatMap(p => m.perturbations.biasNm.map(bias => ({ ...p, bias })))];
    let control;
    for (const [commandIndex, command] of commands.entries()) {
      let first;
      for (let repetition = 0; repetition < 2; repetition++) {
        assert.ok(Date.now() < deadline, 'Wall budget'); assert.ok(report.steps < m.budget.trialsPerState);
        const stem = `${String(commandIndex).padStart(2, '0')}-${repetition}`, trialDir = join(directory, stem); mkdirSync(trialDir);
        const world = RAPIER.World.restoreSnapshot(bytes), queue = new RAPIER.EventQueue(true);
        const began = performance.now();
        try {
          assert.deepEqual(captureBodies(world, failure.handles), failure.preBodies);
          assert.equal(world.timestep, Math.fround(1/60)); assert.equal(world.numSolverIterations, 20);
          assert.equal(world.numInternalPgsIterations, 20); assert.equal(world.maxCcdSubsteps, 4);
          const refs = failure.handles.map(([id, h]) => [id, world.getRigidBody(h)]), colliderRefs = []; world.colliders.forEach(c => colliderRefs.push(c));
          for (const [, b] of refs) {
            assert.ok(b.isDynamic());
            for (const name of ['setTranslation', 'setRotation', 'setLinvel', 'setAngvel', 'setBodyType', 'setNextKinematicTranslation', 'setNextKinematicRotation'])
              b[name] = () => { throw new Error(`Forbidden body setter: ${name}`); };
          }
          let configured = null;
          if (command.id !== 'control') {
            const coordinate = coordinates.find(c => c.id === command.id), i = axes.indexOf(command.axis) + 3, motor = coordinate.native.motors[i];
            const jac = vector(coordinate.native.rawAngularRows.find(r => r.kind === `Motor(${i})`).childAngularJacobian);
            const d = SEGMENT_BY_ID.get(command.id);
            const relative = sub(failure.preBodies[command.id].angularVelocity, failure.preBodies[d.parent].angularVelocity);
            const velocity = Math.fround(motor.targetVelocity + command.bias / motor.damping);
            const requested = motor.damping * (velocity - dot(relative, jac));
            assert.equal(motor.stiffness, 0); assert.ok(motor.damping > 0); assert.ok(Math.abs(requested) <= motor.maxForce);
            configured = { joint: coordinate.handle, axis: i, before: motor, afterTargetVelocity: velocity,
              requestedNm: requested, effectiveBiasNm: motor.damping * (velocity - motor.targetVelocity) };
            world.impulseJoints.raw.jointConfigureMotor(coordinate.handle, i, motor.targetPosition, velocity, motor.stiffness, motor.damping);
          }
          const preSnapshot = world.takeSnapshot(); writeFileSync(join(trialDir, 'pre.bin'), preSnapshot, { flag: 'wx' });
          const beforeContacts = contacts(world, failure);
          world.step(queue, restoredHooks(RAPIER, failure.collision)); report.steps++;
          const bodies = captureBodies(world, failure.handles), measurements = metrics(bodies, failure), measuredContacts = contacts(world, failure);
          const excluded = new Set(failure.collision.excludedPairs);
          measurements.floorPenetrationM = Math.max(0, ...measuredContacts.filter(c => c.pair.includes('floor')).flatMap(c => c.contacts.map(p => -p.distance)));
          measurements.nonExcludedSelfPenetrationM = Math.max(0, ...measuredContacts.filter(c => !c.pair.includes('floor') && !excluded.has([...c.pair].sort().join('|'))).flatMap(c => c.contacts.map(p => -p.distance)));
          finite(bodies); finite(measurements); finite(measuredContacts);
          assert.equal(world.bodies.len(), 26); assert.equal(refs.length, 25);
          for (const [id, b] of refs) { assert.equal(world.getRigidBody(new Map(failure.handles).get(id)), b); assert.ok(b.isDynamic()); assert.equal(b.mass(), failure.preBodies[id].mass); }
          for (const c of colliderRefs) assert.equal(world.getCollider(c.handle), c);
          assert.ok(measurements.anchorM <= 0.08); assert.ok(measurements.limitRad <= 0.06);
          assert.ok(measurements.floorPenetrationM <= 0.08); assert.ok(measurements.nonExcludedSelfPenetrationM <= 0.005);
          if (command.id === 'control') assert.deepEqual(bodies, failure.postBodies, 'Original 25-body replay');
          if (first) assert.deepEqual(bodies, first.bodies, 'Independent repeated command');
          const memory = { ...process.memoryUsage(), wasmLinearBytes: [...wasmMemories].reduce((s, w) => s + w.buffer.byteLength, 0) };
          assert.ok(memory.rss <= m.budget.maximumProcessRssMiB * 2**20, 'RSS budget');
          const result = { command, repetition, configured, bodies, measurements, beforeContacts, contacts: measuredContacts,
            preSnapshotSha256: sha256(preSnapshot), wallMs: performance.now() - began, memory, pid: process.pid };
          writeFileSync(join(trialDir, 'post.bin'), world.takeSnapshot(), { flag: 'wx' });
          json(join(trialDir, 'result.json'), result);
          if (!first) first = result;
          report.trials.push({ stem, command, repetition, bodySha256: sha256(JSON.stringify(bodies)), maximumAngular: measurements.maximumAngular,
            maximumLinear: measurements.maximumLinear, officialSpeedWithin: measurements.officialSpeedWithin, wallMs: result.wallMs, memory });
          appendFileSync(join(directory, 'progress.ndjson'), JSON.stringify(report.trials.at(-1)) + '\n');
        } finally { queue.free(); world.free(); }
      }
      if (command.id === 'control') control = first;
      else report.comparisons.push({ ...command, deltaRelativeRate: first.measurements.joints[command.id].relativeRates[command.axis] - control.measurements.joints[command.id].relativeRates[command.axis],
        angularChange: first.measurements.maximumAngular - control.measurements.maximumAngular,
        forefeet: first.measurements.forefeet, maximumAngular: first.measurements.maximumAngular, officialSpeedWithin: first.measurements.officialSpeedWithin });
      if (commandIndex % 14 === 0) console.log(JSON.stringify({ stateName, completedSteps: report.steps }));
    }
    // Native inspection is read-only and uses only the first repetition of each command.
    for (let commandIndex = 0; commandIndex < commands.length; commandIndex++) {
      assert.ok(Date.now() < deadline, 'Wall budget');
      const trialDir = join(directory, `${String(commandIndex).padStart(2, '0')}-0`);
      const pre = nativeInspect(build.executable, join(trialDir, 'pre.bin'), join(trialDir, 'native-pre.json'), deadline);
      const command = commands[commandIndex];
      let changes = 0;
      for (let j = 0; j < native.joints.length; j++) {
        const old = native.joints[j], now = pre.joints[j];
        assert.deepEqual(now.parentFrame, old.parentFrame); assert.deepEqual(now.childFrame, old.childFrame);
        for (const key of ['lockedAxes', 'motorAxes', 'limitAxes', 'coupledAxes', 'limits']) assert.deepEqual(now[key], old[key]);
        for (let i = 0; i < 6; i++) {
          const expected = structuredClone(old.motors[i]);
          const coord = coordinates.find(c => c.id === command.id);
          if (command.id !== 'control' && same(now.handle, coord.native.handle) && i === axes.indexOf(command.axis) + 3) {
            expected.targetVelocity = Math.fround(expected.targetVelocity + command.bias / expected.damping); changes++;
          }
          assert.deepEqual(now.motors[i], expected, 'Only declared target velocity may change');
        }
      }
      assert.equal(changes, command.id === 'control' ? 0 : 1);
      nativeInspect(build.executable, join(trialDir, 'post.bin'), join(trialDir, 'native-post.json'), deadline);
    }
    report.signs = probes.map(p => ({ ...p, magnitudes: [0.25, 0.5].map(bias => {
      const plus = report.comparisons.find(c => c.id === p.id && c.axis === p.axis && c.bias === bias);
      const minus = report.comparisons.find(c => c.id === p.id && c.axis === p.axis && c.bias === -bias);
      return { bias, plus: plus.deltaRelativeRate, minus: minus.deltaRelativeRate,
        signed: plus.deltaRelativeRate > m.criteria.materialAngularResponseRadps && minus.deltaRelativeRate < -m.criteria.materialAngularResponseRadps };
    }) }));
    report.control = control.measurements; report.tick = failure.tick; report.status = 'pass';
    report.coordinateSummary = { motorAxes: coordinates.flatMap(c => c.jacobianChecks).filter(c => c.enabled).length,
      maxLimitDifferenceRad: Math.max(...coordinates.flatMap(c => c.jacobianChecks.map(a => a.limitDifferenceRad))),
      maxMotorCoordinateDifferenceRad: Math.max(...coordinates.flatMap(c => c.jacobianChecks.filter(a => a.enabled).map(a => Math.abs(a.motorCoordinateDifferenceRad)))),
      maxMotorPositionRequestDifferenceNm: Math.max(...coordinates.flatMap(c => c.jacobianChecks.map(a => Math.abs(a.motorPositionRequestDifferenceNm)))),
      maxJacobianFiniteDifferenceError: Math.max(...coordinates.flatMap(c => c.jacobianChecks.map(a => a.finiteDifferenceError))) };
  } catch (error) { report.status = 'fail'; report.error = error.stack; process.exitCode = 1; }
  report.wallMs = performance.now() - started;
  json(join(directory, 'report.json'), report); const files = inventory(directory); verifyInventory(directory, files); json(join(directory, 'artifacts.json'), files);
  console.log(JSON.stringify({ stateName, status: report.status, steps: report.steps, error: report.error }));
}
async function main(output, buildDirectory) {
  assert.ok(!existsSync(output), 'Use a fresh evidence directory'); mkdirSync(output, { recursive: true });
  const start = performance.now(), deadline = Date.now() + m.budget.maximumExperimentWallS * 1000;
  const report = { diagnostic: 'incomplete', feasibility: 'failed/incomplete', acceptance: 'incomplete', runs: [] };
  let source;
  try {
    assert.ok(process.execArgv.includes('--max-old-space-size=384'), 'Run with the frozen old-space flag');
    const prior = verifyPrior(); json(join(output, 'preflight.json'), prior);
    source = fingerprints(); copyFileSync(manifestPath, join(output, 'experiment.json'));
    const build = inspectorBuild(buildDirectory);
    assert.equal(build.status, 'pass'); assert.equal(hashFile(build.executable), build.executableSha256);
    if (!build.rowStatus) {
      assert.equal(hashFile(manifestPath), build.experimentSha256);
      verifyInventory(join(buildDirectory, 'src'), build.sources); verifyInventory(join(buildDirectory, 'rapier3d/src'), build.nativeSources);
    }
    json(join(output, 'native-build-report.json'), build);
    report.nativeRows = build.rowStatus ?? 'pass';
    const sourceBytes = Object.fromEntries(Object.keys(source).map(p => [p, readFileSync(p).toString('base64')]));
    writeFileSync(join(output, 'source-snapshot.json.gz'), gzipSync(JSON.stringify({ encoding: 'base64', files: sourceBytes })), { flag: 'wx' });
    for (const [name, args] of [['working-tree.patch', ['diff', '--binary', 'HEAD']], ['working-tree-status.txt', ['status', '--short']]]) {
      const r = spawnSync('git', args, { encoding: 'utf8', windowsHide: true, maxBuffer: 16 * 2**20 }); assert.equal(r.status, 0); writeFileSync(join(output, name), r.stdout, { flag: 'wx' });
    }
    json(join(output, 'run-manifest.json'), { started: new Date().toISOString(), source, prior, experimentSha256: hashFile(manifestPath),
      buildReportSha256: hashFile(join(output, 'native-build-report.json')), command: [process.execPath, ...process.execArgv, ...process.argv.slice(1)],
      node: process.version, nodeExecutable: process.execPath, cpu: os.cpus()[0].model, os: `${os.type()} ${os.release()} ${os.arch()}`,
      qualification: 'Serial offline one-step copies, unchanged H74 sources and runtime. Frozen-pose native reconstruction is not direct WASM substep instrumentation.' });
    for (const name of m.inputs.states) {
      const args = ['--max-old-space-size=384', '--import', 'tsx', 'scripts/standing-angular-diagnostic.mjs', '--worker', join(output, name), name, buildDirectory, String(deadline)];
      const fd = openSync(join(output, `${name}.log`), 'wx'); let result;
      try { result = spawnSync(process.execPath, args, { stdio: ['ignore', fd, fd], windowsHide: true, timeout: Math.max(1, deadline - Date.now()) }); }
      finally { closeSync(fd); }
      const path = join(output, name, 'report.json'), workerReport = existsSync(path) ? JSON.parse(readFileSync(path)) : { status: 'incomplete', steps: null };
      report.runs.push({ name, command: [process.execPath, ...args], exit: result.status, error: result.error?.message ?? null, report: workerReport });
      console.log(JSON.stringify({ name, exit: result.status, status: workerReport.status, steps: workerReport.steps }));
      if (result.status !== 0 || workerReport.status !== 'pass') throw new Error(`Diagnostic failed/incomplete: ${name}`);
    }
    report.steps = report.runs.reduce((s, r) => s + r.report.steps, 0); assert.equal(report.steps, m.budget.maximumPhysicalSteps);
    assert.deepEqual(fingerprints(), source); json(join(output, 'integrity-after.json'), verifyPrior());
    report.diagnostic = report.nativeRows === 'pass' ? 'pass' : 'incomplete';
    report.motorProbes = 'pass';
    report.conclusion = 'Completed frozen motor probes, not standing feasibility. Full native row reconstruction remains incomplete when its build failed; no runtime repair included.';
  } catch (error) { report.diagnostic = 'fail/incomplete'; report.error = error.stack; }
  report.wallMs = performance.now() - start;
  json(join(output, 'report.json'), report); const files = inventory(output); verifyInventory(output, files); json(join(output, 'artifacts.json'), files);
  // Standing remains failed/incomplete even if every diagnostic check completes.
  process.exitCode = 1;
}
if (process.argv[2] === '--worker') await worker(resolve(process.argv[3]), process.argv[4], resolve(process.argv[5]), Number(process.argv[6]));
else await main(resolve(process.argv[2] ?? `evidence/standing-h75-v1/attempt-${Date.now()}`), resolve(process.argv[3] ?? 'evidence/standing-h75-v1/native-build-02'));
