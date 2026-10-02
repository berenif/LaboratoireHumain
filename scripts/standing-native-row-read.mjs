import assert from 'node:assert/strict';
import { copyFileSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { basename, join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import { sha256 } from './capture-physics-baseline.mjs';
import { inventory, json, verifyInventory } from './coordinated-standing-evidence.mjs';

const output = resolve(process.argv[2] ?? 'evidence/standing-h76-v1/read-01');
const executable = resolve(process.argv[3] ?? 'evidence/standing-h76-v1/native-build-05/standing-angular-native.exe');
assert.ok(!existsSync(output), 'Use a fresh H76 read directory');
assert.ok(existsSync(executable), 'The H76 scalar inspector must pass preflight first');
mkdirSync(output, { recursive: true });

const inputs = [0, 1, 2].flatMap(reference => ['pre', 'post'].map(phase => ({
  reference,
  phase,
  path: resolve(`evidence/standing-h75-v1/attempt-01/reference-${reference}/00-0/${phase}.bin`),
})));

const finite = value => typeof value === 'number' && Number.isFinite(value);
const allFinite = value => Array.isArray(value) ? value.every(allFinite)
  : value && typeof value === 'object' ? Object.values(value).every(allFinite) : finite(value);
const summaries = [];

for (const input of inputs) {
  const inputName = `reference-${input.reference}-${input.phase}.bin`;
  const archivedInput = join(output, inputName);
  const resultPath = join(output, inputName.replace('.bin', '.rows.json'));
  copyFileSync(input.path, archivedInput, 0);
  const run = spawnSync(executable, [archivedInput, resultPath], {
    cwd: process.cwd(), encoding: 'utf8', windowsHide: true, timeout: 60_000,
  });
  assert.equal(run.status, 0, `${inputName}: ${run.stderr || run.error?.message || 'inspector failed'}`);
  const result = JSON.parse(readFileSync(resultPath));
  assert.equal(result.serializationPayloadExact, true, inputName);
  assert.equal(result.unorderedMembersExact, true, inputName);
  assert.equal(result.joints.length, 24, inputName);
  assert.ok(Math.abs(result.rowDt - 1 / 1200) < 1e-10, `${inputName}: solver row dt`);
  const kinds = { motor: 0, limit: 0, lock: 0 };
  let storedMotorImpulseMaximum = 0;
  let storedDofImpulseMaximum = 0;
  let maximumAbsoluteRhs = 0;
  let minimumInverseEffectiveMass = Infinity;
  let maximumCfmCoefficient = 0;
  for (const joint of result.joints) {
    assert.ok(allFinite(joint.relativeQuaternion), `${inputName}: relative quaternion`);
    assert.ok(joint.coordinates.every(coordinate => finite(coordinate.limitCoordinate)
      && finite(coordinate.motorCoordinate)), `${inputName}: coordinates`);
    for (const motor of joint.motors) {
      assert.ok([motor.targetPosition, motor.targetVelocity, motor.stiffness, motor.damping,
        motor.maxForce, motor.storedImpulse].every(finite), `${inputName}: motor state`);
      storedMotorImpulseMaximum = Math.max(storedMotorImpulseMaximum, Math.abs(motor.storedImpulse));
    }
    assert.ok(joint.storedDofImpulses.every(finite), `${inputName}: stored joint impulses`);
    storedDofImpulseMaximum = Math.max(storedDofImpulseMaximum, ...joint.storedDofImpulses.map(Math.abs));
    for (const row of joint.finalizedRows) {
      if (row.kind.startsWith('Motor')) kinds.motor++;
      else if (row.kind.startsWith('Limit')) kinds.limit++;
      else if (row.kind.startsWith('Dof')) kinds.lock++;
      else assert.fail(`${inputName}: unknown row ${row.kind}`);
      assert.ok(row.inverseLhs > 0 && finite(row.inverseLhs), `${inputName}: inverse effective mass`);
      assert.ok([row.rhs, row.rhsWithoutBias, row.cfmCoefficient, row.cfmGain].every(finite), `${inputName}: RHS/CFM`);
      assert.ok(allFinite(row.linearJacobian) && allFinite(row.parentAngularJacobian)
        && allFinite(row.childAngularJacobian) && allFinite(row.parentInverseInertiaJacobian)
        && allFinite(row.childInverseInertiaJacobian), `${inputName}: Jacobian`);
      maximumAbsoluteRhs = Math.max(maximumAbsoluteRhs, Math.abs(row.rhs), Math.abs(row.rhsWithoutBias));
      minimumInverseEffectiveMass = Math.min(minimumInverseEffectiveMass, row.inverseLhs);
      maximumCfmCoefficient = Math.max(maximumCfmCoefficient, row.cfmCoefficient);
    }
  }
  assert.equal(kinds.motor, 46, `${inputName}: motor row count`);
  assert.ok(kinds.lock >= 72, `${inputName}: structural lock rows`);
  summaries.push({ reference: input.reference, phase: input.phase,
    input: basename(archivedInput), inputSha256: sha256(readFileSync(archivedInput)),
    output: basename(resultPath), outputSha256: sha256(readFileSync(resultPath)),
    joints: result.joints.length, rows: kinds, storedMotorImpulseMaximum,
    storedDofImpulseMaximum, maximumAbsoluteRhs, minimumInverseEffectiveMass,
    maximumCfmCoefficient, processMemory: result.processMemory });
}

const report = {
  id: 'standing-h76-v1', status: 'pass', snapshots: summaries.length,
  physicalSteps: 0, executable, executableSha256: sha256(readFileSync(executable)), summaries,
  result: 'All six saved pre/post snapshots deserialize exactly. Motor, limit and lock rows; raw/finalized Jacobians; inverse effective masses; RHS/CFM; and stored impulses are finite with coherent static row counts.',
  qualification: 'Read-only frozen-snapshot reconstruction. The inspector performs zero physics steps and does not observe rows, warm-start changes, solved impulses, contacts or body velocities inside the original WASM step. Static coherence does not explain the retained within-step forefoot motion.',
};
json(join(output, 'report.json'), report);
const artifacts = inventory(output);
verifyInventory(output, artifacts);
json(join(output, 'artifacts.json'), artifacts);
console.log(JSON.stringify({ output, status: report.status, snapshots: report.snapshots }));
