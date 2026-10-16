// Same pinned WASM Runtime, without browser/rendering work during integration.
import assert from 'node:assert/strict';
import { readFileSync, writeFileSync, appendFileSync, openSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { gunzipSync } from 'node:zlib';
import { initSync, WorkerRuntime, getup_trial_profile } from '../rust/dist/app/lh_worker.js';

const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const output = process.argv[3] ?? 'outputs/getup-complete-verification-2026-10-16';
const save = (name, value) => writeFileSync(`${output}/${name}`, JSON.stringify(value, null, 2)+'\n');
mkdirSync(output, { recursive: true });
initSync({ module: readFileSync('rust/dist/app/lh_worker_bg.wasm') });
const profile = getup_trial_profile();
const makeRuntime = () => new WorkerRuntime('protocol', 0, '1', true, profile, undefined);
const priorRows = gunzipSync(readFileSync('docs/evidence/getup-penetration-retry-2026-10-16/trace.ndjson.gz'))
  .toString().trim().split('\n').map(JSON.parse);

if (process.argv[2] === '--preflight') {
  const runtime = makeRuntime();
  const initial = JSON.parse(runtime.advance(0));
  assert.equal(initial.snapshot.simulation_time_s, 0);
  assert.deepEqual(initial.snapshot.segments, priorRows[0].segments);
  const bytes = runtime.getup_checkpoint();
  const verified = JSON.parse(runtime.verify_getup_checkpoint(bytes));
  const restored = WorkerRuntime.from_getup_checkpoint(bytes);
  assert.deepEqual(JSON.parse(restored.advance(0)).snapshot.segments, initial.snapshot.segments);
  assert.equal(restored.getup_report(), runtime.getup_report());
  assert.equal(verified.nativeSubsteps, 0);
  writeFileSync(`${output}/preflight-checkpoint.bin`, bytes);
  save('preflight.json', { passed: true, physicsIntegrations: 0, initialStateMatchesPrior: true,
    checkpoint: verified, checkpointSha256: sha(bytes), profile: JSON.parse(profile),
    expectedReplayNativeSteps: 8540, maximumAdditionalNativeSteps: 10000,
    anticipatedWallFromPriorSeconds: 36.92994192900369*2.5,
    auxiliaryCheckpointTestNativeMaximum: 20 });
  restored.free(); runtime.free();
} else {
  const ledger = JSON.parse(readFileSync(process.argv[2]));
  assert.equal(ledger.status, 'running');
  assert.equal(ledger.maximumAdditionalNativeSteps, 10000);
  const deadline = Date.parse(ledger.deadlineUtc);
  openSync(`${output}/started-once`, 'wx');
  const runtime = makeRuntime();
  const result = { startedUtc: new Date().toISOString(), maximumPhysicalDurationS: 10,
    counters: { real: 0, teacher: 0, checkpointValidation: 0, total: 0 },
    checkpoints: [], exactPrefixPacketsCompared: 0, qualification: false };
  let reply = JSON.parse(runtime.advance(0));
  const checkpoint = label => {
    const bytes = runtime.getup_checkpoint();
    const verification = JSON.parse(runtime.verify_getup_checkpoint(bytes));
    const path = `${output}/checkpoint-${label}.bin`;
    writeFileSync(path, bytes);
    result.checkpoints.push({ path, sha256: sha(bytes), ...verification });
    save('result.json', result);
    writeFileSync(`${output}/reference-report.json`, runtime.getup_report()+'\n');
    return bytes;
  };
  const record = () => {
    const snapshot = reply.snapshot, p = snapshot.diagnostics.getup.policy;
    const real = Math.round(snapshot.simulation_time_s*800), teacher = p.forecast_substeps;
    assert.equal(snapshot.simulation_time_s, real/800);
    assert.ok(real >= result.counters.real && teacher >= result.counters.teacher);
    result.counters = { real, teacher, checkpointValidation: result.counters.checkpointValidation,
      total: real+teacher+result.counters.checkpointValidation };
    result.final = reply;
    appendFileSync(`${output}/trace.ndjson`, JSON.stringify(snapshot)+'\n');
    save('result.json', result);
    assert.ok(result.counters.total <= 10000 && real <= 8000 && teacher <= 5000);
    assert.equal(snapshot.stamp.generation, 1);
    assert.equal(snapshot.diagnostics.direct_pelvis_force_n, 0);
    assert.equal(snapshot.diagnostics.direct_pelvis_torque_nm, 0);
    assert.ok(snapshot.diagnostics.requested_motor_effort_nm <= 920.001);
    if (snapshot.simulation_time_s <= 4) {
      const old = priorRows.find(s => s.simulation_time_s === snapshot.simulation_time_s);
      if (old) { assert.deepEqual(snapshot.segments, old.segments); result.exactPrefixPacketsCompared++; }
    }
  };
  try {
    record(); checkpoint('00000');
    let nextCheckpoint = 80;
    while (reply.snapshot.stamp.tick < 400) {
      assert.ok(Date.now() < deadline-1500, 'Wall deadline');
      const tick = reply.snapshot.stamp.tick;
      if (tick === 80) {
        runtime.enqueue(JSON.stringify({ stamp: { schema: 4, generation: 1, sequence: 1, tick: 80 }, action: 'Strike' }));
        const acknowledgement = JSON.parse(runtime.advance(0));
        assert.equal(acknowledgement.acknowledgements[0].rejected, null);
      }
      const ticks = Math.min(4, 400-tick, nextCheckpoint-tick);
      const possibleTeacher = result.counters.teacher === 0 ? 5000 : 0;
      if (result.counters.total+ticks*20+possibleTeacher > 10000) { result.budgetStop = true; break; }
      reply = JSON.parse(runtime.advance(ticks));
      record();
      if (reply.failure) { result.physicalFailure = reply.failure; break; }
      assert.equal(reply.completed_ticks, ticks);
      if (reply.snapshot.stamp.tick === nextCheckpoint) {
        const bytes = checkpoint(String(result.counters.real).padStart(5,'0'));
        nextCheckpoint += 80;
        // One planned20-step prediction validates the saved4s world's next tick.
        // It is counted in the same additional-native and walltime allowance.
        if (reply.snapshot.stamp.tick === 160 && result.counters.teacher > 0) {
          assert.ok(result.counters.total+40 <= 10000);
          const restored = WorkerRuntime.from_getup_checkpoint(bytes);
          const predicted = JSON.parse(restored.advance(1));
          const predictedReal = Math.round(predicted.snapshot.simulation_time_s*800)-result.counters.real;
          const predictedTeacher = predicted.snapshot.diagnostics.getup.policy.forecast_substeps-result.counters.teacher;
          result.counters.checkpointValidation += predictedReal+predictedTeacher;
          reply = JSON.parse(runtime.advance(1));
          record();
          result.checkpointNextTick = { nativeSteps: predictedReal+predictedTeacher,
            snapshotEqual: JSON.stringify(predicted.snapshot) === JSON.stringify(reply.snapshot),
            controllerEqual: restored.getup_report() === runtime.getup_report() };
          save('checkpoint-next-tick.json', { ...result.checkpointNextTick, predicted, actual: reply });
          restored.free();
          assert.ok(result.checkpointNextTick.snapshotEqual && result.checkpointNextTick.controllerEqual,
            'Checkpoint next-tick continuation differs');
        }
      }
    }
    result.harnessCompleted = true;
    result.outcome = result.physicalFailure ? 'physical-failure'
      : result.budgetStop ? 'additional-native-cap' : 'ten-second-history-complete';
  } catch (error) {
    result.harnessCompleted = false; result.harnessError = String(error); process.exitCode = 1;
  } finally {
    try { checkpoint('final'); } catch (error) { result.finalCheckpointError = String(error); process.exitCode = 1; }
    result.finishedUtc = new Date().toISOString();
    save('result.json', result);
    runtime.free();
  }
  console.log(JSON.stringify(result));
}
