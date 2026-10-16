// One incremental admission; manual browser transport, no reset or retry.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { appendFileSync, openSync, readFileSync, writeFileSync } from 'node:fs';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';

export function checkCounters(snapshot, previous = { real: 0, forecast: 0 }) {
  // Runtime overwrites stamp.sequence with its observation-message sequence.
  // simulation_time_s comes directly from native substeps_completed / 800.
  const integrated = snapshot.simulation_time_s * 800;
  const real = Math.round(integrated);
  assert.ok(Number.isFinite(integrated) && Math.abs(integrated - real) <= 1e-6);
  assert.ok(Math.abs(snapshot.diagnostics.getup.physics_step_s - 1 / 800) < 1e-9);
  const forecast = snapshot.diagnostics.getup.policy.forecast_substeps;
  assert.ok(Number.isSafeInteger(real) && real >= previous.real && real <= 3200);
  assert.ok(Number.isSafeInteger(forecast) && forecast >= previous.forecast && forecast <= 5000);
  assert.ok(real + forecast <= 8200);
  assert.equal(snapshot.simulation_time_s, real / 800);
  return { real, forecast, total: real + forecast };
}

export async function run(ledgerPath) {
  const ledger = JSON.parse(readFileSync(ledgerPath));
  assert.equal(ledger.status, 'running-reservation-retained');
  assert.equal(ledger.nativeReservation, 8200);
  assert.equal(ledger.wallReservationSeconds, 180);
  const output = ledger.outputDirectory;
  openSync(`${output}/browser-started-once`, 'wx');
  const deadline = Date.parse(ledger.deadlineUtc);
  const result = { startedUtc: new Date().toISOString(), deadlineUtc: ledger.deadlineUtc,
    qualification: false, snapshots: 0, maximumMainNativeSteps: 3200,
    maximumForecastSteps: 5000, maximumWallSeconds: 180, counters: { real: 0, forecast: 0, total: 0 } };
  const save = () => writeFileSync(`${output}/result.json`, JSON.stringify(result, null, 2) + '\n');
  let browser, page, value;
  const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', 'rust/dist/app', '0'],
    { stdio: ['ignore', 'pipe', 'inherit'] });
  const remaining = () => deadline - Date.now();
  const timer = setTimeout(() => { browser?.close(); server.kill(); }, Math.max(1, remaining() - 1000));
  const errors = [];
  const retainReport = async () => {
    const report = await page.evaluate(() => window.__lhRust.getupReport());
    const bytes = JSON.stringify(report) + '\n';
    writeFileSync(`${output}/reference-report.json`, bytes);
    result.referenceReport = { sha256: createHash('sha256').update(bytes).digest('hex'),
      bytes: Buffer.byteLength(bytes), teacherFrames: report.teacher_frames.length,
      studentDecisions: report.student_decisions.length };
    save();
    return report;
  };
  try {
    const origin = await new Promise((resolve, reject) => {
      server.stdout.on('data', data => {
        const url = String(data).match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];
        if (url) resolve(url);
      });
      server.once('error', reject);
      server.once('exit', code => reject(Error(`Preview exited ${code}`)));
    });
    browser = await chromium.launch({ ...browserLaunchOptions(true), executablePath: '/usr/bin/chromium' });
    page = await browser.newPage();
    page.on('pageerror', e => errors.push(String(e)));
    await page.addInitScript(() => {
      window.__lhGetUpTrial = true;
      let api;
      Object.defineProperty(window, '__lhRust', {
        get: () => api, set(value) { api = value; api.setManual(true); },
      });
    });
    await page.goto(`${origin}?renderer=webgl2`);
    await page.waitForFunction(() => window.__lhRust?.diagnostics.ready, null,
      { timeout: Math.min(45000, remaining()) });
    const record = async () => {
      value = await page.evaluate(() => window.__lhRust.diagnostics);
      const snapshot = value.snapshot;
      // Retain the state and counters before a diagnostic assertion can fail.
      appendFileSync(`${output}/trace.ndjson`, JSON.stringify(snapshot) + '\n');
      result.snapshots++;
      result.final = { tick: value.tick, time: value.time, motion: value.motion,
        failure: value.failure, counters: snapshot.counters, diagnostics: snapshot.diagnostics };
      const raw = { real: Math.round(snapshot.simulation_time_s * 800),
        forecast: snapshot.diagnostics.getup?.policy?.forecast_substeps };
      result.lastReportedCounters = raw;
      save();
      const counters = checkCounters(snapshot, result.counters);
      result.counters = counters;
      save();
      const g = snapshot.diagnostics.getup;
      assert.equal(g.learned_policy_loaded, true);
      assert.equal(g.source_bodies, 16);
      assert.equal(g.active_hinges, 21);
      assert.equal(value.workerInstances, 1);
      assert.equal(value.generation, 1);
      assert.equal(snapshot.segments.length, 25);
      assert.equal(snapshot.diagnostics.direct_pelvis_force_n, 0);
      assert.equal(snapshot.diagnostics.direct_pelvis_torque_nm, 0);
      assert.ok(snapshot.diagnostics.requested_motor_effort_nm <= 920.001);
      assert.ok(snapshot.counters.strikes <= 1);
      if (!value.failure) {
        assert.equal(g.motor_controls.length, 21);
        assert.ok(g.motor_controls.every(x => Number.isFinite(x) && Math.abs(x) <= 1));
      }
    };
    await record();
    assert.equal(value.tick, 0);
    assert.equal(result.counters.total, 0);
    await retainReport();
    while (value.tick < 160) {
      assert.ok(remaining() > 1000, 'Wall deadline reached');
      // A single normal apparatus strike, queued without integration at tick 80.
      if (value.tick === 80) {
        assert.equal(value.canStrike, true);
        await page.locator('#strike').click();
        await page.waitForFunction(() => !window.__lhRust.diagnostics.outstanding
          && !window.__lhRust.diagnostics.queueDepth);
        result.strikeQueuedAtS = 2;
      }
      // Even if the full teacher branch is triggered by this tick, its fixed
      // 5,000-step maximum plus the entire 3,200-step history fits admission.
      assert.ok(result.counters.real + 20 <= 3200);
      const reply = await page.evaluate(() => window.__lhRust.advance(1));
      const priorForecast = result.counters.forecast;
      await record();
      if (result.counters.forecast !== priorForecast || reply.failure) await retainReport();
      if (reply.failure) { result.physicalRefusal = reply.failure; break; }
      assert.equal(reply.completed_ticks, 1);
      if (value.snapshot.diagnostics.getup.policy.prototype_standing_passed) {
        result.standingPassed = true;
        break;
      }
    }
    const report = await retainReport();
    assert.equal(report.status.forecast_substeps, result.counters.forecast);
    assert.equal(report.student_decisions.length, value.snapshot.diagnostics.getup.policy.policy_decisions);
    assert.equal(value.snapshot.counters.strikes, 1);
    assert.deepEqual(errors, []);
    result.harnessCompleted = true;
    result.standingPassed ??= false;
    result.outcome = result.standingPassed ? 'prototype-stable-standing'
      : result.physicalRefusal ? 'stopped-at-refusal' : 'four-second-cap-without-stable-standing';
  } catch (error) {
    result.harnessCompleted = false;
    result.harnessError = String(error);
    // A report read never advances either world. Keep diagnostics on failure.
    if (page) try { await retainReport(); } catch (e) { result.reportError = String(e); }
    process.exitCode = 1;
  } finally {
    clearTimeout(timer);
    await browser?.close();
    server.kill();
    result.errors = errors;
    result.finishedUtc = new Date().toISOString();
    save();
  }
  console.log(JSON.stringify(result));
}

if (process.argv[1]?.endsWith('/validate-getup-bounded.mjs')) await run(process.argv[2]);
