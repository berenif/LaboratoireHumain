// Every release physics stage runs in a cold instance of the exact candidate worker.
import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { fingerprint, sourceHashes, treeHashes } from './rust-release-tools.mjs';
import { validateActuatorReports } from './rust-actuator-contract.mjs';

const [outputArg, origin, artifactArg, profileArg] = process.argv.slice(2);
assert.ok(outputArg && origin && artifactArg && profileArg, 'Supply fresh output, origin, artifact and profile');
const output = resolve(outputArg), artifact = resolve(artifactArg);
assert.ok(!existsSync(output)); mkdirSync(output, { recursive: true });
const profile = JSON.parse(readFileSync(profileArg));
const source = sourceHashes(), assets = treeHashes(artifact);
const expected = { primitive:12, canonical:144, joints:216, feet:12, chains:3, stress:216, controls:36,
  actuators:432, disturbances:14, steps:9, legacy:4, recovery:24, cycles:1, terrain:21 };
const report = { schema:1, sourceFingerprint:fingerprint(source), assetsFingerprint:fingerprint(assets), stages:[], passed:false };
const browser = await chromium.launch(browserLaunchOptions());
try {
  const page = await browser.newPage(); await page.goto(origin);
  report.loadedAssets = await page.evaluate(async names => Object.fromEntries(await Promise.all(names.map(async name => {
    const response = await fetch(new URL(name, document.baseURI));
    if (!response.ok) throw new Error(`Missing asset ${name}`);
    const bytes = new Uint8Array(await crypto.subtle.digest('SHA-256', await response.arrayBuffer()));
    return [name, Array.from(bytes, value => value.toString(16).padStart(2, '0')).join('')];
  }))), Object.keys(assets));
  assert.deepEqual(report.loadedAssets, assets);
  for (const [kind, count] of Object.entries(expected)) {
    const stage = { kind, expectedCases:count, passed:false }; report.stages.push(stage);
    try {
      stage.report = await page.evaluate(async ({ kind, profile }) => {
        const worker = new Worker(new URL('sim-worker.js', document.baseURI), { type:'module' });
        try {
          return await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => reject(new Error(`Fixture ${kind} timed out`)), 30 * 60 * 1000);
            worker.onerror = event => { clearTimeout(timeout); reject(new Error(event.message)); };
            worker.onmessage = ({ data }) => {
              if (data.type === 'ready') worker.postMessage({ type:'calibrateFoundation', requestId:1, kind, profile });
              else { clearTimeout(timeout); data.type === 'foundation' ? resolve(data.report) : reject(new Error(data.message ?? data.type)); }
            };
          });
        } finally { worker.terminate(); }
      }, { kind, profile });
      const cases = stage.report.cases;
      stage.actualCases = cases.length;
      if (kind === 'actuators') stage.coverage = validateActuatorReports(cases,
        JSON.parse(readFileSync('rust/crates/model/data/getup-v1.json')));
      stage.passed = cases.length === count && cases.every(result => result.passed === true);
    } catch (error) { stage.error = String(error); }
    writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify({ kind, passed:stage.passed, cases:stage.actualCases, error:stage.error }));
  }
} catch (error) { report.error = String(error); }
finally {
  report.sourceUnchanged = fingerprint(sourceHashes()) === fingerprint(source);
  report.assetsUnchanged = fingerprint(treeHashes(artifact)) === fingerprint(assets);
  report.passed = report.stages.length === Object.keys(expected).length && report.stages.every(stage => stage.passed)
    && report.sourceUnchanged && report.assetsUnchanged && !report.error;
  writeFileSync(join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
  await browser.close();
}
process.exitCode = report.passed ? 0 : 1;
