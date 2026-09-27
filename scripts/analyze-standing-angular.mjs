import assert from 'node:assert/strict';
import { existsSync, mkdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { gunzipSync } from 'node:zlib';
import { inventory, json, norm, verifyInventory } from './coordinated-standing-evidence.mjs';
import { sha256 } from './capture-physics-baseline.mjs';

const [input, output] = process.argv.slice(2).map(p => resolve(p));
assert.ok(input && output && !existsSync(output), 'Supply an existing attempt and a fresh analysis directory');
mkdirSync(output, { recursive: true });
const load = path => JSON.parse(readFileSync(path));
const files = load(join(input, 'artifacts.json')); verifyInventory(input, files);
const manifest = load(join(input, 'run-manifest.json'));
const snapshot = JSON.parse(gunzipSync(readFileSync(join(input, 'source-snapshot.json.gz'))));
assert.equal(snapshot.encoding, 'base64');
for (const [path, bytes] of Object.entries(snapshot.files)) assert.equal(sha256(Buffer.from(bytes, 'base64')), manifest.source[path], path);
for (const [path, hash] of Object.entries(manifest.source)) assert.equal(sha256(readFileSync(path)), hash, `Executed source changed: ${path}`);
const parent = load(join(input, 'report.json'));
const results = parent.runs.map(({ name, report: r }) => {
  const coordinates = load(join(input, name, 'coordinates.json'));
  const control = load(join(input, name, '00-0/result.json'));
  const failure = load(join(input, name, 'first-failure.json'));
  const topology = cs => cs.map(c => ({ pair: c.pair, contacts: c.contacts.length, solverContacts: c.solverContacts.length }));
  let topologyChanges = 0, maxEffectiveBiasErrorNm = 0, matchedRepeatContacts = 0;
  const violations = [], allMeasurements = [];
  for (const t of r.trials) {
    const result = load(join(input, name, t.stem, 'result.json'));
    allMeasurements.push(result.measurements);
    if (result.configured) maxEffectiveBiasErrorNm = Math.max(maxEffectiveBiasErrorNm, Math.abs(result.configured.effectiveBiasNm - result.command.bias));
    if (!result.measurements.officialSpeedWithin) violations.push({ stem: t.stem, command: t.command, angular: result.measurements.maximumAngular });
    if (JSON.stringify(topology(result.contacts)) !== JSON.stringify(topology(control.contacts))) topologyChanges++;
    if (t.repetition === 1) {
      const first = load(join(input, name, t.stem.replace(/1$/, '0'), 'result.json'));
      assert.deepEqual(result.contacts, first.contacts); assert.deepEqual(result.bodies, first.bodies); matchedRepeatContacts++;
    }
  }
  const feet = Object.fromEntries(['leftForefoot', 'rightForefoot'].map(id => {
    const c = coordinates.find(c => c.id === id), f = r.control.forefeet[id];
    return [id, { nativeBodySpeed: norm(f.after), relativePermittedRate: f.relativeRates.x,
      relativeLockedRateNorm: Math.hypot(f.relativeRates.y, f.relativeRates.z),
      poseVsFinalNativeVectorDifference: norm({ x: f.poseRate.x-f.after.x, y: f.poseRate.y-f.after.y, z: f.poseRate.z-f.after.z }),
      permittedMotorCoordinateDifferenceRad: c.jacobianChecks[0].motorCoordinateDifferenceRad,
      permittedJacobianDifference: c.jacobianChecks[0].applicationVsNativeJacobianNorm,
      maximumLockedCoordinateRad: Math.max(Math.abs(f.coordinates.y), Math.abs(f.coordinates.z)) }];
  }));
  const summarizeProbe = c => ({ id: c.id, axis: c.axis, biasNm: c.bias, angularRadps: c.maximumAngular, deltaRelativeRate: c.deltaRelativeRate });
  return { name, tick: r.tick, completedSteps: r.steps, referenceOffsetM: failure.scenario.offset,
    coordinateSummary: r.coordinateSummary, controlsExactBodies: 2*25, repeatedCommands: matchedRepeatContacts,
    signedAxesBothMagnitudes: r.signs.filter(s => s.magnitudes.every(m => m.signed)).length,
    unresolvedSigns: r.signs.filter(s => !s.magnitudes.every(m => m.signed)),
    nativeControlAngularRadps: r.control.maximumAngular, feet,
    minimumPerturbedAngularRadps: Math.min(...r.comparisons.map(c => c.maximumAngular)),
    maximumPerturbedAngularRadps: Math.max(...r.comparisons.map(c => c.maximumAngular)),
    bestProbe: summarizeProbe(r.comparisons.toSorted((a,b) => a.maximumAngular-b.maximumAngular)[0]),
    distalProbes: r.comparisons.filter(c => /Forefoot/.test(c.id)).map(summarizeProbe),
    unchangedContactTopologyTrials: r.trials.length-topologyChanges, changedContactTopologyTrials: topologyChanges,
    measuredBaselineFootContacts: failure.contacts.filter(c => c.loadBearing),
    maximumEffectiveBiasErrorNm: maxEffectiveBiasErrorNm,
    officialSpeedViolationTrials: violations.length, violations,
    invariantMaxima: Object.fromEntries(['anchorM', 'limitRad', 'floorPenetrationM', 'nonExcludedSelfPenetrationM'].map(k => [k, Math.max(...allMeasurements.map(m => m[k]))])),
    memoryMaxima: Object.fromEntries(['rss','heapUsed','external','arrayBuffers','wasmLinearBytes'].map(k => [k, Math.max(...r.trials.map(t => t.memory[k]))])) };
});
const report = { diagnostic: parent.diagnostic, nativeRows: parent.nativeRows, motorProbes: parent.motorProbes,
  feasibility: 'failed/incomplete', controllerRepair: 'none', physicalSteps: parent.steps, wallMs: parent.wallMs,
  verifiedAttemptArtifacts: Object.keys(files).length, sourceSnapshotMembers: Object.keys(snapshot.files).length,
  inputReportSha256: sha256(readFileSync(join(input, 'report.json'))), analysisSourceSha256: sha256(readFileSync(new URL(import.meta.url))),
  results, conclusions: [
    'All native position stiffnesses are zero. The asin/atan2 position-coordinate difference contributes zero native positional motor request in these retained states.',
    'Native WASM frame/limit readback agrees with application coordinates to numerical precision; this is not independent compiled f32 coordinate/row evaluation.',
    'Both signed magnitudes give local authority on all fourteen leg axes at the central and +5 mm failures. Local improvement does not isolate a controller, contact or constraint cause.',
    'Forefoot locked-direction relative angular rates are substantial despite small coordinate errors. Transient substep rows, impulses and contacts must be observed before attributing the jump.',
    'Three native builds failed; the final error was an ambiguous scalar/SIMD update overload. Its explicit scalar type is now in source, but was not rebuilt after the frozen build budget expired.',
    'No runtime/controller repair, standing reference extension, acceptance pass, delivered whole-step torque claim or H73 diagnosis follows.' ],
  nextDiagnostic: 'Freeze a new bounded build-and-read contract. Build the qualified scalar inspector once against the retained native-build-03 library, then inspect the three saved original pre-step and three exact control post-step snapshots (six reads, zero new physical steps). Validate row construction before a separately frozen, output-exact WASM substep instrumentation experiment.' };
json(join(output, 'report.json'), report); const outFiles = inventory(output); verifyInventory(output, outFiles); json(join(output, 'artifacts.json'), outFiles);
console.log(JSON.stringify({ diagnostic: report.diagnostic, physicalSteps: report.physicalSteps, verifiedAttemptArtifacts: report.verifiedAttemptArtifacts,
  results: results.map(r => ({ name:r.name, signs:r.signedAxesBothMagnitudes, best:r.bestProbe, topologyChanges:r.changedContactTopologyTrials,
    violations:r.officialSpeedViolationTrials, feet:r.feet, invariantMaxima:r.invariantMaxima, memory:r.memoryMaxima, unresolvedSigns:r.unresolvedSigns })) }));
