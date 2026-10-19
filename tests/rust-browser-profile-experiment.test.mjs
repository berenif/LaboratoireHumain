import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import test from 'node:test';
import vm from 'node:vm';
import { assertBrowserProfileExperiment, browserProfileExperimentLabel, installBrowserProfileExperiment,
  loadBrowserProfileExperiment, maximumBrowserProfileBytes } from '../scripts/rust-browser-profile-experiment.mjs';

function profileFile(t, raw) {
  const directory = mkdtempSync(join(tmpdir(), 'rust-browser-profile-test-'));
  t.after(() => rmSync(directory, { recursive: true, force: true }));
  const path = join(directory, 'explicit-profile.json');
  writeFileSync(path, raw);
  return path;
}

function browserContext() {
  const scripts = [];
  return {
    scripts,
    async addInitScript(callback, argument) { scripts.push({ callback, argument }); },
    newPage() {
      const calls = [], posted = Symbol('native postMessage result');
      class Worker {
        postMessage(...args) { calls.push({ worker: this, args }); return posted; }
      }
      const window = { Worker };
      const scope = vm.createContext({ window, structuredClone });
      for (const { callback, argument } of scripts) {
        vm.runInContext(`(${callback.toString()})(${JSON.stringify(argument)})`, scope);
      }
      return { window, calls, posted, worker: new window.Worker(),
        async evaluate(callback) { return structuredClone(vm.runInContext(`(${callback.toString()})()`, scope)); } };
    },
  };
}

function experiment() {
  return { path: '/explicit/test-profile.json', rawSha256: 'a'.repeat(64),
    profile: { solver_iterations: 4, additional_body_iterations: 1,
      nested: { unchanged: [true, 0.125, 'complete JSON'] }, arbitrary_existing_field: false } };
}

test('absent profile performs no file access, browser installation or page evaluation', async () => {
  assert.equal(loadBrowserProfileExperiment(), null);
  const context = browserContext();
  await installBrowserProfileExperiment(context, null);
  assert.equal(context.scripts.length, 0);
  assert.equal(await assertBrowserProfileExperiment({ evaluate() { throw new Error('Unexpected evaluation'); } }, null), null);
  assert.equal(browserProfileExperimentLabel(null), null);
  const page = context.newPage(), message = { type: 'initialize' };
  assert.equal(page.worker.postMessage(message), page.posted);
  assert.equal(page.calls[0].args[0], message);
  assert.equal(page.window.__lhAppliedProfiles, undefined);
});

test('explicit object JSON retains the full profile, absolute path and raw-byte SHA-256', t => {
  const raw = '{\n  "solver_iterations": 4, "additional_body_iterations": 1,\n  "nested": {"label":"entière", "values":[1,true,null]}\n}\n';
  const path = profileFile(t, raw), result = loadBrowserProfileExperiment(path);
  assert.deepEqual(result, { path: resolve(path), rawSha256: createHash('sha256').update(raw).digest('hex'), profile: JSON.parse(raw) });
  assert.match(browserProfileExperimentLabel(result), /EXPLICIT NUMERICAL-PROFILE TEST.*explicit-profile\.json.*sha256 /);
  const reformatted = profileFile(t, JSON.stringify(JSON.parse(raw)));
  assert.notEqual(loadBrowserProfileExperiment(reformatted).rawSha256, result.rawSha256);
});

test('malformed, nonobject and missing explicit paths are rejected', t => {
  assert.throws(() => loadBrowserProfileExperiment(''), /explicit profile JSON path/);
  assert.throws(() => loadBrowserProfileExperiment(null), /explicit profile JSON path/);
  for (const raw of ['{', '{"solver_iterations":4,}', 'not JSON']) {
    assert.throws(() => loadBrowserProfileExperiment(profileFile(t, raw)), /Invalid profile JSON/);
  }
  for (const raw of ['null', '[]', '[{}]', 'true', '4', '"profile"']) {
    assert.throws(() => loadBrowserProfileExperiment(profileFile(t, raw)), /must contain an object/);
  }
});

test('profile reads are bounded at 64 KiB of bytes, inclusive', t => {
  assert.equal(maximumBrowserProfileBytes, 65536);
  const atLimit = '{}' + ' '.repeat(maximumBrowserProfileBytes - 2);
  assert.deepEqual(loadBrowserProfileExperiment(profileFile(t, atLimit)).profile, {});
  assert.throws(() => loadBrowserProfileExperiment(profileFile(t, atLimit + ' ')), /must not exceed 64 KiB/);
  const multibyte = JSON.stringify({ text: 'é'.repeat(maximumBrowserProfileBytes / 2) });
  assert.ok(multibyte.length < maximumBrowserProfileBytes);
  assert.throws(() => loadBrowserProfileExperiment(profileFile(t, multibyte)), /must not exceed 64 KiB/);
});

test('only initialize receives a copied message and independent complete profile clone', async () => {
  const metadata = experiment(), context = browserContext();
  await installBrowserProfileExperiment(context, metadata);
  assert.equal(context.scripts.length, 1);
  const page = context.newPage();
  const initialProfile = { existing: 'untouched' }, transfer = [new ArrayBuffer(4)], options = { custom: true };
  const message = { type: 'initialize', profile: initialProfile, mode: 'protocol', generation: 1, floorEnabled: true };
  assert.equal(page.worker.postMessage(message, transfer, options), page.posted);
  const call = page.calls[0], forwarded = call.args[0];
  assert.equal(call.worker, page.worker);
  assert.notEqual(forwarded, message);
  assert.deepEqual(structuredClone(forwarded), { ...message, profile: metadata.profile });
  assert.deepEqual(Object.keys(forwarded).sort(), Object.keys(message).sort());
  assert.equal(call.args[1], transfer); assert.equal(call.args[2], options);
  assert.equal(message.profile, initialProfile);
  assert.notEqual(forwarded.profile, metadata.profile);
  forwarded.profile.nested.unchanged[0] = false;
  assert.equal(metadata.profile.nested.unchanged[0], true);
  assert.deepEqual(await assertBrowserProfileExperiment(page, metadata), [metadata]);
  assert.equal(page.window.__lhQualification, undefined);
  assert.equal(page.window.__lhGetUpTrial, undefined);
  assert.equal(page.window.__lhRust, undefined);
});

test('all other messages and every transfer argument pass through by identity', async () => {
  const context = browserContext(); await installBrowserProfileExperiment(context, experiment());
  const page = context.newPage();
  const messages = [undefined, null, {}, [], { type: 'commands', commands: [{ action: 'Initialize' }] },
    { type: 'advance', ticks: 4 }, { type: 'reset' }, { type: 'Initialize' }, { profile: { untouched: true } }];
  for (const message of messages) {
    const transfer = [new ArrayBuffer(8)], options = { transfer };
    assert.equal(page.worker.postMessage(message, transfer, options), page.posted);
    const call = page.calls.at(-1);
    assert.equal(call.worker, page.worker); assert.equal(call.args.length, 3);
    assert.equal(call.args[0], message); assert.equal(call.args[1], transfer); assert.equal(call.args[2], options);
  }
  assert.deepEqual(structuredClone(page.window.__lhAppliedProfiles), []);
});

test('the prototype hook composes with existing Worker tracing in either init-script order', async () => {
  for (const profileFirst of [true, false]) {
    const metadata = experiment(), context = browserContext();
    const trace = async () => context.addInitScript(() => {
      const NativeWorker = window.Worker;
      window.tracedMessages = [];
      window.Worker = class extends NativeWorker {
        postMessage(message, ...args) {
          window.tracedMessages.push(message);
          return super.postMessage(message, ...args);
        }
      };
    });
    if (profileFirst) await installBrowserProfileExperiment(context, metadata);
    await trace();
    if (!profileFirst) await installBrowserProfileExperiment(context, metadata);
    const page = context.newPage(), transfer = [new ArrayBuffer(8)];
    page.worker.postMessage({ type: 'initialize' }, transfer);
    const commands = { type: 'commands', commands: [{ action: 'Reset' }] };
    page.worker.postMessage(commands, transfer);
    assert.equal(page.window.tracedMessages.length, 2);
    assert.equal(page.calls.length, 2);
    assert.equal(page.calls[0].args[1], transfer);
    assert.equal(page.calls[1].args[0], commands);
    assert.equal(page.calls[1].args[1], transfer);
    assert.deepEqual(await assertBrowserProfileExperiment(page, metadata), [metadata]);
  }
});

test('complete JSON profile keys remain own data properties in the init script', async () => {
  const metadata = experiment();
  metadata.profile = JSON.parse('{"solver_iterations":4,"__proto__":{"preserved":true}}');
  const context = browserContext(); await installBrowserProfileExperiment(context, metadata);
  const page = context.newPage(); page.worker.postMessage({ type: 'initialize' });
  assert.ok(Object.hasOwn(page.calls[0].args[0].profile, '__proto__'));
  assert.deepEqual(await assertBrowserProfileExperiment(page, metadata), [metadata]);
});

test('exactly-once metadata is checked independently for manual and automatic pages', async () => {
  const metadata = experiment(), context = browserContext();
  await installBrowserProfileExperiment(context, metadata);
  for (const page of [context.newPage(), context.newPage()]) {
    await assert.rejects(assertBrowserProfileExperiment(page, metadata), /Exactly one intended numerical profile/);
    page.worker.postMessage({ type: 'initialize', mode: 'protocol' });
    assert.deepEqual(await assertBrowserProfileExperiment(page, metadata), [metadata]);
    page.worker.postMessage({ type: 'commands', commands: [] });
    assert.deepEqual(await assertBrowserProfileExperiment(page, metadata), [metadata]);
  }
  const duplicate = context.newPage();
  duplicate.worker.postMessage({ type: 'initialize' }); duplicate.worker.postMessage({ type: 'initialize' });
  await assert.rejects(assertBrowserProfileExperiment(duplicate, metadata), /Exactly one intended numerical profile/);
});

test('missing or changed path, hash and full-profile metadata fail validation', async () => {
  const metadata = experiment();
  await assert.rejects(assertBrowserProfileExperiment({ evaluate: async () => undefined }, metadata), /must record initialization/);
  for (const field of ['path', 'rawSha256', 'profile']) {
    const context = browserContext(); await installBrowserProfileExperiment(context, metadata);
    const page = context.newPage(); page.worker.postMessage({ type: 'initialize' });
    page.window.__lhAppliedProfiles[0][field] = field === 'profile' ? { solver_iterations: 4 } : 'changed';
    await assert.rejects(assertBrowserProfileExperiment(page, metadata), /complete explicit JSON and raw metadata/);
  }
});
