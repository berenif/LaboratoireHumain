import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { closeSync, fstatSync, openSync, readSync } from 'node:fs';
import { basename, resolve } from 'node:path';

export const maximumBrowserProfileBytes = 64 * 1024;

// Test harness only. An absent path leaves the ordinary application untouched.
// Keep the raw-byte hash: differently formatted JSON is different evidence.
export function loadBrowserProfileExperiment(path) {
  if (path === undefined) return null;
  assert.ok(typeof path === 'string' && path.length > 0, 'An explicit profile JSON path is required');
  const absolutePath = resolve(path), fd = openSync(absolutePath, 'r');
  let raw;
  try {
    assert.ok(fstatSync(fd).isFile(), 'Profile JSON must be a regular file');
    const buffer = Buffer.alloc(maximumBrowserProfileBytes + 1);
    let size = 0, count;
    do {
      count = readSync(fd, buffer, size, buffer.length - size, null);
      size += count;
    } while (count > 0 && size < buffer.length);
    assert.ok(size <= maximumBrowserProfileBytes, 'Profile JSON must not exceed 64 KiB');
    raw = buffer.subarray(0, size);
  } finally { closeSync(fd); }
  let profile;
  try { profile = JSON.parse(raw.toString('utf8')); }
  catch (error) { throw new Error(`Invalid profile JSON: ${error.message}`, { cause: error }); }
  assert.ok(profile !== null && typeof profile === 'object' && !Array.isArray(profile), 'Profile JSON must contain an object');
  return { path: absolutePath, rawSha256: createHash('sha256').update(raw).digest('hex'), profile };
}

export function browserProfileExperimentLabel(experiment) {
  return experiment ? `EXPLICIT NUMERICAL-PROFILE TEST | ${basename(experiment.path)} | sha256 ${experiment.rawSha256.slice(0, 12)}` : null;
}

export async function installBrowserProfileExperiment(context, experiment) {
  if (!experiment) return;
  await context.addInitScript(serialized => {
    // Parse data inside the page so valid JSON keys such as __proto__ remain
    // ordinary own properties when Playwright embeds the init-script argument.
    const metadata = JSON.parse(serialized);
    const original = window.Worker.prototype.postMessage;
    window.__lhAppliedProfiles = [];
    // Only replace the initialization's profile. Commands, state, observer flags,
    // other messages, and every postMessage transfer argument are untouched.
    window.Worker.prototype.postMessage = function (message, ...args) {
      if (message?.type !== 'initialize') return Reflect.apply(original, this, [message, ...args]);
      const forwarded = { ...message, profile: structuredClone(metadata.profile) };
      const result = Reflect.apply(original, this, [forwarded, ...args]);
      window.__lhAppliedProfiles.push({ path: metadata.path, rawSha256: metadata.rawSha256,
        profile: structuredClone(metadata.profile) });
      return result;
    };
  }, JSON.stringify(experiment));
}

// Run this on each evidence page, including a separately created automatic scene.
// Reinitialization must be visible as a failed exactly-once assertion, never hidden.
export async function assertBrowserProfileExperiment(page, experiment) {
  if (!experiment) return null;
  const applied = await page.evaluate(() => window.__lhAppliedProfiles);
  assert.ok(Array.isArray(applied), 'Explicit numerical-profile test must record initialization');
  assert.equal(applied.length, 1, 'Exactly one intended numerical profile must be applied per page');
  assert.deepEqual(applied[0], experiment, 'Applied numerical profile must match the complete explicit JSON and raw metadata');
  return applied;
}
