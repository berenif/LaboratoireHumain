import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';

// Formatting/transport verification only. Install manual mode before the bridge
// can schedule its first frame; never request a native physics integration.
const server = spawn(process.execPath, ['scripts/serve-rust-preview.mjs', 'rust/dist/app', '0'], {
  stdio: ['ignore', 'pipe', 'inherit'],
});
let browser;
try {
  const origin = await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error('Preview startup timed out')), 10000);
    server.stdout.on('data', data => {
      const url = String(data).match(/http:\/\/127\.0\.0\.1:\d+\//)?.[0];
      if (url) { clearTimeout(timeout); resolve(url); }
    });
    server.once('error', reject);
    server.once('exit', code => { clearTimeout(timeout); reject(new Error(`Preview exited ${code}`)); });
  });
  browser = await chromium.launch(browserLaunchOptions(true));
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  page.on('response', response => {
    if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`);
  });
  await page.addInitScript(() => {
    let api;
    Object.defineProperty(window, '__lhRust', {
      configurable: true,
      get: () => api,
      set(value) { api = value; api.setManual(true); },
    });
  });
  await page.goto(`${origin}?renderer=webgl2`);
  await page.waitForFunction(() => window.__lhRust?.diagnostics.ready
    && window.__lhRust.diagnostics.graphics.frames >= 3, null, { timeout: 60000 });
  const initial = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(initial.tick, 0);
  assert.equal(initial.time, 0);
  assert.equal(initial.maxSelfPenetrationM, 0);
  assert.equal(initial.maxFloorPenetrationM, 0);
  const metric = page.locator('#penetration-diagnostic');
  assert.equal(await metric.isVisible(), true);
  assert.match(await metric.innerText(), /body 0\.00 mm · ground 0\.00 mm/);

  // Synthetic UI event, not a worker state or a physical result.
  const rendered = await page.evaluate(async () => {
    const status = { ...window.__lhRust.diagnostics,
      maxSelfPenetrationM: 0.047, maxFloorPenetrationM: 0.12 };
    window.dispatchEvent(new CustomEvent('lh-status', { detail: JSON.stringify(status) }));
    await Promise.resolve();
    await Promise.resolve();
    return document.querySelector('#penetration-diagnostic').textContent;
  });
  assert.match(rendered, /body 47\.00 mm · ground 120\.00 mm/);
  const final = await page.evaluate(() => window.__lhRust.diagnostics);
  assert.equal(final.tick, 0);
  assert.equal(final.time, 0);
  assert.equal(final.maxSelfPenetrationM, 0);
  assert.equal(final.maxFloorPenetrationM, 0);
  assert.deepEqual(final.snapshot.segments, initial.snapshot.segments);
  assert.deepEqual(errors, []);
  console.log(JSON.stringify({ passed: true, physicsIntegrations: 0,
    initialTick: initial.tick, finalTick: final.tick,
    syntheticUiOnly: true, rendered, browserErrors: errors }, null, 2));
} finally {
  await browser?.close();
  server.kill();
}
