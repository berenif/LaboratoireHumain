import assert from 'node:assert/strict';
import { join } from 'node:path';
import { chromium, browserLaunchOptions } from './rust-browser-test.mjs';
import { validateRollbackBrowser } from './typescript-rollback-tools.mjs';

// Run against existing export bytes. Canvas fallback uses the browser's normal
// capability path; no application or simulation code is injected or replaced.
export async function checkTypescriptBrowser(base, output) {
  const report = {cases: [], passed: false};
  let browser;
  try {
    browser = await chromium.launch(browserLaunchOptions());
    report.browser = browser.version();
    for (const mode of ['protocol', 'playground']) for (const renderer of ['webgl', 'canvas2d']) {
      const context = await browser.newContext();
      try {
        if (renderer === 'canvas2d') await context.addInitScript(() => {
          const original = HTMLCanvasElement.prototype.getContext;
          HTMLCanvasElement.prototype.getContext = function(type, ...args) {
            return ['webgl', 'webgl2', 'experimental-webgl'].includes(type) ? null : original.call(this, type, ...args);
          };
        });
        const page = await context.newPage(), errors = [];
        page.on('pageerror', error => errors.push(String(error)));
        page.on('requestfailed', request => errors.push(`${request.url()}: ${request.failure()?.errorText}`));
        const url = new URL(base); url.searchParams.set('mode', mode);
        const response = await page.goto(url.href, {timeout: 60000});
        assert.ok(response?.ok(), 'Rollback entrypoint did not load');
        await page.waitForFunction(() => window.__EMBODIED_DEMO__?.ready(), null, {timeout: 90000});
        const first = await page.evaluate(() => window.__EMBODIED_DEMO__.diagnostics());
        assert.ok(first?.simulationReady && first.interactiveViewReady);
        assert.equal(await page.evaluate(() => window.__EMBODIED_DEMO__.renderer()), renderer);
        assert.equal(await page.locator('main.demo-shell').evaluate(element => element.classList.contains('protocol-mode')),
          mode === 'protocol', 'Rollback did not select the requested mode');
        await page.waitForFunction(tick => window.__EMBODIED_DEMO__.diagnostics()?.fixedSteps > tick,
          first.fixedSteps, {timeout: 15000});
        const final = await page.evaluate(() => window.__EMBODIED_DEMO__.diagnostics());
        assert.equal(final.finite, true); assert.deepEqual(final.errors, []); assert.deepEqual(errors, []);
        await page.screenshot({path: join(output, `${mode}-${renderer}.png`)});
        report.cases.push({mode, renderer, passed: true, finite: final.finite,
          firstFixedSteps: first.fixedSteps, finalFixedSteps: final.fixedSteps, errors});
      } finally { await context.close(); }
    }
    report.passed = true;
    validateRollbackBrowser(report);
  } catch (error) { report.passed = false; report.error = String(error); }
  finally { await browser?.close(); }
  return report;
}
