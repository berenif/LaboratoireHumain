import { createRequire } from "node:module";
import { mkdir } from "node:fs/promises";
import { dirname, resolve } from "node:path";

const runtimeNode = process.env.CODEX_MCP_NODE_PATH;
if (!runtimeNode) throw new Error("CODEX_MCP_NODE_PATH is required for bundled Playwright.");
const requireRuntime = createRequire(resolve(dirname(runtimeNode), "package.json"));
const { chromium } = requireRuntime("playwright");
const baseUrl = process.env.PROTOCOL_VISUAL_URL ?? "http://127.0.0.1:4184/";
const output = resolve(process.argv[2] ?? "evidence/protocol-visual");
await mkdir(output, { recursive: true });
const browser = await chromium.launch({ channel: "msedge", headless: true,
  args: ["--enable-unsafe-swiftshader"] });
const errors = [];
try {
  for (const [name, viewport] of [
    ["wide", { width: 1440, height: 900 }],
    ["narrow", { width: 390, height: 844 }],
  ]) {
    const context = await browser.newContext({ viewport, deviceScaleFactor: 1,
      reducedMotion: "reduce" });
    const page = await context.newPage();
    page.on("pageerror", error => errors.push(`${name}: ${error.message}`));
    await page.goto(baseUrl, { waitUntil: "domcontentloaded", timeout: 120000 });
    await page.locator('[data-simulation-ready="true"]').waitFor({ timeout: 120000 });
    const renderers = await page.locator('[data-testid="renderer-picker"] option').evaluateAll(options =>
      options.map(option => ({ value: option.value, disabled: option.disabled })));
    for (const renderer of ["webgl", "canvas2d"]) {
      if (renderers.find(option => option.value === renderer)?.disabled) continue;
      await page.locator('[data-testid="renderer-picker"]').selectOption(renderer);
      await page.locator(`main[data-renderer="${renderer}"]`).waitFor({ timeout: 30000 });
      await page.screenshot({ path: resolve(output, `${name}-${renderer}-initial.png`) });
      if (name === "wide") {
        await page.locator('[data-testid="strike-button"]').click();
        await page.locator('[data-testid="strike-count"]').filter({ hasText: "01" })
          .waitFor({ timeout: 120000 });
        await page.screenshot({ path: resolve(output, `${name}-${renderer}-impact.png`) });
        await page.locator('[data-testid="reset-button"]').click();
      }
    }
    const metrics = await page.evaluate(() => ({
      title: document.title,
      room: Boolean(document.querySelector(".protocol-mode")),
      button: document.querySelector('[data-testid="strike-button"]')?.textContent,
      horizontalOverflow: document.documentElement.scrollWidth > window.innerWidth,
      canvas: document.querySelector("canvas")?.getBoundingClientRect().toJSON(),
    }));
    process.stdout.write(`${name}: ${JSON.stringify(metrics)}\n`);
    await context.close();
  }
  if (errors.length) throw new Error(errors.join("\n"));
} finally {
  await browser.close();
}
