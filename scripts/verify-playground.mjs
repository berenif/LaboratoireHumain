import { createRequire } from "node:module";
import { mkdir, writeFile } from "node:fs/promises";
import { dirname, resolve } from "node:path";

const requireRuntime = createRequire(resolve(dirname(process.env.CODEX_MCP_NODE_PATH), "package.json"));
const { chromium } = requireRuntime("playwright");
const output = resolve("evidence/playground");
await mkdir(output, { recursive: true });
const browser = await chromium.launch({ channel: "msedge", headless: true, args: ["--enable-unsafe-swiftshader"] });
const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1 });
const errors = [];
page.on("pageerror", error => errors.push(error.message));
const samples = [];
const sample = async label => {
  const diagnostics = await page.evaluate(() => window.__EMBODIED_DEMO__.diagnostics());
  samples.push({ label, state: diagnostics.state, finite: diagnostics.finite, errors: diagnostics.errors,
    contacts: diagnostics.support, penetration: diagnostics.maxFloorPenetrationM });
  console.log(JSON.stringify(samples.at(-1)));
};
try {
  await page.goto(process.env.PLAYGROUND_URL ?? "http://127.0.0.1:5173/", { waitUntil: "domcontentloaded", timeout: 120000 });
  await page.waitForFunction(() => window.__EMBODIED_DEMO__?.ready(), null, { timeout: 120000 });
  await page.waitForTimeout(1800);
  await sample("arena");
  await page.screenshot({ path: resolve(output, "arena.png") });
  await page.getByRole("button", { name: "Focus body" }).click();
  const hand = await page.evaluate(() => window.__EMBODIED_DEMO__.regionPoint("rightHand"));
  await page.mouse.move(hand.x, hand.y);
  await page.mouse.down();
  await page.waitForTimeout(100);
  if (!await page.evaluate(() => window.__EMBODIED_DEMO__.diagnostics().activeGrab)) throw new Error("Body picking failed");
  await page.mouse.move(hand.x + 25, hand.y - 8, { steps: 5 });
  await page.mouse.up();
  await page.getByTestId("reset-button").click();
  for (const station of ["slope", "rubble", "beam", "stones", "wobble", "hurdles"]) {
    await page.getByTestId(`station-${station}`).click();
    await page.waitForTimeout(station === "wobble" ? 4000 : 1200);
    await sample(station);
    await page.screenshot({ path: resolve(output, `${station}.png`) });
  }
  await page.getByLabel("Course difficulty").selectOption("extreme");
  await page.getByTestId("station-wobble").click();
  await page.waitForTimeout(1500);
  await sample("extreme");
  await page.getByTestId("pause-toggle").click();
  await page.getByTestId("station-slope").click();
  if (!await page.getByTestId("pause-toggle").getAttribute("aria-pressed").then(value => value === "true")) throw new Error("Station switch lost pause state");
  await page.getByTestId("reset-button").click();
  await page.getByTestId("renderer-picker").click();
  await page.getByRole("option", { name: "Canvas 2D", exact: true }).click();
  await page.getByRole("button", { name: "Arena view" }).click();
  await page.screenshot({ path: resolve(output, "canvas2d.png") });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.getByRole("button", { name: "Focus body" }).click();
  await page.screenshot({ path: resolve(output, "mobile.png") });
  if (await page.evaluate(() => document.documentElement.scrollWidth > innerWidth)) throw new Error("Mobile layout overflows");
  await writeFile(resolve(output, "results.json"), JSON.stringify({ samples, errors }, null, 2));
  if (errors.length || samples.some(sample => !sample.finite || sample.errors.length)) throw new Error(JSON.stringify({ samples, errors }));
} finally { await browser.close(); }
