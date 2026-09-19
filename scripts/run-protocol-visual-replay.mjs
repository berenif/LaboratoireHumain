/**
 * Short protocol presentation review: one 3 s Rapier trace, four viewports,
 * normal videos and 0.25x copies. No recovery acceptance is inferred here.
 *
 * Start Vite, then run:
 *   node --import tsx scripts/run-protocol-visual-replay.mjs evidence/protocol-visual-replay http://127.0.0.1:4182
 */
import { createRequire } from "node:module";
import { createHash } from "node:crypto";
import { spawn } from "node:child_process";
import { copyFile, mkdir, readFile, writeFile } from "node:fs/promises";
import { dirname, resolve } from "node:path";
import { gzipSync } from "node:zlib";
import { createEmbodiedCharacter } from "../src/character/EmbodiedCharacter.ts";

const outputDirectory = resolve(process.argv[2] ?? "evidence/protocol-visual-replay");
const baseUrl = process.argv[3] ?? process.env.PROTOCOL_REPLAY_URL ?? "http://127.0.0.1:5173";
const runtimeRequire = createRequire(resolve(dirname(process.env.CODEX_MCP_NODE_PATH ?? process.execPath), "visual-replay.js"));
const { chromium } = runtimeRequire("playwright");
const fixedHz = 60;
const finalFrame = 3 * fixedHz;
const sourceFiles = [
  "src/character/EmbodiedCharacter.ts", "src/character/PhysicsStriker.ts",
  "src/character/DynamicRecovery.ts", "src/core/protocol.ts", "src/core/types.ts",
  "src/core/humanoid.ts", "src/core/geometry.ts", "src/scene/pose.ts",
  "src/scene/protocol-visuals.ts", "src/scene/webgl-view.ts",
  "src/scene/canvas2d-view.ts", "scripts/run-protocol-visual-replay.mjs",
];
const fingerprint = async () => Object.fromEntries(await Promise.all(sourceFiles.map(async file =>
  [file, createHash("sha256").update(await readFile(file)).digest("hex")])));
const deepFreeze = (value, seen = new WeakSet()) => {
  if (!value || typeof value !== "object" || seen.has(value)) return value;
  seen.add(value);
  for (const child of Object.values(value)) deepFreeze(child, seen);
  return Object.freeze(value);
};
const ffmpeg = args => new Promise((done, fail) => {
  const child = spawn(process.env.FFMPEG_PATH ?? "ffmpeg", args, { windowsHide: true, stdio: ["ignore", "ignore", "pipe"] });
  let stderr = "";
  child.stderr.on("data", chunk => { stderr = (stderr + chunk).slice(-12000); });
  child.on("error", fail);
  child.on("exit", code => code === 0 ? done(stderr) : fail(new Error(`ffmpeg exited ${code}: ${stderr}`)));
});

await mkdir(outputDirectory, { recursive: true });
const sourceBefore = await fingerprint();
const character = await createEmbodiedCharacter("webgl", { room: true });
const snapshots = [];
let strikeAccepted = false;
try {
  snapshots.push(deepFreeze(structuredClone(character.getSnapshot("webgl"))));
  strikeAccepted = character.requestStrike();
  for (let frame = 1; frame <= finalFrame; frame++) {
    character.fixedUpdate(1 / fixedHz, null);
    snapshots.push(deepFreeze(structuredClone(character.getSnapshot("webgl"))));
  }
} finally {
  character.dispose();
}
const traceJson = JSON.stringify({ schema: 1, fixedHz, strikeAccepted, snapshots });
const traceSha256 = createHash("sha256").update(traceJson).digest("hex");
await writeFile(resolve(outputDirectory, "protocol-3s-trace.json.gz"), gzipSync(traceJson));
const impactFrame = snapshots.findIndex(snapshot => (snapshot.striker?.impactId ?? 0) > 0);
const phases = [...new Set(snapshots.map(snapshot => snapshot.striker?.phase))];
const summary = {
  schema: 1, purpose: "Three-second impact and presentation smoke review; recovery is outside this window.",
  fixedHz, frames: snapshots.length, durationS: finalFrame / fixedHz,
  strikeAccepted, impactFrame: impactFrame < 0 ? null : impactFrame,
  impactTimeS: impactFrame < 0 ? null : impactFrame / fixedHz,
  phases, traceSha256, sourceBefore, views: [], browserErrors: [],
  allFinite: snapshots.every(snapshot => snapshot.diagnostics.finite),
  sameBodies: snapshots.every(snapshot => snapshot.segments.length === 25),
};
await writeFile(resolve(outputDirectory, "trace-summary.json"), JSON.stringify(summary, null, 2));
console.log(JSON.stringify({ stage: "trace", frames: snapshots.length, impactFrame: summary.impactFrame, traceSha256 }));

const pageHtml = `<!doctype html><html lang="fr"><head><meta charset="utf-8"><title>Protocole · rejeu visuel</title><style>
*{box-sizing:border-box}html,body{width:100%;height:100%;margin:0;overflow:hidden;background:#aebcb5;color:#fff;font:12px system-ui}
#host{position:absolute;inset:0}#host canvas{display:block;width:100%;height:100%}
#status{position:absolute;top:12px;left:12px;z-index:2;padding:7px 9px;background:#1c2925d9;border-radius:4px;font-variant-numeric:tabular-nums}
#marker{position:absolute;top:0;right:0;width:8px;height:8px;background:#000;z-index:3}
</style></head><body><div id="host"></div><div id="status"></div><div id="marker"></div></body></html>`;
const layouts = [
  { name: "wide", viewport: { width: 1440, height: 900 } },
  { name: "narrow", viewport: { width: 390, height: 844 } },
];
const browser = await chromium.launch({ channel: "msedge", headless: true, args: ["--enable-unsafe-swiftshader"] });
try {
  for (const { name, viewport } of layouts) for (const renderer of ["webgl", "canvas2d"]) {
    const id = `${name}-${renderer}`;
    const context = await browser.newContext({ viewport, deviceScaleFactor: 1,
      recordVideo: { dir: outputDirectory, size: viewport } });
    try {
      const page = await context.newPage();
      const video = page.video();
      page.on("pageerror", error => summary.browserErrors.push(`${id}: ${error.message}`));
      await page.route(`${baseUrl}/__protocol_visual_replay__`, route => route.fulfill({ contentType: "text/html", body: pageHtml }));
      await page.goto(`${baseUrl}/__protocol_visual_replay__`, { waitUntil: "domcontentloaded", timeout: 90000 });
      await page.evaluate(async ({ snapshots, renderer }) => {
        const deepFreeze = (value, seen = new WeakSet()) => {
          if (!value || typeof value !== "object" || seen.has(value)) return value;
          seen.add(value);
          for (const child of Object.values(value)) deepFreeze(child, seen);
          return Object.freeze(value);
        };
        deepFreeze(snapshots);
        const [{ createPoseView, createSharedCamera }, { PROTOCOL_ARENA_CAMERA }] = await Promise.all([
          import("/src/scene/index.ts"), import("/src/scene/protocol-visuals.ts")]);
        const host = document.querySelector("#host");
        const camera = createSharedCamera(PROTOCOL_ARENA_CAMERA);
        const view = createPoseView(renderer, { camera });
        view.mount(host); view.resize(host.clientWidth, host.clientHeight, 1);
        window.__PROTOCOL_REPLAY__ = { snapshots, view, frame: 0, done: false, timestamps: [], present(frame, alpha = 1) {
          const current = snapshots[frame], previous = snapshots[Math.max(0, frame - 1)];
          view.setSnapshot(previous, current, alpha); view.render();
          document.querySelector("#status").textContent = `${renderer} · ${frame} / 180 · ${(frame / 60).toFixed(2)} s · ${current.striker?.phase ?? "—"} · impact ${current.striker?.impactId ?? 0}`;
          this.frame = frame;
        } };
        window.__PROTOCOL_REPLAY__.present(0);
      }, { snapshots, renderer });
      for (const [label, frame] of [["initial", 0], ["contact", Math.max(0, impactFrame)], ["final", finalFrame]]) {
        await page.evaluate(frame => window.__PROTOCOL_REPLAY__.present(frame), frame);
        await page.screenshot({ path: resolve(outputDirectory, `${id}-${label}.png`) });
      }
      await page.evaluate(() => window.__PROTOCOL_REPLAY__.present(0));
      await page.evaluate(() => { document.querySelector("#marker").style.background = "#fff"; });
      await page.waitForTimeout(350);
      await page.evaluate(() => {
        document.querySelector("#marker").style.background = "#000";
        const replay = window.__PROTOCOL_REPLAY__, start = performance.now();
        const tick = now => {
          const elapsed = Math.max(0, (now - start) / 1000), raw = elapsed * 60;
          const frame = Math.min(180, Math.floor(raw));
          replay.present(frame, raw - Math.floor(raw));
          replay.timestamps.push({ elapsed, frame });
          if (elapsed >= 3.2) replay.done = true;
          else requestAnimationFrame(tick);
        };
        requestAnimationFrame(tick);
      });
      await page.waitForFunction(() => window.__PROTOCOL_REPLAY__.done, null, { timeout: 30000 });
      const playback = await page.evaluate(() => {
        const frames = window.__PROTOCOL_REPLAY__.timestamps.map(entry => entry.frame);
        return { finalFrame: window.__PROTOCOL_REPLAY__.frame,
          sampledFrames: frames.length,
          uniqueFrames: new Set(frames).size,
          largestFrameJump: Math.max(0, ...frames.slice(1).map((frame, index) => frame - frames[index])),
        };
      });
      await page.close();
      await context.close();
      const nativeVideo = resolve(outputDirectory, `${id}-native.webm`);
      await copyFile(await video.path(), nativeVideo);
      const markerLog = await ffmpeg(["-hide_banner", "-loglevel", "info", "-i", nativeVideo,
        "-vf", `crop=8:8:${viewport.width - 8}:0,blackdetect=d=0.1:pix_th=0.1`, "-an", "-f", "null", "-"]);
      const starts = [...markerLog.matchAll(/black_start:([0-9.]+)/g)].map(match => Number(match[1]));
      if (!starts.length) throw new Error(`${id}: video synchronization marker missing`);
      const offset = starts.at(-1);
      const normalVideo = resolve(outputDirectory, `${id}-normal.mp4`);
      const slowVideo = resolve(outputDirectory, `${id}-quarter-speed.mp4`);
      await ffmpeg(["-hide_banner", "-loglevel", "error", "-y", "-ss", String(offset), "-i", nativeVideo,
        "-t", "3.25", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "21", "-movflags", "+faststart", normalVideo]);
      await ffmpeg(["-hide_banner", "-loglevel", "error", "-y", "-i", normalVideo,
        "-vf", "setpts=4*PTS", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "21", "-movflags", "+faststart", slowVideo]);
      summary.views.push({ id, viewport, renderer, playback, normalVideo, slowVideo,
        initialImage: `${id}-initial.png`, contactImage: `${id}-contact.png`, finalImage: `${id}-final.png` });
      console.log(JSON.stringify({ stage: "rendered", id, playback, normalVideo, slowVideo }));
    } finally {
      await context.close().catch(() => undefined);
    }
  }
} finally {
  await browser.close();
  summary.sourceAfter = await fingerprint();
  summary.sourcesUnchanged = JSON.stringify(sourceBefore) === JSON.stringify(summary.sourceAfter);
  await writeFile(resolve(outputDirectory, "visual-summary.json"), JSON.stringify(summary, null, 2));
}
if (!summary.sourcesUnchanged || summary.browserErrors.length || !summary.strikeAccepted || summary.impactFrame === null || !summary.allFinite || !summary.sameBodies) {
  process.exitCode = 1;
}
