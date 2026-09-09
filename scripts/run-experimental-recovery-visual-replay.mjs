/**
 * Deterministic recovery presentation review.
 * Run with the bundled Node runtime: node --import tsx scripts/run-recovery-visual-replay.mjs [output-directory] [fixture-id,...]
 * A local Vite server must be available at RECOVERY_REPLAY_URL (default http://127.0.0.1:5173).
 * The Rapier trace runs once at 60 Hz. Both view adapters replay the same immutable snapshots;
 * slow playback changes video timestamps only and cannot affect the physical trajectory.
 */
import { createRequire } from "node:module";
import { mkdir, writeFile, readFile, copyFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import { dirname, resolve } from "node:path";
import { spawn } from "node:child_process";
import { gzipSync } from "node:zlib";
import { createEmbodiedCharacter } from "../src/character/EmbodiedCharacter.ts";
import { SEGMENTS, HUMAN_PROPORTIONS } from "../src/core/humanoid.ts";
import { RECOVERY_POSE_FIXTURES, seedRecoveryFixture } from "./recovery-fixtures.ts";
import { DynamicRecovery } from "./experimental-load-recovery.ts";

const outputDirectory = resolve(process.argv[2] ?? "evidence/recovery-20260908/representative");
const requested = process.argv[3]?.split(",").filter(Boolean);
const baseUrl = process.env.RECOVERY_REPLAY_URL ?? "http://127.0.0.1:5173";
const runtimeNode = process.env.CODEX_MCP_NODE_PATH;
if (!runtimeNode) throw new Error("CODEX_MCP_NODE_PATH must identify the bundled Node runtime containing Playwright.");
const requireRuntime = createRequire(resolve(dirname(runtimeNode), "package.json"));
const { chromium } = requireRuntime("playwright");
const totalMass = SEGMENTS.reduce((sum, segment) => sum + segment.massKg, 0);
const mass = Object.fromEntries(SEGMENTS.map(segment => [segment.id, segment.massKg]));
const sourceFiles = ["src/character/EmbodiedCharacter.ts", "src/character/BalanceController.ts", "scripts/experimental-load-recovery.ts", "src/character/recovery-support.ts", "src/character/recovery-motors.ts", "src/character/pose.ts", "src/core/humanoid.ts", "src/core/types.ts", "src/scene/canvas2d-view.ts", "src/scene/webgl-view.ts", "scripts/recovery-fixtures.ts", "scripts/run-experimental-recovery-visual-replay.mjs"];
const fingerprint = async () => Object.fromEntries(await Promise.all(sourceFiles.map(async file => [file, createHash("sha256").update(await readFile(file)).digest("hex")])));
const fixtures = requested
  ? requested.map(id => { const fixture = RECOVERY_POSE_FIXTURES.find(item => item.id === id); if (!fixture) throw new Error(`Unknown recovery fixture: ${id}`); return fixture; })
  : ["prone", "supine", "side", "half-kneel", "crouch"].map(pose => RECOVERY_POSE_FIXTURES.find(item => item.pose === pose)).filter(Boolean);
if (!fixtures.length) throw new Error("No recovery fixtures selected.");
await mkdir(outputDirectory, { recursive: true });
const sourceBefore = await fingerprint();
await writeFile(resolve(outputDirectory, "source-before.json"), JSON.stringify(sourceBefore, null, 2));
const evidence = { schema: 1, fixedHz: 60, normalSpeed: 1, slowSpeed: .25, sourceBefore, scenarios: [], browserErrors: [] };
const dot = (a, b) => a.x*b.x + a.y*b.y + a.z*b.z;
const sub = (a, b) => ({ x: a.x-b.x, y: a.y-b.y, z: a.z-b.z });
const magnitude = vector => Math.hypot(vector.x, vector.y, vector.z);
const distance = (a, b) => magnitude(sub(a, b));
const horizontalDistance = (a, b) => Math.hypot(a.x-b.x, a.z-b.z);
const rotate = (q, v) => {
  const tx = 2*(q.y*v.z-q.z*v.y), ty = 2*(q.z*v.x-q.x*v.z), tz = 2*(q.x*v.y-q.y*v.x);
  return { x: v.x+q.w*tx+q.y*tz-q.z*ty, y: v.y+q.w*ty+q.z*tx-q.x*tz, z: v.z+q.w*tz+q.x*ty-q.y*tx };
};
function physicalMetrics(snapshot, previous, supportEpisodes) {
  const segments = Object.fromEntries(snapshot.segments.map(segment => [segment.id, segment]));
  const weighted = field => Object.fromEntries(["x", "y", "z"].map(axis => [axis, snapshot.segments.reduce((sum, segment) => sum+mass[segment.id]*segment[field][axis], 0)/totalMass]));
  const com = weighted("position"), comVelocity = weighted("linearVelocity");
  const projectedCom = { x: com.x + .15*comVelocity.x, y: 0, z: com.z + .15*comVelocity.z };
  const recovery = snapshot.diagnostics.recovery;
  const loaded = recovery.contacts.filter(contact => contact.loadBearing);
  const feet = {};
  for (const side of ["left", "right"]) {
    const foot = segments[`${side}Foot`], thigh = segments[`${side}Thigh`], shin = segments[`${side}Shin`];
    const id = foot.id;
    const contact = loaded.find(item => item.segment === id);
    if (contact && !supportEpisodes.has(id)) supportEpisodes.set(id, { position: foot.position, sequence: snapshot.sequence });
    if (!contact) supportEpisodes.delete(id);
    const episode = supportEpisodes.get(id);
    const thighAxis = rotate(thigh.rotation, { x: 0, y: -1, z: 0 });
    const shinAxis = rotate(shin.rotation, { x: 0, y: -1, z: 0 });
    const bendRadians = Math.acos(Math.min(1, Math.max(-1, dot(thighAxis, shinAxis))));
    const priorFoot = previous?.segments.find(segment => segment.id === id);
    feet[side] = { position: foot.position, forceN: contact?.forceN ?? 0, loaded: Boolean(contact),
      continuousPlantDriftM: episode ? horizontalDistance(foot.position, episode.position) : 0,
      continuousPlantTimeS: episode ? (snapshot.sequence-episode.sequence)/60 : 0,
      speedMps: priorFoot ? distance(foot.position, priorFoot.position)*60 : 0,
      kneeBendRadians: bendRadians,
      kneeForward: dot(sub(thighAxis, shinAxis), rotate(snapshot.rootRotation, { x: 0, y: 0, z: 1 })),
    };
  }
  const loadedTotalN = loaded.reduce((sum, contact) => sum+contact.forceN, 0);
  return { com, comVelocity, projectedCom, feet, loadedSegments: loaded.map(contact => contact.segment), loadedTotalN,
    pelvisHeightM: segments.pelvis.position.y, torsoUp: rotate(segments.torso.rotation, { x: 0, y: 1, z: 0 }).y,
    leftFootLoadShare: loadedTotalN > 0 ? feet.left.forceN/loadedTotalN : 0,
    rightFootLoadShare: loadedTotalN > 0 ? feet.right.forceN/loadedTotalN : 0 };
}
async function generateTrace(fixture) {
  const character = await createEmbodiedCharacter("webgl", { heading: fixture.heading });
  try {
    seedRecoveryFixture(character, fixture);
    const recovery=new DynamicRecovery({feedforwardScale:1,balancedRise:true,groundBudget:true,smoothTransfer:true,fastTrail:true,clearanceArc:true,retryToe:true,uprightRise:true,kneelHeight:.92,rootAdvance:.15});
    recovery.reset(fixture.heading,{x:0,y:0,z:0});character.recovery=recovery;
    const snapshots = [], measurements = [], phaseEntries = [], supportEpisodes = new Map();
    let previous = null, stableFrame = null;
    for (let frame = 0; frame <= 25*60; frame++) {
      const snapshot = character.getSnapshot("webgl");
      const metrics = physicalMetrics(snapshot, previous, supportEpisodes);
      snapshots.push(snapshot); measurements.push(metrics);
      if (!previous || snapshot.diagnostics.recovery.phase !== previous.diagnostics.recovery.phase || snapshot.diagnostics.recovery.transferStage !== previous.diagnostics.recovery.transferStage) {
        phaseEntries.push({ frame, timeS: frame/60, state: snapshot.state, recovery: snapshot.diagnostics.recovery, metrics });
      }
      if (snapshot.diagnostics.authority === "character-motor" && snapshot.state === "upright" && frame > 0 && stableFrame === null) stableFrame = frame;
      if (stableFrame !== null && frame >= stableFrame+60) break;
      previous = snapshot;
      character.fixedUpdate(1/60, null);
    }
    const active = snapshots.filter(snapshot => snapshot.diagnostics.authority === "ragdoll");
    const summary = { fixture, frames: snapshots.length, durationS: (snapshots.length-1)/60, recovered: stableFrame !== null,
      recoveryTimeS: stableFrame === null ? null : stableFrame/60,
      selectedRoutes: [...new Set(snapshots.map(snapshot => snapshot.diagnostics.recovery.route).filter(route => route && route !== "none"))],
      maxJointSeparationM: Math.max(...snapshots.map(snapshot => snapshot.diagnostics.maxJointSeparationM)),
      maxFloorPenetrationM: Math.max(...snapshots.map(snapshot => snapshot.diagnostics.maxFloorPenetrationM)),
      maxUpwardAssistanceN: Math.max(0, ...active.map(snapshot => snapshot.diagnostics.recovery.assistanceForce.y)),
      maxRollingUpwardAssistanceN: Math.max(0, ...active.filter(snapshot => snapshot.diagnostics.recovery.phase === "roll").map(snapshot => snapshot.diagnostics.recovery.assistanceForce.y)),
      maxPelvisAssistanceTorqueNm: Math.max(0, ...active.map(snapshot => magnitude(snapshot.diagnostics.recovery.assistanceTorque))),
      maxMotorTorqueNm: Math.max(0, ...active.map(snapshot => snapshot.diagnostics.recovery.maxMotorTorqueNm)),
      maxContinuousFootDriftM: Math.max(0, ...measurements.flatMap(metrics => [metrics.feet.left.continuousPlantDriftM, metrics.feet.right.continuousPlantDriftM])),
      maxCapturedPlantDriftM: Math.max(0, ...active.flatMap(snapshot => snapshot.diagnostics.recovery.plantedTargets.map(plant => plant.driftM))),
      allFinite: snapshots.every(snapshot => snapshot.diagnostics.finite),
      assistanceWithinLimits: active.every(snapshot => snapshot.diagnostics.recovery.assistanceForce.y <= .2*totalMass*9.81+1e-6 && magnitude(snapshot.diagnostics.recovery.assistanceTorque) <= 60+1e-6),
      phaseEntries,
    };
    return { schema: 1, fixture, fixedHz: 60, snapshots, measurements, summary };
  } finally { character.dispose(); }
}
function runFFmpeg(args) {
  return new Promise((resolvePromise, reject) => {
    const child = spawn(process.env.FFMPEG_PATH ?? "ffmpeg", args, { windowsHide: true, stdio: ["ignore", "ignore", "pipe"] });
    let stderr = "";
    child.stderr.on("data", chunk => { stderr = (stderr+chunk).slice(-6000); });
    child.on("error", reject);
    child.on("exit", code => code === 0 ? resolvePromise(stderr) : reject(new Error(`ffmpeg exited ${code}: ${stderr}`)));
  });
}
const browser = await chromium.launch({ channel: "msedge", headless: true, args: ["--enable-unsafe-swiftshader"] });
try {
  for (const fixture of fixtures) {
    console.log(JSON.stringify({ stage: "trace", fixture: fixture.id }));
    const trace = await generateTrace(fixture);
    await writeFile(resolve(outputDirectory, `${fixture.id}-trace.json.gz`), gzipSync(JSON.stringify(trace)));
    const temporaryVideoDirectory = resolve(process.env.TEMP ?? process.env.TMP ?? outputDirectory, "codex-recovery-replay", fixture.id);
    await mkdir(temporaryVideoDirectory, { recursive: true });
    const context = await browser.newContext({ viewport: { width: 1440, height: 1080 }, deviceScaleFactor: 1, recordVideo: { dir: temporaryVideoDirectory, size: { width: 1440, height: 1080 } } });
    const page = await context.newPage();
    const video = page.video();
    page.on("pageerror", error => evidence.browserErrors.push(`${fixture.id}: ${error.message}`));
    await page.route(`${baseUrl}/__recovery_visual_replay__`, route => route.fulfill({ contentType: "text/html", body: `<!doctype html><html><head><title>Recovery review</title><style>*{box-sizing:border-box}body{margin:0;background:#101820;color:#eaf3fa;font:15px system-ui}header{height:72px;padding:10px 16px}h1{margin:0;font-size:19px}#status{margin-top:6px;font-variant-numeric:tabular-nums}.grid{height:928px;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:2px}.panel{position:relative;min-height:0}.label{position:absolute;top:9px;left:12px;z-index:1;background:#101820c9;padding:5px 8px;border-radius:4px}.host{width:100%;height:100%}footer{height:80px;padding:9px 16px;line-height:1.5;font-size:13px;color:#b9c8d4}canvas{display:block}</style></head><body><div id="recording-marker" style="position:fixed;left:0;top:0;width:8px;height:8px;background:#000;z-index:99"></div><header><h1 id="title"></h1><div id="status"></div></header><main class="grid"></main><footer>Same 60 Hz Rapier trajectory in all four views. Left: side. Right: three-quarter. Top: WebGL. Bottom: Canvas2D.<br>Review foot slip, lift source, knee/elbow direction, leading-leg weight transfer, support release, and transition continuity.</footer></body></html>` }));
    try {
      await page.goto(`${baseUrl}/__recovery_visual_replay__`, { waitUntil: "domcontentloaded", timeout: 90000 });
      await page.evaluate(async ({ trace, proportions }) => {
        const { createPoseView, createSharedCamera } = await import("/src/scene/index.ts");
        const root = trace.snapshots[0].segments.find(segment => segment.id === "pelvis").position;
        const heading = trace.fixture.heading ?? 0;
        const headingOffset = (x, z) => ({ x: root.x+Math.cos(heading)*x+Math.sin(heading)*z, y: 1.9, z: root.z-Math.sin(heading)*x+Math.cos(heading)*z });
        const target = { x: root.x, y: proportions.totalHeightM*.42, z: root.z };
        const views = [];
        for (const renderer of ["webgl", "canvas2d"]) for (const angle of ["side", "three-quarter"]) {
          const panel = document.createElement("section"); panel.className = "panel";
          const label = document.createElement("span"); label.className = "label"; label.textContent = `${renderer === "webgl" ? "WebGL" : "Canvas2D"} · ${angle}`;
          const host = document.createElement("div"); host.className = "host";
          panel.append(label, host); document.querySelector(".grid").append(panel);
          const camera = createSharedCamera({ position: angle === "side" ? headingOffset(4.3, 0) : headingOffset(3.0, 3.8), target, viewportWidth: host.clientWidth, viewportHeight: host.clientHeight });
          const view = createPoseView(renderer, { camera }); view.mount(host); view.resize(host.clientWidth, host.clientHeight, 1);
          views.push({ view, camera, renderer, angle });
        }
        document.querySelector("#title").textContent = `${trace.fixture.id} · 1× normal speed · deterministic recovery`;
        window.__RECOVERY_REPLAY__ = { trace, views, done: false, frame: 0, timestamps: [], present(frame, alpha = 0) {
          const previous = trace.snapshots[Math.max(0, frame-1)], current = trace.snapshots[frame];
          const r = current.diagnostics.recovery, metrics = trace.measurements[frame];
          for (const { view } of views) { view.setSnapshot(previous, current, alpha); view.render(); }
          document.querySelector("#status").textContent = `${(frame/60).toFixed(2)} s · ${r.route ?? "none"} · ${r.phase}/${r.transferStage ?? "none"} · lead ${r.leadingSide ?? "—"} · margin ${Number.isFinite(r.supportMarginM) ? r.supportMarginM.toFixed(3) : "—"} m · foot loads L ${Math.round(metrics.feet.left.forceN)} / R ${Math.round(metrics.feet.right.forceN)} N · lift ${r.assistanceForce.y.toFixed(1)} N`;
          this.frame = frame;
        } };
        window.__RECOVERY_REPLAY__.present(0, 1);
      }, { trace, proportions: HUMAN_PROPORTIONS });
      await page.screenshot({ path: resolve(outputDirectory, `${fixture.id}-initial.png`) });
      const recordingStartedMs = Date.now();
      await page.evaluate(() => { document.querySelector("#recording-marker").style.background = "#fff"; });
      await page.waitForTimeout(350);
      await page.evaluate(() => {
        const replay = window.__RECOVERY_REPLAY__;
        document.querySelector("#recording-marker").style.background = "#000";
        const start = performance.now();
        const tick = now => {
          const elapsed = Math.max(0, (now-start)/1000);
          const rawFrame = elapsed*60;
          const frame = Math.min(replay.trace.snapshots.length-1, Math.floor(rawFrame));
          replay.present(frame, rawFrame-Math.floor(rawFrame));
          replay.timestamps.push({ elapsed, frame });
          if (elapsed >= (replay.trace.snapshots.length-1)/60+.6) replay.done = true;
          else requestAnimationFrame(tick);
        };
        requestAnimationFrame(tick);
      });
      await page.waitForFunction(() => window.__RECOVERY_REPLAY__.done, null, { timeout: 60000 });
      const playback = await page.evaluate(() => ({ frame: window.__RECOVERY_REPLAY__.frame, timestamps: window.__RECOVERY_REPLAY__.timestamps }));
      await page.screenshot({ path: resolve(outputDirectory, `${fixture.id}-final.png`) });
      for (const [index, entry] of trace.summary.phaseEntries.entries()) {
        await page.evaluate(frame => window.__RECOVERY_REPLAY__.present(frame, 1), entry.frame);
        await page.screenshot({ path: resolve(outputDirectory, `${fixture.id}-entry-${String(index).padStart(2, "0")}-${entry.recovery.phase}-${entry.recovery.transferStage ?? "none"}.png`) });
      }
      await page.close();
      await context.close();
      const videoSource = await video.path();
      const nativeVideo = resolve(outputDirectory, `${fixture.id}-native.webm`);
      await copyFile(videoSource, nativeVideo);
      const markerLog = await runFFmpeg(["-hide_banner", "-loglevel", "info", "-i", nativeVideo, "-vf", "crop=8:8:0:0,blackdetect=d=0.1:pix_th=0.1", "-an", "-f", "null", "-"]);
      const markerStarts = [...markerLog.matchAll(/black_start:([0-9.]+)/g)].map(match => Number(match[1]));
      if (!markerStarts.length) throw new Error("Video synchronization marker was not recorded.");
      const playbackVideoOffsetS = markerStarts.at(-1);
      const normalVideo = resolve(outputDirectory, `${fixture.id}-normal.mp4`);
      await runFFmpeg(["-hide_banner", "-loglevel", "error", "-y", "-ss", String(playbackVideoOffsetS), "-i", nativeVideo, "-t", String(trace.summary.durationS+.6), "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-movflags", "+faststart", normalVideo]);
      const slowVideo = resolve(outputDirectory, `${fixture.id}-slow.mp4`);
      await runFFmpeg(["-hide_banner", "-loglevel", "error", "-y", "-i", normalVideo, "-vf", "setpts=4*PTS", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-movflags", "+faststart", slowVideo]);
      const result = { ...trace.summary, recordingWallTimeS: (Date.now()-recordingStartedMs)/1000, normalVideo, slowVideo, playbackVideoOffsetS, playback };
      evidence.scenarios.push(result);
      await writeFile(resolve(outputDirectory, `${fixture.id}-summary.json`), JSON.stringify(result, null, 2));
      console.log(JSON.stringify({ stage: "recorded", fixture: fixture.id, recovered: trace.summary.recovered, durationS: trace.summary.durationS, routes: trace.summary.selectedRoutes, normalVideo, slowVideo }));
    } finally { await context.close().catch(() => undefined); }
  }
} finally {
  await browser.close();
  const sourceAfter = await fingerprint();
  evidence.sourceAfter = sourceAfter;
  evidence.sourcesUnchanged = JSON.stringify(sourceBefore) === JSON.stringify(sourceAfter);
  await writeFile(resolve(outputDirectory, "source-after.json"), JSON.stringify(sourceAfter, null, 2));
  await writeFile(resolve(outputDirectory, "visual-summary.json"), JSON.stringify(evidence, null, 2));
}
if (!evidence.sourcesUnchanged) throw new Error("Controller or presentation source changed while recording; rerun before acceptance.");
if (evidence.browserErrors.length) throw new Error(`Browser errors: ${evidence.browserErrors.join("; ")}`);
if (evidence.scenarios.some(scenario => !scenario.recovered || !scenario.assistanceWithinLimits || scenario.maxRollingUpwardAssistanceN > 1e-6)) process.exitCode = 1;


