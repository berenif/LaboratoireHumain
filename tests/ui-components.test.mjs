import assert from "node:assert/strict";
import { readdir, readFile } from "node:fs/promises";
import path from "node:path";
import test, { after } from "node:test";
import { fileURLToPath } from "node:url";

import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { createServer } from "vite";

const root = fileURLToPath(new URL("..", import.meta.url));
const vite = await createServer({
  appType: "custom",
  configFile: false,
  cacheDir: path.join(root, ".sites-runtime", "vite-tests"),
  root,
  resolve: { alias: { "@": root } },
  server: { middlewareMode: true },
});

after(async () => {
  await vite.close();
});

async function readCssTree(directory) {
  const entries = await readdir(directory, { withFileTypes: true });
  const contents = await Promise.all(
    entries.map(async (entry) => {
      const entryPath = path.join(directory, entry.name);
      if (entry.isDirectory()) {
        return readCssTree(entryPath);
      }
      return entry.name.endsWith(".css") ? readFile(entryPath, "utf8") : "";
    }),
  );
  return contents.join("\n");
}

test("emits the catalog's animation and scrolling utilities", async () => {
  const css = await readCssTree(path.join(root, "dist"));

  assert.match(css, /--tw-enter-opacity/);
  assert.match(css, /scrollbar-width:\s*thin/);
  assert.match(css, /scrollbar-width:\s*none/);
  assert.match(css, /scrollbar-gutter:\s*stable/);
  assert.match(css, /scroll-fade-reveal-b/);
  assert.match(css, /mask-image:/);
  assert.match(css, /tw-shimmer/);
  assert.match(css, /prefers-reduced-motion:\s*reduce/);
});

test("forwards progress semantics to the primitive", async () => {
  const { Progress } = await vite.ssrLoadModule("/components/ui/progress.tsx");
  const html = renderToStaticMarkup(React.createElement(Progress, { value: 37 }));

  assert.match(html, /aria-valuenow="37"/);
  assert.match(html, /aria-valuetext="37%"/);
  assert.match(html, /data-state="loading"/);
});

test("emits chart themes for the starter's media dark mode", async () => {
  const { ChartStyle } = await vite.ssrLoadModule("/components/ui/chart.tsx");
  const html = renderToStaticMarkup(
    React.createElement(ChartStyle, {
      id: "contract",
      config: {
        latency: { theme: { light: "#ffffff", dark: "#000000" } },
      },
    }),
  );

  assert.match(html, /\[data-chart=contract\]/);
  assert.match(html, /@media \(prefers-color-scheme: dark\)/);
  assert.doesNotMatch(html, /\.dark/);
});

test("renders sidebar skeletons deterministically", async () => {
  const { SidebarMenuSkeleton } = await vite.ssrLoadModule(
    "/components/ui/sidebar.tsx",
  );
  const first = renderToStaticMarkup(React.createElement(SidebarMenuSkeleton));
  const second = renderToStaticMarkup(React.createElement(SidebarMenuSkeleton));

  assert.equal(first, second);
  assert.match(first, /--skeleton-width:70%/);
});

test("moves the shared camera without touching simulation state and resets exactly", async () => {
  const [{ SharedCameraProjection }, { OrbitCameraController }] = await Promise.all([
    vite.ssrLoadModule("/src/scene/camera.ts"),
    vite.ssrLoadModule("/src/scene/camera-controls.ts"),
  ]);
  const camera = new SharedCameraProjection();
  camera.setViewport(1280, 720);
  const initial = camera.getState();
  const controls = new OrbitCameraController(camera);

  controls.orbit(84, -26);
  controls.pan(35, -18);
  controls.zoom(0.78);

  const moved = camera.getState();
  assert.notDeepEqual(moved.position, initial.position);
  assert.notDeepEqual(moved.target, initial.target);
  assert.ok(Object.values(moved.position).every(Number.isFinite));
  assert.ok(Object.values(moved.target).every(Number.isFinite));
  assert.ok(Number.isFinite(camera.project({ x: 0, y: 1, z: 0 }).depth));

  controls.reset();
  const reset = camera.getState();
  assert.deepEqual(reset.position, initial.position);
  assert.deepEqual(reset.target, initial.target);
  assert.equal(reset.viewportWidth, 1280);
  assert.equal(reset.viewportHeight, 720);
});

test("renders only the compact renderer and simulation controls", async () => {
  const { ControlPanel } = await vite.ssrLoadModule("/src/ui/ControlPanel.tsx");
  const html = renderToStaticMarkup(
    React.createElement(ControlPanel, {
      diagnostics: null,
      renderer: "canvas2d",
      paused: false,
      webglAvailable: false,
      canvas2dAvailable: true,
      onRendererChange() {},
      onPauseToggle() {},
      onReset() {},
    }),
  );

  assert.match(html, /data-testid="renderer-picker"/);
  assert.match(html, /data-testid="pause-toggle"/);
  assert.match(html, /data-testid="reset-button"/);
  assert.match(html, /drag empty space to orbit/i);
  assert.doesNotMatch(html, /Motion lab|Direct body control|Pull effort/);
});


test("body lockout status uses availability while preserving all simulation controls", async () => {
  const { ControlPanel } = await vite.ssrLoadModule("/src/ui/ControlPanel.tsx");
  for (const state of ["falling", "fallen", "recovering"]) {
    const html = renderToStaticMarkup(React.createElement(ControlPanel, {
      diagnostics: {
        simulationReady: true, interactiveViewReady: true, state, bodyInputAvailable: false,
        authority: "ragdoll", activeGrab: false, selectedRegion: null, stepCount: 1, appliedGrabForceN: 0, errors: [],
      },
      renderer: "canvas2d", paused: false, onRendererChange() {}, onPauseToggle() {}, onReset() {},
    }));
    assert.match(html, /data-testid="body-input-availability">Unavailable/);
    assert.doesNotMatch(html, /Drag body/);
    assert.match(html, state === "recovering" ? /Getting up/ : /Protecting the fall/);
    assert.match(html, /data-testid="renderer-picker"/);
    assert.match(html, /data-testid="pause-toggle"/);
    assert.match(html, /data-testid="reset-button"/);
    assert.doesNotMatch(html, /<button[^>]*\sdisabled(?:=|\s|>)/);
  }
});
