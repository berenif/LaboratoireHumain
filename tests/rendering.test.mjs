import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";
import * as THREE from "three";
const unregister = register();
after(unregister);
const { PresentationBuffer, interpolatePoseSnapshot } = await import("../src/scene/pose.ts");
const { AutoQuality, qualitySettings } = await import("../src/scene/quality.ts");
const { SharedCameraProjection } = await import("../src/scene/camera.ts");
const { Canvas2DView } = await import("../src/scene/canvas2d-view.ts");
const { WebGLView } = await import("../src/scene/webgl-view.ts");
const { batchOpaque, disposeSceneGroup } = await import("../src/scene/resources.ts");
const { restPoseMap } = await import("../src/character/pose.ts");
const { SEGMENT_BY_ID } = await import("../src/core/humanoid.ts");
const freeze = object => {
  if (object && typeof object === "object" && !Object.isFrozen(object)) {
    Object.freeze(object); Object.values(object).forEach(freeze);
  }
  return object;
};
function snapshot() {
  return { sequence: 0, simulationTime: 0, state: "upright", rootPosition: { x: 0, y: 1, z: 0 },
    rootRotation: { x: 0, y: 0, z: 0, w: 1 }, segments: [...restPoseMap().values()].map(pose => ({ ...pose,
      linearVelocity: { x: 0, y: 0, z: 0 }, angularVelocity: { x: 0, y: 0, z: 0 } })),
    support: { planted: ["leftFoot", "rightFoot"], swingFoot: null }, diagnostics: { selectedRegion: null, selectedSegment: null },
    room: { width: 6, depth: 6, height: 3 }, striker: { position: { x: 1, y: 1, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } } };
}

test("preparing a replacement quality cannot resize the active shared camera", () => {
  const camera = new SharedCameraProjection(); camera.setViewport(1366, 768);
  const before = camera.getState();
  for (const Adapter of [WebGLView, Canvas2DView]) {
    const view = new Adapter({ camera, canvas: { style: {}, dataset: {} } });
    view.setQuality("low");
    assert.deepEqual(camera.getState(), before);
    view.dispose(); view.dispose();
  }
});

test("presentation scratch buffers match detached interpolation without modifying immutable snapshots", () => {
  const before = freeze(snapshot()), current = structuredClone(before);
  current.rootPosition.x = 2; current.simulationTime = 1;
  current.segments.reverse();
  current.segments.forEach(pose => { pose.position.x += 1; pose.rotation = { x: 0, y: 0.5, z: 0, w: -0.8 }; });
  current.striker.position.x += 2; freeze(current);
  const bytes = JSON.stringify([before, current]), buffer = new PresentationBuffer();
  const displayed = buffer.update(before, current, 0.3), segment = displayed.segments[0];
  const detached = interpolatePoseSnapshot(before, current, 0.3);
  assert.deepEqual(displayed, detached);
  const saved = structuredClone(detached);
  assert.equal(buffer.update(before, current, 0.7), displayed);
  assert.equal(displayed.segments[0], segment);
  assert.deepEqual(detached, saved, "public helper remains detached from reusable state");
  assert.equal(JSON.stringify([before, current]), bytes);
  buffer.update(current, { ...current, segments: current.segments.slice(1) }, 1);
  assert.equal(buffer.byId.has(segment.id), false);
});

test("quality caps respect device ratio and Auto excludes warmup and uses sustained hysteresis", () => {
  for (const device of [0.5, 1, 1.25, 2, 3]) {
    assert.equal(qualitySettings("low", device).pixelRatio, Math.min(device, 0.75));
    assert.equal(qualitySettings("high", device).pixelRatio, Math.min(device, 1.5));
    assert.equal(qualitySettings("auto", device).pixelRatio, Math.min(device, 1.25));
  }
  assert.equal(qualitySettings("low", 2).shadowResolution, 512);
  assert.equal(qualitySettings("high", 2).shadowResolution, 1024);
  const quality = new AutoQuality(); quality.exclude(0);
  let now = 0; const changes = [];
  while (now < 950) { now += 25; assert.equal(quality.sample(now), false); }
  while (now < 11000) { now += 25; if (quality.sample(now)) changes.push(now); }
  assert.equal(quality.ratio, 0.75);
  assert.ok(changes.every((time, i) => !i || time - changes[i - 1] >= 2000));
  quality.exclude(20000); now = 20000;
  while (now < 25500) { now += 16; quality.sample(now); }
  assert.equal(quality.ratio, 0.75, "five seconds of eligible fast intervals required");
  while (now < 41000) { now += 16; quality.sample(now); }
  assert.equal(quality.ratio, 1.5);
  quality.exclude(100000); quality.sample(100001);
  assert.equal(quality.ratio, 1.5, "hidden/paused gaps cannot downshift");
});

test("Canvas retains geometry and projected commands until camera, viewport or configuration changes", t => {
  const previousPath = globalThis.Path2D;
  globalThis.Path2D = class { moveTo() {} lineTo() {} closePath() {} arc() {} ellipse() {} };
  t.after(() => { globalThis.Path2D = previousPath; });
  const paints = [];
  const context = { setTransform() {}, fillRect() {}, fill(path) { paints.push(path); }, stroke() {}, fillText() {},
    clearRect() {}, beginPath() {}, arc() {} };
  const canvas = { style: {}, dataset: {}, width: 1, height: 1, getContext: () => context };
  const host = { append() {}, getBoundingClientRect: () => ({ width: 800, height: 600 }), ownerDocument: { defaultView: { devicePixelRatio: 2 } } };
  const camera = new SharedCameraProjection(), view = new Canvas2DView({ canvas, camera });
  view.mount(host);
  const pose = freeze(snapshot());
  const render = next => { view.setSnapshot(pose, next, 1); view.render(); };
  render(pose);
  const commands = [...view.staticCommands];
  const first = view.getMetrics();
  render(pose);
  assert.equal(view.getMetrics().projectionBuilds, first.projectionBuilds);
  assert.equal(view.staticCommands[0], commands[0]);
  view.setQuality("low"); render(pose);
  assert.equal(view.getMetrics().effectivePixelRatio, 0.75);
  assert.equal(view.getMetrics().projectionBuilds, first.projectionBuilds, "DPR does not change CSS projections");
  camera.setView({ x: 4, y: 3, z: 7 }, { x: 0, y: 1, z: 0 }); render(pose);
  view.resize(1024, 768, 1); render(pose);
  assert.equal(view.getMetrics().geometryBuilds, first.geometryBuilds);
  assert.equal(view.getMetrics().projectionBuilds, first.projectionBuilds + 2);
  render({ ...pose, room: { ...pose.room, width: 8 } });
  assert.equal(view.getMetrics().geometryBuilds, first.geometryBuilds + 1);
  const course = { ...pose, room: undefined, playground: { station: "flat", difficulty: "gentle" } };
  render(course);
  const count = view.getMetrics().geometryBuilds;
  render({ ...course, simulationTime: 5 });
  assert.equal(view.getMetrics().geometryBuilds, count);
  // All static triangles and moving triangles are painted in a single depth order.
  const paintedDepths = [];
  const draw = view.draw.bind(view);
  view.draw = (context, command) => {
    if (!view.floorCommands.includes(command)) paintedDepths.push(command.depth);
    draw(context, command);
  };
  render({ ...course, simulationTime: 5 });
  assert.ok(paintedDepths.length > 100);
  assert.ok(paintedDepths.every((depth, i) => !i || paintedDepths[i - 1] >= depth));
  canvas.ownerDocument = { createElement: () => ({ width: 0, height: 0, getContext: () => ({ ...context }) }) };
  camera.setViewport(800, 600);
  context.drawImage = () => {};
  render(course);
  const layers = [...view.staticLayers.values()];
  assert.ok(layers.length > 0, "static drawing is retained as raster slices");
  const drawLayer = view.drawLayer.bind(view);
  view.drawLayer = (context, layer) => {
    if (layer !== view.floorLayer) {
      const start = [...view.staticLayers].find(([, candidate]) => candidate === layer)[0];
      for (const command of view.staticCommands.slice(start, layer.end)) paintedDepths.push(command.depth);
    }
    drawLayer(context, layer);
  };
  paintedDepths.length = 0; render({ ...course, simulationTime: 6 });
  assert.ok(paintedDepths.every((depth, i) => !i || paintedDepths[i - 1] >= depth), "raster slices cannot jump over moving geometry");
  const floor = view.floorLayer;
  view.setQuality("high"); render(course);
  assert.equal(floor.canvas.width, 1, "quality invalidation releases old raster storage");
  view.dispose(); view.dispose();
  assert.ok(layers.every(layer => layer.canvas.width === 1));
  assert.equal(view.staticCommands.length, 0);
});

test("opaque batches keep transformed vertices and release shared resources once on rebuild", () => {
  const group = new THREE.Group(), geometry = new THREE.BoxGeometry(), material = new THREE.MeshLambertMaterial({ color: "#aabbcc" });
  let geometryDisposals = 0, materialDisposals = 0;
  geometry.addEventListener("dispose", () => geometryDisposals++); material.addEventListener("dispose", () => materialDisposals++);
  const a = new THREE.Mesh(geometry, material), b = new THREE.Mesh(geometry, material); b.position.x = 4;
  group.add(a, b); batchOpaque(group);
  assert.equal(group.children.length, 1);
  assert.equal(geometryDisposals, 1); assert.equal(materialDisposals, 1);
  const merged = group.children[0]; merged.geometry.computeBoundingBox();
  assert.equal(merged.geometry.boundingBox.max.x, 4.5);
  assert.equal(merged.geometry.groups.length, 0);
  assert.ok(merged.geometry.getAttribute("color"));
  const texture = new THREE.Texture(), shared = new THREE.MeshBasicMaterial({ map: texture });
  const g = new THREE.PlaneGeometry();
  group.add(new THREE.Mesh(g, shared), new THREE.Mesh(g, shared));
  const counts = [0, 0, 0];
  [g, shared, texture].forEach((resource, i) => resource.addEventListener("dispose", () => counts[i]++));
  disposeSceneGroup(group); disposeSceneGroup(group);
  assert.deepEqual(counts, [1, 1, 1]);
  assert.equal(group.children.length, 0);
  assert.ok(SEGMENT_BY_ID.get("torso").geometry.vertices.length, "canonical body definitions remain independent");
});
