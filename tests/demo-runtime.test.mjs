import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const typescript = register({ namespace: "demo-runtime-tests" });
const { DemoRuntime } = await typescript.import("../src/demo/DemoRuntime.ts", import.meta.url);
after(() => typescript.unregister());

class TrackedEventTarget extends EventTarget {
  listeners = [];

  addEventListener(type, listener, options) {
    const capture = typeof options === "boolean" ? options : Boolean(options?.capture);
    this.listeners.push({ type, listener, capture });
    super.addEventListener(type, listener, options);
  }

  removeEventListener(type, listener, options) {
    const capture = typeof options === "boolean" ? options : Boolean(options?.capture);
    this.listeners = this.listeners.filter((entry) => entry.type !== type || entry.listener !== listener || entry.capture !== capture);
    super.removeEventListener(type, listener, options);
  }
}

function browserEnvironment(t) {
  const ownerWindow = new TrackedEventTarget();
  ownerWindow.devicePixelRatio = 3;
  const frames = new Map();
  const observers = [];
  const runtimes = [];
  const originalGlobals = new Map();
  let nextFrame = 0;
  const replacements = {
    window: ownerWindow,
    requestAnimationFrame(callback) {
      frames.set(++nextFrame, callback);
      return nextFrame;
    },
    cancelAnimationFrame(id) { frames.delete(id); },
    ResizeObserver: class {
      disconnected = false;
      constructor(callback) { this.callback = callback; observers.push(this); }
      observe(host) { this.host = host; }
      disconnect() { this.disconnected = true; }
    },
  };
  for (const [name, value] of Object.entries(replacements)) {
    originalGlobals.set(name, Object.getOwnPropertyDescriptor(globalThis, name));
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
  }
  t.after(() => {
    for (const runtime of runtimes) runtime.dispose();
    for (const [name, original] of originalGlobals) {
      if (original) Object.defineProperty(globalThis, name, original);
      else delete globalThis[name];
    }
  });
  const host = new TrackedEventTarget();
  host.ownerDocument = { defaultView: ownerWindow };
  host.style = { touchAction: "pan-y", userSelect: "text" };
  host.rect = { left: 20, top: 30, width: 800, height: 600 };
  host.getBoundingClientRect = () => host.rect;
  host.captures = new Set();
  host.setPointerCapture = (id) => host.captures.add(id);
  host.hasPointerCapture = (id) => host.captures.has(id);
  host.releasePointerCapture = (id) => host.captures.delete(id);
  return {
    host, ownerWindow, frames, observers, runtimes,
    tick(nowMs) {
      const pending = [...frames];
      for (const [id, callback] of pending) {
        frames.delete(id);
        callback(nowMs);
      }
    },
  };
}

function fakeCharacter() {
  return {
    updates: [],
    paused: false,
    activeGrab: false,
    resets: 0,
    disposals: 0,
    sequence: 0,
    state: "upright",
    inputClears: 0,
    afterUpdate: null,
    fixedUpdate(dt, command) {
      this.updates.push({ dt, command });
      if (command?.kind === "begin") this.activeGrab = true;
      if (command?.kind === "end" || command?.kind === "cancel") this.activeGrab = false;
      this.sequence += 1;
      this.afterUpdate?.();
    },
    getSnapshot(renderer) {
      return {
        sequence: this.sequence,
        simulationTime: this.sequence / 60,
        state: this.state,
        segments: [{ id: "torso", position: { x: 0, y: 1.1, z: 0 } }],
        diagnostics: { ...this.diagnostics(), renderer },
      };
    },
    pick() {
      return { region: "torso", segment: "torso", localAnchor: { x: 0, y: 0, z: 0 }, worldPoint: { x: 0, y: 1.1, z: 0 } };
    },
    diagnostics() {
      return {
        simulationReady: true, interactiveViewReady: true, activeGrab: this.activeGrab, paused: this.paused,
        state: this.state, bodyInputAvailable: !this.paused && !["falling", "fallen", "recovering"].includes(this.state),
      };
    },
    clearBodyInput() { this.inputClears += 1; this.activeGrab = false; },
    pause() { this.paused = true; this.activeGrab = false; },
    resume() { this.paused = false; },
    reset() { this.resets += 1; this.activeGrab = false; this.paused = false; this.state = "upright"; },
    dispose() { this.disposals += 1; },
  };
}

function fixture(t, options = {}) {
  const environment = browserEnvironment(t);
  const character = fakeCharacter();
  const views = [];
  const createView = (renderer, { camera }) => {
    const view = {
      renderer, camera, disposals: 0, renders: 0, snapshots: [], sizes: [],
      mount(host) {
        this.host = host;
        if (options.failMount?.(renderer)) throw new Error("View mount failed");
      },
      resize(width, height, dpr) {
        this.sizes.push({ width, height, dpr });
        camera.setViewport(width, height);
      },
      setSnapshot(previous, current, alpha) { this.snapshots.push({ previous, current, alpha }); },
      render() { this.renders += 1; },
      getProjection() { return camera; },
      dispose() { this.disposals += 1; },
    };
    views.push(view);
    return view;
  };
  const runtimeOptions = {
    host: environment.host,
    character,
    capabilities: { webgl2: options.webgl2 ?? true, canvas2d: true, browser: "test", devicePixelRatio: 3, viewport: { width: 800, height: 600 } },
    renderer: "canvas2d",
    createView,
  };
  return { ...environment, character, views, runtimeOptions };
}

function start(fixture) {
  const runtime = new DemoRuntime(fixture.runtimeOptions);
  fixture.runtimes.push(runtime);
  return runtime;
}

function pointer(host, type, pointerId = 7, overrides = {}) {
  const event = new Event(type, { cancelable: true });
  Object.assign(event, { pointerId, pointerType: "mouse", isPrimary: true, button: 0, clientX: 420, clientY: 260, buttons: type === "pointerup" ? 0 : 1, ...overrides });
  host.dispatchEvent(event);
}

test("runtime mounts, resizes, notifies subscribers, and releases every owned resource", (t) => {
  const f = fixture(t);
  const runtime = start(f);
  const view = f.views[0];
  assert.equal(view.host, f.host);
  assert.deepEqual(view.sizes, [{ width: 800, height: 600, dpr: 2 }]);
  assert.equal(view.renders, 1);
  assert.equal(f.observers[0].host, f.host);
  assert.equal(f.frames.size, 1);
  assert.ok(f.host.listeners.length > 0);
  assert.ok(f.ownerWindow.listeners.length > 0);

  f.host.rect = { ...f.host.rect, width: 1024, height: 768 };
  f.observers[0].callback();
  assert.deepEqual(view.sizes.at(-1), { width: 1024, height: 768, dpr: 2 });
  const notifications = [];
  const unsubscribe = runtime.subscribe((current, nowMs) => notifications.push({ current, nowMs }));
  f.tick(200);
  assert.equal(notifications[0].current, runtime);
  assert.equal(notifications[0].nowMs, 200);
  unsubscribe();
  f.tick(400);
  assert.equal(notifications.length, 1);

  runtime.dispose();
  runtime.dispose();
  assert.equal(f.frames.size, 0);
  assert.equal(f.observers[0].disconnected, true);
  assert.equal(f.host.listeners.length, 0);
  assert.equal(f.ownerWindow.listeners.length, 0);
  assert.deepEqual(f.host.style, { touchAction: "pan-y", userSelect: "text" });
  assert.equal(view.disposals, 1);
  assert.equal(f.character.disposals, 1);
});

test("pause cancels queued pointer input and reset preserves pause while clearing the camera and QA trace version", (t) => {
  const f = fixture(t);
  const runtime = start(f);
  const initialCamera = runtime.camera.getState();
  runtime.cameraControls.orbit(40, 15);
  pointer(f.host, "pointerdown");
  pointer(f.host, "pointermove");
  assert.equal(runtime.interaction.getActivePointerId(), 7);
  runtime.pause("blur");
  assert.equal(f.character.inputClears, 1);
  assert.equal(f.character.activeGrab, false);
  assert.equal(runtime.interaction.getStatus().hasQueuedCommand, false);
  assert.equal(f.host.captures.size, 0);
  assert.equal(runtime.paused, true);
  assert.equal(runtime.status, "Paused after focus loss");
  const updatesAfterPause = f.character.updates.length;
  f.tick(200);
  f.tick(400);
  assert.equal(f.character.updates.length, updatesAfterPause);

  const resets = [];
  runtime.subscribe((current) => resets.push(current.resetVersion));
  runtime.reset();
  assert.equal(runtime.paused, true);
  assert.equal(f.character.paused, true);
  assert.equal(f.character.resets, 1);
  assert.deepEqual(runtime.camera.getState(), initialCamera);
  assert.equal(runtime.resetVersion, 1);
  assert.deepEqual(resets, [1]);
  assert.equal(runtime.interaction.consumeCommand(), null);
  assert.equal(f.views[0].snapshots.at(-1).current, runtime.current);
  runtime.togglePause();
  assert.equal(runtime.paused, false);
  assert.equal(f.character.paused, false);
  f.tick(500);
  f.tick(520);
  assert.ok(f.character.updates.length > updatesAfterPause);
  assert.equal(f.character.updates.at(-1).command, null);
});

test("renderer replacement retains the simulation and shared camera while cancelling an active grab", (t) => {
  const f = fixture(t);
  const runtime = start(f);
  pointer(f.host, "pointerdown");
  f.tick(0);
  f.tick(20);
  assert.equal(f.character.activeGrab, true);
  pointer(f.host, "pointermove");
  runtime.cameraControls.orbit(25, -10);
  const cameraState = runtime.camera.getState();
  const integrationsBeforeSwitch = f.character.updates.length;
  runtime.switchRenderer("webgl");
  assert.equal(f.character.updates.length, integrationsBeforeSwitch);
  assert.equal(runtime.character, f.character);
  assert.equal(f.character.disposals, 0);
  assert.equal(f.character.inputClears, 1);
  assert.equal(f.character.activeGrab, false);
  assert.equal(runtime.current.diagnostics.activeGrab, false);
  assert.equal(runtime.interaction.consumeCommand(), null);
  assert.equal(f.views[0].disposals, 1);
  assert.equal(runtime.view, f.views[1]);
  assert.equal(f.views[0].camera, f.views[1].camera);
  assert.equal(f.views[1].camera, runtime.camera);
  assert.deepEqual(runtime.camera.getState(), cameraState);
  assert.equal(runtime.renderer, "webgl");
  assert.equal(runtime.current.diagnostics.renderer, "webgl");
  assert.equal(f.host.style.touchAction, "none");
  assert.equal(f.frames.size, 1);
  assert.equal(runtime.frameSummary().samples, 0);
});

test("unsupported renderer selections leave the active runtime intact", (t) => {
  const f = fixture(t, { webgl2: false });
  const runtime = start(f);
  runtime.switchRenderer("webgl");
  assert.equal(runtime.renderer, "canvas2d");
  assert.equal(f.views.length, 1);
  assert.equal(f.views[0].disposals, 0);
  assert.equal(f.character.updates.length, 0);
});

test("a failed replacement disposes the new view and preserves the current view and input", (t) => {
  const f = fixture(t, { failMount: (renderer) => renderer === "webgl" });
  const runtime = start(f);
  pointer(f.host, "pointerdown");
  assert.throws(() => runtime.switchRenderer("webgl"), /View mount failed/);
  assert.equal(runtime.view, f.views[0]);
  assert.equal(runtime.renderer, "canvas2d");
  assert.equal(f.views[0].disposals, 0);
  assert.equal(f.views[1].disposals, 1);
  assert.equal(runtime.interaction.getActivePointerId(), 7);
  assert.equal(f.character.disposals, 0);
  assert.equal(f.frames.size, 1);
});

test("initial mount failure disposes the view and character without starting a frame loop", (t) => {
  const f = fixture(t, { failMount: () => true });
  assert.throws(() => new DemoRuntime(f.runtimeOptions), /View mount failed/);
  assert.equal(f.views[0].disposals, 1);
  assert.equal(f.character.disposals, 1);
  assert.equal(f.frames.size, 0);
  assert.equal(f.host.listeners.length, 0);
  assert.equal(f.ownerWindow.listeners.length, 0);
});


test("fall entry cancels queued moves and capture before the next substep of the same frame", (t) => {
  const f = fixture(t);
  const runtime = start(f);
  pointer(f.host, "pointerdown");
  pointer(f.host, "pointermove", 7, { clientX: 510 });
  assert.equal(runtime.interaction.getStatus().hasQueuedCommand, true);
  assert.equal(f.host.hasPointerCapture(7), true);
  f.character.afterUpdate = () => {
    if (f.character.sequence === 1) {
      f.character.state = "falling";
      f.character.activeGrab = false;
    } else {
      assert.equal(runtime.interaction.getActivePointerId(), null, "capture owner cleared before later integration");
      assert.equal(runtime.interaction.getStatus().hasQueuedCommand, false, "queued move discarded before later integration");
      assert.equal(f.host.hasPointerCapture(7), false, "capture released within the frame");
    }
  };
  f.tick(0);
  f.tick(60);
  assert.equal(f.character.updates.length, 3, "cleanup must not add a fourth integration");
  assert.deepEqual(f.character.updates.map(({ command }) => command?.kind ?? null), ["begin", null, null]);
  assert.equal(runtime.interaction.consumeCommand(), null);
  assert.equal(runtime.interaction.getStatus().bodyInputAvailable, false);
  assert.match(runtime.status, /Protecting the fall/);
});

test("held and lockout-pressed pointers cannot resume a body grab after recovery", (t) => {
  const f = fixture(t);
  const runtime = start(f);
  pointer(f.host, "pointerdown");
  f.character.afterUpdate = () => {
    if (f.character.sequence === 1) {
      f.character.state = "falling";
      f.character.activeGrab = false;
    }
  };
  f.tick(0);
  f.tick(20);
  pointer(f.host, "pointermove", 7, { clientX: 540 });
  pointer(f.host, "pointerdown", 8);
  assert.equal(runtime.interaction.getActivePointerId(), null);
  for (const [state, time] of [["fallen", 40], ["recovering", 60], ["upright", 80]]) {
    f.character.state = state;
    f.tick(time);
    pointer(f.host, "pointermove", 7, { clientX: 550, buttons: 1 });
    pointer(f.host, "pointermove", 8, { clientX: 560, buttons: 1 });
    assert.equal(runtime.interaction.getActivePointerId(), null, `held pointers cannot acquire in ${state}`);
    assert.equal(runtime.interaction.getStatus().hasQueuedCommand, false);
    if (state === "recovering") assert.match(runtime.status, /Getting up/);
  }
  assert.equal(runtime.current.diagnostics.bodyInputAvailable, true);
  pointer(f.host, "pointerup", 7);
  pointer(f.host, "pointerup", 8);
  pointer(f.host, "pointerdown", 7);
  assert.equal(runtime.interaction.getActivePointerId(), 7, "a fresh press is accepted");
  const updatesBeforeFreshPress = f.character.updates.length;
  f.tick(100);
  assert.equal(f.character.updates[updatesBeforeFreshPress].command.kind, "begin");
});

for (const lockedState of ["falling", "fallen", "recovering"]) {
  test(`camera, pause, renderer switch and Reset remain available during ${lockedState}`, (t) => {
    const f = fixture(t);
    f.character.state = lockedState;
    const runtime = start(f);
    const initialCamera = runtime.camera.getState();
    pointer(f.host, "pointerdown", 20);
    pointer(f.host, "pointermove", 20, { clientX: 470 });
    pointer(f.host, "pointerup", 20);
    assert.notDeepEqual(runtime.camera.getState(), initialCamera, "locked body still permits camera orbit");
    assert.equal(runtime.interaction.getStatus().hasQueuedCommand, false);
    const integrations = f.character.updates.length;
    runtime.pause();
    assert.equal(runtime.paused, true);
    assert.equal(runtime.interaction.getStatus().bodyInputAvailable, false);
    pointer(f.host, "pointerdown", 21);
    assert.equal(runtime.interaction.getActivePointerId(), null);
    runtime.switchRenderer("webgl");
    assert.equal(runtime.renderer, "webgl");
    assert.equal(f.character.state, lockedState);
    runtime.togglePause();
    assert.equal(runtime.paused, false);
    assert.equal(runtime.interaction.getStatus().bodyInputAvailable, false);
    runtime.reset();
    assert.equal(f.character.updates.length, integrations, "all cleanup runs without integrating physics");
    assert.equal(f.character.state, "upright");
    assert.equal(runtime.interaction.getStatus().bodyInputAvailable, true);
    pointer(f.host, "pointermove", 21, { clientX: 520 });
    assert.equal(runtime.interaction.getActivePointerId(), null);
    pointer(f.host, "pointerup", 21);
    pointer(f.host, "pointerdown", 22);
    assert.equal(runtime.interaction.getActivePointerId(), 22);
    f.tick(0);
    f.tick(20);
    assert.equal(f.character.updates.at(-1).command.kind, "begin");
  });
}
