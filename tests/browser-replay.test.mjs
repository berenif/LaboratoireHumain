import assert from "node:assert/strict";
import test, { after } from "node:test";
import { register } from "tsx/esm/api";

const typescript = register({ namespace: "browser-replay-tests" });
const { runInteractionReplay } = await typescript.import("../src/interaction/browser-replay.ts", import.meta.url);
const { REGION_IDS } = await typescript.import("../src/core/types.ts", import.meta.url);
after(() => typescript.unregister());

function fixture(t) {
  const timers = new Map();
  const events = [];
  const clicks = [];
  const reads = [];
  let timerId = 0;
  let selectedRegion = null;
  const zero = { x: 0, y: 0, z: 0 };
  const positions = REGION_IDS.map((id, x) => ({ id, position: { x, y: 0, z: 0 } }));
  const replacements = {
    window: {
      setTimeout(callback) { timers.set(++timerId, callback); return timerId; },
      clearTimeout(id) { timers.delete(id); },
    },
    PointerEvent: class {
      constructor(type, fields) { Object.assign(this, fields, { type }); }
    },
  };
  for (const [name, value] of Object.entries(replacements)) {
    const original = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
    t.after(() => {
      if (original) Object.defineProperty(globalThis, name, original);
      else delete globalThis[name];
    });
  }
  const adapter = {
    host: {
      ownerDocument: {
        querySelector(selector) {
          return {
            getAttribute() { return "false"; },
            click() { clicks.push(selector); selectedRegion = null; },
          };
        },
      },
      getBoundingClientRect() { return { left: 0, top: 0 }; },
      dispatchEvent(event) {
        events.push(event.type);
        selectedRegion = event.type === "pointerdown" ? REGION_IDS[event.clientX] : null;
      },
    },
    interaction: {
      getStatus() {
        reads.push("interaction");
        return { selectedRegion, localAnchor: zero };
      },
      getActivePointerId() { return selectedRegion ? 4001 : null; },
    },
    snapshot() { reads.push("snapshot"); return { segments: positions }; },
    projection() { reads.push("projection"); return { project: (point) => point }; },
    diagnostics() {
      reads.push("diagnostics");
      return { selectedRegion, activeGrab: selectedRegion !== null, appliedGrabForceN: 0, finite: true, errors: [] };
    },
  };
  return {
    adapter, timers, events, clicks, reads,
    async advanceTimer() {
      const pending = timers.entries().next().value;
      assert.ok(pending, "replay has a pending wait");
      timers.delete(pending[0]);
      pending[1]();
      await new Promise((resolve) => setImmediate(resolve));
    },
  };
}

test("an already aborted replay does not access the runtime", async () => {
  const abort = new AbortController();
  abort.abort();
  const adapter = new Proxy({}, { get() { assert.fail("aborted replay accessed its adapter"); } });
  await assert.rejects(runInteractionReplay(adapter, { signal: abort.signal }), { name: "AbortError" });
});

test("aborting an active pointer replay cancels its wait without dispatching teardown input", async (t) => {
  const f = fixture(t);
  const abort = new AbortController();
  const progress = [];
  const replay = runInteractionReplay(f.adapter, {
    suite: "selection", signal: abort.signal, onProgress: (stage) => progress.push(stage),
  });
  await f.advanceTimer();
  assert.deepEqual(f.events, ["pointerdown"]);
  assert.equal(f.timers.size, 1);
  const priorReads = [...f.reads];
  const priorClicks = [...f.clicks];
  abort.abort();
  await assert.rejects(replay, { name: "AbortError" });
  assert.equal(f.timers.size, 0);
  assert.deepEqual(f.reads, priorReads);
  assert.deepEqual(f.clicks, priorClicks);
  assert.deepEqual(f.events, ["pointerdown"]);
  assert.deepEqual(progress, ["selecting head"]);
});

test("selection replay without a signal still completes and resets its UI", async (t) => {
  const f = fixture(t);
  let result;
  const replay = runInteractionReplay(f.adapter, { suite: "selection" }).then((value) => { result = value; });
  for (let waits = 0; !result && waits < 30; waits += 1) await f.advanceTimer();
  await replay;
  assert.equal(result.passed, true);
  assert.equal(result.results.length, REGION_IDS.length);
  assert.equal(result.synthetic, true);
  assert.equal(f.events.filter((type) => type === "pointerdown").length, REGION_IDS.length);
  assert.equal(f.events.at(-1), "pointercancel");
  assert.equal(f.clicks.length, REGION_IDS.length + 1);
  assert.equal(f.timers.size, 0);
});
