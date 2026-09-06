# Architecture

## Module ownership

| Module | Responsibility | Allowed project dependencies |
| --- | --- | --- |
| `app/` | Framework routes, document layout, global styles | Demo composition and hosting integration |
| `src/demo/` | Wires subsystems together and adapts them to React | Core, character, interaction, scene, UI |
| `src/core/` | Shared data contracts, humanoid definitions, geometry, math, fixed-step clock | Core |
| `src/character/` | Physics world, motion ownership, balance and recovery controllers, procedural pose | Core and character |
| `src/interaction/` | Pointer capture, drag planes, queued intent, synthetic replay helpers | Core and interaction |
| `src/scene/` | Camera projection, camera gestures, interpolation and presentation | Core and scene |
| `src/ui/` | Controls receiving state and callbacks | Core types and shared UI components |

ESLint restricts imports between these layers, including type-only imports. Core does not import character implementation details. Character, interaction, and scene do not import one another; their shared contracts and picking geometry live in core. React composition belongs in demo, and Three.js belongs in the WebGL presentation adapter.

The `character/index.ts` and `interaction/index.ts` files expose existing public entrypoints. Implementations have explicit filenames. The old interaction picking path and grab diagnostic type exports remain compatible re-exports.

## Runtime and React

`DemoRuntime` owns one character, one shared camera, pointer and camera controllers, one selected view, the fixed-step loop, resize observation, and performance samples. It provides renderer switching, pause/resume, reset, subscriptions, and idempotent disposal.

`useDemoRuntime` owns the React effect and browser-level error, focus, and visibility listeners. It aborts pending initialization on cleanup and disposes completed runtimes. It converts runtime notifications into React state at approximately 10 Hz, with frame summaries refreshed approximately once per second.

`EmbodiedDemo` renders the workspace and connects control callbacks. `BrowserVerification` owns optional QA state and evidence capture. The replay implementation loads only when a replay is requested. `browser-api.ts` installs the existing read-only browser automation API and removes only the API instance it installed.

Teardown stops animation, disconnects resize observation, detaches input, disposes the renderer, and frees the Rapier world. An initialization failure or cancellation releases any physics resources already allocated.

## Simulation data flow

1. Pointer interaction picks an oriented body primitive and captures a stable region, segment, body-local anchor, and world target.
2. Pointer movement intersects a fixed camera-facing plane through the initial hit. Intent is queued; terminal commands take precedence over begin/move commands.
3. `FixedStepLoop` consumes input and advances the character at 60 Hz, with at most five catch-up steps per rendered frame. Runtime input availability is synchronized before and after each substep, so a fall clears capture and queued movement before the next integration.
4. The character publishes pose and diagnostic snapshots. Previous and current snapshots are retained for interpolation.
5. The selected view interpolates and renders snapshots. Presentation never advances physics or writes simulation poses.

World units are metres, +Y is up, and +Z is forward. Both renderers and picking use the same runtime-owned camera projection. Camera controls mutate that projection without affecting physics.

## Character ownership

The shared humanoid defines sixteen segments, seven selectable regions, masses, shapes, rest offsets, joint anchors and limits, and collision groups.

`EmbodiedCharacter` owns Rapier bodies, joints, the collision-aware root motor, handoffs, motion-state transitions, and snapshots. `BalanceController` reads solved poses and masses to estimate COM, momentum, eligible foot support, and reachable corrective steps. `pose.ts` composes upright poses and connected limb geometry from explicit inputs without accessing Rapier or the DOM. `DynamicRecovery` observes actual floor contacts and applies bounded joint and supported pelvis impulses. `GrabAnchorController` remains a separate bounded-grab utility and compatibility export; dynamic recovery does not use it to accept dragging.

Upright, reacting, and stepping states use the kinematic root and procedural pose compositor. Falling, fallen, and recovering states disable those writes before dynamic bodies and joints take ownership. Every recovery phase remains dynamic and contact-dependent. Body colliders interact with the environment rather than adjacent body segments.

External grab forces, equal/opposite joint muscle torques, and supported pelvis assistance have separate bounds and diagnostics. A fall clears the grab immediately. Recovery returns authority only after persistent load on both feet, low body motion, and upright posture; the procedural motor starts from the recovered world position, heading, and segment poses. See [balance controller](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for the fixed thresholds and phase conditions.

## Lifecycle invariants

- **Fall lockout:** use `bodyInputAvailable` for picking, interaction, and status; drop queued body commands and capture immediately, without advancing physics during cleanup. Held pointers need a fresh press after recovery.
- **Pause and focus loss:** clear pointer intent, consume its terminal command, pause physics and clock accumulation, and synchronize interpolation snapshots.
- **Resume:** clear stale input and resume the character and clock.
- **Reset:** clear input, reset the body, restore the camera while retaining viewport dimensions, clear timing and performance history, synchronize snapshots, and retain the paused state.
- **Renderer switch:** prepare the replacement view, cancel active body interaction, replace only presentation, preserve the shared camera and character, and restart performance warmup.
- **Dispose:** release browser and physics resources exactly once, including after incomplete initialization.

## Verification and tooling

`npm run lint` enforces layer boundaries. `npm run typecheck` checks shared contracts. `npm test` builds and checks rendered HTML, UI semantics, geometry, snapshot isolation, disposal, and runtime transitions. `npm run test:physics` evaluates the deterministic physics acceptance scenarios and regenerates local evidence. The [acceptance guide](docs/physics-acceptance.md) records independent numerical limits, explicit fixtures, handoff measurements, and input-isolation comparisons.

The Node-based tool launcher supports local development and bounded builds across operating systems. Hosting bindings are optional for source-only checkouts. Hosting support, database examples, and the shared UI catalog remain separate from the simulation.

Generated dependencies, build output, tool state, and physics evidence are ignored. Browser replay is explicitly synthetic; its results do not imply native touch support or measured GPU performance.
