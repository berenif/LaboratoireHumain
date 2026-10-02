# Architecture

This is an implementation reference for the working tree. Acceptance results and their source scope are maintained in [current status](docs/status.md); historical test passes do not certify these ownership rules across every scenario.

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

It also owns the selected render quality and `AutoQuality` frame-interval history. Quality changes and renderer switches retain character ownership. Camera/resize/control invalidations request one redraw while paused; active simulation continues scheduling frames. Presentation metrics are available through the browser API's `presentation()` and `diagnostics()`, and the quality selection through `quality()`. CPU submission timing does not measure GPU execution.

`useDemoRuntime` owns the React effect and browser-level error, focus, and visibility listeners. It aborts pending initialization on cleanup and disposes completed runtimes. It converts runtime notifications into React state at approximately 10 Hz, with frame summaries refreshed approximately once per second.

`EmbodiedDemo` renders the workspace and connects control callbacks. `BrowserVerification` owns optional QA state and evidence capture. The replay implementation loads only when a replay is requested. `browser-api.ts` installs the existing read-only browser automation API and removes only the API instance it installed.

Teardown stops animation, disconnects resize observation, detaches input, disposes the renderer, and frees the Rapier world. An initialization failure or cancellation releases any physics resources already allocated.

## Simulation data flow

1. Pointer interaction raycasts the canonical oriented convex surfaces and captures a stable region, exact segment, body-local anchor, and world target.
2. Pointer movement intersects a fixed camera-facing plane through the initial hit. Intent is queued; terminal commands take precedence over begin/move commands.
3. `FixedStepLoop` consumes input and advances the character at 60 Hz, with at most five catch-up steps per rendered frame. Runtime input availability is synchronized before and after each substep, so a fall clears capture and queued movement before the next integration.
4. The character publishes pose and diagnostic snapshots. Previous and current snapshots are retained for interpolation.
5. The selected view interpolates and renders snapshots. Presentation never advances physics or writes simulation poses.

World units are metres, +Y is up, and +Z is forward. Both renderers and picking use the same runtime-owned camera projection. Camera controls mutate that projection without affecting physics.

## Character ownership

The shared humanoid defines 25 segments grouped into seven selectable regions. Each definition owns its mass, canonical procedural convex surface, rest offset, anatomical role, side, joint reference frames, permitted rotation coordinates, asymmetric limits, passive resistance, damping, actuator strength, and explicit collision exclusions. The total remains 72.2 kg at a 1.84 m rest stature.

`EmbodiedCharacter` creates one dynamic Rapier body assembly and keeps it alive until Reset or disposal. Rapier owns every runtime transform and velocity in every motion state. `BalanceController`, `pose.ts`, inverse kinematics, and `DynamicRecovery` generate target intent only; they never install a rendered pose. State changes blend joint targets and strength while preserving body identity, position, rotation, and momentum.

`BalanceController` reads measured segment mass state and loaded contact patches to estimate center of mass, momentum, available support, and reachable corrective steps. Support-chain compensation is expressed as bounded joint torques. `DynamicRecovery` observes actual contacts and advances only from physical support and movement evidence. `GrabAnchorController` applies its force-, torque-, and power-limited command at the exact picked surface anchor.

Standing uses `NativeJointMotors`: finite force-based joint motors participate in Rapier's constraint solve. Most recovery actuation uses `applyCoupledJointMotors`, which applies equal-and-opposite parent/child torque impulses along permitted axes. Rolling arms can be deferred to native motors; settling uses passive joint resistance. These paths retain the joint-profile effort ceilings and structural constraints. There is no direct pelvis force or torque assistance. Nonadjacent character parts self-collide; connected parts and intentionally overlapping joint housings are explicitly excluded.

Explicit standing selectors add H74 coordinated posture, H77 implicit posture, and H78–H80 contact-force controllers on the same body assembly. The latter share a serializable support state with a non-mutating forecast; forecast admission remains disabled. Default startup retains the legacy controller. All these candidates failed their recorded feasibility gates; see [experimental selection](docs/balance-controller.md#experimental-standing-selection).

Joint diagnostics identify their torque source: `native-request` is a bounded requested wrench, while `applied-impulse` describes the explicit torque impulse divided by the timestep. Rapier's delivered native motor impulse is not exposed by the pinned API. Neither value is a measurement of the complete contact-coupled body response. See the [balance actuation and load planner](docs/balance-controller.md#physical-actuation) and [recovery configuration](docs/dynamic-recovery.md#ownership-and-state).

A fall clears the grab immediately but does not rebuild the body. Recovery completes only after persistent bilateral loaded support, low body motion, and upright posture; the standing controller then blends from the measured joint coordinates on the same bodies. See [balance controller](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for the controller and phase rules.

## Lifecycle invariants

- **Fall lockout:** use `bodyInputAvailable` for picking, interaction, and status; drop queued body commands and capture immediately, without advancing physics during cleanup. Held pointers need a fresh press after recovery.
- **Pause and focus loss:** clear pointer intent, consume its terminal command, pause physics and clock accumulation, and synchronize interpolation snapshots.
- **Resume:** clear stale input and resume the character and clock.
- **Reset:** clear input, reset the body, restore the camera while retaining viewport dimensions, clear timing and performance history, synchronize snapshots, and retain the paused state.
- **Renderer switch:** prepare the replacement view, cancel active body interaction, replace only presentation, preserve the shared camera and character, and restart performance warmup.
- **Dispose:** release browser and physics resources exactly once, including after incomplete initialization.

## Verification and tooling

The presentation adapters share interpolation buffers and canonical geometry. WebGL batches opaque scenery and instances joint markers; Canvas2D caches static geometry/projection commands and updates moving pieces separately. Resource disposal deduplicates shared geometry, materials, and textures. Quality caps, metrics, and browser reproduction are documented in [rendering](docs/rendering.md).

`npm run lint` enforces layer boundaries. `npm run typecheck` checks shared contracts. `npm test` builds and checks rendered HTML, UI semantics, geometry, snapshot isolation, disposal, and runtime transitions. `npm run test:physics` evaluates the deterministic physics acceptance scenarios and regenerates local evidence. The [acceptance guide](docs/physics-acceptance.md) records independent numerical limits, explicit fixtures, continuous-ownership checks, structural-limit loading, free-fall integrity, diagnostics, and input-isolation comparisons.

The Node-based tool launcher supports local development and bounded builds across operating systems. Hosting bindings are optional for source-only checkouts. Hosting support, database examples, and the shared UI catalog remain separate from the simulation.

Generated dependencies, build output, tool state, and physics evidence are ignored. Browser replay is explicitly synthetic; its results do not imply native touch support or measured GPU performance.

## Isolated Rust prototype

`rust/` is a separate Cargo workspace with `lh-model`, `lh-contracts`, `lh-sim`, `lh-acceptance`, the `lh-worker` WASM adapter, `lh-renderer` and the Leptos `lh-browser` application. It pins Rust 1.89.0 and an audited Rapier 0.35.0 fork, independent of the production browser's Rapier 0.20.0 dependency. The native simulation uses four 1/240 s substeps per 60 Hz tick and typed, stamped commands/observations. Export scripts capture canonical anatomy and scenario inputs from the existing implementation.

`lh-sim::runtime::Runtime` owns the schema-4 command queue, atomic command packets, pause/resume, visibility, reset/initialize generations, floor toggling and teardown. Paused/hidden time does not integrate; suspension cancels pending pointer intent and requires a fresh press. Strike/station/difficulty commands remain explicitly unsupported. `lh-worker` and `rust/browser/sim-worker.js` expose this owner in a dedicated worker with three transferable pose buffers. The isolated browser preview integrates wgpu WebGPU/WebGL2 rendering and Leptos controls; production still uses the existing TypeScript app. `scripts/run-rust-gates.mjs` preserves source, commands, failures, and repeatability evidence; `.github/workflows/rust-rework.yml` uploads evidence without publishing. Native gates A–C, all eight selected pull probes and the six-case WASM standing matrix pass on the v22 contact profile, while full D and later gates remain incomplete. See the [requirement map and reproduction record](docs/rust-rework.md).
