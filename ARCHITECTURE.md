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

Every actuator produces equal-and-opposite parent/child torque impulses projected onto the joint profile's permitted world axes. Rapier enforces the structural limits independently of active posture control. There is no direct pelvis force or torque assistance. Nonadjacent character parts self-collide; connected parts and intentionally overlapping joint housings are explicitly excluded.

A fall clears the grab immediately but does not rebuild the body. Recovery completes only after persistent bilateral loaded support, low body motion, and upright posture; the standing controller then blends from the measured joint coordinates on the same bodies. See [balance controller](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for the controller and phase rules.

## Lifecycle invariants

- **Fall lockout:** use `bodyInputAvailable` for picking, interaction, and status; drop queued body commands and capture immediately, without advancing physics during cleanup. Held pointers need a fresh press after recovery.
- **Pause and focus loss:** clear pointer intent, consume its terminal command, pause physics and clock accumulation, and synchronize interpolation snapshots.
- **Resume:** clear stale input and resume the character and clock.
- **Reset:** clear input, reset the body, restore the camera while retaining viewport dimensions, clear timing and performance history, synchronize snapshots, and retain the paused state.
- **Renderer switch:** prepare the replacement view, cancel active body interaction, replace only presentation, preserve the shared camera and character, and restart performance warmup.
- **Dispose:** release browser and physics resources exactly once, including after incomplete initialization.

## Verification and tooling

`npm run lint` enforces layer boundaries. `npm run typecheck` checks shared contracts. `npm test` builds and checks rendered HTML, UI semantics, geometry, snapshot isolation, disposal, and runtime transitions. `npm run test:physics` evaluates the deterministic physics acceptance scenarios and regenerates local evidence. The [acceptance guide](docs/physics-acceptance.md) records independent numerical limits, explicit fixtures, continuous-ownership checks, structural-limit loading, free-fall integrity, diagnostics, and input-isolation comparisons.

The Node-based tool launcher supports local development and bounded builds across operating systems. Hosting bindings are optional for source-only checkouts. Hosting support, database examples, and the shared UI catalog remain separate from the simulation.

Generated dependencies, build output, tool state, and physics evidence are ignored. Browser replay is explicitly synthetic; its results do not imply native touch support or measured GPU performance.
