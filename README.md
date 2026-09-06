# Embodied Motion Physics Demo

A browser demo for directly manipulating a procedural humanoid. The character shifts weight and takes corrective steps using its support and momentum. Strong pulls can overpower its balance, triggering a protective fall and automatic get-up through dynamic Rapier physics. The same snapshots drive either a Three.js WebGL2 view or a Canvas2D projection.

## Run locally

Use Node.js **22.13.0 or newer**.

```bash
npm ci
npm run dev
```

The standard development, build, start, and lint commands work on Windows, macOS, and Linux. They use the active Node executable and keep generated tool state inside the project. The optional `install:ci` helper is intended for Bash-based CI environments.

A source checkout runs without `.openai/hosting.json`. When that file exists, Vite reads its D1/R2 binding names; otherwise local bindings are empty.

Use **View** to select Canvas2D or WebGL2. Drag the colored head, torso, pelvis, hands, or feet. Body dragging pauses during falling and recovery, then requires a fresh press after stable standing. Drag empty space to orbit, use Shift-drag to pan, and scroll to zoom. **Reset body** restores the body and camera and preserves the paused state.

## Verify

```bash
npm run typecheck
npm run lint
npm test
npm run test:physics
```

`npm test` builds the application and runs the rendered-page, UI, domain, and runtime lifecycle tests. `test:physics` runs the deterministic balance, fall, recovery, and input-isolation scenarios and writes its assessment to the ignored `evidence/physics-results.json`. Failures exit nonzero.

Add `?qa=1` to the local URL for optional browser verification controls. Replay uses synthetic DOM pointer events; it does not establish native touch or GPU performance.

## Project structure

- `app/`: framework routes, layout, and global styling.
- `src/demo/`: application composition, browser runtime lifecycle, React adapter, and optional QA.
- `src/core/`: shared contracts, humanoid definitions, geometry, math, and fixed-step clock.
- `src/character/`: Rapier ownership, bounded balance and recovery controllers, grab diagnostics, and pure procedural pose composition.
- `src/interaction/`: browser pointer capture and queued grab intent.
- `src/scene/`: shared camera, camera controls, and snapshot rendering adapters.
- `src/ui/`: simulation controls; `components/ui/` contains the existing shared UI catalog.
- `tests/` and `scripts/`: regression checks, physics scenarios, and local tooling.
- `build/`, `worker/`, `db/`, and `examples/`: hosting integration and optional starter infrastructure.

See [ARCHITECTURE.md](ARCHITECTURE.md) for ownership rules and extension points, [balance](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for the controller rules, [physics acceptance](docs/physics-acceptance.md) for fixed scenarios, and [validation](docs/validation.md) for recorded check results and evidence.
