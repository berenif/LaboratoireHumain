# Balance playground

The browser enables the shared playground in `createDemoRuntime`. Headless character fixtures keep their original flat-floor default unless they supply `CharacterInitialOptions.playground`.

Seven stations provide flat ground, an incline/cross-slope, rolling terrain, an elevated beam, stepping stones, a moving deck, and hurdles. Difficulty changes rebuild the physical environment and start a new trial. Station selection focuses the camera and resets the character; pause is retained. Reset keeps the selected station and difficulty. Upright time counts simulated time with loaded foot support outside falling/recovery; a fall is counted once on entering those states.

`src/core/playground.ts` owns deterministic geometry, station metadata, and deck transforms. `PhysicsPlayground` owns fixed/kinematic bodies and support queries. The renderers consume the same geometry and simulation clock. Rough terrain uses solid triangular prisms because the installed Rapier triangle-mesh manifolds do not expose the solver-contact loads required by this controller. Terrain sensing changes motor targets; body transforms remain physics-owned during trials.

The deck starts level, waits one simulated second, and ramps its roll/pitch over the next two seconds. Pause freezes its clock and Reset restores the deck before sampling the new stance. At spawn, feet align with the local surface and clear the terrain at their corners. Runtime steps use queried destination heights. A 12 ms frame budget stops additional catch-up steps after expensive physics work, retaining the fixed timestep and recording dropped time.

## Verification

- Production static build and Pages asset verification passed.
- TypeScript and ESLint passed.
- Playground/contact tests, character-domain tests, runtime lifecycle/trial tests, and fixed-step budgeting tests passed (25 checks).
- `scripts/verify-playground.mjs` passed against the built app: body picking/dragging, all stations, Extreme difficulty, pause retention, reset, renderer switching, and a 390 × 844 mobile viewport. Screenshots and samples are written to ignored `evidence/playground/`.
- The broader balance suite is not fully green: the slow-pull/reversal and planted-reversal scenarios failed. The slow-pull rerun ended in recovery instead of upright. These controller limitations are not treated as successful terrain acceptance.

Run focused checks with `node --test tests/playground.test.mjs tests/character-domain.test.mjs tests/demo-runtime.test.mjs tests/fixed-step-budget.test.mjs`. The browser script uses the bundled Playwright runtime identified by `CODEX_MCP_NODE_PATH`; `PLAYGROUND_URL` overrides the default local development URL.

The procedural recovery controller can stall on difficult or uneven support. The user can retry or change stations at any time; the course does not force successful recovery or install animated body poses.
