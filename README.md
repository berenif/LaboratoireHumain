# Laboratoire Humain

An experimental physical human simulation. **Rust/WASM is the only application runtime**: Leptos provides the UI, wgpu renders with WebGPU or WebGL2, and Rapier runs in a dedicated WASM worker. JavaScript handles browser events and worker transport.

Requires Node 22.13+, npm, rustup and a browser with WebGPU or WebGL2.

```sh
npm ci
npm run setup:rust   # pinned Rust compiler, WASM target, Trunk and wasm-bindgen
npm run build
npm start           # http://127.0.0.1:5185
```

`npm run dev` builds and starts the same application; restart it after source changes. `npm run preview` serves the existing build. `npm run build:pages` builds the `/LaboratoireHumain/` mount into `rust/dist/pages`. The `build:rust` and `preview:rust` aliases remain compatible.

Drag the body to pull, drag empty space to orbit, and scroll to zoom. Space pauses, P triggers the physical striker, R resets, G toggles the ground, F focuses the body and Home resets the camera. The UI includes terrain stations, heading, region, renderer and quality controls. Live measurements are visible; `window.__lhRust.diagnostics` exposes worker and rendering diagnostics.

Grabs now deliver bounded external point forces into the GetUp multibody instead of changing a link velocity cache that the solver overwrote. The [native before/after evidence](docs/evidence/grab-multibody-2026-10-19.json) demonstrates hand-to-torso propagation and force removal on release. [Actual desktop WebGL2 browser verification](docs/evidence/browser-validation-2026-10-19.json) now covers physical hand dragging, release, pointer cancellation, blur, re-grab and reset, with native browser recordings. Auto checks a real WebGPU render submission before selecting that backend and falls back to WebGL2 when it cannot execute; explicit WebGPU reports the failure. These checks do not qualify recovery or real-time performance.

The [current-model contract audit](docs/evidence/getup-fixture-contracts-2026-10-19.json) distinguishes incompatible historical fixtures from current physical failures. Measured-contact joint balance passes bounded native and actual-browser standing trials. The corrected [free-root COM mapping](docs/evidence/getup-root-com-mapping-2026-10-19.json) also passes the unchanged ballistic-COM gate, including [three-heading browser observations](docs/evidence/browser-root-com-validation-2026-10-19.json). An explicit [800 Hz, 3+1 solver profile](docs/evidence/getup-3plus1-qualification-2026-10-19.json) passes native quiet-standing and bounded perturbation gates; three rendered, unencoded [33-second browser trials](docs/evidence/browser-3plus1-validation-2026-10-19.json) average 0.999× physical/wall speed on this desktop/headless executor. The public default profile remains unchanged. Mobile performance, latency tails and the broader historical native suite remain unqualified.

```sh
npm test                 # JavaScript tooling/fixture tests, no physical campaign
npm run verify:smoke     # built app; one trajectory, at most 5 simulated seconds
npm run verify:grab      # built app; real-pointer pull/release and interruption regression
npm run test:rust        # native workspace tests
npm run lint:rust        # Rust format and native/WASM Clippy
```

Install the smoke-test browser with `npx playwright install chromium`. `RUST_BROWSER_EXECUTABLE` can select an existing Chromium executable; `RUST_BROWSER_CHANNEL` supports an installed browser channel. `npm run verify:rust` runs the separate, extensive physical/release qualification and is **not currently passing**.

**Physical status:** pretrained recovery remains opt-in, and public recovery and corrective stepping stay disabled. The explicit [unit-root recovery experiment](docs/evidence/getup-unit-root-rotation-2026-10-19.json) passes four fresh native fixtures and four subsequent 30-second holds with every original 800 Hz standing predicate intact. Its [actual-WASM results](docs/evidence/browser-unit-root-rotation-recovery-2026-10-19.json) are tracked separately: three fixtures complete 30-second holds, while back reaches the unchanged 25-second deadline without standing. Earlier [consistent-budget browser results](docs/evidence/browser-consistent-solver-budget-2026-10-19.json) retain the separate quaternion-domain failure that motivated this experiment. No four-pose browser, repeated-cycle, 24-pose, impact-recovery or release qualification is claimed. Source-policy redistribution rights remain unresolved. Automatic CI checks the application; publication requires the separate accepted-artifact workflow. No deployment is implied by a build.

A native-only [stationary balance experiment](docs/experiments/getup-stationary-balance.md) exposes GetUp-style residual actions, a fixed-location objective and contact-aware evaluation for future SAC fine-tuning. A [bounded CPU SAC pilot](docs/evidence/getup-stationary-balance-pilot-2026-10-17/README.md) trained an experimental candidate, which was rejected on all four held-out comparisons. No learned stationary-balance controller is enabled in the application.

A [bounded actual-WASM frozen-feature adaptation pilot](docs/experiments/getup-wasm-policy-adaptation.md) also rejected its single candidate: the original BACK case retained zero qualifying measured interval, despite improvement on one narrowly related heldout state and preserved belly holds. The local correction head remains default-off; no trained weights are embedded or published.

Source lives in `rust/crates/`, with browser assets in `rust/browser/` and build/test helpers in `scripts/`. The legacy TypeScript application is retained in Git history. Existing reports and immutable experiment records under `docs/` are historical evidence, not current build instructions; start with this README.
