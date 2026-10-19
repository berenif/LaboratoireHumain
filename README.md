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

The [current-model contract audit](docs/evidence/getup-fixture-contracts-2026-10-19.json) rejects incompatible historical 60 Hz disturbance inputs, halts safely on lost body ownership and restores both feet to the unchanged standing-orientation guard. Default flat-floor standing now uses measured-contact joint balance: the [native before/after campaign](docs/evidence/getup-standing-balance-2026-10-19.json) records 33 physical seconds at 0° and 12 seconds each at 60° and -45°, passing every unchanged 800 Hz standing predicate from 0.5 seconds onward. The longest run includes 3 seconds settling plus 30 continuous seconds quiet, with cumulative loaded slip below 2.46 mm per foot against the unchanged 20 mm stop. These are deterministic standing results; the broader native suite remains unqualified, and the original 0.1 mm ballistic-COM gate still fails. The final controller inventory adds no new failure names: 84 native tests pass, 79 historical tests fail and 12 existing tests remain ignored. An [ordinary-app browser hold](docs/evidence/getup-standing-balance-2026-10-19.json) also passes 3 seconds settling plus 30 seconds quiet at snapshot level, but runs at 0.176× simulation/wall speed on the uncontended desktop/headless executor. Real-time usability remains a blocker.

```sh
npm test                 # JavaScript tooling/fixture tests, no physical campaign
npm run verify:smoke     # built app; one trajectory, at most 5 simulated seconds
npm run verify:grab      # built app; real-pointer pull/release and interruption regression
npm run test:rust        # native workspace tests
npm run lint:rust        # Rust format and native/WASM Clippy
```

Install the smoke-test browser with `npx playwright install chromium`. `RUST_BROWSER_EXECUTABLE` can select an existing Chromium executable; `RUST_BROWSER_CHANNEL` supports an installed browser channel. `npm run verify:rust` runs the separate, extensive physical/release qualification and is **not currently passing**.

**Physical status:** pretrained GetUp recovery remains opt-in and public recovery stays disabled. A [single deterministic prototype](docs/evidence/getup-complete-replay-2026-10-16.md) got up after impact and held the native standing criterion for 3.009 continuous seconds. A [three-pose check](docs/evidence/getup-three-poses-2026-10-16.md) reached the native one-second criterion from back and belly fixtures, but both lost current stability by the final sample; the proposed side fixture rolled and was not admitted. A [read-only diagnosis and corrected side attempt](docs/evidence/getup-side-completion-2026-10-16.md) identify the final back speed excursion and belly support-hull loss; the single corrected side posture also settled outside the unchanged lateral orientation criterion. A [grounded lateral fixture](docs/evidence/getup-lateral-grounded-2026-10-17/summary.json) subsequently reached native one-second standing12.75125s after triggering; it remained stable at the final13.025s physical sample. A [checkpoint continuation](docs/evidence/getup-standing-hold-2026-10-17/summary.json) then advanced another five physical seconds with the controller unchanged: all 4,000 native substeps passed the existing standing predicates and continuous stability reached 6.02375 seconds. These results do not qualify the 24-pose recovery suite, cycles or impacts. Corrective stepping remains disabled. Automatic CI checks the application; publication still requires the separate accepted-artifact workflow. No deployment is implied by a build.

A native-only [stationary balance experiment](docs/experiments/getup-stationary-balance.md) exposes GetUp-style residual actions, a fixed-location objective and contact-aware evaluation for future SAC fine-tuning. A [bounded CPU SAC pilot](docs/evidence/getup-stationary-balance-pilot-2026-10-17/README.md) trained an experimental candidate, which was rejected on all four held-out comparisons. No learned stationary-balance controller is enabled in the application.

Source lives in `rust/crates/`, with browser assets in `rust/browser/` and build/test helpers in `scripts/`. The legacy TypeScript application is retained in Git history. Existing reports and immutable experiment records under `docs/` are historical evidence, not current build instructions; start with this README.
