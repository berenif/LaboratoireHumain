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

```sh
npm test                 # JavaScript tooling/fixture tests, no physical campaign
npm run verify:smoke     # built app; one trajectory, at most 5 simulated seconds
npm run test:rust        # native workspace tests
npm run lint:rust        # Rust format and native/WASM Clippy
```

Install the smoke-test browser with `npx playwright install chromium`. `RUST_BROWSER_EXECUTABLE` can select an existing Chromium executable; `RUST_BROWSER_CHANNEL` supports an installed browser channel. `npm run verify:rust` runs the separate, extensive physical/release qualification and is **not currently passing**.

**Physical limitations:** the browser automatically attempts get-up after a fall; successful standing is not established. Corrective stepping remains disabled. Full get-up, the 24-pose recovery suite, cycles and impacts are not qualified. Explicit test profiles retain their own recovery setting. Automatic CI checks the application; publication still requires the separate accepted-artifact workflow. No deployment is implied by a build.

Source lives in `rust/crates/`, with browser assets in `rust/browser/` and build/test helpers in `scripts/`. The legacy TypeScript application is retained in Git history. Existing reports and immutable experiment records under `docs/` are historical evidence, not current build instructions; start with this README.
