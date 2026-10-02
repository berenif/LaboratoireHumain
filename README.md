# Embodied Motion Physics Demo

A browser experiment in procedural humanoid balance, physical impacts, dragging, and recovery. One 25-segment dynamic Rapier assembly remains authoritative throughout the controller's standing, stepping, falling, and recovery states. Strong pulls can overpower bounded joint motors; recovery changes motor intent without reconstructing or teleporting the body. The same canonical convex geometry and physical snapshots drive collision, picking, Three.js WebGL2, and Canvas2D.

**Physics acceptance remains open.** Recorded failures include quiet standing, corrective stepping, and recovery. Successful rendering or an upright state alone does not establish acceptance. See [current status](docs/status.md) for the latest recorded checkpoint and evidence availability.

An isolated [Rust rework](docs/rust-rework.md) implements native geometry, contact, loaded-chain, quiet-standing and selected pull checks. The current v23 browser standing matrix passes five of six cases: +60° playground standing exceeds the 10 mm foot-drift limit. The Rust browser includes the physical striker and seven terrain stations, with scoped WebGPU/WebGL2 checks at root and subpath URLs. Full disturbance coverage, corrective steps, recovery, terrain traversal and release qualification remain incomplete. Production continues to use the TypeScript/Rapier implementation; its H78–H80 standing candidates failed feasibility.

The [prioritized TODO](TODO.md) tracks what is still missing: reproducible baseline evidence, accepted quiet standing, repeatable transfer and steps, then recovery/browser/release verification. It includes dependencies and completion criteria.

The simplified adult anatomy preserves a 1.84 m stature and 72.2 kg total mass. It includes a lumbar link, bilateral shoulder girdles, separate forearm-rotation links, and articulated ankle, hindfoot, and forefoot chains. Joint profiles define parent/child reference frames, permitted coordinates, asymmetric structural limits, passive resistance, damping, and actuator strength. These movements follow the broad anatomical categories summarized by the [OpenStax movement reference](https://openstax.org/books/anatomy-and-physiology-2e/pages/9-5-types-of-body-movements); the model intentionally does not represent individual fingers, individual vertebrae, or deformable tissue.

## Run locally

Use Node.js **22.13.0 or newer**.

```bash
npm ci
npm run dev
```

The standard development, build, start, and lint commands work on Windows, macOS, and Linux. They use the active Node executable and keep generated tool state inside the project. The optional `install:ci` helper is intended for Bash-based CI environments.

A source checkout runs without `.openai/hosting.json`. When that file exists, Vite reads its D1/R2 binding names; otherwise local bindings are empty.

Open the URL printed by Vite. The query string selects the experience:

| URL suffix | Experience and controls |
| --- | --- |
| `/` | The default French **Protocole d’arrêt** room. **Appliquer la procédure** (or P) requests one strike; **Rendu**, **Pause**, **Nouvelle session**, and the camera buttons control the session. |
| `/?mode=playground` | The **Balance playground**, with **View**, **Reset body**, station selection, difficulty, and trial metrics. |
| `/?mode=playground&qa=1` | The playground with optional browser verification controls. |

In the playground, use **View** to select Canvas2D or WebGL2. Drag the colored head, torso, pelvis, hands, or feet. Body dragging pauses during falling and recovery, then requires a fresh press after stable standing. Drag empty space to orbit, use Shift-drag to pan, and scroll to zoom. **Reset body** restores the body and camera and preserves the paused state. On a Pages project site, keep the repository subpath before the query string.

Both modes offer **Quality / Qualité** with Auto, Low, and High settings. Auto adjusts rendering resolution from frame intervals; quality changes preserve the simulation. Paused scenes redraw when the camera, viewport, or controls change. On narrow playground screens, **Stations** opens the station tray. See [rendering and performance](docs/rendering.md) for settings and measured limits.

The **Balance playground** has seven selectable stations: a flat base camp, an incline with a neighboring cross-slope, rolling broken ground, a narrow elevated beam, staggered stepping stones, a pitching/rolling deck, and a hurdle/slalom lane. Choose a station to place the subject there and focus the camera. **Gentle**, **Challenging**, and **Extreme** adjust slope angles, terrain roughness, beam width/height, gaps, barriers, and deck motion. Changing stations or difficulty starts a new trial and preserves pause; **Reset** retries the current station. **Arena view** shows the course and **Focus body** brings the subject closer. The trial display measures continuous supported upright time, its best streak, falls, and loaded feet.

Both renderers use the same course geometry as the physical environment. Sloped and elevated support feeds the balance controller, and the moving deck advances on the physics clock, including pause/reset. Falls on difficult terrain can exceed the procedural controller's ability to recover; Reset remains available. The browser yields after a costly physics step to keep camera and trial controls responsive.

## Run the Rust preview

With Node 22.13+ and rustup installed, bootstrap the pinned Rust/WASM, Trunk
and wasm-bindgen tools once. Setup fetches dependencies; subsequent builds
use the locked local cache:

```bash
npm ci
npm run setup:rust
npm run build:rust
npm run preview:rust
```

Open `http://127.0.0.1:5185/`. The preview includes the physical striker, all
seven terrain stations, WebGPU/WebGL2 rendering and worker lifecycle controls.
Corrective stepping, automatic recovery and real-time performance remain
unqualified. The normal `dev`, `build` and `start` commands run the existing
application until migration acceptance passes.

```bash
npm run test:rust
npm run lint:rust
npm run verify:rust:foundation
npm run verify:rust:browser
npm run verify:rust
```

`test:rust` runs the native workspace tests. `lint:rust` checks formatting and
all native and WASM crates. `verify:rust:foundation` preserves source and two
identical runs of the native foundation, terrain-contact and striker checks.
`verify:rust:browser` builds root and Pages-subpath artifacts, then tests
WebGPU/WebGL2, fault handling, worker lifecycle, all six flat-floor standing
trials, 42 terrain cases and the physical striker. Browser checks use
installed Edge on Windows, or Playwright Chromium elsewhere (`npx playwright
install chromium`). `RUST_BROWSER_CHANNEL` overrides that choice.

Both verification commands create a fresh timestamped directory under
`evidence/`; an explicit path can be passed after `--`. Full `verify:rust`
continues to exit nonzero while release requirements remain incomplete.
Foundation/browser successes do not authorize deployment. The preview now
reports measured falls and locks body dragging during them; Reset starts a
fresh trial. Automatic recovery remains unfinished. See the
[verification record](docs/rust-rework.md) for exact scope and remaining gates.

## Deploy to GitHub Pages

The repository includes a GitHub Actions workflow that builds a static Next.js export and deploys `out/` to GitHub Pages only after its required verification gates pass. The recorded physics failures remain release blockers. The workflow also applies the repository subpath automatically, so project sites such as `https://berenif.github.io/LaboratoireHumain/` load their scripts, styles, and favicon correctly.

1. In the GitHub repository, open **Settings → Pages**.
2. Set **Source** to **GitHub Actions**.
3. Push to `main`, or run **Deploy to GitHub Pages** manually from the Actions tab.

To test the Pages build locally at the domain root:

```bash
npm run test:pages
```

To reproduce a project-site subpath build:

```bash
PAGES_BASE_PATH=/LaboratoireHumain npm run test:pages
```

PowerShell equivalent:

```powershell
$env:PAGES_BASE_PATH = "/LaboratoireHumain"
npm run test:pages
```

The normal `npm run build` command still produces the existing Vinext/Sites build. Use `npm run build:pages` when you specifically need the static GitHub Pages artifact.

## Verify

```bash
npm run typecheck
npm run lint
npm test
npm run test:physics
```

`npm test` builds the application and runs the rendered-page, UI, domain, and runtime lifecycle tests. `test:physics` runs the deterministic anatomy, structural-limit, continuous-ownership, balance, fall, recovery, free-fall, contact, and input-isolation scenarios and writes its assessment to the ignored `evidence/physics-results.json`. Failures exit nonzero.

Add `?qa=1` to a URL with no query string, or `&qa=1` after `?mode=playground`, for optional browser verification controls. Automated replay has additional dependencies: follow [browser verification setup](docs/browser-verification.md). Synthetic pointer events and recorded snapshot playback do not establish native touch or GPU performance.

Versioned standing experiments use `npm run test:standing -- --controller=h80-v1 <fresh-output-directory>` (H74, H77, H78, and H79 are also selectable). H78–H80 require their frozen source and manifest locks; the current tree has subsequent renderer edits, so reproduce them from the [preserved workspace](docs/evidence.md#h78h80-local-evidence). Omitted controller selection defaults to H74. These failed experiments remain opt-in; see [standing selection and scope](docs/balance-controller.md#experimental-standing-selection).

The separate Rust evidence runner is `node scripts/run-rust-gates.mjs <fresh-output-directory>`. It requires the pinned Rust toolchain and locked dependencies described in the [Rust record](docs/rust-rework.md#reproduction-and-rollback), and exits nonzero while acceptance fails or is incomplete.

## Project structure

- `app/`: framework routes, layout, and global styling.
- `src/demo/`: application composition, browser runtime lifecycle, React adapter, and optional QA.
- `src/core/`: shared contracts, humanoid definitions, geometry, math, and fixed-step clock.
- `src/character/`: Rapier ownership, bounded balance and recovery controllers, grab diagnostics, and pure procedural pose composition.
- `src/interaction/`: browser pointer capture and queued grab intent.
- `src/scene/`: shared camera, camera controls, and snapshot rendering adapters.
- `src/ui/`: simulation controls; `components/ui/` contains the existing shared UI catalog.
- `tests/` and `scripts/`: regression checks, physics scenarios, and local tooling.
- `rust/`: isolated native model, contracts, simulation, acceptance runner, and audited Rapier fork; not the browser entrypoint.
- `build/`, `worker/`, `db/`, and `examples/`: hosting integration and optional starter infrastructure.

Start with the [documentation index](docs/README.md) and [current status](docs/status.md). See [ARCHITECTURE.md](ARCHITECTURE.md) for ownership rules, [balance](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for implementation details, and [physics acceptance](docs/physics-acceptance.md) for fixed scenarios. [Validation](docs/validation.md) preserves historical checkpoints; [evidence availability](docs/evidence.md) explains which supporting artifacts can be inspected.
