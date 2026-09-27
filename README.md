# Embodied Motion Physics Demo

A browser experiment in procedural humanoid balance, physical impacts, dragging, and recovery. One 25-segment dynamic Rapier assembly remains authoritative throughout the controller's standing, stepping, falling, and recovery states. Strong pulls can overpower bounded joint motors; recovery changes motor intent without reconstructing or teleporting the body. The same canonical convex geometry and physical snapshots drive collision, picking, Three.js WebGL2, and Canvas2D.

**Physics acceptance remains open.** Recorded failures include quiet standing, corrective stepping, and recovery. Successful rendering or an upright state alone does not establish acceptance. See [current status](docs/status.md) for the latest recorded checkpoint and evidence availability.

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

The **Balance playground** has seven selectable stations: a flat base camp, an incline with a neighboring cross-slope, rolling broken ground, a narrow elevated beam, staggered stepping stones, a pitching/rolling deck, and a hurdle/slalom lane. Choose a station to place the subject there and focus the camera. **Gentle**, **Challenging**, and **Extreme** adjust slope angles, terrain roughness, beam width/height, gaps, barriers, and deck motion. Changing stations or difficulty starts a new trial and preserves pause; **Reset** retries the current station. **Arena view** shows the course and **Focus body** brings the subject closer. The trial display measures continuous supported upright time, its best streak, falls, and loaded feet.

Both renderers use the same course geometry as the physical environment. Sloped and elevated support feeds the balance controller, and the moving deck advances on the physics clock, including pause/reset. Falls on difficult terrain can exceed the procedural controller's ability to recover; Reset remains available. The browser yields after a costly physics step to keep camera and trial controls responsive.

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

Start with the [documentation index](docs/README.md) and [current status](docs/status.md). See [ARCHITECTURE.md](ARCHITECTURE.md) for ownership rules, [balance](docs/balance-controller.md) and [dynamic recovery](docs/dynamic-recovery.md) for implementation details, and [physics acceptance](docs/physics-acceptance.md) for fixed scenarios. [Validation](docs/validation.md) preserves historical checkpoints; [evidence availability](docs/evidence.md) explains which supporting artifacts can be inspected.
