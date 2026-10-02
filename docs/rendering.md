# Rendering quality and performance

Both public modes use the same physics snapshots and shared camera for WebGL2
and Canvas2D. Rendering quality changes presentation resolution and WebGL shadow
resolution; it does not change the physical model or timestep. Default quality
is Auto, and switching renderers retains the selection.

## Quality controls

| Setting | Pixel-ratio cap | WebGL shadow map |
| --- | --- | --- |
| Low / Basse | 0.75 | 512 × 512 |
| High / Haute | 1.5 | 1024 × 1024 |
| Auto | Starts at 1.25; adapts between 0.75 and 1.5 | 512 below 1.25, otherwise 1024 |

Effective pixel ratio also respects the device ratio. Auto uses a rolling window
of frame intervals: sustained means above 20 ms lower the cap, while sustained
means below 17 ms raise it. Adjustments occur in 0.25 increments with hysteresis
and warmup exclusions around runtime changes. These intervals include physics
cost, so a lower quality setting does not guarantee real-time simulation.

Paused scenes schedule a redraw for camera, viewport, quality, reset or renderer
changes. They do not continuously render an unchanged paused scene. Both adapters
reuse interpolation buffers. WebGL batches opaque scenery, instances joint
markers and deduplicates resource disposal. Canvas2D caches static geometry and
projection commands while updating moving pieces from simulation snapshots.

## Diagnostics

`window.__EMBODIED_DEMO__.presentation()` exposes presentation CPU milliseconds,
effective pixel ratio, shadow resolution, draw calls, geometry and texture counts.
WebGL includes primitive counts; Canvas2D adds geometry/projection cache-build
counts and reports zero WebGL draw calls, textures and shadow resolution.
`quality()` returns the selected quality; `diagnostics()` includes these values.

Presentation CPU timing includes interpolation and render submission, not GPU
execution. WebGL draw-call counts include shadow passes. Use `timing()` to inspect
physics step cost, dropped time and simulation/wall-time ratio separately.

## Recorded browser checkpoint

The local `evidence/sandbox-acceptance/report.json` was started on 2026-10-01
(Europe/Paris) on Windows with an i7-8750H and headless Edge 154.0.4258.37. It
records 16 replay combinations (two modes, two renderers, two viewport sizes,
before/after), four UI combinations and four live trials. The report has no
errors and unchanged fingerprints; all 18 recorded scene/runtime/core hashes
match the reviewed tree. This is a partial application fingerprint.

| Live trial | Mean FPS | Presentation CPU p95 | Mean sampled physics step | Simulation / wall time |
| --- | ---: | ---: | ---: | ---: |
| Protocol / WebGL2 | 24.93 | 1.6 ms | 38.70 ms | 0.416 |
| Protocol / Canvas2D | 28.95 | 2.4 ms | 32.72 ms | 0.478 |
| Playground / WebGL2 | 44.74 | 1.6 ms | 17.70 ms | 0.746 |
| Playground / Canvas2D | 44.58 | 2.8 ms | 17.32 ms | 0.739 |

Each live sample covers approximately 60 s after warmup, at 1366 × 768. Protocol
trials request a strike; playground trials use the moving-deck scene. These
physics samples read the last completed fixed-step duration each frame, rather
than the sum of all catch-up work. The measurements show low presentation CPU
cost alongside slower-than-real-time physics. The report explicitly leaves
reference-laptop qualification false;
neither headless frame rates nor recorded pose replay qualify GPU execution,
native touch, sustained hardware performance, or physics acceptance.

## Reproduction

Follow [browser verification prerequisites and renderer commands](browser-verification.md#renderer-replay-ui-and-live-checks).
`scripts/rendering-browser-check.mjs` starts its own Vite server and saves
`report.json`, pose recordings and screenshots. It supports stage selection,
live duration, saved poses and an optional preserved baseline source tree.

For focused implementation checks, run:

```sh
node --test tests/rendering.test.mjs tests/demo-runtime.test.mjs tests/ui-components.test.mjs
```

These checks cover rendering/lifecycle/UI behavior; passing them does not resolve
the [current physical and release gates](status.md). Preserve failed or partial
runs according to the [evidence guide](evidence.md#preserving-future-results).
