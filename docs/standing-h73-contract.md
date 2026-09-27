# H73 — extend failed coupled searches in the fast candidate

H69 reduces queries substantially but fails two headings. H71 identifies a
selection defect when a physically admissible candidate misses a stricter
internal foot margin. H72 finds admissible local corrections at all three
examined first failures by extending the copied search to sixteen iterations.

Keep H69's warm-seed and single-iteration horizontal cache behavior unchanged.
Replace only its coupled fallback: run the original first eight iterations;
if any candidate meets the original preferred guard, keep the original result.
Otherwise continue the same search for at most eight more iterations. Continue
targeting the same reserve. Prefer candidates meeting the original foot margin;
if none exists after the extension, select the lowest unchanged objective among
actually evaluated candidates meeting all original physical speed bounds.
Retain the original command only when no such candidate exists.

Distinguish preferred-guard misses, physically infeasible searches and physical
fallback selections in telemetry. Record both injected modules, loader, runner,
their hashes and complete source with every invocation. The original H66 and
H70 histories remain unchanged. The sixteen-iteration limit applies only to
copied search, never to native solver settings.

Require exact disabled thirty-step H66 compatibility and exact replay of H72's
local extended queries through the new helper. Then screen all three original
headings for 2+10 seconds. Before each first changed fallback, require the full
H69 native/body/bias/query prefix to match. Preserve every failure and original
physical criterion. No long evaluation follows unless all short screens pass.

## First attempt — memory failures, no complete screen

The helper replays all 3,695 H72 search queries exactly, and the disabled
candidate reproduces thirty H66 steps. The [verified preflight archive](checkpoints/2026-09-27/h73-preflight/manifest.json)
reconstructs thirteen originals from fourteen parts. All three concurrently
launched enabled processes later exit 134 with explicit allocation/heap memory
errors; none writes a final simulation report. Preserve all partial outputs.

Repeat in fresh directories with one physical simulation at a time and a
documented 384 MiB Node old-space limit. Record the child PID and runtime flag.
First recheck disabled compatibility under that runtime bound. The preview and
coupled-helper sources remain identical to the failed attempt; the wrapper
adds runtime provenance. Native physical settings and all criteria stay fixed.
