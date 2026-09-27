# H71 — retain a physically admissible fallback

The first failed H66 long step, heading 0 at tick 978, contains 58 copied
candidates within the original 0.1 m/s and 0.5 rad/s physical bounds. None
meets the search's stricter 0.4 rad/s foot guard, so the current helper returns
the uncorrected baseline: 0.1652 m/s and 1.7966 rad/s. The best recorded physical
candidate reaches 0.09745 m/s and 0.46822 rad/s instead.

Before changing selection, replay that candidate on its saved native state.
Repeat the same comparison at the first speed failure of each rejected H69
heading. Keep the original baseline, original selected command and native
motor parameters exact. If a physically admissible recorded candidate exists,
choose the lowest unchanged coupled objective among them, replay its complete
25-body output and repeat it independently. Report explicitly when none exists.

This local diagnostic distinguishes the preferred search margin from actual
physical acceptance. It changes no motor ceiling, solver setting, timestep,
collision rule, ownership rule or acceptance threshold. No new native command
is applied to any running history. Preserve all original failures and source.
Local admissibility alone does not establish subsequent standing or performance.

## Result — selection defect confirmed in two states

H66 tick 978 has 58 physically admissible recorded candidates. Its best replays
at 0.097454 m/s and 0.468220 rad/s. H69 −π/4 tick 497 has two; its best replays
at 0.096095 m/s and 0.427769 rad/s. Both miss the preferred foot margin while
meeting the unchanged physical limits. H69 +π/3 tick 621 has none, so selection
alone cannot repair that local search.

All ten copied checks match their original 25-body digests, including baseline,
retained command and both repeated feasible candidates. Native inspection checks
102 motor axes. No running simulation changes. The
[result archive](checkpoints/2026-09-27/h71-physical-fallback/manifest.json)
retains all input rows, native states, provenance, outputs and verifier source.
