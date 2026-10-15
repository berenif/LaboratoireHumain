# Observed-command raccord: pure validation

The three selected Rust tests passed: **3 passed, 0 failed, 237 filtered out**, in
0.21 s after a 10.96 s compilation. The run used only the
`recovery::planner::observed_raccord::tests` filter, one test thread, Rust 1.89.0,
and `experimental-solver-step-observation`. See `observed-raccord-tests.log`.
There were **zero native integrations and zero FK evaluations**. Rustfmt was
applied to `observed_raccord.rs`; no production behavior was changed in this
validation pass.

The tests load Model metadata with serde, compile the existing command Program,
and exercise the pure raccord helper. They verify preservation of the archived
nonzero entry dq, exact f32 finite-difference dq for every bridge command, absent
and stale readiness rejection, absolute deadline rejection, the additional
standing second, and refusal of nonfinite or anatomically overshooting inputs.

## Entry and meaning of the reference

The input is `observed-entry-step36.json`: the **last installed command at 35**,
with next command clock 36. Its maximum absolute dq is
`0.27974367141723633 rad/s`. No measured body angle replaces that command.
Bridge index zero retains this historical q/dq unchanged and is not an extra
command. Index one is the first new command at native clock 36.

Actual A36 is **not** asserted to satisfy the heel-stable policy. The complete
continuation test supplies a clearly synthetic readiness receipt to exercise
the interface. In production, the full raccord must be prepared from the
latest installed q/dq when the policy supplies a current readiness receipt;
waiting for an event does not make the old A36 entry current again.

## Duration and finite reference

For the stationary target at source clock 432, the Hermite derivative has
quadratic Bernstein controls `[v0, 3*delta/T - v0, 0]`. The helper derives one
sufficient duration bound from these controls, applies a fixed one-second
floor, rounds up to the native grid, and refuses failed validation without a
duration sweep. For this entry, the necessary displacement/rate lower bound is
`0.1775752305984497 s`; the sufficient Bernstein duration floor is
`0.41627530863691525 s`. The selected bridge has **240 substeps**, nominally 1 s
(`1.0000000521540642 s` using the stored f32 native dt).

| Reference boundary | Native command/observation clock |
| --- | ---: |
| Historical entry, never reinstalled by index zero | command 35 |
| First bridge command | command 36 |
| Last bridge command, source 432 position | command 275 |
| First tail command, source 433 position | command 276 |
| Standing reference, source 1680 | command 1523 |
| Standing reference has completed its integration | observation 1524 |
| End after the additional 240 stationary commands | exclusive clock 1764 |
| Absolute deadline, origin zero | exclusive clock 6000 |

The complete reference contains **1,728 new commands**, nominally **7.2 s**.
With this test's entry at 36, it ends at nominal absolute time **7.35 s**. A
different event entry requires a new preparation from that entry; these durations
and the derived latest entry clock 4272 apply to this particular command set.
The finite sampler refuses requests before its start or at/after its exclusive
end; it cannot silently extend the last command forever.

## Export provenance and numerical scope

`observed-raccord-export.json` contains all 1,729 command records including the
historical entry, all 46 axis summaries, bridge polynomial coefficients, timing,
rates and hashes of the inputs, helper, test log and exporter. It was produced by
`export_observed_raccord.py`, an **independent offline arithmetic reconstruction**
of the helper and the existing Program's f32 operation order. It is not presented
as a Rust runtime export or as a physical observation.

The reconstruction reports a bridge continuous maximum rate of
`0.27974367141723633 rad/s` and a maximum emitted f32 rate over both bridge and
tail of `0.2796292304992676 rad/s`. It independently checks every emitted target
against the anatomical limits, the unchanged 1 rad/s slew/rate limit, and exact
f32 finite-difference dq. The helper also evaluates continuous cubic extrema in
f64; this is a floating-point check, **not an interval or rational certificate**
for every real time between samples.

No native parity, retained-support feasibility, stable heels, standing recovery,
or subsequent physical execution is established by this command-only result.

Export SHA-256:
`52d25c09a3c13ccb586420c759662724ff4887ce94cc9ae948cfcd2451a5108f`.
