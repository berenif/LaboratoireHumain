# Source-SLOW observation-aliasing audit

## Finding

The source118 observation omits task variables that determine reward and termination. This is a genuine partial-observability / state-abstraction modeling risk. Exact reachable aliases, or a defect causing this particular failure, have not been established. The closed replay does **not** demonstrate a near-identical observation-and-residual-action collision that causally explains the endpoint critic's optimism.

At cumulative5000, 10,034,740 eligible cross-episode pairs contain zero strict or tight matches. The nearest joint match still differs by 90.75 mrad RMS in residual action. Endpoint critic optimism relative to complete logged soft behavior returns is substantial, especially near observed failure, but off-policy behavior returns do not identify optimal Q, current-policy Q, or the unique cause of the error.

## Scope, integrity, and computation

- Old closed stage: 1580 replay rows, 48 completed episodes, 1573 complete-return rows. Seven rows in its unfinished episode continue into the next stage.
- Closed continuation: 3420 new rows. Combined replay: 5000 rows, 510 completed episodes, 4999 complete-return rows. One final live row is excluded from return comparisons.
- Terminal reasons over the 510 completed episodes: 330 fixed-anchor drift, 179 loaded slip, one flight. No deadline or quiet-success terminal was observed.
- Exact CPU float32 equality was checked between all five checkpoint replay columns and the concatenated old/new transition streams. Consecutive state/next-state continuity, episode return reconstruction, fixed reference equality, endpoint packet observations, monotone cumulative slip, and reconstructed foot-displacement/drift agreement were also checked. Maximum complete-return reconstruction error is below 1e-10.
- Checkpoint loads use torch.load(map_location='cpu', weights_only=True). Critic values use functional linear/ReLU inference from the stored tensor dictionaries. No SAC/module/optimizer/environment constructor, optimizer step, physics integration, policy sampling, source-runtime import, or checkpoint restore is used. One process CPU affinity and one BLAS/Torch thread are enforced.
- Input manifests record exact SHA256/size/mtime after both summary and closed ledger existed. Neither open transition stream was treated as immutable. Fixed milestone5000, its native/controller files and manifest were used, rather than a validation-world cache. Writes are confined to this audit directory.

## What source118 observes and what it omits

`slow_task.py:24-32` overrides the generic stabilization observation with physical67, the original standing target25 twice, and power1. `source_math.py:46-48` concatenates those blocks. All 51 conditioning coordinates are exactly constant in the audited replay. This original actor target is distinct from the measured-entry task reward target.

`source_task.py:120-124,138-150,166-180` maintains external cumulative per-foot loaded slip, fixed prospective foot anchors, support persistence, consecutive quiet count, elapsed/reference clock, previous controls and event history. Slip over10mm or fixed-anchor drift over10mm terminates. Quiet completion needs4000 consecutive800Hz samples. Non-success at8s times out. None of these counters/budgets is added by the SLOW observation override. Physical67 also omits planar free-root position: a flat-floor planar translation preserves its joint, root-local geometry, height, vertical, and velocity features while changing displacement from fixed world anchors.

The same reset and entry anchors are reused here, so the anchor *constants* and measured reward reference do not vary across episodes. Their current world-relative displacement and cumulative history do vary. These must not be conflated.

## Pair design and normalization

Primary eligibility: different episodes, one unordered pair per row pair, pre-action elapsed>=25ms (the first non-reset decision). All511 initial/reset rows are excluded. Exact identical(state,action) pairs are separately excluded, and same-episode temporally adjacent pairs are never counted. A >=100ms sensitivity is also supplied. It gives the same no-strict/tight-match conclusion.

Normalization is fixed engineering-unit scaling, not fitted to outcomes:

- Joint angles0:21: 0.1rad; head height21: 0.04m
- Root-local limb positions22:34: 0.02m; torso vertical34:37: sin(5deg)
- COM velocity37:40 and free-root linear velocity40:43: 0.2m/s
- Free-root angular velocity43:46: 0.65rad/s; joint velocities46:67: 0.5rad/s
- Residual actions: 0.1rad per coordinate

Physical distance is RMS of the67 standardized differences; action distance is RMS of21 standardized residual differences. The constant reference/power block must match exactly and is not allowed to dilute the distance denominator. Joint nearest-neighbor distance is max(physical RMS, action RMS). Maximum-coordinate caps prevent a single large mismatch being hidden by RMS.

Strict: both RMS<=0.25, both coordinate maxima<=1. Tight: both RMS<=0.5, coordinate maxima<=2. Exploratory: both RMS<=1, coordinate maxima<=3. These are descriptive sensitivity thresholds, not a learned equivalence tolerance. Material contrasts are per-foot slip or fixed-world displacement differences>=2mm, elapsed difference>=250ms, quiet difference>=80samples, soft-return gap>=5, and differing immediate/within100ms observed failure.

| Closed data | Eligible rows | Compared pairs | Strict | Tight | Exploratory |
|---|---:|---:|---:|---:|---:|
| Old1580 |1531|1,147,216|0|0|4|
| Cumulative5000 |4489|10,034,740|0|0|6|

Among six exploratory5000 pairs, two have>=2mm slip gaps, four>=2mm fixed-world displacement gaps, two>=250ms elapsed gaps and two>=5 soft-return gaps. None contrasts immediate or within100ms failure. These are coarse matches: action RMS differences are roughly91-97mrad and may themselves materially affect outcomes. They are not evidence of identical-action aliasing.

A qualifying coarse example is replay424 versus520: normalized physical RMS0.4574, action RMS0.9423 (94.23mrad); pre maximum slip1.576 versus4.992mm; logged soft returns16.014 versus1.664. Different actions and subsequent changing behavior policies prevent causal attribution to hidden slip.

The closest pair overall is4391 versus4518: normalized physical RMS0.6459, residual RMS90.75mrad; maximum normalized physical coordinate3.116, so it even fails the exploratory coordinate cap. The closest immediate-terminal/nonterminal contrast is985 versus1305 and still requires106.13mrad RMS action difference. There is no tight collision to present as proof.

Clock limits are particularly untested empirically: pre elapsed never exceeds1.0s against an8s deadline, and pre quiet count never exceeds60 against4000. Absence of quiet/timeout collisions in this replay says almost nothing about those omitted boundaries.

## Endpoint critic calibration, carefully interpreted

Complete logged behavior returns are reconstructed with gamma0.97 and the recorded source-epsilon log probability, using S_t=r_t+gamma*(S_next-alpha*logged_logpi_next), terminal S=r, alpha0.000612579835321518. Administrative/live endings are not assigned zero returns. These trajectories use changing behavior policies.

| Endpoint / subset | n | Mean min(Q1,Q2) | Mean logged soft return | Mean difference |
|---|---:|---:|---:|---:|
| Old1580 endpoint, complete rows |1573|2.756|4.871|-2.115|
| Fixed5000 endpoint, complete rows |4999|6.193|-1.744|+7.937|
| Fixed5000, observed failure within100ms |1897|3.304|-8.197|+11.502|
| Fixed5000, immediate terminal transitions |510|-1.160|-9.619|+8.459|
| Fixed5000, more than100ms to observed failure |3102|7.959|2.203|+5.757|

Overall fixed5000 RMSE against these logged soft returns is9.935. Immediate terminal targets equal the realized reward and do not bootstrap, making their large fitted residual direct evidence of endpoint approximation/calibration error on those recorded pairs. It still does not identify the cause, isolate missing state, or estimate population bias from a single changing-policy dataset. The old endpoint's average discrepancy had the opposite sign despite the same observation contract.

## Mathematical state-abstraction contract

Let augmented task state z=(x,L,A,t,k,f,u_prev,H,...) and observation phi(z)=(physical67(x),r_hold,r_hold,1). A Markov state abstraction would require the conditional law of (reward,termination,next observation), given action a, to agree for any admissible z1,z2 with phi(z1)=phi(z2). This invariance is not guaranteed by the implemented contract:

1. **Cumulative slip.** L_next=L+sum(v_loaded*dt). For an identical physical state/action with a0.2mm next-sample increment, admissible budgets L=1.0mm versus9.9mm yield1.2mm versus10.1mm. One crosses the physical terminal threshold and incurs a full failure penalty/mask0; the other does not. L never enters phi. At policy intervals the early physical stop additionally changes actual duration.
2. **Fixed world anchor.** A planar translation of the current physical state leaves physical67 unchanged but changes current foot positions minus fixed prospective anchors. States just below the10mm boundary can therefore have different terminal proximity and task foot-position errors under the same observed action.
3. **Elapsed time.** With the same physical observation/action and no quiet success, t=7.99s versus t=0.1s can cause a25ms request to encounter the8s terminal deadline only in the first case.
4. **Quiet history.** With identical current physical state/action and20 subsequent quiet samples, k=3990 versus0 yields an absorbing quietComplete/+5 event only for the former. Foot-load persistence and previous controls/event history create further reward/gate dependence absent from phi.

These are conditional contract counterexamples, not claims that this replay contains those exact pairs or that both histories have been proved reachable from this one reset under current dynamics. They show why finite failure to find a collision cannot certify source118 as Markov. Conversely, a coarse pair with different action/future behavior does not prove omitted history caused its return gap.

## Conclusion

Keep the two claims separate: **history omission is real; causal attribution of current optimism to that omission is unproved.** The endpoint critic has a large observed terminal-proximity calibration gap, while the replay is sparse in joint observation/action space. No current policy rescue, safety relaxation, observation expansion, or learner mutation follows from this audit. A future augmentation would require a separately reviewed contract and new evidence; cumulative slip budget, world-relative foot displacement, elapsed/quiet/persistence state and previous-control terms are the directly task-relevant quantities to consider.

## Files

- `audit.py`: reproducible closed-artifact computation
- `closed1580-audit.json`, `cumulative5000-audit.json`: full metrics, normalization, nearest/qualifying witnesses and conditional calibration
- Corresponding `*-post100ms-audit.json`: reset-exclusion sensitivity
- `*-input-manifest.json`: exact closed input hashes and script hash
- `*-transitions.csv`: reconstructed pre-action hidden quantities, endpoint Q, complete returns and observed terminal proximity
