# Original 800 Hz SLOW stabilization viability

Status: closed after the preregistered actor-drift guard. The source baseline
remains selected. No quiet-standing or forward-step qualification was obtained.

## Scope and accounting

The separately admitted local stage began2026-10-18 at01:07:10 UTC with a
01:37:10 deadline. Setup, code, fixtures and reviews counted against30min wall.
The10s physics allocation protected1s benchmark,6s collection/training and3s
validation, with300wall seconds and640actual policy intervals reserved for the
whole frozen/candidate validation pair. The candidate was limited to356new actual
transitions and at most25actor updates. Earlier failed fit/analytic-torque budgets
were not reused.

The stage actually closed at01:26:06.814 UTC. It used0.40306047s aggregate physics,
6538actual800Hz integrations and328policy intervals. Candidate updates stopped at
100critic-only calls plus2source SAC calls, each containing critic/actor/alpha
optimizer steps:106charged candidate optimizer calls. The16benchmark critic
updates were separate and discarded. All requests and optimizer charges are
resolved. Unused allowance did not release a retry or another initializer.

## Source mechanism preserved

The experiment uses original MuJoCo2.1.3 `humanoid_high_freq.xml`,800Hz inner PD
and40Hz qpos-order residual-angle actions. Source standingq, actor terminal-height
sentinel1.4, torso vertical[0,0,1] and power1 remain the actor's now/ahead reference.
The measured COM and task reward reference remain physical quantities.

Referenceq and residuala are held for20native samples. Each sample refreshes
actualq/dq and applies `clip(qref+a-q-.2*dq,-1,1)[ORDER]`, with original gear once.
The initial actor mean/trunk tensors and identical-Torch-batch1 outputs match the
imported source SLOW actor exactly. Only its logstd output head was initialized
to latentstd0.02. Fresh critics/targets, optimizers and alpha0.1 were declared.

The actual8s SLOW endpoint is an explicitly training-only initial cache: the
original source run was upright but had20.59mm anchor drift. No already-quiet5s
condition was required to learn stabilization. The same complete800Hz cache was
restored; there was no200Hz switch, physical pose injection or helper root force.
Prospective episode anchors did not retroactively qualify the original run.

## Interface benchmark passes

The64-command benchmark completed1280charged native integrations, including a
charged full-cache repeat of the first20-sample command. All repeated physical
packets and complete controller results are bit-exact. Every recomputed original
motor torque equation has0Nm residual. No physical stop occurred.

It took12.0878s wall and0.0839616s physics, about5.2946complete policy intervals/s
including decoder/journal work and16discarded critic-only timing updates. The
retained branch reached1.575s with loaded slip L5.633/R5.425mm and1181consecutive
quiet samples. This is an interface/throughput result, shorter than5s.

## Candidate optimization and guard outcome

The reviewed runtime was separately hash-admitted over the immutable default-
disabled equation preset. The entire actor and alpha stayed unchanged during
256actual replay transitions and100critic-only updates. Targets received explicit
Polyak updates after each critic call. Actor/critic/alpha updates then used the
unchanged source SAC equations once per4NEW transitions.

- Critic-only loss:34.89795 initially →0.202359 at update100
- Actor update1, transition260: critic loss0.157515; actor loss10.75818;
  deterministic residual drift RMS0.0167653 / maximum0.0310502rad;
  maximum probe latentstd0.0200630; alpha0.1000100
- Actor update2, transition264: critic loss0.908347; actor loss10.75167;
  drift RMS0.0335580 / maximum0.0621635rad; maximum std0.0201264;
  alpha0.1000200

Both fixed drift limits,0.02rad RMS and0.05rad maximum, were exceeded at update2.
The failed weights were retained locally and rolled back from selection. Actor2
was not given a further physical action or a validation rollout. There was no
threshold change, extra update, automatic fallback or reuse of validation reserve.

The264actual transitions contain262full20-sample nonterminal intervals and two
finite loaded-slip terminals of13and5samples. Both terminal masks are0; no
physical failures were censored. The first left-slip crossings were10.003997mm
at2.76625s and10.000857mm at2.75625s. No warning, flight or nonfoot sample occurred.
All benchmark and training torque equations have0Nm error across6538integrations.

No paired validation was released after the actor guard failed. The latest saved
source/learner/controller checkpoint is a complete training boundary at source
ID7240, completedSamples20 and episode time1.05s. Its quiet run is761samples,
short of4000. There are no validation rows, quiet5s certificate or forward-step
result.

## What the evidence establishes

The source800Hz residual mechanism avoids the immediate200Hz held-torque cliff.
The bounded full-network SAC adaptation still moves the pretrained action means
too far under the declared guard. Low critic loss alone does not establish a
reliable policy gradient. Layer/coordinate and entropy-versus-Q gradient reduction
is performed from saved checkpoints with no new engine or optimizer steps.
Any trust-region, parameter restriction or source-temperature warm start is a
separate design requiring review and fresh admission.

The116Python tests pass. Fixtures cover original batch1 mean parity, actual
velocity refresh within20held-residual substeps, full/partial terminal masks,
full-cache controller restore, and stochastic next-action/update equality after
populating all three Adam states, target critic, alpha and replay/RNG state.

Code lives in `scripts/getup-step/slow_task.py`, `slow_learning.py`,
`slow_benchmark.py` and `slow_train.py`. Compact own receipts are in
`docs/evidence/getup-source-slow-stabilization-2026-10-18/`. Original and derived
weights, raw journals and full physical caches remain local. Original policy
assets and application defaults were not overwritten.


## Exact causal reduction after rollback

An independent pure audit restored the saved replay/Torch RNG and replayed four
recorded collection actions exactly. The first critic loss0.1575152725 and actor
loss10.7581768036 close with ZERO numerical error. The actor loss is entropy
5.27793598 plus negative-Q5.48024082. No optimizer or engine call was used.

Mean-affecting parameter gradients have Q norm7.65320 versus entropy3.08652,
cosine0.74424. The total entropy norm9.35120 largely belongs to the std head;
it cannot be used to claim entropy alone caused the mean shift. On actual update2
parameter deltas, directional projections attribute74.35% of observed mean drift
to the second hidden layer,16.51% to the input hidden layer and8.54% to the mean
head. Their local sum reconstructs output drift within1.11% RMS. Total parameter
L2 change is only0.01436%, despite0.033558rad action RMS change. First-step Adam
algebra reproduces the actual parameter deltas within1.86e-9.

The pinned original SLOW log-alpha is−7.397831280657282, giving
alpha0.000612579835321518: fresh0.1 is163.244× larger. Original learned latentstd
on the same201×21probe values is0.09109–0.20596, median0.16321; all exceed the reset
0.02. A pure fixed-critic original-alpha reweighting reduces entropy loss to0.03233
but leaves Q gradient7.65320 unchanged. It does not predict retrained critics or
physical recovery. The local audit exposes a substantially changed warm-start
objective and strong shared-trunk sensitivity; neither lower alpha alone nor a
larger drift threshold is established as a remedy.

Full own numerical receipts are in `causal-gradient-audit.json`. The next design
is separately proposed in plan§13: original fixedtemperature, frozen trunk/std,
mean-head-only projected Adam with finite-witness Gaussian-KL backtracking.
The closed stage executed none of those proposals.
