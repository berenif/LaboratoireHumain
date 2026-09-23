# Physical-chain investigation — September 18, 2026

## Release status: blocked, draft PR #3

This is a production-source investigation checkpoint, **not an accepted balance release**. Do not merge until the full 510-tick recovery, measured touchdown, collision, orientation, regression, and interaction gates pass.

## Identity and persistence

- Verified starting main: `8cdd7f06d4d83abcf165adfda93e1954e67d6a5c`, merged diagnostic PR #2.
- New working branch: `agent/physical-chain-frame-repair`.
- Production-source commit: `99c74f4906162fcac15404b33d3dcb1ce3aff62a`, exact audited tree `0c23f33bd2114cb766fdd0ec8203ea56870cb27f`.
- Diagnostic type-import correction: `135f8f29785a6387cbbe0c99959bac81dfded01d`.

No main writes, reset, force push, collision-filter relaxation, direct root transform correction, or kinematic parenting was used. Temporary source-transport workflows are removed after publication. The original historical diagnostic patch script remains available but **is not used to prepare source in verification CI**.

## Baseline reproduction

GitHub Actions baseline run `35381331328` installed dependencies and successfully typechecked main, applied `scripts/apply-balance-recovery-window.mjs` in its disposable checkout, then successfully typechecked again.

| Command / scenario | Historical patched baseline |
| --- | --- |
| `node scripts/probe-balance-fall-transition.mjs` | FAIL: slow pull enters recovery at tick 146; planted reversal at tick 117 |
| `node --test tests/balance-controller.test.mjs` | 4 pass, 2 fail |
| `npm run test:recovery` | 80 pass, 0 fail |

The isolated source workspace exported from CI matched its expected Git tree before local edits.

## Implemented source changes

The target contract now explicitly uses world-space hindfoot centres. Ankle-to-hindfoot conversion uses the actual two-joint anchors rather than the old whole-foot offset. A committed landing does not silently rotate or translate when the body heading changes. Flat landing height uses the shared geometry rather than a below-floor target.

Standing diagnostics expose copied individual contacts, force, measured points, load-bearing status, and persistence before recovery starts. They include pelvis/hip positions, stance ankle, measured support polygon, immutable and requested step targets, radial leg reach, desired/physical leg segments, and forward-kinematic reconstruction of the exact clamped motor commands. Motor-input and physical snapshots have separate sample times. Joint diagnostics include world torque, target coordinates, saturation, and limit error.

Radial reach is labelled `radiallyReachable`; it is not a claim that joint limits, ground contact, or the full dynamic chain can achieve the target. Diagnostic forward kinematics is never written into Rapier.

## Local committed-source candidate results

The following results were obtained from the source candidate; final remote run results are recorded in the PR conversation and Actions artifacts.

| Check | Result |
| --- | --- |
| Target/frame/contact diagnostic tests | 6 pass, 0 fail |
| Added strong-pull and torso-collision integrity tests | 2 pass, 0 fail |
| Existing coordinate, motor, and replay unit tests | 17 pass, 0 fail |
| Recovery regression tests | 80 pass, 0 fail |
| Focused balance tests | 4 pass, 2 fail |
| 510-tick transition probe | FAIL: slow pull first recovery tick 298; planted reversal tick 131 |

A hip-limit sign-only trial caused two recovery regressions and was removed. Trials that prevented a fall only by leaving the controller in an unfinished step were rejected. No fall-threshold relaxation was retained.

The first remote build found a missing diagnostic `SegmentPose` type import, fixed in the source correction above. This document does not claim a build or full-suite pass before those checks complete.

## Remaining physical blocker

The requested world-space landing and the chain reconstructed from local motor commands still diverge when the physical pelvis moves or rotates. The swing does not achieve measured target-bounded touchdown before the body loses support. Removing anchor ambiguity is necessary but insufficient; the floating-root/stance/swing control model still needs a verified coherent solution. Slow pull and reversal are not accepted merely because their first failure happens later.

The two new physical integrity tests cover a five-tick strong pull followed for 180 ticks, plus a 90-tick ordinary inward hand pull. They assert unchanged dynamic bodies, exact Rapier-owned rendered transforms, finite state, bounded joint/floor error, and nonexcluded torso collisions. **They do not certify every interaction or the failing full-length balance scenarios.** Full orientation, collision, loaded-stance, and successful touchdown acceptance remains open.

## Reproduction without modifying source

```sh
npm ci
npm run typecheck
BALANCE_TRACE_DIR=evidence/physical-chain/traces node scripts/probe-balance-fall-transition.mjs
node --test tests/balance-controller.test.mjs
node --test tests/leg-target-frame.test.mjs tests/physical-chain-integrity.test.mjs
npm run test:recovery
npm test
npm run test:physics
npm run test:pages
npm run lint
```

`focused-balance.yml` records each focused command separately and remains red on failure. `physical-chain.yml` runs the application/build, physics, static-pages, lint, and existing mouse/synthetic-touch replay checks against committed files, retaining logs and artifacts even on failures. Browser quick mode is an interaction smoke check, not full recovery acceptance.
