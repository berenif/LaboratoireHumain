# Cold-start worker variant audit — 2026-10-20

**The two-block attempt is closed and invalid for comparison with the intended
learned-policy baseline. No optimizer update, candidate evaluation, or promotion
occurred. Its raw records are diagnostic evidence of a different worker variant,
not proof that the previously qualified nearby BACK baseline regressed.**

## Exact durable identity chain

- The prior adaptation receipt, `docs/evidence/getup-wasm-adaptation-2026-10-20.json`,
  pins baseline WASM `53395c6d8b968774e07ee38eeb1931047acefb0df3cd090a59bc1943b9867e28`
  and adaptation WASM `921a373a0964db6ecaa86f3c1f10a91124f6f672b3dc123f0a0232721424cf56`.
- The identical baseline hash in
  `docs/evidence/browser-terminal-tail-ab-2026-10-19.json`, `arms.baseline`,
  binds its explicit build features. The build procedure is also recorded in
  `docs/evidence/probes/record-getup-terminal-tail-ab-2026-10-19.mjs`.
- `docs/experiments/getup-wasm-policy-adaptation.md` specifies adding only the
  local policy-adaptation head while preserving the existing controller. The
  intended adaptation feature closure below is inferred from that contract plus
  the hash-linked baseline build, not a claim that the lost candidate binary or
  exact original compiler invocation was recovered.

| Effective `lh-sim` feature | Intended baseline + adapter | Rebuilt cold worker |
|---|---:|---:|
| `root-translation-carry-experiment` | yes | yes |
| `unit-root-rotation-experiment` | yes | yes |
| `getup-terminal-balance-experiment` | yes | yes |
| `getup-repeat-cycle-experiment` | yes | yes |
| `standing-dynamics-experiment` | yes | **no** |
| `getup-terminal-tail-observer-experiment` | yes | yes |
| `getup-policy-adaptation-experiment` | yes | yes |
| `getup-physical-roll-experiment` | **no** | **yes** |

The rebuilt worker SHA256 is
`ff3da0d06c22d444972ecb7cde04b66304b6b65ae88f07ca73d876b1f3a1bdc1`.
Its retained source-identity manifest explicitly lists physical roll. The existing
Cargo `lh-sim` fingerprint confirms the effective feature set, including the
missing standing-dynamics feature. These are read-only build records; this audit
performed no rebuild or native simulation.

`getup-physical-roll-experiment` is default-off and transitively enables repeat
cycle and terminal balance. In `rust/crates/sim/src/getup/controller.rs`, the
feature-gated pre-reference branch automatically enters physical roll for settled
supine BACK. There is no profile switch that disables that compiled branch.
BELLY does not satisfy that supine condition. Separately, standing dynamics changes
supported-standing compensation in `getup/supported_standing.rs`; its omission is
also a semantic difference, not just a binary hash difference.

## What the closed attempt actually established

The attempt used source `7c567a03ef9f303b8eb956461d438806a1d4375d` and the rebuilt
worker above. Native off/zero and restored next-transition/reward/terminal/mapper
parity passed **within that worker variant**. They did not compare it to the lost
historical baseline. Input hashes and successful roundtrips proved identity and
recoverability, not intended semantic feature equivalence.

The attempt stopped after 7,700 charged integrations (7,160 real + 540 forecast),
71 source decisions and zero learning updates. BELLY supplied 70 mapped success
rows, but the required complete stratified replay was never constructed.

Nearby BACK reached `physical-roll-Rejected` at native 4,560 after its fixed
five-second roll cap. The next raw record (zero-based line 306) was an honest
zero-integration terminal `getup-policy` refusal: native 4,560 → 4,560, unchanged
impact epoch 560 and null source epoch, no training decision. The mapper expected
4,580 and reported a generic chronology error, masking the original refusal.
The diagnostic fix checks an existing non-deadline physical failure first,
retains the observed failed state and raw evidence, and does not insert the
refusal into replay, relax chronology, refund charges, or retry physics.

Terminal archive transfer and partial diagnostic restoration passed. Steady-state
current/previous cleanup did not: the available Drive connector exposes permanent
delete but no recoverable Trash operation. No deletion was attempted. Three
generations temporarily remain, all counted in 22,056,886 cumulative bytes. This
is an unresolved retention gate, not a successful two-generation cadence.

## Proposed minimal pre-clock semantic gate (not implemented authority)

Before another resource decision or runtime load, require the exact reviewed
**effective** feature closure, not merely a self-consistent archive hash:

1. Bind the intended baseline hash-linked build contract and adapter addition.
2. Require the effective feature set to equal the seven intended features above;
   explicitly reject physical-roll and any other unapproved controller variant.
3. Bind a build receipt to source, compiler/toolchain, Cargo feature closure,
   profile/model hashes and resulting WASM/glue hashes. Prefer a no-step compiled
   identity export for independent verification; a handwritten feature list alone
   cannot attest the binary.
4. Refuse old shared/seed bundles bound to the wrong variant. Any corrected build,
   immutable-input replacement, seed generation and real qualification require
   a separately reviewed plan and resource admission. Do not reinterpret existing
   raw records or silently reuse old parity as baseline equivalence.

No new trial, altered runtime, feature rebuild, physical acceptance relaxation,
or automatic restart is authorized by this audit or its diagnostic test.
