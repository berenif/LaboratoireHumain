# Concurrent body-physics checkpoint integration — WIP

## Scope and authority

The user requested `Merge main` after the prior result explicitly disclosed the
failed physics/recovery gates. This is a WIP merge, not a release acceptance.
No existing threshold, deployment gate, test case, or historical evidence is removed.

Original saved feature head: `58f4183ad19ea9d010af77cd4cdecde012d4ea73`.
Its earlier support-test commit: `b597df7d86d5f741fa9a75ff5b9c6d76eda2ac9a`.
Concurrent main/PR #5: `d670e755303fbaead6112c3e5bf6c93592951212`.
The integration merge has both the saved feature and this main as parents. It does
not replace main with an old source tree or rewrite either history.

## Resolution of the six conflicted files

- Anatomy/geometry: retain main's 0.25 m shoulder socket and 0.075 m shoulder-girdle
  half-length. Those dimensions already clear the ribcage; do not also clip the
  production arm. Retain the clipping helper as a general geometry utility. Use
  the saved proper half-turn frames for positive forward shoulder/elbow/hip
  flexion. Express main's physical shoulder limits in those frames exactly once:
  X [-35,120] and mirrored Z [-20,100]/[-100,20], not main's identity-frame scalar
  intervals combined with another axis reversal. Preserve main's mirrored girdles.
- Mass and applied loads: retain main's shared `measureMassState` helper, extending
  it to prefer actual Rapier `massKg` and `centerOfMass` from the saved snapshots.
  Only poses lacking measurements use main's integrated convex-volume centroid.
  Standing and recovery call the same helper. `appliedGrabForce` is the runtime
  input; `externalForce` remains a compatibility alias for saved planning fixtures.
  Explicit zero suppresses spring estimation, and a released grab cannot reuse a
  stale force. Native motor effort caps and actual-support transitions are retained.
- Limb targets/recovery: share main's exported anatomical `hingeParentRotation`,
  torso-relative arm bend, and recovery-joint conversion. Keep all main recovery
  eligibility corrections, native snapshot fixtures and their provenance tests.
  Keep the saved ankle conversion and physically measured support accounting.

The existing shoulder-abduction test still requests the same physical 60-degree
outward movement and the same >0.85 outward component. It now derives the scalar
coordinate through the authoritative joint frame rather than assuming identity
axes. The upper-arm manifold test only changes its title because clipping is no
longer the production geometry. All assertions and penetration limits remain.
One additional test checks measured-mass precedence and geometry-only fallback.

## Local integration verification

Dependencies were installed from the prior locked CI npm cache using `npm ci
--offline --ignore-scripts --no-audit --no-fund`. Package manifests/lock are unchanged.

- `npm run typecheck`: pass.
- `npm run lint`: pass; the two existing unused-import warnings remain.
- `npm test`: build passes; 156/168 tests pass, 12 fail.
- `npm run test:recovery`: 72/80 pass, 8 fail.
- `node --test tests/body-coherence.test.mjs tests/body-physics.test.mjs`: 24/24 pass.
- Full physics harness, Pages build, WebGL/browser motion and independent review
  have not been reverified on this integrated tree. Prior saved failures and
  timeouts must not be represented as passes for this integration.

Failing unit cases include slow pulls/reversal, planted reversal, fall/recovery
input lockout, historical initial snapshot parity, short strong-pull falling,
planted crouch stand entry, supported crouch/half-kneel fixture contacts, crouch
sole projection, foot-plan clearance, prone arm-brace placement, and rotated
anatomical arm-brace candidates. The two slow-pull/reversal categories were already
failing before both branches; other failures are not relabelled as inherited.

The historical native snapshots from main are intentionally unchanged. Their
initial-assembly mismatch is an explicit failing gate, not silently regenerated
evidence. This merge does not fix the remaining balance/recovery behavior and
must not be promoted to a passing deployment by disabling those checks.

Full test logs and integration evidence are retained in the conversation's merge
delivery. A merge/push alone does not establish any deployment.
