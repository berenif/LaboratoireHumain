# H28 — upstream runtime comparison

Evidence: [verified H27–H29 archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json),
including the official package tarball, integrity receipt and failed test output.

Rapier JS 0.21.0 is published from commit
`b716d375efc0201003f0cd9ef7168eee0b62c177`, whose binding template selects
Rapier 0.36.0 and Parry 0.31.x. The earlier upstream source receipts document
contact-impulse and generic multibody solver fixes after the currently pinned
Rapier 0.35.0. This experiment tests the published runtime rather than assuming
those fixes repair standing.

The package is downloaded to an ignored diagnostic directory and verified
against the official npm SHA-512 integrity. The application's package files
and installed dependency remain unchanged. A process-local module loader
redirects the Rapier import and replaces only the joint adapter's expected
version literal in memory. All raw-method checks, angular-limit validation,
readback checks, motor ceilings and existing test assertions remain active.
Reports must record hashes of the runtime, original adapter and overlay.

First run the existing adapter/coordinate/motor selection on 0.21.0. Then
compare the unchanged controller's standing behavior at all three headings,
starting with the unchanged 2+10 s screen. Verify the loader itself by replaying
the pinned 0.20.0 control. Preserve failures and distinguish API compatibility
from physical acceptance. Any production upgrade would require the full
unchanged standing, structural, collision, actuation and ownership gates,
followed by the remaining TODO milestones. No tolerance or fixture refresh is
permitted.

The original 0.20.0 runtime passes all 21 selected tests through the loader.
On 0.21.0, the five raw-adapter tests and seven coordinate tests pass, but two
of nine motor/inertia tests fail: the free post-constraint ankle response and
the supported swing hip/ankle response. Their original assertions remain
unchanged. The three plain standing screens also fail their speed bounds;
the runtime update alone is rejected, and the application remains on 0.20.0.
The diagnostic loader emits Node's experimental `stripTypeScriptTypes`
warning; it is not part of the application runtime.
