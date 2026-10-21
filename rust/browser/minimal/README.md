# Minimal physical demo (experimental)

The body starts visibly prone, at construction only. It settles for 400 native
samples, autonomously gets up, then accepts a real Strike (`P` or the button).
The same worker/world continues; there is no mid-run pose reset or teleport.
The existing joint controller, model, solver and numerical integration are unchanged.

Serve with `node scripts/serve-minimal-demo.mjs <verified-worker-assets> 5186`.
Assets must match the hashes in the server; compiled binaries are not committed here.
This small diagnostic view does not replace the richer application's source.

Behavior check: `node scripts/check-minimal-physics.mjs <assets> <result.json> 0 belly`.
It permits two real strikes, at most 120 physical seconds / five wall minutes,
with the existing 25-second recovery deadline. The optional heading argument is
in degrees; `entry-only` instead of `belly` records a short neutral-start diagnostic.

The current worker reproduced initial get-up and **one actual strike/fall/get-up**
without reset. A second strike fell but did not recover within 25 seconds.
Starting upright and striking immediately also failed recovery. These failures
are not hidden or automatically restarted. Reliable repeated recovery remains
unfinished. The private browser preview is not yet browser-validated because
its owner-authentication gate prevented automated access; the actual WASM
behavior above was checked in Node with the same runtime and profile.
