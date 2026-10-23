# MuJoCo interactive MVP

`npm install` then `npm run build && npm start` opens a local preview at
http://127.0.0.1:5185. No SDK or Rust build is required for this path.
`npm test` runs the Chromium interaction checks (Playwright dev dependency;
set CHROMIUM_PATH if Chromium is not /usr/bin/chromium).

The pinned official `@mujoco/mujoco` 3.17.0 single-thread WASM and JavaScript
are served locally. There are no CDN calls, telemetry, recordings, remote APIs
or model downloads at runtime. The server binds only to loopback. Deployment
and external sharing are separate actions, not performed by these scripts.

The existing GetUp MJCF remains the source: 16 massive bodies, 21 hinge
actuators, 800 Hz timestep, unchanged geometry, collisions, limits, passive
parameters and actuator definitions. The build removes only visual includes
and named material attributes, because rendering reuses the existing compact
capsule renderer and app styles. MuJoCo's contact solver is not Rapier, and
old Rapier verdicts do not qualify this engine.

The default remains passive (`ctrl=0`). The optional Balance mode uses only
the 21 existing joint actuators: bounded posture PD, COM position/velocity
feedback on ankles and torso inclination feedback on hips. No root force,
gravity compensation, contact injection or state overwrite is used. Walking
and get-up are not implemented. A measured fall (torso below 0.65 m or upright
cosine below 0.4 after 0.5 s) latches passive commands until reset/mode change.
The original reset starts above the floor and includes a short landing; no
initial-state correction was made to hide this transient. User drag applies a bounded spring/damper force
at the selected body point, including its moment around body COM; release,
blur, pointer cancellation, pause and reset clear the grab. Push is a finite
60 ms user force on the torso. Only the explicit reset changes qpos directly.
The app starts paused. Drag empty space to orbit; use wheel/buttons to zoom.

The main view uses the same 3D capsule positions and depth-sorted projection
as the previous compact renderer, now reading MuJoCo geometry transforms.
This is not a prerecorded animation. Body contacts and joint dynamics run in
WASM. The active path is three small modules plus the original CSS and model.

Legacy Impact, Playground, Recovery, terrains and controllers are preserved
in `rust/` and Git, outside this MVP's active path. Use `build:legacy`,
`start:legacy`, `test:legacy` with their original toolchain. Return reference:
`local/rapier-before-mujoco-ea21723`; unlaunched029 work retained in d82bf4d.
No historical proof, video, checkpoint or local toolchain was deleted.

Browser acceptance covers real contacts, physical drag/release, push, pause,
camera, focus-loss release and 25 consecutive resets. Local screenshots and
machine-readable results are under web/proofs (gitignored). MuJoCo tick counts
are reported separately; Rapier history remains 45,735 ticks.

## Double-support validation

Frozen push: 15 N for 0.150 s at torso COM, world +X, −X and +Y.
Each case starts from the same reset, stands for 10 s and settles for 5 s
after the push. Criteria after the first second: torso tilt <20 degrees,
feet-only support, both foot loads >1 N, horizontal foot displacement <30 mm,
continuous saturation <0.25 s, final COM speed <0.03 m/s.

All three passed with the final controller. Maximum tilt 9.55 degrees;
maximum horizontal foot displacement 0.614 mm; minimum individual foot load
158.4 N. Final speed below 0.052 mm/s. No saturation after the first second;
initial landing can reach the unchanged torque limits. Quiet posture retains
about 7.95 degrees torso inclination: this is limited balance, not perfectly
vertical tracking or a general recovery controller.

`node scripts/verify-mujoco-balance.mjs` performs the fixed numeric protocol;
it refuses to overwrite its final proof. `node scripts/verify-mujoco-balance-browser.mjs`
checks the actual interface in Chromium. `npm test` preserves passive MVP tests.
The existing screenshot-based test needed scrolling back to the canvas before
its final focus-loss grab; controls now make the sidebar taller.

No physical phone was tested. No walking, recovery or robustness beyond the
three frozen pushes is claimed. The passive rollback is
`local/mujoco-passive-mvp-494674c`.

Accounting: web/proofs/balance-ledger.json retains all measured runs. One
legacy passive-browser test failed before its final counter was persisted;
its integration count is unknown and explicitly recorded as such. The
confirmed cumulative MuJoCo count is therefore a lower bound, not an exact
total. Rapier remains frozen at 45,735 ticks.
