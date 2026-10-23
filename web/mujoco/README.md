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

The mannequin is passive (`ctrl=0`): no balance, walking or get-up controller
is represented as working. User drag applies a bounded spring/damper force
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
