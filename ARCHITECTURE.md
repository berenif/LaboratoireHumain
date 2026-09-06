# Embodied Motion Physics Demo

## Delivery boundary

- Fresh project and private Site. No prior body-lab source, assets, evidence, or deployment were inspected or reused.
- Movement and physics are the release surface. The figure is original procedural geometry.
- World units are metres, +Y is up, and +Z is forward. One fixed-step integrator runs at 60 Hz; rendering interpolates snapshots and never advances simulation.

## Stack decision

Three.js 0.185.1 + `@dimforge/rapier3d-compat` 0.20.0 was selected over Babylon.js + Havok. Rapier exposes a renderer-independent physics `World` and a collider-based kinematic character controller, so the identical real simulation can feed WebGL2, Canvas2D, and non-rendering verification. Three.js is used only by the WebGL2 presentation adapter. Canvas2D is a second view, not a second solver.

Official references checked on 2026-09-06:

- https://rapier.rs/docs/user_guides/javascript/character_controller/
- https://rapier.rs/docs/user_guides/javascript/rigid_body_type/
- https://doc.babylonjs.com/features/featuresDeepDive/physics/characterController
- https://threejs.org/docs/pages/WebGLRenderer.html

## Ownership and data flow

`PointerInteraction` emits grab intent with a stable region, segment, body-local anchor, and world target. `EmbodiedCharacter` consumes that intent only on fixed updates and publishes complete immutable pose snapshots. Exactly one selected view reads the same previous/current snapshots. `EmbodiedDemo` owns one shared camera projection used by both views, picking, and drag-plane ray construction. Camera controls mutate only this projection, never simulation state or poses.

Upright, reacting, and stepping use a collision-aware kinematic root plus one pose compositor. Falling and fallen states disable root/pose writes before Rapier dynamic bodies and joints take ownership. Reset clears grabs, queued input, physics bodies, pose history, clock accumulation, and restores the exact default camera while retaining the viewport.

## Procedural body

Sixteen rigid/procedural segments total 72.2 kg: pelvis, torso, neck, head, paired upper arms, forearms, hands, thighs, shins, and feet. Seven stable selectable regions are head, torso, pelvis, both hands, and both feet. Shared definitions in `src/core/humanoid.ts` record masses, shapes, rest offsets, joint anchors and limits, region mappings, and collision group intent. Body colliders interact with the floor/boundaries but not one another, preventing adjacent-body activation explosions.

## Drag rule

Picking uses a camera ray against shared body proxies. The initial hit creates a camera-facing plane through the world hit point. Pointer motion intersects that fixed plane, preserving depth and the anatomical local anchor. A second pointer is ignored while the primary pointer owns capture.


## S1 functional successor (2026-09-06)

The old dynamic grab used `addForceAtPoint` every fixed update. Rapier stores user forces, so those calls accumulated and continued to accelerate the body after input release. S1 replaces that complete path with one `applyImpulseAtPoint` per fixed update. The point controller solves the full effective inverse-mass matrix using each selected body's world inverse inertia and actual center of mass. An implicit damped velocity response is bounded by 180 N × dt linear impulse, 12 Nm × dt angular impulse, and 36 W × dt positive kinetic work. The control target is speed/acceleration limited; raw target and body-local anchor remain separate and unchanged at handoff.

Handoff reads newly constructed Rapier bodies before any physics step, measures all segment and selected-anchor continuity errors, and clears target derivative history. Falling/fallen motion remains exclusively Rapier owned. Release removes the one-step contribution without modifying velocity. Reset clears input, bodies, controller, timing, interpolation, and camera. Rendering never writes simulation poses or advances physics.

The S1 launch passed with the original camera. The one separately recorded S1-F1 follow-up removes control-target arrival snapping, which bypassed the already frozen acceleration limit. No controller constants or acceptance limits changed.

Picking now intersects each oriented visible primitive and sorts by nearest surface distance, with stable region priority only for coincident hits. The original oblique camera physically occluded the left-hand center with the pelvis. The accepted more frontal view at (1.6, 2.25, 5.1), looking at (0, 1.02, 0), remains the exact default/reset view and removes that overlap across the full idle sway. It changes only through explicit user input and never follows the body or alters the physics trajectory.

Camera input is attached in the bubble phase behind the capture-phase body interaction. A selectable-body hit stops propagation and exclusively owns the pointer. Blank-space primary or right drag orbits; Shift-primary, Shift-right, or middle drag pans; wheel zooms. On a supported touch surface, one blank-space touch orbits and two blank-space touches pan/pinch. Orbit, pan, and zoom are finite and bounded. Renderer changes preserve the shared view, and Reset restores it exactly.

Browser QA is available only when the URL includes `?qa=1`. Its replay dispatches synthetic DOM PointerEvents through installed listeners and uses the real fixed clock and UI controls. It is explicitly labeled synthetic, and native mouse CUA evidence is recorded separately. Native touch and GPU performance are not inferred from synthetic events or Canvas2D results.

The visible interface is a full-viewport body view with one always-compact dock for Canvas 2D/WebGL2 selection, Pause, and Reset. Motion state and a one-line gesture hint remain visible; detailed diagnostic values stay screen-reader/test accessible without becoming a rapidly updating live region. Visibility replay retains every missed sample and keeps the original 100% visibility threshold.
