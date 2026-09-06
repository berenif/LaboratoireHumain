# Embodied Motion Physics Demo

A browser demo for directly manipulating a procedural humanoid. The upright character uses a collision-aware root motor and procedural pose compositor; overload transfers ownership to a connected Rapier ragdoll. The same snapshots drive a Three.js WebGL2 view or a Canvas2D projection.

## Run locally

```bash
npm ci
npm run dev
```

Use the **View** control to select Canvas2D explicitly. Drag the colored head, torso, pelvis, hands, or feet. **Reset body** is the only get-up action.

## Verify

```bash
npm run build
npm run lint
npm run test:physics
```

`test:physics` intentionally exits nonzero while the frozen fast-overload readability scenario remains outside its acceptance bounds. See `STATUS.md` and `evidence/physics-results.json` for the exact release assessment.
