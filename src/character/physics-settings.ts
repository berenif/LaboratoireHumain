import type { World } from '@dimforge/rapier3d-compat';

/** Metre-scale human contacts. Pinned Rapier 0.20 defaults read back as 5 mm
 * allowed error and 20 mm prediction, not the 1/2 mm documented defaults.
 * A slow thigh contact retained stale witnesses and reached 12.36 mm overlap.
 * These explicit tolerances remove that excess; solver iterations are unchanged.
 */
export function configureHumanoidWorld(world: World): void {
  world.timestep = 1 / 60;
  world.numSolverIterations = 20;
  world.numInternalPgsIterations = 4;
  world.integrationParameters.maxCcdSubsteps = 4;
  world.integrationParameters.normalizedAllowedLinearError = 0.001;
  world.integrationParameters.normalizedPredictionDistance = 0.002;
}
