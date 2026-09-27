import type { CharacterController, GrabCommand, RendererMode } from "../core/types";

/** The application and standing runner share the complete update/snapshot order. */
export function characterFrame(character: CharacterController, dt: number, command: GrabCommand | null, renderer: RendererMode) {
  character.fixedUpdate(dt, command);
  return character.getSnapshot(renderer);
}
