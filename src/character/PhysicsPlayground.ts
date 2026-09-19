import RAPIER, { type Collider, type RigidBody, type World } from "@dimforge/rapier3d-compat";
import { courseTransform, getPlaygroundCourse, type CoursePiece, type PlaygroundDifficulty } from "../core/playground";
import type { Vec3 } from "../core/types";

/** Environment ownership stays in physics; renderers share only its geometry and clock. */
export class PhysicsPlayground {
  readonly colliders: Collider[] = [];
  private readonly pieces: { definition: CoursePiece; body: RigidBody }[] = [];
  private readonly queries: { collider: Collider; minX: number; maxX: number; minZ: number; maxZ: number }[] = [];

  constructor(private readonly world: World, difficulty: PlaygroundDifficulty) {
    for (const definition of getPlaygroundCourse(difficulty)) {
      const { position } = definition;
      const body = world.createRigidBody((definition.motion
        ? RAPIER.RigidBodyDesc.kinematicPositionBased() : RAPIER.RigidBodyDesc.fixed())
        .setTranslation(position.x, position.y, position.z));
      // Rapier 0.20's mesh manifolds omit solver contacts/loads. Convex solids
      // preserve the exact visible top triangles and expose measured impulses.
      const hulls = definition.tiled
        ? definition.geometry.triangles.slice(0, definition.geometry.surfaceTriangles).map(triangle => {
          const top = triangle.map(index => definition.geometry.vertices[index]);
          return [...top, ...top.map(vertex => ({ ...vertex, y: 0 }))];
        }) : [definition.geometry.vertices];
      for (const hull of hulls) {
        const descriptor = RAPIER.ColliderDesc.convexHull(new Float32Array(hull.flatMap(v => [v.x, v.y, v.z])));
        if (!descriptor) throw new Error(`Invalid course solid: ${definition.id}`);
        const collider = world.createCollider(descriptor
          .setFriction(definition.friction).setRestitution(0.01)
          .setCollisionGroups((0x0002 << 16) | 0x0001), body);
        this.colliders.push(collider);
        const margin = definition.motion ? 0.5 : 0.002;
        this.queries.push({ collider,
          minX: position.x + Math.min(...hull.map(v => v.x)) - margin,
          maxX: position.x + Math.max(...hull.map(v => v.x)) + margin,
          minZ: position.z + Math.min(...hull.map(v => v.z)) - margin,
          maxZ: position.z + Math.max(...hull.map(v => v.z)) + margin });
      }
      this.pieces.push({ definition, body });
    }
  }

  update(time: number): void {
    for (const { definition, body } of this.pieces) {
      if (!definition.motion) continue;
      const transform = courseTransform(definition, time);
      body.setNextKinematicRotation(transform.rotation);
    }
  }

  reset(): void {
    for (const { definition, body } of this.pieces) {
      if (!definition.motion) continue;
      const transform = courseTransform(definition, 0);
      body.setRotation(transform.rotation, true);
      body.setNextKinematicRotation(transform.rotation);
      body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
    this.world.propagateModifiedBodyPositionsToColliders();
  }

  heightAt(x: number, z: number, ceiling = 5): number {
    return this.surfaceAt(x, z, ceiling).height;
  }

  surfaceAt(x: number, z: number, ceiling = 5): { height: number; normal: Vec3 } {
    const ray = new RAPIER.Ray({ x, y: ceiling, z }, { x: 0, y: -1, z: 0 });
    let height = 0;
    let normal: Vec3 = { x: 0, y: 1, z: 0 };
    for (const { collider, minX, maxX, minZ, maxZ } of this.queries) {
      if (x < minX || x > maxX || z < minZ || z > maxZ) continue;
      if (!collider.isEnabled()) continue;
      const hit = collider.castRayAndGetNormal(ray, Math.max(0, ceiling), false);
      if (hit && hit.normal.y > 0.35 && ceiling - hit.timeOfImpact > height) {
        height = ceiling - hit.timeOfImpact;
        normal = { ...hit.normal };
      }
    }
    return { height, normal };
  }

  dispose(): void {
    for (const { body } of this.pieces) this.world.removeRigidBody(body);
    this.pieces.length = 0;
    this.colliders.length = 0;
    this.queries.length = 0;
  }
}
