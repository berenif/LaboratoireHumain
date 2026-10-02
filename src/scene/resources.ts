import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";

/** Groups own their resources. Shared resources inside a group are released once. */
export function disposeSceneGroup(group: THREE.Object3D, ownedMaterials: Iterable<THREE.Material> = []): void {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>(ownedMaterials);
  const textures = new Set<THREE.Texture>();
  group.traverse(object => {
    if (object instanceof THREE.InstancedMesh) object.dispose();
    if (object instanceof THREE.Light && "shadow" in object) {
      (object as THREE.DirectionalLight).shadow?.dispose();
    }
    if (!(object instanceof THREE.Mesh || object instanceof THREE.Line || object instanceof THREE.Points)) return;
    geometries.add(object.geometry);
    for (const material of Array.isArray(object.material) ? object.material : [object.material]) materials.add(material);
  });
  for (const material of materials) {
    for (const value of Object.values(material)) if (value instanceof THREE.Texture) textures.add(value);
  }
  for (const texture of textures) texture.dispose();
  for (const geometry of geometries) geometry.dispose();
  for (const material of materials) material.dispose();
  group.clear();
}

/** Static opaque meshes only; one draw without material groups. */
export function batchOpaque(group: THREE.Group): void {
  const meshes = group.children.filter((child): child is THREE.Mesh<THREE.BufferGeometry, THREE.MeshLambertMaterial> =>
    child instanceof THREE.Mesh && child.material instanceof THREE.MeshLambertMaterial && !child.material.transparent);
  if (!meshes.length) return;
  const sources: THREE.BufferGeometry[] = [];
  const originalGeometries = new Set<THREE.BufferGeometry>();
  const originalMaterials = new Set<THREE.Material>();
  for (const mesh of meshes) {
    mesh.updateMatrix();
    const geometry = mesh.geometry.index ? mesh.geometry.toNonIndexed() : mesh.geometry.clone();
    geometry.applyMatrix4(mesh.matrix);
    // Scenery has no textures; normalize attributes for merging mixed primitives.
    geometry.deleteAttribute("uv");
    const colors = new Float32Array(geometry.getAttribute("position").count * 3);
    for (let i = 0; i < colors.length; i += 3) mesh.material.color.toArray(colors, i);
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    sources.push(geometry);
    originalGeometries.add(mesh.geometry);
    originalMaterials.add(mesh.material);
    group.remove(mesh);
  }
  const merged = mergeGeometries(sources, false);
  for (const geometry of sources) geometry.dispose();
  for (const geometry of originalGeometries) geometry.dispose();
  for (const material of originalMaterials) material.dispose();
  if (!merged) throw new Error("Scenery batching requires matching geometry attributes.");
  merged.computeBoundingSphere();
  const mesh = new THREE.Mesh(merged, new THREE.MeshLambertMaterial({ vertexColors: true, side: THREE.DoubleSide }));
  mesh.name = "static-scenery";
  mesh.receiveShadow = true;
  group.add(mesh);
}
