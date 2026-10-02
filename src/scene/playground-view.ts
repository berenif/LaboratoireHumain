import * as THREE from "three";
import { batchOpaque, disposeSceneGroup } from "./resources";
import { sceneryColor } from "./palette";
export { disposeSceneGroup } from "./resources";
import { courseTransform, getPlaygroundCourse, PLAYGROUND_STATIONS, type CoursePiece, type PlaygroundConfig } from "../core/playground";

export class PlaygroundView {
  readonly group = new THREE.Group();
  readonly movingGroup = new THREE.Group();
  private difficulty = "";
  private readonly moving: { piece: CoursePiece; mesh: THREE.Mesh }[] = [];
  private readonly markers = new Map<string, THREE.Mesh<THREE.RingGeometry, THREE.MeshBasicMaterial>>();

  sync(config: PlaygroundConfig, time: number): void {
    if (this.difficulty !== config.difficulty) this.build(config);
    for (const { piece, mesh } of this.moving) {
      const { rotation } = courseTransform(piece, time);
      mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    }
    for (const [id, marker] of this.markers) marker.material.opacity = id === config.station ? 0.85 : 0.18;
  }

  dispose(): void {
    disposeSceneGroup(this.group);
    this.movingGroup.clear();
    this.moving.length = 0;
    this.markers.clear();
  }

  private build(config: PlaygroundConfig): void {
    this.dispose();
    this.difficulty = config.difficulty;
    this.group.add(this.movingGroup);
    for (const piece of getPlaygroundCourse(config.difficulty)) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.Float32BufferAttribute(piece.geometry.vertices.flatMap(v => [v.x, v.y, v.z]), 3));
      geometry.setIndex(piece.geometry.triangles.flatMap(t => [...t]));
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color: sceneryColor(piece.color) }));
      mesh.position.set(piece.position.x, piece.position.y, piece.position.z);
      mesh.name = `course:${piece.id}`;
      mesh.castShadow = Boolean(piece.motion);
      mesh.receiveShadow = true;
      (piece.motion ? this.movingGroup : this.group).add(mesh);
      if (piece.motion) this.moving.push({ piece, mesh });
    }
    batchOpaque(this.group);
    for (const [index, station] of PLAYGROUND_STATIONS.entries()) {
      const marker = new THREE.Mesh(new THREE.RingGeometry(0.66, 0.69, 48),
        new THREE.MeshBasicMaterial({ color: station.color, transparent: true, opacity: 0.2, depthWrite: false }));
      marker.rotation.x = -Math.PI / 2;
      marker.position.set(station.position.x, 0.009, station.position.z);
      this.markers.set(station.id, marker);
      this.group.add(marker);
      const canvas = document.createElement("canvas");
      canvas.width = 768; canvas.height = 112;
      const context = canvas.getContext("2d");
      if (!context) continue;
      context.fillStyle = "#596965";
      context.font = "600 40px system-ui, sans-serif";
      context.textAlign = "center";
      context.fillText(`${String(index + 1).padStart(2, "0")}  /  ${station.name.toUpperCase()}`, 384, 52);
      context.font = "24px system-ui, sans-serif";
      context.globalAlpha = 0.65;
      context.fillText(station.subtitle.toUpperCase(), 384, 92);
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      const label = new THREE.Mesh(new THREE.PlaneGeometry(2.8, 0.41),
        new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false }));
      label.rotation.x = -Math.PI / 2;
      label.position.set(station.position.x, 0.012, station.id === "flat" ? 4.55
        : station.id === "wobble" ? -7.3 : station.id === "stones" || station.id === "hurdles" ? 6.7 : 0.28);
      this.group.add(label);
    }
  }
}
