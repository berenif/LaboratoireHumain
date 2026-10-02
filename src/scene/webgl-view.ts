import * as THREE from "three";
import { V3 } from "../core/math";
import { ARENA } from "../core/playground";
import { PlaygroundView } from "./playground-view";
import { PROTOCOL_STRIKER_PIECES, protocolRoomPieces, type ProtocolVisualPiece } from "./protocol-visuals";
import { flattenGeometryIndices, flattenGeometryVertices } from "../core/geometry";
import {
  PRIMARY_SEGMENT_BY_REGION,
  SEGMENTS,
  SEGMENT_BY_ID,
} from "../core/humanoid";
import type { PoseSnapshot, PoseView, RegionId, SegmentDefinition, SegmentId, SegmentPose, Vec3, RenderQuality, RenderMetrics } from "../core/types";
import { SharedCameraProjection } from "./camera";
import type { SceneViewOptions } from "./options";
import { PresentationBuffer, transformLocalPoint } from "./pose";

import { qualitySettings } from "./quality";
import { BACKGROUND, BODY_COLORS, JOINT_COLOR, SELECTION } from "./palette";
import { batchOpaque, disposeSceneGroup } from "./resources";

export interface WebGLRenderMetrics extends RenderMetrics {
  readonly drawCalls: number;
  readonly triangles: number;
  readonly points: number;
  readonly lines: number;
  readonly width: number;
  readonly height: number;
  readonly devicePixelRatio: number;
}

function average(a: Vec3, b: Vec3): Vec3 {
  return V3.scale(V3.add(a, b), 0.5);
}

function shapeGeometry(definition: SegmentDefinition): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(flattenGeometryVertices(definition.geometry), 3));
  geometry.setIndex(new THREE.BufferAttribute(flattenGeometryIndices(definition.geometry), 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

export class WebGLView implements PoseView {
  readonly mode = "webgl" as const;
  readonly canvas: HTMLCanvasElement;

  private readonly ownsCanvas: boolean;
  private readonly projection: SharedCameraProjection;
  private readonly scene = new THREE.Scene();
  private readonly playgroundView = new PlaygroundView();
  private readonly playgroundDecor = new THREE.Group();
  private readonly protocolRoom = new THREE.Group();
  private readonly protocolStriker = new THREE.Group();
  private roomKey = "";
  private readonly camera = new THREE.PerspectiveCamera();
  private readonly segmentMeshes = new Map<SegmentId, THREE.Mesh<THREE.BufferGeometry, THREE.MeshLambertMaterial>>();
  private readonly jointMarkers = new THREE.InstancedMesh(new THREE.SphereGeometry(0.037, 8, 6),
    new THREE.MeshLambertMaterial({ color: JOINT_COLOR }), SEGMENTS.length - 1);
  private readonly jointTransform = new THREE.Object3D();
  private readonly presentation = new PresentationBuffer();
  private presentationStart = 0;
  private presentationCpuMs = 0;
  private quality: RenderQuality = "auto";
  private autoRatio = 1.25;
  private deviceRatio = 1;
  private readonly keyLight = new THREE.DirectionalLight(0xffffff, 2);
  private readonly casterBounds = new THREE.Box3();
  private readonly lightBounds = new THREE.Box3();
  private readonly corner = new THREE.Vector3();
  private readonly center = new THREE.Vector3();
  private readonly supportMarkers = new Map<"leftFoot" | "rightFoot", THREE.Mesh<THREE.RingGeometry, THREE.MeshBasicMaterial>>();
  private readonly bodyMaterials = new Map<string, THREE.MeshLambertMaterial>();
  private readonly selectedMaterial = new THREE.MeshLambertMaterial({ color: SELECTION });
  private readonly selectionMarker: THREE.Mesh<THREE.SphereGeometry, THREE.MeshBasicMaterial>;
  private renderer: THREE.WebGLRenderer | null = null;
  private context: WebGL2RenderingContext | null = null;
  private container: HTMLElement | null = null;
  private snapshot: PoseSnapshot | null = null;
  private cssWidth = 1;
  private cssHeight = 1;
  private sized = false;
  private pixelRatio = 1;
  private initialized = false;
  private disposed = false;

  constructor(options: SceneViewOptions = {}) {
    if (typeof document === "undefined" && !options.canvas) {
      throw new Error("WebGLView requires a browser canvas.");
    }
    this.ownsCanvas = !options.canvas;
    this.canvas = options.canvas ?? document.createElement("canvas");
    this.projection = options.camera ?? new SharedCameraProjection();
    this.canvas.className = options.className ?? "embodied-scene embodied-scene--webgl";
    this.canvas.dataset.renderer = "webgl";
    this.canvas.style.display = "block";
    this.canvas.style.width = "100%";
    this.canvas.style.height = "100%";
    this.canvas.style.touchAction = "none";

    this.selectionMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.19, 16, 10),
      new THREE.MeshBasicMaterial({ color: SELECTION, transparent: true, opacity: 0.46, wireframe: true }),
    );
    this.selectionMarker.visible = false;
    this.selectionMarker.renderOrder = 10;
  }

  mount(container: HTMLElement): void {
    this.assertUsable();
    this.container = container;
    if (this.canvas.parentElement !== container) container.append(this.canvas);
    if (!this.initialized) this.initializeRenderer();
    const rect = container.getBoundingClientRect();
    this.resize(rect.width || container.clientWidth || 1, rect.height || container.clientHeight || 1,
      typeof window === "undefined" ? 1 : window.devicePixelRatio || 1);
  }

  setSnapshot(previous: PoseSnapshot, current: PoseSnapshot, alpha: number): void {
    const start = performance.now();
    this.snapshot = this.presentation.update(previous, current, alpha);
    this.presentationStart = performance.now() - start;
  }

  render(): void {
    if (!this.renderer || this.disposed) return;
    const start = performance.now();
    this.syncCamera();
    if (this.snapshot) this.applyPose(this.snapshot);
    this.fitShadowCamera();
    this.renderer.info.reset();
    this.renderer.render(this.scene, this.camera);
    this.presentationCpuMs = performance.now() - start + this.presentationStart;
    this.presentationStart = 0;
  }

  resize(width: number, height: number, devicePixelRatio: number): void {
    this.assertUsable();
    this.sized = true;
    this.cssWidth = Math.max(1, Math.round(width));
    this.cssHeight = Math.max(1, Math.round(height));
    this.deviceRatio = devicePixelRatio;
    this.pixelRatio = qualitySettings(this.quality, devicePixelRatio, this.autoRatio).pixelRatio;
    this.projection.setViewport(this.cssWidth, this.cssHeight);
    if (this.renderer) {
      this.renderer.setPixelRatio(this.pixelRatio);
      this.renderer.setSize(this.cssWidth, this.cssHeight, false);
    } else {
      this.canvas.width = Math.max(1, Math.round(this.cssWidth * this.pixelRatio));
      this.canvas.height = Math.max(1, Math.round(this.cssHeight * this.pixelRatio));
    }
    this.canvas.style.width = `${this.cssWidth}px`;
    this.canvas.style.height = `${this.cssHeight}px`;
    this.syncCamera();
  }

  setQuality(quality: RenderQuality, autoPixelRatio = 1.25): void {
    this.quality = quality;
    this.autoRatio = autoPixelRatio;
    const settings = qualitySettings(quality, this.deviceRatio, autoPixelRatio);
    const shadow = this.keyLight.shadow;
    if (shadow.mapSize.x !== settings.shadowResolution) {
      shadow.dispose();
      shadow.map = null;
      shadow.mapPass = null;
      shadow.mapSize.setScalar(settings.shadowResolution);
      shadow.needsUpdate = true;
    }
    if (this.sized) this.resize(this.cssWidth, this.cssHeight, this.deviceRatio);
  }

  getProjection(): SharedCameraProjection {
    return this.projection;
  }

  getElement(): HTMLCanvasElement {
    return this.canvas;
  }

  getMetrics(): WebGLRenderMetrics {
    const info = this.renderer?.info.render;
    return {
      drawCalls: info?.calls ?? 0,
      presentationCpuMs: this.presentationCpuMs,
      effectivePixelRatio: this.pixelRatio,
      shadowResolution: this.keyLight.shadow.mapSize.x,
      geometries: this.renderer?.info.memory.geometries ?? 0,
      textures: this.renderer?.info.memory.textures ?? 0,
      triangles: info?.triangles ?? 0,
      points: info?.points ?? 0,
      lines: info?.lines ?? 0,
      width: this.canvas.width,
      height: this.canvas.height,
      devicePixelRatio: this.pixelRatio,
    };
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.playgroundView.dispose();
    // Playground owns and clears its subtree first; the scene owns the rest.
    if (!this.initialized) {
      this.scene.add(this.selectionMarker, this.jointMarkers, this.keyLight);
    }
    disposeSceneGroup(this.scene, [...this.bodyMaterials.values(), this.selectedMaterial]);
    this.renderer?.dispose();
    this.renderer = null;
    this.context = null;
    this.segmentMeshes.clear();
    this.supportMarkers.clear();
    this.bodyMaterials.clear();
    this.snapshot = null;
    if (this.ownsCanvas) this.canvas.remove();
    this.container = null;
  }

  private initializeRenderer(): void {
    const context = this.canvas.getContext("webgl2", {
      alpha: false,
      antialias: true,
      depth: true,
      powerPreference: "high-performance",
      preserveDrawingBuffer: false,
      stencil: false,
    });
    if (!context) throw new Error("WebGL2 context creation failed; select the Canvas2D view.");
    this.context = context;
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      context,
      alpha: false,
      antialias: true,
      powerPreference: "high-performance",
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setClearColor(BACKGROUND, 1);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.info.autoReset = false;

    this.scene.background = new THREE.Color(BACKGROUND);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xb8bbb0, 2.1));
    const key = this.keyLight;
    key.position.set(-6, 10, 6);
    key.castShadow = true;
    const resolution = qualitySettings(this.quality, this.deviceRatio, this.autoRatio).shadowResolution;
    key.shadow.mapSize.set(resolution, resolution);
    key.shadow.normalBias = 0.012;
    key.shadow.bias = -0.0001;
    this.scene.add(key, key.target);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(ARENA.width, ARENA.depth),
      new THREE.MeshLambertMaterial({ color: 0xe9e9e2 }),
    );
    floor.rotation.x = -Math.PI * 0.5;
    floor.position.set(0, -0.012, 0);
    floor.receiveShadow = true;
    this.playgroundDecor.add(floor);
    const grid = new THREE.GridHelper(20, 40, 0xb9c3bf, 0xd5dad5);
    grid.position.set(0, 0.002, 0);
    grid.scale.z = ARENA.depth / ARENA.width;
    this.playgroundDecor.add(grid);

    const boundaryMaterial = new THREE.MeshLambertMaterial({
      color: 0x8ba39b,
    });
    const boundaries: ReadonlyArray<readonly [number, number, number, number, number]> = [
      [0, -0.025, -8.5, 20, 0.05],
      [0, -0.025, 8.5, 20, 0.05],
      [-10, -0.025, 0, 0.05, 17],
      [10, -0.025, 0, 0.05, 17],
    ];
    for (const [x, y, z, width, depth] of boundaries) {
      const boundary = new THREE.Mesh(new THREE.BoxGeometry(width, 0.05, depth), boundaryMaterial);
      boundary.position.set(x, y, z);
      this.playgroundDecor.add(boundary);
    }

    batchOpaque(this.playgroundDecor);
    for (const definition of SEGMENTS) {
      const color = definition.region ? BODY_COLORS[definition.region] : "#a5b1ad";
      let material = this.bodyMaterials.get(color);
      if (!material) {
        material = new THREE.MeshLambertMaterial({ color });
        this.bodyMaterials.set(color, material);
      }
      const mesh = new THREE.Mesh(shapeGeometry(definition), material);
      mesh.name = `segment:${definition.id}`;
      mesh.castShadow = mesh.receiveShadow = true;
      mesh.userData.baseMaterial = material;
      this.segmentMeshes.set(definition.id, mesh);
      this.scene.add(mesh);
    }
    this.jointMarkers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.jointMarkers.frustumCulled = false;
    this.scene.add(this.jointMarkers);

    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const marker = new THREE.Mesh(
        new THREE.RingGeometry(0.12, 0.165, 24),
        new THREE.MeshBasicMaterial({ color: 0x80e7a8, transparent: true, opacity: 0.72, side: THREE.DoubleSide }),
      );
      marker.rotation.x = -Math.PI * 0.5;
      marker.position.y = 0.008;
      marker.name = `support:${foot}`;
      marker.visible = false;
      this.supportMarkers.set(foot, marker);
      this.scene.add(marker);
    }
    this.scene.add(this.selectionMarker);
    this.scene.add(this.playgroundDecor);
    this.scene.add(this.playgroundView.group);
    this.scene.add(this.protocolRoom);
    this.scene.add(this.protocolStriker);
    for (const piece of PROTOCOL_STRIKER_PIECES) this.protocolStriker.add(this.protocolMesh(piece));
    batchOpaque(this.protocolStriker);
    for (const child of this.protocolStriker.children) child.castShadow = true;
    this.initialized = true;
    this.syncCamera();
  }

  private syncCamera(): void {
    const state = this.projection.getState();
    this.camera.position.set(state.position.x, state.position.y, state.position.z);
    this.camera.up.set(state.up.x, state.up.y, state.up.z);
    this.camera.fov = THREE.MathUtils.radToDeg(state.fovYRadians);
    this.camera.aspect = state.viewportWidth / state.viewportHeight;
    this.camera.near = state.near;
    this.camera.far = state.far;
    this.camera.lookAt(state.target.x, state.target.y, state.target.z);
    this.camera.updateProjectionMatrix();
    this.camera.updateMatrixWorld(true);
  }

  private applyPose(snapshot: PoseSnapshot): void {
    const protocol = Boolean(snapshot.room);
    this.playgroundDecor.visible = !protocol;
    this.playgroundView.group.visible = !protocol;
    this.protocolRoom.visible = protocol;
    this.protocolStriker.visible = protocol && Boolean(snapshot.striker);
    if (snapshot.playground) this.playgroundView.sync(snapshot.playground, snapshot.simulationTime);
    if (snapshot.room) {
      this.syncProtocolRoom(snapshot.room);
      if (snapshot.striker) {
        this.protocolStriker.position.set(snapshot.striker.position.x, snapshot.striker.position.y, snapshot.striker.position.z);
        this.protocolStriker.quaternion.set(snapshot.striker.rotation.x, snapshot.striker.rotation.y, snapshot.striker.rotation.z, snapshot.striker.rotation.w);
      }
    }
    const poses = this.presentation.byId;
    for (const mesh of this.segmentMeshes.values()) mesh.visible = false;
    for (const pose of snapshot.segments) {
      const mesh = this.segmentMeshes.get(pose.id);
      if (!mesh) continue;
      mesh.visible = true;
      mesh.position.set(pose.position.x, pose.position.y, pose.position.z);
      mesh.quaternion.set(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w).normalize();
      const exactSelection = snapshot.diagnostics.selectedSegment;
      const selected = exactSelection
        ? pose.id === exactSelection
        : Boolean(snapshot.diagnostics.selectedRegion) && SEGMENT_BY_ID.get(pose.id)?.region === snapshot.diagnostics.selectedRegion;
      mesh.material = selected ? this.selectedMaterial : mesh.userData.baseMaterial;
    }

    let markerCount = 0;
    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const marker = this.jointTransform;
      const child = poses.get(definition.id);
      const parent = poses.get(definition.parent);
      if (!child || !parent) continue;
      const parentPoint = definition.jointAnchorParent
        ? transformLocalPoint(parent, definition.jointAnchorParent)
        : parent.position;
      const childPoint = definition.jointAnchorChild
        ? transformLocalPoint(child, definition.jointAnchorChild)
        : child.position;
      const point = average(parentPoint, childPoint);
      marker.position.set(point.x, point.y, point.z);
      marker.updateMatrix();
      this.jointMarkers.setMatrixAt(markerCount++, marker.matrix);
    }

    this.jointMarkers.count = markerCount;
    this.jointMarkers.instanceMatrix.needsUpdate = true;

    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const marker = this.supportMarkers.get(foot)!;
      const pose = poses.get(foot);
      marker.visible = !protocol && Boolean(pose);
      if (!pose) continue;
      marker.position.x = pose.position.x;
      marker.position.y = pose.position.y - 0.025;
      marker.position.z = pose.position.z;
      marker.quaternion.set(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
      marker.rotateX(-Math.PI / 2);
      const planted = snapshot.support.planted.includes(foot);
      const swinging = snapshot.support.swingFoot === foot;
      marker.material.color.set(swinging ? 0xffd166 : planted ? 0x80e7a8 : 0x7c8ba1);
      marker.material.opacity = swinging ? 0.9 : planted ? 0.72 : 0.28;
    }

    const selectedPose = this.findSelectedPose(
      snapshot.diagnostics.selectedSegment ?? null,
      snapshot.diagnostics.selectedRegion,
      poses,
    );
    this.selectionMarker.visible = Boolean(selectedPose);
    if (selectedPose) {
      this.selectionMarker.position.set(selectedPose.position.x, selectedPose.position.y, selectedPose.position.z);
      const scale = snapshot.diagnostics.activeGrab
        ? 1 + Math.sin(snapshot.simulationTime * 12) * 0.08
        : 0.9;
      this.selectionMarker.scale.setScalar(scale);
      this.selectionMarker.material.opacity = snapshot.diagnostics.activeGrab ? 0.72 : 0.38;
    }
  }

  private findSelectedPose(
    segment: SegmentId | null,
    region: RegionId | null,
    poses: ReadonlyMap<SegmentId, SegmentPose>,
  ): SegmentPose | null {
    if (segment) return poses.get(segment) ?? null;
    if (!region) return null;
    const primary = PRIMARY_SEGMENT_BY_REGION.get(region);
    return primary ? poses.get(primary) ?? null : null;
  }

  private protocolMesh(piece: ProtocolVisualPiece): THREE.Mesh {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(piece.geometry.vertices.flatMap(vertex => [vertex.x, vertex.y, vertex.z]), 3));
    geometry.setIndex(piece.geometry.triangles.flatMap(triangle => [...triangle]));
    geometry.computeVertexNormals();
    const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({
      color: piece.color,
      transparent: Boolean(piece.opacity), opacity: piece.opacity ?? 1,
      depthWrite: !piece.opacity, side: THREE.DoubleSide,
    }));
    mesh.position.set(piece.position.x, piece.position.y, piece.position.z);
    mesh.name = `protocol:${piece.id}`;
    mesh.castShadow = false;
    mesh.receiveShadow = true;
    return mesh;
  }

  private syncProtocolRoom(room: NonNullable<PoseSnapshot["room"]>): void {
    const key = `${room.width}:${room.depth}:${room.height}`;
    if (key === this.roomKey) return;
    this.roomKey = key;
    disposeSceneGroup(this.protocolRoom);
    for (const piece of protocolRoomPieces(room)) this.protocolRoom.add(this.protocolMesh(piece));
    batchOpaque(this.protocolRoom);
    const grid = new THREE.GridHelper(room.width, Math.max(1, Math.round(room.width * 2)), 0xb9c3bf, 0xcbd2cb);
    grid.position.y = 0.004;
    grid.scale.z = room.depth / room.width;
    this.protocolRoom.add(grid);
  }

  private fitShadowCamera(): void {
    const bounds = this.casterBounds.makeEmpty();
    this.scene.updateMatrixWorld(true);
    for (const mesh of this.segmentMeshes.values()) if (mesh.visible) bounds.expandByObject(mesh);
    if (this.protocolStriker.visible) bounds.expandByObject(this.protocolStriker);
    if (this.playgroundView.group.visible) bounds.expandByObject(this.playgroundView.movingGroup);
    if (bounds.isEmpty()) return;
    // Include receiver space in the direction of the light rays. This retains
    // floor and wall shadows without covering the whole 20m arena at all times.
    const height = Math.max(0, bounds.max.y);
    bounds.max.x += height * 0.6;
    bounds.min.z -= height * 0.6;
    bounds.min.y = Math.min(-0.1, bounds.min.y);
    bounds.expandByScalar(0.35);
    bounds.getCenter(this.center);
    const light = this.keyLight;
    light.target.position.copy(this.center);
    light.position.copy(this.center).add(this.corner.set(-6, 10, 6));
    light.updateMatrixWorld();
    light.target.updateMatrixWorld();
    light.shadow.updateMatrices(light);
    const camera = light.shadow.camera;
    const fit = this.lightBounds.makeEmpty();
    for (let i = 0; i < 8; i++) {
      this.corner.set(i & 1 ? bounds.max.x : bounds.min.x,
        i & 2 ? bounds.max.y : bounds.min.y, i & 4 ? bounds.max.z : bounds.min.z);
      fit.expandByPoint(this.corner.applyMatrix4(camera.matrixWorldInverse));
    }
    camera.left = fit.min.x; camera.right = fit.max.x;
    camera.bottom = fit.min.y; camera.top = fit.max.y;
    camera.near = Math.max(0.1, -fit.max.z - 1);
    camera.far = Math.max(camera.near + 1, -fit.min.z + 1);
    camera.updateProjectionMatrix();
  }

  private assertUsable(): void {
    if (this.disposed) throw new Error("WebGLView has been disposed.");
  }
}
