import * as THREE from "three";
import { V3 } from "../core/math";
import { PASSIVE_COLOR, REGION_COLORS, SEGMENTS, SEGMENT_BY_ID } from "../core/humanoid";
import type { PoseSnapshot, PoseView, RegionId, SegmentDefinition, SegmentId, SegmentPose, Vec3 } from "../core/types";
import { SharedCameraProjection } from "./camera";
import type { SceneViewOptions } from "./options";
import { interpolatePoseSnapshot, transformLocalPoint } from "./pose";

export interface WebGLRenderMetrics {
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
  if (definition.shape.kind === "box") {
    const half = definition.shape.halfExtents;
    return new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2, 1, 1, 1);
  }
  if (definition.shape.kind === "sphere") {
    return new THREE.SphereGeometry(definition.shape.radius, 18, 12);
  }
  return new THREE.CapsuleGeometry(
    definition.shape.radius,
    definition.shape.halfHeight * 2,
    5,
    12,
  );
}

export class WebGLView implements PoseView {
  readonly mode = "webgl" as const;
  readonly canvas: HTMLCanvasElement;

  private readonly ownsCanvas: boolean;
  private readonly projection: SharedCameraProjection;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera();
  private readonly segmentMeshes = new Map<SegmentId, THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>>();
  private readonly jointMarkers = new Map<SegmentId, THREE.Mesh>();
  private readonly supportMarkers = new Map<"leftFoot" | "rightFoot", THREE.Mesh<THREE.RingGeometry, THREE.MeshBasicMaterial>>();
  private readonly baseColors = new Map<SegmentId, THREE.Color>();
  private readonly selectionMarker: THREE.Mesh<THREE.SphereGeometry, THREE.MeshBasicMaterial>;
  private renderer: THREE.WebGLRenderer | null = null;
  private context: WebGL2RenderingContext | null = null;
  private container: HTMLElement | null = null;
  private snapshot: PoseSnapshot | null = null;
  private cssWidth = 1;
  private cssHeight = 1;
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
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.46, wireframe: true }),
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
    this.snapshot = interpolatePoseSnapshot(previous, current, alpha);
  }

  render(): void {
    if (!this.renderer || this.disposed) return;
    this.syncCamera();
    if (this.snapshot) this.applyPose(this.snapshot);
    this.renderer.render(this.scene, this.camera);
  }

  resize(width: number, height: number, devicePixelRatio: number): void {
    this.assertUsable();
    this.cssWidth = Math.max(1, Math.round(width));
    this.cssHeight = Math.max(1, Math.round(height));
    this.pixelRatio = Math.max(1, Math.min(3, Number.isFinite(devicePixelRatio) ? devicePixelRatio : 1));
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
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    geometries.add(this.selectionMarker.geometry);
    materials.add(this.selectionMarker.material);
    this.scene.traverse((object) => {
      if (!(object instanceof THREE.Mesh) && !(object instanceof THREE.Line) && !(object instanceof THREE.Points)) return;
      const renderable = object as THREE.Mesh;
      if (renderable.geometry) geometries.add(renderable.geometry);
      const objectMaterials = Array.isArray(renderable.material) ? renderable.material : [renderable.material];
      for (const material of objectMaterials) if (material) materials.add(material);
    });
    for (const geometry of geometries) geometry.dispose();
    for (const material of materials) material.dispose();
    this.renderer?.dispose();
    this.renderer = null;
    this.context = null;
    this.segmentMeshes.clear();
    this.jointMarkers.clear();
    this.supportMarkers.clear();
    this.baseColors.clear();
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
    this.renderer.setClearColor(0x101a2b, 1);

    this.scene.background = new THREE.Color(0x101a2b);
    this.scene.add(new THREE.HemisphereLight(0xc6e6ff, 0x243047, 2.15));
    const key = new THREE.DirectionalLight(0xffffff, 2.35);
    key.position.set(2.6, 5.2, 3.7);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x7888ff, 1.1);
    rim.position.set(-3.1, 2.6, -2.4);
    this.scene.add(rim);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(6, 5),
      new THREE.MeshStandardMaterial({ color: 0x18263a, roughness: 0.96, metalness: 0 }),
    );
    floor.rotation.x = -Math.PI * 0.5;
    floor.position.set(0, -0.012, 0.5);
    this.scene.add(floor);
    const grid = new THREE.GridHelper(6, 12, 0x62d5ff, 0x334762);
    grid.position.set(0, 0.002, 0.5);
    this.scene.add(grid);

    const boundaryMaterial = new THREE.MeshStandardMaterial({
      color: 0x496c89,
      emissive: 0x163548,
      emissiveIntensity: 0.5,
      roughness: 0.72,
    });
    const boundaries: ReadonlyArray<readonly [number, number, number, number, number]> = [
      [0, 0.025, -2, 6, 0.05],
      [0, 0.025, 3, 6, 0.05],
      [-3, 0.025, 0.5, 0.05, 5],
      [3, 0.025, 0.5, 0.05, 5],
    ];
    for (const [x, y, z, width, depth] of boundaries) {
      const boundary = new THREE.Mesh(new THREE.BoxGeometry(width, 0.05, depth), boundaryMaterial);
      boundary.position.set(x, y, z);
      this.scene.add(boundary);
    }

    for (const definition of SEGMENTS) {
      const baseColor = new THREE.Color(definition.region ? REGION_COLORS[definition.region] : PASSIVE_COLOR);
      const material = new THREE.MeshStandardMaterial({
        color: baseColor,
        emissive: 0x000000,
        roughness: 0.62,
        metalness: 0.02,
      });
      const mesh = new THREE.Mesh(shapeGeometry(definition), material);
      mesh.name = `segment:${definition.id}`;
      mesh.userData.segmentId = definition.id;
      mesh.userData.regionId = definition.region;
      this.segmentMeshes.set(definition.id, mesh);
      this.baseColors.set(definition.id, baseColor);
      this.scene.add(mesh);
      if (definition.parent) {
        const joint = new THREE.Mesh(
          new THREE.SphereGeometry(0.037, 10, 7),
          new THREE.MeshBasicMaterial({ color: 0xe1eeff, transparent: true, opacity: 0.82 }),
        );
        joint.name = `joint:${definition.parent}:${definition.id}`;
        this.jointMarkers.set(definition.id, joint);
        this.scene.add(joint);
      }
    }

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
    const poses = new Map<SegmentId, SegmentPose>();
    for (const mesh of this.segmentMeshes.values()) mesh.visible = false;
    for (const pose of snapshot.segments) {
      poses.set(pose.id, pose);
      const mesh = this.segmentMeshes.get(pose.id);
      if (!mesh) continue;
      mesh.visible = true;
      mesh.position.set(pose.position.x, pose.position.y, pose.position.z);
      mesh.quaternion.set(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w).normalize();
      const selected = SEGMENT_BY_ID.get(pose.id)?.region === snapshot.diagnostics.selectedRegion;
      mesh.material.emissive.set(selected ? 0x4ae1ff : 0x000000);
      mesh.material.emissiveIntensity = selected ? 0.55 : 0;
      mesh.scale.setScalar(selected ? 1.035 : 1);
    }

    for (const definition of SEGMENTS) {
      if (!definition.parent) continue;
      const marker = this.jointMarkers.get(definition.id);
      const child = poses.get(definition.id);
      const parent = poses.get(definition.parent);
      if (!marker || !child || !parent) {
        if (marker) marker.visible = false;
        continue;
      }
      const parentPoint = definition.jointAnchorParent
        ? transformLocalPoint(parent, definition.jointAnchorParent)
        : parent.position;
      const childPoint = definition.jointAnchorChild
        ? transformLocalPoint(child, definition.jointAnchorChild)
        : child.position;
      const point = average(parentPoint, childPoint);
      marker.position.set(point.x, point.y, point.z);
      marker.visible = true;
    }

    for (const foot of ["leftFoot", "rightFoot"] as const) {
      const marker = this.supportMarkers.get(foot)!;
      const pose = poses.get(foot);
      marker.visible = Boolean(pose);
      if (!pose) continue;
      marker.position.x = pose.position.x;
      marker.position.z = pose.position.z;
      const planted = snapshot.support.planted.includes(foot);
      const swinging = snapshot.support.swingFoot === foot;
      marker.material.color.set(swinging ? 0xffd166 : planted ? 0x80e7a8 : 0x7c8ba1);
      marker.material.opacity = swinging ? 0.9 : planted ? 0.72 : 0.28;
    }

    const selectedPose = this.findSelectedPose(snapshot.diagnostics.selectedRegion, poses);
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

  private findSelectedPose(region: RegionId | null, poses: ReadonlyMap<SegmentId, SegmentPose>): SegmentPose | null {
    if (!region) return null;
    const definition = SEGMENTS.find((candidate) => candidate.region === region);
    return definition ? poses.get(definition.id) ?? null : null;
  }

  private assertUsable(): void {
    if (this.disposed) throw new Error("WebGLView has been disposed.");
  }
}
